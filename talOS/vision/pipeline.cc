#include "talOS/vision/pipeline.h"

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <exception>
#include <format>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "talOS/process/process.h"
#include "talOS/vision/metal/metal_apriltag_backend.h"

namespace talos::vision {
namespace {
void ValidateConfig(const PipelineConfig& config) {
  const auto& options = config.detector_options;
  if (!std::isfinite(options.quad_decimate) || options.quad_decimate < 1 ||
      options.quad_decimate > 8) {
    throw std::invalid_argument("detector.decimate must be between 1 and 8");
  }
  if (options.threads < 1 || options.threads > 16) {
    throw std::invalid_argument("detector.threads must be between 1 and 16");
  }
  if (!std::isfinite(options.quad_sigma) || options.quad_sigma < 0 ||
      options.quad_sigma > 4) {
    throw std::invalid_argument("detector.sigma must be between 0 and 4");
  }
  if (options.max_hamming < 0 || options.max_hamming > 2) {
    throw std::invalid_argument(
        "detector.hamming_threshold must be between 0 and 2");
  }
  if (!std::isfinite(config.min_decision_margin) ||
      config.min_decision_margin < 0 || config.min_decision_margin > 1000) {
    throw std::invalid_argument(
        "detector.min_decision_margin must be between 0 and 1000");
  }
  if (config.backend_name != "cpu" && config.backend_name != "metal" &&
      config.backend_name != "auto") {
    throw std::invalid_argument("backend must be cpu, metal or auto");
  }
  if (config.backend_name == "metal" &&
      (options.quad_sigma != 0 ||
       (options.quad_decimate != 1 && options.quad_decimate != 2))) {
    throw std::invalid_argument(
        "backend metal requires detector.sigma = 0 and detector.decimate = 1 "
        "or 2; select auto to allow CPU fallback");
  }
}

bool MetalSupports(const apriltag::DetectorOptions& options) {
  return options.quad_sigma == 0 &&
         (options.quad_decimate == 1 || options.quad_decimate == 2);
}

std::shared_ptr<ComputeBackend> MakeGpuBackend(const PipelineConfig& config) {
  if (config.gpu_backend_factory) {
    return config.gpu_backend_factory(config.detector_options);
  }
  return std::make_shared<metal::MetalAprilTagBackend>(config.detector_options);
}

std::shared_ptr<ComputeBackend> MakeBackend(const PipelineConfig& config) {
  if (config.backend_name == "metal" || config.backend_name == "auto") {
    auto backend = MakeGpuBackend(config);
    if (config.backend_name == "metal" && !backend->supports_gpu()) {
      throw std::invalid_argument(
          "backend metal device unavailable; select auto to allow CPU "
          "fallback");
    }
    return backend;
  }
  return std::make_shared<CpuAprilTagBackend>(config.detector_options);
}

void Log(const PipelineConfig& config, const std::string& message) {
  std::fprintf(stderr, "%s: %s\n",
               config.log_name.empty() ? "vision" : config.log_name.c_str(),
               message.c_str());
}
}  // namespace

// The outcome of one background GPU backend construction. The compile thread
// holds its own reference, so a pipeline destroyed while that thread is
// blocked (on the compiler-cache lock, say) never waits for it.
struct AprilTagPipeline::GpuInit {
  std::mutex mutex;
  std::condition_variable done_cv;
  bool done = false;                        // Guarded by mutex.
  std::shared_ptr<ComputeBackend> backend;  // Guarded; null: GPU unavailable.
  std::string info;                         // Guarded; device or failure.
  std::chrono::milliseconds elapsed{0};     // Guarded.
  std::atomic<bool> ready{false};           // `done`, readable without lock.
};

AprilTagPipeline::AprilTagPipeline(const PipelineConfig& config,
                                   std::shared_ptr<ComputeBackend> backend)
    : config_{config}, backend_{std::move(backend)} {
  ValidateConfig(config_);
  if (backend_) return;
  if (!config_.deferred_gpu_init) {
    backend_ = MakeBackend(config_);
    return;
  }
  backend_ = std::make_shared<CpuAprilTagBackend>(config_.detector_options);
  if (config_.backend_name == "cpu") return;  // As configured; nothing to log.
#if !defined(__APPLE__)
  // No Metal to compile: no thread, nothing to log.
  if (!config_.gpu_backend_factory) {
    if (config_.backend_name == "metal") {
      throw std::invalid_argument(
          "backend metal device unavailable; select auto to allow CPU "
          "fallback");
    }
    return;
  }
#endif
  if (!MetalSupports(config_.detector_options)) {
    Log(config_,
        "detector on cpu: Metal needs detector.sigma = 0 and "
        "detector.decimate 1 or 2");
    return;
  }
  gpu_init_ = std::make_shared<GpuInit>();
  // Detached: joining could block shutdown on the same lock construction
  // must not wait for.
  std::thread([init = gpu_init_, gpu_config = config_] {
    // Compiling Metal pipelines takes seconds; the node loops come first.
    talos::process::ConfigureBackgroundThread();
    const auto start = std::chrono::steady_clock::now();
    std::shared_ptr<ComputeBackend> gpu;
    std::string info;
    try {
      gpu = MakeGpuBackend(gpu_config);
      info = gpu->device_info();
      if (!gpu->supports_gpu()) gpu.reset();
    } catch (const std::exception& error) {
      gpu.reset();
      info = error.what();
    }
    {
      std::lock_guard lock(init->mutex);
      init->backend = std::move(gpu);
      init->info = std::move(info);
      init->elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - start);
      init->done = true;
      init->ready.store(true, std::memory_order_release);
    }
    init->done_cv.notify_all();
  }).detach();
  if (config_.backend_name == "auto") {
    Log(config_, "detector on cpu while Metal pipelines compile");
    return;
  }
  // "metal" accepts no CPU detection, so it waits, but boundedly.
  {
    std::unique_lock lock(gpu_init_->mutex);
    if (!gpu_init_->done_cv.wait_for(lock, config_.gpu_init_timeout,
                                     [this] { return gpu_init_->done; })) {
      throw std::invalid_argument(std::format(
          "backend metal: Metal pipelines did not compile within {} ms (the "
          "Metal compiler cache lock may be held); select auto to detect on "
          "the CPU meanwhile",
          config_.gpu_init_timeout.count()));
    }
    if (!gpu_init_->backend) {
      throw std::invalid_argument("backend metal device unavailable (" +
                                  gpu_init_->info +
                                  "); select auto to allow CPU fallback");
    }
  }
  AdoptGpuBackend();
}

void AprilTagPipeline::AdoptGpuBackend() {
  std::shared_ptr<ComputeBackend> gpu;
  std::string info;
  std::chrono::milliseconds elapsed{0};
  {
    std::lock_guard lock(gpu_init_->mutex);
    gpu = std::move(gpu_init_->backend);
    info = std::move(gpu_init_->info);
    elapsed = gpu_init_->elapsed;
  }
  gpu_init_.reset();
  if (gpu) {
    backend_ = std::move(gpu);
    Log(config_, std::format("detector on metal after {} ms: {}",
                             elapsed.count(), info));
  } else {
    Log(config_,
        std::format("Metal unavailable after {} ms ({}); detector stays on cpu",
                    elapsed.count(), info));
  }
}

PipelineResult AprilTagPipeline::ProcessFrame(const Frame& frame) {
  if (gpu_init_ && gpu_init_->ready.load(std::memory_order_acquire)) {
    AdoptGpuBackend();
  }
  PipelineResult result;
  auto backend_result = backend_->Execute(frame);
  result.backend = backend_result.effective_backend.empty()
                       ? backend_->name()
                       : backend_result.effective_backend;
  if (config_.backend_name == "metal" &&
      backend_result.effective_backend != "metal") {
    result.error_message =
        "Requested Metal execution failed: " + backend_result.fallback_reason;
    return result;
  }
  if (!backend_result.success) {
    result.error_message = std::move(backend_result.error_message);
    return result;
  }

  result.detections.reserve(backend_result.detections.size());
  for (const auto& raw : backend_result.detections) {
    TagDetection3D det;
    det.tag_id = raw.id;
    det.hamming = raw.hamming;
    det.center_x = raw.center_x;
    det.center_y = raw.center_y;
    det.corners = raw.corners;
    det.decision_margin = raw.decision_margin;
    det.homography = raw.homography;

    const double tag_size = config_.layout.GetTagSize(raw.id);
    talos::geometry::SE3<double> primary_pose;
    talos::geometry::SE3<double> secondary_pose;
    double ambiguity = 0, primary_error = 0, secondary_error = 0;
    if (EstimateTagPoseFromHomography(raw.homography, config_.intrinsics,
                                      tag_size, &primary_pose, &secondary_pose,
                                      &primary_error, &secondary_error,
                                      &ambiguity, &raw.corners)) {
      det.T_camera_tag = primary_pose;
      det.secondary_T_camera_tag = secondary_pose;
      det.secondary_reprojection_error_px = secondary_error;
      det.ambiguity = ambiguity;
      det.reprojection_error = primary_error;
      det.valid = raw.hamming <= config_.detector_options.max_hamming &&
                  raw.decision_margin >= config_.min_decision_margin &&
                  det.reprojection_error <=
                      config_.validation_options.max_reprojection_error_px;
      if (!det.valid) {
        if (raw.hamming > config_.detector_options.max_hamming) {
          det.reject_reason = "hamming";
        } else if (raw.decision_margin < config_.min_decision_margin) {
          det.reject_reason = "decision_margin";
        } else {
          det.reject_reason = "reprojection";
        }
      }
    } else {
      det.reject_reason = "reprojection";
    }
    result.detections.push_back(std::move(det));
  }
  result.success = true;
  return result;
}

}  // namespace talos::vision
