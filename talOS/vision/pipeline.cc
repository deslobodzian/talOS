#include "talOS/vision/pipeline.h"

#include <cmath>
#include <stdexcept>
#include <utility>

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

std::shared_ptr<ComputeBackend> MakeBackend(const PipelineConfig& config) {
  if (config.backend_name == "metal" || config.backend_name == "auto") {
    auto backend =
        std::make_shared<metal::MetalAprilTagBackend>(config.detector_options);
    if (config.backend_name == "metal" && !backend->supports_gpu()) {
      throw std::invalid_argument(
          "backend metal device unavailable; select auto to allow CPU "
          "fallback");
    }
    return backend;
  }
  return std::make_shared<CpuAprilTagBackend>(config.detector_options);
}
}  // namespace

AprilTagPipeline::AprilTagPipeline(const PipelineConfig& config,
                                   std::shared_ptr<ComputeBackend> backend)
    : config_{config}, backend_{std::move(backend)} {
  ValidateConfig(config_);
  if (!backend_) backend_ = MakeBackend(config_);
}

PipelineResult AprilTagPipeline::ProcessFrame(const Frame& frame) {
  PipelineResult result;
  auto backend_result = backend_->Execute(frame);
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
