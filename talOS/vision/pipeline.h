#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "talOS/vision/calibration.h"
#include "talOS/vision/compute_backend.h"
#include "talOS/vision/frame.h"
#include "talOS/vision/localization.h"

namespace talos::vision {

// Immutable, typed settings validated when constructing AprilTagPipeline.
struct PipelineConfig {
  apriltag::DetectorOptions detector_options{.quad_decimate = 1.0f};
  double min_decision_margin{30.0};
  std::string backend_name{"cpu"};
  CameraIntrinsics intrinsics;
  CameraMounting mounting;
  FieldLayout layout;
  PoseValidationOptions validation_options;
  // Bounds GPU setup for a long-lived node. Compiling Metal pipelines takes
  // Apple's shared compiler-cache file lock, which another process can hold
  // for seconds. When set, "auto" never waits for it: it detects on the CPU
  // while a background thread compiles, and switches to Metal at the first
  // frame after compilation succeeds. "metal" waits at most
  // `gpu_init_timeout`, then construction throws. A GPU-capable backend's
  // starting choice and each switch are logged to stderr, prefixed with
  // `log_name`. Unset (one-shot tools, tests), backend construction is
  // synchronous and the backend is fixed from the first frame.
  bool deferred_gpu_init{false};
  std::chrono::milliseconds gpu_init_timeout{5000};
  std::string log_name;
  // Builds the GPU backend (default: Metal); tests inject a slow one. A
  // backend whose supports_gpu() is false means the GPU is unavailable.
  std::function<std::shared_ptr<ComputeBackend>(
      const apriltag::DetectorOptions&)>
      gpu_backend_factory;
};

struct PipelineResult {
  std::vector<TagDetection3D> detections;
  bool success{false};
  std::string error_message;
  // The backend that ran this frame ("cpu" or "metal").
  std::string backend;
};

// AprilTag decoding and camera-relative pose estimation.
class AprilTagPipeline {
 public:
  explicit AprilTagPipeline(const PipelineConfig& config,
                            std::shared_ptr<ComputeBackend> backend = nullptr);

  // Not thread-safe: call from one thread at a time.
  PipelineResult ProcessFrame(const Frame& frame);

  // Name of the backend the next frame runs on, before any deferred switch.
  std::string backend_name() const { return backend_->name(); }

 private:
  struct GpuInit;
  void AdoptGpuBackend();

  PipelineConfig config_;
  std::shared_ptr<ComputeBackend> backend_;
  // Shared with the compile thread, which may outlive this pipeline.
  std::shared_ptr<GpuInit> gpu_init_;
};

}  // namespace talos::vision
