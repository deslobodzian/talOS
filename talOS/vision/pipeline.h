#pragma once

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
};

struct PipelineResult {
  std::vector<TagDetection3D> detections;
  bool success{false};
  std::string error_message;
};

// AprilTag decoding and camera-relative pose estimation.
class AprilTagPipeline {
 public:
  explicit AprilTagPipeline(const PipelineConfig& config,
                            std::shared_ptr<ComputeBackend> backend = nullptr);

  PipelineResult ProcessFrame(const Frame& frame);

 private:
  PipelineConfig config_;
  std::shared_ptr<ComputeBackend> backend_;
};

}  // namespace talos::vision
