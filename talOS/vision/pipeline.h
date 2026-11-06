#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "talOS/events/time.h"
#include "talOS/geometry/se3.h"
#include "talOS/vision/calibration.h"
#include "talOS/vision/compute_backend.h"
#include "talOS/vision/frame.h"
#include "talOS/vision/localization.h"

namespace talos::vision {

// Structured configuration preset for a vision pipeline.
struct PipelineConfig {
  std::string kind{"apriltag"};
  std::string backend_name{"cpu"};
  uint64_t revision{1};

  CameraIntrinsics intrinsics;
  CameraMounting mounting;
  FieldLayout layout;
  PoseValidationOptions validation_options;

  std::unordered_map<std::string, std::string> parameters;

  // Helper parameter accessors
  std::string GetString(const std::string& key,
                        const std::string& default_val = "") const {
    auto it = parameters.find(key);
    return it != parameters.end() ? it->second : default_val;
  }

  double GetDouble(const std::string& key, double default_val = 0.0) const {
    auto it = parameters.find(key);
    if (it != parameters.end()) {
      try {
        return std::stod(it->second);
      } catch (...) {
      }
    }
    return default_val;
  }

  int GetInt(const std::string& key, int default_val = 0) const {
    auto it = parameters.find(key);
    if (it != parameters.end()) {
      try {
        return std::stoi(it->second);
      } catch (...) {
      }
    }
    return default_val;
  }

  bool GetBool(const std::string& key, bool default_val = false) const {
    auto it = parameters.find(key);
    if (it != parameters.end()) {
      return it->second == "true" || it->second == "1";
    }
    return default_val;
  }
};

// Tag detection result with 2D observations and 3D camera-relative pose.

// Field-relative robot pose measurement derived from a single tag.
struct FieldPoseResult {
  int tag_id{-1};
  uint32_t camera_id{0};
  talos::geometry::SE3<double> T_field_robot{
      talos::geometry::SE3<double>::Identity()};
  double reprojection_error{0.0};
  double ambiguity{0.0};
  uint64_t capture_timestamp_ns{0};
  uint64_t revision{0};
  bool valid{false};
};

// Overall typed result emitted by a Pipeline execution on a Frame.
struct PipelineResult {
  uint32_t camera_id{0};
  uint64_t session_id{0};
  uint64_t sequence_number{0};
  talos::event::MonotonicTime capture_time{
      talos::event::MonotonicTime::from_nanos(0)};
  talos::event::MonotonicTime completion_time{
      talos::event::MonotonicTime::from_nanos(0)};
  uint64_t revision{0};
  TimestampKind timestamp_kind{TimestampKind::kDelivery};
  TimestampMappingQuality mapping_quality{
      TimestampMappingQuality::kUnsynchronized};

  StageTimings timings;
  std::string effective_backend;
  std::string fallback_reason;
  std::vector<TagDetection3D> detections;
  std::vector<FieldPoseResult> field_poses;

  bool success{false};
  std::string error_message;
};

// Abstract Pipeline interface: validates presets, executes typed processing
// stages, and emits results tagged with effective configuration revision.
class Pipeline {
 public:
  virtual ~Pipeline() = default;

  virtual std::string kind() const = 0;
  virtual uint64_t current_revision() const = 0;
  virtual const PipelineConfig& current_config() const = 0;

  // Validates a preset before application. Returns false and fills error_msg if
  // invalid. Pass nullptr when no diagnostic is needed.
  virtual bool ValidateConfig(const PipelineConfig& config,
                              std::string* error_msg) const = 0;

  // Applies new configuration at a frame boundary.
  virtual bool ApplyConfig(const PipelineConfig& config) = 0;

  // Processes a frame through typed pipeline stages.
  virtual PipelineResult ProcessFrame(const Frame& frame) = 0;
};

// AprilTag detection and localization pipeline.
class AprilTagPipeline : public Pipeline {
 public:
  explicit AprilTagPipeline(const PipelineConfig& config,
                            std::shared_ptr<ComputeBackend> backend = nullptr);

  std::string kind() const override { return "apriltag"; }
  uint64_t current_revision() const override { return config_.revision; }
  const PipelineConfig& current_config() const override { return config_; }

  bool ValidateConfig(const PipelineConfig& config,
                      std::string* error_msg) const override;
  bool ApplyConfig(const PipelineConfig& config) override;

  PipelineResult ProcessFrame(const Frame& frame) override;

 private:
  PipelineConfig config_;
  std::shared_ptr<ComputeBackend> backend_;
  bool external_backend_{false};
};

}  // namespace talos::vision
