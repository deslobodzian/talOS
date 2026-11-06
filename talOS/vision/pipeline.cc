#include "talOS/vision/pipeline.h"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <stdexcept>
#include <string_view>
#include <utility>

#include "talOS/vision/metal/metal_apriltag_backend.h"

namespace talos::vision {
namespace {
std::shared_ptr<ComputeBackend> MakeBackend(const PipelineConfig& config) {
  apriltag::DetectorOptions options;
  options.threads = config.GetInt("threads", 1);
  options.quad_decimate = static_cast<float>(config.GetDouble("decimate", 1.0));
  options.quad_sigma = static_cast<float>(config.GetDouble("sigma", 0.0));
  options.refine_edges = config.GetBool("refine_edges", true);
  options.max_hamming = config.GetInt("hamming_threshold", 1);
  if (config.backend_name == "metal" || config.backend_name == "auto") {
    auto backend = std::make_shared<metal::MetalAprilTagBackend>(options);
    if (config.backend_name == "metal" && !backend->supports_gpu()) {
      throw std::invalid_argument(
          "Metal device unavailable; select auto to allow CPU fallback");
    }
    return backend;
  }
  return std::make_shared<CpuAprilTagBackend>(options);
}
}  // namespace

AprilTagPipeline::AprilTagPipeline(const PipelineConfig& config,
                                   std::shared_ptr<ComputeBackend> backend)
    : config_{config},
      backend_{std::move(backend)},
      external_backend_(backend_ != nullptr) {
  std::string error;
  if (!AprilTagPipeline::ValidateConfig(config, &error)) {
    throw std::invalid_argument(error);
  }
  if (!backend_) backend_ = MakeBackend(config);
}

bool AprilTagPipeline::ValidateConfig(const PipelineConfig& config,
                                      std::string* error_msg) const {
  if (config.kind != "apriltag") {
    if (error_msg) *error_msg = "Config kind must be 'apriltag'";
    return false;
  }

  for (const char* key : {"decimate", "sigma", "threads", "hamming_threshold",
                          "min_decision_margin"}) {
    if (auto it = config.parameters.find(key); it != config.parameters.end()) {
      try {
        std::size_t used = 0;
        double value = std::stod(it->second, &used);
        if (used != it->second.size() || !std::isfinite(value)) {
          throw std::invalid_argument("invalid number");
        }
        if ((std::string_view(key) == "threads" ||
             std::string_view(key) == "hamming_threshold") &&
            std::trunc(value) != value) {
          throw std::invalid_argument("integer required");
        }
      } catch (...) {
        if (error_msg) *error_msg = std::string("Invalid parameter: ") + key;
        return false;
      }
    }
  }
  double decimate = config.GetDouble("decimate", 1.0);
  if (!std::isfinite(decimate) || decimate < 1.0 || decimate > 8.0) {
    if (error_msg) *error_msg = "Decimate factor must be >= 1.0";
    return false;
  }

  int threads = config.GetInt("threads", 1);
  if (threads < 1 || threads > 16) {
    if (error_msg) *error_msg = "Thread count must be >= 1";
    return false;
  }

  const double sigma = config.GetDouble("sigma", 0.0);
  if (config.backend_name == "metal" &&
      (sigma != 0 || (decimate != 1 && decimate != 2))) {
    if (error_msg) {
      *error_msg =
          "Metal supports decimation 1 or 2 and blur 0; select auto to allow "
          "CPU fallback";
    }
    return false;
  }
  if (config.GetDouble("min_decision_margin", 0) < 0 ||
      config.GetDouble("min_decision_margin", 0) > 1000) {
    if (error_msg) *error_msg = "Decision margin must be between 0 and 1000";
    return false;
  }
  if (auto it = config.parameters.find("refine_edges");
      it != config.parameters.end() && it->second != "true" &&
      it->second != "false" && it->second != "1" && it->second != "0") {
    if (error_msg) *error_msg = "refine_edges must be a boolean";
    return false;
  }
  if (!std::isfinite(sigma) || sigma < 0 || sigma > 4 ||
      config.GetInt("hamming_threshold", 1) < 0 ||
      config.GetInt("hamming_threshold", 1) > 2 ||
      (config.backend_name != "cpu" && config.backend_name != "metal" &&
       config.backend_name != "auto")) {
    if (error_msg) *error_msg = "Invalid detector settings or backend";
    return false;
  }
  return true;
}

bool AprilTagPipeline::ApplyConfig(const PipelineConfig& config) {
  std::string err;
  if (!ValidateConfig(config, &err)) {
    return false;
  }
  try {
    auto prepared = external_backend_ ? backend_ : MakeBackend(config);
    config_ = config;
    backend_ = std::move(prepared);
  } catch (...) {
    return false;
  }
  return true;
}

PipelineResult AprilTagPipeline::ProcessFrame(const Frame& frame) {
  PipelineResult result;
  result.camera_id = frame.camera_id();
  result.session_id = frame.session_id();
  result.sequence_number = frame.sequence_number();
  result.capture_time = frame.capture_time();
  result.timestamp_kind = frame.timestamp_kind();
  result.mapping_quality = frame.mapping_quality();
  result.revision = config_.revision;

  if (!backend_) {
    result.success = false;
    result.error_message = "No compute backend configured";
    result.completion_time = talos::event::steady_now();
    return result;
  }

  // Stage 1 & 2 & 3: Grayscale conversion, quad detection, tag decode
  BackendExecutionResult backend_res = backend_->Execute(frame);
  result.timings = backend_res.timings;
  result.effective_backend = backend_res.effective_backend;
  result.fallback_reason = backend_res.fallback_reason;
  if (config_.backend_name == "metal" &&
      backend_res.effective_backend != "metal") {
    result.success = false;
    result.error_message =
        "Requested Metal execution failed: " + backend_res.fallback_reason;
    result.completion_time = talos::event::steady_now();
    return result;
  }
  if (!backend_res.success) {
    result.success = false;
    result.error_message = backend_res.error_message;
    result.completion_time = talos::event::steady_now();
    return result;
  }

  // Stage 4: 3D Pose estimation and field localization
  const auto t_pose_start = std::chrono::steady_clock::now();

  for (const auto& raw : backend_res.detections) {
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
    double ambiguity = 0.0;

    bool pose_ok = EstimateTagPoseFromHomography(
        raw.homography, config_.intrinsics, tag_size, &primary_pose,
        &secondary_pose, &ambiguity, &raw.corners);

    if (pose_ok) {
      det.T_camera_tag = primary_pose;
      det.secondary_T_camera_tag = secondary_pose;
      det.secondary_reprojection_error_px = ComputeReprojectionError(
          raw.corners, secondary_pose, config_.intrinsics, tag_size);
      det.ambiguity = ambiguity;
      det.reprojection_error = ComputeReprojectionError(
          raw.corners, primary_pose, config_.intrinsics, tag_size);
      det.valid =
          (raw.hamming <= config_.GetInt("hamming_threshold", 1) &&
           raw.decision_margin >= config_.GetDouble("min_decision_margin", 0) &&
           det.reprojection_error <=
               config_.validation_options.max_reprojection_error_px);

      // Field localization
      RobotPoseEstimate field_res = ComputeRobotFieldPose(
          raw.id, frame.camera_id(), primary_pose, config_.mounting,
          config_.layout, det.reprojection_error, ambiguity,
          config_.validation_options, &secondary_pose,
          det.secondary_reprojection_error_px);

      if (det.valid && field_res.validity.valid) {
        FieldPoseResult fp;
        fp.tag_id = raw.id;
        fp.camera_id = frame.camera_id();
        fp.T_field_robot = field_res.T_field_robot;
        fp.reprojection_error = field_res.reprojection_error_px;
        fp.ambiguity = det.ambiguity;
        fp.capture_timestamp_ns = frame.capture_timestamp_ns();
        fp.revision = config_.revision;
        fp.valid = true;
        result.field_poses.push_back(fp);
      }
    } else {
      det.valid = false;
    }

    result.detections.push_back(det);
  }

  const auto t_pose_end = std::chrono::steady_clock::now();
  result.timings.pose_us =
      std::chrono::duration_cast<std::chrono::microseconds>(t_pose_end -
                                                            t_pose_start)
          .count();
  result.timings.total_us += result.timings.pose_us;

  result.completion_time = talos::event::steady_now();
  result.success = true;
  return result;
}

}  // namespace talos::vision
