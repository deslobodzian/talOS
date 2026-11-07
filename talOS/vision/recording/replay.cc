#include "talOS/vision/recording/replay.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/vision/pipeline.h"
#include "talOS/vision/recording/recording.h"

namespace talos::vision::recording {
namespace {

std::string ReadFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) throw std::runtime_error("cannot read " + path);
  return {std::istreambuf_iterator<char>{in}, {}};
}

// The camera node's range filter (vision::Vision::FilterDetections).
std::uint32_t FilterDetections(std::vector<TagDetection3D>& detections,
                               double max_range_m) {
  std::uint32_t rejected = 0;
  std::size_t accepted = 0;
  for (std::size_t i = 0; i < detections.size(); ++i) {
    const auto& detection = detections[i];
    if (!detection.valid) continue;
    const double range = detection.T_camera_tag.translation().norm();
    if (!std::isfinite(range) || range > max_range_m) {
      ++rejected;
      continue;
    }
    if (accepted != i) detections[accepted] = std::move(detections[i]);
    ++accepted;
  }
  detections.resize(accepted);
  return rejected;
}

// The camera node's acceptance gate (vision::Vision::AcceptEstimate).
bool AcceptEstimate(const RobotPoseEstimate& e, double max_reprojection_px,
                    double field_length_m, double field_width_m) {
  const auto& t = e.T_field_robot.translation();
  return e.validity.valid && t.allFinite() &&
         e.T_field_robot.rotation().rotationMatrix().allFinite() &&
         std::isfinite(e.reprojection_error_px) &&
         e.reprojection_error_px <= max_reprojection_px && t.x() >= 0 &&
         t.y() >= 0 && t.x() <= field_length_m && t.y() <= field_width_m;
}

double WrapAngle(double rad) {
  constexpr double kPi = 3.14159265358979323846;
  while (rad > kPi) rad -= 2.0 * kPi;
  while (rad < -kPi) rad += 2.0 * kPi;
  return rad;
}

struct Rms {
  double sx = 0, sy = 0, syaw = 0;
  std::uint64_t count = 0;
  void Add(double x, double y, double yaw, const TruthRecord& t) {
    sx += (x - t.x_m) * (x - t.x_m);
    sy += (y - t.y_m) * (y - t.y_m);
    const double dyaw = WrapAngle(yaw - t.yaw_rad);
    syaw += dyaw * dyaw;
    ++count;
  }
};

void AddRejection(std::map<std::string, std::uint64_t>& counts,
                  const std::optional<std::string>& reason) {
  if (reason.has_value())
    ++counts[*reason];
  else
    ++counts["(valid)"];
}

}  // namespace

bool ReplayFrames(const std::string& frames_dir,
                  const field_sim::FieldLayoutData& layout,
                  const DetectorOverrides& overrides, ReplayStats* stats,
                  std::string* error) {
  if (stats == nullptr) {
    if (error != nullptr) *error = "null stats";
    return false;
  }
  *stats = ReplayStats{};
  try {
    std::vector<std::string> stems;
    std::error_code ec;
    for (const auto& entry :
         std::filesystem::directory_iterator(frames_dir, ec)) {
      if (ec) break;
      if (!entry.is_regular_file()) continue;
      const std::string name = entry.path().filename().string();
      if (name.size() < 5 || name.substr(name.size() - 5) != ".json") continue;
      stems.push_back(name.substr(0, name.size() - 5));
    }
    std::sort(stems.begin(), stems.end());

    Rms before_rms, after_rms;
    for (const auto& stem : stems) {
      CameraContext context;
      FrameRecord recorded;
      std::string parse_error;
      if (!ParseRecord(ReadFile(frames_dir + "/" + stem + ".json"), &context,
                       &recorded, &parse_error)) {
        if (error != nullptr) *error = stem + ".json: " + parse_error;
        return false;
      }
      cv::Mat image =
          cv::imread(frames_dir + "/" + stem + ".png", cv::IMREAD_GRAYSCALE);
      if (image.empty()) {
        if (error != nullptr) *error = "cannot read " + stem + ".png";
        return false;
      }

      PipelineConfig config;
      config.backend_name = overrides.backend;
      config.detector_options.quad_decimate = static_cast<float>(
          overrides.decimate.value_or(context.detector.decimate));
      config.detector_options.threads =
          overrides.threads.value_or(context.detector.threads);
      config.detector_options.quad_sigma =
          static_cast<float>(overrides.sigma.value_or(context.detector.sigma));
      config.detector_options.refine_edges =
          overrides.refine_edges.value_or(context.detector.refine_edges);
      config.detector_options.max_hamming =
          overrides.hamming.value_or(context.detector.hamming_threshold);
      config.min_decision_margin = overrides.min_decision_margin.value_or(
          context.detector.min_decision_margin);
      config.intrinsics = context.intrinsics;
      config.mounting = CameraMounting::FromExtrinsics(
          context.mounting.x_m, context.mounting.y_m, context.mounting.z_m,
          context.mounting.roll_rad, context.mounting.pitch_rad,
          context.mounting.yaw_rad);
      config.layout = layout.layout;

      AprilTagPipeline pipeline{config};

      std::vector<std::uint8_t> gray(static_cast<std::size_t>(image.rows) *
                                     image.cols);
      std::copy(image.data, image.data + gray.size(), gray.begin());
      Frame frame{0,
                  0,
                  0,
                  talos::event::MonotonicTime::from_nanos(0),
                  static_cast<std::uint32_t>(image.cols),
                  static_cast<std::uint32_t>(image.rows),
                  static_cast<std::uint32_t>(image.cols),
                  PixelFormat::kGray8,
                  std::move(gray)};

      PipelineResult result = pipeline.ProcessFrame(frame);
      std::vector<TagDetection3D>& detections = result.detections;
      AnnotateDetectionReasons(detections, layout.layout, overrides.max_range_m,
                               config.validation_options);
      // Tally rejections from the full annotated list, before range filtering
      // removes the rejects, matching the node's recording snapshot.
      for (const auto& d : detections) {
        AddRejection(stats->rejections_after,
                     d.reject_reason.empty()
                         ? std::optional<std::string>{}
                         : std::optional<std::string>{d.reject_reason});
      }
      FilterDetections(detections, overrides.max_range_m);
      const RobotPoseEstimate estimate =
          SolveMultiTagRobotPose(detections, 0, config.mounting, layout.layout,
                                 config.intrinsics, config.validation_options);
      // The lone detection behind a single-tag ambiguity rejection was tallied
      // as "(valid)" before filtering; reattribute it to the ambiguity gate so
      // the "after" column matches the recorder's new truthful sidecar.
      if (IsSingleTagAmbiguityRejection(estimate)) {
        auto& valid = stats->rejections_after["(valid)"];
        if (valid > 0) --valid;
        ++stats->rejections_after["ambiguity"];
      }
      const bool accepted =
          result.success &&
          AcceptEstimate(estimate,
                         config.validation_options.max_reprojection_error_px,
                         layout.dims.length_m, layout.dims.width_m);

      ++stats->frames;
      if (recorded.observation.valid) ++stats->valid_before;
      if (accepted) ++stats->valid_after;

      for (const auto& d : recorded.detections) {
        AddRejection(stats->rejections_before, d.reject_reason);
      }

      if (recorded.truth.has_value()) {
        ++stats->truth_frames;
        const TruthRecord& truth = *recorded.truth;
        if (recorded.observation.valid) {
          before_rms.Add(recorded.observation.x_m, recorded.observation.y_m,
                         recorded.observation.yaw_rad, truth);
        }
        if (accepted) {
          const auto& t = estimate.T_field_robot.translation();
          const auto r = estimate.T_field_robot.rotation().rotationMatrix();
          const double yaw = std::atan2(r(1, 0), r(0, 0));
          after_rms.Add(t.x(), t.y(), yaw, truth);
        }
      }
    }
    stats->rms_x_before =
        before_rms.count ? std::sqrt(before_rms.sx / before_rms.count) : 0.0;
    stats->rms_y_before =
        before_rms.count ? std::sqrt(before_rms.sy / before_rms.count) : 0.0;
    stats->rms_yaw_before =
        before_rms.count ? std::sqrt(before_rms.syaw / before_rms.count) : 0.0;
    stats->rms_x_after =
        after_rms.count ? std::sqrt(after_rms.sx / after_rms.count) : 0.0;
    stats->rms_y_after =
        after_rms.count ? std::sqrt(after_rms.sy / after_rms.count) : 0.0;
    stats->rms_yaw_after =
        after_rms.count ? std::sqrt(after_rms.syaw / after_rms.count) : 0.0;
    return true;
  } catch (const std::exception& e) {
    if (error != nullptr) *error = e.what();
    return false;
  }
}

}  // namespace talos::vision::recording
