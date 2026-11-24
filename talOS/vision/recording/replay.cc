#include "talOS/vision/recording/replay.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <string>
#include <string_view>
#include <tuple>
#include <vector>

#include "talOS/vision/pipeline.h"
#include "talOS/vision/recording/recording.h"

namespace talos::vision::recording {
namespace {

std::string ReadFile(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  if (!in) {
    throw std::runtime_error("cannot read " + path);
  }
  return {std::istreambuf_iterator<char>{in}, {}};
}

// The camera node's range filter (vision::Vision::FilterDetections).
std::uint32_t FilterDetections(std::vector<TagDetection3D>& detections,
                               double max_range_m) {
  std::uint32_t rejected = 0;
  std::size_t accepted = 0;
  for (std::size_t i = 0; i < detections.size(); ++i) {
    const auto& detection = detections[i];
    if (!detection.valid) {
      continue;
    }
    const double range = detection.T_camera_tag.translation().norm();
    if (!std::isfinite(range) || range > max_range_m) {
      ++rejected;
      continue;
    }
    if (accepted != i) {
      detections[accepted] = std::move(detections[i]);
    }
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
  if (!std::isfinite(rad)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
  return std::remainder(rad, kTwoPi);
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
  if (reason.has_value()) {
    ++counts[*reason];
  } else {
    ++counts["(valid)"];
  }
}

struct PipelineKey {
  std::string backend_name;
  float quad_decimate = 0.0f;
  int threads = 0;
  float quad_sigma = 0.0f;
  bool refine_edges = false;
  int max_hamming = 0;
  double min_decision_margin = 0.0;
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  double k1 = 0.0;
  double k2 = 0.0;
  double k3 = 0.0;
  double p1 = 0.0;
  double p2 = 0.0;
  std::uint32_t image_width = 0;
  std::uint32_t image_height = 0;
  double x_m = 0.0;
  double y_m = 0.0;
  double z_m = 0.0;
  double roll_rad = 0.0;
  double pitch_rad = 0.0;
  double yaw_rad = 0.0;

  bool operator==(const PipelineKey& o) const {
    return std::tie(backend_name, quad_decimate, threads, quad_sigma,
                    refine_edges, max_hamming, min_decision_margin, fx, fy, cx,
                    cy, k1, k2, k3, p1, p2, image_width, image_height, x_m, y_m,
                    z_m, roll_rad, pitch_rad, yaw_rad) ==
           std::tie(o.backend_name, o.quad_decimate, o.threads, o.quad_sigma,
                    o.refine_edges, o.max_hamming, o.min_decision_margin, o.fx,
                    o.fy, o.cx, o.cy, o.k1, o.k2, o.k3, o.p1, o.p2,
                    o.image_width, o.image_height, o.x_m, o.y_m, o.z_m,
                    o.roll_rad, o.pitch_rad, o.yaw_rad);
  }
};

std::vector<std::uint8_t> CopyGrayRows(const cv::Mat& image) {
  const std::size_t rows = static_cast<std::size_t>(image.rows);
  const std::size_t cols = static_cast<std::size_t>(image.cols);
  std::vector<std::uint8_t> gray(rows * cols);
  if (image.isContinuous()) {
    std::memcpy(gray.data(), image.data, gray.size());
  } else {
    for (std::size_t r = 0; r < rows; ++r) {
      std::memcpy(gray.data() + r * cols,
                  image.ptr<std::uint8_t>(static_cast<int>(r)), cols);
    }
  }
  return gray;
}

}  // namespace

bool ReplayFrames(const std::string& frames_dir,
                  const field_sim::FieldLayoutData& layout,
                  const DetectorOverrides& overrides, ReplayStats* stats,
                  std::string* error) {
  if (stats == nullptr) {
    if (error != nullptr) {
      *error = "null stats";
    }
    return false;
  }
  *stats = ReplayStats{};
  try {
    std::vector<std::string> stems;
    std::error_code ec;
    std::filesystem::directory_iterator it(frames_dir, ec);
    if (ec) {
      if (error != nullptr) {
        *error = "cannot list " + frames_dir + ": " + ec.message();
      }
      return false;
    }
    for (const std::filesystem::directory_iterator end; it != end;
         it.increment(ec)) {
      if (ec) {
        break;
      }
      const auto& entry = *it;
      if (!entry.is_regular_file(ec) || ec) {
        if (ec) {
          break;
        }
        continue;
      }
      const std::string name = entry.path().filename().string();
      if (name.size() < 5 || name.substr(name.size() - 5) != ".json") {
        continue;
      }
      stems.push_back(name.substr(0, name.size() - 5));
    }
    if (ec) {
      if (error != nullptr) {
        *error = "cannot list " + frames_dir + ": " + ec.message();
      }
      return false;
    }
    std::sort(stems.begin(), stems.end());

    Rms before_rms, after_rms;
    std::optional<PipelineKey> cached_key;
    std::unique_ptr<AprilTagPipeline> pipeline;
    for (const auto& stem : stems) {
      CameraContext context;
      FrameRecord recorded;
      std::string parse_error;
      std::string json_path = frames_dir;
      json_path.append("/").append(stem).append(".json");
      if (!ParseRecord(ReadFile(json_path), &context, &recorded,
                       &parse_error)) {
        if (error != nullptr) {
          *error = stem;
          error->append(".json: ").append(parse_error);
        }
        return false;
      }
      std::string png_path = frames_dir;
      png_path.append("/").append(stem).append(".png");
      cv::Mat image = cv::imread(png_path, cv::IMREAD_GRAYSCALE);
      if (image.empty() || image.type() != CV_8UC1 || image.rows <= 0 ||
          image.cols <= 0) {
        if (error != nullptr) {
          *error = "cannot read ";
          error->append(stem).append(".png");
        }
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

      const PipelineKey key{
          .backend_name = config.backend_name,
          .quad_decimate = config.detector_options.quad_decimate,
          .threads = config.detector_options.threads,
          .quad_sigma = config.detector_options.quad_sigma,
          .refine_edges = config.detector_options.refine_edges,
          .max_hamming = config.detector_options.max_hamming,
          .min_decision_margin = config.min_decision_margin,
          .fx = config.intrinsics.fx,
          .fy = config.intrinsics.fy,
          .cx = config.intrinsics.cx,
          .cy = config.intrinsics.cy,
          .k1 = config.intrinsics.k1,
          .k2 = config.intrinsics.k2,
          .k3 = config.intrinsics.k3,
          .p1 = config.intrinsics.p1,
          .p2 = config.intrinsics.p2,
          .image_width = config.intrinsics.image_width,
          .image_height = config.intrinsics.image_height,
          .x_m = context.mounting.x_m,
          .y_m = context.mounting.y_m,
          .z_m = context.mounting.z_m,
          .roll_rad = context.mounting.roll_rad,
          .pitch_rad = context.mounting.pitch_rad,
          .yaw_rad = context.mounting.yaw_rad,
      };
      if (!pipeline || !cached_key.has_value() || !(*cached_key == key)) {
        pipeline = std::make_unique<AprilTagPipeline>(config);
        cached_key = key;
      }

      std::vector<std::uint8_t> gray = CopyGrayRows(image);
      Frame frame{0,
                  0,
                  0,
                  talos::event::MonotonicTime::from_nanos(0),
                  static_cast<std::uint32_t>(image.cols),
                  static_cast<std::uint32_t>(image.rows),
                  static_cast<std::uint32_t>(image.cols),
                  PixelFormat::kGray8,
                  std::move(gray)};

      PipelineResult result = pipeline->ProcessFrame(frame);
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
        if (valid > 0) {
          --valid;
        }
        ++stats->rejections_after["ambiguity"];
      }
      const bool accepted =
          result.success &&
          AcceptEstimate(estimate,
                         config.validation_options.max_reprojection_error_px,
                         layout.dims.length_m, layout.dims.width_m);

      ++stats->frames;
      if (recorded.observation.valid) {
        ++stats->valid_before;
      }
      if (accepted) {
        ++stats->valid_after;
      }

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
        before_rms.count
            ? std::sqrt(before_rms.sx / static_cast<double>(before_rms.count))
            : 0.0;
    stats->rms_y_before =
        before_rms.count
            ? std::sqrt(before_rms.sy / static_cast<double>(before_rms.count))
            : 0.0;
    stats->rms_yaw_before =
        before_rms.count
            ? std::sqrt(before_rms.syaw / static_cast<double>(before_rms.count))
            : 0.0;
    stats->rms_x_after =
        after_rms.count
            ? std::sqrt(after_rms.sx / static_cast<double>(after_rms.count))
            : 0.0;
    stats->rms_y_after =
        after_rms.count
            ? std::sqrt(after_rms.sy / static_cast<double>(after_rms.count))
            : 0.0;
    stats->rms_yaw_after =
        after_rms.count
            ? std::sqrt(after_rms.syaw / static_cast<double>(after_rms.count))
            : 0.0;
    return true;
  } catch (const std::exception& e) {
    if (error != nullptr) {
      *error = e.what();
    }
    return false;
  }
}

}  // namespace talos::vision::recording
