// recording_eval: runs the camera node's detection and localization (the same
// AprilTagPipeline and MultiTagPoseSolver) over a recording and reports how
// well it detects and tracks. Hand-run on footage that may not be
// redistributed; see README.md in this directory.

#include <time.h>

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <numbers>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "talOS/geometry/se3.h"
#include "talOS/geometry/so3.h"
#include "talOS/vision/calibration.h"
#include "talOS/vision/evaluation/metrics.h"
#include "talOS/vision/field_sim/field_layout_loader.h"
#include "talOS/vision/field_sim/json_parser.h"
#include "talOS/vision/localization.h"
#include "talOS/vision/pipeline.h"
#include "talOS/vision/playback/image_sequence.h"

namespace talos::vision::evaluation {
namespace {

using talos::geometry::SE3;
using talos::geometry::SO3;
constexpr double kDeg = 180.0 / std::numbers::pi;

[[noreturn]] void Die(const std::string& message) {
  std::fprintf(stderr, "recording_eval: %s\n", message.c_str());
  std::exit(2);
}

std::vector<double> Numbers(std::string_view text, std::size_t min_count,
                            std::size_t max_count, const char* flag) {
  std::vector<double> values;
  std::stringstream in{std::string(text)};
  std::string item;
  while (std::getline(in, item, ',')) {
    char* end = nullptr;
    const double value = std::strtod(item.c_str(), &end);
    if (end == item.c_str() || *end != '\0' || !std::isfinite(value)) {
      Die(std::string(flag) + ": not a number: " + item);
    }
    values.push_back(value);
  }
  if (values.size() < min_count || values.size() > max_count) {
    Die(std::string(flag) + ": expected " + std::to_string(min_count) +
        (max_count != min_count ? " to " + std::to_string(max_count) : "") +
        " comma-separated numbers");
  }
  return values;
}

std::string ReadFile(const std::string& path) {
  std::ifstream in(path);
  if (!in) Die("cannot read " + path);
  std::stringstream text;
  text << in.rdbuf();
  return text.str();
}

double NowMs() {
  struct timespec ts {};
  ::clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<double>(ts.tv_sec) * 1e3 +
         static_cast<double>(ts.tv_nsec) / 1e6;
}

struct Options {
  std::string frames;
  double fps = 0;
  std::optional<CameraIntrinsics> intrinsics;
  std::optional<double> hfov_deg;
  std::optional<std::array<double, 4>> equidistant;
  std::vector<double> opencv_distortion;
  double tag_size_m = 0.1651;
  std::string layout_json;
  std::optional<std::array<double, 4>> aprilgrid;  // rows, cols, size, spacing
  int grid_variant = 0;
  bool detect_only = false;
  std::array<double, 6> mount{};  // x, y, z, roll, pitch, yaw
  bool floor_prior = true;
  bool floor_prior_set = false;
  float decimate = 2.0f;  // vision.toml's shared detector settings
  int threads = 2;
  float sigma = 0.0f;
  bool refine_edges = true;
  int hamming = 1;
  double min_margin = 30.0;
  double pixel_std_px = 0.33;
  std::size_t max_frames = 0;
  std::string mocap_csv;
  std::optional<SE3<double>> T_cam_imu;
  std::string out_dir;
  std::string label;
};

Options Parse(int argc, char** argv) {
  Options o;
  for (int i = 1; i < argc; ++i) {
    const std::string_view flag = argv[i];
    auto value = [&]() -> std::string {
      if (i + 1 >= argc) Die(std::string(flag) + " needs a value");
      return argv[++i];
    };
    if (flag == "--frames") {
      o.frames = value();
    } else if (flag == "--fps") {
      o.fps = Numbers(value(), 1, 1, "--fps")[0];
    } else if (flag == "--intrinsics") {
      const auto v = Numbers(value(), 4, 9, "--intrinsics");
      CameraIntrinsics k;
      k.fx = v[0];
      k.fy = v[1];
      k.cx = v[2];
      k.cy = v[3];
      if (v.size() > 4) k.k1 = v[4];
      if (v.size() > 5) k.k2 = v[5];
      if (v.size() > 6) k.p1 = v[6];
      if (v.size() > 7) k.p2 = v[7];
      if (v.size() > 8) k.k3 = v[8];
      o.intrinsics = k;
    } else if (flag == "--photonvision-calibration") {
      field_sim::JsonValue json;
      std::string error;
      if (!field_sim::ParseJson(ReadFile(value()), &json, &error)) Die(error);
      const auto* k = json.Find("cameraIntrinsics");
      const auto* d = json.Find("distCoeffs");
      const auto* kd = k ? k->Find("data") : nullptr;
      const auto* dd = d ? d->Find("data") : nullptr;
      if (!kd || kd->arr.size() != 9 || !dd || dd->arr.size() < 5) {
        Die("--photonvision-calibration: no cameraIntrinsics/distCoeffs data");
      }
      CameraIntrinsics c;
      c.fx = kd->arr[0].AsDouble(0);
      c.cx = kd->arr[2].AsDouble(0);
      c.fy = kd->arr[4].AsDouble(0);
      c.cy = kd->arr[5].AsDouble(0);
      if (dd->arr.size() == 5) {
        c.k1 = dd->arr[0].AsDouble(0);
        c.k2 = dd->arr[1].AsDouble(0);
        c.p1 = dd->arr[2].AsDouble(0);
        c.p2 = dd->arr[3].AsDouble(0);
        c.k3 = dd->arr[4].AsDouble(0);
      } else {
        // The rational model (k4..k6) is beyond the pipeline's
        // Brown-Conrady: rectify with OpenCV instead.
        for (const auto& v : dd->arr)
          o.opencv_distortion.push_back(v.AsDouble(0));
      }
      o.intrinsics = c;
    } else if (flag == "--opencv-distortion") {
      o.opencv_distortion = Numbers(value(), 4, 14, "--opencv-distortion");
    } else if (flag == "--hfov-deg") {
      o.hfov_deg = Numbers(value(), 1, 1, "--hfov-deg")[0];
    } else if (flag == "--equidistant") {
      const auto v = Numbers(value(), 4, 4, "--equidistant");
      o.equidistant = {v[0], v[1], v[2], v[3]};
    } else if (flag == "--tag-size-m") {
      o.tag_size_m = Numbers(value(), 1, 1, "--tag-size-m")[0];
    } else if (flag == "--layout") {
      o.layout_json = value();
    } else if (flag == "--aprilgrid") {
      const auto v = Numbers(value(), 4, 4, "--aprilgrid");
      o.aprilgrid = {v[0], v[1], v[2], v[3]};
    } else if (flag == "--detect-only") {
      // Unknown tag placement: every tag36h11 id counts as known (one row of
      // tags), so detection, id and pixel statistics are meaningful and
      // every pose, range and localization number is not.
      o.aprilgrid = {1, 587, o.tag_size_m, 0.3};
      o.detect_only = true;
    } else if (flag == "--grid-variant") {
      o.grid_variant = static_cast<int>(Numbers(value(), 1, 1, flag.data())[0]);
    } else if (flag == "--mount") {
      const auto v = Numbers(value(), 6, 6, "--mount");
      std::copy(v.begin(), v.end(), o.mount.begin());
    } else if (flag == "--floor-prior") {
      const auto v = value();
      if (v != "on" && v != "off") Die("--floor-prior on|off");
      o.floor_prior = v == "on";
      o.floor_prior_set = true;
    } else if (flag == "--decimate") {
      o.decimate = static_cast<float>(Numbers(value(), 1, 1, "--decimate")[0]);
    } else if (flag == "--threads") {
      o.threads = static_cast<int>(Numbers(value(), 1, 1, "--threads")[0]);
    } else if (flag == "--sigma") {
      o.sigma = static_cast<float>(Numbers(value(), 1, 1, "--sigma")[0]);
    } else if (flag == "--refine-edges") {
      o.refine_edges = value() != "off";
    } else if (flag == "--hamming") {
      o.hamming = static_cast<int>(Numbers(value(), 1, 1, "--hamming")[0]);
    } else if (flag == "--min-margin") {
      o.min_margin = Numbers(value(), 1, 1, "--min-margin")[0];
    } else if (flag == "--pixel-std-px") {
      o.pixel_std_px = Numbers(value(), 1, 1, "--pixel-std-px")[0];
    } else if (flag == "--max-frames") {
      o.max_frames =
          static_cast<std::size_t>(Numbers(value(), 1, 1, "--max-frames")[0]);
    } else if (flag == "--mocap") {
      o.mocap_csv = value();
    } else if (flag == "--t-cam-imu") {
      const auto v = Numbers(value(), 12, 16, "--t-cam-imu");
      Eigen::Matrix3d r;
      r << v[0], v[1], v[2], v[4], v[5], v[6], v[8], v[9], v[10];
      o.T_cam_imu = SE3<double>(SO3<double>::FromMatrixNormalized(r),
                                Eigen::Vector3d(v[3], v[7], v[11]));
    } else if (flag == "--out") {
      o.out_dir = value();
    } else if (flag == "--label") {
      o.label = value();
    } else {
      Die("unknown flag " + std::string(flag) + " (see README.md)");
    }
  }
  if (o.frames.empty()) Die("--frames DIR is required");
  if (o.layout_json.empty() == !o.aprilgrid) {
    Die("give exactly one of --layout JSON or --aprilgrid R,C,SIZE,SPACING");
  }
  if (!o.mocap_csv.empty() && !o.T_cam_imu) Die("--mocap needs --t-cam-imu");
  if (!o.floor_prior_set) o.floor_prior = !o.aprilgrid;
  return o;
}

// A Kalibr aprilgrid as a layout: tag id r*cols+c at row r, column c, in a
// grid frame with x along columns, y along rows, z out of the board. The
// variant picks the printed convention: bit 0 runs rows downward, bits 1-2
// rotate every tag by a multiple of 90 degrees in its plane.
FieldLayout AprilGridLayout(const std::array<double, 4>& grid, int variant) {
  const int rows = static_cast<int>(grid[0]);
  const int cols = static_cast<int>(grid[1]);
  const double size = grid[2];
  const double pitch = size * (1.0 + grid[3]);
  FieldLayout layout(size);
  // Tag frame axes in grid coordinates: X out of the face (+z), Y right (+x),
  // Z up (+y), then rotated in plane about the face normal.
  Eigen::Matrix3d base;
  base.col(0) = Eigen::Vector3d::UnitZ();
  base.col(1) = Eigen::Vector3d::UnitX();
  base.col(2) = Eigen::Vector3d::UnitY();
  const double turn = ((variant >> 1) & 3) * std::numbers::pi / 2;
  const Eigen::Matrix3d spin =
      Eigen::AngleAxisd(turn, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const double row_sign = (variant & 1) ? -1.0 : 1.0;
  for (int r = 0; r < rows; ++r) {
    for (int c = 0; c < cols; ++c) {
      const Eigen::Vector3d center(c * pitch + size / 2,
                                   row_sign * (r * pitch + size / 2), 0);
      layout.SetTagPose(r * cols + c,
                        SE3<double>(SO3<double>(spin * base), center));
    }
  }
  return layout;
}

// Mocap pose of the IMU (EuRoC CSV: t_ns, p xyz, q wxyz) at time t, linearly
// interpolated (translation; rotation by slerp) between samples at most
// 20 ms apart; nullopt outside the track or across a gap.
class MocapTrack {
 public:
  explicit MocapTrack(const std::string& path) {
    std::ifstream in(path);
    if (!in) Die("cannot read " + path);
    std::string line;
    while (std::getline(in, line)) {
      if (line.empty() || line[0] == '#') continue;
      std::vector<double> v;
      std::stringstream s(line);
      std::string item;
      std::int64_t t = 0;
      bool first = true;
      while (std::getline(s, item, ',')) {
        if (first) {
          t = std::stoll(item);
          first = false;
        } else {
          v.push_back(std::stod(item));
        }
      }
      if (v.size() < 7) continue;
      times_.push_back(t);
      positions_.emplace_back(v[0], v[1], v[2]);
      rotations_.emplace_back(v[3], v[4], v[5], v[6]);
    }
  }
  std::optional<SE3<double>> At(std::int64_t t) const {
    const auto it = std::lower_bound(times_.begin(), times_.end(), t);
    if (it == times_.begin() || it == times_.end()) return std::nullopt;
    const auto i = static_cast<std::size_t>(it - times_.begin());
    const auto t0 = times_[i - 1], t1 = times_[i];
    if (t1 - t0 > 20'000'000) return std::nullopt;
    const double a = static_cast<double>(t - t0) / static_cast<double>(t1 - t0);
    const Eigen::Vector3d p = (1 - a) * positions_[i - 1] + a * positions_[i];
    const Eigen::Quaterniond q =
        rotations_[i - 1].normalized().slerp(a, rotations_[i].normalized());
    return SE3<double>(SO3<double>::FromMatrixNormalized(q.toRotationMatrix()),
                       p);
  }

 private:
  std::vector<std::int64_t> times_;
  std::vector<Eigen::Vector3d> positions_;
  std::vector<Eigen::Quaterniond> rotations_;
};

// Heading as the node publishes it (geometry::Pose2d yaw).
double Yaw(const SE3<double>& pose) {
  const auto& r = pose.rotation().rotationMatrix();
  return std::atan2(r(1, 0), r(0, 0));
}

double RotationDeg(const SO3<double>& a, const SO3<double>& b) {
  return (a.inverse() * b).Log().norm() * kDeg;
}

struct Projected {
  std::array<Eigen::Vector2d, 4> corners;
  bool in_view = false;
};

// Where `layout` tag `id` should appear for a camera at `T_layout_camera`
// (optical frame): in view when every corner projects at least 4 px inside
// the image, its face is towards the camera and its shortest edge is at
// least `min_side_px`.
Projected Project(const FieldLayout& layout, int id,
                  const SE3<double>& T_layout_camera, const CameraIntrinsics& k,
                  double min_side_px) {
  Projected out;
  const auto tag = layout.GetTagPose(id);
  if (!tag) return out;
  const auto camera_tag = T_layout_camera.inverse() * *tag;
  const Eigen::Vector3d normal = camera_tag.rotation().rotationMatrix().col(0);
  const Eigen::Vector3d to_tag = camera_tag.translation().normalized();
  const auto model = GetTagModelCorners(layout.GetTagSize(id));
  bool inside = normal.dot(-to_tag) > 0.2;
  for (int c = 0; c < 4; ++c) {
    const Eigen::Vector3d p = camera_tag.act(model[c]);
    if (p.z() <= 0.05) return out;
    out.corners[c] = {k.fx * p.x() / p.z() + k.cx, k.fy * p.y() / p.z() + k.cy};
    inside = inside && out.corners[c].x() >= 4 && out.corners[c].y() >= 4 &&
             out.corners[c].x() <= k.image_width - 5.0 &&
             out.corners[c].y() <= k.image_height - 5.0;
  }
  double side = std::numeric_limits<double>::infinity();
  for (int c = 0; c < 4; ++c) {
    side = std::min(side, (out.corners[c] - out.corners[(c + 1) % 4]).norm());
  }
  out.in_view = inside && side >= min_side_px;
  return out;
}

// part / whole in percent, 0 for an empty whole.
double Percent(std::size_t part, std::size_t whole) {
  return whole == 0
             ? 0.0
             : 100.0 * static_cast<double>(part) / static_cast<double>(whole);
}

double MinSide(const std::array<float, 8>& c) {
  double side = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i < 4; ++i) {
    const std::size_t j = (i + 1) % 4;
    side = std::min(
        side, std::hypot(static_cast<double>(c[2 * i] - c[2 * j]),
                         static_cast<double>(c[2 * i + 1] - c[2 * j + 1])));
  }
  return side;
}

struct FrameResult {
  std::int64_t pts_ns = 0;
  std::vector<TagDetection3D> detections;
  RobotPoseEstimate multi;  // every usable detection, as the node solves
  bool decoded = false;
  double load_ms = 0, detect_ms = 0, solve_ms = 0;
};

struct Summary {
  std::ostringstream text;
  // printf-style; the attribute has the compiler check every call's format
  // literal against its arguments.
  __attribute__((format(printf, 2, 3))) void Line(const char* format, ...) {
    char buffer[512];
    std::va_list args;
    va_start(args, format);
    std::vsnprintf(buffer, sizeof buffer, format, args);
    va_end(args);
    text << buffer << '\n';
  }
};

std::string Dist(const std::vector<double>& v, double scale = 1.0,
                 const char* unit = "") {
  if (v.empty()) return "n=0";
  char buffer[256];
  std::snprintf(buffer, sizeof buffer,
                "n=%zu p50=%.3g p95=%.3g p99=%.3g max=%.3g%s", v.size(),
                Quantile(v, 0.5) * scale, Quantile(v, 0.95) * scale,
                Quantile(v, 0.99) * scale, Quantile(v, 1.0) * scale, unit);
  return buffer;
}

int Run(const Options& o) {
  std::string error;
  const auto sequence = playback::ImageSequence::Open(o.frames, o.fps, &error);
  if (!sequence) Die(error);
  const std::size_t count = o.max_frames > 0
                                ? std::min(o.max_frames, sequence->size())
                                : sequence->size();
  std::vector<std::uint8_t> pixels;
  int width = 0, height = 0;
  if (!sequence->LoadGray(0, &pixels, &width, &height, &error)) Die(error);

  bool assumed_intrinsics = false;
  CameraIntrinsics k;
  if (o.intrinsics) {
    k = *o.intrinsics;
  } else if (o.hfov_deg) {
    assumed_intrinsics = true;
    k.fx = k.fy = width / 2.0 / std::tan(*o.hfov_deg / 2 / kDeg);
    k.cx = width / 2.0;
    k.cy = height / 2.0;
  } else {
    Die("give --intrinsics, --photonvision-calibration or --hfov-deg");
  }
  k.image_width = static_cast<std::uint32_t>(width);
  k.image_height = static_cast<std::uint32_t>(height);
  // A fisheye recording is rectified to a pinhole image with the same K
  // before detection: the pipeline models Brown-Conrady distortion only.
  cv::Mat map1, map2;
  if (o.equidistant) {
    const cv::Matx33d K(k.fx, 0, k.cx, 0, k.fy, k.cy, 0, 0, 1);
    const cv::Vec4d D((*o.equidistant)[0], (*o.equidistant)[1],
                      (*o.equidistant)[2], (*o.equidistant)[3]);
    cv::fisheye::initUndistortRectifyMap(K, D, cv::Matx33d::eye(), K,
                                         cv::Size(width, height), CV_16SC2,
                                         map1, map2);
    k.k1 = k.k2 = k.k3 = k.p1 = k.p2 = 0;
  } else if (!o.opencv_distortion.empty()) {
    const cv::Matx33d K(k.fx, 0, k.cx, 0, k.fy, k.cy, 0, 0, 1);
    cv::initUndistortRectifyMap(K, o.opencv_distortion, cv::Matx33d::eye(), K,
                                cv::Size(width, height), CV_16SC2, map1, map2);
    k.k1 = k.k2 = k.k3 = k.p1 = k.p2 = 0;
  }
  const bool rectify = !map1.empty();

  field_sim::FieldLayoutData field;
  bool field_bounds = false;
  if (o.aprilgrid) {
    field.layout = AprilGridLayout(*o.aprilgrid, o.grid_variant);
  } else {
    if (!field_sim::LoadFieldLayoutFromJson(ReadFile(o.layout_json),
                                            o.tag_size_m, &field, &error)) {
      Die(error);
    }
    field_bounds = field.dims.length_m > 0 && field.dims.width_m > 0;
  }
  const auto mounting = CameraMounting::FromExtrinsics(
      o.mount[0], o.mount[1], o.mount[2], o.mount[3], o.mount[4], o.mount[5]);

  PipelineConfig config;
  config.backend_name = "cpu";
  config.detector_options.quad_decimate = o.decimate;
  config.detector_options.threads = o.threads;
  config.detector_options.quad_sigma = o.sigma;
  config.detector_options.refine_edges = o.refine_edges;
  config.detector_options.max_hamming = o.hamming;
  config.min_decision_margin = o.min_margin;
  config.intrinsics = k;
  config.mounting = mounting;
  config.layout = field.layout;
  AprilTagPipeline pipeline(config);
  PoseSolverOptions solver_options;
  solver_options.pixel_std_px = o.pixel_std_px;
  if (!o.floor_prior) {
    // A handheld camera has no floor: make the prior uninformative.
    solver_options.floor_z_std_m = 1e6;
    solver_options.floor_tilt_std_rad = 1e6;
  }
  const PoseValidationOptions validation;
  MultiTagPoseSolver solver;
  const double max_range_m = 8.0;  // vision.toml camera.max_range_m

  std::vector<FrameResult> frames(count);
  for (std::size_t i = 0; i < count; ++i) {
    auto& f = frames[i];
    f.pts_ns = (*sequence)[i].pts_ns;
    const double t0 = NowMs();
    int w = 0, h = 0;
    if (!sequence->LoadGray(i, &pixels, &w, &h, &error) || w != width ||
        h != height) {
      std::fprintf(stderr, "frame %zu: %s\n", i, error.c_str());
      continue;
    }
    if (rectify) {
      cv::Mat in(height, width, CV_8UC1, pixels.data());
      cv::Mat out;
      cv::remap(in, out, map1, map2, cv::INTER_LINEAR);
      std::copy(out.datastart, out.dataend, pixels.begin());
    }
    const double t1 = NowMs();
    const Frame frame(
        0, 1, i, talos::event::MonotonicTime::from_nanos(f.pts_ns),
        static_cast<std::uint32_t>(width), static_cast<std::uint32_t>(height),
        static_cast<std::uint32_t>(width), PixelFormat::kGray8, pixels.data(),
        pixels.size());
    auto result = pipeline.ProcessFrame(frame);
    const double t2 = NowMs();
    f.decoded = result.success;
    f.detections = std::move(result.detections);
    AnnotateDetectionReasons(f.detections, field.layout, max_range_m,
                             validation);
    // The node's localization input: valid, in range (FilterDetections).
    std::vector<TagDetection3D> usable;
    for (const auto& d : f.detections) {
      const double range = d.T_camera_tag.translation().norm();
      if (d.valid && std::isfinite(range) && range <= max_range_m) {
        usable.push_back(d);
      }
    }
    f.multi = solver.Solve(usable, 0, mounting, field.layout, k, validation,
                           solver_options);
    const double t3 = NowMs();
    f.load_ms = t1 - t0;
    f.detect_ms = t2 - t1;
    f.solve_ms = t3 - t2;
  }

  // The node publishes an estimate (AcceptEstimate) when it is valid, within
  // the reprojection gate and, on a field, inside the field rectangle.
  auto published = [&](const RobotPoseEstimate& e) {
    const auto& t = e.T_field_robot.translation();
    return e.validity.valid && t.allFinite() &&
           e.reprojection_error_px <= validation.max_reprojection_error_px &&
           (!field_bounds ||
            (t.x() >= 0 && t.y() >= 0 && t.x() <= field.dims.length_m &&
             t.y() <= field.dims.width_m));
  };
  const auto& optical = mounting.T_robot_camera;

  // Per-tag accumulation.
  struct TagStats {
    std::size_t detected = 0, expected = 0, expected_detected = 0;
    std::size_t misplaced = 0, flips = 0, flips_accepted = 0;
    std::size_t single_valid = 0, single_rejected_ambiguity = 0;
    std::vector<bool> seen;
    std::vector<std::optional<Eigen::Matrix<double, 8, 1>>> corners;
    std::vector<double> margin, reprojection, ambiguity, single_vs_multi_m,
        single_vs_multi_yaw_deg, corner_vs_multi_px;
  };
  std::map<int, TagStats> tags;
  std::map<std::string, std::size_t> rejects;
  std::vector<double> margin_valid, margin_unknown, hamming_valid,
      hamming_unknown, multi_reprojection, load_ms, detect_ms, solve_ms;
  std::size_t frames_with_valid = 0, frames_multi = 0, frames_published = 0,
              frames_single_published = 0, unknown_ids = 0, id_switches = 0,
              decode_failures = 0;
  std::vector<std::optional<Eigen::Vector3d>> multi_position(count);
  std::vector<std::optional<Eigen::Vector3d>> multi_rotation(count);
  std::vector<TagSighting> previous;
  // Ground truth: the fixed pose of the layout in the mocap world, one
  // sample per published estimate; its spread is the estimate's error.
  std::optional<MocapTrack> mocap;
  if (!o.mocap_csv.empty()) mocap.emplace(o.mocap_csv);
  struct GtSample {
    SE3<double> world_layout;
    int tag_count;
  };
  std::vector<GtSample> gt;
  std::vector<std::int64_t> gt_frame;

  for (std::size_t i = 0; i < count; ++i) {
    const auto& f = frames[i];
    load_ms.push_back(f.load_ms);
    detect_ms.push_back(f.detect_ms);
    solve_ms.push_back(f.solve_ms);
    if (!f.decoded) ++decode_failures;
    std::vector<TagSighting> sightings;
    bool any_valid = false;
    const bool have_multi = published(f.multi) && f.multi.tag_count >= 2;
    const auto T_layout_camera = f.multi.T_field_robot * optical;
    std::set<int> detected_ids;
    for (const auto& d : f.detections) {
      const bool known = field.layout.HasTag(d.tag_id);
      sightings.push_back(
          {d.tag_id, {d.center_x, d.center_y}, MinSide(d.corners)});
      if (!d.reject_reason.empty()) ++rejects[d.reject_reason];
      if (!known) {
        ++unknown_ids;
        margin_unknown.push_back(d.decision_margin);
        hamming_unknown.push_back(d.hamming);
        continue;
      }
      if (!d.valid) continue;
      margin_valid.push_back(d.decision_margin);
      hamming_valid.push_back(d.hamming);
      any_valid = true;
      detected_ids.insert(d.tag_id);
      auto& s = tags[d.tag_id];
      ++s.detected;
      s.margin.push_back(d.decision_margin);
      s.reprojection.push_back(d.reprojection_error);
      s.ambiguity.push_back(d.ambiguity);
      // The node's single-tag observation of this detection alone.
      const auto single = SolveMultiTagRobotPose({d}, 0, mounting, field.layout,
                                                 k, validation, solver_options);
      if (published(single)) {
        ++s.single_valid;
      } else if (IsSingleTagAmbiguityRejection(single)) {
        ++s.single_rejected_ambiguity;
      }
      if (have_multi) {
        const auto expected =
            Project(field.layout, d.tag_id, T_layout_camera, k, 0);
        double worst = 0;
        for (std::size_t c = 0; c < 4; ++c) {
          worst = std::max(
              worst,
              (expected.corners[c] -
               k.UndistortPoint({d.corners[2 * c], d.corners[2 * c + 1]}))
                  .norm());
        }
        s.corner_vs_multi_px.push_back(worst);
        if (worst > 0.25 * MinSide(d.corners)) ++s.misplaced;
        // Which planar-ambiguity branch is right, by the multi-tag pose.
        const auto truth_camera_tag =
            T_layout_camera.inverse() * *field.layout.GetTagPose(d.tag_id);
        const double primary =
            RotationDeg(d.T_camera_tag.rotation(), truth_camera_tag.rotation());
        const double secondary =
            d.secondary_reprojection_error_px >= 0
                ? RotationDeg(d.secondary_T_camera_tag.rotation(),
                              truth_camera_tag.rotation())
                : 180.0;
        if (primary > 15 && secondary < primary && secondary < 15) {
          ++s.flips;
          if (published(single) &&
              RotationDeg(single.T_field_robot.rotation(),
                          f.multi.T_field_robot.rotation()) > 10) {
            ++s.flips_accepted;
          }
        }
        if (published(single)) {
          s.single_vs_multi_m.push_back((single.T_field_robot.translation() -
                                         f.multi.T_field_robot.translation())
                                            .norm());
          s.single_vs_multi_yaw_deg.push_back(
              std::abs(std::remainder(
                  Yaw(single.T_field_robot) - Yaw(f.multi.T_field_robot),
                  2 * std::numbers::pi)) *
              kDeg);
        }
      }
    }
    if (any_valid) ++frames_with_valid;
    if (published(f.multi)) {
      ++frames_published;
      if (f.multi.tag_count == 1) ++frames_single_published;
      multi_reprojection.push_back(f.multi.reprojection_error_px);
    }
    if (have_multi) {
      ++frames_multi;
      multi_position[i] = T_layout_camera.translation();
      multi_rotation[i] = T_layout_camera.rotation().Log();
    }
    if (mocap && published(f.multi)) {
      const auto world_imu = mocap->At(sequence->origin_ns() + f.pts_ns);
      if (world_imu) {
        const auto world_camera = *world_imu * o.T_cam_imu->inverse();
        gt.push_back(
            {world_camera * T_layout_camera.inverse(), f.multi.tag_count});
        gt_frame.push_back(static_cast<std::int64_t>(i));
      }
    }
    // Every layout tag the multi-tag pose puts in view, detected or not.
    for (auto& [id, s] : tags) s.seen.resize(i + 1, false);
    for (const int id : detected_ids) {
      auto& s = tags[id];
      s.seen.resize(i + 1, false);
      s.seen[i] = true;
    }
    if (have_multi) {
      for (int id = 0; id < 1000; ++id) {
        if (!field.layout.HasTag(id)) continue;
        if (!Project(field.layout, id, T_layout_camera, k, 12).in_view)
          continue;
        auto& s = tags[id];
        s.seen.resize(i + 1, false);
        ++s.expected;
        if (detected_ids.count(id)) ++s.expected_detected;
      }
    }
    for (const auto& d : f.detections) {
      if (!d.valid || !field.layout.HasTag(d.tag_id)) continue;
      auto& s = tags[d.tag_id];
      s.corners.resize(i + 1);
      Eigen::Matrix<double, 8, 1> c;
      for (int j = 0; j < 8; ++j) c[j] = d.corners[j];
      s.corners[i] = c;
    }
    id_switches +=
        static_cast<std::size_t>(CountIdSwitches(previous, sightings));
    previous = std::move(sightings);
  }

  Summary out;
  const double duration_s =
      count > 1 ? static_cast<double>(frames[count - 1].pts_ns) / 1e9 : 0;
  out.Line("recording: %s%s%s", o.frames.c_str(),
           o.label.empty() ? "" : "  label: ", o.label.c_str());
  out.Line("frames: %zu over %.2f s (%.1f fps), %dx%d", count, duration_s,
           duration_s > 0 ? static_cast<double>(count - 1) / duration_s : 0.0,
           width, height);
  out.Line(
      "intrinsics: fx %.1f fy %.1f cx %.1f cy %.1f k1 %.4f k2 %.4f p1 %.5f "
      "p2 %.5f k3 %.4f%s%s",
      k.fx, k.fy, k.cx, k.cy, k.k1, k.k2, k.p1, k.p2, k.k3,
      o.equidistant ? "  (equidistant fisheye rectified to this pinhole)"
      : rectify     ? "  (OpenCV distortion rectified to this pinhole)"
                    : "",
      assumed_intrinsics ? "  ASSUMED from --hfov-deg: metric pose, range, "
                           "reprojection, ambiguity and multi/single numbers "
                           "are not meaningful"
                         : "");
  out.Line(
      "detector: decimate %.1f threads %d sigma %.1f refine %d hamming<=%d "
      "margin>=%.0f; floor prior %s; layout tags %zu",
      o.decimate, o.threads, o.sigma, o.refine_edges ? 1 : 0, o.hamming,
      o.min_margin, o.floor_prior ? "on" : "off", field.layout.size());
  if (o.detect_only) {
    out.Line(
        "DETECT-ONLY: no tag layout; every pose, range, reprojection, "
        "ambiguity, single/multi and expected-in-view number is "
        "meaningless; detection, decision margin, hamming, id and pixel "
        "jitter numbers stand");
  }
  out.Line("");
  out.Line("frames with >=1 valid layout tag: %zu (%.1f%%)", frames_with_valid,
           Percent(frames_with_valid, count));
  out.Line(
      "frames the node would publish valid: %zu (%.1f%%), single-tag %zu, "
      "multi-tag %zu",
      frames_published, Percent(frames_published, count),
      frames_single_published, frames_multi);
  out.Line(
      "detector failures: %zu; unknown ids decoded (false positives): %zu; "
      "id switches between associated detections: %zu",
      decode_failures, unknown_ids, id_switches);
  std::string reasons;
  for (const auto& [reason, n] : rejects) {
    reasons += reason + "=" + std::to_string(n) + " ";
  }
  out.Line("rejected detections by reason: %s",
           reasons.empty() ? "none" : reasons.c_str());
  out.Line("decision margin, valid: %s", Dist(margin_valid).c_str());
  out.Line("decision margin, unknown ids: %s", Dist(margin_unknown).c_str());
  out.Line("hamming, valid: %s; unknown ids: %s", Dist(hamming_valid).c_str(),
           Dist(hamming_unknown).c_str());
  out.Line("published joint reprojection error px: %s",
           Dist(multi_reprojection).c_str());
  const auto pos_jitter = SecondDifferenceJitter<3>(multi_position);
  const auto rot_jitter = SecondDifferenceJitter<3>(multi_rotation);
  out.Line(
      "multi-tag camera pose jitter (second difference, includes real "
      "acceleration): position sigma %.2f mm, rotation sigma %.3f deg (%zu "
      "samples)",
      pos_jitter.sigma_estimate * 1e3, rot_jitter.sigma_estimate * kDeg,
      pos_jitter.samples);
  out.Line(
      "time per frame (busy host, no realtime grant; ms): load+rectify %s; "
      "detect+tag pose %s; localize %s",
      Dist(load_ms).c_str(), Dist(detect_ms).c_str(), Dist(solve_ms).c_str());

  std::size_t all_flips = 0, all_flips_accepted = 0, all_single = 0,
              all_detected = 0, all_amb = 0, all_misplaced = 0;
  std::vector<double> all_svm, all_svm_yaw, all_corner, all_jitter;
  out.Line("");
  out.Line("%4s %6s %6s %8s %7s %6s %6s %6s %5s %5s %8s %9s %9s %8s", "tag",
           "det", "exp", "rate%", "dropout", "jitter", "margin", "reproj",
           "amb", "flips", "1tag_ok", "1v_p50_m", "1v_p95_m", "misplace");
  for (auto& [id, s] : tags) {
    s.seen.resize(count, false);
    s.corners.resize(count);
    const auto jitter = SecondDifferenceJitter<8>(s.corners);
    if (jitter.samples > 0) all_jitter.push_back(jitter.sigma_estimate);
    const auto dropout = LongestDropout(s.seen);
    out.Line(
        "%4d %6zu %6zu %8.1f %7zu %6.3f %6.0f %6.2f %5.2f %5zu %8zu %9.3f "
        "%9.3f %8zu",
        id, s.detected, s.expected, Percent(s.expected_detected, s.expected),
        dropout, jitter.sigma_estimate, Quantile(s.margin, 0.5),
        Quantile(s.reprojection, 0.5), Quantile(s.ambiguity, 0.5), s.flips,
        s.single_valid, Quantile(s.single_vs_multi_m, 0.5),
        Quantile(s.single_vs_multi_m, 0.95), s.misplaced);
    all_flips += s.flips;
    all_flips_accepted += s.flips_accepted;
    all_single += s.single_valid;
    all_detected += s.detected;
    all_amb += s.single_rejected_ambiguity;
    all_misplaced += s.misplaced;
    all_svm.insert(all_svm.end(), s.single_vs_multi_m.begin(),
                   s.single_vs_multi_m.end());
    all_svm_yaw.insert(all_svm_yaw.end(), s.single_vs_multi_yaw_deg.begin(),
                       s.single_vs_multi_yaw_deg.end());
    all_corner.insert(all_corner.end(), s.corner_vs_multi_px.begin(),
                      s.corner_vs_multi_px.end());
  }
  std::size_t expected = 0, expected_detected = 0;
  for (const auto& [id, s] : tags) {
    expected += s.expected;
    expected_detected += s.expected_detected;
  }
  out.Line("");
  out.Line(
      "expected-in-view detection rate (tags the multi-tag pose puts in view, "
      ">=12 px): %zu / %zu = %.1f%%",
      expected_detected, expected, Percent(expected_detected, expected));
  out.Line("corner jitter per tag, sigma px (second difference): %s",
           Dist(all_jitter).c_str());
  out.Line(
      "detected corner vs multi-tag reprojection, worst corner px: %s; "
      "misplaced (> 1/4 side) %zu",
      Dist(all_corner).c_str(), all_misplaced);
  out.Line(
      "single-tag: %zu of %zu valid detections would publish alone; %zu "
      "rejected by the ambiguity gate",
      all_single, all_detected, all_amb);
  out.Line(
      "single-tag planar-ambiguity flips (primary wrong branch vs multi-tag): "
      "%zu, of them published with yaw/attitude off > 10 deg: %zu",
      all_flips, all_flips_accepted);
  out.Line("single-tag vs multi-tag robot position m: %s",
           Dist(all_svm).c_str());
  out.Line("single-tag vs multi-tag yaw deg: %s", Dist(all_svm_yaw).c_str());

  if (mocap) {
    out.Line("");
    if (gt.size() < 2) {
      out.Line("ground truth: fewer than 2 frames with mocap");
    } else {
      // The layout is fixed in the mocap world: its estimated pose there,
      // per frame, scatters by the estimate's error (plus mocap and hand-eye
      // error). The reference is the per-axis median translation and the
      // rotation of the frame nearest it.
      std::vector<double> xs, ys, zs;
      for (const auto& g : gt) {
        xs.push_back(g.world_layout.translation().x());
        ys.push_back(g.world_layout.translation().y());
        zs.push_back(g.world_layout.translation().z());
      }
      const Eigen::Vector3d center(Quantile(xs, 0.5), Quantile(ys, 0.5),
                                   Quantile(zs, 0.5));
      std::size_t ref = 0;
      for (std::size_t i = 1; i < gt.size(); ++i) {
        if ((gt[i].world_layout.translation() - center).norm() <
            (gt[ref].world_layout.translation() - center).norm())
          ref = i;
      }
      std::vector<double> t_multi, r_multi, t_single, r_single;
      for (const auto& g : gt) {
        const double t = (g.world_layout.translation() - center).norm();
        const double r = RotationDeg(g.world_layout.rotation(),
                                     gt[ref].world_layout.rotation());
        (g.tag_count >= 2 ? t_multi : t_single).push_back(t);
        (g.tag_count >= 2 ? r_multi : r_single).push_back(r);
      }
      out.Line(
          "ground truth (mocap, layout pose held fixed in the world): "
          "%zu frames",
          gt.size());
      out.Line("  multi-tag position error: %s",
               Dist(t_multi, 1e3, " mm").c_str());
      out.Line("  multi-tag rotation error: %s",
               Dist(r_multi, 1, " deg").c_str());
      out.Line("  single-tag position error: %s",
               Dist(t_single, 1e3, " mm").c_str());
      out.Line("  single-tag rotation error: %s",
               Dist(r_single, 1, " deg").c_str());
    }
  }
  std::fputs(out.text.str().c_str(), stdout);

  if (!o.out_dir.empty()) {
    std::filesystem::create_directories(o.out_dir);
    std::ofstream(std::filesystem::path(o.out_dir) / "summary.txt")
        << out.text.str();
    std::ofstream det(std::filesystem::path(o.out_dir) / "detections.csv");
    det << "frame,pts_ns,tag_id,hamming,margin,valid,reject,reprojection_px,"
           "ambiguity,center_x,center_y,side_px,range_m,secondary_px,"
           "single_published,single_x_m,single_y_m,single_yaw_deg,"
           "single_reason,primary_vs_multi_deg,secondary_vs_multi_deg\n";
    std::ofstream fr(std::filesystem::path(o.out_dir) / "frames.csv");
    fr << "frame,pts_ns,detections,published,tag_count,x_m,y_m,z_m,yaw_deg,"
          "x_std_m,y_std_m,yaw_std_deg,reprojection_px,detect_ms,solve_ms\n";
    for (std::size_t i = 0; i < count; ++i) {
      const auto& f = frames[i];
      for (const auto& d : f.detections) {
        det << i << ',' << f.pts_ns << ',' << d.tag_id << ',' << d.hamming
            << ',' << d.decision_margin << ',' << (d.valid ? 1 : 0) << ','
            << d.reject_reason << ',' << d.reprojection_error << ','
            << d.ambiguity << ',' << d.center_x << ',' << d.center_y << ','
            << MinSide(d.corners) << ',' << d.T_camera_tag.translation().norm()
            << ',' << d.secondary_reprojection_error_px << ',';
        const auto single =
            d.valid && field.layout.HasTag(d.tag_id)
                ? SolveMultiTagRobotPose({d}, 0, mounting, field.layout, k,
                                         validation, solver_options)
                : RobotPoseEstimate{};
        det << (published(single) ? 1 : 0) << ','
            << single.T_field_robot.translation().x() << ','
            << single.T_field_robot.translation().y() << ','
            << Yaw(single.T_field_robot) * kDeg << ','
            << PoseInvalidReasonToString(single.validity.reason) << ',';
        if (published(f.multi) && f.multi.tag_count >= 2 &&
            field.layout.HasTag(d.tag_id)) {
          const auto truth = (f.multi.T_field_robot * optical).inverse() *
                             *field.layout.GetTagPose(d.tag_id);
          det << RotationDeg(d.T_camera_tag.rotation(), truth.rotation()) << ','
              << RotationDeg(d.secondary_T_camera_tag.rotation(),
                             truth.rotation())
              << '\n';
        } else {
          det << ",\n";
        }
      }
      const auto& t = f.multi.T_field_robot.translation();
      fr << i << ',' << f.pts_ns << ',' << f.detections.size() << ','
         << (published(f.multi) ? 1 : 0) << ',' << f.multi.tag_count << ','
         << t.x() << ',' << t.y() << ',' << t.z() << ','
         << Yaw(f.multi.T_field_robot) * kDeg << ',' << f.multi.std_devs.x_std_m
         << ',' << f.multi.std_devs.y_std_m << ','
         << f.multi.std_devs.yaw_std_rad * kDeg << ','
         << f.multi.reprojection_error_px << ',' << f.detect_ms << ','
         << f.solve_ms << '\n';
    }
  }
  return 0;
}

}  // namespace
}  // namespace talos::vision::evaluation

int main(int argc, char** argv) {
  return talos::vision::evaluation::Run(
      talos::vision::evaluation::Parse(argc, argv));
}
