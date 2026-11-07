// Pinhole projection of field-layout AprilTags into a simulated camera.
//
// Given the robot's field pose, a camera's robot-relative mount, and its
// intrinsics, SimulateVisibleTags answers "what would this camera detect?"
// with the same shape as real detector output: pixel corners in AprilTag
// order, center, and the camera-relative tag pose.
//
// Frame conventions (all right-handed):
//   field:  WPILib NWU, +X away from the blue wall, +Y left, +Z up.
//   robot:  +X forward, +Y left, +Z up. T_field_robot = Trans(x, y, 0)*Rz(yaw).
//   camera: optical frame, +X right, +Y down, +Z forward (out of the lens).
//   tag:    WPILib layout convention: +X out of the tag face (normal),
//           +Z up, +Y completing the right-handed frame; corners ordered
//           like GetTagModelCorners.
// Mount angles are extrinsic Z-Y-X applied in the robot frame: yaw about +Z,
// then pitch about the yawed +Y (positive tips the lens downward), then roll
// about the resulting +X. Zero mount = lens facing robot-forward, upright.
//
// A tag is reported only when all four corners land inside the image. Real
// detectors sometimes fire on partial tags; the sim deliberately excludes
// them so estimator tests never train on an observation the real pipeline
// would only produce unreliably. Other honest limitations: no occlusion by
// field structures or other robots, no dropped frames. The
// ambiguity score is a deterministic incidence-angle heuristic, not a
// measured homography property.

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

#include "talOS/vision/calibration.h"

namespace talos::vision::field_sim {

struct SimCameraView {
  CameraIntrinsics intrinsics;
  // T_robot_camera: maps camera-frame points into the robot frame.
  CameraMounting mounting = CameraMounting::Identity();
  double max_range_m = 8.0;
  // Minimum projected tag edge length; below this a real detector's quads
  // fall apart (mirrors quad_min_size in spirit, in pixels).
  double min_tag_px = 12.0;
  // Minimum tag-face incidence: -normal.z() in the camera frame must clear
  // this. 1.0 is dead-on, 0.0 is edge-on.
  double facing_min = 0.2;
  // Fast-path corner/center observation noise, standard deviation in pixels.
  double pixel_noise_std = 0.0;
  // Rendered sensor noise, standard deviation in 8-bit grayscale levels.
  double sensor_noise_std_gray = 0.0;
  // Frame-wide multiplicative exposure gain standard deviation (unitless).
  double exposure_gain_std = 0.0;
  double exposure_ms = 0.0;
  uint64_t rng_seed = 0;
};

struct SimTagDetection {
  int tag_id = 0;
  // Pixel corners in GetTagModelCorners order: bottom-left, bottom-right,
  // top-right, top-left as seen facing the tag face.
  std::array<Eigen::Vector2d, 4> corners;
  Eigen::Vector2d center = Eigen::Vector2d::Zero();
  // T_camera_tag: maps tag-frame points into the camera frame.
  talos::geometry::SE3<double> T_camera_tag =
      talos::geometry::SE3<double>::Identity();
  double ambiguity = 0.0;
};

// Builds T_robot_camera from position plus Z-Y-X extrinsic degrees. This is
// the same mount vocabulary as the studio VisionConfig drawer, so a mount
// tuned in the UI means the same thing in sim.
inline CameraMounting MountFromXyzYpr(double x, double y, double z,
                                      double yaw_deg, double pitch_deg,
                                      double roll_deg) {
  // Local pi: <cmath>'s M_PI is unavailable under strict standard modes
  // (notably Linux glibc with -std=c++2x), and this header must compile
  // under every toolchain in the matrix.
  constexpr double kPi = 3.14159265358979323846;
  constexpr double kDeg = kPi / 180.0;
  return CameraMounting::FromExtrinsics(x, y, z, roll_deg * kDeg,
                                        pitch_deg * kDeg, yaw_deg * kDeg);
}

// Builds T_field_robot for a flat-floor robot pose.
inline talos::geometry::SE3<double> RobotPoseOnField(double x_m, double y_m,
                                                     double yaw_rad) {
  return talos::geometry::SE3<double>(
      talos::geometry::SO3<double>::Exp(Eigen::Vector3d(0.0, 0.0, yaw_rad)),
      Eigen::Vector3d(x_m, y_m, 0.0));
}

// Projects every layout tag into the camera; returns the visible ones in
// increasing tag-id order. Only an explicit caller-owned RNG enables noise.
inline void SimulateVisibleTags(
    const FieldLayout& layout, const SimCameraView& cam,
    const talos::geometry::SE3<double>& T_field_robot, double tag_size_m,
    std::vector<SimTagDetection>* output, bool geometry_only = false,
    std::mt19937_64* rng = nullptr) {
  if (!output) return;
  auto& out = *output;
  out.clear();
  if (!(tag_size_m > 0.0) || !cam.intrinsics.IsValid()) return;
  if (!(cam.max_range_m > 0.0) || !(cam.min_tag_px >= 0.0)) return;

  const talos::geometry::SE3<double> T_field_camera =
      T_field_robot * cam.mounting.T_robot_camera;
  const talos::geometry::SE3<double> T_camera_field = T_field_camera.inverse();
  // Single source of truth with the estimator: same frame, same order, so
  // projected corners pair with detected corners by index.
  const std::array<Eigen::Vector3d, 4> model = GetTagModelCorners(tag_size_m);

  std::normal_distribution<double> noise(
      0.0, cam.pixel_noise_std > 0.0 ? cam.pixel_noise_std : 1.0);
  const bool add_noise = rng && !geometry_only && cam.pixel_noise_std > 0.0;

  // FieldLayout offers lookup by id only; probe the plausible id range.
  // FRC fields number tags densely from 1, so 256 covers every season.
  for (int tag_id = 1; tag_id <= 256; ++tag_id) {
    const auto tag_pose = layout.GetTagPose(tag_id);
    if (!tag_pose.has_value()) continue;
    const talos::geometry::SE3<double> T_camera_tag =
        T_camera_field * *tag_pose;

    // Tag face must look at the lens: +X out of the tag face should point
    // back toward the camera (-Z side of the camera frame).
    const Eigen::Vector3d normal_cam =
        T_camera_tag.rotation().rotationMatrix().col(0);
    const double facing = -normal_cam.z();
    if (geometry_only ? !(normal_cam.dot(T_camera_tag.translation()) < 0)
                      : !(facing >= cam.facing_min))
      continue;

    if (!geometry_only && T_camera_tag.translation().norm() > cam.max_range_m)
      continue;

    std::array<Eigen::Vector2d, 4> px;
    bool ok = true;
    for (size_t i = 0; i < 4; ++i) {
      const Eigen::Vector3d p_cam = T_camera_tag.act(model[i]);
      Eigen::Vector2d uv;
      if (!cam.intrinsics.ProjectPoint(p_cam, &uv)) {
        ok = false;
        break;
      }
      if (!geometry_only &&
          (uv.x() < 0.0 || uv.y() < 0.0 ||
           uv.x() > static_cast<double>(cam.intrinsics.image_width) ||
           uv.y() > static_cast<double>(cam.intrinsics.image_height))) {
        ok = false;
        break;
      }
      px[i] = uv;
    }
    if (!ok) continue;

    double min_edge = 0.0;
    for (size_t i = 0; i < 4; ++i) {
      const double edge = (px[(i + 1) % 4] - px[i]).norm();
      if (i == 0 || edge < min_edge) min_edge = edge;
    }
    if (!geometry_only && min_edge < cam.min_tag_px) continue;
    if (geometry_only) {
      double xmin = px[0].x(), xmax = xmin, ymin = px[0].y(), ymax = ymin;
      for (const auto& p : px) {
        xmin = std::min(xmin, p.x());
        xmax = std::max(xmax, p.x());
        ymin = std::min(ymin, p.y());
        ymax = std::max(ymax, p.y());
      }
      if (xmax < 0 || ymax < 0 || xmin >= cam.intrinsics.image_width ||
          ymin >= cam.intrinsics.image_height)
        continue;
    }

    SimTagDetection det;
    det.tag_id = tag_id;
    det.T_camera_tag = T_camera_tag;
    // Face-on tags disambiguate cleanly; grazing tags do not. Heuristic,
    // deterministic, and documented as such (see header).
    det.ambiguity = 0.02 + 0.6 * (1.0 - std::min(1.0, facing));
    det.center = Eigen::Vector2d::Zero();
    for (size_t i = 0; i < 4; ++i) {
      det.corners[i] = px[i];
      det.center += px[i];
    }
    det.center *= 0.25;
    if (add_noise) {
      for (size_t i = 0; i < 4; ++i) {
        det.corners[i].x() += noise(*rng);
        det.corners[i].y() += noise(*rng);
      }
      det.center.x() += noise(*rng);
      det.center.y() += noise(*rng);
    }
    out.push_back(det);
  }
}

inline std::vector<SimTagDetection> SimulateVisibleTags(
    const FieldLayout& layout, const SimCameraView& cam,
    const talos::geometry::SE3<double>& T_field_robot, double tag_size_m,
    bool geometry_only = false, std::mt19937_64* rng = nullptr) {
  std::vector<SimTagDetection> out;
  SimulateVisibleTags(layout, cam, T_field_robot, tag_size_m, &out,
                      geometry_only, rng);
  return out;
}

// Camera description used by configured simulation nodes.
struct SimCameraEntry {
  int id = 0;
  std::string name;
  SimCameraView view;
};

}  // namespace talos::vision::field_sim
