#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

extern "C" {
#include "apriltag_pose.h"
#include "common/matd.h"
}

#include "talOS/geometry/se3.h"
#include "talOS/geometry/so3.h"
#include "talOS/vision/calibration.h"

namespace talos::vision {

namespace detail {
// Upstream does not promise candidate ordering. Keep pose/error pairs together.
inline double OrderPoseCandidates(talos::geometry::SE3<double>& primary,
                                  talos::geometry::SE3<double>& secondary,
                                  double& primary_error,
                                  double& secondary_error) {
  if (secondary_error < primary_error) {
    std::swap(primary, secondary);
    std::swap(primary_error, secondary_error);
  }
  if (!std::isfinite(secondary_error)) return 0;
  if (secondary_error <=
      std::numeric_limits<double>::epsilon() * std::max(1.0, primary_error))
    return 1;
  return std::clamp(primary_error / secondary_error, 0.0, 1.0);
}
}  // namespace detail

// Calculates root-mean-square reprojection error between observed 2D image
// corners and projected 3D tag model corners through camera intrinsics.
inline double ComputeReprojectionError(
    const std::array<float, 8>& observed_corners,
    const talos::geometry::SE3<double>& T_camera_tag,
    const CameraIntrinsics& intrinsics, double tag_size_m) {
  const auto model_corners = GetTagModelCorners(tag_size_m);
  double sum_sq_err = 0.0;

  for (size_t i = 0; i < 4; ++i) {
    const Eigen::Vector3d p_cam = T_camera_tag.act(model_corners[i]);
    Eigen::Vector2d uv_proj;
    if (!intrinsics.ProjectPoint(p_cam, &uv_proj)) {
      return 1e9;  // Point behind or on focal plane
    }

    const double obs_u = observed_corners[i * 2];
    const double obs_v = observed_corners[i * 2 + 1];
    const double du = obs_u - uv_proj.x();
    const double dv = obs_v - uv_proj.y();
    sum_sq_err += du * du + dv * dv;
  }

  return std::sqrt(sum_sq_err / 4.0);
}

// Decomposes planar homography H into camera-to-tag SE(3) pose estimates
// using AprilTag Orthogonal Iteration.
// Returns primary and secondary (ambiguous dual) solutions.
inline bool EstimateTagPoseFromHomography(
    const std::array<float, 9>& homography, const CameraIntrinsics& intrinsics,
    double tag_size_m, talos::geometry::SE3<double>* primary_pose,
    talos::geometry::SE3<double>* secondary_pose, double* primary_err_out,
    double* secondary_err_out, double* ambiguity_out,
    const std::array<float, 8>* observed_corners = nullptr) {
  if (!intrinsics.IsValid() || tag_size_m <= 0.0) {
    return false;
  }

  double h_data[9];
  for (int i = 0; i < 9; ++i) {
    h_data[i] = static_cast<double>(homography[i]);
  }
  apriltag_detection_t temp_det{};
  if (observed_corners) {
    Eigen::Matrix<double, 8, 8> a;
    Eigen::Matrix<double, 8, 1> b;
    constexpr double xs[4] = {-1, 1, 1, -1};
    // Upstream detector p[0..3] corresponds to (-,+),(+,+),(+,-),(-,-).
    constexpr double ys[4] = {1, 1, -1, -1};
    for (int c = 0; c < 4; ++c) {
      const auto uv = intrinsics.UndistortPoint(
          {(*observed_corners)[2 * c], (*observed_corners)[2 * c + 1]});
      const double x = xs[c], y = ys[c], u = uv.x(), v = uv.y();
      a.row(2 * c) << x, y, 1, 0, 0, 0, -u * x, -u * y;
      a.row(2 * c + 1) << 0, 0, 0, x, y, 1, -v * x, -v * y;
      b(2 * c) = u;
      b(2 * c + 1) = v;
      temp_det.p[c][0] = u;
      temp_det.p[c][1] = v;
    }
    const auto lu = a.fullPivLu();
    if (!lu.isInvertible()) return false;
    const Eigen::Matrix<double, 8, 1> h = lu.solve(b);
    for (int i = 0; i < 8; ++i) h_data[i] = h(i);
    h_data[8] = 1;
  } else {
    constexpr double xs[4] = {-1, 1, 1, -1};
    constexpr double ys[4] = {1, 1, -1, -1};
    for (int c = 0; c < 4; ++c) {
      const double w = h_data[6] * xs[c] + h_data[7] * ys[c] + h_data[8];
      if (std::abs(w) < 1e-12) return false;
      temp_det.p[c][0] =
          (h_data[0] * xs[c] + h_data[1] * ys[c] + h_data[2]) / w;
      temp_det.p[c][1] =
          (h_data[3] * xs[c] + h_data[4] * ys[c] + h_data[5]) / w;
    }
  }
  temp_det.H = matd_create_data(3, 3, h_data);
  for (int c = 0; c < 4; ++c) {
    temp_det.c[0] += temp_det.p[c][0] * 0.25;
    temp_det.c[1] += temp_det.p[c][1] * 0.25;
  }

  apriltag_detection_info_t info{
      .det = &temp_det,
      .tagsize = tag_size_m,
      .fx = intrinsics.fx,
      .fy = intrinsics.fy,
      .cx = intrinsics.cx,
      .cy = intrinsics.cy,
  };

  apriltag_pose_t p1{};
  apriltag_pose_t p2{};
  double err1 = 0.0;
  double err2 = 0.0;

  estimate_tag_pose_orthogonal_iteration(&info, &err1, &p1, &err2, &p2, 50);
  matd_destroy(temp_det.H);

  if (!p1.R || !p1.t) {
    if (p1.R) matd_destroy(p1.R);
    if (p1.t) matd_destroy(p1.t);
    if (p2.R) matd_destroy(p2.R);
    if (p2.t) matd_destroy(p2.t);
    return false;
  }

  // Convert p1 to SE3
  Eigen::Matrix3d R1;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      R1(r, c) = MATD_EL(p1.R, r, c);
    }
  }
  Eigen::Vector3d t1(MATD_EL(p1.t, 0, 0), MATD_EL(p1.t, 1, 0),
                     MATD_EL(p1.t, 2, 0));
  talos::geometry::SE3<double> pose1(
      talos::geometry::SO3<double>::FromMatrixNormalized(R1), t1);

  // Convert p2 to SE3 if available
  talos::geometry::SE3<double> pose2 = pose1;
  const bool has_secondary = p2.R && p2.t;
  if (p2.R && p2.t) {
    Eigen::Matrix3d R2;
    for (int r = 0; r < 3; ++r) {
      for (int c = 0; c < 3; ++c) {
        R2(r, c) = MATD_EL(p2.R, r, c);
      }
    }
    Eigen::Vector3d t2(MATD_EL(p2.t, 0, 0), MATD_EL(p2.t, 1, 0),
                       MATD_EL(p2.t, 2, 0));
    pose2 = talos::geometry::SE3<double>(
        talos::geometry::SO3<double>::FromMatrixNormalized(R2), t2);
  }

  matd_destroy(p1.R);
  matd_destroy(p1.t);
  if (p2.R) matd_destroy(p2.R);
  if (p2.t) matd_destroy(p2.t);

  // The orthogonal-iteration solutions live in the solver's tag frame;
  // convert to the WPILib/layout frame before anyone composes or reprojects
  // them. (Reprojection is numerically unchanged: the model corners moved
  // frames with the pose, so the pairing projects to the same pixels.)
  pose1 = SolverTagFrameToLayoutFrame(pose1);
  pose2 = SolverTagFrameToLayoutFrame(pose2);

  if (observed_corners) {
    err1 = ComputeReprojectionError(*observed_corners, pose1, intrinsics,
                                    tag_size_m);
    err2 = has_secondary ? ComputeReprojectionError(*observed_corners, pose2,
                                                    intrinsics, tag_size_m)
                         : std::numeric_limits<double>::infinity();
  } else if (!has_secondary) {
    err2 = std::numeric_limits<double>::infinity();
  }
  const double ambiguity =
      detail::OrderPoseCandidates(pose1, pose2, err1, err2);
  if (primary_err_out) *primary_err_out = err1;
  if (secondary_err_out) *secondary_err_out = err2;
  if (primary_pose) *primary_pose = pose1;
  if (secondary_pose) *secondary_pose = pose2;
  if (ambiguity_out) *ambiguity_out = ambiguity;
  return true;
}

inline bool EstimateTagPoseFromHomography(
    const std::array<float, 9>& h, const CameraIntrinsics& intrinsics,
    double size, talos::geometry::SE3<double>* primary,
    talos::geometry::SE3<double>* secondary, double* ambiguity,
    const std::array<float, 8>* corners = nullptr) {
  return EstimateTagPoseFromHomography(h, intrinsics, size, primary, secondary,
                                       nullptr, nullptr, ambiguity, corners);
}

// Criteria for validating pose estimates before robot localization.
struct PoseValidationOptions {
  double max_reprojection_error_px{3.5};
  double max_ambiguity{0.45};
  double min_distance_m{0.1};
  double max_distance_m{15.0};
  double min_z_m{0.05};
};

// Reasons for pose invalidity.
enum class PoseInvalidReason {
  kValid,
  kTagNotInLayout,
  kHighReprojectionError,
  kHighAmbiguity,
  kBehindCamera,
  kTooFar,
  kTooClose,
  kInvalidRotation,
  kDegenerateHomography,
};

constexpr std::string_view PoseInvalidReasonToString(PoseInvalidReason reason) {
  switch (reason) {
    case PoseInvalidReason::kValid:
      return "Valid";
    case PoseInvalidReason::kTagNotInLayout:
      return "Tag ID not found in field layout";
    case PoseInvalidReason::kHighReprojectionError:
      return "Reprojection error exceeds threshold";
    case PoseInvalidReason::kHighAmbiguity:
      return "Ambiguity exceeds threshold";
    case PoseInvalidReason::kBehindCamera:
      return "Tag is behind camera (Z <= 0)";
    case PoseInvalidReason::kTooFar:
      return "Tag distance exceeds maximum limit";
    case PoseInvalidReason::kTooClose:
      return "Tag distance is below minimum limit";
    case PoseInvalidReason::kInvalidRotation:
      return "Rotation matrix is non-orthogonal or reflection";
    case PoseInvalidReason::kDegenerateHomography:
      return "Homography decomposition failed";
  }
  return "Unknown";
}

struct PoseValidity {
  bool valid{false};
  PoseInvalidReason reason{PoseInvalidReason::kValid};
  std::string message;
};

struct ObservationStdDevs {
  double x_std_m{0.1}, y_std_m{0.1}, yaw_std_rad{0.1};
};
inline ObservationStdDevs ComputeObservationStdDevs(
    double distance, int count, double ambiguity, double base_xy_std_m = 0.08,
    double base_yaw_std_rad = 0.06) {
  const double s = std::pow(std::max(distance, 0.5), 2) /
                   std::sqrt(std::max(count, 1)) *
                   (1 + 2 * std::max(0.0, ambiguity));
  return {base_xy_std_m * s, base_xy_std_m * s, base_yaw_std_rad * s};
}
struct DetectionResult {
  int tag_id{-1}, hamming{0};
  float center_x{0}, center_y{0}, decision_margin{0};
  std::array<float, 8> corners{};
  std::array<float, 9> homography{};
  talos::geometry::SE3<double> T_camera_tag{};
  talos::geometry::SE3<double> secondary_T_camera_tag{};
  double reprojection_error{0}, secondary_reprojection_error_px{-1};
  double ambiguity{0};
  bool valid{false};
};
using TagDetection3D = DetectionResult;
struct RobotPoseEstimate {
  talos::geometry::SE3<double> T_field_robot{}, T_camera_tag{};
  int tag_id{-1}, tag_count{0};
  uint32_t camera_id{0};
  double average_distance_m{0}, reprojection_error_px{0}, ambiguity{0};
  ObservationStdDevs std_devs{};
  PoseValidity validity;
};
inline Eigen::Vector3d FloorResidual(const talos::geometry::SE3<double>& p) {
  const auto r = p.rotation().rotationMatrix();
  return {p.translation().z(), std::atan2(r(2, 1), r(2, 2)),
          std::atan2(-r(2, 0), std::hypot(r(2, 1), r(2, 2)))};
}
inline double FloorPenalty(const talos::geometry::SE3<double>& p) {
  const auto v = FloorResidual(p);
  return std::abs(v.x()) + 0.5 * std::hypot(v.y(), v.z());
}
inline RobotPoseEstimate ComputeRobotFieldPose(
    int tag_id, uint32_t camera_id,
    const talos::geometry::SE3<double>& T_camera_tag,
    const CameraMounting& mounting, const FieldLayout& layout,
    double reprojection_error = 0, double ambiguity = 0,
    const PoseValidationOptions& options = {},
    const talos::geometry::SE3<double>* secondary_T_camera_tag = nullptr,
    double secondary_reprojection_error_px = -1) {
  RobotPoseEstimate out;
  out.tag_id = tag_id;
  out.camera_id = camera_id;
  out.T_camera_tag = T_camera_tag;
  out.reprojection_error_px = reprojection_error;
  out.ambiguity = ambiguity;
  auto reject = [&](PoseInvalidReason reason) {
    out.validity = {false, reason,
                    std::string(PoseInvalidReasonToString(reason))};
    return out;
  };
  const auto field = layout.GetTagPose(tag_id);
  if (!field) return reject(PoseInvalidReason::kTagNotInLayout);
  auto compose = [&](const auto& p) {
    return *field * p.inverse() * mounting.T_robot_camera.inverse();
  };
  out.T_field_robot = compose(T_camera_tag);
  bool resolved = false;
  if (secondary_T_camera_tag && secondary_reprojection_error_px >= 0 &&
      secondary_reprojection_error_px <=
          options.max_reprojection_error_px * 1.5) {
    const auto secondary = compose(*secondary_T_camera_tag);
    const double a = FloorPenalty(out.T_field_robot),
                 b = FloorPenalty(secondary);
    double other = b;
    if (b + 0.15 < a) {
      out.T_field_robot = secondary;
      out.T_camera_tag = *secondary_T_camera_tag;
      out.reprojection_error_px = secondary_reprojection_error_px;
      other = a;
    }
    resolved = FloorPenalty(out.T_field_robot) < 0.25 && other > 0.5;
  }
  const auto& p = out.T_camera_tag;
  const double d = p.translation().norm();
  if (!p.matrix().allFinite())
    return reject(PoseInvalidReason::kInvalidRotation);
  if (p.translation().z() <= options.min_z_m)
    return reject(PoseInvalidReason::kBehindCamera);
  if (d < options.min_distance_m) return reject(PoseInvalidReason::kTooClose);
  if (d > options.max_distance_m) return reject(PoseInvalidReason::kTooFar);
  if (!std::isfinite(out.reprojection_error_px) ||
      out.reprojection_error_px >
          options.max_reprojection_error_px * (resolved ? 1.5 : 1))
    return reject(PoseInvalidReason::kHighReprojectionError);
  if (!std::isfinite(ambiguity) ||
      (ambiguity > options.max_ambiguity && !resolved))
    return reject(PoseInvalidReason::kHighAmbiguity);
  const auto r = p.rotation().rotationMatrix();
  if ((r * r.transpose() - Eigen::Matrix3d::Identity()).norm() > 1e-2 ||
      r.determinant() <= 0)
    return reject(PoseInvalidReason::kInvalidRotation);
  out.tag_count = 1;
  out.average_distance_m = d;
  out.std_devs = ComputeObservationStdDevs(d, 1, ambiguity);
  out.validity = {true, PoseInvalidReason::kValid, "Valid"};
  return out;
}

// Joint pinhole PnP uses fixed per-observation undistortion, right SE(3)
// increments, and a weak NWU ground-plane prior.
inline RobotPoseEstimate SolveMultiTagRobotPose(
    const std::vector<TagDetection3D>& observations, uint32_t camera_id,
    const CameraMounting& mounting, const FieldLayout& layout,
    const CameraIntrinsics& intrinsics,
    const PoseValidationOptions& options = {}) {
  std::vector<const TagDetection3D*> tags;
  RobotPoseEstimate out;
  out.camera_id = camera_id;
  double best = 1e100, distance_sum = 0;
  for (const auto& t : observations) {
    const double d = t.T_camera_tag.translation().norm();
    if (!t.valid || !layout.HasTag(t.tag_id) || !std::isfinite(d) ||
        d < options.min_distance_m || d > options.max_distance_m ||
        t.T_camera_tag.translation().z() <= options.min_z_m)
      continue;
    if (std::any_of(tags.begin(), tags.end(),
                    [&](const auto* q) { return q->tag_id == t.tag_id; }))
      continue;
    tags.push_back(&t);
    distance_sum += d;
    auto seed_options = options;
    seed_options.max_ambiguity = 1;
    auto seed = ComputeRobotFieldPose(
        t.tag_id, camera_id, t.T_camera_tag, mounting, layout,
        t.reprojection_error, t.ambiguity, seed_options,
        &t.secondary_T_camera_tag, t.secondary_reprojection_error_px);
    const double score =
        seed.reprojection_error_px + FloorPenalty(seed.T_field_robot);
    if (seed.validity.valid && score < best) {
      best = score;
      out = seed;
    }
  }
  if (tags.empty() || best == 1e100 || !intrinsics.IsValid()) {
    out.validity = {false, PoseInvalidReason::kDegenerateHomography,
                    "No usable PnP seed"};
    return out;
  }
  if (tags.size() == 1) {
    const auto& t = *tags.front();
    return ComputeRobotFieldPose(t.tag_id, camera_id, t.T_camera_tag, mounting,
                                 layout, t.reprojection_error, t.ambiguity,
                                 options, &t.secondary_T_camera_tag,
                                 t.secondary_reprojection_error_px);
  }
  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Vector2d> pixels;
  points.reserve(4 * tags.size());
  pixels.reserve(4 * tags.size());
  for (const auto* t : tags) {
    const auto field = *layout.GetTagPose(t->tag_id);
    const auto corners = GetTagModelCorners(layout.GetTagSize(t->tag_id));
    for (int j = 0; j < 4; ++j) {
      points.push_back(field.act(corners[j]));
      pixels.push_back(intrinsics.UndistortPoint(
          {t->corners[2 * j], t->corners[2 * j + 1]}));
    }
  }
  const int rows = static_cast<int>(2 * points.size() + 3);
  auto residual = [&](const auto& pose, Eigen::VectorXd& r) {
    const auto camera_field =
        mounting.T_robot_camera.inverse() * pose.inverse();
    for (size_t i = 0; i < points.size(); ++i) {
      const auto p = camera_field.act(points[i]);
      if (!p.allFinite() || p.z() <= options.min_z_m) return false;
      r(2 * i) = intrinsics.fx * p.x() / p.z() + intrinsics.cx - pixels[i].x();
      r(2 * i + 1) =
          intrinsics.fy * p.y() / p.z() + intrinsics.cy - pixels[i].y();
    }
    r.tail<3>() = FloorResidual(pose);
    return r.allFinite();
  };
  Eigen::VectorXd r(rows), trial_r(rows), shifted(rows);
  if (!residual(out.T_field_robot, r)) {
    out.validity = {false, PoseInvalidReason::kBehindCamera,
                    "Invalid initial projection"};
    return out;
  }
  Eigen::MatrixXd jac(rows, 6);
  double lambda = 1e-3;
  for (int iteration = 0; iteration < 10; ++iteration) {
    bool ok = true;
    for (int c = 0; c < 6; ++c) {
      Eigen::Matrix<double, 6, 1> step = Eigen::Matrix<double, 6, 1>::Zero();
      step(c) = 1e-6;
      if (!residual(out.T_field_robot.boxplus(step), shifted)) {
        ok = false;
        break;
      }
      jac.col(c) = (shifted - r) / 1e-6;
    }
    if (!ok) break;
    Eigen::Matrix<double, 6, 6> normal = jac.transpose() * jac;
    normal.diagonal().array() += lambda;
    const Eigen::Matrix<double, 6, 1> delta =
        normal.ldlt().solve(-jac.transpose() * r);
    if (!delta.allFinite()) break;
    const auto trial = out.T_field_robot.boxplus(delta);
    if (residual(trial, trial_r) && trial_r.squaredNorm() < r.squaredNorm()) {
      out.T_field_robot = trial;
      r = trial_r;
      lambda *= 0.25;
      if (delta.norm() < 1e-7) break;
    } else {
      lambda *= 10;
    }
  }
  out.tag_id = -1;
  out.tag_count = static_cast<int>(tags.size());
  out.average_distance_m = distance_sum / tags.size();
  out.reprojection_error_px =
      std::sqrt(r.head(rows - 3).squaredNorm() / points.size());
  out.ambiguity = 0;
  out.std_devs =
      ComputeObservationStdDevs(out.average_distance_m, out.tag_count, 0);
  out.validity = {
      out.reprojection_error_px <= options.max_reprojection_error_px,
      PoseInvalidReason::kValid, "Valid"};
  if (!out.validity.valid)
    out.validity = {false, PoseInvalidReason::kHighReprojectionError,
                    "Joint PnP residual too high"};
  return out;
}

}  // namespace talos::vision
