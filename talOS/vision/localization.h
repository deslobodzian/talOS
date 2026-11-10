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
      const Eigen::Index row = Eigen::Index{2} * c;
      const auto corner = static_cast<std::size_t>(row);
      const auto uv = intrinsics.UndistortPoint(
          {(*observed_corners)[corner], (*observed_corners)[corner + 1]});
      const double x = xs[c], y = ys[c], u = uv.x(), v = uv.y();
      a.row(row) << x, y, 1, 0, 0, 0, -u * x, -u * y;
      a.row(row + 1) << 0, 0, 0, x, y, 1, -v * x, -v * y;
      b(row) = u;
      b(row + 1) = v;
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

  const int new_minima =
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
      has_secondary ? detail::OrderPoseCandidates(pose1, pose2, err1, err2)
                    : (new_minima > 1 ? 1.0 : 0.0);
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
  double max_ambiguity{0.2};
  double min_distance_m{0.1};
  double max_distance_m{15.0};
  double min_z_m{0.05};
};

// Measurement noise, extrinsic calibration uncertainty and robust loss, in
// pixels, metres and radians. mount_* model the camera extrinsic calibration
// uncertainty (not sim-only): a mount whose true angle is off by ~1 deg puts a
// `range * mount_angle_std_rad` lateral bias in x/y that the corner noise does
// not admit, so it is added to the published covariance before the floors.
struct PoseSolverOptions {
  double pixel_std_px{0.33};
  double floor_z_std_m{0.05};
  double floor_tilt_std_rad{0.05};
  double huber_delta_px{2.0};
  double min_xy_std_m{0.02};
  double min_yaw_std_rad{0.035};
  double mount_angle_std_rad{0.0175};
  double mount_translation_std_m{0.01};
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
  kInvalidCovariance,
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
    case PoseInvalidReason::kInvalidCovariance:
      return "Joint pose covariance is not finite positive definite";
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
  // Why this detection is excluded, in the offline-recording vocabulary
  // (empty = usable). The pipeline sets "hamming", "decision_margin" or
  // "reprojection"; AnnotateDetectionReasons below adds "range" and
  // "not_in_layout"; AnnotateAmbiguityRejection adds "ambiguity" only when a
  // single-tag observation is actually rejected for it. Never published.
  std::string reject_reason;
};
using TagDetection3D = DetectionResult;

// Annotates each detection's reject_reason for offline recording, mirroring
// the camera node's range filter and layout membership. Only adds a reason; a
// detection already marked by the pipeline keeps it. "range" covers a
// non-finite range, a range above `max_range_m`, behind the camera, too
// close, or too far; "not_in_layout" a tag absent from the layout. The
// single-tag ambiguity gate is applied separately by
// AnnotateAmbiguityRejection below, never per detection.
inline void AnnotateDetectionReasons(std::vector<TagDetection3D>& detections,
                                     const FieldLayout& layout,
                                     double max_range_m,
                                     const PoseValidationOptions& validation) {
  for (auto& d : detections) {
    if (!d.reject_reason.empty()) continue;
    if (!layout.HasTag(d.tag_id)) {
      d.reject_reason = "not_in_layout";
      continue;
    }
    const auto& p = d.T_camera_tag;
    const double range = p.translation().norm();
    const double z = p.translation().z();
    if (!std::isfinite(range) || range > max_range_m ||
        z <= validation.min_z_m || range < validation.min_distance_m ||
        range > validation.max_distance_m) {
      d.reject_reason = "range";
      continue;
    }
  }
}

struct RobotPoseEstimate {
  talos::geometry::SE3<double> T_field_robot{}, T_camera_tag{};
  int tag_id{-1}, tag_count{0};
  uint32_t camera_id{0};
  double average_distance_m{0}, reprojection_error_px{0}, ambiguity{0};
  ObservationStdDevs std_devs{};
  PoseValidity validity;
};
// True when `estimate` is the single-tag observation the ambiguity gate
// rejected. Multi-tag joint solves never carry this reason: the joint geometry
// resolves the planar ambiguity, so their detections stay decode-valid.
inline bool IsSingleTagAmbiguityRejection(const RobotPoseEstimate& estimate) {
  return estimate.tag_count == 1 &&
         estimate.validity.reason == PoseInvalidReason::kHighAmbiguity;
}
// Marks the lone detection behind a single-tag observation rejected for high
// ambiguity, so the recorder can attribute the rejection truthfully. Detections
// in a multi-tag solve are left untouched.
inline void AnnotateAmbiguityRejection(std::vector<TagDetection3D>& detections,
                                       const RobotPoseEstimate& estimate) {
  if (!IsSingleTagAmbiguityRejection(estimate)) return;
  for (auto& d : detections) {
    if (d.tag_id == estimate.tag_id) d.reject_reason = "ambiguity";
  }
}
inline Eigen::Vector3d FloorResidual(const talos::geometry::SE3<double>& p) {
  const auto r = p.rotation().rotationMatrix();
  return {p.translation().z(), std::atan2(r(2, 1), r(2, 2)),
          std::atan2(-r(2, 0), std::hypot(r(2, 1), r(2, 2)))};
}
inline double FloorSquaredMahalanobis(const talos::geometry::SE3<double>& pose,
                                      const PoseSolverOptions& options) {
  return FloorResidual(pose)
      .cwiseQuotient(Eigen::Vector3d(options.floor_z_std_m,
                                     options.floor_tilt_std_rad,
                                     options.floor_tilt_std_rad))
      .squaredNorm();
}
inline RobotPoseEstimate ComputeRobotFieldPose(
    int tag_id, uint32_t camera_id,
    const talos::geometry::SE3<double>& T_camera_tag,
    const CameraMounting& mounting, const FieldLayout& layout,
    double reprojection_error = 0, double ambiguity = 0,
    const PoseValidationOptions& options = {},
    const talos::geometry::SE3<double>* secondary_T_camera_tag = nullptr,
    double secondary_reprojection_error_px = -1,
    const PoseSolverOptions& solver_options = {}) {
  RobotPoseEstimate out;
  out.tag_id = tag_id;
  out.tag_count = 1;
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
    const double primary_floor =
        FloorSquaredMahalanobis(out.T_field_robot, solver_options);
    const double secondary_floor =
        FloorSquaredMahalanobis(secondary, solver_options);
    // Override pixel ordering only with decisive floor evidence: the winner
    // is inside a 3-sigma prior radius and the alternative outside 6 sigma.
    // In particular, a merely smaller floor residual is not disambiguation.
    constexpr double kCompatibleFloorRadius = 3;
    constexpr double kExcludedFloorRadius = 6;
    const double compatible_sq =
        kCompatibleFloorRadius * kCompatibleFloorRadius;
    const double excluded_sq = kExcludedFloorRadius * kExcludedFloorRadius;
    const bool secondary_resolved =
        secondary_floor <= compatible_sq && primary_floor >= excluded_sq;
    resolved = secondary_resolved || (primary_floor <= compatible_sq &&
                                      secondary_floor >= excluded_sq);
    if (secondary_resolved) {
      out.T_field_robot = secondary;
      out.T_camera_tag = *secondary_T_camera_tag;
      out.reprojection_error_px = secondary_reprojection_error_px;
    }
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
  out.average_distance_m = d;
  out.validity = {true, PoseInvalidReason::kValid, "Valid"};
  return out;
}

namespace detail {
using PoseNormalMatrix = Eigen::Matrix<double, 6, 6>;

// Right boxplus orders its tangent (omega, v): field translation changes by
// R*v, not v. Yaw's derivative is for atan2(R(1,0), R(0,0)), not omega.z().
inline bool SetCovarianceStdDevs(const PoseNormalMatrix& normal,
                                 const PoseSolverOptions& options,
                                 RobotPoseEstimate* estimate) {
  auto reject = [&] {
    estimate->validity = {
        false, PoseInvalidReason::kInvalidCovariance,
        "Joint pose covariance is not finite positive definite"};
    return false;
  };
  if (!normal.allFinite()) return reject();
  const Eigen::LLT<PoseNormalMatrix> factor(normal);
  if (factor.info() != Eigen::Success) return reject();
  const PoseNormalMatrix covariance =
      factor.solve(PoseNormalMatrix::Identity());
  if (!covariance.allFinite() ||
      Eigen::LLT<PoseNormalMatrix>(covariance).info() != Eigen::Success)
    return reject();
  const auto r = estimate->T_field_robot.rotation().rotationMatrix();
  const double yaw_denom = r(0, 0) * r(0, 0) + r(1, 0) * r(1, 0);
  if (yaw_denom <= 1e-12) return reject();
  Eigen::Matrix<double, 3, 6> map = Eigen::Matrix<double, 3, 6>::Zero();
  map.block<2, 3>(0, 3) = r.topRows<2>();
  map(2, 1) = (r(1, 0) * r(0, 2) - r(0, 0) * r(1, 2)) / yaw_denom;
  map(2, 2) = (r(0, 0) * r(1, 1) - r(1, 0) * r(0, 1)) / yaw_denom;
  const Eigen::Matrix3d base_covariance = map * covariance * map.transpose();
  if (!base_covariance.allFinite() ||
      Eigen::LLT<Eigen::Matrix3d>(base_covariance).info() != Eigen::Success)
    return reject();
  Eigen::Matrix3d field_covariance = base_covariance;
  // Extrinsic calibration uncertainty: an unknown mount rotation of
  // mount_angle_std_rad tilts the optical axis and shifts the lever arm by
  // `range * angle`, while an unknown mount translation shifts it directly.
  // Both are added isotropically to field x/y (the tilt direction is
  // unobservable without calibration) and the angle also adds to yaw. Added
  // before the floors so a perfectly calibrated mount (zero std) reproduces
  // the pure corner covariance exactly.
  const double range = estimate->average_distance_m;
  const double mount_xy_var =
      options.mount_translation_std_m * options.mount_translation_std_m +
      (range * options.mount_angle_std_rad) *
          (range * options.mount_angle_std_rad);
  field_covariance(0, 0) += mount_xy_var;
  field_covariance(1, 1) += mount_xy_var;
  field_covariance(2, 2) +=
      options.mount_angle_std_rad * options.mount_angle_std_rad;
  estimate->std_devs = {
      std::max(options.min_xy_std_m, std::sqrt(field_covariance(0, 0))),
      std::max(options.min_xy_std_m, std::sqrt(field_covariance(1, 1))),
      std::max(options.min_yaw_std_rad, std::sqrt(field_covariance(2, 2)))};
  return true;
}
}  // namespace detail

// All observations, including single tags, share this right-increment SE(3)
// LM solve. Pixel rows and floor rows are whitened before Huber IRLS; the
// undamped final information matrix, scaled by the fit's residual variance
// factor below, supplies measurement covariance.
inline RobotPoseEstimate SolveMultiTagRobotPose(
    const std::vector<TagDetection3D>& observations, uint32_t camera_id,
    const CameraMounting& mounting, const FieldLayout& layout,
    const CameraIntrinsics& intrinsics,
    const PoseValidationOptions& options = {},
    const PoseSolverOptions& solver_options = {}) {
  std::vector<const TagDetection3D*> tags;
  tags.reserve(observations.size());
  RobotPoseEstimate out;
  out.camera_id = camera_id;
  auto reject = [&](PoseInvalidReason reason, const char* message) {
    out.validity = {false, reason, message};
    return out;
  };
  const auto& noise = solver_options;
  if (!std::isfinite(noise.pixel_std_px) || noise.pixel_std_px <= 0 ||
      !std::isfinite(noise.floor_z_std_m) || noise.floor_z_std_m <= 0 ||
      !std::isfinite(noise.floor_tilt_std_rad) ||
      noise.floor_tilt_std_rad <= 0 || !std::isfinite(noise.huber_delta_px) ||
      noise.huber_delta_px <= 0 || !std::isfinite(noise.min_xy_std_m) ||
      noise.min_xy_std_m < 0 || !std::isfinite(noise.min_yaw_std_rad) ||
      noise.min_yaw_std_rad < 0 || !std::isfinite(noise.mount_angle_std_rad) ||
      noise.mount_angle_std_rad < 0 ||
      !std::isfinite(noise.mount_translation_std_m) ||
      noise.mount_translation_std_m < 0)
    return reject(PoseInvalidReason::kInvalidCovariance,
                  "Invalid solver noise");
  for (const auto& t : observations) {
    const double d = t.T_camera_tag.translation().norm();
    if (!t.valid || !layout.HasTag(t.tag_id) || !std::isfinite(d) ||
        d < options.min_distance_m || d > options.max_distance_m ||
        t.T_camera_tag.translation().z() <= options.min_z_m)
      continue;
    // Layout ids are unique, so a repeated id is at most one real tag: keep
    // the stronger decode (higher decision margin, then lower per-tag
    // reprojection error) wherever the detector listed it.
    const auto same =
        std::find_if(tags.begin(), tags.end(),
                     [&](const auto* q) { return q->tag_id == t.tag_id; });
    if (same == tags.end()) {
      tags.push_back(&t);
    } else if (t.decision_margin > (*same)->decision_margin ||
               (t.decision_margin == (*same)->decision_margin &&
                t.reprojection_error < (*same)->reprojection_error)) {
      *same = &t;
    }
  }
  double distance_sum = 0;
  for (const auto* t : tags)
    distance_sum += t->T_camera_tag.translation().norm();
  if (tags.empty() || !intrinsics.IsValid())
    return reject(PoseInvalidReason::kDegenerateHomography,
                  "No usable PnP seed");

  std::vector<Eigen::Vector3d> points;
  std::vector<Eigen::Vector2d> pixels;
  points.reserve(4 * tags.size());
  pixels.reserve(4 * tags.size());
  for (const auto* t : tags) {
    const auto field = *layout.GetTagPose(t->tag_id);
    const auto corners = GetTagModelCorners(layout.GetTagSize(t->tag_id));
    for (std::size_t j = 0; j < 4; ++j) {
      points.push_back(field.act(corners[j]));
      pixels.push_back(intrinsics.UndistortPoint(
          {t->corners[2 * j], t->corners[2 * j + 1]}));
    }
  }
  const int pixel_rows = static_cast<int>(2 * points.size());
  const int rows = pixel_rows + 3;
  const auto camera_robot = mounting.T_robot_camera.inverse();
  const auto camera_rotation = camera_robot.rotation().rotationMatrix();
  auto evaluate = [&](const auto& pose, Eigen::VectorXd& residual,
                      Eigen::MatrixXd* jacobian) {
    const auto robot_field = pose.inverse();
    for (std::size_t i = 0; i < points.size(); ++i) {
      const auto row = static_cast<Eigen::Index>(2 * i);
      const auto p_robot = robot_field.act(points[i]);
      const auto p = camera_robot.act(p_robot);
      if (!p.allFinite() || p.z() <= options.min_z_m) return false;
      residual(row) =
          (intrinsics.fx * p.x() / p.z() + intrinsics.cx - pixels[i].x()) /
          noise.pixel_std_px;
      residual(row + 1) =
          (intrinsics.fy * p.y() / p.z() + intrinsics.cy - pixels[i].y()) /
          noise.pixel_std_px;
      if (jacobian) {
        Eigen::Matrix<double, 2, 3> projection;
        projection << intrinsics.fx / p.z(), 0,
            -intrinsics.fx * p.x() / (p.z() * p.z()), 0, intrinsics.fy / p.z(),
            -intrinsics.fy * p.y() / (p.z() * p.z());
        Eigen::Matrix<double, 3, 6> tangent;
        tangent.leftCols<3>() = talos::geometry::SO3<double>::Hat(p_robot);
        tangent.rightCols<3>() = -Eigen::Matrix3d::Identity();
        jacobian->block<2, 6>(row, 0) =
            projection * camera_rotation * tangent / noise.pixel_std_px;
      }
    }
    const auto floor = FloorResidual(pose);
    residual.tail<3>() = floor.cwiseQuotient(
        Eigen::Vector3d(noise.floor_z_std_m, noise.floor_tilt_std_rad,
                        noise.floor_tilt_std_rad));
    if (jacobian) {
      const double sr = std::sin(floor.y()), cr = std::cos(floor.y());
      const double cp = std::cos(floor.z()), tp = std::tan(floor.z());
      if (std::abs(cp) < 1e-6) return false;
      jacobian->bottomRows<3>().setZero();
      jacobian->block<1, 3>(pixel_rows, 3) =
          pose.rotation().rotationMatrix().row(2) / noise.floor_z_std_m;
      jacobian->block<1, 3>(pixel_rows + 1, 0) =
          Eigen::RowVector3d(1, sr * tp, cr * tp) / noise.floor_tilt_std_rad;
      jacobian->block<1, 3>(pixel_rows + 2, 0) =
          Eigen::RowVector3d(0, cr, -sr) / noise.floor_tilt_std_rad;
    }
    return residual.allFinite();
  };
  const double huber_delta = noise.huber_delta_px / noise.pixel_std_px;
  auto cost = [&](const Eigen::VectorXd& r) {
    double value = r.tail<3>().squaredNorm();
    for (int i = 0; i < pixel_rows; ++i) {
      const double a = std::abs(r(i));
      value += a <= huber_delta ? a * a : huber_delta * (2 * a - huber_delta);
    }
    return value;
  };
  Eigen::VectorXd residual(rows), trial_residual(rows), weighted_residual(rows);
  Eigen::MatrixXd jacobian(rows, 6), weighted_jacobian(rows, 6);
  detail::PoseNormalMatrix normal;

  // Runs the joint LM from `seed`, returning the refined estimate. The scratch
  // buffers are shared across seeds; each run reinitializes them via linearize.
  auto solve_from = [&](const talos::geometry::SE3<double>& seed) {
    RobotPoseEstimate est;
    est.camera_id = camera_id;
    est.T_field_robot = seed;
    auto linearize = [&] {
      if (!evaluate(est.T_field_robot, residual, &jacobian)) return false;
      weighted_jacobian = jacobian;
      weighted_residual = residual;
      for (int i = 0; i < pixel_rows; ++i) {
        const double a = std::abs(residual(i));
        if (a > huber_delta) {
          const double sqrt_weight = std::sqrt(huber_delta / a);
          weighted_jacobian.row(i) *= sqrt_weight;
          weighted_residual(i) *= sqrt_weight;
        }
      }
      normal.noalias() = weighted_jacobian.transpose() * weighted_jacobian;
      return normal.allFinite();
    };
    double lambda = 1e-3;
    for (int iteration = 0; iteration < 30; ++iteration) {
      if (!linearize()) {
        est.validity = {false, PoseInvalidReason::kBehindCamera,
                        "Invalid joint projection"};
        return est;
      }
      detail::PoseNormalMatrix damped = normal;
      damped.diagonal().array() += lambda;
      const Eigen::Matrix<double, 6, 1> delta = damped.ldlt().solve(
          -weighted_jacobian.transpose() * weighted_residual);
      if (!delta.allFinite()) {
        est.validity = {false, PoseInvalidReason::kInvalidCovariance,
                        "Invalid joint step"};
        return est;
      }
      const auto trial = est.T_field_robot.boxplus(delta);
      if (evaluate(trial, trial_residual, nullptr) &&
          cost(trial_residual) < cost(residual)) {
        est.T_field_robot = trial;
        lambda *= 0.25;
        if (delta.norm() < 1e-8) break;
      } else {
        if (delta.norm() < 1e-8) break;
        lambda *= 10;
      }
    }
    if (!linearize()) {
      est.validity = {false, PoseInvalidReason::kBehindCamera,
                      "Invalid final projection"};
      return est;
    }
    est.tag_id = -1;
    est.tag_count = static_cast<int>(tags.size());
    est.average_distance_m = distance_sum / static_cast<double>(tags.size());
    est.reprojection_error_px =
        noise.pixel_std_px * std::sqrt(residual.head(pixel_rows).squaredNorm() /
                                       static_cast<double>(points.size()));
    est.ambiguity = 0;
    // The information matrix assumes pixel_std_px corner noise. A worse fit
    // scales the covariance by the a-posteriori variance factor
    // s^2 = chi^2 / (rows - 6) of the Huber-weighted whitened residual (pixel
    // and floor rows), clamped to [1, max_reprojection_error_px^2 /
    // (2 pixel_std_px^2)]: never below nominal, and at most the per-coordinate
    // variance ratio of a fit at the reprojection reject threshold.
    const int dof = rows - 6;
    const double threshold_ratio =
        options.max_reprojection_error_px / noise.pixel_std_px;
    const double variance_scale =
        dof > 0
            ? std::max(1.0, std::min(weighted_residual.squaredNorm() /
                                         static_cast<double>(dof),
                                     0.5 * threshold_ratio * threshold_ratio))
            : 1.0;
    const detail::PoseNormalMatrix information = normal / variance_scale;
    if (!detail::SetCovarianceStdDevs(information, noise, &est)) return est;
    if (est.reprojection_error_px > options.max_reprojection_error_px) {
      est.validity = {false, PoseInvalidReason::kHighReprojectionError,
                      "Joint PnP residual too high"};
      return est;
    }
    est.validity = {true, PoseInvalidReason::kValid, "Valid"};
    return est;
  };

  // A single tag keeps the ComputeRobotFieldPose path so the ambiguity gate
  // (and the floor-disambiguation override) decides the observation; the LM
  // then only refines and computes covariance.
  if (tags.size() == 1) {
    const auto& t = *tags.front();
    auto seed = ComputeRobotFieldPose(
        t.tag_id, camera_id, t.T_camera_tag, mounting, layout,
        t.reprojection_error, t.ambiguity, options, &t.secondary_T_camera_tag,
        t.secondary_reprojection_error_px, noise);
    if (!seed.validity.valid) return seed;
    auto result = solve_from(seed.T_field_robot);
    result.tag_id = t.tag_id;
    result.ambiguity = t.ambiguity;
    return result;
  }

  // Multi-tag: a single tag's primary is unreliable near the planar-ambiguity
  // boundary, so evaluate every pose candidate (primary and secondary, when
  // present) of every tag by the joint unweighted reprojection residual over
  // all tags' corners (no LM), and seed the joint solve from the best.
  auto joint_rms = [&](const talos::geometry::SE3<double>& T_field_robot) {
    const auto robot_field = T_field_robot.inverse();
    double sum_sq = 0.0;
    for (size_t i = 0; i < points.size(); ++i) {
      const auto p = camera_robot.act(robot_field.act(points[i]));
      if (!p.allFinite() || p.z() <= options.min_z_m)
        return std::numeric_limits<double>::infinity();
      const double u = intrinsics.fx * p.x() / p.z() + intrinsics.cx;
      const double v = intrinsics.fy * p.y() / p.z() + intrinsics.cy;
      const double du = u - pixels[i].x(), dv = v - pixels[i].y();
      sum_sq += du * du + dv * dv;
    }
    return std::sqrt(sum_sq / static_cast<double>(points.size()));
  };
  struct SeedCandidate {
    talos::geometry::SE3<double> T_field_robot;
    double residual;
  };
  std::vector<SeedCandidate> seeds;
  seeds.reserve(2 * tags.size());
  for (const auto* t : tags) {
    const auto field = *layout.GetTagPose(t->tag_id);
    auto add = [&](const talos::geometry::SE3<double>& camera_tag) {
      const auto robot =
          field * camera_tag.inverse() * mounting.T_robot_camera.inverse();
      const double residual = joint_rms(robot);
      if (std::isfinite(residual)) seeds.push_back({robot, residual});
    };
    add(t->T_camera_tag);
    if (t->secondary_reprojection_error_px >= 0.0 &&
        std::isfinite(t->secondary_reprojection_error_px))
      add(t->secondary_T_camera_tag);
  }
  if (seeds.empty())
    return reject(PoseInvalidReason::kDegenerateHomography,
                  "No usable PnP seed");
  std::sort(seeds.begin(), seeds.end(), [](const auto& a, const auto& b) {
    return a.residual < b.residual;
  });

  RobotPoseEstimate best_estimate = solve_from(seeds.front().T_field_robot);
  double best_score = best_estimate.validity.valid
                          ? best_estimate.reprojection_error_px
                          : std::numeric_limits<double>::infinity();
  // The runner-up seed guards against converging into the planar-flip local
  // minimum; keep whichever converges to the lower final reprojection error.
  for (size_t k = 1; k < seeds.size() && k < 2; ++k) {
    auto alt = solve_from(seeds[k].T_field_robot);
    const double alt_score = alt.validity.valid
                                 ? alt.reprojection_error_px
                                 : std::numeric_limits<double>::infinity();
    if (alt_score < best_score) {
      best_estimate = std::move(alt);
      best_score = alt_score;
    }
  }
  return best_estimate;
}

}  // namespace talos::vision
