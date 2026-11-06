#pragma once

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

#include "talOS/geometry/se3.h"

namespace talos::vision {

// Pinhole camera intrinsic parameters with 5-parameter Brown-Conrady distortion
// model (k1, k2, p1, p2, k3).
struct CameraIntrinsics {
  double fx{0.0};
  double fy{0.0};
  double cx{0.0};
  double cy{0.0};

  double k1{0.0};
  double k2{0.0};
  double p1{0.0};
  double p2{0.0};
  double k3{0.0};

  uint32_t image_width{0};
  uint32_t image_height{0};

  bool IsValid() const { return fx > 0.0 && fy > 0.0 && cx > 0.0 && cy > 0.0; }

  Eigen::Matrix3d K() const {
    Eigen::Matrix3d k = Eigen::Matrix3d::Identity();
    k(0, 0) = fx;
    k(1, 1) = fy;
    k(0, 2) = cx;
    k(1, 2) = cy;
    return k;
  }

  Eigen::Matrix3d KInverse() const {
    Eigen::Matrix3d k = Eigen::Matrix3d::Identity();
    k(0, 0) = 1.0 / fx;
    k(1, 1) = 1.0 / fy;
    k(0, 2) = -cx / fx;
    k(1, 2) = -cy / fy;
    return k;
  }

  // Projects a 3D point in the camera frame (Z > 0) to distorted pixel
  // coordinates.
  bool ProjectPoint(const Eigen::Vector3d& p_cam,
                    Eigen::Vector2d* uv_out) const {
    if (p_cam.z() <= 1e-6) {
      return false;
    }

    const double inv_z = 1.0 / p_cam.z();
    const double x = p_cam.x() * inv_z;
    const double y = p_cam.y() * inv_z;

    const double r2 = x * x + y * y;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;
    const double radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;

    const double dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
    const double dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;

    const double x_dist = x * radial + dx;
    const double y_dist = y * radial + dy;

    uv_out->x() = fx * x_dist + cx;
    uv_out->y() = fy * y_dist + cy;
    return true;
  }

  // Inverts Brown-Conrady distortion into ideal normalized ray coordinates.
  Eigen::Vector2d UndistortNormalizedPoint(const Eigen::Vector2d& uv,
                                           int max_iterations = 8) const {
    const double x_dist = (uv.x() - cx) / fx;
    const double y_dist = (uv.y() - cy) / fy;

    double x = x_dist;
    double y = y_dist;

    // Iterative Newton-Raphson inversion of distortion
    for (int iter = 0; iter < max_iterations; ++iter) {
      const double r2 = x * x + y * y;
      const double r4 = r2 * r2;
      const double r6 = r4 * r2;
      const double radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;

      const double dx = 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x);
      const double dy = p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y;

      x = (x_dist - dx) / radial;
      y = (y_dist - dy) / radial;
    }
    return Eigen::Vector2d(x, y);
  }

  // Ideal pinhole pixel coordinates, suitable for homography/PnP solvers.
  Eigen::Vector2d UndistortPoint(const Eigen::Vector2d& uv,
                                 int iterations = 8) const {
    const auto p = UndistortNormalizedPoint(uv, iterations);
    return {fx * p.x() + cx, fy * p.y() + cy};
  }
};

// Maps OpenCV optical coordinates (+X right, +Y down, +Z forward) to robot NWU.
struct CameraMounting {
  static talos::geometry::SO3<double> OpticalToRobotRotation() {
    Eigen::Matrix3d r;
    r << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    return talos::geometry::SO3<double>(r);
  }
  talos::geometry::SE3<double> T_robot_camera{OpticalToRobotRotation(),
                                              Eigen::Vector3d::Zero()};

  static CameraMounting Identity() { return FromExtrinsics(0, 0, 0, 0, 0, 0); }
  static CameraMounting FromExtrinsics(double x, double y, double z,
                                       double roll, double pitch, double yaw) {
    const Eigen::Matrix3d r =
        (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
            .toRotationMatrix();
    return {{talos::geometry::SO3<double>(r) * OpticalToRobotRotation(),
             Eigen::Vector3d(x, y, z)}};
  }
  static CameraMounting FromOpticalTransform(
      const talos::geometry::SE3<double>& transform) {
    return CameraMounting{transform};
  }
};

// Represents field layout: 3D poses of AprilTags in field coordinates
// (T_field_tag) and their physical dimensions.
class FieldLayout {
 public:
  explicit FieldLayout(double default_tag_size_m = 0.1651)
      : default_tag_size_m_{default_tag_size_m} {}

  double default_tag_size_m() const { return default_tag_size_m_; }
  void set_default_tag_size_m(double size_m) { default_tag_size_m_ = size_m; }

  void SetTagPose(int tag_id, const talos::geometry::SE3<double>& T_field_tag) {
    tag_poses_[tag_id] = T_field_tag;
  }

  std::optional<talos::geometry::SE3<double>> GetTagPose(int tag_id) const {
    auto it = tag_poses_.find(tag_id);
    if (it != tag_poses_.end()) {
      return it->second;
    }
    return std::nullopt;
  }

  bool HasTag(int tag_id) const {
    return tag_poses_.find(tag_id) != tag_poses_.end();
  }

  double GetTagSize(int tag_id) const {
    auto it = custom_tag_sizes_.find(tag_id);
    if (it != custom_tag_sizes_.end()) {
      return it->second;
    }
    return default_tag_size_m_;
  }

  void SetTagSize(int tag_id, double size_m) {
    custom_tag_sizes_[tag_id] = size_m;
  }

  size_t size() const { return tag_poses_.size(); }
  void Clear() {
    tag_poses_.clear();
    custom_tag_sizes_.clear();
  }

 private:
  double default_tag_size_m_{0.1651};
  std::unordered_map<int, talos::geometry::SE3<double>> tag_poses_;
  std::unordered_map<int, double> custom_tag_sizes_;
};

// 3D tag model corners in the WPILib/layout tag frame: +X out of the tag
// face (normal), +Z up, +Y completing the right-handed frame (the tag lives
// in the x=0 plane, centered at the origin). Index order matches the
// detector's corner order — p[0] is the tag-model-origin corner the decoder
// starts from — so pairing observed[i] with model[i] is geometrically
// correct (verified against apriltag-imgs tag36h11-0.png: the real detector
// starts an upright tag at the bottom-left corner):
// p[0]: (0, -s/2, -s/2)  bottom-left as seen facing the tag face
// p[1]: (0, s/2, -s/2)   bottom-right
// p[2]: (0, s/2, s/2)    top-right
// p[3]: (0, -s/2, s/2)   top-left
//
// Shared by the field renderer and pose estimator.
inline std::array<Eigen::Vector3d, 4> GetTagModelCorners(double tag_size_m) {
  const double half = tag_size_m * 0.5;
  return {
      Eigen::Vector3d(0.0, -half, -half),
      Eigen::Vector3d(0.0, half, -half),
      Eigen::Vector3d(0.0, half, half),
      Eigen::Vector3d(0.0, -half, half),
  };
}

// Converts a pose from the AprilTag solver's tag frame (+X right, +Y down,
// +Z toward the scene) into the WPILib/layout tag frame (+X out of the tag
// face, +Z up): T_layout = T_solver * LayoutToSolver, where LayoutToSolver
// re-expresses layout-frame triples in the solver frame. Without this,
// field composition mixes frames and every single-tag robot pose is wrong.
// (Rows assigned explicitly; verified: for a frontal upright tag the solver
// returns identity and this yields the true tag-to-camera rotation, and the
// layout bottom-left corner maps onto solver model corner 0.)
inline talos::geometry::SE3<double> SolverTagFrameToLayoutFrame(
    const talos::geometry::SE3<double>& T_camera_tag_solver) {
  Eigen::Matrix3d layout_to_solver;
  layout_to_solver.row(0) = Eigen::Vector3d(0.0, 1.0, 0.0);
  layout_to_solver.row(1) = Eigen::Vector3d(0.0, 0.0, -1.0);
  layout_to_solver.row(2) = Eigen::Vector3d(-1.0, 0.0, 0.0);
  return T_camera_tag_solver *
         talos::geometry::SE3<double>(
             talos::geometry::SO3<double>(layout_to_solver),
             Eigen::Vector3d::Zero());
}

inline talos::geometry::SE3<double> LayoutFrameToSolverTagFrame(
    const talos::geometry::SE3<double>& pose) {
  return pose *
         SolverTagFrameToLayoutFrame(talos::geometry::SE3<double>::Identity())
             .inverse();
}

}  // namespace talos::vision
