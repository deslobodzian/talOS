#pragma once

// Robot-relative swerve kinematics. Chassis units are meters and radians;
// module units are mechanism rotations and rotations per second.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <optional>
#include <string>

#include "talOS/geometry/se2.h"
#include "talOS/geometry/so2.h"

namespace drivetrain {

inline constexpr std::size_t kModuleCount = 4;

// One module, as [subsystems.drivetrain.geometry] describes it (geometry.h).
struct SwerveModule {
  std::string name;   // "front_left"
  std::string drive;  // motor device names in this subsystem
  std::string steer;
  double x_m{};  // from the robot centre: x forward, y left
  double y_m{};
  double wheel_radius_m{};
  double max_drive_rps{};  // the drive motor's max_velocity_rps
  bool operator==(const SwerveModule&) const = default;
};

struct SwerveGeometry {
  std::array<SwerveModule, kModuleCount> modules{};
  bool operator==(const SwerveGeometry&) const = default;
};

// Robot-relative chassis motion: x forward, y left, yaw CCW.
struct ChassisSpeeds {
  double vx_mps{};
  double vy_mps{};
  double omega_radps{};
};

// A module's wheel speed and azimuth.
struct ModuleState {
  double drive_rps{};
  double steer_rot{};
};

inline constexpr double kTwoPi = 2.0 * std::numbers::pi;
inline double ToRadians(double rot) { return rot * kTwoPi; }
inline double ToRotations(double rad) { return rad / kTwoPi; }
inline double WheelTravel(double rot, double wheel_radius_m) {
  return ToRadians(rot) * wheel_radius_m;
}
inline double WheelSpeed(double rps, double wheel_radius_m) {
  return ToRadians(rps) * wheel_radius_m;
}
inline double WheelRps(double speed_mps, double wheel_radius_m) {
  return speed_mps / (kTwoPi * wheel_radius_m);
}

class SwerveKinematics {
 public:
  explicit SwerveKinematics(const SwerveGeometry& geometry)
      : geometry_(geometry) {
    Eigen::Matrix3d normal = Eigen::Matrix3d::Zero();
    for (std::size_t i = 0; i < kModuleCount; ++i) {
      const auto& module = geometry_.modules[i];
      const Eigen::Vector3d row_x(1.0, 0.0, -module.y_m);
      const Eigen::Vector3d row_y(0.0, 1.0, module.x_m);
      normal.noalias() += row_x * row_x.transpose();
      normal.noalias() += row_y * row_y.transpose();
    }
    normal_ldlt_.compute(normal);
  }

  // Shortest-path steering, cosine compensation, and one common speed scale
  // respecting each module's individual drive limit. Stopped wheels hold angle.
  std::optional<std::array<ModuleState, kModuleCount>> ToModuleStates(
      const ChassisSpeeds& speeds,
      const std::array<ModuleState, kModuleCount>& current, double dt_s = 0.0) {
    if (!std::isfinite(speeds.vx_mps) || !std::isfinite(speeds.vy_mps) ||
        !std::isfinite(speeds.omega_radps))
      return std::nullopt;
    double vx = speeds.vx_mps, vy = speeds.vy_mps;
    const double omega = speeds.omega_radps;
    if (dt_s > 0) {
      const auto discrete =
          talos::geometry::SE2<double>::Discretize(vx, vy, omega, dt_s);
      vx = discrete(1);
      vy = discrete(2);
    }
    std::array<ModuleState, kModuleCount> out{};
    double scale = 1.0;
    for (std::size_t i = 0; i < kModuleCount; ++i) {
      const auto& module = geometry_.modules[i];
      if (!std::isfinite(current[i].steer_rot)) return std::nullopt;
      const double wheel_vx = vx - omega * module.y_m;
      const double wheel_vy = vy + omega * module.x_m;
      double speed = std::hypot(wheel_vx, wheel_vy);
      out[i].steer_rot = current[i].steer_rot;
      if (speed <= 1e-6) continue;
      using Rotation = talos::geometry::SO2<double>;
      double delta = (Rotation(ToRadians(current[i].steer_rot)).inverse() *
                      Rotation(std::atan2(wheel_vy, wheel_vx)))
                         .angle();
      if (std::abs(delta) > std::numbers::pi / 2.0) {
        speed = -speed;
        delta = std::atan2(std::sin(delta + std::numbers::pi),
                           std::cos(delta + std::numbers::pi));
      }
      speed *= std::max(0.0, std::cos(delta));
      out[i].drive_rps = WheelRps(speed, module.wheel_radius_m);
      out[i].steer_rot += ToRotations(delta);
      if (!std::isfinite(out[i].drive_rps) || !std::isfinite(out[i].steer_rot))
        return std::nullopt;
      if (std::abs(out[i].drive_rps) > module.max_drive_rps)
        scale =
            std::min(scale, module.max_drive_rps / std::abs(out[i].drive_rps));
    }
    for (auto& state : out) state.drive_rps *= scale;
    return out;
  }

  // Full least-squares solve, including non-centered module layouts.
  ChassisSpeeds ToChassisSpeeds(
      const std::array<ModuleState, kModuleCount>& measured) const {
    Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
    for (std::size_t i = 0; i < kModuleCount; ++i) {
      const auto& module = geometry_.modules[i];
      const double speed =
          WheelSpeed(measured[i].drive_rps, module.wheel_radius_m);
      const double angle = ToRadians(measured[i].steer_rot);
      const double vx = speed * std::cos(angle);
      const double vy = speed * std::sin(angle);
      const Eigen::Vector3d row_x(1.0, 0.0, -module.y_m);
      const Eigen::Vector3d row_y(0.0, 1.0, module.x_m);
      rhs.noalias() += row_x * vx + row_y * vy;
    }
    const Eigen::Vector3d result = normal_ldlt_.solve(rhs);
    return {result.x(), result.y(), result.z()};
  }

 private:
  SwerveGeometry geometry_;
  Eigen::LDLT<Eigen::Matrix3d> normal_ldlt_{};
};

}  // namespace drivetrain
