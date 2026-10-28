#pragma once

// Swerve kinematics: plain math on module geometry, no hardware and no
// framework. Units: metres, radians/s for the chassis, rotations and
// rotations/s (mechanism units) for the modules.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <optional>
#include <string>

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

// Inverse kinematics: the module states that produce `speeds`. Each module
// steers the shorter way from its current azimuth (`current[i].steer_rot`),
// reversing the wheel rather than turning past a quarter rotation, and every
// wheel is scaled by the same factor so none exceeds its drive's maximum.
// nullopt for non-finite input.
inline std::optional<std::array<ModuleState, kModuleCount>> ToModuleStates(
    const SwerveGeometry& geometry, const ChassisSpeeds& speeds,
    const std::array<ModuleState, kModuleCount>& current) {
  constexpr double kTau = 2 * std::numbers::pi;
  if (!std::isfinite(speeds.vx_mps) || !std::isfinite(speeds.vy_mps) ||
      !std::isfinite(speeds.omega_radps))
    return std::nullopt;
  std::array<ModuleState, kModuleCount> out{};
  double scale = 1;
  for (std::size_t i = 0; i < kModuleCount; ++i) {
    const SwerveModule& module = geometry.modules[i];
    const double x = speeds.vx_mps - speeds.omega_radps * module.y_m;
    const double y = speeds.vy_mps + speeds.omega_radps * module.x_m;
    out[i].drive_rps = std::hypot(x, y) / (kTau * module.wheel_radius_m);
    if (!std::isfinite(out[i].drive_rps)) return std::nullopt;
    out[i].steer_rot = std::atan2(y, x) / kTau;
    if (out[i].drive_rps > module.max_drive_rps)
      scale = std::min(scale, module.max_drive_rps / out[i].drive_rps);
  }
  for (std::size_t i = 0; i < kModuleCount; ++i) {
    double delta = std::remainder(out[i].steer_rot - current[i].steer_rot, 1.0);
    if (std::abs(delta) > 0.25) {
      delta -= std::copysign(0.5, delta);
      out[i].drive_rps = -out[i].drive_rps;
    }
    if (std::abs(out[i].drive_rps) < 1e-6) delta = 0;  // a stopped wheel holds
    out[i].drive_rps *= scale;
    out[i].steer_rot = current[i].steer_rot + delta;
  }
  return out;
}

// Forward kinematics: the chassis motion the measured module states imply,
// averaged over the modules.
inline ChassisSpeeds ToChassisSpeeds(
    const SwerveGeometry& geometry,
    const std::array<ModuleState, kModuleCount>& measured) {
  constexpr double kTau = 2 * std::numbers::pi;
  ChassisSpeeds sum{};
  for (std::size_t i = 0; i < kModuleCount; ++i) {
    const SwerveModule& module = geometry.modules[i];
    const double wheel_mps =
        measured[i].drive_rps * kTau * module.wheel_radius_m;
    const double angle_rad = measured[i].steer_rot * kTau;
    const double vx = wheel_mps * std::cos(angle_rad);
    const double vy = wheel_mps * std::sin(angle_rad);
    sum.vx_mps += vx;
    sum.vy_mps += vy;
    const double r2 = module.x_m * module.x_m + module.y_m * module.y_m;
    if (r2 > 1e-6) sum.omega_radps += (-module.y_m * vx + module.x_m * vy) / r2;
  }
  constexpr double n = kModuleCount;
  return {sum.vx_mps / n, sum.vy_mps / n, sum.omega_radps / n};
}

}  // namespace drivetrain
