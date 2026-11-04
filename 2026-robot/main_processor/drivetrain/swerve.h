#pragma once

// Swerve kinematics on wpimath's SwerveDriveKinematics<4>: no hardware and no
// framework. Units: meters and radians/s for the chassis, rotations and
// rotations/s (mechanism units, `_rot`/`_rps`) for the modules. The functions
// under "unit boundary" are the only place the drivetrain turns one into the
// other.

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <wpi/array.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
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

// --- unit boundary ----------------------------------------------------------

// A shaft or gyro angle in rotations as a frame angle.
inline units::radian_t ToRadians(double rot) { return units::turn_t{rot}; }
inline double ToRotations(units::radian_t angle) {
  return units::turn_t{angle}.value();
}
// Arc length the wheel rolls through: radians times radius.
inline units::meter_t WheelTravel(double rot, double wheel_radius_m) {
  return units::meter_t{ToRadians(rot).value() * wheel_radius_m};
}
inline units::meters_per_second_t WheelSpeed(double rps,
                                             double wheel_radius_m) {
  return units::meters_per_second_t{
      units::radians_per_second_t{units::turns_per_second_t{rps}}.value() *
      wheel_radius_m};
}
inline double WheelRps(units::meters_per_second_t speed,
                       double wheel_radius_m) {
  return units::turns_per_second_t{
      units::radians_per_second_t{speed.value() / wheel_radius_m}}
      .value();
}

// ----------------------------------------------------------------------------

class SwerveKinematics {
 public:
  explicit SwerveKinematics(const SwerveGeometry& geometry)
      : geometry_{geometry}, kinematics_{Translations(geometry)} {}

  // Inverse kinematics: the module states that produce `speeds`. Each module
  // steers the shorter way from its current azimuth (`current[i].steer_rot`),
  // reversing the wheel rather than turning past a quarter rotation; a
  // stopped wheel holds its azimuth; every wheel is scaled by the same factor
  // so none exceeds its own drive's maximum. nullopt for non-finite input.
  //
  // The scaling is not SwerveDriveKinematics::DesaturateWheelSpeeds: that
  // takes one attainable speed for every module, and here each module has
  // its own wheel radius and drive max_velocity_rps.
  std::optional<std::array<ModuleState, kModuleCount>> ToModuleStates(
      const ChassisSpeeds& speeds,
      const std::array<ModuleState, kModuleCount>& current) {
    if (!std::isfinite(speeds.vx_mps) || !std::isfinite(speeds.vy_mps) ||
        !std::isfinite(speeds.omega_radps))
      return std::nullopt;
    wpi::array<frc::Rotation2d, kModuleCount> headings(wpi::empty_array);
    for (std::size_t i = 0; i < kModuleCount; ++i)
      headings[i] = frc::Rotation2d{ToRadians(current[i].steer_rot)};
    // A module with no speed comes back on its remembered heading: the one it
    // has now.
    kinematics_.ResetHeadings(headings);
    const auto states = kinematics_.ToSwerveModuleStates(
        frc::ChassisSpeeds{units::meters_per_second_t{speeds.vx_mps},
                           units::meters_per_second_t{speeds.vy_mps},
                           units::radians_per_second_t{speeds.omega_radps}});
    std::array<ModuleState, kModuleCount> out{};
    double scale = 1;
    for (std::size_t i = 0; i < kModuleCount; ++i) {
      const SwerveModule& module = geometry_.modules[i];
      frc::SwerveModuleState state = states[i];
      state.Optimize(headings[i]);
      out[i].drive_rps = WheelRps(state.speed, module.wheel_radius_m);
      if (!std::isfinite(out[i].drive_rps)) return std::nullopt;
      out[i].steer_rot = current[i].steer_rot +
                         ToRotations((state.angle - headings[i]).Radians());
      if (std::abs(out[i].drive_rps) > module.max_drive_rps)
        scale =
            std::min(scale, module.max_drive_rps / std::abs(out[i].drive_rps));
    }
    for (ModuleState& m : out) m.drive_rps *= scale;
    return out;
  }

  // Forward kinematics: the chassis motion the measured module states imply
  // (least squares over the modules).
  ChassisSpeeds ToChassisSpeeds(
      const std::array<ModuleState, kModuleCount>& measured) const {
    wpi::array<frc::SwerveModuleState, kModuleCount> states(wpi::empty_array);
    for (std::size_t i = 0; i < kModuleCount; ++i) {
      states[i] = {WheelSpeed(measured[i].drive_rps,
                              geometry_.modules[i].wheel_radius_m),
                   frc::Rotation2d{ToRadians(measured[i].steer_rot)}};
    }
    const frc::ChassisSpeeds speeds = kinematics_.ToChassisSpeeds(states);
    return {speeds.vx.value(), speeds.vy.value(), speeds.omega.value()};
  }

 private:
  static wpi::array<frc::Translation2d, kModuleCount> Translations(
      const SwerveGeometry& geometry) {
    wpi::array<frc::Translation2d, kModuleCount> out(wpi::empty_array);
    for (std::size_t i = 0; i < kModuleCount; ++i) {
      out[i] = frc::Translation2d{units::meter_t{geometry.modules[i].x_m},
                                  units::meter_t{geometry.modules[i].y_m}};
    }
    return out;
  }

  SwerveGeometry geometry_;
  frc::SwerveDriveKinematics<kModuleCount> kinematics_;
};

}  // namespace drivetrain
