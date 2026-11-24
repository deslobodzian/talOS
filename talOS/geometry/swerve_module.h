#pragma once

// One swerve module's setpoint: shortest-path steering with the drive
// reversed past a quarter turn, and cosine compensation. Used by the 2026
// drivetrain and the bench swerve module node.

#include <algorithm>
#include <cmath>
#include <numbers>

#include "talOS/geometry/so2.h"

namespace talos::geometry {

struct SwerveModuleSetpoint {
  double speed{};            // same unit as the asked speed
  double steer_delta_rad{};  // add to the measured angle; in [-pi/2, pi/2]
  bool reversed{};           // true when the opposite heading was taken
};

// asked_heading_rad: the heading the wheel should travel along.
// measured_rad: the measured steering angle (may be unwrapped, any magnitude).
//
// The steering change is the shortest rotation from the measured angle to the
// asked heading; past a quarter turn the module takes the opposite heading
// and negates the speed instead. The speed is then scaled by the cosine of
// the steering change (never below zero), so a wheel still far from its
// heading does not push the robot sideways. A non-finite input returns
// {0, 0, false}. Allocates nothing.
inline SwerveModuleSetpoint OptimizeSwerveModule(double speed,
                                                 double asked_heading_rad,
                                                 double measured_rad) {
  if (!std::isfinite(speed) || !std::isfinite(asked_heading_rad) ||
      !std::isfinite(measured_rad))
    return {};
  using Rotation = SO2<double>;
  double delta =
      (Rotation(measured_rad).inverse() * Rotation(asked_heading_rad)).angle();
  bool reversed = false;
  if (std::abs(delta) > std::numbers::pi / 2.0) {
    speed = -speed;
    delta = delta > 0.0 ? delta - std::numbers::pi : delta + std::numbers::pi;
    reversed = true;
  }
  return {speed * std::max(0.0, std::cos(delta)), delta, reversed};
}

}  // namespace talos::geometry
