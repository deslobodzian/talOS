#pragma once

#include <algorithm>
#include <cmath>

namespace talos::teleop {

// Zero inside the deadband, then rescaled to ramp from 0 at the threshold
// to full scale at 1 instead of stepping. Continuous at the threshold edge.
inline double Deadband(double value, double deadband_frac) {
  const double magnitude = std::abs(value);
  if (magnitude <= deadband_frac || deadband_frac >= 1.0) return 0.0;
  return std::copysign((magnitude - deadband_frac) / (1.0 - deadband_frac),
                       value);
}

// The least band, as a fraction of full stick, an axis counts as neutral in
// for arming a talos::OperatorArming: a real stick rests a few counts off
// zero, so an axis mapped without a deadband would otherwise never arm.
inline constexpr double kArmingNeutralFloorFrac = 0.05;

// Whether an axis reading is neutral for arming: finite and inside the wider
// of its deadband and kArmingNeutralFloorFrac. The output mapping (Deadband)
// still uses the deadband alone.
inline bool ArmingNeutral(double value, double deadband_frac) {
  return std::isfinite(value) &&
         std::abs(value) <= std::max(deadband_frac, kArmingNeutralFloorFrac);
}

}  // namespace talos::teleop
