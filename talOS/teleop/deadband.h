#pragma once

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

// Whether an axis reading is neutral for arming a talos::OperatorArming:
// finite and inside its deadband, the band Deadband maps to zero. So an
// axis's arming-neutral band is its output dead zone, and an axis resting
// inside the band that armed it commands nothing on the armed evaluations
// that follow (the evaluation that arms never sends). Config load refuses a
// teleop deadband below config::kTeleopMinDeadbandFrac, wide enough for a
// real stick resting a few counts off zero.
inline bool ArmingNeutral(double value, double deadband_frac) {
  return std::isfinite(value) && std::abs(value) <= deadband_frac;
}

}  // namespace talos::teleop
