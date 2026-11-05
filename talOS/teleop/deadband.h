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

}  // namespace talos::teleop
