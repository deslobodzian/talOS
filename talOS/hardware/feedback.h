#pragma once

// The gateway's rules for an invalid feedback sample and for total feedback
// loss, shared with testing::SimHardware so a node test sees what the robot
// reports for the same condition.

// Relative, as in the rest of the gateway: the RoboRIO's GradleRIO build
// compiles the gateway with talOS/, not the repository root, on its path.
#include <cstddef>

#include "messages.h"

namespace talos::hardware {

// An invalid motor or CAN-sensor sample: `valid` false and zero in every
// measured field (position, velocity, applied voltage, stator and supply
// current), so no consumer can read a stale measurement as current. Ages,
// temperature and fault flags are left as they are.
inline void MarkInvalid(MotorSample& sample) {
  sample.valid = false;
  sample.position_rot = 0;
  sample.velocity_rps = 0;
  sample.voltage = 0;
  sample.stator_current_a = 0;
  sample.supply_current_a = 0;
}
inline void MarkInvalid(SensorSample& sample) {
  sample.valid = false;
  sample.position_rot = 0;
  sample.velocity_rps = 0;
}

// Whether any motor or CAN sensor in `state` has a valid sample, or none is
// configured. False is total feedback loss: the gateway raises
// kHardwareFault, starts a new epoch (when it was healthy or a command was in
// force) and holds every output neutral until a sample is valid again.
inline bool AnyFeedbackValid(const State& state) {
  if (state.motor_count == 0 && state.sensor_count == 0) return true;
  for (std::size_t i = 0; i < state.motor_count; ++i) {
    if (state.motors[i].valid) return true;
  }
  for (std::size_t i = 0; i < state.sensor_count; ++i) {
    if (state.sensors[i].valid) return true;
  }
  return false;
}

}  // namespace talos::hardware
