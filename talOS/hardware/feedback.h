#pragma once

// The gateway's rules for an invalid feedback sample and for total feedback
// loss. The gateway, testing::SimHardware and the simulated backends call
// these, so a node test and a simulated gateway see what the robot reports
// for the same condition.

// Relative, as in the rest of the gateway: the RoboRIO's GradleRIO build
// compiles the gateway with talOS/, not the repository root, on its path.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>

#include "messages.h"

namespace talos::hardware {

// PhoenixBackend::Read reports a motor's temperature as 0 once its signal is
// this old (Fresh(temperature, 500_ms), controller_processor/rio/src/main/
// cpp/PhoenixBackend.cpp:975). testing::SimHardware and PhysicsSimBackend
// age a device that stopped answering by it.
inline constexpr uint64_t kTemperatureMaxAgeUs = 500'000;
// PhoenixBackend::Read clears a motor's limit and limiter bits
// (kMotorNotFaultFlags) and keeps its fault bits once its FaultField is this
// old (Fresh's 50 ms default, PhoenixBackend.cpp:53 and :953).
inline constexpr uint64_t kFaultFieldMaxAgeUs = 50'000;

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

// A CANcoder's absolute position as the RoboRIO reports it: Phoenix 6
// GetAbsolutePosition with AbsoluteSensorDiscontinuityPoint = 0.5 (set for
// every CANcoder by PhoenixBackend), so in [-0.5, 0.5) rotations. A remote
// CANcoder motor's own position is continuous and is not wrapped.
inline double CANcoderAbsoluteRot(double rot) {
  double wrapped = rot - std::floor(rot + 0.5);
  if (wrapped >= 0.5) wrapped -= 1.0;  // rounding at the discontinuity
  return wrapped;
}

// Whether any motor or CAN sensor in `state` has a valid sample, or none is
// configured. The gateway and testing::SimHardware ask it of the State they
// publish, after the age bound and their masks; false is total feedback
// loss: kHardwareFault, a new epoch (when it was healthy or a command was in
// force) and every output neutral until a published sample is valid again.
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

// Whether a restore of `ids` (Backend::BeginRestoreAfterReset) reapplies
// motor `i`, as PhoenixBackend widens it: a listed motor, a motor fed by a
// listed remote CANcoder, and a follower of either. PhoenixBackend::Read
// reports each such motor invalid from that call until PollRestore completes
// its restore (a failed device stays invalid until a later one does);
// SimBackend and PhysicsSimBackend do the same. A CANcoder's own sample stays
// valid while it restores. The gateway masks the whole chain, the CANcoder
// included, either way.
inline bool RestoredWith(std::span<const MotorConfig> motors, std::size_t i,
                         std::span<const uint16_t> ids) {
  const auto listed = [ids](uint16_t id) {
    return std::find(ids.begin(), ids.end(), id) != ids.end();
  };
  const auto reapplied = [&listed](const MotorConfig& m) {
    return listed(m.id) || (m.feedback == Feedback::kRemoteCANcoder &&
                            listed(m.feedback_sensor_id));
  };
  if (reapplied(motors[i])) return true;
  // Config refuses follower chains, so a master is never a follower.
  return motors[i].master_id != 0 &&
         std::any_of(motors.begin(), motors.end(), [&](const MotorConfig& m) {
           return m.id == motors[i].master_id && reapplied(m);
         });
}

}  // namespace talos::hardware
