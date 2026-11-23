#pragma once

// Shared gating predicates for hardware-owning subsystem nodes, and the
// gateway's feedback age bound (FeedbackFresh) and per-motor drive rules
// (MotorMayDrive).
//
// Every subsystem node that turns hardware state into a command repeats the
// same preamble: accept the sample only if it is newer than the last one from
// the same boot and matches this node's config, then drive only when the
// gateway is ready and the latched target is fresh. Those copies drifted
// before (see rule 11 in ARCHITECTURE.md), so they live here as pure
// functions — no loop, no transport, no config file — where a loop-free test
// (gate_test.cc) pins their exact semantics for every node at once.
//
// What stays in the node: device presence (which samples a subsystem needs is
// node policy), the command law itself, and the state it publishes.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>

// Relative, as in the rest of the gateway: the RoboRIO's GradleRIO build
// compiles gateway.cc with talOS/, not the repository root, on its path.
#include "feedback.h"
#include "messages.h"

namespace talos::hardware {

// Whether `next` should replace the last accepted state. Every sample is
// dropped when it carries another config id (zero accepts any, for standalone
// and simulation runs) or comes from a recently retired gateway boot
// (BootHistory::retired); beyond that, the first sample is accepted and later
// ones must not be stale or duplicate samples from the same boot.
inline bool AcceptState(bool have_prev, const State& prev, uint64_t config_id,
                        const State& next, bool boot_retired) {
  if (config_id != 0 && next.config_id != config_id) return false;
  if (boot_retired) return false;
  if (have_prev && next.boot_id == prev.boot_id &&
      next.sample_time_us <= prev.sample_time_us)
    return false;
  return true;
}

// Whether a motor's or CAN sensor's position or velocity sample `age_us` old
// still counts for actuation: younger than `command_timeout_us`, the bound a
// command, its State and a target are held to. A backend's unknown age
// (UINT32_MAX) never does. The gateway applies it to every such sample after
// Backend::Read.
inline bool FeedbackFresh(uint32_t age_us, uint64_t command_timeout_us) {
  return age_us != std::numeric_limits<uint32_t>::max() &&
         age_us < command_timeout_us;
}

// The gateway's per-motor rules, shared with testing::SimHardware. `motors`
// is the configuration in manifest order, the order of `state.motors`.
//
// Whether motor `i` follows a master whose sample is invalid in `state`: the
// gateway publishes such a follower's sample invalid too (MaskUnavailable).
inline bool MasterSampleInvalid(std::span<const MotorConfig> motors,
                                const State& state, std::size_t i) {
  const uint16_t master_id = motors[i].master_id;
  if (master_id == 0) return false;
  for (std::size_t j = 0; j < motors.size(); ++j) {
    if (motors[j].id == master_id) return !state.motors[j].valid;
  }
  return false;
}

// Whether motor `i`'s own sample, and its remote CANcoder's, is valid.
inline bool MotorFeedbackValid(std::span<const MotorConfig> motors,
                               const State& state, std::size_t i) {
  if (!state.motors[i].valid) return false;
  if (motors[i].feedback != Feedback::kRemoteCANcoder) return true;
  for (std::size_t j = 0; j < state.sensor_count; ++j) {
    if (state.sensors[j].id == motors[i].feedback_sensor_id) {
      return state.sensors[j].valid;
    }
  }
  return false;
}

// Whether motor `i` may drive in `state`: MotorFeedbackValid for it and, for a
// follower, for its master (config refuses follower chains). A motor that may
// not is neutral from that tick and drives again only on a command observing
// a later State.
inline bool MotorMayDrive(std::span<const MotorConfig> motors,
                          const State& state, std::size_t i) {
  if (!MotorFeedbackValid(motors, state, i)) return false;
  const uint16_t master_id = motors[i].master_id;
  if (master_id == 0) return true;
  for (std::size_t j = 0; j < motors.size(); ++j) {
    if (motors[j].id == master_id) {
      return MotorFeedbackValid(motors, state, j);
    }
  }
  return false;
}

// Propagates invalid master samples to their followers in `state` (after every
// master and sensor has been masked; config refuses follower chains, so a
// master is never a follower), then visits every motor that may not drive in
// `state` (MotorMayDrive) with the earliest observation time that may drive it
// again (`state.sample_time_us + 1`).
//
// `on_follower_masked(std::size_t i)` runs for each follower invalidated here,
// after MarkInvalid zeroes its sample.
// `on_motor_unable(std::size_t i, uint64_t drives_from_us)` runs for each
// motor that may not drive in `state`.
template <typename OnFollowerMasked, typename OnMotorUnable>
inline void MaskFollowersAndDriveCutoffs(std::span<const MotorConfig> motors,
                                         State& state,
                                         OnFollowerMasked&& on_follower_masked,
                                         OnMotorUnable&& on_motor_unable) {
  const std::size_t count =
      motors.size() < state.motor_count ? motors.size() : state.motor_count;
  for (std::size_t i = 0; i < count; ++i) {
    if (MasterSampleInvalid(motors, state, i)) {
      MarkInvalid(state.motors[i]);
      on_follower_masked(i);
    }
  }
  const uint64_t drives_from_us = state.sample_time_us + 1;
  for (std::size_t i = 0; i < count; ++i) {
    if (!MotorMayDrive(motors, state, i)) {
      on_motor_unable(i, drives_from_us);
    }
  }
}

// Whether a torque-current request may drive motor `m` (REQ-SAFE-43): `m`
// has foc and a positive max_torque_current_a, |demand| is at most that
// limit, and feedforward_v is 0. The whole rule for the mode, feedforward
// included: Gateway::Accept neutralizes a request that fails it, and
// PhoenixBackend::Apply sends NeutralOut for one that reaches it anyway.
// A NaN demand, feedforward or limit fails its comparison; an infinite demand
// exceeds the limit, which Validate holds finite. It does not govern a
// follower: the gateway replaces a follower's request with its master's mode
// whatever this says, and Apply sends a follower Follower without asking it.
inline bool TorqueCurrentAllowed(const MotorConfig& m, const MotorRequest& r) {
  return m.foc && m.max_torque_current_a > 0 &&
         std::abs(r.demand) <= m.max_torque_current_a && r.feedforward_v == 0;
}

// Whether the gateway is ready to be commanded: configured and enabled, with
// no hardware fault latched.
inline bool GatewayReady(const State& state) {
  return (state.flags & (kConfigured | kEnabled)) == (kConfigured | kEnabled) &&
         !(state.flags & kHardwareFault);
}

// The neutral fallback: a command that echoes the state's identity words and
// addresses exactly `motor_ids`, leaving every mode at neutral with zero
// demand. A subsystem that cannot drive says stop this way rather than by
// going silent, so the gateway never holds a stale demand.
inline Command NeutralCommand(const State& state,
                              std::span<const uint16_t> motor_ids) {
  Command command;
  command.config_id = state.config_id;
  command.boot_id = state.boot_id;
  command.epoch = state.epoch;
  command.observed_time_us = state.sample_time_us;
  command.count = static_cast<uint16_t>(
      motor_ids.size() > kMaxMotors ? kMaxMotors : motor_ids.size());
  for (std::size_t i = 0; i < command.count; ++i)
    command.motors[i].id = motor_ids[i];
  return command;
}

}  // namespace talos::hardware
