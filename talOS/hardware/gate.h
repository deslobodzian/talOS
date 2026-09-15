#pragma once

// Shared gating predicates for hardware-owning subsystem nodes.
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

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <span>

#include "talOS/events/time.h"
#include "talOS/hardware/messages.h"

namespace talos::hardware {

// Whether `next` should replace the last accepted state. Every sample is
// dropped when it carries another config id (zero accepts any, for standalone
// and simulation runs); beyond that, the first sample is accepted and later
// ones must not be stale or duplicate samples from the same boot.
inline bool AcceptState(bool have_prev, const State& prev, uint64_t config_id,
                        const State& next) {
  if (config_id != 0 && next.config_id != config_id) return false;
  if (have_prev && next.boot_id == prev.boot_id &&
      next.sample_time_us <= prev.sample_time_us)
    return false;
  return true;
}

// Whether the gateway is ready to be commanded: configured and enabled, with
// no hardware fault latched.
inline bool GatewayReady(const State& state) {
  return (state.flags & (kConfigured | kEnabled)) == (kConfigured | kEnabled) &&
         !(state.flags & kHardwareFault);
}

// Whether a latched target may drive: enabled, issued no later than now, and
// both the target and the last accepted state younger than the timeout. A
// consumer treats a stale target as absent rather than as zero, so the check
// is strict (`<`, not `<=`): a target exactly one timeout old is already gone.
inline bool TargetFresh(event::MonotonicTime now, event::MonotonicTime issued,
                        bool enabled, event::MonotonicTime state_received,
                        std::chrono::microseconds timeout) {
  return enabled && issued <= now && now - issued < timeout &&
         now - state_received < timeout;
}

// The sample for one device id, or nullptr when the state carries none.
inline const MotorSample* FindMotor(const State& state, uint16_t id) {
  for (std::size_t i = 0; i < state.motor_count; ++i) {
    if (state.motors[i].id == id) return &state.motors[i];
  }
  return nullptr;
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
