#pragma once

// Shared gating predicates for hardware-owning subsystem nodes, and the
// gateway's feedback age bound (FeedbackFresh).
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

#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>

// Relative, as in the rest of the gateway: the RoboRIO's GradleRIO build
// compiles gateway.cc with talOS/, not the repository root, on its path.
#include "messages.h"

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

// Whether a motor's or CAN sensor's position or velocity sample `age_us` old
// still counts for actuation: younger than `command_timeout_us`, the bound a
// command, its State and a target are held to. A backend's unknown age
// (UINT32_MAX) never does. The gateway applies it to every such sample after
// Backend::Read.
inline bool FeedbackFresh(uint32_t age_us, uint64_t command_timeout_us) {
  return age_us != std::numeric_limits<uint32_t>::max() &&
         age_us < command_timeout_us;
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
