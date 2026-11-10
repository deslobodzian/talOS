#pragma once

// What a loop thread asked the OS for and what it got: the loop tier and the
// outcome of ConfigureRealtime (process.h). Header-only and free of any OS
// call, so a reader of the live registry (talOS/introspection), which carries
// it in each node's row, does not link the code that applies it.

#include <cstdint>
#include <string>

namespace talos::process {

// Which loop wins when loops compete for a CPU, lowest first. A process
// decides its own tier from what it already knows: the hardware bridge is
// kBridge; a node on the actuation path is kControl: its own subsystem config
// owns an actuator (hardware::Devices::owns_actuators: a motor, digital
// output or PWM output) or it writes a target lane (`/<sub>/target/<source>`:
// driver, teleop, auto); every other node (telemetry, cameras, the pose
// estimator) is kObserver.
enum class LoopTier : std::uint8_t {
  kObserver,
  kControl,
  kBridge,
};

// The steps ConfigureRealtime takes, as bits of RealtimeOutcome::refused.
// kStepPolicy is the scheduling policy (SCHED_FIFO on Linux; on macOS the
// time-constraint policy, or QOS_CLASS_USER_INTERACTIVE for an observer);
// the rest keep the loop's memory resident.
inline constexpr std::uint8_t kStepPolicy = 1u << 0;
inline constexpr std::uint8_t kStepMlockall = 1u << 1;      // Linux only
inline constexpr std::uint8_t kStepTextPrefault = 1u << 2;  // macOS only
inline constexpr std::uint8_t kStepRingLock = 1u << 3;
inline constexpr std::uint8_t kStepStackLock = 1u << 4;
inline constexpr std::uint8_t kMemorySteps =
    kStepMlockall | kStepTextPrefault | kStepRingLock | kStepStackLock;
inline constexpr std::uint8_t kAllSteps = kStepPolicy | kMemorySteps;

// How much of the realtime scheduling a loop asked for it got, best first.
enum class RealtimeGrant : std::uint8_t {
  kFull,     // every step took effect
  kPartial,  // the policy took effect; a memory step was refused
  // Never asked (TALOS_REALTIME=0, or --sim without TALOS_REALTIME=1): the
  // loop runs at the default timeshare priority.
  kNotRequested,
  // The OS refused the policy: the loop runs at the default timeshare
  // priority, beside every other process.
  kRefused,
};

struct RealtimeOutcome {
  LoopTier tier = LoopTier::kObserver;
  // Whether the steps were attempted at all.
  bool requested = false;
  // The kStep* bits the OS refused; 0 when every step took effect or none
  // was requested.
  std::uint8_t refused = 0;

  constexpr RealtimeGrant grant() const noexcept {
    if (!requested) return RealtimeGrant::kNotRequested;
    if ((refused & kStepPolicy) != 0) return RealtimeGrant::kRefused;
    if (refused != 0) return RealtimeGrant::kPartial;
    return RealtimeGrant::kFull;
  }

  // The bridge and control loops are the actuation path: a gap in one is a
  // gap in a hardware request (ARCHITECTURE.md "The tick").
  constexpr bool actuation_path() const noexcept {
    return tier != LoopTier::kObserver;
  }

  friend constexpr bool operator==(const RealtimeOutcome&,
                                   const RealtimeOutcome&) = default;
};

constexpr const char* TierName(LoopTier tier) noexcept {
  switch (tier) {
    case LoopTier::kObserver:
      return "observer";
    case LoopTier::kControl:
      return "control";
    case LoopTier::kBridge:
      return "bridge";
  }
  return "unknown";
}

constexpr const char* GrantName(RealtimeGrant grant) noexcept {
  switch (grant) {
    case RealtimeGrant::kFull:
      return "full";
    case RealtimeGrant::kPartial:
      return "partial";
    case RealtimeGrant::kNotRequested:
      return "not_requested";
    case RealtimeGrant::kRefused:
      return "refused";
  }
  return "unknown";
}

// The refused steps, comma separated, in kStep* order ("" for none).
inline std::string RefusedStepNames(std::uint8_t refused) {
  static constexpr struct {
    std::uint8_t step;
    const char* name;
  } kNames[] = {{kStepPolicy, "policy"},
                {kStepMlockall, "mlockall"},
                {kStepTextPrefault, "text prefault"},
                {kStepRingLock, "ring mlock"},
                {kStepStackLock, "stack mlock"}};
  std::string out;
  for (const auto& entry : kNames) {
    if ((refused & entry.step) == 0) continue;
    if (!out.empty()) out += ", ";
    out += entry.name;
  }
  return out;
}

}  // namespace talos::process
