#pragma once

#include <cstdint>
#include <limits>
#include <optional>

#include "driver_station/driver_station.h"

// Safety helpers shared between the RoboRIO main loop (Robot.cpp) and its
// deterministic unit tests (RobotSafety_test.cpp):
//   1. Shutdown neutralization order (REQ-SAFE-01, REQ-SAFE-25).
//   2. Pre-tick and post-tick LoopWatchdog deadline calculation (REQ-SAFE-25).
//   3. Effective Driver Station mode flags across closed-socket ticks and
//      immediate DS publication on reopen (REQ-SAFE-10).
namespace talos::rio {

// Saturating addition on uint64_t so a near-UINT64_MAX clock or large margin
// never wraps to 0 (which disarms LoopWatchdog) or into the past.
constexpr uint64_t SaturatingAddUs(uint64_t a, uint64_t b) {
  return std::numeric_limits<uint64_t>::max() - a < b
             ? std::numeric_limits<uint64_t>::max()
             : a + b;
}

// Watchdog deadline for a command expiring at `expiry_us`: two loop periods
// past expiry (one for the tick that sees it and one for that tick's own work
// before its beat), or 0 (disarmed) when no command is in force.
constexpr uint64_t WatchdogDeadlineUs(uint64_t expiry_us, uint64_t period_us) {
  if (expiry_us == 0) return 0;
  return SaturatingAddUs(expiry_us, SaturatingAddUs(period_us, period_us));
}

// Pre-arm deadline before an enabled Endpoint::Tick() when no command is
// currently active (`command_expiry_us == 0`): bounds a stall inside that
// tick's first Apply() / ApplyOutputs() / SendFrame() using the maximum expiry
// a newly accepted command could receive (`now_us + command_timeout_us`) plus
// the two-period margin. Returns 0 when no pre-arm is needed (disabled, or a
// command is already active so the watchdog already holds a tighter deadline).
constexpr uint64_t WatchdogPreArmDeadlineUs(uint64_t now_us, bool enabled,
                                            uint64_t command_expiry_us,
                                            uint32_t command_timeout_us,
                                            uint64_t period_us) {
  if (!enabled || command_expiry_us != 0) return 0;
  const uint64_t max_expiry_us =
      SaturatingAddUs(now_us, uint64_t{command_timeout_us});
  const uint64_t deadline_us = WatchdogDeadlineUs(max_expiry_us, period_us);
  // 0 means disarmed in LoopWatchdog; never return 0 when arming.
  return deadline_us == 0 ? 1 : deadline_us;
}

// Driver Station mode flags aligned with the gateway's effective enable state:
// a closed-socket tick runs `gateway_->Tick(now_us, false)` (disabling the
// gateway and bumping epoch), so clearing kEnabled ensures ModeTracker
// advances its mode generation across short socket outages (< command_timeout)
// and forces OperatorArming to re-arm on reopen.
constexpr uint32_t EffectiveDsModeFlags(uint32_t raw_flags, bool socket_open) {
  return socket_open ? raw_flags
                     : (raw_flags & ~talos::driver_station::kEnabled);
}

// Whether an open-socket tick should emit a Driver Station packet: on the
// first open tick (or the first after a closed-socket disable, where
// `last_ds_mode` was cleared), on any mode change, or once kSamplePeriodUs has
// elapsed since the last send.
constexpr bool ShouldSendDsSample(uint64_t now_us, uint64_t last_ds_sample_us,
                                  uint32_t mode_key,
                                  std::optional<uint32_t> last_ds_mode) {
  return !last_ds_mode || mode_key != *last_ds_mode ||
         now_us - last_ds_sample_us >= talos::driver_station::kSamplePeriodUs;
}

// Runs the RoboRIO shutdown sequence after the main loop exits:
// 1. If no background configure currently owns the devices, neutralizes every
//    output immediately BEFORE stopping the watchdog or Driver Station reader
//    (whose teardown can block 100+ ms on NetComm).
// 2. Stops the watchdog and Driver Station reader.
// 3. Polls any in-flight configure to completion disabled, then neutralizes
//    what it built.
template <typename IsConfiguring, typename Neutralize, typename StopWatchdog,
          typename StopDriverStation, typename PollDisabled>
void ShutdownRioLoop(const IsConfiguring& is_configuring,
                     const Neutralize& neutralize,
                     const StopWatchdog& stop_watchdog,
                     const StopDriverStation& stop_driver_station,
                     const PollDisabled& poll_disabled) {
  if (!is_configuring()) neutralize();
  stop_watchdog();
  stop_driver_station();
  while (is_configuring()) poll_disabled();
  neutralize();
}

}  // namespace talos::rio
