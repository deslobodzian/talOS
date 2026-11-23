// Regression tests for the RoboRIO loop's safety invariants (Robot.cpp):
//   1. Shutdown neutralizes outputs before stopping LoopWatchdog and
//      DriverStationReader (while still avoiding a race with an in-flight
//      background configure) (REQ-SAFE-01, REQ-SAFE-25).
//   2. LoopWatchdog is pre-armed before an enabled Endpoint::Tick() when no
//      command is active, so a stall inside the first Apply()/ApplyOutputs()
//      is neutralized (REQ-SAFE-25).
// The loop's Driver Station generations are DriverStationSampler_test.cpp's.
#include "RobotSafety.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include "LoopWatchdog.h"

namespace {

using talos::rio::LoopWatchdog;
using talos::rio::ShutdownRioLoop;
using talos::rio::WatchdogDeadlineUs;
using talos::rio::WatchdogPreArmDeadlineUs;

class RecordingWatchdogOutputs final : public talos::rio::WatchdogOutputs {
 public:
  bool Neutralize() override {
    ++neutralize_calls;
    live = false;
    return true;
  }
  void Report(std::string_view text) override { reports.emplace_back(text); }

  bool live{false};
  int neutralize_calls{0};
  std::vector<std::string> reports;
};

constexpr uint64_t kPeriodUs = 5'000;
constexpr uint32_t kTimeoutUs = 100'000;

// Defect 1: when EndCompetition() exits the loop while outputs are energized
// and no configure is running, outputs must be neutralized BEFORE
// watchdog_->Stop() and driver_station.Stop() (which can block 100+ ms on
// NetComm teardown), and neutralized again after the configure-poll step.
// REQ-SAFE-01
// REQ-SAFE-25
TEST(RobotSafety, ShutdownNeutralizesOutputsBeforeStoppingWatchdogAndDs) {
  bool outputs_energized = true;
  bool outputs_energized_at_watchdog_stop = true;
  bool outputs_energized_at_ds_stop = true;
  std::vector<std::string> events;

  ShutdownRioLoop(
      /*is_configuring=*/[] { return false; },
      /*neutralize=*/
      [&] {
        outputs_energized = false;
        events.emplace_back("neutral");
      },
      /*stop_watchdog=*/
      [&] {
        outputs_energized_at_watchdog_stop = outputs_energized;
        events.emplace_back("stop_watchdog");
      },
      /*stop_driver_station=*/
      [&] {
        outputs_energized_at_ds_stop = outputs_energized;
        events.emplace_back("stop_ds");
      },
      /*poll_disabled=*/
      [&] { events.emplace_back("poll_disabled"); });

  EXPECT_FALSE(outputs_energized_at_watchdog_stop)
      << "outputs remained energized when LoopWatchdog stopped";
  EXPECT_FALSE(outputs_energized_at_ds_stop)
      << "outputs remained energized during DriverStationReader::Stop()";
  EXPECT_FALSE(outputs_energized);
  ASSERT_EQ(events.size(), 4u);
  EXPECT_EQ(events[0], "neutral");
  EXPECT_EQ(events[1], "stop_watchdog");
  EXPECT_EQ(events[2], "stop_ds");
  EXPECT_EQ(events[3], "neutral");
}

// Companion safety invariant for Defect 1: when a background configure owns
// the backend's devices at shutdown, the pre-stop Neutral() must be skipped so
// it does not race device construction; once polling completes the configure
// disabled, Neutral() runs on what was built.
// REQ-SAFE-01
// REQ-SAFE-25
TEST(RobotSafety, ShutdownSkipsPreStopNeutralWhileConfiguring) {
  int remaining_configure_polls = 2;
  std::vector<std::string> events;

  ShutdownRioLoop(
      /*is_configuring=*/[&] { return remaining_configure_polls > 0; },
      /*neutralize=*/[&] { events.emplace_back("neutral"); },
      /*stop_watchdog=*/[&] { events.emplace_back("stop_watchdog"); },
      /*stop_driver_station=*/[&] { events.emplace_back("stop_ds"); },
      /*poll_disabled=*/
      [&] {
        --remaining_configure_polls;
        events.emplace_back("poll_disabled");
      });

  EXPECT_EQ(events, (std::vector<std::string>{
                        "stop_watchdog",
                        "stop_ds",
                        "poll_disabled",
                        "poll_disabled",
                        "neutral",
                    }));
}

// Defect 2: after any disabled/neutral tick (watchdog disarmed, deadline 0),
// an enabled tick must pre-arm LoopWatchdog before Endpoint::Tick() so a stall
// inside the first Apply()/ApplyOutputs() (after motor 0 is energized and
// before the post-tick Beat) is caught and neutralized.
// REQ-SAFE-25
TEST(RobotSafety, PreArmWatchdogCatchesStallInsideFirstApplyAfterNeutralTick) {
  RecordingWatchdogOutputs outputs;
  LoopWatchdog watchdog{outputs};

  // Previous tick ended neutral/disabled -> watchdog disarmed.
  const uint64_t t0 = 1'000'000;
  watchdog.Beat(t0, WatchdogDeadlineUs(0, kPeriodUs));

  // Next tick is enabled with no active command yet.
  const uint64_t t1 = t0 + kPeriodUs;
  const uint64_t pre_arm = WatchdogPreArmDeadlineUs(
      t1, /*enabled=*/true, /*command_expiry_us=*/0, kTimeoutUs, kPeriodUs);
  ASSERT_NE(pre_arm, 0u) << "enabled tick with no active command must pre-arm";
  EXPECT_EQ(pre_arm, t1 + kTimeoutUs + 2 * kPeriodUs);
  watchdog.Beat(t1, pre_arm);

  // Simulate Endpoint::Tick() energizing outputs and stalling before post-tick
  // Beat().
  outputs.live = true;
  watchdog.Poll(pre_arm - 1);
  EXPECT_TRUE(outputs.live) << "watchdog fired before pre-arm deadline";
  EXPECT_EQ(outputs.neutralize_calls, 0);

  watchdog.Poll(pre_arm);
  EXPECT_FALSE(outputs.live)
      << "watchdog failed to neutralize outputs energized during stalled "
         "first Apply()";
  EXPECT_EQ(outputs.neutralize_calls, 1);
  EXPECT_EQ(watchdog.stall_count(), 1u);
}

// Post-tick refinement and boundary conditions for Defect 2:
// - When a command IS accepted, post-tick Beat refines to expiry + 2*period.
// - When no command is accepted, post-tick Beat(0) disarms cleanly.
// - When a command is already active, pre-arm returns 0 so the tighter active
//   deadline is not relaxed.
// - Disabled ticks never pre-arm.
// - Near UINT64_MAX, deadlines saturate instead of wrapping to 0 (disarm) or
//   into the past.
// REQ-SAFE-25
TEST(RobotSafety, WatchdogPreArmRefinementDisarmAndOverflowSafety) {
  RecordingWatchdogOutputs outputs;
  LoopWatchdog watchdog{outputs};

  const uint64_t t = 2'000'000;
  // Disabled: no pre-arm.
  EXPECT_EQ(
      WatchdogPreArmDeadlineUs(t, /*enabled=*/false, 0, kTimeoutUs, kPeriodUs),
      0u);
  // Already active command: no pre-arm (keeps existing tighter deadline).
  EXPECT_EQ(WatchdogPreArmDeadlineUs(t, /*enabled=*/true,
                                     /*command_expiry_us=*/t + 40'000,
                                     kTimeoutUs, kPeriodUs),
            0u);

  // Enabled tick with no command accepted: pre-arm then disarm at post-tick.
  const uint64_t pre_arm =
      WatchdogPreArmDeadlineUs(t, /*enabled=*/true, 0, kTimeoutUs, kPeriodUs);
  ASSERT_GT(pre_arm, t);
  watchdog.Beat(t, pre_arm);
  watchdog.Beat(t, WatchdogDeadlineUs(/*expiry_us=*/0, kPeriodUs));
  watchdog.Poll(t + 10'000'000);
  EXPECT_EQ(outputs.neutralize_calls, 0)
      << "disarming post-tick beat after no accepted command must disarm";

  // Enabled tick accepting a command observed 10 ms earlier: post-tick beat
  // refines deadline earlier than pre-arm.
  const uint64_t accepted_expiry = (t - 10'000) + kTimeoutUs;
  const uint64_t refined = WatchdogDeadlineUs(accepted_expiry, kPeriodUs);
  EXPECT_LT(refined, pre_arm);
  watchdog.Beat(t, pre_arm);
  watchdog.Beat(t, refined);
  watchdog.Poll(refined);
  EXPECT_EQ(outputs.neutralize_calls, 1);

  // Overflow safety near UINT64_MAX: must saturate, never wrap to 0 or < now.
  constexpr uint64_t kMax = std::numeric_limits<uint64_t>::max();
  EXPECT_EQ(WatchdogPreArmDeadlineUs(kMax - 1'000, /*enabled=*/true, 0,
                                     kTimeoutUs, kPeriodUs),
            kMax);
  EXPECT_EQ(WatchdogDeadlineUs(kMax - 1'000, kPeriodUs), kMax);
  // Degenerate zero-clock, zero-timeout, zero-period still never returns 0
  // (which would disarm LoopWatchdog).
  EXPECT_EQ(WatchdogPreArmDeadlineUs(0, /*enabled=*/true, 0, 0, 0), 1u);
}

}  // namespace
