// The RoboRIO loop's Driver Station generations and packet cadence
// (DriverStationSampler.h, as Robot.cpp ticks it): a new mode generation on
// each transition ARCHITECTURE.md "Targets" lists, stamped through
// driver_station::ModeTracker; none when the companion link drops and returns
// in the same mode; a packet on the tick of a mode change and otherwise every
// driver_station::kSamplePeriodUs.
#include "DriverStationSampler.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "driver_station/driver_station.h"

namespace {

namespace ds = talos::driver_station;
using talos::rio::DriverStationSampler;

constexpr uint64_t kPeriodUs = 5'000;
constexpr uint64_t kStartUs = 1'000'000;
constexpr uint64_t kBootId = 0x1234'5678'9abc'def1;

ds::DriverStationData View(uint32_t flags) {
  ds::DriverStationData view;
  view.fms.flags = flags;
  return view;
}

// One loop tick as Robot.cpp runs it: observe the view's flags, then sample
// it on the socket's state. The packet sent, if any.
struct Tick {
  bool sent{false};
  ds::DriverStationData packet;
};
Tick RunTick(DriverStationSampler& sampler, uint32_t flags, uint64_t now_us,
             bool socket_open = true) {
  const ds::DriverStationData view = View(flags);
  sampler.Observe(view.fms.flags, now_us);
  Tick tick;
  tick.sent = sampler.Sample(view, now_us, kBootId, socket_open, tick.packet);
  return tick;
}

// Each transition ARCHITECTURE.md "Targets" names starts a new generation on
// the tick that observes it, and that tick sends it: disabling, re-enabling,
// switching mode, e-stopping, and re-attaching after the Driver Station view
// went away (the reader's view of a refresh command_timeout_us old has no
// flags).
// REQ-SAFE-10
TEST(DriverStationSampler, EachListedTransitionStartsANewGeneration) {
  struct Step {
    const char* transition;
    uint32_t flags;
  };
  const std::vector<Step> steps{
      {"enable in teleop", ds::kDsAttached | ds::kEnabled | ds::kTeleop},
      {"disable", ds::kDsAttached | ds::kTeleop},
      {"re-enable", ds::kDsAttached | ds::kEnabled | ds::kTeleop},
      {"switch to autonomous, enabled",
       ds::kDsAttached | ds::kEnabled | ds::kAutonomous},
      {"e-stop", ds::kDsAttached | ds::kEStop | ds::kAutonomous},
      {"lose the Driver Station", 0},
      {"re-attach", ds::kDsAttached | ds::kTeleop},
  };
  DriverStationSampler sampler;
  uint64_t now_us = kStartUs;
  const Tick first = RunTick(sampler, ds::kDsAttached | ds::kTeleop, now_us);
  ASSERT_TRUE(first.sent);
  uint64_t generation = first.packet.mode_since_us;
  ASSERT_NE(generation, 0u);
  for (const Step& step : steps) {
    SCOPED_TRACE(step.transition);
    // One tick later: well inside kSamplePeriodUs of the last packet.
    now_us += kPeriodUs;
    const Tick tick = RunTick(sampler, step.flags, now_us);
    ASSERT_TRUE(tick.sent) << "no packet on the tick of the mode change";
    EXPECT_GT(tick.packet.mode_since_us, generation);
    EXPECT_EQ(tick.packet.mode_since_us, now_us);
    EXPECT_EQ(tick.packet.sample_time_us, now_us);
    EXPECT_EQ(tick.packet.boot_id, kBootId);
    generation = tick.packet.mode_since_us;
  }
}

// A companion link that drops and returns while the Driver Station stays in
// one mode: the loop's socket is closed for SocketRetry's 100 ms (nothing is
// sent), the mode is still observed every tick, and the first packet after
// the socket reopens carries the generation the last one did. A closed socket
// is not a mode change (bdf56ace reverted one that made it one).
// REQ-SAFE-10
TEST(DriverStationSampler, ASameModeCompanionLinkDropKeepsTheGeneration) {
  constexpr uint32_t kTeleopEnabled =
      ds::kDsAttached | ds::kEnabled | ds::kTeleop;
  DriverStationSampler sampler;
  uint64_t now_us = kStartUs;
  const Tick before = RunTick(sampler, kTeleopEnabled, now_us);
  ASSERT_TRUE(before.sent);
  for (int i = 0; i < 20; ++i) {
    now_us += kPeriodUs;
    EXPECT_FALSE(RunTick(sampler, kTeleopEnabled, now_us, false).sent)
        << "a packet went out on a closed socket at " << now_us;
  }
  now_us += kPeriodUs;
  const Tick after = RunTick(sampler, kTeleopEnabled, now_us);
  ASSERT_TRUE(after.sent) << "no packet once the socket reopened";
  EXPECT_EQ(after.packet.mode_since_us, before.packet.mode_since_us)
      << "the closed socket started a new generation";
}

// In one mode a packet goes out every kSamplePeriodUs (every fourth 5 ms
// tick), never between; a mode change goes out on its own tick, and the
// cadence restarts from it.
// REQ-SAFE-10, REQ-RT-13
TEST(DriverStationSampler, SendsOnAModeChangeAndEverySamplePeriod) {
  static_assert(ds::kSamplePeriodUs == 4 * kPeriodUs);
  constexpr uint32_t kDisabled = ds::kDsAttached | ds::kTeleop;
  constexpr uint32_t kEnabled = kDisabled | ds::kEnabled;
  DriverStationSampler sampler;
  std::vector<uint64_t> sent;
  const uint64_t change_us = kStartUs + 9 * kPeriodUs;
  for (uint64_t now_us = kStartUs; now_us < kStartUs + 20 * kPeriodUs;
       now_us += kPeriodUs) {
    const uint32_t flags = now_us < change_us ? kDisabled : kEnabled;
    if (RunTick(sampler, flags, now_us).sent) sent.push_back(now_us);
  }
  const std::vector<uint64_t> expected{
      kStartUs,  kStartUs + 4 * kPeriodUs,  kStartUs + 8 * kPeriodUs,
      change_us, change_us + 4 * kPeriodUs, change_us + 8 * kPeriodUs,
  };
  EXPECT_EQ(sent, expected);
}

// On the first tick with a small monotonic clock (< kSamplePeriodUs) and zero
// flags, a packet still goes out immediately; a subsequent backward clock step
// must not underflow now_us - last_sample_us_ into a spurious send.
// REQ-SAFE-10, REQ-RT-13
TEST(DriverStationSampler,
     InitialZeroModeNearZeroClockSendsAndBackwardClockDoesNotUnderflow) {
  DriverStationSampler sampler;
  const Tick first = RunTick(sampler, /*flags=*/0, /*now_us=*/5'000);
  EXPECT_TRUE(first.sent)
      << "first tick with flags=0 and now_us < kSamplePeriodUs did not send";

  const Tick backward = RunTick(sampler, /*flags=*/0, /*now_us=*/1'000);
  EXPECT_FALSE(backward.sent)
      << "backward clock step underflowed into a spurious send";
}

// If encoding or sending a mode-change packet fails, RevertSample() rolls back
// the cadence/mode state so the next open-socket tick retries immediately
// without waiting kSamplePeriodUs, while preserving the exact mode_since_us
// generation recorded on the transition tick.
// REQ-SAFE-10, REQ-RT-13, REQ-RT-22
TEST(DriverStationSampler,
     RevertSampleRetriesModeChangeImmediatelyWhileKeepingGeneration) {
  constexpr uint32_t kTeleopEnabled =
      ds::kDsAttached | ds::kEnabled | ds::kTeleop;
  constexpr uint32_t kAutoEnabled =
      ds::kDsAttached | ds::kEnabled | ds::kAutonomous;
  DriverStationSampler sampler;

  const Tick initial = RunTick(sampler, kTeleopEnabled, kStartUs);
  ASSERT_TRUE(initial.sent);

  // Enabled mode switch (teleop -> auto) 5 ms later; simulate UDP send failure.
  const uint64_t switch_us = kStartUs + kPeriodUs;
  const Tick failed = RunTick(sampler, kAutoEnabled, switch_us);
  ASSERT_TRUE(failed.sent);
  ASSERT_EQ(failed.packet.mode_since_us, switch_us);
  sampler.RevertSample();
  // Idempotent on repeated calls without an intervening Sample().
  sampler.RevertSample();

  // Next tick (still 10 ms < kSamplePeriodUs from initial): must retry at once
  // with the original transition's mode_since_us generation.
  const uint64_t retry_us = switch_us + kPeriodUs;
  const Tick retried = RunTick(sampler, kAutoEnabled, retry_us);
  ASSERT_TRUE(retried.sent)
      << "mode change was delayed up to kSamplePeriodUs after failed send";
  EXPECT_EQ(retried.packet.mode_since_us, switch_us);
  EXPECT_EQ(retried.packet.sample_time_us, retry_us);
  EXPECT_EQ(retried.packet.boot_id, kBootId);

  // Once sent, normal kSamplePeriodUs cadence resumes from the successful send.
  EXPECT_FALSE(RunTick(sampler, kAutoEnabled, retry_us + kPeriodUs).sent);
  EXPECT_TRUE(
      RunTick(sampler, kAutoEnabled, retry_us + ds::kSamplePeriodUs).sent);
}

}  // namespace
