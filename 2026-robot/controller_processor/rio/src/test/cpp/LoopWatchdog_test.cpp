// The RoboRIO loop watchdog's decision: once the deadline the loop last beat
// passes with a command in force, every poll neutralizes the outputs until a
// beat moves it, the stall is reported once, and a loop that keeps beating is
// never touched.
#include "LoopWatchdog.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace {

using talos::rio::LoopWatchdog;

// Outputs are live after the loop applies a command and neutral after the
// watchdog's Neutralize.
class RecordingOutputs final : public talos::rio::WatchdogOutputs {
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

// Expiry plus one 5 ms period: the deadline Robot.cpp beats.
constexpr uint64_t kDeadlineAfterUs = 105'000;
constexpr uint64_t kPeriodUs = 5'000;

// A loop that beats every period with a command in force is never touched,
// however long it runs; a missing beat is acted on only once its deadline
// has passed.
TEST(LoopWatchdog, ActsOnlyOnceTheLastArmedDeadlinePasses) {
  RecordingOutputs outputs;
  LoopWatchdog watchdog{outputs};
  uint64_t now_us = 1'000'000;
  for (int tick = 0; tick < 100; ++tick, now_us += kPeriodUs) {
    watchdog.Beat(now_us, now_us + kDeadlineAfterUs);
    watchdog.Poll(now_us + kPeriodUs - 1);
  }
  EXPECT_EQ(outputs.neutralize_calls, 0);

  const uint64_t deadline_us = now_us - kPeriodUs + kDeadlineAfterUs;
  watchdog.Poll(deadline_us - 1);
  EXPECT_EQ(outputs.neutralize_calls, 0);
  watchdog.Poll(deadline_us);
  EXPECT_EQ(outputs.neutralize_calls, 1);
  EXPECT_EQ(watchdog.stall_count(), 1u);
  EXPECT_EQ(outputs.reports.size(), 1u);
}

// A tick that ends with no command in force neutralized the outputs itself:
// however long the loop then stays silent, the watchdog does nothing.
TEST(LoopWatchdog, ADisarmingBeatLeavesALongSilenceAlone) {
  RecordingOutputs outputs;
  LoopWatchdog watchdog{outputs};
  watchdog.Beat(1'000'000, 1'000'000 + kDeadlineAfterUs);
  watchdog.Beat(1'005'000, 0);
  watchdog.Poll(10'000'000);
  EXPECT_EQ(outputs.neutralize_calls, 0);
  EXPECT_EQ(watchdog.stall_count(), 0u);
}

// The loop resumes on a clock reading taken before its stall, re-applies the
// command it accepted then, and stalls again before its next beat: the
// re-applied outputs are neutralized on the next poll, and every poll after,
// with the stall reported once.
TEST(LoopWatchdog, OutputsReappliedWithoutABeatAreNeutralizedAgain) {
  RecordingOutputs outputs;
  LoopWatchdog watchdog{outputs};
  const uint64_t deadline_us = 1'000'000 + kDeadlineAfterUs;
  watchdog.Beat(1'000'000, deadline_us);
  outputs.live = true;
  watchdog.Poll(deadline_us);
  EXPECT_FALSE(outputs.live);

  outputs.live = true;  // the resumed tick's Apply, then a second stall
  watchdog.Poll(deadline_us + kPeriodUs);
  EXPECT_FALSE(outputs.live);
  for (uint64_t now_us = deadline_us + 2 * kPeriodUs;
       now_us < deadline_us + 1'000'000; now_us += kPeriodUs)
    watchdog.Poll(now_us);
  EXPECT_EQ(outputs.neutralize_calls, 200);
  EXPECT_EQ(outputs.reports.size(), 1u);
  EXPECT_EQ(watchdog.stall_count(), 1u);
}

// A beat from the resumed loop moves the deadline: no action while it keeps
// beating, and a second stall is acted on and reported again.
TEST(LoopWatchdog, AResumedLoopReArmsIt) {
  RecordingOutputs outputs;
  LoopWatchdog watchdog{outputs};
  watchdog.Beat(1'000'000, 1'000'000 + kDeadlineAfterUs);
  watchdog.Poll(2'000'000);
  ASSERT_EQ(outputs.neutralize_calls, 1);

  uint64_t now_us = 2'000'000;
  for (int tick = 0; tick < 40; ++tick, now_us += kPeriodUs) {
    watchdog.Beat(now_us, now_us + kDeadlineAfterUs);
    watchdog.Poll(now_us + 1);
  }
  EXPECT_EQ(outputs.neutralize_calls, 1);

  watchdog.Poll(now_us - kPeriodUs + kDeadlineAfterUs);
  EXPECT_EQ(outputs.neutralize_calls, 2);
  EXPECT_EQ(watchdog.stall_count(), 2u);
  EXPECT_EQ(outputs.reports.size(), 2u);
}

}  // namespace
