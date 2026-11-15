// When the RoboRIO loop wakes for its next tick. It runs at SCHED_FIFO on the
// RoboRIO, so how it sleeps after an overrun decides whether every
// default-scheduled thread on its core still gets the CPU, and how it sleeps
// on time decides whether its State keeps the 5 ms cadence the watchdog's
// two-period margin assumes.
#include "LoopSchedule.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

namespace {

using talos::rio::kOverrunSleepUs;
using talos::rio::NextWakeUs;

constexpr uint64_t kPeriodUs = 5'000;
// The loop's first deadline: an arbitrary steady-clock time.
constexpr uint64_t kStartUs = 7'000'000;

// Runs the loop as Robot.cpp does (tick start, deadline += period, work,
// NextWakeUs, sleep), each tick's work taking the next entry of `work_us`, and
// returns the tick start times.
std::vector<uint64_t> TickStarts(const std::vector<uint64_t>& work_us) {
  std::vector<uint64_t> starts;
  uint64_t deadline = kStartUs;
  uint64_t wake = kStartUs;
  for (const uint64_t work : work_us) {
    starts.push_back(wake);
    deadline += kPeriodUs;
    deadline = NextWakeUs(deadline, wake + work);
    wake = deadline;
  }
  return starts;
}

// Scenario: ticks whose work ends anywhere up to their deadline, including
// exactly on it. Each tick must start on the original phase, one period after
// the previous: the work's length must not drift the cadence.
// REQ-SAFE-25
TEST(LoopSchedule, KeepsPhaseWhileEveryTickEndsByItsDeadline) {
  const std::vector<uint64_t> work{100, 4'999, 5'000, 0, 2'500};
  const auto starts = TickStarts(work);
  for (std::size_t i = 0; i < starts.size(); ++i) {
    SCOPED_TRACE("tick " + std::to_string(i));
    EXPECT_EQ(starts[i], kStartUs + i * kPeriodUs);
  }
}

// Scenario (bench, ssh session on the RoboRIO): a tick's work runs 11.5 ms of
// a 5 ms period, then the loop is on time again. The missed deadlines must be
// dropped, not caught up back to back; the loop must yield kOverrunSleepUs
// before its next tick (at SCHED_FIFO, all a loop that overruns every tick
// leaves the default-scheduled threads on its core); and that tick must
// restart the period, so the phase moves.
// REQ-SAFE-25
TEST(LoopSchedule, AnOverrunYieldsThenRestartsThePeriod) {
  const auto starts = TickStarts({11'500, 100, 100});
  ASSERT_EQ(starts.size(), 3U);
  const uint64_t resumed = kStartUs + 11'500 + kOverrunSleepUs;
  EXPECT_EQ(starts[1], resumed) << "the tick after an 11.5 ms overrun";
  EXPECT_EQ(starts[2], resumed + kPeriodUs) << "the period counts from it";
}

}  // namespace
