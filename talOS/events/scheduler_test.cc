#include "talOS/events/scheduler.h"

#include <gtest/gtest.h>

#include <vector>

namespace talos::event {
namespace {

using namespace std::chrono_literals;

MonotonicTime at(std::int64_t nanos) {
  return MonotonicTime::from_nanos(nanos);
}

std::vector<std::uint16_t> drain(Scheduler& scheduler, MonotonicTime now) {
  std::vector<std::uint16_t> fired;
  Expiration expiration{};

  while (scheduler.pop_expired(now, expiration)) {
    fired.push_back(expiration.id);
  }
  return fired;
}

TEST(Scheduler, EmptyHasNoDeadline) {
  Scheduler scheduler;

  EXPECT_TRUE(scheduler.empty());
  EXPECT_EQ(scheduler.next_deadline(), MonotonicTime::max());

  Expiration expiration{};
  EXPECT_FALSE(scheduler.pop_expired(at(1'000), expiration));
}

TEST(Scheduler, FiresInDeadlineOrder) {
  Scheduler scheduler;

  scheduler.schedule(3, at(300));
  scheduler.schedule(1, at(100));
  scheduler.schedule(2, at(200));

  EXPECT_EQ(scheduler.next_deadline(), at(100));
  EXPECT_EQ(drain(scheduler, at(250)), (std::vector<std::uint16_t>{1, 2}));
  EXPECT_EQ(drain(scheduler, at(300)), (std::vector<std::uint16_t>{3}));
}

// The whole point of the id tiebreak: two timers due in the same iteration
// must fire in a fixed order, or a replayed run could dispatch them the other
// way round and diverge for no reason.
TEST(Scheduler, TiesBreakOnIdRegardlessOfInsertionOrder) {
  Scheduler ascending;
  ascending.schedule(1, at(500));
  ascending.schedule(2, at(500));
  ascending.schedule(3, at(500));

  Scheduler descending;
  descending.schedule(3, at(500));
  descending.schedule(2, at(500));
  descending.schedule(1, at(500));

  const std::vector<std::uint16_t> expected{1, 2, 3};
  EXPECT_EQ(drain(ascending, at(500)), expected);
  EXPECT_EQ(drain(descending, at(500)), expected);
}

TEST(Scheduler, NotDueBeforeDeadline) {
  Scheduler scheduler;
  scheduler.schedule(1, at(1'000));

  Expiration expiration{};
  EXPECT_FALSE(scheduler.pop_expired(at(999), expiration));
  EXPECT_TRUE(scheduler.pop_expired(at(1'000), expiration));
  EXPECT_EQ(expiration.deadline, at(1'000));
  EXPECT_EQ(expiration.cycles, 1u);
}

TEST(Scheduler, PeriodicStaysOnPhaseGrid) {
  Scheduler scheduler;
  scheduler.schedule(1, at(1'000), Duration{1'000});

  Expiration expiration{};

  for (std::int64_t cycle = 1; cycle <= 5; ++cycle) {
    // Poll slightly late every time. Deadlines must not drift.
    ASSERT_TRUE(scheduler.pop_expired(at(cycle * 1'000 + 17), expiration));
    EXPECT_EQ(expiration.deadline, at(cycle * 1'000));
    EXPECT_EQ(expiration.cycles, 1u);
  }
}

// After an overrun the timer fires once and says how many periods it missed,
// rather than firing once per missed period. A control loop that fell behind
// must not then be handed a burst it also cannot service.
TEST(Scheduler, OverrunReportsCyclesAndDoesNotBurst) {
  Scheduler scheduler;
  scheduler.schedule(1, at(1'000), Duration{1'000});

  Expiration expiration{};
  ASSERT_TRUE(scheduler.pop_expired(at(4'500), expiration));
  EXPECT_EQ(expiration.deadline, at(1'000));
  EXPECT_EQ(expiration.cycles, 4u);

  // Next deadline is back on the grid at 5000, not at 2000.
  EXPECT_EQ(scheduler.next_deadline(), at(5'000));
  EXPECT_FALSE(scheduler.pop_expired(at(4'999), expiration));
}

TEST(Scheduler, OneShotDoesNotRepeat) {
  Scheduler scheduler;
  scheduler.schedule(7, at(100));

  Expiration expiration{};
  ASSERT_TRUE(scheduler.pop_expired(at(100), expiration));
  EXPECT_EQ(expiration.id, 7);
  EXPECT_TRUE(scheduler.empty());
}

TEST(Scheduler, DisableRemovesTimer) {
  Scheduler scheduler;
  scheduler.schedule(1, at(100));
  scheduler.schedule(2, at(200));

  scheduler.disable(1);
  EXPECT_EQ(scheduler.next_deadline(), at(200));
  EXPECT_EQ(drain(scheduler, at(500)), (std::vector<std::uint16_t>{2}));

  // Disabling something that is not armed is harmless.
  scheduler.disable(99);
  EXPECT_TRUE(scheduler.empty());
}

TEST(Scheduler, ReschedulingReplacesTheDeadline) {
  Scheduler scheduler;
  scheduler.schedule(1, at(100));
  scheduler.schedule(1, at(900));

  EXPECT_EQ(scheduler.next_deadline(), at(900));
  EXPECT_EQ(drain(scheduler, at(1'000)), (std::vector<std::uint16_t>{1}));
  EXPECT_TRUE(scheduler.empty());
}

TEST(Scheduler, DisablingAPeriodicTimerStopsIt) {
  Scheduler scheduler;
  scheduler.schedule(1, at(100), Duration{100});

  Expiration expiration{};
  ASSERT_TRUE(scheduler.pop_expired(at(100), expiration));

  scheduler.disable(1);
  EXPECT_TRUE(scheduler.empty());
  EXPECT_FALSE(scheduler.pop_expired(at(10'000), expiration));
}

// Same inputs, same order, every time: the property the log depends on.
TEST(Scheduler, IsReproducible) {
  const auto run = [] {
    Scheduler scheduler;
    scheduler.schedule(1, at(1'000), Duration{1'000});
    scheduler.schedule(2, at(1'000), Duration{2'500});
    scheduler.schedule(3, at(500), Duration{750});

    std::vector<std::pair<std::uint16_t, std::int64_t>> fired;
    Expiration expiration{};

    for (std::int64_t now = 0; now <= 10'000; now += 250) {
      while (scheduler.pop_expired(at(now), expiration)) {
        fired.emplace_back(expiration.id, expiration.deadline.nanos());
      }
    }
    return fired;
  };

  EXPECT_EQ(run(), run());
  EXPECT_FALSE(run().empty());
}

}  // namespace
}  // namespace talos::event
