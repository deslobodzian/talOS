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

// REQ-RT-04
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
// REQ-RT-04
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

// REQ-RT-03
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
// REQ-RT-03
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

// REQ-RT-04
// REQ-OPS-14
TEST(Scheduler, ShiftSaturatesAtTimeBoundsAndPreservesHeapOrder) {
  {
    Scheduler at_max;
    at_max.schedule(1, MonotonicTime::max());
    at_max.shift(1ms);
    EXPECT_EQ(at_max.next_deadline(), MonotonicTime::max());
  }

  Scheduler upper;
  upper.schedule(2, MonotonicTime::max() - Duration{5});
  upper.schedule(1, MonotonicTime::max() - Duration{1});
  upper.shift(Duration{10});
  EXPECT_EQ(upper.next_deadline(), MonotonicTime::max());
  // Both saturated to MonotonicTime::max(), so tie-breaking must fire id 1
  // before id 2.
  EXPECT_EQ(drain(upper, MonotonicTime::max()),
            (std::vector<std::uint16_t>{1, 2}));
  EXPECT_TRUE(upper.empty());

  Scheduler lower;
  lower.schedule(1, MonotonicTime::min() + Duration{5});
  lower.shift(Duration{-10});
  EXPECT_EQ(lower.next_deadline(), MonotonicTime::min());
  EXPECT_EQ(drain(lower, MonotonicTime::min()),
            (std::vector<std::uint16_t>{1}));
}

// REQ-RT-04
// REQ-OPS-14
TEST(Scheduler, PeriodicTimerNearMaxStopsRearmingInsteadOfOverflowing) {
  {
    // Firing on time at INT64_MAX - 500 with period 1000 must not overflow
    // deadline + period into a negative deadline and loop forever.
    Scheduler scheduler;
    const MonotonicTime deadline = MonotonicTime::max() - Duration{500};
    scheduler.schedule(1, deadline, Duration{1000});

    Expiration expiration{};
    ASSERT_TRUE(scheduler.pop_expired(deadline, expiration));
    EXPECT_EQ(expiration.id, 1u);
    EXPECT_EQ(expiration.deadline, deadline);
    EXPECT_EQ(expiration.cycles, 1u);
    EXPECT_TRUE(scheduler.empty());
    EXPECT_FALSE(scheduler.pop_expired(MonotonicTime::max(), expiration));
  }

  {
    Scheduler scheduler;
    const MonotonicTime deadline = MonotonicTime::max() - Duration{5};
    scheduler.schedule(1, deadline, Duration{10});

    Expiration expiration{};
    ASSERT_TRUE(scheduler.pop_expired(MonotonicTime::max(), expiration));
    EXPECT_EQ(expiration.id, 1u);
    EXPECT_EQ(expiration.deadline, deadline);
    EXPECT_EQ(expiration.cycles, 1u);
    // Next phase would exceed MonotonicTime::max(), so it stops re-arming
    // instead of wrapping negative and looping forever.
    EXPECT_TRUE(scheduler.empty());
    EXPECT_FALSE(scheduler.pop_expired(MonotonicTime::max(), expiration));
  }

  {
    Scheduler scheduler;
    const MonotonicTime deadline = MonotonicTime::max() - Duration{10};
    scheduler.schedule(1, deadline, Duration{10});

    Expiration expiration{};
    ASSERT_TRUE(scheduler.pop_expired(deadline, expiration));
    EXPECT_EQ(expiration.deadline, deadline);
    EXPECT_EQ(expiration.cycles, 1u);
    EXPECT_EQ(scheduler.next_deadline(), MonotonicTime::max());

    ASSERT_TRUE(scheduler.pop_expired(MonotonicTime::max(), expiration));
    EXPECT_EQ(expiration.deadline, MonotonicTime::max());
    EXPECT_EQ(expiration.cycles, 1u);
    EXPECT_TRUE(scheduler.empty());
  }

  {
    // Negative deadline to MonotonicTime::max() spans more than INT64_MAX
    // nanoseconds; unsigned arithmetic must compute exact cycles without UB.
    Scheduler scheduler;
    scheduler.schedule(1, at(-1000), Duration{1000});

    Expiration expiration{};
    ASSERT_TRUE(scheduler.pop_expired(MonotonicTime::max(), expiration));
    EXPECT_EQ(expiration.deadline, at(-1000));
    EXPECT_EQ(expiration.cycles, 9'223'372'036'854'777u);
    EXPECT_TRUE(scheduler.empty());
  }

  {
    Scheduler scheduler;
    scheduler.schedule(1, MonotonicTime::min(), Duration{10});

    Expiration expiration{};
    ASSERT_TRUE(scheduler.pop_expired(MonotonicTime::max(), expiration));
    EXPECT_EQ(expiration.deadline, MonotonicTime::min());
    EXPECT_GT(expiration.cycles, 1u);
    EXPECT_TRUE(scheduler.empty());
  }
}

}  // namespace
}  // namespace talos::event
