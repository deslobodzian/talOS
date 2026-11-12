// SplitLogBudget as a pure function: shares and the session's log window
// (REQ-OPS-32). No process, file or clock.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "talOS/launcher/log_budget.h"

namespace talos::launcher {
namespace {

// REQ-OPS-32: shares are in proportion to the expected rates, so equal
// rates of history fit each share; an unknown rate weighs as the mean of the
// known; a member whose share would not rotate gets its floor and the others
// share the rest; a member without a log gets nothing; the shares never
// exceed the budget, and floors that do not fit are scaled into it. The
// window is the budget left after the floors over the free members' summed
// rates (T), less the largest member's eight segment prefixes over its rate
// (half a floor each), and never less than 7/8 of T; none when the floors do
// not fit.
TEST(SplitLogBudget, IsProportionalAboveEachFloor) {
  const auto split = SplitLogBudget(4000, {true, true, true, false},
                                    {300.0, 100.0, -1.0, 500.0}, {0, 0, 0, 0});
  const auto& shares = split.shares;
  ASSERT_EQ(shares.size(), 4u);
  // 300 : 100 : 200 (the mean).
  EXPECT_EQ(shares[0], 2000u);
  EXPECT_EQ(shares[1], 666u);
  EXPECT_EQ(shares[2], 1333u);
  EXPECT_EQ(shares[3], 0u);
  // 4000 B over 600 B/s, no prefixes.
  EXPECT_EQ(split.window, std::chrono::microseconds{6'666'666});

  // The second member's proportional 400 is below its 1000 floor: it is
  // held there and the other two share the remaining 3000 as 3 : 2, so
  // T = 3000 B / 500 B/s. The held member's prefixes (4 x 1000 B / 40 B/s)
  // exceed T / 8, so the window is 7/8 of T.
  const auto floored = SplitLogBudget(4000, {true, true, true},
                                      {300.0, 40.0, 200.0}, {100, 1000, 100});
  EXPECT_EQ(floored.shares[0], 1800u);
  EXPECT_EQ(floored.shares[1], 1000u);
  EXPECT_EQ(floored.shares[2], 1200u);
  EXPECT_EQ(floored.window, std::chrono::microseconds{5'250'000});
  // Prefixes within T / 8: T = 4000 B / 400 B/s, less the larger prefix
  // cost, 4 x 10 B / 100 B/s.
  const auto small =
      SplitLogBudget(4000, {true, true}, {300.0, 100.0}, {10, 10});
  EXPECT_EQ(small.window, std::chrono::microseconds{9'600'000});

  const auto tight =
      SplitLogBudget(1000, {true, true}, {1.0, 1.0}, {1500, 500});
  EXPECT_EQ(tight.shares[0], 750u);
  EXPECT_EQ(tight.shares[1], 250u);
  EXPECT_EQ(tight.window, std::chrono::microseconds{0});
}

// REQ-OPS-32. A larger budget never shortens the window, and while the
// floors fit it is at least 7/8 of T (a free member's share over its rate);
// a member whose prefixes cost no more than T / 8 holds the whole window in
// its records. A window that followed each member's share less a whole
// number of prefixes collapsed whenever a share sat just above a multiple of
// its prefix: rates 20000 and 1000 B/s, floors 40000 and 20000 gave 9.9 s at
// 627900 B and 0.1 s at 632100 B, and everyone's log then rotated every
// 12.5 ms.
TEST(SplitLogBudget, WindowNeverShrinksAsTheBudgetGrows) {
  struct Case {
    std::vector<double> rates;
    std::vector<uint64_t> floors;
  };
  const std::vector<Case> cases = {
      {{20000.0, 1000.0}, {40000, 20000}},
      {{300.0, 40.0, 200.0}, {100, 1000, 100}},
      {{71400.0, 187200.0, 45400.0, 45400.0}, {9000, 12000, 8000, 8000}},
      {{5.0, 50000.0, -1.0}, {30000, 2000, 0}},
  };
  for (std::size_t c = 0; c < cases.size(); ++c) {
    SCOPED_TRACE(c);
    const Case& in = cases[c];
    const std::vector<bool> logs(in.rates.size(), true);
    uint64_t floor_sum = 0;
    for (const uint64_t floor : in.floors) floor_sum += floor;
    std::chrono::microseconds previous{0};
    for (uint64_t budget = 0; budget <= 4'000'000; budget += 700) {
      SCOPED_TRACE(budget);
      const LogSplit split = SplitLogBudget(budget, logs, in.rates, in.floors);
      ASSERT_GE(split.window, previous);
      previous = split.window;
      if (budget < floor_sum) continue;
      // T and the largest prefix cost from a free (unfloored) known member.
      double t = 0;
      double cost = 0;
      for (std::size_t i = 0; i < in.rates.size(); ++i) {
        if (in.rates[i] <= 0) continue;
        cost = std::max(cost,
                        4.0 * static_cast<double>(in.floors[i]) / in.rates[i]);
        if (split.shares[i] > in.floors[i]) {
          t = static_cast<double>(split.shares[i]) / in.rates[i];
        }
      }
      if (t == 0) continue;
      const double window = std::chrono::duration<double>(split.window).count();
      EXPECT_GE(window, 7.0 / 8.0 * t - 1e-3);
      if (cost <= t / 8) {
        EXPECT_GE(window, t - cost - 1e-3);
      }
    }
  }
}

}  // namespace
}  // namespace talos::launcher
