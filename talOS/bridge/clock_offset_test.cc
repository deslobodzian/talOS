#include "talOS/bridge/clock_offset.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <iterator>

namespace talos::hardware {
namespace {

using Estimator = ClockOffsetEstimator;

// The gateway's clock starts near zero at boot; the bridge's reads node time.
constexpr std::uint64_t kGatewayStartUs = 1000;
constexpr std::uint64_t kBridgeStartUs = 50'000'000;
constexpr std::int64_t kTrueOffsetUs =
    static_cast<std::int64_t>(kBridgeStartUs - kGatewayStartUs);

TEST(ClockOffset, InvalidUntilEnoughSamplesThenTheMinimumDelay) {
  Estimator clock;
  EXPECT_FALSE(clock.converged());
  EXPECT_EQ(clock.boot_id(), 0u);
  EXPECT_EQ(clock.window_count(), 0u);

  // One-way latencies from 300 to 1200 us: the offset is the true one plus
  // the smallest, and the spread is the largest minus the smallest.
  const std::uint64_t latencies[] = {900, 300, 700, 1200, 450, 300, 800, 650};
  static_assert(std::size(latencies) == Estimator::kConvergedSamples);
  for (std::size_t i = 0; i < std::size(latencies); ++i) {
    EXPECT_FALSE(clock.converged()) << "converged after " << i << " samples";
    const std::uint64_t sample = kGatewayStartUs + 5000 * i;
    ASSERT_TRUE(
        clock.Observe(3, sample, kBridgeStartUs + 5000 * i + latencies[i]));
  }
  EXPECT_TRUE(clock.converged());
  EXPECT_EQ(clock.boot_id(), 3u);
  EXPECT_EQ(clock.window_count(), Estimator::kConvergedSamples);
  EXPECT_EQ(clock.offset_us(), kTrueOffsetUs + 300);
  EXPECT_EQ(clock.spread_us(), 1200u - 300u);
  EXPECT_EQ(clock.resets_count(), 0u);
  EXPECT_EQ(clock.ignored_count(), 0u);
}

TEST(ClockOffset, AGatewayRebootRestartsTheWindow) {
  Estimator clock;
  for (std::uint64_t i = 0; i < 20; ++i) {
    clock.Observe(3, kGatewayStartUs + 5000 * i,
                  kBridgeStartUs + 5000 * i + 200);
  }
  ASSERT_TRUE(clock.converged());
  ASSERT_EQ(clock.offset_us(), kTrueOffsetUs + 200);

  // Boot 4 restarts the gateway clock at 1000 again, 10 s later in node time,
  // and with a slower link: the old boot's 200 us minimum must not survive.
  const std::uint64_t reboot_us = kBridgeStartUs + 10'000'000;
  ASSERT_TRUE(clock.Observe(4, kGatewayStartUs, reboot_us + 600));
  EXPECT_FALSE(clock.converged());
  EXPECT_EQ(clock.boot_id(), 4u);
  EXPECT_EQ(clock.window_count(), 1u);
  EXPECT_EQ(clock.resets_count(), 1u);
  EXPECT_EQ(clock.spread_us(), 0u);
  for (std::uint64_t i = 1; i < Estimator::kConvergedSamples; ++i) {
    EXPECT_FALSE(clock.converged());
    ASSERT_TRUE(clock.Observe(4, kGatewayStartUs + 5000 * i,
                              reboot_us + 5000 * i + 600));
  }
  EXPECT_TRUE(clock.converged());
  EXPECT_EQ(clock.offset_us(),
            static_cast<std::int64_t>(reboot_us - kGatewayStartUs) + 600);
}

TEST(ClockOffset, DuplicateAndReorderedSamplesAreIgnored) {
  Estimator clock;
  for (std::uint64_t i = 0; i < 10; ++i) {
    clock.Observe(3, kGatewayStartUs + 5000 * i,
                  kBridgeStartUs + 5000 * i + 500);
  }
  const std::uint64_t last_sample = kGatewayStartUs + 5000 * 9;
  ASSERT_EQ(clock.offset_us(), kTrueOffsetUs + 500);

  // A duplicate of the last frame and the frame before it, both delivered
  // again 3 ms late (a retransmit, a reordering switch). Taken in, either
  // would widen the spread and keep a sample the gateway's own clock says
  // is not new.
  const std::uint64_t late_us = kBridgeStartUs + 5000 * 9 + 3500;
  EXPECT_FALSE(clock.Observe(3, last_sample, late_us));
  EXPECT_FALSE(clock.Observe(3, last_sample - 5000, late_us + 10));
  EXPECT_EQ(clock.ignored_count(), 2u);
  EXPECT_EQ(clock.window_count(), 10u);
  EXPECT_EQ(clock.offset_us(), kTrueOffsetUs + 500);
  EXPECT_EQ(clock.spread_us(), 0u);

  // The next fresh frame is taken.
  EXPECT_TRUE(
      clock.Observe(3, last_sample + 5000, kBridgeStartUs + 5000 * 10 + 400));
  EXPECT_EQ(clock.offset_us(), kTrueOffsetUs + 400);
  EXPECT_EQ(clock.window_count(), 11u);
}

TEST(ClockOffset, TheFirstSampleOfABootMayBeAtGatewayTimeZero) {
  Estimator clock;
  EXPECT_TRUE(clock.Observe(5, 0, kBridgeStartUs));
  EXPECT_TRUE(clock.Observe(5, 5000, kBridgeStartUs + 5000));
  EXPECT_EQ(clock.ignored_count(), 0u);
  EXPECT_EQ(clock.offset_us(), static_cast<std::int64_t>(kBridgeStartUs));
}

// A bridge clock that runs fast against the gateway's raises the one-way
// delay a little every sample: the old minimum must age out of the window
// rather than pin the offset to where the clocks were long ago.
TEST(ClockOffset, FollowsABridgeClockThatRunsFast) {
  Estimator clock;
  const std::uint64_t n = 3 * Estimator::kWindowSamples;
  for (std::uint64_t i = 0; i < n; ++i) {
    // 5000 us of gateway time is 5001 us of bridge time: 200 ppm.
    clock.Observe(3, kGatewayStartUs + 5000 * i, kBridgeStartUs + 5001 * i);
  }
  // The oldest sample in the window is i = n - kWindowSamples, whose delay
  // is the true starting offset plus i us.
  const std::int64_t oldest = n - Estimator::kWindowSamples;
  EXPECT_EQ(clock.offset_us(), kTrueOffsetUs + oldest);
  EXPECT_EQ(clock.spread_us(), Estimator::kWindowSamples - 1);
  EXPECT_EQ(clock.window_count(), Estimator::kWindowSamples);
}

// One that runs slow lowers the delay every sample, and the newest sample
// is the minimum at once.
TEST(ClockOffset, FollowsABridgeClockThatRunsSlow) {
  Estimator clock;
  for (std::uint64_t i = 0; i < 50; ++i) {
    clock.Observe(3, kGatewayStartUs + 5000 * i, kBridgeStartUs + 4999 * i);
  }
  EXPECT_EQ(clock.offset_us(), kTrueOffsetUs - 49);
  EXPECT_EQ(clock.spread_us(), 49u);
}

// A single slow frame widens the spread but never moves the offset, and it
// leaves the window after kWindowSamples more.
TEST(ClockOffset, ALatencySpikeOnlyWidensTheSpreadUntilItAgesOut) {
  Estimator clock;
  std::uint64_t i = 0;
  auto observe = [&](std::uint64_t latency_us) {
    clock.Observe(3, kGatewayStartUs + 5000 * i,
                  kBridgeStartUs + 5000 * i + latency_us);
    ++i;
  };
  for (int k = 0; k < 10; ++k) observe(250);
  observe(8250);
  EXPECT_EQ(clock.offset_us(), kTrueOffsetUs + 250);
  EXPECT_EQ(clock.spread_us(), 8000u);
  for (std::size_t k = 0; k + 1 < Estimator::kWindowSamples; ++k) {
    observe(250);
  }
  EXPECT_EQ(clock.spread_us(), 8000u) << "the spike is still in the window";
  observe(250);
  EXPECT_EQ(clock.spread_us(), 0u);
  EXPECT_EQ(clock.offset_us(), kTrueOffsetUs + 250);
}

}  // namespace
}  // namespace talos::hardware
