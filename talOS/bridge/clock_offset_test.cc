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

// A datagram delayed from a boot the gateway has left must not restart the
// window back onto it: it is ignored, however high its sample time. Only the
// last BootHistory::kRetiredBoots boots are remembered.
TEST(ClockOffset, ARetiredBootIsIgnored) {
  Estimator clock;
  ASSERT_TRUE(clock.Observe(3, 900'000'000, kBridgeStartUs));
  ASSERT_TRUE(clock.Observe(4, 1000, kBridgeStartUs + 5000));
  EXPECT_FALSE(clock.Observe(3, 900'005'000, kBridgeStartUs + 6000));
  EXPECT_EQ(clock.boot_id(), 4u);
  EXPECT_EQ(clock.window_count(), 1u);
  EXPECT_EQ(clock.resets_count(), 1u);
  EXPECT_EQ(clock.ignored_count(), 1u);
  EXPECT_TRUE(clock.Observe(4, 6000, kBridgeStartUs + 10000));

  // Boots 5..8 push boot 3 out of the retired set; boot 4 is still in it.
  for (std::uint64_t boot = 5; boot <= 8; ++boot) {
    ASSERT_TRUE(clock.Observe(boot, 1000, kBridgeStartUs + 20000 * boot));
  }
  EXPECT_FALSE(clock.Observe(4, 11000, kBridgeStartUs + 200000));
  EXPECT_TRUE(clock.Observe(3, 1000, kBridgeStartUs + 210000));
  EXPECT_EQ(clock.boot_id(), 3u);
}

TEST(ClockOffset, DuplicateAndReorderedSamplesAreIgnored) {
  Estimator clock;
  for (std::uint64_t i = 0; i < 10; ++i) {
    clock.Observe(3, kGatewayStartUs + 5000 * i,
                  kBridgeStartUs + 5000 * i + 500);
  }
  const std::uint64_t last_sample = kGatewayStartUs + 5000ULL * 9;
  ASSERT_EQ(clock.offset_us(), kTrueOffsetUs + 500);

  // A duplicate of the last frame and the frame before it, both delivered
  // again 3 ms late (a retransmit, a reordering switch). Taken in, either
  // would widen the spread and keep a sample the gateway's own clock says
  // is not new.
  const std::uint64_t late_us = kBridgeStartUs + 5000ULL * 9 + 3500;
  EXPECT_FALSE(clock.Observe(3, last_sample, late_us));
  EXPECT_FALSE(clock.Observe(3, last_sample - 5000, late_us + 10));
  EXPECT_EQ(clock.ignored_count(), 2u);
  EXPECT_EQ(clock.window_count(), 10u);
  EXPECT_EQ(clock.offset_us(), kTrueOffsetUs + 500);
  EXPECT_EQ(clock.spread_us(), 0u);

  // The next fresh frame is taken.
  EXPECT_TRUE(clock.Observe(3, last_sample + 5000,
                            kBridgeStartUs + 5000ULL * 10 + 400));
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

// Either sign of clock drift must move the window's minimum: fast clocks
// retire the old minimum, while slow clocks immediately use the newest.
TEST(ClockOffset, FollowsBridgeClockDriftInEitherDirection) {
  const std::uint64_t n = 3 * Estimator::kWindowSamples;
  for (const std::int64_t drift : {-1, 1}) {
    Estimator clock;
    for (std::uint64_t i = 0; i < n; ++i) {
      clock.Observe(3, kGatewayStartUs + 5000 * i,
                    kBridgeStartUs + (5000 + drift) * i);
    }
    const std::int64_t minimum_index =
        drift > 0 ? n - Estimator::kWindowSamples : n - 1;
    EXPECT_EQ(clock.offset_us(), kTrueOffsetUs + drift * minimum_index);
    EXPECT_EQ(clock.spread_us(), Estimator::kWindowSamples - 1);
    EXPECT_EQ(clock.window_count(), Estimator::kWindowSamples);
  }
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

TEST(ClockOffset, ConstantDelaySpreadIsNotAnAccuracyBound) {
  Estimator estimator;
  for (uint64_t i = 0; i < 8; ++i) {
    estimator.Observe(1, 1000 + i * 5000, 11000 + i * 5000);
  }
  ASSERT_TRUE(estimator.converged());
  EXPECT_EQ(estimator.spread_us(), 0u);
  // True offset is 7000 us; constant 3000 us transport delay is invisible.
  EXPECT_GT(estimator.offset_us() - 7000,
            static_cast<int64_t>(estimator.spread_us()));
  EXPECT_FALSE(estimator.accuracy_valid());
}

TEST(ClockOffset, CausalBoundContainsErrorWithOffsetDelayJitterAndAsymmetry) {
  for (const int64_t offset : {-500000, 0, 900000}) {
    for (const uint64_t forward : {0u, 100u, 5000u, 30000u}) {
      for (const uint64_t reverse : {0u, 300u, 7000u, 40000u}) {
        Estimator estimator;
        for (uint64_t i = 0; i < 500; ++i) {
          const uint64_t send = 2000000 + i * 20000;
          const uint64_t gateway_sample =
              send + forward + (i * 97 % 1000) + 5000 - offset;
          const uint64_t receive =
              gateway_sample + offset + reverse + (i * 71 % 2000) + 1000;
          ASSERT_TRUE(estimator.Observe(1, gateway_sample, receive));
          estimator.ObserveExchange(1, send, gateway_sample, receive);
          ASSERT_TRUE(estimator.accuracy_valid());
          const auto error = estimator.bounded_offset_us() - offset;
          EXPECT_LE(error, static_cast<int64_t>(estimator.uncertainty_us()));
          EXPECT_GE(error, -static_cast<int64_t>(estimator.uncertainty_us()));
        }
        EXPECT_TRUE(estimator.converged());
      }
    }
  }
}

TEST(ClockOffset, ExchangeExpiresAndResetsWithBoot) {
  Estimator estimator;
  estimator.Observe(1, 1000, 10000);
  estimator.ObserveExchange(1, 8000, 1000, 10000);
  ASSERT_TRUE(estimator.accuracy_valid());
  estimator.Observe(1, 1001001, 1010001);
  EXPECT_FALSE(estimator.accuracy_valid());
  estimator.ObserveExchange(1, 1008000, 1001001, 1010001);
  ASSERT_TRUE(estimator.accuracy_valid());
  estimator.Observe(2, 1, 1015000);
  EXPECT_FALSE(estimator.accuracy_valid());
  estimator.ObserveExchange(1, 1010000, 1, 1015000);
  EXPECT_FALSE(estimator.accuracy_valid());
}

TEST(ClockOffset, RetainedMinimumRoundTripBoundPropagatesOscillatorDrift) {
  for (const int64_t drift_per_tick : {-10, 10}) {
    Estimator estimator;
    estimator.Observe(1, 1000000, 2000000);
    estimator.ObserveExchange(1, 2000000, 1000000, 2000000);
    for (uint64_t i = 1; i < 100; ++i) {
      const uint64_t sample = 1000000 + i * 10000;
      const int64_t offset = 1000000 + static_cast<int64_t>(i) * drift_per_tick;
      estimator.Observe(1, sample, sample + offset);
      ASSERT_TRUE(estimator.accuracy_valid());
      const auto error = estimator.bounded_offset_us() - offset;
      EXPECT_LE(error, static_cast<int64_t>(estimator.uncertainty_us()));
      EXPECT_GE(error, -static_cast<int64_t>(estimator.uncertainty_us()));
    }
  }
}

}  // namespace
}  // namespace talos::hardware
