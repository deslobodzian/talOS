#include "talOS/hardware/clock_offset.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <deque>
#include <iterator>
#include <limits>

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
// REQ-SAFE-11
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

// A bridge cadence no finer than the real one: one probe outstanding and at
// most one every 20 ms, States received on a 1 ms grid (the real bridge
// wakes for each), and each exchange completed on the tick after the State
// that echoes it (the real bridge makes that tick at once). The gateway samples
// every 5 ms of its own clock and echoes a probe on the tick after the one that
// received it. Both one-way delays jitter between 0.1 and 3.1 ms, so each
// exchange's RTT differs, and the gateway clock runs 500 ppm fast. Once valid,
// the mapping stays valid on every State for a minute, and its bound always
// holds.
TEST(ClockOffset, BridgeCadenceExchangesKeepTheBoundValidWithoutGaps) {
  constexpr std::uint64_t kBoot = 7;
  // Gateway microseconds per bridge microsecond, and the bridge time of the
  // gateway's clock zero.
  constexpr double kGatewayRate = 1.0005;
  constexpr std::uint64_t kGatewayBootUs = 40'000'000;
  constexpr std::uint64_t kSimulatedUs = 60'000'000;
  std::uint64_t seed = 1;
  const auto delay_us = [&seed] {
    seed = seed * 6364136223846793005ULL + 1442695040888963407ULL;
    return 100.0 + static_cast<double>((seed >> 33) % 3000);
  };
  const auto bridge_us = [](std::uint64_t gateway_us) {
    return static_cast<double>(kGatewayBootUs) +
           static_cast<double>(gateway_us) / kGatewayRate;
  };

  struct Datagram {
    double arrival_us;       // bridge clock
    std::uint64_t time_us;   // probe: bridge send time; State: gateway sample
    std::uint64_t probe_us;  // State: the echoed probe's send time
  };
  std::deque<Datagram> probes;  // bridge -> gateway
  std::deque<Datagram> states;  // gateway -> bridge
  std::uint64_t gateway_tick_us = 0;
  // The probe the gateway's next State echoes.
  std::uint64_t gateway_probe_us = 0;
  const auto run_gateway_until = [&](std::uint64_t bridge_now_us) {
    for (; bridge_us(gateway_tick_us) <= static_cast<double>(bridge_now_us);
         gateway_tick_us += 5000) {
      const double tick_us = bridge_us(gateway_tick_us);
      const double arrival_us = tick_us + delay_us();
      states.push_back({arrival_us, gateway_tick_us, gateway_probe_us});
      while (!probes.empty() && probes.front().arrival_us <= tick_us) {
        gateway_probe_us = probes.front().time_us;
        probes.pop_front();
      }
    }
  };

  Estimator clock;
  std::uint64_t last_probe_us = 0;
  bool probe_waiting = false;
  bool exchange_pending = false;
  std::uint64_t pending_t1 = 0;
  std::uint64_t pending_t2 = 0;
  std::uint64_t valid_from_us = 0;
  std::uint64_t states_seen = 0;
  std::uint64_t invalid_after_valid = 0;
  std::uint64_t outside_bound = 0;
  for (std::uint64_t now = kGatewayBootUs + 1000;
       now < kGatewayBootUs + kSimulatedUs; now += 1000) {
    if (exchange_pending) {
      clock.ObserveExchange(kBoot, pending_t1, pending_t2, now);
      exchange_pending = false;
      probe_waiting = false;
    }
    if (!probe_waiting &&
        (last_probe_us == 0 || now - last_probe_us >= 20000)) {
      last_probe_us = now;
      probe_waiting = true;
      probes.push_back({static_cast<double>(now) + delay_us(), now, 0});
    }
    run_gateway_until(now);
    while (!states.empty() &&
           states.front().arrival_us <= static_cast<double>(now)) {
      const Datagram state = states.front();
      states.pop_front();
      ASSERT_TRUE(clock.Observe(kBoot, state.time_us, now));
      ++states_seen;
      if (probe_waiting && !exchange_pending &&
          state.probe_us == last_probe_us) {
        exchange_pending = true;
        pending_t1 = state.probe_us;
        pending_t2 = state.time_us;
      }
      if (!clock.accuracy_valid()) {
        invalid_after_valid += valid_from_us != 0;
        continue;
      }
      if (valid_from_us == 0) valid_from_us = now;
      const double true_offset_us =
          bridge_us(state.time_us) - static_cast<double>(state.time_us);
      const double error_us =
          static_cast<double>(clock.bounded_offset_us()) - true_offset_us;
      if (std::abs(error_us) > static_cast<double>(clock.uncertainty_us())) {
        ++outside_bound;
      }
    }
  }
  EXPECT_GT(states_seen, 11'900u);
  ASSERT_NE(valid_from_us, 0u);
  EXPECT_LT(valid_from_us, kGatewayBootUs + 100'000);
  EXPECT_EQ(invalid_after_valid, 0u);
  EXPECT_EQ(outside_bound, 0u);
}

// REQ-RT-20
TEST(ClockOffset, ExtremeTimestampsDoNotOverflowSignedArithmetic) {
  Estimator clock;
  const std::uint64_t max_u64 = std::numeric_limits<std::uint64_t>::max();
  const std::uint64_t max_i64 =
      static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max());
  // Difference > INT64_MAX between receipt_us and sample_time_us, and spread
  // spanning INT64_MIN..INT64_MAX across samples.
  EXPECT_TRUE(clock.Observe(1, 0, max_i64));
  EXPECT_EQ(clock.offset_us(), static_cast<std::int64_t>(max_i64));
  EXPECT_TRUE(clock.Observe(1, 1, 0));
  EXPECT_EQ(clock.offset_us(), -1);
  EXPECT_TRUE(clock.Observe(1, max_u64 / 2, max_u64));
  EXPECT_EQ(clock.offset_us(), std::numeric_limits<std::int64_t>::min());
  EXPECT_GT(clock.spread_us(), max_i64);

  // Two-way exchange with extreme gateway timestamp (t2 near UINT64_MAX):
  // mid = 1500, unsigned modular (mid - t2) = 1500 - (max_u64 - 10) = 1511.
  clock.ObserveExchange(1, /*t1=*/1000, /*t2=*/max_u64 - 10, /*t4=*/2000);
  EXPECT_TRUE(clock.accuracy_valid());
  EXPECT_EQ(clock.round_trip_us(), 1000u);
  EXPECT_EQ(clock.uncertainty_us(), 503u);
  EXPECT_EQ(clock.bounded_offset_us(), 1511);
}

// Guards RT-29: a corrupt or spoofed far-future sample_time_us within a boot
// (including UINT64_MAX) must be ignored rather than advancing
// last_sample_time_us_ and permanently locking out subsequent valid States or
// poisoning the one-way offset estimate.
// REQ-RT-20
TEST(ClockOffset, FarFutureSampleJumpIsIgnoredWithoutLockingOutLaterSamples) {
  Estimator clock;
  for (std::uint64_t i = 0; i < Estimator::kConvergedSamples; ++i) {
    ASSERT_TRUE(clock.Observe(11, 1'000 + 5'000 * i, 1'000'000 + 5'000 * i));
  }
  const std::uint64_t last_sample =
      1'000 + 5'000 * (Estimator::kConvergedSamples - 1);
  const std::uint64_t last_receipt =
      1'000'000 + 5'000 * (Estimator::kConvergedSamples - 1);
  const std::int64_t offset_before = clock.offset_us();
  const std::size_t window_before = clock.window_count();

  // Far-future jumps (+1 hour, UINT64_MAX, and 1 us past the 1 s margin at
  // zero elapsed bridge time), duplicate UINT64_MAX, and inconsistent jump
  // pairs (+1 h then +5 h, or a frozen corrupt pair separated by >1 s) are all
  // ignored without disturbing state.
  EXPECT_FALSE(
      clock.Observe(11, last_sample + 3'600'000'000ULL, last_receipt + 5'000));
  EXPECT_FALSE(clock.Observe(11, last_sample + 18'000'000'000ULL,
                             last_receipt + 10'000));
  EXPECT_FALSE(clock.Observe(11, std::numeric_limits<std::uint64_t>::max(),
                             last_receipt + 15'000));
  EXPECT_FALSE(clock.Observe(11, std::numeric_limits<std::uint64_t>::max(),
                             last_receipt + 20'000));
  EXPECT_FALSE(clock.Observe(11, last_sample + 1'000'001ULL, last_receipt));
  // A frozen corrupt timestamp pair separated by >1 s of bridge time fails the
  // symmetric physical-rate check and must not re-sync.
  EXPECT_FALSE(
      clock.Observe(11, last_sample + 7'200'000'000ULL, last_receipt + 5'000));
  EXPECT_FALSE(clock.Observe(11, last_sample + 7'200'000'001ULL,
                             last_receipt + 2'005'000));
  EXPECT_EQ(clock.ignored_count(), 7u);
  EXPECT_EQ(clock.window_count(), window_before);
  EXPECT_EQ(clock.offset_us(), offset_before);
  EXPECT_EQ(clock.spread_us(), 0u);

  // Subsequent normal samples clear any pending jump candidate and continue to
  // be accepted cleanly; a later corrupt frame matching the old candidate's
  // trajectory is therefore still ignored.
  EXPECT_TRUE(clock.Observe(11, last_sample + 5'000, last_receipt + 5'000));
  EXPECT_FALSE(clock.Observe(11, last_sample + 7'200'005'001ULL,
                             last_receipt + 2'010'000));
  EXPECT_TRUE(clock.Observe(11, last_sample + 10'000, last_receipt + 10'000));
  EXPECT_EQ(clock.ignored_count(), 8u);
  EXPECT_EQ(clock.window_count(), window_before + 2);
  EXPECT_EQ(clock.offset_us(), offset_before);
}

// Guards RT-29 against delayed buffered frames after a long (>500 s) outage:
// a packet delayed past the 1 s jump margin must never pull the prompt anchor
// backward and lock out the live frame immediately behind it, while sustained
// bridge-fast oscillator drift refreshes the anchor over normal streaming so
// the drift allowance never grows unboundedly.
// REQ-RT-20
TEST(ClockOffset, DelayedFrameAfterLongOutageDoesNotLockOutLiveSamples) {
  // 1. 1000 s outage + 1.5 s delayed buffered packet + live frame 5 ms later.
  // Because 1000 s / 500 (2 s) > 1.5 s, a naive anchor-retreat check would
  // pull the anchor back to the delayed packet and reject live frames for
  // ~250 s. Instead, the live frame is accepted on frame 1 (zero frames lost).
  {
    Estimator clock;
    ASSERT_TRUE(clock.Observe(1, 1'000'000, 1'000'000));
    ASSERT_TRUE(clock.Observe(1, 1'000'000 + 999'500'000ULL,
                              1'000'000 + 1'001'000'000ULL));
    EXPECT_TRUE(clock.Observe(1, 1'000'000 + 1'001'005'000ULL,
                              1'000'000 + 1'001'005'000ULL));
    EXPECT_EQ(clock.ignored_count(), 0u);
    // Anchor advanced to the live frame, so 1 us past the 1 s margin is
    // rejected immediately.
    EXPECT_FALSE(clock.Observe(1, 1'000'000 + 1'002'005'001ULL,
                               1'000'000 + 1'001'005'000ULL));
    EXPECT_EQ(clock.ignored_count(), 1u);
  }

  // 2. One hour of steady 200 Hz streaming with bridge 1000 ppm fast (every
  // frame slower than the first by increasing drift): every valid frame is
  // accepted, and the anchor refreshes periodically so a corrupt +1.5 s jump
  // after an hour is still rejected rather than accepted inside a huge drift
  // cone.
  {
    Estimator clock;
    constexpr std::uint64_t kSteps = 720'000;  // 1 hour at 5 ms
    for (std::uint64_t i = 0; i <= kSteps; ++i) {
      ASSERT_TRUE(
          clock.Observe(2, 1'000'000 + 5'000 * i, 10'000'000 + 5'005 * i));
    }
    const std::uint64_t last_sample = 1'000'000 + 5'000 * kSteps;
    const std::uint64_t last_receipt = 10'000'000 + 5'005 * kSteps;
    EXPECT_FALSE(
        clock.Observe(2, last_sample + 1'500'000, last_receipt + 5'005));
    EXPECT_TRUE(clock.Observe(2, last_sample + 5'000, last_receipt + 5'005));
    EXPECT_EQ(clock.ignored_count(), 1u);
  }
}

// Guards RT-29 against SIGSTOP / startup stall + SO_RCVBUF tail-drop on an
// unseen boot (e.g. a mid-boot bridge restart): draining stale buffered frames
// leaves the initial anchor behind live time, so the first live frame is held
// as a jump candidate and the second live frame (5 ms later, consistent at
// physical clock rate) immediately re-syncs the anchor and resumes acceptance.
// REQ-RT-20
TEST(ClockOffset, UnseenBootStallTailDropRecoversOnTwoConsistentLiveFrames) {
  {
    Estimator clock;
    // Unseen boot: bridge stalled 3 s while SO_RCVBUF buffered 10.000 s ..
    // 11.000 s and tail-dropped 11.005 s .. 12.995 s.
    ASSERT_TRUE(clock.Observe(9, 10'000'000, 13'000'000));
    for (std::uint64_t i = 1; i <= Estimator::kWindowSamples; ++i) {
      ASSERT_TRUE(clock.Observe(9, 10'000'000 + 5'000 * i, 13'001'000));
    }
    ASSERT_TRUE(clock.converged());
    const std::int64_t stale_offset = clock.offset_us();
    ASSERT_EQ(stale_offset, 2'001'000);

    // Live frame 1 (2.005 s ahead of the stale anchor): held as jump candidate
    // without disturbing the window or offset.
    EXPECT_FALSE(clock.Observe(9, 13'005'000, 13'005'000));
    EXPECT_EQ(clock.ignored_count(), 1u);
    EXPECT_EQ(clock.offset_us(), stale_offset);

    // A reordered stale duplicate does not disturb the pending candidate.
    EXPECT_FALSE(clock.Observe(9, 10'995'000, 13'006'000));
    EXPECT_EQ(clock.ignored_count(), 2u);

    // Live frame 2 (5 ms later, consistent with live frame 1 at physical clock
    // rate): re-syncs the anchor and is accepted immediately, snapping
    // offset_us_ to 0 while stale delays age out of spread_us_.
    EXPECT_TRUE(clock.Observe(9, 13'010'000, 13'010'000));
    EXPECT_EQ(clock.ignored_count(), 2u);
    EXPECT_TRUE(clock.converged());
    EXPECT_EQ(clock.offset_us(), 0);
    EXPECT_GT(clock.spread_us(), 2'000'000u);

    // After a full window of live frames, stale buffered delays age out cleanly
    // and a single corrupt jump is still rejected without locking out live
    // time.
    for (std::uint64_t i = 1; i <= Estimator::kWindowSamples; ++i) {
      ASSERT_TRUE(
          clock.Observe(9, 13'010'000 + 5'000 * i, 13'010'000 + 5'000 * i));
    }
    EXPECT_EQ(clock.offset_us(), 0);
    EXPECT_EQ(clock.spread_us(), 0u);
    const std::uint64_t live = 13'010'000 + 5'000 * Estimator::kWindowSamples;
    EXPECT_FALSE(clock.Observe(9, live + 1'000'001, live));
    EXPECT_TRUE(clock.Observe(9, live + 5'000, live + 5'000));
    EXPECT_EQ(clock.ignored_count(), 3u);
  }

  // A backward bridge receipt must not confirm a pending jump candidate.
  {
    Estimator clock;
    ASSERT_TRUE(clock.Observe(10, 10'000'000, 13'000'000));
    ASSERT_TRUE(clock.Observe(10, 11'000'000, 13'001'000));
    EXPECT_FALSE(clock.Observe(10, 13'005'000, 13'005'000));
    EXPECT_FALSE(clock.Observe(10, 13'010'000, 13'004'000));
    EXPECT_TRUE(clock.Observe(10, 13'015'000, 13'015'000));
    EXPECT_EQ(clock.ignored_count(), 2u);
  }
}

// Guards RT-29 boundary cases: exact 1 s jitter margin, burst catch-up after
// delayed buffered frames (including UDP tail-drop after a multi-second pause),
// long outages with up to 1000 ppm relative oscillator drift in either
// direction, backward bridge clock without underflow, and arbitrary starting
// sample_time_us on a new boot.
// REQ-RT-20
TEST(ClockOffset, JumpGuardAcceptsOutagesJitterAndNewBootWhileEnforcingBound) {
  // 1. Exact boundary at receipt_delta == 0 and burst jitter catch-up.
  {
    Estimator clock;
    // Frame 0 delayed by 400 ms; Frame 1 arrives promptly in the same Tick.
    ASSERT_TRUE(clock.Observe(1, 1'000'000, 2'400'000));
    EXPECT_TRUE(clock.Observe(1, 1'400'000, 2'400'000));
    // Exact 1 s margin above the prompt anchor is accepted; 1 us past is not.
    EXPECT_FALSE(clock.Observe(1, 2'400'001, 2'400'000));
    EXPECT_TRUE(clock.Observe(1, 2'400'000, 2'400'000));
    EXPECT_EQ(clock.ignored_count(), 1u);
  }

  // 2. Multi-second pause with 200 buffered stale frames followed by a live
  // frame (UDP tail-drop): delayed frames must not pull the anchor back and
  // lock out the live frame behind them.
  {
    Estimator clock;
    ASSERT_TRUE(clock.Observe(2, 1'000'000, 1'000'000));
    // 200 delayed frames drained at t = 3.5 s carrying s = 1.005 s .. 2.000 s.
    for (std::uint64_t i = 1; i <= Estimator::kWindowSamples; ++i) {
      ASSERT_TRUE(clock.Observe(2, 1'000'000 + 5'000 * i, 3'500'000));
    }
    // Live frame arrives at t = 3.505 s with s = 3.505 s (+1.505 s ahead of
    // the last buffered frame, but matching the pre-pause anchor).
    EXPECT_TRUE(clock.Observe(2, 3'505'000, 3'505'000));
    // Once back at live time, 1 us past the 1 s margin is still rejected.
    EXPECT_FALSE(clock.Observe(2, 3'505'000 + 1'000'001, 3'505'000));
    EXPECT_EQ(clock.ignored_count(), 1u);
  }

  // 3. Long link outages (10 s and 1 hour) with 1000 ppm relative oscillator
  // drift in either direction, plus exact bound check at 10 s (drift = 20 ms).
  {
    Estimator clock;
    ASSERT_TRUE(clock.Observe(3, 1'000'000, 10'000'000));
    // 10 s outage: max allowed = 10 s + 20 ms (2000 ppm) + 1 s = 11.020 s.
    EXPECT_FALSE(clock.Observe(3, 1'000'000 + 11'020'001, 20'000'000));
    // Gateway 1000 ppm fast across 10 s (+10 ms): accepted.
    EXPECT_TRUE(clock.Observe(3, 1'000'000 + 10'010'000, 20'000'000));
    // 1-hour outage with gateway 1000 ppm fast (+3.6 s): accepted.
    EXPECT_TRUE(clock.Observe(3, 11'010'000 + 3'603'600'000ULL,
                              20'000'000 + 3'600'000'000ULL));
    // 1-hour outage with bridge 1000 ppm fast (-3.6 s): accepted without
    // retreating the anchor on one frame (a far-future jump outside the 1-hour
    // drift cone is still rejected immediately, and once 1 s of live frames
    // refreshes the anchor a +1.5 s jump is rejected again).
    const std::uint64_t s2 = 11'010'000 + 3'603'600'000ULL + 3'596'400'000ULL;
    const std::uint64_t r2 = 20'000'000 + 7'200'000'000ULL;
    EXPECT_TRUE(clock.Observe(3, s2, r2));
    EXPECT_FALSE(clock.Observe(3, s2 + 15'000'000, r2 + 5'000));
    constexpr std::uint64_t kRefreshSteps = 201;
    for (std::uint64_t i = 1; i <= kRefreshSteps; ++i) {
      ASSERT_TRUE(clock.Observe(3, s2 + 5'000 * i, r2 + 5'005 * i));
    }
    const std::uint64_t s3 = s2 + 5'000 * kRefreshSteps;
    const std::uint64_t r3 = r2 + 5'005 * kRefreshSteps;
    EXPECT_FALSE(clock.Observe(3, s3 + 1'500'000, r3 + 5'005));
    EXPECT_EQ(clock.ignored_count(), 3u);
  }

  // 4. Backward bridge receipt_us must not underflow into a huge allowance,
  // and a new boot resets the baseline for arbitrary starting sample_time_us.
  {
    Estimator clock;
    ASSERT_TRUE(clock.Observe(4, 5'000, 10'000'000));
    EXPECT_FALSE(clock.Observe(4, 5'000 + 1'000'001, 1'000'000));
    EXPECT_TRUE(clock.Observe(4, 10'000, 1'000'000));
    // New boot may jump far higher or far lower than the previous boot.
    EXPECT_TRUE(clock.Observe(5, 900'000'000'000ULL, 10'005'000));
    EXPECT_TRUE(clock.Observe(6, 0, 10'010'000));
    EXPECT_FALSE(clock.Observe(6, 1'000'001, 10'010'000));
    EXPECT_TRUE(clock.Observe(6, 5'000, 10'015'000));
  }
}

}  // namespace
}  // namespace talos::hardware
