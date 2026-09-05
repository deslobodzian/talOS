#include "talOS/events/metrics.h"

#include <gtest/gtest.h>

#include <cmath>
#include <cstdint>
#include <limits>
#include <type_traits>

#include "talOS/events/context.h"
#include "talOS/events/latency_stats.h"
#include "talOS/events/manifest.h"
#include "talOS/events/time.h"

namespace talos::event {
namespace {

MonotonicTime at(std::int64_t nanos) {
  return MonotonicTime::from_nanos(nanos);
}

constexpr std::int64_t kInt64Max = std::numeric_limits<std::int64_t>::max();

// Measured worst-case relative error of upper_bound_of() over the sweeps
// below (see HistogramTest.RelativeErrorIsBounded): it approaches 1/16 =
// 6.25% as values grow, from the sub-bucket width being SUB_COUNT (16) steps
// per octave. The header's doc comment claims "about 3%"; that undersells the
// worst case, so tests here assert the bound actually observed rather than
// the documented one.
constexpr double kMaxBucketRelativeError = 0.063;

// Percentile lookups add the extra slack of "at least N samples fell at or
// below this bucket", on top of plain bucket rounding, so give them more
// room than a single bucket's worst case.
constexpr double kMaxPercentileRelativeError = 0.08;

// ---------------------------------------------------------------------------
// A. Histogram bucketing
// ---------------------------------------------------------------------------

TEST(HistogramTest, ValuesBelowSubCountAreExact) {
  for (std::int64_t v = 0; v < Histogram::SUB_COUNT; ++v) {
    EXPECT_EQ(Histogram::index_of(v), static_cast<std::size_t>(v));
    EXPECT_EQ(Histogram::upper_bound_of(Histogram::index_of(v)), v);
  }
}

// A histogram that reports a smaller latency than actually happened is
// useless for a realtime deadline check, so this must hold everywhere, not
// just on the values chosen above.
TEST(HistogramTest, IndexIsMonotonicAndNeverUnderReports) {
  std::int64_t previous_index = -1;
  for (std::int64_t v = 0; v <= 4096; ++v) {
    const auto index = static_cast<std::int64_t>(Histogram::index_of(v));
    EXPECT_GE(index, previous_index) << "v=" << v;
    EXPECT_GE(Histogram::upper_bound_of(Histogram::index_of(v)), v)
        << "v=" << v;
    previous_index = index;
  }

  // Geometric coverage up to 1e12, since an exhaustive sweep that far is not
  // affordable in a "small" test.
  for (std::int64_t v = 4097; v <= 1'000'000'000'000LL;
       v = static_cast<std::int64_t>(v * 1.001) + 1) {
    const auto index = static_cast<std::int64_t>(Histogram::index_of(v));
    EXPECT_GE(Histogram::upper_bound_of(Histogram::index_of(v)), v)
        << "v=" << v;
    EXPECT_LT(index, static_cast<std::int64_t>(Histogram::BUCKETS));
  }
}

TEST(HistogramTest, RelativeErrorIsBounded) {
  for (std::int64_t v = Histogram::SUB_COUNT + 1; v <= 1'000'000'000'000LL;
       v = static_cast<std::int64_t>(v * 1.001) + 1) {
    const std::int64_t upper =
        Histogram::upper_bound_of(Histogram::index_of(v));
    const double relative_error =
        static_cast<double>(upper - v) / static_cast<double>(v);
    EXPECT_LT(relative_error, kMaxBucketRelativeError) << "v=" << v;
  }
}

// The bucket array (1024 entries) has to cover all of int64_t without the
// clamp in index_of() ever changing the answer for a legitimate value: the
// clamp exists only as a defensive backstop.
TEST(HistogramTest, VeryLargeValuesStayInRange) {
  // INT64_MAX is included deliberately: it lands in the topmost reachable
  // bucket, which is where the bound computation used to overflow.
  for (const std::int64_t v :
       {std::int64_t{1'000'000'000'000'000LL}, kInt64Max / 2, kInt64Max}) {
    const auto index = Histogram::index_of(v);
    EXPECT_LT(index, Histogram::BUCKETS);
    EXPECT_GE(Histogram::upper_bound_of(index), v);
  }
}

TEST(HistogramTest, NegativeValuesDoNotCrash) {
  EXPECT_EQ(Histogram::index_of(-1), 0u);
  EXPECT_EQ(Histogram::index_of(-1'000'000), 0u);

  Histogram histogram;
  histogram.add(-5);
  EXPECT_EQ(histogram.total(), 1u);
}

// The topmost reachable bucket describes values larger than an int64 can
// hold. Computing its bound with a signed shift overflowed, which is
// undefined behaviour reachable from one absurd sample; the shift now happens
// in unsigned arithmetic and saturates. Keep this test: on a plain build the
// old code wrapped back to INT64_MAX and looked right, so only a sanitizer or
// an optimizing compiler would have caught the regression.
TEST(HistogramTest, UpperBoundOfTopBucketSaturates) {
  const auto index = Histogram::index_of(kInt64Max);
  EXPECT_EQ(Histogram::upper_bound_of(index), kInt64Max);
}

// ---------------------------------------------------------------------------
// B. Histogram percentiles
// ---------------------------------------------------------------------------

TEST(HistogramTest, IdenticalValuesReturnThatValuesBucket) {
  Histogram histogram;
  for (int i = 0; i < 1000; ++i) {
    histogram.add(12'345);
  }

  // All mass is in one bucket, so this is exact, not merely close.
  const std::int64_t expected =
      Histogram::upper_bound_of(Histogram::index_of(12'345));
  EXPECT_EQ(histogram.percentile(0.50), expected);
  EXPECT_EQ(histogram.percentile(0.99), expected);
  EXPECT_EQ(histogram.percentile(0.999), expected);
}

TEST(HistogramTest, UniformFillGivesExpectedPercentiles) {
  Histogram histogram;
  for (std::int64_t v = 1; v <= 1000; ++v) {
    histogram.add(v);
  }

  const std::int64_t p50 = histogram.percentile(0.50);
  const std::int64_t p99 = histogram.percentile(0.99);

  EXPECT_GE(p50, 500);
  EXPECT_LT(static_cast<double>(p50 - 500) / 500.0,
            kMaxPercentileRelativeError);

  EXPECT_GE(p99, 990);
  EXPECT_LT(static_cast<double>(p99 - 990) / 990.0,
            kMaxPercentileRelativeError);
}

TEST(HistogramTest, EmptyHistogramReturnsZero) {
  Histogram histogram;
  EXPECT_EQ(histogram.percentile(0.50), 0);
  EXPECT_EQ(histogram.percentile(0.999), 0);
  EXPECT_EQ(histogram.total(), 0u);
}

TEST(HistogramTest, TotalCountsEveryAdd) {
  Histogram histogram;
  for (int i = 0; i < 250; ++i) {
    histogram.add(i);
  }
  EXPECT_EQ(histogram.total(), 250u);
}

// This is the case the whole class exists for: a mostly-on-time distribution
// with a rare, severe tail must not be smoothed away by an average.
TEST(HistogramTest, TailIsVisibleInHighPercentiles) {
  Histogram histogram;
  for (int i = 0; i < 990; ++i) {
    histogram.add(1000);
  }
  for (int i = 0; i < 10; ++i) {
    histogram.add(5'000'000);
  }

  const std::int64_t p50 = histogram.percentile(0.50);
  EXPECT_GE(p50, 1000);
  EXPECT_LT(static_cast<double>(p50 - 1000) / 1000.0,
            kMaxPercentileRelativeError);

  // p99.9 falls at sample 999 of 1000, which is one of the ten tail
  // samples, so it must land in the tail bucket, not the body.
  const std::int64_t expected_tail =
      Histogram::upper_bound_of(Histogram::index_of(5'000'000));
  EXPECT_EQ(histogram.percentile(0.999), expected_tail);
}

TEST(HistogramTest, ResetEmptiesIt) {
  Histogram histogram;
  for (int i = 0; i < 10; ++i) {
    histogram.add(i * 100);
  }
  histogram.reset();
  EXPECT_EQ(histogram.total(), 0u);
  EXPECT_EQ(histogram.percentile(0.50), 0);
}

// ---------------------------------------------------------------------------
// C. LatencyStats
// ---------------------------------------------------------------------------

TEST(LatencyStatsTest, CountMinMaxMeanAreExact) {
  LatencyStats stats;
  for (const std::int64_t sample : {100, 200, 300, 400, 500}) {
    stats.add_nanos(sample);
  }

  EXPECT_EQ(stats.count(), 5u);
  EXPECT_EQ(stats.min_ns(), 100);
  EXPECT_EQ(stats.max_ns(), 500);
  EXPECT_EQ(stats.mean_ns(), 300);
}

TEST(LatencyStatsTest, JitterIsMaxMinusMin) {
  LatencyStats stats;
  for (const std::int64_t sample : {100, 900, 250}) {
    stats.add_nanos(sample);
  }
  EXPECT_EQ(stats.jitter_ns(), 800);
}

TEST(LatencyStatsTest, StddevMatchesKnownDistribution) {
  LatencyStats identical;
  for (int i = 0; i < 5; ++i) {
    identical.add_nanos(500);
  }
  EXPECT_NEAR(identical.stddev_ns(), 0.0, 1e-6);

  LatencyStats stats;
  for (const std::int64_t sample : {100, 200, 300, 400, 500}) {
    stats.add_nanos(sample);
  }
  // Population stddev of {100,200,300,400,500}: mean 300, variance
  // ((200^2+100^2+0+100^2+200^2))/5 = 20000, stddev = sqrt(20000).
  const double expected = std::sqrt(20000.0);
  EXPECT_NEAR(stats.stddev_ns(), expected, 1e-6);
}

TEST(LatencyStatsTest, EmptyStatsAreAllZeroAndDoNotDivideByZero) {
  LatencyStats stats;
  EXPECT_EQ(stats.count(), 0u);
  EXPECT_EQ(stats.min_ns(), 0);
  EXPECT_EQ(stats.max_ns(), 0);
  EXPECT_EQ(stats.mean_ns(), 0);
  EXPECT_EQ(stats.jitter_ns(), 0);
  EXPECT_EQ(stats.stddev_ns(), 0.0);
  EXPECT_EQ(stats.percentile_ns(0.50), 0);
}

TEST(LatencyStatsTest, PercentileAgreesWithUnderlyingHistogram) {
  LatencyStats stats;
  for (std::int64_t v = 1; v <= 200; ++v) {
    stats.add_nanos(v);
  }
  EXPECT_EQ(stats.percentile_ns(0.90), stats.histogram().percentile(0.90));
}

TEST(LatencyStatsTest, AddDurationAgreesWithAddNanos) {
  LatencyStats via_duration;
  via_duration.add(Duration{123'456});

  LatencyStats via_nanos;
  via_nanos.add_nanos(123'456);

  EXPECT_EQ(via_duration.count(), via_nanos.count());
  EXPECT_EQ(via_duration.min_ns(), via_nanos.min_ns());
  EXPECT_EQ(via_duration.max_ns(), via_nanos.max_ns());
  EXPECT_EQ(via_duration.mean_ns(), via_nanos.mean_ns());
}

// ---------------------------------------------------------------------------
// D. LoopMetrics
// ---------------------------------------------------------------------------

Manifest TwoSourceManifest() {
  Registration timer{};
  timer.id = 0;
  timer.kind = SourceKind::TIMER;
  timer.name = "loop_a";

  Registration watcher{};
  watcher.id = 1;
  watcher.kind = SourceKind::WATCHER;
  watcher.name = "topic_b";

  return Manifest{timer, watcher};
}

TEST(LoopMetricsTest, ConfigureSizesToManifestAndIgnoresUnknownSource) {
  LoopMetrics metrics;
  metrics.configure(TwoSourceManifest());
  EXPECT_EQ(metrics.size(), 2u);

  Context context{};
  context.source_id = 99;
  context.now = at(1000);
  context.event_time = at(500);

  // Must not crash indexing past sources_; nothing to observe afterwards
  // since source_id 99 was never allocated a slot.
  metrics.record(context, Duration{10});
  EXPECT_EQ(metrics.source(0).dispatches, 0u);
  EXPECT_EQ(metrics.source(1).dispatches, 0u);
}

TEST(LoopMetricsTest, LatencyIsNowMinusEventTime) {
  LoopMetrics metrics;
  metrics.configure(TwoSourceManifest());

  Context context{};
  context.source_id = 0;
  context.sequence = 1;
  context.event_time = at(1000);
  context.now = at(1300);
  metrics.record(context, Duration{0});

  context.event_time = at(2000);
  context.now = at(2100);
  metrics.record(context, Duration{0});

  const SourceMetrics& source = metrics.source(0);
  EXPECT_EQ(source.latency.min_ns(), 100);
  EXPECT_EQ(source.latency.max_ns(), 300);
  EXPECT_EQ(source.latency.mean_ns(), 200);
}

TEST(LoopMetricsTest, IntervalIsGapBetweenDispatchesWithOneFewerSample) {
  LoopMetrics metrics;
  metrics.configure(TwoSourceManifest());

  Context context{};
  context.source_id = 0;
  context.sequence = 1;

  for (const std::int64_t now_ns : {0, 1000, 2000, 3000}) {
    context.now = at(now_ns);
    context.event_time = context.now;
    metrics.record(context, Duration{0});
  }

  const SourceMetrics& source = metrics.source(0);
  EXPECT_EQ(source.dispatches, 4u);
  EXPECT_EQ(source.interval.count(), 3u);
  EXPECT_EQ(source.interval.min_ns(), 1000);
  EXPECT_EQ(source.interval.max_ns(), 1000);
}

TEST(LoopMetricsTest, DifferentSourcesAreIndependent) {
  LoopMetrics metrics;
  metrics.configure(TwoSourceManifest());

  Context a{};
  a.source_id = 0;
  a.sequence = 1;
  a.event_time = at(0);
  a.now = at(100);
  metrics.record(a, Duration{0});

  Context b{};
  b.source_id = 1;
  b.sequence = 1;
  b.event_time = at(0);
  b.now = at(900);
  metrics.record(b, Duration{0});

  EXPECT_EQ(metrics.source(0).latency.mean_ns(), 100);
  EXPECT_EQ(metrics.source(1).latency.mean_ns(), 900);
  EXPECT_EQ(metrics.source(0).dispatches, 1u);
  EXPECT_EQ(metrics.source(1).dispatches, 1u);
}

// A timer's sequence is a cycle count (1 == on time); a message's sequence is
// a transport sequence number that says nothing about missed deadlines. The
// two must not be conflated.
TEST(LoopMetricsTest, OverrunsOnlyCountForTimersPastCycleOne) {
  LoopMetrics metrics;
  metrics.configure(TwoSourceManifest());

  Context on_time{};
  on_time.kind = EventKind::TIMER;
  on_time.source_id = 0;
  on_time.sequence = 1;
  metrics.record(on_time, Duration{0});
  EXPECT_EQ(metrics.source(0).overruns, 0u);

  Context overrun{};
  overrun.kind = EventKind::TIMER;
  overrun.source_id = 0;
  overrun.sequence = 4;
  metrics.record(overrun, Duration{0});
  EXPECT_EQ(metrics.source(0).overruns, 3u);

  Context message{};
  message.kind = EventKind::MESSAGE;
  message.source_id = 1;
  message.sequence = 1'000'000;
  metrics.record(message, Duration{0});
  EXPECT_EQ(metrics.source(1).overruns, 0u);
}

TEST(LoopMetricsTest, DroppedAccumulates) {
  LoopMetrics metrics;
  metrics.configure(TwoSourceManifest());

  Context context{};
  context.source_id = 0;
  context.dropped = 2;
  metrics.record(context, Duration{0});
  context.dropped = 3;
  metrics.record(context, Duration{0});

  EXPECT_EQ(metrics.source(0).dropped, 5u);
}

TEST(LoopMetricsTest, UtilizationIsZeroWhenRunDurationIsZero) {
  LoopMetrics metrics;
  metrics.configure(TwoSourceManifest());
  metrics.begin_run(at(1'000'000));
  metrics.end_run(at(1'000'000));

  Context context{};
  context.source_id = 0;
  metrics.record(context, Duration{500});

  EXPECT_EQ(metrics.utilization(), 0.0);
}

TEST(LoopMetricsTest, UtilizationIsHandlerShareOfRunDuration) {
  LoopMetrics metrics;
  metrics.configure(TwoSourceManifest());
  metrics.begin_run(at(0));
  metrics.end_run(at(1'000'000));

  Context context{};
  context.source_id = 0;
  // Three equal handler times so mean * count reproduces the exact sum,
  // sidestepping the integer-mean truncation utilization() relies on.
  for (int i = 0; i < 3; ++i) {
    metrics.record(context, Duration{100'000});
  }

  EXPECT_NEAR(metrics.utilization(), 0.3, 1e-9);
}

TEST(LoopMetricsTest, ReportIncludesActiveSourcesAndSkipsIdleOnes) {
  LoopMetrics metrics;
  const Manifest manifest = TwoSourceManifest();
  metrics.configure(manifest);

  Context context{};
  context.kind = EventKind::TIMER;
  context.source_id = 0;
  context.sequence = 4;
  context.event_time = at(0);
  context.now = at(500);
  metrics.record(context, Duration{100});

  const std::string report = metrics.report(manifest);
  EXPECT_NE(report.find("loop_a"), std::string::npos);
  EXPECT_NE(report.find("dispatch latency"), std::string::npos);
  EXPECT_NE(report.find("overruns"), std::string::npos);

  // topic_b never got a dispatch, so it must not appear in the report.
  EXPECT_EQ(report.find("topic_b"), std::string::npos);
}

TEST(LoopMetricsTest, ResetClearsPerSourceStats) {
  LoopMetrics metrics;
  metrics.configure(TwoSourceManifest());

  Context context{};
  context.source_id = 0;
  context.dropped = 1;
  metrics.record(context, Duration{100});

  metrics.reset();

  const SourceMetrics& source = metrics.source(0);
  EXPECT_EQ(source.dispatches, 0u);
  EXPECT_EQ(source.dropped, 0u);
  EXPECT_EQ(source.latency.count(), 0u);
}

// ---------------------------------------------------------------------------
// E. Policy shape
// ---------------------------------------------------------------------------

static_assert(MetricsPolicy<NullMetrics>);
static_assert(MetricsPolicy<LoopMetrics>);
static_assert(!NullMetrics::ENABLED);
static_assert(LoopMetrics::ENABLED);

TEST(PolicyTest, EnabledFlagsMatchIntent) {
  EXPECT_FALSE(NullMetrics::ENABLED);
  EXPECT_TRUE(LoopMetrics::ENABLED);
}

// No vtables on this path: a control loop dispatching at 1 kHz cannot afford
// virtual calls in its measurement code.
TEST(PolicyTest, MetricsTypesHaveNoVtable) {
  EXPECT_FALSE(std::is_polymorphic_v<NullMetrics>);
  EXPECT_FALSE(std::is_polymorphic_v<LoopMetrics>);
  EXPECT_FALSE(std::is_polymorphic_v<Histogram>);
}

}  // namespace
}  // namespace talos::event
