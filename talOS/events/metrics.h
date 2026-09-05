#pragma once

#include <concepts>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "talOS/events/context.h"
#include "talOS/events/latency_stats.h"
#include "talOS/events/manifest.h"
#include "talOS/events/time.h"

namespace talos::event {

// Everything measured about one event source.
struct SourceMetrics {
  // How late the loop was to service the event: now - event_time. For a
  // periodic timer this is the wake-up delay, and its spread is the number a
  // control engineer means by jitter.
  LatencyStats latency;

  // Wall time between consecutive dispatches. For a 1 kHz timer this should
  // sit on 1 ms; the spread shows whether the loop is holding its rate.
  LatencyStats interval;

  // How long the handler itself ran. This is the budget: when it approaches
  // the period, the loop is about to start overrunning.
  LatencyStats handler_time;

  std::uint64_t dispatches{0};

  // Periods that elapsed without being serviced. Any non-zero value means a
  // deadline was missed.
  std::uint64_t overruns{0};

  // Messages the transport lapped before this source could read them.
  std::uint64_t dropped{0};

  MonotonicTime last_dispatch{};
  bool seen{false};
};

// A metrics policy observes every dispatch. Like the recorder it is a template
// parameter rather than an interface, so NullMetrics compiles away entirely
// and an uninstrumented loop pays nothing, not even the clock reads.
template <typename T>
concept MetricsPolicy =
    requires(T metrics, const Manifest& manifest, const Context& context,
             Duration duration, MonotonicTime time) {
      { T::ENABLED } -> std::convertible_to<bool>;
      { metrics.configure(manifest) } -> std::same_as<void>;
      { metrics.record(context, duration) } -> std::same_as<void>;
      { metrics.begin_run(time) } -> std::same_as<void>;
      { metrics.end_run(time) } -> std::same_as<void>;
    };

// The default: measures nothing, costs nothing.
struct NullMetrics {
  static constexpr bool ENABLED = false;

  void configure(const Manifest&) {}
  void record(const Context&, Duration) {}
  void begin_run(MonotonicTime) {}
  void end_run(MonotonicTime) {}
};

// Collects per-source latency, jitter and handler cost.
//
// Recording costs two steady-clock reads plus a few histogram increments per
// dispatch: measured at 40 ns on an M-series Mac at -O3, of which 26 ns is the
// clock reads themselves. That is 0.004% of a 1 ms cycle, and worth paying: a
// loop whose timing you cannot see is a loop you cannot trust.
class LoopMetrics {
 public:
  static constexpr bool ENABLED = true;

  void configure(const Manifest& manifest) { sources_.resize(manifest.size()); }

  void begin_run(MonotonicTime start) {
    start_ = start;
    end_ = start;
  }

  void end_run(MonotonicTime end) { end_ = end; }

  void record(const Context& context, Duration handler_time) {
    if (context.source_id >= sources_.size()) {
      return;
    }

    SourceMetrics& metrics = sources_[context.source_id];

    metrics.latency.add(context.now - context.event_time);
    metrics.handler_time.add(handler_time);

    if (metrics.seen) {
      metrics.interval.add(context.now - metrics.last_dispatch);
    }

    metrics.last_dispatch = context.now;
    metrics.seen = true;

    ++metrics.dispatches;
    metrics.dropped += context.dropped;

    // A timer reports how many periods elapsed; more than one means the loop
    // did not get there in time.
    if (context.kind == EventKind::TIMER && context.sequence > 1) {
      metrics.overruns += context.sequence - 1;
    }
  }

  const SourceMetrics& source(std::uint16_t id) const { return sources_[id]; }
  std::size_t size() const { return sources_.size(); }

  Duration run_duration() const { return end_ - start_; }

  // Fraction of wall time spent inside handlers. Approaching 1.0 means there
  // is no headroom left for anything else.
  double utilization() const {
    const std::int64_t elapsed = run_duration().count();
    if (elapsed <= 0) {
      return 0.0;
    }

    // The exact sum, not mean times count: integer division in the mean would
    // quietly under-report the busiest loops by up to a nanosecond per sample.
    std::int64_t busy = 0;
    for (const SourceMetrics& metrics : sources_) {
      busy += metrics.handler_time.sum_ns();
    }
    return static_cast<double>(busy) / static_cast<double>(elapsed);
  }

  void reset() {
    for (SourceMetrics& metrics : sources_) {
      metrics = SourceMetrics{};
    }
  }

  std::string report(const Manifest& manifest) const;

 private:
  std::vector<SourceMetrics> sources_;
  MonotonicTime start_{};
  MonotonicTime end_{};
};

static_assert(MetricsPolicy<NullMetrics>);
static_assert(MetricsPolicy<LoopMetrics>);

// Formats nanoseconds as microseconds, the unit robot timing lives in.
inline std::string format_micros(std::int64_t nanos) {
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.3f",
                static_cast<double>(nanos) / 1000.0);
  return std::string{buffer};
}

inline std::string LoopMetrics::report(const Manifest& manifest) const {
  std::string out;
  char line[256];

  const auto row = [&out, &line](const char* name, const LatencyStats& stats) {
    std::snprintf(line, sizeof(line),
                  "  %-24s %8llu %10s %10s %10s %10s %10s %10s\n", name,
                  static_cast<unsigned long long>(stats.count()),
                  format_micros(stats.mean_ns()).c_str(),
                  format_micros(stats.percentile_ns(0.50)).c_str(),
                  format_micros(stats.percentile_ns(0.99)).c_str(),
                  format_micros(stats.percentile_ns(0.999)).c_str(),
                  format_micros(stats.max_ns()).c_str(),
                  format_micros(stats.jitter_ns()).c_str());
    out += line;
  };

  std::snprintf(line, sizeof(line), "run duration: %s ms, utilization %.2f%%\n",
                format_micros(run_duration().count() / 1000).c_str(),
                utilization() * 100.0);
  out += line;

  std::snprintf(line, sizeof(line),
                "  %-24s %8s %10s %10s %10s %10s %10s %10s\n", "metric (us)",
                "count", "mean", "p50", "p99", "p99.9", "max", "jitter");
  out += line;

  for (std::size_t i = 0; i < sources_.size() && i < manifest.size(); ++i) {
    const SourceMetrics& metrics = sources_[i];
    if (metrics.dispatches == 0) {
      continue;
    }

    std::snprintf(line, sizeof(line), "\n[%u] %s %s\n",
                  static_cast<unsigned>(manifest[i].id),
                  to_string(manifest[i].kind), manifest[i].name.c_str());
    out += line;

    row("dispatch latency", metrics.latency);
    row("interval", metrics.interval);
    row("handler time", metrics.handler_time);

    if (metrics.overruns > 0 || metrics.dropped > 0) {
      std::snprintf(line, sizeof(line),
                    "  overruns: %llu, dropped messages: %llu\n",
                    static_cast<unsigned long long>(metrics.overruns),
                    static_cast<unsigned long long>(metrics.dropped));
      out += line;
    }
  }

  return out;
}

}  // namespace talos::event
