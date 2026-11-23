#pragma once

// Host qualification and latency distribution helpers for timed perf and
// integration tests (docs/TESTING.md §6.7, §6.8; docs/METHODOLOGY.md §10).

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <stop_token>
#include <thread>
#include <vector>

namespace talos::process {

// Measures host scheduling noise over a timed test: the longest a 1 ms sleep
// of an ordinary thread overran, for a timed test to read before it judges
// (docs/TESTING.md §6.7), so a stalled host fails as [HOST_UNQUALIFIED].
class StallMeter {
 public:
  StallMeter()
      : thread_{[this](const std::stop_token& stop) {
          do {
            const auto before = std::chrono::steady_clock::now();
            std::this_thread::sleep_for(std::chrono::milliseconds{1});
            const auto over = std::chrono::steady_clock::now() - before -
                              std::chrono::milliseconds{1};
            const auto over_us =
                std::chrono::duration_cast<std::chrono::microseconds>(over)
                    .count();
            worst_us_.store(std::max<std::int64_t>(worst_us_.load(), over_us));
            samples_.fetch_add(1, std::memory_order_relaxed);
          } while (!stop.stop_requested());
        }} {}

  // Stops measuring and returns the worst 1 ms sleep overrun in microseconds.
  std::uint64_t Stop() {
    thread_.request_stop();
    if (thread_.joinable()) thread_.join();
    return static_cast<std::uint64_t>(
        std::max<std::int64_t>(worst_us_.load(), 0));
  }

  std::uint64_t samples() const {
    return samples_.load(std::memory_order_relaxed);
  }

 private:
  std::atomic<std::int64_t> worst_us_{0};
  std::atomic<std::uint64_t> samples_{0};
  std::jthread thread_;
};

struct DurationPercentiles {
  double p50_us = 0.0;
  double p95_us = 0.0;
  double p99_us = 0.0;
  double max_us = 0.0;
};

// Computes nearest-rank p50, p95, p99 and max (in microseconds) over a
// non-empty vector of nanosecond durations.
inline DurationPercentiles ComputePercentiles(
    std::vector<std::int64_t> samples_ns) {
  if (samples_ns.empty()) {
    throw std::invalid_argument(
        "ComputePercentiles: samples must be non-empty");
  }
  std::sort(samples_ns.begin(), samples_ns.end());
  const auto at_percentile = [&](double pct) -> double {
    const double rank =
        std::ceil((pct / 100.0) * static_cast<double>(samples_ns.size()));
    const std::size_t idx =
        rank <= 1.0 ? 0
                    : std::min<std::size_t>(samples_ns.size() - 1,
                                            static_cast<std::size_t>(rank) - 1);
    return static_cast<double>(samples_ns[idx]) / 1000.0;
  };
  return DurationPercentiles{
      .p50_us = at_percentile(50.0),
      .p95_us = at_percentile(95.0),
      .p99_us = at_percentile(99.0),
      .max_us = static_cast<double>(samples_ns.back()) / 1000.0,
  };
}

}  // namespace talos::process
