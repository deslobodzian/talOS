#pragma once

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>

#include "talOS/events/time.h"

namespace talos::event {

// A fixed-size latency histogram.
//
// Averages hide exactly what matters on a robot. A control loop that is on
// time 99% of the time and 8 ms late for the rest has a fine mean and a real
// problem, so this keeps the whole distribution and answers percentile
// queries.
//
// Buckets are logarithmic with 16 linear sub-buckets per octave, the
// HdrHistogram arrangement: exact below 16 ns, 6.25% worst-case relative error
// above it (one part in SUB_COUNT, at the bottom of each octave, falling to
// 3.1% at the top), and the whole nanosecond-to-century range in 4 KiB.
// Recording is a shift, a count-leading-zeros and an increment, with no
// allocation and no data-dependent branching, so it is cheap enough to leave
// on in a 1 kHz loop.
//
// Exact minimum, maximum and mean are kept separately by LatencyStats, so the
// bucket error only ever affects percentiles.
class Histogram {
 public:
  static constexpr int SUB_BITS = 4;
  static constexpr std::int64_t SUB_COUNT = 1 << SUB_BITS;
  static constexpr std::size_t BUCKETS = 1024;

  void add(std::int64_t value) { ++counts_[index_of(value)]; }

  void reset() { counts_.fill(0); }

  std::uint64_t total() const {
    std::uint64_t sum = 0;
    for (const std::uint32_t count : counts_) {
      sum += count;
    }
    return sum;
  }

  // The smallest recorded value at or below which `fraction` of samples fall.
  // Returns the bucket's upper bound, so a reported latency is never
  // optimistic.
  std::int64_t percentile(double fraction) const {
    const std::uint64_t samples = total();
    if (samples == 0) {
      return 0;
    }

    const auto target = static_cast<std::uint64_t>(
        std::ceil(fraction * static_cast<double>(samples)));

    std::uint64_t seen = 0;
    for (std::size_t i = 0; i < BUCKETS; ++i) {
      seen += counts_[i];
      if (seen >= std::max<std::uint64_t>(target, 1)) {
        return upper_bound_of(i);
      }
    }
    return upper_bound_of(BUCKETS - 1);
  }

  // Maps a value to its bucket. Values below SUB_COUNT are stored exactly;
  // above that each octave is split into SUB_COUNT even steps.
  static std::size_t index_of(std::int64_t value) {
    if (value < 0) {
      return 0;
    }
    if (value < SUB_COUNT) {
      return static_cast<std::size_t>(value);
    }

    const int octave = 63 - std::countl_zero(static_cast<std::uint64_t>(value));
    const int shift = octave - SUB_BITS;
    const std::int64_t sub = (value >> shift) - SUB_COUNT;

    const auto index =
        static_cast<std::size_t>((octave - SUB_BITS + 1) * SUB_COUNT + sub);

    return std::min(index, BUCKETS - 1);
  }

  // Largest value that falls in this bucket.
  static std::int64_t upper_bound_of(std::size_t index) {
    if (index < static_cast<std::size_t>(SUB_COUNT)) {
      return static_cast<std::int64_t>(index);
    }

    const auto group = static_cast<std::int64_t>(index) / SUB_COUNT;
    const auto sub = static_cast<std::int64_t>(index) % SUB_COUNT;
    const std::int64_t shift = group - 1;

    // The topmost buckets describe values that do not fit in an int64 at all,
    // so the shift is done in unsigned arithmetic and saturates. Letting it
    // overflow a signed shift would be undefined behaviour, reachable from a
    // single absurd sample.
    const auto base = static_cast<std::uint64_t>(sub + SUB_COUNT + 1);
    constexpr auto LIMIT =
        static_cast<std::uint64_t>(std::numeric_limits<std::int64_t>::max());

    if (shift >= 63 || base > (LIMIT >> shift)) {
      return std::numeric_limits<std::int64_t>::max();
    }

    return static_cast<std::int64_t>(base << shift) - 1;
  }

 private:
  std::array<std::uint32_t, BUCKETS> counts_{};
};

// Latency distribution plus the exact figures a histogram cannot give back:
// the true minimum, maximum and mean.
class LatencyStats {
 public:
  void add(Duration sample) { add_nanos(sample.count()); }

  void add_nanos(std::int64_t nanos) {
    if (count_ == 0) {
      min_ = nanos;
      max_ = nanos;
    } else {
      min_ = std::min(min_, nanos);
      max_ = std::max(max_, nanos);
    }

    ++count_;
    sum_ += nanos;

    const auto value = static_cast<double>(nanos);
    sum_squares_ += value * value;

    histogram_.add(nanos);
  }

  void reset() { *this = LatencyStats{}; }

  std::uint64_t count() const { return count_; }
  std::int64_t sum_ns() const { return sum_; }
  std::int64_t min_ns() const { return count_ == 0 ? 0 : min_; }
  std::int64_t max_ns() const { return count_ == 0 ? 0 : max_; }

  std::int64_t mean_ns() const {
    return count_ == 0 ? 0 : sum_ / static_cast<std::int64_t>(count_);
  }

  double stddev_ns() const {
    if (count_ < 2) {
      return 0.0;
    }
    const auto n = static_cast<double>(count_);
    const auto mean = static_cast<double>(sum_) / n;
    const double variance = std::max(0.0, (sum_squares_ / n) - (mean * mean));
    return std::sqrt(variance);
  }

  std::int64_t percentile_ns(double fraction) const {
    return count_ == 0 ? 0 : histogram_.percentile(fraction);
  }

  // Peak-to-peak spread. The blunt definition of jitter, and the one that
  // matters when asking whether a deadline was ever missed.
  std::int64_t jitter_ns() const { return count_ == 0 ? 0 : max_ - min_; }

  const Histogram& histogram() const { return histogram_; }

 private:
  Histogram histogram_;
  std::uint64_t count_{0};
  std::int64_t sum_{0};
  std::int64_t min_{0};
  std::int64_t max_{0};
  double sum_squares_{0.0};
};

}  // namespace talos::event
