#include "talOS/bridge/clock_offset.h"

#include <algorithm>

namespace talos::hardware {

bool ClockOffsetEstimator::Observe(std::uint64_t boot_id,
                                   std::uint64_t sample_time_us,
                                   std::uint64_t receipt_us) {
  if (boots_.retired(boot_id)) {
    ++ignored_;
    return false;
  }
  if (!boots_.current(boot_id)) {
    if (!boots_.empty()) ++resets_;
    boots_.Enter(boot_id);
    next_ = 0;
    count_ = 0;
    offset_us_ = 0;
    spread_us_ = 0;
    exchange_valid_ = false;
    bounded_offset_us_ = 0;
    uncertainty_us_ = 0;
    round_trip_us_ = 0;
    exchange_time_us_ = 0;
    exchange_sample_us_ = 0;
  } else if (count_ > 0 && sample_time_us <= last_sample_time_us_) {
    ++ignored_;
    return false;
  }
  last_sample_time_us_ = sample_time_us;
  if (exchange_valid_) {
    const auto age =
        receipt_us >= exchange_time_us_ && sample_time_us >= exchange_sample_us_
            ? std::max(receipt_us - exchange_time_us_,
                       sample_time_us - exchange_sample_us_)
            : 1'000'001;
    exchange_valid_ = age <= 1'000'000;
    // Assumed maximum relative oscillator drift: 1000 ppm. Include integer
    // timestamp quantization, and age since the selected exchange.
    uncertainty_us_ =
        (round_trip_us_ + 1) / 2 + (round_trip_us_ + age) / 999 + 2;
  }

  delays_us_[next_] = static_cast<std::int64_t>(receipt_us) -
                      static_cast<std::int64_t>(sample_time_us);
  next_ = (next_ + 1) % kWindowSamples;
  count_ = std::min(count_ + 1, kWindowSamples);

  const auto [lowest, highest] =
      std::minmax_element(delays_us_.begin(), delays_us_.begin() + count_);
  offset_us_ = *lowest;
  spread_us_ = static_cast<std::uint64_t>(*highest - *lowest);
  return true;
}

void ClockOffsetEstimator::ObserveExchange(std::uint64_t boot_id,
                                           std::uint64_t t1, std::uint64_t t2,
                                           std::uint64_t t4) {
  if (!boots_.current(boot_id) || t1 == 0 || t4 < t1) return;
  const auto rtt = t4 - t1;
  if (rtt > 1'000'000) return;
  // Keep the minimum RTT for at most one second, not forever across drift.
  if (exchange_valid_ && t4 - exchange_time_us_ <= 1'000'000 &&
      rtt >= round_trip_us_) {
    return;
  }
  bounded_offset_us_ =
      static_cast<std::int64_t>(t1 + rtt / 2) - static_cast<std::int64_t>(t2);
  round_trip_us_ = rtt;
  exchange_sample_us_ = t2;
  exchange_time_us_ = t4;
  uncertainty_us_ = (rtt + 1) / 2 + rtt / 1000 + 2;
  exchange_valid_ = true;
}

}  // namespace talos::hardware
