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
    next_exchange_ = 0;
    exchange_count_ = 0;
  } else if (count_ > 0 && sample_time_us <= last_sample_time_us_) {
    ++ignored_;
    return false;
  }
  last_sample_time_us_ = sample_time_us;
  SelectExchange(receipt_us, sample_time_us);

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
  if (rtt > kExchangeLifetimeUs) return;
  exchanges_[next_exchange_] = {
      .offset_us = static_cast<std::int64_t>(t1 + rtt / 2) -
                   static_cast<std::int64_t>(t2),
      .round_trip_us = rtt,
      .sample_us = t2,
      .receipt_us = t4,
  };
  next_exchange_ = (next_exchange_ + 1) % kExchangeSlots;
  exchange_count_ = std::min(exchange_count_ + 1, kExchangeSlots);
  SelectExchange(t4, std::max(t2, last_sample_time_us_));
}

void ClockOffsetEstimator::SelectExchange(std::uint64_t receipt_us,
                                          std::uint64_t sample_time_us) {
  // Every kept exchange is a causal bound until it is a second old, so the
  // minimum-RTT one among them is used; the next best is already held when
  // it expires.
  const Exchange* best = nullptr;
  std::uint64_t best_age_us = 0;
  for (std::size_t i = 0; i < exchange_count_; ++i) {
    const Exchange& exchange = exchanges_[i];
    if (receipt_us < exchange.receipt_us ||
        sample_time_us < exchange.sample_us) {
      continue;
    }
    const auto age_us = std::max(receipt_us - exchange.receipt_us,
                                 sample_time_us - exchange.sample_us);
    if (age_us > kExchangeLifetimeUs) continue;
    if (best == nullptr || exchange.round_trip_us < best->round_trip_us ||
        (exchange.round_trip_us == best->round_trip_us &&
         age_us < best_age_us)) {
      best = &exchange;
      best_age_us = age_us;
    }
  }
  exchange_valid_ = best != nullptr;
  if (best == nullptr) return;
  bounded_offset_us_ = best->offset_us;
  round_trip_us_ = best->round_trip_us;
  // Assumed maximum relative oscillator drift: 1000 ppm. Include integer
  // timestamp quantization, and age since the selected exchange.
  uncertainty_us_ =
      (round_trip_us_ + 1) / 2 + (round_trip_us_ + best_age_us) / 999 + 2;
}

}  // namespace talos::hardware
