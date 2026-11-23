#include "talOS/hardware/clock_offset.h"

#include <algorithm>
#include <limits>

namespace talos::hardware {
namespace {

std::uint64_t SaturatingAdd(std::uint64_t a, std::uint64_t b) {
  return b > std::numeric_limits<std::uint64_t>::max() - a
             ? std::numeric_limits<std::uint64_t>::max()
             : a + b;
}

bool WithinClockRateBound(std::uint64_t delta_a, std::uint64_t delta_b) {
  const std::uint64_t drift_us =
      delta_b / ClockOffsetEstimator::kMaxClockDriftDivisor;
  const std::uint64_t max_delta = SaturatingAdd(
      delta_b,
      SaturatingAdd(drift_us, ClockOffsetEstimator::kMaxSampleJumpMarginUs));
  return delta_a <= max_delta;
}

}  // namespace

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
    anchor_sample_us_ = sample_time_us;
    anchor_receipt_us_ = receipt_us;
    next_anchor_count_ = 0;
    has_jump_candidate_ = false;
  } else if (count_ > 0) {
    if (sample_time_us <= last_sample_time_us_) {
      ++ignored_;
      return false;
    }
    // anchor_sample_us_ is always an earlier accepted sample of this boot, so
    // anchor_sample_us_ <= last_sample_time_us_ < sample_time_us.
    const std::uint64_t sample_delta_us = sample_time_us - anchor_sample_us_;
    const std::uint64_t receipt_delta_us =
        receipt_us > anchor_receipt_us_ ? receipt_us - anchor_receipt_us_ : 0;
    if (!WithinClockRateBound(sample_delta_us, receipt_delta_us)) {
      // An isolated far-future sample (corrupt timestamp, UINT64_MAX) is held
      // as a candidate and ignored. Only the next sample past the bound
      // advancing strictly past the candidate with non-decreasing receipt time
      // at a symmetrically consistent physical clock rate (e.g. live frames
      // after an unseen-boot SIGSTOP + SO_RCVBUF tail-drop) re-syncs the
      // anchor; a duplicate or reordered sample returned early above without
      // touching the candidate, while any normal accepted sample or boot
      // change clears it.
      const bool confirms_candidate =
          has_jump_candidate_ && sample_time_us > jump_sample_us_ &&
          receipt_us >= jump_receipt_us_ &&
          WithinClockRateBound(sample_time_us - jump_sample_us_,
                               receipt_us - jump_receipt_us_) &&
          WithinClockRateBound(receipt_us - jump_receipt_us_,
                               sample_time_us - jump_sample_us_);
      if (!confirms_candidate) {
        has_jump_candidate_ = true;
        jump_sample_us_ = sample_time_us;
        jump_receipt_us_ = receipt_us;
        ++ignored_;
        return false;
      }
      anchor_sample_us_ = sample_time_us;
      anchor_receipt_us_ = receipt_us;
      next_anchor_count_ = 0;
      has_jump_candidate_ = false;
    } else {
      has_jump_candidate_ = false;
      // Advance the anchor immediately only when this sample is at least as
      // prompt as it (never letting a delayed packet pull the anchor back).
      // Under sustained bridge-fast drift (or after an unusually prompt early
      // sample), track the window's promptest accepted sample and promote it
      // once at least kConvergedSamples frames span kAnchorRefreshUs of bridge
      // receipt time so the drift allowance stays bounded over long runs.
      if (sample_delta_us >= receipt_delta_us) {
        anchor_sample_us_ = sample_time_us;
        anchor_receipt_us_ = receipt_us;
        next_anchor_count_ = 0;
      } else {
        if (next_anchor_count_ == 0) {
          next_anchor_sample_us_ = sample_time_us;
          next_anchor_receipt_us_ = receipt_us;
          next_anchor_since_us_ = receipt_us;
          next_anchor_count_ = 1;
        } else {
          ++next_anchor_count_;
          const std::uint64_t next_sample_delta_us =
              sample_time_us - next_anchor_sample_us_;
          const std::uint64_t next_receipt_delta_us =
              receipt_us > next_anchor_receipt_us_
                  ? receipt_us - next_anchor_receipt_us_
                  : 0;
          if (next_sample_delta_us >= next_receipt_delta_us) {
            next_anchor_sample_us_ = sample_time_us;
            next_anchor_receipt_us_ = receipt_us;
          }
        }
        if (next_anchor_count_ >= kConvergedSamples &&
            receipt_us > next_anchor_since_us_ &&
            receipt_us - next_anchor_since_us_ >= kAnchorRefreshUs) {
          anchor_sample_us_ = next_anchor_sample_us_;
          anchor_receipt_us_ = next_anchor_receipt_us_;
          next_anchor_count_ = 0;
        }
      }
    }
  }
  last_sample_time_us_ = sample_time_us;
  SelectExchange(receipt_us, sample_time_us);

  delays_us_[next_] = static_cast<std::int64_t>(receipt_us - sample_time_us);
  next_ = (next_ + 1) % kWindowSamples;
  count_ = std::min(count_ + 1, kWindowSamples);

  const auto [lowest, highest] =
      std::minmax_element(delays_us_.begin(), delays_us_.begin() + count_);
  offset_us_ = *lowest;
  spread_us_ = static_cast<std::uint64_t>(*highest) -
               static_cast<std::uint64_t>(*lowest);
  return true;
}

void ClockOffsetEstimator::ObserveExchange(std::uint64_t boot_id,
                                           std::uint64_t t1, std::uint64_t t2,
                                           std::uint64_t t4) {
  if (!boots_.current(boot_id) || t1 == 0 || t4 < t1) return;
  const auto rtt = t4 - t1;
  if (rtt > kExchangeLifetimeUs) return;
  exchanges_[next_exchange_] = {
      .offset_us = static_cast<std::int64_t>((t1 + rtt / 2) - t2),
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
