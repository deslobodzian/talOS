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
  } else if (count_ > 0 && sample_time_us <= last_sample_time_us_) {
    ++ignored_;
    return false;
  }
  last_sample_time_us_ = sample_time_us;

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

}  // namespace talos::hardware
