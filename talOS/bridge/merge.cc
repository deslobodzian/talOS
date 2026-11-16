#include "talOS/bridge/merge.h"

#include <algorithm>

namespace talos::hardware {

std::uint64_t OwnerTimeoutUs(std::uint64_t period_us,
                             std::uint64_t command_timeout_us) {
  return std::min(std::max(2 * period_us, command_timeout_us / 2),
                  command_timeout_us);
}

bool MergeOwner::Admits(std::uint64_t observed_time_us,
                        std::uint64_t latest_sample_time_us,
                        std::uint64_t command_timeout_us) const {
  // Both stamps are gateway clock, never differenced against the merging
  // side's clock. A request computed from sample N stays admissible while
  // newer samples arrive, until its age reaches the command timeout; the
  // gateway re-enforces the same bound against its own clock.
  if (observed_time_us > latest_sample_time_us) return false;
  if (latest_sample_time_us - observed_time_us >= command_timeout_us) {
    return false;
  }
  // Equal re-publishes from the same sample are fine.
  return observed_time_us >= last_observed_time_us;
}

void MergeOwner::Accept(std::uint64_t now_us, std::uint64_t observed_time_us) {
  seen = true;
  last_seen_us = now_us;
  last_observed_time_us = observed_time_us;
  timed_out = false;
}

bool MergeOwner::Expire(std::uint64_t now_us) {
  if (!seen || now_us - last_seen_us > timeout_us) timed_out = true;
  return timed_out;
}

std::optional<std::uint64_t> MergeOwner::ExpiresAtUs() const {
  // Expire()'s test, solved for now_us: the first instant past timeout_us.
  if (timed_out || !seen) return std::nullopt;
  return last_seen_us + timeout_us + 1;
}

void MergeOwner::Reset() {
  seen = false;
  last_seen_us = 0;
  last_observed_time_us = 0;
  timed_out = true;
}

}  // namespace talos::hardware
