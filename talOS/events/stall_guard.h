#pragma once

#include <cstdint>

namespace talos::event {

// Outcome of one failed wake-arm (or idle-pass) check against StallGuard.
enum class StallAction : std::uint8_t {
  // Below the threshold: retry/drain immediately instead of sleeping.
  kRetry,
  // Reached the threshold for the first time since the last reset: count the
  // stall once in stalled_arms and sleep instead of spinning.
  kTripped,
  // Already tripped since the last reset: keep sleeping without recounting.
  kStalled,
};

// Guards a wake-on-publish loop against spinning when a reader's arm keeps
// reporting unread data that no read returns (for example, a slot the writer
// keeps tearing). Allows up to `max_stalled` consecutive failed arms to retry
// immediately; the next one transitions once to tripped (kTripped) so the
// caller counts it in stalled_arms and sleeps, and further failures stay
// stalled (kStalled) until reset() on a clean arm or read progress.
class StallGuard {
 public:
  static constexpr std::uint32_t kDefaultMaxStalled = 3;

  constexpr explicit StallGuard(
      std::uint32_t max_stalled = kDefaultMaxStalled) noexcept
      : max_stalled_{max_stalled} {}

  constexpr StallAction on_failed_arm() noexcept {
    if (tripped_) {
      return StallAction::kStalled;
    }
    if (stalls_ < max_stalled_) {
      ++stalls_;
      return StallAction::kRetry;
    }
    tripped_ = true;
    return StallAction::kTripped;
  }

  constexpr void reset() noexcept {
    stalls_ = 0;
    tripped_ = false;
  }

  constexpr bool stalled() const noexcept { return tripped_; }
  constexpr std::uint32_t max_stalled() const noexcept { return max_stalled_; }

 private:
  std::uint32_t max_stalled_{kDefaultMaxStalled};
  std::uint32_t stalls_{0};
  bool tripped_{false};
};

}  // namespace talos::event
