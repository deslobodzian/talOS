#pragma once

#include <atomic>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <optional>
#include <utility>

#include "talOS/events/os/poller.h"
#include "talOS/events/stall_guard.h"
#include "talOS/rtms/wake.h"

namespace talos::event {

// What a wake-channel wait lost so far, safe to read from any thread:
// - stalled_arms: readers whose failed arms stopped holding the loop awake
//   (StallGuard::kDefaultMaxStalled consecutive failed arms with no read
//   progress);
// - wake_fallbacks: runs or opens that could not build or register their wake
//   receiver (or wait fd) and fell back to polling.
// Each should stay 0; a nonzero count means some reader waited for its
// backstop or the loop polled instead of waking on publication.
struct WakeLoss {
  std::uint64_t stalled_arms{0};
  std::uint64_t wake_fallbacks{0};
};

// Shared wake-channel wait helper composed by both RealtimeEventLoop and
// HardwareNode. Holds the optional WakeReceiver, its poller registration and
// polling-fallback accounting, per-reader wake arming with StallGuard, drain(),
// and atomic WakeLoss counters readable from another thread.
class WakeChannelWait {
 public:
  using MakeWakeReceiver = WakeReceiver (*)();

  WakeChannelWait() = default;
  WakeChannelWait(const WakeChannelWait&) = delete;
  WakeChannelWait& operator=(const WakeChannelWait&) = delete;
  ~WakeChannelWait() = default;

  // Initializes a WakeReceiver (through `make_wake_receiver` when non-null) and
  // registers its fd on `poller`. On failure, resets the receiver, records a
  // polling fallback (stderr warning + atomic wake_fallbacks) and returns
  // false.
  bool open(Poller& poller, const char* context, std::int64_t poll_us,
            MakeWakeReceiver make_wake_receiver = nullptr) {
    wake_.reset();
    try {
      if (make_wake_receiver != nullptr) {
        wake_.emplace(make_wake_receiver());
      } else {
        wake_.emplace();
      }
      poller.add_fd(wake_->fd());
      return true;
    } catch (const std::exception& error) {
      wake_.reset();
      record_fallback(context, error.what(), poll_us);
      return false;
    }
  }

  // Records a fallback to polling detected outside open() (for example, when
  // Poller construction or registering a non-wake fd fails). Increments
  // wake_fallbacks and logs one warning line to stderr.
  void record_fallback(const char* context, const char* reason,
                       std::int64_t poll_us) noexcept {
    wake_fallbacks_.fetch_add(1, std::memory_order_relaxed);
    std::fprintf(stderr, "%s (%s); polling every %lld us instead\n", context,
                 reason, static_cast<long long>(poll_us));
  }

  void close() noexcept { wake_.reset(); }

  bool is_open() const noexcept { return wake_.has_value(); }
  explicit operator bool() const noexcept { return wake_.has_value(); }

  // The receiver's id (0 when closed) and fd (-1 when closed).
  std::uint64_t id() const noexcept { return wake_ ? wake_->id() : 0; }
  int fd() const noexcept { return wake_ ? wake_->fd() : -1; }

  // Whether `poller`'s last wait reported this wake receiver readable.
  bool reported_readable(const Poller& poller) const noexcept {
    return wake_ && poller.reported_readable(wake_->fd());
  }

  // Consumes all pending notifications on the wake receiver, if open.
  void drain() noexcept {
    if (wake_) wake_->drain();
  }

  // Arms one reader through `arm_fn()` (returning true when caught up and
  // false when unread data is reported) under its per-reader `guard`.
  // - Clean arm: resets `guard` and returns false (safe to sleep).
  // - Failed arm within the guard's threshold (kRetry): returns true (drain or
  //   retry immediately without sleeping).
  // - Failed arm reaching the threshold (kTripped): increments stalled_arms
  //   once and returns false (sleep instead of spinning).
  // - Already tripped (kStalled): returns false without recounting.
  template <typename ArmFn>
  bool arm_reader(StallGuard& guard, ArmFn&& arm_fn) {
    if (std::forward<ArmFn>(arm_fn)()) {
      guard.reset();
      return false;
    }
    switch (guard.on_failed_arm()) {
      case StallAction::kRetry:
        return true;
      case StallAction::kTripped:
        stalled_arms_.fetch_add(1, std::memory_order_relaxed);
        return false;
      case StallAction::kStalled:
        return false;
    }
    return false;
  }

  // Thread-safe snapshot of wake loss counters.
  WakeLoss loss() const noexcept {
    return WakeLoss{
        .stalled_arms = stalled_arms_.load(std::memory_order_relaxed),
        .wake_fallbacks = wake_fallbacks_.load(std::memory_order_relaxed),
    };
  }

 private:
  std::optional<WakeReceiver> wake_;
  std::atomic<std::uint64_t> stalled_arms_{0};
  std::atomic<std::uint64_t> wake_fallbacks_{0};
};

}  // namespace talos::event
