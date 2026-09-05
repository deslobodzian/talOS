#pragma once

#include <cstdint>

#include "talOS/events/time.h"

namespace talos::event {

// kqueue-backed readiness backend for macOS.
//
// This is a concrete, non-virtual class: poller.h aliases it as `Poller`
// directly rather than dispatching through an interface, so wait_until()
// never pays for a vtable indirection on the control loop's hot path.
class KqueuePoller {
 public:
  enum class WakeReason { DEADLINE, READABLE, WOKEN };

  // Throws std::system_error if the kqueue fd cannot be created or the
  // internal wake event cannot be registered.
  KqueuePoller();
  ~KqueuePoller();

  KqueuePoller(const KqueuePoller&) = delete;
  KqueuePoller& operator=(const KqueuePoller&) = delete;

  // The moved-from poller holds no fd afterwards, so destroying it is a
  // no-op rather than a double close.
  KqueuePoller(KqueuePoller&& other) noexcept;
  KqueuePoller& operator=(KqueuePoller&& other) noexcept;

  // Blocks until `deadline` (on the timeline returned by now()), until a
  // watched fd becomes readable, or until wake() is called. A deadline of
  // MonotonicTime::max() blocks indefinitely; a deadline already in the past
  // returns DEADLINE immediately without blocking.
  WakeReason wait_until(MonotonicTime deadline);

  // Thread-safe. May be called from any thread to break a concurrent or
  // future wait_until(). Multiple calls before the next wait_until()
  // collapse into a single WOKEN wakeup (see kqueue_poller.cc for how
  // EV_CLEAR gives us this for free).
  void wake();

  // Watches `fd` for readability. Throws std::system_error on failure.
  void add_fd(int fd);
  // Stops watching `fd`. A no-op if `fd` was not being watched.
  void remove_fd(int fd);

  // Monotonic clock reading on the same timeline wait_until()'s deadline is
  // measured against.
  static MonotonicTime now();

 private:
  // Upper bound on events drained per kevent() call. A control loop watches
  // a handful of fds at most; if this is ever exceeded, the remaining
  // events simply surface on the next wait_until() call instead of being
  // lost.
  static constexpr int kMaxEvents = 16;

  // Identifier for the EVFILT_USER wake event registered in the
  // constructor. EVFILT_USER idents live in their own namespace (they are
  // not fds), so any fixed value works here.
  static constexpr std::uintptr_t kWakeIdent = 1;

  void close_fd() noexcept;

  int kq_fd_{-1};
};

}  // namespace talos::event
