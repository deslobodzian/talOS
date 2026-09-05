#pragma once

#include "talOS/events/time.h"

namespace talos::event {

// epoll + timerfd readiness backend for Linux.
//
// This is a concrete, non-virtual class: poller.h aliases it as `Poller`
// directly rather than dispatching through an interface, so wait_until()
// never pays for a vtable indirection on the control loop's hot path.
//
// epoll_wait()'s own timeout is milliseconds only, which is too coarse for
// the 1kHz loops this backs -- a rounded-up millisecond is a real defect at
// that rate. A timerfd armed with an absolute CLOCK_MONOTONIC deadline
// (TFD_TIMER_ABSTIME) gives nanosecond-resolution deadlines instead, at the
// cost of epoll_wait() always being called with an infinite timeout and the
// timerfd doing the actual timing out.
class EpollPoller {
 public:
  enum class WakeReason { DEADLINE, READABLE, WOKEN };

  // Throws std::system_error if any of the epoll, timerfd or eventfd fds
  // cannot be created, or the internal fds cannot be registered.
  EpollPoller();
  ~EpollPoller();

  EpollPoller(const EpollPoller&) = delete;
  EpollPoller& operator=(const EpollPoller&) = delete;

  // The moved-from poller holds no fds afterwards, so destroying it is a
  // no-op rather than a double close.
  EpollPoller(EpollPoller&& other) noexcept;
  EpollPoller& operator=(EpollPoller&& other) noexcept;

  // Blocks until `deadline` (on the timeline returned by now()), until a
  // watched fd becomes readable, or until wake() is called. A deadline of
  // MonotonicTime::max() blocks indefinitely; a deadline already in the past
  // returns DEADLINE immediately without blocking.
  WakeReason wait_until(MonotonicTime deadline);

  // Thread-safe. May be called from any thread to break a concurrent or
  // future wait_until(). Multiple calls before the next wait_until()
  // collapse into a single WOKEN wakeup: they all increment the same
  // eventfd counter, which is drained back to zero in one read.
  void wake();

  // Watches `fd` for readability. Throws std::system_error on failure.
  void add_fd(int fd);
  // Stops watching `fd`. A no-op if `fd` was not being watched.
  void remove_fd(int fd);

  // Monotonic clock reading on the same timeline wait_until()'s deadline is
  // measured against (CLOCK_MONOTONIC, matching the timerfd's clock id).
  static MonotonicTime now();

 private:
  // Upper bound on events drained per epoll_wait() call. A control loop
  // watches a handful of fds at most, plus our own timer and wake fds; if
  // this is ever exceeded, the remaining events simply surface on the next
  // wait_until() call instead of being lost.
  static constexpr int kMaxEvents = 16;

  void arm_timer(MonotonicTime deadline);
  void register_internal_fd(int fd);
  void close_all() noexcept;

  int epoll_fd_{-1};
  int timer_fd_{-1};
  int wake_fd_{-1};
};

}  // namespace talos::event
