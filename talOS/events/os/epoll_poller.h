#pragma once

#include <sys/epoll.h>

#include <ctime>

#include "talOS/events/time.h"

namespace talos::event {

// epoll readiness backend for Linux.
//
// This is a concrete, non-virtual class: poller.h aliases it as `Poller`
// directly rather than dispatching through an interface, so wait_until()
// never pays for a vtable indirection on the control loop's hot path.
//
// wait_until() waits with epoll_pwait2(), whose timeout is a timespec, given
// the time left to the deadline (CLOCK_MONOTONIC) right before the call.
// epoll_wait()'s millisecond timeout is too coarse for the 1kHz loops this
// backs. The wait is a sleeper hrtimer, which PREEMPT_RT expires in hard
// interrupt context for a realtime thread, so the woken loop then runs at
// its own tier; a timerfd's hrtimer expires there in softirq context, in
// ktimers/N at SCHED_FIFO 1, behind whatever loop holds that CPU, an
// observer included. A kernel before 5.11 (ENOSYS), or a seccomp filter that
// refuses the call (EPERM), makes the poller fall back, for its lifetime, to
// a timerfd armed with the absolute deadline (TFD_TIMER_ABSTIME) under
// epoll_wait(-1): the same nanosecond deadlines, with that softirq expiry.
class EpollPoller {
 public:
  enum class WakeReason { DEADLINE, READABLE, WOKEN };

  // The epoll_pwait2() call wait_until() makes, without the signal mask
  // (always null). A null Pwait2 waits on the timerfd.
  using Pwait2 = int (*)(int epoll_fd, epoll_event* events, int max_events,
                         const timespec* timeout);

  // glibc's epoll_pwait2() (2.35 and later), else the raw syscall when the
  // kernel headers number it, else null.
  static Pwait2 system_pwait2();

  // Throws std::system_error if any of the epoll, timerfd or eventfd fds
  // cannot be created, or the internal fds cannot be registered.
  EpollPoller() : EpollPoller(system_pwait2()) {}
  // Waits with `pwait2` instead of the system's; tests pass a stand-in that
  // reports ENOSYS to exercise the fallback.
  explicit EpollPoller(Pwait2 pwait2);
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
  // returns DEADLINE immediately without blocking. Never returns DEADLINE
  // before `deadline`.
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

  // Whether the last wait_until() reported `fd` readable, for a caller
  // watching several fds that must tell which one returned READABLE.
  bool reported_readable(int fd) const noexcept;

  // Whether wait_until() waits with epoll_pwait2() rather than the timerfd:
  // false once a call has reported it unsupported.
  bool waits_with_pwait2() const { return pwait2_ != nullptr; }

  // Monotonic clock reading on the same timeline wait_until()'s deadline is
  // measured against (CLOCK_MONOTONIC, the clock epoll_pwait2()'s timeout
  // and the timerfd run on).
  static MonotonicTime now();

 private:
  // Upper bound on events drained per wait. A control loop watches a handful
  // of fds at most, plus our own timer and wake fds; if this is ever
  // exceeded, the remaining events simply surface on the next wait_until()
  // call instead of being lost.
  static constexpr int kMaxEvents = 16;

  // Waits for `deadline` or an event and returns the number of `events`
  // filled (0 when the deadline passed with none).
  int wait_for_events(MonotonicTime deadline, epoll_event* events);
  void arm_timer(MonotonicTime deadline);
  void register_internal_fd(int fd);
  void close_all() noexcept;

  Pwait2 pwait2_{nullptr};
  // The watched fds the last wait_until() reported readable.
  int readable_[kMaxEvents]{};
  int readable_count_{0};
  int epoll_fd_{-1};
  int timer_fd_{-1};
  int wake_fd_{-1};
};

}  // namespace talos::event
