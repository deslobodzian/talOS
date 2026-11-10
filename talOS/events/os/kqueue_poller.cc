#include "talOS/events/os/kqueue_poller.h"

#include <fcntl.h>
#include <mach/mach_time.h>
#include <sys/event.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <ctime>
#include <stdexcept>
#include <system_error>

namespace talos::event {

namespace {

// The deadline is a one-shot EVFILT_TIMER armed at an absolute time, and
// kevent() itself waits with no timeout (the same shape as the Linux
// backend's TFD_TIMER_ABSTIME timerfd under epoll_wait(-1)). A relative
// kevent() timeout is not used because macOS timer coalescing pads it with a
// leeway that grows with the timeout, even on a THREAD_TIME_CONSTRAINT_POLICY
// thread, so every timer would wake late by about a fifth of its wait.
// NOTE_CRITICAL asks for no coalescing.
//
// NOTE_MACHTIME | NOTE_ABSOLUTE | NOTE_MACH_CONTINUOUS_TIME: `data` is an
// absolute mach_continuous_time() value, the clock CLOCK_MONOTONIC_RAW (and
// so now()) reads on macOS. Without NOTE_MACH_CONTINUOUS_TIME it would be
// mach_absolute_time(), which stops during system sleep and so drifts from
// now() by the total time slept.
constexpr std::uint32_t kDeadlineTimerFlags =
    NOTE_CRITICAL | NOTE_MACHTIME | NOTE_ABSOLUTE | NOTE_MACH_CONTINUOUS_TIME;

// Converts a now() reading to mach continuous-time ticks, rounding up so the
// timer never fires before `nanos` as now() would read it. A deadline at or
// before the clock's epoch maps to 0, which is in the past and fires at once.
std::int64_t ToMachTicks(std::int64_t nanos) {
  static const mach_timebase_info_data_t timebase = [] {
    mach_timebase_info_data_t info{};
    if (::mach_timebase_info(&info) != KERN_SUCCESS || info.numer == 0 ||
        info.denom == 0) {
      throw std::runtime_error("mach_timebase_info failed");
    }
    return info;
  }();
  if (nanos <= 0) return 0;
  const std::int64_t numer = timebase.numer;
  const std::int64_t denom = timebase.denom;
  // ticks = ceil(nanos * denom / numer), split so the product cannot
  // overflow for any int64 deadline (Apple silicon is 125/3, Intel 1/1).
  return nanos / numer * denom + ((nanos % numer) * denom + numer - 1) / numer;
}

}  // namespace

KqueuePoller::KqueuePoller() : kq_fd_(::kqueue()) {
  if (kq_fd_ == -1) {
    throw std::system_error(errno, std::generic_category(), "kqueue");
  }

  // kqueue() has no CLOEXEC-on-create form (there is no kqueue1 on macOS),
  // so set it explicitly to keep the fd from leaking across fork+exec.
  if (::fcntl(kq_fd_, F_SETFD, FD_CLOEXEC) == -1) {
    const int saved_errno = errno;
    ::close(kq_fd_);
    throw std::system_error(saved_errno, std::generic_category(),
                            "fcntl F_SETFD");
  }

  // Register the wake event once, up front. EV_CLEAR makes EVFILT_USER
  // edge-triggered: once kevent() reports it as ready, the kernel resets it
  // to not-ready on its own. That is exactly the "wake() calls collapse
  // into one wakeup, and don't persist into a later wait" contract, with no
  // draining required (contrast the eventfd counter the Linux backend has
  // to drain by hand).
  struct kevent kev;
  EV_SET(&kev, kWakeIdent, EVFILT_USER, EV_ADD | EV_CLEAR, 0, 0, nullptr);
  if (::kevent(kq_fd_, &kev, 1, nullptr, 0, nullptr) == -1) {
    const int saved_errno = errno;
    ::close(kq_fd_);
    throw std::system_error(saved_errno, std::generic_category(),
                            "kevent EV_ADD wake");
  }
}

KqueuePoller::~KqueuePoller() { close_fd(); }

void KqueuePoller::close_fd() noexcept {
  if (kq_fd_ != -1) {
    ::close(kq_fd_);
    kq_fd_ = -1;
  }
}

KqueuePoller::KqueuePoller(KqueuePoller&& other) noexcept
    : kq_fd_(other.kq_fd_) {
  other.kq_fd_ = -1;
}

KqueuePoller& KqueuePoller::operator=(KqueuePoller&& other) noexcept {
  if (this != &other) {
    close_fd();
    kq_fd_ = other.kq_fd_;
    other.kq_fd_ = -1;
  }
  return *this;
}

MonotonicTime KqueuePoller::now() {
  // CLOCK_MONOTONIC_RAW is not subject to NTP frequency slewing. A
  // deterministic control loop must not have its measured intervals nudged
  // by clock discipline. On macOS it reads mach_continuous_time(), the clock
  // wait_until() arms its deadline timer in (kDeadlineTimerFlags).
  struct timespec ts;
  ::clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  return MonotonicTime::from_nanos(
      static_cast<std::int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec);
}

void KqueuePoller::wake() {
  struct kevent kev;
  EV_SET(&kev, kWakeIdent, EVFILT_USER, 0, NOTE_TRIGGER, 0, nullptr);
  if (::kevent(kq_fd_, &kev, 1, nullptr, 0, nullptr) == -1) {
    throw std::system_error(errno, std::generic_category(),
                            "kevent NOTE_TRIGGER");
  }
}

void KqueuePoller::add_fd(int fd) {
  struct kevent kev;
  EV_SET(&kev, fd, EVFILT_READ, EV_ADD, 0, 0, nullptr);
  if (::kevent(kq_fd_, &kev, 1, nullptr, 0, nullptr) == -1) {
    throw std::system_error(errno, std::generic_category(), "kevent EV_ADD fd");
  }
}

void KqueuePoller::remove_fd(int fd) {
  struct kevent kev;
  EV_SET(&kev, fd, EVFILT_READ, EV_DELETE, 0, 0, nullptr);
  // Errors are ignored here rather than thrown: ENOENT just means `fd` was
  // never watched (the documented no-op case), and any other failure (say,
  // the fd was already closed) leaves nothing for the caller to act on --
  // removal is best-effort cleanup, not a resource acquisition.
  ::kevent(kq_fd_, &kev, 1, nullptr, 0, nullptr);
}

KqueuePoller::WakeReason KqueuePoller::wait_until(MonotonicTime deadline) {
  struct kevent events[kMaxEvents];

  // One fixed ident, EV_ADD | EV_ONESHOT on every finite wait, carried as the
  // changelist of the waiting kevent() call itself (one syscall per wait).
  // No timer event from an earlier deadline can be reported as this one's:
  //  - a delivered EV_ONESHOT timer deletes its own knote;
  //  - a timer that fired but was not delivered (the previous call returned
  //    READABLE or WOKEN first) is replaced by this EV_ADD, which re-arms the
  //    existing knote and discards its pending fire;
  //  - an infinite wait deletes the knote, pending fire included.
  struct kevent timer;
  int nchanges = 0;
  if (deadline == MonotonicTime::max()) {
    EV_SET(&timer, kDeadlineIdent, EVFILT_TIMER, EV_DELETE, 0, 0, nullptr);
    // A separate call: deleting a knote that is already gone (a delivered
    // one-shot) fails with ENOENT, which in the waiting call would come back
    // as an EV_ERROR event and end the wait at once. ENOENT is the expected
    // case and nothing else leaves a timer armed, so errors are ignored.
    ::kevent(kq_fd_, &timer, 1, nullptr, 0, nullptr);
  } else {
    // A deadline already past arms a timer that fires at once, so the wait
    // still returns promptly (after collecting any readable fd or wake).
    EV_SET(&timer, kDeadlineIdent, EVFILT_TIMER, EV_ADD | EV_ONESHOT,
           kDeadlineTimerFlags, ToMachTicks(deadline.nanos()), nullptr);
    nchanges = 1;
  }

  int n;
  for (;;) {
    n = ::kevent(kq_fd_, &timer, nchanges, events, kMaxEvents, nullptr);
    if (n != -1 || errno != EINTR) break;
    // EINTR is retried, not surfaced: wait_until()'s contract is to return
    // only for one of the three documented reasons, and a signal landing
    // mid-wait is not one of them. Re-applying the same absolute EV_ADD on
    // the retry is harmless: the deadline does not move.
  }

  if (n == -1) {
    throw std::system_error(errno, std::generic_category(), "kevent");
  }

  // A single kevent() call can report both a readable fd and the wake event
  // together. READABLE outranks WOKEN here for the same reason it outranks
  // a passed deadline: it carries data the caller must not miss a chance to
  // observe, whereas a deferred WOKEN or DEADLINE simply surfaces on the
  // next call with no information lost (EV_CLEAR already reset the wake
  // event's state regardless of whether we act on it now). Otherwise the
  // event is the EVFILT_TIMER: DEADLINE.
  bool woken = false;
  for (int i = 0; i < n; ++i) {
    if ((events[i].flags & EV_ERROR) != 0) {
      // Only the timer change can fail here: a finite deadline is the only
      // wait that passes a changelist (the infinite wait's EV_DELETE is a
      // separate call). Returning DEADLINE for it would spin the caller's
      // loop without ever waiting.
      throw std::system_error(static_cast<int>(events[i].data),
                              std::generic_category(),
                              "kevent EV_ADD deadline timer");
    }
    if (events[i].filter == EVFILT_READ) {
      return WakeReason::READABLE;
    }
    if (events[i].filter == EVFILT_USER) {
      woken = true;
    }
  }
  return woken ? WakeReason::WOKEN : WakeReason::DEADLINE;
}

}  // namespace talos::event
