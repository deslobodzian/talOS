#include "talOS/events/os/kqueue_poller.h"

#include <fcntl.h>
#include <mach/mach_time.h>
#include <sys/event.h>
#include <sys/time.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <ctime>
#include <stdexcept>
#include <system_error>

namespace talos::event {

namespace {

// The deadline is a one-shot EVFILT_TIMER armed at an absolute time, and
// kevent() also waits with a relative timeout of the time left to it. The
// timer is what wakes the loop on time: macOS timer coalescing pads a
// relative kevent() timeout with a leeway that grows with the timeout, even
// on a THREAD_TIME_CONSTRAINT_POLICY thread, so it wakes late by about a
// fifth of its wait. NOTE_CRITICAL asks for no coalescing. The timeout is the
// backstop: XNU delivers every EVFILT_TIMER through one shared "high"
// thread_call worker (priority 93, below time-constraint threads at 97), so
// while that worker does not run, every kqueue timer on the machine is late
// together, without bound; the timeout expires the waiting thread's own wait
// from the timer interrupt instead, so a stalled delivery costs at most the
// timeout's leeway.
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

KqueuePoller::KqueuePoller(DeadlineTimer deadline_timer)
    : arm_deadline_timer_(deadline_timer == DeadlineTimer::ARMED),
      kq_fd_(::kqueue()) {
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
    : arm_deadline_timer_(other.arm_deadline_timer_), kq_fd_(other.kq_fd_) {
  other.kq_fd_ = -1;
}

KqueuePoller& KqueuePoller::operator=(KqueuePoller&& other) noexcept {
  if (this != &other) {
    close_fd();
    arm_deadline_timer_ = other.arm_deadline_timer_;
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

bool KqueuePoller::reported_readable(int fd) const noexcept {
  return std::find(readable_, readable_ + readable_count_, fd) !=
         readable_ + readable_count_;
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
  struct kevent timer {};
  int nchanges = 0;
  const bool finite = deadline != MonotonicTime::max();
  if (!finite) {
    EV_SET(&timer, kDeadlineIdent, EVFILT_TIMER, EV_DELETE, 0, 0, nullptr);
    // A separate call: deleting a knote that is already gone (a delivered
    // one-shot) fails with ENOENT, which in the waiting call would come back
    // as an EV_ERROR event and end the wait at once. ENOENT is the expected
    // case and nothing else leaves a timer armed, so errors are ignored.
    ::kevent(kq_fd_, &timer, 1, nullptr, 0, nullptr);
  } else if (arm_deadline_timer_) {
    // A deadline already past arms a timer that fires at once, so the wait
    // still returns promptly (after collecting any readable fd or wake).
    EV_SET(&timer, kDeadlineIdent, EVFILT_TIMER, EV_ADD | EV_ONESHOT,
           kDeadlineTimerFlags, ToMachTicks(deadline.nanos()), nullptr);
    nchanges = 1;
  }

  int n;
  for (;;) {
    // The backstop timeout, recomputed on every pass so a retry waits only
    // for what is left. A deadline already past polls (a zero timeout).
    struct timespec timeout {};
    if (finite) {
      const std::int64_t left =
          std::max<std::int64_t>(0, (deadline - now()).count());
      timeout.tv_sec = static_cast<time_t>(left / 1'000'000'000);
      timeout.tv_nsec = static_cast<long>(left % 1'000'000'000);
    }
    n = ::kevent(kq_fd_, &timer, nchanges, events, kMaxEvents,
                 finite ? &timeout : nullptr);
    if (n == -1) {
      // EINTR is retried, not surfaced: wait_until()'s contract is to return
      // only for one of the three documented reasons, and a signal landing
      // mid-wait is not one of them. Re-applying the same absolute EV_ADD on
      // the retry is harmless: the deadline does not move.
      if (errno == EINTR) continue;
      throw std::system_error(errno, std::generic_category(), "kevent");
    }
    // The timeout runs on mach_absolute_time() and is converted to it by
    // truncation, so it can expire a fraction of a tick before now() reaches
    // the deadline; wait out the rest rather than report DEADLINE early.
    if (n > 0 || !finite || now() >= deadline) break;
  }

  // A single kevent() call can report both a readable fd and the wake event
  // together. READABLE outranks WOKEN here for the same reason it outranks
  // a passed deadline: it carries data the caller must not miss a chance to
  // observe, whereas a deferred WOKEN or DEADLINE simply surfaces on the
  // next call with no information lost (EV_CLEAR already reset the wake
  // event's state regardless of whether we act on it now). Otherwise the
  // event is the EVFILT_TIMER, or there is none because the timeout expired:
  // DEADLINE.
  bool woken = false;
  bool timer_failed = false;
  std::intptr_t timer_error = 0;
  readable_count_ = 0;
  for (int i = 0; i < n; ++i) {
    if ((events[i].flags & EV_ERROR) != 0) {
      // Only the timer change can fail here: a finite deadline is the only
      // wait that passes a changelist (the infinite wait's EV_DELETE is a
      // separate call).
      timer_failed = true;
      timer_error = events[i].data;
    } else if (events[i].filter == EVFILT_READ) {
      readable_[readable_count_++] = static_cast<int>(events[i].ident);
    } else if (events[i].filter == EVFILT_USER) {
      woken = true;
    }
  }
  if (readable_count_ > 0) return WakeReason::READABLE;
  if (timer_failed) {
    // Returning DEADLINE for it would spin the caller's loop without ever
    // waiting. Behind a readable fd it surfaces on the next wait, which
    // passes the same change again.
    throw std::system_error(static_cast<int>(timer_error),
                            std::generic_category(),
                            "kevent EV_ADD deadline timer");
  }
  return woken ? WakeReason::WOKEN : WakeReason::DEADLINE;
}

}  // namespace talos::event
