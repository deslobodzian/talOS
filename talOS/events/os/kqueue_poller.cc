#include "talOS/events/os/kqueue_poller.h"

#include <fcntl.h>
#include <sys/event.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <ctime>
#include <system_error>

namespace talos::event {

namespace {

// kevent()'s timeout is relative, so a Duration must be converted to a
// timespec on every call rather than once at registration time (unlike an
// EVFILT_TIMER, which would need updating -- and cleaning up -- for every
// new deadline. A relative kevent() timeout has no such state).
timespec ToRelativeTimespec(Duration remaining) {
  const auto secs = std::chrono::duration_cast<std::chrono::seconds>(remaining);
  const auto nanos = remaining - secs;
  return timespec{
      .tv_sec = static_cast<time_t>(secs.count()),
      .tv_nsec = static_cast<decltype(timespec::tv_nsec)>(nanos.count())};
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
  // by clock discipline, so this is the one clock id used for both now()
  // and the relative timeouts computed from it below.
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
  const bool block_indefinitely = deadline == MonotonicTime::max();

  int n;
  for (;;) {
    timespec timeout;
    const timespec* timeout_ptr = nullptr;
    if (!block_indefinitely) {
      // Recomputed every retry (see the EINTR loop below) so a signal that
      // arrives mid-wait can't make this overrun its deadline: each retry
      // re-measures how much time is actually left.
      const Duration remaining = deadline - now();
      timeout = remaining > Duration::zero() ? ToRelativeTimespec(remaining)
                                             : timespec{0, 0};
      timeout_ptr = &timeout;
    }

    n = ::kevent(kq_fd_, nullptr, 0, events, kMaxEvents, timeout_ptr);
    if (n != -1 || errno != EINTR) break;
    // EINTR is retried, not surfaced: wait_until()'s contract is to return
    // only for one of the three documented reasons, and a signal landing
    // mid-wait is not one of them.
  }

  if (n == -1) {
    throw std::system_error(errno, std::generic_category(), "kevent");
  }

  // A single kevent() call can report both a readable fd and the wake event
  // together. READABLE outranks WOKEN here for the same reason it outranks
  // a passed deadline: it carries data the caller must not miss a chance to
  // observe, whereas a deferred WOKEN or DEADLINE simply surfaces on the
  // next call with no information lost (EV_CLEAR already reset the wake
  // event's state regardless of whether we act on it now).
  bool woken = false;
  for (int i = 0; i < n; ++i) {
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
