#include "talOS/events/os/epoll_poller.h"

#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/timerfd.h>
#include <unistd.h>

#include <cerrno>
#include <cstdint>
#include <ctime>
#include <system_error>

namespace talos::event {

namespace {

timespec ToAbsoluteTimespec(MonotonicTime deadline) {
  constexpr std::int64_t kNanosPerSecond = 1'000'000'000LL;
  return timespec{
      .tv_sec = static_cast<time_t>(deadline.nanos() / kNanosPerSecond),
      .tv_nsec = static_cast<decltype(timespec::tv_nsec)>(deadline.nanos() %
                                                          kNanosPerSecond)};
}

// eventfd/timerfd reads always return the whole accumulated counter (or
// expiration count) in one call and reset it to zero, so a single
// non-blocking read is sufficient to fully drain either. EAGAIN just means
// there was nothing pending, which is not an error here.
void Drain(int fd) noexcept {
  std::uint64_t value;
  [[maybe_unused]] const ssize_t result = ::read(fd, &value, sizeof(value));
}

}  // namespace

EpollPoller::EpollPoller()
    : epoll_fd_(::epoll_create1(EPOLL_CLOEXEC)),
      timer_fd_(::timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC)),
      wake_fd_(::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC)) {
  if (epoll_fd_ == -1) {
    throw std::system_error(errno, std::generic_category(), "epoll_create1");
  }
  if (timer_fd_ == -1) {
    const int saved_errno = errno;
    ::close(epoll_fd_);
    throw std::system_error(saved_errno, std::generic_category(),
                            "timerfd_create");
  }
  if (wake_fd_ == -1) {
    const int saved_errno = errno;
    ::close(epoll_fd_);
    ::close(timer_fd_);
    throw std::system_error(saved_errno, std::generic_category(), "eventfd");
  }

  try {
    register_internal_fd(timer_fd_);
    register_internal_fd(wake_fd_);
  } catch (...) {
    close_all();
    throw;
  }
}

EpollPoller::~EpollPoller() { close_all(); }

void EpollPoller::close_all() noexcept {
  // Order doesn't matter for correctness (each fd is independent), but
  // closing epoll_fd_ last means the other two are never referenced by a
  // dangling epoll registration in between.
  if (wake_fd_ != -1) {
    ::close(wake_fd_);
    wake_fd_ = -1;
  }
  if (timer_fd_ != -1) {
    ::close(timer_fd_);
    timer_fd_ = -1;
  }
  if (epoll_fd_ != -1) {
    ::close(epoll_fd_);
    epoll_fd_ = -1;
  }
}

EpollPoller::EpollPoller(EpollPoller&& other) noexcept
    : epoll_fd_(other.epoll_fd_),
      timer_fd_(other.timer_fd_),
      wake_fd_(other.wake_fd_) {
  other.epoll_fd_ = -1;
  other.timer_fd_ = -1;
  other.wake_fd_ = -1;
}

EpollPoller& EpollPoller::operator=(EpollPoller&& other) noexcept {
  if (this != &other) {
    close_all();
    epoll_fd_ = other.epoll_fd_;
    timer_fd_ = other.timer_fd_;
    wake_fd_ = other.wake_fd_;
    other.epoll_fd_ = -1;
    other.timer_fd_ = -1;
    other.wake_fd_ = -1;
  }
  return *this;
}

MonotonicTime EpollPoller::now() {
  // Must be the same clock id the timerfd is created with (CLOCK_MONOTONIC)
  // so that a deadline computed from now() lines up with what
  // TFD_TIMER_ABSTIME below actually measures against.
  struct timespec ts;
  ::clock_gettime(CLOCK_MONOTONIC, &ts);
  return MonotonicTime::from_nanos(
      static_cast<std::int64_t>(ts.tv_sec) * 1'000'000'000LL + ts.tv_nsec);
}

void EpollPoller::wake() {
  const std::uint64_t one = 1;
  if (::write(wake_fd_, &one, sizeof(one)) == -1) {
    throw std::system_error(errno, std::generic_category(), "write eventfd");
  }
}

void EpollPoller::register_internal_fd(int fd) {
  epoll_event event{};
  event.events = EPOLLIN;
  event.data.fd = fd;
  if (::epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, fd, &event) == -1) {
    throw std::system_error(errno, std::generic_category(), "epoll_ctl ADD");
  }
}

void EpollPoller::add_fd(int fd) { register_internal_fd(fd); }

void EpollPoller::remove_fd(int fd) {
  // Errors are ignored here rather than thrown: ENOENT/EBADF just mean `fd`
  // was never watched or already closed (the documented no-op case), and
  // removal is best-effort cleanup with nothing for the caller to act on.
  ::epoll_ctl(epoll_fd_, EPOLL_CTL_DEL, fd, nullptr);
}

void EpollPoller::arm_timer(MonotonicTime deadline) {
  // Drain any expiration count left over from a previous call before
  // rearming. timerfd_settime() replaces it_value/it_interval but does not
  // reset the unread expiration counter, so without this an old, already
  // reported DEADLINE could bleed into the next wait_until() as a spurious
  // immediate return.
  Drain(timer_fd_);

  itimerspec spec{};
  if (deadline != MonotonicTime::max()) {
    // it_value all-zero (the default above) disarms the timer, which is how
    // MonotonicTime::max() blocks indefinitely. Otherwise it_value is the
    // absolute deadline; it_interval stays zero for one-shot. Because
    // TFD_TIMER_ABSTIME is passed below, a deadline already in the past
    // makes the timer expire on the very next epoll_wait() instead of
    // blocking -- exactly the "past deadline returns immediately" contract.
    spec.it_value = ToAbsoluteTimespec(deadline);
  }

  if (::timerfd_settime(timer_fd_, TFD_TIMER_ABSTIME, &spec, nullptr) == -1) {
    throw std::system_error(errno, std::generic_category(), "timerfd_settime");
  }
}

EpollPoller::WakeReason EpollPoller::wait_until(MonotonicTime deadline) {
  arm_timer(deadline);

  epoll_event events[kMaxEvents];
  int n;
  do {
    // epoll_wait()'s own timeout is left infinite; arm_timer() above is what
    // actually bounds this wait, at the granularity the deadline needs.
    n = ::epoll_wait(epoll_fd_, events, kMaxEvents, -1);
    // EINTR is retried, not surfaced: wait_until()'s contract is to return
    // only for one of the three documented reasons, and a signal landing
    // mid-wait is not one of them. The timerfd deadline is unaffected by
    // the retry since it is armed to an absolute time, not a countdown.
  } while (n == -1 && errno == EINTR);

  if (n == -1) {
    throw std::system_error(errno, std::generic_category(), "epoll_wait");
  }

  // A single epoll_wait() call can report the watched fd, the wake fd and
  // the timer fd together. The whole batch is scanned (and the wake/timer
  // fds fully drained) before deciding what to return, so that never
  // reporting a WOKEN or DEADLINE that also happened to fire this round
  // doesn't leave stale state to confuse a later call. READABLE outranks
  // the other two because it carries data the caller must not miss a
  // chance to observe; a deferred WOKEN or DEADLINE simply surfaces (or, in
  // the timer's case, gets rearmed and re-evaluated) on the next call.
  bool woken = false;
  bool readable = false;
  for (int i = 0; i < n; ++i) {
    const int fd = events[i].data.fd;
    if (fd == timer_fd_) {
      Drain(timer_fd_);
    } else if (fd == wake_fd_) {
      Drain(wake_fd_);
      woken = true;
    } else {
      readable = true;
    }
  }

  if (readable) return WakeReason::READABLE;
  if (woken) return WakeReason::WOKEN;
  return WakeReason::DEADLINE;
}

}  // namespace talos::event
