#include "talOS/events/os/epoll_poller.h"

#include <sys/epoll.h>
#include <sys/eventfd.h>
#include <sys/prctl.h>
#include <sys/syscall.h>
#include <sys/timerfd.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <system_error>

// glibc 2.35 added the epoll_pwait2() wrapper (<sys/epoll.h> includes
// <features.h>, which defines __GLIBC_PREREQ). Without it, the raw syscall
// when the kernel headers number it.
#if defined(__GLIBC__) && defined(__GLIBC_PREREQ)
#if __GLIBC_PREREQ(2, 35)
#define TALOS_GLIBC_EPOLL_PWAIT2 1
#endif
#endif

namespace talos::event {

namespace {

constexpr std::int64_t kNanosPerSecond = 1'000'000'000LL;

timespec ToTimespec(std::int64_t nanos) {
  return timespec{.tv_sec = static_cast<time_t>(nanos / kNanosPerSecond),
                  .tv_nsec = static_cast<decltype(timespec::tv_nsec)>(
                      nanos % kNanosPerSecond)};
}

#if defined(TALOS_GLIBC_EPOLL_PWAIT2)
int SystemEpollPwait2(int epoll_fd, epoll_event* events, int max_events,
                      const timespec* timeout) {
  return ::epoll_pwait2(epoll_fd, events, max_events, timeout, nullptr);
}
#elif defined(__NR_epoll_pwait2)
// The syscall takes the kernel's 64-bit struct __kernel_timespec, which is
// not the libc timespec on a 32-bit target with a 32-bit time_t.
struct KernelTimespec {
  std::int64_t tv_sec;
  long long tv_nsec;
};

int SystemEpollPwait2(int epoll_fd, epoll_event* events, int max_events,
                      const timespec* timeout) {
  KernelTimespec kernel_timeout{};
  const KernelTimespec* kernel_timeout_ptr = nullptr;
  if (timeout != nullptr) {
    kernel_timeout.tv_sec = static_cast<std::int64_t>(timeout->tv_sec);
    kernel_timeout.tv_nsec = static_cast<long long>(timeout->tv_nsec);
    kernel_timeout_ptr = &kernel_timeout;
  }
  // syscall() reads every argument as a long; a null sigmask ignores the
  // sigset size.
  return static_cast<int>(
      ::syscall(__NR_epoll_pwait2, static_cast<long>(epoll_fd), events,
                static_cast<long>(max_events), kernel_timeout_ptr,
                static_cast<const void*>(nullptr), static_cast<long>(0)));
}
#endif

// The kernel pads a timeshare thread's epoll_pwait2() timeout by the larger
// of the thread's timer slack (50 us by default) and a thousandth of the
// wait, a realtime thread's by nothing. The timerfd has no slack, so a
// timeshare loop (in simulation, or one refused its realtime grant) sets its
// slack to 1 ns, once per thread, which leaves the thousandth (1 us on a
// 1 ms tick).
void MinimizeTimerSlack() noexcept {
  thread_local bool minimized = false;
  if (minimized) return;
  minimized = true;
  ::prctl(PR_SET_TIMERSLACK, 1UL, 0UL, 0UL, 0UL);
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

EpollPoller::Pwait2 EpollPoller::system_pwait2() {
#if defined(TALOS_GLIBC_EPOLL_PWAIT2) || defined(__NR_epoll_pwait2)
  return &SystemEpollPwait2;
#else
  return nullptr;
#endif
}

EpollPoller::EpollPoller(Pwait2 pwait2)
    : pwait2_(pwait2),
      epoll_fd_(::epoll_create1(EPOLL_CLOEXEC)),
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
    : pwait2_(other.pwait2_),
      epoll_fd_(other.epoll_fd_),
      timer_fd_(other.timer_fd_),
      wake_fd_(other.wake_fd_) {
  other.epoll_fd_ = -1;
  other.timer_fd_ = -1;
  other.wake_fd_ = -1;
}

EpollPoller& EpollPoller::operator=(EpollPoller&& other) noexcept {
  if (this != &other) {
    close_all();
    pwait2_ = other.pwait2_;
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
  // Must be the clock epoll_pwait2() times out on and the timerfd is
  // created with (CLOCK_MONOTONIC), so that a deadline computed from now()
  // lines up with what both actually measure against.
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

bool EpollPoller::reported_readable(int fd) const noexcept {
  return std::find(readable_, readable_ + readable_count_, fd) !=
         readable_ + readable_count_;
}

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
    spec.it_value = ToTimespec(deadline.nanos());
  }

  if (::timerfd_settime(timer_fd_, TFD_TIMER_ABSTIME, &spec, nullptr) == -1) {
    throw std::system_error(errno, std::generic_category(), "timerfd_settime");
  }
}

int EpollPoller::wait_for_events(MonotonicTime deadline, epoll_event* events) {
  while (pwait2_ != nullptr) {
    MinimizeTimerSlack();
    // The timeout is relative, so it is computed from the absolute deadline
    // right before every call, a retry after EINTR included: the retry
    // neither ends early nor restarts the whole wait. A preemption between
    // now() and the call delays the wake by its length (an absolute timerfd
    // would not); it is the price of a hard-interrupt expiry.
    timespec timeout{};
    const timespec* timeout_ptr = nullptr;
    if (deadline != MonotonicTime::max()) {
      timeout =
          ToTimespec(std::max<std::int64_t>(0, (deadline - now()).count()));
      timeout_ptr = &timeout;
    }
    const int n = pwait2_(epoll_fd_, events, kMaxEvents, timeout_ptr);
    if (n >= 0) return n;
    const int error = errno;
    if (error == EINTR) continue;
    if (error != ENOSYS && error != EPERM) {
      throw std::system_error(error, std::generic_category(), "epoll_pwait2");
    }
    // Unsupported: the kernel predates 5.11 (ENOSYS), or a seccomp filter
    // refuses a syscall it does not know (EPERM; epoll_pwait2 itself never
    // returns it). Probed once; the timerfd times every later wait. Said
    // once, in the node's log, since on PREEMPT_RT it puts the tick back
    // behind ktimers/N.
    std::fprintf(stderr,
                 "talos: epoll_pwait2 unavailable (%s); event loop ticks wait "
                 "on a timerfd\n",
                 error == ENOSYS ? "ENOSYS" : "EPERM");
    pwait2_ = nullptr;
  }

  arm_timer(deadline);
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
  return n;
}

EpollPoller::WakeReason EpollPoller::wait_until(MonotonicTime deadline) {
  epoll_event events[kMaxEvents];
  const int n = wait_for_events(deadline, events);

  // A single wait can report the watched fd, the wake fd and the timer fd
  // together. The whole batch is scanned (and the wake/timer fds fully
  // drained) before deciding what to return, so that never
  // reporting a WOKEN or DEADLINE that also happened to fire this round
  // doesn't leave stale state to confuse a later call. READABLE outranks
  // the other two because it carries data the caller must not miss a
  // chance to observe; a deferred WOKEN or DEADLINE simply surfaces (or, in
  // the timer's case, gets rearmed and re-evaluated) on the next call.
  bool woken = false;
  readable_count_ = 0;
  for (int i = 0; i < n; ++i) {
    const int fd = events[i].data.fd;
    if (fd == timer_fd_) {
      Drain(timer_fd_);
    } else if (fd == wake_fd_) {
      Drain(wake_fd_);
      woken = true;
    } else {
      readable_[readable_count_++] = fd;
    }
  }

  if (readable_count_ > 0) return WakeReason::READABLE;
  if (woken) return WakeReason::WOKEN;
  return WakeReason::DEADLINE;
}

}  // namespace talos::event
