#pragma once

#include <sys/timerfd.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <ctime>
#include <system_error>
class TimerFd {
 public:
  TimerFd(clockid_t clock, int flags)
      : timer_fd_(::timerfd_create(clock, flags)) {
    if (timer_fd_ < 0) {
      throw std::system_error(errno, std::generic_category(), "timerfd_create");
    }
  }

  void set_time(std::chrono::nanoseconds period_ns) {
    timer_spec_.it_value = to_timer_spec(period_ns);
    timer_spec_.it_interval = to_timer_spec(period_ns);

    if (timerfd_settime(timer_fd_, 0, &timer_spec_, nullptr) < 0) {
      const int current_errno = errno;
      ::close(timer_fd_);

      throw std::system_error(current_errno, std::generic_category(),
                              "timerfd_settime");
    }
  }

  std::uint64_t read() {
    uint64_t expirations;
    ssize_t result = ::read(timer_fd_, &expirations, sizeof(expirations));

    if (result == sizeof(expirations)) {
      return result;
    }

    if (result < 0 && errno == EAGAIN) {
      return 0;
    }

    throw std::system_error(errno, std::generic_category(), "read counter fd");
  }

  int get_timer_fd() const { return timer_fd_; }

 private:
  timespec to_timer_spec(std::chrono::nanoseconds duration) {
    using namespace std::chrono;
    const auto seconds_part = duration_cast<seconds>(duration);
    const auto nano_seconds =
        duration_cast<nanoseconds>(duration - seconds_part);

    return timespec{.tv_sec = static_cast<time_t>(seconds_part.count()),
                    .tv_nsec = static_cast<int64_t>(nano_seconds.count())};
  }

  int timer_fd_;
  struct itimerspec timer_spec_{};
};
