
#pragma once

#include <sys/epoll.h>
#include <sys/timerfd.h>
#include <chrono>
#include <ctime>
#include "talOS/events/epoll.h"
#include "talOS/events/timerfd.h"

namespace talos::event {

template <typename Derived>
class PeriodicEvent : public EpollHandler<PeriodicEvent<Derived>>  {
public:
    using Clock = std::chrono::steady_clock;
    std::vector<std::uint64_t> counts_;

    void on_event(uint32_t events) {
        auto now = Clock::now();
        auto elapsed = now - last_time_;
        auto diff = std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed);
        counts_.emplace_back(diff.count());
        //std::printf("Periodic timer %luu us loop period at %luu\n", period_.count(), diff.count());
        //static_cast<Derived*>(this)->run();
        last_time_ = now;
    }

protected:
    explicit PeriodicEvent(std::chrono::nanoseconds period)
    : EpollHandler<PeriodicEvent<Derived>> {EPOLL_EVENT_FD_NOT_DEFINED}, timer_{CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC}, period_{period} {
        this->set_fd(timer_.get_timer_fd());
        timer_.set_time(period);
        last_time_ = Clock::now();
    }
private:
    TimerFd timer_;
    [[maybe_unused]]std::chrono::nanoseconds period_{};
    std::chrono::time_point<Clock> last_time_;
};

class EventLoop {
public:
    explicit EventLoop(int flags = EPOLL_CLOEXEC);
    ~EventLoop() {
    }

    EventLoop(const EventLoop&) = delete;
    EventLoop& operator=(const EventLoop&) = delete;

    EventLoop(EventLoop&&) = delete;
    EventLoop& operator=(EventLoop&&) = delete;

    void run() {
        while (running_) {
            auto result = epoll_.poll();
            if (!result) {
                std::printf("Failed to run event");
            }
        }
    }


    void remove(int fd) noexcept {
//        ::epoll_ctl(
//            epoll_fd_,
//            EPOLL_CTL_DEL,
//            fd,
//            nullptr
//        );
    }


private:
    bool running_ = false;
    int event_fd_;
    Epoll epoll_;
};

} // namespace talos::event

