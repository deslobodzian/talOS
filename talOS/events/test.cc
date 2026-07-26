#include <gtest/gtest.h>
#include <sys/epoll.h>
#include <sys/timerfd.h>
#include <unistd.h>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
#include "epoll.h"
#include "talOS/events/event_loop.h"
#include "timerfd.h"
#include <sys/eventfd.h>

TEST(Event, EventTest) {
  EXPECT_STRNE("hello", "world");
  EXPECT_EQ(6 * 7, 42);
}

class TestEvent : public EpollHandler<TestEvent> {
public:
    explicit TestEvent(int fd) : EpollHandler{fd} {}

    void on_event(uint32_t events) {
        std::printf("running on events %i\n", events);
    }
};


class TestPeriodEvent : public talos::event::PeriodicEvent<TestPeriodEvent> {
public:
    TestPeriodEvent() : PeriodicEvent<TestPeriodEvent>(std::chrono::milliseconds(50)) {}

    void run() {
        std::printf("Test Period Event\n");
    }
};

TEST(Epoll, EventTest) {
    Epoll epoll(EPOLL_CLOEXEC);
    int event_fd = ::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);

    TestEvent event{event_fd};
    epoll.register_event(event, EPOLLIN | EPOLLET);

    uint64_t wakeup = 1;
    [[maybe_unused]] auto s = ::write(event_fd, &wakeup, sizeof(uint64_t));
    bool ret = epoll.poll();
    EXPECT_TRUE(ret);
};


TEST(PeriodicEpollNested, EventTest) {
    Epoll epoll(EPOLL_CLOEXEC);
    using namespace std::chrono_literals;
    using Clock = std::chrono::steady_clock;
    TestPeriodEvent event{};
    epoll.register_event(event, EPOLLIN | EPOLLET);
    auto start = Clock::now();
    auto run_duration = start + 0.5s;
    int triggers = 0;

    while (Clock::now() < run_duration) {
        [[maybe_unused]]bool ret = epoll.poll();
        triggers++;
    };

    for (const auto& count : event.counts_) {
        std::printf("Periodic timer loop period at %luu\n", count);
    }

    EXPECT_EQ(triggers, 10);
}

