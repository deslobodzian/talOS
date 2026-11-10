#include "talOS/events/os/poller.h"

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "talOS/events/time.h"

#if defined(__APPLE__)
#include <mach/mach.h>
#include <mach/mach_time.h>
#include <mach/thread_policy.h>
#include <pthread.h>
#endif

namespace talos::event {
namespace {

constexpr Duration kMillisecond{1'000'000};
constexpr Duration kSecond{1'000'000'000};

TEST(PollerTest, DeadlineElapsesAtLeastTheRequestedDuration) {
  Poller poller;
  const MonotonicTime start = Poller::now();
  const Poller::WakeReason reason =
      poller.wait_until(start + 20 * kMillisecond);
  const Duration elapsed = Poller::now() - start;

  EXPECT_EQ(reason, Poller::WakeReason::DEADLINE);
  EXPECT_GE(elapsed, 20 * kMillisecond);
  // Generous upper slack: this only needs to prove wait_until() didn't
  // return early, not that it's precise on loaded CI.
  EXPECT_LT(elapsed, 500 * kMillisecond);
}

TEST(PollerTest, PastDeadlineReturnsImmediately) {
  Poller poller;
  const MonotonicTime start = Poller::now();
  const Poller::WakeReason reason = poller.wait_until(start - kMillisecond);
  const Duration elapsed = Poller::now() - start;

  EXPECT_EQ(reason, Poller::WakeReason::DEADLINE);
  EXPECT_LT(elapsed, 5 * kMillisecond);
}

TEST(PollerTest, WakeFromAnotherThreadReturnsPromptlyNotAfterDeadline) {
  Poller poller;
  std::thread waker([&poller] {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    poller.wake();
  });

  const MonotonicTime start = Poller::now();
  const Poller::WakeReason reason = poller.wait_until(start + 5 * kSecond);
  const Duration elapsed = Poller::now() - start;
  waker.join();

  EXPECT_EQ(reason, Poller::WakeReason::WOKEN);
  // Well under the 5s deadline -- proves wake() broke the wait rather than
  // the wait simply running to completion.
  EXPECT_LT(elapsed, kSecond);
}

TEST(PollerTest, WakeBeforeWaitIsConsumedOnceThenDeadlineNext) {
  Poller poller;
  poller.wake();

  // A wake() issued before wait_until() is called must still be observed:
  // it must not require the wait to already be in progress.
  EXPECT_EQ(poller.wait_until(Poller::now() + 5 * kSecond),
            Poller::WakeReason::WOKEN);

  // The wake must not persist: a second wait sees none of it and runs to
  // its own deadline.
  const MonotonicTime start = Poller::now();
  const Poller::WakeReason reason =
      poller.wait_until(start + 20 * kMillisecond);
  const Duration elapsed = Poller::now() - start;

  EXPECT_EQ(reason, Poller::WakeReason::DEADLINE);
  EXPECT_GE(elapsed, 20 * kMillisecond);
}

TEST(PollerTest, WatchedFdReportsReadableThenDeadlineAfterDraining) {
  int fds[2];
  ASSERT_EQ(::pipe(fds), 0);
  const int read_fd = fds[0];
  const int write_fd = fds[1];

  Poller poller;
  poller.add_fd(read_fd);

  std::thread writer([write_fd] {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    const char byte = 'x';
    EXPECT_EQ(::write(write_fd, &byte, 1), 1);
  });

  const Poller::WakeReason reason =
      poller.wait_until(Poller::now() + 5 * kSecond);
  writer.join();
  EXPECT_EQ(reason, Poller::WakeReason::READABLE);

  char buf;
  ASSERT_EQ(::read(read_fd, &buf, 1), 1);  // drain the byte

  const MonotonicTime start = Poller::now();
  const Poller::WakeReason drained_reason =
      poller.wait_until(start + 20 * kMillisecond);
  const Duration elapsed = Poller::now() - start;

  EXPECT_EQ(drained_reason, Poller::WakeReason::DEADLINE);
  EXPECT_GE(elapsed, 20 * kMillisecond);

  ::close(write_fd);
  ::close(read_fd);
}

TEST(PollerTest, RemoveFdStopsItFromWakingThePoller) {
  int fds[2];
  ASSERT_EQ(::pipe(fds), 0);
  const int read_fd = fds[0];
  const int write_fd = fds[1];

  Poller poller;
  poller.add_fd(read_fd);
  poller.remove_fd(read_fd);

  const char byte = 'x';
  ASSERT_EQ(::write(write_fd, &byte, 1), 1);

  const MonotonicTime start = Poller::now();
  const Poller::WakeReason reason =
      poller.wait_until(start + 20 * kMillisecond);
  const Duration elapsed = Poller::now() - start;

  EXPECT_EQ(reason, Poller::WakeReason::DEADLINE);
  EXPECT_GE(elapsed, 20 * kMillisecond);

  ::close(write_fd);
  ::close(read_fd);
}

TEST(PollerTest, MoveConstructionPreservesWatchedFdAndMovedFromIsSafe) {
  int fds[2];
  ASSERT_EQ(::pipe(fds), 0);
  const int read_fd = fds[0];
  const int write_fd = fds[1];

  auto original = std::make_unique<Poller>();
  original->add_fd(read_fd);

  Poller moved(std::move(*original));
  original.reset();  // destroy the moved-from poller; must not double-close.

  const char byte = 'x';
  ASSERT_EQ(::write(write_fd, &byte, 1), 1);

  const Poller::WakeReason reason = moved.wait_until(Poller::now() + kSecond);
  EXPECT_EQ(reason, Poller::WakeReason::READABLE);

  ::close(write_fd);
  ::close(read_fd);
}

#if defined(__APPLE__)
// The time-constraint policy talos::process::ConfigureRealtime gives a control
// node's or the bridge's loop thread on macOS (set inline: talOS/events sits
// below talOS/process).
bool SetTimeConstraintPolicy(Duration period) {
  mach_timebase_info_data_t timebase{};
  if (::mach_timebase_info(&timebase) != KERN_SUCCESS) return false;
  const auto to_abs = [&](Duration duration) {
    return static_cast<std::uint32_t>(duration.count() * timebase.denom /
                                      timebase.numer);
  };
  thread_time_constraint_policy_data_t policy{};
  policy.period = to_abs(period);
  policy.computation = to_abs(period / 2);
  policy.constraint = to_abs(period);
  policy.preemptible = TRUE;
  return ::thread_policy_set(::pthread_mach_thread_np(::pthread_self()),
                             THREAD_TIME_CONSTRAINT_POLICY,
                             reinterpret_cast<thread_policy_t>(&policy),
                             THREAD_TIME_CONSTRAINT_POLICY_COUNT) ==
         KERN_SUCCESS;
}

// A relative kevent() timeout is padded by macOS timer coalescing even on a
// realtime thread (milliseconds late per 10 ms wait); the non-coalesced
// absolute timer is not. The median bound is loose enough for a loaded
// machine yet far below the coalescing leeway.
TEST(PollerTest, RealtimeDeadlinesWakeWithoutCoalescingLeeway) {
  constexpr Duration kPeriod = 10 * kMillisecond;
  constexpr std::size_t kWaits = 40;
  std::vector<Duration> lateness;
  lateness.reserve(kWaits);
  bool policy_set = false;

  std::thread loop([&] {
    policy_set = SetTimeConstraintPolicy(kPeriod);
    Poller poller;
    MonotonicTime deadline = Poller::now() + kPeriod;
    for (std::size_t i = 0; i < kWaits; ++i, deadline += kPeriod) {
      EXPECT_EQ(poller.wait_until(deadline), Poller::WakeReason::DEADLINE);
      lateness.push_back(Poller::now() - deadline);
    }
  });
  loop.join();

  ASSERT_TRUE(policy_set);
  ASSERT_EQ(lateness.size(), kWaits);
  auto median = lateness.begin() + kWaits / 2;
  std::nth_element(lateness.begin(), median, lateness.end());
  EXPECT_LT(*median, kMillisecond / 2);
}
#endif

}  // namespace
}  // namespace talos::event
