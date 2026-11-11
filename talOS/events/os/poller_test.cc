#include "talOS/events/os/poller.h"

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
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

#if defined(__linux__)
#include <sys/epoll.h>
#include <sys/utsname.h>

#include <cerrno>
#include <charconv>
#include <ctime>
#include <string_view>
#include <system_error>
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

// A loop watching several fds (its wake receiver and the process's stop
// pipe) acts on the one that woke it, and only that one: READABLE names it,
// and the next wait's answer replaces the report.
TEST(PollerTest, AReadableWakeNamesOnlyTheFdsThatWereReady) {
  int ready[2];
  int quiet[2];
  ASSERT_EQ(::pipe(ready), 0);
  ASSERT_EQ(::pipe(quiet), 0);
  Poller poller;
  poller.add_fd(ready[0]);
  poller.add_fd(quiet[0]);
  const char byte = 'x';
  ASSERT_EQ(::write(ready[1], &byte, 1), 1);

  EXPECT_EQ(poller.wait_until(Poller::now() + 5 * kSecond),
            Poller::WakeReason::READABLE);
  EXPECT_TRUE(poller.reported_readable(ready[0]));
  EXPECT_FALSE(poller.reported_readable(quiet[0]));

  char buf;
  ASSERT_EQ(::read(ready[0], &buf, 1), 1);
  EXPECT_EQ(poller.wait_until(Poller::now() + kMillisecond),
            Poller::WakeReason::DEADLINE);
  EXPECT_FALSE(poller.reported_readable(ready[0]));

  for (const int fd : {ready[0], ready[1], quiet[0], quiet[1]}) ::close(fd);
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
// XNU delivers every EVFILT_TIMER through one shared thread_call worker; on a
// loaded CI runner it stalled and every loop on the machine overslept its
// deadline together. With the timer never armed (a delivery that never
// comes), the kevent() timeout alone must end each wait: never before the
// deadline, and not grossly after. Without that timeout the wait never
// returns, so a guard wakes it and the test fails on the reason, not a hang.
TEST(PollerTest, DeadlineHoldsWhenTimerDeliveryStalls) {
  constexpr Duration kWait = 10 * kMillisecond;
  constexpr std::size_t kWaits = 5;
  Poller poller(Poller::DeadlineTimer::NOT_ARMED);

  std::mutex mutex;
  std::condition_variable finished;
  bool done = false;
  std::thread guard([&] {
    std::unique_lock<std::mutex> lock(mutex);
    if (!finished.wait_for(lock, std::chrono::seconds(2),
                           [&] { return done; })) {
      poller.wake();
    }
  });

  std::vector<Duration> lateness;
  for (std::size_t i = 0; i < kWaits; ++i) {
    const MonotonicTime deadline = Poller::now() + kWait;
    const Poller::WakeReason reason = poller.wait_until(deadline);
    lateness.push_back(Poller::now() - deadline);
    if (reason != Poller::WakeReason::DEADLINE) {
      ADD_FAILURE() << "wait " << i << " did not end at its deadline";
      break;
    }
  }
  {
    const std::lock_guard<std::mutex> lock(mutex);
    done = true;
  }
  finished.notify_one();
  guard.join();

  ASSERT_EQ(lateness.size(), kWaits);
  for (const Duration late : lateness) EXPECT_GE(late, Duration::zero());
  // A gross bound: the timeout's coalescing leeway is about a fifth of the
  // wait, and a loaded or sanitizer runner adds scheduling delay (7.4 ms
  // seen under macOS TSan); an unbounded wait is what this catches.
  auto median = lateness.begin() + kWaits / 2;
  std::nth_element(lateness.begin(), median, lateness.end());
  EXPECT_LT(*median, kWait);
}

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

#if defined(__linux__)
// Lateness of `waits` consecutive absolute deadlines `period` apart, the way
// RealtimeEventLoop waits for its next timer.
std::vector<Duration> PeriodicLateness(Poller& poller, Duration period,
                                       std::size_t waits) {
  std::vector<Duration> lateness;
  lateness.reserve(waits);
  MonotonicTime deadline = Poller::now() + period;
  for (std::size_t i = 0; i < waits; ++i, deadline += period) {
    EXPECT_EQ(poller.wait_until(deadline), Poller::WakeReason::DEADLINE);
    lateness.push_back(Poller::now() - deadline);
  }
  return lateness;
}

Duration Median(std::vector<Duration> values) {
  auto median = values.begin() + static_cast<std::ptrdiff_t>(values.size() / 2);
  std::nth_element(values.begin(), median, values.end());
  return *median;
}

// epoll_pwait2 arrived in Linux 5.11.
bool KernelHasEpollPwait2() {
  utsname name{};
  if (::uname(&name) != 0) return false;
  const std::string_view release(name.release);
  int major = 0;
  int minor = 0;
  const char* end = release.data() + release.size();
  const auto parsed_major = std::from_chars(release.data(), end, major);
  if (parsed_major.ec != std::errc{} || parsed_major.ptr == end ||
      *parsed_major.ptr != '.') {
    return false;
  }
  const auto parsed_minor = std::from_chars(parsed_major.ptr + 1, end, minor);
  if (parsed_minor.ec != std::errc{}) return false;
  return major > 5 || (major == 5 && minor >= 11);
}

// On a 5.11+ kernel the default poller waits with epoll_pwait2 (whose
// hrtimer PREEMPT_RT expires in hard interrupt context for a realtime thread,
// unlike a timerfd's), never wakes before a deadline, and is not grossly
// late. Fails if the build stops compiling epoll_pwait2 in, or the relative
// timeout is computed in the wrong unit or from a stale clock reading.
TEST(PollerTest, PeriodicDeadlinesWakeOnTimeWithEpollPwait2) {
  if (!KernelHasEpollPwait2()) GTEST_SKIP() << "kernel before 5.11";
  Poller poller;
  const std::vector<Duration> lateness =
      PeriodicLateness(poller, 2 * kMillisecond, 50);

  EXPECT_TRUE(poller.waits_with_pwait2());
  for (const Duration late : lateness) EXPECT_GE(late, Duration::zero());
  // A gross bound: a timeshare thread on a loaded CI runner can wait a
  // scheduler slice (about 2 ms) for its CPU, so this only catches a
  // timeout in the wrong unit, not milliseconds of jitter.
  EXPECT_LT(Median(lateness), 2 * kMillisecond);
}

int unsupported_pwait2_calls = 0;

int UnsupportedPwait2(int /*epoll_fd*/, epoll_event* /*events*/,
                      int /*max_events*/, const timespec* /*timeout*/) {
  ++unsupported_pwait2_calls;
  errno = ENOSYS;
  return -1;
}

// A kernel before 5.11 answers ENOSYS: the poller must fall back to the
// timerfd on the first wait, keep its deadlines, and not probe again.
TEST(PollerTest, KernelWithoutEpollPwait2FallsBackToTheTimerfd) {
  unsupported_pwait2_calls = 0;
  Poller poller(&UnsupportedPwait2);
  const std::vector<Duration> lateness =
      PeriodicLateness(poller, 2 * kMillisecond, 50);

  EXPECT_FALSE(poller.waits_with_pwait2());
  EXPECT_EQ(unsupported_pwait2_calls, 1);
  for (const Duration late : lateness) EXPECT_GE(late, Duration::zero());
  EXPECT_LT(Median(lateness), 2 * kMillisecond);
}
#endif

}  // namespace
}  // namespace talos::event
