#include "talOS/process/process.h"

#include <gtest/gtest.h>
#include <pthread.h>

#include <algorithm>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <string>
#include <thread>

#if defined(__APPLE__)
#include <mach/mach.h>
#include <mach/thread_policy.h>
#else
#include <sched.h>
#endif

namespace talos::process {
namespace {

using namespace std::chrono_literals;

TEST(ProcessTest, StopHandlersSetStopRequestedOnSigintAndSigterm) {
  stop_requested.store(false, std::memory_order_relaxed);
  InstallStopHandlers();
  EXPECT_FALSE(stop_requested.load(std::memory_order_relaxed));

  std::raise(SIGINT);
  EXPECT_TRUE(stop_requested.load(std::memory_order_relaxed));

  stop_requested.store(false, std::memory_order_relaxed);
  std::raise(SIGTERM);
  EXPECT_TRUE(stop_requested.load(std::memory_order_relaxed));

  stop_requested.store(false, std::memory_order_relaxed);
}

// Whether the calling thread runs under the realtime policy
// ConfigureRealtime sets.
bool ThreadIsRealtime() {
#if defined(__APPLE__)
  thread_time_constraint_policy_data_t policy{};
  mach_msg_type_number_t count = THREAD_TIME_CONSTRAINT_POLICY_COUNT;
  boolean_t get_default = FALSE;
  const kern_return_t result = ::thread_policy_get(
      ::pthread_mach_thread_np(::pthread_self()), THREAD_TIME_CONSTRAINT_POLICY,
      reinterpret_cast<thread_policy_t>(&policy), &count, &get_default);
  return result == KERN_SUCCESS && get_default == FALSE;
#else
  int policy = 0;
  sched_param param{};
  ::pthread_getschedparam(::pthread_self(), &policy, &param);
  return policy == SCHED_FIFO;
#endif
}

// Runs ConfigureRealtime on a fresh thread, so the test's own thread keeps
// its scheduling, and reports what it returned and whether that thread ended
// up realtime.
struct Outcome {
  bool returned = false;
  bool realtime = false;
};
Outcome ConfigureOnFreshThread() {
  Outcome outcome;
  std::thread worker{[&outcome] {
    outcome.returned = ConfigureRealtime({.period = 10ms, .priority = 50});
    outcome.realtime = ThreadIsRealtime();
  }};
  worker.join();
  return outcome;
}

TEST(ProcessTest, RealtimeDisabledByEnvironmentChangesNothing) {
  ::setenv("TALOS_REALTIME", "0", 1);
  ::testing::internal::CaptureStderr();
  const Outcome outcome = ConfigureOnFreshThread();
  const std::string printed = ::testing::internal::GetCapturedStderr();
  ::unsetenv("TALOS_REALTIME");

  EXPECT_FALSE(outcome.returned);
  EXPECT_FALSE(outcome.realtime);
  EXPECT_EQ(printed, "");
}

// Whatever the machine allows: no exception, no abort, and either success in
// silence or one line saying what was refused.
TEST(ProcessTest, RealtimeIsBestEffort) {
  ::unsetenv("TALOS_REALTIME");
  static_assert(noexcept(ConfigureRealtime(RealtimeOptions{})));
  ::testing::internal::CaptureStderr();
  const Outcome outcome = ConfigureOnFreshThread();
  const std::string printed = ::testing::internal::GetCapturedStderr();

  if (outcome.returned) {
    EXPECT_EQ(printed, "");
    EXPECT_TRUE(outcome.realtime);
  } else {
    EXPECT_EQ(std::count(printed.begin(), printed.end(), '\n'), 1) << printed;
  }
}

}  // namespace
}  // namespace talos::process
