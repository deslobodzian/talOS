#include "talOS/process/process.h"

#include <gtest/gtest.h>
#include <pthread.h>

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
  const Outcome outcome = ConfigureOnFreshThread();
  ::unsetenv("TALOS_REALTIME");

  EXPECT_FALSE(outcome.returned);
  EXPECT_FALSE(outcome.realtime);
}

// Simulation alone does not decide: TALOS_REALTIME=1 qualifies a --sim run
// under the production policy, and =0 keeps even the robot unprivileged.
TEST(ProcessTest, RealtimeRequestFollowsSimulationUnlessTheEnvironmentSays) {
  ::unsetenv("TALOS_REALTIME");
  EXPECT_TRUE(RealtimeRequested(false));
  EXPECT_FALSE(RealtimeRequested(true));
  ::setenv("TALOS_REALTIME", "1", 1);
  EXPECT_TRUE(RealtimeRequested(true));
  EXPECT_TRUE(RealtimeRequested(false));
  ::setenv("TALOS_REALTIME", "0", 1);
  EXPECT_FALSE(RealtimeRequested(false));
  EXPECT_FALSE(RealtimeRequested(true));
  ::unsetenv("TALOS_REALTIME");
}

// Report success only when the OS actually applied the realtime policy.
TEST(ProcessTest, RealtimeIsBestEffort) {
  ::unsetenv("TALOS_REALTIME");
  static_assert(noexcept(ConfigureRealtime(RealtimeOptions{})));
  const Outcome outcome = ConfigureOnFreshThread();

  EXPECT_EQ(outcome.returned, outcome.realtime);
}

// A helper thread doing unbounded work (camera capture, detection) must run
// below every loop thread, which runs at the default timeshare priority under
// --sim.
TEST(ProcessTest, BackgroundThreadRunsBelowTheDefaultPriority) {
  static_assert(noexcept(ConfigureBackgroundThread()));
  bool before = true;
  bool returned = false;
  bool after = false;
  std::thread worker{[&] {
    before = ThreadIsBackground();
    returned = ConfigureBackgroundThread();
    after = ThreadIsBackground();
  }};
  worker.join();

  EXPECT_FALSE(before);
  EXPECT_TRUE(returned);
  EXPECT_TRUE(after);
}

// A thread created by a realtime loop thread inherits SCHED_FIFO on Linux.
// The helper must take it out of realtime, and so must keep any thread it
// creates afterwards (a detector's worker pool) out of realtime too; on Linux
// that thread also inherits the background nice value.
TEST(ProcessTest, BackgroundThreadLeavesAnInheritedRealtimePolicy) {
  ::unsetenv("TALOS_REALTIME");
  bool parent_realtime = false;
  bool returned = false;
  bool child_realtime = true;
  bool grandchild_realtime = true;
  bool grandchild_background = false;
  std::thread parent{[&] {
    ConfigureRealtime({.period = 10ms, .priority = 1});
    parent_realtime = ThreadIsRealtime();
    if (!parent_realtime) return;
    std::thread child{[&] {
      returned = ConfigureBackgroundThread();
      child_realtime = ThreadIsRealtime();
      std::thread grandchild{[&] {
        grandchild_realtime = ThreadIsRealtime();
        grandchild_background = ThreadIsBackground();
      }};
      grandchild.join();
    }};
    child.join();
  }};
  parent.join();
  if (!parent_realtime) GTEST_SKIP() << "realtime refused; nothing to inherit";

  EXPECT_TRUE(returned);
  EXPECT_FALSE(child_realtime);
  EXPECT_FALSE(grandchild_realtime);
#if !defined(__APPLE__)
  EXPECT_TRUE(grandchild_background);
#endif
}

}  // namespace
}  // namespace talos::process
