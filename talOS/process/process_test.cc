#include "talOS/process/process.h"

#include <gtest/gtest.h>
#include <pthread.h>

#include <chrono>
#include <csignal>
#include <cstdlib>
#include <string>
#include <thread>

#include "talOS/memory/residency.h"

#if defined(__APPLE__)
#include <mach/mach.h>
#include <mach/thread_policy.h>
#include <pthread/qos.h>
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

// Bridge above control above observers on Linux, all below the SCHED_FIFO 50
// of PREEMPT_RT's threaded interrupt handlers (a loop must not hold off the
// NIC delivering the RoboRIO's packets). On macOS, whose public
// time-constraint policy has no priority, observers stay off the
// time-constraint band so they cannot delay a control loop.
TEST(ProcessTest, TiersMapToOrderedPolicies) {
  const LoopPolicy bridge =
      PolicyFor({.period = 1ms, .tier = LoopTier::kBridge});
  const LoopPolicy control =
      PolicyFor({.period = 20ms, .tier = LoopTier::kControl});
  const LoopPolicy observer =
      PolicyFor({.period = 20ms, .tier = LoopTier::kObserver});

  EXPECT_LT(0, observer.fifo_priority);
  EXPECT_LT(observer.fifo_priority, control.fifo_priority);
  EXPECT_LT(control.fifo_priority, bridge.fifo_priority);
  EXPECT_LT(bridge.fifo_priority, 50);

  EXPECT_TRUE(bridge.time_constraint);
  EXPECT_TRUE(control.time_constraint);
  EXPECT_FALSE(observer.time_constraint);
  // An unknown period must not become a zero-length constraint.
  EXPECT_GT(PolicyFor({.tier = LoopTier::kControl}).constraint, 0ns);
}

#if defined(__APPLE__)
bool HasTimeConstraint() {
  thread_time_constraint_policy_data_t policy{};
  mach_msg_type_number_t count = THREAD_TIME_CONSTRAINT_POLICY_COUNT;
  boolean_t get_default = FALSE;
  const kern_return_t result = ::thread_policy_get(
      ::pthread_mach_thread_np(::pthread_self()), THREAD_TIME_CONSTRAINT_POLICY,
      reinterpret_cast<thread_policy_t>(&policy), &count, &get_default);
  return result == KERN_SUCCESS && get_default == FALSE;
}
#endif

// Whether the calling thread runs under a realtime policy (SCHED_FIFO, or
// the Mach time-constraint policy).
bool ThreadIsRealtime() {
#if defined(__APPLE__)
  return HasTimeConstraint();
#else
  int policy = 0;
  sched_param param{};
  ::pthread_getschedparam(::pthread_self(), &policy, &param);
  return policy == SCHED_FIFO;
#endif
}

// Whether the calling thread runs under exactly the policy PolicyFor gives
// `tier`.
bool ThreadHasTierPolicy(LoopTier tier) {
  const LoopPolicy expected = PolicyFor({.period = 10ms, .tier = tier});
#if defined(__APPLE__)
  if (expected.time_constraint) return HasTimeConstraint();
  return !HasTimeConstraint() &&
         ::qos_class_self() == QOS_CLASS_USER_INTERACTIVE;
#else
  int policy = 0;
  sched_param param{};
  ::pthread_getschedparam(::pthread_self(), &policy, &param);
  return policy == SCHED_FIFO && param.sched_priority == expected.fifo_priority;
#endif
}

// Runs ConfigureRealtime on a fresh thread, so the test's own thread keeps
// its scheduling, and reports what it returned and whether that thread ended
// up under the tier's policy.
struct Outcome {
  bool returned = false;
  bool applied = false;
};
Outcome ConfigureOnFreshThread(LoopTier tier) {
  Outcome outcome;
  std::thread worker{[&outcome, tier] {
    outcome.returned = ConfigureRealtime({.period = 10ms, .tier = tier});
    outcome.applied = ThreadHasTierPolicy(tier);
  }};
  worker.join();
  return outcome;
}

// In a re-executed child (the "threadsafe" death-test style runs the binary
// afresh): whether rings are kept resident is process-wide and sticky, so a
// ConfigureRealtime from another test, before this one or in an earlier
// repeat, must not decide it.
TEST(ProcessTest, RealtimeDisabledByEnvironmentChangesNothing) {
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
      {
        ::setenv("TALOS_REALTIME", "0", 1);
        const Outcome outcome = ConfigureOnFreshThread(LoopTier::kControl);
        const bool unchanged = !outcome.returned && !outcome.applied &&
                               !memory::MappingsKeptResident();
        std::_Exit(unchanged ? 0 : 1);
      },
      ::testing::ExitedWithCode(0), "");
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

// Report success only when the OS actually applied the tier's policy, and
// never fail on the memory steps where the OS allows them: a host that grants
// the policy here also grants the stack lock (no rings are mapped), so a
// memory step that always failed would break the equality.
TEST(ProcessTest, RealtimeIsBestEffortForEveryTier) {
  ::unsetenv("TALOS_REALTIME");
  static_assert(noexcept(ConfigureRealtime(RealtimeOptions{})));
  for (const LoopTier tier :
       {LoopTier::kObserver, LoopTier::kControl, LoopTier::kBridge}) {
    const Outcome outcome = ConfigureOnFreshThread(tier);
    EXPECT_EQ(outcome.returned, outcome.applied)
        << "tier " << static_cast<int>(tier);
  }
  EXPECT_TRUE(memory::MappingsKeptResident());
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
    ConfigureRealtime({.period = 10ms, .tier = LoopTier::kControl});
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
