#include "talOS/process/process.h"

#include <fcntl.h>
#include <gtest/gtest.h>
#include <poll.h>
#include <pthread.h>
#include <sys/wait.h>
#include <unistd.h>

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

// Whether `fd` has something to read, without waiting.
bool Readable(int fd) {
  pollfd entry{.fd = fd, .events = POLLIN, .revents = 0};
  return ::poll(&entry, 1, 0) == 1 && (entry.revents & POLLIN) != 0;
}

// A stop signal must reach a node's event loop wherever it waits: SIGINT and
// SIGTERM set the flag and make the stop pipe readable, which is what the
// loop watches.
// REQ-OPS-06
TEST(ProcessTest, StopSignalsSetTheFlagAndWakeTheStopFd) {
  InstallStopHandlers();
  stop_requested.store(false);
  ASSERT_FALSE(ConsumeStopWake());
  ASSERT_GE(StopFd(), 0);
  EXPECT_FALSE(Readable(StopFd()));

  for (const int signal : {SIGINT, SIGTERM}) {
    SCOPED_TRACE(signal);
    std::raise(signal);
    EXPECT_TRUE(stop_requested.load());
    EXPECT_TRUE(Readable(StopFd()));
    EXPECT_TRUE(ConsumeStopWake());
    stop_requested.store(false);
    EXPECT_FALSE(ConsumeStopWake());
    EXPECT_FALSE(Readable(StopFd()));
  }
}

// A storm of stops never blocks the handler, which would hang the signalled
// thread, however full the pipe: here more stale bytes than a pipe holds
// (64 KiB on Linux and macOS), each left by a stop the flag was then cleared
// of. The next watcher reads them out instead of spinning on them, and a stop
// requested from code afterwards still wakes it and stays visible.
// REQ-OPS-06
TEST(ProcessTest, RepeatedStopsNeverBlockAndStaleWakesAreReadOut) {
  InstallStopHandlers();
  stop_requested.store(false);
  ASSERT_FALSE(ConsumeStopWake());
  constexpr int kStops = 70'000;
  for (int i = 0; i < kStops; ++i) {
    std::raise(SIGTERM);
    stop_requested.store(false);
  }
  EXPECT_TRUE(Readable(StopFd()));
  EXPECT_FALSE(ConsumeStopWake());
  EXPECT_FALSE(Readable(StopFd()));

  RequestStop();
  EXPECT_TRUE(Readable(StopFd()));
  EXPECT_TRUE(ConsumeStopWake());
  EXPECT_TRUE(Readable(StopFd()));  // for every other watcher
  stop_requested.store(false);
  EXPECT_FALSE(ConsumeStopWake());
}

// A stop sent to a node the launcher started with the stop signals blocked
// (a member whose --describe says holds_stop) while its runtime was still
// starting stays pending, and becomes the node's stop the moment it installs
// its handlers: stop_requested is set when InstallStopHandlers returns. Here
// the signal is process-directed, as the launcher's kill of the member's
// group is, and arrives before the handlers exist. Each row runs in a forked
// child, whose exit code is the flag: 0 for a requested stop, 1 for a stop
// still pending or lost.
// REQ-OPS-06
TEST(ProcessTest, AStopHeldSinceStartIsTakenOnceTheHandlersAreInstalled) {
  for (const int signal : {SIGINT, SIGTERM}) {
    SCOPED_TRACE(signal);
    const pid_t child = ::fork();
    ASSERT_GE(child, 0);
    if (child == 0) {
      stop_requested.store(false);
      sigset_t held;
      sigemptyset(&held);
      for (const int stop : kHeldStopSignals) sigaddset(&held, stop);
      ::pthread_sigmask(SIG_BLOCK, &held, nullptr);
      ::kill(::getpid(), signal);
      InstallStopHandlers();
      ::_exit(stop_requested.load() ? 0 : 1);
    }
    int status = 0;
    ASSERT_EQ(::waitpid(child, &status, 0), child);
    ASSERT_TRUE(WIFEXITED(status))
        << "the held signal ended the process (signal "
        << (WIFSIGNALED(status) ? WTERMSIG(status) : 0) << ")";
    EXPECT_EQ(WEXITSTATUS(status), 0)
        << "no stop requested once the handlers were installed: the held "
           "signal is still pending";
  }
  // The child's RequestStop wrote into the stop pipe it shares with this
  // process: drain it so later tests start with no wake pending.
  stop_requested.store(false);
  while (ConsumeStopWake()) {
  }
}

// A child a node execs must not inherit the stop pipe, and a watcher reading
// it out must never block.
TEST(ProcessTest, TheStopFdIsCloseOnExecAndNonBlocking) {
  const int fd = StopFd();
  ASSERT_GE(fd, 0);
  EXPECT_NE(::fcntl(fd, F_GETFD) & FD_CLOEXEC, 0);
  EXPECT_NE(::fcntl(fd, F_GETFL) & O_NONBLOCK, 0);
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
  RealtimeOutcome returned;
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
        const bool unchanged =
            outcome.returned.grant() == RealtimeGrant::kNotRequested &&
            outcome.returned.refused == 0 && !outcome.applied &&
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

// Report a full grant only when the OS actually applied the tier's policy,
// and name the policy as refused exactly when it was not applied: that is
// what the node's registry row and session_report judge a run by. Never fail
// on the memory steps where the OS allows them: a host that grants the policy
// here also grants the stack lock (no rings are mapped), so a memory step
// that always failed would break the first equality. An unprivileged Linux
// host refuses SCHED_FIFO, which exercises the refusal.
TEST(ProcessTest, RealtimeReportsWhatTheOsGrantedForEveryTier) {
  ::unsetenv("TALOS_REALTIME");
  static_assert(noexcept(ConfigureRealtime(RealtimeOptions{})));
  for (const LoopTier tier :
       {LoopTier::kObserver, LoopTier::kControl, LoopTier::kBridge}) {
    const Outcome outcome = ConfigureOnFreshThread(tier);
    const RealtimeOutcome& returned = outcome.returned;
    EXPECT_EQ(returned.tier, tier);
    EXPECT_TRUE(returned.requested);
    EXPECT_EQ(returned.grant() == RealtimeGrant::kFull, outcome.applied)
        << TierName(tier) << ": " << RefusedStepNames(returned.refused);
    EXPECT_EQ(returned.grant() == RealtimeGrant::kRefused, !outcome.applied)
        << TierName(tier) << ": " << RefusedStepNames(returned.refused);
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
    (void)ConfigureRealtime({.period = 10ms, .tier = LoopTier::kControl});
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
