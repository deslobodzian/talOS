#include "talOS/process/process.h"

#include <fcntl.h>
#include <gtest/gtest.h>
#include <poll.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
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

// A stop requested while ConsumeStopWake() is draining stale bytes from
// StopFd() may have its wake byte drained too. ConsumeStopWake() must still
// put a byte back so every other watcher on StopFd() sees the stop
// (process.h: "A stop leaves the pipe readable, so every watcher sees it").
// REQ-OPS-06
TEST(ProcessTest, ConsumeStopWakePreservesReadabilityWhenStopLandsMidDrain) {
  const int fd = StopFd();
  ASSERT_GE(fd, 0);
  stop_requested.store(false);
  ASSERT_FALSE(ConsumeStopWake());
  ASSERT_FALSE(Readable(fd));

  // Push stale bytes into the stop pipe without filling it (8 KiB stays below
  // macOS's initial 16 KiB and Linux's 64 KiB pipe buffers so RequestStop()'s
  // write never hits EAGAIN while giving ConsumeStopWake() 128 read(64) calls).
  constexpr int kStaleBytes = 8 * 1024;
  constexpr int kMaxTrials = 32;
  bool observed_mid_drain = false;
  for (int trial = 0; trial < kMaxTrials && !observed_mid_drain; ++trial) {
    stop_requested.store(false);
    while (ConsumeStopWake()) {
    }
    for (int i = 0; i < kStaleBytes; ++i) {
      RequestStop();
      stop_requested.store(false);
    }
    int initial_pending = 0;
    ASSERT_EQ(::ioctl(fd, FIONREAD, &initial_pending), 0);
    ASSERT_EQ(initial_pending, kStaleBytes);

    std::atomic<bool> observer_ready{false};
    std::atomic<bool> drain_done{false};
    bool saw_partial_drain = false;
    std::thread stopper{[&] {
      observer_ready.store(true, std::memory_order_release);
      for (;;) {
        int pending = initial_pending;
        if (::ioctl(fd, FIONREAD, &pending) == 0 && pending < initial_pending) {
          saw_partial_drain = pending > 0;
          RequestStop();
          return;
        }
        if (drain_done.load(std::memory_order_acquire)) {
          RequestStop();
          return;
        }
      }
    }};

    while (!observer_ready.load(std::memory_order_acquire)) {
    }
    const bool consumed = ConsumeStopWake();
    drain_done.store(true, std::memory_order_release);
    stopper.join();

    ASSERT_TRUE(stop_requested.load());
    // Whether the stop landed mid-drain (consumed == true) or right after the
    // drain ended (consumed == false), StopFd() must be readable for every
    // other watcher.
    EXPECT_TRUE(Readable(fd))
        << "stop wake lost (consumed=" << consumed
        << ", saw_partial_drain=" << saw_partial_drain << ")";
    EXPECT_TRUE(ConsumeStopWake());
    EXPECT_TRUE(Readable(fd));

    if (consumed && saw_partial_drain) {
      observed_mid_drain = true;
    }
  }
  EXPECT_TRUE(observed_mid_drain);

  stop_requested.store(false);
  EXPECT_FALSE(ConsumeStopWake());
  EXPECT_FALSE(Readable(fd));
}

// First creation of the stop pipe (StopFd()) racing with a concurrent
// RequestStop() from another thread must never lose the stop wake: StopFd()
// must end up readable and ConsumeStopWake() must report true while keeping
// it readable. Runs in a re-executed child so StopFd()'s static pipe is
// created fresh inside the race.
// REQ-OPS-06
TEST(ProcessTest, ConcurrentRequestStopAndStopFdCreationLeavesStopFdReadable) {
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(
      {
        std::atomic<int> ready{0};
        int fd = -1;
        std::thread creator{[&] {
          ready.fetch_add(1, std::memory_order_acq_rel);
          while (ready.load(std::memory_order_acquire) < 2) {
          }
          fd = StopFd();
        }};
        std::thread stopper{[&] {
          ready.fetch_add(1, std::memory_order_acq_rel);
          while (ready.load(std::memory_order_acquire) < 2) {
          }
          RequestStop();
        }};
        creator.join();
        stopper.join();
        const bool ok = stop_requested.load() && fd >= 0 && Readable(fd) &&
                        ConsumeStopWake() && Readable(fd);
        std::_Exit(ok ? 0 : 1);
      },
      ::testing::ExitedWithCode(0), "");
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
#elif defined(__linux__)
  const int policy = ::sched_getscheduler(0);
  return policy >= 0 && (policy & ~SCHED_RESET_ON_FORK) == SCHED_FIFO;
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
#elif defined(__linux__)
  const int policy = ::sched_getscheduler(0);
  sched_param param{};
  if (policy < 0 || ::sched_getparam(0, &param) != 0) return false;
  return policy == (SCHED_FIFO | SCHED_RESET_ON_FORK) &&
         param.sched_priority == expected.fifo_priority;
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

// A thread created by a realtime thread that did not set SCHED_RESET_ON_FORK
// inherits SCHED_FIFO on Linux. The helper must take it out of realtime, and so
// must keep any thread it creates afterwards (a detector's worker pool) out of
// realtime too; on Linux that thread also inherits the background nice value.
TEST(ProcessTest, BackgroundThreadLeavesAnInheritedRealtimePolicy) {
  ::unsetenv("TALOS_REALTIME");
  bool parent_realtime = false;
  bool returned = false;
  bool child_realtime = true;
  bool grandchild_realtime = true;
  bool grandchild_background = false;
  std::thread parent{[&] {
#if defined(__linux__)
    sched_param param{};
    param.sched_priority = kControlFifoPriority;
    (void)::sched_setscheduler(0, SCHED_FIFO, &param);
#else
    (void)ConfigureRealtime({.period = 10ms, .tier = LoopTier::kControl});
#endif
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

#if defined(__linux__)
// ConfigureRealtime passes SCHED_RESET_ON_FORK on Linux, so neither a forked
// child process nor a newly created thread inherits SCHED_FIFO from the loop
// thread. Runs in a forked child so fork() stays single-threaded and the test
// process keeps its own scheduling.
// REQ-RT-08
TEST(ProcessTest, RealtimeResetsPolicyOnForkAndThreadCreation) {
  ::unsetenv("TALOS_REALTIME");
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    const RealtimeOutcome outcome =
        ConfigureRealtime({.period = 10ms, .tier = LoopTier::kControl});
    if (outcome.grant() == RealtimeGrant::kRefused) ::_exit(2);
    if (!ThreadHasTierPolicy(LoopTier::kControl)) ::_exit(3);

    const pid_t grandchild = ::fork();
    if (grandchild < 0) ::_exit(4);
    if (grandchild == 0) {
      ::_exit(::sched_getscheduler(0) == SCHED_OTHER ? 0 : 1);
    }
    int gc_status = 0;
    if (::waitpid(grandchild, &gc_status, 0) != grandchild ||
        !WIFEXITED(gc_status) || WEXITSTATUS(gc_status) != 0) {
      ::_exit(5);
    }

    int thread_policy = -1;
    std::thread worker{[&] { thread_policy = ::sched_getscheduler(0); }};
    worker.join();
    ::_exit(thread_policy == SCHED_OTHER ? 0 : 6);
  }
  int status = 0;
  ASSERT_EQ(::waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFEXITED(status));
  if (WEXITSTATUS(status) == 2) {
    GTEST_SKIP() << "realtime refused; cannot verify SCHED_RESET_ON_FORK";
  }
  EXPECT_EQ(WEXITSTATUS(status), 0)
      << "child failed at step " << WEXITSTATUS(status);
}
#endif

}  // namespace
}  // namespace talos::process
