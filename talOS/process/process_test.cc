#include "talOS/process/process.h"

#include <fcntl.h>
#include <gtest/gtest.h>
#include <poll.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "talOS/memory/residency.h"
#include "talOS/process/durable_sync.h"

#if defined(__APPLE__)
#include <mach/mach.h>
#include <mach/mach_time.h>
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
// REQ-RT-08
TEST(ProcessTest, TiersMapToOrderedPolicies) {
  const LoopPolicy bridge =
      PolicyFor({.period = 1ms, .tier = LoopTier::kBridge});
  const LoopPolicy control =
      PolicyFor({.period = 20ms, .tier = LoopTier::kControl});
  const LoopPolicy observer =
      PolicyFor({.period = 20ms, .tier = LoopTier::kObserver});

  // Exact NDG "Realtime scheduling" SCHED_FIFO priorities: 49 / 45 / 40.
  EXPECT_EQ(bridge.fifo_priority, 49);
  EXPECT_EQ(control.fifo_priority, 45);
  EXPECT_EQ(observer.fifo_priority, 40);
  EXPECT_LT(0, observer.fifo_priority);
  EXPECT_LT(observer.fifo_priority, control.fifo_priority);
  EXPECT_LT(control.fifo_priority, bridge.fifo_priority);
  EXPECT_LT(bridge.fifo_priority, 50);

  // Exact NDG Mach time-constraint ratios: computation = period/2,
  // constraint = period; unknown/non-positive period defaults to 1 ms.
  EXPECT_TRUE(bridge.time_constraint);
  EXPECT_EQ(bridge.period, 1ms);
  EXPECT_EQ(bridge.computation, 500us);
  EXPECT_EQ(bridge.constraint, 1ms);

  EXPECT_TRUE(control.time_constraint);
  EXPECT_EQ(control.period, 20ms);
  EXPECT_EQ(control.computation, 10ms);
  EXPECT_EQ(control.constraint, 20ms);

  EXPECT_FALSE(observer.time_constraint);
  for (const LoopTier tier : {LoopTier::kBridge, LoopTier::kControl}) {
    const LoopPolicy fallback = PolicyFor({.tier = tier});
    EXPECT_TRUE(fallback.time_constraint);
    EXPECT_EQ(fallback.period, 1ms);
    EXPECT_EQ(fallback.computation, 500us);
    EXPECT_EQ(fallback.constraint, 1ms);
  }

  // XNU enforces min_rt_quantum (50 us) <= computation <= max_rt_quantum
  // (50 ms) and computation <= constraint <= period. Valid kControl periods
  // above 100 ms (e.g. 125 ms, 200 ms) and tiny positive periods stay within
  // those bounds instead of failing thread_policy_set with
  // KERN_INVALID_ARGUMENT.
  for (const std::chrono::nanoseconds long_period : {125ms, 200ms}) {
    SCOPED_TRACE(long_period.count());
    const LoopPolicy policy =
        PolicyFor({.period = long_period, .tier = LoopTier::kControl});
    EXPECT_TRUE(policy.time_constraint);
    EXPECT_EQ(policy.period, long_period);
    EXPECT_EQ(policy.computation, 50ms);
    EXPECT_EQ(policy.constraint, long_period);
    EXPECT_LE(policy.computation, policy.constraint);
    EXPECT_LE(policy.constraint, policy.period);
  }
  {
    const LoopPolicy short_policy =
        PolicyFor({.period = 60us, .tier = LoopTier::kControl});
    EXPECT_EQ(short_policy.period, 60us);
    EXPECT_EQ(short_policy.computation, 50us);
    EXPECT_EQ(short_policy.constraint, 60us);
  }
  {
    const LoopPolicy tiny_policy =
        PolicyFor({.period = 1ns, .tier = LoopTier::kControl});
    EXPECT_EQ(tiny_policy.period, 50us);
    EXPECT_EQ(tiny_policy.computation, 50us);
    EXPECT_EQ(tiny_policy.constraint, 50us);
  }
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
// `tier` and `period`.
bool ThreadHasTierPolicy(LoopTier tier,
                         std::chrono::nanoseconds period = 10ms) {
  const LoopPolicy expected = PolicyFor({.period = period, .tier = tier});
#if defined(__APPLE__)
  if (expected.time_constraint) {
    thread_time_constraint_policy_data_t actual{};
    mach_msg_type_number_t count = THREAD_TIME_CONSTRAINT_POLICY_COUNT;
    boolean_t get_default = FALSE;
    if (::thread_policy_get(::pthread_mach_thread_np(::pthread_self()),
                            THREAD_TIME_CONSTRAINT_POLICY,
                            reinterpret_cast<thread_policy_t>(&actual), &count,
                            &get_default) != KERN_SUCCESS ||
        get_default != FALSE) {
      return false;
    }
    mach_timebase_info_data_t timebase{};
    if (::mach_timebase_info(&timebase) != KERN_SUCCESS ||
        timebase.numer == 0) {
      return false;
    }
    const auto to_abs = [&](std::chrono::nanoseconds duration) {
      const __int128 scaled = static_cast<__int128>(duration.count()) *
                              timebase.denom / timebase.numer;
      return static_cast<std::uint32_t>(std::clamp<__int128>(
          scaled, 0, std::numeric_limits<std::uint32_t>::max()));
    };
    const auto within_tick = [](std::uint32_t got, std::uint32_t want) {
      return got >= want ? (got - want) <= 1u : (want - got) <= 1u;
    };
    return within_tick(actual.period, to_abs(expected.period)) &&
           within_tick(actual.computation, to_abs(expected.computation)) &&
           within_tick(actual.constraint, to_abs(expected.constraint)) &&
           actual.preemptible == TRUE;
  }
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
Outcome ConfigureOnFreshThread(LoopTier tier,
                               std::chrono::nanoseconds period = 10ms) {
  Outcome outcome;
  std::thread worker{[&outcome, tier, period] {
    outcome.returned = ConfigureRealtime({.period = period, .tier = tier});
    outcome.applied = ThreadHasTierPolicy(tier, period);
  }};
  worker.join();
  return outcome;
}

// In a re-executed child (the "threadsafe" death-test style runs the binary
// afresh): whether rings are kept resident is process-wide and sticky, so a
// ConfigureRealtime from another test, before this one or in an earlier
// repeat, must not decide it.
// REQ-RT-08, REQ-RT-09
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
// REQ-RT-08
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
// REQ-RT-08, REQ-RT-09
TEST(ProcessTest, RealtimeReportsWhatTheOsGrantedForEveryTier) {
  ::unsetenv("TALOS_REALTIME");
  static_assert(noexcept(ConfigureRealtime(RealtimeOptions{})));
  for (const LoopTier tier :
       {LoopTier::kObserver, LoopTier::kControl, LoopTier::kBridge}) {
    const Outcome outcome = ConfigureOnFreshThread(tier);
    const RealtimeOutcome& returned = outcome.returned;
    EXPECT_EQ(returned.tier, tier);
    EXPECT_TRUE(returned.requested);
    EXPECT_EQ((returned.refused & kStepPolicy) == 0, outcome.applied)
        << TierName(tier) << ": " << RefusedStepNames(returned.refused);
    EXPECT_EQ(returned.grant() == RealtimeGrant::kRefused, !outcome.applied)
        << TierName(tier) << ": " << RefusedStepNames(returned.refused);
    if (outcome.applied) {
      EXPECT_TRUE(returned.grant() == RealtimeGrant::kFull ||
                  returned.grant() == RealtimeGrant::kPartial)
          << TierName(tier) << ": " << RefusedStepNames(returned.refused);
    }
  }
  EXPECT_TRUE(memory::MappingsKeptResident());
}

#if defined(__APPLE__)
// Control loops with valid periods above 100 ms (computation = period / 2 >
// XNU's 50 ms max_rt_quantum) must still be granted the time-constraint policy
// with clamped computation rather than failing KERN_INVALID_ARGUMENT.
// REQ-RT-08
TEST(ProcessTest, LongControlPeriodsGrantTimeConstraintOnMacOs) {
  ::unsetenv("TALOS_REALTIME");
  for (const std::chrono::nanoseconds period : {125ms, 200ms}) {
    SCOPED_TRACE(period.count());
    const Outcome outcome = ConfigureOnFreshThread(LoopTier::kControl, period);
    EXPECT_EQ((outcome.returned.refused & kStepPolicy) == 0, outcome.applied)
        << RefusedStepNames(outcome.returned.refused);
    EXPECT_NE(outcome.returned.grant(), RealtimeGrant::kRefused)
        << RefusedStepNames(outcome.returned.refused);
    EXPECT_TRUE(outcome.applied);
  }
}
#endif

// Calling ConfigureBackgroundThread on a thread that is already realtime (for
// instance, after ConfigureRealtime(kControl) on macOS where child threads do
// not inherit THREAD_TIME_CONSTRAINT_POLICY) must leave the realtime policy
// first and apply the background priority/QoS cleanly. On macOS, XNU
// permanently opts a formerly time-constraint thread out of POSIX QoS, so
// ConfigureBackgroundThread lowers its Mach precedence below default timeshare
// priority instead.
// REQ-RT-10
TEST(ProcessTest, BackgroundThreadDropsRealtimeOnSameThread) {
  ::unsetenv("TALOS_REALTIME");
  bool entered_realtime = false;
  bool returned = false;
  bool after_realtime = true;
  bool after_background = false;
  std::thread worker{[&] {
    (void)ConfigureRealtime({.period = 10ms, .tier = LoopTier::kControl});
    entered_realtime = ThreadIsRealtime();
    if (!entered_realtime) return;
    returned = ConfigureBackgroundThread();
    after_realtime = ThreadIsRealtime();
    after_background = ThreadIsBackground();
  }};
  worker.join();
  if (!entered_realtime) GTEST_SKIP() << "realtime refused on this host";

  EXPECT_TRUE(returned);
  EXPECT_FALSE(after_realtime);
  EXPECT_TRUE(after_background);
}

// DurableSync returns -1 with EBADF on an invalid file descriptor without
// masking the error, and succeeds on an open regular file.
// REQ-OPS-12
TEST(ProcessTest, DurableSyncPreservesBadFdErrorAndSyncsOpenFile) {
  errno = 0;
  EXPECT_EQ(DurableSync(-1), -1);
  EXPECT_EQ(errno, EBADF);

  std::string path = ::testing::TempDir() + "/talos_durable_sync_XXXXXX";
  const int fd = ::mkstemp(path.data());
  ASSERT_GE(fd, 0);
  ::unlink(path.c_str());
  const char payload = 'x';
  ASSERT_EQ(::write(fd, &payload, 1), 1);
  EXPECT_EQ(DurableSync(fd), 0);
  ::close(fd);
}

// A helper thread doing unbounded work (camera capture, detection) must run
// below every loop thread, which runs at the default timeshare priority under
// --sim.
// REQ-RT-08, REQ-RT-10
TEST(ProcessTest, BackgroundThreadRunsBelowTheDefaultPriority) {
  static_assert(noexcept(ConfigureBackgroundThread()));
  bool before = true;
  bool returned = false;
  bool after = false;
#if defined(__APPLE__)
  qos_class_t after_qos = QOS_CLASS_UNSPECIFIED;
#endif
  std::thread worker{[&] {
    before = ThreadIsBackground();
    returned = ConfigureBackgroundThread();
    after = ThreadIsBackground();
#if defined(__APPLE__)
    after_qos = ::qos_class_self();
#endif
  }};
  worker.join();

  EXPECT_FALSE(before);
  EXPECT_TRUE(returned);
  EXPECT_TRUE(after);
#if defined(__APPLE__)
  EXPECT_EQ(after_qos, QOS_CLASS_UTILITY);
#endif
}

// A thread created by a realtime thread that did not set SCHED_RESET_ON_FORK
// inherits SCHED_FIFO on Linux. The helper must take it out of realtime, and so
// must keep any thread it creates afterwards (a detector's worker pool) out of
// realtime too; on Linux that thread also inherits the background nice value.
// REQ-RT-08, REQ-RT-10
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

// Probes in a forked child whether RLIMIT_MEMLOCK=0 actually causes mlock to
// fail with ENOMEM or EPERM. Processes with CAP_IPC_LOCK (including root by
// default) bypass RLIMIT_MEMLOCK, in which case the caller skips rather than
// reporting a false pass.
enum class MemlockProbe { kRefused, kBypassed, kError };

MemlockProbe ProbeMemlockZero() {
  const auto page = static_cast<std::size_t>(::sysconf(_SC_PAGESIZE));
  const pid_t child = ::fork();
  if (child < 0) return MemlockProbe::kError;
  if (child == 0) {
    const rlimit zero{0, 0};
    if (::setrlimit(RLIMIT_MEMLOCK, &zero) != 0) ::_exit(2);
    void* p = ::mmap(nullptr, page, PROT_READ | PROT_WRITE,
                     MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (p == MAP_FAILED) ::_exit(3);
    const int err = ::mlock(p, page) == 0 ? 0 : errno;
    if (err == 0) {
      ::munlock(p, page);
      ::_exit(1);
    }
    ::_exit((err == ENOMEM || err == EPERM) ? 0 : 4);
  }
  int status = 0;
  if (::waitpid(child, &status, 0) != child || !WIFEXITED(status)) {
    return MemlockProbe::kError;
  }
  if (WEXITSTATUS(status) == 0) return MemlockProbe::kRefused;
  if (WEXITSTATUS(status) == 1) return MemlockProbe::kBypassed;
  return MemlockProbe::kError;
}

// Child body for MemlockZeroReportsRingAndStackLockRefusalsOnceOnStderr,
// factored out of EXPECT_EXIT to avoid preprocessor comma splitting.
[[noreturn]] void RunMemlockZeroChild() {
  ::unsetenv("TALOS_REALTIME");
  const rlimit zero{0, 0};
  if (::setrlimit(RLIMIT_MEMLOCK, &zero) != 0) {
    std::fprintf(stderr, "setrlimit failed: %s\n", std::strerror(errno));
    std::fflush(stderr);
    std::_Exit(10);
  }
  const auto page = static_cast<std::size_t>(::sysconf(_SC_PAGESIZE));
  const auto map_page = [page]() -> void* {
    void* p = ::mmap(nullptr, page, PROT_READ | PROT_WRITE,
                     MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    return p == MAP_FAILED ? nullptr : p;
  };
  void* r1 = map_page();
  void* r2 = map_page();
  void* late1 = map_page();
  void* late2 = map_page();
  if (r1 == nullptr || r2 == nullptr || late1 == nullptr || late2 == nullptr) {
    std::fprintf(stderr, "mmap failed\n");
    std::fflush(stderr);
    std::_Exit(11);
  }
  // Probe what errno mlock returns under RLIMIT_MEMLOCK=0 on this kernel
  // (EPERM or ENOMEM) without locking anything.
  const int probe_err = ::mlock(r1, page) == 0 ? 0 : errno;
  if (probe_err == 0) ::munlock(r1, page);
  if (probe_err != ENOMEM && probe_err != EPERM) {
    std::fprintf(stderr, "unexpected mlock probe errno: %d (%s)\n", probe_err,
                 std::strerror(probe_err));
    std::fflush(stderr);
    std::_Exit(14);
  }

  const bool before_rt_ok =
      !memory::MappingsKeptResident() && !memory::LateLockRefused();
  memory::TrackMapping(r1, page);
  memory::TrackMapping(r2, page);

  std::fflush(stderr);
  int pipefd[2]{-1, -1};
  if (::pipe(pipefd) != 0) std::_Exit(12);
  const int saved_stderr = ::dup(STDERR_FILENO);
  if (saved_stderr < 0 || ::dup2(pipefd[1], STDERR_FILENO) < 0) {
    std::_Exit(13);
  }
  ::close(pipefd[1]);

  const RealtimeOutcome outcome =
      ConfigureRealtime({.period = 10ms, .tier = LoopTier::kControl});
  // Pre-existing ring refusals surface in outcome.refused, not LateLockRefused.
  const bool after_rt_ok =
      memory::MappingsKeptResident() && !memory::LateLockRefused();
  // Two late mappings after realtime setup: LateLockRefused flips on the first
  // and only the first prints.
  memory::TrackMapping(late1, page);
  const bool after_late1_ok = memory::LateLockRefused();
  memory::TrackMapping(late2, page);
  const bool after_late2_ok = memory::LateLockRefused();
  std::fflush(stderr);

  ::dup2(saved_stderr, STDERR_FILENO);
  ::close(saved_stderr);

  for (void* p : {late2, late1, r2, r1}) {
    memory::UntrackMapping(p);
    ::munmap(p, page);
  }

  std::string captured;
  char buf[512];
  for (ssize_t n = 0; (n = ::read(pipefd[0], buf, sizeof buf)) > 0;) {
    captured.append(buf, static_cast<std::size_t>(n));
  }
  ::close(pipefd[0]);

  std::vector<std::string> lines;
  std::istringstream stream{captured};
  for (std::string line; std::getline(stream, line);) {
    lines.push_back(line);
  }

  const RealtimeGrant expected_grant = (outcome.refused & kStepPolicy) == 0
                                           ? RealtimeGrant::kPartial
                                           : RealtimeGrant::kRefused;
  const bool outcome_ok = before_rt_ok && after_rt_ok && after_late1_ok &&
                          after_late2_ok && outcome.requested &&
                          outcome.tier == LoopTier::kControl &&
                          outcome.grant() == expected_grant &&
                          (outcome.refused & kStepMlockall) != 0 &&
                          (outcome.refused & kStepRingLock) != 0 &&
                          (outcome.refused & kStepStackLock) != 0;

  const std::string err_text = std::strerror(probe_err);
  const std::string expected_prefix =
      std::string{
          "talos: realtime scheduling incomplete (control tier, "
          "grant "} +
      GrantName(outcome.grant()) + "): mlockall (" + err_text +
      "); ring mlock (2 of 2: " + err_text + "); stack mlock (" + err_text +
      ")";
  const std::string expected_late =
      "talos: shared memory mapped after realtime setup is prefaulted "
      "but not locked (mlock: " +
      err_text + "); further refusals are not reported";

  bool lines_ok = lines.size() == 2 && lines[1] == expected_late;
  if (lines_ok) {
    if ((outcome.refused & kStepPolicy) == 0) {
      lines_ok = lines[0] == expected_prefix;
    } else {
      lines_ok = lines[0].starts_with(expected_prefix + "; SCHED_FIFO (");
    }
  }

  if (!outcome_ok || !lines_ok) {
    std::fprintf(
        stderr,
        "outcome_ok=%d before_rt=%d after_rt=%d late1=%d late2=%d refused=0x%x "
        "lines=%zu\ncaptured:\n%s\nexpected0: %s\nexpected1: %s\n",
        outcome_ok ? 1 : 0, before_rt_ok ? 1 : 0, after_rt_ok ? 1 : 0,
        after_late1_ok ? 1 : 0, after_late2_ok ? 1 : 0, outcome.refused,
        lines.size(), captured.c_str(), expected_prefix.c_str(),
        expected_late.c_str());
    std::fflush(stderr);
    std::_Exit(1);
  }
  std::_Exit(0);
}

// Under RLIMIT_MEMLOCK=0 on Linux (macOS ignores RLIMIT_MEMLOCK in favor of
// the system-wide wired limit), mlockall, ring mlock and stack mlock are
// refused: ConfigureRealtime reports kStepMlockall, kStepRingLock and
// kStepStackLock in RealtimeOutcome and prints one summary line to stderr, and
// subsequent TrackMapping calls set LateLockRefused() and print the
// late-refusal line exactly once. Runs in a re-executed child so
// MappingsKeptResident starts false and RLIMIT_MEMLOCK=0 is isolated to the
// child.
// REQ-RT-09
TEST(ProcessTest, MemlockZeroReportsRingAndStackLockRefusalsOnceOnStderr) {
  const MemlockProbe probe = ProbeMemlockZero();
  if (probe == MemlockProbe::kBypassed) {
    GTEST_SKIP() << "process bypasses RLIMIT_MEMLOCK=0 (e.g. CAP_IPC_LOCK)";
  }
  ASSERT_EQ(probe, MemlockProbe::kRefused)
      << "failed to probe RLIMIT_MEMLOCK=0 behaviour";
  GTEST_FLAG_SET(death_test_style, "threadsafe");
  EXPECT_EXIT(RunMemlockZeroChild(), ::testing::ExitedWithCode(0), "");
}
#endif

}  // namespace
}  // namespace talos::process
