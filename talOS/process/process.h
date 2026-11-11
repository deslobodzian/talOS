#pragma once

// Process-lifetime plumbing shared by every talOS node binary: stop signals
// the realtime scheduling of the thread that runs the loop, and the
// background scheduling of helper threads.

#include <signal.h>

#include <atomic>
#include <chrono>
#include <cstdint>

#include "talOS/process/realtime_outcome.h"

namespace talos::process {

// Defined once, in process.cc. An inline variable is defined in every shared
// object that uses it, and one loaded RTLD_LOCAL binds to its own copy: a
// node library hosted by talos.cpp_node would watch a flag libtalos_node.so
// never sets. Set it only through RequestStop, which also wakes the watchers
// of StopFd(); a test may clear it between cases.
extern std::atomic<bool> stop_requested;
static_assert(std::atomic<bool>::is_always_lock_free);

// Requests a stop: sets stop_requested and, when it was clear, writes one
// byte to the stop pipe, so that StopFd() reads ready and an event loop
// watching it wakes at once wherever it waits. The SIGINT/SIGTERM handler,
// and callable from any thread: async-signal-safe (a lock-free exchange and
// one write(2) to a non-blocking pipe), keeps errno, never blocks and never
// fails (a pipe too full to take the byte is readable already). Before the
// pipe exists it sets only the flag, and the pipe starts readable.
void RequestStop(int signal = 0) noexcept;

// The read end of this process's stop pipe, for an event loop to watch
// (event::RealtimeEventLoop::watch_fd): readable once a stop is requested.
// Created on the first call (InstallStopHandlers makes it), non-blocking and
// close-on-exec, so no child a node execs inherits it, and never closed. -1
// when it could not be created (no descriptor left).
int StopFd() noexcept;

// For a watcher of StopFd() that found it readable: whether a stop is
// requested. A stop leaves the pipe readable, so every watcher sees it. When
// none is requested (stop_requested was cleared after an earlier stop), it
// reads the stale bytes out, so a level-triggered watcher does not spin, and
// then answers from the flag again, so a stop that lands meanwhile is not
// lost.
bool ConsumeStopWake() noexcept;

// SIGINT and SIGTERM call RequestStop instead of ending the process, and the
// stop pipe exists before either can arrive. The handlers restart an
// interrupted system call (SA_RESTART): a stop must not turn a blocking call
// in progress, such as a node constructor waiting on a lock, into a spurious
// error. A restartable call keeps blocking until it returns on its own (poll,
// select, sleeps and timed socket calls still return EINTR); the launcher
// escalates to SIGKILL after its grace period. std::signal already installed
// restarting handlers, so behaviour after construction is unchanged.
void InstallStopHandlers();

struct RealtimeOptions {
  std::chrono::nanoseconds period{};  // loop period (Mach time-constraint)
  LoopTier tier = LoopTier::kObserver;
};

// SCHED_FIFO priorities per tier on Linux. All below 50, where PREEMPT_RT
// runs every threaded interrupt handler (the NIC's, which delivers the
// RoboRIO's UDP to the bridge, among them), so an interrupt is never held off
// by a loop; all above every SCHED_OTHER thread. Gaps leave room for a tier
// between them.
//
// A node's event-loop tick, and the bridge between its inputs, waits in
// epoll_pwait2 (talOS/events epoll_poller), a sleeper hrtimer that
// PREEMPT_RT expires in hard interrupt context for a realtime thread. On a
// kernel before 5.11 that wait falls back to a timerfd, whose hrtimer
// PREEMPT_RT expires in softirq context, in ktimers/N at SCHED_FIFO 1
// (ksoftirqd, SCHED_OTHER, on some older RT kernels), behind any loop running
// on that CPU, a lower tier included, until it sleeps. On a kernel without
// PREEMPT_RT hrtimers expire in hard interrupt context, but softirq work
// deferred to ksoftirqd (SCHED_OTHER), such as network receive under load,
// waits behind every loop on that CPU. TODO.md RT-4.
inline constexpr int kObserverFifoPriority = 40;
inline constexpr int kControlFifoPriority = 45;
inline constexpr int kBridgeFifoPriority = 49;

// The scheduling ConfigureRealtime asks the OS for, for both OSes, so the
// mapping is checked anywhere without privilege.
struct LoopPolicy {
  int fifo_priority = 0;  // Linux SCHED_FIFO priority
  // macOS: THREAD_TIME_CONSTRAINT_POLICY with the three durations below, or,
  // when false, timeshare at QOS_CLASS_USER_INTERACTIVE. Every
  // time-constraint thread runs above every timeshare thread, but the public
  // policy has no priority (the kernel orders those threads by deadline, wake
  // time plus constraint), so on macOS only observers are ordered: they stay
  // off the time-constraint band, which the bridge and kControl loops share.
  bool time_constraint = false;
  std::chrono::nanoseconds period{};
  std::chrono::nanoseconds computation{};
  std::chrono::nanoseconds constraint{};
};

// The policy for `options`: the tier's FIFO priority; for the bridge and
// kControl, a time-constraint period of options.period (1 ms when not
// positive), computation half of it and constraint all of it.
LoopPolicy PolicyFor(const RealtimeOptions& options) noexcept;

// Whether a node binary (talos::Run, the hardware bridge) asks for realtime
// scheduling of its loop thread. By default it does on the robot and not
// under --sim. TALOS_REALTIME=1 asks under --sim as well, so a host without a
// RoboRIO runs the control path against a simulated (in-memory commissioned)
// gateway under the production scheduling policy; TALOS_REALTIME=0 never
// asks. Nodes inherit the variable from the launcher's environment.
bool RealtimeRequested(bool simulation) noexcept;

// Best effort, for the calling thread. Memory: on Linux
// mlockall(MCL_CURRENT|MCL_FUTURE); on macOS, which has no working mlockall,
// a read of every page of the __TEXT segment of the image holding talOS (the
// node binary, or libtalos_node.so under Python), prefaulted but not locked
// because macOS refuses mlock of executable pages. On both, every
// shared-memory ring (SharedMemoryPtr) mapped now or later is prefaulted for
// writing and mlocked (memory::KeepMappingsResident), and the calling
// thread's stack is prefaulted 256 KiB below this call (less on a smaller
// stack) and mlocked up to its top. Scheduling: PolicyFor(options), SCHED_FIFO
// on Linux, the time-constraint policy or QOS_CLASS_USER_INTERACTIVE on macOS.
// Threads created afterwards inherit the policy on Linux, so call it last,
// from the thread that runs the loop.
//
// Returns what it asked for and what it got (realtime_outcome.h): `tier`
// from options, `requested` false without touching anything when the
// environment sets TALOS_REALTIME=0, and otherwise the kStep* bits the OS
// refused (no privilege, RLIMIT_MEMLOCK or the wired-memory limit, a refused
// Mach policy or QoS), after printing them with their reasons as one line to
// stderr. The steps it could take stay in effect, and a refused lock still
// leaves the rings and stack prefaulted. A ring mapped later whose lock is
// refused is reported by talOS/memory, not here. The caller publishes the
// outcome (the node's registry row, introspect::NodeRegistration::
// set_realtime), where the launcher and session_report look for it. Never
// throws, never aborts.
[[nodiscard]] RealtimeOutcome ConfigureRealtime(
    const RealtimeOptions& options) noexcept;

// The nice value ConfigureBackgroundThread gives a thread on Linux.
inline constexpr int kBackgroundNice = 10;

// Best effort, for the calling thread: puts it below every node loop thread,
// for a helper thread doing unbounded work (camera capture and detection,
// image encoding). On Linux it leaves any realtime policy it inherited (a
// thread created after ConfigureRealtime inherits SCHED_FIFO) for SCHED_OTHER
// and raises its nice value to at least kBackgroundNice; threads it creates
// afterwards inherit both. On macOS it sets QOS_CLASS_UTILITY, below the
// default QoS a loop thread runs at under timeshare; a thread created with
// pthread_create starts at the default QoS again (macOS does not inherit
// QoS or the time-constraint policy), so every thread calls this itself.
// Independent of TALOS_REALTIME.
//
// Returns true when every step took effect; otherwise prints one line to
// stderr and returns false. Never throws.
bool ConfigureBackgroundThread() noexcept;

// Whether the calling thread runs at the background priority
// ConfigureBackgroundThread sets (or lower).
bool ThreadIsBackground() noexcept;

}  // namespace talos::process
