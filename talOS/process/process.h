#pragma once

// Process-lifetime plumbing shared by every talOS node binary: stop signals
// the realtime scheduling of the thread that runs the loop, and the
// background scheduling of helper threads.

#include <signal.h>

#include <atomic>
#include <chrono>

namespace talos::process {

// Defined once, in process.cc. An inline variable is defined in every shared
// object that uses it, and one loaded RTLD_LOCAL binds to its own copy: a
// node library hosted by talos.cpp_node would watch a flag libtalos_node.so
// never sets.
extern std::atomic<bool> stop_requested;
static_assert(std::atomic<bool>::is_always_lock_free);
inline void RequestStop(int) {
  stop_requested.store(true, std::memory_order_relaxed);
}
// SIGINT and SIGTERM set stop_requested instead of ending the process. The
// handlers restart an interrupted system call (SA_RESTART): a stop must not
// turn a blocking call in progress, such as a node constructor waiting on a
// lock, into a spurious error. A restartable call keeps blocking until it
// returns on its own (poll, select, sleeps and timed socket calls still return
// EINTR); the launcher escalates to SIGKILL after its grace period. std::signal
// already installed restarting handlers, so behaviour after construction is
// unchanged.
inline void InstallStopHandlers() {
  struct sigaction action {};
  action.sa_handler = RequestStop;
  sigemptyset(&action.sa_mask);
  action.sa_flags = SA_RESTART;
  sigaction(SIGINT, &action, nullptr);
  sigaction(SIGTERM, &action, nullptr);
}

struct RealtimeOptions {
  std::chrono::nanoseconds
      period{};       // loop period (Mach time-constraint policy)
  int priority = 50;  // SCHED_FIFO priority on Linux, 1..99
};

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
// stack) and mlocked up to its top. Scheduling:
// SCHED_FIFO at `priority` (Linux) or THREAD_TIME_CONSTRAINT_POLICY derived
// from `period` (macOS, where a non-positive period counts as 1 ms). Threads
// created afterwards inherit the policy on Linux, so call it last, from the
// thread that runs the loop.
//
// Returns true when every step took effect. Returns false without touching
// anything when the environment sets TALOS_REALTIME=0. Returns false and
// prints one line to stderr when the OS refuses a step (no privilege,
// RLIMIT_MEMLOCK or the wired-memory limit, a refused Mach policy); the
// steps it could take stay in effect, and a refused lock still leaves the
// rings and stack prefaulted. Never throws, never aborts.
bool ConfigureRealtime(const RealtimeOptions& options) noexcept;

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
