#pragma once

// Process-lifetime plumbing shared by every talOS node binary: stop signals
// and the realtime scheduling of the thread that runs the loop.

#include <atomic>
#include <chrono>
#include <csignal>

namespace talos::process {

inline std::atomic<bool> stop_requested{false};
static_assert(std::atomic<bool>::is_always_lock_free);
inline void RequestStop(int) {
  stop_requested.store(true, std::memory_order_relaxed);
}
inline void InstallStopHandlers() {
  std::signal(SIGINT, RequestStop);
  std::signal(SIGTERM, RequestStop);
}

struct RealtimeOptions {
  std::chrono::nanoseconds
      period{};       // loop period (Mach time-constraint policy)
  int priority = 50;  // SCHED_FIFO priority on Linux, 1..99
};

// Best effort, for the calling thread: mlockall(MCL_CURRENT|MCL_FUTURE) (Linux
// only; macOS has no working mlockall) plus
// SCHED_FIFO at `priority` (Linux) or THREAD_TIME_CONSTRAINT_POLICY derived
// from `period` (macOS, where a non-positive period counts as 1 ms). Threads
// created afterwards inherit the policy on Linux, so call it last, from the
// thread that runs the loop.
//
// Returns true when every step took effect. Returns false without touching
// anything when the environment sets TALOS_REALTIME=0. Returns false and
// prints one line to stderr when the OS refuses a step (no privilege, a
// memlock limit, a refused Mach policy); the steps it could take
// stay in effect. Never throws, never aborts.
bool ConfigureRealtime(const RealtimeOptions& options) noexcept;

}  // namespace talos::process
