#pragma once

// Process-lifetime signal handling shared by every talOS node binary.

#include <atomic>
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
}  // namespace talos::process
