#pragma once

#include <cstdint>
#include <string>
#include <string_view>

namespace talos::launcher {

// Holds a macOS IOKit `kIOPMAssertionTypePreventSystemSleep` assertion (as
// `caffeinate -s` does) for the lifetime of a running robot session, as
// defense-in-depth independent of `pmset sleep 0` against idle or DarkWake
// maintenance sleep (TODO.md DEPLOY-1). Released cleanly via RAII on
// destruction or `Release()`. On non-macOS hosts (Linux), the default backend
// is a silent no-op.
class SleepGuard {
 public:
  // Injectable power-assertion backend for unit tests. `create` returns true
  // on success (writing a platform assertion ID to `assertion_id` and setting
  // `active = true`, or leaving `active = false` for a no-op backend) and
  // false on failure (populating `error`). `release` is called at most once
  // per active assertion ID.
  struct Ops {
    bool (*create)(std::string_view reason, std::uint32_t& assertion_id,
                   bool& active, std::string& error, void* context);
    void (*release)(std::uint32_t assertion_id, void* context) noexcept;
    void* context{nullptr};
  };

  SleepGuard() = default;
  ~SleepGuard() noexcept { Release(); }

  SleepGuard(const SleepGuard&) = delete;
  SleepGuard& operator=(const SleepGuard&) = delete;

  SleepGuard(SleepGuard&& other) noexcept;
  SleepGuard& operator=(SleepGuard&& other) noexcept;

  // Acquires a PreventSystemSleep assertion using the platform backend (IOKit
  // on macOS, silent no-op elsewhere). Non-fatal on failure: returns an
  // inactive guard and, if `error` is non-null, writes the failure reason to
  // `*error`.
  static SleepGuard Acquire(std::string_view reason,
                            std::string* error = nullptr);

  // Acquires using a caller-supplied backend (for testing lifecycle, move
  // semantics, and failure handling on every OS).
  static SleepGuard AcquireWithOps(std::string_view reason, Ops ops,
                                   std::string* error = nullptr);

  // Whether this guard currently holds a live power assertion.
  [[nodiscard]] bool active() const noexcept { return active_; }

  // Releases the held assertion early, if any. Idempotent.
  void Release() noexcept;

 private:
  SleepGuard(std::uint32_t assertion_id, Ops ops) noexcept
      : active_{true}, assertion_id_{assertion_id}, ops_{ops} {}

  bool active_{false};
  std::uint32_t assertion_id_{0};
  Ops ops_{};
};

}  // namespace talos::launcher
