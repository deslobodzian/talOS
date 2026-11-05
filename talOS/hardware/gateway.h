#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "config.h"
#include "config_error.h"
#include "messages.h"

namespace talos::hardware {
// Platform-specific code owns device objects. Configure runs at startup or
// disabled and may block; Read/Apply/Neutral and reset polling must not block.
class Backend {
 public:
  virtual ~Backend() = default;
  virtual bool Configure(const Config&) = 0;
  virtual bool Read(State&) = 0;
  virtual bool Apply(std::span<const MotorRequest>) = 0;
  virtual bool ApplyOutputs(
      std::span<const DigitalOutputRequest> /*digital_outputs*/,
      std::span<const PwmRequest> /*pwm_outputs*/) {
    return true;
  }
  virtual void Neutral() = 0;
  // Precise failure behind the last false Configure(). Absent when
  // the backend cannot say more than "it failed".
  virtual std::optional<ConfigError> last_config_error() const {
    return std::nullopt;
  }
  // Individual device resets. Returns true and fills reset_ids with
  // the logical IDs of devices that reset since the last poll. The gateway
  // invalidates outstanding commands (epoch bump plus Neutral) BEFORE
  // restoring configuration, so no obsolete demand resumes on a freshly
  // reset device.
  virtual bool PollDeviceResets(std::vector<uint16_t>& reset_ids) {
    reset_ids.clear();
    return false;
  }
  enum class RestoreStatus { kPending, kDone, kFailed };
  // Start nonblocking restoration; PollRestore never waits for device traffic.
  // Failed restoration is retried by Gateway with capped exponential backoff.
  virtual void BeginRestoreAfterReset() {}
  virtual RestoreStatus PollRestore() { return RestoreStatus::kDone; }
};

class Gateway {
 public:
  static constexpr uint32_t kConsecutiveApplyFailureLimit = 3;
  static constexpr uint64_t kRestoreInitialBackoffUs = 250000;
  static constexpr uint64_t kRestoreMaxBackoffUs = 2000000;
  Gateway(Config config, Backend& backend, uint64_t boot_id);
  Gateway(Backend& backend, uint64_t boot_id);
  // Replaces the active hardware configuration. Safe-reconfiguration rules:
  // - Changed configurations require disabled operation.
  // - Identical pushes succeed even while enabled without device traffic or
  //   an epoch change, and clear the apply-failure latch.
  // - Neutralizes outputs via Backend::Neutral() BEFORE Backend::Configure()
  //   so no stale setpoint survives a device change; a successful change
  //   invalidates outstanding commands by advancing the epoch.
  // Backend::Configure() may block — Phoenix configurator Apply calls are
  // synchronous CAN transactions (see
  // https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/configuration.html)
  // — so callers must keep Reconfigure off the Driver Station fast path. The
  // disabled/idempotent/validation/ceiling rejects above never block; only a
  // genuine disabled-side change blocks.
  bool Reconfigure(const Config& config);
  void Tick(uint64_t now_us, bool driver_station_enabled);
  // Structural errors reject all outputs; range errors neutralize one motor.
  // Apply failures invalidate commands and retry until the consecutive limit;
  // re-enabling or any successful Reconfigure clears the apply-failure latch.
  bool Accept(const Command& command, uint64_t sequence, uint64_t now_us);
  State snapshot() const;
  const Config& config() const { return config_; }
  bool configured() const { return configured_; }
  // Precise failure behind the last rejected configuration, if the
  // backend could say more than "it failed".
  std::optional<ConfigError> config_error() const { return config_error_; }
  uint32_t restore_attempt_count() const { return restore_attempt_count_; }

 private:
  void Invalidate();
  void PollResets();
  Config config_;
  Backend& backend_;
  State state_{};
  bool configured_{false}, enabled_{false}, active_{false}, fault_{false};
  bool have_sequence_{false};
  bool healthy_{false};
  bool restoring_{false};
  bool restore_waiting_{false};
  uint64_t restore_retry_at_{};
  uint64_t restore_backoff_us_{kRestoreInitialBackoffUs};
  uint32_t restore_attempt_count_{};
  uint32_t consecutive_apply_failures_{};
  uint64_t accepted_at_{}, observed_at_{};
  std::optional<ConfigError> config_error_;
};
}  // namespace talos::hardware
