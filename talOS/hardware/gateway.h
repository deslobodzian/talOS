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
  enum class ConfigureStatus { kPending, kDone, kFailed };
  // The default is suitable only for bounded, nonblocking backends.
  virtual void BeginConfigure(const Config& config) {
    configure_result_ = Configure(config);
  }
  virtual ConfigureStatus PollConfigure() {
    return configure_result_ ? ConfigureStatus::kDone
                             : ConfigureStatus::kFailed;
  }
  // A successful Configure may leave individual devices unavailable.
  virtual std::vector<uint16_t> UnavailableDevices() const { return {}; }
  // Return false for a failed read pass or total motor/CAN-sensor feedback
  // loss, not an individual dropout. With no such devices configured, return
  // true. Individual sample validity gates only the affected motor in Accept().
  virtual bool Read(State&) = 0;
  // Followers ignore demand values, but kNeutral explicitly gates their output.
  // Gateway resolves a live follower's mode from its sanitized master.
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
  // neutralizes only affected motors and their dependents before restoring;
  // their old observations cannot authorize motion after recovery.
  virtual bool PollDeviceResets(std::vector<uint16_t>& reset_ids) {
    reset_ids.clear();
    return false;
  }
  enum class RestoreStatus { kPending, kDone, kFailed };
  // Start nonblocking restoration; PollRestore never waits for device traffic.
  // Failed restoration is retried by Gateway with capped exponential backoff.
  virtual void BeginRestoreAfterReset(std::span<const uint16_t>) {}
  virtual RestoreStatus PollRestore() { return RestoreStatus::kDone; }

 private:
  bool configure_result_{false};
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
  // Nonblocking configuration path used by Endpoint. Completion is polled on
  // Tick; no backend reads or writes race device construction.
  bool BeginReconfigure(const Config& config);
  bool configuring() const { return configuring_; }
  void Tick(uint64_t now_us, bool driver_station_enabled);
  // Structural errors reject all outputs; range errors or invalid motor/remote
  // CANcoder feedback neutralize only that motor. Total backend loss
  // neutralizes all outputs and invalidates outstanding commands by advancing
  // the epoch. Apply failures invalidate commands and retry until the
  // consecutive limit; re-enabling or any successful Reconfigure clears the
  // apply-failure latch.
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
  bool DeviceUnavailable(uint16_t id) const;
  void MaskUnavailable();
  void NeutralUnavailable();
  void CommitConfiguration(const Config& config);
  Config config_;
  Backend& backend_;
  State state_{};
  bool configured_{false}, enabled_{false}, active_{false}, fault_{false};
  bool have_sequence_{false};
  bool healthy_{false};
  bool restoring_{false};
  bool restore_waiting_{false};
  uint64_t restore_retry_at_{};
  bool configuring_{false};
  Config pending_config_;
  std::vector<uint16_t> unavailable_;
  std::vector<uint16_t> restoring_ids_;
  std::array<MotorRequest, kMaxMotors> last_requests_{};
  std::array<uint64_t, kMaxMotors> motor_reset_at_{};
  uint64_t restore_backoff_us_{kRestoreInitialBackoffUs};
  uint32_t restore_attempt_count_{};
  uint32_t consecutive_apply_failures_{};
  uint64_t accepted_at_{}, observed_at_{};
  std::optional<ConfigError> config_error_;
};
}  // namespace talos::hardware
