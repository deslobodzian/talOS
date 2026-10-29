#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <vector>

#include "config.h"
#include "config_error.h"
#include "messages.h"

namespace talos::hardware {
// Platform-specific code owns device objects. Configure is startup-only and
// may block; Read/Apply must use cached or nonblocking device APIs.
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
  // Re-applies configuration after a reset. Returns false on failure. The
  // gateway neutralizes and invalidates before calling; on false it latches
  // a fault and neutralizes again.
  virtual bool RestoreAfterReset() { return true; }
};

class Gateway {
 public:
  Gateway(Config config, Backend& backend, uint64_t boot_id);
  Gateway(Backend& backend, uint64_t boot_id);
  // Replaces the active hardware configuration. Safe-reconfiguration rules:
  // - Disabled only: returns false while enabled (see Tick) without touching
  //   devices or latched fault/configured state.
  // - Idempotent: a push identical to the active configuration is a no-op
  //   that succeeds without device traffic or an epoch change.
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
  bool Accept(const Command& command, uint64_t sequence, uint64_t now_us);
  State snapshot() const;
  const Config& config() const { return config_; }
  bool configured() const { return configured_; }
  // Precise failure behind the last rejected configuration, if the
  // backend could say more than "it failed".
  std::optional<ConfigError> config_error() const { return config_error_; }

 private:
  void Invalidate();
  void PollResets();
  Config config_;
  Backend& backend_;
  State state_{};
  bool configured_{false}, enabled_{false}, active_{false}, fault_{false};
  bool have_sequence_{false};
  bool healthy_{false};
  uint64_t accepted_at_{}, observed_at_{};
  std::optional<ConfigError> config_error_;
};
}  // namespace talos::hardware
