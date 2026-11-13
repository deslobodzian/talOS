#pragma once

#include <cstddef>
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
  // A backend marks a sample invalid only for a signal error, a non-finite
  // value or an isolated device, and reports each sample's ages. The gateway
  // invalidates a motor or CAN-sensor sample whose position or velocity age is
  // at least `command_timeout_us` (UINT32_MAX: unknown; FeedbackFresh in
  // gate.h) and treats every configured motor and CAN sensor invalid as total
  // loss. Return false for a failed read pass or total motor/CAN-sensor
  // feedback loss, not an individual dropout. With no such devices configured,
  // return true. An invalid sample gates only the affected motor and its
  // followers and dependents: neutral from the Tick that finds it, and driven
  // again only by a command observing a later State.
  virtual bool Read(State&) = 0;
  // Followers ignore demand values, but kNeutral explicitly gates their output.
  // Gateway resolves a live follower's mode from its sanitized master.
  // Return false only for a global failure (no motor output can be trusted).
  // A send failure on an individual motor is device-local: try neutral on that
  // motor, keep applying the rest, return true and report it through
  // ApplyFailedDevices().
  virtual bool Apply(std::span<const MotorRequest>) = 0;
  // Logical motor IDs whose control send failed during the last Apply() that
  // returned true; valid until the next Apply(). Gateway isolates them (and
  // their dependents) exactly like unavailable devices and restores them
  // through BeginRestoreAfterReset(). Backends whose sends cannot fail per
  // device return an empty span.
  virtual std::span<const uint16_t> ApplyFailedDevices() const = 0;
  virtual bool ApplyOutputs(
      std::span<const DigitalOutputRequest> /*digital_outputs*/,
      std::span<const PwmRequest> /*pwm_outputs*/) {
    return true;
  }
  // Called every Tick while no accepted command is active (and on every
  // safe-state transition); a dropped neutral is retried on the next Tick.
  virtual void Neutral() = 0;
  // Precise failure behind the last false Configure(). Absent when
  // the backend cannot say more than "it failed".
  virtual std::optional<ConfigError> last_config_error() const {
    return std::nullopt;
  }
  // Individual device resets. Returns true and fills reset_ids with
  // the logical IDs of devices that reset since the last poll. The gateway
  // neutralizes only affected motors and their dependents before restoring;
  // their old observations cannot authorize motion after recovery. It masks
  // the restored chain in every State read before PollRestore reports the
  // restore ended, that tick's included, whatever Read reported for it, so
  // only a command observing a State read after the restore drives it.
  virtual bool PollDeviceResets(std::vector<uint16_t>& reset_ids) {
    reset_ids.clear();
    return false;
  }
  enum class RestoreStatus { kPending, kDone, kFailed };
  // Start nonblocking restoration; PollRestore never waits for device traffic.
  // Failed restoration is retried by Gateway with capped exponential backoff.
  virtual void BeginRestoreAfterReset(std::span<const uint16_t>) {}
  virtual RestoreStatus PollRestore() { return RestoreStatus::kDone; }
  // Whether a valid motor position moves only as fast as the motor turns it,
  // so the gateway can read a larger step as a reset (Gateway::kMaxRotorRps).
  // The ideal SimBackend, whose position demands apply instantly, says no.
  virtual bool PositionsArePhysical() const { return true; }

 private:
  bool configure_result_{false};
};

class Gateway {
 public:
  static constexpr uint32_t kConsecutiveApplyFailureLimit = 3;
  static constexpr uint64_t kRestoreInitialBackoffUs = 250000;
  static constexpr uint64_t kRestoreMaxBackoffUs = 2000000;
  // A rotor-feedback motor's valid position steps between samples by at most
  // this rotor speed over their interval plus kPositionJumpSlackUs; a larger
  // step is a reset (PollResets). Twice the fastest TalonFX motor's free
  // speed (Kraken X44, about 125 rps at 12 V), leaving room for a charged
  // battery and a backdriven rotor.
  static constexpr double kMaxRotorRps = 250;
  // Frame timestamp error, CAN queueing and tick jitter.
  static constexpr uint64_t kPositionJumpSlackUs = 20000;
  // Phoenix 6 reports Position as fixed point in [-16384, 16384) of its own
  // units, so a long spin can wrap it; steps are compared modulo this span.
  static constexpr double kPositionSpanRot = 32768;
  // The clock Tick's `now` comes from. Read again after Backend::Read, it
  // ends the position-jump window when the feedback was read rather than when
  // the tick began, so a loop preempted in between does not see more motion
  // than its window allows. Without one (explicit simulated time) the window
  // ends at `now`.
  void SetClock(uint64_t (*clock)()) { clock_ = clock; }
  Gateway(Config config, Backend& backend, uint64_t boot_id);
  Gateway(Backend& backend, uint64_t boot_id);
  // The program and build every State of this boot reports. Identification
  // only: nothing in the gateway or the bridge acts on it.
  void SetBuild(const GatewayBuild& build) { state_.build = build; }
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
  // the epoch. A device-local send failure (ApplyFailedDevices) isolates only
  // that motor's chain and the command is still accepted. Global apply
  // failures (Apply/ApplyOutputs false, or every motor failing) invalidate
  // commands and retry until the consecutive limit; re-enabling or any
  // successful Reconfigure clears the apply-failure latch.
  bool Accept(const Command& command, uint64_t sequence, uint64_t now_us);
  // The State to publish. Every invalid motor or CAN-sensor sample carries
  // zero measurements (MarkInvalid); while unconfigured (configuring, or the
  // last configure failed) every sample is invalid, since no Read ran.
  State snapshot() const;
  const Config& config() const { return config_; }
  bool configured() const { return configured_; }
  // While a command is in force, the time (on Tick's clock) from which Tick
  // invalidates it: its acceptance or the State it was built from, whichever
  // is older, plus command_timeout_us. 0 while none is. The RoboRIO's loop
  // watchdog arms on it.
  uint64_t command_expiry_us() const {
    if (!active_) return 0;
    return (accepted_at_ < observed_at_ ? accepted_at_ : observed_at_) +
           config_.command_timeout_us;
  }
  // Precise failure behind the last rejected configuration, if the
  // backend could say more than "it failed".
  std::optional<ConfigError> config_error() const { return config_error_; }
  uint32_t restore_attempt_count() const { return restore_attempt_count_; }

 private:
  void Invalidate();
  bool JudgeFeedbackAge();
  void PollResets();
  void FindPositionJumps(std::vector<uint16_t>& reset_ids);
  bool DeviceUnavailable(uint16_t id) const;
  // Marks motor `i`'s sample in this tick's State invalid and drops its
  // position reference.
  void MaskMotor(std::size_t i);
  void MaskUnavailable();
  // MotorMayDrive (gate.h) in this tick's State.
  bool MayDrive(std::size_t i) const;
  void NeutralUnavailable();
  bool ApplyMotors(std::span<const MotorRequest> requests);
  bool IsolateFailedMotors();
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
  // Per motor, the earliest observation time a command may drive it on: one
  // past the last State in which it could not drive (MayDrive false), the
  // State of the tick its restore ended included (PollResets masks it).
  std::array<uint64_t, kMaxMotors> motor_drives_from_{};
  // A rotor-feedback motor's last valid position since the gateway last
  // masked it, and the earliest time that sample can have been taken.
  struct PositionReference {
    bool valid{false};
    double position_rot{};
    uint64_t earliest_us{};
  };
  std::array<PositionReference, kMaxMotors> position_reference_{};
  uint64_t (*clock_)() = nullptr;
  uint64_t read_us_{};  // When this tick's Backend::Read returned.
  uint64_t restore_backoff_us_{kRestoreInitialBackoffUs};
  uint32_t restore_attempt_count_{};
  uint32_t consecutive_apply_failures_{};
  uint64_t accepted_at_{}, observed_at_{};
  std::optional<ConfigError> config_error_;
};
}  // namespace talos::hardware
