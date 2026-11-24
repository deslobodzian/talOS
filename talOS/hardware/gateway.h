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
// Leaves no non-finite value in `state`'s configured samples, the gateway's
// rule for every backend (Gateway::Tick, after Backend::Read),
// testing::SimHardware, and Encode(const State&, ...) (messages.h). A motor or
// CAN sensor whose position or velocity is not finite is invalid (MarkInvalid);
// a motor's non-finite voltage, current or temperature is 0 and does not by
// itself change its validity; an analog input, encoder or PWM output sample
// with one is invalid with 0; a non-finite battery voltage is 0 with the
// health sample invalid; and a CAN bus slot with a non-finite utilization is
// a failed read: utilization 0, a kOk or kPending read state kFailed, its
// counters kept.
void ReplaceNonFinite(State& state);

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
  // gate.h), and replaces any non-finite value Read leaves, whatever the
  // backend (ReplaceNonFinite, above). Return false for a failed read pass or
  // total motor/CAN-sensor feedback loss, not an individual dropout. With no
  // such devices configured, return true. An invalid sample gates only the
  // affected motor and its followers and dependents: neutral from the Tick that
  // finds it, and driven again only by a command observing a later State. Read
  // also fills `state.health.can_bus`, the health of CAN bus
  // `state.health.can_bus_index` (CanBuses order), or leaves it kNone; the
  // gateway sets the index and count and clears the slot before each Read.
  // Total loss is judged on the State as published (AnyFeedbackValid,
  // feedback.h): after the age bound and after the gateway's own masks (an
  // unavailable, restoring or send-failed device, a follower or
  // remote-feedback motor of one, a chain whose restore ends that tick). A
  // reset that masks every motor and CAN sensor is total loss until its
  // restore ends, whatever Read reports for a restoring device (PhoenixBackend
  // and both simulated backends read a restoring motor invalid and a
  // restoring CAN sensor valid).
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
  // Each PWM request, in config order, is off (PwmMode::kOff: stop the
  // output's pulse) or a pulse (kPulse: `output` in [-1, 1], 0 the centre);
  // the gateway has refused any other. Neutral() turns every PWM output off.
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
  // neutralizes only affected motors and their dependents before restoring
  // (every output, when the masks leave no valid motion sample: total loss);
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
  // The clock Tick's `now` comes from. Read again after Backend::Read, it ends
  // the position-jump window when the feedback was read rather than when the
  // tick began (so a loop preempted in between does not see more motion than
  // its window allows), re-checks active command expiry after a stalled Read,
  // and ages Accept() observations against the current clock. Without one
  // (explicit simulated time) each check uses `now`.
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
  // CANcoder feedback neutralize only that motor. Total loss (no valid motor or
  // CAN-sensor sample in the State Tick publishes, its masks included)
  // neutralizes all outputs, refuses every command and invalidates
  // outstanding commands by advancing the epoch; it is not latched, and the
  // first State with a valid sample clears it. A command observing a State
  // of the loss drives nothing after it clears: its motors neutral, digital
  // outputs false, PWM outputs off. A device-local send failure
  // (ApplyFailedDevices) isolates only that motor's chain and the command is
  // still accepted, unless the isolation leaves no valid sample, which is
  // total loss: Accept then returns false. Global apply failures
  // (Apply/ApplyOutputs false, or every motor failing) invalidate commands and
  // retry until the consecutive limit; re-enabling or any successful
  // Reconfigure clears the apply-failure latch.
  bool Accept(const Command& command, uint64_t sequence, uint64_t now_us);
  // The State to publish. Every invalid motor or CAN-sensor sample carries
  // zero measurements (MarkInvalid), and no floating field is non-finite
  // (ReplaceNonFinite); while unconfigured (configuring, or the last configure
  // failed) every sample is invalid, since no Read ran.
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
  // The last command expiry: a Tick that found the command in force at or
  // past command_expiry_us() (or its clock behind that command) and
  // invalidated it. `count` is the expiries since construction, `neutral_us`
  // that Tick's time, and `observed_us` and `accepted_us` the expired
  // command's observation and acceptance, all on Tick's clock. Reporting
  // only (the RoboRIO's console line); nothing in the gateway reads it.
  struct CommandExpiry {
    uint64_t count{};
    uint64_t neutral_us{};
    uint64_t observed_us{};
    uint64_t accepted_us{};
  };
  const CommandExpiry& last_command_expiry() const {
    return last_command_expiry_;
  }
  // The last changed configuration refused because the gateway was enabled
  // (Reconfigure, BeginReconfigure): `count` such refusals since
  // construction, `refused_id` its ConfigurationId (0 when it would not
  // validate) and `active_id` the configuration in force then (0 with
  // none). Reporting only (the RoboRIO's console line); nothing in the
  // gateway reads it.
  struct EnabledRefusal {
    uint64_t count{};
    uint64_t refused_id{};
    uint64_t active_id{};
  };
  const EnabledRefusal& last_enabled_refusal() const {
    return last_enabled_refusal_;
  }
  // Precise failure behind the last rejected configuration, if the
  // backend could say more than "it failed".
  const std::optional<ConfigError>& config_error() const {
    return config_error_;
  }
  uint32_t restore_attempt_count() const { return restore_attempt_count_; }

 private:
  void Invalidate();
  void ExpireCommand(uint64_t at_us);
  void RecordEnabledRefusal(const Config& refused);
  // This tick's CAN bus slot: index `sequence % count` of the applied
  // configuration's buses (state_.health.can_bus_count, set only where a
  // configuration is committed), cleared for Backend::Read to fill. Called
  // once the tick knows its configuration: after a commit in the tick, or
  // before returning while a configure is pending.
  void PlaceCanBusSlot();
  bool JudgeFeedbackAge();
  // Total loss on this tick's State after the gateway's masks
  // (AnyFeedbackValid): healthy_ false and, when `was_healthy` or a command
  // is in force, Invalidate. While healthy_ is false, moves
  // outputs_drive_from_ past this tick's State.
  void JudgeMaskedLoss(bool was_healthy);
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
  bool unattempted_restore_{false};
  uint64_t restore_retry_at_{};
  bool configuring_{false};
  Config pending_config_;
  FixedVector<uint16_t, kMaxMotors + kMaxSensors> unavailable_;
  FixedVector<uint16_t, kMaxMotors + kMaxSensors> restoring_ids_;
  std::array<MotorRequest, kMaxMotors> last_requests_{};
  // Per motor, the earliest observation time a command may drive it on: one
  // past the last State in which it could not drive (MayDrive false), the
  // State of the tick its restore ended included (PollResets masks it).
  std::array<uint64_t, kMaxMotors> motor_drives_from_{};
  // The earliest observation time a command drives any output on: one past
  // the last State of total loss (healthy_ false), whatever samples the
  // backend left valid in it. Before it its motors are sent neutral, digital
  // outputs false and PWM outputs off.
  uint64_t outputs_drive_from_{};
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
  CommandExpiry last_command_expiry_{};
  EnabledRefusal last_enabled_refusal_{};
  std::optional<ConfigError> config_error_;
};
}  // namespace talos::hardware
