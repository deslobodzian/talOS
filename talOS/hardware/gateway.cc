#include "gateway.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "config_wire.h"
#include "gate.h"

namespace talos::hardware {
namespace {
// Whether `c` addresses exactly the configured outputs, in manifest order, and
// drives none of them: motors neutral with zero demand and feedforward, digital
// outputs off, PWM outputs zero. The bridge forwards this merge while every
// node's devices layer flushes neutral.
bool DrivesNothing(const Command& c, const Config& config) {
  if (c.count != config.motors.size() ||
      c.digital_output_count != config.digital_outputs.size() ||
      c.pwm_output_count != config.pwm_outputs.size()) {
    return false;
  }
  for (std::size_t i = 0; i < c.count; ++i) {
    const auto& r = c.motors[i];
    if (r.id != config.motors[i].id || r.mode != Mode::kNeutral || r.slot > 2 ||
        r.demand != 0 || r.feedforward_v != 0) {
      return false;
    }
  }
  for (std::size_t i = 0; i < c.digital_output_count; ++i) {
    if (c.digital_outputs[i].id != config.digital_outputs[i].id ||
        c.digital_outputs[i].value) {
      return false;
    }
  }
  for (std::size_t i = 0; i < c.pwm_output_count; ++i) {
    if (c.pwm_outputs[i].id != config.pwm_outputs[i].id ||
        c.pwm_outputs[i].output != 0) {
      return false;
    }
  }
  return true;
}
}  // namespace

Gateway::Gateway(Config config, Backend& backend, uint64_t boot_id)
    : config_{std::move(config)}, backend_{backend} {
  state_.config_id = ConfigurationId(config_);
  if (boot_id == 0) {
    throw std::invalid_argument("gateway boot ID must be nonzero");
  }
  state_.boot_id = boot_id;
  state_.epoch = 1;
  state_.motor_count = config_.motors.size();
  state_.sensor_count = config_.sensors.size();
  state_.digital_input_count = config_.digital_inputs.size();
  state_.digital_output_count = config_.digital_outputs.size();
  state_.analog_input_count = config_.analog_inputs.size();
  state_.encoder_count = config_.encoders.size();
  state_.pwm_output_count = config_.pwm_outputs.size();
  for (std::size_t i = 0; i < config_.motors.size(); ++i) {
    state_.motors[i].id = config_.motors[i].id;
  }
  for (std::size_t i = 0; i < config_.sensors.size(); ++i) {
    state_.sensors[i].id = config_.sensors[i].id;
  }
  for (std::size_t i = 0; i < config_.digital_inputs.size(); ++i) {
    state_.digital_inputs[i].id = config_.digital_inputs[i].id;
  }
  for (std::size_t i = 0; i < config_.digital_outputs.size(); ++i) {
    state_.digital_outputs[i].id = config_.digital_outputs[i].id;
  }
  for (std::size_t i = 0; i < config_.analog_inputs.size(); ++i) {
    state_.analog_inputs[i].id = config_.analog_inputs[i].id;
  }
  for (std::size_t i = 0; i < config_.encoders.size(); ++i) {
    state_.encoders[i].id = config_.encoders[i].id;
  }
  for (std::size_t i = 0; i < config_.pwm_outputs.size(); ++i) {
    state_.pwm_outputs[i].id = config_.pwm_outputs[i].id;
  }
  configured_ = backend_.Configure(config_);
  fault_ = !configured_;
  if (!configured_) config_error_ = backend_.last_config_error();
  backend_.Neutral();
  unavailable_ = backend_.UnavailableDevices();
  restore_waiting_ = !unavailable_.empty();
  restore_retry_at_ = kRestoreInitialBackoffUs;
}
Gateway::Gateway(Backend& backend, uint64_t boot_id) : backend_{backend} {
  if (boot_id == 0) {
    throw std::invalid_argument("gateway boot ID must be nonzero");
  }
  state_.boot_id = boot_id;
  state_.epoch = 1;
  configured_ = false;
  fault_ = false;
  backend_.Neutral();
}
bool Gateway::Reconfigure(const Config& config) {
  if (restoring_ || configuring_) return false;
  // Repeated pushes never touch devices, including after a restart enabled.
  if (configured_ && config == config_) {
    if (enabled_ && fault_) return false;
    fault_ = false;
    consecutive_apply_failures_ = 0;
    config_error_.reset();
    return true;
  }
  if (enabled_) return false;
  try {
    Validate(config);
  } catch (...) {
    fault_ = true;
    return false;
  }
  const auto ceiling = CheckCeilings(config);
  if (!ceiling.ok) {
    fault_ = true;
    return false;
  }
  // Neutralize outputs BEFORE changing devices so no stale setpoint survives
  // across the device change. Neutral is a nonblocking one-shot.
  backend_.Neutral();
  if (!backend_.Configure(config)) {
    configured_ = false;
    fault_ = true;
    config_error_ = backend_.last_config_error();
    backend_.Neutral();
    return false;
  }
  CommitConfiguration(config);
  return true;
}

bool Gateway::BeginReconfigure(const Config& config) {
  if (configuring_ || restoring_) return false;
  if (configured_ && config == config_) return Reconfigure(config);
  if (enabled_) return false;
  try {
    Validate(config);
  } catch (...) {
    return false;
  }
  if (!CheckCeilings(config).ok) return false;
  Invalidate();
  configured_ = false;
  healthy_ = false;
  pending_config_ = config;
  configuring_ = true;
  backend_.BeginConfigure(pending_config_);
  return true;
}

void Gateway::CommitConfiguration(const Config& config) {
  config_ = config;
  state_.config_id = ConfigurationId(config_);
  state_.motor_count = config_.motors.size();
  state_.sensor_count = config_.sensors.size();
  state_.digital_input_count = config_.digital_inputs.size();
  state_.digital_output_count = config_.digital_outputs.size();
  state_.analog_input_count = config_.analog_inputs.size();
  state_.encoder_count = config_.encoders.size();
  state_.pwm_output_count = config_.pwm_outputs.size();
  for (std::size_t i = 0; i < config_.motors.size(); ++i) {
    state_.motors[i].id = config_.motors[i].id;
  }
  for (std::size_t i = 0; i < config_.sensors.size(); ++i) {
    state_.sensors[i].id = config_.sensors[i].id;
  }
  for (std::size_t i = 0; i < config_.digital_inputs.size(); ++i) {
    state_.digital_inputs[i].id = config_.digital_inputs[i].id;
  }
  for (std::size_t i = 0; i < config_.digital_outputs.size(); ++i) {
    state_.digital_outputs[i].id = config_.digital_outputs[i].id;
  }
  for (std::size_t i = 0; i < config_.analog_inputs.size(); ++i) {
    state_.analog_inputs[i].id = config_.analog_inputs[i].id;
  }
  for (std::size_t i = 0; i < config_.encoders.size(); ++i) {
    state_.encoders[i].id = config_.encoders[i].id;
  }
  for (std::size_t i = 0; i < config_.pwm_outputs.size(); ++i) {
    state_.pwm_outputs[i].id = config_.pwm_outputs[i].id;
  }
  configured_ = true;
  fault_ = false;
  consecutive_apply_failures_ = 0;
  config_error_.reset();
  unavailable_ = backend_.UnavailableDevices();
  restore_waiting_ = !unavailable_.empty();
  restore_retry_at_ = state_.sample_time_us + kRestoreInitialBackoffUs;
  restore_backoff_us_ = kRestoreInitialBackoffUs;
  last_requests_.fill(MotorRequest{});
  motor_drives_from_.fill(0);
  position_reference_.fill(PositionReference{});
  Invalidate();
}
void Gateway::Invalidate() {
  active_ = false;
  have_sequence_ = false;
  ++state_.epoch;
  backend_.Neutral();
}
bool Gateway::DeviceUnavailable(uint16_t id) const {
  return std::find(unavailable_.begin(), unavailable_.end(), id) !=
         unavailable_.end();
}

void Gateway::MaskUnavailable() {
  for (std::size_t i = 0; i < config_.sensors.size(); ++i) {
    if (DeviceUnavailable(config_.sensors[i].id))
      state_.sensors[i].valid = false;
  }
  for (std::size_t pass = 0; pass < config_.motors.size(); ++pass) {
    for (std::size_t i = 0; i < config_.motors.size(); ++i) {
      const auto& motor = config_.motors[i];
      bool unavailable = DeviceUnavailable(motor.id) ||
                         (motor.feedback == Feedback::kRemoteCANcoder &&
                          DeviceUnavailable(motor.feedback_sensor_id));
      for (std::size_t j = 0; j < config_.motors.size(); ++j) {
        if (motor.master_id != 0 && motor.master_id == config_.motors[j].id &&
            !state_.motors[j].valid)
          unavailable = true;
      }
      if (unavailable) {
        state_.motors[i].valid = false;
        // Its frame may restart before it returns: the first sample after
        // that is a new reference, not a jump from this one.
        position_reference_[i].valid = false;
      }
    }
  }
  // A command observed no later than a State in which a motor could not
  // drive never drives it, whatever its feedback shows when it arrives.
  for (std::size_t i = 0; i < config_.motors.size(); ++i) {
    if (!MayDrive(i)) motor_drives_from_[i] = state_.sample_time_us + 1;
  }
}

bool Gateway::FeedbackValid(std::size_t i) const {
  const auto& motor = config_.motors[i];
  if (!state_.motors[i].valid) return false;
  if (motor.feedback != Feedback::kRemoteCANcoder) return true;
  for (std::size_t j = 0; j < state_.sensor_count; ++j) {
    if (state_.sensors[j].id == motor.feedback_sensor_id)
      return state_.sensors[j].valid;
  }
  return false;
}

bool Gateway::MayDrive(std::size_t i) const {
  if (!FeedbackValid(i)) return false;
  const auto master_id = config_.motors[i].master_id;
  if (master_id == 0) return true;
  // Config refuses follower chains, so a master is never a follower.
  for (std::size_t j = 0; j < config_.motors.size(); ++j) {
    if (config_.motors[j].id == master_id) return FeedbackValid(j);
  }
  return false;
}

bool Gateway::ApplyMotors(std::span<const MotorRequest> requests) {
  if (!backend_.Apply(requests)) return false;
  const auto failed = backend_.ApplyFailedDevices();
  // An empty motor configuration still allows independent DIO/PWM owners.
  return requests.empty() ||
         !std::all_of(requests.begin(), requests.end(), [&](const auto& r) {
           return std::find(failed.begin(), failed.end(), r.id) != failed.end();
         });
}

void Gateway::NeutralUnavailable() {
  // A neutral pass may expose another failing motor; each extra pass isolates
  // at least one more configured motor, so this is bounded by the motor count.
  while (true) {
    for (std::size_t i = 0; i < config_.motors.size(); ++i) {
      if (!MayDrive(i)) {
        last_requests_[i] = MotorRequest{config_.motors[i].id, Mode::kNeutral};
      }
    }
    if (!active_) return;
    if (!ApplyMotors({last_requests_.data(), config_.motors.size()})) {
      ++state_.counters.apply_failure_count;
      fault_ = ++consecutive_apply_failures_ >= kConsecutiveApplyFailureLimit;
      Invalidate();
      return;
    }
    if (!IsolateFailedMotors()) return;
    MaskUnavailable();
  }
}

bool Gateway::IsolateFailedMotors() {
  bool isolated = false;
  for (const auto id : backend_.ApplyFailedDevices()) {
    const bool motor =
        std::any_of(config_.motors.begin(), config_.motors.end(),
                    [id](const MotorConfig& m) { return m.id == id; });
    if (!motor || DeviceUnavailable(id)) continue;
    // Same quarantine as a device reset: masked, dependents neutral, restored
    // with backoff, and only a post-isolation observation re-authorizes it.
    std::erase(restoring_ids_, id);
    unavailable_.push_back(id);
    isolated = true;
  }
  if (isolated) {
    restore_waiting_ = true;
    restore_retry_at_ = std::max(restore_retry_at_, state_.sample_time_us);
  }
  return isolated;
}

void Gateway::PollResets() {
  if (!configured_) return;
  if (restoring_) {
    const auto result = backend_.PollRestore();
    if (result != Backend::RestoreStatus::kPending) {
      restoring_ = false;
      if (result == Backend::RestoreStatus::kDone) {
        const auto remaining = backend_.UnavailableDevices();
        std::erase_if(unavailable_, [&](uint16_t id) {
          return std::find(restoring_ids_.begin(), restoring_ids_.end(), id) !=
                     restoring_ids_.end() &&
                 std::find(remaining.begin(), remaining.end(), id) ==
                     remaining.end();
        });
        for (auto id : remaining)
          if (!DeviceUnavailable(id)) unavailable_.push_back(id);
      }
      restore_waiting_ = !unavailable_.empty();
      restore_retry_at_ = state_.sample_time_us + restore_backoff_us_;
      restore_backoff_us_ =
          std::min(restore_backoff_us_ * 2, kRestoreMaxBackoffUs);
    }
  }
  // Keep detecting new resets even while a previous device is being restored.
  std::vector<uint16_t> reset_ids;
  if (!backend_.PollDeviceResets(reset_ids)) reset_ids.clear();
  FindPositionJumps(reset_ids);
  if (!reset_ids.empty()) {
    state_.counters.device_reset_count += reset_ids.size();
    if (unavailable_.empty()) restore_backoff_us_ = kRestoreInitialBackoffUs;
    for (auto id : reset_ids) {
      std::erase(restoring_ids_, id);
      if (!DeviceUnavailable(id)) unavailable_.push_back(id);
    }
    restore_waiting_ = true;
    restore_retry_at_ = state_.sample_time_us;
  }
  MaskUnavailable();
  // A motor that cannot drive goes neutral in this tick, not at the next
  // command: between commands the backend keeps the last request it sent.
  bool neutralize = !reset_ids.empty();
  for (std::size_t i = 0; i < config_.motors.size(); ++i) {
    neutralize = neutralize ||
                 (last_requests_[i].mode != Mode::kNeutral && !MayDrive(i));
  }
  if (neutralize) NeutralUnavailable();
  if (!restoring_ && restore_waiting_ &&
      state_.sample_time_us >= restore_retry_at_) {
    restore_waiting_ = false;
    restoring_ = true;
    ++restore_attempt_count_;
    restoring_ids_ = unavailable_;
    backend_.BeginRestoreAfterReset(restoring_ids_);
  }
}

// A TalonFX reset restarts its rotor position at zero wherever the mechanism
// is, and Phoenix can deliver the restarted position frames before
// HasResetOccurred() reports the reset. A rotor-feedback motor (always a
// "rezero" position frame) whose valid position moved farther since its last
// valid sample than kMaxRotorRps allows is therefore a reset too. The
// interval runs from the earliest time the earlier sample can have been taken
// (its tick minus its age), so a held frame or a gap of invalid samples
// widens the bound instead of tripping it, to when this tick's feedback was
// read. A step is taken modulo kPositionSpanRot, so a wrap of Phoenix's
// fixed-point position is never a jump. Remote CANcoder motors are
// not checked: the gateway cannot tell an absolute frame from a rezero one,
// and their position can step legitimately when the TalonFX's remote sensor
// frames resume.
void Gateway::FindPositionJumps(std::vector<uint16_t>& reset_ids) {
  if (!backend_.PositionsArePhysical()) return;
  const uint64_t now = state_.sample_time_us;
  for (std::size_t i = 0; i < config_.motors.size(); ++i) {
    const auto& m = config_.motors[i];
    const auto& sample = state_.motors[i];
    auto& reference = position_reference_[i];
    // An invalid sample keeps the reference, so a reset is still caught when
    // feedback returns; a masked motor's reference is dropped when masked.
    if (m.feedback != Feedback::kRotor || DeviceUnavailable(m.id) ||
        !sample.valid || !std::isfinite(sample.position_rot) ||
        std::find(reset_ids.begin(), reset_ids.end(), m.id) !=
            reset_ids.end()) {
      continue;
    }
    if (reference.valid && read_us_ >= reference.earliest_us) {
      // Phoenix applies only SensorToMechanismRatio to rotor feedback; the
      // physics simulator divides by both ratios, faster when the rotor
      // ratio is below 1.
      const double rotor_per_mechanism =
          m.sensor_to_mechanism_ratio * std::min(1.0, m.rotor_to_sensor_ratio);
      const double interval_s =
          static_cast<double>(read_us_ - reference.earliest_us +
                              kPositionJumpSlackUs) *
          1e-6;
      const double step = std::remainder(
          sample.position_rot - reference.position_rot, kPositionSpanRot);
      if (std::abs(step) > kMaxRotorRps / rotor_per_mechanism * interval_s) {
        reset_ids.push_back(m.id);
        reference.valid = false;
        continue;
      }
    }
    reference = {
        .valid = true,
        .position_rot = sample.position_rot,
        .earliest_us = now - std::min<uint64_t>(now, sample.position_age_us)};
  }
}

// The backend judged each sample's status, value and isolation; the age bound
// is the gateway's, the same for every backend. Returns whether any configured
// motor or CAN sensor still has valid feedback (true with none configured):
// false is total loss.
bool Gateway::JudgeFeedbackAge() {
  const uint64_t bound = config_.command_timeout_us;
  const auto fresh = [bound](const auto& sample) {
    return FeedbackFresh(sample.position_age_us, bound) &&
           FeedbackFresh(sample.velocity_age_us, bound);
  };
  bool any_valid = config_.motors.empty() && config_.sensors.empty();
  for (std::size_t i = 0; i < config_.motors.size(); ++i) {
    auto& sample = state_.motors[i];
    sample.valid = sample.valid && fresh(sample);
    any_valid = any_valid || sample.valid;
  }
  for (std::size_t i = 0; i < config_.sensors.size(); ++i) {
    auto& sample = state_.sensors[i];
    sample.valid = sample.valid && fresh(sample);
    any_valid = any_valid || sample.valid;
  }
  return any_valid;
}

void Gateway::Tick(uint64_t now, bool enabled) {
  // Every tick, whether or not its State reaches the link, so a consumer
  // that sees sequence + 1 knows it missed nothing in between.
  ++state_.sequence;
  state_.sample_time_us = now;
  if (configuring_) {
    const auto status = backend_.PollConfigure();
    if (status == Backend::ConfigureStatus::kPending) {
      enabled_ = enabled;
      return;
    }
    configuring_ = false;
    if (status == Backend::ConfigureStatus::kDone) {
      CommitConfiguration(pending_config_);
    } else {
      fault_ = true;
      config_error_ = backend_.last_config_error();
      backend_.Neutral();
    }
  }
  if (enabled != enabled_) {
    if (enabled && !enabled_ &&
        consecutive_apply_failures_ >= kConsecutiveApplyFailureLimit) {
      fault_ = false;
      consecutive_apply_failures_ = 0;
    }
    enabled_ = enabled;
    Invalidate();
  }
  if (active_ && (now < accepted_at_ || now < observed_at_ ||
                  now - accepted_at_ >= config_.command_timeout_us ||
                  now - observed_at_ >= config_.command_timeout_us)) {
    Invalidate();
  }
  state_.sample_time_us = now;
  const bool was_healthy = healthy_;
  const bool read = configured_ && backend_.Read(state_);
  healthy_ = configured_ && JudgeFeedbackAge() && read;
  read_us_ = clock_ != nullptr ? std::max(now, clock_()) : now;
  // Total loss starts a new epoch whether or not a command is in force: a
  // request built from a State before it must never drive after it.
  if (!healthy_ && (was_healthy || active_)) Invalidate();
  PollResets();
  // A watchdog stop is not a one-shot: with no accepted command in force,
  // refresh neutral every tick (as disabled/fault do) so a dropped neutral is
  // retried until a fresh command supersedes it.
  if (!active_ || !enabled_ || !config_.commissioned || fault_ || !healthy_ ||
      !configured_) {
    backend_.Neutral();
  }
}

bool Gateway::Accept(const Command& c, uint64_t sequence, uint64_t now) {
  // Wrong whether or not the gateway takes commands now: another identity, an
  // expired or future observation, a replayed sequence or another manifest
  // size.
  const bool mismatched =
      c.config_id != state_.config_id || c.boot_id != state_.boot_id ||
      c.epoch != state_.epoch || c.observed_time_us > state_.sample_time_us ||
      c.observed_time_us > now ||
      now - c.observed_time_us >= config_.command_timeout_us ||
      (have_sequence_ && sequence <= state_.last_command_sequence) ||
      c.count != config_.motors.size();
  if (!configured_ || !config_.commissioned || !enabled_) {
    // While the robot cannot drive, every node flushes neutral and the bridge
    // keeps forwarding the merge. Refusing that is this gate working, not a
    // rejection: counting it would bury the real ones.
    if (mismatched || !DrivesNothing(c, config_)) {
      ++state_.counters.rejected_command_count;
    }
    return false;
  }
  if (fault_ || configuring_ || !healthy_ || mismatched) {
    ++state_.counters.rejected_command_count;
    return false;
  }
  // Structure errors reject the transaction; range/feedback errors isolate a
  // motor.
  std::array<MotorRequest, kMaxMotors> requests{};
  for (std::size_t i = 0; i < c.count; ++i) {
    const auto& r = c.motors[i];
    const auto& m = config_.motors[i];
    if (r.id != m.id || r.slot > 2 || r.mode > Mode::kMotionMagic) {
      ++state_.counters.rejected_command_count;
      return false;
    }
    requests[i] = r;
    const bool feedback_valid =
        FeedbackValid(i) && c.observed_time_us >= motor_drives_from_[i];
    const bool invalid =
        (r.mode != Mode::kNeutral && !feedback_valid) ||
        !std::isfinite(r.demand) || !std::isfinite(r.feedforward_v) ||
        std::abs(r.feedforward_v) > m.max_voltage ||
        (r.mode == Mode::kDutyCycle && std::abs(r.demand) > 1) ||
        (r.mode == Mode::kVoltage && std::abs(r.demand) > m.max_voltage) ||
        (r.mode == Mode::kVelocity &&
         std::abs(r.demand) > m.max_velocity_rps) ||
        ((r.mode == Mode::kPosition || r.mode == Mode::kMotionMagic) &&
         m.soft_limits &&
         (r.demand < m.reverse_limit_rot || r.demand > m.forward_limit_rot)) ||
        ((r.mode == Mode::kNeutral || r.mode == Mode::kDutyCycle ||
          r.mode == Mode::kVoltage) &&
         r.feedforward_v != 0);
    if (invalid) requests[i] = MotorRequest{m.id, Mode::kNeutral, 0, 0, 0};
  }
  // Resolve followers after every master has been sanitized, independent of
  // manifest order. Invalid local/remote feedback or an invalid master demand
  // must neutralize its followers in the same transaction.
  for (std::size_t i = 0; i < c.count; ++i) {
    const auto master_id = config_.motors[i].master_id;
    if (master_id == 0) continue;
    const bool valid =
        FeedbackValid(i) && c.observed_time_us >= motor_drives_from_[i];
    for (std::size_t j = 0; j < c.count; ++j) {
      if (config_.motors[j].id != master_id) continue;
      // Backends follow the master's output only for a non-neutral follower
      // request. Independent follower demands remain ignored.
      requests[i] = MotorRequest{config_.motors[i].id,
                                 valid ? requests[j].mode : Mode::kNeutral};
      break;
    }
  }
  if (c.digital_output_count != config_.digital_outputs.size() ||
      c.pwm_output_count != config_.pwm_outputs.size()) {
    ++state_.counters.rejected_command_count;
    return false;
  }
  for (std::size_t i = 0; i < c.digital_output_count; ++i) {
    if (c.digital_outputs[i].id != config_.digital_outputs[i].id) {
      ++state_.counters.rejected_command_count;
      return false;
    }
  }
  for (std::size_t i = 0; i < c.pwm_output_count; ++i) {
    const auto& p = c.pwm_outputs[i];
    if (p.id != config_.pwm_outputs[i].id || !std::isfinite(p.output) ||
        p.output < -1.0 || p.output > 1.0) {
      ++state_.counters.rejected_command_count;
      return false;
    }
  }
  for (std::size_t i = 0; i < c.count; ++i) {
    if (requests[i].mode == Mode::kNeutral && requests[i] != c.motors[i])
      ++state_.counters.neutralized_motor_count;
  }
  if (!ApplyMotors({requests.data(), c.count}) ||
      !backend_.ApplyOutputs({c.digital_outputs.data(), c.digital_output_count},
                             {c.pwm_outputs.data(), c.pwm_output_count})) {
    ++state_.counters.apply_failure_count;
    fault_ = ++consecutive_apply_failures_ >= kConsecutiveApplyFailureLimit;
    Invalidate();
    return false;
  }
  last_requests_ = requests;
  active_ = true;
  have_sequence_ = true;
  accepted_at_ = now;
  observed_at_ = c.observed_time_us;
  state_.last_command_sequence = sequence;
  // Device-local send failures quarantine only the failed chain; the rest of
  // this command stays applied.
  if (IsolateFailedMotors()) {
    MaskUnavailable();
    NeutralUnavailable();
  }
  if (active_) consecutive_apply_failures_ = 0;
  return active_;
}
State Gateway::snapshot() const {
  State s = state_;
  s.flags = (configured_ && config_.commissioned ? kConfigured : 0u) |
            (configured_ && enabled_ ? kEnabled : 0u) |
            (active_ ? kCommandActive : 0u) |
            (fault_ || (!healthy_ && configured_) ? kHardwareFault : 0u);
  return s;
}
}  // namespace talos::hardware
