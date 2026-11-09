#include "gateway.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

#include "config_wire.h"

namespace talos::hardware {
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
  motor_reset_at_.fill(0);
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
        motor_reset_at_[i] = state_.sample_time_us;
      }
    }
  }
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
      if (!state_.motors[i].valid) {
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
  if (backend_.PollDeviceResets(reset_ids)) {
    state_.counters.device_reset_count += reset_ids.size();
    if (unavailable_.empty()) restore_backoff_us_ = kRestoreInitialBackoffUs;
    for (auto id : reset_ids) {
      std::erase(restoring_ids_, id);
      if (!DeviceUnavailable(id)) unavailable_.push_back(id);
    }
    if (!reset_ids.empty()) {
      restore_waiting_ = true;
      restore_retry_at_ = state_.sample_time_us;
    }
  }
  MaskUnavailable();
  if (!reset_ids.empty()) NeutralUnavailable();
  if (!restoring_ && restore_waiting_ &&
      state_.sample_time_us >= restore_retry_at_) {
    restore_waiting_ = false;
    restoring_ = true;
    ++restore_attempt_count_;
    restoring_ids_ = unavailable_;
    backend_.BeginRestoreAfterReset(restoring_ids_);
  }
}

void Gateway::Tick(uint64_t now, bool enabled) {
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
  healthy_ = configured_ && backend_.Read(state_);
  if (!healthy_ && active_) Invalidate();
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
  if (!configured_ || !config_.commissioned || !enabled_ || fault_ ||
      configuring_ || !healthy_ || c.config_id != state_.config_id ||
      c.boot_id != state_.boot_id || c.epoch != state_.epoch ||
      c.observed_time_us > state_.sample_time_us || c.observed_time_us > now ||
      now - c.observed_time_us >= config_.command_timeout_us ||
      (have_sequence_ && sequence <= state_.last_command_sequence) ||
      c.count != config_.motors.size()) {
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
    bool feedback_valid =
        state_.motors[i].valid && c.observed_time_us >= motor_reset_at_[i];
    if (m.feedback == Feedback::kRemoteCANcoder) {
      bool sensor_valid = false;
      for (std::size_t j = 0; j < state_.sensor_count; ++j) {
        const auto& sensor = state_.sensors[j];
        if (sensor.id == m.feedback_sensor_id) {
          sensor_valid = sensor.valid;
          break;
        }
      }
      feedback_valid &= sensor_valid;
    }
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
    bool valid =
        state_.motors[i].valid && c.observed_time_us >= motor_reset_at_[i];
    if (config_.motors[i].feedback == Feedback::kRemoteCANcoder) {
      bool sensor_valid = false;
      for (std::size_t j = 0; j < state_.sensor_count; ++j)
        if (state_.sensors[j].id == config_.motors[i].feedback_sensor_id)
          sensor_valid = state_.sensors[j].valid;
      valid &= sensor_valid;
    }
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
