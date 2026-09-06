#include "gateway.h"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "config_wire.h"

namespace talos::hardware {
Gateway::Gateway(Config config, Backend& backend, uint64_t boot_id)
    : config_{std::move(config)}, backend_{backend} {
  state_.config_id = ConfigurationId(config_);
  if (boot_id == 0)
    throw std::invalid_argument("gateway boot ID must be nonzero");
  state_.boot_id = boot_id;
  state_.epoch = 1;
  state_.motor_count = config_.motors.size();
  state_.sensor_count = config_.sensors.size();
  state_.digital_input_count = config_.digital_inputs.size();
  state_.digital_output_count = config_.digital_outputs.size();
  state_.analog_input_count = config_.analog_inputs.size();
  state_.encoder_count = config_.encoders.size();
  state_.pwm_output_count = config_.pwm_outputs.size();
  for (std::size_t i = 0; i < config_.motors.size(); ++i)
    state_.motors[i].id = config_.motors[i].id;
  for (std::size_t i = 0; i < config_.sensors.size(); ++i)
    state_.sensors[i].id = config_.sensors[i].id;
  for (std::size_t i = 0; i < config_.digital_inputs.size(); ++i)
    state_.digital_inputs[i].id = config_.digital_inputs[i].id;
  for (std::size_t i = 0; i < config_.digital_outputs.size(); ++i)
    state_.digital_outputs[i].id = config_.digital_outputs[i].id;
  for (std::size_t i = 0; i < config_.analog_inputs.size(); ++i)
    state_.analog_inputs[i].id = config_.analog_inputs[i].id;
  for (std::size_t i = 0; i < config_.encoders.size(); ++i)
    state_.encoders[i].id = config_.encoders[i].id;
  for (std::size_t i = 0; i < config_.pwm_outputs.size(); ++i)
    state_.pwm_outputs[i].id = config_.pwm_outputs[i].id;
  configured_ = backend_.Configure(config_);
  fault_ = !configured_;
  backend_.Neutral();
}
Gateway::Gateway(Backend& backend, uint64_t boot_id)
    : backend_{backend} {
  if (boot_id == 0)
    throw std::invalid_argument("gateway boot ID must be nonzero");
  state_.boot_id = boot_id;
  state_.epoch = 1;
  configured_ = false;
  fault_ = false;
  backend_.Neutral();
}
bool Gateway::Reconfigure(const Config& config) {
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
  if (!backend_.Configure(config)) {
    configured_ = false;
    fault_ = true;
    backend_.Neutral();
    return false;
  }
  config_ = config;
  state_.config_id = ConfigurationId(config_);
  state_.motor_count = config_.motors.size();
  state_.sensor_count = config_.sensors.size();
  state_.digital_input_count = config_.digital_inputs.size();
  state_.digital_output_count = config_.digital_outputs.size();
  state_.analog_input_count = config_.analog_inputs.size();
  state_.encoder_count = config_.encoders.size();
  state_.pwm_output_count = config_.pwm_outputs.size();
  for (std::size_t i = 0; i < config_.motors.size(); ++i)
    state_.motors[i].id = config_.motors[i].id;
  for (std::size_t i = 0; i < config_.sensors.size(); ++i)
    state_.sensors[i].id = config_.sensors[i].id;
  for (std::size_t i = 0; i < config_.digital_inputs.size(); ++i)
    state_.digital_inputs[i].id = config_.digital_inputs[i].id;
  for (std::size_t i = 0; i < config_.digital_outputs.size(); ++i)
    state_.digital_outputs[i].id = config_.digital_outputs[i].id;
  for (std::size_t i = 0; i < config_.analog_inputs.size(); ++i)
    state_.analog_inputs[i].id = config_.analog_inputs[i].id;
  for (std::size_t i = 0; i < config_.encoders.size(); ++i)
    state_.encoders[i].id = config_.encoders[i].id;
  for (std::size_t i = 0; i < config_.pwm_outputs.size(); ++i)
    state_.pwm_outputs[i].id = config_.pwm_outputs[i].id;
  configured_ = true;
  fault_ = false;
  Invalidate();
  return true;
}
void Gateway::Invalidate() {
  active_ = false;
  have_sequence_ = false;
  ++state_.epoch;
  backend_.Neutral();
}
void Gateway::Tick(uint64_t now, bool enabled) {
  if (enabled != enabled_) {
    enabled_ = enabled;
    Invalidate();
  }
  if (active_ && (now < accepted_at_ || now < observed_at_ ||
                  now - accepted_at_ >= config_.command_timeout_us ||
                  now - observed_at_ >= config_.command_timeout_us))
    Invalidate();
  state_.sample_time_us = now;
  healthy_ = configured_ && backend_.Read(state_);
  if (!healthy_ && active_) Invalidate();
  if (!enabled_ || !config_.commissioned || fault_ || !healthy_ || !configured_)
    backend_.Neutral();
}
bool Gateway::Accept(const Command& c, uint64_t sequence, uint64_t now) {
  if (!configured_ || !config_.commissioned || !enabled_ || fault_ ||
      !healthy_ || c.config_id != state_.config_id ||
      c.boot_id != state_.boot_id || c.epoch != state_.epoch ||
      c.observed_time_us > state_.sample_time_us || c.observed_time_us > now ||
      now - c.observed_time_us >= config_.command_timeout_us ||
      (have_sequence_ && sequence <= state_.last_command_sequence) ||
      c.count != config_.motors.size())
    return false;
  // Validate the complete transaction before touching any actuator.
  for (std::size_t i = 0; i < c.count; ++i) {
    const auto& r = c.motors[i];
    const auto& m = config_.motors[i];
    if (r.id != m.id || r.slot > 2 || r.mode > Mode::kMotionMagic ||
        !std::isfinite(r.demand) || !std::isfinite(r.feedforward_v) ||
        std::abs(r.feedforward_v) > m.max_voltage)
      return false;
    if (r.mode == Mode::kDutyCycle && std::abs(r.demand) > 1) return false;
    if (r.mode == Mode::kVoltage && std::abs(r.demand) > m.max_voltage)
      return false;
    if (r.mode == Mode::kVelocity && std::abs(r.demand) > m.max_velocity_rps)
      return false;
    if ((r.mode == Mode::kPosition || r.mode == Mode::kMotionMagic) &&
        m.soft_limits &&
        (r.demand < m.reverse_limit_rot || r.demand > m.forward_limit_rot))
      return false;
    if ((r.mode == Mode::kNeutral || r.mode == Mode::kDutyCycle ||
         r.mode == Mode::kVoltage) &&
        r.feedforward_v != 0)
      return false;
  }
  if (c.digital_output_count != config_.digital_outputs.size() ||
      c.pwm_output_count != config_.pwm_outputs.size())
    return false;
  for (std::size_t i = 0; i < c.digital_output_count; ++i) {
    if (c.digital_outputs[i].id != config_.digital_outputs[i].id)
      return false;
  }
  for (std::size_t i = 0; i < c.pwm_output_count; ++i) {
    const auto& p = c.pwm_outputs[i];
    if (p.id != config_.pwm_outputs[i].id || !std::isfinite(p.output) ||
        p.output < -1.0 || p.output > 1.0)
      return false;
  }
  if (!backend_.Apply({c.motors.data(), c.count}) ||
      !backend_.ApplyOutputs({c.digital_outputs.data(), c.digital_output_count},
                             {c.pwm_outputs.data(), c.pwm_output_count})) {
    fault_ = true;
    Invalidate();
    return false;
  }
  active_ = true;
  have_sequence_ = true;
  accepted_at_ = now;
  observed_at_ = c.observed_time_us;
  state_.last_command_sequence = sequence;
  return true;
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
