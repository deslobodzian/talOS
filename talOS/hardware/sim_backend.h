#pragma once

#include <algorithm>
#include <unordered_map>

#include "gateway.h"

namespace talos::hardware {
// A deterministic ideal-actuator fake for integration tests, not a motor or
// tire physics model. Real hardware always uses the Phoenix backend.
class SimBackend final : public Backend {
 public:
  bool Configure(const Config& config) override {
    config_ = config;
    return true;
  }
  bool Read(State& state) override {
    const double dt =
        previous_time_ ? (state.sample_time_us - previous_time_) * 1e-6 : 0;
    previous_time_ = state.sample_time_us;
    for (std::size_t i = 0; i < config_.motors.size(); ++i) {
      auto& m = samples_[i];
      const auto& r = commands_[i];
      m.id = config_.motors[i].id;
      m.valid = true;
      m.velocity_rps = r.mode == Mode::kVelocity ? r.demand : 0;
      if (r.mode == Mode::kPosition || r.mode == Mode::kMotionMagic)
        m.position_rot = r.demand;
      else
        m.position_rot += m.velocity_rps * dt;
      m.voltage = r.mode == Mode::kVoltage ? r.demand : 0;
      state.motors[i] = m;
    }
    for (std::size_t i = 0; i < config_.sensors.size(); ++i) {
      state.sensors[i] = {config_.sensors[i].id, true, 0, 0};
      for (std::size_t j = 0; j < config_.motors.size(); ++j) {
        if (config_.motors[j].feedback_sensor_id == config_.sensors[i].id) {
          state.sensors[i].position_rot = samples_[j].position_rot;
          state.sensors[i].velocity_rps = samples_[j].velocity_rps;
        }
      }
    }
    for (std::size_t i = 0; i < config_.digital_inputs.size(); ++i) {
      const auto id = config_.digital_inputs[i].id;
      state.digital_inputs[i] = {id, true, digital_inputs_[id]};
    }
    for (std::size_t i = 0; i < config_.digital_outputs.size(); ++i) {
      const auto id = config_.digital_outputs[i].id;
      state.digital_outputs[i] = {id, true, digital_outputs_[id]};
    }
    for (std::size_t i = 0; i < config_.analog_inputs.size(); ++i) {
      const auto id = config_.analog_inputs[i].id;
      state.analog_inputs[i] = {id, true, analog_inputs_[id], 0};
    }
    for (std::size_t i = 0; i < config_.encoders.size(); ++i) {
      const auto id = config_.encoders[i].id;
      const auto& enc = encoders_[id];
      state.encoders[i] = {id, true, enc.position_rot, enc.velocity_rps, 0};
    }
    for (std::size_t i = 0; i < config_.pwm_outputs.size(); ++i) {
      const auto id = config_.pwm_outputs[i].id;
      state.pwm_outputs[i] = {id, true, pwm_outputs_[id]};
    }
    return true;
  }
  bool Apply(std::span<const MotorRequest> commands) override {
    std::copy(commands.begin(), commands.end(), commands_.begin());
    return true;
  }
  bool ApplyOutputs(std::span<const DigitalOutputRequest> digital_outputs,
                    std::span<const PwmRequest> pwm_outputs) override {
    for (const auto& d : digital_outputs) {
      digital_outputs_[d.id] = d.value;
    }
    for (const auto& p : pwm_outputs) {
      pwm_outputs_[p.id] = p.output;
    }
    return true;
  }
  void Neutral() override {
    commands_ = {};
    pwm_outputs_ = {};
  }

  void SetDigitalInput(uint16_t id, bool value) { digital_inputs_[id] = value; }
  void SetAnalogInput(uint16_t id, double voltage) { analog_inputs_[id] = voltage; }
  void SetEncoder(uint16_t id, double position_rot, double velocity_rps) {
    encoders_[id] = {position_rot, velocity_rps};
  }
  bool GetDigitalOutput(uint16_t id) const {
    auto it = digital_outputs_.find(id);
    return it != digital_outputs_.end() ? it->second : false;
  }
  double GetPwm(uint16_t id) const {
    auto it = pwm_outputs_.find(id);
    return it != pwm_outputs_.end() ? it->second : 0.0;
  }

 private:
  struct SimEncoder {
    double position_rot{};
    double velocity_rps{};
  };
  Config config_;
  uint64_t previous_time_{};
  std::array<MotorRequest, kMaxMotors> commands_{};
  std::array<MotorSample, kMaxMotors> samples_{};
  std::unordered_map<uint16_t, bool> digital_inputs_{};
  std::unordered_map<uint16_t, bool> digital_outputs_{};
  std::unordered_map<uint16_t, double> analog_inputs_{};
  std::unordered_map<uint16_t, SimEncoder> encoders_{};
  std::unordered_map<uint16_t, double> pwm_outputs_{};
};
}  // namespace talos::hardware
