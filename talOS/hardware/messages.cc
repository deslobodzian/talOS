#include "messages.h"

#include <bit>
#include <cmath>
#include <limits>

namespace talos::hardware {
static_assert(sizeof(double) == 8 && std::numeric_limits<double>::is_iec559);
namespace {
struct Writer {
  std::span<uint8_t> data;
  std::size_t pos{};
  bool ok{true};
  void Int(uint64_t value, int size) {
    for (int i = 0; i < size; ++i) {
      if (pos == data.size()) {
        ok = false;
        return;
      }
      data[pos++] = static_cast<uint8_t>(value);
      value >>= 8;
    }
  }
  void Real(double x) { Int(std::bit_cast<uint64_t>(x), 8); }
};
struct Reader {
  std::span<const uint8_t> data;
  std::size_t pos{};
  bool ok{true};
  uint64_t Int(int size) {
    uint64_t value{};
    for (int i = 0; i < size; ++i) {
      if (pos == data.size()) {
        ok = false;
        return 0;
      }
      value |= uint64_t{data[pos++]} << (i * 8);
    }
    return value;
  }
  double Real() {
    const double x = std::bit_cast<double>(Int(8));
    ok &= std::isfinite(x);
    return x;
  }
  bool Done() const { return ok && pos == data.size(); }
};
}  // namespace
std::size_t Encode(const Command& c, std::span<uint8_t> data) {
  if (c.count > kMaxMotors ||
      c.digital_output_count > kMaxDigitalOutputs ||
      c.pwm_output_count > kMaxPwmOutputs)
    return 0;
  Writer w{data};
  w.Int(c.config_id, 8);
  w.Int(c.boot_id, 8);
  w.Int(c.epoch, 8);
  w.Int(c.observed_time_us, 8);
  w.Int(c.count, 2);
  w.Int(c.digital_output_count, 2);
  w.Int(c.pwm_output_count, 2);
  for (std::size_t i = 0; i < c.count; ++i) {
    const auto& m = c.motors[i];
    w.Int(m.id, 2);
    w.Int(static_cast<uint8_t>(m.mode), 1);
    w.Int(m.slot, 1);
    w.Real(m.demand);
    w.Real(m.feedforward_v);
  }
  for (std::size_t i = 0; i < c.digital_output_count; ++i) {
    const auto& d = c.digital_outputs[i];
    w.Int(d.id, 2);
    w.Int(d.value ? 1 : 0, 1);
  }
  for (std::size_t i = 0; i < c.pwm_output_count; ++i) {
    const auto& p = c.pwm_outputs[i];
    w.Int(p.id, 2);
    w.Real(p.output);
  }
  return w.ok ? w.pos : 0;
}
bool Decode(std::span<const uint8_t> data, Command& out) {
  Command c{};
  Reader r{data};
  c.config_id = r.Int(8);
  c.boot_id = r.Int(8);
  c.epoch = r.Int(8);
  c.observed_time_us = r.Int(8);
  c.count = r.Int(2);
  c.digital_output_count = r.Int(2);
  c.pwm_output_count = r.Int(2);
  if (c.count > kMaxMotors ||
      c.digital_output_count > kMaxDigitalOutputs ||
      c.pwm_output_count > kMaxPwmOutputs)
    return false;
  for (std::size_t i = 0; i < c.count; ++i) {
    auto& m = c.motors[i];
    m.id = r.Int(2);
    m.mode = static_cast<Mode>(r.Int(1));
    m.slot = r.Int(1);
    m.demand = r.Real();
    m.feedforward_v = r.Real();
    if (m.mode > Mode::kMotionMagic || m.slot > 2) return false;
  }
  for (std::size_t i = 0; i < c.digital_output_count; ++i) {
    auto& d = c.digital_outputs[i];
    d.id = r.Int(2);
    auto val = r.Int(1);
    if (val > 1) return false;
    d.value = (val == 1);
  }
  for (std::size_t i = 0; i < c.pwm_output_count; ++i) {
    auto& p = c.pwm_outputs[i];
    p.id = r.Int(2);
    p.output = r.Real();
  }
  if (!r.Done()) return false;
  out = c;
  return true;
}
std::size_t Encode(const State& s, std::span<uint8_t> data) {
  if (s.motor_count > kMaxMotors || s.sensor_count > kMaxSensors ||
      s.digital_input_count > kMaxDigitalInputs ||
      s.digital_output_count > kMaxDigitalOutputs ||
      s.analog_input_count > kMaxAnalogInputs ||
      s.encoder_count > kMaxEncoders ||
      s.pwm_output_count > kMaxPwmOutputs)
    return 0;
  Writer w{data};
  w.Int(s.config_id, 8);
  w.Int(s.boot_id, 8);
  w.Int(s.epoch, 8);
  w.Int(s.sample_time_us, 8);
  w.Int(s.last_command_sequence, 8);
  w.Int(s.flags, 4);
  w.Int(s.motor_count, 2);
  w.Int(s.sensor_count, 2);
  w.Int(s.digital_input_count, 2);
  w.Int(s.digital_output_count, 2);
  w.Int(s.analog_input_count, 2);
  w.Int(s.encoder_count, 2);
  w.Int(s.pwm_output_count, 2);
  for (std::size_t i = 0; i < s.motor_count; ++i) {
    const auto& m = s.motors[i];
    w.Int(m.id, 2);
    w.Int(m.valid ? 1 : 0, 1);
    w.Real(m.position_rot);
    w.Real(m.velocity_rps);
    w.Real(m.voltage);
    w.Real(m.stator_current_a);
    w.Int(m.position_age_us, 4);
    w.Int(m.velocity_age_us, 4);
  }
  for (std::size_t i = 0; i < s.sensor_count; ++i) {
    const auto& x = s.sensors[i];
    w.Int(x.id, 2);
    w.Int(x.valid ? 1 : 0, 1);
    w.Real(x.position_rot);
    w.Real(x.velocity_rps);
    w.Int(x.position_age_us, 4);
    w.Int(x.velocity_age_us, 4);
  }
  for (std::size_t i = 0; i < s.digital_input_count; ++i) {
    const auto& d = s.digital_inputs[i];
    w.Int(d.id, 2);
    w.Int(d.valid ? 1 : 0, 1);
    w.Int(d.value ? 1 : 0, 1);
  }
  for (std::size_t i = 0; i < s.digital_output_count; ++i) {
    const auto& d = s.digital_outputs[i];
    w.Int(d.id, 2);
    w.Int(d.valid ? 1 : 0, 1);
    w.Int(d.value ? 1 : 0, 1);
  }
  for (std::size_t i = 0; i < s.analog_input_count; ++i) {
    const auto& a = s.analog_inputs[i];
    w.Int(a.id, 2);
    w.Int(a.valid ? 1 : 0, 1);
    w.Real(a.voltage);
    w.Int(a.age_us, 4);
  }
  for (std::size_t i = 0; i < s.encoder_count; ++i) {
    const auto& e = s.encoders[i];
    w.Int(e.id, 2);
    w.Int(e.valid ? 1 : 0, 1);
    w.Real(e.position_rot);
    w.Real(e.velocity_rps);
    w.Int(e.age_us, 4);
  }
  for (std::size_t i = 0; i < s.pwm_output_count; ++i) {
    const auto& p = s.pwm_outputs[i];
    w.Int(p.id, 2);
    w.Int(p.valid ? 1 : 0, 1);
    w.Real(p.output);
  }
  return w.ok ? w.pos : 0;
}
bool Decode(std::span<const uint8_t> data, State& out) {
  State s{};
  Reader r{data};
  s.config_id = r.Int(8);
  s.boot_id = r.Int(8);
  s.epoch = r.Int(8);
  s.sample_time_us = r.Int(8);
  s.last_command_sequence = r.Int(8);
  s.flags = r.Int(4);
  s.motor_count = r.Int(2);
  s.sensor_count = r.Int(2);
  s.digital_input_count = r.Int(2);
  s.digital_output_count = r.Int(2);
  s.analog_input_count = r.Int(2);
  s.encoder_count = r.Int(2);
  s.pwm_output_count = r.Int(2);
  if (s.motor_count > kMaxMotors || s.sensor_count > kMaxSensors ||
      s.digital_input_count > kMaxDigitalInputs ||
      s.digital_output_count > kMaxDigitalOutputs ||
      s.analog_input_count > kMaxAnalogInputs ||
      s.encoder_count > kMaxEncoders ||
      s.pwm_output_count > kMaxPwmOutputs)
    return false;
  for (std::size_t i = 0; i < s.motor_count; ++i) {
    auto& m = s.motors[i];
    m.id = r.Int(2);
    auto valid = r.Int(1);
    if (valid > 1) return false;
    m.valid = (valid == 1);
    m.position_rot = r.Real();
    m.velocity_rps = r.Real();
    m.voltage = r.Real();
    m.stator_current_a = r.Real();
    m.position_age_us = r.Int(4);
    m.velocity_age_us = r.Int(4);
  }
  for (std::size_t i = 0; i < s.sensor_count; ++i) {
    auto& x = s.sensors[i];
    x.id = r.Int(2);
    auto valid = r.Int(1);
    if (valid > 1) return false;
    x.valid = (valid == 1);
    x.position_rot = r.Real();
    x.velocity_rps = r.Real();
    x.position_age_us = r.Int(4);
    x.velocity_age_us = r.Int(4);
  }
  for (std::size_t i = 0; i < s.digital_input_count; ++i) {
    auto& d = s.digital_inputs[i];
    d.id = r.Int(2);
    auto valid = r.Int(1);
    if (valid > 1) return false;
    d.valid = (valid == 1);
    auto val = r.Int(1);
    if (val > 1) return false;
    d.value = (val == 1);
  }
  for (std::size_t i = 0; i < s.digital_output_count; ++i) {
    auto& d = s.digital_outputs[i];
    d.id = r.Int(2);
    auto valid = r.Int(1);
    if (valid > 1) return false;
    d.valid = (valid == 1);
    auto val = r.Int(1);
    if (val > 1) return false;
    d.value = (val == 1);
  }
  for (std::size_t i = 0; i < s.analog_input_count; ++i) {
    auto& a = s.analog_inputs[i];
    a.id = r.Int(2);
    auto valid = r.Int(1);
    if (valid > 1) return false;
    a.valid = (valid == 1);
    a.voltage = r.Real();
    a.age_us = r.Int(4);
  }
  for (std::size_t i = 0; i < s.encoder_count; ++i) {
    auto& e = s.encoders[i];
    e.id = r.Int(2);
    auto valid = r.Int(1);
    if (valid > 1) return false;
    e.valid = (valid == 1);
    e.position_rot = r.Real();
    e.velocity_rps = r.Real();
    e.age_us = r.Int(4);
  }
  for (std::size_t i = 0; i < s.pwm_output_count; ++i) {
    auto& p = s.pwm_outputs[i];
    p.id = r.Int(2);
    auto valid = r.Int(1);
    if (valid > 1) return false;
    p.valid = (valid == 1);
    p.output = r.Real();
  }
  if (!r.Done()) return false;
  out = s;
  return true;
}
static_assert(32 + 6 + kMaxMotors * 20 + kMaxDigitalOutputs * 3 + kMaxPwmOutputs * 10 <= 1200);
static_assert(44 + 14 + kMaxMotors * 43 + kMaxSensors * 27 +
              kMaxDigitalInputs * 4 + kMaxDigitalOutputs * 4 +
              kMaxAnalogInputs * 15 + kMaxEncoders * 23 +
              kMaxPwmOutputs * 11 <= 1200);
}  // namespace talos::hardware
