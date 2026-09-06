#include "config.h"

#include <bit>
#include <cmath>
#include <set>
#include <stdexcept>
#include <string_view>

namespace talos::hardware {
namespace {
void Require(bool ok, const char* message) {
  if (!ok) throw std::invalid_argument(message);
}
bool Positive(double x) { return std::isfinite(x) && x > 0; }
struct Hash {
  uint64_t value{14695981039346656037ull};
  void Add(uint64_t x) {
    for (int i = 0; i < 8; ++i) {
      value = (value ^ (x & 255)) * 1099511628211ull;
      x >>= 8;
    }
  }
  void Number(double x) { Add(std::bit_cast<uint64_t>(x)); }
  void Text(std::string_view s) {
    Add(s.size());
    for (unsigned char c : s) Add(c);
  }
};
}  // namespace

void Validate(const Config& c) {
  Require(!c.motors.empty() && c.motors.size() <= kMaxMotors &&
              c.sensors.size() <= kMaxSensors &&
              c.digital_inputs.size() <= kMaxDigitalInputs &&
              c.digital_outputs.size() <= kMaxDigitalOutputs &&
              c.analog_inputs.size() <= kMaxAnalogInputs &&
              c.encoders.size() <= kMaxEncoders &&
              c.pwm_outputs.size() <= kMaxPwmOutputs,
          "invalid hardware device count");
  Require(c.period_us >= 1000 && c.period_us <= 20000 &&
              c.command_timeout_us >= 2 * c.period_us &&
              c.command_timeout_us <= 500000,
          "invalid gateway period or command timeout");
  Require(Positive(c.status_hz) && c.status_hz <= 1000,
          "invalid status frequency");
  std::set<uint16_t> ids;
  std::set<std::string> names;
  std::set<std::pair<std::string, int>> addresses;
  std::set<int> dio_channels;
  std::set<int> analog_channels;
  std::set<int> pwm_channels;

  auto check_base = [&](uint16_t id, const std::string& name) {
    Require(id != 0 && ids.insert(id).second,
            "duplicate or zero logical device ID");
    Require(!name.empty() && name.size() <= 63 && names.insert(name).second,
            "invalid or duplicate device name");
  };

  auto device = [&](uint16_t id, const std::string& name,
                    const std::string& bus, int can) {
    check_base(id, name);
    Require(!bus.empty() && bus.size() <= 63 && can >= 0 && can <= 62,
            "invalid CAN address");
    Require(addresses.emplace(bus, can).second,
            "duplicate CAN address on a bus");
  };
  for (const auto& s : c.sensors) {
    device(s.id, s.name, s.bus, s.can_id);
    Require(s.kind == SensorKind::kCANcoder || s.kind == SensorKind::kPigeon2,
            "unknown sensor kind");
    Require(std::isfinite(s.offset_rot) && s.offset_rot >= -0.5 &&
                s.offset_rot < 0.5,
            "invalid encoder offset");
    Require(
        s.kind != SensorKind::kPigeon2 || (!s.inverted && s.offset_rot == 0),
        "Pigeon mounting must use its native frame");
  }
  for (const auto& m : c.motors) {
    device(m.id, m.name, m.bus, m.can_id);
    Require(Positive(m.supply_limit_a) && Positive(m.stator_limit_a) &&
                Positive(m.max_voltage) && m.max_voltage <= 12,
            "invalid motor limits");
    Require(Positive(m.rotor_to_sensor_ratio) &&
                Positive(m.sensor_to_mechanism_ratio),
            "invalid gearing");
    Require(Positive(m.max_velocity_rps) && Positive(m.cruise_velocity_rps) &&
                m.cruise_velocity_rps <= m.max_velocity_rps &&
                Positive(m.acceleration_rps2) && std::isfinite(m.jerk_rps3) &&
                m.jerk_rps3 >= 0,
            "invalid motion profile");
    Require(std::isfinite(m.reverse_limit_rot) &&
                std::isfinite(m.forward_limit_rot) &&
                m.reverse_limit_rot < m.forward_limit_rot,
            "invalid soft limits");
    for (const auto& g : m.slots) {
      for (double x : {g.p, g.i, g.d, g.s, g.v, g.a, g.g})
        Require(std::isfinite(x) && x >= 0, "invalid gain");
    }
    Require(m.feedback == Feedback::kRotor ||
                m.feedback == Feedback::kRemoteCANcoder,
            "unknown feedback source");
    if (m.feedback == Feedback::kRemoteCANcoder) {
      bool found = false;
      for (const auto& s : c.sensors)
        found |= s.id == m.feedback_sensor_id &&
                 s.kind == SensorKind::kCANcoder && s.bus == m.bus;
      Require(found, "remote CANcoder must exist on the motor's CAN bus");
    } else
      Require(m.feedback_sensor_id == 0,
              "rotor feedback cannot reference a remote sensor");
  }
  for (const auto& di : c.digital_inputs) {
    check_base(di.id, di.name);
    Require(di.dio >= 0 && di.dio <= 31, "invalid DIO channel");
    Require(dio_channels.insert(di.dio).second, "duplicate DIO channel");
  }
  for (const auto& do_dev : c.digital_outputs) {
    check_base(do_dev.id, do_dev.name);
    Require(do_dev.dio >= 0 && do_dev.dio <= 31, "invalid DIO channel");
    Require(dio_channels.insert(do_dev.dio).second, "duplicate DIO channel");
  }
  for (const auto& ai : c.analog_inputs) {
    check_base(ai.id, ai.name);
    Require(ai.channel >= 0 && ai.channel <= 7, "invalid analog channel");
    Require(analog_channels.insert(ai.channel).second, "duplicate analog channel");
  }
  for (const auto& enc : c.encoders) {
    check_base(enc.id, enc.name);
    Require(enc.dio_a >= 0 && enc.dio_a <= 31, "invalid DIO channel");
    Require(dio_channels.insert(enc.dio_a).second, "duplicate DIO channel");
    if (enc.kind == EncoderKind::kQuadrature) {
      Require(enc.dio_b >= 0 && enc.dio_b <= 31, "invalid DIO channel");
      Require(enc.dio_a != enc.dio_b, "quadrature encoder dio_a and dio_b must differ");
      Require(dio_channels.insert(enc.dio_b).second, "duplicate DIO channel");
      Require(Positive(enc.counts_per_rev), "invalid counts_per_rev");
    } else {
      Require(enc.kind == EncoderKind::kDutyCycle, "unknown encoder kind");
      Require(std::isfinite(enc.offset_rot) && enc.offset_rot >= -0.5 && enc.offset_rot < 0.5,
              "invalid encoder offset");
    }
  }
  for (const auto& pwm : c.pwm_outputs) {
    check_base(pwm.id, pwm.name);
    Require(pwm.channel >= 0 && pwm.channel <= 19, "invalid PWM channel");
    Require(pwm_channels.insert(pwm.channel).second, "duplicate PWM channel");
  }
}

uint64_t ConfigurationId(const Config& c) {
  Validate(c);
  Hash h;
  h.Add(1);
  h.Add(c.period_us);
  h.Add(c.command_timeout_us);
  h.Number(c.status_hz);
  h.Add(c.commissioned);
  h.Add(c.motors.size());
  h.Add(c.sensors.size());
  h.Add(c.digital_inputs.size());
  h.Add(c.digital_outputs.size());
  h.Add(c.analog_inputs.size());
  h.Add(c.encoders.size());
  h.Add(c.pwm_outputs.size());
  for (const auto& m : c.motors) {
    h.Add(m.id);
    h.Text(m.name);
    h.Add(m.can_id);
    h.Text(m.bus);
    h.Add(m.inverted);
    h.Add(m.brake);
    h.Number(m.supply_limit_a);
    h.Number(m.stator_limit_a);
    h.Number(m.max_voltage);
    h.Add(static_cast<uint8_t>(m.feedback));
    h.Add(m.feedback_sensor_id);
    h.Number(m.rotor_to_sensor_ratio);
    h.Number(m.sensor_to_mechanism_ratio);
    h.Add(m.continuous_wrap);
    h.Add(m.soft_limits);
    h.Number(m.reverse_limit_rot);
    h.Number(m.forward_limit_rot);
    h.Number(m.max_velocity_rps);
    h.Number(m.cruise_velocity_rps);
    h.Number(m.acceleration_rps2);
    h.Number(m.jerk_rps3);
    for (const auto& g : m.slots)
      for (double x : {g.p, g.i, g.d, g.s, g.v, g.a, g.g}) h.Number(x);
  }
  for (const auto& s : c.sensors) {
    h.Add(s.id);
    h.Text(s.name);
    h.Add(static_cast<uint8_t>(s.kind));
    h.Add(s.can_id);
    h.Text(s.bus);
    h.Add(s.inverted);
    h.Number(s.offset_rot);
  }
  for (const auto& di : c.digital_inputs) {
    h.Add(di.id);
    h.Text(di.name);
    h.Add(di.dio);
  }
  for (const auto& do_dev : c.digital_outputs) {
    h.Add(do_dev.id);
    h.Text(do_dev.name);
    h.Add(do_dev.dio);
  }
  for (const auto& ai : c.analog_inputs) {
    h.Add(ai.id);
    h.Text(ai.name);
    h.Add(ai.channel);
  }
  for (const auto& enc : c.encoders) {
    h.Add(enc.id);
    h.Text(enc.name);
    h.Add(static_cast<uint8_t>(enc.kind));
    h.Add(enc.dio_a);
    h.Add(enc.dio_b);
    h.Number(enc.counts_per_rev);
    h.Number(enc.offset_rot);
  }
  for (const auto& pwm : c.pwm_outputs) {
    h.Add(pwm.id);
    h.Text(pwm.name);
    h.Add(pwm.channel);
  }
  return h.value;
}
}  // namespace talos::hardware
