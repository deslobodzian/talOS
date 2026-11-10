#include "config.h"

#include <bit>
#include <cmath>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>

#include "../driver_station/driver_station.h"

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
  Require(c.period_us >= 1000 && c.period_us <= 20000,
          "invalid gateway period");
  // Every node's Driver Station view expires at command_timeout_us. The
  // gateway sends a Driver Station packet on its first tick at least
  // kSamplePeriodUs after the previous one, so packets can be
  // kSamplePeriodUs + period_us apart; two such gaps leave a whole gap of
  // margin, where a shorter timeout can drop the mode between packets and
  // every Target with it. That also covers two gateway periods.
  const uint64_t min_timeout_us =
      2 * (driver_station::kSamplePeriodUs + c.period_us);
  if (c.command_timeout_us < min_timeout_us || c.command_timeout_us > 500000) {
    throw std::invalid_argument(
        "command_timeout_us " + std::to_string(c.command_timeout_us) +
        " is outside [" + std::to_string(min_timeout_us) +
        ", 500000]: it must span two Driver Station packet gaps of up to " +
        std::to_string(driver_station::kSamplePeriodUs) +
        " us + period_us each");
  }
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
  }
  for (const auto& m : c.motors) {
    device(m.id, m.name, m.bus, m.can_id);
    if (m.master_id != 0) {
      Require(m.master_id != m.id, "motor cannot follow itself");
      const MotorConfig* master = nullptr;
      for (const auto& candidate : c.motors) {
        if (candidate.id == m.master_id) master = &candidate;
      }
      Require(master && master->bus == m.bus,
              "follower master must exist on the same CAN bus");
      Require(master->master_id == 0, "follower chains are not supported");
    }
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
      for (double x : {g.p, g.i, g.d, g.s, g.v, g.a, g.g}) {
        Require(std::isfinite(x) && x >= 0, "invalid gain");
      }
      Require(g.gravity_type <= GravityType::kArmCosine,
              "unknown gravity type");
    }
    Require(m.feedback == Feedback::kRotor ||
                m.feedback == Feedback::kRemoteCANcoder,
            "unknown feedback source");
    if (m.feedback == Feedback::kRemoteCANcoder) {
      bool found = false;
      for (const auto& s : c.sensors) {
        found |= s.id == m.feedback_sensor_id &&
                 s.kind == SensorKind::kCANcoder && s.bus == m.bus;
      }
      Require(found, "remote CANcoder must exist on the motor's CAN bus");
    } else {
      Require(m.feedback_sensor_id == 0,
              "rotor feedback cannot reference a remote sensor");
    }
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
    Require(analog_channels.insert(ai.channel).second,
            "duplicate analog channel");
  }
  for (const auto& enc : c.encoders) {
    check_base(enc.id, enc.name);
    Require(enc.dio_a >= 0 && enc.dio_a <= 31, "invalid DIO channel");
    Require(dio_channels.insert(enc.dio_a).second, "duplicate DIO channel");
    if (enc.kind == EncoderKind::kQuadrature) {
      Require(enc.dio_b >= 0 && enc.dio_b <= 31, "invalid DIO channel");
      Require(enc.dio_a != enc.dio_b,
              "quadrature encoder dio_a and dio_b must differ");
      Require(dio_channels.insert(enc.dio_b).second, "duplicate DIO channel");
      Require(Positive(enc.counts_per_rev), "invalid counts_per_rev");
    } else {
      Require(enc.kind == EncoderKind::kDutyCycle, "unknown encoder kind");
      Require(std::isfinite(enc.offset_rot) && enc.offset_rot >= -0.5 &&
                  enc.offset_rot < 0.5,
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
    h.Add(m.master_id);
    h.Add(m.oppose_master);
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
    for (const auto& g : m.slots) {
      for (double x : {g.p, g.i, g.d, g.s, g.v, g.a, g.g}) h.Number(x);
      h.Add(static_cast<uint8_t>(g.gravity_type));
    }
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
