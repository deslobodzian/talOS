#include "config.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>

#include "../driver_station/driver_station.h"

namespace talos::hardware {
namespace {
constexpr std::size_t kMaxTotalDevices =
    kMaxMotors + kMaxSensors + kMaxDigitalInputs + kMaxDigitalOutputs +
    kMaxAnalogInputs + kMaxEncoders + kMaxPwmOutputs;

bool EqualIgnoringAsciiCase(std::string_view a, std::string_view b) noexcept {
  const auto lower = [](char ch) {
    return ch >= 'A' && ch <= 'Z' ? static_cast<char>(ch - 'A' + 'a') : ch;
  };
  return a.size() == b.size() &&
         std::equal(a.begin(), a.end(), b.begin(),
                    [&](char x, char y) { return lower(x) == lower(y); });
}

bool Positive(double x) noexcept { return std::isfinite(x) && x > 0; }

struct Hash {
  uint64_t value{14695981039346656037ull};
  void Add(uint64_t x) noexcept {
    for (int i = 0; i < 8; ++i) {
      value = (value ^ (x & 255)) * 1099511628211ull;
      x >>= 8;
    }
  }
  void Number(double x) noexcept { Add(std::bit_cast<uint64_t>(x)); }
  void Text(std::string_view s) noexcept {
    Add(s.size());
    for (unsigned char c : s) Add(c);
  }
};

bool CanBusesImpl(const Config& c, CanBusList& out, bool throw_on_error) {
  // Each distinct bus with the first device naming it; "rio" is always
  // there. Names compare byte for byte, as the wire carries them, but two
  // that differ only by case are refused: Phoenix may resolve both to one
  // CANivore, whose load and health would then be split over two slots.
  constexpr std::size_t kCanDeviceCapacity =
      decltype(Config::motors)::kCapacity +
      decltype(Config::sensors)::kCapacity;
  std::array<std::pair<std::string_view, std::string_view>,
             kCanDeviceCapacity + 1>
      buses{};
  std::size_t bus_count = 1;
  buses[0] = {"rio", "the RoboRIO itself"};
  const auto add = [&](std::string_view bus, std::string_view device) -> bool {
    for (std::size_t i = 0; i < bus_count; ++i) {
      const auto& [name, first_device] = buses[i];
      if (name == bus) return true;
      if (EqualIgnoringAsciiCase(name, bus)) {
        if (throw_on_error) {
          throw std::invalid_argument(
              "CAN buses \"" + std::string{name} + "\" (" +
              std::string{first_device} + ") and \"" + std::string{bus} +
              "\" (" + std::string{device} +
              ") differ only by case; spell one bus one way");
        }
        return false;
      }
    }
    if (bus_count >= buses.size()) return false;
    buses[bus_count++] = {bus, device};
    return true;
  };
  for (const auto& m : c.motors) {
    if (!add(m.bus, m.name)) return false;
  }
  for (const auto& s : c.sensors) {
    if (!add(s.bus, s.name)) return false;
  }
  std::array<std::string_view, kCanDeviceCapacity> others{};
  const std::size_t other_count = bus_count - 1;
  for (std::size_t i = 0; i < other_count; ++i) {
    others[i] = buses[i + 1].first;
  }
  // std::string_view compares as unsigned bytes (char_traits<char>).
  std::sort(others.begin(), others.begin() + other_count);
  if (other_count > kMaxCanBuses - 1) {
    if (throw_on_error) {
      std::string names;
      for (std::size_t i = 0; i < other_count; ++i) {
        if (!names.empty()) names += ", ";
        names += others[i];
      }
      throw std::invalid_argument(
          "the config names " + std::to_string(other_count) +
          " CAN buses besides rio (" + names + "); at most " +
          std::to_string(kMaxCanBuses - 1) + " fit /hw/status");
    }
    return false;
  }
  out.names = {};
  out.names[0] = "rio";
  out.count = 1;
  for (std::size_t i = 0; i < other_count; ++i) {
    out.names[out.count++] = others[i];
  }
  return true;
}

bool ValidateImpl(const Config& c, bool throw_on_error) {
  const auto fail = [throw_on_error](const char* message) -> bool {
    if (throw_on_error) throw std::invalid_argument(message);
    return false;
  };
  // No device at all is a robot brought up before its first mechanism.
  if (!(c.motors.size() <= kMaxMotors && c.sensors.size() <= kMaxSensors &&
        c.digital_inputs.size() <= kMaxDigitalInputs &&
        c.digital_outputs.size() <= kMaxDigitalOutputs &&
        c.analog_inputs.size() <= kMaxAnalogInputs &&
        c.encoders.size() <= kMaxEncoders &&
        c.pwm_outputs.size() <= kMaxPwmOutputs)) {
    return fail("invalid hardware device count");
  }
  if (!(c.period_us >= 1000 && c.period_us <= 20000)) {
    return fail("invalid gateway period");
  }
  // Every node's Driver Station view expires at command_timeout_us. The
  // gateway sends a Driver Station packet on its first tick at least
  // kSamplePeriodUs after the previous one, so packets can be
  // kSamplePeriodUs + period_us apart; two such gaps leave a whole gap of
  // margin, where a shorter timeout can drop the mode between packets and
  // every Target with it. That also covers two gateway periods.
  const uint64_t min_timeout_us =
      2 * (driver_station::kSamplePeriodUs + c.period_us);
  if (c.command_timeout_us < min_timeout_us || c.command_timeout_us > 500000) {
    if (throw_on_error) {
      throw std::invalid_argument(
          "command_timeout_us " + std::to_string(c.command_timeout_us) +
          " is outside [" + std::to_string(min_timeout_us) +
          ", 500000]: it must span two Driver Station packet gaps of up to " +
          std::to_string(driver_station::kSamplePeriodUs) +
          " us + period_us each");
    }
    return false;
  }
  if (!(Positive(c.status_hz) && c.status_hz <= 1000)) {
    return fail("invalid status frequency");
  }
  CanBusList buses{};
  if (!CanBusesImpl(c, buses, throw_on_error)) return false;
  // The gateway counts motion feedback command_timeout_us old as invalid
  // (FeedbackFresh, gate.h). Four status periods inside it, the factor an
  // actuator owner's period has too, keep a slow rate from reading as a
  // dropout at every frame.
  if (c.status_hz * static_cast<double>(c.command_timeout_us) < 4'000'000) {
    if (throw_on_error) {
      throw std::invalid_argument(
          "status_hz " + std::to_string(c.status_hz) +
          " gives fewer than four status periods per command_timeout_us " +
          std::to_string(c.command_timeout_us) +
          ": it must be at least 4000000 / command_timeout_us");
    }
    return false;
  }

  std::array<uint16_t, kMaxTotalDevices> ids{};
  std::size_t id_count = 0;
  std::array<std::string_view, kMaxTotalDevices> names{};
  std::size_t name_count = 0;
  std::array<std::pair<std::string_view, int>, kMaxMotors + kMaxSensors>
      addresses{};
  std::size_t address_count = 0;
  uint32_t dio_mask = 0;
  uint32_t analog_mask = 0;
  uint32_t pwm_mask = 0;

  const auto insert_id = [&](uint16_t id) {
    for (std::size_t i = 0; i < id_count; ++i) {
      if (ids[i] == id) return false;
    }
    ids[id_count++] = id;
    return true;
  };
  const auto insert_name = [&](std::string_view name) {
    for (std::size_t i = 0; i < name_count; ++i) {
      if (names[i] == name) return false;
    }
    names[name_count++] = name;
    return true;
  };
  const auto insert_address = [&](std::string_view bus, int can) {
    for (std::size_t i = 0; i < address_count; ++i) {
      if (addresses[i].first == bus && addresses[i].second == can) return false;
    }
    addresses[address_count++] = {bus, can};
    return true;
  };
  const auto claim_channel = [](uint32_t& mask, int ch) {
    const uint32_t bit = uint32_t{1} << static_cast<unsigned>(ch);
    if ((mask & bit) != 0) return false;
    mask |= bit;
    return true;
  };

  auto check_base = [&](uint16_t id, const ConfigString& name) -> bool {
    if (id == 0 || !insert_id(id)) {
      return fail("duplicate or zero logical device ID");
    }
    if (name.empty() || name.overflowed() ||
        name.size() > kMaxConfigNameLength || !insert_name(name.view())) {
      return fail("invalid or duplicate device name");
    }
    return true;
  };

  auto device = [&](uint16_t id, const ConfigString& name,
                    const ConfigString& bus, int can) -> bool {
    if (!check_base(id, name)) return false;
    if (bus.empty() || bus.overflowed() || bus.size() > kMaxConfigNameLength ||
        can < 0 || can > 62) {
      return fail("invalid CAN address");
    }
    if (!insert_address(bus.view(), can)) {
      return fail("duplicate CAN address on a bus");
    }
    return true;
  };

  for (const auto& s : c.sensors) {
    if (!device(s.id, s.name, s.bus, s.can_id)) return false;
    if (!(s.kind == SensorKind::kCANcoder || s.kind == SensorKind::kPigeon2)) {
      return fail("unknown sensor kind");
    }
    if (!(std::isfinite(s.offset_rot) && s.offset_rot >= -0.5 &&
          s.offset_rot < 0.5)) {
      return fail("invalid encoder offset");
    }
  }
  for (const auto& m : c.motors) {
    if (!device(m.id, m.name, m.bus, m.can_id)) return false;
    if (m.master_id != 0) {
      if (m.master_id == m.id) return fail("motor cannot follow itself");
      const MotorConfig* master = nullptr;
      for (const auto& candidate : c.motors) {
        if (candidate.id == m.master_id) master = &candidate;
      }
      if (!(master && master->bus == m.bus)) {
        return fail("follower master must exist on the same CAN bus");
      }
      if (master->master_id != 0) {
        return fail("follower chains are not supported");
      }
    }
    if (!(m.master_id == 0 || !m.foc)) return fail("a follower cannot set foc");
    if (!(Positive(m.supply_limit_a) && Positive(m.stator_limit_a) &&
          Positive(m.max_voltage) && m.max_voltage <= 12)) {
      return fail("invalid motor limits");
    }
    // TorqueCurrentFOC is a Phoenix Pro request, so the limit needs foc; the
    // TalonFX applies stator_limit_a in torque mode too. The mode drives a
    // single motor: a Follower copies its master's torque current but not
    // its MaxAbsDutyCycle, so max_voltage would not hold a pair. The limit
    // is therefore refused on a follower and on a motor that has one.
    const auto fail_torque = [&](std::string_view why) -> bool {
      if (throw_on_error) {
        throw std::invalid_argument("max_torque_current_a for motor '" +
                                    m.name + "' " + std::string{why});
      }
      return false;
    };
    if (!(std::isfinite(m.max_torque_current_a) &&
          m.max_torque_current_a >= 0)) {
      return fail_torque("must be a finite number of amperes, 0 or more");
    }
    if (m.max_torque_current_a > 0) {
      for (const auto& other : c.motors) {
        if (m.master_id != 0 && other.id == m.master_id) {
          if (throw_on_error) {
            throw std::invalid_argument(
                "max_torque_current_a for motor '" + m.name +
                "' is set on a follower of motor '" + other.name +
                "'; torque-current mode drives a single motor");
          }
          return false;
        }
        if (other.master_id == m.id) {
          if (throw_on_error) {
            throw std::invalid_argument(
                "max_torque_current_a for motor '" + m.name +
                "' is set on a motor that motor '" + other.name +
                "' follows; a follower copies its master's torque current but "
                "not its duty cap, so torque-current mode drives a single "
                "motor");
          }
          return false;
        }
      }
      if (!m.foc) return fail_torque("needs foc = true (TorqueCurrentFOC)");
      if (!(m.max_torque_current_a <= m.stator_limit_a)) {
        return fail_torque("exceeds stator_limit_a");
      }
    }
    if (!(Positive(m.rotor_to_sensor_ratio) &&
          Positive(m.sensor_to_mechanism_ratio))) {
      return fail("invalid gearing");
    }
    if (!(Positive(m.max_velocity_rps) && Positive(m.cruise_velocity_rps) &&
          m.cruise_velocity_rps <= m.max_velocity_rps &&
          Positive(m.acceleration_rps2) && std::isfinite(m.jerk_rps3) &&
          m.jerk_rps3 >= 0)) {
      return fail("invalid motion profile");
    }
    if (!(std::isfinite(m.reverse_limit_rot) &&
          std::isfinite(m.forward_limit_rot) &&
          m.reverse_limit_rot < m.forward_limit_rot)) {
      return fail("invalid soft limits");
    }
    if (!(m.forward_limit_switch <= LimitSwitch::kDisabled &&
          m.reverse_limit_switch <= LimitSwitch::kDisabled)) {
      return fail("unknown limit switch type");
    }
    for (const auto& g : m.slots) {
      for (double x : {g.p, g.i, g.d, g.s, g.v, g.a, g.g}) {
        if (!(std::isfinite(x) && x >= 0)) return fail("invalid gain");
      }
      if (!(g.gravity_type <= GravityType::kArmCosine)) {
        return fail("unknown gravity type");
      }
    }
    if (!(m.feedback == Feedback::kRotor ||
          m.feedback == Feedback::kRemoteCANcoder)) {
      return fail("unknown feedback source");
    }
    if (m.feedback == Feedback::kRemoteCANcoder) {
      bool found = false;
      for (const auto& s : c.sensors) {
        found |= s.id == m.feedback_sensor_id &&
                 s.kind == SensorKind::kCANcoder && s.bus == m.bus;
      }
      if (!found) {
        return fail("remote CANcoder must exist on the motor's CAN bus");
      }
    } else {
      if (m.feedback_sensor_id != 0) {
        return fail("rotor feedback cannot reference a remote sensor");
      }
    }
  }
  for (const auto& di : c.digital_inputs) {
    if (!check_base(di.id, di.name)) return false;
    if (!(di.dio >= 0 && di.dio <= 31)) return fail("invalid DIO channel");
    if (!claim_channel(dio_mask, di.dio)) return fail("duplicate DIO channel");
  }
  for (const auto& do_dev : c.digital_outputs) {
    if (!check_base(do_dev.id, do_dev.name)) return false;
    if (!(do_dev.dio >= 0 && do_dev.dio <= 31)) {
      return fail("invalid DIO channel");
    }
    if (!claim_channel(dio_mask, do_dev.dio)) {
      return fail("duplicate DIO channel");
    }
  }
  for (const auto& ai : c.analog_inputs) {
    if (!check_base(ai.id, ai.name)) return false;
    if (!(ai.channel >= 0 && ai.channel <= 7)) {
      return fail("invalid analog channel");
    }
    if (!claim_channel(analog_mask, ai.channel)) {
      return fail("duplicate analog channel");
    }
  }
  for (const auto& enc : c.encoders) {
    if (!check_base(enc.id, enc.name)) return false;
    if (!(enc.dio_a >= 0 && enc.dio_a <= 31)) {
      return fail("invalid DIO channel");
    }
    if (!claim_channel(dio_mask, enc.dio_a)) {
      return fail("duplicate DIO channel");
    }
    if (enc.kind == EncoderKind::kQuadrature) {
      if (!(enc.dio_b >= 0 && enc.dio_b <= 31)) {
        return fail("invalid DIO channel");
      }
      if (enc.dio_a == enc.dio_b) {
        return fail("quadrature encoder dio_a and dio_b must differ");
      }
      if (!claim_channel(dio_mask, enc.dio_b)) {
        return fail("duplicate DIO channel");
      }
      if (!Positive(enc.counts_per_rev)) return fail("invalid counts_per_rev");
      if (!std::isfinite(enc.offset_rot)) return fail("invalid encoder offset");
    } else {
      if (enc.kind != EncoderKind::kDutyCycle) {
        return fail("unknown encoder kind");
      }
      if (!std::isfinite(enc.counts_per_rev)) {
        return fail("invalid counts_per_rev");
      }
      if (!(std::isfinite(enc.offset_rot) && enc.offset_rot >= -0.5 &&
            enc.offset_rot < 0.5)) {
        return fail("invalid encoder offset");
      }
    }
  }
  for (const auto& pwm : c.pwm_outputs) {
    if (!check_base(pwm.id, pwm.name)) return false;
    if (!(pwm.channel >= 0 && pwm.channel <= 19)) {
      return fail("invalid PWM channel");
    }
    if (!claim_channel(pwm_mask, pwm.channel)) {
      return fail("duplicate PWM channel");
    }
  }
  return true;
}

uint64_t HashConfig(const Config& c) noexcept {
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
    h.Add(static_cast<uint8_t>(m.forward_limit_switch));
    h.Add(static_cast<uint8_t>(m.reverse_limit_switch));
    h.Number(m.max_velocity_rps);
    h.Number(m.cruise_velocity_rps);
    h.Number(m.acceleration_rps2);
    h.Number(m.jerk_rps3);
    for (const auto& g : m.slots) {
      for (double x : {g.p, g.i, g.d, g.s, g.v, g.a, g.g}) h.Number(x);
      h.Add(static_cast<uint8_t>(g.gravity_type));
    }
    h.Add(m.foc);
    h.Number(m.max_torque_current_a);
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
}  // namespace

void Validate(const Config& c) { (void)ValidateImpl(c, true); }

bool IsValid(const Config& c) noexcept { return ValidateImpl(c, false); }

CanBusList CanBuses(const Config& c) {
  CanBusList list{};
  (void)CanBusesImpl(c, list, true);
  return list;
}

uint64_t ConfigurationId(const Config& c) {
  Validate(c);
  return HashConfig(c);
}

uint64_t TryConfigurationId(const Config& c) noexcept {
  return IsValid(c) ? HashConfig(c) : 0;
}
}  // namespace talos::hardware
