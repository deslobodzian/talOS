#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "common/hardware/config.h"
#include "toml.hpp"

namespace talos::config {

struct RobotConfig {
  hardware::Config hardware;
  std::map<std::string, hardware::Devices> subsystems;
  toml::table toml_data;

  const hardware::Devices* GetDevices(const std::string& subsystem) const {
    auto it = subsystems.find(subsystem);
    if (it != subsystems.end()) return &it->second;
    return nullptr;
  }

  const toml::table* GetSubsystemTable(const std::string& subsystem,
                                      const std::string& section = "") const {
    if (auto* subs = toml_data["subsystems"].as_table()) {
      if (auto* sub = (*subs)[subsystem].as_table()) {
        if (section.empty()) return sub;
        return (*sub)[section].as_table();
      }
    }
    return nullptr;
  }
};

namespace detail {

inline bool Positive(double x) { return std::isfinite(x) && x > 0; }
inline bool NonNegative(double x) { return std::isfinite(x) && x >= 0; }

enum class DeviceKind {
  kMotor,
  kCANcoder,
  kPigeon2,
  kDigitalInput,
  kDigitalOutput,
  kAnalogInput,
  kEncoder,
  kPwm,
};

struct RawDevice {
  std::string subsystem;
  std::string name;
  DeviceKind kind;

  hardware::MotorConfig motor{};
  hardware::SensorConfig sensor{};
  hardware::DigitalInputConfig digital_input{};
  hardware::DigitalOutputConfig digital_output{};
  hardware::AnalogInputConfig analog_input{};
  hardware::EncoderConfig encoder{};
  hardware::PwmConfig pwm{};

  std::string feedback_sensor_name;
  bool feedback_is_remote{false};
};

inline double ReadNumber(const toml::table& tbl, const char* key, double def,
                         bool (*check_fn)(double) = nullptr,
                         const char* err_msg = "invalid numeric value") {
  if (auto node = tbl[key]) {
    if (auto val = node.value<double>()) {
      if (check_fn && !check_fn(*val)) {
        throw std::invalid_argument(err_msg);
      }
      return *val;
    } else {
      throw std::invalid_argument(std::string("field '") + key + "' must be a number");
    }
  }
  return def;
}

inline int64_t ReadInteger(const toml::table& tbl, const char* key, int64_t def,
                           bool (*check_fn)(int64_t) = nullptr,
                           const char* err_msg = "invalid integer value") {
  if (auto node = tbl[key]) {
    if (auto val = node.value<int64_t>()) {
      if (check_fn && !check_fn(*val)) {
        throw std::invalid_argument(err_msg);
      }
      return *val;
    } else {
      throw std::invalid_argument(std::string("field '") + key + "' must be an integer");
    }
  }
  return def;
}

inline bool ReadBool(const toml::table& tbl, const char* key, bool def) {
  if (auto node = tbl[key]) {
    if (auto val = node.value<bool>()) {
      return *val;
    } else {
      throw std::invalid_argument(std::string("field '") + key + "' must be a boolean");
    }
  }
  return def;
}

inline std::string ReadString(const toml::table& tbl, const char* key,
                             const std::string& def = "") {
  if (auto node = tbl[key]) {
    if (auto val = node.value<std::string_view>()) {
      return std::string(*val);
    } else {
      throw std::invalid_argument(std::string("field '") + key + "' must be a string");
    }
  }
  return def;
}

}  // namespace detail

inline RobotConfig ParseRobotConfigString(std::string_view toml_content) {
  toml::table tbl;
  try {
    tbl = toml::parse(toml_content);
  } catch (const toml::parse_error& err) {
    throw std::invalid_argument(std::string("TOML parse error: ") + err.what());
  }

  RobotConfig result;
  result.toml_data = tbl;

  // 1. Parse [robot] table
  uint32_t period_us = 5000;
  uint32_t command_timeout_us = 100000;
  double status_hz = 200.0;
  bool commissioned = false;

  if (auto* robot_tbl = tbl["robot"].as_table()) {
    if (auto node = (*robot_tbl)["period_us"]) {
      auto val = node.value<int64_t>();
      if (!val || *val < 1000 || *val > 20000) {
        throw std::invalid_argument("invalid gateway period or command timeout: period_us must be in [1000, 20000]");
      }
      period_us = static_cast<uint32_t>(*val);
    }
    if (auto node = (*robot_tbl)["command_timeout_us"]) {
      auto val = node.value<int64_t>();
      if (!val || *val < 2 * period_us || *val > 500000) {
        throw std::invalid_argument("invalid gateway period or command timeout: command_timeout_us must be in [2 * period_us, 500000]");
      }
      command_timeout_us = static_cast<uint32_t>(*val);
    } else {
      // Default timeout check against period_us
      if (command_timeout_us < 2 * period_us) {
        command_timeout_us = 2 * period_us;
      }
    }
    if (auto node = (*robot_tbl)["status_hz"]) {
      auto val = node.value<double>();
      if (!val || !detail::Positive(*val) || *val > 1000) {
        throw std::invalid_argument("invalid status frequency: status_hz must be in (0, 1000]");
      }
      status_hz = *val;
    }
    commissioned = (*robot_tbl)["commissioned"].value_or(false);
  }

  result.hardware.period_us = period_us;
  result.hardware.command_timeout_us = command_timeout_us;
  result.hardware.status_hz = status_hz;
  result.hardware.commissioned = commissioned;

  auto* subsystems_tbl = tbl["subsystems"].as_table();
  if (!subsystems_tbl) {
    throw std::invalid_argument("missing [subsystems] table");
  }

  std::map<std::string, std::set<std::string>> subsystem_device_names;
  std::vector<detail::RawDevice> raw_devices;

  auto validate_device_name = [&](const std::string& subsystem, const std::string& name) {
    if (name.empty() || name.size() > 63) {
      throw std::invalid_argument("invalid or empty device name: '" + name + "'");
    }
    if (!subsystem_device_names[subsystem].insert(name).second) {
      throw std::invalid_argument("duplicate device name within subsystem '" + subsystem + "': " + name);
    }
  };

  for (auto&& [sub_key, sub_val] : *subsystems_tbl) {
    std::string sub_name = std::string(sub_key.str());
    auto* sub_tbl = sub_val.as_table();
    if (!sub_tbl) continue;

    // Ensure subsystem entry exists in result
    result.subsystems[sub_name].subsystem = sub_name;

    // Parse motors
    if (auto* motors_tbl = (*sub_tbl)["motors"].as_table()) {
      for (auto&& [dev_key, dev_val] : *motors_tbl) {
        std::string dev_name = std::string(dev_key.str());
        validate_device_name(sub_name, dev_name);
        auto* motor_tbl = dev_val.as_table();
        if (!motor_tbl) {
          throw std::invalid_argument("motor '" + dev_name + "' must be a table");
        }

        std::string type = detail::ReadString(*motor_tbl, "type", "TalonFX");
        if (type != "TalonFX") {
          throw std::invalid_argument("unknown motor type: '" + type + "' for " + dev_name);
        }

        detail::RawDevice raw;
        raw.subsystem = sub_name;
        raw.name = dev_name;
        raw.kind = detail::DeviceKind::kMotor;

        raw.motor.bus = detail::ReadString(*motor_tbl, "bus", "rio");
        if (raw.motor.bus.empty() || raw.motor.bus.size() > 63) {
          throw std::invalid_argument("invalid CAN address: empty or too long bus name for " + dev_name);
        }

        if (!(*motor_tbl)["can_id"]) {
          throw std::invalid_argument("missing can_id for motor: " + dev_name);
        }
        raw.motor.can_id = static_cast<int>(detail::ReadInteger(
            *motor_tbl, "can_id", -1,
            [](int64_t v) { return v >= 0 && v <= 62; },
            "invalid CAN address: can_id must be in [0, 62]"));

        raw.motor.inverted = detail::ReadBool(*motor_tbl, "inverted", false);
        raw.motor.brake = detail::ReadBool(*motor_tbl, "brake", true);

        raw.motor.supply_limit_a = detail::ReadNumber(
            *motor_tbl, "supply_limit_a", 40.0,
            detail::Positive, "invalid motor limits: supply_limit_a must be positive");
        raw.motor.stator_limit_a = detail::ReadNumber(
            *motor_tbl, "stator_limit_a", 80.0,
            detail::Positive, "invalid motor limits: stator_limit_a must be positive");
        raw.motor.max_voltage = detail::ReadNumber(
            *motor_tbl, "max_voltage", 12.0,
            [](double v) { return detail::Positive(v) && v <= 12.0; },
            "invalid motor limits: max_voltage must be positive and <= 12");

        raw.motor.rotor_to_sensor_ratio = detail::ReadNumber(
            *motor_tbl, "rotor_to_sensor_ratio", 1.0,
            detail::Positive, "invalid gearing: rotor_to_sensor_ratio must be positive");
        raw.motor.sensor_to_mechanism_ratio = detail::ReadNumber(
            *motor_tbl, "sensor_to_mechanism_ratio", 1.0,
            detail::Positive, "invalid gearing: sensor_to_mechanism_ratio must be positive");

        raw.motor.continuous_wrap = detail::ReadBool(*motor_tbl, "continuous_wrap", false);
        raw.motor.soft_limits = detail::ReadBool(*motor_tbl, "soft_limits", false);

        raw.motor.reverse_limit_rot = detail::ReadNumber(
            *motor_tbl, "reverse_limit_rot", -1.0,
            [](double v) { return std::isfinite(v); }, "invalid soft limits: reverse_limit_rot must be finite");
        raw.motor.forward_limit_rot = detail::ReadNumber(
            *motor_tbl, "forward_limit_rot", 1.0,
            [](double v) { return std::isfinite(v); }, "invalid soft limits: forward_limit_rot must be finite");

        if (raw.motor.reverse_limit_rot >= raw.motor.forward_limit_rot) {
          throw std::invalid_argument("invalid soft limits: reverse_limit_rot must be < forward_limit_rot");
        }

        raw.motor.max_velocity_rps = detail::ReadNumber(
            *motor_tbl, "max_velocity_rps", 100.0,
            detail::Positive, "invalid motion profile: max_velocity_rps must be positive");

        double default_cruise = std::min(20.0, raw.motor.max_velocity_rps);
        if (auto node = (*motor_tbl)["cruise_velocity_rps"]) {
          if (auto val = node.value<double>()) {
            if (!detail::Positive(*val) || *val > raw.motor.max_velocity_rps) {
              throw std::invalid_argument("invalid motion profile: cruise_velocity_rps must be positive and <= max_velocity_rps");
            }
            raw.motor.cruise_velocity_rps = *val;
          } else {
            throw std::invalid_argument("cruise_velocity_rps must be a number");
          }
        } else {
          raw.motor.cruise_velocity_rps = default_cruise;
        }

        raw.motor.acceleration_rps2 = detail::ReadNumber(
            *motor_tbl, "acceleration_rps2", 40.0,
            detail::Positive, "invalid motion profile: acceleration_rps2 must be positive");
        raw.motor.jerk_rps3 = detail::ReadNumber(
            *motor_tbl, "jerk_rps3", 0.0,
            detail::NonNegative, "invalid motion profile: jerk_rps3 must be non-negative");

        // Parse gains slots
        for (int s = 0; s < 3; ++s) {
          std::string slot_key = "slot" + std::to_string(s);
          if (auto* slot_tbl = (*motor_tbl)[slot_key].as_table()) {
            auto parse_gain = [&](const char* name, double& dest) {
              if (auto node = (*slot_tbl)[name]) {
                if (auto val = node.value<double>()) {
                  if (!detail::NonNegative(*val)) {
                    throw std::invalid_argument(std::string("invalid gain '") + name + "' in " + slot_key);
                  }
                  dest = *val;
                } else {
                  throw std::invalid_argument(std::string("gain '") + name + "' in " + slot_key + " must be a number");
                }
              }
            };
            parse_gain("p", raw.motor.slots[s].p);
            parse_gain("i", raw.motor.slots[s].i);
            parse_gain("d", raw.motor.slots[s].d);
            parse_gain("s", raw.motor.slots[s].s);
            parse_gain("v", raw.motor.slots[s].v);
            parse_gain("a", raw.motor.slots[s].a);
            parse_gain("g", raw.motor.slots[s].g);
          }
        }

        // Feedback
        std::string fb_str = detail::ReadString(*motor_tbl, "feedback", "");
        std::string fb_sensor = detail::ReadString(*motor_tbl, "feedback_sensor", "");
        if (fb_sensor.empty()) {
          fb_sensor = detail::ReadString(*motor_tbl, "feedback_sensor_name", "");
        }
        if (fb_sensor.empty() && (*motor_tbl)["feedback_sensor_id"]) {
          if (auto str_val = (*motor_tbl)["feedback_sensor_id"].value<std::string_view>()) {
            fb_sensor = std::string(*str_val);
          }
        }

        if (fb_str == "RemoteCANcoder" || fb_str == "kRemoteCANcoder" || fb_str == "cancoder") {
          raw.feedback_is_remote = true;
        } else if (fb_str == "Rotor" || fb_str == "kRotor" || fb_str == "rotor") {
          raw.feedback_is_remote = false;
        } else if (!fb_str.empty()) {
          throw std::invalid_argument("unknown feedback source: " + fb_str);
        } else {
          // If feedback wasn't specified but feedback_sensor was, infer RemoteCANcoder
          raw.feedback_is_remote = !fb_sensor.empty();
        }

        if (!raw.feedback_is_remote && !fb_sensor.empty()) {
          throw std::invalid_argument("rotor feedback cannot reference a remote sensor");
        }
        if (raw.feedback_is_remote && fb_sensor.empty()) {
          throw std::invalid_argument("remote feedback requires feedback_sensor to be specified");
        }
        raw.feedback_sensor_name = fb_sensor;

        raw_devices.push_back(std::move(raw));
      }
    }

    // Parse sensors
    if (auto* sensors_tbl = (*sub_tbl)["sensors"].as_table()) {
      for (auto&& [dev_key, dev_val] : *sensors_tbl) {
        std::string dev_name = std::string(dev_key.str());
        validate_device_name(sub_name, dev_name);
        auto* sensor_tbl = dev_val.as_table();
        if (!sensor_tbl) {
          throw std::invalid_argument("sensor '" + dev_name + "' must be a table");
        }

        std::string type = detail::ReadString(*sensor_tbl, "type", "CANcoder");
        detail::RawDevice raw;
        raw.subsystem = sub_name;
        raw.name = dev_name;

        if (type == "CANcoder" || type == "kCANcoder") {
          raw.kind = detail::DeviceKind::kCANcoder;
          raw.sensor.kind = hardware::SensorKind::kCANcoder;
          raw.sensor.bus = detail::ReadString(*sensor_tbl, "bus", "rio");
          if (raw.sensor.bus.empty() || raw.sensor.bus.size() > 63) {
            throw std::invalid_argument("invalid CAN address: empty or too long bus name for " + dev_name);
          }
          if (!(*sensor_tbl)["can_id"]) {
            throw std::invalid_argument("missing can_id for sensor: " + dev_name);
          }
          raw.sensor.can_id = static_cast<int>(detail::ReadInteger(
              *sensor_tbl, "can_id", -1,
              [](int64_t v) { return v >= 0 && v <= 62; },
              "invalid CAN address: can_id must be in [0, 62]"));
          raw.sensor.inverted = detail::ReadBool(*sensor_tbl, "inverted", false);
          raw.sensor.offset_rot = detail::ReadNumber(
              *sensor_tbl, "offset_rot", 0.0,
              [](double v) { return std::isfinite(v) && v >= -0.5 && v < 0.5; },
              "invalid encoder offset: offset_rot must be in [-0.5, 0.5)");
        } else if (type == "Pigeon2" || type == "kPigeon2") {
          raw.kind = detail::DeviceKind::kPigeon2;
          raw.sensor.kind = hardware::SensorKind::kPigeon2;
          raw.sensor.bus = detail::ReadString(*sensor_tbl, "bus", "rio");
          if (raw.sensor.bus.empty() || raw.sensor.bus.size() > 63) {
            throw std::invalid_argument("invalid CAN address: empty or too long bus name for " + dev_name);
          }
          if (!(*sensor_tbl)["can_id"]) {
            throw std::invalid_argument("missing can_id for sensor: " + dev_name);
          }
          raw.sensor.can_id = static_cast<int>(detail::ReadInteger(
              *sensor_tbl, "can_id", -1,
              [](int64_t v) { return v >= 0 && v <= 62; },
              "invalid CAN address: can_id must be in [0, 62]"));
          raw.sensor.inverted = detail::ReadBool(*sensor_tbl, "inverted", false);
          raw.sensor.offset_rot = detail::ReadNumber(*sensor_tbl, "offset_rot", 0.0);
          if (raw.sensor.inverted || raw.sensor.offset_rot != 0.0) {
            throw std::invalid_argument("Pigeon mounting must use its native frame");
          }
        } else if (type == "DigitalInput") {
          raw.kind = detail::DeviceKind::kDigitalInput;
          if (!(*sensor_tbl)["dio"]) throw std::invalid_argument("missing dio channel for DigitalInput " + dev_name);
          raw.digital_input.dio = static_cast<int>(detail::ReadInteger(
              *sensor_tbl, "dio", -1, [](int64_t v) { return v >= 0 && v <= 31; },
              "invalid dio channel: dio must be in [0, 31]"));
        } else if (type == "AnalogInput") {
          raw.kind = detail::DeviceKind::kAnalogInput;
          const char* ch_key = (*sensor_tbl)["analog"] ? "analog" : "channel";
          if (!(*sensor_tbl)[ch_key]) throw std::invalid_argument("missing analog channel for AnalogInput " + dev_name);
          raw.analog_input.channel = static_cast<int>(detail::ReadInteger(
              *sensor_tbl, ch_key, -1, [](int64_t v) { return v >= 0 && v <= 7; },
              "invalid analog channel: channel must be in [0, 7]"));
        } else if (type == "QuadratureEncoder" || type == "DutyCycleEncoder") {
          raw.kind = detail::DeviceKind::kEncoder;
          if (type == "QuadratureEncoder") {
            raw.encoder.kind = hardware::EncoderKind::kQuadrature;
            if (!(*sensor_tbl)["dio_a"]) throw std::invalid_argument("missing dio_a for QuadratureEncoder " + dev_name);
            raw.encoder.dio_a = static_cast<int>(detail::ReadInteger(
                *sensor_tbl, "dio_a", -1, [](int64_t v) { return v >= 0 && v <= 31; },
                "invalid dio_a channel"));
            raw.encoder.dio_b = static_cast<int>(detail::ReadInteger(
                *sensor_tbl, "dio_b", -1, [](int64_t v) { return v >= -1 && v <= 31; },
                "invalid dio_b channel"));
            raw.encoder.counts_per_rev = detail::ReadNumber(
                *sensor_tbl, "counts_per_rev", 2048.0, detail::Positive,
                "counts_per_rev must be positive");
          } else {
            raw.encoder.kind = hardware::EncoderKind::kDutyCycle;
            if (!(*sensor_tbl)["dio"]) throw std::invalid_argument("missing dio for DutyCycleEncoder " + dev_name);
            raw.encoder.dio_a = static_cast<int>(detail::ReadInteger(
                *sensor_tbl, "dio", -1, [](int64_t v) { return v >= 0 && v <= 31; },
                "invalid dio channel"));
            raw.encoder.offset_rot = detail::ReadNumber(*sensor_tbl, "offset_rot", 0.0);
          }
        } else {
          throw std::invalid_argument("unknown sensor type: '" + type + "' for " + dev_name);
        }

        raw_devices.push_back(std::move(raw));
      }
    }

    // Parse digital_inputs
    if (auto* dis_tbl = (*sub_tbl)["digital_inputs"].as_table()) {
      for (auto&& [dev_key, dev_val] : *dis_tbl) {
        std::string dev_name = std::string(dev_key.str());
        validate_device_name(sub_name, dev_name);
        auto* di_tbl = dev_val.as_table();
        if (!di_tbl) throw std::invalid_argument("digital input '" + dev_name + "' must be a table");
        if (!(*di_tbl)["dio"]) throw std::invalid_argument("missing dio channel for " + dev_name);
        detail::RawDevice raw;
        raw.subsystem = sub_name;
        raw.name = dev_name;
        raw.kind = detail::DeviceKind::kDigitalInput;
        raw.digital_input.dio = static_cast<int>(detail::ReadInteger(
            *di_tbl, "dio", -1, [](int64_t v) { return v >= 0 && v <= 31; }, "invalid dio channel"));
        raw_devices.push_back(std::move(raw));
      }
    }

    // Parse digital_outputs
    if (auto* dos_tbl = (*sub_tbl)["digital_outputs"].as_table()) {
      for (auto&& [dev_key, dev_val] : *dos_tbl) {
        std::string dev_name = std::string(dev_key.str());
        validate_device_name(sub_name, dev_name);
        auto* do_tbl = dev_val.as_table();
        if (!do_tbl) throw std::invalid_argument("digital output '" + dev_name + "' must be a table");
        if (!(*do_tbl)["dio"]) throw std::invalid_argument("missing dio channel for " + dev_name);
        detail::RawDevice raw;
        raw.subsystem = sub_name;
        raw.name = dev_name;
        raw.kind = detail::DeviceKind::kDigitalOutput;
        raw.digital_output.dio = static_cast<int>(detail::ReadInteger(
            *do_tbl, "dio", -1, [](int64_t v) { return v >= 0 && v <= 31; }, "invalid dio channel"));
        raw_devices.push_back(std::move(raw));
      }
    }

    // Parse analog_inputs
    if (auto* ais_tbl = (*sub_tbl)["analog_inputs"].as_table()) {
      for (auto&& [dev_key, dev_val] : *ais_tbl) {
        std::string dev_name = std::string(dev_key.str());
        validate_device_name(sub_name, dev_name);
        auto* ai_tbl = dev_val.as_table();
        if (!ai_tbl) throw std::invalid_argument("analog input '" + dev_name + "' must be a table");
        const char* ch_key = (*ai_tbl)["analog"] ? "analog" : "channel";
        if (!(*ai_tbl)[ch_key]) throw std::invalid_argument("missing analog channel for " + dev_name);
        detail::RawDevice raw;
        raw.subsystem = sub_name;
        raw.name = dev_name;
        raw.kind = detail::DeviceKind::kAnalogInput;
        raw.analog_input.channel = static_cast<int>(detail::ReadInteger(
            *ai_tbl, ch_key, -1, [](int64_t v) { return v >= 0 && v <= 7; }, "invalid analog channel"));
        raw_devices.push_back(std::move(raw));
      }
    }

    // Parse encoders
    if (auto* encs_tbl = (*sub_tbl)["encoders"].as_table()) {
      for (auto&& [dev_key, dev_val] : *encs_tbl) {
        std::string dev_name = std::string(dev_key.str());
        validate_device_name(sub_name, dev_name);
        auto* enc_tbl = dev_val.as_table();
        if (!enc_tbl) throw std::invalid_argument("encoder '" + dev_name + "' must be a table");
        std::string type = detail::ReadString(*enc_tbl, "type", "QuadratureEncoder");
        detail::RawDevice raw;
        raw.subsystem = sub_name;
        raw.name = dev_name;
        raw.kind = detail::DeviceKind::kEncoder;
        if (type == "QuadratureEncoder" || type == "Quadrature") {
          raw.encoder.kind = hardware::EncoderKind::kQuadrature;
          if (!(*enc_tbl)["dio_a"]) throw std::invalid_argument("missing dio_a for " + dev_name);
          raw.encoder.dio_a = static_cast<int>(detail::ReadInteger(
              *enc_tbl, "dio_a", -1, [](int64_t v) { return v >= 0 && v <= 31; }, "invalid dio_a channel"));
          raw.encoder.dio_b = static_cast<int>(detail::ReadInteger(
              *enc_tbl, "dio_b", -1, [](int64_t v) { return v >= -1 && v <= 31; }, "invalid dio_b channel"));
          raw.encoder.counts_per_rev = detail::ReadNumber(
              *enc_tbl, "counts_per_rev", 2048.0, detail::Positive, "counts_per_rev must be positive");
        } else if (type == "DutyCycleEncoder" || type == "DutyCycle") {
          raw.encoder.kind = hardware::EncoderKind::kDutyCycle;
          if (!(*enc_tbl)["dio"]) throw std::invalid_argument("missing dio for " + dev_name);
          raw.encoder.dio_a = static_cast<int>(detail::ReadInteger(
              *enc_tbl, "dio", -1, [](int64_t v) { return v >= 0 && v <= 31; }, "invalid dio channel"));
          raw.encoder.offset_rot = detail::ReadNumber(*enc_tbl, "offset_rot", 0.0);
        } else {
          throw std::invalid_argument("unknown encoder type: " + type);
        }
        raw_devices.push_back(std::move(raw));
      }
    }

    // Parse pwm_outputs / pwms
    auto* pwms_tbl = (*sub_tbl)["pwm_outputs"].as_table();
    if (!pwms_tbl) pwms_tbl = (*sub_tbl)["pwms"].as_table();
    if (pwms_tbl) {
      for (auto&& [dev_key, dev_val] : *pwms_tbl) {
        std::string dev_name = std::string(dev_key.str());
        validate_device_name(sub_name, dev_name);
        auto* pwm_tbl = dev_val.as_table();
        if (!pwm_tbl) throw std::invalid_argument("pwm '" + dev_name + "' must be a table");
        const char* ch_key = (*pwm_tbl)["pwm"] ? "pwm" : "channel";
        if (!(*pwm_tbl)[ch_key]) throw std::invalid_argument("missing channel for " + dev_name);
        detail::RawDevice raw;
        raw.subsystem = sub_name;
        raw.name = dev_name;
        raw.kind = detail::DeviceKind::kPwm;
        raw.pwm.channel = static_cast<int>(detail::ReadInteger(
            *pwm_tbl, ch_key, -1, [](int64_t v) { return v >= 0 && v <= 19; }, "invalid PWM channel"));
        raw_devices.push_back(std::move(raw));
      }
    }
  }

  // 2. Exclusive ownership checks: Duplicate addresses / channels
  std::set<std::pair<std::string, int>> can_addresses;
  std::set<int> dio_channels;
  std::set<int> pwm_channels;
  std::set<int> analog_channels;

  for (const auto& dev : raw_devices) {
    if (dev.kind == detail::DeviceKind::kMotor) {
      if (!can_addresses.emplace(dev.motor.bus, dev.motor.can_id).second) {
        throw std::invalid_argument("duplicate CAN address on a bus: (" + dev.motor.bus + ", " +
                                    std::to_string(dev.motor.can_id) + ")");
      }
    } else if (dev.kind == detail::DeviceKind::kCANcoder || dev.kind == detail::DeviceKind::kPigeon2) {
      if (!can_addresses.emplace(dev.sensor.bus, dev.sensor.can_id).second) {
        throw std::invalid_argument("duplicate CAN address on a bus: (" + dev.sensor.bus + ", " +
                                    std::to_string(dev.sensor.can_id) + ")");
      }
    } else if (dev.kind == detail::DeviceKind::kDigitalInput) {
      if (!dio_channels.insert(dev.digital_input.dio).second) {
        throw std::invalid_argument("duplicate DIO channel: " + std::to_string(dev.digital_input.dio));
      }
    } else if (dev.kind == detail::DeviceKind::kDigitalOutput) {
      if (!dio_channels.insert(dev.digital_output.dio).second) {
        throw std::invalid_argument("duplicate DIO channel: " + std::to_string(dev.digital_output.dio));
      }
    } else if (dev.kind == detail::DeviceKind::kEncoder) {
      if (!dio_channels.insert(dev.encoder.dio_a).second) {
        throw std::invalid_argument("duplicate DIO channel: " + std::to_string(dev.encoder.dio_a));
      }
      if (dev.encoder.dio_b >= 0) {
        if (!dio_channels.insert(dev.encoder.dio_b).second) {
          throw std::invalid_argument("duplicate DIO channel: " + std::to_string(dev.encoder.dio_b));
        }
      }
    } else if (dev.kind == detail::DeviceKind::kAnalogInput) {
      if (!analog_channels.insert(dev.analog_input.channel).second) {
        throw std::invalid_argument("duplicate analog channel: " + std::to_string(dev.analog_input.channel));
      }
    } else if (dev.kind == detail::DeviceKind::kPwm) {
      if (!pwm_channels.insert(dev.pwm.channel).second) {
        throw std::invalid_argument("duplicate PWM channel: " + std::to_string(dev.pwm.channel));
      }
    }
  }

  // 3. Deterministic sorting: sort by subsystem name, then device name, and number from 1
  std::sort(raw_devices.begin(), raw_devices.end(),
            [](const detail::RawDevice& a, const detail::RawDevice& b) {
              if (a.subsystem != b.subsystem) return a.subsystem < b.subsystem;
              return a.name < b.name;
            });

  uint16_t next_id = 1;
  for (auto& dev : raw_devices) {
    uint16_t id = next_id++;
    switch (dev.kind) {
      case detail::DeviceKind::kMotor:
        dev.motor.id = id;
        dev.motor.name = dev.name;
        break;
      case detail::DeviceKind::kCANcoder:
      case detail::DeviceKind::kPigeon2:
        dev.sensor.id = id;
        dev.sensor.name = dev.name;
        break;
      case detail::DeviceKind::kDigitalInput:
        dev.digital_input.id = id;
        dev.digital_input.name = dev.name;
        break;
      case detail::DeviceKind::kDigitalOutput:
        dev.digital_output.id = id;
        dev.digital_output.name = dev.name;
        break;
      case detail::DeviceKind::kAnalogInput:
        dev.analog_input.id = id;
        dev.analog_input.name = dev.name;
        break;
      case detail::DeviceKind::kEncoder:
        dev.encoder.id = id;
        dev.encoder.name = dev.name;
        break;
      case detail::DeviceKind::kPwm:
        dev.pwm.id = id;
        dev.pwm.name = dev.name;
        break;
    }
  }

  // 4. Resolve feedback sensors and validate feedback references
  for (auto& dev : raw_devices) {
    if (dev.kind != detail::DeviceKind::kMotor) continue;

    if (dev.feedback_is_remote) {
      const detail::RawDevice* target_sensor = nullptr;
      for (const auto& other : raw_devices) {
        if (other.name == dev.feedback_sensor_name) {
          target_sensor = &other;
          break;
        }
      }

      if (!target_sensor) {
        throw std::invalid_argument("remote feedback sensor not found: " + dev.feedback_sensor_name);
      }
      if (target_sensor->subsystem != dev.subsystem) {
        throw std::invalid_argument("cross-subsystem device claim: motor '" + dev.name + "' in subsystem '" +
                                    dev.subsystem + "' references sensor '" + target_sensor->name +
                                    "' in subsystem '" + target_sensor->subsystem + "'");
      }
      if (target_sensor->kind != detail::DeviceKind::kCANcoder) {
        throw std::invalid_argument("remote feedback sensor must be a CANcoder: " + target_sensor->name);
      }
      if (target_sensor->sensor.bus != dev.motor.bus) {
        throw std::invalid_argument("remote CANcoder must exist on the motor's CAN bus: " + target_sensor->name);
      }

      dev.motor.feedback = hardware::Feedback::kRemoteCANcoder;
      dev.motor.feedback_sensor_id = target_sensor->sensor.id;
    } else {
      dev.motor.feedback = hardware::Feedback::kRotor;
      dev.motor.feedback_sensor_id = 0;
    }
  }

  // 5. Check subsystem custom tables (e.g. geometry) for cross-subsystem claims
  for (auto&& [sub_key, sub_val] : *subsystems_tbl) {
    std::string sub_name = std::string(sub_key.str());
    auto* sub_tbl = sub_val.as_table();
    if (!sub_tbl) continue;

    if (auto* geom_tbl = (*sub_tbl)["geometry"].as_table()) {
      if (auto* modules_arr = (*geom_tbl)["modules"].as_array()) {
        for (auto&& mod_node : *modules_arr) {
          if (auto* mod_tbl = mod_node.as_table()) {
            auto check_ref = [&](const char* field, detail::DeviceKind expected_kind) {
              if (auto node = (*mod_tbl)[field]) {
                if (auto name_view = node.value<std::string_view>()) {
                  std::string ref_name = std::string(*name_view);
                  const detail::RawDevice* found = nullptr;
                  for (const auto& d : raw_devices) {
                    if (d.name == ref_name) {
                      found = &d;
                      break;
                    }
                  }
                  if (!found) {
                    throw std::invalid_argument("geometry module references unknown device: " + ref_name);
                  }
                  if (found->subsystem != sub_name) {
                    throw std::invalid_argument("subsystem '" + sub_name + "' claims device '" + ref_name +
                                                "' declared under subsystem '" + found->subsystem + "'");
                  }
                  if (found->kind != expected_kind) {
                    throw std::invalid_argument("device '" + ref_name + "' has unexpected type in geometry");
                  }
                }
              }
            };
            check_ref("drive", detail::DeviceKind::kMotor);
            check_ref("steer", detail::DeviceKind::kMotor);
            if ((*mod_tbl)["encoder"]) {
              check_ref("encoder", detail::DeviceKind::kCANcoder);
            }
          }
        }
      }
    }
  }

  // 6. Populate hardware::Config and subsystem Devices
  for (const auto& dev : raw_devices) {
    auto& sub_devs = result.subsystems[dev.subsystem];
    switch (dev.kind) {
      case detail::DeviceKind::kMotor:
        result.hardware.motors.push_back(dev.motor);
        sub_devs.motors.push_back(dev.motor.id);
        break;
      case detail::DeviceKind::kCANcoder:
      case detail::DeviceKind::kPigeon2:
        result.hardware.sensors.push_back(dev.sensor);
        sub_devs.sensors.push_back(dev.sensor.id);
        break;
      case detail::DeviceKind::kDigitalInput:
        result.hardware.digital_inputs.push_back(dev.digital_input);
        sub_devs.digital_inputs.push_back(dev.digital_input.id);
        break;
      case detail::DeviceKind::kDigitalOutput:
        result.hardware.digital_outputs.push_back(dev.digital_output);
        sub_devs.digital_outputs.push_back(dev.digital_output.id);
        break;
      case detail::DeviceKind::kAnalogInput:
        result.hardware.analog_inputs.push_back(dev.analog_input);
        sub_devs.analog_inputs.push_back(dev.analog_input.id);
        break;
      case detail::DeviceKind::kEncoder:
        result.hardware.encoders.push_back(dev.encoder);
        sub_devs.encoders.push_back(dev.encoder.id);
        break;
      case detail::DeviceKind::kPwm:
        result.hardware.pwm_outputs.push_back(dev.pwm);
        sub_devs.pwm_outputs.push_back(dev.pwm.id);
        break;
    }
  }

  // 7. Validate full hardware config
  hardware::Validate(result.hardware);

  return result;
}

inline RobotConfig ParseRobotConfig(const std::string& path) {
  std::ifstream f(path);
  if (!f.is_open()) {
    throw std::invalid_argument("cannot open file: " + path);
  }
  std::stringstream ss;
  ss << f.rdbuf();
  return ParseRobotConfigString(ss.str());
}

}  // namespace talos::config
