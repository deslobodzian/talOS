#pragma once

// Robot configuration: TOML -> hardware::Config plus per-subsystem ownership.
//
// Two file layouts, both reduced to one canonical view before parsing:
//   - Manifest: robot.toml holds [robot], optional [studio], and
//     [[subsystems]] entries { name, path }. Each path (relative to robot.toml)
//     names a file with a [subsystem] header (whose `name` must match the
//     entry) and that subsystem's device and custom tables at top level.
//   - Single file: everything inline under [subsystems.<name>].
//
// Device tables per subsystem: motors (type "TalonFX"), sensors (type
// "CANcoder" or "Pigeon2"), digital_inputs, digital_outputs, analog_inputs,
// encoders (type "QuadratureEncoder" or "DutyCycleEncoder"), pwm_outputs.
// Logical ids are assigned 1..N sorted by (subsystem, device name). Ranges,
// uniqueness, and exclusive ownership are enforced by hardware::Validate; the
// parser only checks what Validate cannot see (types, names, references).

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "talOS/hardware/config.h"
#include "toml.hpp"

namespace talos::config {

struct RobotConfig {
  hardware::Config hardware;
  std::map<std::string, hardware::Devices> subsystems;
  toml::table toml_data;

  // Deployment addressing: where the controller processor (RoboRIO gateway)
  // lives as seen from this companion. Loopback by default, so simulation and
  // loopback tests work with no extra configuration. Set to the RIO address
  // (e.g. "10.56.87.2") for a real robot, either here or via --remote.
  std::string controller_address{"127.0.0.1"};
  uint16_t controller_port{5802};
  uint16_t local_port{5803};

  // Sim-session viewer. Studio's bridge is a host-side process, not a robot
  // node, so it is not a subsystem; this table carries what the launcher
  // needs to start and supervise it in sim sessions. The watched topic is
  // read from [subsystems.telemetry] topic, the same key the telemetry node
  // reads, so the two ends cannot drift apart.
  struct StudioConfig {
    std::string bridge{"//studio/bridge:studio_bridge"};
    uint16_t http_port{5800};
    uint16_t udp_port{5801};
    // Relative to the launcher's working directory.
    std::string webroot{"bazel-bin/studio/dist"};
  };
  StudioConfig studio;

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

// Reads an optional typed field; a present field of the wrong type (or an
// integer that does not fit T) is an error, never silently defaulted.
template <typename T>
T Read(const toml::table& tbl, std::string_view key, T def) {
  auto node = tbl[key];
  if (!node) return def;
  const std::string field = "field '" + std::string(key) + "'";
  if constexpr (std::is_same_v<T, bool>) {
    if (auto v = node.value<bool>()) return *v;
    throw std::invalid_argument(field + " must be a boolean");
  } else if constexpr (std::is_integral_v<T>) {
    if (auto v = node.value<int64_t>(); v && std::in_range<T>(*v)) {
      return static_cast<T>(*v);
    }
    throw std::invalid_argument(field + " must be an integer in range");
  } else if constexpr (std::is_floating_point_v<T>) {
    if (auto v = node.value<double>()) return *v;
    throw std::invalid_argument(field + " must be a number");
  } else {
    if (auto v = node.value<std::string_view>()) return std::string(*v);
    throw std::invalid_argument(field + " must be a string");
  }
}

template <typename T>
T ReadRequired(const toml::table& tbl, const char* key,
               const std::string& device) {
  if (!tbl.contains(key)) {
    throw std::invalid_argument(std::string("missing '") + key + "' for " +
                                device);
  }
  return Read<T>(tbl, key, T{});
}

inline std::string ReadNonEmpty(const toml::table& tbl, const char* key,
                                std::string def, const char* table_name) {
  std::string v = Read<std::string>(tbl, key, def);
  if (v.empty()) {
    throw std::invalid_argument(std::string(table_name) + " field '" + key +
                                "' must be a non-empty string");
  }
  return v;
}

inline uint16_t ReadPort(const toml::table& tbl, const char* key,
                         uint16_t def, const char* table_name) {
  if (!tbl.contains(key)) return def;
  auto v = tbl[key].value<int64_t>();
  if (!v || *v < 1 || *v > 65535) {
    throw std::invalid_argument(std::string(table_name) + " field '" + key +
                                "' must be a port in [1, 65535]");
  }
  return static_cast<uint16_t>(*v);
}

using DeviceConfig =
    std::variant<hardware::MotorConfig, hardware::SensorConfig,
                 hardware::DigitalInputConfig, hardware::DigitalOutputConfig,
                 hardware::AnalogInputConfig, hardware::EncoderConfig,
                 hardware::PwmConfig>;

struct RawDevice {
  std::string subsystem;
  std::string name;
  DeviceConfig config;
  std::string feedback_sensor;  // Motors with RemoteCANcoder feedback only.
};

inline bool IsCANcoder(const RawDevice& d) {
  auto* s = std::get_if<hardware::SensorConfig>(&d.config);
  return s && s->kind == hardware::SensorKind::kCANcoder;
}

inline RawDevice ParseMotor(const toml::table& t, const std::string& name) {
  if (Read<std::string>(t, "type", "TalonFX") != "TalonFX") {
    throw std::invalid_argument("unknown motor type for " + name);
  }
  hardware::MotorConfig m;
  m.bus = Read(t, "bus", m.bus);
  m.can_id = ReadRequired<int>(t, "can_id", name);
  m.inverted = Read(t, "inverted", m.inverted);
  m.brake = Read(t, "brake", m.brake);
  m.supply_limit_a = Read(t, "supply_limit_a", m.supply_limit_a);
  m.stator_limit_a = Read(t, "stator_limit_a", m.stator_limit_a);
  m.max_voltage = Read(t, "max_voltage", m.max_voltage);
  m.rotor_to_sensor_ratio =
      Read(t, "rotor_to_sensor_ratio", m.rotor_to_sensor_ratio);
  m.sensor_to_mechanism_ratio =
      Read(t, "sensor_to_mechanism_ratio", m.sensor_to_mechanism_ratio);
  m.continuous_wrap = Read(t, "continuous_wrap", m.continuous_wrap);
  m.soft_limits = Read(t, "soft_limits", m.soft_limits);
  m.reverse_limit_rot = Read(t, "reverse_limit_rot", m.reverse_limit_rot);
  m.forward_limit_rot = Read(t, "forward_limit_rot", m.forward_limit_rot);
  m.max_velocity_rps = Read(t, "max_velocity_rps", m.max_velocity_rps);
  m.cruise_velocity_rps = Read(t, "cruise_velocity_rps",
                               std::min(20.0, m.max_velocity_rps));
  m.acceleration_rps2 = Read(t, "acceleration_rps2", m.acceleration_rps2);
  m.jerk_rps3 = Read(t, "jerk_rps3", m.jerk_rps3);
  for (int s = 0; s < 3; ++s) {
    if (auto* slot = t["slot" + std::to_string(s)].as_table()) {
      auto& g = m.slots[s];
      g.p = Read(*slot, "p", g.p);
      g.i = Read(*slot, "i", g.i);
      g.d = Read(*slot, "d", g.d);
      g.s = Read(*slot, "s", g.s);
      g.v = Read(*slot, "v", g.v);
      g.a = Read(*slot, "a", g.a);
      g.g = Read(*slot, "g", g.g);
    }
  }

  // The sensor is resolved to a logical id after numbering, so Validate
  // never sees the name; the parser checks the pairing.
  RawDevice raw{.name = name};
  const std::string feedback = Read<std::string>(t, "feedback", "Rotor");
  raw.feedback_sensor = Read<std::string>(t, "feedback_sensor", "");
  if (feedback == "RemoteCANcoder") {
    m.feedback = hardware::Feedback::kRemoteCANcoder;
    if (raw.feedback_sensor.empty()) {
      throw std::invalid_argument(
          "remote feedback requires feedback_sensor for " + name);
    }
  } else if (feedback == "Rotor") {
    if (!raw.feedback_sensor.empty()) {
      throw std::invalid_argument(
          "rotor feedback cannot reference a remote sensor: " + name);
    }
  } else {
    throw std::invalid_argument("unknown feedback source: " + feedback);
  }
  raw.config = std::move(m);
  return raw;
}

inline RawDevice ParseSensor(const toml::table& t, const std::string& name) {
  hardware::SensorConfig s;
  const std::string type = Read<std::string>(t, "type", "CANcoder");
  if (type == "CANcoder") {
    s.kind = hardware::SensorKind::kCANcoder;
  } else if (type == "Pigeon2") {
    s.kind = hardware::SensorKind::kPigeon2;
  } else {
    throw std::invalid_argument("unknown sensor type '" + type + "' for " +
                                name);
  }
  s.bus = Read(t, "bus", s.bus);
  s.can_id = ReadRequired<int>(t, "can_id", name);
  s.inverted = Read(t, "inverted", s.inverted);
  s.offset_rot = Read(t, "offset_rot", s.offset_rot);
  return {.name = name, .config = std::move(s)};
}

inline RawDevice ParseEncoder(const toml::table& t, const std::string& name) {
  hardware::EncoderConfig e;
  const std::string type = Read<std::string>(t, "type", "QuadratureEncoder");
  if (type == "QuadratureEncoder") {
    e.kind = hardware::EncoderKind::kQuadrature;
    e.dio_a = ReadRequired<int>(t, "dio_a", name);
    e.dio_b = ReadRequired<int>(t, "dio_b", name);
    e.counts_per_rev = Read(t, "counts_per_rev", e.counts_per_rev);
  } else if (type == "DutyCycleEncoder") {
    e.kind = hardware::EncoderKind::kDutyCycle;
    e.dio_a = ReadRequired<int>(t, "dio", name);
    e.offset_rot = Read(t, "offset_rot", e.offset_rot);
  } else {
    throw std::invalid_argument("unknown encoder type '" + type + "' for " +
                                name);
  }
  return {.name = name, .config = std::move(e)};
}

}  // namespace detail

// Effective controller endpoint: an explicit CLI value wins, otherwise the
// deployment configuration. A zero CLI port means "not given".
inline std::string ResolveControllerAddress(const RobotConfig& config,
                                            const std::string& cli_override) {
  if (!cli_override.empty()) return cli_override;
  return config.controller_address;
}

inline uint16_t ResolveControllerPort(const RobotConfig& config,
                                      uint16_t cli_override) {
  if (cli_override != 0) return cli_override;
  return config.controller_port;
}

inline uint16_t ResolveLocalPort(const RobotConfig& config,
                                 uint16_t cli_override) {
  if (cli_override != 0) return cli_override;
  return config.local_port;
}

// Parses the canonical single-file view. toml_data keeps the table, which is
// what GetSubsystemTable answers from.
inline RobotConfig ParseRobotConfigTable(toml::table tbl) {
  using detail::Read;
  using detail::ReadRequired;
  RobotConfig result;
  result.toml_data = tbl;
  auto& hw = result.hardware;

  if (auto* robot = tbl["robot"].as_table()) {
    hw.period_us = Read(*robot, "period_us", hw.period_us);
    hw.command_timeout_us =
        Read(*robot, "command_timeout_us", hw.command_timeout_us);
    hw.status_hz = Read(*robot, "status_hz", hw.status_hz);
    hw.commissioned = Read(*robot, "commissioned", hw.commissioned);
    result.controller_address = detail::ReadNonEmpty(
        *robot, "controller_address", result.controller_address, "[robot]");
    result.controller_port = detail::ReadPort(
        *robot, "controller_port", result.controller_port, "[robot]");
    result.local_port =
        detail::ReadPort(*robot, "local_port", result.local_port, "[robot]");
  }

  if (auto* studio = tbl["studio"].as_table()) {
    auto& s = result.studio;
    s.bridge = detail::ReadNonEmpty(*studio, "bridge", s.bridge, "[studio]");
    s.webroot = detail::ReadNonEmpty(*studio, "webroot", s.webroot, "[studio]");
    s.http_port = detail::ReadPort(*studio, "http_port", s.http_port, "[studio]");
    s.udp_port = detail::ReadPort(*studio, "udp_port", s.udp_port, "[studio]");
  }

  auto* subsystems_tbl = tbl["subsystems"].as_table();
  if (!subsystems_tbl) {
    throw std::invalid_argument("missing [subsystems] table");
  }

  std::vector<detail::RawDevice> raw_devices;
  for (auto&& [sub_key, sub_val] : *subsystems_tbl) {
    const std::string sub_name(sub_key.str());
    auto* sub_tbl = sub_val.as_table();
    if (!sub_tbl) continue;
    result.subsystems[sub_name].subsystem = sub_name;

    auto each = [&](const char* section, auto parse) {
      auto* devices = (*sub_tbl)[section].as_table();
      if (!devices) return;
      for (auto&& [dev_key, dev_val] : *devices) {
        const std::string name(dev_key.str());
        auto* dev_tbl = dev_val.as_table();
        if (!dev_tbl) {
          throw std::invalid_argument(std::string(section) + " entry '" +
                                      name + "' must be a table");
        }
        detail::RawDevice raw = parse(*dev_tbl, name);
        raw.subsystem = sub_name;
        raw.name = name;
        raw_devices.push_back(std::move(raw));
      }
    };
    each("motors", detail::ParseMotor);
    each("sensors", detail::ParseSensor);
    each("encoders", detail::ParseEncoder);
    each("digital_inputs", [](const toml::table& t, const std::string& n) {
      return detail::RawDevice{
          .config = hardware::DigitalInputConfig{
              .dio = ReadRequired<int>(t, "dio", n)}};
    });
    each("digital_outputs", [](const toml::table& t, const std::string& n) {
      return detail::RawDevice{
          .config = hardware::DigitalOutputConfig{
              .dio = ReadRequired<int>(t, "dio", n)}};
    });
    each("analog_inputs", [](const toml::table& t, const std::string& n) {
      return detail::RawDevice{
          .config = hardware::AnalogInputConfig{
              .channel = ReadRequired<int>(t, "channel", n)}};
    });
    each("pwm_outputs", [](const toml::table& t, const std::string& n) {
      return detail::RawDevice{
          .config = hardware::PwmConfig{
              .channel = ReadRequired<int>(t, "channel", n)}};
    });
  }

  // Deterministic logical ids: sort by (subsystem, device), number from 1.
  std::sort(raw_devices.begin(), raw_devices.end(),
            [](const detail::RawDevice& a, const detail::RawDevice& b) {
              if (a.subsystem != b.subsystem) return a.subsystem < b.subsystem;
              return a.name < b.name;
            });
  uint16_t next_id = 1;
  for (auto& dev : raw_devices) {
    std::visit(
        [&](auto& c) {
          c.id = next_id++;
          c.name = dev.name;
        },
        dev.config);
  }

  auto find = [&](const std::string& name) -> const detail::RawDevice* {
    for (const auto& d : raw_devices) {
      if (d.name == name) return &d;
    }
    return nullptr;
  };
  auto require_same_subsystem = [](const detail::RawDevice& target,
                                   const std::string& subsystem,
                                   const std::string& claimant) {
    if (target.subsystem != subsystem) {
      throw std::invalid_argument(
          "cross-subsystem device claim: '" + claimant + "' in subsystem '" +
          subsystem + "' references '" + target.name + "' in subsystem '" +
          target.subsystem + "'");
    }
  };

  // Remote feedback: name -> logical id. Validate checks kind and bus.
  for (auto& dev : raw_devices) {
    auto* motor = std::get_if<hardware::MotorConfig>(&dev.config);
    if (!motor || dev.feedback_sensor.empty()) continue;
    const auto* sensor = find(dev.feedback_sensor);
    if (!sensor) {
      throw std::invalid_argument("remote feedback sensor not found: " +
                                  dev.feedback_sensor);
    }
    require_same_subsystem(*sensor, dev.subsystem, dev.name);
    motor->feedback_sensor_id =
        std::visit([](const auto& c) { return c.id; }, sensor->config);
  }

  // Swerve geometry module references must name this subsystem's devices.
  for (auto&& [sub_key, sub_val] : *subsystems_tbl) {
    const std::string sub_name(sub_key.str());
    auto* sub_tbl = sub_val.as_table();
    auto* modules =
        sub_tbl ? (*sub_tbl)["geometry"]["modules"].as_array() : nullptr;
    if (!modules) continue;
    for (auto&& mod_node : *modules) {
      auto* mod = mod_node.as_table();
      if (!mod) continue;
      auto check_ref = [&](const char* field, bool (*is_kind)(const detail::RawDevice&)) {
        auto ref = (*mod)[field].value<std::string_view>();
        if (!ref) return;
        const auto* found = find(std::string(*ref));
        if (!found) {
          throw std::invalid_argument(
              "geometry module references unknown device: " + std::string(*ref));
        }
        require_same_subsystem(*found, sub_name, "geometry");
        if (!is_kind(*found)) {
          throw std::invalid_argument("device '" + found->name +
                                      "' has unexpected type in geometry");
        }
      };
      auto is_motor = [](const detail::RawDevice& d) {
        return std::holds_alternative<hardware::MotorConfig>(d.config);
      };
      check_ref("drive", is_motor);
      check_ref("steer", is_motor);
      check_ref("encoder", detail::IsCANcoder);
    }
  }

  for (const auto& dev : raw_devices) {
    auto& owned = result.subsystems[dev.subsystem];
    std::visit(
        [&](const auto& c) {
          using T = std::decay_t<decltype(c)>;
          auto add = [&](auto& all, auto& ids) {
            all.push_back(c);
            ids.push_back(c.id);
          };
          if constexpr (std::is_same_v<T, hardware::MotorConfig>) {
            add(hw.motors, owned.motors);
          } else if constexpr (std::is_same_v<T, hardware::SensorConfig>) {
            add(hw.sensors, owned.sensors);
          } else if constexpr (std::is_same_v<T, hardware::DigitalInputConfig>) {
            add(hw.digital_inputs, owned.digital_inputs);
          } else if constexpr (std::is_same_v<T, hardware::DigitalOutputConfig>) {
            add(hw.digital_outputs, owned.digital_outputs);
          } else if constexpr (std::is_same_v<T, hardware::AnalogInputConfig>) {
            add(hw.analog_inputs, owned.analog_inputs);
          } else if constexpr (std::is_same_v<T, hardware::EncoderConfig>) {
            add(hw.encoders, owned.encoders);
          } else {
            add(hw.pwm_outputs, owned.pwm_outputs);
          }
        },
        dev.config);
  }

  hardware::Validate(hw);
  return result;
}

inline RobotConfig ParseRobotConfigString(std::string_view toml_content) {
  try {
    return ParseRobotConfigTable(toml::parse(toml_content));
  } catch (const toml::parse_error& err) {
    throw std::invalid_argument(std::string("TOML parse error: ") + err.what());
  }
}

namespace detail {

// Read and parse one TOML file; every error names the file.
inline toml::table ParseTomlFile(const std::string& path) {
  std::ifstream f(path);
  if (!f.is_open()) {
    throw std::invalid_argument("cannot open file: " + path);
  }
  std::stringstream ss;
  ss << f.rdbuf();
  try {
    return toml::parse(ss.str());
  } catch (const toml::parse_error& err) {
    throw std::invalid_argument("TOML parse error in " + path + ": " +
                                err.what());
  }
}

// Replaces a [[subsystems]] manifest with the canonical [subsystems.<name>]
// view: each listed file's [subsystem] scalars (minus the name) plus its
// other top-level tables. Validation then runs once over the merged view, so
// splitting files cannot weaken a check.
inline toml::table ExpandManifest(const toml::array& manifest,
                                  const std::string& path) {
  namespace fs = std::filesystem;
  const fs::path base = fs::path(path).parent_path();
  toml::table subs_view;
  for (auto&& entry_node : manifest) {
    auto* entry = entry_node.as_table();
    auto name_view = entry ? (*entry)["name"].value<std::string_view>() : std::nullopt;
    auto rel_view = entry ? (*entry)["path"].value<std::string_view>() : std::nullopt;
    if (!name_view || !rel_view) {
      throw std::invalid_argument("invalid [[subsystems]] entry in " + path +
                                  ": needs 'name' and 'path'");
    }
    const std::string name(*name_view);
    const fs::path rel{std::string(*rel_view)};
    const std::string file = (rel.is_absolute() ? rel : base / rel).string();
    toml::table sub_file = ParseTomlFile(file);

    auto* header = sub_file["subsystem"].as_table();
    if (!header || (*header)["name"].value<std::string_view>() != name) {
      throw std::invalid_argument(
          file + ": [subsystem].name must match manifest entry '" + name + "'");
    }
    if (subs_view.contains(name)) {
      throw std::invalid_argument("duplicate [[subsystems]] entry in " + path +
                                  ": '" + name + "'");
    }
    toml::table view;
    for (auto&& [k, v] : *header) {
      if (k.str() != "name") view.insert_or_assign(std::string(k.str()), v);
    }
    for (auto&& [k, v] : sub_file) {
      if (k.str() != "subsystem") view.insert_or_assign(std::string(k.str()), v);
    }
    subs_view.insert_or_assign(name, std::move(view));
  }
  return subs_view;
}

}  // namespace detail

inline RobotConfig ParseRobotConfig(const std::string& path) {
  toml::table tbl = detail::ParseTomlFile(path);
  if (auto* manifest = tbl["subsystems"].as_array()) {
    tbl.insert_or_assign("subsystems", detail::ExpandManifest(*manifest, path));
  }
  return ParseRobotConfigTable(std::move(tbl));
}

}  // namespace talos::config
