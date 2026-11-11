#pragma once

// Robot configuration: TOML -> hardware::Config plus per-subsystem ownership.
//
// One layout: optional `include = "base.toml"`, [robot], optional [studio],
// and [subsystems.<name>] tables. [[subsystems]] { name, path, node?,
// defaults? } entries are an include over it: each file (path relative to
// robot.toml) maps its [subsystem] header and tables to [subsystems.<name>],
// optionally layered over a `defaults` TOML file, the entry's `node` winning;
// then it parses as one file.
//
// Device tables per subsystem: motors (type "TalonFX"), sensors (type
// "CANcoder" or "Pigeon2"), digital_inputs, digital_outputs, analog_inputs,
// encoders (type "QuadratureEncoder" or "DutyCycleEncoder"), pwm_outputs.
// Logical ids are assigned 1..N sorted by (subsystem, device name). Ranges,
// uniqueness, and exclusive ownership are enforced by hardware::Validate; the
// parser only checks what Validate cannot see (types, names, references, and
// each actuator owner's period_us against command_timeout_us).
//
// A subsystem's [trajectories] table names data files (name = "path",
// relative to the file that sets it); config load requires each to exist,
// and the build lists each in the config's CONFIG_FILES (resolve.cc).

#include <arpa/inet.h>

#include <algorithm>
#include <charconv>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <initializer_list>
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

#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/config.h"
#include "talOS/introspection/names.h"
#include "toml.hpp"

namespace talos::config {

// Which target lanes (/<sub>/target/<source>) drive a subsystem in each
// Driver Station mode, highest priority first. [robot.targets] sets the
// default for every subsystem; a subsystem's own [targets] table (in a
// single-file config, [subsystems.<name>.targets]) replaces the modes it
// lists.
struct TargetPolicy {
  std::vector<std::string> teleop{"teleop"};
  std::vector<std::string> autonomous{"auto"};
  std::vector<std::string> test{};  // pit tests opt in: test = ["test"]

  // Every source named by any mode, once, in the order teleop, autonomous,
  // test list them.
  std::vector<std::string> Sources() const {
    std::vector<std::string> out;
    for (const auto* mode : {&teleop, &autonomous, &test}) {
      for (const auto& source : *mode) {
        if (std::find(out.begin(), out.end(), source) == out.end()) {
          out.push_back(source);
        }
      }
    }
    return out;
  }

  bool operator==(const TargetPolicy&) const = default;
};
using TeleopValue = std::variant<bool, int64_t, double, std::string>;

struct TeleopBinding {
  int port{0};
  std::optional<int> button;
  std::optional<int> axis;
  std::optional<int> pov;
  int pov_index{0};
  std::vector<std::pair<std::string, TeleopValue>> on_toggle;
  std::vector<std::pair<std::string, TeleopValue>> while_held;
  std::string field;
  double scale{1.0};
  double deadband_frac{0.0};

  bool operator==(const TeleopBinding&) const = default;
};

struct RobotConfig {
  hardware::Config hardware;
  std::map<std::string, hardware::Devices> subsystems;
  toml::table toml_data;

  // Deployment addressing: where the controller processor (RoboRIO gateway)
  // lives as seen from this companion. Loopback by default, so simulation and
  // loopback tests work with no extra configuration. Set to the RIO address
  // (e.g. "10.56.87.2") for a real robot, either here or via --remote.
  // An IPv4 literal: the bridge's socket (talOS/protocol/udp.h) takes one
  // and never resolves a hostname, so config load refuses anything else.
  std::string controller_address{"127.0.0.1"};
  uint16_t controller_port{5802};
  uint16_t local_port{5803};

  // Launcher log retention: sessions kept under the log root (the new one
  // included; the newest matches and unclassifiable sessions may exceed it)
  // and the byte budget: half for the new session's event logs, half for
  // older sessions but those matches, and the space kept free on the root's
  // filesystem, which alone removes them.
  // Both positive integers. See talOS/launcher/README.md.
  int64_t log_keep_sessions{50};
  int64_t log_max_bytes{int64_t{2} * 1024 * 1024 * 1024};

  // Refuse a described graph with no producer for an actuated subsystem's mode.
  bool require_target_coverage{false};

  // Sim-session viewer. Studio's bridge is a host-side process, not a robot
  // node, so it is not a subsystem; this table carries what the launcher
  // needs to start and supervise it in sim sessions. `topic` is the topic it
  // watches, the telemetry node's byte topic (/talos/telemetry).
  struct StudioConfig {
    std::string bridge{"//studio/bridge:studio_bridge"};
    uint16_t http_port{5800};
    uint16_t max_clients{4};
    // Relative to the launcher's working directory.
    std::string webroot{"bazel-bin/studio/dist"};
    std::string topic{"/talos/telemetry"};
    std::string pose_topic;
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

  // [robot.targets], and each subsystem's policy with its [targets] applied.
  TargetPolicy targets;
  std::map<std::string, TargetPolicy, std::less<>> subsystem_targets;

  // The policy of `subsystem`; the robot default for a name not in the
  // config.
  const TargetPolicy& TargetsFor(std::string_view subsystem) const {
    const auto it = subsystem_targets.find(subsystem);
    return it != subsystem_targets.end() ? it->second : targets;
  }
  // Subsystem teleop bindings, preserving list order.
  std::map<std::string, std::vector<TeleopBinding>> teleop;
  std::map<std::string, std::vector<std::pair<std::string, TeleopValue>>>
      teleop_defaults;

  // Each subsystem's [trajectories]: name -> file, resolved against the file
  // that set it (in a config parsed from a string, the working directory).
  // Config load refused a file that does not exist.
  std::map<std::string, std::map<std::string, std::string, std::less<>>,
           std::less<>>
      trajectories;

  // The absolute path of `subsystem`'s [trajectories] entry `name`; throws
  // std::invalid_argument naming the subsystem and `trajectories.<name>` when
  // the config does not declare it.
  std::filesystem::path TrajectoryPath(std::string_view subsystem,
                                       std::string_view name) const {
    const auto sub = trajectories.find(subsystem);
    if (sub != trajectories.end()) {
      if (const auto it = sub->second.find(name); it != sub->second.end()) {
        return std::filesystem::absolute(it->second).lexically_normal();
      }
    }
    std::string declared;
    if (sub != trajectories.end()) {
      for (const auto& [known, path] : sub->second) {
        declared += (declared.empty() ? "" : ", ") + known;
      }
    }
    throw std::invalid_argument(
        "[subsystems." + std::string(subsystem) +
        "] declares no 'trajectories." + std::string(name) +
        "' (declared: " + (declared.empty() ? "none" : declared) + ")");
  }
};

// The keys of a [subsystems.<name>] table the framework reads: config load,
// the launcher, talOS/node (period_us, remap) and the devices layer. Every
// other key is a parameter of some node (Node::param, param_of), which the
// launcher checks against the keys the nodes' --describe probes report.
inline constexpr std::string_view kFrameworkSubsystemKeys[] = {
    "node",        "critical",       "period_us",       "remap",
    "targets",     "teleop",         "motors",          "sensors",
    "encoders",    "digital_inputs", "digital_outputs", "analog_inputs",
    "pwm_outputs", "trajectories",
};

constexpr bool IsFrameworkSubsystemKey(std::string_view key) {
  return std::ranges::find(kFrameworkSubsystemKeys, key) !=
         std::end(kFrameworkSubsystemKeys);
}

namespace detail {

// Reads an optional typed field; a present field of the wrong type (or an
// integer that does not fit T) is an error, never silently defaulted.
template <typename T>
T Read(const toml::table& tbl, std::string_view key, T def) {
  auto node = tbl[key];
  if (!node) return def;
  const std::string field = "field '" + std::string(key) + "'";
  if constexpr (std::is_same_v<T, bool>) {
    if (auto v = node.value_exact<bool>()) return *v;
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
  std::string v = Read<std::string>(tbl, key, std::move(def));
  if (v.empty()) {
    throw std::invalid_argument(std::string(table_name) + " field '" + key +
                                "' must be a non-empty string");
  }
  return v;
}

inline bool IsIpv4Address(const std::string& address) {
  in_addr parsed{};
  return inet_pton(AF_INET, address.c_str(), &parsed) == 1;
}

inline int64_t ReadPositive(const toml::table& tbl, const char* key,
                            int64_t def, const char* table_name) {
  const int64_t v = Read(tbl, key, def);
  if (v < 1) {
    throw std::invalid_argument(std::string(table_name) + " field '" + key +
                                "' must be a positive integer");
  }
  return v;
}

inline uint16_t ReadPort(const toml::table& tbl, const char* key, uint16_t def,
                         const char* table_name) {
  if (!tbl.contains(key)) return def;
  auto v = tbl[key].value<int64_t>();
  if (!v || *v < 1 || *v > 65535) {
    throw std::invalid_argument(std::string(table_name) + " field '" + key +
                                "' must be a port in [1, 65535]");
  }
  return static_cast<uint16_t>(*v);
}

inline void CheckTopLevelKeys(const toml::table& tbl) {
  for (auto&& [k, v] : tbl) {
    if (k.str() != "include" && k.str() != "robot" && k.str() != "studio" &&
        k.str() != "subsystems") {
      throw std::invalid_argument("unknown top-level key '" +
                                  std::string(k.str()) + "'");
    }
  }
}

// A subsystem table also carries its node's own parameters (Node::param), so
// a key the framework does not know cannot be refused here (the launcher
// refuses one no node's --describe reports reading). A framework section
// spelled in the singular can: `[subsystems.arm.target]` would otherwise
// leave arm on the default target policy, and `[subsystems.arm.motor.x]`
// would leave the motor unconfigured, both silently. `where` names the table
// or the file holding it.
inline void CheckSubsystemKeys(const toml::table& tbl,
                               const std::string& where) {
  static constexpr std::pair<std::string_view, std::string_view> kSections[] = {
      {"target", "targets"},
      {"motor", "motors"},
      {"sensor", "sensors"},
      {"encoder", "encoders"},
      {"digital_input", "digital_inputs"},
      {"digital_output", "digital_outputs"},
      {"analog_input", "analog_inputs"},
      {"pwm_output", "pwm_outputs"},
      {"trajectory", "trajectories"},
  };
  static_assert(std::ranges::all_of(kSections, [](const auto& section) {
    return IsFrameworkSubsystemKey(section.second);
  }));
  for (auto&& [k, v] : tbl) {
    for (const auto& [wrong, right] : kSections) {
      if (k.str() == wrong) {
        throw std::invalid_argument(where + ": unknown key '" +
                                    std::string(wrong) + "'; did you mean '" +
                                    std::string(right) + "'?");
      }
    }
  }
}

// A subsystem file (or its defaults file) before merging, so the error names
// the file that holds the key: its top level and its [subsystem] header both
// land in the subsystem's table.
inline void CheckSubsystemFileKeys(const toml::table& file,
                                   const std::string& path) {
  CheckSubsystemKeys(file, path);
  if (const auto* header = file["subsystem"].as_table()) {
    CheckSubsystemKeys(*header, path + " [subsystem]");
  }
}

// The entries of `holder`'s [trajectories] table as (name, path), refusing a
// non-table section, a name that is not a topic segment (talOS/NAMING.md) and
// a value that is not a non-empty string. `where` names the file or table.
inline std::vector<std::pair<std::string, std::string>> ReadTrajectories(
    const toml::table& holder, const std::string& where) {
  std::vector<std::pair<std::string, std::string>> out;
  if (!holder.contains("trajectories")) return out;
  const auto* section = holder["trajectories"].as_table();
  if (section == nullptr) {
    throw std::invalid_argument(
        where + ": 'trajectories' must be a table of name = \"path\"");
  }
  for (auto&& [key, value] : *section) {
    const std::string name(key.str());
    const std::string field = where + ": trajectories." + name;
    if (auto problem =
            introspect::naming::CheckSegmentName(name, "trajectory name")) {
      throw std::invalid_argument(field + ": " + *problem);
    }
    const auto path = value.value_exact<std::string>();
    if (!path || path->empty()) {
      throw std::invalid_argument(field + " must be a non-empty string path");
    }
    out.emplace_back(name, *path);
  }
  return out;
}

// A [trajectories] file must exist; the error names where it was set, the
// key and the path as resolved.
inline void RequireTrajectoryFile(const std::string& where,
                                  const std::string& name,
                                  const std::string& path) {
  if (!std::filesystem::is_regular_file(path)) {
    throw std::invalid_argument(where + ": trajectories." + name +
                                ": no such file '" + path + "'");
  }
}

// Rewrites `holder`'s [trajectories] paths relative to `file`'s directory and
// requires each file. `data` sees each resolved path first, so a build tool
// can list it (resolve.cc) before its absence is an error.
template <typename DataFile>
void ResolveTrajectories(toml::table& holder, const std::string& file,
                         const std::string& where, DataFile& data) {
  namespace fs = std::filesystem;
  const fs::path base = fs::path(file).parent_path();
  for (auto& [name, path] : ReadTrajectories(holder, where)) {
    const fs::path rel{path};
    const std::string resolved =
        (rel.is_absolute() || base.empty() ? rel : base / rel)
            .lexically_normal()
            .string();
    holder["trajectories"].as_table()->insert_or_assign(name, resolved);
    data(resolved);
    RequireTrajectoryFile(where, name, resolved);
  }
}

// A subsystem (or defaults) file: its top level and its [subsystem] header.
template <typename DataFile>
void ResolveSubsystemFileTrajectories(toml::table& document,
                                      const std::string& file, DataFile& data) {
  ResolveTrajectories(document, file, file, data);
  if (auto* header = document["subsystem"].as_table()) {
    ResolveTrajectories(*header, file, file + " [subsystem]", data);
  }
}

// The [subsystems.<name>] tables of one robot file.
template <typename DataFile>
void ResolveSubsystemTablesTrajectories(toml::table& document,
                                        const std::string& file,
                                        DataFile& data) {
  if (auto* subs = document["subsystems"].as_table()) {
    for (auto&& [name, value] : *subs) {
      if (auto* sub = value.as_table()) {
        ResolveTrajectories(
            *sub, file, file + " [subsystems." + std::string(name.str()) + "]",
            data);
      }
    }
  }
}

// The default DataFile: config load needs only each file's existence.
inline void IgnoreDataFile(const std::string& /*path*/) {}

inline void MergeTables(toml::table& base, const toml::table& over) {
  for (auto&& [k, v] : over) {
    if (auto* over_sub = v.as_table()) {
      if (auto* base_sub = base[k.str()].as_table()) {
        MergeTables(*base_sub, *over_sub);
        continue;
      }
    }
    base.insert_or_assign(std::string(k.str()), v);
  }
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
  std::string follow_motor;
};

inline bool IsCANcoder(const RawDevice& d) {
  auto* s = std::get_if<hardware::SensorConfig>(&d.config);
  return s && s->kind == hardware::SensorKind::kCANcoder;
}

// A switch on the TalonFX's own limit input; absent keeps Phoenix's default,
// an enabled normally open switch.
inline hardware::LimitSwitch ReadLimitSwitch(const toml::table& t,
                                             const char* key,
                                             const std::string& name) {
  const auto value = Read<std::string>(t, key, "NormallyOpen");
  if (value == "NormallyOpen") return hardware::LimitSwitch::kNormallyOpen;
  if (value == "NormallyClosed") return hardware::LimitSwitch::kNormallyClosed;
  if (value == "Disabled") return hardware::LimitSwitch::kDisabled;
  throw std::invalid_argument(std::string(key) + " for " + name +
                              " must be \"NormallyOpen\", \"NormallyClosed\" "
                              "or \"Disabled\", not \"" +
                              value + "\"");
}

inline RawDevice ParseMotor(const toml::table& t, const std::string& name) {
  for (auto&& [key, value] : t) {
    const auto k = key.str();
    if (k != "type" && k != "bus" && k != "can_id" && k != "inverted" &&
        k != "brake" && k != "supply_limit_a" && k != "stator_limit_a" &&
        k != "max_voltage" && k != "rotor_to_sensor_ratio" &&
        k != "sensor_to_mechanism_ratio" && k != "continuous_wrap" &&
        k != "soft_limits" && k != "reverse_limit_rot" &&
        k != "forward_limit_rot" && k != "max_velocity_rps" &&
        k != "cruise_velocity_rps" && k != "acceleration_rps2" &&
        k != "jerk_rps3" && k != "follow" && k != "oppose_master" &&
        k != "feedback" && k != "feedback_sensor" && k != "slot0" &&
        k != "slot1" && k != "slot2" && k != "position_frame" &&
        k != "forward_limit_switch" && k != "reverse_limit_switch") {
      throw std::invalid_argument("unknown motor key '" + std::string(k) +
                                  "' for " + name);
    }
  }
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
  m.forward_limit_switch = ReadLimitSwitch(t, "forward_limit_switch", name);
  m.reverse_limit_switch = ReadLimitSwitch(t, "reverse_limit_switch", name);
  m.max_velocity_rps = Read(t, "max_velocity_rps", m.max_velocity_rps);
  m.cruise_velocity_rps =
      Read(t, "cruise_velocity_rps", std::min(20.0, m.max_velocity_rps));
  m.acceleration_rps2 = Read(t, "acceleration_rps2", m.acceleration_rps2);
  m.jerk_rps3 = Read(t, "jerk_rps3", m.jerk_rps3);
  for (int s = 0; s < 3; ++s) {
    const auto slot_name = "slot" + std::to_string(s);
    if (t.contains(slot_name) && !t[slot_name].is_table()) {
      throw std::invalid_argument(slot_name + " must be a table for " + name);
    }
    if (auto* slot = t["slot" + std::to_string(s)].as_table()) {
      for (auto&& [key, value] : *slot) {
        const auto k = key.str();
        if (k != "p" && k != "i" && k != "d" && k != "s" && k != "v" &&
            k != "a" && k != "g" && k != "gravity_type") {
          throw std::invalid_argument("unknown motor slot key '" +
                                      std::string(k) + "' for " + name);
        }
      }
      auto& g = m.slots[s];
      g.p = Read(*slot, "p", g.p);
      g.i = Read(*slot, "i", g.i);
      g.d = Read(*slot, "d", g.d);
      g.s = Read(*slot, "s", g.s);
      g.v = Read(*slot, "v", g.v);
      g.a = Read(*slot, "a", g.a);
      g.g = Read(*slot, "g", g.g);
      const auto gravity = Read<std::string>(*slot, "gravity_type", "Elevator");
      if (gravity == "Elevator" || gravity == "ElevatorStatic") {
        g.gravity_type = hardware::GravityType::kElevatorStatic;
      } else if (gravity == "Arm" || gravity == "ArmCosine") {
        g.gravity_type = hardware::GravityType::kArmCosine;
      } else {
        throw std::invalid_argument("unknown gravity_type for motor " + name +
                                    " slot" + std::to_string(s));
      }
    }
  }

  // The sensor is resolved to a logical id after numbering, so Validate
  // never sees the name; the parser checks the pairing.
  RawDevice raw{.name = name};
  raw.follow_motor = Read<std::string>(t, "follow", "");
  m.oppose_master = Read(t, "oppose_master", false);
  if (t.contains("oppose_master") && raw.follow_motor.empty()) {
    throw std::invalid_argument("oppose_master requires follow for " + name);
  }
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
  // A TalonFX reset restarts its rotor position, so only a remote CANcoder's
  // frame can be absolute (see MotorConfig::absolute_position_frame).
  const auto frame = Read<std::string>(t, "position_frame", "rezero");
  if (frame == "absolute") {
    if (m.feedback != hardware::Feedback::kRemoteCANcoder) {
      throw std::invalid_argument(
          "position_frame = \"absolute\" requires feedback = "
          "\"RemoteCANcoder\" for " +
          name + ": a TalonFX reset restarts its rotor position");
    }
    m.absolute_position_frame.value = true;
  } else if (frame != "rezero") {
    throw std::invalid_argument("position_frame for " + name +
                                " must be \"rezero\" or \"absolute\", not \"" +
                                frame + "\"");
  }
  // The TalonFX enforces its soft limits on its own position, in every
  // control mode, from thresholds applied once at configuration. A reset
  // restarts a rezero frame wherever the mechanism is, so those thresholds
  // would then sit elsewhere in the travel (limits [0, 50] after a reset at
  // 30 allow 30 more past the top), and no rezero can move them.
  if (m.soft_limits && !m.absolute_position_frame.value) {
    throw std::invalid_argument(
        "soft_limits = true for " + name +
        " is refused in a \"rezero\" position frame: a device reset restarts "
        "it, moving the soft limits along the travel. Set soft_limits = "
        "false, or use a remote CANcoder's absolute position frame "
        "(feedback = \"RemoteCANcoder\", position_frame = \"absolute\")");
  }
  raw.config = std::move(m);
  return raw;
}

inline void CheckDeviceKeys(const toml::table& t, const std::string& name,
                            std::string_view kind,
                            std::initializer_list<std::string_view> allowed) {
  for (auto&& [key, value] : t) {
    if (std::find(allowed.begin(), allowed.end(), key.str()) == allowed.end()) {
      std::string message = "unknown " + std::string(kind) + " key '" +
                            std::string(key.str()) + "' for " + name;
      if (key.source().path) {
        message += " in " + *key.source().path;
      }
      throw std::invalid_argument(message);
    }
  }
}

inline RawDevice ParseSensor(const toml::table& t, const std::string& name) {
  CheckDeviceKeys(t, name, "sensor",
                  {"type", "bus", "can_id", "inverted", "offset_rot"});
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
    CheckDeviceKeys(t, name, "encoder",
                    {"type", "dio_a", "dio_b", "counts_per_rev"});
    e.kind = hardware::EncoderKind::kQuadrature;
    e.dio_a = ReadRequired<int>(t, "dio_a", name);
    e.dio_b = ReadRequired<int>(t, "dio_b", name);
    e.counts_per_rev = Read(t, "counts_per_rev", e.counts_per_rev);
  } else if (type == "DutyCycleEncoder") {
    CheckDeviceKeys(t, name, "encoder", {"type", "dio", "offset_rot"});
    e.kind = hardware::EncoderKind::kDutyCycle;
    e.dio_a = ReadRequired<int>(t, "dio", name);
    e.offset_rot = Read(t, "offset_rot", e.offset_rot);
  } else {
    throw std::invalid_argument("unknown encoder type '" + type + "' for " +
                                name);
  }
  return {.name = name, .config = std::move(e)};
}

// Applies a [targets] table over `policy`: each listed mode is replaced. Keys
// are the Driver Station modes; values are arrays of source names, which are
// topic segments (talOS/NAMING.md) because they end a lane's topic.
inline void ApplyTargetPolicy(const toml::table& tbl, TargetPolicy& policy,
                              const std::string& where) {
  for (auto&& [key, value] : tbl) {
    const std::string mode{key.str()};
    std::vector<std::string>* lanes = mode == "teleop" ? &policy.teleop
                                      : mode == "autonomous"
                                          ? &policy.autonomous
                                      : mode == "test" ? &policy.test
                                                       : nullptr;
    if (lanes == nullptr) {
      throw std::invalid_argument(where + ": unknown mode '" + mode +
                                  "' (modes are teleop, autonomous, test)");
    }
    const auto* sources = value.as_array();
    if (sources == nullptr) {
      throw std::invalid_argument(where + "." + mode +
                                  " must be an array of source names");
    }
    std::vector<std::string> parsed;
    for (auto&& item : *sources) {
      const auto source = item.value_exact<std::string>();
      if (!source) {
        throw std::invalid_argument(where + "." + mode +
                                    " must be an array of source names");
      }
      if (auto problem =
              introspect::naming::CheckSegmentName(*source, "target source")) {
        throw std::invalid_argument(where + "." + mode + ": " + *problem);
      }
      if (std::find(parsed.begin(), parsed.end(), *source) != parsed.end()) {
        throw std::invalid_argument(where + "." + mode + " lists '" + *source +
                                    "' twice");
      }
      parsed.push_back(*source);
    }
    *lanes = std::move(parsed);
  }
}

inline std::vector<std::pair<std::string, TeleopValue>> ParseTeleopAssignments(
    const toml::table& table, const std::string& where) {
  std::vector<std::pair<std::string, TeleopValue>> assignments;
  std::vector<std::string> keys;
  for (auto&& [key, value] : table) keys.emplace_back(key.str());
  std::sort(keys.begin(), keys.end());
  for (const auto& key : keys) {
    const auto& value = table[key];
    if (const auto* flag = value.as_boolean()) {
      assignments.emplace_back(key, flag->get());
    } else if (const auto* integer = value.as_integer()) {
      assignments.emplace_back(key, integer->get());
    } else if (const auto* real = value.as_floating_point()) {
      assignments.emplace_back(key, real->get());
    } else if (const auto* text = value.as_string()) {
      assignments.emplace_back(key, text->get());
    } else {
      throw std::invalid_argument(where + "." + key + " must be a scalar");
    }
  }
  return assignments;
}

inline std::vector<TeleopBinding> ParseTeleop(const toml::table& sub_tbl,
                                              const std::string& sub_name) {
  std::vector<TeleopBinding> result;
  if (!sub_tbl.contains("teleop")) return result;
  const auto& teleop_node = sub_tbl["teleop"];
  const auto* teleop_tbl = teleop_node.as_table();
  if (teleop_tbl == nullptr) {
    throw std::invalid_argument("[subsystems." + sub_name +
                                ".teleop] must be a table");
  }
  for (auto&& [k, v] : *teleop_tbl) {
    if (k.str() != "bindings" && k.str() != "defaults") {
      throw std::invalid_argument("[subsystems." + sub_name +
                                  ".teleop] unknown key '" +
                                  std::string(k.str()) + "'");
    }
  }
  if (!teleop_tbl->contains("bindings")) return result;
  const auto* bindings = (*teleop_tbl)["bindings"].as_array();
  if (bindings == nullptr) {
    throw std::invalid_argument("[subsystems." + sub_name +
                                ".teleop.bindings] must be an array");
  }
  for (std::size_t i = 0; i < bindings->size(); ++i) {
    const std::string entry_where = "[subsystems." + sub_name +
                                    "] teleop.bindings[" + std::to_string(i) +
                                    "]";
    const auto* entry = (*bindings)[i].as_table();
    if (entry == nullptr) {
      throw std::invalid_argument(entry_where + " must be a table");
    }
    for (auto&& [k, v] : *entry) {
      const std::string_view ks = k.str();
      if (ks != "button" && ks != "axis" && ks != "port" && ks != "pov" &&
          ks != "pov_index" && ks != "on_toggle" && ks != "while_held" &&
          ks != "field" && ks != "scale" && ks != "deadband_frac") {
        throw std::invalid_argument(entry_where + " unknown key '" +
                                    std::string(ks) + "'");
      }
    }
    const bool has_button = entry->contains("button");
    const bool has_axis = entry->contains("axis");
    const bool has_pov = entry->contains("pov");
    if (static_cast<int>(has_button) + static_cast<int>(has_axis) +
            static_cast<int>(has_pov) !=
        1) {
      throw std::invalid_argument(
          entry_where +
          ": binding must specify exactly one of button/axis/pov");
    }
    TeleopBinding b;
    if (entry->contains("pov_index") && !has_pov) {
      throw std::invalid_argument(entry_where + ".pov_index requires 'pov'");
    }
    if (has_pov) {
      const auto angle = (*entry)["pov"].value<int64_t>();
      if (!(*entry)["pov"].is_integer() || !angle || *angle < 0 ||
          *angle >= 360 || *angle % 45 != 0) {
        throw std::invalid_argument(entry_where +
                                    ".pov must be a compass angle");
      }
      b.pov = static_cast<int>(*angle);
      if (entry->contains("pov_index")) {
        const auto index = (*entry)["pov_index"].value<int64_t>();
        if (!(*entry)["pov_index"].is_integer() || !index || *index < 0 ||
            *index >= static_cast<int64_t>(driver_station::kMaxPovs)) {
          throw std::invalid_argument(entry_where + ".pov_index out of range");
        }
        b.pov_index = static_cast<int>(*index);
      }
    }
    if (entry->contains("port")) {
      const auto& p_node = (*entry)["port"];
      if (!p_node.is_integer()) {
        throw std::invalid_argument(
            entry_where + ".port must be an integer in [0, " +
            std::to_string(driver_station::kMaxJoysticks - 1) + "]");
      }
      const auto p = p_node.value<int64_t>();
      if (!p || *p < 0 ||
          *p >= static_cast<int64_t>(driver_station::kMaxJoysticks)) {
        throw std::invalid_argument(
            entry_where + ".port must be an integer in [0, " +
            std::to_string(driver_station::kMaxJoysticks - 1) + "]");
      }
      b.port = static_cast<int>(*p);
    }
    if (has_button || has_pov) {
      for (const char* forbidden : {"field", "scale", "deadband_frac"}) {
        if (entry->contains(forbidden)) {
          throw std::invalid_argument(entry_where +
                                      " button binding cannot have axis keys");
        }
      }
      if (has_button) {
        const auto& b_node = (*entry)["button"];
        if (!b_node.is_integer()) {
          throw std::invalid_argument(
              entry_where + ".button must be an integer in [1, " +
              std::to_string(driver_station::kMaxButtons) + "]");
        }
        const auto btn = b_node.value<int64_t>();
        if (!btn || *btn < 1 ||
            *btn > static_cast<int64_t>(driver_station::kMaxButtons)) {
          throw std::invalid_argument(
              entry_where + ".button must be an integer in [1, " +
              std::to_string(driver_station::kMaxButtons) + "]");
        }
        b.button = static_cast<int>(*btn);
      }
      if (entry->contains("while_held") == entry->contains("on_toggle")) {
        throw std::invalid_argument(
            entry_where + " requires exactly one of while_held/on_toggle");
      }
      const std::string action =
          entry->contains("on_toggle") ? "on_toggle" : "while_held";
      auto& assignments = action == "on_toggle" ? b.on_toggle : b.while_held;
      const auto* wh = (*entry)[action].as_table();
      if (wh == nullptr || wh->empty()) {
        throw std::invalid_argument(entry_where + "." + action +
                                    " must be a non-empty table");
      }
      assignments = ParseTeleopAssignments(*wh, entry_where + "." + action);
    }
    if (has_axis) {
      if (entry->contains("while_held") || entry->contains("on_toggle")) {
        throw std::invalid_argument(entry_where +
                                    " axis binding cannot have action tables");
      }
      const auto& a_node = (*entry)["axis"];
      if (!a_node.is_integer()) {
        throw std::invalid_argument(
            entry_where + ".axis must be an integer in [0, " +
            std::to_string(driver_station::kMaxAxes - 1) + "]");
      }
      const auto a = a_node.value<int64_t>();
      if (!a || *a < 0 ||
          *a >= static_cast<int64_t>(driver_station::kMaxAxes)) {
        throw std::invalid_argument(
            entry_where + ".axis must be an integer in [0, " +
            std::to_string(driver_station::kMaxAxes - 1) + "]");
      }
      b.axis = static_cast<int>(*a);
      if (!entry->contains("field")) {
        throw std::invalid_argument(entry_where +
                                    " axis binding missing 'field'");
      }
      const auto f = (*entry)["field"].value<std::string_view>();
      if (!f || f->empty()) {
        throw std::invalid_argument(entry_where +
                                    ".field must be a non-empty string");
      }
      b.field = std::string(*f);
      if (!entry->contains("scale")) {
        throw std::invalid_argument(entry_where +
                                    " axis binding missing 'scale'");
      }
      const auto& sc = (*entry)["scale"];
      if (!sc.is_floating_point() && !sc.is_integer()) {
        throw std::invalid_argument(entry_where + ".scale must be a number");
      }
      b.scale = sc.is_floating_point()
                    ? sc.as_floating_point()->get()
                    : static_cast<double>(sc.as_integer()->get());
      if (entry->contains("deadband_frac")) {
        const auto& df = (*entry)["deadband_frac"];
        if (!df.is_floating_point() && !df.is_integer()) {
          throw std::invalid_argument(entry_where +
                                      ".deadband_frac must be a number");
        }
        const double d = df.as_floating_point()
                             ? df.as_floating_point()->get()
                             : static_cast<double>(df.as_integer()->get());
        if (d < 0.0 || d >= 1.0) {
          throw std::invalid_argument(entry_where +
                                      ".deadband_frac must be in [0, 1)");
        }
        b.deadband_frac = d;
      }
    }
    result.push_back(std::move(b));
  }
  return result;
}

}  // namespace detail

// A port flag's value: a decimal integer in [1, 65535], else
// std::invalid_argument naming `flag` (std::stoi would accept "5802x" and a
// uint16_t cast would wrap 70000 to 4464).
inline uint16_t ParsePortFlag(std::string_view flag, std::string_view value) {
  int port = 0;
  const auto* end = value.data() + value.size();
  const auto [ptr, ec] = std::from_chars(value.data(), end, port);
  if (value.empty() || ec != std::errc{} || ptr != end || port < 1 ||
      port > 65535) {
    throw std::invalid_argument(std::string(flag) +
                                " must be a port in [1, 65535], not '" +
                                std::string(value) + "'");
  }
  return static_cast<uint16_t>(port);
}

// Effective controller endpoint: an explicit CLI value wins, otherwise the
// deployment configuration. A zero CLI port means "not given". A CLI address
// that is not an IPv4 literal throws std::invalid_argument, as config load
// does for controller_address.
inline std::string ResolveControllerAddress(const RobotConfig& config,
                                            const std::string& cli_override) {
  if (cli_override.empty()) return config.controller_address;
  if (!detail::IsIpv4Address(cli_override)) {
    throw std::invalid_argument("--remote must be an IPv4 address, not '" +
                                cli_override + "'");
  }
  return cli_override;
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
// what GetSubsystemTable answers from. Build-time resolution can parse an
// incomplete hardware inventory; only runtime requires a deployable inventory.
inline RobotConfig ParseRobotConfigTable(toml::table tbl,
                                         bool validate_hardware = true) {
  using detail::Read;
  using detail::ReadRequired;
  detail::CheckTopLevelKeys(tbl);
  RobotConfig result;
  result.toml_data = tbl;
  auto& hw = result.hardware;

  if (tbl.contains("robot") && !tbl["robot"].is_table()) {
    throw std::invalid_argument("[robot] must be a table");
  }
  if (auto* robot = tbl["robot"].as_table()) {
    hw.period_us = Read(*robot, "period_us", hw.period_us);
    hw.command_timeout_us =
        Read(*robot, "command_timeout_us", hw.command_timeout_us);
    hw.status_hz = Read(*robot, "status_hz", hw.status_hz);
    hw.commissioned = Read(*robot, "commissioned", hw.commissioned);
    result.require_target_coverage =
        Read(*robot, "require_target_coverage", result.require_target_coverage);
    result.controller_address = detail::ReadNonEmpty(
        *robot, "controller_address", result.controller_address, "[robot]");
    if (!detail::IsIpv4Address(result.controller_address)) {
      throw std::invalid_argument(
          "[robot] field 'controller_address' must be an IPv4 address such "
          "as \"10.56.87.2\", not '" +
          result.controller_address + "': the bridge resolves no hostname");
    }
    result.controller_port = detail::ReadPort(
        *robot, "controller_port", result.controller_port, "[robot]");
    result.local_port =
        detail::ReadPort(*robot, "local_port", result.local_port, "[robot]");
    result.log_keep_sessions = detail::ReadPositive(
        *robot, "log_keep_sessions", result.log_keep_sessions, "[robot]");
    result.log_max_bytes = detail::ReadPositive(
        *robot, "log_max_bytes", result.log_max_bytes, "[robot]");
    if (robot->contains("targets")) {
      const auto* targets = (*robot)["targets"].as_table();
      if (targets == nullptr) {
        throw std::invalid_argument("[robot.targets] must be a table");
      }
      detail::ApplyTargetPolicy(*targets, result.targets, "[robot.targets]");
    }
  }

  if (tbl.contains("studio")) {
    auto* studio = tbl["studio"].as_table();
    if (!studio) {
      throw std::invalid_argument("[studio] must be a table");
    }
    for (auto&& [k, v] : *studio) {
      if (k.str() != "bridge" && k.str() != "webroot" &&
          k.str() != "http_port" && k.str() != "topic" &&
          k.str() != "pose_topic" && k.str() != "max_clients") {
        throw std::invalid_argument("[studio] unknown key '" +
                                    std::string(k.str()) + "'");
      }
    }
    auto& s = result.studio;
    s.bridge = detail::ReadNonEmpty(*studio, "bridge", s.bridge, "[studio]");
    s.webroot = detail::ReadNonEmpty(*studio, "webroot", s.webroot, "[studio]");
    s.http_port =
        detail::ReadPort(*studio, "http_port", s.http_port, "[studio]");
    if (studio->contains("max_clients")) {
      const auto clients = (*studio)["max_clients"].value<int64_t>();
      if (!clients || *clients < 1 || *clients > 64) {
        throw std::invalid_argument(
            "[studio] field 'max_clients' must be an integer in [1, 64]");
      }
      s.max_clients = static_cast<uint16_t>(*clients);
    }
    if (studio->contains("topic")) {
      s.topic = detail::ReadNonEmpty(*studio, "topic", "", "[studio]");
    }
    if (studio->contains("pose_topic")) {
      s.pose_topic =
          detail::ReadNonEmpty(*studio, "pose_topic", "", "[studio]");
    }
  }

  auto* subsystems_tbl = tbl["subsystems"].as_table();
  if (!subsystems_tbl) {
    throw std::invalid_argument("missing [subsystems] table");
  }

  std::vector<detail::RawDevice> raw_devices;
  for (auto&& [sub_key, sub_val] : *subsystems_tbl) {
    const std::string sub_name(sub_key.str());
    auto* sub_tbl = sub_val.as_table();
    if (!sub_tbl) {
      throw std::invalid_argument("[subsystems." + sub_name +
                                  "] must be a table");
    }
    result.subsystems[sub_name].subsystem = sub_name;
    detail::CheckSubsystemKeys(*sub_tbl, "[subsystems." + sub_name + "]");
    // The launcher starts a subsystem's `node`; a malformed one must not
    // silently leave the subsystem without a process.
    if (sub_tbl->contains("node")) {
      detail::ReadNonEmpty(*sub_tbl, "node", "",
                           ("[subsystems." + sub_name + "]").c_str());
    }
    // Fail closed: unknown nodes are required for safe robot operation.
    detail::Read<bool>(*sub_tbl, "critical", true);
    TargetPolicy& policy = result.subsystem_targets[sub_name] = result.targets;
    if (sub_tbl->contains("targets")) {
      const auto* targets = (*sub_tbl)["targets"].as_table();
      const std::string where = "[subsystems." + sub_name + ".targets]";
      if (targets == nullptr) {
        throw std::invalid_argument(where + " must be a table");
      }
      detail::ApplyTargetPolicy(*targets, policy, where);
    }
    {
      const std::string where = "[subsystems." + sub_name + "]";
      for (auto& [name, path] : detail::ReadTrajectories(*sub_tbl, where)) {
        detail::RequireTrajectoryFile(where, name, path);
        result.trajectories[sub_name].insert_or_assign(name, std::move(path));
      }
    }
    auto bindings = detail::ParseTeleop(*sub_tbl, sub_name);
    if (!bindings.empty()) {
      result.teleop[sub_name] = std::move(bindings);
    }
    if (const auto* teleop = (*sub_tbl)["teleop"].as_table();
        teleop != nullptr && teleop->contains("defaults")) {
      const auto* defaults = (*teleop)["defaults"].as_table();
      const std::string where = "[subsystems." + sub_name + "] teleop.defaults";
      if (defaults == nullptr) {
        throw std::invalid_argument(where + " must be a table");
      }
      result.teleop_defaults[sub_name] =
          detail::ParseTeleopAssignments(*defaults, where);
    }
    auto each = [&](const char* section, auto parse) {
      auto* devices = (*sub_tbl)[section].as_table();
      if (!devices) {
        if (sub_tbl->contains(section)) {
          throw std::invalid_argument(std::string(section) +
                                      " must be a table");
        }
        return;
      }
      for (auto&& [dev_key, dev_val] : *devices) {
        const std::string name(dev_key.str());
        auto* dev_tbl = dev_val.as_table();
        if (!dev_tbl) {
          throw std::invalid_argument(std::string(section) + " entry '" + name +
                                      "' must be a table");
        }
        try {
          detail::RawDevice raw = parse(*dev_tbl, name);
          raw.subsystem = sub_name;
          raw.name = name;
          raw_devices.push_back(std::move(raw));
        } catch (const std::invalid_argument& error) {
          throw std::invalid_argument("[subsystems." + sub_name + "]." +
                                      section + "." + name + ": " +
                                      error.what());
        }
      }
    };
    each("motors", detail::ParseMotor);
    each("sensors", detail::ParseSensor);
    each("encoders", detail::ParseEncoder);
    each("digital_inputs", [](const toml::table& t, const std::string& n) {
      detail::CheckDeviceKeys(t, n, "digital input", {"dio"});
      return detail::RawDevice{.config = hardware::DigitalInputConfig{
                                   .dio = ReadRequired<int>(t, "dio", n)}};
    });
    each("digital_outputs", [](const toml::table& t, const std::string& n) {
      detail::CheckDeviceKeys(t, n, "digital output", {"dio"});
      return detail::RawDevice{.config = hardware::DigitalOutputConfig{
                                   .dio = ReadRequired<int>(t, "dio", n)}};
    });
    each("analog_inputs", [](const toml::table& t, const std::string& n) {
      detail::CheckDeviceKeys(t, n, "analog input", {"channel"});
      return detail::RawDevice{
          .config = hardware::AnalogInputConfig{
              .channel = ReadRequired<int>(t, "channel", n)}};
    });
    each("pwm_outputs", [](const toml::table& t, const std::string& n) {
      detail::CheckDeviceKeys(t, n, "PWM output", {"channel"});
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
      throw std::invalid_argument("cross-subsystem device claim: '" + claimant +
                                  "' in subsystem '" + subsystem +
                                  "' references '" + target.name +
                                  "' in subsystem '" + target.subsystem + "'");
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
  for (auto& dev : raw_devices) {
    auto* motor = std::get_if<hardware::MotorConfig>(&dev.config);
    if (!motor || dev.follow_motor.empty()) continue;
    const auto* master = find(dev.follow_motor);
    if (!master ||
        !std::holds_alternative<hardware::MotorConfig>(master->config)) {
      throw std::invalid_argument("follow motor not found: " +
                                  dev.follow_motor);
    }
    require_same_subsystem(*master, dev.subsystem, dev.name);
    motor->master_id = std::get<hardware::MotorConfig>(master->config).id;
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
      auto check_ref = [&](const char* field,
                           bool (*is_kind)(const detail::RawDevice&)) {
        auto ref = (*mod)[field].value<std::string_view>();
        if (!ref) return;
        const auto* found = find(std::string(*ref));
        if (!found) {
          throw std::invalid_argument(
              "geometry module references unknown device: " +
              std::string(*ref));
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
          } else if constexpr (std::is_same_v<T,
                                              hardware::DigitalInputConfig>) {
            add(hw.digital_inputs, owned.digital_inputs);
          } else if constexpr (std::is_same_v<T,
                                              hardware::DigitalOutputConfig>) {
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

  if (validate_hardware) {
    hardware::Validate(hw);
    // The bridge holds an actuator owner's last request for
    // max(2 x period_us, command_timeout_us / 2), and the merged command
    // carries the oldest observation it holds, which the gateway refuses once
    // that is command_timeout_us old. At period_us <= command_timeout_us / 4
    // the owner timeout is half the timeout, so a silent owner leaves the
    // merge while its last observation can still be younger than the
    // gateway's limit; a slower owner would hold it longer and expire every
    // owner's command with its own.
    for (const auto& [name, owned] : result.subsystems) {
      if (!owned.owns_actuators()) continue;
      const auto period_us = Read(*result.GetSubsystemTable(name), "period_us",
                                  int64_t{hw.period_us});
      if (period_us > int64_t{hw.command_timeout_us / 4}) {
        throw std::invalid_argument(
            "[subsystems." + name + "] owns actuators, so its period_us (" +
            std::to_string(period_us) +
            ") must be at most a quarter of [robot] command_timeout_us (" +
            std::to_string(hw.command_timeout_us) + ")");
      }
    }
  }
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
    return toml::parse(ss.str(), path);
  } catch (const toml::parse_error& err) {
    throw std::invalid_argument("TOML parse error in " + path + ": " +
                                err.what());
  }
}

// Replaces a [[subsystems]] manifest with the canonical [subsystems.<name>]
// view: each listed file's [subsystem] scalars (minus the name) plus its
// other top-level tables, then the entry's optional `node` over the file's.
// Validation then runs once over the merged view, so splitting files cannot
// weaken a check.
inline void NormalizeManifestPaths(toml::array& manifest,
                                   const std::string& file_path) {
  namespace fs = std::filesystem;
  const fs::path base = fs::path(file_path).parent_path();
  if (base.empty()) return;
  for (auto&& entry_node : manifest) {
    auto* entry = entry_node.as_table();
    if (!entry) continue;
    for (const char* key : {"path", "defaults"}) {
      if (auto val = (*entry)[key].value<std::string_view>();
          val && !val->empty()) {
        const fs::path rel{std::string(*val)};
        if (!rel.is_absolute()) {
          entry->insert_or_assign(key, (base / rel).string());
        }
      }
    }
  }
}

inline void MergeManifests(toml::array& base_arr, const toml::array& child_arr,
                           const std::string& child_path) {
  toml::table seen_in_child;
  for (auto&& child_node : child_arr) {
    auto* child_entry = child_node.as_table();
    auto name_view = child_entry
                         ? (*child_entry)["name"].value<std::string_view>()
                         : std::nullopt;
    if (!name_view || name_view->empty()) {
      throw std::invalid_argument("invalid [[subsystems]] entry in " +
                                  child_path + ": needs 'name'");
    }
    const std::string name(*name_view);
    if (seen_in_child.contains(name)) {
      throw std::invalid_argument("duplicate [[subsystems]] entry in " +
                                  child_path + ": '" + name + "'");
    }
    seen_in_child.insert_or_assign(name, true);

    toml::table* matched = nullptr;
    for (auto&& base_node : base_arr) {
      auto* base_entry = base_node.as_table();
      if (base_entry &&
          (*base_entry)["name"].value<std::string_view>() == name) {
        matched = base_entry;
        break;
      }
    }
    if (matched) {
      for (auto&& [k, v] : *child_entry) {
        matched->insert_or_assign(std::string(k.str()), v);
      }
    } else {
      base_arr.push_back(*child_entry);
    }
  }
}

template <typename Reader = decltype(&ParseTomlFile),
          typename DataFile = decltype(&IgnoreDataFile)>
inline toml::table ExpandManifest(const toml::array& manifest,
                                  const std::string& path,
                                  bool paths_normalized = false,
                                  Reader read = ParseTomlFile,
                                  DataFile data = IgnoreDataFile) {
  namespace fs = std::filesystem;
  const fs::path base = fs::path(path).parent_path();
  auto resolve_entry_path = [&](std::string_view p) {
    const fs::path rel{std::string(p)};
    if (paths_normalized || rel.is_absolute() || base.empty()) {
      return rel.string();
    }
    return (base / rel).string();
  };
  toml::table subs_view;
  for (auto&& entry_node : manifest) {
    auto* entry = entry_node.as_table();
    auto name_view =
        entry ? (*entry)["name"].value<std::string_view>() : std::nullopt;
    auto rel_view =
        entry ? (*entry)["path"].value<std::string_view>() : std::nullopt;
    if (!name_view || !rel_view || name_view->empty() || rel_view->empty()) {
      throw std::invalid_argument("invalid [[subsystems]] entry in " + path +
                                  ": needs 'name' and 'path'");
    }
    const std::string name(*name_view);
    for (auto&& [k, v] : *entry) {
      if (k.str() != "name" && k.str() != "path" && k.str() != "node" &&
          k.str() != "defaults") {
        throw std::invalid_argument("unknown key '" + std::string(k.str()) +
                                    "' in [[subsystems]] entry '" + name +
                                    "' in " + path +
                                    " (expected name, path, node, defaults)");
      }
    }
    std::optional<std::string> node;
    if (entry->contains("node")) {
      node = ReadNonEmpty(*entry, "node", "", "[[subsystems]] entry");
    }
    std::optional<std::string> entry_defaults;
    if (entry->contains("defaults")) {
      entry_defaults = resolve_entry_path(
          ReadNonEmpty(*entry, "defaults", "", "[[subsystems]] entry"));
    }
    const std::string file = resolve_entry_path(*rel_view);
    toml::table sub_file = read(file);
    CheckSubsystemFileKeys(sub_file, file);
    ResolveSubsystemFileTrajectories(sub_file, file, data);

    auto* header = sub_file["subsystem"].as_table();
    if (!header || (*header)["name"].value<std::string_view>() != name) {
      throw std::invalid_argument(
          file + ": [subsystem].name must match manifest entry '" + name + "'");
    }
    if (subs_view.contains(name)) {
      throw std::invalid_argument("duplicate [[subsystems]] entry in " + path +
                                  ": '" + name + "'");
    }
    std::optional<std::string> defaults_file;
    if (header->contains("defaults")) {
      const fs::path sub_base = fs::path(file).parent_path();
      const fs::path def_rel{
          ReadNonEmpty(*header, "defaults", "", "[subsystem]")};
      defaults_file =
          (def_rel.is_absolute() || sub_base.empty() ? def_rel
                                                     : sub_base / def_rel)
              .string();
    }
    if (entry_defaults) {
      defaults_file = *entry_defaults;
    }
    if (defaults_file) {
      toml::table merged = read(*defaults_file);
      CheckSubsystemFileKeys(merged, *defaults_file);
      ResolveSubsystemFileTrajectories(merged, *defaults_file, data);
      MergeTables(merged, sub_file);
      sub_file = std::move(merged);
      header = sub_file["subsystem"].as_table();
    }
    toml::table view;
    for (auto&& [k, v] : *header) {
      if (k.str() != "name" && k.str() != "defaults") {
        view.insert_or_assign(std::string(k.str()), v);
      }
    }
    for (auto&& [k, v] : sub_file) {
      if (k.str() != "subsystem")
        view.insert_or_assign(std::string(k.str()), v);
    }
    if (node) view.insert_or_assign("node", *node);
    subs_view.insert_or_assign(name, std::move(view));
  }
  return subs_view;
}

}  // namespace detail

// Resolve once for runtime and build tools. Readers may track dependencies or
// supply pending file contents without changing include/manifest semantics.
// `data` sees every non-TOML file the config names ([trajectories]), resolved
// against the file that names it, before config load requires it to exist.
template <typename Reader = decltype(&detail::ParseTomlFile),
          typename DataFile = decltype(&detail::IgnoreDataFile)>
inline toml::table ResolveRobotConfig(const std::string& path,
                                      Reader read = detail::ParseTomlFile,
                                      DataFile data = detail::IgnoreDataFile) {
  namespace fs = std::filesystem;
  toml::table tbl = read(path);
  detail::CheckTopLevelKeys(tbl);
  detail::ResolveSubsystemTablesTrajectories(tbl, path, data);
  if (auto* manifest = tbl["subsystems"].as_array()) {
    detail::NormalizeManifestPaths(*manifest, path);
  }
  if (tbl.contains("include")) {
    const std::string inc_str =
        detail::ReadNonEmpty(tbl, "include", "", "top-level");
    const fs::path base_dir = fs::path(path).parent_path();
    const fs::path inc_rel{inc_str};
    const std::string inc_file =
        (inc_rel.is_absolute() || base_dir.empty() ? inc_rel
                                                   : base_dir / inc_rel)
            .string();
    toml::table base = read(inc_file);
    detail::CheckTopLevelKeys(base);
    detail::ResolveSubsystemTablesTrajectories(base, inc_file, data);
    if (base.contains("include")) {
      throw std::invalid_argument("nested 'include' is not supported in " +
                                  inc_file);
    }
    if (auto* base_manifest = base["subsystems"].as_array()) {
      detail::NormalizeManifestPaths(*base_manifest, inc_file);
    }
    tbl.erase("include");
    if (auto* base_subs = base["subsystems"].as_array()) {
      if (auto* child_subs = tbl["subsystems"].as_array()) {
        detail::MergeManifests(*base_subs, *child_subs, path);
        tbl.erase("subsystems");
      } else if (tbl.contains("subsystems")) {
        base.insert_or_assign(
            "subsystems",
            detail::ExpandManifest(*base_subs, inc_file,
                                   /*paths_normalized=*/true, read, data));
      }
    }
    detail::MergeTables(base, tbl);
    tbl = std::move(base);
  }
  if (auto* manifest = tbl["subsystems"].as_array()) {
    tbl.insert_or_assign(
        "subsystems",
        detail::ExpandManifest(*manifest, path,
                               /*paths_normalized=*/true, read, data));
  }
  return tbl;
}

inline RobotConfig ParseRobotConfig(const std::string& path) {
  return ParseRobotConfigTable(ResolveRobotConfig(path));
}

// The files of the config at `path` (itself, its include, subsystem and
// defaults files) that set `key` (a dotted path, "camera.fps") of
// `subsystem`'s table, for a diagnostic to name: a [subsystems.<subsystem>]
// table, or a subsystem file whose [subsystem] names it, or a defaults file
// (whose [subsystem] names none). Empty when the config no longer resolves.
inline std::vector<std::string> SubsystemKeyFiles(const std::string& path,
                                                  std::string_view subsystem,
                                                  std::string_view key) {
  std::vector<std::string> out;
  const auto read = [&](const std::string& file) {
    toml::table document = detail::ParseTomlFile(file);
    bool sets = false;
    if (const auto* tables = document["subsystems"].as_table()) {
      if (const auto* table = (*tables)[subsystem].as_table()) {
        sets = static_cast<bool>(table->at_path(key));
      }
    } else if (const auto* header = document["subsystem"].as_table()) {
      const auto name = (*header)["name"].value<std::string_view>();
      sets = (!name || *name == subsystem) &&
             (document.at_path(key) || header->at_path(key));
    }
    if (sets && std::ranges::find(out, file) == out.end()) out.push_back(file);
    return document;
  };
  try {
    ResolveRobotConfig(path, read);
  } catch (const std::exception&) {
    return {};
  }
  return out;
}

}  // namespace talos::config
