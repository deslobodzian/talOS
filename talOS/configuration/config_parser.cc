#include "talOS/configuration/config_parser.h"

#include <arpa/inet.h>

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <functional>
#include <initializer_list>
#include <map>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/config.h"
#include "talOS/introspection/names.h"

namespace talos::config {

namespace {

using ReaderCallback = std::function<toml::table(const std::string&)>;
using DataFileCallback =
    std::function<void(const std::string&, const std::string&)>;

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
    if (auto v = node.value<double>()) return static_cast<T>(*v);
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

std::string ReadNonEmpty(const toml::table& tbl, const char* key,
                         std::string def, const char* table_name) {
  std::string v = Read<std::string>(tbl, key, std::move(def));
  if (v.empty()) {
    throw std::invalid_argument(std::string(table_name) + " field '" + key +
                                "' must be a non-empty string");
  }
  return v;
}

bool IsIpv4Address(const std::string& address) {
  in_addr parsed{};
  return inet_pton(AF_INET, address.c_str(), &parsed) == 1;
}

int64_t ReadPositive(const toml::table& tbl, const char* key, int64_t def,
                     const char* table_name) {
  const int64_t v = Read(tbl, key, def);
  if (v < 1) {
    throw std::invalid_argument(std::string(table_name) + " field '" + key +
                                "' must be a positive integer");
  }
  return v;
}

uint16_t ReadPort(const toml::table& tbl, const char* key, uint16_t def,
                  const char* table_name) {
  if (!tbl.contains(key)) return def;
  auto v = tbl[key].value<int64_t>();
  if (!v || *v < 1 || *v > 65535) {
    throw std::invalid_argument(std::string(table_name) + " field '" + key +
                                "' must be a port in [1, 65535]");
  }
  return static_cast<uint16_t>(*v);
}

void CheckTopLevelKeys(const toml::table& tbl) {
  for (auto&& [k, v] : tbl) {
    if (k.str() != "include" && k.str() != "robot" && k.str() != "studio" &&
        k.str() != "simulation" && k.str() != "subsystems") {
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
void CheckSubsystemKeys(const toml::table& tbl, const std::string& where) {
  for (auto&& [k, v] : tbl) {
    for (const auto& [wrong, right] : detail::kSections) {
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
void CheckSubsystemFileKeys(const toml::table& file, const std::string& path) {
  CheckSubsystemKeys(file, path);
  if (const auto* header = file["subsystem"].as_table()) {
    CheckSubsystemKeys(*header, path + " [subsystem]");
  }
}

// The entries of `holder`'s [trajectories] table as (name, path), refusing a
// non-table section, a name that is not a topic segment (talOS/NAMING.md) and
// a value that is not a non-empty string. `where` names the file or table.
std::vector<std::pair<std::string, std::string>> ReadTrajectories(
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
void RequireTrajectoryFile(const std::string& where, const std::string& name,
                           const std::string& path) {
  if (!std::filesystem::is_regular_file(path)) {
    throw std::invalid_argument(where + ": trajectories." + name +
                                ": no such file '" + path + "'");
  }
}

// Where each [trajectories] entry (by name) was set, for the errors of the
// check that runs once files are merged.
using TrajectorySources = std::map<std::string, std::string>;

// Rewrites `holder`'s [trajectories] paths relative to `file`'s directory,
// validating each entry, and records into `sources` where each was set
// (`where`). Existence is checked later, on the merged result, so an entry a
// later file overrides is never required or listed.
void RebaseTrajectories(toml::table& holder, const std::string& file,
                        const std::string& where, TrajectorySources& sources) {
  namespace fs = std::filesystem;
  const fs::path base = fs::path(file).parent_path();
  auto entries = ReadTrajectories(holder, where);
  auto* section = holder["trajectories"].as_table();
  if (section == nullptr) return;
  for (auto& [name, path] : entries) {
    const fs::path rel{path};
    const std::string resolved =
        (rel.is_absolute() || base.empty() ? rel : base / rel)
            .lexically_normal()
            .string();
    section->insert_or_assign(name, resolved);
    sources.insert_or_assign(name, where);
  }
}

// The effective entries of `holder`'s [trajectories] (already rebased):
// `data` sees each path with its label (where it was set and
// trajectories.<name>), so a build tool can list it (resolve.cc) before its
// absence is an error; then the file must exist.
void RequireTrajectories(const toml::table& holder,
                         const TrajectorySources& sources,
                         const std::string& fallback,
                         const DataFileCallback& data) {
  for (const auto& [name, path] : ReadTrajectories(holder, fallback)) {
    const auto source = sources.find(name);
    const std::string& where =
        source == sources.end() ? fallback : source->second;
    data(path, where + ": trajectories." + name);
    RequireTrajectoryFile(where, name, path);
  }
}

// [studio] dashboard names a file relative to the file that sets it:
// rewrites the path in `document` (read from `file`) against `file`'s
// directory and returns `file`, or "" when `document` does not set it.
std::string RebaseStudioDashboard(toml::table& document,
                                  const std::string& file) {
  auto* studio = document["studio"].as_table();
  if (studio == nullptr || !studio->contains("dashboard")) return {};
  const auto value = (*studio)["dashboard"].value_exact<std::string>();
  if (!value || value->empty()) {
    throw std::invalid_argument(
        file + ": [studio] dashboard must be a non-empty string path");
  }
  const std::filesystem::path base = std::filesystem::path(file).parent_path();
  const std::filesystem::path rel{*value};
  studio->insert_or_assign(
      "dashboard", (rel.is_absolute() || base.empty() ? rel : base / rel)
                       .lexically_normal()
                       .string());
  return file;
}

// The dashboard layout file must exist; the error names where it was set,
// the key and the path as resolved.
void RequireStudioDashboard(const std::string& where, const std::string& path) {
  if (!std::filesystem::is_regular_file(path)) {
    throw std::invalid_argument(where + ": [studio] dashboard: no such file '" +
                                path + "'");
  }
}

// [trajectories] belongs at a subsystem file's top level. Under its
// [subsystem] header it would be merged into the subsystem's table and then
// replaced by a top-level [trajectories], so it is refused, not dropped.
void RefuseHeaderTrajectories(const toml::table& document,
                              const std::string& file) {
  if (const auto* header = document["subsystem"].as_table();
      header != nullptr && header->contains("trajectories")) {
    throw std::invalid_argument(
        file +
        ": [subsystem.trajectories] is not read; put the entries in a "
        "top-level [trajectories] table");
  }
}

// The [subsystems.<name>] tables of one robot file: rebased, with where each
// entry was set recorded per subsystem (a later call overrides).
void RebaseSubsystemTablesTrajectories(
    toml::table& document, const std::string& file,
    std::map<std::string, TrajectorySources>& sources) {
  if (auto* subs = document["subsystems"].as_table()) {
    for (auto&& [name, value] : *subs) {
      if (auto* sub = value.as_table()) {
        const std::string subsystem{name.str()};
        RebaseTrajectories(*sub, file, file + " [subsystems." + subsystem + "]",
                           sources[subsystem]);
      }
    }
  }
}

void MergeTables(toml::table& base, const toml::table& over) {
  for (auto&& [k, v] : over) {
    if (const auto* over_sub = v.as_table()) {
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

bool IsCANcoder(const RawDevice& d) {
  const auto* s = std::get_if<hardware::SensorConfig>(&d.config);
  return s != nullptr && s->kind == hardware::SensorKind::kCANcoder;
}

// A switch on the TalonFX's own limit input; absent keeps Phoenix's default,
// an enabled normally open switch.
hardware::LimitSwitch ReadLimitSwitch(const toml::table& t, const char* key,
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

// The motor the TalonFX drives (MotorConfig::motor_type); absent is a Kraken
// X60.
hardware::MotorType ReadMotorType(const toml::table& t,
                                  const std::string& name) {
  const auto value = Read<std::string>(t, "motor_type", "kraken_x60");
  if (value == "kraken_x60") return hardware::MotorType::kKrakenX60;
  if (value == "kraken_x44") return hardware::MotorType::kKrakenX44;
  if (value == "falcon_500") return hardware::MotorType::kFalcon500;
  std::string message = "motor_type for " + name +
                        " must be \"kraken_x60\", \"kraken_x44\" or "
                        "\"falcon_500\", not \"" +
                        value + "\"";
  // The key keeps its source through table copies; a value node does not.
  for (auto&& [key, node] : t) {
    if (key.str() == "motor_type" && key.source().path) {
      message += " in " + *key.source().path;
    }
  }
  throw std::invalid_argument(message);
}

void CheckDeviceKeys(const toml::table& t, const std::string& name,
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

RawDevice ParseMotor(const toml::table& t, const std::string& name) {
  CheckDeviceKeys(t, name, "motor",
                  {"type",
                   "bus",
                   "can_id",
                   "inverted",
                   "brake",
                   "supply_limit_a",
                   "stator_limit_a",
                   "max_voltage",
                   "rotor_to_sensor_ratio",
                   "sensor_to_mechanism_ratio",
                   "continuous_wrap",
                   "soft_limits",
                   "reverse_limit_rot",
                   "forward_limit_rot",
                   "max_velocity_rps",
                   "cruise_velocity_rps",
                   "acceleration_rps2",
                   "jerk_rps3",
                   "follow",
                   "oppose_master",
                   "feedback",
                   "feedback_sensor",
                   "slot0",
                   "slot1",
                   "slot2",
                   "position_frame",
                   "forward_limit_switch",
                   "reverse_limit_switch",
                   "motor_type",
                   "foc",
                   "max_torque_current_a"});
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
  m.motor_type.value = ReadMotorType(t, name);
  for (int s = 0; s < 3; ++s) {
    const auto slot_name = "slot" + std::to_string(s);
    if (t.contains(slot_name) && !t[slot_name].is_table()) {
      throw std::invalid_argument(slot_name + " must be a table for " + name);
    }
    if (const auto* slot = t[slot_name].as_table()) {
      CheckDeviceKeys(*slot, name, "motor slot",
                      {"p", "i", "d", "s", "v", "a", "g", "gravity_type"});
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
                                    " " + slot_name);
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
  // A follower copies its master's output, FOC commutation included
  // (MotorConfig::foc).
  m.foc = Read(t, "foc", m.foc);
  if (m.foc && !raw.follow_motor.empty()) {
    throw std::invalid_argument("foc for " + name +
                                ": a follower follows its master's output; "
                                "set foc on " +
                                raw.follow_motor);
  }
  // Its range, foc and follower rules are hardware::Validate's.
  m.max_torque_current_a =
      Read(t, "max_torque_current_a", m.max_torque_current_a);
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

// A digital input, with its simulated home switch (DigitalInputConfig::
// sim_home_switch): the sim_closed_* keys need sim_motor, which needs a
// finite sim_closed_at_rot. Whether sim_motor names a motor of the same
// subsystem is checked once every device is parsed.
RawDevice ParseDigitalInput(const toml::table& t, const std::string& name) {
  CheckDeviceKeys(t, name, "digital input",
                  {"dio", "sim_motor", "sim_closed_at_rot", "sim_closed_above",
                   "sim_closed_reads"});
  hardware::DigitalInputConfig input{.dio = ReadRequired<int>(t, "dio", name)};
  auto& sim = input.sim_home_switch;
  if (!t.contains("sim_motor")) {
    for (const char* key :
         {"sim_closed_at_rot", "sim_closed_above", "sim_closed_reads"}) {
      if (t.contains(key)) {
        throw std::invalid_argument(std::string(key) + " for digital input " +
                                    name + " needs sim_motor");
      }
    }
    return RawDevice{.config = std::move(input)};
  }
  sim.motor = ReadNonEmpty(t, "sim_motor", "", "digital input");
  sim.closed_at_rot = ReadRequired<double>(t, "sim_closed_at_rot", name);
  if (!std::isfinite(sim.closed_at_rot)) {
    throw std::invalid_argument("sim_closed_at_rot for digital input " + name +
                                " must be finite");
  }
  sim.closed_above = Read(t, "sim_closed_above", sim.closed_above);
  sim.closed_reads = Read(t, "sim_closed_reads", sim.closed_reads);
  return RawDevice{.config = std::move(input)};
}

RawDevice ParseSensor(const toml::table& t, const std::string& name) {
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

RawDevice ParseEncoder(const toml::table& t, const std::string& name) {
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
void ApplyTargetPolicy(const toml::table& tbl, TargetPolicy& policy,
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

std::vector<std::pair<std::string, TeleopValue>> ParseTeleopAssignments(
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
      const double v = real->get();
      if (!std::isfinite(v)) {
        throw std::invalid_argument(where + "." + key + " must be finite");
      }
      assignments.emplace_back(key, v);
    } else if (const auto* text = value.as_string()) {
      assignments.emplace_back(key, text->get());
    } else {
      throw std::invalid_argument(where + "." + key + " must be a scalar");
    }
  }
  return assignments;
}

std::vector<TeleopBinding> ParseTeleop(const toml::table& sub_tbl,
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
      if (!std::isfinite(b.scale)) {
        throw std::invalid_argument(entry_where + ".scale must be finite");
      }
      if (entry->contains("deadband_frac")) {
        const auto& df = (*entry)["deadband_frac"];
        if (!df.is_floating_point() && !df.is_integer()) {
          throw std::invalid_argument(entry_where +
                                      ".deadband_frac must be a number");
        }
        const double d = df.as_floating_point()
                             ? df.as_floating_point()->get()
                             : static_cast<double>(df.as_integer()->get());
        // Negated, so NaN is refused too.
        if (!(d >= kTeleopMinDeadbandFrac && d < 1.0)) {
          // Shortest round-trip text, so a refused value never prints as an
          // accepted one (0.0499999999 is not "0.05").
          const auto text = [](double value) {
            std::array<char, 32> buffer{};
            const auto conv = std::to_chars(
                buffer.data(), buffer.data() + buffer.size(), value);
            return std::string(buffer.data(), conv.ptr);
          };
          throw std::invalid_argument(
              entry_where + ".deadband_frac = " + text(d) + " must be in [" +
              text(kTeleopMinDeadbandFrac) +
              ", 1): the axis counts as neutral for arming only inside its "
              "deadband, which is at least " +
              text(kTeleopMinDeadbandFrac) + " of full stick");
        }
        b.deadband_frac = d;
      }
    }
    result.push_back(std::move(b));
  }
  return result;
}

// Replaces a [[subsystems]] manifest with the canonical [subsystems.<name>]
// view: each listed file's [subsystem] scalars (minus the name) plus its
// other top-level tables, then the entry's optional `node` over the file's.
// Validation then runs once over the merged view, so splitting files cannot
// weaken a check.
void NormalizeManifestPaths(toml::array& manifest,
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

void MergeManifests(toml::array& base_arr, const toml::array& child_arr,
                    const std::string& child_path) {
  toml::table seen_in_child;
  for (auto&& child_node : child_arr) {
    const auto* child_entry = child_node.as_table();
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

toml::table ExpandManifest(const toml::array& manifest, const std::string& path,
                           bool paths_normalized, const ReaderCallback& read,
                           const DataFileCallback& data) {
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
    const auto* entry = entry_node.as_table();
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
    RefuseHeaderTrajectories(sub_file, file);
    TrajectorySources sources;
    RebaseTrajectories(sub_file, file, file, sources);

    const auto* header = sub_file["subsystem"].as_table();
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
      RefuseHeaderTrajectories(merged, *defaults_file);
      TrajectorySources defaults_sources;
      RebaseTrajectories(merged, *defaults_file, *defaults_file,
                         defaults_sources);
      // The subsystem file's entries override its defaults' entries.
      for (auto& [entry_key, where] : sources) {
        defaults_sources.insert_or_assign(entry_key, std::move(where));
      }
      sources = std::move(defaults_sources);
      MergeTables(merged, sub_file);
      sub_file = std::move(merged);
      header = sub_file["subsystem"].as_table();
    }
    // Only the effective entries, after the defaults merge, are listed and
    // required.
    RequireTrajectories(sub_file, sources, file, data);
    toml::table view;
    if (header != nullptr) {
      for (auto&& [k, v] : *header) {
        if (k.str() != "name" && k.str() != "defaults") {
          view.insert_or_assign(std::string(k.str()), v);
        }
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

}  // namespace

std::filesystem::path RobotConfig::TrajectoryPath(std::string_view subsystem,
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
      "[subsystems." + std::string(subsystem) + "] declares no 'trajectories." +
      std::string(name) +
      "' (declared: " + (declared.empty() ? "none" : declared) + ")");
}

namespace detail {

toml::table ParseTomlFile(const std::string& path) {
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

void IgnoreDataFile(const std::string& /*path*/, const std::string& /*label*/) {
}

}  // namespace detail

uint16_t ParsePortFlag(std::string_view flag, std::string_view value,
                       bool allow_zero) {
  const int min_port = allow_zero ? 0 : 1;
  int port = 0;
  const auto* end = value.data() + value.size();
  const auto [ptr, ec] = std::from_chars(value.data(), end, port);
  if (value.empty() || ec != std::errc{} || ptr != end || port < min_port ||
      port > 65535) {
    throw std::invalid_argument(std::string(flag) + " must be a port in [" +
                                std::to_string(min_port) + ", 65535], not '" +
                                std::string(value) + "'");
  }
  return static_cast<uint16_t>(port);
}

std::string ResolveControllerAddress(const RobotConfig& config,
                                     const std::string& cli_override) {
  if (cli_override.empty()) return config.controller_address;
  if (!IsIpv4Address(cli_override)) {
    throw std::invalid_argument("--remote must be an IPv4 address, not '" +
                                cli_override + "'");
  }
  return cli_override;
}

uint16_t ResolveControllerPort(const RobotConfig& config,
                               uint16_t cli_override) {
  if (cli_override != 0) return cli_override;
  return config.controller_port;
}

uint16_t ResolveLocalPort(const RobotConfig& config, uint16_t cli_override) {
  if (cli_override != 0) return cli_override;
  return config.local_port;
}

RobotConfig ParseRobotConfigTable(toml::table tbl, bool validate_hardware) {
  CheckTopLevelKeys(tbl);
  RobotConfig result;
  result.toml_data = std::move(tbl);
  const toml::table& root = result.toml_data;
  auto& hw = result.hardware;

  if (root.contains("robot") && !root["robot"].is_table()) {
    throw std::invalid_argument("[robot] must be a table");
  }
  if (const auto* robot = root["robot"].as_table()) {
    static constexpr std::array<std::string_view, 16> kRobotKeys = {
        "name",
        "period_us",
        "command_timeout_us",
        "status_hz",
        "commissioned",
        "require_target_coverage",
        "simulation_only",
        "controller_address",
        "controller_port",
        "local_port",
        "log_keep_sessions",
        "log_max_bytes",
        "max_restarts",
        "hardware_node",
        "sim_gateway",
        "targets",
    };
    for (auto&& [k, v] : *robot) {
      if (std::ranges::find(kRobotKeys, k.str()) == kRobotKeys.end()) {
        throw std::invalid_argument("[robot] unknown key '" +
                                    std::string(k.str()) + "'");
      }
    }
    hw.period_us = Read(*robot, "period_us", hw.period_us);
    hw.command_timeout_us =
        Read(*robot, "command_timeout_us", hw.command_timeout_us);
    hw.status_hz = Read(*robot, "status_hz", hw.status_hz);
    hw.commissioned = Read(*robot, "commissioned", hw.commissioned);
    // A commissioned robot drives hardware, so a mode its policy lists with
    // no producer would lose that mode on the field: it requires coverage,
    // and refuses a config that turns it off.
    if (hw.commissioned && robot->contains("require_target_coverage") &&
        !Read(*robot, "require_target_coverage", true)) {
      throw std::invalid_argument(
          "[robot] commissioned = true requires target coverage, but "
          "require_target_coverage = false; remove the key or set it to "
          "true, and give each subsystem's modes a producer or empty them "
          "in its [targets]");
    }
    result.require_target_coverage =
        Read(*robot, "require_target_coverage", hw.commissioned);
    result.simulation_only =
        Read(*robot, "simulation_only", result.simulation_only);
    result.controller_address = ReadNonEmpty(
        *robot, "controller_address", result.controller_address, "[robot]");
    if (!IsIpv4Address(result.controller_address)) {
      throw std::invalid_argument(
          "[robot] field 'controller_address' must be an IPv4 address such "
          "as \"10.56.87.2\", not '" +
          result.controller_address + "': the bridge resolves no hostname");
    }
    result.controller_port =
        ReadPort(*robot, "controller_port", result.controller_port, "[robot]");
    result.local_port =
        ReadPort(*robot, "local_port", result.local_port, "[robot]");
    result.log_keep_sessions = ReadPositive(
        *robot, "log_keep_sessions", result.log_keep_sessions, "[robot]");
    result.log_max_bytes =
        ReadPositive(*robot, "log_max_bytes", result.log_max_bytes, "[robot]");
    if (robot->contains("targets")) {
      const auto* targets = (*robot)["targets"].as_table();
      if (targets == nullptr) {
        throw std::invalid_argument("[robot.targets] must be a table");
      }
      ApplyTargetPolicy(*targets, result.targets, "[robot.targets]");
    }
  }

  if (root.contains("studio")) {
    const auto* studio = root["studio"].as_table();
    if (!studio) {
      throw std::invalid_argument("[studio] must be a table");
    }
    for (auto&& [k, v] : *studio) {
      if (k.str() != "bridge" && k.str() != "webroot" &&
          k.str() != "http_port" && k.str() != "topic" &&
          k.str() != "pose_topic" && k.str() != "max_clients" &&
          k.str() != "dashboard") {
        throw std::invalid_argument("[studio] unknown key '" +
                                    std::string(k.str()) + "'");
      }
    }
    auto& s = result.studio;
    s.bridge = ReadNonEmpty(*studio, "bridge", s.bridge, "[studio]");
    s.webroot = ReadNonEmpty(*studio, "webroot", s.webroot, "[studio]");
    s.http_port = ReadPort(*studio, "http_port", s.http_port, "[studio]");
    if (studio->contains("max_clients")) {
      const auto clients = (*studio)["max_clients"].value<int64_t>();
      if (!clients || *clients < 1 || *clients > 64) {
        throw std::invalid_argument(
            "[studio] field 'max_clients' must be an integer in [1, 64]");
      }
      s.max_clients = static_cast<uint16_t>(*clients);
    }
    if (studio->contains("topic")) {
      s.topic = ReadNonEmpty(*studio, "topic", "", "[studio]");
    }
    if (studio->contains("pose_topic")) {
      s.pose_topic = ReadNonEmpty(*studio, "pose_topic", "", "[studio]");
    }
    if (studio->contains("dashboard")) {
      s.dashboard = ReadNonEmpty(*studio, "dashboard", "", "[studio]");
      RequireStudioDashboard("config", s.dashboard);
    }
  }

  const auto* subsystems_tbl = root["subsystems"].as_table();
  if (!subsystems_tbl) {
    throw std::invalid_argument("missing [subsystems] table");
  }

  std::vector<RawDevice> raw_devices;
  for (auto&& [sub_key, sub_val] : *subsystems_tbl) {
    const std::string sub_name(sub_key.str());
    const auto* sub_tbl = sub_val.as_table();
    if (!sub_tbl) {
      throw std::invalid_argument("[subsystems." + sub_name +
                                  "] must be a table");
    }
    result.subsystems[sub_name].subsystem = sub_name;
    CheckSubsystemKeys(*sub_tbl, "[subsystems." + sub_name + "]");
    // The launcher starts a subsystem's `node`; a malformed one must not
    // silently leave the subsystem without a process.
    if (sub_tbl->contains("node")) {
      ReadNonEmpty(*sub_tbl, "node", "",
                   ("[subsystems." + sub_name + "]").c_str());
    }
    // Fail closed: unknown nodes are required for safe robot operation.
    Read<bool>(*sub_tbl, "critical", true);
    TargetPolicy& policy = result.subsystem_targets[sub_name] = result.targets;
    if (sub_tbl->contains("targets")) {
      const auto* targets = (*sub_tbl)["targets"].as_table();
      const std::string where = "[subsystems." + sub_name + ".targets]";
      if (targets == nullptr) {
        throw std::invalid_argument(where + " must be a table");
      }
      ApplyTargetPolicy(*targets, policy, where);
    }
    {
      const std::string where = "[subsystems." + sub_name + "]";
      for (auto& [name, path] : ReadTrajectories(*sub_tbl, where)) {
        RequireTrajectoryFile(where, name, path);
        result.trajectories[sub_name].insert_or_assign(name, std::move(path));
      }
    }
    auto bindings = ParseTeleop(*sub_tbl, sub_name);
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
          ParseTeleopAssignments(*defaults, where);
    }
    auto each = [&](const char* section, auto parse) {
      const auto* devices = (*sub_tbl)[section].as_table();
      if (!devices) {
        if (sub_tbl->contains(section)) {
          throw std::invalid_argument(std::string(section) +
                                      " must be a table");
        }
        return;
      }
      for (auto&& [dev_key, dev_val] : *devices) {
        const std::string name(dev_key.str());
        const auto* dev_tbl = dev_val.as_table();
        if (!dev_tbl) {
          throw std::invalid_argument(std::string(section) + " entry '" + name +
                                      "' must be a table");
        }
        try {
          RawDevice raw = parse(*dev_tbl, name);
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
    each("motors", ParseMotor);
    each("sensors", ParseSensor);
    each("encoders", ParseEncoder);
    each("digital_inputs", ParseDigitalInput);
    each("digital_outputs", [](const toml::table& t, const std::string& n) {
      CheckDeviceKeys(t, n, "digital output", {"dio"});
      return RawDevice{.config = hardware::DigitalOutputConfig{
                           .dio = ReadRequired<int>(t, "dio", n)}};
    });
    each("analog_inputs", [](const toml::table& t, const std::string& n) {
      CheckDeviceKeys(t, n, "analog input", {"channel"});
      return RawDevice{.config = hardware::AnalogInputConfig{
                           .channel = ReadRequired<int>(t, "channel", n)}};
    });
    each("pwm_outputs", [](const toml::table& t, const std::string& n) {
      CheckDeviceKeys(t, n, "PWM output", {"channel"});
      return RawDevice{.config = hardware::PwmConfig{
                           .channel = ReadRequired<int>(t, "channel", n)}};
    });
  }

  // Deterministic logical ids: sort by (subsystem, device), number from 1.
  std::sort(raw_devices.begin(), raw_devices.end(),
            [](const RawDevice& a, const RawDevice& b) {
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

  auto find = [&](const std::string& name) -> const RawDevice* {
    for (const auto& d : raw_devices) {
      if (d.name == name) return &d;
    }
    return nullptr;
  };
  auto require_same_subsystem = [](const RawDevice& target,
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

  // A simulated home switch's motor is a motor of the input's subsystem.
  for (const auto& dev : raw_devices) {
    const auto* input = std::get_if<hardware::DigitalInputConfig>(&dev.config);
    if (input == nullptr || input->sim_home_switch.motor.empty()) continue;
    const std::string& motor = input->sim_home_switch.motor;
    const auto* found = find(motor);
    if (found == nullptr || found->subsystem != dev.subsystem ||
        !std::holds_alternative<hardware::MotorConfig>(found->config)) {
      throw std::invalid_argument(
          "[subsystems." + dev.subsystem + "].digital_inputs." + dev.name +
          ": sim_motor '" + motor + "' names no motor of subsystem '" +
          dev.subsystem + "'");
    }
  }

  // Swerve geometry module references must name this subsystem's devices.
  for (auto&& [sub_key, sub_val] : *subsystems_tbl) {
    const std::string sub_name(sub_key.str());
    const auto* sub_tbl = sub_val.as_table();
    const auto* modules =
        sub_tbl ? (*sub_tbl)["geometry"]["modules"].as_array() : nullptr;
    if (!modules) continue;
    for (auto&& mod_node : *modules) {
      const auto* mod = mod_node.as_table();
      if (!mod) continue;
      auto check_ref = [&](const char* field,
                           bool (*is_kind)(const RawDevice&)) {
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
      auto is_motor = [](const RawDevice& d) {
        return std::holds_alternative<hardware::MotorConfig>(d.config);
      };
      check_ref("drive", is_motor);
      check_ref("steer", is_motor);
      check_ref("encoder", IsCANcoder);
    }
  }

  if (root.contains("simulation")) {
    const auto* sim = root["simulation"].as_table();
    if (sim == nullptr) {
      throw std::invalid_argument("[simulation] must be a table");
    }
    for (auto&& [k, v] : *sim) {
      if (k.str() != "wheel_radius_m" && k.str() != "swerve_modules" &&
          k.str() != "hard_stops") {
        throw std::invalid_argument("[simulation] unknown key '" +
                                    std::string(k.str()) + "'");
      }
    }
    auto& s = result.simulation;
    if (sim->contains("wheel_radius_m")) {
      const double radius = Read<double>(*sim, "wheel_radius_m", 0.0);
      if (!std::isfinite(radius) || radius <= 0.0) {
        throw std::invalid_argument(
            "[simulation] field 'wheel_radius_m' must be finite and positive");
      }
      s.wheel_radius_m = radius;
    }
    if (sim->contains("swerve_modules")) {
      const auto* modules = (*sim)["swerve_modules"].as_array();
      if (modules == nullptr) {
        throw std::invalid_argument(
            "[simulation] field 'swerve_modules' must be an array of tables");
      }
      std::vector<std::string> seen_motors;
      s.swerve_modules.reserve(modules->size());
      for (std::size_t i = 0; i < modules->size(); ++i) {
        const std::string entry_where =
            "[simulation] swerve_modules[" + std::to_string(i) + "]";
        const auto* entry = (*modules)[i].as_table();
        if (entry == nullptr) {
          throw std::invalid_argument(entry_where + " must be a table");
        }
        for (auto&& [k, v] : *entry) {
          const std::string_view ks = k.str();
          if (ks != "drive" && ks != "steer" && ks != "x_m" && ks != "y_m") {
            throw std::invalid_argument(entry_where + " unknown key '" +
                                        std::string(ks) + "'");
          }
        }
        SimulationSwerveModule mod;
        mod.drive = ReadRequired<std::string>(*entry, "drive", entry_where);
        if (mod.drive.empty()) {
          throw std::invalid_argument(
              entry_where + " field 'drive' must be a non-empty string");
        }
        mod.steer = ReadRequired<std::string>(*entry, "steer", entry_where);
        if (mod.steer.empty()) {
          throw std::invalid_argument(
              entry_where + " field 'steer' must be a non-empty string");
        }
        mod.x_m = ReadRequired<double>(*entry, "x_m", entry_where);
        if (!std::isfinite(mod.x_m)) {
          throw std::invalid_argument(entry_where +
                                      " field 'x_m' must be finite");
        }
        mod.y_m = ReadRequired<double>(*entry, "y_m", entry_where);
        if (!std::isfinite(mod.y_m)) {
          throw std::invalid_argument(entry_where +
                                      " field 'y_m' must be finite");
        }
        if (mod.x_m == 0.0 && mod.y_m == 0.0) {
          throw std::invalid_argument(
              entry_where + ": module position (0, 0) must be nonzero");
        }
        if (mod.drive == mod.steer) {
          throw std::invalid_argument(
              entry_where + ": drive and steer cannot be the same motor '" +
              mod.drive + "'");
        }
        auto check_motor = [&](const char* role, const std::string& name) {
          const bool is_motor = std::any_of(
              raw_devices.begin(), raw_devices.end(), [&](const RawDevice& d) {
                return d.name == name &&
                       std::holds_alternative<hardware::MotorConfig>(d.config);
              });
          if (!is_motor) {
            throw std::invalid_argument(entry_where + ": " + role + " '" +
                                        name + "' names no configured motor");
          }
          if (std::find(seen_motors.begin(), seen_motors.end(), name) !=
              seen_motors.end()) {
            throw std::invalid_argument(
                entry_where + ": motor '" + name +
                "' is already used by another swerve module");
          }
          seen_motors.push_back(name);
        };
        check_motor("drive", mod.drive);
        check_motor("steer", mod.steer);
        s.swerve_modules.push_back(std::move(mod));
      }
    }
    if (sim->contains("hard_stops")) {
      const auto* stops = (*sim)["hard_stops"].as_array();
      if (stops == nullptr) {
        throw std::invalid_argument(
            "[simulation] field 'hard_stops' must be an array of tables");
      }
      std::vector<std::string> seen_motors;
      s.hard_stops.reserve(stops->size());
      for (std::size_t i = 0; i < stops->size(); ++i) {
        const std::string entry_where =
            "[simulation] hard_stops[" + std::to_string(i) + "]";
        const auto* entry = (*stops)[i].as_table();
        if (entry == nullptr) {
          throw std::invalid_argument(entry_where + " must be a table");
        }
        for (auto&& [k, v] : *entry) {
          const std::string_view ks = k.str();
          if (ks != "motor" && ks != "min_rot" && ks != "max_rot") {
            throw std::invalid_argument(entry_where + " unknown key '" +
                                        std::string(ks) + "'");
          }
        }
        SimulationHardStop stop;
        stop.motor = ReadRequired<std::string>(*entry, "motor", entry_where);
        if (stop.motor.empty()) {
          throw std::invalid_argument(
              entry_where + " field 'motor' must be a non-empty string");
        }
        if (entry->contains("min_rot")) {
          const double v = Read<double>(*entry, "min_rot", 0.0);
          if (!std::isfinite(v)) {
            throw std::invalid_argument(entry_where +
                                        " field 'min_rot' must be finite");
          }
          stop.min_rot = v;
        }
        if (entry->contains("max_rot")) {
          const double v = Read<double>(*entry, "max_rot", 0.0);
          if (!std::isfinite(v)) {
            throw std::invalid_argument(entry_where +
                                        " field 'max_rot' must be finite");
          }
          stop.max_rot = v;
        }
        if (!stop.min_rot && !stop.max_rot) {
          throw std::invalid_argument(
              entry_where +
              ": must specify at least one of 'min_rot' or 'max_rot'");
        }
        if (stop.min_rot && stop.max_rot && !(*stop.min_rot < *stop.max_rot)) {
          throw std::invalid_argument(
              entry_where + ": 'min_rot' must be less than 'max_rot'");
        }
        const auto motor_it = std::find_if(
            raw_devices.begin(), raw_devices.end(), [&](const RawDevice& d) {
              return d.name == stop.motor &&
                     std::holds_alternative<hardware::MotorConfig>(d.config);
            });
        if (motor_it == raw_devices.end()) {
          throw std::invalid_argument(entry_where + ": motor '" + stop.motor +
                                      "' names no configured motor");
        }
        const auto& motor_cfg =
            std::get<hardware::MotorConfig>(motor_it->config);
        if (motor_cfg.master_id != 0) {
          throw std::invalid_argument(
              entry_where + ": motor '" + stop.motor + "' follows '" +
              motor_it->follow_motor +
              "'; configure the hard stop on its master instead");
        }
        if (std::find(seen_motors.begin(), seen_motors.end(), stop.motor) !=
            seen_motors.end()) {
          throw std::invalid_argument(entry_where +
                                      ": duplicate hard stop for motor '" +
                                      stop.motor + "'");
        }
        seen_motors.push_back(stop.motor);
        s.hard_stops.push_back(std::move(stop));
      }
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
      const auto* sub_tbl = result.GetSubsystemTable(name);
      if (sub_tbl == nullptr) {
        throw std::logic_error(
            "[subsystems." + name +
            "] owns actuators but has no subsystem table in toml_data");
      }
      const auto period_us = Read(*sub_tbl, "period_us", int64_t{hw.period_us});
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

RobotConfig ParseRobotConfigString(std::string_view toml_content) {
  try {
    return ParseRobotConfigTable(toml::parse(toml_content));
  } catch (const toml::parse_error& err) {
    throw std::invalid_argument(std::string("TOML parse error: ") + err.what());
  }
}

toml::table ResolveRobotConfig(const std::string& path,
                               const ReaderCallback& read,
                               const DataFileCallback& data) {
  namespace fs = std::filesystem;
  toml::table tbl = read(path);
  CheckTopLevelKeys(tbl);
  // [subsystems.<name>.trajectories] in this file and its include: rebased
  // now, required once the include is merged, so an entry this file
  // overrides is never required.
  std::map<std::string, TrajectorySources> table_sources;
  RebaseSubsystemTablesTrajectories(tbl, path, table_sources);
  // Where the effective [studio] dashboard was set: this file's wins over
  // its include's.
  std::string dashboard_source = RebaseStudioDashboard(tbl, path);
  if (auto* manifest = tbl["subsystems"].as_array()) {
    NormalizeManifestPaths(*manifest, path);
  }
  if (tbl.contains("include")) {
    const std::string inc_str = ReadNonEmpty(tbl, "include", "", "top-level");
    const fs::path base_dir = fs::path(path).parent_path();
    const fs::path inc_rel{inc_str};
    const std::string inc_file =
        (inc_rel.is_absolute() || base_dir.empty() ? inc_rel
                                                   : base_dir / inc_rel)
            .string();
    toml::table base = read(inc_file);
    CheckTopLevelKeys(base);
    const std::string base_dashboard = RebaseStudioDashboard(base, inc_file);
    if (dashboard_source.empty()) dashboard_source = base_dashboard;
    std::map<std::string, TrajectorySources> base_sources;
    RebaseSubsystemTablesTrajectories(base, inc_file, base_sources);
    for (auto& [subsystem, entries] : table_sources) {
      for (auto& [entry, where] : entries) {
        base_sources[subsystem].insert_or_assign(entry, std::move(where));
      }
    }
    table_sources = std::move(base_sources);
    if (base.contains("include")) {
      throw std::invalid_argument("nested 'include' is not supported in " +
                                  inc_file);
    }
    // A simulation-only config stays one in every config that includes it:
    // an override back to false would make simulation settings drivable.
    if (base["robot"]["simulation_only"].value_exact<bool>().value_or(false) &&
        !tbl["robot"]["simulation_only"].value_exact<bool>().value_or(true)) {
      throw std::invalid_argument(
          path +
          " sets [robot] simulation_only = false, but the file it "
          "includes, " +
          inc_file +
          ", sets it true; a simulation-only config cannot be included as "
          "a deployable one");
    }
    if (auto* base_manifest = base["subsystems"].as_array()) {
      NormalizeManifestPaths(*base_manifest, inc_file);
    }
    tbl.erase("include");
    if (auto* base_subs = base["subsystems"].as_array()) {
      if (auto* child_subs = tbl["subsystems"].as_array()) {
        MergeManifests(*base_subs, *child_subs, path);
        tbl.erase("subsystems");
      } else if (tbl.contains("subsystems")) {
        base.insert_or_assign(
            "subsystems",
            ExpandManifest(*base_subs, inc_file, /*paths_normalized=*/true,
                           read, data));
      }
    }
    MergeTables(base, tbl);
    tbl = std::move(base);
  }
  if (!dashboard_source.empty()) {
    const std::string dashboard =
        tbl["studio"]["dashboard"].value_or(std::string{});
    data(dashboard, dashboard_source + ": [studio] dashboard");
    RequireStudioDashboard(dashboard_source, dashboard);
  }
  if (auto* subs = tbl["subsystems"].as_table()) {
    for (const auto& [subsystem, entries] : table_sources) {
      if (const auto* sub = (*subs)[subsystem].as_table()) {
        RequireTrajectories(*sub, entries,
                            path + " [subsystems." + subsystem + "]", data);
      }
    }
  }
  if (auto* manifest = tbl["subsystems"].as_array()) {
    tbl.insert_or_assign(
        "subsystems",
        ExpandManifest(*manifest, path, /*paths_normalized=*/true, read, data));
  }
  return tbl;
}

RobotConfig ParseRobotConfig(const std::string& path) {
  return ParseRobotConfigTable(ResolveRobotConfig(path));
}

std::vector<std::string> SubsystemKeyFiles(const std::string& path,
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
