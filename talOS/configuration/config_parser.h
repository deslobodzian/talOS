#pragma once

// Robot configuration: TOML -> hardware::Config plus per-subsystem ownership.
//
// One layout: optional `include = "base.toml"`, [robot], optional [studio],
// optional [simulation], and [subsystems.<name>] tables. [[subsystems]] { name,
// path, node?, defaults? } entries are an include over it: each file (path
// relative to robot.toml) maps its [subsystem] header and tables to
// [subsystems.<name>], optionally layered over a `defaults` TOML file, the
// entry's `node` winning; then it parses as one file.
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
// and the build lists each in the config's CONFIG_FILES (resolve.cc). So
// do [studio] dashboard, the driver dashboard's layout file, and
// [simulation.mujoco] robot_model and robot_model_files, the MuJoCo model.

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <iterator>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "talOS/hardware/config.h"
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

// The narrowest teleop axis deadband config load accepts, and the default of
// a binding that states none, as a fraction of full stick. A binding's one
// deadband is both its output dead zone and the band it must return to
// before teleop arms (talOS/teleop/deadband.h): a real stick rests a few
// counts off zero, so a narrower band could keep teleop from ever arming.
inline constexpr double kTeleopMinDeadbandFrac = 0.05;

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
  double deadband_frac{kTeleopMinDeadbandFrac};

  bool operator==(const TeleopBinding&) const = default;
};

struct SimulationSwerveModule {
  std::string drive;
  std::string steer;
  double x_m{0.0};
  double y_m{0.0};

  bool operator==(const SimulationSwerveModule&) const = default;
};

struct SimulationHardStop {
  std::string motor;
  std::optional<double> min_rot;
  std::optional<double> max_rot;

  bool operator==(const SimulationHardStop&) const = default;
};

struct SimulationBatteryConfig {
  double nominal_voltage_v{12.8};
  double internal_resistance_ohm{0.0};
  double base_current_a{0.0};
  double brownout_voltage_v{6.75};

  bool operator==(const SimulationBatteryConfig&) const = default;
};

struct SimulationChassisConfig {
  double mass_kg{0.0};                            // required > 0 when present
  std::optional<double> moment_of_inertia_kg_m2;  // optional > 0
  std::optional<double> traction_coefficient;     // optional > 0

  bool operator==(const SimulationChassisConfig&) const = default;
};

enum class SimulationLoadKind : std::uint8_t { kRoller = 0, kFlywheel = 1 };

struct SimulationMechanismConfig {
  std::string motor;          // required non-empty master motor, no duplicates
  double inertia_kg_m2{0.0};  // required > 0 (mechanism-shaft J)
  SimulationLoadKind load{SimulationLoadKind::kRoller};
  double viscous_damping_nm_per_radps{0.0};  // optional >= 0
  double static_friction_nm{0.0};            // optional >= 0

  bool operator==(const SimulationMechanismConfig&) const = default;
};

// A field model `[simulation.mujoco] field` may name, with its floor in the
// field frame: x from 0 to length_m, y from 0 to width_m (twice the model's
// `field_centre` site, which //talOS/simulation/mujoco:fields_test holds it
// to). The configuration layer cannot depend on talOS/simulation, so the
// list lives here; each name must have its package,
// //talOS/simulation/mujoco/fields/<name>.
struct SimulationMujocoField {
  std::string_view name;
  double length_m;
  double width_m;
};
inline constexpr SimulationMujocoField kSimulationMujocoFields[] = {
    // The drawings' 651.22 x 317.69 in.
    {.name = "rebuilt_2026", .length_m = 16.541188, .width_m = 8.069326},
};

// Optional `[simulation.start_pose]`: where the simulator places the robot
// at start, in the field frame (metres, yaw radians CCW+). A file that sets
// it sets all three keys, each a finite number; with a `[simulation.mujoco]
// field`, (x_m, y_m) lies on that field's floor. Unset, the gateway starts
// the robot at the defaults below; its `--start-pose` overrides either.
struct SimulationStartPose {
  double x_m{2.0};
  double y_m{2.0};
  double yaw_rad{0.0};
  bool operator==(const SimulationStartPose&) const = default;
};

// Optional `[simulation.mujoco]`: the MuJoCo model to simulate. A config file's
// paths are relative to the file that sets them, never absolute
// (ResolveRobotConfig refuses one), and resolved here; a config parsed from a
// string keeps them as written, relative to the working directory. Config load
// refuses a file that does not exist, and the build lists each in the config's
// CONFIG_FILES.
struct SimulationMujocoConfig {
  std::string robot_model;                     // resolved
  std::vector<std::string> robot_model_files;  // resolved
  std::string field;                           // "" = flat carpet
  // The seed a field's game rules draw from: 0 to 2^63 - 1 (a TOML integer
  // is signed 64-bit; a negative one is refused).
  std::uint64_t seed{1};
  // Whether MuJoCo puts resting free bodies to sleep (the robot's never
  // sleep): off unless a file sets it.
  bool sleeping{false};
  bool operator==(const SimulationMujocoConfig&) const = default;
};

// Optional `[simulation]` table: physics simulator geometry, battery, chassis,
// mechanisms, end-of-travel hard stops, the start pose and the MuJoCo model,
// all off the hardware wire and never affecting hardware::ConfigurationId.
struct SimulationConfig {
  static constexpr double kDefaultWheelRadiusM = 0.0508;
  double wheel_radius_m{kDefaultWheelRadiusM};
  std::vector<SimulationSwerveModule> swerve_modules;
  std::vector<SimulationHardStop> hard_stops;
  std::optional<SimulationBatteryConfig> battery;
  std::optional<SimulationChassisConfig> chassis;
  std::vector<SimulationMechanismConfig> mechanisms;
  std::optional<SimulationStartPose> start_pose;  // unset: the defaults
  std::optional<SimulationMujocoConfig> mujoco;

  bool operator==(const SimulationConfig&) const = default;
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
  // Optional IPv4 local interface address for the bridge's gateway UDP socket
  // (`[robot] local_address`). Empty when unset, keeping wildcard `0.0.0.0`.
  // When set, config load requires an IPv4 literal without leading zeros
  // (REQ-RT-22, REQ-OPS-25).
  std::string local_address;
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
  // `[robot] require_target_coverage`, default false; true whenever the
  // config file sets `commissioned = true`, which refuses an explicit false.
  bool require_target_coverage{false};

  // `[robot] simulation_only`, default false: the config (simulation gains,
  // bindings, wide travel) must never drive real hardware, so the launcher
  // refuses it without --sim. Once a file sets it true, a file including
  // that one cannot set it back to false (ResolveRobotConfig refuses).
  bool simulation_only{false};

  // Sim-session viewer. Studio's bridge is a host-side process, not a robot
  // node, so it is not a subsystem; this table carries what the launcher
  // needs to start and supervise it in sim sessions. `topic` is the topic it
  // watches, the telemetry node's byte topic (/talos/telemetry).
  struct StudioConfig {
    std::string bridge{"//studio/bridge:studio_bridge"};
    uint16_t http_port{5800};
    uint16_t max_clients{4};
    // Relative to the launcher's working directory; a bundle that carries
    // Studio serves its own webroot unless this is absolute
    // (EffectiveStudioWebroot in talOS/launcher/roster.h).
    std::string webroot{"bazel-bin/studio/dist"};
    std::string topic{"/talos/telemetry"};
    std::string pose_topic;
    // The driver dashboard's layout file (talOS/telemetry:dashboard_layout),
    // resolved against the file that set `dashboard` (in a config parsed
    // from a string, the working directory); empty when unset. Config load
    // refused a file that does not exist.
    std::string dashboard;
  };
  StudioConfig studio;
  SimulationConfig simulation;

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
                                       std::string_view name) const;
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

// Singular spellings of framework subsystem sections that config load refuses
// with a suggestion. Kept in the header so tools/new_subsystem_test.py verifies
// new_subsystem's copy against both lists.
inline constexpr std::pair<std::string_view, std::string_view> kSections[] = {
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

// Read and parse one TOML file; every error names the file.
toml::table ParseTomlFile(const std::string& path);

// The default DataFile: config load needs only each file's existence.
void IgnoreDataFile(const std::string& path, const std::string& label);

}  // namespace detail

// A port flag's value: a decimal integer in [1, 65535] (or [0, 65535] when
// `allow_zero` is true, for `--studio-http-port 0`), else
// std::invalid_argument naming `flag` (std::stoi would accept "5802x" and a
// uint16_t cast would wrap 70000 to 4464).
uint16_t ParsePortFlag(std::string_view flag, std::string_view value,
                       bool allow_zero = false);

// Effective controller endpoint: an explicit CLI value wins, otherwise the
// deployment configuration. A zero CLI port means "not given". A CLI address
// that is not an IPv4 literal throws std::invalid_argument, as config load
// does for controller_address.
std::string ResolveControllerAddress(const RobotConfig& config,
                                     const std::string& cli_override);

// Effective local bind address: an explicit CLI value wins, otherwise `[robot]
// local_address`, or "" when neither is set (wildcard `0.0.0.0`). A non-empty
// value that is not an IPv4 literal throws std::invalid_argument naming
// `--local-address` or `local_address`.
std::string ResolveLocalAddress(const RobotConfig& config,
                                std::string_view cli_override);

uint16_t ResolveControllerPort(const RobotConfig& config,
                               uint16_t cli_override);

uint16_t ResolveLocalPort(const RobotConfig& config, uint16_t cli_override);

// Parses the canonical single-file view. toml_data keeps the table, which is
// what GetSubsystemTable answers from. Build-time resolution can parse an
// incomplete hardware inventory; only runtime requires a deployable inventory.
RobotConfig ParseRobotConfigTable(toml::table tbl,
                                  bool validate_hardware = true);

RobotConfig ParseRobotConfigString(std::string_view toml_content);

// Resolve once for runtime and build tools. Readers may track dependencies or
// supply pending file contents without changing include/manifest semantics.
// `data(path, label)` sees every non-TOML file the config names (the
// effective [trajectories] entries, [studio] dashboard and
// [simulation.mujoco] robot_model and robot_model_files), resolved against
// the file that names it, with a label naming that file and the key, before
// config load requires it to exist.
toml::table ResolveRobotConfig(
    const std::string& path,
    const std::function<toml::table(const std::string&)>& read =
        detail::ParseTomlFile,
    const std::function<void(const std::string&, const std::string&)>& data =
        detail::IgnoreDataFile);

RobotConfig ParseRobotConfig(const std::string& path);

// The files of the config at `path` (itself, its include, subsystem and
// defaults files) that set `key` (a dotted path, "camera.fps") of
// `subsystem`'s table, for a diagnostic to name: a [subsystems.<subsystem>]
// table, or a subsystem file whose [subsystem] names it, or a defaults file
// (whose [subsystem] names none). Empty when the config no longer resolves.
std::vector<std::string> SubsystemKeyFiles(const std::string& path,
                                           std::string_view subsystem,
                                           std::string_view key);

}  // namespace talos::config
