#include "talOS/simulation/gateway_main.h"

#include <algorithm>
#include <array>
#include <bit>
#include <cctype>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <iterator>
#include <optional>
#include <random>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "geometry/geometry_descriptor.h"
#include "geometry/geometry_generated.h"
#include "sim/topics.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/driver_station/schedule.h"
#include "talOS/events/os/poller.h"
#include "talOS/hardware/config_error.h"
#include "talOS/hardware/endpoint.h"
#include "talOS/hardware/sim_backend.h"
#include "talOS/hardware/sim_physics.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/registry.h"
#include "talOS/ipc/publisher.h"
#include "talOS/memory/residency.h"
#include "talOS/process/process.h"
#include "talOS/simulation/ports.h"
#include "talOS/simulation/schedules.h"

namespace {

namespace ds = talos::driver_station;

constexpr const char* kNodeName = "sim_gateway";
constexpr const char* kUsage =
    "usage: sim_gateway --config PATH [--physics "
    "[--start-pose X_M,Y_M,YAW_RAD] [--wheel-radius-error FRAC] "
    "[--wheel-slip-gain G] [--gyro-drift-radps R]] [--duration-s N] "
    "[--mode-schedule mode:seconds,...] [--alliance blue|red] "
    "[--joystick-schedule 'time:axes=...;buttons=MASK;pov=DEG|...'] "
    "[--fault-schedule 'start:end:kind:target[:volts]|...'] "
    "[--schedule-start-file PATH] [--port N] [--remote-port N] [--session-id "
    "N]";

// Shared-memory slot contract: docs/DATA_FLOW.md.
static_assert(sizeof(geometry::OdometrySample) == 56);

// REQ-SAFE-02: matches Robot.cpp's enable gate — e-stop disables the gateway
// even if kEnabled is still set in the control word.
constexpr bool GatewayEnabled(uint32_t ds_flags) {
  return (ds_flags & ds::kEnabled) != 0 && (ds_flags & ds::kEStop) == 0;
}
static_assert(GatewayEnabled(ds::kDsAttached | ds::kEnabled | ds::kTeleop));
static_assert(!GatewayEnabled(ds::kDsAttached | ds::kEnabled | ds::kTeleop |
                              ds::kEStop));
static_assert(!GatewayEnabled(ds::kDsAttached | ds::kTeleop));
static_assert(!GatewayEnabled(ds::kDsAttached | ds::kEStop));

// Next loop deadline on the node clock after a tick finishes at `end`, with
// `deadline` the previous deadline and `period` the configured tick period.
// While each tick finishes on time, advancing by `period` keeps phase; after a
// host pause or tick overrun (`deadline + period <= end`), missed deadlines are
// dropped and the next tick aligns to `end + period` instead of firing a
// zero-sleep UDP catch-up burst (matching LoopSchedule::NextWakeUs semantics).
constexpr talos::event::MonotonicTime NextTickDeadline(
    talos::event::MonotonicTime deadline, talos::event::MonotonicTime end,
    talos::event::Duration period) {
  const auto next = deadline + period;
  return next > end ? next : end + period;
}
static_assert(
    NextTickDeadline(talos::event::MonotonicTime::from_nanos(10'000'000),
                     talos::event::MonotonicTime::from_nanos(10'500'000),
                     std::chrono::milliseconds{5}) ==
    talos::event::MonotonicTime::from_nanos(15'000'000));
static_assert(
    NextTickDeadline(talos::event::MonotonicTime::from_nanos(10'000'000),
                     talos::event::MonotonicTime::from_nanos(15'000'000),
                     std::chrono::milliseconds{5}) ==
    talos::event::MonotonicTime::from_nanos(20'000'000));
static_assert(
    NextTickDeadline(talos::event::MonotonicTime::from_nanos(10'000'000),
                     talos::event::MonotonicTime::from_nanos(200'000'000),
                     std::chrono::milliseconds{5}) ==
    talos::event::MonotonicTime::from_nanos(205'000'000));

double FiniteNumber(const std::string& text) {
  if (text.empty() ||
      std::isspace(static_cast<unsigned char>(text.front())) != 0) {
    throw std::invalid_argument("number must be finite: " + text);
  }
  std::size_t used = 0;
  double value = 0.0;
  try {
    value = std::stod(text, &used);
  } catch (const std::out_of_range&) {
    throw std::invalid_argument("number must be finite: " + text);
  }
  if (used != text.size() || !std::isfinite(value)) {
    throw std::invalid_argument("number must be finite: " + text);
  }
  return value;
}

talos::config::SimulationStartPose StartPose(const std::string& text) {
  const auto first = text.find(',');
  const auto second =
      first == std::string::npos ? first : text.find(',', first + 1);
  if (first == std::string::npos || second == std::string::npos ||
      text.find(',', second + 1) != std::string::npos) {
    throw std::invalid_argument("--start-pose needs X_M,Y_M,YAW_RAD");
  }
  return {.x_m = FiniteNumber(text.substr(0, first)),
          .y_m = FiniteNumber(text.substr(first + 1, second - first - 1)),
          .yaw_rad = FiniteNumber(text.substr(second + 1))};
}

std::string Shortest(double value) {
  std::array<char, 32> text{};
  const auto result =
      std::to_chars(text.data(), text.data() + text.size(), value);
  return {text.data(), result.ptr};
}

double Seconds(const std::string& text) {
  const double value = FiniteNumber(text);
  if (value <= 0) {
    throw std::invalid_argument("seconds must be finite and positive");
  }
  return value;
}

bool AllAsciiDigits(const std::string& text) {
  return !text.empty() && std::all_of(text.begin(), text.end(), [](char c) {
    return c >= '0' && c <= '9';
  });
}

uint16_t Port(const std::string& text) {
  if (!AllAsciiDigits(text)) {
    throw std::invalid_argument("port must be between 1 and 65535");
  }
  std::size_t used = 0;
  int value = 0;
  try {
    value = std::stoi(text, &used);
  } catch (const std::out_of_range&) {
    throw std::invalid_argument("port must be between 1 and 65535");
  }
  if (used != text.size() || value <= 0 || value > 65535) {
    throw std::invalid_argument("port must be between 1 and 65535");
  }
  return static_cast<uint16_t>(value);
}

uint64_t SessionId(const std::string& text) {
  if (!AllAsciiDigits(text)) {
    throw std::invalid_argument("--session-id must be an unsigned integer");
  }
  std::size_t used = 0;
  uint64_t value = 0;
  try {
    value = std::stoull(text, &used);
  } catch (const std::out_of_range&) {
    throw std::invalid_argument("--session-id must be an unsigned integer");
  }
  if (used != text.size()) {
    throw std::invalid_argument("--session-id must be an unsigned integer");
  }
  return value;
}

// /sim/state under --physics, then the extension's own topics (Topics(),
// asked once per program), numbered after it. Refuses a topic declared twice
// (the gateway's own included) and more sources than a registry row holds,
// which the row would otherwise drop.
talos::event::Manifest Manifest(bool physics,
                                const talos::event::Manifest& extension) {
  talos::event::Manifest manifest;
  if (physics) {
    manifest.push_back({
        .id = 0,
        .kind = talos::event::SourceKind::SENDER,
        .name = sim::topics::kState,
        .message_bytes = sizeof(geometry::OdometrySample),
        .alignment = alignof(geometry::OdometrySample),
        .schema = talos::schema::Of<geometry::OdometrySample>(),
    });
  }
  if (manifest.size() + extension.size() >
      talos::introspect::kMaxSourcesPerNode) {
    throw std::invalid_argument(
        "sim_gateway: " + std::to_string(manifest.size() + extension.size()) +
        " topics, more than the " +
        std::to_string(talos::introspect::kMaxSourcesPerNode) +
        " a registry row holds");
  }
  for (const auto& topic : extension) {
    for (const auto& earlier : manifest) {
      if (earlier.name == topic.name) {
        throw std::invalid_argument("sim_gateway: topic " + topic.name +
                                    " is declared twice");
      }
    }
    manifest.push_back(topic);
    manifest.back().id = static_cast<uint16_t>(manifest.size() - 1);
  }
  return manifest;
}

// The ring of each extension topic (GatewayExtension::TopicSlots), MAX_SLOTS
// for 0; refuses one RTMS cannot open.
std::vector<uint32_t> ExtensionSlots(
    const talos::simulation::GatewayExtension* extension,
    const talos::event::Manifest& topics) {
  std::vector<uint32_t> slots;
  for (std::size_t i = 0; i < topics.size(); ++i) {
    const uint32_t ring = extension->TopicSlots(i);
    if (ring != 0 &&
        (ring < 2 || ring > MAX_SLOTS || !std::has_single_bit(ring))) {
      throw std::invalid_argument(
          "sim_gateway: topic " + topics[i].name + " declares a ring of " +
          std::to_string(ring) + " slots; RTMS needs a power of two in [2, " +
          std::to_string(MAX_SLOTS) + "]");
    }
    slots.push_back(ring == 0 ? static_cast<uint32_t>(MAX_SLOTS) : ring);
  }
  return slots;
}

// Use the same manifest for the launcher probe and the live registration;
// the extension's topics, last in it, on their own rings.
int Describe(std::string_view target, const talos::event::Manifest& manifest,
             const std::vector<uint32_t>& extension_slots) {
  talos::introspect::Description description =
      talos::introspect::DescribeManifest(kNodeName, std::string{target},
                                          manifest, MAX_SLOTS);
  const std::size_t first = description.sources.size() - extension_slots.size();
  for (std::size_t i = 0; i < extension_slots.size(); ++i) {
    description.sources[first + i].slots = extension_slots[i];
  }
  std::cout << talos::introspect::DescribeToJson(description);
  return 0;
}

// The program's body stays outside talos:: so its unqualified namespaces
// resolve at global scope (geometry:: is the FlatBuffers message namespace,
// not talos::geometry).
int Run(int argc, char** argv,
        const talos::simulation::GatewayProgramInfo& program,
        talos::simulation::GatewayExtension* extension) {
  try {
    bool describe = false;
    bool saw_sim = false;
    // An extension configures the physics backend, so it implies --physics.
    bool physics = program.physics_only || extension != nullptr;
    double duration_s = 0;
    std::vector<ds::SchedulePhase> phases;
    std::vector<ds::JoystickEvent> joysticks;
    std::vector<talos::simulation::FaultEvent> faults;
    ds::Alliance alliance = ds::Alliance::kBlue;
    bool physics_options = false;
    uint64_t session_id = 0;
    // Where a harness writes the node-clock time the schedules count from;
    // unset, they count from the loop's start.
    std::optional<std::string> schedule_start_file;
    talos::hardware::sim::PhysicsOptions options;
    // --start-pose's pose; unset, the config's or the default applies.
    std::optional<talos::config::SimulationStartPose> start_pose_flag;
    const char* config_path = nullptr;
    // 0 until a flag sets one; unset, the config's port applies.
    uint16_t port_flag = 0;
    uint16_t remote_port_flag = 0;
    for (int i = 1; i < argc; ++i) {
      const std::string arg{argv[i]};
      if (arg == "--describe") {
        describe = true;
        continue;
      }
      if (arg == "--sim") {
        saw_sim = true;
        continue;
      }
      if (arg == "--physics") {
        physics = true;
      } else if (arg == "--start-pose" && i + 1 < argc) {
        physics_options = true;
        start_pose_flag = StartPose(argv[++i]);
      } else if (arg == "--wheel-radius-error" && i + 1 < argc) {
        physics_options = true;
        options.wheel_radius_error_frac = FiniteNumber(argv[++i]);
        if (options.wheel_radius_error_frac <= -0.5 ||
            options.wheel_radius_error_frac >= 0.5) {
          throw std::invalid_argument(
              "--wheel-radius-error must be > -0.5 and < 0.5");
        }
      } else if (arg == "--wheel-slip-gain" && i + 1 < argc) {
        physics_options = true;
        options.wheel_slip_gain = FiniteNumber(argv[++i]);
        if (options.wheel_slip_gain < 0.0) {
          throw std::invalid_argument("--wheel-slip-gain must be nonnegative");
        }
      } else if (arg == "--gyro-drift-radps" && i + 1 < argc) {
        physics_options = true;
        options.gyro_drift_radps = FiniteNumber(argv[++i]);
      } else if (arg == "--session-id" && i + 1 < argc) {
        session_id = SessionId(argv[++i]);
      } else if (arg == "--schedule-start-file" && i + 1 < argc) {
        schedule_start_file = argv[++i];
      } else if (arg == "--port" && i + 1 < argc) {
        port_flag = Port(argv[++i]);
      } else if (arg == "--remote-port" && i + 1 < argc) {
        remote_port_flag = Port(argv[++i]);
      } else if (arg == "--duration-s" && i + 1 < argc) {
        const std::string value = argv[++i];
        duration_s = value == "0" ? 0 : Seconds(value);
      } else if (arg == "--mode-schedule" && i + 1 < argc) {
        phases = ds::ParseModeSchedule(argv[++i]);
      } else if (arg == "--joystick-schedule" && i + 1 < argc) {
        joysticks = ds::ParseJoystickSchedule(argv[++i]);
      } else if (arg == "--fault-schedule" && i + 1 < argc) {
        physics_options = true;
        faults = talos::simulation::ParseFaults(argv[++i]);
      } else if (arg == "--alliance" && i + 1 < argc) {
        const std::string value = argv[++i];
        if (value == "blue") {
          alliance = ds::Alliance::kBlue;
        } else if (value == "red") {
          alliance = ds::Alliance::kRed;
        } else {
          throw std::invalid_argument("alliance must be blue or red");
        }
      } else if (arg == "--config" && i + 1 < argc) {
        config_path = argv[++i];
      } else {
        throw std::invalid_argument(kUsage);
      }
    }
    if (saw_sim && !describe) {
      throw std::invalid_argument(kUsage);
    }
    if (physics_options && !physics) {
      throw std::invalid_argument(
          std::string{"physics options require --physics\n"} + kUsage);
    }
    // The extension's topics, asked once: the probe and the live row must
    // declare the same endpoints.
    const talos::event::Manifest extension_topics =
        extension != nullptr ? extension->Topics() : talos::event::Manifest{};
    const talos::event::Manifest manifest = Manifest(physics, extension_topics);
    const std::vector<uint32_t> extension_slots =
        extension != nullptr ? ExtensionSlots(extension, extension_topics)
                             : std::vector<uint32_t>{};
    if (describe) return Describe(program.target, manifest, extension_slots);
    if (config_path == nullptr) {
      throw std::invalid_argument(
          "sim_gateway needs --config PATH: an unconfigured gateway stamps "
          "config_id 0, which every mechanism's AcceptState gate rejects, so "
          "nothing downstream would ever move");
    }
    talos::process::InstallStopHandlers();
    talos::simulation::GatewayPorts ports{};
    talos::hardware::Config hw_config;
    std::vector<talos::hardware::sim::HomeSwitch> home_switches;
    std::vector<talos::hardware::sim::SwerveModuleSpec> swerve_modules;
    std::vector<talos::hardware::sim::HardStop> hard_stops;
    std::optional<talos::hardware::sim::BatteryModel> battery_model;
    std::optional<talos::hardware::sim::ChassisDynamics> chassis_dynamics;
    std::vector<talos::hardware::sim::MechanismSpec> mechanisms;
    const auto robot_config = talos::config::ParseRobotConfig(config_path);
    std::string_view start_pose_source;
    if (extension == nullptr && robot_config.simulation.mujoco) {
      std::fprintf(stderr,
                   "sim_gateway: ignoring [simulation.mujoco]; "
                   "//talOS/simulation/mujoco:sim_gateway simulates it\n");
    }
    {
      ports = talos::simulation::ResolveGatewayPorts(robot_config, port_flag,
                                                     remote_port_flag);
      options.wheel_radius_m = robot_config.simulation.wheel_radius_m;
      start_pose_source = talos::simulation::ApplyStartPose(
          robot_config, start_pose_flag, options);
      hw_config = robot_config.hardware;
      hw_config.commissioned = true;
      home_switches.reserve(hw_config.digital_inputs.size());
      for (const auto& input : hw_config.digital_inputs) {
        const auto& sim = input.sim_home_switch;
        if (sim.motor.empty()) continue;
        home_switches.push_back({.input = input.name.str(),
                                 .motor = sim.motor,
                                 .closed_at_rot = sim.closed_at_rot,
                                 .closed_above = sim.closed_above,
                                 .closed_reads = sim.closed_reads});
      }
      swerve_modules.reserve(robot_config.simulation.swerve_modules.size());
      for (const auto& m : robot_config.simulation.swerve_modules) {
        swerve_modules.push_back(
            {.drive = m.drive, .steer = m.steer, .x_m = m.x_m, .y_m = m.y_m});
      }
      hard_stops.reserve(robot_config.simulation.hard_stops.size());
      for (const auto& stop : robot_config.simulation.hard_stops) {
        hard_stops.push_back({.motor = stop.motor,
                              .min_rot = stop.min_rot,
                              .max_rot = stop.max_rot});
      }
      if (robot_config.simulation.battery) {
        const auto& b = *robot_config.simulation.battery;
        battery_model = talos::hardware::sim::BatteryModel{
            .nominal_voltage_v = b.nominal_voltage_v,
            .internal_resistance_ohm = b.internal_resistance_ohm,
            .base_current_a = b.base_current_a,
            .brownout_voltage_v = b.brownout_voltage_v,
        };
      }
      if (robot_config.simulation.chassis) {
        const auto& c = *robot_config.simulation.chassis;
        chassis_dynamics = talos::hardware::sim::ChassisDynamics{
            .mass_kg = c.mass_kg,
            .moment_of_inertia_kg_m2 = c.moment_of_inertia_kg_m2.value_or(0.0),
            .traction_coefficient = c.traction_coefficient.value_or(0.0),
        };
      }
      mechanisms.reserve(robot_config.simulation.mechanisms.size());
      for (const auto& mech : robot_config.simulation.mechanisms) {
        mechanisms.push_back({
            .motor = mech.motor,
            .inertia_kg_m2 = mech.inertia_kg_m2,
            .load = mech.load == talos::config::SimulationLoadKind::kFlywheel
                        ? talos::hardware::sim::LoadKind::kFlywheel
                        : talos::hardware::sim::LoadKind::kRoller,
            .viscous_damping_nm_per_radps = mech.viscous_damping_nm_per_radps,
            .static_friction_nm = mech.static_friction_nm,
        });
      }
    }
    talos::hardware::SimBackend ideal;
    talos::hardware::sim::PhysicsSimBackend physics_backend{options};
    talos::hardware::Backend& backend =
        physics ? static_cast<talos::hardware::Backend&>(physics_backend)
                : static_cast<talos::hardware::Backend&>(ideal);
    const std::string backend_name =
        physics ? std::string{program.backend_name} : std::string{"ideal"};
    if (physics) {
      // Shortest round-trip text, so the line repeats the pose exactly.
      const std::string pose = Shortest(options.start_x_m) + "," +
                               Shortest(options.start_y_m) + "," +
                               Shortest(options.start_yaw_rad);
      // A MuJoCo world with sleeping on says so (nothing is added when off).
      const bool sleeping = extension != nullptr &&
                            robot_config.simulation.mujoco &&
                            robot_config.simulation.mujoco->sleeping;
      std::fprintf(stderr, "sim_gateway: backend=%s start_pose=%s (%.*s)%s\n",
                   backend_name.c_str(), pose.c_str(),
                   static_cast<int>(start_pose_source.size()),
                   start_pose_source.data(), sleeping ? " sleeping=on" : "");
    } else {
      std::fprintf(stderr, "sim_gateway: backend=%s\n", backend_name.c_str());
    }
    std::random_device random;
    // One boot id for the State and the Driver Station packets alike.
    const uint64_t boot_id = (uint64_t{random()} << 32) | random() | 1;
    talos::hardware::Gateway gateway{backend, boot_id};
    // No revision: it builds with the main processor's nodes, from the same
    // Bazel checkout, so it names only the program.
    gateway.SetBuild({.revision = 0,
                      .program = talos::hardware::GatewayProgram::kSimGateway,
                      .dirty = false});
    // Configure from the robot manifest before the loop: an unconfigured
    // gateway stamps config_id 0 and no configured flag, which every
    // mechanism's AcceptState/GatewayReady gates reject, so nothing
    // downstream would ever move. Commissioned is forced the way the
    // Driver Station enable already is below: this fake stands in for
    // commissioned hardware, and a simulator that coasts every demand to
    // zero for lack of a commissioning bit would conceal the control it is
    // meant to exercise.
    //
    // The wire carries no motor type, home switches ([digital_inputs.*]
    // sim_motor) or [simulation] swerve geometry, hard stops, battery,
    // chassis, or mechanism inertia: configured before Reconfigure, they apply
    // on the first State and survive any config the bridge pushes later.
    physics_backend.SetLocalMotorTypes(hw_config);
    physics_backend.SetHomeSwitches(std::move(home_switches));
    physics_backend.SetHardStops(std::move(hard_stops));
    if (!swerve_modules.empty()) {
      physics_backend.SetSwerveModules(std::move(swerve_modules));
    }
    if (battery_model) {
      physics_backend.SetBatteryModel(*battery_model);
    }
    if (chassis_dynamics) {
      physics_backend.SetChassisDynamics(*chassis_dynamics);
    }
    if (!mechanisms.empty()) {
      physics_backend.SetMechanisms(std::move(mechanisms));
    }
    if (extension != nullptr) {
      extension->Configure(robot_config, options, physics_backend);
    }
    if (!gateway.Reconfigure(hw_config)) {
      const std::optional<talos::hardware::ConfigError>& error =
          gateway.config_error();
      throw std::runtime_error(
          "sim_gateway: Reconfigure failed for the robot config" +
          (error ? ": " + talos::hardware::FormatConfigError(*error)
                 : std::string()));
    }
    talos::simulation::ResolveFaults(faults, hw_config);
    talos::hardware::Endpoint endpoint{gateway};
    if (endpoint.Open("127.0.0.1", ports.listen, "127.0.0.1", ports.send) !=
        talos::protocol::UdpStatus::kOk) {
      throw std::runtime_error("cannot open simulation socket");
    }
    std::optional<ipc::Publisher<geometry::OdometrySample>> truth_publisher;
    if (physics) truth_publisher.emplace(sim::topics::kState);
    std::optional<talos::introspect::NodeRegistration> registration;
    try {
      registration.emplace(talos::introspect::NodeRegistration::Identity{
          .name = kNodeName,
          .target = std::string{program.target},
          .session_id = session_id,
          .flags = talos::introspect::kFlagSimulation,
      });
      registration->publish(manifest);
    } catch (const std::exception& e) {
      std::fprintf(stderr, "sim_gateway: node registry unavailable: %s\n",
                   e.what());
    }
    // The gateway stands in for the RoboRIO, whose loop never waits behind the
    // main processor's nodes, so it takes the bridge's tier whenever the node
    // loops take theirs (TALOS_REALTIME=1 under --sim): left at timeshare, a
    // host stall that the time-constraint nodes ride out would stop every
    // State and expire their targets. Called last, from the only thread.
    talos::process::RealtimeOutcome realtime{
        .tier = talos::process::LoopTier::kBridge};
    if (talos::process::RealtimeRequested(/*simulation=*/true)) {
      realtime = talos::process::ConfigureRealtime(
          {.period = std::chrono::microseconds{gateway.config().period_us},
           .tier = realtime.tier});
    }
    if (registration) registration->set_realtime(realtime);
    // One node clock (Poller::now(): CLOCK_MONOTONIC on Linux,
    // CLOCK_MONOTONIC_RAW on macOS) drives schedules, State/DS stamps,
    // /sim/state truth, and loop wakeups so macOS sleep never skews them apart.
    talos::event::Poller poller;
    if (const int stop_fd = talos::process::StopFd(); stop_fd >= 0) {
      poller.add_fd(stop_fd);
    }
    auto next = talos::event::Poller::now();
    // The mode, joystick and fault schedules and --duration-s count from one
    // node-clock time zero, so a harness that chose it times its own actions
    // from the same instant. Until it is known and reached, the gateway holds
    // the schedules' time-zero state.
    std::optional<int64_t> start_ns;
    if (!schedule_start_file) start_ns = next.nanos();
    std::size_t phase = 0;
    std::size_t joystick_event = 0;
    ds::ModeTracker mode_tracker;
    ds::DriverStationData driver_station;
    driver_station.fms.alliance = alliance;
    driver_station.boot_id = boot_id;
    std::array<uint8_t, talos::protocol::kMaxPayloadSize> ds_buffer{};
    uint64_t active_ticks = 0;
    uint64_t last_ds_us = 0;
    uint32_t last_ds_mode = 0;  // ModeKey of the last packet sent
    uint64_t ticks = 0;
    uint64_t truth_samples = 0;
    uint64_t truth_dropped = 0;
    uint64_t truth_sequence = 0;
    int64_t last_truth_ns = 0;
    // What the extension stamps into the Driver Station packets (FmsStamp),
    // kept across ticks; and its topics' counts, sized once.
    talos::simulation::FmsStamp fms_stamp;
    std::vector<talos::simulation::TopicCount> topic_counts(
        extension_topics.size());
    const std::size_t first_extension_source =
        manifest.size() - extension_topics.size();
    while (!talos::process::stop_requested.load()) {
      const auto tick_time = talos::event::Poller::now();
      if (!start_ns) {
        // Written atomically by the harness (rename), so a read sees all of
        // it or no file.
        std::ifstream file{*schedule_start_file};
        if (file) {
          try {
            start_ns = talos::simulation::ParseScheduleStart(
                std::string{std::istreambuf_iterator<char>{file}, {}});
          } catch (const std::invalid_argument& e) {
            throw std::invalid_argument(*schedule_start_file + ": " + e.what());
          }
          const int64_t late_ns = tick_time.nanos() - *start_ns;
          if (late_ns > 0) {
            std::fprintf(stderr,
                         "sim_gateway: schedule start is %.1f ms in the past\n",
                         static_cast<double>(late_ns) / 1e6);
            talos::simulation::SkipPastFaults(
                faults, static_cast<double>(late_ns) / 1e9);
          }
        }
      }
      const int64_t since_ns =
          start_ns ? std::max<int64_t>(0, tick_time.nanos() - *start_ns) : 0;
      const auto elapsed = std::chrono::floor<std::chrono::microseconds>(
          std::chrono::nanoseconds{since_ns});
      const double elapsed_s = std::chrono::duration<double>(elapsed).count();
      if (duration_s > 0 && elapsed_s >= duration_s) break;
      while (phase + 1 < phases.size() && elapsed >= phases[phase].end) {
        ++phase;
      }
      while (joystick_event < joysticks.size() &&
             elapsed >= joysticks[joystick_event].start) {
        driver_station.joysticks[0] = joysticks[joystick_event++].joystick;
      }
      endpoint.SetSendEnabled(
          !talos::simulation::ApplyFaults(faults, elapsed_s, physics_backend));
      const uint64_t now =
          static_cast<uint64_t>(std::max<int64_t>(0, tick_time.nanos()) / 1000);
      driver_station.sample_time_us = now;
      driver_station.fms.flags =
          phases.empty() ? ds::kDsAttached | ds::kEnabled | ds::kTeleop
                         : phases[phase].flags;
      mode_tracker.Stamp(driver_station);
      if (extension != nullptr) {
        std::optional<uint64_t> schedule_us;
        if (start_ns && tick_time.nanos() >= *start_ns) {
          schedule_us = static_cast<uint64_t>(elapsed.count());
        }
        extension->BeforeTick({.now_us = now,
                               .schedule_us = schedule_us,
                               .driver_station = &driver_station});
      }
      endpoint.Tick(now, GatewayEnabled(driver_station.fms.flags));
      // After the tick's plant steps, so a packet sent on this tick carries
      // the extension's stamp as of this tick.
      if (extension != nullptr) {
        extension->Stamp(fms_stamp);
        driver_station.fms.match_time_s = fms_stamp.match_time_s;
        std::copy(fms_stamp.game_specific_message.begin(),
                  fms_stamp.game_specific_message.end() - 1,
                  std::begin(driver_station.fms.game_specific_message));
        std::end(driver_station.fms.game_specific_message)[-1] = '\0';
      }
      if (truth_publisher) {
        const auto sample_time = talos::event::Poller::now();
        const auto& truth = physics_backend.truth();
        const auto status = truth_publisher->write(geometry::OdometrySample{
            static_cast<uint64_t>(sample_time.nanos()),
            geometry::Pose2d{truth.x_m, truth.y_m, truth.yaw_rad},
            geometry::Twist2d{truth.vx_mps, truth.vy_mps, truth.omega_radps}});
        if (status == WriteResult::SUCCESS) {
          ++truth_samples;
          truth_sequence = status.sequence;
          last_truth_ns = sample_time.nanos();
        } else {
          ++truth_dropped;
        }
      }
      // Match the RoboRIO: a DS snapshot every 20 ms, independently of the
      // hardware/physics tick rate, and one at once on the tick that observes
      // a new mode.
      const uint32_t mode = ds::ModeKey(driver_station.fms.flags);
      if (last_ds_us == 0 || mode != last_ds_mode ||
          now - last_ds_us >= ds::kSamplePeriodUs) {
        const auto ds_size = ds::Encode(driver_station, ds_buffer);
        endpoint.SendFrame(talos::protocol::FrameType::kDriverStation, now,
                           ds_buffer.data(), ds_size);
        last_ds_us = now;
        last_ds_mode = mode;
      }
      if (extension != nullptr) extension->AfterTick(now);
      ++ticks;
      if (registration) {
        registration->begin_sample();
        registration->heartbeat();
        registration->set_dispatch_count(ticks);
        if (physics) {
          registration->set_source(0, truth_samples, truth_dropped,
                                   truth_sequence, last_truth_ns, 0, 0);
        }
        if (!topic_counts.empty()) {
          std::fill(topic_counts.begin(), topic_counts.end(),
                    talos::simulation::TopicCount{});
          extension->Counts(topic_counts);
          for (std::size_t i = 0; i < topic_counts.size(); ++i) {
            const auto& count = topic_counts[i];
            registration->set_source(
                static_cast<uint32_t>(first_extension_source + i),
                count.samples, count.dropped, count.sequence, count.last_ns, 0,
                0);
          }
        }
        if (talos::memory::LateLockRefused()) {
          registration->note_realtime_refused(talos::process::kStepRingLock);
        }
        registration->end_sample();
      }
      active_ticks +=
          (gateway.snapshot().flags & talos::hardware::kCommandActive) != 0;
      const auto period = talos::event::Duration{std::chrono::microseconds{
          gateway.configured() ? gateway.config().period_us : 5000}};
      next = NextTickDeadline(next, talos::event::Poller::now(), period);
      while (!talos::process::stop_requested.load()) {
        if (poller.wait_until(next) ==
            talos::event::Poller::WakeReason::DEADLINE) {
          break;
        }
        if (poller.reported_readable(talos::process::StopFd()) &&
            talos::process::ConsumeStopWake()) {
          break;
        }
      }
    }
    backend.Neutral();
    double first_wheel_rot = gateway.snapshot().motor_count > 0
                                 ? gateway.snapshot().motors[0].position_rot
                                 : 0.0;
    double first_wheel_rps = gateway.snapshot().motor_count > 0
                                 ? gateway.snapshot().motors[0].velocity_rps
                                 : 0.0;
    std::printf(
        "sim_gateway: active_ticks=%llu first_wheel_rotations=%.6f "
        "first_wheel_rps=%.6f command_active=%d\n",
        static_cast<unsigned long long>(active_ticks), first_wheel_rot,
        first_wheel_rps,
        (gateway.snapshot().flags & talos::hardware::kCommandActive) != 0);
    if (extension != nullptr) extension->Finish();
  } catch (const std::exception& e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 1;
  }
  return 0;
}

}  // namespace

namespace talos::simulation {

std::string_view ApplyStartPose(
    const config::RobotConfig& config,
    const std::optional<config::SimulationStartPose>& flag,
    hardware::sim::PhysicsOptions& options) {
  std::string_view source = "default";
  config::SimulationStartPose pose;
  if (flag) {
    pose = *flag;
    source = "--start-pose";
  } else if (config.simulation.start_pose) {
    pose = *config.simulation.start_pose;
    source = "config";
  }
  options.start_x_m = pose.x_m;
  options.start_y_m = pose.y_m;
  options.start_yaw_rad = pose.yaw_rad;
  return source;
}

int RunSimGateway(int argc, char** argv, const GatewayProgramInfo& program,
                  GatewayExtension* extension) {
  return Run(argc, argv, program, extension);
}

}  // namespace talos::simulation
