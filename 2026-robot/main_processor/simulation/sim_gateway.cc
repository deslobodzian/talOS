#include <algorithm>
#include <array>
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
#include <thread>
#include <vector>

#include "geometry/geometry_descriptor.h"
#include "geometry/geometry_generated.h"
#include "schedules.h"
#include "sim/topics.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/driver_station/schedule.h"
#include "talOS/events/os/poller.h"
#include "talOS/hardware/endpoint.h"
#include "talOS/hardware/sim_backend.h"
#include "talOS/hardware/sim_physics.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/registry.h"
#include "talOS/ipc/publisher.h"
#include "talOS/process/process.h"

namespace {

namespace ds = talos::driver_station;

constexpr const char* kNodeName = "sim_gateway";
constexpr const char* kNodeTarget =
    "//2026-robot/main_processor/simulation:sim_gateway";
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

double FiniteNumber(const std::string& text) {
  std::size_t used = 0;
  const double value = std::stod(text, &used);
  if (used != text.size() || !std::isfinite(value)) {
    throw std::invalid_argument("number must be finite: " + text);
  }
  return value;
}

void StartPose(const std::string& text,
               talos::hardware::sim::PhysicsOptions& options) {
  const auto first = text.find(',');
  const auto second =
      first == std::string::npos ? first : text.find(',', first + 1);
  if (first == std::string::npos || second == std::string::npos ||
      text.find(',', second + 1) != std::string::npos) {
    throw std::invalid_argument("--start-pose needs X_M,Y_M,YAW_RAD");
  }
  options.start_x_m = FiniteNumber(text.substr(0, first));
  options.start_y_m = FiniteNumber(text.substr(first + 1, second - first - 1));
  options.start_yaw_rad = FiniteNumber(text.substr(second + 1));
}

double Seconds(const std::string& text) {
  std::size_t used = 0;
  const double value = std::stod(text, &used);
  if (used != text.size() || !std::isfinite(value) || value <= 0) {
    throw std::invalid_argument("seconds must be finite and positive");
  }
  return value;
}

uint16_t Port(const std::string& text) {
  std::size_t used = 0;
  const int value = std::stoi(text, &used);
  if (used != text.size() || value <= 0 || value > 65535) {
    throw std::invalid_argument("port must be between 1 and 65535");
  }
  return static_cast<uint16_t>(value);
}

talos::event::Manifest Manifest(bool physics) {
  if (!physics) return {};
  return {{
      .id = 0,
      .kind = talos::event::SourceKind::SENDER,
      .name = sim::topics::kState,
      .message_bytes = sizeof(geometry::OdometrySample),
      .alignment = alignof(geometry::OdometrySample),
      .schema = talos::schema::Of<geometry::OdometrySample>(),
  }};
}

// Use the same manifest for the launcher probe and the live registration.
int Describe(bool physics) {
  std::cout << talos::introspect::DescribeToJson(
      talos::introspect::DescribeManifest(kNodeName, kNodeTarget,
                                          Manifest(physics), MAX_SLOTS));
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    bool describe = false;
    bool physics = false;
    for (int i = 1; i < argc; ++i) {
      describe |= std::string{argv[i]} == "--describe";
    }
    double duration_s = 0;
    std::vector<ds::SchedulePhase> phases;
    std::vector<ds::JoystickEvent> joysticks;
    std::vector<robot::simulation::FaultEvent> faults;
    ds::Alliance alliance = ds::Alliance::kBlue;
    bool physics_options = false;
    uint64_t session_id = 0;
    // Where a harness writes the node-clock time the schedules count from;
    // unset, they count from the loop's start.
    std::optional<std::string> schedule_start_file;
    talos::hardware::sim::PhysicsOptions options{.start_x_m = 2.0,
                                                 .start_y_m = 2.0};
    const char* config_path = nullptr;
    uint16_t local_port = 5802;
    uint16_t remote_port = 5803;
    for (int i = 1; i < argc; ++i) {
      const std::string arg{argv[i]};
      if (arg == "--describe" || (describe && arg == "--sim")) continue;
      if (arg == "--physics") {
        physics = true;
      } else if (arg == "--start-pose" && i + 1 < argc) {
        physics_options = true;
        StartPose(argv[++i], options);
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
        const std::string value = argv[++i];
        std::size_t used = 0;
        session_id = std::stoull(value, &used);
        if (used != value.size() || value.starts_with('-')) {
          throw std::invalid_argument(
              "--session-id must be an unsigned integer");
        }
      } else if (arg == "--schedule-start-file" && i + 1 < argc) {
        schedule_start_file = argv[++i];
      } else if (arg == "--port" && i + 1 < argc) {
        local_port = Port(argv[++i]);
      } else if (arg == "--remote-port" && i + 1 < argc) {
        remote_port = Port(argv[++i]);
      } else if (arg == "--duration-s" && i + 1 < argc) {
        const std::string value = argv[++i];
        duration_s = value == "0" ? 0 : Seconds(value);
      } else if (arg == "--mode-schedule" && i + 1 < argc) {
        phases = ds::ParseModeSchedule(argv[++i]);
      } else if (arg == "--joystick-schedule" && i + 1 < argc) {
        joysticks = ds::ParseJoystickSchedule(argv[++i]);
      } else if (arg == "--fault-schedule" && i + 1 < argc) {
        physics_options = true;
        faults = robot::simulation::ParseFaults(argv[++i]);
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
    if (physics_options && !physics) {
      throw std::invalid_argument(
          std::string{"physics options require --physics\n"} + kUsage);
    }
    if (describe) return Describe(physics);
    if (config_path == nullptr) {
      throw std::invalid_argument(
          "sim_gateway needs --config PATH: an unconfigured gateway stamps "
          "config_id 0, which every mechanism's AcceptState gate rejects, so "
          "nothing downstream would ever move");
    }
    talos::process::InstallStopHandlers();
    talos::hardware::SimBackend ideal;
    talos::hardware::sim::PhysicsSimBackend physics_backend{options};
    talos::hardware::Backend& backend =
        physics ? static_cast<talos::hardware::Backend&>(physics_backend)
                : static_cast<talos::hardware::Backend&>(ideal);
    std::fprintf(stderr, "sim_gateway: backend=%s\n",
                 physics ? "physics" : "ideal");
    std::random_device random;
    // One boot id for the State and the Driver Station packets alike.
    const uint64_t boot_id = (uint64_t{random()} << 32) | random() | 1;
    talos::hardware::Gateway gateway{backend, boot_id};
    // Configure from the robot manifest before the loop: an unconfigured
    // gateway stamps config_id 0 and no configured flag, which every
    // mechanism's AcceptState/GatewayReady gates reject, so nothing
    // downstream would ever move. Commissioned is forced the way the
    // Driver Station enable already is below: this fake stands in for
    // commissioned hardware, and a simulator that coasts every demand to
    // zero for lack of a commissioning bit would conceal the control it is
    // meant to exercise.
    {
      auto robot_config = talos::config::ParseRobotConfig(config_path);
      auto hw_config = robot_config.hardware;
      hw_config.commissioned = true;
      if (!gateway.Reconfigure(hw_config)) {
        throw std::runtime_error(
            "sim_gateway: Reconfigure failed for the robot config");
      }
      robot::simulation::ResolveFaults(faults, hw_config);
    }
    talos::hardware::Endpoint endpoint{gateway};
    if (endpoint.Open("127.0.0.1", local_port, "127.0.0.1", remote_port) !=
        talos::protocol::UdpStatus::kOk) {
      throw std::runtime_error("cannot open simulation socket");
    }
    std::optional<ipc::Publisher<geometry::OdometrySample>> truth_publisher;
    if (physics) truth_publisher.emplace(sim::topics::kState);
    std::optional<talos::introspect::NodeRegistration> registration;
    try {
      registration.emplace(talos::introspect::NodeRegistration::Identity{
          .name = kNodeName,
          .target = kNodeTarget,
          .session_id = session_id,
          .flags = talos::introspect::kFlagSimulation,
      });
      registration->publish(Manifest(physics));
    } catch (const std::exception& e) {
      std::fprintf(stderr, "sim_gateway: node registry unavailable: %s\n",
                   e.what());
    }
    auto next = std::chrono::steady_clock::now();
    // The mode, joystick and fault schedules and --duration-s count from one
    // node-clock time zero, so a harness that chose it times its own actions
    // from the same instant. Until it is known and reached, the gateway holds
    // the schedules' time-zero state.
    std::optional<int64_t> start_ns;
    if (!schedule_start_file) start_ns = talos::event::Poller::now().nanos();
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
    while (!talos::process::stop_requested.load()) {
      const auto tick_time = std::chrono::steady_clock::now();
      if (!start_ns) {
        // Written atomically by the harness (rename), so a read sees all of
        // it or no file.
        std::ifstream file{*schedule_start_file};
        if (file) {
          try {
            start_ns = robot::simulation::ParseScheduleStart(
                std::string{std::istreambuf_iterator<char>{file}, {}});
          } catch (const std::invalid_argument& e) {
            throw std::invalid_argument(*schedule_start_file + ": " + e.what());
          }
          const int64_t late_ns =
              talos::event::Poller::now().nanos() - *start_ns;
          if (late_ns > 0) {
            std::fprintf(stderr,
                         "sim_gateway: schedule start is %.1f ms in the past\n",
                         static_cast<double>(late_ns) / 1e6);
            robot::simulation::SkipPastFaults(
                faults, static_cast<double>(late_ns) / 1e9);
          }
        }
      }
      const int64_t since_ns =
          start_ns ? std::max<int64_t>(
                         0, talos::event::Poller::now().nanos() - *start_ns)
                   : 0;
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
          !robot::simulation::ApplyFaults(faults, elapsed_s, physics_backend));
      driver_station.sample_time_us =
          std::chrono::duration_cast<std::chrono::microseconds>(
              tick_time.time_since_epoch())
              .count();
      driver_station.fms.flags =
          phases.empty() ? ds::kDsAttached | ds::kEnabled | ds::kTeleop
                         : phases[phase].flags;
      mode_tracker.Stamp(driver_station);
      const auto now = driver_station.sample_time_us;
      endpoint.Tick(now, (driver_station.fms.flags & ds::kEnabled) != 0);
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
      ++ticks;
      if (registration) {
        registration->begin_sample();
        registration->heartbeat();
        registration->set_dispatch_count(ticks);
        if (physics) {
          registration->set_source(0, truth_samples, truth_dropped,
                                   truth_sequence, last_truth_ns, 0, 0);
        }
        registration->end_sample();
      }
      active_ticks +=
          (gateway.snapshot().flags & talos::hardware::kCommandActive) != 0;
      next += std::chrono::microseconds{
          gateway.configured() ? gateway.config().period_us : 5000};
      std::this_thread::sleep_until(next);
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
  } catch (const std::exception& e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 1;
  }
}
