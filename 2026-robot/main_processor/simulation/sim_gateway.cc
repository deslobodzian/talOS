#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "talOS/configuration/config_parser.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/endpoint.h"
#include "talOS/hardware/sim_backend.h"
#include "talOS/hardware/sim_physics.h"
#include "talOS/introspection/describe.h"
#include "talOS/process/process.h"

namespace {

namespace ds = talos::driver_station;

struct Phase {
  uint32_t flags;
  double end_s;
};

double Seconds(const std::string& text) {
  std::size_t used = 0;
  const double value = std::stod(text, &used);
  if (used != text.size() || !std::isfinite(value) || value <= 0) {
    throw std::invalid_argument("seconds must be finite and positive");
  }
  return value;
}

std::vector<Phase> ParseSchedule(const std::string& spec) {
  std::vector<Phase> phases;
  std::size_t start = 0;
  double end_s = 0;
  while (start < spec.size()) {
    const auto comma = spec.find(',', start);
    const auto part = spec.substr(start, comma - start);
    const auto colon = part.find(':');
    if (colon == std::string::npos) {
      throw std::invalid_argument("mode schedule needs mode:seconds phases");
    }
    const auto mode = part.substr(0, colon);
    uint32_t flags = ds::kDsAttached;
    if (mode == "auto" || mode == "autonomous") {
      flags |= ds::kEnabled | ds::kAutonomous;
    } else if (mode == "teleop") {
      flags |= ds::kEnabled | ds::kTeleop;
    } else if (mode == "test") {
      flags |= ds::kEnabled | ds::kTest;
    } else if (mode != "disabled") {
      throw std::invalid_argument("unknown mode: " + mode);
    }
    end_s += Seconds(part.substr(colon + 1));
    if (!std::isfinite(end_s)) {
      throw std::invalid_argument("mode schedule duration overflow");
    }
    phases.push_back({flags, end_s});
    if (comma == std::string::npos) break;
    start = comma + 1;
    if (start == spec.size()) {
      throw std::invalid_argument("empty mode schedule phase");
    }
  }
  if (phases.empty()) throw std::invalid_argument("empty mode schedule");
  return phases;
}

// The launcher probes every roster binary with `--describe --config PATH
// [--sim]` before spawning anything. The gateway answers from its binary
// identity: it holds no RTMS endpoints, so the envelope carries no sources.
int Describe() {
  talos::introspect::Description description;
  description.name = "sim_gateway";
  description.target = "//2026-robot/main_processor/simulation:sim_gateway";
  std::cout << talos::introspect::DescribeToJson(description);
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    for (int i = 1; i < argc; ++i) {
      if (std::string{argv[i]} == "--describe") return Describe();
    }
    double duration_s = 0;
    std::vector<Phase> phases;
    ds::Alliance alliance = ds::Alliance::kBlue;
    bool physics = false;
    const char* config_path = nullptr;
    for (int i = 1; i < argc; ++i) {
      const std::string arg{argv[i]};
      if (arg == "--physics") {
        physics = true;
      } else if (arg == "--duration-s" && i + 1 < argc) {
        const std::string value = argv[++i];
        duration_s = value == "0" ? 0 : Seconds(value);
      } else if (arg == "--mode-schedule" && i + 1 < argc) {
        phases = ParseSchedule(argv[++i]);
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
        throw std::invalid_argument(
            "usage: sim_gateway --config PATH [--physics] [--duration-s N] "
            "[--mode-schedule mode:seconds,...] [--alliance blue|red]");
      }
    }
    if (config_path == nullptr) {
      throw std::invalid_argument(
          "sim_gateway needs --config PATH: an unconfigured gateway stamps "
          "config_id 0, which every mechanism's AcceptState gate rejects, so "
          "nothing downstream would ever move");
    }
    talos::process::InstallStopHandlers();
    talos::hardware::SimBackend ideal;
    talos::hardware::sim::PhysicsSimBackend physics_backend;
    talos::hardware::Backend& backend =
        physics ? static_cast<talos::hardware::Backend&>(physics_backend)
                : static_cast<talos::hardware::Backend&>(ideal);
    std::fprintf(stderr, "sim_gateway: backend=%s\n",
                 physics ? "physics" : "ideal");
    std::random_device random;
    talos::hardware::Gateway gateway{backend,
                                     (uint64_t{random()} << 32) | random() | 1};
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
    }
    talos::hardware::Endpoint endpoint{gateway};
    if (endpoint.Open("127.0.0.1", 5802, "127.0.0.1", 5803) !=
        talos::protocol::UdpStatus::kOk) {
      throw std::runtime_error("cannot open simulation socket");
    }
    const auto start = std::chrono::steady_clock::now();
    auto next = start;
    std::size_t phase = 0;
    ds::ModeTracker mode_tracker;
    ds::DriverStationData driver_station;
    driver_station.fms.alliance = alliance;
    std::array<uint8_t, talos::protocol::kMaxPayloadSize> ds_buffer{};
    uint64_t active_ticks = 0;
    while (!talos::process::stop_requested.load()) {
      const auto tick_time = std::chrono::steady_clock::now();
      const double elapsed_s =
          std::chrono::duration<double>(tick_time - start).count();
      if (duration_s > 0 && elapsed_s >= duration_s) break;
      while (phase + 1 < phases.size() && elapsed_s >= phases[phase].end_s) {
        ++phase;
      }
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
      const auto ds_size = ds::Encode(driver_station, ds_buffer);
      endpoint.SendFrame(talos::protocol::FrameType::kDriverStation, now,
                         ds_buffer.data(), ds_size);
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
