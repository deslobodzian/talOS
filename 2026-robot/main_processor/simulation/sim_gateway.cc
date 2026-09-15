#include <chrono>
#include <cstdio>
#include <random>
#include <string>
#include <thread>

#include "talOS/configuration/config_parser.h"
#include "talOS/hardware/endpoint.h"
#include "talOS/hardware/sim_backend.h"
#include "talOS/hardware/sim_physics.h"
#include "talOS/introspection/describe.h"
#include "talOS/process/process.h"

#include <iostream>

namespace {

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
    int duration_s = 0;
    bool physics = false;
    const char* config_path = nullptr;
    for (int i = 1; i < argc; ++i) {
      const std::string arg{argv[i]};
      if (arg == "--physics") {
        physics = true;
      } else if (arg == "--duration-s" && i + 1 < argc) {
        duration_s = std::stoi(argv[++i]);
      } else if (arg == "--config" && i + 1 < argc) {
        config_path = argv[++i];
      } else {
        throw std::invalid_argument(
            "usage: sim_gateway --config PATH [--physics] [--duration-s N]");
      }
    }
    if (duration_s < 0)
      throw std::invalid_argument("duration must be nonnegative");
    if (config_path == nullptr)
      throw std::invalid_argument(
          "sim_gateway needs --config PATH: an unconfigured gateway stamps "
          "config_id 0, which every mechanism's AcceptState gate rejects, so "
          "nothing downstream would ever move");
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
        talos::protocol::UdpStatus::kOk)
      throw std::runtime_error("cannot open simulation socket");
    const auto end =
        std::chrono::steady_clock::now() + std::chrono::seconds{duration_s};
    auto next = std::chrono::steady_clock::now();
    uint64_t active_ticks = 0;
    while (!talos::process::stop_requested.load() &&
           (!duration_s || std::chrono::steady_clock::now() < end)) {
      const auto now = std::chrono::duration_cast<std::chrono::microseconds>(
                           std::chrono::steady_clock::now().time_since_epoch())
                           .count();
      endpoint.Tick(now,
                    true);  // Only this fake bypasses Driver Station enable.
      active_ticks +=
          (gateway.snapshot().flags & talos::hardware::kCommandActive) != 0;
      next += std::chrono::microseconds{gateway.configured() ? gateway.config().period_us : 5000};
      std::this_thread::sleep_until(next);
    }
    backend.Neutral();
    double first_wheel_rot = gateway.snapshot().motor_count > 0 ? gateway.snapshot().motors[0].position_rot : 0.0;
    double first_wheel_rps = gateway.snapshot().motor_count > 0 ? gateway.snapshot().motors[0].velocity_rps : 0.0;
    std::printf(
        "sim_gateway: active_ticks=%llu first_wheel_rotations=%.6f "
        "first_wheel_rps=%.6f command_active=%d\n",
        static_cast<unsigned long long>(active_ticks),
        first_wheel_rot,
        first_wheel_rps,
        (gateway.snapshot().flags & talos::hardware::kCommandActive) != 0);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 1;
  }
}
