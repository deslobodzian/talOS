#include <chrono>
#include <cstdio>
#include <random>
#include <string>
#include <thread>

#include "talOS/hardware/endpoint.h"
#include "talOS/hardware/sim_backend.h"
#include "talOS/process/process.h"

int main(int argc, char** argv) {
  try {
    int duration_s = 0;
    if (argc == 3 && std::string{argv[1]} == "--duration-s")
      duration_s = std::stoi(argv[2]);
    else if (argc != 1)
      throw std::invalid_argument("usage: sim_gateway [--duration-s N]");
    if (duration_s < 0)
      throw std::invalid_argument("duration must be nonnegative");
    talos::process::InstallStopHandlers();
    talos::hardware::SimBackend backend;
    std::random_device random;
    talos::hardware::Gateway gateway{backend,
                                     (uint64_t{random()} << 32) | random() | 1};
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
