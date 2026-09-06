#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <thread>

#include "2026-robot/main_processor/drivetrain/drive_message_generated.h"
#include "2026-robot/main_processor/drivetrain/packet.h"
#include "talOS/events/os/poller.h"
#include "talOS/ipc/publisher.h"

int main(int argc, char** argv) {
  try {
    if (argc != 5)
      throw std::invalid_argument(
          "usage: send_target VX_MPS VY_MPS OMEGA_RADPS DURATION_S");
    const double vx = std::stod(argv[1]), vy = std::stod(argv[2]),
                 omega = std::stod(argv[3]);
    const int seconds = std::stoi(argv[4]);
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(omega) ||
        seconds <= 0)
      throw std::invalid_argument("invalid target");
    ipc::Publisher<talos::drive::ChassisTarget> publisher{
        talos::drive::kTargetTopic};
    const auto end =
        std::chrono::steady_clock::now() + std::chrono::seconds{seconds};
    while (std::chrono::steady_clock::now() < end) {
      publisher.write(talos::drive::ChassisTarget{
          vx, vy, omega, talos::event::Poller::now().nanos(), true});
      std::this_thread::sleep_for(std::chrono::milliseconds{20});
    }
    publisher.write(talos::drive::ChassisTarget{
        0, 0, 0, talos::event::Poller::now().nanos(), false});
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 1;
  }
}
