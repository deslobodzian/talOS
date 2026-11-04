// Sim/bench tool only: publishes a chassis target on one drivetrain lane,
// /drivetrain/target/<source>, bypassing the operator interface. The
// drivetrain follows it only while the robot is enabled and the Driver
// Station mode's [robot.targets] policy selects that source (teleop by
// default). Never use it on a robot at a match.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "drivetrain/drive_message_generated.h"
#include "talOS/devices/target.h"
#include "talOS/ipc/publisher.h"

namespace {

constexpr const char* kUsage =
    "usage: send_target [--source NAME] VX_MPS VY_MPS OMEGA_RADPS DURATION_S\n"
    "  Sim/bench tool only: writes /drivetrain/target/<NAME> (default teleop)\n"
    "  directly, bypassing the operator interface. The drivetrain drives only\n"
    "  while enabled in a Driver Station mode whose policy selects NAME.";

}  // namespace

int main(int argc, char** argv) {
  try {
    std::string source = "teleop";
    std::vector<std::string> positional;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "--source" && i + 1 < argc) {
        source = argv[++i];
      } else {
        positional.push_back(arg);
      }
    }
    if (positional.size() != 4 || source.empty()) {
      throw std::invalid_argument(kUsage);
    }
    const double vx = std::stod(positional[0]), vy = std::stod(positional[1]),
                 omega = std::stod(positional[2]);
    const int seconds = std::stoi(positional[3]);
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(omega) ||
        seconds <= 0) {
      throw std::invalid_argument("invalid target");
    }
    using Envelope = talos::TargetEnvelope<drivetrain::ChassisTarget>;
    ipc::Publisher<Envelope> publisher{"/drivetrain/target/" + source};
    const auto end =
        std::chrono::steady_clock::now() + std::chrono::seconds{seconds};
    while (std::chrono::steady_clock::now() < end) {
      publisher.write(Envelope{1, drivetrain::ChassisTarget{vx, vy, omega}});
      std::this_thread::sleep_for(std::chrono::milliseconds{20});
    }
    publisher.write(Envelope{});  // release: active = 0
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 1;
  }
}
