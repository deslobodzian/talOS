#pragma once

// Shared main for hardware-owning subsystem nodes: the nodes that turn
// /hw/state into a /hw/request/<name>.
//
// Those mains differed only in includes, the node name, the device lookup,
// and (for swerve) the geometry build, so the factory preamble and the
// period-offset starter live here, parameterized by a build lambda that
// constructs the node from its parsed subsystem:
//
//   int main(int argc, char** argv) {
//     return talos::process::RunHardwareNode(
//         argc, argv, kNodeName, kNodeTarget,
//         [&](auto& loop, const talos::process::HardwareSubsystem& sub,
//             bool simulation) {
//           return talos::drive::DrivetrainNode{loop, sub.robot.hardware,
//                                               geometry, sub.devices,
//                                               simulation};
//         });
//   }
//
// Log identity derives from the node name (see NodeSpec). Nodes that own no
// hardware keep calling RunNode directly.

#include <chrono>
#include <string>
#include <string_view>

#include "talOS/configuration/config_parser.h"
#include "talOS/process/node_main.h"

namespace talos::process {

// The parsed robot configuration plus this subsystem's slice of it.
struct HardwareSubsystem {
  config::RobotConfig robot;
  hardware::Devices devices;
  int period_us = 0;
};

// Parse once, commission in simulation, and take this subsystem's devices
// (empty when the config declares none for it).
inline HardwareSubsystem LoadHardwareSubsystem(const std::string& config_path,
                                               std::string_view subsystem,
                                               bool simulation) {
  HardwareSubsystem sub;
  sub.robot = config::ParseRobotConfig(config_path);
  if (simulation) {
    sub.robot.hardware.commissioned = true;
  }
  if (const hardware::Devices* dev_ptr =
          sub.robot.GetDevices(std::string{subsystem})) {
    sub.devices = *dev_ptr;
  }
  sub.period_us = static_cast<int>(sub.robot.hardware.period_us);
  return sub;
}

// Run a hardware-owning node: the same three branches as RunNode, with the
// node built by `build` and its periodic timer started one period after now.
// The starter is identical on live and replay: the replay loop's clock already
// reads the recorded origin, so the same line produces the recorded schedule.
template <typename Build>
int RunHardwareNode(int argc, char** argv, const char* node_name,
                    const char* node_target, Build build) {
  int period_us = 0;
  auto factory = [&](auto& loop, const NodeFlags& flags) {
    HardwareSubsystem sub =
        LoadHardwareSubsystem(flags.config_path, node_name, flags.simulation);
    period_us = sub.period_us;
    return build(loop, sub, flags.simulation);
  };
  auto starter = [&](auto& node, auto& loop) {
    node.Start(loop.monotonic_now() + std::chrono::microseconds{period_us});
  };
  return RunNode(argc, argv,
                 {.node_name = node_name, .node_target = node_target}, factory,
                 starter);
}

}  // namespace talos::process
