#include <chrono>

#include "2026-robot/main_processor/drivetrain/geometry.h"
#include "2026-robot/main_processor/drivetrain/node.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/process/node_main.h"

namespace {
// This node's identity, written once. `--describe` and the live registry row
// have to name the same node and the same build target, or a viewer cannot tell
// a graph that changed from a graph it is reading two different names for.
constexpr const char* kNodeName = "drivetrain";
constexpr const char* kNodeTarget =
    "//2026-robot/main_processor/drivetrain:node";
}  // namespace

int main(int argc, char** argv) {
  // The start offset, derived from the config by the factory. Exactly one
  // runner branch executes per process, so the starter always observes the
  // value the factory stored.
  int period_us = 0;
  auto factory = [&](auto& loop, const talos::process::NodeFlags& flags) {
    auto robot_config = talos::config::ParseRobotConfig(flags.config_path);
    if (flags.simulation) {
      robot_config.hardware.commissioned = true;
    }
    const auto* dev_ptr = robot_config.GetDevices("drivetrain");
    talos::hardware::Devices devices =
        dev_ptr ? *dev_ptr : talos::hardware::Devices{};
    auto config = robot_config.hardware;
    const auto geometry = talos::drive::BuildSwerveGeometry(robot_config);
    period_us = config.period_us;
    return talos::drive::DrivetrainNode{loop, config, geometry, devices,
                                        flags.simulation};
  };
  // Identical to the live setup below: the replay loop's clock already reads
  // the recorded origin, so the same line produces the recorded schedule.
  auto starter = [&](auto& node, auto& loop) {
    node.Start(loop.monotonic_now() +
               std::chrono::microseconds{period_us});
  };
  return talos::process::RunNode(
      argc, argv,
      {.node_name = kNodeName,
       .node_target = kNodeTarget,
       .log_stream = "drivetrain",
       .default_log_path = "/tmp/drivetrain.tlog"},
      factory, starter);
}
