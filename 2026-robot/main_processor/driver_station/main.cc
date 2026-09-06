#include "2026-robot/main_processor/driver_station/node.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/process/node_main.h"

namespace {
// This node's identity, written once. `--describe` and the live registry row
// have to name the same node and the same build target, or a viewer cannot tell
// a graph that changed from a graph it is reading two different names for.
constexpr const char* kNodeName = "driver_station";
constexpr const char* kNodeTarget =
    "//2026-robot/main_processor/driver_station:node";
}  // namespace

int main(int argc, char** argv) {
  // --sim only gates hardware commissioning, and this node owns no hardware,
  // so the launcher's flag changes nothing here beyond telling the registry
  // which mode the session is running in.
  auto factory = [&](auto& loop, const talos::process::NodeFlags& flags) {
    // Parsed only to fail fast on a broken configuration; this node holds no
    // policy of its own.
    (void)talos::config::ParseRobotConfig(flags.config_path);
    return talos::driver_station::DriverStationNode{loop};
  };
  return talos::process::RunNode(
      argc, argv,
      {.node_name = kNodeName,
       .node_target = kNodeTarget,
       .log_stream = "driver_station",
       .default_log_path = "/tmp/driver_station.tlog"},
      factory);
}
