#include "2026-robot/main_processor/odometry/node.h"
#include "talOS/process/node_main.h"

namespace {
// This node's identity, written once. `--describe` and the live registry row
// have to name the same node and the same build target, or a viewer cannot tell
// a graph that changed from a graph it is reading two different names for.
constexpr const char* kNodeName = "odometry";
constexpr const char* kNodeTarget = "//2026-robot/main_processor/odometry:node";
}  // namespace

int main(int argc, char** argv) {
  // The launcher passes the same flags to every node it starts. This one
  // reads no hardware and no configuration, so --sim and --config are
  // accepted rather than refused; --sim additionally tells the registry
  // which mode the session is running in.
  auto factory = [&](auto& loop, const talos::process::NodeFlags&) {
    return talos::odometry::OdometryNode{loop};
  };
  return talos::process::RunNode(
      argc, argv,
      {.node_name = kNodeName,
       .node_target = kNodeTarget,
       .log_stream = "odometry",
       .default_log_path = "/tmp/odometry.tlog",
       .ignore_config_value = true},
      factory, talos::process::NoStart{});
}
