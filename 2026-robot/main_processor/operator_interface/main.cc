#include "2026-robot/main_processor/operator_interface/node.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/process/node_main.h"

namespace {
// This node's identity, written once. `--describe` and the live registry row
// have to name the same node and the same build target, or a viewer cannot tell
// a graph that changed from a graph it is reading two different names for.
constexpr const char* kNodeName = "operator_interface";
constexpr const char* kNodeTarget =
    "//2026-robot/main_processor/operator_interface:node";
}  // namespace

int main(int argc, char** argv) {
  // --sim only gates hardware commissioning, and this node owns no hardware,
  // so the launcher's flag changes nothing here beyond telling the registry
  // which mode the session is running in.
  auto factory = [&](auto& loop, const talos::process::NodeFlags& flags) {
    const auto robot_config = talos::config::ParseRobotConfig(flags.config_path);
    talos::oi::OperatorInterfaceConfig oi{};
    if (const auto* tbl =
            robot_config.GetSubsystemTable("operator_interface")) {
      oi.max_linear_mps = (*tbl)["max_linear_mps"].value_or(oi.max_linear_mps);
      oi.max_angular_radps =
          (*tbl)["max_angular_radps"].value_or(oi.max_angular_radps);
      oi.deadband = (*tbl)["deadband"].value_or(oi.deadband);
      oi.shooter_target_rps =
          (*tbl)["shooter_target_rps"].value_or(oi.shooter_target_rps);
      oi.field_oriented = (*tbl)["field_oriented"].value_or(oi.field_oriented);
      oi.shoot_button_mask =
          (*tbl)["shoot_button_mask"].value_or(oi.shoot_button_mask);
    }
    return talos::oi::OperatorInterfaceNode{loop, oi};
  };
  return talos::process::RunNode(
      argc, argv,
      {.node_name = kNodeName,
       .node_target = kNodeTarget,
       .log_stream = "operator_interface",
       .default_log_path = "/tmp/operator_interface.tlog"},
      factory);
}
