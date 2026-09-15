#include "talos_nodes/year2026_operator_interface/node.h"

#include "talOS/configuration/config_parser.h"
#include "talOS/process/node_main.h"

namespace {
constexpr const char* kNodeName = "operator_interface";
constexpr const char* kNodeTarget =
    "//2026-robot/main_processor/operator_interface:node";
}  // namespace

int main(int argc, char** argv) {
  auto factory = [&](auto& loop, const talos::process::NodeFlags& flags) {
    return talos::oi::year2026::CreateNode(loop, flags);
  };
  return talos::process::RunNode(
      argc, argv,
      {.node_name = kNodeName, .node_target = kNodeTarget}, factory);
}
