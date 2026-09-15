#include "cpp_node/node.h"
#include "talOS/process/node_main.h"

namespace {
constexpr const char* kNodeName = "sample_node";
constexpr const char* kNodeTarget = "//cpp_node:node";
}  // namespace

int main(int argc, char** argv) {
  auto factory = [&](auto& loop, const talos::process::NodeFlags&) {
    return sample::SampleNode{loop};
  };
  return talos::process::RunNode(
      argc, argv,
      {.node_name = kNodeName,
       .node_target = kNodeTarget,
       // Reads neither hardware nor configuration: accept --config and
       // discard it so the launcher can pass one flag set to every binary.
       .ignore_config_value = true},
      factory);
}
