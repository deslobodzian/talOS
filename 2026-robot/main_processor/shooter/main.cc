#include "talos_nodes/shooter/node.h"
#include "talOS/process/hardware_main.h"

namespace {
// Identity: the registry row and --describe name the same node and target;
// log identity derives from the name (see talos::process::NodeSpec).
constexpr const char* kNodeName = "shooter";
constexpr const char* kNodeTarget = "//2026-robot/main_processor/shooter:node";
}  // namespace

int main(int argc, char** argv) {
  return talos::process::RunHardwareNode(
      argc, argv, kNodeName, kNodeTarget,
      [&](auto& loop, const talos::process::HardwareSubsystem& sub,
          bool /*simulation*/) {
        return talos::shooter::ShooterNode{loop, sub.robot.hardware,
                                           sub.devices};
      });
}
