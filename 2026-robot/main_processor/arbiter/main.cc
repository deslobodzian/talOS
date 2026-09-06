#include "2026-robot/main_processor/arbiter/node.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/introspection/names.h"
#include "talOS/process/node_main.h"

namespace {
// This node's identity, written once. `--describe` and the live registry row
// have to name the same node and the same build target, or a viewer cannot tell
// a graph that changed from a graph it is reading two different names for.
constexpr const char* kNodeName = "arbiter";
constexpr const char* kNodeTarget = "//2026-robot/main_processor/arbiter:node";

// The two autonomous lanes have no publisher, and will not until autonomous is
// written. That is this node's design, not a broken link, so it says so: a
// graph report that flags a known gap on every launch is a report people learn
// to skip, and then it cannot tell them about the link that really is broken.
const std::vector<talos::introspect::EndpointAttribute> kEndpoints = {
    {talos::arbiter::kAutoChassisTopic,
     talos::introspect::naming::kSourceFlagOptional},
    {talos::arbiter::kAutoShooterTopic,
     talos::introspect::naming::kSourceFlagOptional},
    {talos::arbiter::kAutoIntakeTopic,
     talos::introspect::naming::kSourceFlagOptional},
};
}  // namespace

int main(int argc, char** argv) {
  // --sim only gates hardware commissioning, and this node owns no hardware,
  // so the launcher's flag changes nothing here beyond telling the registry
  // which mode the session is running in.
  auto factory = [&](auto& loop, const talos::process::NodeFlags& flags) {
    // Parsed only to fail early on a broken configuration; this node reads
    // no hardware and has no policy of its own.
    (void)talos::config::ParseRobotConfig(flags.config_path);
    return talos::arbiter::ArbiterNode{loop};
  };
  return talos::process::RunNode(
      argc, argv,
      {.node_name = kNodeName,
       .node_target = kNodeTarget,
       .log_stream = "arbiter",
       .default_log_path = "/tmp/arbiter.tlog",
       .endpoints = kEndpoints},
      factory);
}
