#include <chrono>
#include <cstdint>
#include <cstdio>

#include "2026-robot/main_processor/telemetry/node.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/introspection/names.h"
#include "talOS/process/node_main.h"

namespace {
// This node's identity, written once. `--describe` and the live registry row
// have to name the same node and the same build target, or a viewer cannot tell
// a graph that changed from a graph it is reading two different names for.
constexpr const char* kNodeName = "telemetry";
constexpr const char* kNodeTarget =
    "//2026-robot/main_processor/telemetry:node";
}  // namespace

int main(int argc, char** argv) {
  // --sim only gates hardware commissioning, and this node owns no hardware,
  // so the launcher's flag changes nothing here beyond telling the registry
  // which mode the session is running in.
  //
  // The config, read by the factory. Exactly one runner branch executes per
  // process, so the hooks below always observe the value the factory stored.
  talos::telemetry::TelemetryConfig telemetry{};
  auto factory = [&](auto& loop, const talos::process::NodeFlags& flags) {
    const auto robot_config =
        talos::config::ParseRobotConfig(flags.config_path);
    if (const auto* tbl = robot_config.GetSubsystemTable("telemetry")) {
      telemetry.period_us =
          (*tbl)["period_us"].value_or(telemetry.period_us);
      telemetry.topic = (*tbl)["topic"].value_or(telemetry.topic);
    }
    return talos::telemetry::TelemetryNode{loop, telemetry};
  };
  auto starter = [&](auto& node, auto& loop) {
    node.Start(loop.monotonic_now() +
               std::chrono::microseconds{telemetry.period_us});
  };
  // The Studio feed goes straight to RTMS rather than through a loop sender,
  // so it is not in the manifest -- and it is the whole point of this node.
  // The Reporter below declares it into the live registry for the same
  // reason: a graph missing it reports the feed as published by nobody, and
  // the declared graph would then disagree with the running one about a topic
  // that is fine.
  auto describe_hook = [&](talos::introspect::Description& description,
                           auto&) {
    description.sources.push_back(
        {talos::event::SourceKind::SENDER, telemetry.topic,
         static_cast<std::uint32_t>(studio::slot_bytes),
         talos::introspect::naming::kSourceFlagExternal});
  };
  // The Studio topic is written straight to RTMS rather than through a loop
  // sender, so it has to be declared explicitly: otherwise the system graph
  // reports the telemetry feed as a topic nobody publishes.
  auto extra_hook = [&](auto& node) {
    return std::vector<talos::introspect::Reporter::ExtraSource>{{
        .kind = talos::event::SourceKind::SENDER,
        .name = telemetry.topic,
        .message_bytes = static_cast<std::uint32_t>(studio::slot_bytes),
        .alignment = 8,
        // External: the consumer is Studio's bridge, which is not a node in
        // the session. No node will ever subscribe, so a graph that called
        // this an unread feed would say so on every single launch, and a
        // warning that can never be fixed is one people stop reading.
        .flags = talos::introspect::naming::kSourceFlagExternal,
        .events = [&node] { return node.frames_published(); },
        .dropped = [&node] { return node.frames_dropped(); },
    }};
  };
  auto summary = [&](auto& node, auto& loop) {
    std::printf(
        "telemetry: dispatches=%llu published=%llu dropped=%llu rejected=%llu "
        "recording_failed=%d\n",
        static_cast<unsigned long long>(loop.dispatch_count()),
        static_cast<unsigned long long>(node.frames_published()),
        static_cast<unsigned long long>(node.frames_dropped()),
        static_cast<unsigned long long>(node.frames_rejected()),
        loop.recorder().failed());
  };
  return talos::process::RunNode(
      argc, argv,
      {.node_name = kNodeName,
       .node_target = kNodeTarget,
       .log_stream = "telemetry",
       .default_log_path = "/tmp/telemetry.tlog"},
      factory, starter, describe_hook, extra_hook, summary);
}
