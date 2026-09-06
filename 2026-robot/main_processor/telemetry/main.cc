#include <cstdio>
#include <stdexcept>
#include <string>
#include <thread>

#include "2026-robot/main_processor/telemetry/node.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/realtime_event_loop.h"
#include "talOS/events/replay_event_loop.h"
#include "talOS/events/simulated_event_loop.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/names.h"
#include "talOS/introspection/reporter.h"
#include "talOS/process/process.h"

namespace {
// This node's identity, written once. `--describe` and the live registry row
// have to name the same node and the same build target, or a viewer cannot tell
// a graph that changed from a graph it is reading two different names for.
constexpr const char* kNodeName = "telemetry";
constexpr const char* kNodeTarget =
    "//2026-robot/main_processor/telemetry:node";
}  // namespace

int main(int argc, char** argv) {
  try {
    std::string log_path = "/tmp/telemetry.tlog", replay_path;
    std::string config_path =
        "2026-robot/main_processor/configuration/robot.toml";
    int duration_s = 0;
    uint64_t session_id = 0;
    bool simulation = false;
    bool describe = false;
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      // --sim only gates hardware commissioning, and this node owns no
      // hardware, so the launcher's flag changes nothing here beyond telling
      // the registry which mode the session is running in.
      if (arg == "--sim") {
        simulation = true;
        continue;
      } else if (arg == "--describe") {
        describe = true;
        continue;
      } else if (arg == "--log" && i + 1 < argc)
        log_path = argv[++i];
      else if (arg == "--session-id" && i + 1 < argc)
        session_id = std::stoull(argv[++i]);
      else if (arg == "--config" && i + 1 < argc)
        config_path = argv[++i];
      else if (arg == "--replay" && i + 1 < argc)
        replay_path = argv[++i];
      else if (arg == "--duration-s" && i + 1 < argc)
        duration_s = std::stoi(argv[++i]);
      else
        throw std::invalid_argument(
            "usage: node [--sim] [--describe] [--duration-s N] [--log PATH] "
            "[--session-id ID] [--config PATH] [--replay PATH]");
    }
    if (duration_s < 0)
      throw std::invalid_argument("duration must be nonnegative");
    const auto robot_config = talos::config::ParseRobotConfig(config_path);
    talos::telemetry::TelemetryConfig telemetry{};
    if (const auto* tbl = robot_config.GetSubsystemTable("telemetry")) {
      telemetry.period_us = (*tbl)["period_us"].value_or(telemetry.period_us);
      telemetry.topic = (*tbl)["topic"].value_or(telemetry.topic);
    }
    // --describe builds the node for real and prints what its constructor
    // registered. The loop is the only difference: a simulated loop's channels
    // live in this process, so no shared memory, no hardware and no network is
    // touched, and the launcher can ask what a node is without starting a
    // robot. Describing from the same constructor is the point -- a topology
    // written down a second time is a topology to forget to update.
    if (describe) {
      talos::event::SimulationEnvironment environment;
      talos::event::SimulatedEventLoop<> loop{environment};
      talos::telemetry::TelemetryNode node{loop, telemetry};
      auto description = talos::introspect::DescribeManifest(
          kNodeName, kNodeTarget, loop.manifest());
      // The Studio feed goes straight to RTMS rather than through a loop
      // sender, so it is not in the manifest -- and it is the whole point of
      // this node. The Reporter below declares it into the live registry for
      // the same reason: a graph missing it reports the feed as published by
      // nobody, and the declared graph would then disagree with the running one
      // about a topic that is fine.
      description.sources.push_back(
          {talos::event::SourceKind::SENDER, telemetry.topic,
           static_cast<std::uint32_t>(studio::slot_bytes),
           talos::introspect::naming::kSourceFlagExternal});
      std::printf("%s", talos::introspect::DescribeToJson(description).c_str());
      return 0;
    }
    if (!replay_path.empty()) {
      talos::event::log::LogReader reader{replay_path};
      talos::event::ReplayEventLoop<> loop{reader};
      talos::telemetry::TelemetryNode node{loop, telemetry};
      node.Start(loop.monotonic_now() +
                 std::chrono::microseconds{telemetry.period_us});
      loop.run();
      std::printf("replay: diverged=%d clean_exit=%d\n", loop.diverged(),
                  loop.reached_exit());
      return loop.diverged() || !loop.reached_exit() ? 1 : 0;
    }
    talos::event::RealtimeEventLoop<talos::event::log::LogWriter> loop{
        talos::event::log::LogWriter{log_path, "telemetry",
                                     talos::event::log::LogWriterOptions{},
                                     session_id}};
    talos::telemetry::TelemetryNode node{loop, telemetry};
    node.Start(loop.monotonic_now() +
               std::chrono::microseconds{telemetry.period_us});
    talos::process::InstallStopHandlers();
    // Publishes this node's topics and activity into the live registry, so
    // Studio and any agent can see what is running without being told.
    // Declared after the loop: reverse destruction order joins its thread
    // before the counters it samples are destroyed.
    //
    // The Studio topic is written straight to RTMS rather than through a loop
    // sender, so it has to be declared explicitly: otherwise the system graph
    // reports the telemetry feed as a topic nobody publishes.
    talos::introspect::Reporter reporter{
        loop,
        {.name = kNodeName,
         .target = kNodeTarget,
         .session_id = session_id,
         .simulation = simulation,
         .extra = {{
             .kind = talos::event::SourceKind::SENDER,
             .name = telemetry.topic,
             .message_bytes = static_cast<std::uint32_t>(studio::slot_bytes),
             .alignment = 8,
             // External: the consumer is Studio's bridge, which is not a node
             // in the session. No node will ever subscribe, so a graph that
             // called this an unread feed would say so on every single launch,
             // and a warning that can never be fixed is one people stop
             // reading.
             .flags = talos::introspect::naming::kSourceFlagExternal,
             .events = [&node] { return node.frames_published(); },
             .dropped = [&node] { return node.frames_dropped(); },
         }}}};

    std::jthread stopper{[&](std::stop_token stop) {
      while (!stop.stop_requested()) {
        if (talos::process::stop_requested.load() && loop.running()) {
          loop.exit();
          return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{20});
      }
    }};
    if (duration_s)
      loop.run_for(std::chrono::seconds{duration_s});
    else
      loop.run();
    stopper.request_stop();
    std::printf(
        "telemetry: dispatches=%llu published=%llu dropped=%llu rejected=%llu "
        "recording_failed=%d\n",
        static_cast<unsigned long long>(loop.dispatch_count()),
        static_cast<unsigned long long>(node.frames_published()),
        static_cast<unsigned long long>(node.frames_dropped()),
        static_cast<unsigned long long>(node.frames_rejected()),
        loop.recorder().failed());
    if (loop.recorder().failed()) {
      std::fprintf(stderr, "%s\n", loop.recorder().error().c_str());
      return 1;
    }
    return 0;
  } catch (const std::exception& e) {
    std::fprintf(stderr, "%s\n", e.what());
    return 1;
  }
}
