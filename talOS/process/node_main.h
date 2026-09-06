#pragma once

// Shared runner for talOS loop-node binaries.
//
// Every node main parses the same flags (--sim/--describe/--log/--session-id/
// --config/--replay/--duration-s), then takes one of three branches: describe
// (build the node on a simulated loop and print its manifest), replay (drive
// the node from a log), or realtime (drive it live with a Reporter and a stop
// handler). Only node construction varies -- some nodes build from the loop
// alone, others also take config, geometry, or doubles -- so the runner is
// parameterized by a factory lambda invoked once per branch:
//
//   auto factory = [&](auto& loop, const talos::process::NodeFlags& flags) {
//     return talos::drive::DrivetrainNode{loop, config, geometry, devices,
//                                         flags.simulation};
//   };
//
// The remaining per-node differences are secondary hooks with defaults:
//   - Starter: how the node is started (StartNow by default; NoStart for nodes
//     with no Start call; a lambda adding a period offset otherwise).
//   - DescribeHook: extra manifest entries for --describe (telemetry's Studio
//     feed, which bypasses the loop and so is absent from the manifest).
//   - ExtraHook: extra live-registry sources for the Reporter (same feed).
//   - Summary: the exit line (DefaultSummary prints dispatches/recording
//     state; telemetry prints its frame counters instead).
//
// Construction order inside the realtime branch is load-bearing and preserved
// here: the node starts, stop handlers install, then the Reporter is declared
// after the loop so reverse destruction order joins its thread before the
// counters it samples are destroyed. The stopper thread polls
// talos::process::stop_requested, which is why every node -- including ones
// that once defined their own stop flag -- must use InstallStopHandlers.
//
// talOS/bridge and talOS/launcher mains are intentionally excluded: they are
// not loop nodes and share none of this shape.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <stdexcept>
#include <stop_token>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/realtime_event_loop.h"
#include "talOS/events/replay_event_loop.h"
#include "talOS/events/simulated_event_loop.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/reporter.h"
#include "talOS/process/process.h"

namespace talos::process {

// Flags every node binary accepts. Parsed once, then handed to the factory so
// node construction (config loading, geometry, doubles) can use them.
struct NodeFlags {
  std::string log_path;
  std::string replay_path;
  std::string config_path;
  int duration_s = 0;
  std::uint64_t session_id = 0;
  bool simulation = false;
  bool describe = false;
};

// Per-node identity. Endpoints are copied, not referenced, but the underlying
// topic views must outlive the run -- string literals and topic constants at
// the node's main, which is where these are written.
struct NodeSpec {
  const char* node_name = nullptr;
  const char* node_target = nullptr;
  const char* log_stream = nullptr;
  const char* default_log_path = nullptr;
  const char* default_config_path =
      "2026-robot/main_processor/configuration/robot.toml";
  std::vector<introspect::EndpointAttribute> endpoints;
  // Nodes that read neither hardware nor configuration accept --config and
  // discard it, so the launcher can pass one flag set to every binary.
  bool ignore_config_value = false;
};

inline NodeFlags ParseNodeFlags(int argc, char** argv, const NodeSpec& spec) {
  NodeFlags flags;
  flags.log_path = spec.default_log_path;
  flags.config_path = spec.default_config_path;
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--sim") {
      flags.simulation = true;
      continue;
    } else if (arg == "--describe") {
      flags.describe = true;
      continue;
    } else if (arg == "--log" && i + 1 < argc) {
      flags.log_path = argv[++i];
    } else if (arg == "--session-id" && i + 1 < argc) {
      flags.session_id = std::stoull(argv[++i]);
    } else if (arg == "--config" && i + 1 < argc) {
      if (spec.ignore_config_value) {
        ++i;
      } else {
        flags.config_path = argv[++i];
      }
    } else if (arg == "--replay" && i + 1 < argc) {
      flags.replay_path = argv[++i];
    } else if (arg == "--duration-s" && i + 1 < argc) {
      flags.duration_s = std::stoi(argv[++i]);
    } else {
      throw std::invalid_argument(
          "usage: node [--sim] [--describe] [--duration-s N] [--log PATH] "
          "[--session-id ID] [--config PATH] [--replay PATH]");
    }
  }
  if (flags.duration_s < 0)
    throw std::invalid_argument("duration must be nonnegative");
  return flags;
}

// Starts the node at the loop's current time.
struct StartNow {
  template <typename Node, typename Loop>
  void operator()(Node& node, Loop& loop) const {
    node.Start(loop.monotonic_now());
  }
};

// For nodes with no Start call (odometry): construction is the whole setup.
struct NoStart {
  template <typename Node, typename Loop>
  void operator()(Node&, Loop&) const {}
};

// Default --describe hook: the manifest is the whole description.
struct NoDescribeHook {
  template <typename Node>
  void operator()(introspect::Description&, Node&) const {}
};

// Default Reporter hook: every source comes from the loop manifest.
struct NoExtraSources {
  template <typename Node>
  std::vector<introspect::Reporter::ExtraSource> operator()(Node&) const {
    return {};
  }
};

// Default exit line: dispatches plus the recorder state.
struct DefaultSummary {};

template <typename Factory, typename Starter = StartNow,
          typename DescribeHook = NoDescribeHook,
          typename ExtraHook = NoExtraSources, typename Summary = DefaultSummary>
int RunNode(int argc, char** argv, NodeSpec spec, Factory factory,
            Starter starter = {}, DescribeHook describe_hook = {},
            ExtraHook extra_hook = {}, Summary summary = {}) {
  try {
    const NodeFlags flags = ParseNodeFlags(argc, argv, spec);
    // --describe builds the node for real and prints what its constructor
    // registered. The loop is the only difference: a simulated loop's channels
    // live in this process, so no shared memory, no hardware and no network is
    // touched, and the launcher can ask what a node is without starting a
    // robot. Describing from the same constructor is the point -- a topology
    // written down a second time is a topology to forget to update.
    if (flags.describe) {
      event::SimulationEnvironment environment;
      event::SimulatedEventLoop<> loop{environment};
      auto node = factory(loop, flags);
      auto description = introspect::DescribeManifest(
          spec.node_name, spec.node_target, loop.manifest(), spec.endpoints);
      describe_hook(description, node);
      std::printf("%s", introspect::DescribeToJson(description).c_str());
      return 0;
    }
    if (!flags.replay_path.empty()) {
      event::log::LogReader reader{flags.replay_path};
      event::ReplayEventLoop<> loop{reader};
      auto node = factory(loop, flags);
      starter(node, loop);
      loop.run();
      std::printf("replay: diverged=%d clean_exit=%d\n", loop.diverged(),
                  loop.reached_exit());
      return loop.diverged() || !loop.reached_exit() ? 1 : 0;
    }
    event::RealtimeEventLoop<event::log::LogWriter> loop{
        event::log::LogWriter{flags.log_path, spec.log_stream,
                              event::log::LogWriterOptions{},
                              flags.session_id}};
    auto node = factory(loop, flags);
    starter(node, loop);
    InstallStopHandlers();
    // Publishes this node's topics and activity into the live registry, so
    // Studio and any agent can see what is running without being told.
    // Declared after the loop: reverse destruction order joins its thread
    // before the counters it samples are destroyed.
    introspect::Reporter reporter{loop,
                                  {.name = spec.node_name,
                                   .target = spec.node_target,
                                   .session_id = flags.session_id,
                                   .simulation = flags.simulation,
                                   .extra = extra_hook(node),
                                   .endpoints = spec.endpoints}};

    std::jthread stopper{[&](std::stop_token stop) {
      while (!stop.stop_requested()) {
        if (stop_requested.load() && loop.running()) {
          loop.exit();
          return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds{20});
      }
    }};
    if (flags.duration_s)
      loop.run_for(std::chrono::seconds{flags.duration_s});
    else
      loop.run();
    stopper.request_stop();
    if constexpr (std::is_same_v<Summary, DefaultSummary>) {
      std::printf("%s: dispatches=%llu recording_failed=%d\n", spec.node_name,
                  static_cast<unsigned long long>(loop.dispatch_count()),
                  loop.recorder().failed());
    } else {
      summary(node, loop);
    }
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

}  // namespace talos::process
