#pragma once

// Shared runner for talOS loop-node binaries: the internals of talos::Run
// (talOS/node), and the main of the one node not written on talos::Node
// (2026-robot telemetry, whose Studio feed bypasses the loop).
//
// Every node main parses the same flags (--sim/--describe/--log/--session-id/
// --config/--replay/--duration-s), then takes one of three branches: describe
// (build the node on a simulated loop and print its manifest), replay (drive
// the node from a log), or realtime (drive it live with a Reporter and a stop
// handler). The node is built by a factory invoked once per branch:
//
//   auto factory = [&](auto& loop, const talos::process::NodeFlags& flags) {
//     return MyNode{loop, config::ParseRobotConfig(flags.config_path)};
//   };
//
// The remaining per-node differences are hooks with no-op defaults:
//   - Starter: what starts the node once built (talos::Run starts inside the
//     factory; telemetry arms its timer one period out).
//   - DescribeHook: extra manifest entries for --describe (telemetry's Studio
//     feed, which bypasses the loop and so is absent from the manifest).
//   - ExtraHook: extra live-registry sources for the Reporter (same feed).
//   - Summary: the exit line (DefaultSummary prints dispatches/recording
//     state; telemetry prints its frame counters instead).
//   - EndpointHook: endpoint attributes known only once the node is built
//     (talos::Node's optional subscriptions), for both --describe and the
//     Reporter.
//
// Construction order inside the realtime branch is load-bearing and preserved
// here: the node starts, stop handlers install, then the Reporter is declared
// after the loop so reverse destruction order joins its thread before the
// counters it samples are destroyed. The stopper thread polls
// talos::process::stop_requested, which is why every node must use
// InstallStopHandlers.
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
// node construction (config loading) can use them.
struct NodeFlags {
  std::string log_path;
  std::string replay_path;
  std::string config_path;
  int duration_s = 0;
  std::uint64_t session_id = 0;
  bool simulation = false;
  bool describe = false;
};

// Per-node identity: the registry row and --describe name this node and
// target, and the log stream and default /tmp/<name>.tlog derive from the
// name.
struct NodeSpec {
  const char* node_name = nullptr;
  const char* node_target = nullptr;
};

inline NodeFlags ParseNodeFlags(int argc, char** argv, const NodeSpec& spec) {
  NodeFlags flags;
  flags.log_path = std::string{"/tmp/"} + spec.node_name + ".tlog";
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
      flags.config_path = argv[++i];
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
  if (flags.config_path.empty())
    throw std::invalid_argument("missing --config PATH");
  return flags;
}

// Default starter: the factory left the node ready to run.
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

// Default endpoint hook: no endpoint attributes.
struct NoEndpointHook {
  template <typename Node>
  std::vector<introspect::EndpointAttribute> operator()(Node&) const {
    return {};
  }
};

template <
    typename Factory, typename Starter = NoStart,
    typename DescribeHook = NoDescribeHook, typename ExtraHook = NoExtraSources,
    typename Summary = DefaultSummary, typename EndpointHook = NoEndpointHook>
int RunNode(int argc, char** argv, NodeSpec spec, Factory factory,
            Starter starter = {}, DescribeHook describe_hook = {},
            ExtraHook extra_hook = {}, Summary summary = {},
            EndpointHook endpoint_hook = {}) {
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
      auto description =
          introspect::DescribeManifest(spec.node_name, spec.node_target,
                                       loop.manifest(), endpoint_hook(node));
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
    event::RealtimeEventLoop<event::log::LogWriter> loop{event::log::LogWriter{
        flags.log_path, spec.node_name, event::log::LogWriterOptions{},
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
                                   .endpoints = endpoint_hook(node)}};

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
