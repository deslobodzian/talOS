#pragma once

/*
 * Publishes one node's shape and activity into the live registry.
 *
 * Deliberately not an event source. Registering a timer would put introspection
 * into the manifest and into the log, which means every recording made with it
 * on would refuse to replay against a build with it off, and every node's
 * source ids would shift. Observability must not change the program being
 * observed. So this runs on its own thread and reads only the loop's atomics
 * plus its manifest, which is immutable once the run has started.
 *
 * Construct it after the loop, so that reverse destruction order joins this
 * thread before the loop's counters go away:
 *
 *   RealtimeEventLoop<LogWriter> loop{...};
 *   OdometryNode node{loop};
 *   introspect::Reporter reporter{loop, {.name = "odometry"}};
 *   loop.run();
 *
 * A registry that cannot be opened is reported once and then ignored. A node
 * that refuses to run because a telemetry viewer might not see it would be a
 * worse failure than not being seen.
 */

#include <atomic>
#include <chrono>
#include <cstdio>
#include <functional>
#include <optional>
#include <stop_token>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "talOS/events/manifest.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/registry.h"

namespace talos::introspect {

class Reporter {
 public:
  // A topic the node owns directly rather than through the event loop.
  //
  // Not every transport belongs in a manifest. The telemetry node writes
  // variable-length FlatBuffers into a 64 KiB slot, which a loop sender cannot
  // express -- and registering one would copy every frame into the dispatch
  // log. But that topic is the whole point of the node, and a graph that omits
  // it reports the telemetry feed as published by nobody. So the node declares
  // it here, and supplies its own counters.
  struct ExtraSource {
    event::SourceKind kind{event::SourceKind::SENDER};
    std::string name;
    std::uint32_t message_bytes{0};
    std::uint32_t alignment{0};

    // naming::kSourceFlagExternal / kSourceFlagOptional, declared here rather
    // than in `Options::endpoints` because an extra source is already a
    // by-value declaration of one topic: the node is spelling the name out on
    // this line, so the attribute belongs on the same line.
    std::uint32_t flags{0};

    // Cumulative, read once per refresh. Must be safe to call from another
    // thread: return a copy of an atomic or a plain counter, never anything
    // that takes a lock the loop might hold.
    std::function<std::uint64_t()> events;
    std::function<std::uint64_t()> dropped;
  };

  struct Options {
    std::string name;
    std::string target;
    std::uint64_t session_id{0};
    bool simulation{false};
    bool replay{false};

    std::vector<ExtraSource> extra;

    // How the far end of a topic behaves, keyed by topic name, for the loop's
    // own sources. The loop cannot tell: a sender whose consumer is the RoboRIO
    // over UDP is registered exactly like one whose consumer is another node,
    // and only the node knows the difference. Without this, the system graph
    // reports every deliberate dead end as a fault, which is how a report stops
    // being read.
    //
    // The `topic` views are not copied, so they must outlive the Reporter --
    // string literals at a node's main, which is where these are written.
    std::vector<EndpointAttribute> endpoints;

    // How often the slot is refreshed. Four times a second is well inside the
    // liveness timeout and far below any rate a human or an agent polls at.
    std::chrono::milliseconds period{250};

    std::string registry_path{kRegistryPath};
  };

  template <typename Loop>
  Reporter(Loop& loop, Options options) : options_{std::move(options)} {
    // Captured by pointer rather than reference so the lambda stays copyable
    // and the intent -- this thread outlives no part of the loop -- stays
    // visible at the call site.
    thread_ =
        std::jthread{[this, &loop](std::stop_token stop) { run(loop, stop); }};
  }

  ~Reporter() {
    thread_.request_stop();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  Reporter(const Reporter&) = delete;
  Reporter& operator=(const Reporter&) = delete;

  // True once the slot has been claimed and the topology written. Tests wait
  // on this rather than sleeping.
  bool published() const { return published_.load(std::memory_order_acquire); }

  const std::string& error() const { return error_; }

 private:
  template <typename Loop>
  void run(Loop& loop, std::stop_token stop) {
    // The loop publishes its counters array before it stores running(), and
    // that store synchronizes with this load, so a visible `true` means the
    // manifest is frozen and the counters exist.
    while (!stop.stop_requested() && !loop.running()) {
      std::this_thread::sleep_for(std::chrono::milliseconds{2});
    }
    if (stop.stop_requested()) {
      return;
    }

    std::optional<NodeRegistration> registration;
    try {
      registration.emplace(
          NodeRegistration::Identity{
              .name = options_.name,
              .target = options_.target,
              .session_id = options_.session_id,
              .flags = static_cast<std::uint32_t>(
                  (options_.simulation ? kFlagSimulation : 0u) |
                  (options_.replay ? kFlagReplay : 0u)),
          },
          options_.registry_path);
    } catch (const std::exception& e) {
      error_ = e.what();
      std::fprintf(stderr, "%s: node registry unavailable: %s\n",
                   options_.name.c_str(), e.what());
      return;
    }

    // The loop's own sources first, keeping their ids, then anything the node
    // owns outside the loop.
    event::Manifest manifest = loop.manifest();
    loop_sources_ = manifest.size();
    for (const ExtraSource& extra : options_.extra) {
      manifest.push_back({
          .id = static_cast<std::uint16_t>(manifest.size()),
          .kind = extra.kind,
          .name = extra.name,
          .message_bytes = extra.message_bytes,
          .alignment = extra.alignment,
      });
    }

    // Attributes from both places a node can declare one: by topic name for
    // the loop's sources, and inline on the extra sources.
    std::vector<EndpointAttribute> endpoints = options_.endpoints;
    for (const ExtraSource& extra : options_.extra) {
      if (extra.flags != 0) {
        endpoints.push_back({extra.name, extra.flags});
      }
    }

    registration->publish(manifest, endpoints);
    published_.store(true, std::memory_order_release);

    while (!stop.stop_requested() && loop.running()) {
      sample(loop, *registration);
      // Short sleeps rather than one long one, so shutdown is prompt: a node
      // that has exited should leave the viewer immediately, not a quarter
      // second later.
      const auto deadline = std::chrono::steady_clock::now() + options_.period;
      while (!stop.stop_requested() && loop.running() &&
             std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds{10});
      }
    }

    // A final sample so the last cycle's counts are not lost when a node exits
    // between refreshes.
    sample(loop, *registration);
  }

  template <typename Loop>
  void sample(Loop& loop, NodeRegistration& registration) {
    // One sample: the heartbeat, the dispatch count and every source move
    // together behind sample_seq, so a reader never pairs a fresh heartbeat
    // with the previous refresh's counts.
    registration.begin_sample();
    registration.heartbeat();
    registration.set_dispatch_count(loop.dispatch_count());

    const event::SourceCounters* counters = loop.source_counters();
    const std::size_t count = loop.source_counter_count();
    for (std::size_t i = 0; i < count; ++i) {
      const event::SourceCounters& source = counters[i];
      registration.set_source(
          static_cast<std::uint32_t>(i),
          source.events.load(std::memory_order_relaxed),
          source.dropped.load(std::memory_order_relaxed),
          source.sequence.load(std::memory_order_relaxed),
          source.last_now_ns.load(std::memory_order_relaxed),
          source.last_latency_ns.load(std::memory_order_relaxed),
          source.max_latency_ns.load(std::memory_order_relaxed));
    }

    for (std::size_t i = 0; i < options_.extra.size(); ++i) {
      const ExtraSource& extra = options_.extra[i];
      const std::uint64_t events = extra.events ? extra.events() : 0;
      registration.set_source(static_cast<std::uint32_t>(loop_sources_ + i),
                              events, extra.dropped ? extra.dropped() : 0,
                              events,
                              /*last_monotonic_ns=*/0, /*last_latency_ns=*/0,
                              /*max_latency_ns=*/0);
    }
    registration.end_sample();
  }

  Options options_;
  std::size_t loop_sources_{0};
  std::atomic<bool> published_{false};
  std::string error_;
  std::jthread thread_;
};

}  // namespace talos::introspect
