#pragma once

/*
 * Publishes one node's shape and activity into the live registry.
 *
 * Deliberately not an event source. Registering a timer would put introspection
 * into the manifest and into the log, which means every recording made with it
 * on would refuse to replay against a build with it off, and every node's
 * source ids would shift. Observability must not change the program being
 * observed. So this runs on its own thread and reads only the loop's atomics
 * -- its counters, and its recorder's capture state -- plus its manifest and
 * which watchers are latest-only, both immutable once the run has started.
 *
 * The one exception is the row's dispatch count, the launcher's evidence that
 * the node is not stalled. The loop thread stores it at every dispatch
 * (event::DispatchObserver): this thread runs below every realtime loop, and
 * a host busy with them can starve it for longer than the launcher's stall
 * timeout while the loop itself keeps time.
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
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <mutex>
#include <optional>
#include <stop_token>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "talOS/events/dispatch_observer.h"
#include "talOS/events/manifest.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/registry.h"

namespace talos::introspect {

// How often a running node's reporter refreshes its row: the heartbeat, the
// source counters, the selections and the event log flags. Once a loop
// runs, the one deadline the heartbeat must meet is liveness: a viewer's
// `alive`, and the next claim freeing a row whose heartbeat is
// kLivenessTimeout old. The launcher's stall check reads the dispatch count,
// which the loop thread stores itself, and counts the heartbeat as progress
// only while a node constructs, when the reporter beats on a cadence of its
// own (Reporter's constructor). So the refresh owes the heartbeat only a wide
// margin under kLivenessTimeout, for a reporting thread starved below the
// realtime loops: 20 refreshes per timeout. Each refresh is the reporting
// thread's only wake while the loop runs; nothing else it writes has a
// deadline (Studio's view and the launcher's recording and realtime notes
// read it as it comes).
inline constexpr std::chrono::milliseconds kRefreshPeriod{250};
static_assert(kRefreshPeriod * 20 <= kLivenessTimeout,
              "a starved reporter must not lose its row");

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

  // One of the node's Targets (talOS/devices/target.h): who drives the
  // subsystem, which no manifest or counter says. The node selects a lane on
  // every get(); this reads the latest selection once per refresh.
  struct Selection {
    // The policy's sources, in TargetPolicy::Sources() order.
    std::vector<std::string> sources;

    // Index into `sources` of the lane the Target selected last, or -1 for
    // none. Called from the reporting thread, so the same rule as
    // ExtraSource::events: a copy of an atomic, never anything that locks.
    std::function<std::int32_t()> selected;
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

    // The node's Targets, in the order it opened them.
    std::vector<Selection> selections;

    // Full refresh cadence: heartbeat, source counters and selections. The
    // dispatch count the launcher supervises is the loop thread's own.
    std::chrono::milliseconds period{kRefreshPeriod};

    std::string registry_path{kRegistryPath};
  };

  explicit Reporter(Options options) : options_{std::move(options)} {
    try {
      registration_.emplace(
          NodeRegistration::Identity{
              .name = options_.name,
              .target = options_.target,
              .session_id = options_.session_id,
              .flags = static_cast<std::uint32_t>(
                  kFlagConstructing |
                  (options_.simulation ? kFlagSimulation : 0u) |
                  (options_.replay ? kFlagReplay : 0u))},
          options_.registry_path);
    } catch (const std::exception& e) {
      error_ = e.what();
      std::fprintf(stderr, "%s: node registry unavailable: %s\n",
                   options_.name.c_str(), e.what());
    }
    // The construction heartbeat: while a node constructs, the launcher's
    // stall check reads it as progress, against a timeout of at least
    // 500 ms. Its 10 ms cadence lasts only as long as construction, which
    // the launcher bounds (kConstructionTimeout); start() ends it.
    thread_ = std::jthread{[this](const std::stop_token& stop) {
      while (!stop.stop_requested()) {
        if (registration_) registration_->heartbeat();
        pause(stop, std::chrono::milliseconds{10});
      }
    }};
  }

  template <typename Loop>
  Reporter(Loop& loop, Options options) : Reporter(std::move(options)) {
    start(loop);
  }

  // Call after construction, before running the loop. The owner must keep the
  // loop and every sampled node alive until this Reporter has been destroyed,
  // and destroy it only while the loop is not running: from then on the loop
  // stores its dispatch count in this Reporter's row.
  template <typename Loop>
  void start(Loop& loop, std::vector<ExtraSource> extra,
             std::vector<EndpointAttribute> endpoints,
             std::vector<Selection> selections) {
    stop();
    options_.extra = std::move(extra);
    options_.endpoints = std::move(endpoints);
    options_.selections = std::move(selections);
    start(loop);
  }

  template <typename Loop>
  void start(Loop& loop) {
    stop();
    detach();
    if (registration_) {
      progress_.registration = &*registration_;
      loop.set_dispatch_observer(&progress_);
      detach_ = [&loop] { loop.set_dispatch_observer(nullptr); };
    }
    thread_ = std::jthread{
        [this, &loop](const std::stop_token& stop) { run(loop, stop); }};
  }

  ~Reporter() {
    stop();
    detach();
  }

  void stop() {
    thread_.request_stop();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  Reporter(const Reporter&) = delete;
  Reporter& operator=(const Reporter&) = delete;

  // Publishes the loop's realtime outcome in the row
  // (NodeRegistration::set_realtime): call once, from the loop thread, right
  // after process::ConfigureRealtime, or with requested false when the node
  // did not ask. Safe while the reporting thread runs.
  void set_realtime(const process::RealtimeOutcome& outcome) {
    if (registration_) registration_->set_realtime(outcome);
  }

  // True once the slot has been claimed and the topology written. Tests wait
  // on this rather than sleeping.
  bool published() const { return published_.load(std::memory_order_acquire); }

  // How many times the reporting thread has woken from its waits, for
  // measuring what reporting costs a node: once per `period` while its loop
  // runs.
  std::uint64_t wakeups() const {
    return wakeups_.load(std::memory_order_relaxed);
  }

  const std::string& error() const { return error_; }

 private:
  template <typename Loop>
  void run(Loop& loop, const std::stop_token& stop) {
    // The loop publishes its counters array before it stores running(), and
    // that store synchronizes with this load, so a visible `true` means the
    // manifest is frozen and the counters exist. The 2 ms polls last only
    // from start() to the run, a few calls in talos::Run.
    while (!stop.stop_requested() && !loop.running()) {
      pause(stop, std::chrono::milliseconds{2});
      if (registration_) registration_->heartbeat();
    }
    if (stop.stop_requested()) {
      return;
    }

    if (!registration_) return;
    NodeRegistration& registration = *registration_;

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

    // What only the loop knows about its own sources, by id: which watchers
    // skip to the newest message, so a viewer does not read their skips as
    // loss.
    registration.publish(manifest, endpoints, LoopSourceFlags(loop));
    registration.publish_selections(options_.selections);
    registration.activate();
    published_.store(true, std::memory_order_release);

    // One wait per refresh. The end of the loop is not polled for: the row
    // stays until this Reporter goes, which talos::Run does right after its
    // loop returns, and stop() wakes the wait at once.
    while (!stop.stop_requested() && loop.running()) {
      sample(loop, registration);
      pause(stop, options_.period);
    }

    // A final sample so the last cycle's counts are not lost when a node exits
    // between refreshes.
    sample(loop, registration);
  }

  template <typename Loop>
  void sample(Loop& loop, NodeRegistration& registration) {
    // One full sample. The dispatch count is the loop thread's to store
    // (ProgressObserver).
    registration.begin_sample();
    registration.heartbeat();
    // Whether the event log stopped early, from a recorder that can say
    // (event::log::LogWriter, whose answers are lock-free loads); an
    // unlogged loop's NullRecorder cannot.
    if constexpr (requires {
                    loop.recorder().failed();
                    loop.recorder().capped();
                  }) {
      registration.set_recording(loop.recorder().failed(),
                                 loop.recorder().capped());
    }

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

    for (std::size_t i = 0; i < options_.selections.size(); ++i) {
      const Selection& selection = options_.selections[i];
      registration.set_selected(static_cast<std::uint32_t>(i),
                                selection.selected ? selection.selected() : -1);
    }
    registration.end_sample();
  }

  // Stores the loop's dispatch count in the row from the loop thread.
  struct ProgressObserver final : event::DispatchObserver {
    NodeRegistration* registration{nullptr};
    void on_dispatch_count(std::uint64_t count) noexcept override {
      registration->set_dispatch_count(count);
    }
  };

  // Detaches ProgressObserver from the loop start() gave it, if any.
  void detach() {
    if (detach_) std::exchange(detach_, nullptr)();
  }

  // Waits `period`, returning as soon as a stop is requested, so stop() --
  // and with it start(), which ends the construction heartbeat -- joins
  // without sitting out the rest of a sleep. Only the reporting thread waits.
  void pause(const std::stop_token& stop, std::chrono::milliseconds period) {
    std::unique_lock lock{pause_mutex_};
    pause_.wait_for(lock, stop, period, [] { return false; });
    wakeups_.fetch_add(1, std::memory_order_relaxed);
  }

  Options options_;
  std::size_t loop_sources_{0};
  std::atomic<bool> published_{false};
  std::atomic<std::uint64_t> wakeups_{0};
  std::string error_;
  std::optional<NodeRegistration> registration_;
  ProgressObserver progress_;
  std::function<void()> detach_;
  std::mutex pause_mutex_;
  std::condition_variable_any pause_;
  std::jthread thread_;
};

}  // namespace talos::introspect
