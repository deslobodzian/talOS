#include "talOS/introspection/registry.h"

#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <string>
#include <thread>

#include "talOS/events/manifest.h"
#include "talOS/events/realtime_event_loop.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/names.h"
#include "talOS/introspection/reporter.h"

namespace talos::introspect {
namespace {

using namespace std::chrono_literals;

// macOS caps shared-memory object names at 31 characters, and concurrent test
// targets must not share a segment, so every test gets a short unique name.
std::string temp_registry(std::string_view tag) {
  return "/tr_" + std::string{tag} + "_" + std::to_string(::getpid());
}

class Registry : public ::testing::Test {
 protected:
  void SetUp() override { RegistryMapping::unlink(path_); }
  void TearDown() override { RegistryMapping::unlink(path_); }

  std::string path_ = temp_registry("reg");
};

event::Manifest sample_manifest() {
  event::Manifest manifest;
  manifest.push_back({.id = 0,
                      .kind = event::SourceKind::TIMER,
                      .name = "tick",
                      .period_ns = 5'000'000});
  manifest.push_back({.id = 1,
                      .kind = event::SourceKind::WATCHER,
                      .name = "/drivetrain/state",
                      .message_bytes = 48,
                      .alignment = 8});
  manifest.push_back({.id = 2,
                      .kind = event::SourceKind::SENDER,
                      .name = "/odometry/state",
                      .message_bytes = 64,
                      .alignment = 8});
  return manifest;
}

TEST_F(Registry, PublishesTopologyAndCountersToAReader) {
  NodeRegistration registration{
      {.name = "odometry", .target = "//2026-robot:odometry", .session_id = 7},
      path_};
  registration.publish(sample_manifest());
  registration.set_dispatch_count(99);
  registration.set_source(2, /*events=*/12, /*dropped=*/1, /*sequence=*/11,
                          /*last_monotonic_ns=*/1234, /*last_latency_ns=*/56,
                          /*max_latency_ns=*/78);

  auto reader = RegistryReader::open(path_);
  ASSERT_TRUE(reader.has_value());

  const auto nodes = reader->nodes();
  ASSERT_EQ(nodes.size(), 1u);
  const NodeSnapshot& node = nodes.front();

  EXPECT_EQ(node.name, "odometry");
  EXPECT_EQ(node.target, "//2026-robot:odometry");
  EXPECT_EQ(node.session_id, 7u);
  EXPECT_EQ(node.pid, static_cast<std::uint64_t>(::getpid()));
  EXPECT_EQ(node.dispatch_count, 99u);
  EXPECT_TRUE(node.alive);
  ASSERT_EQ(node.sources.size(), 3u);

  EXPECT_EQ(node.sources[0].kind, event::SourceKind::TIMER);
  EXPECT_EQ(node.sources[0].name, "tick");
  EXPECT_EQ(node.sources[0].period_ns, 5'000'000);

  EXPECT_EQ(node.sources[1].kind, event::SourceKind::WATCHER);
  EXPECT_EQ(node.sources[1].name, "/drivetrain/state");
  EXPECT_EQ(node.sources[1].message_bytes, 48u);

  EXPECT_EQ(node.sources[2].kind, event::SourceKind::SENDER);
  EXPECT_EQ(node.sources[2].name, "/odometry/state");
  EXPECT_EQ(node.sources[2].events, 12u);
  EXPECT_EQ(node.sources[2].dropped, 1u);
  EXPECT_EQ(node.sources[2].sequence, 11u);
  EXPECT_EQ(node.sources[2].max_latency_ns, 78);
}

TEST_F(Registry, SeveralNodesCoexistAndAReleasedSlotDisappears) {
  auto reader = [this] {
    NodeRegistration first{{.name = "a"}, path_};
    first.publish(sample_manifest());
    {
      NodeRegistration second{{.name = "b"}, path_};
      second.publish({});
      auto view = RegistryReader::open(path_);
      EXPECT_EQ(view->nodes().size(), 2u);
    }
    // `second` released its slot; `first` must still be there and must not
    // have been disturbed by the neighbour coming and going.
    auto view = RegistryReader::open(path_);
    const auto nodes = view->nodes();
    EXPECT_EQ(nodes.size(), 1u);
    EXPECT_EQ(nodes.front().name, "a");
    EXPECT_EQ(nodes.front().sources.size(), 3u);
    return true;
  }();
  EXPECT_TRUE(reader);

  // Both registrations are gone now, so the registry is empty but still valid.
  auto view = RegistryReader::open(path_);
  ASSERT_TRUE(view.has_value());
  EXPECT_TRUE(view->nodes().empty());
}

TEST_F(Registry, ReaderReportsNothingBeforeAnyNodeStarts) {
  EXPECT_FALSE(RegistryReader::open(temp_registry("none")).has_value());
}

TEST_F(Registry, SourcesBeyondCapacityAreCountedButNotStored) {
  event::Manifest manifest;
  for (std::size_t i = 0; i < kMaxSourcesPerNode + 5; ++i) {
    manifest.push_back({.id = static_cast<std::uint16_t>(i),
                        .kind = event::SourceKind::TIMER,
                        .name = "t" + std::to_string(i)});
  }

  NodeRegistration registration{{.name = "busy"}, path_};
  registration.publish(manifest);

  auto reader = RegistryReader::open(path_);
  ASSERT_TRUE(reader.has_value());
  const auto nodes = reader->nodes();
  ASSERT_EQ(nodes.size(), 1u);
  EXPECT_EQ(nodes.front().sources.size(), kMaxSourcesPerNode);
  EXPECT_EQ(nodes.front().declared_source_count, manifest.size());
}

TEST_F(Registry, RefusesToRegisterMoreNodesThanTheSegmentHolds) {
  std::vector<std::unique_ptr<NodeRegistration>> held;
  for (std::size_t i = 0; i < kMaxNodes; ++i) {
    held.push_back(std::make_unique<NodeRegistration>(
        NodeRegistration::Identity{.name = "n" + std::to_string(i)}, path_));
    held.back()->publish({});
  }
  // Every slot is claimed and every one is heartbeating, so a further node is
  // refused rather than stealing a live slot.
  EXPECT_THROW(NodeRegistration({.name = "overflow"}, path_),
               std::runtime_error);

  held.pop_back();
  EXPECT_NO_THROW(NodeRegistration({.name = "fits_now"}, path_));
}

TEST_F(Registry, ReporterPublishesARunningLoopWithoutTouchingItsManifest) {
  const std::string topic = "/probe/state/p" + std::to_string(::getpid());
  const std::string feed = "/probe/telemetry/p" + std::to_string(::getpid());
  event::RealtimeEventLoop<> loop;

  struct Handler {
    std::uint64_t fired{0};
    void OnTick(const event::Context&) { ++fired; }
  } handler;

  auto timer =
      event::make_timer<&Handler::OnTick>(loop, "introspect_tick", &handler);
  const std::uint16_t sender =
      loop.register_sender(topic, /*message_bytes=*/8, /*alignment=*/8);
  const std::size_t sources_before = loop.manifest().size();

  Reporter reporter{loop,
                    {.name = "probe",
                     .target = "//talOS/introspection:registry_test",
                     .session_id = 42,
                     .simulation = true,
                     // A topic the node owns outside the loop, carrying its
                     // attribute inline.
                     .extra = {{.kind = event::SourceKind::SENDER,
                                .name = feed,
                                .message_bytes = 64,
                                .flags = naming::kSourceFlagOptional}},
                     // The loop's own sender, keyed by name because the loop
                     // has no idea that this one's consumer is off-box.
                     .endpoints = {{topic, naming::kSourceFlagExternal}},
                     .period = 20ms,
                     .registry_path = path_}};

  timer.setup_periodic(event::Poller::now() + 1ms, 2ms);
  loop.run_for(220ms);

  // Introspection must not appear in the manifest: it would change every
  // source id and make existing logs unreplayable.
  EXPECT_EQ(loop.manifest().size(), sources_before);
  ASSERT_TRUE(reporter.published()) << reporter.error();
  EXPECT_GT(handler.fired, 0u);

  auto reader = RegistryReader::open(path_);
  ASSERT_TRUE(reader.has_value());
  const auto nodes = reader->nodes();
  ASSERT_EQ(nodes.size(), 1u);
  const NodeSnapshot& node = nodes.front();

  EXPECT_EQ(node.name, "probe");
  EXPECT_EQ(node.session_id, 42u);
  EXPECT_TRUE(node.flags & kFlagSimulation);
  EXPECT_TRUE(node.alive);
  EXPECT_GT(node.dispatch_count, 0u);

  ASSERT_EQ(node.sources.size(), 3u);
  EXPECT_EQ(node.sources[0].kind, event::SourceKind::TIMER);
  EXPECT_EQ(node.sources[0].name, "introspect_tick");
  EXPECT_GT(node.sources[0].events, 0u);
  // A timer has no far end, so it carries no attributes.
  EXPECT_EQ(node.sources[0].flags, 0u);

  EXPECT_EQ(node.sources[1].kind, event::SourceKind::SENDER);
  EXPECT_EQ(node.sources[1].name, topic);
  EXPECT_EQ(node.sources[1].id, sender);
  // Nothing was published on it, so it must read as idle rather than as
  // traffic that never happened -- and it must read as deliberately
  // unsubscribed rather than as a topic whose subscriber is missing.
  EXPECT_EQ(node.sources[1].events, 0u);
  EXPECT_TRUE(node.sources[1].external());
  EXPECT_FALSE(node.sources[1].optional());

  EXPECT_EQ(node.sources[2].name, feed);
  EXPECT_TRUE(node.sources[2].optional());
  EXPECT_FALSE(node.sources[2].external());
}

TEST_F(Registry, LoopCountsPublishesAndTimerDispatchesPerSource) {
  const std::string topic = "/tc_" + std::to_string(::getpid());
  event::RealtimeEventLoop<> loop;

  struct Publisher {
    event::Sender<event::RealtimeEventLoop<>, std::uint64_t> sender;
    std::uint64_t sent{0};
    void OnTick(const event::Context&) {
      if (sender.send(sent)) ++sent;
    }
  } publisher;

  publisher.sender = event::make_sender<std::uint64_t>(loop, topic);
  auto timer = event::make_timer<&Publisher::OnTick>(loop, "pub", &publisher);
  timer.setup_periodic(event::Poller::now() + 1ms, 2ms);
  loop.run_for(120ms);

  ASSERT_EQ(loop.source_counter_count(), 2u);
  const event::SourceCounters* counters = loop.source_counters();
  ASSERT_NE(counters, nullptr);

  // Source 0 is the sender, 1 the timer: registration order, which is also
  // the order the manifest and the log use.
  EXPECT_GT(publisher.sent, 0u);
  EXPECT_EQ(counters[0].events.load(), publisher.sent);

  // Every tick published once, so the timer fired exactly as often as the
  // sender wrote. The loop's own dispatch count is one higher because it also
  // records the run's EXIT.
  EXPECT_EQ(counters[1].events.load(), publisher.sent);
  EXPECT_GE(loop.dispatch_count(), counters[1].events.load());
  EXPECT_GE(counters[1].last_latency_ns.load(), 0);
}

// The point of the flags word: a reader gets back exactly what the node
// declared, so a graph tool can tell a designed dead end from a fault without
// asking the node again.
TEST_F(Registry, EndpointAttributesSurviveTheSegment) {
  event::Manifest manifest;
  manifest.push_back({.id = 0,
                      .kind = event::SourceKind::SENDER,
                      .name = "/hw/command",
                      .message_bytes = 16,
                      .alignment = 8});
  manifest.push_back({.id = 1,
                      .kind = event::SourceKind::WATCHER,
                      .name = "/drivetrain/target/auto",
                      .message_bytes = 24,
                      .alignment = 8});
  manifest.push_back({.id = 2,
                      .kind = event::SourceKind::SENDER,
                      .name = "/arbiter/request/shooter",
                      .message_bytes = 32,
                      .alignment = 8});
  manifest.push_back({.id = 3,
                      .kind = event::SourceKind::SENDER,
                      .name = "/arbiter/state",
                      .message_bytes = 8,
                      .alignment = 8});

  const std::vector<EndpointAttribute> endpoints = {
      // Consumed by the RoboRIO over UDP: no shared-memory subscriber, ever.
      {"/hw/command", naming::kSourceFlagExternal},
      // Autonomous is not written yet, so nothing publishes this.
      {"/drivetrain/target/auto", naming::kSourceFlagOptional},
      // Two entries for one topic, which is how a node says both things
      // without having to OR the constants at the call site.
      {"/arbiter/request/shooter", naming::kSourceFlagExternal},
      {"/arbiter/request/shooter", naming::kSourceFlagOptional},
  };

  NodeRegistration registration{{.name = "arbiter"}, path_};
  registration.publish(manifest, endpoints);

  auto reader = RegistryReader::open(path_);
  ASSERT_TRUE(reader.has_value());
  const auto nodes = reader->nodes();
  ASSERT_EQ(nodes.size(), 1u);
  const auto& sources = nodes.front().sources;
  ASSERT_EQ(sources.size(), 4u);

  EXPECT_EQ(sources[0].name, "/hw/command");
  EXPECT_TRUE(sources[0].external());
  EXPECT_FALSE(sources[0].optional());

  EXPECT_EQ(sources[1].name, "/drivetrain/target/auto");
  EXPECT_FALSE(sources[1].external());
  EXPECT_TRUE(sources[1].optional());

  EXPECT_EQ(sources[2].name, "/arbiter/request/shooter");
  EXPECT_TRUE(sources[2].external());
  EXPECT_TRUE(sources[2].optional());

  // Declared nothing, so it reads as an ordinary topic: the attribute is
  // keyed by name and must not leak to its neighbours.
  EXPECT_EQ(sources[3].name, "/arbiter/state");
  EXPECT_EQ(sources[3].flags, 0u);

  // The descriptive fields the flags word sits between are unaffected, which
  // is the claim that let the version stay at 1.
  EXPECT_EQ(sources[0].message_bytes, 16u);
  EXPECT_EQ(sources[0].alignment, 8u);
  EXPECT_EQ(sources[1].kind, event::SourceKind::WATCHER);
}

// The v1 case, which is also the default case: a node that declares no
// attributes -- or a writer built before attributes existed, which wrote this
// word as a zeroed `reserved` -- must read as neither external nor optional,
// not as some third state.
TEST_F(Registry, ASourceWithNoAttributesIsNeitherExternalNorOptional) {
  NodeRegistration registration{{.name = "odometry"}, path_};
  registration.publish(sample_manifest());

  auto reader = RegistryReader::open(path_);
  ASSERT_TRUE(reader.has_value());
  const auto nodes = reader->nodes();
  ASSERT_EQ(nodes.size(), 1u);
  const auto& sources = nodes.front().sources;
  ASSERT_EQ(sources.size(), 3u);

  for (const SourceSnapshot& source : sources) {
    EXPECT_EQ(source.flags, 0u) << source.name;
    EXPECT_FALSE(source.external()) << source.name;
    EXPECT_FALSE(source.optional()) << source.name;
  }
}

// A misspelled topic is reported and then published anyway. Taking the node off
// the robot to punish a name would be a worse failure than the name, and the
// registry showing the spelling the node actually used is the whole point: that
// is how the two ends are found not to meet.
TEST_F(Registry, AMisnamedTopicIsWarnedAboutAndStillPublished) {
  event::Manifest manifest;
  manifest.push_back({.id = 0,
                      .kind = event::SourceKind::TIMER,
                      .name = "tick",
                      .period_ns = 5'000'000});
  manifest.push_back({.id = 1,
                      .kind = event::SourceKind::SENDER,
                      .name = "/hw/cmd",
                      .message_bytes = 8,
                      .alignment = 8});

  NodeRegistration registration{{.name = "hardware_bridge"}, path_};

  ::testing::internal::CaptureStderr();
  EXPECT_NO_THROW(registration.publish(manifest));
  // Publishing twice must not double the warning: the reporting thread calls
  // into the registry forever, and a warning that repeats is a warning that
  // gets filtered out.
  registration.publish(manifest);
  const std::string warnings = ::testing::internal::GetCapturedStderr();

  EXPECT_NE(warnings.find("/hw/cmd"), std::string::npos) << warnings;
  EXPECT_NE(warnings.find("hardware_bridge"), std::string::npos) << warnings;
  EXPECT_EQ(std::count(warnings.begin(), warnings.end(), '\n'), 1) << warnings;
  // A timer's name is a label, not an address, so the topic grammar must not
  // be applied to it.
  EXPECT_EQ(warnings.find("tick"), std::string::npos) << warnings;

  auto reader = RegistryReader::open(path_);
  ASSERT_TRUE(reader.has_value());
  const auto nodes = reader->nodes();
  ASSERT_EQ(nodes.size(), 1u);
  ASSERT_EQ(nodes.front().sources.size(), 2u);
  EXPECT_EQ(nodes.front().sources[1].name, "/hw/cmd");
}

// The heartbeat, the dispatch count and the per-source counters move as one
// sample: a reader that arrives between samples sees the whole sample, never
// a fresh heartbeat paired with the previous refresh's counts.
TEST_F(Registry, WholeSamplesReadWhole) {
  NodeRegistration registration{{.name = "seamed"}, path_};
  registration.publish(sample_manifest());

  auto reader = RegistryReader::open(path_);
  ASSERT_TRUE(reader.has_value());

  for (std::uint64_t i = 1; i <= 20; ++i) {
    registration.begin_sample();
    registration.heartbeat();
    registration.set_dispatch_count(i);
    registration.set_source(2, /*events=*/i, /*dropped=*/0, /*sequence=*/i,
                            /*last_monotonic_ns=*/0, /*last_latency_ns=*/0,
                            /*max_latency_ns=*/0);
    registration.end_sample();

    const auto nodes = reader->nodes();
    ASSERT_EQ(nodes.size(), 1u);
    EXPECT_EQ(nodes.front().dispatch_count, i);
    ASSERT_EQ(nodes.front().sources.size(), 3u);
    EXPECT_EQ(nodes.front().sources[2].events, i);
    EXPECT_EQ(nodes.front().sources[2].sequence, i);
  }
}

// A sample parked open -- the heartbeat advanced but the sample never closed
// -- must not wedge or drop the reader. The bounded retries give up and return
// the plain copy an older reader would have taken; once the sample closes,
// the whole new sample is visible at once.
TEST_F(Registry, ReaderMakesProgressWhileASampleIsOpen) {
  NodeRegistration registration{{.name = "parked"}, path_};
  registration.publish(sample_manifest());
  registration.set_dispatch_count(7);
  registration.set_source(2, /*events=*/7, /*dropped=*/0, /*sequence=*/7,
                          /*last_monotonic_ns=*/0, /*last_latency_ns=*/0,
                          /*max_latency_ns=*/0);

  auto reader = RegistryReader::open(path_);
  ASSERT_TRUE(reader.has_value());

  registration.begin_sample();
  registration.heartbeat();

  // The sequence stays odd for as long as the sample stays open; the reader
  // still terminates and still reports the node.
  const auto mid = reader->nodes();
  ASSERT_EQ(mid.size(), 1u);
  EXPECT_EQ(mid.front().name, "parked");

  registration.set_dispatch_count(8);
  registration.set_source(2, /*events=*/8, /*dropped=*/0, /*sequence=*/8,
                          /*last_monotonic_ns=*/0, /*last_latency_ns=*/0,
                          /*max_latency_ns=*/0);
  registration.end_sample();

  const auto after = reader->nodes();
  ASSERT_EQ(after.size(), 1u);
  EXPECT_EQ(after.front().dispatch_count, 8u);
  ASSERT_EQ(after.front().sources.size(), 3u);
  EXPECT_EQ(after.front().sources[2].events, 8u);
}

// A writer that predates the guard never touches the sequence word, which
// reads back as the zero its segment was created with: undisturbed zero is
// even, so old writers unconditionally read as stable.
TEST_F(Registry, WriterWithoutTheGuardReadsAsStable) {
  NodeRegistration registration{{.name = "legacy"}, path_};
  registration.publish(sample_manifest());

  // Bypass the guard exactly as an older build does: store the counters and
  // the heartbeat straight into the slot, leaving the sequence word alone.
  RegistryMapping mapping{path_};
  NodeRecord& record = mapping.header()->nodes[registration.slot()];
  record.dispatch_count.store(41, std::memory_order_relaxed);
  record.sources[2].events.store(41, std::memory_order_relaxed);
  record.heartbeat_wall_ns.store(wall_now_ns(), std::memory_order_relaxed);
  EXPECT_EQ(record.sample_seq.load(std::memory_order_relaxed), 0u);

  auto reader = RegistryReader::open(path_);
  ASSERT_TRUE(reader.has_value());
  const auto nodes = reader->nodes();
  ASSERT_EQ(nodes.size(), 1u);
  EXPECT_EQ(nodes.front().dispatch_count, 41u);
  ASSERT_EQ(nodes.front().sources.size(), 3u);
  EXPECT_EQ(nodes.front().sources[2].events, 41u);
  EXPECT_TRUE(nodes.front().alive);
}

}  // namespace
}  // namespace talos::introspect
