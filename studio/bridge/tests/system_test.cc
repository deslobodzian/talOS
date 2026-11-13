#include "studio/bridge/system.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/hardware/packet.h"
#include "talOS/introspection/names.h"

namespace studio {
namespace {

const TopicRow* Row(const std::vector<TopicRow>& rows, std::string_view name) {
  for (const TopicRow& row : rows) {
    if (row.name == name) return &row;
  }
  return nullptr;
}

introspect::NodeSnapshot Odometry() {
  introspect::NodeSnapshot node;
  node.name = "odometry";
  node.target = "//2026-robot/main_processor/odometry:node";
  node.pid = 4242;
  node.session_id = 18446744073709551615ull;  // uint64 max
  node.generation = 3;
  node.dispatch_count = 9007199254740993ull;  // 2^53 + 1
  node.start_wall_ns = 1'700'000'000'000'000'000;
  node.heartbeat_monotonic_ns = 500'000'000;
  node.flags = introspect::kFlagSimulation;
  node.declared_source_count = 2;
  node.alive = true;
  node.sources = {
      {.id = 0,
       .kind = event::SourceKind::WATCHER,
       .name = "/drivetrain/state",
       .message_bytes = 48,
       .alignment = 8,
       .events = 500,
       .dropped = 2},
      {.id = 1,
       .kind = event::SourceKind::SENDER,
       .name = "/odometry/state",
       .message_bytes = 64,
       .alignment = 8,
       .events = 499},
  };
  return node;
}

introspect::NodeSnapshot Telemetry() {
  introspect::NodeSnapshot node;
  node.name = "telemetry";
  node.alive = true;
  node.sources = {
      {.id = 0,
       .kind = event::SourceKind::WATCHER,
       .name = "/odometry/state",
       .message_bytes = 64,
       .events = 480},
      {.id = 1,
       .kind = event::SourceKind::TIMER,
       .name = "telemetry",
       .period_ns = 10'000'000,
       .events = 500},
  };
  return node;
}

// The hardware bridge, as talOS/bridge/node.cc actually declares itself: it
// publishes `/hw/command` for a consumer that lives across a UDP link.
introspect::NodeSnapshot HardwareBridge() {
  introspect::NodeSnapshot node;
  node.name = "hardware_bridge";
  node.alive = true;
  node.declared_source_count = 2;
  node.sources = {
      {.id = 0,
       .kind = event::SourceKind::SENDER,
       .name = talos::hardware::kStateTopic,
       .message_bytes = 1032,
       .alignment = 8,
       .events = 40},
      {.id = 1,
       .kind = event::SourceKind::SENDER,
       .name = talos::hardware::kCommandTopic,
       .message_bytes = 1032,
       .alignment = 8,
       .flags = naming::kSourceFlagExternal,
       .events = 12},
  };
  return node;
}

TEST(SystemGraph, DerivesTopicsFromBothEndsAcrossNodes) {
  const auto topics = TopicsOf({Odometry(), Telemetry()});
  ASSERT_EQ(topics.size(), 2u);

  // Sorted by name, and the timer is absent: it has a name but no other end.
  //
  // /drivetrain/state has a reader and no writer in this system, which is
  // exactly the shape a viewer needs to surface: a node waiting on a topic
  // nothing publishes looks identical to a healthy one from the inside.
  EXPECT_EQ(topics[0].name, "/drivetrain/state");
  EXPECT_TRUE(topics[0].publishers.empty());
  EXPECT_EQ(topics[0].subscribers, std::vector<std::string>{"odometry"});
  EXPECT_EQ(topics[0].received, 500u);
  EXPECT_EQ(topics[0].dropped, 2u);

  // Both ends of /odometry/state, from two different nodes, collapsed into
  // one row.
  EXPECT_EQ(topics[1].name, "/odometry/state");
  EXPECT_EQ(topics[1].publishers, std::vector<std::string>{"odometry"});
  EXPECT_EQ(topics[1].subscribers, std::vector<std::string>{"telemetry"});
  EXPECT_EQ(topics[1].published, 499u);
  EXPECT_EQ(topics[1].received, 480u);
  EXPECT_EQ(topics[1].message_bytes, 64u);
}

TEST(SystemGraph, LargeCountersSurviveAsDecimalStrings) {
  SystemGraphInput input;
  input.nodes = {Odometry()};
  input.registry_available = true;
  input.node_capacity = 32;
  input.source_capacity = 64;
  input.wall_ns = 1'700'000'000'500'000'000;
  input.monotonic_ns = 500'000'000;
  input.bridge = {
      .topic = "/talos/telemetry", .clients = 2, .frames_forwarded = 7};

  const std::string json = SystemGraphJson(input);

  // A JSON number is a double: past 2^53 these values would come back wrong.
  EXPECT_NE(json.find("\"session_id\":\"18446744073709551615\""),
            std::string::npos);
  EXPECT_NE(json.find("\"dispatch_count\":\"9007199254740993\""),
            std::string::npos);
  EXPECT_NE(json.find("\"wall_ns\":\"1700000000500000000\""),
            std::string::npos);

  // Small, bounded values stay numbers so a reader does not have to parse them.
  EXPECT_NE(json.find("\"message_bytes\":48"), std::string::npos);
  EXPECT_NE(json.find("\"node_capacity\":32"), std::string::npos);
  EXPECT_NE(json.find("\"clients\":2"), std::string::npos);

  EXPECT_NE(json.find("\"kind\":\"talos.system_graph\""), std::string::npos);
  EXPECT_NE(json.find("\"simulation\":true"), std::string::npos);
  EXPECT_NE(json.find("\"replay\":false"), std::string::npos);
  EXPECT_NE(json.find("\"alive\":true"), std::string::npos);
  EXPECT_NE(json.find("\"kind\":\"SENDER\""), std::string::npos);
  EXPECT_NE(json.find("\"kind\":\"WATCHER\""), std::string::npos);
}

TEST(SystemGraph, ConvertsMonotonicHeartbeatAgeToWallTimeForTheViewer) {
  SystemGraphInput input;
  input.nodes = {Odometry()};
  input.monotonic_ns = 750'000'000;
  input.wall_ns = 1'700'000'000'750'000'000;
  auto json = SystemGraphJson(input);
  EXPECT_NE(json.find("\"heartbeat_wall_ns\":\"1700000000500000000\""),
            std::string::npos);
  // A wall-clock step changes the display epoch, not the heartbeat's age or
  // the registry's already-computed liveness decision.
  input.wall_ns += 60'000'000'000;
  json = SystemGraphJson(input);
  EXPECT_NE(json.find("\"heartbeat_wall_ns\":\"1700000060500000000\""),
            std::string::npos);
  EXPECT_NE(json.find("\"alive\":true"), std::string::npos);
}

TEST(SystemGraph, ReportsAnAbsentRegistryRatherThanFailing) {
  SystemGraphInput input;
  input.bridge.topic = "/talos/telemetry";
  const std::string json = SystemGraphJson(input);

  EXPECT_NE(json.find("\"available\":false"), std::string::npos);
  EXPECT_NE(json.find("\"nodes\":[]"), std::string::npos);
  EXPECT_NE(json.find("\"topics\":[]"), std::string::npos);
}

TEST(SystemGraph, NamesWhoDrivesEachTargetAndNullWhenNobodyDoes) {
  introspect::NodeSnapshot drivetrain;
  drivetrain.name = "drivetrain";
  drivetrain.alive = true;
  drivetrain.selections = {
      {.sources = {"teleop", "auto"}, .selected = 1},
      {.sources = {"teleop", "auto"}, .selected = -1},
  };
  SystemGraphInput input;
  input.nodes = {drivetrain, Telemetry()};
  const std::string json = SystemGraphJson(input);

  EXPECT_NE(json.find("\"selections\":[{\"sources\":[\"teleop\",\"auto\"],"
                      "\"selected\":\"auto\"},{\"sources\":[\"teleop\","
                      "\"auto\"],\"selected\":null}]"),
            std::string::npos)
      << json;
  // A node without Targets says so with an empty list, not a missing field.
  EXPECT_NE(json.find("\"name\":\"telemetry\""), std::string::npos);
  EXPECT_NE(json.find("\"selections\":[]"), std::string::npos) << json;
}

TEST(SystemGraph, EscapesNamesThatWouldOtherwiseBreakTheDocument) {
  introspect::NodeSnapshot node;
  node.name = "odd\"name\\with\ttabs";
  node.sources = {{.id = 0,
                   .kind = event::SourceKind::SENDER,
                   .name = std::string{"/topic\n"} + char{0x01}}};

  SystemGraphInput input;
  input.nodes = {node};
  const std::string json = SystemGraphJson(input);

  EXPECT_NE(json.find("odd\\\"name\\\\with\\ttabs"), std::string::npos);
  EXPECT_NE(json.find("/topic\\n\\u0001"), std::string::npos);
  // The raw control character must not appear literally.
  EXPECT_EQ(json.find(char{0x01}), std::string::npos);
}

TEST(TopicHealth, SeparatesADesignedDeadEndFromABrokenOne) {
  introspect::NodeSnapshot drivetrain;
  drivetrain.name = "drivetrain";
  drivetrain.alive = true;
  drivetrain.sources = {
      // Autonomous is not written yet, and the node says so out loud.
      {.id = 0,
       .kind = event::SourceKind::FETCHER,
       .name = "/drivetrain/target/auto",
       .message_bytes = 32,
       .flags = naming::kSourceFlagOptional},
      // The identical shape with nothing declared about it. This one really is
      // a fault, and it has to keep reading as one -- the point of the two new
      // verdicts is to make this row stand out, not to soften it.
      {.id = 1,
       .kind = event::SourceKind::FETCHER,
       .name = "/shooter/target/teleop",
       .message_bytes = 32},
  };

  const std::vector<TopicRow> topics = TopicsOf({HardwareBridge(), drivetrain});

  // Published with no shared-memory subscriber -- but the subscriber is a
  // RoboRIO across a UDP link, so there is nothing absent to go and find.
  const TopicRow* command = Row(topics, talos::hardware::kCommandTopic);
  ASSERT_NE(command, nullptr);
  EXPECT_TRUE(command->publishers.size() == 1 && command->subscribers.empty());
  EXPECT_TRUE(command->external);
  EXPECT_FALSE(command->optional);
  EXPECT_EQ(HealthOf(*command), TopicHealth::BRIDGED);
  EXPECT_STREQ(HealthName(HealthOf(*command)), "bridged");

  // Subscribed with no publisher, and declared that way on purpose.
  const TopicRow* automatic = Row(topics, "/drivetrain/target/auto");
  ASSERT_NE(automatic, nullptr);
  EXPECT_TRUE(automatic->optional);
  EXPECT_EQ(HealthOf(*automatic), TopicHealth::UNCONNECTED);
  EXPECT_STREQ(HealthName(HealthOf(*automatic)), "unconnected");

  // The same two shapes with no declaration keep the verdicts they had. A
  // release that quietly reclassified these would be worse than no report.
  const TopicRow* state = Row(topics, talos::hardware::kStateTopic);
  ASSERT_NE(state, nullptr);
  EXPECT_FALSE(state->external);
  EXPECT_FALSE(state->optional);
  EXPECT_EQ(HealthOf(*state), TopicHealth::UNREAD);

  const TopicRow* teleop = Row(topics, "/shooter/target/teleop");
  ASSERT_NE(teleop, nullptr);
  EXPECT_EQ(HealthOf(*teleop), TopicHealth::ORPHANED);
}

TEST(TopicHealth, ExternalOutranksOptionalOnTheSameTopic) {
  // Both flags on one topic is not a contradiction: an end can be outside
  // talOS and not yet wired up. `bridged` wins because it is the more specific
  // claim -- it says where the far end is, not merely that it may be absent.
  introspect::NodeSnapshot node;
  node.name = "hardware_bridge";
  node.sources = {
      {.id = 0,
       .kind = event::SourceKind::SENDER,
       .name = talos::hardware::kCommandTopic,
       .flags = naming::kSourceFlagExternal | naming::kSourceFlagOptional}};

  const std::vector<TopicRow> topics = TopicsOf({node});
  ASSERT_EQ(topics.size(), 1u);
  EXPECT_TRUE(topics[0].external);
  EXPECT_TRUE(topics[0].optional);
  EXPECT_EQ(HealthOf(topics[0]), TopicHealth::BRIDGED);
}

TEST(TopicHealth, KeepsTheVerdictsThatAlreadyWorkedForConnectedTopics) {
  introspect::NodeSnapshot writer;
  writer.name = "odometry";
  writer.sources = {
      {.id = 0,
       .kind = event::SourceKind::SENDER,
       .name = "/odometry/state",
       .events = 500},
      {.id = 1, .kind = event::SourceKind::SENDER, .name = "/odometry/status"},
      {.id = 2,
       .kind = event::SourceKind::SENDER,
       .name = "/odometry/event",
       .events = 9,
       .dropped = 3},
  };
  introspect::NodeSnapshot reader;
  reader.name = "telemetry";
  reader.sources = {
      {.id = 0,
       .kind = event::SourceKind::WATCHER,
       .name = "/odometry/state",
       .events = 500},
      {.id = 1, .kind = event::SourceKind::WATCHER, .name = "/odometry/status"},
      {.id = 2, .kind = event::SourceKind::WATCHER, .name = "/odometry/event"},
  };

  const std::vector<TopicRow> topics = TopicsOf({writer, reader});
  EXPECT_EQ(HealthOf(*Row(topics, "/odometry/state")), TopicHealth::OK);
  EXPECT_EQ(HealthOf(*Row(topics, "/odometry/status")), TopicHealth::IDLE);
  EXPECT_EQ(HealthOf(*Row(topics, "/odometry/event")), TopicHealth::LOSSY);
}

// A latest-only watcher skips to the newest message by design and counts the
// skips as dropped: a 50 Hz reader of a 200 Hz topic drops three in four. That
// is not loss. A sequence watcher's drops are.
TEST(TopicHealth, LossyCountsOnlyDropsThatAreNotLatestOnlySkips) {
  introspect::NodeSnapshot writer;
  writer.name = "drivetrain";
  writer.sources = {
      {.id = 0,
       .kind = event::SourceKind::SENDER,
       .name = "/drivetrain/state",
       .events = 800},
      {.id = 1,
       .kind = event::SourceKind::SENDER,
       .name = "/drivetrain/event",
       .events = 800},
  };
  introspect::NodeSnapshot reader;
  reader.name = "pose_estimator";
  reader.sources = {
      {.id = 0,
       .kind = event::SourceKind::WATCHER,
       .name = "/drivetrain/state",
       .flags = naming::kSourceFlagLatestOnly,
       .events = 200,
       .dropped = 600},
      {.id = 1,
       .kind = event::SourceKind::WATCHER,
       .name = "/drivetrain/event",
       .events = 798,
       .dropped = 2},
  };

  const std::vector<TopicRow> topics = TopicsOf({writer, reader});
  EXPECT_EQ(HealthOf(*Row(topics, "/drivetrain/state")), TopicHealth::OK);
  EXPECT_EQ(Row(topics, "/drivetrain/state")->dropped, 0u);
  EXPECT_EQ(HealthOf(*Row(topics, "/drivetrain/event")), TopicHealth::LOSSY);
}

TEST(SystemGraph, CarriesEndpointAttributesOnEverySourceAndTopic) {
  SystemGraphInput input;
  introspect::NodeSnapshot drivetrain;
  drivetrain.name = "drivetrain";
  drivetrain.alive = true;
  drivetrain.sources = {{.id = 0,
                         .kind = event::SourceKind::FETCHER,
                         .name = "/drivetrain/target/auto",
                         .message_bytes = 32,
                         .flags = naming::kSourceFlagOptional}};
  introspect::NodeSnapshot telemetry;
  telemetry.name = "telemetry";
  telemetry.alive = true;
  telemetry.sources = {{.id = 0,
                        .kind = event::SourceKind::WATCHER,
                        .name = "/drivetrain/state",
                        .message_bytes = 48,
                        .flags = naming::kSourceFlagObserver}};
  input.nodes = {HardwareBridge(), drivetrain, telemetry};
  input.registry_available = true;

  const std::string json = SystemGraphJson(input);

  // The version says the document has fields a classifier must respect. A
  // reader still on version 1 would go on calling /hw/command orphaned.
  EXPECT_NE(json.find("\"version\":2"), std::string::npos);

  // Per source, so a viewer can explain one endpoint without re-deriving the
  // topic table.
  EXPECT_NE(json.find("\"name\":\"/hw/command\",\"message_bytes\":1032,"
                      "\"alignment\":8,\"external\":true,\"optional\":false,"
                      "\"observer\":false"),
            std::string::npos);
  EXPECT_NE(json.find("\"name\":\"/hw/state\",\"message_bytes\":1032,"
                      "\"alignment\":8,\"external\":false,\"optional\":false,"
                      "\"observer\":false"),
            std::string::npos);
  EXPECT_NE(json.find("\"name\":\"/drivetrain/target/auto\",\"message_bytes\":"
                      "32,\"alignment\":0,\"external\":false,\"optional\":true,"
                      "\"observer\":false"),
            std::string::npos);
  EXPECT_NE(json.find("\"name\":\"/drivetrain/state\",\"message_bytes\":48,"
                      "\"alignment\":0,\"external\":false,\"optional\":false,"
                      "\"observer\":true"),
            std::string::npos);

  // And per topic, with the verdict, so `curl` alone answers the question.
  EXPECT_NE(json.find("{\"name\":\"/hw/command\",\"message_bytes\":1032,"
                      "\"publishers\":[\"hardware_bridge\"],\"subscribers\":[],"
                      "\"observers\":[],"
                      "\"published\":\"12\",\"received\":\"0\","
                      "\"dropped\":\"0\",\"external\":true,\"optional\":false,"
                      "\"health\":\"bridged\"}"),
            std::string::npos);
  EXPECT_NE(json.find("{\"name\":\"/drivetrain/state\",\"message_bytes\":48,"
                      "\"publishers\":[],\"subscribers\":[],"
                      "\"observers\":[\"telemetry\"],"),
            std::string::npos);
  EXPECT_NE(json.find("\"external\":false,\"optional\":true,"
                      "\"health\":\"unconnected\""),
            std::string::npos);
  EXPECT_NE(json.find("\"health\":\"unread\""), std::string::npos);
}

// The graph the golden files hold: three nodes whose fast values (wall time,
// bridge counters, heartbeats, dispatch counts, liveness, delivery counters,
// source counters, selected sources) are numbered in turn and all move on
// from `refresh` 0 to 1 with the structure unchanged, so a value written into
// another slot of its type reads as a wrong value, not the same one.
SystemGraphInput GoldenGraph(int refresh) {
  std::uint64_t next = refresh == 0 ? 1000 : 5000;
  const auto value = [&next] { return next++; };
  const auto signed_value = [&next] {
    return static_cast<std::int64_t>(next++);
  };
  const auto counted = [&](introspect::SourceSnapshot source) {
    source.events = value();
    source.dropped = value();
    source.sequence = value();
    source.last_monotonic_ns = signed_value();
    source.last_latency_ns = signed_value();
    source.max_latency_ns = signed_value();
    return source;
  };
  const auto row = [&](std::string name, std::uint32_t slot) {
    introspect::NodeSnapshot node;
    node.name = std::move(name);
    node.target = "//golden:" + node.name;
    node.slot = slot;
    node.pid = 100 + slot;
    node.session_id = 7;
    node.generation = 1;
    node.start_wall_ns = 1'700'000'000'000'000'000;
    node.heartbeat_monotonic_ns = signed_value();
    node.dispatch_count = value();
    node.wake_failures = value();
    node.stalled_arms = value();
    node.wake_fallbacks = value();
    return node;
  };

  SystemGraphInput input;
  input.registry_available = true;
  input.node_capacity = 64;
  input.source_capacity = 32;
  input.monotonic_ns = 1'000'000'000;
  input.wall_ns = 1'700'000'000'000'000'000 + signed_value();
  input.bridge.topic = "/talos/telemetry";
  input.bridge.clients = refresh == 0 ? 1 : 2;
  input.bridge.frames_forwarded = value();
  input.bridge.invalid = value();
  input.bridge.ws_drops = value();
  input.bridge.refused = value();
  input.bridge.egress_bytes = value();

  introspect::NodeSnapshot shooter = row("shooter", 0);
  shooter.alive = true;
  shooter.declared_source_count = 3;
  shooter.sources = {
      counted({.id = 0,
               .kind = event::SourceKind::SENDER,
               .name = "/shooter/state",
               .message_bytes = 24,
               .alignment = 8}),
      counted({.id = 1,
               .kind = event::SourceKind::WATCHER,
               .name = "/shooter/target/teleop",
               .message_bytes = 32,
               .alignment = 8}),
      counted({.id = 2,
               .kind = event::SourceKind::SENDER,
               .name = "/hw/request/shooter",
               .message_bytes = 1032,
               .alignment = 8}),
  };
  shooter.selections = {
      {.sources = {"teleop", "auto"}, .selected = refresh == 0 ? 0 : 1},
      {.sources = {"teleop"}, .selected = refresh == 0 ? -1 : 0},
  };

  introspect::NodeSnapshot bridge = row("hardware_bridge", 1);
  bridge.alive = true;
  bridge.declared_source_count = 1;
  bridge.sources = {counted({.id = 0,
                             .kind = event::SourceKind::FETCHER,
                             .name = "/hw/request/shooter",
                             .message_bytes = 1032,
                             .alignment = 8})};

  introspect::NodeSnapshot telemetry = row("telemetry", 2);
  telemetry.alive = refresh != 0;
  telemetry.declared_source_count = 1;
  telemetry.sources = {counted({.id = 0,
                                .kind = event::SourceKind::WATCHER,
                                .name = "/shooter/state",
                                .message_bytes = 24,
                                .alignment = 8})};

  input.nodes = {shooter, bridge, telemetry};
  return input;
}

std::string ReadFile(const std::filesystem::path& path) {
  std::ifstream stream{path, std::ios::binary};
  return {std::istreambuf_iterator<char>{stream}, {}};
}

// The order of a graph's fast values has one source: the order
// BuildSystemGraph appends them in. These golden files are its output: the
// whole graph a client is sent first (graph.json), the update GraphFeed
// sends it at the next refresh (update.json), and that refresh's whole graph
// (later.json). //studio:system_test applies the update to graph.json with
// studio/src/system.ts, and //studio/bridge:fast_slots_test with the Python
// harness's fast_slots, each requiring later.json; so a reorder here fails
// this test, and a consumer left in the old order fails its own. An
// intended change regenerates them (studio/bridge/README.md, "HTTP and
// WebSocket views"):
//   TALOS_UPDATE_GOLDEN=1 bazel run //studio/bridge:system_test --
//       --gtest_filter=SystemGraph.GoldenFilesHoldTheFastValueOrder
// REQ-OPS-23
TEST(SystemGraph, GoldenFilesHoldTheFastValueOrder) {
  const SystemGraphDocument first = BuildSystemGraph(GoldenGraph(0));
  const SystemGraphDocument later = BuildSystemGraph(GoldenGraph(1));
  // The refresh changes only fast values, so the client is sent an update.
  ASSERT_EQ(first.structure, later.structure);
  GraphFeed feed;
  std::vector<std::string> sent;
  const auto afford = [](std::size_t) { return true; };
  const auto send = [&sent](std::string_view text) {
    sent.emplace_back(text);
    return true;
  };
  constexpr std::size_t kMaxBytes = std::size_t{1} << 20;
  feed.Pump(first, 0, kMaxBytes, afford, send);
  feed.Pump(later, GraphFeed::kUpdateNs, kMaxBytes, afford, send);
  ASSERT_EQ(sent.size(), 2u);
  ASSERT_EQ(sent[0], first.json);
  const std::pair<const char*, std::string> files[] = {
      {"graph.json", first.json},
      {"update.json", sent[1]},
      {"later.json", later.json},
  };

  const char* update = std::getenv("TALOS_UPDATE_GOLDEN");
  if (update != nullptr && std::string_view{update} == "1") {
    const char* workspace = std::getenv("BUILD_WORKSPACE_DIRECTORY");
    ASSERT_NE(workspace, nullptr) << "update the golden files with bazel run";
    const std::filesystem::path golden =
        std::filesystem::path{workspace} / "studio/bridge/testdata/golden";
    for (const auto& [file, text] : files) {
      std::ofstream{golden / file, std::ios::binary} << text;
    }
    return;
  }
  std::string error;
  const std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  ASSERT_NE(runfiles, nullptr) << error;
  const char* graph = std::getenv("TALOS_GOLDEN_GRAPH");
  ASSERT_NE(graph, nullptr) << "TALOS_GOLDEN_GRAPH is not set";
  const std::filesystem::path golden =
      std::filesystem::path{runfiles->Rlocation(graph)}.parent_path();
  for (const auto& [file, text] : files) {
    EXPECT_EQ(text, ReadFile(golden / file))
        << (golden / file).string()
        << " differs from the bridge's output; regenerate it as this test's "
           "comment says";
  }
}

}  // namespace
}  // namespace studio
