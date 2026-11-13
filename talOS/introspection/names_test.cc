// The naming grammar (talOS/NAMING.md) and the graph lint. The rejected names
// include the ones that shipped in this repo once, so no edit to the rules can
// quietly re-admit them. The real robot's graph is checked by the launcher
// against its binaries (launcher_test).

#include "talOS/introspection/names.h"

#include <algorithm>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <map>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "gtest/gtest.h"

namespace talos::introspect::naming {
namespace {

using event::SourceKind;

TEST(TopicName, AcceptsAndRejectsPerTheNamingRules) {
  // 31 bytes is the transport's limit; 24 the segment limit.
  const std::string longest = "/drivetrain/state/" + std::string(13, 'x');
  ASSERT_EQ(longest.size(), 31u);
  const std::string longest_segment = "/" + std::string(24, 'a') + "/state";

  for (const std::string& name : {
           std::string{"/hw/state"},
           std::string{"/hw/command"},
           std::string{"/hw/state/driver_station"},
           std::string{"/hw/request/drivetrain"},
           std::string{"/drivetrain/state"},
           std::string{"/drivetrain/target"},
           std::string{"/drivetrain/target/teleop"},
           std::string{"/shooter/status"},
           std::string{"/shooter/event"},
           std::string{"/talos/telemetry"},
           std::string{"/a/state/b/c"},
           std::string{"/odometry2/state"},
           longest,
           longest_segment,
       }) {
    const auto problem = CheckTopicName(name);
    EXPECT_FALSE(problem.has_value()) << name << ": " << problem.value_or("");
  }

  for (const std::string& name : {
           // Form: leading '/', 2 to 4 non-empty segments.
           std::string{""},
           std::string{"drivetrain/state"},
           std::string{"/drivetrain/state/"},
           std::string{"/drivetrain//state"},
           std::string{"/odometry"},
           std::string{"/talos_studio"},
           std::string{"/a/state/b/c/d"},
           // Length: 31 bytes total, 24 per segment.
           longest + "x",
           "/" + std::string(25, 'a') + "/state",
           // Segment characters: lowercase start, [a-z0-9_], no __ or trailing
           // _.
           std::string{"/Drivetrain/state"},
           std::string{"/drive-train/state"},
           std::string{"/2026/state"},
           std::string{"/drivetrain__x/state"},
           std::string{"/drivetrain_/state"},
           // Role outside the closed set.
           std::string{"/drivetrain/pose"},
           // Abbreviations, in any segment.
           std::string{"/hw/req/drivetrain"},
           std::string{"/hw/request/drive"},
           std::string{"/hw/cmd"},
           std::string{"/hw/ds"},
           std::string{"/drivetrain/tgt"},
           std::string{"/dt/state"},
           std::string{"/odom/state"},
           std::string{"/shooter/state/vel"},
           // Direction suffixes.
           std::string{"/hw/command_in"},
           std::string{"/drivetrain/state_out"},
           std::string{"/shooter/state_pub"},
           std::string{"/shooter/target_sub"},
           std::string{"/odometry/state_tx"},
           std::string{"/odometry/state/vision_rx"},
           std::string{"/odometry/state_topic"},
       }) {
    EXPECT_TRUE(CheckTopicName(name).has_value()) << name;
  }
}

TEST(TargetLane, OneGrammarRecognisesAndBuildsLanes) {
  EXPECT_EQ(TargetLaneTopic("drivetrain", "teleop"),
            "/drivetrain/target/teleop");
  EXPECT_TRUE(IsTargetLane("/drivetrain/target/teleop"));

  // A source and subsystem must each be a segment, and the lane must fit the
  // transport's 31 bytes.
  for (const auto& [subsystem, source] :
       std::initializer_list<std::pair<std::string_view, std::string_view>>{
           {"drivetrain", "Teleop"},
           {"drivetrain", "tele/op"},
           {"drivetrain", ""},
           {"drivetrain", "cmd"},
           {"drivetrain", "auto_"},
           {"Drivetrain", "teleop"},
           {"drivetrain", "a_source_that_is_long"},
       }) {
    EXPECT_TRUE(CheckTargetLane(subsystem, source).has_value()) << source;
    EXPECT_THROW(TargetLaneTopic(subsystem, source), std::invalid_argument)
        << subsystem << " " << source;
  }

  // Not lanes, though each contains "target": the launcher's critical = false
  // rule for target producers must not apply to their writers.
  for (const auto* topic :
       {"/target/state", "/drivetrain/target", "/drivetrain/target/a/b",
        "/drivetrain/state/target/x", "/Drivetrain/target/teleop",
        "/drivetrain/target/teleop_"}) {
    EXPECT_FALSE(IsTargetLane(topic)) << topic;
  }
}

TEST(NodeName, FollowsTheSegmentRulesAndIsNotReserved) {
  for (const auto* name :
       {"drivetrain", "shooter", "driver_station", "hardware_node"}) {
    EXPECT_FALSE(CheckNodeName(name).has_value()) << name;
  }
  for (const auto* name : {"", "Drivetrain", "drive-train", "odom", "state_out",
                           "hw", "talos", "sim", "replay", "test"}) {
    EXPECT_TRUE(CheckNodeName(name).has_value()) << name;
  }
}

TEST(NodeTarget, MatchesThePackageLeafRepositoryOrRule) {
  struct Case {
    const char* name;
    const char* target;
    bool ok;
  };
  for (const auto& [name, target, ok] : std::vector<Case>{
           {"odometry", "//2026-robot/main_processor/odometry:node", true},
           {"hardware_node", "//talOS/bridge:hardware_node", true},
           {"wrist", "@wrist//:node", true},
           {"node", "@wrist//pkg:node", true},
           {"canon", "@@canon//:node", true},
           {"wrist", "@@wrist+//:node", true},
           {"shooter", "//2026-robot/main_processor/odometry:node", false},
           {"wrist", "@wrist//pkg:other", false},
           {"odometry", "@talos_odometry//:node", false},
           {"wrist", "wrist//:node", false},
           {"odometry", "odometry/node", false},
           {"odometry", "//2026-robot/main_processor/odometry", false},
       }) {
    EXPECT_EQ(!CheckNodeTarget(name, target).has_value(), ok)
        << name << " built by " << target;
  }
}

SourceShape Sender(std::string name, std::uint32_t bytes = 32,
                   std::uint32_t flags = 0) {
  return {SourceKind::SENDER, std::move(name), bytes, flags};
}
SourceShape Watcher(std::string name, std::uint32_t bytes = 32,
                    std::uint32_t flags = 0) {
  return {SourceKind::WATCHER, std::move(name), bytes, flags};
}

// The finding about `subject` at `severity`, if any.
const Diagnostic* Find(const std::vector<Diagnostic>& diagnostics,
                       Severity severity, std::string_view subject) {
  const auto it = std::find_if(diagnostics.begin(), diagnostics.end(),
                               [&](const Diagnostic& diagnostic) {
                                 return diagnostic.severity == severity &&
                                        diagnostic.subject == subject;
                               });
  return it == diagnostics.end() ? nullptr : &*it;
}

long Errors(const std::vector<Diagnostic>& diagnostics) {
  return std::count_if(diagnostics.begin(), diagnostics.end(),
                       [](const Diagnostic& diagnostic) {
                         return diagnostic.severity == Severity::ERROR;
                       });
}

std::string Describe(const std::vector<Diagnostic>& diagnostics) {
  std::string out;
  for (const auto& diagnostic : diagnostics) {
    out += "\n  " + diagnostic.ToString();
  }
  return out.empty() ? " (none)" : out;
}

// A linter that cries wolf is worse than none: a correct graph using every
// exemption (reserved namespaces, external and optional ends, observers,
// timer labels that would fail as topic names) produces no finding at all.
TEST(LintGraph, ACorrectGraphProducesNoFindings) {
  ASSERT_TRUE(CheckTopicName("swerve").has_value());
  const std::vector<NodeShape> nodes = {
      {"hardware_node",
       "//talOS/bridge:hardware_node",
       // /hw/* is published for a controller that is not a node, and
       // /hw/command is consumed over UDP.
       {Sender("/hw/state"), Sender("/hw/state/driver_station"),
        Sender("/hw/command", 32, kSourceFlagExternal),
        Watcher("/hw/request/drivetrain")},
       /*hardware_bridge=*/true},
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {{SourceKind::TIMER, "swerve", 0, 0},
        Watcher("/drivetrain/target/teleop", 32, kSourceFlagOptional),
        Watcher("/drivetrain/state/vision", 32, kSourceFlagOptional),
        Watcher("/hw/state"),
        Sender("/hw/request/drivetrain", 32, kSourceFlagHardwareRequest),
        Sender("/drivetrain/state")}},
      {"driver",
       "//2026-robot/main_processor/driver:node",
       {Watcher("/hw/state/driver_station"),
        Sender("/drivetrain/target/teleop")}},
      {"odometry",
       "//2026-robot/main_processor/odometry:node",
       {Watcher("/drivetrain/state"), Sender("/odometry/state")}},
      {"telemetry",
       "//talOS/telemetry:node",
       {Watcher("/drivetrain/state", 32, kSourceFlagObserver),
        Watcher("/odometry/state"),
        Sender("/talos/telemetry", 32, kSourceFlagExternal)}},
  };
  const auto diagnostics =
      LintGraph(nodes, {{"drivetrain", {"teleop", "auto"}}});
  EXPECT_TRUE(diagnostics.empty()) << Describe(diagnostics);
}

// Nine to 32 readers of one topic, observers included, lint clean; a 33rd is
// an error naming the topic and the limit. Nine is the first count the
// eight-slot table refused; 32 is about twice the Driver Station readers of
// a robot with six mechanisms.
// REQ-CONTRACT-36
TEST(LintGraph, ReaderLimitIncludesObserversAndAllowsExactlyTheLimit) {
  // Written out, not MAX_READERS, so that a smaller table fails here.
  constexpr std::size_t kRequiredReaders = 32;
  std::vector<NodeShape> nodes = {
      {"drivetrain", "//robot/drivetrain:node", {Sender("/drivetrain/state")}}};
  for (std::size_t i = 0; i < kRequiredReaders; ++i) {
    // Every other reader an observer: both kinds take a slot.
    const auto name = "reader_" + std::to_string(i);
    nodes.push_back(
        {name,
         "//robot/" + name + ":node",
         {Watcher("/drivetrain/state", 32,
                  i % 2 == 0 ? kSourceFlagObserver : std::uint32_t{0})}});
    if (nodes.size() - 1 < 9) continue;
    SCOPED_TRACE(std::to_string(nodes.size() - 1) + " readers");
    const auto diagnostics = LintGraph(nodes);
    EXPECT_FALSE(HasError(diagnostics)) << Describe(diagnostics);
  }
  nodes.push_back(
      {"consumer", "//robot/consumer:node", {Watcher("/drivetrain/state")}});
  const auto diagnostics = LintGraph(nodes);
  const auto* error = Find(diagnostics, Severity::ERROR, "/drivetrain/state");
  ASSERT_NE(error, nullptr) << Describe(diagnostics);
  EXPECT_NE(error->message.find(std::to_string(kRequiredReaders)),
            std::string::npos)
      << error->message;
}

// The defect this protocol exists to catch: both ends present, both valid
// names, different strings, so the reader is waiting on a topic nobody writes.
TEST(LintGraph, AReadTopicNobodyWritesIsAnError) {
  const std::vector<NodeShape> nodes = {
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {Sender("/hw/request/drive_train")}},
      {"hardware_node",
       "//talOS/bridge:hardware_node",
       {Watcher("/hw/request/drivetrain")}},
  };
  const auto diagnostics = LintGraph(nodes);
  EXPECT_NE(Find(diagnostics, Severity::ERROR, "/hw/request/drivetrain"),
            nullptr)
      << Describe(diagnostics);
}

// RULES.md 2: the roster's bridge writes the hw namespace, and
// /hw/request/<node> is written only by <node>'s devices-layer flush
// (kSourceFlagHardwareRequest), the one path that gates a request on
// robot_enabled and update(). A plain publisher on a node's own request, the
// flush aimed at another node's, or a node faking the bridge's state are each
// one error on that topic. The bridge is its roster slot, not its name: a
// replacement bridge binary may call itself anything, and a node calling
// itself hardware_node gains nothing.
TEST(LintGraph, OnlyTheBridgeAndEachNodesFlushWriteTheHwNamespace) {
  struct Case {
    const char* writer;
    bool bridge;
    SourceShape source;
    bool error;
  };
  for (const auto& [writer, bridge, source, error] : std::vector<Case>{
           {"drivetrain", false,
            Sender("/hw/request/drivetrain", 32, kSourceFlagHardwareRequest),
            false},
           {"drivetrain", false, Sender("/hw/request/drivetrain"), true},
           {"rogue", false,
            Sender("/hw/request/drivetrain", 32, kSourceFlagHardwareRequest),
            true},
           {"rogue", false, Sender("/hw/state"), true},
           {"hardware_node", false, Sender("/hw/state"), true},
           {"rio_bridge", true, Sender("/hw/state"), false},
           {"hardware_node", true, Sender("/hw/request/drivetrain"), true},
       }) {
    const std::vector<NodeShape> nodes = {
        {writer, std::string{"//robot/"} + writer + ":node", {source}, bridge},
        {"reader", "//robot/reader:node", {Watcher(source.name)}},
    };
    const auto diagnostics = LintGraph(nodes);
    const auto label = std::string{writer} + " writing " + source.name;
    ASSERT_EQ(Errors(diagnostics), error ? 1 : 0)
        << label << Describe(diagnostics);
    if (error) {
      EXPECT_NE(Find(diagnostics, Severity::ERROR, source.name), nullptr)
          << label << Describe(diagnostics);
    }
  }
}

// One writer per topic keeps the dispatch log replayable. Both writers are
// named, because one of them has to be moved to its own lane.
// REQ-CONTRACT-09: two writers on one topic are a lint error naming both.
TEST(LintGraph, TwoWritersIsAnErrorNamingBoth) {
  const std::vector<NodeShape> nodes = {
      {"autonomous",
       "//2026-robot/main_processor/autonomous:node",
       {Sender("/drivetrain/target/teleop")}},
      {"driver",
       "//2026-robot/main_processor/driver:node",
       {Sender("/drivetrain/target/teleop")}},
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {Watcher("/drivetrain/target/teleop")}},
  };
  const auto diagnostics = LintGraph(nodes);
  const auto* error =
      Find(diagnostics, Severity::ERROR, "/drivetrain/target/teleop");
  ASSERT_NE(error, nullptr) << Describe(diagnostics);
  EXPECT_NE(error->message.find("autonomous"), std::string::npos);
  EXPECT_NE(error->message.find("driver"), std::string::npos);
}

// The owner segment names the subject: state-like roles are published by it,
// target-like roles are published by anyone but it. Only the writing end makes
// that claim (ACorrectGraphProducesNoFindings reads others' topics).
TEST(LintGraph, OwnerRolesFlowTheRightWay) {
  struct Case {
    const char* writer;
    const char* topic;
  };
  for (const auto& [writer, topic] : std::vector<Case>{
           {"odometry", "/drivetrain/state"},
           {"drivetrain", "/drivetrain/target/teleop"},
       }) {
    const std::vector<NodeShape> nodes = {
        {writer, std::string{"//robot/"} + writer + ":node", {Sender(topic)}},
        {"reader", "//robot/reader:node", {Watcher(topic)}},
    };
    const auto diagnostics = LintGraph(nodes);
    EXPECT_NE(Find(diagnostics, Severity::ERROR, topic), nullptr)
        << writer << " writing " << topic << Describe(diagnostics);
  }
}

// A lane whose source no mode of the subsystem's policy lists is never read:
// a dead producer, refused before launch.
TEST(LintGraph, ALaneOutsideTheTargetPolicyIsAnError) {
  const std::vector<NodeShape> nodes = {
      {"aligner",
       "//2026-robot/main_processor/aligner:node",
       {Sender("/drivetrain/target/align")}},
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {Watcher("/drivetrain/target/teleop", 32, kSourceFlagOptional)}},
  };
  const auto diagnostics =
      LintGraph(nodes, {{"drivetrain", {"teleop", "auto"}}});
  EXPECT_NE(Find(diagnostics, Severity::ERROR, "/drivetrain/target/align"),
            nullptr)
      << Describe(diagnostics);

  // Listed: fine. A subsystem the policy map does not know is not checked.
  EXPECT_FALSE(HasError(
      LintGraph(nodes, {{"drivetrain", {"align", "teleop", "auto"}}})));
  EXPECT_FALSE(HasError(LintGraph(nodes, {{"shooter", {"teleop"}}})));
}

// Required coverage: each nonempty mode of a target-consuming subsystem's
// policy needs a writer on at least one of its lanes, judged per mode rather
// than over the union. Without it the gap is only a per-lane warning.
TEST(LintGraph, RequiredTargetCoverageIsCheckedPerMode) {
  const std::vector<NodeShape> nodes = {
      {"drivetrain",
       "//robot/drivetrain:node",
       {Watcher("/drivetrain/target/teleop", 32, kSourceFlagOptional),
        Watcher("/drivetrain/target/auto", 32, kSourceFlagOptional)}},
      {"driver", "//robot/driver:node", {Sender("/drivetrain/target/teleop")}},
  };
  const TargetSources sources{{"drivetrain", {"teleop", "auto", "pit"}}};
  struct Case {
    std::map<std::string, std::vector<std::string>, std::less<>> modes;
    const char* uncovered;  // nullptr: every mode is covered.
  };
  for (const auto& [modes, uncovered] : std::vector<Case>{
           // teleop is written; an empty mode needs nothing.
           {{{"teleop", {"teleop"}}, {"autonomous", {}}}, nullptr},
           // One written lane covers its mode.
           {{{"teleop", {"auto", "teleop"}}}, nullptr},
           // A declared but unwritten lane.
           {{{"teleop", {"teleop"}}, {"autonomous", {"auto"}}}, "autonomous"},
           // A lane the subsystem does not even read.
           {{{"teleop", {"teleop"}}, {"test", {"pit"}}}, "test"},
       }) {
    const auto diagnostics =
        LintGraph(nodes, sources, {{"drivetrain", modes}}, true);
    if (uncovered == nullptr) {
      EXPECT_FALSE(HasError(diagnostics)) << Describe(diagnostics);
      continue;
    }
    EXPECT_EQ(Errors(diagnostics), 1) << uncovered << Describe(diagnostics);
    const auto* error = Find(diagnostics, Severity::ERROR, "drivetrain");
    ASSERT_NE(error, nullptr) << uncovered << Describe(diagnostics);
    EXPECT_NE(error->message.find(uncovered), std::string::npos)
        << Describe(diagnostics);
  }

  const auto partial = LintGraph(
      nodes, sources,
      {{"drivetrain", {{"teleop", {"teleop"}}, {"autonomous", {"auto"}}}}},
      false);
  EXPECT_FALSE(HasError(partial)) << Describe(partial);
  EXPECT_NE(Find(partial, Severity::WARNING, "/drivetrain/target/auto"),
            nullptr)
      << Describe(partial);
}

SourceShape Typed(SourceShape shape, std::string schema, std::uint64_t hash,
                  std::uint32_t slots) {
  shape.schema = std::move(schema);
  shape.schema_hash = hash;
  shape.alignment = 8;
  shape.slots = slots;
  return shape;
}

// Both ends of a topic must agree on its type: by size when there is no schema
// (a registry snapshot), else by schema name and hash (a field can change type
// without changing size), and on the ring layout the segment is created with.
// A disagreement is exactly one error on the topic, naming both nodes.
TEST(LintGraph, BothEndsMustAgreeOnTypeAndRingLayout) {
  struct Case {
    std::uint32_t bytes;
    const char* schema;  // nullptr: an untyped (registry) shape.
    std::uint64_t hash;
    std::uint32_t slots;
    bool error;
  };
  for (const auto& [bytes, schema, hash, slots, error] : std::vector<Case>{
           {24, nullptr, 0, 0, false},
           {32, nullptr, 0, 0, true},
           {24, "shooter.State", 0x1, 1024, false},
           {24, "shooter.OldState", 0x2, 1024, true},
           {24, "shooter.State", 0x3, 1024, true},
           {24, "shooter.State", 0x1, 64, true},
       }) {
    SourceShape writer = Sender("/shooter/state", 24);
    SourceShape reader = Watcher("/shooter/state", bytes);
    if (schema != nullptr) {
      writer = Typed(writer, "shooter.State", 0x1, 1024);
      reader = Typed(reader, schema, hash, slots);
    }
    const std::vector<NodeShape> nodes = {
        {"shooter", "//robot/shooter:node", {writer}},
        {"odometry", "//robot/odometry:node", {reader}},
    };
    const auto diagnostics = LintGraph(nodes);
    const auto label = std::to_string(bytes) + " " + (schema ? schema : "-") +
                       " " + std::to_string(hash) + " " + std::to_string(slots);
    ASSERT_EQ(Errors(diagnostics), error ? 1 : 0)
        << label << Describe(diagnostics);
    if (!error) continue;
    const auto* finding = Find(diagnostics, Severity::ERROR, "/shooter/state");
    ASSERT_NE(finding, nullptr) << label << Describe(diagnostics);
    EXPECT_NE(finding->message.find("shooter"), std::string::npos);
    EXPECT_NE(finding->message.find("odometry"), std::string::npos);
  }
}

// What is normal during development or in a partial launch warns rather than
// refusing the launch: an unread topic, a policy lane nobody writes yet, an
// owner that is not in this session.
TEST(LintGraph, DevelopmentGapsAreWarningsNotErrors) {
  struct Case {
    std::vector<NodeShape> nodes;
    const char* subject;
  };
  for (const auto& [nodes, subject] : std::vector<Case>{
           {{{"telemetry",
              "//talOS/telemetry:node",
              {Sender("/talos/telemetry")}}},
            "/talos/telemetry"},
           {{{"drivetrain",
              "//robot/drivetrain:node",
              {Watcher("/drivetrain/target/auto", 32, kSourceFlagOptional)}}},
            "/drivetrain/target/auto"},
           {{{"drivetrain",
              "//robot/drivetrain:node",
              {Watcher("/shooterr/target")}},
             {"driver", "//robot/driver:node", {Sender("/shooterr/target")}}},
            "/shooterr/target"},
       }) {
    const auto diagnostics = LintGraph(nodes, {{"drivetrain", {"auto"}}});
    EXPECT_FALSE(HasError(diagnostics)) << subject << Describe(diagnostics);
    EXPECT_NE(Find(diagnostics, Severity::WARNING, subject), nullptr)
        << subject << Describe(diagnostics);
  }
}

TEST(LintGraph, ErrorsSortAheadOfWarnings) {
  const std::vector<NodeShape> nodes = {
      {"telemetry", "//talOS/telemetry:node", {Sender("/talos/telemetry")}},
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {Watcher("/drivetrain/target")}},
  };
  const auto diagnostics = LintGraph(nodes);
  ASSERT_GE(diagnostics.size(), 2u);
  EXPECT_EQ(diagnostics.front().severity, Severity::ERROR)
      << Describe(diagnostics);
  EXPECT_EQ(diagnostics.back().severity, Severity::WARNING)
      << Describe(diagnostics);
}

TEST(LintGraph, RejectsTheTestNamespaceAndDuplicateNodeNames) {
  const std::vector<NodeShape> in_test_namespace = {
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {Sender("/test/state")}},
  };
  EXPECT_NE(Find(LintGraph(in_test_namespace), Severity::ERROR, "/test/state"),
            nullptr);

  const std::vector<NodeShape> duplicated = {
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {Sender("/drivetrain/state")}},
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {Watcher("/drivetrain/state")}},
  };
  const auto diagnostics = LintGraph(duplicated);
  EXPECT_NE(Find(diagnostics, Severity::ERROR, "drivetrain"), nullptr)
      << Describe(diagnostics);
}

TEST(LintGraph, AMalformedNameDoesNotStopTheLint) {
  const std::vector<NodeShape> nodes = {
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {Sender("/drivetrain/tgt"), Sender("/drivetrain/state")}},
      {"odometry",
       "//2026-robot/main_processor/odometry:node",
       {Watcher("/drivetrain/state"), Sender("/odometry/state")}},
  };
  const auto diagnostics = LintGraph(nodes);
  const auto* error = Find(diagnostics, Severity::ERROR, "/drivetrain/tgt");
  ASSERT_NE(error, nullptr) << Describe(diagnostics);
  // The declaring node is named, because the reader has to go and fix it.
  EXPECT_NE(error->message.find("drivetrain"), std::string::npos);
  // The rest of the graph is still checked: /odometry/state is unread.
  EXPECT_NE(Find(diagnostics, Severity::WARNING, "/odometry/state"), nullptr)
      << Describe(diagnostics);
}

TEST(HardwareConfig, SensorVelocityLimitIsAdvisoryAndUsesMechanismRatio) {
  hardware::Config config;
  hardware::MotorConfig motor;
  motor.name = "wrist";
  motor.max_velocity_rps = 21;
  motor.sensor_to_mechanism_ratio = 5;
  motor.rotor_to_sensor_ratio = 100;
  config.motors.push_back(motor);
  EXPECT_TRUE(CheckHardwareConfig(config).empty());

  config.motors.front().max_velocity_rps = 21.1;
  const auto diagnostics = CheckHardwareConfig(config);
  ASSERT_EQ(diagnostics.size(), 1u);
  EXPECT_NE(Find(diagnostics, Severity::WARNING, "wrist"), nullptr)
      << Describe(diagnostics);
}

TEST(HardwareConfig, RioTrafficCountsMotorAndSensorFramesAtStatusFrequency) {
  hardware::Config config;
  config.motors.resize(3);
  config.sensors.resize(4);
  config.status_hz = 750;
  EXPECT_TRUE(CheckHardwareConfig(config).empty());

  config.status_hz = 751;
  const auto diagnostics = CheckHardwareConfig(config);
  ASSERT_EQ(diagnostics.size(), 1u);
  EXPECT_NE(Find(diagnostics, Severity::WARNING, "rio"), nullptr)
      << Describe(diagnostics);
}

TEST(HardwareConfig, OtherCanBusesDoNotCountTowardRioTraffic) {
  hardware::Config config;
  config.status_hz = 1000;
  config.motors.resize(3);
  config.sensors.resize(2);
  ASSERT_EQ(CheckHardwareConfig(config).size(), 1u);
  config.motors.front().bus = "canivore";
  EXPECT_TRUE(CheckHardwareConfig(config).empty());
  config.motors.front().bus = "rio";
  config.sensors.front().bus = "canivore";
  EXPECT_TRUE(CheckHardwareConfig(config).empty());
}

}  // namespace
}  // namespace talos::introspect::naming
