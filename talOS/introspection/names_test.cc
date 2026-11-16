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
#include <tuple>
#include <utility>
#include <vector>

#include "gtest/gtest.h"
#include "schema_test/shapes_generated.h"
#include "talOS/schema/identity.h"

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

// REQ-CONTRACT-33: the one alert grammar. Telemetry tags a channel with the
// level it returns and the lint refuses an `alert_` field it does not accept,
// so a viewer never sees an alert of a level nobody defined.
TEST(AlertField, OneGrammarAcceptsThreeLevelsAndRefusesTheRest) {
  for (const auto& [field, level, name] : std::initializer_list<
           std::tuple<std::string_view, AlertLevel, std::string_view>>{
           {"alert_error_link_lost", AlertLevel::kError, "link_lost"},
           {"alert_warning_brownout", AlertLevel::kWarning, "brownout"},
           {"alert_info_homed", AlertLevel::kInfo, "homed"},
           {"alert_error_can2", AlertLevel::kError, "can2"},
           // The level is the first word: the rest is the name.
           {"alert_info_error_x", AlertLevel::kInfo, "error_x"},
       }) {
    const auto parsed = ParseAlertField(field);
    ASSERT_TRUE(parsed.has_value()) << field;
    EXPECT_EQ(parsed->level, level) << field;
    EXPECT_EQ(parsed->name, name) << field;
  }
  for (const auto* field : {
           "link_lost",               // no prefix
           "alerts_error_x",          // not the prefix
           "Alert_error_x",           // not the prefix
           "alert_error",             // no name
           "alert_error_",            // empty name
           "alert_fatal_x",           // unknown level
           "alert_errors_x",          // unknown level
           "alert_Error_x",           // unknown level
           "alert__error_x",          // unknown level
           "alert_warning_Hot",       // name not lowercase
           "alert_warning_2hot",      // name not starting with a letter
           "alert_warning_hot_",      // trailing underscore
           "alert_warning_too__hot",  // doubled underscore
           "alert_info_ok-ish",       // not a letter, digit or underscore
       }) {
    EXPECT_FALSE(ParseAlertField(field).has_value()) << field;
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
// REQ-CONTRACT-36, REQ-CONTRACT-09
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
// REQ-CONTRACT-09
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
// REQ-CONTRACT-09
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
// REQ-CONTRACT-09
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
// REQ-CONTRACT-09
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

// The bridge reads `node`'s hardware request, which `node` writes with
// `flags`, so the request itself is neither unread nor unwritten.
std::vector<NodeShape> ActuatorGraph(const std::string& node,
                                     std::uint32_t flags,
                                     std::vector<SourceShape> others = {}) {
  others.push_back(Sender(HardwareRequestTopic(node), 32,
                          kSourceFlagHardwareRequest | flags));
  return {{"hardware_node",
           "//talOS/bridge:hardware_node",
           {Watcher(HardwareRequestTopic(node))},
           /*hardware_bridge=*/true},
          {node, "//robot/" + node + ":node", std::move(others)}};
}

// The findings about `subject`.
std::vector<Diagnostic> About(const std::vector<Diagnostic>& diagnostics,
                              std::string_view subject) {
  std::vector<Diagnostic> out;
  for (const auto& diagnostic : diagnostics)
    if (diagnostic.subject == subject) out.push_back(diagnostic);
  return out;
}

// A node that opted out of the flush's live-target gate drives on
// robot_enabled alone: the lint names it, a warning while the graph does not
// require target coverage and an error while it does (commissioned = true
// implies it). It is then one finding: the no-target-lane warning, which
// the opt-out answers, stays silent.
// REQ-SAFE-42
TEST(LintGraph, UntargetedOutputsWarnAndErrorUnderTargetCoverage) {
  const auto nodes = ActuatorGraph("fixture", kSourceFlagUntargetedOutputs);
  for (const bool coverage : {false, true}) {
    SCOPED_TRACE(coverage ? "coverage required" : "coverage not required");
    const auto diagnostics = LintGraph(nodes, {}, {}, coverage);
    const auto findings = About(diagnostics, "fixture");
    ASSERT_EQ(findings.size(), 1u) << Describe(diagnostics);
    EXPECT_EQ(findings[0].severity,
              coverage ? Severity::ERROR : Severity::WARNING)
        << Describe(diagnostics);
    EXPECT_EQ(Errors(diagnostics), coverage ? 1 : 0) << Describe(diagnostics);
  }
}

// A node that writes its hardware request but reads no target lane never
// drives (the flush holds its outputs neutral without a live Target), so the
// lint warns, naming it, before anyone enables. A lane reader of its own, even
// of a lane nobody writes yet, silences it; so does having no outputs (no
// request). Another subsystem's lane is no consumer (a node's Target reads
// only its own lanes), nor is an observer's read of a lane.
// REQ-SAFE-41
TEST(LintGraph, AnActuatorNodeThatReadsNoTargetLaneIsWarned) {
  struct Case {
    const char* label;
    std::vector<NodeShape> nodes;
    bool warned;
  };
  for (const auto& [label, nodes, warned] : std::vector<Case>{
           {"request, no lane", ActuatorGraph("roller", 0), true},
           {"request and its teleop lane",
            ActuatorGraph(
                "roller", 0,
                {Watcher("/roller/target/teleop", 32, kSourceFlagOptional)}),
            false},
           {"request and another subsystem's lane",
            ActuatorGraph(
                "roller", 0,
                {Watcher("/arm/target/teleop", 32, kSourceFlagOptional)}),
            true},
           {"request and a lane read as an observer",
            ActuatorGraph("roller", 0,
                          {Watcher("/roller/target/teleop", 32,
                                   kSourceFlagOptional | kSourceFlagObserver)}),
            true},
           {"sensors only",
            {{"hardware_node",
              "//talOS/bridge:hardware_node",
              {Sender("/hw/state")},
              /*hardware_bridge=*/true},
             {"roller", "//robot/roller:node", {Watcher("/hw/state")}}},
            false},
       }) {
    SCOPED_TRACE(label);
    const auto diagnostics = LintGraph(nodes);
    EXPECT_FALSE(HasError(diagnostics)) << Describe(diagnostics);
    const auto findings = About(diagnostics, "roller");
    ASSERT_EQ(findings.size(), warned ? 1u : 0u) << Describe(diagnostics);
    if (warned) {
      EXPECT_EQ(findings[0].severity, Severity::WARNING);
      // The request it writes is named, since that is what makes it an
      // actuator node.
      EXPECT_NE(findings[0].message.find("/hw/request/roller"),
                std::string::npos)
          << findings[0].ToString();
    }
  }
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
// REQ-CONTRACT-09
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

// A topic no node reads warns; one an observer records (telemetry) is read,
// so it is no finding: a mechanism's state published for display is a
// design, not a dead end, and a warning on every such topic of a robot reads
// as a fault.
TEST(LintGraph, AnUnreadTopicWarnsUnlessAnObserverRecordsIt) {
  const NodeShape motor{
      "motor", "//robot/motor:node", {Sender("/motor/state")}};
  for (const auto& [name, nodes, warned] :
       std::vector<std::tuple<const char*, std::vector<NodeShape>, bool>>{
           {"no reader", {motor}, true},
           {"an observer only",
            {motor,
             {"telemetry",
              "//talOS/telemetry:node",
              {Watcher("/motor/state", 32, kSourceFlagObserver),
               Sender("/talos/telemetry", 32, kSourceFlagExternal)}}},
            false},
       }) {
    SCOPED_TRACE(name);
    const auto diagnostics = LintGraph(nodes);
    EXPECT_EQ(Find(diagnostics, Severity::WARNING, "/motor/state") != nullptr,
              warned)
        << Describe(diagnostics);
    EXPECT_EQ(diagnostics.size(), warned ? 1u : 0u) << Describe(diagnostics);
  }
}

// REQ-CONTRACT-33: from the writer's schema, an `alert_` field that is not a
// bool or not of a known level is an error naming the field (nested structs
// included), and a well-formed alert outside a `status` topic is a warning.
// A reader's end, which carries the same schema, adds nothing.
TEST(LintGraph, AlertFieldsAreBoolsOfAKnownLevelInAStatusTopic) {
  const auto with = [](SourceShape shape, const schema::Identity& identity) {
    shape.schema = identity.name;
    shape.schema_hash = identity.hash;
    shape.bfbs = identity.bfbs;
    return shape;
  };
  const schema::Identity bad = schema::Of<schema_test::BadAlerts>();
  const schema::Identity good = schema::Of<schema_test::Alerts>();
  const auto alert_findings = [](const std::vector<Diagnostic>& diagnostics,
                                 std::string_view topic) {
    std::vector<Diagnostic> out;
    for (const auto& diagnostic : diagnostics) {
      if (diagnostic.subject == topic &&
          diagnostic.message.find("alert") != std::string::npos) {
        out.push_back(diagnostic);
      }
    }
    return out;
  };

  {
    SCOPED_TRACE("malformed alerts in a status topic");
    const std::vector<NodeShape> nodes = {
        {"shooter",
         "//robot/shooter:node",
         {with(Sender("/shooter/status"), bad)}},
        {"odometry",
         "//robot/odometry:node",
         {with(Watcher("/shooter/status"), bad)}},
    };
    const auto findings = alert_findings(LintGraph(nodes), "/shooter/status");
    ASSERT_EQ(findings.size(), 2u) << Describe(findings);
    for (const auto* field :
         {"alert_error_fault_count", "alert_fatal_overheat"}) {
      const auto it = std::find_if(
          findings.begin(), findings.end(), [field](const Diagnostic& d) {
            return d.message.find(field) != std::string::npos;
          });
      ASSERT_NE(it, findings.end()) << field << Describe(findings);
      EXPECT_EQ(it->severity, Severity::ERROR) << field;
    }
  }
  {
    SCOPED_TRACE("well-formed alerts outside a status topic");
    const std::vector<NodeShape> nodes = {
        {"shooter",
         "//robot/shooter:node",
         {with(Sender("/shooter/state"), good)}},
        {"odometry",
         "//robot/odometry:node",
         {with(Watcher("/shooter/state"), good)}},
    };
    const auto findings = alert_findings(LintGraph(nodes), "/shooter/state");
    ASSERT_EQ(findings.size(), 3u) << Describe(findings);
    for (const auto& finding : findings) {
      EXPECT_EQ(finding.severity, Severity::WARNING) << finding.ToString();
    }
  }
  {
    SCOPED_TRACE("well-formed alerts in a status topic");
    const std::vector<NodeShape> nodes = {
        {"shooter",
         "//robot/shooter:node",
         {with(Sender("/shooter/status"), good)}},
        {"odometry",
         "//robot/odometry:node",
         {with(Watcher("/shooter/status"), good)}},
    };
    const auto diagnostics = LintGraph(nodes);
    EXPECT_TRUE(diagnostics.empty()) << Describe(diagnostics);
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

// `motors` Talon FX, `cancoders` CANcoders and `gyros` Pigeon 2 on `bus`.
void AddDevices(hardware::Config& config, const std::string& bus,
                std::size_t motors, std::size_t cancoders, std::size_t gyros) {
  for (std::size_t i = 0; i < motors; ++i) {
    hardware::MotorConfig motor;
    motor.bus = bus;
    config.motors.push_back(motor);
  }
  for (std::size_t i = 0; i < cancoders + gyros; ++i) {
    hardware::SensorConfig sensor;
    sensor.bus = bus;
    sensor.kind = i < cancoders ? hardware::SensorKind::kCANcoder
                                : hardware::SensorKind::kPigeon2;
    config.sensors.push_back(sensor);
  }
}

// Each CAN bus's load is estimated from its own devices and judged on its
// own: the bench's five devices on a CANivore stay far under 70 %, a full
// swerve-and-mechanisms load on one CANivore and ten motors on the
// RoboRIO's classic bus warn, naming the bus, and two CANivores each under
// the threshold do not warn however much they carry together.
// REQ-CONTRACT-37
TEST(HardwareConfig, EachCanBusIsWarnedAboveSeventyPercentOnItsOwnLoad) {
  struct Case {
    const char* what;
    std::vector<std::tuple<std::string, std::size_t, std::size_t, std::size_t>>
        buses;
    std::vector<std::string> warned;
  };
  const Case cases[] = {
      {"bench: 3 motors, a CANcoder and a Pigeon 2 on a CANivore",
       {{"Shooter", 3, 1, 1}},
       {}},
      {"20 motors, 4 CANcoders and a Pigeon 2 on one CANivore",
       {{"canivore", 20, 4, 1}},
       {"canivore"}},
      {"10 motors on the RoboRIO's bus", {{"rio", 10, 0, 0}}, {"rio"}},
      {"12 motors on each of two CANivores",
       {{"left", 12, 0, 0}, {"right", 12, 0, 0}},
       {}},
  };
  for (const auto& c : cases) {
    SCOPED_TRACE(c.what);
    hardware::Config config;
    config.status_hz = 200;
    for (const auto& [bus, motors, cancoders, gyros] : c.buses) {
      AddDevices(config, bus, motors, cancoders, gyros);
    }
    const auto diagnostics = CheckHardwareConfig(config);
    std::vector<std::string> warned;
    for (const auto& d : diagnostics) {
      EXPECT_EQ(d.severity, Severity::WARNING) << d.ToString();
      warned.push_back(d.subject);
      EXPECT_NE(d.message.find("0.70"), std::string::npos) << d.ToString();
    }
    EXPECT_EQ(warned, c.warned) << Describe(diagnostics);
  }
  // The estimate the warning names, calibrated to the bench's measured 0.16
  // to 0.18: 2 x (3 x 0.018 + 0.009 + 0.013) + 0.05.
  hardware::Config bench;
  bench.status_hz = 200;
  AddDevices(bench, "Shooter", 3, 1, 1);
  const auto loads = EstimateCanBusLoads(bench);
  ASSERT_EQ(loads.size(), 1u);
  EXPECT_NEAR(loads[0].estimate_frac, 0.202, 1e-9);
  // Ten motors on "rio": 2 x 10 x 0.041 + 0.05, named with its counts.
  hardware::Config rio;
  rio.status_hz = 200;
  AddDevices(rio, "rio", 10, 0, 0);
  const auto diagnostics = CheckHardwareConfig(rio);
  ASSERT_EQ(diagnostics.size(), 1u);
  EXPECT_NE(diagnostics[0].message.find("0.87"), std::string::npos)
      << diagnostics[0].message;
  EXPECT_NE(diagnostics[0].message.find("10 motors"), std::string::npos)
      << diagnostics[0].message;
}

}  // namespace
}  // namespace talos::introspect::naming
