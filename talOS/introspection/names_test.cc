// The naming grammar (talOS/NAMING.md) and the graph lint. The rejected names
// include the ones that shipped in this repo once, so no edit to the rules can
// quietly re-admit them. The real robot's graph is checked by the launcher
// against its binaries (launcher_test).

#include "talOS/introspection/names.h"

#include <algorithm>
#include <cctype>
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

#include "flatbuffers/reflection.h"
#include "gtest/gtest.h"
#include "schema_test/shapes_generated.h"
#include "talOS/hardware/packet.h"
#include "talOS/introspection/hardware_lint.h"
#include "talOS/rtms/rtms.h"
#include "talOS/schema/identity.h"

namespace talos::introspect::naming {
namespace {

static_assert(std::string_view{hardware::kRequestTopicPrefix} ==
                  kRequestTopicPrefix,
              "naming::kRequestTopicPrefix must match "
              "hardware::kRequestTopicPrefix");
static_assert(std::string_view{hardware::kDriverStationTopic} ==
                  kDriverStationTopic,
              "naming::kDriverStationTopic must match "
              "hardware::kDriverStationTopic");
static_assert(
    std::string_view{kDriverStationTopic} == "/hw/state/driver_station",
    "naming::kDriverStationTopic must spell /hw/state/driver_station");
static_assert(kMaxTopicReaders == MAX_READERS,
              "naming::kMaxTopicReaders must match rtms MAX_READERS");

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
           {"arm", "@@//arm:node", true},
           {"node", "@@//:node", true},
           {"wrist", "@@wrist~//:node", true},
           {"wrist", "@@wrist+1.0//:node", true},
           {"wrist", "@@wrist~1.2.3//:node", true},
           {"wrist", "@@wrist~override+ext//:node", true},
           {"arm", "@//arm:node", false},
           {"arm", "@@//:node", false},
           {"wrist", "@@+//:node", false},
           {"wrist", "@@~//:node", false},
           {"wrist", "@@wrist+bad/seg//:node", false},
           {"wrist", "@@wrist+bad:seg//:node", false},
           {"wrist", "@@@wrist//:node", false},
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

// Whether `message` names `identifier` as a whole word (a node or topic
// segment), so a test checks who a finding names, not its sentence.
bool Names(std::string_view message, std::string_view identifier) {
  const auto word = [](char c) {
    return std::isalnum(static_cast<unsigned char>(c)) != 0 || c == '_';
  };
  for (auto at = message.find(identifier); at != std::string_view::npos;
       at = message.find(identifier, at + 1)) {
    const auto end = at + identifier.size();
    if ((at == 0 || !word(message[at - 1])) &&
        (end == message.size() || !word(message[end])))
      return true;
  }
  return false;
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
        // Simulation ground truth, absent from a robot session.
        Watcher("/sim/state", 32, kSourceFlagOptional),
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

// REQ-SAFE-27: ShmObjectName strips all leading '/' and maps interior '/' to
// '.', so IsHardwareRequest must recognise multiple-leading-slash and
// dot-colliding spellings so Node::publish / publish_bytes refuse them all.
TEST(TopicName, IsHardwareRequestRecognisesLeadingSlashesAndDotCollisions) {
  for (const std::string_view topic : {
           "/hw/request",
           "/hw/request/foo",
           "//hw/request",
           "//hw/request/foo",
           "///hw/request/foo",
           "////hw/request/foo/bar",
           "/hw.request",
           "/hw.request.foo",
           "//hw.request.foo",
           "/hw.request/foo",
           "/hw/request.foo",
       }) {
    EXPECT_TRUE(IsHardwareRequest(topic)) << topic;
  }
  for (const std::string_view topic : {
           "",
           "/",
           "//",
           "///",
           "hw/request/foo",
           "/hw/state",
           "/hw/requests/foo",
           "/other/request/foo",
           "/hw/request_other/foo",
       }) {
    EXPECT_FALSE(IsHardwareRequest(topic)) << topic;
  }
}

// RULES.md 2: the roster's bridge writes the hw namespace, and
// /hw/request/<node> is written only by <node>'s devices-layer flush
// (kSourceFlagHardwareRequest), the one path that gates a request on
// robot_enabled and update(). A plain publisher on a node's own request, the
// flush aimed at another node's, or a node faking the bridge's state are each
// one error on that topic. The bridge is its roster slot, not its name: a
// replacement bridge binary may call itself anything, and a node calling
// itself hardware_node gains nothing.
// REQ-CONTRACT-09, REQ-SAFE-27
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
                  std::uint32_t slots, std::uint32_t alignment = 8) {
  shape.schema = std::move(schema);
  shape.schema_hash = hash;
  shape.alignment = alignment;
  shape.slots = slots;
  return shape;
}

// Both ends of a topic must agree on its type: by size when there is no schema
// (a registry snapshot) or when both share a schema (e.g. talos.Bytes), else by
// schema name and hash (a field can change type without changing size), and on
// the ring layout (slots and alignment) the segment is created with.
// A disagreement is exactly one error on the topic, naming both nodes.
// REQ-CONTRACT-09
TEST(LintGraph, BothEndsMustAgreeOnTypeAndRingLayout) {
  struct Case {
    std::uint32_t bytes;
    const char* writer_schema;  // nullptr: an untyped (registry) shape.
    const char* reader_schema;
    std::uint64_t writer_hash;
    std::uint64_t reader_hash;
    std::uint32_t alignment;
    std::uint32_t slots;
    bool error;
  };
  const std::uint64_t bytes_hash = schema::BytesHash();
  for (const auto& [bytes, writer_schema, reader_schema, writer_hash,
                    reader_hash, alignment, slots, error] : std::vector<Case>{
           {24, nullptr, nullptr, 0, 0, 8, 0, false},
           {32, nullptr, nullptr, 0, 0, 8, 0, true},
           {24, "shooter.State", "shooter.State", 0x1, 0x1, 8, 1024, false},
           {24, "shooter.State", "shooter.OldState", 0x1, 0x2, 8, 1024, true},
           // Schema name-only mismatch with equal hash.
           {24, "shooter.State", "shooter.OldState", 0x1, 0x1, 8, 1024, true},
           // Schema hash-only mismatch with equal name.
           {24, "shooter.State", "shooter.State", 0x1, 0x3, 8, 1024, true},
           // Slot count mismatch with equal schema, size and alignment.
           {24, "shooter.State", "shooter.State", 0x1, 0x1, 8, 64, true},
           // Alignment-only mismatch with equal schema, size and slots.
           {24, "shooter.State", "shooter.State", 0x1, 0x1, 4, 1024, true},
           // Byte-topic ends with equal schema/hash must still agree on size.
           {24, "talos.Bytes", "talos.Bytes", bytes_hash, bytes_hash, 8, 1024,
            false},
           {32, "talos.Bytes", "talos.Bytes", bytes_hash, bytes_hash, 8, 1024,
            true},
       }) {
    SourceShape writer = Sender("/shooter/state", 24);
    SourceShape reader = Watcher("/shooter/state", bytes);
    if (writer_schema != nullptr) {
      writer = Typed(writer, writer_schema, writer_hash, 1024, 8);
      reader = Typed(reader, reader_schema, reader_hash, slots, alignment);
    }
    const std::vector<NodeShape> nodes = {
        {"shooter", "//robot/shooter:node", {writer}},
        {"odometry", "//robot/odometry:node", {reader}},
    };
    const auto diagnostics = LintGraph(nodes);
    const auto label = std::to_string(bytes) + " " +
                       (reader_schema ? reader_schema : "-") + " " +
                       std::to_string(reader_hash) + " " +
                       std::to_string(alignment) + " " + std::to_string(slots);
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

// REQ-CONTRACT-09: a topic no node reads warns. One an observer records
// (telemetry) is read when the owner publishes it (state, status, event,
// telemetry): a mechanism's state published for display is a design, not a
// dead end, and a warning on every such topic of a robot reads as a fault.
// An instruction (target, request, command) only an observer reads still
// warns: telemetry observes every topic, so otherwise a writer whose only
// reader subscribed `optional` to a misspelled topic has no finding at all.
TEST(LintGraph, AnUnreadTopicWarnsUnlessAnObserverRecordsItsOwnersFeed) {
  const NodeShape motor{
      "motor", "//robot/motor:node", {Sender("/motor/state")}};
  const auto telemetry = [](const char* observed) {
    return NodeShape{"telemetry",
                     "//talOS/telemetry:node",
                     {Watcher(observed, 32, kSourceFlagObserver),
                      Sender("/talos/telemetry", 32, kSourceFlagExternal)}};
  };
  for (const auto& [name, nodes, subject, warned] : std::vector<
           std::tuple<const char*, std::vector<NodeShape>, const char*, bool>>{
           {"no reader", {motor}, "/motor/state", true},
           {"an observer only",
            {motor, telemetry("/motor/state")},
            "/motor/state",
            false},
           {"an instruction its reader misspelled, observed",
            {telemetry("/arm/command"),
             {"planner", "//robot/planner:node", {Sender("/arm/command")}},
             {"arm",
              "//robot/arm:node",
              {Watcher("/arm/command/goal", 32, kSourceFlagOptional)}}},
            "/arm/command",
            true},
       }) {
    SCOPED_TRACE(name);
    const auto diagnostics = LintGraph(nodes);
    EXPECT_EQ(Find(diagnostics, Severity::WARNING, subject) != nullptr, warned)
        << Describe(diagnostics);
    EXPECT_EQ(diagnostics.size(), warned ? 1u : 0u) << Describe(diagnostics);
  }
}

// REQ-CONTRACT-09: the owner-published side of the case above. An `optional`
// read of a state topic nobody writes is no error (the reader tolerates its
// absence), and the topic the owner does write is recorded by telemetry, so
// it has no finding either; only the owner may write a state topic, so when
// the owner is a node in this session and writes no such topic, the read is
// misspelled and warns, naming the reader and the owner. With the owner
// outside the session the one finding stays the owner warning.
TEST(LintGraph, AnOptionalReadOfAFeedItsOwnerDoesNotPublishWarns) {
  const NodeShape motor{
      "motor", "//robot/motor:node", {Sender("/motor/state")}};
  const NodeShape telemetry{
      "telemetry",
      "//talOS/telemetry:node",
      {Watcher("/motor/state", 32, kSourceFlagObserver),
       Sender("/talos/telemetry", 32, kSourceFlagExternal)}};
  const NodeShape arm{"arm",
                      "//robot/arm:node",
                      {Watcher("/motor/state/wheel", 32, kSourceFlagOptional)}};
  {
    SCOPED_TRACE("owner in the session");
    const auto diagnostics = LintGraph({motor, telemetry, arm});
    EXPECT_FALSE(HasError(diagnostics)) << Describe(diagnostics);
    const auto* warning =
        Find(diagnostics, Severity::WARNING, "/motor/state/wheel");
    ASSERT_NE(warning, nullptr) << Describe(diagnostics);
    EXPECT_TRUE(Names(warning->message, "arm")) << warning->message;
    EXPECT_TRUE(Names(warning->message, "motor")) << warning->message;
    EXPECT_EQ(diagnostics.size(), 1u) << Describe(diagnostics);
  }
  {
    SCOPED_TRACE("owner outside the session");
    const auto diagnostics = LintGraph({arm});
    const auto* warning =
        Find(diagnostics, Severity::WARNING, "/motor/state/wheel");
    ASSERT_NE(warning, nullptr) << Describe(diagnostics);
    EXPECT_NE(warning->message.find("not a node in this session"),
              std::string::npos)
        << warning->message;
    EXPECT_EQ(diagnostics.size(), 1u) << Describe(diagnostics);
  }
}

// REQ-CONTRACT-09: `optional` is one reader's tolerance, so it exempts only
// that reader: a required read of a topic nothing writes is an error naming
// the required reader even when another reader of it declared it optional.
// (Among readers that are not observers; telemetry's observer read here
// requires nothing.) With every reader optional the owner warning above is
// unchanged. `external`
// is a fact about the topic (written outside the graph), so one end
// declaring it exempts every reader.
TEST(LintGraph, AnOptionalReaderExemptsOnlyItselfFromTheUnwrittenError) {
  const NodeShape motor{
      "motor", "//robot/motor:node", {Sender("/motor/state")}};
  const NodeShape telemetry{
      "telemetry",
      "//talOS/telemetry:node",
      {Watcher("/motor/state", 32, kSourceFlagObserver),
       Sender("/talos/telemetry", 32, kSourceFlagExternal)}};
  const auto reader = [](const char* name, std::uint32_t flags) {
    return NodeShape{name,
                     std::string("//robot/") + name + ":node",
                     {Watcher("/motor/state/wheel", 32, flags)}};
  };
  {
    SCOPED_TRACE("one required, one optional");
    const auto diagnostics = LintGraph({motor, telemetry, reader("arm", 0),
                                        reader("panel", kSourceFlagOptional)});
    const auto* error =
        Find(diagnostics, Severity::ERROR, "/motor/state/wheel");
    ASSERT_NE(error, nullptr) << Describe(diagnostics);
    EXPECT_TRUE(Names(error->message, "arm")) << error->message;
    EXPECT_FALSE(Names(error->message, "panel")) << error->message;
    EXPECT_EQ(Errors(diagnostics), 1) << Describe(diagnostics);
  }
  {
    SCOPED_TRACE("every reader optional");
    const auto diagnostics =
        LintGraph({motor, telemetry, reader("arm", kSourceFlagOptional),
                   reader("panel", kSourceFlagOptional)});
    EXPECT_FALSE(HasError(diagnostics)) << Describe(diagnostics);
    EXPECT_NE(Find(diagnostics, Severity::WARNING, "/motor/state/wheel"),
              nullptr)
        << Describe(diagnostics);
  }
  {
    SCOPED_TRACE("one required, one external");
    const auto diagnostics = LintGraph({motor, telemetry, reader("arm", 0),
                                        reader("panel", kSourceFlagExternal)});
    EXPECT_TRUE(diagnostics.empty()) << Describe(diagnostics);
  }
  {
    SCOPED_TRACE("observer only on unwritten topic");
    const NodeShape bad_telemetry{
        "telemetry",
        "//talOS/telemetry:node",
        {Watcher("/motor/state", 32, kSourceFlagObserver),
         Watcher("/motor/state/wheel", 32, kSourceFlagObserver),
         Sender("/talos/telemetry", 32, kSourceFlagExternal)}};
    const auto diagnostics = LintGraph({motor, bad_telemetry});
    const auto* error =
        Find(diagnostics, Severity::ERROR, "/motor/state/wheel");
    ASSERT_NE(error, nullptr) << Describe(diagnostics);
    EXPECT_TRUE(Names(error->message, "telemetry")) << error->message;
    EXPECT_EQ(error->message.find("is read by  but"), std::string::npos)
        << error->message;
    EXPECT_EQ(Errors(diagnostics), 1) << Describe(diagnostics);
  }
  {
    SCOPED_TRACE("regular reader and observer on unwritten topic");
    const NodeShape bad_telemetry{
        "telemetry",
        "//talOS/telemetry:node",
        {Watcher("/motor/state", 32, kSourceFlagObserver),
         Watcher("/motor/state/wheel", 32, kSourceFlagObserver),
         Sender("/talos/telemetry", 32, kSourceFlagExternal)}};
    const auto diagnostics =
        LintGraph({motor, bad_telemetry, reader("arm", 0)});
    const auto* error =
        Find(diagnostics, Severity::ERROR, "/motor/state/wheel");
    ASSERT_NE(error, nullptr) << Describe(diagnostics);
    EXPECT_TRUE(Names(error->message, "arm")) << error->message;
    EXPECT_TRUE(Names(error->message, "telemetry")) << error->message;
    EXPECT_EQ(Errors(diagnostics), 1) << Describe(diagnostics);
  }
  {
    SCOPED_TRACE("observer plus optional reader stays non-error");
    const NodeShape bad_telemetry{
        "telemetry",
        "//talOS/telemetry:node",
        {Watcher("/motor/state", 32, kSourceFlagObserver),
         Watcher("/motor/state/wheel", 32, kSourceFlagObserver),
         Sender("/talos/telemetry", 32, kSourceFlagExternal)}};
    const auto diagnostics =
        LintGraph({motor, bad_telemetry, reader("panel", kSourceFlagOptional)});
    EXPECT_FALSE(HasError(diagnostics)) << Describe(diagnostics);
    EXPECT_NE(Find(diagnostics, Severity::WARNING, "/motor/state/wheel"),
              nullptr)
        << Describe(diagnostics);
  }
}

// REQ-CONTRACT-33: from the writer's schema, an `alert_` field that is not a
// bool, not of a known level, or outside a `status` topic is an error naming
// the field (nested structs included). A reader's end, which carries the same
// schema, adds nothing.
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
      EXPECT_EQ(finding.severity, Severity::ERROR) << finding.ToString();
    }
  }
  {
    SCOPED_TRACE("nested alerts outside a status topic");
    const std::vector<NodeShape> nodes = {
        {"shooter",
         "//robot/shooter:node",
         {with(Sender("/shooter/state"), bad)}},
        {"odometry",
         "//robot/odometry:node",
         {with(Watcher("/shooter/state"), bad)}},
    };
    const auto findings = alert_findings(LintGraph(nodes), "/shooter/state");
    ASSERT_EQ(findings.size(), 5u) << Describe(findings);
    for (const auto* field :
         {"alert_error_fault_count", "alert_fatal_overheat",
          "nested.alert_error_link_lost", "nested.alert_warning_hot",
          "nested.alert_info_homed"}) {
      const auto it = std::find_if(
          findings.begin(), findings.end(), [field](const Diagnostic& d) {
            return d.message.find(field) != std::string::npos;
          });
      ASSERT_NE(it, findings.end()) << field << Describe(findings);
      EXPECT_EQ(it->severity, Severity::ERROR) << field;
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
  {
    // REQ-CONTRACT-09: corrupt or mismatched bfbs must fail closed with
    // Severity::ERROR rather than silently bypassing alert inspection.
    SCOPED_TRACE("corrupt bfbs is an error");
    SourceShape corrupt = Sender("/shooter/status");
    corrupt.schema = good.name;
    corrupt.schema_hash = good.hash;
    corrupt.bfbs = "not-a-valid-bfbs";
    const std::vector<NodeShape> nodes = {
        {"shooter", "//robot/shooter:node", {corrupt}},
        {"odometry", "//robot/odometry:node", {Watcher("/shooter/status")}},
    };
    const auto diagnostics = LintGraph(nodes);
    const auto* err = Find(diagnostics, Severity::ERROR, "/shooter/status");
    ASSERT_NE(err, nullptr) << Describe(diagnostics);
    EXPECT_TRUE(Names(err->message, "shooter")) << err->message;
  }
  {
    SCOPED_TRACE("bfbs missing declared root struct is an error");
    SourceShape mismatched = Sender("/shooter/status");
    mismatched.schema = "schema_test.DoesNotExist";
    mismatched.schema_hash = good.hash;
    mismatched.bfbs = good.bfbs;
    const std::vector<NodeShape> nodes = {
        {"shooter", "//robot/shooter:node", {mismatched}},
        {"odometry", "//robot/odometry:node", {Watcher("/shooter/status")}},
    };
    const auto diagnostics = LintGraph(nodes);
    const auto* err = Find(diagnostics, Severity::ERROR, "/shooter/status");
    ASSERT_NE(err, nullptr) << Describe(diagnostics);
    EXPECT_TRUE(Names(err->message, "shooter")) << err->message;
  }
  {
    // REQ-CONTRACT-09, REQ-OPS-17: target lanes wrap their payload struct in
    // talos.TargetEnvelope<M> while carrying M's bfbs. AlertPrefixedFields
    // unwraps the envelope so valid target lanes pass, illegal alerts inside
    // target payloads are caught, and malformed/missing inner structs fail
    // closed.
    const schema::Identity plain_target =
        schema::Envelope(schema::Of<schema_test::Mixed>());
    {
      SCOPED_TRACE("valid target lane with TargetEnvelope passes");
      const std::vector<NodeShape> nodes = {
          {"driver",
           "//robot/driver:node",
           {with(Sender("/shooter/target/teleop"), plain_target)}},
          {"shooter",
           "//robot/shooter:node",
           {with(Watcher("/shooter/target/teleop"), plain_target)}},
      };
      const auto diagnostics = LintGraph(nodes, {{"shooter", {"teleop"}}});
      EXPECT_TRUE(diagnostics.empty()) << Describe(diagnostics);
    }
    {
      SCOPED_TRACE("alert fields inside TargetEnvelope payload are rejected");
      const schema::Identity alert_target = schema::Envelope(good);
      const std::vector<NodeShape> nodes = {
          {"driver",
           "//robot/driver:node",
           {with(Sender("/shooter/target/teleop"), alert_target)}},
          {"shooter",
           "//robot/shooter:node",
           {with(Watcher("/shooter/target/teleop"), alert_target)}},
      };
      const auto diagnostics = LintGraph(nodes, {{"shooter", {"teleop"}}});
      const auto findings =
          alert_findings(diagnostics, "/shooter/target/teleop");
      ASSERT_EQ(findings.size(), 3u) << Describe(diagnostics);
      for (const auto* field :
           {"alert_error_link_lost", "alert_warning_hot", "alert_info_homed"}) {
        const auto it = std::find_if(
            findings.begin(), findings.end(), [field](const Diagnostic& d) {
              return d.message.find(field) != std::string::npos;
            });
        ASSERT_NE(it, findings.end()) << field << Describe(findings);
        EXPECT_EQ(it->severity, Severity::ERROR) << field;
      }
    }
    for (const auto* bad_envelope :
         {"talos.TargetEnvelope<schema_test.DoesNotExist>",
          "talos.TargetEnvelope<>"}) {
      SCOPED_TRACE(std::string{"malformed envelope fails closed: "} +
                   bad_envelope);
      SourceShape bad_sender = Sender("/shooter/target/teleop");
      bad_sender.schema = bad_envelope;
      bad_sender.schema_hash = plain_target.hash;
      bad_sender.bfbs = plain_target.bfbs;
      SourceShape bad_watcher = Watcher("/shooter/target/teleop");
      bad_watcher.schema = bad_envelope;
      bad_watcher.schema_hash = plain_target.hash;
      bad_watcher.bfbs = plain_target.bfbs;
      const std::vector<NodeShape> nodes = {
          {"driver", "//robot/driver:node", {bad_sender}},
          {"shooter", "//robot/shooter:node", {bad_watcher}},
      };
      const auto diagnostics = LintGraph(nodes, {{"shooter", {"teleop"}}});
      const auto* err =
          Find(diagnostics, Severity::ERROR, "/shooter/target/teleop");
      ASSERT_NE(err, nullptr) << Describe(diagnostics);
      EXPECT_TRUE(Names(err->message, "driver")) << err->message;
    }
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

// REQ-CONTRACT-09, REQ-RT-16: an operator producer (a non-bridge node reading
// /hw/state/driver_station and writing a target lane in a subsystem's teleop
// policy) reaching an actuator owner in more than 2 node hops over non-/hw,
// non-observer command-flow edges (target/request/command; teleop-policy target
// lanes out of the producer; not expanding past an actuator owner) is a warning
// naming the producer, the actuator owner, the hop count, and the shortest node
// chain.
TEST(LintGraph, OperatorToActuatorPathOverTwoHopsWarns) {
  const auto bridge = [](std::vector<SourceShape> requests) {
    requests.push_back(Sender("/hw/state/driver_station"));
    return NodeShape{"hardware_node", "//talOS/bridge:hardware_node",
                     std::move(requests), /*hardware_bridge=*/true};
  };

  {
    SCOPED_TRACE("1-hop direct teleop -> arm is clean");
    const std::vector<NodeShape> nodes = {
        bridge({Watcher("/hw/request/arm")}),
        {"teleop",
         "//robot/teleop:node",
         {Watcher("/hw/state/driver_station"),
          Sender("/arm/target/teleop", 32, kSourceFlagOperatorArming)}},
        {"arm",
         "//robot/arm:node",
         {Watcher("/arm/target/teleop"),
          Sender("/hw/request/arm", 32, kSourceFlagHardwareRequest)}},
    };
    const TargetModeSources modes{{"arm", {{"teleop", {"teleop"}}}}};
    const auto diagnostics = LintGraph(nodes, {{"arm", {"teleop"}}}, modes);
    EXPECT_TRUE(diagnostics.empty()) << Describe(diagnostics);
  }

  {
    SCOPED_TRACE("2-hop teleop -> planner -> arm is clean");
    const std::vector<NodeShape> nodes = {
        bridge({Watcher("/hw/request/arm")}),
        {"teleop",
         "//robot/teleop:node",
         {Watcher("/hw/state/driver_station"),
          Sender("/planner/target/teleop", 32, kSourceFlagOperatorArming)}},
        {"planner",
         "//robot/planner:node",
         {Watcher("/planner/target/teleop"), Sender("/arm/target/planner")}},
        {"arm",
         "//robot/arm:node",
         {Watcher("/arm/target/planner"),
          Sender("/hw/request/arm", 32, kSourceFlagHardwareRequest)}},
    };
    const TargetModeSources modes{
        {"planner", {{"teleop", {"teleop"}}}},
        {"arm", {{"teleop", {"planner"}}}},
    };
    const auto diagnostics = LintGraph(
        nodes, {{"planner", {"teleop"}}, {"arm", {"planner"}}}, modes);
    EXPECT_TRUE(diagnostics.empty()) << Describe(diagnostics);
  }

  {
    SCOPED_TRACE("3-hop teleop -> planner -> coordinator -> arm warns");
    const std::vector<NodeShape> nodes = {
        bridge({Watcher("/hw/request/arm")}),
        {"teleop",
         "//robot/teleop:node",
         {Watcher("/hw/state/driver_station"),
          Sender("/planner/target/teleop", 32, kSourceFlagOperatorArming)}},
        {"planner",
         "//robot/planner:node",
         {Watcher("/planner/target/teleop"),
          Sender("/coordinator/target/planner")}},
        {"coordinator",
         "//robot/coordinator:node",
         {Watcher("/coordinator/target/planner"),
          Sender("/arm/target/coordinator")}},
        {"arm",
         "//robot/arm:node",
         {Watcher("/arm/target/coordinator"),
          Sender("/hw/request/arm", 32, kSourceFlagHardwareRequest)}},
    };
    const TargetModeSources modes{
        {"planner", {{"teleop", {"teleop"}}}},
        {"coordinator", {{"teleop", {"planner"}}}},
        {"arm", {{"teleop", {"coordinator"}}}},
    };
    const auto diagnostics = LintGraph(nodes,
                                       {{"planner", {"teleop"}},
                                        {"coordinator", {"planner"}},
                                        {"arm", {"coordinator"}}},
                                       modes);
    EXPECT_FALSE(HasError(diagnostics)) << Describe(diagnostics);
    const auto* warning = Find(diagnostics, Severity::WARNING, "teleop");
    ASSERT_NE(warning, nullptr) << Describe(diagnostics);
    EXPECT_TRUE(Names(warning->message, "teleop")) << warning->message;
    EXPECT_TRUE(Names(warning->message, "arm")) << warning->message;
    EXPECT_NE(warning->message.find('3'), std::string::npos)
        << warning->message;
    EXPECT_NE(warning->message.find("teleop -> planner -> coordinator -> arm"),
              std::string::npos)
        << warning->message;
    EXPECT_EQ(diagnostics.size(), 1u) << Describe(diagnostics);
  }

  {
    SCOPED_TRACE(
        "autonomous DS reader writing only an auto lane does not warn");
    const std::vector<NodeShape> nodes = {
        bridge({Watcher("/hw/request/arm")}),
        {"autonomous",
         "//robot/autonomous:node",
         {Watcher("/hw/state/driver_station"), Sender("/planner/target/auto")}},
        {"planner",
         "//robot/planner:node",
         {Watcher("/planner/target/auto"),
          Sender("/coordinator/target/planner")}},
        {"coordinator",
         "//robot/coordinator:node",
         {Watcher("/coordinator/target/planner"),
          Sender("/arm/target/coordinator")}},
        {"arm",
         "//robot/arm:node",
         {Watcher("/arm/target/coordinator"),
          Sender("/hw/request/arm", 32, kSourceFlagHardwareRequest)}},
    };
    const TargetModeSources modes{
        {"planner", {{"autonomous", {"auto"}}, {"teleop", {"teleop"}}}},
        {"coordinator", {{"autonomous", {"planner"}}}},
        {"arm", {{"autonomous", {"coordinator"}}}},
    };
    const auto diagnostics = LintGraph(nodes,
                                       {{"planner", {"auto", "teleop"}},
                                        {"coordinator", {"planner"}},
                                        {"arm", {"coordinator"}}},
                                       modes);
    EXPECT_TRUE(diagnostics.empty()) << Describe(diagnostics);
  }

  {
    SCOPED_TRACE("observer edges are not traversed");
    const std::vector<NodeShape> nodes = {
        bridge({Watcher("/hw/request/arm")}),
        {"teleop",
         "//robot/teleop:node",
         {Watcher("/hw/state/driver_station"),
          Sender("/planner/target/teleop", 32, kSourceFlagOperatorArming),
          Sender("/teleop/state")}},
        {"planner",
         "//robot/planner:node",
         {Watcher("/planner/target/teleop")}},
        {"telemetry",
         "//talOS/telemetry:node",
         {Watcher("/teleop/state", 32, kSourceFlagObserver),
          Sender("/coordinator/target/telemetry")}},
        {"coordinator",
         "//robot/coordinator:node",
         {Watcher("/coordinator/target/telemetry"),
          Sender("/arm/target/coordinator")}},
        {"arm",
         "//robot/arm:node",
         {Watcher("/arm/target/coordinator"),
          Sender("/hw/request/arm", 32, kSourceFlagHardwareRequest)}},
    };
    const TargetModeSources modes{
        {"planner", {{"teleop", {"teleop"}}}},
        {"coordinator", {{"teleop", {"telemetry"}}}},
        {"arm", {{"teleop", {"coordinator"}}}},
    };
    const auto diagnostics = LintGraph(nodes,
                                       {{"planner", {"teleop"}},
                                        {"coordinator", {"telemetry"}},
                                        {"arm", {"coordinator"}}},
                                       modes);
    EXPECT_TRUE(diagnostics.empty()) << Describe(diagnostics);
  }

  {
    SCOPED_TRACE(
        "state feedback out of an actuator owner does not cause a false "
        "positive on another subsystem");
    const std::vector<NodeShape> nodes = {
        bridge({Watcher("/hw/request/drivetrain"),
                Watcher("/hw/request/shooter")}),
        {"teleop",
         "//robot/teleop:node",
         {Watcher("/hw/state/driver_station"),
          Sender("/drivetrain/target/teleop", 32, kSourceFlagOperatorArming)}},
        {"drivetrain",
         "//robot/drivetrain:node",
         {Watcher("/drivetrain/target/teleop"), Sender("/drivetrain/state"),
          Sender("/hw/request/drivetrain", 32, kSourceFlagHardwareRequest)}},
        {"pose",
         "//robot/pose:node",
         {Watcher("/drivetrain/state"), Sender("/pose/state")}},
        {"shooter",
         "//robot/shooter:node",
         {Watcher("/pose/state"), Watcher("/shooter/target/auto"),
          Sender("/hw/request/shooter", 32, kSourceFlagHardwareRequest)}},
        {"autonomous",
         "//robot/autonomous:node",
         {Sender("/shooter/target/auto")}},
    };
    const TargetModeSources modes{
        {"drivetrain", {{"teleop", {"teleop"}}}},
        {"shooter", {{"autonomous", {"auto"}}}},
    };
    const auto diagnostics = LintGraph(
        nodes, {{"drivetrain", {"teleop"}}, {"shooter", {"auto"}}}, modes);
    EXPECT_TRUE(diagnostics.empty()) << Describe(diagnostics);
  }

  {
    SCOPED_TRACE(
        "dual teleop and auto producer does not warn via its 3-hop auto lane");
    const std::vector<NodeShape> nodes = {
        bridge({Watcher("/hw/request/drivetrain"), Watcher("/hw/request/arm")}),
        {"operator",
         "//robot/operator:node",
         {Watcher("/hw/state/driver_station"),
          Sender("/drivetrain/target/teleop", 32, kSourceFlagOperatorArming),
          Sender("/planner/target/auto")}},
        {"drivetrain",
         "//robot/drivetrain:node",
         {Watcher("/drivetrain/target/teleop"),
          Sender("/hw/request/drivetrain", 32, kSourceFlagHardwareRequest)}},
        {"planner",
         "//robot/planner:node",
         {Watcher("/planner/target/auto"),
          Sender("/coordinator/target/planner")}},
        {"coordinator",
         "//robot/coordinator:node",
         {Watcher("/coordinator/target/planner"),
          Sender("/arm/target/coordinator")}},
        {"arm",
         "//robot/arm:node",
         {Watcher("/arm/target/coordinator"),
          Sender("/hw/request/arm", 32, kSourceFlagHardwareRequest)}},
    };
    const TargetModeSources modes{
        {"drivetrain", {{"teleop", {"teleop"}}}},
        {"planner", {{"autonomous", {"auto"}}}},
        {"coordinator", {{"autonomous", {"planner"}}}},
        {"arm", {{"autonomous", {"coordinator"}}}},
    };
    const auto diagnostics = LintGraph(nodes,
                                       {{"drivetrain", {"teleop"}},
                                        {"planner", {"auto"}},
                                        {"coordinator", {"planner"}},
                                        {"arm", {"coordinator"}}},
                                       modes);
    EXPECT_TRUE(diagnostics.empty()) << Describe(diagnostics);
  }

  {
    SCOPED_TRACE(
        "reading producer status does not mask a real 3-hop command chain");
    const std::vector<NodeShape> nodes = {
        bridge({Watcher("/hw/request/arm")}),
        {"teleop",
         "//robot/teleop:node",
         {Watcher("/hw/state/driver_station"), Sender("/teleop/status"),
          Sender("/planner/target/teleop", 32, kSourceFlagOperatorArming)}},
        {"planner",
         "//robot/planner:node",
         {Watcher("/planner/target/teleop"),
          Sender("/coordinator/target/planner")}},
        {"coordinator",
         "//robot/coordinator:node",
         {Watcher("/coordinator/target/planner"),
          Sender("/arm/target/coordinator")}},
        {"arm",
         "//robot/arm:node",
         {Watcher("/teleop/status"), Watcher("/arm/target/coordinator"),
          Sender("/hw/request/arm", 32, kSourceFlagHardwareRequest)}},
    };
    const TargetModeSources modes{
        {"planner", {{"teleop", {"teleop"}}}},
        {"coordinator", {{"teleop", {"planner"}}}},
        {"arm", {{"teleop", {"coordinator"}}}},
    };
    const auto diagnostics = LintGraph(nodes,
                                       {{"planner", {"teleop"}},
                                        {"coordinator", {"planner"}},
                                        {"arm", {"coordinator"}}},
                                       modes);
    ASSERT_EQ(diagnostics.size(), 1u) << Describe(diagnostics);
    const auto* warning = Find(diagnostics, Severity::WARNING, "teleop");
    ASSERT_NE(warning, nullptr) << Describe(diagnostics);
    EXPECT_NE(warning->message.find("teleop -> planner -> coordinator -> arm"),
              std::string::npos)
        << warning->message;
  }

  {
    SCOPED_TRACE("3-hop path via request and command roles warns");
    const std::vector<NodeShape> nodes = {
        bridge({Watcher("/hw/request/arm")}),
        {"teleop",
         "//robot/teleop:node",
         {Watcher("/hw/state/driver_station"),
          Sender("/planner/target/teleop", 32, kSourceFlagOperatorArming)}},
        {"planner",
         "//robot/planner:node",
         {Watcher("/planner/target/teleop"), Sender("/coordinator/request")}},
        {"coordinator",
         "//robot/coordinator:node",
         {Watcher("/coordinator/request"), Sender("/arm/command")}},
        {"autonomous", "//robot/autonomous:node", {Sender("/arm/target/auto")}},
        {"arm",
         "//robot/arm:node",
         {Watcher("/arm/command"), Watcher("/arm/target/auto"),
          Sender("/hw/request/arm", 32, kSourceFlagHardwareRequest)}},
    };
    const TargetModeSources modes{
        {"planner", {{"teleop", {"teleop"}}}},
        {"arm", {{"autonomous", {"auto"}}}},
    };
    const auto diagnostics =
        LintGraph(nodes, {{"planner", {"teleop"}}, {"arm", {"auto"}}}, modes);
    EXPECT_FALSE(HasError(diagnostics)) << Describe(diagnostics);
    ASSERT_EQ(diagnostics.size(), 1u) << Describe(diagnostics);
    const auto* warning = Find(diagnostics, Severity::WARNING, "teleop");
    ASSERT_NE(warning, nullptr) << Describe(diagnostics);
    EXPECT_TRUE(Names(warning->message, "teleop")) << warning->message;
    EXPECT_TRUE(Names(warning->message, "arm")) << warning->message;
    EXPECT_NE(warning->message.find('3'), std::string::npos)
        << warning->message;
    EXPECT_NE(warning->message.find("teleop -> planner -> coordinator -> arm"),
              std::string::npos)
        << warning->message;
  }

  {
    SCOPED_TRACE(
        "actuator owner emitting target and command edges does not expand past "
        "the first actuator owner");
    const std::vector<NodeShape> nodes = {
        bridge({Watcher("/hw/request/drivetrain"), Watcher("/hw/request/arm")}),
        {"teleop",
         "//robot/teleop:node",
         {Watcher("/hw/state/driver_station"),
          Sender("/drivetrain/target/teleop", 32, kSourceFlagOperatorArming)}},
        {"drivetrain",
         "//robot/drivetrain:node",
         {Watcher("/drivetrain/target/teleop"),
          Sender("/coordinator/target/drivetrain"),
          Sender("/coordinator/command"),
          Sender("/hw/request/drivetrain", 32, kSourceFlagHardwareRequest)}},
        {"coordinator",
         "//robot/coordinator:node",
         {Watcher("/coordinator/target/drivetrain"),
          Watcher("/coordinator/command"), Sender("/arm/target/coordinator")}},
        {"arm",
         "//robot/arm:node",
         {Watcher("/arm/target/coordinator"),
          Sender("/hw/request/arm", 32, kSourceFlagHardwareRequest)}},
    };
    const TargetModeSources modes{
        {"drivetrain", {{"teleop", {"teleop"}}}},
        {"coordinator", {{"teleop", {"drivetrain"}}}},
        {"arm", {{"teleop", {"coordinator"}}}},
    };
    const auto diagnostics = LintGraph(nodes,
                                       {{"drivetrain", {"teleop"}},
                                        {"coordinator", {"drivetrain"}},
                                        {"arm", {"coordinator"}}},
                                       modes);
    EXPECT_TRUE(diagnostics.empty()) << Describe(diagnostics);
  }

  {
    SCOPED_TRACE(
        "diamond graph with both 1-hop and 3-hop paths uses shortest path and "
        "does not warn");
    const std::vector<NodeShape> nodes = {
        bridge({Watcher("/hw/request/arm")}),
        {"teleop",
         "//robot/teleop:node",
         {Watcher("/hw/state/driver_station"),
          Sender("/planner/target/teleop", 32, kSourceFlagOperatorArming),
          Sender("/arm/target/teleop", 32, kSourceFlagOperatorArming)}},
        {"planner",
         "//robot/planner:node",
         {Watcher("/planner/target/teleop"),
          Sender("/coordinator/target/planner")}},
        {"coordinator",
         "//robot/coordinator:node",
         {Watcher("/coordinator/target/planner"),
          Sender("/arm/target/coordinator")}},
        {"arm",
         "//robot/arm:node",
         {Watcher("/arm/target/teleop"), Watcher("/arm/target/coordinator"),
          Sender("/hw/request/arm", 32, kSourceFlagHardwareRequest)}},
    };
    const TargetModeSources modes{
        {"planner", {{"teleop", {"teleop"}}}},
        {"coordinator", {{"teleop", {"planner"}}}},
        {"arm", {{"teleop", {"teleop", "coordinator"}}}},
    };
    const auto diagnostics = LintGraph(nodes,
                                       {{"planner", {"teleop"}},
                                        {"coordinator", {"planner"}},
                                        {"arm", {"teleop", "coordinator"}}},
                                       modes);
    EXPECT_TRUE(diagnostics.empty()) << Describe(diagnostics);
  }
}

// REQ-SAFE-14, REQ-CONTRACT-09: an explicit Driver Station reader writing a
// teleop-policy target lane without kSourceFlagOperatorArming is an error
// naming the lane and node. Armed lanes, autonomous-only lanes, internal-only
// DS watchers (kSourceFlagInternalDriverStation, modeling coordinators like
// Superstructure), and observer DS watchers pass; internal-only DS watchers
// are also excluded from operator-producer 2-hop warnings.
TEST(LintGraph, OperatorProducerTeleopLaneWithoutArmingIsAnError) {
  const auto bridge = [](std::vector<SourceShape> requests) {
    requests.push_back(Sender("/hw/state/driver_station"));
    return NodeShape{"hardware_node", "//talOS/bridge:hardware_node",
                     std::move(requests), /*hardware_bridge=*/true};
  };

  {
    SCOPED_TRACE("explicit DS + unarmed teleop lane errors naming lane & node");
    const std::vector<NodeShape> nodes = {
        bridge({Watcher("/hw/request/arm")}),
        {"driver",
         "//robot/driver:node",
         {Watcher("/hw/state/driver_station"), Sender("/arm/target/teleop")}},
        {"arm",
         "//robot/arm:node",
         {Watcher("/arm/target/teleop"),
          Sender("/hw/request/arm", 32, kSourceFlagHardwareRequest)}},
    };
    const TargetModeSources modes{{"arm", {{"teleop", {"teleop"}}}}};
    const auto diagnostics = LintGraph(nodes, {{"arm", {"teleop"}}}, modes);
    ASSERT_TRUE(HasError(diagnostics)) << Describe(diagnostics);
    const auto* err = Find(diagnostics, Severity::ERROR, "/arm/target/teleop");
    ASSERT_NE(err, nullptr) << Describe(diagnostics);
    EXPECT_TRUE(Names(err->message, "driver")) << err->message;
    EXPECT_NE(err->message.find("OperatorArming"), std::string::npos)
        << err->message;
    EXPECT_NE(err->message.find("REQ-SAFE-14"), std::string::npos)
        << err->message;
    EXPECT_EQ(diagnostics.size(), 1u) << Describe(diagnostics);
  }

  {
    SCOPED_TRACE("one armed + one unarmed teleop lane errors only on unarmed");
    const std::vector<NodeShape> nodes = {
        bridge({Watcher("/hw/request/arm"), Watcher("/hw/request/intake")}),
        {"driver",
         "//robot/driver:node",
         {Watcher("/hw/state/driver_station"),
          Sender("/arm/target/teleop", 32, kSourceFlagOperatorArming),
          Sender("/intake/target/teleop")}},
        {"arm",
         "//robot/arm:node",
         {Watcher("/arm/target/teleop"),
          Sender("/hw/request/arm", 32, kSourceFlagHardwareRequest)}},
        {"intake",
         "//robot/intake:node",
         {Watcher("/intake/target/teleop"),
          Sender("/hw/request/intake", 32, kSourceFlagHardwareRequest)}},
    };
    const TargetModeSources modes{
        {"arm", {{"teleop", {"teleop"}}}},
        {"intake", {{"teleop", {"teleop"}}}},
    };
    const auto diagnostics =
        LintGraph(nodes, {{"arm", {"teleop"}}, {"intake", {"teleop"}}}, modes);
    ASSERT_EQ(diagnostics.size(), 1u) << Describe(diagnostics);
    EXPECT_EQ(Find(diagnostics, Severity::ERROR, "/arm/target/teleop"),
              nullptr);
    const auto* err =
        Find(diagnostics, Severity::ERROR, "/intake/target/teleop");
    ASSERT_NE(err, nullptr) << Describe(diagnostics);
    EXPECT_TRUE(Names(err->message, "driver")) << err->message;
  }

  {
    SCOPED_TRACE("explicit DS + armed teleop lane is clean");
    const std::vector<NodeShape> nodes = {
        bridge({Watcher("/hw/request/arm")}),
        {"driver",
         "//robot/driver:node",
         {Watcher("/hw/state/driver_station"),
          Sender("/arm/target/teleop", 32, kSourceFlagOperatorArming)}},
        {"arm",
         "//robot/arm:node",
         {Watcher("/arm/target/teleop"),
          Sender("/hw/request/arm", 32, kSourceFlagHardwareRequest)}},
    };
    const TargetModeSources modes{{"arm", {{"teleop", {"teleop"}}}}};
    const auto diagnostics = LintGraph(nodes, {{"arm", {"teleop"}}}, modes);
    EXPECT_TRUE(diagnostics.empty()) << Describe(diagnostics);
  }

  {
    SCOPED_TRACE("explicit DS + autonomous-only lane without arming is clean");
    const std::vector<NodeShape> nodes = {
        bridge({Watcher("/hw/request/arm")}),
        {"autonomous",
         "//robot/autonomous:node",
         {Watcher("/hw/state/driver_station"), Sender("/arm/target/auto")}},
        {"arm",
         "//robot/arm:node",
         {Watcher("/arm/target/auto"),
          Sender("/hw/request/arm", 32, kSourceFlagHardwareRequest)}},
    };
    const TargetModeSources modes{
        {"arm", {{"teleop", {"teleop"}}, {"autonomous", {"auto"}}}}};
    const auto diagnostics =
        LintGraph(nodes, {{"arm", {"teleop", "auto"}}}, modes);
    EXPECT_TRUE(diagnostics.empty()) << Describe(diagnostics);
  }

  {
    SCOPED_TRACE(
        "internal-only DS watcher (Superstructure coordinator) + teleop lane "
        "is clean and excluded from operator-producer 2-hop warning");
    const std::vector<NodeShape> nodes = {
        bridge({Watcher("/hw/request/arm")}),
        {"superstructure",
         "//robot/superstructure:node",
         {Watcher("/hw/state/driver_station", 32,
                  kSourceFlagInternalDriverStation),
          Sender("/planner/target/superstructure")}},
        {"planner",
         "//robot/planner:node",
         {Watcher("/planner/target/superstructure"),
          Sender("/coordinator/target/planner")}},
        {"coordinator",
         "//robot/coordinator:node",
         {Watcher("/coordinator/target/planner"),
          Sender("/arm/target/coordinator")}},
        {"arm",
         "//robot/arm:node",
         {Watcher("/arm/target/coordinator"),
          Sender("/hw/request/arm", 32, kSourceFlagHardwareRequest)}},
    };
    const TargetModeSources modes{
        {"planner", {{"teleop", {"superstructure"}}}},
        {"coordinator", {{"teleop", {"planner"}}}},
        {"arm", {{"teleop", {"coordinator"}}}},
    };
    const auto diagnostics = LintGraph(nodes,
                                       {{"planner", {"superstructure"}},
                                        {"coordinator", {"planner"}},
                                        {"arm", {"coordinator"}}},
                                       modes);
    EXPECT_TRUE(diagnostics.empty()) << Describe(diagnostics);
  }

  {
    SCOPED_TRACE("observer DS watcher + unarmed teleop lane is clean");
    const std::vector<NodeShape> nodes = {
        bridge({Watcher("/hw/request/arm")}),
        {"replay_driver",
         "//robot/replay_driver:node",
         {Watcher("/hw/state/driver_station", 32, kSourceFlagObserver),
          Sender("/arm/target/teleop")}},
        {"arm",
         "//robot/arm:node",
         {Watcher("/arm/target/teleop"),
          Sender("/hw/request/arm", 32, kSourceFlagHardwareRequest)}},
    };
    const TargetModeSources modes{{"arm", {{"teleop", {"teleop"}}}}};
    const auto diagnostics = LintGraph(nodes, {{"arm", {"teleop"}}}, modes);
    EXPECT_TRUE(diagnostics.empty()) << Describe(diagnostics);
  }
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

TEST(AlertPrefixedFields, RejectsOutOfRangeBaseTypeWithoutUb) {
  for (const std::int8_t bad_byte :
       {static_cast<std::int8_t>(-128), static_cast<std::int8_t>(127)}) {
    SCOPED_TRACE("raw BaseType=" + std::to_string(bad_byte));
    flatbuffers::FlatBufferBuilder builder;
    const auto obj_name = builder.CreateString("bad.Status");
    const auto field_name = builder.CreateString("alert_error_fault");
    const auto type_start = builder.StartTable();
    builder.AddElement<std::int8_t>(reflection::Type::VT_BASE_TYPE, bad_byte,
                                    0);
    const flatbuffers::Offset<reflection::Type> type{
        builder.EndTable(type_start)};
    reflection::FieldBuilder fb(builder);
    fb.add_name(field_name);
    fb.add_type(type);
    const auto field = fb.Finish();
    const auto fields = builder.CreateVector(&field, 1);
    reflection::ObjectBuilder ob(builder);
    ob.add_name(obj_name);
    ob.add_fields(fields);
    ob.add_is_struct(true);
    ob.add_bytesize(8);
    const auto obj = ob.Finish();
    const auto objects = builder.CreateVector(&obj, 1);
    const auto enums = builder.CreateVector(
        std::vector<flatbuffers::Offset<reflection::Enum>>{});
    reflection::SchemaBuilder sb(builder);
    sb.add_objects(objects);
    sb.add_enums(enums);
    reflection::FinishSchemaBuffer(builder, sb.Finish());
    const std::string_view bfbs{
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize()};
    const auto result = detail::AlertPrefixedFields(bfbs, "bad.Status");
    EXPECT_TRUE(result.fields.empty());
    EXPECT_FALSE(result.error.empty());
  }
}

TEST(AlertPrefixedFields, DeepObjectChainAndSeededMutationsDoNotCrash) {
  // Deep 500-object chain must not stack-overflow (and bounds depth safely).
  {
    flatbuffers::FlatBufferBuilder builder;
    constexpr int kChain = 500;
    std::vector<flatbuffers::Offset<reflection::Object>> objs;
    objs.reserve(kChain);
    for (int i = 0; i < kChain; ++i) {
      const auto obj_name = builder.CreateString("c.o" + std::to_string(i));
      const auto field_name = builder.CreateString("alert_error_x");
      reflection::TypeBuilder tb(builder);
      if (i + 1 < kChain) {
        tb.add_base_type(reflection::BaseType::Obj);
        tb.add_index(i + 1);
      } else {
        tb.add_base_type(reflection::BaseType::Bool);
      }
      const auto type = tb.Finish();
      reflection::FieldBuilder fb(builder);
      fb.add_name(field_name);
      fb.add_type(type);
      const auto field = fb.Finish();
      const auto fields = builder.CreateVector(&field, 1);
      reflection::ObjectBuilder ob(builder);
      ob.add_name(obj_name);
      ob.add_fields(fields);
      ob.add_is_struct(true);
      ob.add_bytesize(8);
      objs.push_back(ob.Finish());
    }
    const auto objects = builder.CreateVector(objs);
    const auto enums = builder.CreateVector(
        std::vector<flatbuffers::Offset<reflection::Enum>>{});
    reflection::SchemaBuilder sb(builder);
    sb.add_objects(objects);
    sb.add_enums(enums);
    reflection::FinishSchemaBuffer(builder, sb.Finish());
    const std::string_view bfbs{
        reinterpret_cast<const char*>(builder.GetBufferPointer()),
        builder.GetSize()};
    const auto found = detail::AlertPrefixedFields(bfbs, "c.o0");
    EXPECT_TRUE(found.error.empty()) << found.error;
    EXPECT_FALSE(found.fields.empty());
    EXPECT_LE(found.fields.size(), 64u);
  }

  // Deterministic seeded mutations over AlertPrefixedFields.
  const schema::Identity alerts = schema::Of<schema_test::BadAlerts>();
  std::uint64_t rng_state = 0x414c45525453ULL;
  auto next_u64 = [&]() {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 7;
    rng_state ^= rng_state << 17;
    return rng_state;
  };
  for (int iter = 0; iter < 1500; ++iter) {
    std::string bfbs = alerts.bfbs;
    const auto op = next_u64() % 3;
    if (op == 0 && bfbs.size() > 8) {
      bfbs.resize(1 + (next_u64() % bfbs.size()));
    } else if (op == 1 && !bfbs.empty()) {
      for (int k = 0; k < 3; ++k) {
        char& byte = bfbs[next_u64() % bfbs.size()];
        byte = static_cast<char>(static_cast<unsigned char>(byte) ^
                                 (1u << (next_u64() % 8)));
      }
    } else {
      bfbs.resize(next_u64() % 96);
      for (char& c : bfbs) c = static_cast<char>(next_u64() & 0xffu);
    }
    (void)detail::AlertPrefixedFields(bfbs, "schema_test.BadAlerts");
  }
}

}  // namespace
}  // namespace talos::introspect::naming
