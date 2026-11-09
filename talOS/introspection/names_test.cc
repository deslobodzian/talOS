// The naming grammar and the graph lint. Every name in
// RejectsTheMistakesThatActuallyHappened shipped in this repo once; they are
// listed so no edit to the rules can quietly re-admit them. The real robot's
// graph is checked by the launcher against its binaries (launcher_test).

#include "talOS/introspection/names.h"

#include <algorithm>
#include <string>
#include <vector>

#include "gtest/gtest.h"

namespace talos::introspect::naming {
namespace {

using event::SourceKind;

// Every topic name the robot uses today, all of which must conform. Kept short
// rather than exhaustive: the point is that the canonical shapes pass, not to
// mirror the graph, which the launcher checks against the binaries themselves.
TEST(TopicName, AcceptsTheShapesTheRobotUses) {
  for (const auto* name : {
           "/hw/state",
           "/hw/command",
           "/hw/state/driver_station",
           "/hw/request/drivetrain",
           "/hw/request/shooter",
           "/drivetrain/state",
           "/drivetrain/target",
           "/drivetrain/target/teleop",
           "/drivetrain/target/auto",
           "/shooter/state",
           "/shooter/target",
           "/odometry/state",
           "/talos/telemetry",
       }) {
    const auto problem = CheckTopicName(name);
    EXPECT_FALSE(problem.has_value())
        << name << " should conform but: " << problem.value_or("");
  }
}

// Each of these was a live defect. The message must name the fix, because the
// person reading it is the person who is about to write the other end.
TEST(TopicName, RejectsTheMistakesThatActuallyHappened) {
  struct Case {
    const char* name;
    const char* must_mention;
    const char* why;
  };
  for (const auto& [name, must_mention, why] : std::vector<Case>{
           {"/hw/req/drivetrain", "request",
            "abbreviated role; the bridge derived this while the drivetrain "
            "package published /hw/req/drive"},
           {"/hw/req/drive", "request",
            "abbreviated role; the qualifier 'drive' for 'drivetrain' is the "
            "other half of the same defect"},
           {"/hw/cmd", "command", "abbreviated role"},
           {"/hw/cmd_in", "direction",
            "abbreviated role and a direction suffix; the suffix is what the "
            "reader is told about first"},
           {"/hw/ds", "driver_station",
            "abbreviation nobody can expand twice "
            "the same way"},
           {"/drivetrain/tgt", "target", "abbreviated role"},
           {"/odometry", "/owner/role", "one segment, so there is no role"},
           {"/talos_studio", "/owner/role",
            "one segment, and an owner glued to a subject"},
       }) {
    const auto problem = CheckTopicName(name);
    if (!problem.has_value()) {
      FAIL() << name << " must be rejected: " << why;
    }
    EXPECT_NE(problem->find(must_mention), std::string::npos)
        << "rejecting '" << name << "' must point at '" << must_mention
        << "', got: " << *problem;
  }
}

TEST(TopicName, RejectsDirectionSuffixesFromEitherEnd) {
  // Direction belongs to the endpoint. Naming one side is wrong from the other,
  // and invites a second topic for the other direction.
  for (const auto* name :
       {"/drivetrain/state_out", "/drivetrain/target_in", "/shooter/state_pub",
        "/shooter/target_sub", "/odometry/state_tx", "/odometry/state_topic"}) {
    const auto problem = CheckTopicName(name);
    if (!problem.has_value()) {
      FAIL() << name;
    }
    EXPECT_NE(problem->find("direction"), std::string::npos) << *problem;
  }
}

TEST(TopicName, RejectsMalformedNames) {
  EXPECT_TRUE(CheckTopicName("").has_value());
  EXPECT_TRUE(CheckTopicName("drivetrain/state").has_value());  // no leading /
  EXPECT_TRUE(CheckTopicName("/drivetrain/state/").has_value());  // trailing /
  EXPECT_TRUE(CheckTopicName("/drivetrain//state").has_value());  // empty
  EXPECT_TRUE(CheckTopicName("/Drivetrain/state").has_value());   // uppercase
  EXPECT_TRUE(CheckTopicName("/drive-train/state").has_value());  // hyphen
  EXPECT_TRUE(CheckTopicName("/2026/state").has_value());  // leading digit
  EXPECT_TRUE(CheckTopicName("/drivetrain__x/state").has_value());
  EXPECT_TRUE(CheckTopicName("/drivetrain_/state").has_value());
  EXPECT_TRUE(CheckTopicName("/a/state/b/c/d").has_value());  // 5 segments
  EXPECT_TRUE(
      CheckTopicName("/drivetrain/pose").has_value());  // role not in set
}

// The transport's limit (rtms::ValidatePath), not the event loop's 63.
TEST(TopicName, LengthLimitIsTheTransportLimit) {
  EXPECT_EQ(kMaxTopicBytes, 31u);
  EXPECT_LE(kMaxTopicBytes, event::MAX_SOURCE_NAME);

  const std::string prefix = "/drivetrain/state/";
  const std::string fits =
      prefix + std::string(kMaxTopicBytes - prefix.size(), 'x');
  ASSERT_EQ(fits.size(), kMaxTopicBytes);
  EXPECT_FALSE(CheckTopicName(fits).has_value()) << fits;
  EXPECT_TRUE(CheckTopicName(fits + "x").has_value());

  // One segment may not spend the whole budget on its own.
  EXPECT_TRUE(
      CheckTopicName("/" + std::string(kMaxSegmentBytes + 1, 'a') + "/state")
          .has_value());
}

TEST(NodeName, MatchesTheSubsystemKeyRules) {
  for (const auto* name :
       {"drivetrain", "shooter", "odometry", "autonomous", "driver_station",
        "driver", "telemetry", "hardware_node"}) {
    EXPECT_FALSE(CheckNodeName(name).has_value()) << name;
  }
  EXPECT_TRUE(CheckNodeName("").has_value());
  EXPECT_TRUE(CheckNodeName("Drivetrain").has_value());
  EXPECT_TRUE(CheckNodeName("drive-train").has_value());
  // A node named after a reserved namespace would own topics it does not own.
  EXPECT_TRUE(CheckNodeName("hw").has_value());
  EXPECT_TRUE(CheckNodeName("talos").has_value());
}

TEST(NodeTarget, AcceptsThePackageLeafOrTheRuleName) {
  // A robot's node: the package's last segment carries the name.
  EXPECT_FALSE(
      CheckNodeTarget("odometry", "//2026-robot/main_processor/odometry:node")
          .has_value());
  // A framework node has no subsystem package, so the rule name carries it.
  EXPECT_FALSE(CheckNodeTarget("hardware_node", "//talOS/bridge:hardware_node")
                   .has_value());

  // A name no build target produces cannot be traced back to source.
  const auto problem =
      CheckNodeTarget("shooter", "//2026-robot/main_processor/odometry:node");
  if (!problem.has_value()) {
    FAIL() << "shooter under the odometry package must be rejected";
  }
  EXPECT_NE(problem->find("odometry"), std::string::npos) << *problem;

  // External repository labels: root package matches the repo name, otherwise
  // package leaf or rule name as in the main repository.
  EXPECT_FALSE(CheckNodeTarget("wrist", "@wrist//:node").has_value());
  EXPECT_FALSE(CheckNodeTarget("node", "@wrist//pkg:node").has_value());
  EXPECT_FALSE(CheckNodeTarget("canon", "@@canon//:node").has_value());
  EXPECT_FALSE(CheckNodeTarget("wrist", "@@wrist+//:node").has_value());
  EXPECT_TRUE(CheckNodeTarget("wrist", "@wrist//pkg:other").has_value());
  EXPECT_TRUE(CheckNodeTarget("wrist", "wrist//:node").has_value());

  EXPECT_TRUE(CheckNodeTarget("odometry", "odometry/node").has_value());
  EXPECT_TRUE(
      CheckNodeTarget("odometry", "@talos_odometry//:node").has_value());
  EXPECT_TRUE(
      CheckNodeTarget("odometry", "//2026-robot/main_processor/odometry")
          .has_value());
}

SourceShape Sender(std::string name, std::uint32_t bytes = 32,
                   std::uint32_t flags = 0) {
  return {SourceKind::SENDER, std::move(name), bytes, flags};
}
SourceShape Watcher(std::string name, std::uint32_t bytes = 32,
                    std::uint32_t flags = 0) {
  return {SourceKind::WATCHER, std::move(name), bytes, flags};
}

bool Mentions(const std::vector<Diagnostic>& diagnostics, Severity severity,
              std::string_view subject, std::string_view fragment) {
  for (const auto& diagnostic : diagnostics) {
    if (diagnostic.severity == severity && diagnostic.subject == subject &&
        diagnostic.message.find(fragment) != std::string::npos) {
      return true;
    }
  }
  return false;
}

std::string Describe(const std::vector<Diagnostic>& diagnostics) {
  std::string out;
  for (const auto& diagnostic : diagnostics) {
    out += "\n  " + diagnostic.ToString();
  }
  return out.empty() ? " (none)" : out;
}

TEST(LintGraph, ReaderLimitIncludesObserversAndAllowsExactlyTheLimit) {
  std::vector<NodeShape> nodes = {
      {"drivetrain", "//robot/drivetrain:node", {Sender("/drivetrain/state")}}};
  for (std::size_t i = 0; i < MAX_READERS; ++i) {
    const auto name = "observer_" + std::to_string(i);
    nodes.push_back({name,
                     "//robot/" + name + ":node",
                     {Watcher("/drivetrain/state", 32, kSourceFlagObserver)}});
  }
  EXPECT_FALSE(HasError(LintGraph(nodes))) << Describe(LintGraph(nodes));
  nodes.push_back(
      {"consumer", "//robot/consumer:node", {Watcher("/drivetrain/state")}});
  const auto diagnostics = LintGraph(nodes);
  EXPECT_TRUE(Mentions(diagnostics, Severity::ERROR, "/drivetrain/state",
                       "reader limit is " + std::to_string(MAX_READERS)))
      << Describe(diagnostics);
}

TEST(LintGraph, AcceptsAWellFormedGraph) {
  const std::vector<NodeShape> nodes = {
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {Watcher("/drivetrain/target/teleop", 32, kSourceFlagOptional),
        Sender("/drivetrain/state")}},
      {"driver",
       "//2026-robot/main_processor/driver:node",
       {Sender("/drivetrain/target/teleop")}},
      {"odometry",
       "//2026-robot/main_processor/odometry:node",
       {Watcher("/drivetrain/state"), Sender("/odometry/state")}},
      {"telemetry",
       "//talOS/telemetry:node",
       // The Studio bridge reads /talos/telemetry but is not a registered
       // node, so the feed is declared external rather than left to read as a
       // topic nobody consumes.
       {Watcher("/odometry/state"),
        Sender("/talos/telemetry", 32, kSourceFlagExternal)}},
  };
  const auto diagnostics =
      LintGraph(nodes, {{"drivetrain", {"teleop", "auto"}}});
  EXPECT_FALSE(HasError(diagnostics)) << Describe(diagnostics);
  EXPECT_TRUE(diagnostics.empty()) << Describe(diagnostics);
}

// The exact defect this protocol exists to catch: both ends present, both
// individually valid, different strings.
TEST(LintGraph, CatchesTwoSpellingsOfOneTopic) {
  const std::vector<NodeShape> nodes = {
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {Sender("/hw/request/drive_train")}},
      {"hardware_node",
       "//talOS/bridge:hardware_node",
       {Watcher("/hw/request/drivetrain")}},
  };
  const auto diagnostics = LintGraph(nodes);
  ASSERT_TRUE(HasError(diagnostics)) << Describe(diagnostics);
  EXPECT_TRUE(Mentions(diagnostics, Severity::ERROR, "/hw/request/drivetrain",
                       "nothing publishes it"))
      << Describe(diagnostics);
  EXPECT_TRUE(Mentions(diagnostics, Severity::ERROR, "/hw/request/drivetrain",
                       "spelled differently"))
      << Describe(diagnostics);
}

TEST(LintGraph, TwoWritersIsAnErrorAndBothAreNamed) {
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
  ASSERT_TRUE(HasError(diagnostics)) << Describe(diagnostics);
  EXPECT_TRUE(Mentions(diagnostics, Severity::ERROR,
                       "/drivetrain/target/teleop", "autonomous"));
  EXPECT_TRUE(Mentions(diagnostics, Severity::ERROR,
                       "/drivetrain/target/teleop", "driver"));
  // Replay is the reason, and the message should say so.
  EXPECT_TRUE(Mentions(diagnostics, Severity::ERROR,
                       "/drivetrain/target/teleop", "replayable"));
}
TEST(LintGraph,
     TeleopNodeAndAnotherWriterOnSameTeleopLaneIsRefusedAsTwoWriters) {
  const std::vector<NodeShape> nodes = {
      {"teleop", "//talOS/teleop:node", {Sender("/shooter/target/teleop")}},
      {"driver",
       "//2026-robot/main_processor/driver:node",
       {Sender("/shooter/target/teleop")}},
      {"shooter",
       "//2026-robot/main_processor/shooter:node",
       {Watcher("/shooter/target/teleop")}},
  };
  const auto diagnostics = LintGraph(nodes);
  ASSERT_TRUE(HasError(diagnostics)) << Describe(diagnostics);
  EXPECT_TRUE(Mentions(diagnostics, Severity::ERROR, "/shooter/target/teleop",
                       "teleop"));
  EXPECT_TRUE(Mentions(diagnostics, Severity::ERROR, "/shooter/target/teleop",
                       "driver"));
  EXPECT_TRUE(Mentions(diagnostics, Severity::ERROR, "/shooter/target/teleop",
                       "2 writers"));
}

// The owner segment names the subject, and the role says which way the data
// moves relative to it. Getting this backwards was the first draft's mistake:
// "the owner is the publisher" reads as obvious and is false, because the
// operator interface publishes /drivetrain/target/teleop and the drivetrain
// consumes it.
TEST(LintGraph, StateFlowsOutOfItsOwner) {
  const std::vector<NodeShape> nodes = {
      {"odometry",
       "//2026-robot/main_processor/odometry:node",
       // Publishing somebody else's state.
       {Sender("/drivetrain/state")}},
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {Watcher("/drivetrain/state")}},
  };
  const auto diagnostics = LintGraph(nodes);
  ASSERT_TRUE(HasError(diagnostics)) << Describe(diagnostics);
  EXPECT_TRUE(Mentions(diagnostics, Severity::ERROR, "/drivetrain/state",
                       "published by its own subject"))
      << Describe(diagnostics);

  // The right way round.
  const std::vector<NodeShape> correct = {
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {Sender("/drivetrain/state")}},
      {"odometry",
       "//2026-robot/main_processor/odometry:node",
       {Watcher("/drivetrain/state")}},
  };
  EXPECT_TRUE(LintGraph(correct).empty()) << Describe(LintGraph(correct));
}

TEST(LintGraph, TargetsFlowIntoTheirOwner) {
  // A node publishing its own target is talking to itself: either the name is
  // wrong or the producer is confused.
  const std::vector<NodeShape> nodes = {
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {Sender("/drivetrain/target/teleop"),
        Watcher("/drivetrain/target/teleop")}},
  };
  const auto diagnostics = LintGraph(nodes);
  ASSERT_TRUE(HasError(diagnostics)) << Describe(diagnostics);
  EXPECT_TRUE(Mentions(diagnostics, Severity::ERROR,
                       "/drivetrain/target/teleop",
                       "not by the subsystem being asked"))
      << Describe(diagnostics);
}

// A lane whose source no mode of the subsystem's policy lists is never read:
// a dead producer, refused before launch.
TEST(LintGraph, LaneOutsideThePolicyIsAnError) {
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
  ASSERT_TRUE(HasError(diagnostics)) << Describe(diagnostics);
  EXPECT_TRUE(Mentions(diagnostics, Severity::ERROR, "/drivetrain/target/align",
                       "in no mode of drivetrain's target policy"))
      << Describe(diagnostics);

  // Listed: fine. A subsystem the policy map does not know is not checked.
  EXPECT_FALSE(HasError(
      LintGraph(nodes, {{"drivetrain", {"align", "teleop", "auto"}}})));
  EXPECT_FALSE(HasError(LintGraph(nodes, {{"shooter", {"teleop"}}})));
}

// A policy lane with no producer yet (autonomous is not written) is normal
// during development, so it warns rather than refusing the launch.
TEST(LintGraph, PolicyLaneWithNoWriterIsAWarning) {
  const std::vector<NodeShape> nodes = {
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {Watcher("/drivetrain/target/auto", 32, kSourceFlagOptional)}},
  };
  const auto diagnostics = LintGraph(nodes, {{"drivetrain", {"auto"}}});
  EXPECT_FALSE(HasError(diagnostics)) << Describe(diagnostics);
  EXPECT_TRUE(Mentions(diagnostics, Severity::WARNING,
                       "/drivetrain/target/auto", "nothing writes it yet"))
      << Describe(diagnostics);
}

TEST(LintGraph, RequiredCoverageChecksEachModeRatherThanTheirUnion) {
  const std::vector<NodeShape> nodes = {
      {"drivetrain",
       "//robot/drivetrain:node",
       {Watcher("/drivetrain/target/teleop", 32, kSourceFlagOptional),
        Watcher("/drivetrain/target/auto", 32, kSourceFlagOptional)}},
      {"driver", "//robot/driver:node", {Sender("/drivetrain/target/teleop")}},
  };
  const TargetSources sources{{"drivetrain", {"teleop", "auto", "align"}}};
  const TargetModeSources modes{{"drivetrain",
                                 {{"teleop", {"teleop"}},
                                  {"autonomous", {"auto", "align"}},
                                  {"test", {}}}}};
  const auto diagnostics = LintGraph(nodes, sources, modes, true);
  ASSERT_TRUE(HasError(diagnostics)) << Describe(diagnostics);
  EXPECT_TRUE(
      Mentions(diagnostics, Severity::ERROR, "drivetrain", "mode 'autonomous'"))
      << Describe(diagnostics);
  EXPECT_TRUE(Mentions(diagnostics, Severity::ERROR, "drivetrain",
                       "/drivetrain/target/auto, /drivetrain/target/align"))
      << Describe(diagnostics);
  EXPECT_EQ(std::count_if(diagnostics.begin(), diagnostics.end(),
                          [](const Diagnostic& diagnostic) {
                            return diagnostic.severity == Severity::ERROR;
                          }),
            1);

  // Partial development launches retain the per-lane warning.
  const auto optional = LintGraph(nodes, sources, modes, false);
  EXPECT_FALSE(HasError(optional)) << Describe(optional);
  EXPECT_TRUE(Mentions(optional, Severity::WARNING, "/drivetrain/target/auto",
                       "nothing writes it yet"))
      << Describe(optional);
}

TEST(LintGraph, RequiredCoverageAllowsOneWriterPerModeAndOptionalLanes) {
  const std::vector<NodeShape> nodes = {
      {"drivetrain",
       "//robot/drivetrain:node",
       {Watcher("/drivetrain/target/teleop", 32, kSourceFlagOptional),
        Watcher("/drivetrain/target/auto", 32, kSourceFlagOptional),
        Watcher("/drivetrain/target/align", 32, kSourceFlagOptional)}},
      {"driver", "//robot/driver:node", {Sender("/drivetrain/target/teleop")}},
      {"autonomous",
       "//robot/autonomous:node",
       {Sender("/drivetrain/target/auto")}},
      {"telemetry", "//robot/telemetry:node", {}},
  };
  const auto diagnostics = LintGraph(
      nodes, {{"drivetrain", {"teleop", "auto", "align"}}},
      {{"drivetrain",
        {{"teleop", {"teleop"}},
         {"autonomous", {"align", "auto"}},
         {"test", {}}}},
       {"telemetry", {{"teleop", {"teleop"}}, {"autonomous", {"auto"}}}}},
      true);
  EXPECT_FALSE(HasError(diagnostics)) << Describe(diagnostics);
  EXPECT_TRUE(Mentions(diagnostics, Severity::WARNING,
                       "/drivetrain/target/align", "nothing writes it yet"))
      << Describe(diagnostics);
}

TEST(LintGraph, RequiredCoverageDetectsAnEntirelyUndeclaredModeLane) {
  const std::vector<NodeShape> nodes = {
      {"drivetrain",
       "//robot/drivetrain:node",
       {Watcher("/drivetrain/target/teleop", 32, kSourceFlagOptional)}},
      {"driver", "//robot/driver:node", {Sender("/drivetrain/target/teleop")}},
  };
  const auto diagnostics = LintGraph(
      nodes, {{"drivetrain", {"teleop", "auto"}}},
      {{"drivetrain", {{"teleop", {"teleop"}}, {"autonomous", {"auto"}}}}},
      true);
  EXPECT_TRUE(Mentions(diagnostics, Severity::ERROR, "drivetrain",
                       "/drivetrain/target/auto"))
      << Describe(diagnostics);
}

TEST(LintGraph, RequiredCoverageChecksOptedInTestMode) {
  const std::vector<NodeShape> nodes = {
      {"drivetrain",
       "//robot/drivetrain:node",
       {Watcher("/drivetrain/target/pit", 32, kSourceFlagOptional)}},
  };
  const auto diagnostics = LintGraph(
      nodes, {{"drivetrain", {"pit"}}},
      {{"drivetrain", {{"teleop", {}}, {"autonomous", {}}, {"test", {"pit"}}}}},
      true);
  EXPECT_TRUE(
      Mentions(diagnostics, Severity::ERROR, "drivetrain", "mode 'test'"))
      << Describe(diagnostics);
}

// A reserved namespace is exempt from both directions: the bridge publishes
// /hw/* on behalf of a controller processor that is not a talOS node at all,
// so neither "the owner publishes it" nor "the owner consumes it" applies.
TEST(LintGraph, ReservedNamespacesAreExemptFromTheOwnerRule) {
  const std::vector<NodeShape> nodes = {
      {"hardware_node",
       "//talOS/bridge:hardware_node",
       {Sender("/hw/state"), Sender("/hw/state/driver_station"),
        Watcher("/hw/request/drivetrain")}},
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {Sender("/hw/request/drivetrain"), Watcher("/hw/state")}},
      {"driver",
       "//2026-robot/main_processor/driver:node",
       {Watcher("/hw/state/driver_station")}},
  };
  EXPECT_TRUE(LintGraph(nodes).empty()) << Describe(LintGraph(nodes));
}

// A subscriber has no claim on the name of what it reads, so nothing is checked
// on the reading end.
TEST(LintGraph, ReadingEndIsNotCheckedAgainstTheOwner) {
  const std::vector<NodeShape> nodes = {
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {Sender("/drivetrain/state")}},
      {"telemetry",
       "//talOS/telemetry:node",
       {Watcher("/drivetrain/state"),
        Sender("/talos/telemetry", 32, kSourceFlagExternal)}},
  };
  EXPECT_TRUE(LintGraph(nodes).empty()) << Describe(LintGraph(nodes));
}

// A typo in the owner segment, or a name left behind by a rename, shows up as
// an owner that is nobody. A warning, not an error: a partial launch is a
// legitimate session in which the owner really is absent.
TEST(LintGraph, UnknownOwnerIsAWarning) {
  const std::vector<NodeShape> nodes = {
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {Watcher("/drivetraim/state")}},
      {"odometry",
       "//2026-robot/main_processor/odometry:node",
       {Sender("/drivetraim/state")}},
  };
  const auto diagnostics = LintGraph(nodes);
  EXPECT_TRUE(Mentions(diagnostics, Severity::WARNING, "/drivetraim/state",
                       "not a node in this session"))
      << Describe(diagnostics);
}

// An external or optional end is a design, not a fault. Reporting it as a
// fault is what trains people to ignore the report.
TEST(LintGraph, ExternalAndOptionalEndsAreNotFaults) {
  const std::vector<NodeShape> nodes = {
      {"hardware_node",
       "//talOS/bridge:hardware_node",
       {
           // Consumed by the RoboRIO over UDP; no shared-memory peer exists.
           Sender("/hw/command", 32, kSourceFlagExternal),
       }},
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {
           // A feed nobody may be publishing yet.
           Watcher("/drivetrain/state/vision", 32, kSourceFlagOptional),
       }},
  };
  const auto diagnostics = LintGraph(nodes);
  EXPECT_FALSE(HasError(diagnostics)) << Describe(diagnostics);
  for (const auto& diagnostic : diagnostics) {
    EXPECT_NE(diagnostic.subject, "/hw/command");
    EXPECT_NE(diagnostic.subject, "/drivetrain/state/vision");
  }
}

// A timer's name is a label, not an address. This robot has timers called
// `swerve`, `shooter` and `telemetry`, and an earlier version of LintGraph
// checked them as topic names -- reporting three errors on a graph that was
// entirely correct, and refusing to launch. A linter that cries wolf is worse
// than no linter, so this is pinned.
TEST(LintGraph, TimerLabelsAreNotTopics) {
  const std::vector<NodeShape> nodes = {
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {{SourceKind::TIMER, "swerve", 0, 0}, Sender("/drivetrain/state")}},
      {"shooter",
       "//2026-robot/main_processor/shooter:node",
       {{SourceKind::TIMER, "shooter", 0, 0}, Sender("/shooter/state")}},
      {"odometry",
       "//2026-robot/main_processor/odometry:node",
       {{SourceKind::TIMER, "telemetry", 0, 0},
        Watcher("/drivetrain/state"),
        Watcher("/shooter/state"),
        Sender("/odometry/state", 32, kSourceFlagExternal)}},
  };
  const auto diagnostics = LintGraph(nodes);
  EXPECT_TRUE(diagnostics.empty()) << Describe(diagnostics);
  // Named explicitly: these three strings would each fail CheckTopicName.
  for (const auto* label : {"swerve", "shooter", "telemetry"}) {
    EXPECT_TRUE(CheckTopicName(label).has_value())
        << label << " would pass as a topic, so this test proves nothing";
  }
}

// A shape without a schema (a registry snapshot) is still checked on byte
// size: two ends that disagree on the size of one topic cannot be
// wire-compatible.
TEST(LintGraph, TwoSizesOnOneTopicIsAnErrorAndBothEndsAreNamed) {
  const std::vector<NodeShape> nodes = {
      {"shooter",
       "//2026-robot/main_processor/shooter:node",
       {Sender("/shooter/state", 24)}},
      {"odometry",
       "//2026-robot/main_processor/odometry:node",
       {Watcher("/shooter/state", 32)}},
  };
  const auto diagnostics = LintGraph(nodes);
  ASSERT_TRUE(HasError(diagnostics)) << Describe(diagnostics);
  EXPECT_TRUE(Mentions(diagnostics, Severity::ERROR, "/shooter/state",
                       "two message sizes"))
      << Describe(diagnostics);
  EXPECT_TRUE(
      Mentions(diagnostics, Severity::ERROR, "/shooter/state", "shooter"))
      << Describe(diagnostics);
  EXPECT_TRUE(
      Mentions(diagnostics, Severity::ERROR, "/shooter/state", "odometry"))
      << Describe(diagnostics);
}

SourceShape Typed(SourceShape shape, std::string schema, std::uint64_t hash,
                  std::uint32_t slots = 1024) {
  shape.schema = std::move(schema);
  shape.schema_hash = hash;
  shape.alignment = 8;
  shape.slots = slots;
  return shape;
}

// A size match is not a compatibility proof: a field can change type without
// changing size. Describe reports each end's schema identity, and two
// schemas on one topic are an error naming both types and both nodes.
TEST(LintGraph, TwoSchemasOfTheSameSizeOnOneTopicIsAnErrorNamingBothTypes) {
  const std::vector<NodeShape> nodes = {
      {"shooter",
       "//2026-robot/main_processor/shooter:node",
       {Typed(Sender("/shooter/state", 24), "shooter.State", 0x1)}},
      {"odometry",
       "//2026-robot/main_processor/odometry:node",
       {Typed(Watcher("/shooter/state", 24), "shooter.OldState", 0x2)}},
  };
  const auto diagnostics = LintGraph(nodes);
  ASSERT_TRUE(HasError(diagnostics)) << Describe(diagnostics);
  EXPECT_TRUE(
      Mentions(diagnostics, Severity::ERROR, "/shooter/state", "two schemas"))
      << Describe(diagnostics);
  for (const auto* named :
       {"shooter.State (0x0000000000000001)",
        "shooter.OldState (0x0000000000000002)", "by shooter", "by odometry"}) {
    EXPECT_TRUE(Mentions(diagnostics, Severity::ERROR, "/shooter/state", named))
        << named << "\n"
        << Describe(diagnostics);
  }
  // One finding: the schema error explains the topic.
  EXPECT_EQ(std::count_if(diagnostics.begin(), diagnostics.end(),
                          [](const Diagnostic& d) {
                            return d.severity == Severity::ERROR;
                          }),
            1);
}

TEST(LintGraph, ASameNamedSchemaWithAnotherHashIsAnError) {
  const std::vector<NodeShape> nodes = {
      {"shooter",
       "//2026-robot/main_processor/shooter:node",
       {Typed(Sender("/shooter/state", 24), "shooter.State", 0x1)}},
      {"odometry",
       "//2026-robot/main_processor/odometry:node",
       {Typed(Watcher("/shooter/state", 24), "shooter.State", 0x3)}},
  };
  EXPECT_TRUE(Mentions(LintGraph(nodes), Severity::ERROR, "/shooter/state",
                       "two schemas"));
}

TEST(LintGraph, MatchingSchemasPassAndDifferentRingLayoutsDoNot) {
  std::vector<NodeShape> nodes = {
      {"shooter",
       "//2026-robot/main_processor/shooter:node",
       {Typed(Sender("/shooter/state", 24), "shooter.State", 0x1)}},
      {"odometry",
       "//2026-robot/main_processor/odometry:node",
       {Typed(Watcher("/shooter/state", 24), "shooter.State", 0x1)}},
  };
  EXPECT_FALSE(HasError(LintGraph(nodes))) << Describe(LintGraph(nodes));
  nodes[1].sources[0].slots = 64;
  EXPECT_TRUE(Mentions(LintGraph(nodes), Severity::ERROR, "/shooter/state",
                       "two ring layouts"));
}

TEST(LintGraph, UnreadTopicIsAWarningNotAnError) {
  const std::vector<NodeShape> nodes = {
      {"telemetry", "//talOS/telemetry:node", {Sender("/talos/telemetry")}},
  };
  const auto diagnostics = LintGraph(nodes);
  EXPECT_FALSE(HasError(diagnostics)) << Describe(diagnostics);
  EXPECT_TRUE(Mentions(diagnostics, Severity::WARNING, "/talos/telemetry",
                       "nothing subscribes"))
      << Describe(diagnostics);
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
  EXPECT_FALSE(HasError(diagnostics));
  EXPECT_TRUE(Mentions(diagnostics, Severity::WARNING, "wrist", "105 rps"))
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
  EXPECT_FALSE(HasError(diagnostics));
  EXPECT_TRUE(Mentions(diagnostics, Severity::WARNING, "rio", "15000 frames/s"))
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

TEST(HardwareConfig, ReportsAllWarningsWithoutRejectingConfiguration) {
  hardware::Config config;
  config.status_hz = 1000;
  config.motors.resize(4);
  config.motors[0].name = "wrist";
  config.motors[0].max_velocity_rps = 106;
  config.motors[1].name = "shoulder";
  config.motors[1].sensor_to_mechanism_ratio = 2;
  const auto diagnostics = CheckHardwareConfig(config);
  ASSERT_EQ(diagnostics.size(), 3u);
  EXPECT_FALSE(HasError(diagnostics));
  EXPECT_TRUE(Mentions(diagnostics, Severity::WARNING, "wrist", "105 rps"));
  EXPECT_TRUE(Mentions(diagnostics, Severity::WARNING, "shoulder", "105 rps"));
  EXPECT_TRUE(
      Mentions(diagnostics, Severity::WARNING, "rio", "15000 frames/s"));
  EXPECT_TRUE(CheckHardwareConfig(hardware::Config{}).empty());
}

TEST(LintGraph, RejectsTheTestNamespaceAndDuplicateNodeNames) {
  const std::vector<NodeShape> in_test_namespace = {
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {Sender("/test/state")}},
  };
  EXPECT_TRUE(HasError(LintGraph(in_test_namespace)));

  const std::vector<NodeShape> duplicated = {
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {Sender("/drivetrain/state")}},
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {Watcher("/drivetrain/state")}},
  };
  const auto diagnostics = LintGraph(duplicated);
  ASSERT_TRUE(HasError(diagnostics)) << Describe(diagnostics);
  EXPECT_TRUE(Mentions(diagnostics, Severity::ERROR, "drivetrain",
                       "identifies one process"))
      << Describe(diagnostics);
}

TEST(LintGraph, ReportsAMalformedNameOnceAndKeepsGoing) {
  const std::vector<NodeShape> nodes = {
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {Sender("/drivetrain/tgt"), Sender("/drivetrain/state")}},
      {"odometry",
       "//2026-robot/main_processor/odometry:node",
       {Watcher("/drivetrain/state"), Sender("/odometry/state")}},
  };
  const auto diagnostics = LintGraph(nodes);
  ASSERT_TRUE(HasError(diagnostics)) << Describe(diagnostics);
  EXPECT_TRUE(
      Mentions(diagnostics, Severity::ERROR, "/drivetrain/tgt", "target"))
      << Describe(diagnostics);
  // The declaring node is named, because the reader has to go and fix it.
  EXPECT_TRUE(
      Mentions(diagnostics, Severity::ERROR, "/drivetrain/tgt", "drivetrain"));
  // A bad name must not stop the rest of the graph being checked; /odometry
  // /state is published and unread, which should still show up.
  EXPECT_TRUE(Mentions(diagnostics, Severity::WARNING, "/odometry/state",
                       "nothing subscribes"))
      << Describe(diagnostics);
}

}  // namespace
}  // namespace talos::introspect::naming
