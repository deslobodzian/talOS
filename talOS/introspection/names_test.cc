// The naming protocol's own test.
//
// Two jobs. The first is the grammar: what the rules accept and reject, and
// that a rejection says what to write instead -- an error that only says "bad
// name" leaves the reader to guess, which is how the bad name got written.
//
// The second is regression. Every name in
// RejectsTheMistakesThatActuallyHappened is one that shipped in this repo and
// cost real debugging: two spellings of one topic that both worked and never
// met. They are listed by name so that no future edit to the rules can quietly
// re-admit them.
//
// The real robot's graph is not checked here -- that needs the binaries, so the
// launcher does it with `--describe` before it spawns anything, and
// launcher_test covers it.

#include "talOS/introspection/names.h"

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
           "/hw/command/override",
           "/hw/state/driver_station",
           "/hw/request/drivetrain",
           "/hw/request/shooter",
           "/drivetrain/state",
           "/drivetrain/target",
           "/drivetrain/target/teleop",
           "/drivetrain/target/auto",
           "/shooter/state",
           "/shooter/target",
           "/driver_station/state",
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
    ASSERT_TRUE(problem.has_value()) << name << " must be rejected: " << why;
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
    ASSERT_TRUE(problem.has_value()) << name;
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

// The limit is the transport's, not the event loop's: a topic is a POSIX
// shared-memory object and macOS caps a shm name at PSHMNAMLEN, which
// rtms::ValidatePath enforces as 30 characters after the leading slash. A
// validator that accepted the event loop's 63 would pass names that throw
// inside a node's constructor at startup, which is worse than no validator.
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

// The budget is tight enough to hit by accident, so the shapes that are close
// to it are pinned here rather than discovered on a robot. Every topic the
// robot uses today fits; the obvious next one does not.
TEST(TopicName, TheBudgetIsTightAndTheHeadroomIsKnown) {
  // The longest name in the system today.
  EXPECT_EQ(std::string_view{"/drivetrain/target/teleop"}.size(), 25u);
  EXPECT_FALSE(CheckTopicName("/drivetrain/target/teleop").has_value());

  // A name a person would reasonably write next, which does not fit. When that
  // happens the qualifier is what gives -- the owner and the role are what
  // make the graph readable.
  EXPECT_GT(std::string_view{"/operator_interface/target/teleop"}.size(),
            kMaxTopicBytes);
  const auto problem = CheckTopicName("/operator_interface/target/teleop");
  ASSERT_TRUE(problem.has_value());
  EXPECT_NE(problem->find("limit"), std::string::npos) << *problem;
}

TEST(TopicName, OwnerAndRoleAreReadableFromTheNameAlone) {
  EXPECT_EQ(TopicOwner("/drivetrain/target/teleop"), "drivetrain");
  EXPECT_EQ(TopicRole("/drivetrain/target/teleop"), "target");
  EXPECT_EQ(TopicOwner("/hw/request/shooter"), "hw");
  EXPECT_TRUE(IsReservedOwner(TopicOwner("/talos/telemetry")));
  EXPECT_FALSE(IsReservedOwner(TopicOwner("/odometry/state")));
  // A name that does not parse yields empty rather than something misleading.
  EXPECT_TRUE(TopicOwner("odometry").empty());
  EXPECT_TRUE(TopicRole("/odometry").empty());
}

TEST(NodeName, MatchesTheSubsystemKeyRules) {
  for (const auto* name :
       {"drivetrain", "shooter", "odometry", "arbiter", "driver_station",
        "operator_interface", "telemetry", "hardware_node"}) {
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
  ASSERT_TRUE(problem.has_value());
  EXPECT_NE(problem->find("odometry"), std::string::npos) << *problem;

  EXPECT_TRUE(CheckNodeTarget("odometry", "odometry/node").has_value());
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
  for (const auto& diagnostic : diagnostics)
    out += "\n  " + diagnostic.ToString();
  return out.empty() ? " (none)" : out;
}

TEST(LintGraph, AcceptsAWellFormedGraph) {
  const std::vector<NodeShape> nodes = {
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {Watcher("/drivetrain/target"), Sender("/drivetrain/state")}},
      {"arbiter",
       "//2026-robot/main_processor/arbiter:node",
       {Watcher("/drivetrain/target/teleop"), Sender("/drivetrain/target")}},
      {"operator_interface",
       "//2026-robot/main_processor/operator_interface:node",
       {Sender("/drivetrain/target/teleop")}},
      {"odometry",
       "//2026-robot/main_processor/odometry:node",
       {Watcher("/drivetrain/state"), Sender("/odometry/state")}},
      {"telemetry",
       "//2026-robot/main_processor/telemetry:node",
       // The Studio bridge reads /talos/telemetry but is not a registered
       // node, so the feed is declared external rather than left to read as a
       // topic nobody consumes.
       {Watcher("/odometry/state"),
        Sender("/talos/telemetry", 32, kSourceFlagExternal)}},
  };
  const auto diagnostics = LintGraph(nodes);
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
      {"arbiter",
       "//2026-robot/main_processor/arbiter:node",
       {Sender("/drivetrain/target")}},
      {"operator_interface",
       "//2026-robot/main_processor/operator_interface:node",
       {Sender("/drivetrain/target")}},
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {Watcher("/drivetrain/target")}},
  };
  const auto diagnostics = LintGraph(nodes);
  ASSERT_TRUE(HasError(diagnostics)) << Describe(diagnostics);
  EXPECT_TRUE(
      Mentions(diagnostics, Severity::ERROR, "/drivetrain/target", "arbiter"));
  EXPECT_TRUE(Mentions(diagnostics, Severity::ERROR, "/drivetrain/target",
                       "operator_interface"));
  // Replay is the reason, and the message should say so.
  EXPECT_TRUE(Mentions(diagnostics, Severity::ERROR, "/drivetrain/target",
                       "replayable"));
}

// The owner segment names the subject, and the role says which way the data
// moves relative to it. Getting this backwards was the first draft's mistake:
// "the owner is the publisher" reads as obvious and is false, because the
// arbiter publishes /drivetrain/target and the drivetrain consumes it.
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
  // wrong or the arbitration is confused.
  const std::vector<NodeShape> nodes = {
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {Sender("/drivetrain/target"), Watcher("/drivetrain/target")}},
  };
  const auto diagnostics = LintGraph(nodes);
  ASSERT_TRUE(HasError(diagnostics)) << Describe(diagnostics);
  EXPECT_TRUE(Mentions(diagnostics, Severity::ERROR, "/drivetrain/target",
                       "not by the subsystem being asked"))
      << Describe(diagnostics);
}

// A reserved namespace is exempt from both directions: the bridge publishes
// /hw/* on behalf of a controller processor that is not a talOS node at all,
// so neither "the owner publishes it" nor "the owner consumes it" applies.
TEST(LintGraph, ReservedNamespacesAreExemptFromTheOwnerRule) {
  const std::vector<NodeShape> nodes = {
      {"hardware_node",
       "//talOS/bridge:hardware_node",
       {Sender("/hw/state"), Watcher("/hw/request/drivetrain")}},
      {"driver_station",
       "//2026-robot/main_processor/driver_station:node",
       {Watcher("/hw/state"), Sender("/driver_station/state")}},
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {Sender("/hw/request/drivetrain"), Watcher("/hw/state")}},
      {"arbiter",
       "//2026-robot/main_processor/arbiter:node",
       {Watcher("/driver_station/state")}},
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
       "//2026-robot/main_processor/telemetry:node",
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
           // A debug hook with no in-tree publisher.
           Watcher("/hw/command/override", 32, kSourceFlagOptional),
       }},
      {"arbiter",
       "//2026-robot/main_processor/arbiter:node",
       {
           // Autonomous is not written yet.
           Watcher("/drivetrain/target/auto", 32, kSourceFlagOptional),
           Sender("/drivetrain/target"),
       }},
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {Watcher("/drivetrain/target")}},
  };
  const auto diagnostics = LintGraph(nodes);
  EXPECT_FALSE(HasError(diagnostics)) << Describe(diagnostics);
  for (const auto& diagnostic : diagnostics) {
    EXPECT_NE(diagnostic.subject, "/hw/command");
    EXPECT_NE(diagnostic.subject, "/hw/command/override");
    EXPECT_NE(diagnostic.subject, "/drivetrain/target/auto");
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
  for (const auto* label : {"swerve", "shooter", "telemetry"})
    EXPECT_TRUE(CheckTopicName(label).has_value())
        << label << " would pass as a topic, so this test proves nothing";
}

TEST(LintGraph, UnreadTopicIsAWarningNotAnError) {
  const std::vector<NodeShape> nodes = {
      {"telemetry",
       "//2026-robot/main_processor/telemetry:node",
       {Sender("/talos/telemetry")}},
  };
  const auto diagnostics = LintGraph(nodes);
  EXPECT_FALSE(HasError(diagnostics)) << Describe(diagnostics);
  EXPECT_TRUE(Mentions(diagnostics, Severity::WARNING, "/talos/telemetry",
                       "nothing subscribes"))
      << Describe(diagnostics);
}

TEST(LintGraph, ErrorsSortAheadOfWarnings) {
  const std::vector<NodeShape> nodes = {
      {"telemetry",
       "//2026-robot/main_processor/telemetry:node",
       {Sender("/talos/telemetry")}},
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

// A prefix family is the arbitration shape and carries no extra rule: RTMS
// lookups are exact, so a parent name never captures a child's traffic. What
// this pins is that the linter does not invent a complaint about the shape --
// an earlier draft checked that a family shared one owner, which cannot fail,
// since the owner is the first segment.
TEST(LintGraph, PrefixFamilyIsPermitted) {
  const std::vector<NodeShape> nodes = {
      {"arbiter",
       "//2026-robot/main_processor/arbiter:node",
       {Watcher("/drivetrain/target/teleop"),
        Watcher("/drivetrain/target/auto", 32, kSourceFlagOptional),
        Sender("/drivetrain/target")}},
      {"operator_interface",
       "//2026-robot/main_processor/operator_interface:node",
       {Sender("/drivetrain/target/teleop")}},
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {Watcher("/drivetrain/target")}},
      // The same shape under a reserved namespace, with two different nodes on
      // the parent and the child.
      {"hardware_node",
       "//talOS/bridge:hardware_node",
       {Sender("/hw/state"), Sender("/hw/state/driver_station")}},
      {"driver_station",
       "//2026-robot/main_processor/driver_station:node",
       {Watcher("/hw/state"), Watcher("/hw/state/driver_station")}},
  };
  const auto diagnostics = LintGraph(nodes);
  EXPECT_TRUE(diagnostics.empty()) << Describe(diagnostics);
}

TEST(LintGraph, OneTopicMayNotHaveTwoMessageSizes) {
  const std::vector<NodeShape> nodes = {
      {"drivetrain",
       "//2026-robot/main_processor/drivetrain:node",
       {Sender("/drivetrain/state", 48)}},
      {"odometry",
       "//2026-robot/main_processor/odometry:node",
       {Watcher("/drivetrain/state", 32)}},
  };
  const auto diagnostics = LintGraph(nodes);
  ASSERT_TRUE(HasError(diagnostics)) << Describe(diagnostics);
  EXPECT_TRUE(Mentions(diagnostics, Severity::ERROR, "/drivetrain/state",
                       "two message sizes"))
      << Describe(diagnostics);
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
