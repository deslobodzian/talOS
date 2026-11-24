// Unit tests for launcher phase-1 describe, graph, and roster rules
// (REQ-SAFE-28, REQ-OPS-01, REQ-OPS-03).
#include "talOS/launcher/describe_graph.h"

#include <gtest/gtest.h>
#include <sys/stat.h>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <string_view>
#include <vector>

#include "schema_test/shapes_generated.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/events/manifest.h"
#include "talOS/introspection/names.h"
#include "talOS/launcher/roster.h"
#include "talOS/schema/identity.h"

namespace talos::launcher {
namespace {

const NodeSpec* FindSpec(const std::vector<NodeSpec>& specs,
                         std::string_view name) {
  const auto it =
      std::find_if(specs.begin(), specs.end(),
                   [name](const NodeSpec& spec) { return spec.name == name; });
  return it == specs.end() ? nullptr : &*it;
}

naming::SourceShape MakeSource(event::SourceKind kind, std::string name) {
  naming::SourceShape source;
  source.kind = kind;
  source.name = std::move(name);
  return source;
}

// REQ-SAFE-28
TEST(DescribeGraphUnitTest, HardwareBridgeIsAlwaysCritical) {
  constexpr std::string_view kConfig = R"(
[robot]
name = "optional_everything"
period_us = 5000
hardware_node = "//example/bridge:node"

[subsystems.alpha]
node = "//example/alpha:node"
critical = false
)";
  const auto specs =
      DiscoverNodes(config::ParseRobotConfigString(kConfig), LauncherOptions{});
  const NodeSpec* bridge = FindSpec(specs, kHardwareNodeName);
  ASSERT_NE(bridge, nullptr);
  EXPECT_EQ(bridge->target, "//example/bridge:node");
  EXPECT_TRUE(bridge->critical);

  const NodeSpec* alpha = FindSpec(specs, "alpha");
  ASSERT_NE(alpha, nullptr);
  EXPECT_FALSE(alpha->critical);
  EXPECT_FALSE(alpha->owns_actuators);

  std::vector<NodeDescription> described;
  described.reserve(specs.size());
  for (const auto& spec : specs) {
    described.push_back(NodeDescription{
        .roster_name = spec.name, .target = spec.target, .answered = true});
  }
  EXPECT_TRUE(LintCriticalNodes(specs, described).empty());
}

// REQ-SAFE-28
TEST(DescribeGraphUnitTest, NonCriticalActuatorOwnersProduceGraphError) {
  struct ActuatorCase {
    const char* section;
    const char* device_toml;
  };
  const std::vector<ActuatorCase> cases{
      {"motors",
       "[subsystems.alpha.motors.lift]\n"
       "type = \"TalonFX\"\nbus = \"test\"\ncan_id = 1\n"
       "slot0 = { p = 0.0, i = 0.0, d = 0.0, s = 0.0, v = 0.0, a = 0.0, "
       "g = 0.0 }\n"},
      {"digital_outputs",
       "[subsystems.alpha.digital_outputs.solenoid]\ndio = 1\n"},
      {"pwm_outputs", "[subsystems.alpha.pwm_outputs.servo]\nchannel = 1\n"},
  };

  for (const auto& c : cases) {
    SCOPED_TRACE(c.section);
    const std::string base =
        "[robot]\nperiod_us = 5000\ncommand_timeout_us = 100000\n"
        "[subsystems.alpha]\nnode = \"//example/alpha:node\"\n";

    // critical = false on any actuator owner -> graph error naming the node.
    const auto non_critical_specs =
        DiscoverNodes(config::ParseRobotConfigString(
                          base + "critical = false\n" + c.device_toml),
                      LauncherOptions{});
    const NodeSpec* alpha = FindSpec(non_critical_specs, "alpha");
    ASSERT_NE(alpha, nullptr);
    EXPECT_FALSE(alpha->critical);
    EXPECT_TRUE(alpha->owns_actuators);

    std::vector<NodeDescription> described;
    described.reserve(non_critical_specs.size());
    for (const auto& spec : non_critical_specs) {
      described.push_back(NodeDescription{
          .roster_name = spec.name, .target = spec.target, .answered = true});
    }
    const auto findings = LintCriticalNodes(non_critical_specs, described);
    ASSERT_EQ(findings.size(), 1u);
    EXPECT_EQ(findings[0].severity, naming::Severity::ERROR);
    EXPECT_EQ(findings[0].subject, "alpha");
    EXPECT_NE(findings[0].message.find("critical=false"), std::string::npos)
        << findings[0].message;

    // Even when the actuator owner did not answer --describe (answered =
    // false), critical = false is still a graph ERROR.
    std::vector<NodeDescription> unanswered;
    unanswered.reserve(non_critical_specs.size());
    for (const auto& spec : non_critical_specs) {
      unanswered.push_back(NodeDescription{
          .roster_name = spec.name, .target = spec.target, .answered = false});
    }
    const auto unanswered_findings =
        LintCriticalNodes(non_critical_specs, unanswered);
    ASSERT_EQ(unanswered_findings.size(), 1u);
    EXPECT_EQ(unanswered_findings[0].severity, naming::Severity::ERROR);
    EXPECT_EQ(unanswered_findings[0].subject, "alpha");
    EXPECT_NE(unanswered_findings[0].message.find("critical=false"),
              std::string::npos)
        << unanswered_findings[0].message;

    // Default (critical = true) actuator owner -> no error.
    const auto critical_specs =
        DiscoverNodes(config::ParseRobotConfigString(base + c.device_toml),
                      LauncherOptions{});
    const NodeSpec* critical_alpha = FindSpec(critical_specs, "alpha");
    ASSERT_NE(critical_alpha, nullptr);
    EXPECT_TRUE(critical_alpha->critical);
    EXPECT_TRUE(critical_alpha->owns_actuators);
    EXPECT_TRUE(LintCriticalNodes(critical_specs, described).empty());
  }
}

// REQ-SAFE-28
TEST(DescribeGraphUnitTest, NonCriticalTargetLaneSenderProducesGraphError) {
  constexpr std::string_view kConfig = R"(
[robot]
period_us = 5000

[subsystems.alpha]
node = "//example/alpha:node"
critical = false
)";
  const auto specs =
      DiscoverNodes(config::ParseRobotConfigString(kConfig), LauncherOptions{});
  std::vector<NodeDescription> described;
  described.reserve(specs.size());
  for (const auto& spec : specs) {
    NodeDescription desc{
        .roster_name = spec.name, .target = spec.target, .answered = true};
    if (spec.name == "alpha") {
      desc.declared.sources.push_back(
          MakeSource(event::SourceKind::SENDER, "/beta/target/teleop"));
    }
    described.push_back(std::move(desc));
  }

  const auto findings = LintCriticalNodes(specs, described);
  ASSERT_EQ(findings.size(), 1u);
  EXPECT_EQ(findings[0].severity, naming::Severity::ERROR);
  EXPECT_EQ(findings[0].subject, "alpha");
  EXPECT_NE(findings[0].message.find("critical=false"), std::string::npos)
      << findings[0].message;

  // Reading a target lane (WATCHER) or sending a non-target topic is allowed
  // on a non-critical node.
  for (auto& desc : described) {
    if (desc.roster_name == "alpha") {
      desc.declared.sources = {
          MakeSource(event::SourceKind::WATCHER, "/beta/target/teleop"),
          MakeSource(event::SourceKind::SENDER, "/alpha/state"),
      };
    }
  }
  EXPECT_TRUE(LintCriticalNodes(specs, described).empty());
}

// REQ-OPS-34, REQ-OPS-35, REQ-OPS-38, REQ-OPS-39, REQ-CONTRACT-33 (VIS-1):
// LintTelemetryConfig promotes invalid [[subsystem.labels]] entries and
// invalid [studio] dashboard layouts to hard phase-1 ERROR diagnostics even
// when telemetry is non-critical or did not answer --describe.
TEST(DescribeGraphUnitTest, LintTelemetryConfigRefusesBadLabelsAndDashboard) {
  const auto make_alerts_sender = [](std::string topic) {
    const schema::Identity id = schema::Of<schema_test::Alerts>();
    naming::SourceShape source;
    source.kind = event::SourceKind::SENDER;
    source.name = std::move(topic);
    source.message_bytes = sizeof(schema_test::Alerts);
    source.alignment = alignof(schema_test::Alerts);
    source.schema = id.name;
    source.schema_hash = id.hash;
    source.bfbs.assign(id.bfbs.begin(), id.bfbs.end());
    return source;
  };
  const auto make_mixed_sender = [](std::string topic) {
    const schema::Identity id = schema::Of<schema_test::Mixed>();
    naming::SourceShape source;
    source.kind = event::SourceKind::SENDER;
    source.name = std::move(topic);
    source.message_bytes = sizeof(schema_test::Mixed);
    source.alignment = alignof(schema_test::Mixed);
    source.schema = id.name;
    source.schema_hash = id.hash;
    source.bfbs.assign(id.bfbs.begin(), id.bfbs.end());
    return source;
  };

  NodeDescription shooter{.roster_name = "shooter",
                          .target = "//example/shooter:node",
                          .answered = true};
  shooter.declared.name = "shooter";
  shooter.declared.target = "//example/shooter:node";
  shooter.declared.sources = {make_alerts_sender("/shooter/status"),
                              make_mixed_sender("/shooter/state")};

  NodeDescription telemetry_unanswered{.roster_name = "telemetry",
                                       .target = "//talOS/telemetry:node",
                                       .answered = false};
  const std::vector<NodeDescription> described{shooter, telemetry_unanswered};

  // Valid [[subsystem.labels]] produces no diagnostics.
  {
    const auto cfg = config::ParseRobotConfigString(R"(
[robot]
period_us = 10000
[subsystems.shooter]
node = "//example/shooter:node"
[subsystems.autonomous]
node = "//example/autonomous:node"
routines = ["preload", "none"]
[subsystems.telemetry]
node = "//talOS/telemetry:node"
critical = false
[[subsystems.telemetry.labels]]
channel = "shooter/status/routine_index"
from = "autonomous.routines"
)");
    EXPECT_TRUE(LintTelemetryConfig(cfg, described).empty());
  }

  // Invalid [[subsystem.labels]] cases (config-only and dictionary-level).
  struct LabelCase {
    const char* name;
    const char* telemetry_toml;
    const char* expect_substr;
  };
  const std::vector<LabelCase> label_cases{
      {"non-array labels", "labels = \"bad\"\n", "labels"},
      {"empty table entry", "[[subsystems.telemetry.labels]]\n",
       "labels[0].channel"},
      {"unknown key in label entry",
       "[[subsystems.telemetry.labels]]\nchannel = "
       "\"shooter/status/routine_index\"\nfrom = \"autonomous.routines\"\n"
       "extra = \"oops\"\n",
       "labels[0].extra"},
      {"missing from subsystem",
       "[[subsystems.telemetry.labels]]\nchannel = "
       "\"shooter/status/routine_index\"\nfrom = \"ghost.routines\"\n",
       "labels[0].from"},
      {"channel absent from dictionary",
       "[[subsystems.telemetry.labels]]\nchannel = \"shooter/status/nope\"\n"
       "from = \"autonomous.routines\"\n",
       "labels[0].channel"},
      {"excluded channel",
       "exclude = [\"shooter/status/routine_index\"]\n"
       "[[subsystems.telemetry.labels]]\nchannel = "
       "\"shooter/status/routine_index\"\nfrom = \"autonomous.routines\"\n",
       "labels[0].channel"},
      {"non-integer channel",
       "[[subsystems.telemetry.labels]]\nchannel = "
       "\"shooter/status/supply_v\"\nfrom = \"autonomous.routines\"\n",
       "labels[0].channel"},
      {"already labelled enum channel",
       "[[subsystems.telemetry.labels]]\nchannel = \"shooter/state/mode\"\n"
       "from = \"autonomous.routines\"\n",
       "labels[0].channel"},
      {"duplicate label entry for same channel",
       "[[subsystems.telemetry.labels]]\nchannel = "
       "\"shooter/status/routine_index\"\nfrom = \"autonomous.routines\"\n"
       "[[subsystems.telemetry.labels]]\nchannel = "
       "\"shooter/status/routine_index\"\nfrom = \"autonomous.routines\"\n",
       "labels[1].channel"},
  };
  for (const auto& c : label_cases) {
    SCOPED_TRACE(c.name);
    const std::string toml =
        "[robot]\nperiod_us = 10000\n"
        "[subsystems.shooter]\nnode = \"//example/shooter:node\"\n"
        "[subsystems.autonomous]\nnode = \"//example/autonomous:node\"\n"
        "routines = [\"preload\", \"none\"]\n"
        "[subsystems.telemetry]\nnode = \"//talOS/telemetry:node\"\n"
        "critical = false\n" +
        std::string{c.telemetry_toml};
    const auto cfg = config::ParseRobotConfigString(toml);
    const auto findings = LintTelemetryConfig(cfg, described);
    ASSERT_EQ(findings.size(), 1u);
    EXPECT_EQ(findings[0].severity, naming::Severity::ERROR);
    EXPECT_EQ(findings[0].subject, "telemetry");
    EXPECT_NE(findings[0].message.find(c.expect_substr), std::string::npos)
        << findings[0].message;
  }

  // Dashboard layout checks (file-level, dictionary channel presence, budget
  // with alerts, excluded channel, and no-telemetry config).
  const char* tmp = std::getenv("TEST_TMPDIR");
  ASSERT_NE(tmp, nullptr);
  const std::filesystem::path dir =
      std::filesystem::path{tmp} / "lint_telemetry_dashboard";
  std::filesystem::create_directories(dir);
  const std::filesystem::path layout_file = dir / "dashboard.toml";

  const auto run_dashboard = [&](const std::string& layout_text,
                                 const std::string& telemetry_section,
                                 const std::vector<NodeDescription>& nodes) {
    std::ofstream(layout_file) << layout_text;
    const std::string toml =
        "[robot]\nperiod_us = 10000\n"
        "[studio]\ndashboard = \"" +
        layout_file.string() +
        "\"\n"
        "[subsystems.shooter]\nnode = \"//example/shooter:node\"\n" +
        telemetry_section;
    const auto cfg = config::ParseRobotConfigString(toml);
    return LintTelemetryConfig(cfg, nodes);
  };

  const std::string telemetry_sec =
      "[subsystems.telemetry]\nnode = \"//talOS/telemetry:node\"\n"
      "critical = false\n";

  // Valid layout passes.
  {
    const auto findings = run_dashboard(
        "[dashboard]\ntitle = \"Drive\"\n"
        "[[widgets]]\nkind = \"number\"\nat = [0, 0, 2, 1]\n"
        "channel = \"shooter/status/supply_v\"\n",
        telemetry_sec, described);
    EXPECT_TRUE(findings.empty());
  }

  // File-level error (overlapping widgets) fails even with no telemetry node
  // and even when no producer answered --describe.
  {
    NodeDescription unanswered_shooter{.roster_name = "shooter",
                                       .target = "//example/shooter:node",
                                       .answered = false};
    const auto findings = run_dashboard(
        "[[widgets]]\nkind = \"number\"\nat = [0, 0, 2, 1]\n"
        "channel = \"shooter/status/supply_v\"\n"
        "[[widgets]]\nkind = \"number\"\nat = [1, 0, 2, 1]\n"
        "channel = \"shooter/status/supply_v\"\n",
        "", {unanswered_shooter});
    ASSERT_EQ(findings.size(), 1u);
    EXPECT_EQ(findings[0].severity, naming::Severity::ERROR);
    EXPECT_EQ(findings[0].subject, "studio");
    EXPECT_NE(findings[0].message.find("widgets[1].at"), std::string::npos)
        << findings[0].message;
  }

  // Dictionary-level channel absent fails.
  {
    const auto findings = run_dashboard(
        "[[widgets]]\nkind = \"number\"\nat = [0, 0, 2, 1]\n"
        "channel = \"shooter/status/missing\"\n",
        telemetry_sec, described);
    ASSERT_EQ(findings.size(), 1u);
    EXPECT_EQ(findings[0].severity, naming::Severity::ERROR);
    EXPECT_EQ(findings[0].subject, "studio");
    EXPECT_NE(findings[0].message.find("widgets[0].channel"), std::string::npos)
        << findings[0].message;
  }

  // Dictionary-level channel excluded by telemetry.exclude fails.
  {
    const auto findings = run_dashboard(
        "[[widgets]]\nkind = \"number\"\nat = [0, 0, 2, 1]\n"
        "channel = \"shooter/status/supply_v\"\n",
        telemetry_sec + "exclude = [\"shooter/status/supply_v\"]\n", described);
    ASSERT_EQ(findings.size(), 1u);
    EXPECT_EQ(findings[0].severity, naming::Severity::ERROR);
    EXPECT_EQ(findings[0].subject, "studio");
    EXPECT_NE(findings[0].message.find("widgets[0].channel"), std::string::npos)
        << findings[0].message;
  }

  // Dictionary-level budget overflow once alerts widget adds alert channels.
  {
    const auto findings = run_dashboard(
        "[dashboard]\nupdate_hz = 10\nbudget_bytes_per_s = 12532\n"
        "[[widgets]]\nkind = \"number\"\nat = [0, 0, 1, 1]\n"
        "channel = \"shooter/status/supply_v\"\n"
        "[[widgets]]\nkind = \"alerts\"\nat = [1, 0, 1, 1]\n",
        telemetry_sec, described);
    ASSERT_EQ(findings.size(), 1u);
    EXPECT_EQ(findings[0].severity, naming::Severity::ERROR);
    EXPECT_EQ(findings[0].subject, "studio");
    EXPECT_NE(findings[0].message.find("[dashboard].budget_bytes_per_s"),
              std::string::npos)
        << findings[0].message;
  }

  // When no producer answered (e.g. binaries not built yet), a file-valid
  // dashboard layout does not emit spurious dictionary-channel errors.
  {
    NodeDescription unanswered_shooter{.roster_name = "shooter",
                                       .target = "//example/shooter:node",
                                       .answered = false};
    const auto findings = run_dashboard(
        "[[widgets]]\nkind = \"number\"\nat = [0, 0, 2, 1]\n"
        "channel = \"shooter/status/supply_v\"\n",
        telemetry_sec, {unanswered_shooter, telemetry_unanswered});
    EXPECT_TRUE(findings.empty());
  }

  // Dictionary-page overflow from an oversized [[subsystem.labels]] array
  // (500 labels x 40 bytes exceeds the 16 KiB dictionary page) must be refused
  // in --describe-only rather than slipping through to non-critical runtime.
  {
    std::string big_names = "[";
    for (int i = 0; i < 500; ++i) {
      if (i > 0) big_names += ", ";
      big_names +=
          "\"label_" + std::to_string(i) + "_" + std::string(32, 'x') + "\"";
    }
    big_names += "]";
    const std::string cfg_text =
        "[robot]\nname = \"r\"\n"
        "[subsystems.auto]\nnode = \"//example/auto:node\"\n"
        "routines = " +
        big_names +
        "\n"
        "[subsystems.telemetry]\nnode = \"//talOS/telemetry:node\"\n"
        "[[subsystems.telemetry.labels]]\n"
        "channel = \"shooter/status/routine_index\"\n"
        "from = \"auto.routines\"\n";
    const auto cfg = config::ParseRobotConfigString(cfg_text);
    const auto findings = LintTelemetryConfig(cfg, described);
    ASSERT_EQ(findings.size(), 1u);
    EXPECT_EQ(findings[0].severity, naming::Severity::ERROR);
    EXPECT_EQ(findings[0].subject, "telemetry");
    EXPECT_NE(findings[0].message.find("shooter/status/routine_index"),
              std::string::npos)
        << findings[0].message;
  }

  // A non-critical node (gyro) that did not answer --describe leaves the
  // dictionary incomplete: a label or widget naming its channels is a warning
  // naming gyro, not an error refusing the launch. With every node answered,
  // the same absent channel is an error (the "channel absent from dictionary"
  // label case and the dictionary-level channel-absent dashboard case above).
  // Malformed label entries and an unparsable layout stay errors.
  {
    NodeDescription gyro_unanswered{.roster_name = "gyro",
                                    .target = "//example/gyro:node",
                                    .answered = false};
    const std::vector<NodeDescription> partial{shooter, gyro_unanswered,
                                               telemetry_unanswered};
    const std::string gyro_sec =
        "[subsystems.gyro]\nnode = \"//example/gyro:node\"\n"
        "critical = false\n"
        "[subsystems.autonomous]\nnode = \"//example/autonomous:node\"\n"
        "routines = [\"preload\", \"none\"]\n";
    const std::string gyro_label =
        "[[subsystems.telemetry.labels]]\nchannel = \"gyro/state/mode_index\"\n"
        "from = \"autonomous.routines\"\n";
    const std::string gyro_widget =
        "[[widgets]]\nkind = \"number\"\nat = [0, 0, 2, 1]\n"
        "channel = \"gyro/state/hardware_fresh\"\n";
    const std::string overlapping =
        gyro_widget + "[[widgets]]\nkind = \"number\"\nat = [1, 0, 2, 1]\n" +
        "channel = \"shooter/status/supply_v\"\n";
    struct PartialCase {
      const char* name;
      std::string layout;
      std::string telemetry_extra;
      naming::Severity severity;
      std::size_t count;
      const char* expect_substr;
    };
    const std::vector<PartialCase> partial_cases{
        {"label and widget on an unanswered node's channels", gyro_widget,
         gyro_label, naming::Severity::WARNING, 2, "gyro did not answer"},
        {"malformed label entry",
         "[[widgets]]\nkind = \"number\"\nat = [0, 0, 2, 1]\n"
         "channel = \"shooter/status/supply_v\"\n",
         "[[subsystems.telemetry.labels]]\n", naming::Severity::ERROR, 1,
         "labels[0].channel"},
        {"unparsable layout", overlapping, "", naming::Severity::ERROR, 1,
         "widgets[1].at"},
    };
    for (const auto& c : partial_cases) {
      SCOPED_TRACE(c.name);
      const auto findings = run_dashboard(
          c.layout, telemetry_sec + c.telemetry_extra + gyro_sec, partial);
      ASSERT_EQ(findings.size(), c.count);
      for (const auto& finding : findings) {
        EXPECT_EQ(finding.severity, c.severity) << finding.message;
        EXPECT_NE(finding.message.find(c.expect_substr), std::string::npos)
            << finding.message;
      }
    }
  }

  // A telemetry subsystem named in a robot repository's form,
  // `@talos//talOS/telemetry:node`, is linted whatever its subsystem name.
  {
    const auto cfg = config::ParseRobotConfigString(R"(
[robot]
period_us = 10000
[subsystems.shooter]
node = "//example/shooter:node"
[subsystems.autonomous]
node = "//example/autonomous:node"
routines = ["preload", "none"]
[subsystems.recorder]
node = "@talos//talOS/telemetry:node"
[[subsystems.recorder.labels]]
channel = "shooter/status/nope"
from = "autonomous.routines"
)");
    const auto findings = LintTelemetryConfig(cfg, {shooter});
    ASSERT_EQ(findings.size(), 1u);
    EXPECT_EQ(findings[0].severity, naming::Severity::ERROR);
    EXPECT_EQ(findings[0].subject, "recorder");
    EXPECT_NE(findings[0].message.find("labels[0].channel"), std::string::npos)
        << findings[0].message;
  }

  std::filesystem::remove_all(dir);
}

// REQ-OPS-22, REQ-CONTRACT-28: LintTelemetryConfig refuses a dictionary whose
// combined dictionary + value pages exceed the 64-slot `/talos/telemetry` ring
// (matching Telemetry construction).
TEST(DescribeGraphUnitTest,
     LintTelemetryConfigRefusesCombinedPagesOverSlotCount) {
  const schema::Identity alerts_id = schema::Of<schema_test::Alerts>();
  NodeDescription producer{
      .roster_name = "bank", .target = "//example/bank:node", .answered = true};
  producer.declared.name = "bank";
  producer.declared.target = "//example/bank:node";

  // 64 status topics each carrying routine_index labelled with 100 x 110-byte
  // labels (~12 KiB per channel -> 1 channel per dictionary page): 64
  // dictionary pages + value pages > 64 slots, while total channels (321) stay
  // far below kMaxChannels (2048).
  std::string big_labels = "[";
  for (int i = 0; i < 100; ++i) {
    if (i > 0) big_labels += ", ";
    big_labels +=
        "\"lbl_" + std::to_string(i) + "_" + std::string(100, 'x') + "\"";
  }
  big_labels += "]";

  std::string toml =
      "[robot]\nperiod_us = 10000\n"
      "[subsystems.bank]\nnode = \"//example/bank:node\"\n"
      "names = " +
      big_labels +
      "\n"
      "[subsystems.telemetry]\nnode = \"//talOS/telemetry:node\"\n";
  for (int i = 0; i < 64; ++i) {
    naming::SourceShape source;
    source.kind = event::SourceKind::SENDER;
    source.name = "/bank/status/ch" + std::to_string(i);
    source.message_bytes = sizeof(schema_test::Alerts);
    source.alignment = alignof(schema_test::Alerts);
    source.schema = alerts_id.name;
    source.schema_hash = alerts_id.hash;
    source.bfbs.assign(alerts_id.bfbs.begin(), alerts_id.bfbs.end());
    producer.declared.sources.push_back(std::move(source));

    toml +=
        "[[subsystems.telemetry.labels]]\n"
        "channel = \"bank/status/ch" +
        std::to_string(i) +
        "/routine_index\"\n"
        "from = \"bank.names\"\n";
  }
  const auto cfg = config::ParseRobotConfigString(toml);
  const auto findings = LintTelemetryConfig(cfg, {producer});
  ASSERT_EQ(findings.size(), 1u);
  EXPECT_EQ(findings[0].severity, naming::Severity::ERROR);
  EXPECT_EQ(findings[0].subject, "telemetry");
  EXPECT_NE(findings[0].message.find("64-slot"), std::string::npos)
      << findings[0].message;
}

// REQ-OPS-38, REQ-CONTRACT-10: LintParameterKeys accepts an explicit empty
// `labels = []` when telemetry reports reading `"labels"`.
TEST(DescribeGraphUnitTest, LintParameterKeysAcceptsEmptyTelemetryLabels) {
  const auto cfg = config::ParseRobotConfigString(R"(
[robot]
period_us = 10000
[subsystems.telemetry]
node = "//talOS/telemetry:node"
labels = []
)");
  NodeDescription telemetry{.roster_name = "telemetry",
                            .target = "//talOS/telemetry:node",
                            .answered = true};
  telemetry.declared.name = "telemetry";
  telemetry.declared.target = "//talOS/telemetry:node";
  telemetry.declared.params = std::vector<introspect::ParamRead>{
      {"telemetry", "stale_timeout_us"},  {"telemetry", "include"},
      {"telemetry", "exclude"},           {"telemetry", "labels"},
      {"telemetry", "labels[0].channel"}, {"telemetry", "labels[0].from"},
  };
  EXPECT_TRUE(LintParameterKeys(cfg, {telemetry}, "").empty());
}

void WriteExecutableStub(const std::filesystem::path& path,
                         std::string_view script) {
  std::ofstream out{path, std::ios::binary | std::ios::trunc};
  ASSERT_TRUE(out.is_open()) << path;
  out << script;
  out.close();
  ASSERT_EQ(::chmod(path.c_str(), 0755), 0) << path;
}

// REQ-OPS-01, REQ-OPS-03: a Pass-2 --graph probe failure clears any Pass-1
// declared.sources so stale topics never reach GraphToJson, topic counts,
// session segments, or target-lane criticality checks.
TEST(DescribeGraphUnitTest,
     Pass2FailureClearsStalePass1SourcesFromGraphAndTopics) {
  const std::filesystem::path dir =
      std::filesystem::path{::testing::TempDir()} /
      "describe_graph_pass2_failure";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);

  introspect::Description pass1;
  pass1.name = "reader";
  pass1.target = "//example/reader:node";
  pass1.reads_graph = true;
  naming::SourceShape state_sender =
      MakeSource(event::SourceKind::SENDER, "/reader/state");
  state_sender.message_bytes = 16;
  state_sender.alignment = 8;
  state_sender.slots = 16;
  state_sender.schema = "example.State";
  state_sender.schema_hash = 0x123456789abcdef0ULL;
  pass1.sources.push_back(state_sender);

  const std::filesystem::path stub = dir / "reader_stub.sh";
  ASSERT_NO_FATAL_FAILURE(
      WriteExecutableStub(stub,
                          "#!/bin/sh\n"
                          "for arg in \"$@\"; do\n"
                          "  if [ \"$arg\" = \"--graph\" ]; then\n"
                          "    exit 1\n"
                          "  fi\n"
                          "done\n"
                          "cat <<'TALOS_DESCRIBE'\n" +
                              introspect::DescribeToJson(pass1) +
                              "TALOS_DESCRIBE\n"
                              "exit 0\n"));

  NodeSpec spec;
  spec.name = "reader";
  spec.target = "//example/reader:node";
  spec.binary_path = stub.string();
  spec.critical = false;
  spec.owns_actuators = false;

  LauncherOptions options;
  options.config_path = (dir / "robot.toml").string();
  options.output_dir = dir.string();

  std::vector<NodeDescription> described;
  const auto diagnostics = DescribeGraph({spec}, options, described);
  EXPECT_FALSE(naming::HasError(diagnostics));
  ASSERT_EQ(described.size(), 1u);
  EXPECT_FALSE(described[0].answered);
  EXPECT_TRUE(described[0].declared.sources.empty());
  EXPECT_EQ(CountDeclaredTopics(described), 0u);

  const std::string json =
      GraphToJson(1, options.config_path, described, diagnostics);
  EXPECT_NE(json.find("\"described\": false"), std::string::npos) << json;
  EXPECT_NE(json.find("\"sources\": []"), std::string::npos) << json;
  EXPECT_EQ(json.find("/reader/state"), std::string::npos) << json;

  // Synthetic unanswered NodeDescription carrying stale sources (including a
  // target-lane sender and invalid slots = 0) must be ignored by all
  // downstream helpers.
  NodeSpec synthetic_spec;
  synthetic_spec.name = "reader";
  synthetic_spec.target = "//example/reader:node";
  synthetic_spec.critical = false;
  synthetic_spec.owns_actuators = false;

  NodeDescription synthetic{.roster_name = "reader",
                            .target = "//example/reader:node",
                            .answered = false,
                            .error = "probe failed"};
  naming::SourceShape stale_target =
      MakeSource(event::SourceKind::SENDER, "/beta/target/teleop");
  stale_target.message_bytes = 16;
  stale_target.alignment = 8;
  stale_target.slots = 0;
  stale_target.schema = "example.Target";
  stale_target.schema_hash = 0x1111222233334444ULL;
  synthetic.declared.sources.push_back(std::move(stale_target));

  EXPECT_EQ(CountDeclaredTopics({synthetic}), 0u);
  EXPECT_TRUE(LintCriticalNodes({synthetic_spec}, {synthetic}).empty());
  EXPECT_NO_THROW({
    const std::vector<RTMSQueue> segments = CreateSessionSegments({synthetic});
    EXPECT_TRUE(segments.empty());
  });

  const std::string synthetic_json = GraphToJson(2, "", {synthetic}, {});
  EXPECT_NE(synthetic_json.find("\"described\": false"), std::string::npos)
      << synthetic_json;
  EXPECT_NE(synthetic_json.find("\"sources\": []"), std::string::npos)
      << synthetic_json;
  EXPECT_EQ(synthetic_json.find("/beta/target/teleop"), std::string::npos)
      << synthetic_json;

  std::filesystem::remove_all(dir);
}

// REQ-OPS-01: ProbeDescribe refuses a probe that prints valid JSON and then
// exits non-zero or dies by a fatal signal, clearing `out` rather than
// keeping parsed or pre-existing fields.
TEST(DescribeGraphUnitTest,
     ProbeDescribeRefusesNonZeroExitOrSignalAfterValidJson) {
  const std::filesystem::path dir =
      std::filesystem::path{::testing::TempDir()} /
      "describe_graph_probe_exit_status";
  std::filesystem::remove_all(dir);
  std::filesystem::create_directories(dir);

  introspect::Description valid;
  valid.name = "probe_node";
  valid.target = "//example/probe_node:node";
  naming::SourceShape state_sender =
      MakeSource(event::SourceKind::SENDER, "/probe_node/state");
  state_sender.message_bytes = 16;
  state_sender.alignment = 8;
  state_sender.slots = 16;
  state_sender.schema = "example.State";
  state_sender.schema_hash = 0x123456789abcdef0ULL;
  valid.sources.push_back(state_sender);
  const std::string valid_json = introspect::DescribeToJson(valid);

  const std::filesystem::path ok_stub = dir / "ok.sh";
  ASSERT_NO_FATAL_FAILURE(WriteExecutableStub(ok_stub,
                                              "#!/bin/sh\n"
                                              "cat <<'TALOS_DESCRIBE'\n" +
                                                  valid_json +
                                                  "TALOS_DESCRIBE\n"
                                                  "exit 0\n"));

  const std::filesystem::path exit1_stub = dir / "exit1.sh";
  ASSERT_NO_FATAL_FAILURE(WriteExecutableStub(exit1_stub,
                                              "#!/bin/sh\n"
                                              "cat <<'TALOS_DESCRIBE'\n" +
                                                  valid_json +
                                                  "TALOS_DESCRIBE\n"
                                                  "exit 1\n"));

  const std::filesystem::path abrt_stub = dir / "abrt.sh";
  ASSERT_NO_FATAL_FAILURE(
      WriteExecutableStub(abrt_stub,
                          "#!/bin/sh\n"
                          "cat <<'TALOS_DESCRIBE'\n" +
                              valid_json +
                              "TALOS_DESCRIBE\n"
                              "ulimit -c 0 2>/dev/null || true\n"
                              "kill -ABRT $$\n"));

  // Positive control: the same JSON with exit 0 succeeds.
  {
    introspect::Description out;
    std::string error;
    EXPECT_TRUE(ProbeDescribe({ok_stub.string(), "--describe"},
                              std::chrono::seconds{10}, out, error))
        << error;
    EXPECT_EQ(out.name, "probe_node");
    ASSERT_EQ(out.sources.size(), 1u);
  }

  // Non-zero exit after valid JSON is refused and clears pre-seeded `out`.
  {
    introspect::Description out;
    out.name = "sentinel";
    out.sources.push_back(
        MakeSource(event::SourceKind::SENDER, "/sentinel/state"));
    std::string error;
    EXPECT_FALSE(ProbeDescribe({exit1_stub.string(), "--describe"},
                               std::chrono::seconds{10}, out, error));
    EXPECT_TRUE(out.sources.empty());
    EXPECT_TRUE(out.name.empty());
    EXPECT_NE(error.find("exit code 1"), std::string::npos) << error;
  }

  // Fatal signal after valid JSON is refused and clears pre-seeded `out`.
  {
    introspect::Description out;
    out.name = "sentinel";
    out.sources.push_back(
        MakeSource(event::SourceKind::SENDER, "/sentinel/state"));
    std::string error;
    EXPECT_FALSE(ProbeDescribe({abrt_stub.string(), "--describe"},
                               std::chrono::seconds{10}, out, error));
    EXPECT_TRUE(out.sources.empty());
    EXPECT_TRUE(out.name.empty());
    EXPECT_NE(error.find("SIGABRT"), std::string::npos) << error;
  }

  std::filesystem::remove_all(dir);
}

}  // namespace
}  // namespace talos::launcher
