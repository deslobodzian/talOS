// Unit tests for launcher phase-1 graph/roster rules without spawning
// subprocesses (REQ-SAFE-28).
#include "talOS/launcher/describe_graph.h"

#include <gtest/gtest.h>

#include <algorithm>
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

// VIS-1 (REQ-OPS-34, REQ-OPS-35, REQ-OPS-38, REQ-OPS-39, REQ-CONTRACT-33):
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

  std::filesystem::remove_all(dir);
}

}  // namespace
}  // namespace talos::launcher
