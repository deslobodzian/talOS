// Unit tests for launcher phase-1 graph/roster rules without spawning
// subprocesses (REQ-SAFE-28).
#include "talOS/launcher/describe_graph.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <string>
#include <string_view>
#include <vector>

#include "talOS/configuration/config_parser.h"
#include "talOS/events/manifest.h"
#include "talOS/introspection/names.h"
#include "talOS/launcher/roster.h"

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

}  // namespace
}  // namespace talos::launcher
