// REQ-RT-23: the launcher's graph lint refuses a Python actuator owner that
// runs faster than kMinPythonActuatorPeriodUs.
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "talOS/configuration/config_parser.h"
#include "talOS/introspection/names.h"
#include "talOS/launcher/describe_graph.h"

namespace talos::launcher {
namespace {

enum class Actuator { kNone, kMotor, kDigitalOutput, kPwmOutput };

struct Case {
  const char* label;
  bool abi_update;     // The node's update() is Python's.
  Actuator actuator;   // What actuator, if any, its own subsystem owns.
  const char* period;  // The subsystem's period_us line, or "" for [robot]'s.
  std::string refused_period;  // Empty when the graph is accepted.
};

// [robot] period_us is 5000, so a subsystem without its own runs at 5 ms.
std::string RobotToml(const Case& c) {
  std::string toml =
      "[robot]\nperiod_us = 5000\ncommand_timeout_us = 100000\n"
      "[subsystems.arm]\nnode = \"//arm:node\"\n";
  toml += c.period;
  switch (c.actuator) {
    case Actuator::kNone:
      break;
    case Actuator::kMotor:
      toml +=
          "[subsystems.arm.motors.lift]\ntype = \"TalonFX\"\nbus = \"test\"\n"
          "can_id = 1\nslot0 = { p = 0.0, i = 0.0, d = 0.0, s = 0.0, v = 0.0, "
          "a = 0.0, g = 0.0 }\n";
      break;
    case Actuator::kDigitalOutput:
      toml += "[subsystems.arm.digital_outputs.solenoid]\ndio = 1\n";
      break;
    case Actuator::kPwmOutput:
      toml += "[subsystems.arm.pwm_outputs.servo]\nchannel = 1\n";
      break;
  }
  return toml;
}

TEST(LintPythonActuatorPeriods, RefusesAPythonActuatorOwnerFasterThan20Ms) {
  const std::vector<Case> cases{
      {"python motor owner at 10 ms", true, Actuator::kMotor,
       "period_us = 10000\n", "10000"},
      {"python motor owner at [robot]'s 5 ms", true, Actuator::kMotor, "",
       "5000"},
      {"python motor owner at 20 ms", true, Actuator::kMotor,
       "period_us = 20000\n", ""},
      {"c++ motor owner at 5 ms", false, Actuator::kMotor, "period_us = 5000\n",
       ""},
      {"python DIO-only owner at 10 ms", true, Actuator::kDigitalOutput,
       "period_us = 10000\n", "10000"},
      {"python DIO-only owner at 20 ms", true, Actuator::kDigitalOutput,
       "period_us = 20000\n", ""},
      {"c++ DIO-only owner at 5 ms", false, Actuator::kDigitalOutput,
       "period_us = 5000\n", ""},
      {"python PWM-only owner at 10 ms", true, Actuator::kPwmOutput,
       "period_us = 10000\n", "10000"},
      {"python PWM-only owner at 20 ms", true, Actuator::kPwmOutput,
       "period_us = 20000\n", ""},
      {"c++ PWM-only owner at 5 ms", false, Actuator::kPwmOutput,
       "period_us = 5000\n", ""},
      {"python observer at 10 ms", true, Actuator::kNone, "period_us = 10000\n",
       ""},
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(c.label);
    const config::RobotConfig robot =
        config::ParseRobotConfigString(RobotToml(c));
    NodeDescription arm{.roster_name = "arm", .answered = true};
    arm.declared.abi_update = c.abi_update;
    const auto findings = LintPythonActuatorPeriods(robot, {arm});
    if (c.refused_period.empty()) {
      EXPECT_TRUE(findings.empty())
          << (findings.empty() ? "" : findings.front().message);
      continue;
    }
    ASSERT_EQ(findings.size(), 1u);
    EXPECT_EQ(findings[0].severity, naming::Severity::ERROR);
    EXPECT_EQ(findings[0].subject, "arm");
    EXPECT_NE(findings[0].message.find(c.refused_period), std::string::npos)
        << findings[0].message;
    EXPECT_NE(
        findings[0].message.find(std::to_string(kMinPythonActuatorPeriodUs)),
        std::string::npos)
        << findings[0].message;
  }
}

}  // namespace
}  // namespace talos::launcher
