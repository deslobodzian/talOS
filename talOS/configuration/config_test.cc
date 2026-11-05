#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "config_parser.h"

namespace talos::config {
namespace {

// One valid motor, so a fixture only spells out what it is testing.
constexpr std::string_view kMotor = R"(
[subsystems.test.motors.m1]
can_id = 1
)";

RobotConfig Parse(std::string_view body) {
  return ParseRobotConfigString(std::string(kMotor) + std::string(body));
}

void ExpectRejected(std::string_view body) {
  EXPECT_THROW(Parse(body), std::invalid_argument) << body;
}

TEST(ConfigTest, ParsesEveryDeviceTableIntoOwnership) {
  const auto cfg = Parse(R"(
[subsystems.test.sensors.enc]
can_id = 2
[subsystems.test.sensors.imu]
type = "Pigeon2"
can_id = 3
[subsystems.test.motors.steer]
can_id = 4
feedback = "RemoteCANcoder"
feedback_sensor = "enc"
[subsystems.test.digital_inputs.beam]
dio = 0
[subsystems.test.digital_outputs.light]
dio = 1
[subsystems.test.analog_inputs.pressure]
channel = 0
[subsystems.test.encoders.quad]
dio_a = 2
dio_b = 3
[subsystems.test.encoders.abs]
type = "DutyCycleEncoder"
dio = 4
[subsystems.test.pwm_outputs.servo]
channel = 0
)");
  const auto& hw = cfg.hardware;
  ASSERT_EQ(hw.motors.size(), 2u);
  ASSERT_EQ(hw.sensors.size(), 2u);
  EXPECT_EQ(hw.digital_inputs.size(), 1u);
  EXPECT_EQ(hw.digital_outputs.size(), 1u);
  EXPECT_EQ(hw.analog_inputs.size(), 1u);
  ASSERT_EQ(hw.encoders.size(), 2u);
  EXPECT_EQ(hw.pwm_outputs.size(), 1u);

  // Sorted by name: abs, beam, enc, imu, light, m1, pressure, quad, servo,
  // steer.
  EXPECT_EQ(hw.encoders[0].kind, hardware::EncoderKind::kDutyCycle);
  EXPECT_EQ(hw.encoders[0].dio_a, 4);
  EXPECT_EQ(hw.sensors[1].kind, hardware::SensorKind::kPigeon2);
  EXPECT_EQ(hw.motors[1].name, "steer");
  EXPECT_EQ(hw.motors[1].feedback, hardware::Feedback::kRemoteCANcoder);
  EXPECT_EQ(hw.motors[1].feedback_sensor_id, hw.sensors[0].id);

  const auto* devs = cfg.GetDevices("test");
  ASSERT_NE(devs, nullptr);
  EXPECT_EQ(devs->motors.size(), 2u);
  EXPECT_EQ(devs->pwm_outputs, std::vector<uint16_t>{hw.pwm_outputs[0].id});
}

TEST(ConfigTest, DeterministicOrderingIndependentOfTomlTableOrder) {
  const auto config = ParseRobotConfigString(R"(
[subsystems.sub_z.motors.z_motor]
can_id = 1
[subsystems.sub_a.motors.b_motor]
can_id = 2
[subsystems.sub_a.motors.a_motor]
can_id = 3
[subsystems.sub_a.sensors.s_sensor]
can_id = 4
)");
  ASSERT_EQ(config.hardware.motors.size(), 3u);
  EXPECT_EQ(config.hardware.motors[0].name, "a_motor");
  EXPECT_EQ(config.hardware.motors[0].id, 1);
  EXPECT_EQ(config.hardware.motors[1].name, "b_motor");
  EXPECT_EQ(config.hardware.motors[1].id, 2);
  EXPECT_EQ(config.hardware.sensors[0].id, 3);
  EXPECT_EQ(config.hardware.motors[2].name, "z_motor");
  EXPECT_EQ(config.hardware.motors[2].id, 4);
  EXPECT_EQ(config.GetDevices("sub_a")->motors, (std::vector<uint16_t>{1, 2}));
  EXPECT_EQ(config.GetDevices("sub_z")->motors, (std::vector<uint16_t>{4}));
}

TEST(ConfigTest, RejectsParserOwnedErrors) {
  ExpectRejected("[subsystems.test.motors.m2]\nbus = \"rio\"\n");  // no can_id
  ExpectRejected("[subsystems.test.motors.m2]\ncan_id = \"2\"\n");
  ExpectRejected(
      "[subsystems.test.motors.m2]\ncan_id = 2\ntype = \"SparkMax\"\n");
  ExpectRejected(
      "[subsystems.test.sensors.s]\ncan_id = 2\ntype = \"kCANcoder\"\n");
  ExpectRejected(
      "[subsystems.test.encoders.e]\ntype = \"Quadrature\"\ndio_a = 0\ndio_b = "
      "1\n");
  ExpectRejected("[subsystems.test.encoders.e]\ndio_a = 0\n");  // no dio_b
  ExpectRejected("[subsystems.test.analog_inputs.a]\nanalog = 0\n");
  ExpectRejected("[subsystems.test.pwm_outputs.p]\npwm = 0\n");
  ExpectRejected(
      "[subsystems.test.motors.m2]\ncan_id = 2\nfeedback = \"rotor\"\n");
  ExpectRejected(
      "[subsystems.test.motors.m2]\ncan_id = 2\nfeedback = "
      "\"RemoteCANcoder\"\n");
  ExpectRejected(R"(
[subsystems.test.sensors.enc]
can_id = 2
[subsystems.test.motors.m2]
can_id = 3
feedback_sensor = "enc"
)");  // Rotor feedback naming a sensor.
  ExpectRejected(R"(
[subsystems.test.motors.steer]
can_id = 2
feedback = "RemoteCANcoder"
feedback_sensor = "ghost_encoder"
)");
  ExpectRejected("[unknown_top_level]\nfoo = 1\n");
  EXPECT_THROW(
      ParseRobotConfigString("typo_key = \"val\"\n" + std::string(kMotor)),
      std::invalid_argument);
}

TEST(ConfigTest, RejectsCrossSubsystemClaims) {
  ExpectRejected(R"(
[subsystems.subA.sensors.encA]
can_id = 20
[subsystems.subB.motors.steerB]
can_id = 2
feedback = "RemoteCANcoder"
feedback_sensor = "encA"
)");
  ExpectRejected(R"(
[subsystems.drivetrain.motors.steer]
can_id = 2
[subsystems.drivetrain.geometry]
modules = [{ name = "m1", drive = "m1", steer = "steer" }]
)");
  ExpectRejected(R"(
[subsystems.drivetrain.motors.steer]
can_id = 2
[subsystems.drivetrain.geometry]
modules = [{ name = "m1", drive = "steer", steer = "nonexistent_steer" }]
)");
}

// Ranges and exclusive ownership belong to hardware::Validate; these prove the
// parsed fields reach it.
TEST(ConfigTest, ValidateRunsOnParsedConfig) {
  ExpectRejected("[subsystems.other.motors.m2]\ncan_id = 1\n");
  ExpectRejected(R"(
[subsystems.test.digital_inputs.d]
dio = 3
[subsystems.test.encoders.e]
dio_a = 3
dio_b = 4
)");
  ExpectRejected("[robot]\nperiod_us = 500\n");
  ExpectRejected("[robot]\ncommand_timeout_us = 6000\n");
  ExpectRejected(
      "[subsystems.test.sensors.s]\ncan_id = 20\noffset_rot = 0.5\n");
  ExpectRejected(
      "[subsystems.test.motors.m2]\ncan_id = 2\nslot0 = { p = -0.5 }\n");
  ExpectRejected(R"(
[subsystems.test.sensors.imu]
type = "Pigeon2"
can_id = 30
[subsystems.test.motors.steer]
can_id = 2
feedback = "RemoteCANcoder"
feedback_sensor = "imu"
)");
  EXPECT_THROW(
      ParseRobotConfigString("[subsystems.test.sensors.s]\ncan_id = 1\n"),
      std::invalid_argument);  // No motors.
}

TEST(ConfigTest, ControllerAddressDefaultsAndOverrides) {
  const auto defaults = Parse("");
  EXPECT_EQ(defaults.controller_address, "127.0.0.1");
  EXPECT_EQ(defaults.controller_port, 5802);
  EXPECT_EQ(defaults.local_port, 5803);

  const auto config = Parse(R"(
[robot]
controller_address = "10.56.87.2"
controller_port = 5804
local_port = 5805
)");
  EXPECT_EQ(config.controller_address, "10.56.87.2");
  EXPECT_EQ(config.controller_port, 5804);
  EXPECT_EQ(config.local_port, 5805);

  // CLI override wins over the deployment configuration.
  EXPECT_EQ(ResolveControllerAddress(config, "10.0.0.5"), "10.0.0.5");
  EXPECT_EQ(ResolveControllerAddress(config, ""), "10.56.87.2");
  EXPECT_EQ(ResolveControllerPort(config, 5900), 5900);
  EXPECT_EQ(ResolveControllerPort(config, 0), 5804);
  EXPECT_EQ(ResolveLocalPort(config, 5901), 5901);
  EXPECT_EQ(ResolveLocalPort(config, 0), 5805);

  ExpectRejected("[robot]\ncontroller_port = 0\n");
  ExpectRejected("[robot]\ncontroller_address = \"\"\n");
}

TEST(ConfigTest, StudioTable) {
  const auto defaults = Parse("");
  EXPECT_EQ(defaults.studio.bridge, "//studio/bridge:studio_bridge");
  EXPECT_EQ(defaults.studio.http_port, 5800);
  EXPECT_EQ(defaults.studio.webroot, "bazel-bin/studio/dist");
  EXPECT_EQ(defaults.studio.topic, "/talos/telemetry");

  const auto no_topic = Parse(R"(
[studio]
bridge = "//custom/studio:bridge"
)");
  EXPECT_EQ(no_topic.studio.bridge, "//custom/studio:bridge");
  EXPECT_EQ(no_topic.studio.topic, "/talos/telemetry");

  const auto cfg = Parse(R"(
[studio]
bridge = "//custom/studio:bridge"
http_port = 8080
webroot = "custom/dist"
topic = "/custom/telemetry"
pose_topic = "/custom/pose"
)");
  EXPECT_EQ(cfg.studio.bridge, "//custom/studio:bridge");
  EXPECT_EQ(cfg.studio.http_port, 8080);
  EXPECT_EQ(cfg.studio.webroot, "custom/dist");
  EXPECT_EQ(cfg.studio.topic, "/custom/telemetry");
  EXPECT_EQ(cfg.studio.pose_topic, "/custom/pose");

  ExpectRejected("[studio]\nhttp_port = 70000\n");
  ExpectRejected("[studio]\nbridge = \"\"\n");
  ExpectRejected("[studio]\ntopic = \"\"\n");
  ExpectRejected("[studio]\npose_topic = \"\"\n");
  ExpectRejected("[studio]\nunknown = 1\n");
}

using Sources = std::vector<std::string>;

TEST(ConfigTest, TargetPolicyDefaultsAndOverrides) {
  const auto defaults = ParseRobotConfigString(kMotor);
  EXPECT_EQ(defaults.targets.teleop, (Sources{"teleop"}));
  EXPECT_EQ(defaults.targets.autonomous, (Sources{"auto"}));
  EXPECT_TRUE(defaults.targets.test.empty());
  EXPECT_EQ(defaults.TargetsFor("test"), defaults.targets);
  EXPECT_EQ(defaults.TargetsFor("not_configured"), defaults.targets);

  const auto cfg = Parse(R"(
[robot.targets]
autonomous = ["auto", "align"]
test = ["test"]
[subsystems.wrist]
[subsystems.wrist.targets]
teleop = ["align", "teleop"]
test = []
[subsystems.roller]
)");
  EXPECT_EQ(cfg.targets.teleop, (Sources{"teleop"}));
  EXPECT_EQ(cfg.targets.autonomous, (Sources{"auto", "align"}));
  const TargetPolicy& wrist = cfg.TargetsFor("wrist");
  EXPECT_EQ(wrist.teleop, (Sources{"align", "teleop"}));
  EXPECT_EQ(wrist.autonomous, (Sources{"auto", "align"}));  // the robot's
  EXPECT_TRUE(wrist.test.empty());
  EXPECT_EQ(wrist.Sources(), (Sources{"align", "teleop", "auto"}));
  EXPECT_EQ(cfg.TargetsFor("roller"), cfg.targets);
}

TEST(ConfigTest, RejectsBadTargetPolicies) {
  for (const char* body : {
           "[robot.targets]\nteleoperated = [\"teleop\"]\n",  // unknown mode
           "[robot.targets]\nteleop = \"teleop\"\n",          // not an array
           "[robot.targets]\nteleop = [1]\n",                 // not a name
           "[robot.targets]\nteleop = [\"Teleop\"]\n",        // not a segment
           "[robot.targets]\nteleop = [\"tgt\"]\n",           // abbreviation
           "[robot.targets]\nteleop = [\"a\", \"a\"]\n",      // twice
           "[robot]\ntargets = 3\n",
           "[subsystems.test.targets]\nauto = [\"auto\"]\n",
           "[subsystems.test]\ntargets = [\"teleop\"]\n",
       }) {
    EXPECT_THROW(
        ParseRobotConfigString(std::string(body) + std::string(kMotor)),
        std::invalid_argument)
        << body;
  }
}

class ManifestTest : public ::testing::Test {
 protected:
  void SetUp() override {
    dir_ = std::filesystem::path(::testing::TempDir()) /
           ::testing::UnitTest::GetInstance()->current_test_info()->name();
    std::filesystem::create_directories(dir_ / "subsystems");
  }
  void Write(const std::string& rel, std::string_view content) {
    std::ofstream(dir_ / rel) << content;
  }
  std::string Robot() const { return (dir_ / "robot.toml").string(); }

  std::filesystem::path dir_;
};

TEST_F(ManifestTest, MergesSubsystemFilesAndKeepsTopLevelTables) {
  Write("robot.toml", R"(
[robot]
period_us = 10000
[studio]
http_port = 9000
topic = "/talos/telemetry"
[[subsystems]]
name = "shooter"
path = "subsystems/shooter.toml"
[[subsystems]]
name = "intake"
path = "subsystems/intake.toml"
)");
  Write("subsystems/shooter.toml", R"(
[subsystem]
name = "shooter"
topic = "/shooter/state"
[motors.flywheel]
can_id = 2
)");
  Write("subsystems/intake.toml", R"(
[subsystem]
name = "intake"
[motors.roller]
can_id = 1
)");
  const auto cfg = ParseRobotConfig(Robot());
  EXPECT_EQ(cfg.hardware.period_us, 10000u);
  EXPECT_EQ(cfg.studio.http_port, 9000);
  EXPECT_EQ(cfg.GetDevices("intake")->motors, (std::vector<uint16_t>{1}));
  EXPECT_EQ(cfg.GetDevices("shooter")->motors, (std::vector<uint16_t>{2}));
  const auto* shooter = cfg.GetSubsystemTable("shooter");
  ASSERT_NE(shooter, nullptr);
  EXPECT_EQ((*shooter)["topic"].value_or(std::string{}), "/shooter/state");
  EXPECT_FALSE(shooter->contains("name"));
}

TEST_F(ManifestTest, EntryNodeOverridesSubsystemFileNode) {
  Write("subsystems/intake.toml", R"(
[subsystem]
name = "intake"
node = "//cpp:intake"
period_us = 20000
[motors.roller]
can_id = 1
)");
  Write("subsystems/planner.toml", R"(
[subsystem]
name = "planner"
gain = 2.0
)");
  Write("robot.toml", R"(
[[subsystems]]
name = "intake"
path = "subsystems/intake.toml"
node = "//py:intake"
[[subsystems]]
name = "planner"
path = "subsystems/planner.toml"
node = "//py:planner"
)");
  const auto cfg = ParseRobotConfig(Robot());
  const auto* intake = cfg.GetSubsystemTable("intake");
  ASSERT_NE(intake, nullptr);
  EXPECT_EQ((*intake)["node"].value_or(std::string{}), "//py:intake");
  EXPECT_EQ((*intake)["period_us"].value_or(0), 20000);
  EXPECT_EQ(cfg.GetDevices("intake")->motors, (std::vector<uint16_t>{1}));
  // The file has no `node`; the entry supplies it.
  const auto* planner = cfg.GetSubsystemTable("planner");
  ASSERT_NE(planner, nullptr);
  EXPECT_EQ((*planner)["node"].value_or(std::string{}), "//py:planner");
  EXPECT_EQ((*planner)["gain"].value_or(0.0), 2.0);

  // Without the override the file's own label stands.
  Write(
      "robot.toml",
      "[[subsystems]]\nname = \"intake\"\npath = \"subsystems/intake.toml\"\n");
  EXPECT_EQ(
      (*ParseRobotConfig(Robot()).GetSubsystemTable("intake"))["node"].value_or(
          std::string{}),
      "//cpp:intake");
}

TEST_F(ManifestTest, RejectsBadEntries) {
  Write("subsystems/a.toml",
        "[subsystem]\nname = \"a\"\n[motors.m]\ncan_id = 1\n");

  Write("robot.toml",
        "[[subsystems]]\nname = \"b\"\npath = \"subsystems/a.toml\"\n");
  EXPECT_THROW(ParseRobotConfig(Robot()),
               std::invalid_argument);  // name mismatch

  Write("robot.toml",
        "[[subsystems]]\nname = \"a\"\npath = \"subsystems/a.toml\"\n"
        "[[subsystems]]\nname = \"a\"\npath = \"subsystems/a.toml\"\n");
  EXPECT_THROW(ParseRobotConfig(Robot()), std::invalid_argument);  // duplicate

  Write("robot.toml", "[[subsystems]]\nname = \"a\"\n");
  EXPECT_THROW(ParseRobotConfig(Robot()), std::invalid_argument);  // no path

  Write("robot.toml",
        "[[subsystems]]\nname = \"a\"\npath = \"missing.toml\"\n");
  EXPECT_THROW(ParseRobotConfig(Robot()), std::invalid_argument);

  // Entries take only name, path and node; anything else is a typo.
  Write("robot.toml",
        "[[subsystems]]\nname = \"a\"\npath = \"subsystems/a.toml\"\n"
        "nod = \"//py:a\"\n");
  EXPECT_THROW(ParseRobotConfig(Robot()), std::invalid_argument);

  Write("robot.toml",
        "[[subsystems]]\nname = \"a\"\npath = \"subsystems/a.toml\"\nnode = "
        "\"\"\n");
  EXPECT_THROW(ParseRobotConfig(Robot()), std::invalid_argument);  // empty node

  Write(
      "robot.toml",
      "[[subsystems]]\nname = \"a\"\npath = \"subsystems/a.toml\"\nnode = 1\n");
  EXPECT_THROW(ParseRobotConfig(Robot()),
               std::invalid_argument);  // not a string
}

TEST_F(ManifestTest, TopLevelIncludeMergesTablesAndSubsystemOverrides) {
  Write("subsystems/intake.toml", R"(
[subsystem]
name = "intake"
node = "//cpp:intake"
[motors.roller]
can_id = 1
)");
  Write("subsystems/elevator.toml", R"(
[subsystem]
name = "elevator"
node = "//cpp:elevator"
[motors.lift]
can_id = 2
)");
  Write("subsystems/extra.toml", R"(
[subsystem]
name = "extra"
[motors.aux]
can_id = 3
)");
  Write("robot.toml", R"(
[robot]
name = "base_robot"
period_us = 10000
controller_port = 5804

[studio]
http_port = 8080
webroot = "base/dist"
topic = "/talos/telemetry"

[[subsystems]]
name = "intake"
path = "subsystems/intake.toml"
node = "//cpp:intake"

[[subsystems]]
name = "elevator"
path = "subsystems/elevator.toml"
node = "//cpp:elevator"
)");
  Write("robot_py.toml", R"(
include = "robot.toml"

[robot]
period_us = 5000

[studio]
http_port = 9090
topic = "/talos/telemetry"

[[subsystems]]
name = "intake"
node = "//py:intake"

[[subsystems]]
name = "extra"
path = "subsystems/extra.toml"
node = "//py:extra"
)");
  const auto cfg = ParseRobotConfig((dir_ / "robot_py.toml").string());
  EXPECT_EQ(cfg.hardware.period_us, 5000u);
  EXPECT_EQ(cfg.controller_port, 5804);
  EXPECT_EQ(cfg.studio.http_port, 9090);
  EXPECT_EQ(cfg.studio.webroot, "base/dist");
  EXPECT_FALSE(cfg.toml_data.contains("include"));

  const auto* intake = cfg.GetSubsystemTable("intake");
  ASSERT_NE(intake, nullptr);
  EXPECT_EQ((*intake)["node"].value_or(std::string{}), "//py:intake");
  const auto* elevator = cfg.GetSubsystemTable("elevator");
  ASSERT_NE(elevator, nullptr);
  EXPECT_EQ((*elevator)["node"].value_or(std::string{}), "//cpp:elevator");
  const auto* extra = cfg.GetSubsystemTable("extra");
  ASSERT_NE(extra, nullptr);
  EXPECT_EQ((*extra)["node"].value_or(std::string{}), "//py:extra");

  // Reject non-string, empty, missing, and nested includes.
  Write("bad_empty.toml", "include = \"\"\n");
  EXPECT_THROW(ParseRobotConfig((dir_ / "bad_empty.toml").string()),
               std::invalid_argument);
  Write("bad_type.toml", "include = 42\n");
  EXPECT_THROW(ParseRobotConfig((dir_ / "bad_type.toml").string()),
               std::invalid_argument);
  Write("bad_nested.toml", "include = \"robot_py.toml\"\n");
  EXPECT_THROW(ParseRobotConfig((dir_ / "bad_nested.toml").string()),
               std::invalid_argument);
  Write("bad_key.toml", "includes = \"robot.toml\"\n");
  EXPECT_THROW(ParseRobotConfig((dir_ / "bad_key.toml").string()),
               std::invalid_argument);
}

TEST_F(ManifestTest, SubsystemDefaultsLayerDeviceAndSubsystemTables) {
  Write("subsystems/mech_defaults.toml", R"(
[subsystem]
period_us = 20000
feed_volts = 6.0
hold_volts = 1.5

[motors.roller]
type = "TalonFX"
supply_limit_a = 40.0
stator_limit_a = 80.0
sensor_to_mechanism_ratio = 3.0
slot0 = { p = 1.0, v = 0.2 }
)");
  // Defaults referenced inside [subsystem].
  Write("subsystems/mech.toml", R"(
[subsystem]
name = "mech"
defaults = "mech_defaults.toml"
feed_volts = 8.0

[motors.roller]
can_id = 21
supply_limit_a = 35.0
slot0 = { p = 2.5, d = 0.05 }
)");
  // Defaults referenced on the [[subsystems]] manifest entry.
  Write("robot.toml", R"(
[[subsystems]]
name = "mech"
path = "subsystems/mech.toml"
)");
  const auto cfg = ParseRobotConfig(Robot());
  ASSERT_EQ(cfg.hardware.motors.size(), 1u);
  const auto& m1 = cfg.hardware.motors[0];
  EXPECT_EQ(m1.can_id, 21);
  EXPECT_DOUBLE_EQ(m1.supply_limit_a, 35.0);
  EXPECT_DOUBLE_EQ(m1.stator_limit_a, 80.0);
  EXPECT_DOUBLE_EQ(m1.sensor_to_mechanism_ratio, 3.0);
  EXPECT_DOUBLE_EQ(m1.slots[0].p, 2.5);
  EXPECT_DOUBLE_EQ(m1.slots[0].v, 0.2);
  EXPECT_DOUBLE_EQ(m1.slots[0].d, 0.05);

  // Defaults referenced on the [[subsystems]] manifest entry.
  Write("subsystems/mech2.toml", R"(
[subsystem]
name = "mech2"

[motors.roller]
can_id = 22
)");
  Write("robot2.toml", R"(
[[subsystems]]
name = "mech2"
path = "subsystems/mech2.toml"
defaults = "subsystems/mech_defaults.toml"
)");
  const auto cfg2 = ParseRobotConfig((dir_ / "robot2.toml").string());
  ASSERT_EQ(cfg2.hardware.motors.size(), 1u);
  const auto& m2 = cfg2.hardware.motors[0];
  EXPECT_EQ(m2.can_id, 22);
  EXPECT_DOUBLE_EQ(m2.supply_limit_a, 40.0);
  EXPECT_DOUBLE_EQ(m2.sensor_to_mechanism_ratio, 3.0);
  EXPECT_DOUBLE_EQ(m2.slots[0].p, 1.0);
  const auto* mech_tbl = cfg.GetSubsystemTable("mech");
  ASSERT_NE(mech_tbl, nullptr);
  EXPECT_FALSE(mech_tbl->contains("name"));
  EXPECT_FALSE(mech_tbl->contains("defaults"));
  EXPECT_EQ((*mech_tbl)["period_us"].value_or(0), 20000);
  EXPECT_DOUBLE_EQ((*mech_tbl)["feed_volts"].value_or(0.0), 8.0);
  EXPECT_DOUBLE_EQ((*mech_tbl)["hold_volts"].value_or(0.0), 1.5);
}

// talOS/configuration/testdata: each case is a config tree plus expect.toml
// (`config`, default "robot.toml"; `accept`; `nodes`; `files`, every file the
// config reads besides itself). //tools:sync_manifest_test runs the same
// cases, so the parser and the Bazel manifest cannot disagree untested.
namespace fs = std::filesystem;

std::vector<std::string> Strings(const toml::table& t, const char* key) {
  std::vector<std::string> out;
  if (const auto* arr = t[key].as_array()) {
    for (auto&& v : *arr) {
      const auto value = v.value<std::string>();
      EXPECT_TRUE(value.has_value());
      if (value.has_value()) {
        out.emplace_back(*value);
      }
    }
  }
  return out;
}

std::vector<std::string> Nodes(const RobotConfig& cfg) {
  std::vector<std::string> out;
  for (auto&& [name, sub] : *cfg.toml_data["subsystems"].as_table()) {
    if (auto node = (*sub.as_table())["node"].value<std::string>()) {
      out.push_back(*node);
    }
  }
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end()), out.end());
  return out;
}

TEST(ConfigTest, SharedResolutionCases) {
  const fs::path testdata = "talOS/configuration/testdata";
  int cases = 0;
  for (const auto& dir : fs::directory_iterator(testdata)) {
    ++cases;
    const std::string name = dir.path().filename().string();
    SCOPED_TRACE(name);
    const toml::table expect =
        toml::parse_file((dir.path() / "expect.toml").string());
    const std::string config = expect["config"].value_or("robot.toml");
    // A private copy, so files can be deleted to prove which are read.
    const fs::path copy = fs::path(::testing::TempDir()) / "cases" / name;
    fs::remove_all(copy);
    fs::create_directories(copy);
    fs::copy(dir.path(), copy, fs::copy_options::recursive);
    const std::string path = (copy / config).string();
    if (!expect["accept"].value_or(false)) {
      EXPECT_THROW(ParseRobotConfig(path), std::invalid_argument);
      continue;
    }
    EXPECT_EQ(Nodes(ParseRobotConfig(path)), Strings(expect, "nodes"));

    const auto files = Strings(expect, "files");
    // Every listed file is read: without it the config does not load.
    for (const auto& file : files) {
      const fs::path saved = copy / (file + ".saved");
      fs::rename(copy / file, saved);
      EXPECT_THROW(ParseRobotConfig(path), std::invalid_argument) << file;
      fs::rename(saved, copy / file);
    }
    // No other file is read: the config loads with all of them gone.
    std::vector<fs::path> others;
    for (const auto& f : fs::recursive_directory_iterator(copy)) {
      const std::string rel = fs::relative(f.path(), copy).generic_string();
      if (f.is_regular_file() && rel != config && rel != "expect.toml" &&
          std::find(files.begin(), files.end(), rel) == files.end()) {
        others.push_back(f.path());
      }
    }
    for (const auto& f : others) fs::remove(f);
    EXPECT_EQ(Nodes(ParseRobotConfig(path)), Strings(expect, "nodes"));
  }
  EXPECT_GT(cases, 0);
}

}  // namespace
}  // namespace talos::config
