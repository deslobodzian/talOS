#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

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
  EXPECT_EQ(defaults.studio.udp_port, 5801);
  EXPECT_EQ(defaults.studio.webroot, "bazel-bin/studio/dist");

  const auto cfg = Parse(R"(
[studio]
bridge = "//custom/studio:bridge"
http_port = 8080
udp_port = 8081
webroot = "custom/dist"
)");
  EXPECT_EQ(cfg.studio.bridge, "//custom/studio:bridge");
  EXPECT_EQ(cfg.studio.http_port, 8080);
  EXPECT_EQ(cfg.studio.udp_port, 8081);
  EXPECT_EQ(cfg.studio.webroot, "custom/dist");

  ExpectRejected("[studio]\nhttp_port = 70000\n");
  ExpectRejected("[studio]\nbridge = \"\"\n");
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

}  // namespace
}  // namespace talos::config
