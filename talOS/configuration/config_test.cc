#include <gtest/gtest.h>

#include "config_parser.h"

namespace talos::config {
namespace {


TEST(ConfigTest, RejectionRule_DuplicateCanAddressOnSameBus) {
  constexpr std::string_view kToml = R"(
[robot]
period_us = 5000

[subsystems.test.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 5

[subsystems.test.motors.m2]
type = "TalonFX"
bus = "rio"
can_id = 5
)";
  EXPECT_THROW(ParseRobotConfigString(kToml), std::invalid_argument);
}

TEST(ConfigTest, RejectionRule_DuplicateDioChannel) {
  constexpr std::string_view kToml = R"(
[robot]
period_us = 5000

[subsystems.test.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.test.sensors.s1]
type = "DigitalInput"
dio = 3

[subsystems.test.sensors.s2]
type = "DigitalInput"
dio = 3
)";
  EXPECT_THROW(ParseRobotConfigString(kToml), std::invalid_argument);
}

TEST(ConfigTest, RejectionRule_DuplicatePwmChannel) {
  constexpr std::string_view kToml = R"(
[robot]
period_us = 5000

[subsystems.test.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.test.pwm_outputs.p1]
channel = 2

[subsystems.test.pwm_outputs.p2]
channel = 2
)";
  EXPECT_THROW(ParseRobotConfigString(kToml), std::invalid_argument);
}

TEST(ConfigTest, RejectionRule_DuplicateAnalogChannel) {
  constexpr std::string_view kToml = R"(
[robot]
period_us = 5000

[subsystems.test.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.test.analog_inputs.a1]
channel = 0

[subsystems.test.analog_inputs.a2]
channel = 0
)";
  EXPECT_THROW(ParseRobotConfigString(kToml), std::invalid_argument);
}

TEST(ConfigTest, RejectionRule_DuplicateDeviceNameWithinSubsystem) {
  constexpr std::string_view kToml = R"(
[robot]
period_us = 5000

[subsystems.test.motors.device_foo]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.test.sensors.device_foo]
type = "CANcoder"
bus = "rio"
can_id = 2
)";
  EXPECT_THROW(ParseRobotConfigString(kToml), std::invalid_argument);
}

TEST(ConfigTest, RejectionRule_FeedbackSensorNonExistent) {
  constexpr std::string_view kToml = R"(
[robot]
period_us = 5000

[subsystems.test.motors.steer]
type = "TalonFX"
bus = "rio"
can_id = 1
feedback = "RemoteCANcoder"
feedback_sensor = "ghost_encoder"
)";
  EXPECT_THROW(ParseRobotConfigString(kToml), std::invalid_argument);
}

TEST(ConfigTest, RejectionRule_FeedbackSensorDifferentBus) {
  constexpr std::string_view kToml = R"(
[robot]
period_us = 5000

[subsystems.test.sensors.enc]
type = "CANcoder"
bus = "canivore"
can_id = 20

[subsystems.test.motors.steer]
type = "TalonFX"
bus = "rio"
can_id = 1
feedback = "RemoteCANcoder"
feedback_sensor = "enc"
)";
  EXPECT_THROW(ParseRobotConfigString(kToml), std::invalid_argument);
}

TEST(ConfigTest, RejectionRule_FeedbackSensorNotCANcoder) {
  constexpr std::string_view kToml = R"(
[robot]
period_us = 5000

[subsystems.test.sensors.imu]
type = "Pigeon2"
bus = "rio"
can_id = 30

[subsystems.test.motors.steer]
type = "TalonFX"
bus = "rio"
can_id = 1
feedback = "RemoteCANcoder"
feedback_sensor = "imu"
)";
  EXPECT_THROW(ParseRobotConfigString(kToml), std::invalid_argument);
}

TEST(ConfigTest, RejectionRule_FeedbackSensorCrossSubsystemClaim) {
  constexpr std::string_view kToml = R"(
[robot]
period_us = 5000

[subsystems.subA.sensors.encA]
type = "CANcoder"
bus = "rio"
can_id = 20

[subsystems.subB.motors.steerB]
type = "TalonFX"
bus = "rio"
can_id = 1
feedback = "RemoteCANcoder"
feedback_sensor = "encA"
)";
  EXPECT_THROW(ParseRobotConfigString(kToml), std::invalid_argument);
}

TEST(ConfigTest, RejectionRule_SubsystemClaimCrossSubsystemInGeometry) {
  constexpr std::string_view kToml = R"(
[robot]
period_us = 5000

[subsystems.arm.motors.arm_motor]
type = "TalonFX"
bus = "rio"
can_id = 10

[subsystems.drivetrain.motors.drive]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.drivetrain.motors.steer]
type = "TalonFX"
bus = "rio"
can_id = 2

[subsystems.drivetrain.geometry]
wheel_radius_m = 0.0508
modules = [
  { name = "m1", drive = "arm_motor", steer = "steer" }
]
)";
  EXPECT_THROW(ParseRobotConfigString(kToml), std::invalid_argument);
}

TEST(ConfigTest, RejectionRule_GeometryReferencesUnknownDevice) {
  constexpr std::string_view kToml = R"(
[robot]
period_us = 5000

[subsystems.drivetrain.motors.drive]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.drivetrain.geometry]
wheel_radius_m = 0.0508
modules = [
  { name = "m1", drive = "drive", steer = "nonexistent_steer" }
]
)";
  EXPECT_THROW(ParseRobotConfigString(kToml), std::invalid_argument);
}

TEST(ConfigTest, RejectionRule_OutOfRangeSupplyLimit) {
  constexpr std::string_view kTomlNegative = R"(
[robot]
period_us = 5000

[subsystems.test.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1
supply_limit_a = -10.0
)";
  EXPECT_THROW(ParseRobotConfigString(kTomlNegative), std::invalid_argument);

  constexpr std::string_view kTomlZero = R"(
[robot]
period_us = 5000

[subsystems.test.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1
supply_limit_a = 0.0
)";
  EXPECT_THROW(ParseRobotConfigString(kTomlZero), std::invalid_argument);
}

TEST(ConfigTest, RejectionRule_OutOfRangeStatorLimit) {
  constexpr std::string_view kToml = R"(
[robot]
period_us = 5000

[subsystems.test.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1
stator_limit_a = 0.0
)";
  EXPECT_THROW(ParseRobotConfigString(kToml), std::invalid_argument);
}

TEST(ConfigTest, RejectionRule_OutOfRangeMaxVoltage) {
  constexpr std::string_view kTomlOver12 = R"(
[robot]
period_us = 5000

[subsystems.test.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1
max_voltage = 14.0
)";
  EXPECT_THROW(ParseRobotConfigString(kTomlOver12), std::invalid_argument);

  constexpr std::string_view kTomlZero = R"(
[robot]
period_us = 5000

[subsystems.test.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1
max_voltage = 0.0
)";
  EXPECT_THROW(ParseRobotConfigString(kTomlZero), std::invalid_argument);
}

TEST(ConfigTest, RejectionRule_OutOfRangeVelocity) {
  constexpr std::string_view kToml = R"(
[robot]
period_us = 5000

[subsystems.test.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1
max_velocity_rps = 10.0
cruise_velocity_rps = 20.0
)";
  EXPECT_THROW(ParseRobotConfigString(kToml), std::invalid_argument);
}

TEST(ConfigTest, RejectionRule_OutOfRangeEncoderOffset) {
  constexpr std::string_view kTomlOver = R"(
[robot]
period_us = 5000

[subsystems.test.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.test.sensors.s1]
type = "CANcoder"
bus = "rio"
can_id = 20
offset_rot = 0.5
)";
  EXPECT_THROW(ParseRobotConfigString(kTomlOver), std::invalid_argument);

  constexpr std::string_view kTomlUnder = R"(
[robot]
period_us = 5000

[subsystems.test.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.test.sensors.s1]
type = "CANcoder"
bus = "rio"
can_id = 20
offset_rot = -0.6
)";
  EXPECT_THROW(ParseRobotConfigString(kTomlUnder), std::invalid_argument);
}

TEST(ConfigTest, RejectionRule_OutOfRangePeriod) {
  constexpr std::string_view kTomlTooSmall = R"(
[robot]
period_us = 500

[subsystems.test.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1
)";
  EXPECT_THROW(ParseRobotConfigString(kTomlTooSmall), std::invalid_argument);

  constexpr std::string_view kTomlTooLarge = R"(
[robot]
period_us = 30000

[subsystems.test.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1
)";
  EXPECT_THROW(ParseRobotConfigString(kTomlTooLarge), std::invalid_argument);
}

TEST(ConfigTest, RejectionRule_OutOfRangeCommandTimeout) {
  constexpr std::string_view kToml = R"(
[robot]
period_us = 5000
command_timeout_us = 6000

[subsystems.test.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1
)";
  EXPECT_THROW(ParseRobotConfigString(kToml), std::invalid_argument);
}

TEST(ConfigTest, RejectionRule_PigeonNativeFrame) {
  constexpr std::string_view kTomlInverted = R"(
[robot]
period_us = 5000

[subsystems.test.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.test.sensors.imu]
type = "Pigeon2"
bus = "rio"
can_id = 30
inverted = true
)";
  EXPECT_THROW(ParseRobotConfigString(kTomlInverted), std::invalid_argument);

  constexpr std::string_view kTomlOffset = R"(
[robot]
period_us = 5000

[subsystems.test.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.test.sensors.imu]
type = "Pigeon2"
bus = "rio"
can_id = 30
offset_rot = 0.1
)";
  EXPECT_THROW(ParseRobotConfigString(kTomlOffset), std::invalid_argument);
}

TEST(ConfigTest, RejectionRule_NegativeGain) {
  constexpr std::string_view kToml = R"(
[robot]
period_us = 5000

[subsystems.test.motors.m1]
type = "TalonFX"
bus = "rio"
can_id = 1
slot0 = { p = -0.5 }
)";
  EXPECT_THROW(ParseRobotConfigString(kToml), std::invalid_argument);
}

TEST(ConfigTest, RejectionRule_ZeroMotors) {
  constexpr std::string_view kToml = R"(
[robot]
period_us = 5000

[subsystems.test.sensors.s1]
type = "CANcoder"
bus = "rio"
can_id = 10
)";
  EXPECT_THROW(ParseRobotConfigString(kToml), std::invalid_argument);
}

TEST(ConfigTest, DeterministicOrderingIndependentOfTomlTableOrder) {
  // Declare devices in reverse alphabetical order across two subsystems:
  constexpr std::string_view kToml = R"(
[robot]
period_us = 5000

[subsystems.sub_z.motors.z_motor]
type = "TalonFX"
bus = "rio"
can_id = 1

[subsystems.sub_a.motors.b_motor]
type = "TalonFX"
bus = "rio"
can_id = 2

[subsystems.sub_a.motors.a_motor]
type = "TalonFX"
bus = "rio"
can_id = 3

[subsystems.sub_a.sensors.s_sensor]
type = "CANcoder"
bus = "rio"
can_id = 4
)";

  auto config = ParseRobotConfigString(kToml);

  // Sorting by (subsystem, device_name):
  // 1. sub_a / a_motor  -> id 1
  // 2. sub_a / b_motor  -> id 2
  // 3. sub_a / s_sensor -> id 3
  // 4. sub_z / z_motor  -> id 4
  ASSERT_EQ(config.hardware.motors.size(), 3);
  ASSERT_EQ(config.hardware.sensors.size(), 1);

  EXPECT_EQ(config.hardware.motors[0].name, "a_motor");
  EXPECT_EQ(config.hardware.motors[0].id, 1);

  EXPECT_EQ(config.hardware.motors[1].name, "b_motor");
  EXPECT_EQ(config.hardware.motors[1].id, 2);

  EXPECT_EQ(config.hardware.sensors[0].name, "s_sensor");
  EXPECT_EQ(config.hardware.sensors[0].id, 3);

  EXPECT_EQ(config.hardware.motors[2].name, "z_motor");
  EXPECT_EQ(config.hardware.motors[2].id, 4);

  // Per-subsystem ownership
  const auto* devs_a = config.GetDevices("sub_a");
  ASSERT_NE(devs_a, nullptr);
  EXPECT_EQ(devs_a->motors, (std::vector<uint16_t>{1, 2}));
  EXPECT_EQ(devs_a->sensors, (std::vector<uint16_t>{3}));

  const auto* devs_z = config.GetDevices("sub_z");
  ASSERT_NE(devs_z, nullptr);
  EXPECT_EQ(devs_z->motors, (std::vector<uint16_t>{4}));
  EXPECT_TRUE(devs_z->sensors.empty());
}

}  // namespace
}  // namespace talos::config
