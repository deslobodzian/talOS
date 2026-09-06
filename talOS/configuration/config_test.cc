#include <gtest/gtest.h>

#include "common/hardware/config.h"
#include "config_parser.h"

namespace talos::config {
namespace {

struct ModuleConfig {
  const char* name;
  uint16_t drive_id, steer_id, encoder_id;
  double x_m, y_m;
  double wheel_radius_m;
  int drive_can_id, steer_can_id, encoder_can_id;
  const char* bus{"rio"};
  double encoder_offset_rot{0};
  bool drive_inverted{false}, steer_inverted{false}, encoder_inverted{false};
};

inline constexpr std::array<ModuleConfig, 4> kReferenceSwerveModules{{
    {"back_left", 1, 3, 2, -0.30, 0.30, 0.0508, 5, 6, 22},
    {"back_right", 4, 6, 5, -0.30, -0.30, 0.0508, 7, 8, 23},
    {"front_left", 8, 10, 9, 0.30, 0.30, 0.0508, 1, 2, 20},
    {"front_right", 11, 13, 12, 0.30, -0.30, 0.0508, 3, 4, 21},
}};

inline hardware::Config ReferenceSwerveConfig(bool simulation = false) {
  hardware::Config c;
  c.commissioned = simulation;
  for (std::size_t i = 0; i < kReferenceSwerveModules.size(); ++i) {
    const auto& module = kReferenceSwerveModules[i];
    hardware::SensorConfig encoder;
    encoder.id = module.encoder_id;
    encoder.name = std::string{module.name} + "_encoder";
    encoder.can_id = module.encoder_can_id;
    encoder.bus = module.bus;
    encoder.offset_rot = module.encoder_offset_rot;
    encoder.inverted = module.encoder_inverted;
    c.sensors.push_back(encoder);
    hardware::MotorConfig drive;
    drive.id = module.drive_id;
    drive.name = std::string{module.name} + "_drive";
    drive.can_id = module.drive_can_id;
    drive.bus = module.bus;
    drive.inverted = module.drive_inverted;
    drive.sensor_to_mechanism_ratio = 6.75;
    drive.max_velocity_rps = 20;
    drive.slots[0].p = 0.1;
    drive.slots[0].v = 0.12;
    c.motors.push_back(drive);
    hardware::MotorConfig steer;
    steer.id = module.steer_id;
    steer.name = std::string{module.name} + "_steer";
    steer.can_id = module.steer_can_id;
    steer.bus = module.bus;
    steer.inverted = module.steer_inverted;
    steer.feedback = hardware::Feedback::kRemoteCANcoder;
    steer.feedback_sensor_id = encoder.id;
    steer.rotor_to_sensor_ratio = 12.8;
    steer.continuous_wrap = true;
    steer.supply_limit_a = 20;
    steer.stator_limit_a = 40;
    steer.cruise_velocity_rps = 5;
    steer.acceleration_rps2 = 20;
    steer.slots[0].p = 24.0;
    steer.slots[0].d = 0.5;
    c.motors.push_back(steer);
  }
  c.sensors.push_back({7, "drivetrain_imu", hardware::SensorKind::kPigeon2, 30, "rio"});
  std::sort(c.sensors.begin(), c.sensors.end(),
            [](const auto& a, const auto& b) { return a.name < b.name; });
  std::sort(c.motors.begin(), c.motors.end(),
            [](const auto& a, const auto& b) { return a.name < b.name; });

  hardware::DigitalInputConfig beam_break;
  beam_break.id = 14;
  beam_break.name = "beam_break";
  beam_break.dio = 0;
  c.digital_inputs.push_back(beam_break);

  hardware::MotorConfig flywheel;
  flywheel.id = 15;
  flywheel.name = "flywheel";
  flywheel.can_id = 9;
  flywheel.bus = "rio";
  flywheel.inverted = false;
  flywheel.brake = false;
  flywheel.supply_limit_a = 40.0;
  flywheel.stator_limit_a = 80.0;
  flywheel.max_voltage = 12.0;
  flywheel.feedback = hardware::Feedback::kRotor;
  flywheel.feedback_sensor_id = 0;
  flywheel.rotor_to_sensor_ratio = 1.0;
  flywheel.sensor_to_mechanism_ratio = 1.0;
  flywheel.continuous_wrap = false;
  flywheel.soft_limits = false;
  flywheel.reverse_limit_rot = -1.0;
  flywheel.forward_limit_rot = 1.0;
  flywheel.max_velocity_rps = 100.0;
  flywheel.cruise_velocity_rps = 80.0;
  flywheel.acceleration_rps2 = 160.0;
  flywheel.jerk_rps3 = 0.0;
  flywheel.slots[0].p = 0.1;
  flywheel.slots[0].v = 0.12;
  c.motors.push_back(flywheel);

  return c;
}

TEST(ConfigTest, GoldenTestParsesSwerveRobotTomlEqualToSwerveConfig) {
  auto config = ParseRobotConfig("talOS/configuration/robot.toml");
  auto expected = ReferenceSwerveConfig(false);

  // Validate passes
  EXPECT_NO_THROW(hardware::Validate(config.hardware));

  // Assert hardware::Config equality
  EXPECT_EQ(config.hardware, expected);
  EXPECT_EQ(hardware::ConfigurationId(config.hardware),
            hardware::ConfigurationId(expected));

  // Verify subsystem device ownership
  const auto* devs = config.GetDevices("drivetrain");
  ASSERT_NE(devs, nullptr);
  EXPECT_EQ(devs->subsystem, "drivetrain");
  EXPECT_EQ(devs->motors.size(), 8);
  EXPECT_EQ(devs->sensors.size(), 5);

  const auto* shooter_devs = config.GetDevices("shooter");
  ASSERT_NE(shooter_devs, nullptr);
  EXPECT_EQ(shooter_devs->subsystem, "shooter");
  EXPECT_EQ(shooter_devs->motors.size(), 1);
  EXPECT_EQ(shooter_devs->digital_inputs.size(), 1);

  // Logical IDs are 1..15 assigned deterministically
  for (size_t i = 0; i < devs->motors.size(); ++i) {
    EXPECT_EQ(config.hardware.motors[i].id, devs->motors[i]);
  }
  EXPECT_EQ(config.hardware.motors[8].id, shooter_devs->motors[0]);
  for (size_t i = 0; i < devs->sensors.size(); ++i) {
    EXPECT_EQ(config.hardware.sensors[i].id, devs->sensors[i]);
  }
  EXPECT_EQ(config.hardware.digital_inputs[0].id, shooter_devs->digital_inputs[0]);

  // Verify access to subsystem-specific table: [subsystems.drivetrain.geometry]
  const auto* geom = config.GetSubsystemTable("drivetrain", "geometry");
  ASSERT_NE(geom, nullptr);
  EXPECT_DOUBLE_EQ((*geom)["wheel_radius_m"].value_or(0.0), 0.0508);
  auto* modules = (*geom)["modules"].as_array();
  ASSERT_NE(modules, nullptr);
  EXPECT_EQ(modules->size(), 4);
}

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
