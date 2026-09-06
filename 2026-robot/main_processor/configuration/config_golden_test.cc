#include <gtest/gtest.h>

#include <array>
#include <cstdint>

#include "talOS/configuration/config_parser.h"
#include "talOS/hardware/config.h"

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

  // "intake" sorts between "driver_station" and "odometry", so its devices
  // take ids 14-15 and the shooter pair shifts to 16-17.
  hardware::DigitalInputConfig intake_beam;
  intake_beam.id = 14;
  intake_beam.name = "intake_beam";
  intake_beam.dio = 1;
  c.digital_inputs.push_back(intake_beam);

  hardware::MotorConfig roller;
  roller.id = 15;
  roller.name = "roller";
  roller.can_id = 10;
  roller.bus = "rio";
  roller.inverted = false;
  roller.brake = true;
  roller.supply_limit_a = 40.0;
  roller.stator_limit_a = 80.0;
  roller.max_velocity_rps = 100.0;
  roller.feedback = hardware::Feedback::kRotor;
  roller.feedback_sensor_id = 0;
  roller.rotor_to_sensor_ratio = 1.0;
  roller.sensor_to_mechanism_ratio = 1.0;
  c.motors.push_back(roller);

  hardware::DigitalInputConfig beam_break;
  beam_break.id = 16;
  beam_break.name = "beam_break";
  beam_break.dio = 0;
  c.digital_inputs.push_back(beam_break);

  hardware::MotorConfig flywheel;
  flywheel.id = 17;
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
  auto config = ParseRobotConfig("2026-robot/main_processor/configuration/robot.toml");
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

  const auto* intake_devs = config.GetDevices("intake");
  ASSERT_NE(intake_devs, nullptr);
  EXPECT_EQ(intake_devs->subsystem, "intake");
  EXPECT_EQ(intake_devs->motors.size(), 1);
  EXPECT_EQ(intake_devs->digital_inputs.size(), 1);

  // Logical IDs are 1..17 assigned deterministically
  for (size_t i = 0; i < devs->motors.size(); ++i) {
    EXPECT_EQ(config.hardware.motors[i].id, devs->motors[i]);
  }
  EXPECT_EQ(config.hardware.motors[8].id, intake_devs->motors[0]);
  EXPECT_EQ(config.hardware.motors[9].id, shooter_devs->motors[0]);
  for (size_t i = 0; i < devs->sensors.size(); ++i) {
    EXPECT_EQ(config.hardware.sensors[i].id, devs->sensors[i]);
  }
  EXPECT_EQ(config.hardware.digital_inputs[0].id, intake_devs->digital_inputs[0]);
  EXPECT_EQ(config.hardware.digital_inputs[1].id, shooter_devs->digital_inputs[0]);

  // Verify access to subsystem-specific table: [subsystems.drivetrain.geometry]
  const auto* geom = config.GetSubsystemTable("drivetrain", "geometry");
  ASSERT_NE(geom, nullptr);
  EXPECT_DOUBLE_EQ((*geom)["wheel_radius_m"].value_or(0.0), 0.0508);
  auto* modules = (*geom)["modules"].as_array();
  ASSERT_NE(modules, nullptr);
  EXPECT_EQ(modules->size(), 4);
}

}  // namespace
}  // namespace talos::config
