#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>

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
  c.sensors.push_back(
      {7, "drivetrain_imu", hardware::SensorKind::kPigeon2, 30, "rio"});
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

const hardware::MotorConfig* FindMotor(const hardware::Config& hw,
                                       std::string_view name) {
  auto it = std::find_if(hw.motors.begin(), hw.motors.end(),
                         [&](const auto& m) { return m.name == name; });
  return it != hw.motors.end() ? &*it : nullptr;
}

const hardware::SensorConfig* FindSensor(const hardware::Config& hw,
                                         std::string_view name) {
  auto it = std::find_if(hw.sensors.begin(), hw.sensors.end(),
                         [&](const auto& s) { return s.name == name; });
  return it != hw.sensors.end() ? &*it : nullptr;
}

const hardware::DigitalInputConfig* FindDigitalInput(const hardware::Config& hw,
                                                     std::string_view name) {
  auto it = std::find_if(hw.digital_inputs.begin(), hw.digital_inputs.end(),
                         [&](const auto& d) { return d.name == name; });
  return it != hw.digital_inputs.end() ? &*it : nullptr;
}

void VerifyReferenceRobotConfig(const RobotConfig& config,
                                const hardware::Config& expected) {
  const auto& actual = config.hardware;

  // Validate passes
  EXPECT_NO_THROW(hardware::Validate(actual));

  // Global robot settings and Studio / deployment addressing
  EXPECT_EQ(actual.period_us, expected.period_us);
  EXPECT_EQ(actual.command_timeout_us, expected.command_timeout_us);
  EXPECT_DOUBLE_EQ(actual.status_hz, expected.status_hz);
  EXPECT_EQ(actual.commissioned, expected.commissioned);
  EXPECT_EQ(config.controller_address, "127.0.0.1");
  EXPECT_EQ(config.controller_port, 5802);
  EXPECT_EQ(config.local_port, 5803);
  EXPECT_EQ(config.studio.bridge, "//studio/bridge:studio_bridge");
  EXPECT_EQ(config.studio.http_port, 5800);
  EXPECT_EQ(config.studio.webroot, "bazel-bin/studio/dist");

  // Verify every reference sensor by name
  for (const auto& exp_sensor : expected.sensors) {
    const auto* act_sensor = FindSensor(actual, exp_sensor.name);
    ASSERT_NE(act_sensor, nullptr) << exp_sensor.name;
    EXPECT_GT(act_sensor->id, 0u) << exp_sensor.name;
    EXPECT_EQ(act_sensor->kind, exp_sensor.kind) << exp_sensor.name;
    EXPECT_EQ(act_sensor->can_id, exp_sensor.can_id) << exp_sensor.name;
    EXPECT_EQ(act_sensor->bus, exp_sensor.bus) << exp_sensor.name;
    EXPECT_EQ(act_sensor->inverted, exp_sensor.inverted) << exp_sensor.name;
    EXPECT_DOUBLE_EQ(act_sensor->offset_rot, exp_sensor.offset_rot)
        << exp_sensor.name;
    hardware::SensorConfig normalized = *act_sensor;
    normalized.id = exp_sensor.id;
    EXPECT_EQ(normalized, exp_sensor) << exp_sensor.name;
  }

  // Verify every reference motor by name, including feedback_sensor_id
  // resolution
  for (const auto& exp_motor : expected.motors) {
    const auto* act_motor = FindMotor(actual, exp_motor.name);
    ASSERT_NE(act_motor, nullptr) << exp_motor.name;
    EXPECT_GT(act_motor->id, 0u) << exp_motor.name;
    EXPECT_EQ(act_motor->can_id, exp_motor.can_id) << exp_motor.name;
    EXPECT_EQ(act_motor->bus, exp_motor.bus) << exp_motor.name;
    EXPECT_EQ(act_motor->inverted, exp_motor.inverted) << exp_motor.name;
    EXPECT_EQ(act_motor->brake, exp_motor.brake) << exp_motor.name;
    EXPECT_DOUBLE_EQ(act_motor->supply_limit_a, exp_motor.supply_limit_a)
        << exp_motor.name;
    EXPECT_DOUBLE_EQ(act_motor->stator_limit_a, exp_motor.stator_limit_a)
        << exp_motor.name;
    EXPECT_DOUBLE_EQ(act_motor->max_voltage, exp_motor.max_voltage)
        << exp_motor.name;
    EXPECT_EQ(act_motor->feedback, exp_motor.feedback) << exp_motor.name;
    EXPECT_DOUBLE_EQ(act_motor->rotor_to_sensor_ratio,
                     exp_motor.rotor_to_sensor_ratio)
        << exp_motor.name;
    EXPECT_DOUBLE_EQ(act_motor->sensor_to_mechanism_ratio,
                     exp_motor.sensor_to_mechanism_ratio)
        << exp_motor.name;
    EXPECT_EQ(act_motor->continuous_wrap, exp_motor.continuous_wrap)
        << exp_motor.name;
    EXPECT_EQ(act_motor->soft_limits, exp_motor.soft_limits) << exp_motor.name;
    EXPECT_DOUBLE_EQ(act_motor->reverse_limit_rot, exp_motor.reverse_limit_rot)
        << exp_motor.name;
    EXPECT_DOUBLE_EQ(act_motor->forward_limit_rot, exp_motor.forward_limit_rot)
        << exp_motor.name;
    EXPECT_DOUBLE_EQ(act_motor->max_velocity_rps, exp_motor.max_velocity_rps)
        << exp_motor.name;
    EXPECT_DOUBLE_EQ(act_motor->cruise_velocity_rps,
                     exp_motor.cruise_velocity_rps)
        << exp_motor.name;
    EXPECT_DOUBLE_EQ(act_motor->acceleration_rps2, exp_motor.acceleration_rps2)
        << exp_motor.name;
    EXPECT_DOUBLE_EQ(act_motor->jerk_rps3, exp_motor.jerk_rps3)
        << exp_motor.name;
    EXPECT_EQ(act_motor->slots, exp_motor.slots) << exp_motor.name;

    if (exp_motor.feedback_sensor_id == 0) {
      EXPECT_EQ(act_motor->feedback_sensor_id, 0u) << exp_motor.name;
    } else {
      auto exp_sensor_it = std::find_if(
          expected.sensors.begin(), expected.sensors.end(),
          [&](const auto& s) { return s.id == exp_motor.feedback_sensor_id; });
      ASSERT_NE(exp_sensor_it, expected.sensors.end()) << exp_motor.name;
      const auto* act_sensor = FindSensor(actual, exp_sensor_it->name);
      ASSERT_NE(act_sensor, nullptr) << exp_sensor_it->name;
      EXPECT_EQ(act_motor->feedback_sensor_id, act_sensor->id)
          << exp_motor.name;
    }

    hardware::MotorConfig normalized = *act_motor;
    normalized.id = exp_motor.id;
    normalized.feedback_sensor_id = exp_motor.feedback_sensor_id;
    EXPECT_EQ(normalized, exp_motor) << exp_motor.name;
  }

  // Verify every reference digital input by name
  for (const auto& exp_di : expected.digital_inputs) {
    const auto* act_di = FindDigitalInput(actual, exp_di.name);
    ASSERT_NE(act_di, nullptr) << exp_di.name;
    EXPECT_GT(act_di->id, 0u) << exp_di.name;
    EXPECT_EQ(act_di->dio, exp_di.dio) << exp_di.name;
    hardware::DigitalInputConfig normalized = *act_di;
    normalized.id = exp_di.id;
    EXPECT_EQ(normalized, exp_di) << exp_di.name;
  }

  // Verify subsystem device ownership by resolving names to actual logical IDs
  const auto* devs = config.GetDevices("drivetrain");
  ASSERT_NE(devs, nullptr);
  EXPECT_EQ(devs->subsystem, "drivetrain");
  EXPECT_EQ(devs->motors.size(), 8u);
  EXPECT_EQ(devs->sensors.size(), 5u);
  for (const auto& module : kReferenceSwerveModules) {
    const auto* drive = FindMotor(actual, std::string{module.name} + "_drive");
    const auto* steer = FindMotor(actual, std::string{module.name} + "_steer");
    const auto* enc = FindSensor(actual, std::string{module.name} + "_encoder");
    ASSERT_NE(drive, nullptr);
    ASSERT_NE(steer, nullptr);
    ASSERT_NE(enc, nullptr);
    EXPECT_NE(std::find(devs->motors.begin(), devs->motors.end(), drive->id),
              devs->motors.end());
    EXPECT_NE(std::find(devs->motors.begin(), devs->motors.end(), steer->id),
              devs->motors.end());
    EXPECT_NE(std::find(devs->sensors.begin(), devs->sensors.end(), enc->id),
              devs->sensors.end());
  }
  const auto* imu = FindSensor(actual, "drivetrain_imu");
  ASSERT_NE(imu, nullptr);
  EXPECT_NE(std::find(devs->sensors.begin(), devs->sensors.end(), imu->id),
            devs->sensors.end());

  const auto* shooter_devs = config.GetDevices("shooter");
  ASSERT_NE(shooter_devs, nullptr);
  EXPECT_EQ(shooter_devs->subsystem, "shooter");
  ASSERT_EQ(shooter_devs->motors.size(), 1u);
  ASSERT_EQ(shooter_devs->digital_inputs.size(), 1u);
  EXPECT_EQ(shooter_devs->motors[0], FindMotor(actual, "flywheel")->id);
  EXPECT_EQ(shooter_devs->digital_inputs[0],
            FindDigitalInput(actual, "beam_break")->id);

  const auto* intake_devs = config.GetDevices("intake");
  ASSERT_NE(intake_devs, nullptr);
  EXPECT_EQ(intake_devs->subsystem, "intake");
  ASSERT_EQ(intake_devs->motors.size(), 1u);
  ASSERT_EQ(intake_devs->digital_inputs.size(), 1u);
  EXPECT_EQ(intake_devs->motors[0], FindMotor(actual, "roller")->id);
  EXPECT_EQ(intake_devs->digital_inputs[0],
            FindDigitalInput(actual, "intake_beam")->id);

  // Verify access to subsystem-specific table: [subsystems.drivetrain.geometry]
  const auto* geom = config.GetSubsystemTable("drivetrain", "geometry");
  ASSERT_NE(geom, nullptr);
  EXPECT_DOUBLE_EQ((*geom)["wheel_radius_m"].value_or(0.0), 0.0508);
  auto* modules = (*geom)["modules"].as_array();
  ASSERT_NE(modules, nullptr);
  EXPECT_EQ(modules->size(), 4u);
}

TEST(ConfigTest, GoldenTestParsesSwerveRobotTomlEqualToSwerveConfig) {
  const auto config =
      ParseRobotConfig("2026-robot/main_processor/configuration/robot.toml");
  const auto expected = ReferenceSwerveConfig(false);
  VerifyReferenceRobotConfig(config, expected);
}

TEST(ConfigTest, ExtraSubsystemShiftingLogicalIdsStillMatchesReferenceDevices) {
  auto base =
      ParseRobotConfig("2026-robot/main_processor/configuration/robot.toml");
  toml::table tbl = base.toml_data;
  auto* subs = tbl["subsystems"].as_table();
  ASSERT_NE(subs, nullptr);

  // "climber" sorts before "drivetrain", shifting every reference logical ID.
  toml::table climber_motor;
  climber_motor.insert_or_assign("type", "TalonFX");
  climber_motor.insert_or_assign("bus", "rio");
  climber_motor.insert_or_assign("can_id", 15);
  toml::table climber_motors;
  climber_motors.insert_or_assign("climber_motor", std::move(climber_motor));
  toml::table climber;
  climber.insert_or_assign("period_us", 20000);
  climber.insert_or_assign("motors", std::move(climber_motors));
  subs->insert_or_assign("climber", std::move(climber));

  const auto with_climber = ParseRobotConfigTable(std::move(tbl));
  const auto* climber_dev = FindMotor(with_climber.hardware, "climber_motor");
  ASSERT_NE(climber_dev, nullptr);
  EXPECT_EQ(climber_dev->id, 1u);

  const auto expected = ReferenceSwerveConfig(false);
  VerifyReferenceRobotConfig(with_climber, expected);
}

}  // namespace
}  // namespace talos::config
