#include "2026-robot/main_processor/drivetrain/node.h"

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdio>
#include <limits>
#include <string>
#include <string_view>
#include <utility>

#include "2026-robot/main_processor/drivetrain/geometry.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/replay_event_loop.h"
#include "talOS/events/simulated_event_loop.h"
#include "talOS/hardware/sim_backend.h"

namespace talos::drive {
namespace {
using namespace std::chrono_literals;

config::RobotConfig TestRobotConfig() {
  auto cfg = config::ParseRobotConfig(
      "2026-robot/main_processor/configuration/robot.toml");
  cfg.hardware.commissioned = true;
  return cfg;
}

SwerveGeometry TestGeometry() { return BuildSwerveGeometry(TestRobotConfig()); }

hardware::State InitialState() {
  hardware::SimBackend backend;
  auto robot_cfg = TestRobotConfig();
  hardware::Gateway gateway{robot_cfg.hardware, backend, 42};
  gateway.Tick(1000, true);
  return gateway.snapshot();
}
Packet StatePacket(const hardware::State& state) {
  Packet p{};
  p.size = hardware::Encode(state, p.data);
  return p;
}
hardware::Command LastCommand(event::SimulationEnvironment& env) {
  auto& channel = env.channel(kDriveRequestTopic, sizeof(Packet));
  EXPECT_GT(channel.next_sequence(), 0);
  Packet p{};
  if (!channel.next_sequence()) return {};
  channel.copy_to(channel.next_sequence() - 1,
                  {reinterpret_cast<std::byte*>(&p), sizeof(p)});
  hardware::Command command;
  EXPECT_TRUE(hardware::Decode(p.bytes(), command));
  return command;
}
TEST(Swerve, TranslationAndReverseUseShortestSteeringPath) {
  const auto robot_cfg = TestRobotConfig();
  const auto& config = robot_cfg.hardware;
  const auto state = InitialState();
  const auto geometry = TestGeometry();
  const double rps =
      1.0 / (2 * std::numbers::pi * geometry.modules[0].wheel_radius_m);
  for (double vx : {1.0, -1.0}) {
    const auto c = SwerveCommand(config, geometry, state, vx, 0, 0);
    ASSERT_EQ(c.count, static_cast<uint16_t>(config.motors.size()));
    for (int i = 0; i < 4; ++i) {
      EXPECT_EQ(c.motors[2 * i].mode, hardware::Mode::kVelocity);
      EXPECT_NEAR(c.motors[2 * i].demand, vx * rps, 1e-12);
      EXPECT_NEAR(c.motors[2 * i + 1].demand, 0, 1e-12);
    }
  }
  const auto c = SwerveCommand(config, geometry, state, 0, 1, 0);
  for (int i = 0; i < 4; ++i)
    EXPECT_NEAR(c.motors[2 * i + 1].demand, 0.25, 1e-12);
}
TEST(Swerve, RotationDesaturationAndInvalidInput) {
  const auto robot_cfg = TestRobotConfig();
  const auto& config = robot_cfg.hardware;
  const auto state = InitialState();
  const auto geometry = TestGeometry();
  auto c = SwerveCommand(config, geometry, state, 0, 0, 1);
  EXPECT_LT(c.motors[0].demand, 0);
  EXPECT_GT(c.motors[2].demand, 0);
  EXPECT_NEAR(c.motors[1].demand, 0.125, 1e-12);
  EXPECT_NEAR(c.motors[3].demand, -0.125, 1e-12);
  c = SwerveCommand(config, geometry, state, 100, 0, 0);
  for (int i = 0; i < 4; ++i) EXPECT_NEAR(c.motors[2 * i].demand, 20, 1e-12);
  c = SwerveCommand(config, geometry, state,
                    std::numeric_limits<double>::quiet_NaN(), 0, 0);
  for (int i = 0; i < 8; ++i)
    EXPECT_EQ(c.motors[i].mode, hardware::Mode::kNeutral);
}
// Writes robot.toml with an extra subsystem spliced in. Logical ids are handed
// out by sorting on (subsystem name, device name) across the whole robot, so a
// subsystem sorting before "drivetrain" renumbers every swerve device.
std::string WriteConfigWithExtraSubsystem(const char* subsystem_name) {
  std::string toml;
  {
    FILE* in =
        std::fopen("2026-robot/main_processor/configuration/robot.toml", "rb");
    EXPECT_NE(in, nullptr);
    char buffer[4096];
    std::size_t n;
    while ((n = std::fread(buffer, 1, sizeof(buffer), in)) > 0)
      toml.append(buffer, n);
    std::fclose(in);
  }
  toml += std::string{"\n[subsystems."} + subsystem_name +
          "]\nnode = \"//example/x:node\"\nperiod_us = 5000\n\n" +
          "[subsystems." + subsystem_name +
          ".motors.joint]\ntype = \"TalonFX\"\nbus = \"rio\"\ncan_id = 40\n";

  std::string path = std::string{"/tmp/talos_geometry_"} + subsystem_name +
                     "_" + std::to_string(getpid()) + ".toml";
  FILE* out = std::fopen(path.c_str(), "wb");
  EXPECT_NE(out, nullptr);
  std::fwrite(toml.data(), 1, toml.size(), out);
  std::fclose(out);
  return path;
}

uint16_t MotorIdNamed(const hardware::Config& config, std::string_view name) {
  for (const auto& motor : config.motors)
    if (motor.name == name) return motor.id;
  ADD_FAILURE() << "no motor named " << name;
  return 0;
}

TEST(SwerveGeometry, FollowsDeviceRenumberingCausedByAnUnrelatedSubsystem) {
  const auto baseline = TestRobotConfig();
  const auto baseline_geometry = BuildSwerveGeometry(baseline);

  // "arm" sorts before "drivetrain", so every drivetrain id shifts by one.
  const auto path = WriteConfigWithExtraSubsystem("arm");
  const auto shifted = config::ParseRobotConfig(path);
  std::remove(path.c_str());
  const auto shifted_geometry = BuildSwerveGeometry(shifted);

  ASSERT_NE(MotorIdNamed(baseline.hardware, "front_left_drive"),
            MotorIdNamed(shifted.hardware, "front_left_drive"))
      << "test is vacuous unless the extra subsystem actually renumbers ids";
  EXPECT_NE(baseline_geometry, shifted_geometry);

  // Whatever the numbering, every module still points at the device the
  // configuration named for it.
  const std::pair<const SwerveGeometry&, const hardware::Config&> cases[]{
      {baseline_geometry, baseline.hardware},
      {shifted_geometry, shifted.hardware}};
  for (const auto& [geometry, config] : cases) {
    for (const auto& module : geometry.modules) {
      const std::string name{module.name.data()};
      EXPECT_EQ(module.drive_id, MotorIdNamed(config, name + "_drive"));
      EXPECT_EQ(module.steer_id, MotorIdNamed(config, name + "_steer"));
    }
  }
}

TEST(SwerveGeometry, RejectsUnknownAndCrossedDeviceReferences) {
  const auto robot_cfg = TestRobotConfig();
  const auto* geometry = robot_cfg.GetSubsystemTable("drivetrain", "geometry");
  ASSERT_NE(geometry, nullptr);

  auto mutated = *geometry;
  auto* modules = mutated["modules"].as_array();
  ASSERT_NE(modules, nullptr);
  auto* first = modules->get(0)->as_table();
  ASSERT_NE(first, nullptr);

  first->insert_or_assign("drive", "no_such_motor");
  EXPECT_THROW(BuildSwerveGeometry(robot_cfg.hardware, mutated),
               std::invalid_argument);

  // Point a module at another module's encoder: the steer motor's configured
  // feedback sensor no longer matches, which must not be silently accepted.
  mutated = *geometry;
  modules = mutated["modules"].as_array();
  first = modules->get(0)->as_table();
  first->insert_or_assign("encoder", "front_right_encoder");
  EXPECT_THROW(BuildSwerveGeometry(robot_cfg.hardware, mutated),
               std::invalid_argument);
}

TEST(SwerveGeometry, RequiresExactlyFourModulesAndAPositiveWheelRadius) {
  const auto robot_cfg = TestRobotConfig();
  const auto* geometry = robot_cfg.GetSubsystemTable("drivetrain", "geometry");
  ASSERT_NE(geometry, nullptr);

  auto short_table = *geometry;
  short_table["modules"].as_array()->pop_back();
  EXPECT_THROW(BuildSwerveGeometry(robot_cfg.hardware, short_table),
               std::invalid_argument);

  auto bad_radius = *geometry;
  bad_radius.insert_or_assign("wheel_radius_m", 0.0);
  EXPECT_THROW(BuildSwerveGeometry(robot_cfg.hardware, bad_radius),
               std::invalid_argument);
}

TEST(Drivetrain, StaleTargetAndDisabledGatewayNeutralizeAllMotors) {
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  const auto robot_cfg = TestRobotConfig();
  const auto* devs = robot_cfg.GetDevices("drivetrain");
  DrivetrainNode node{loop, robot_cfg.hardware, TestGeometry(),
                      devs ? *devs : hardware::Devices{}};
  node.Start(event::MonotonicTime::from_nanos(1'000'000));
  auto state = InitialState();
  loop.inject(kStateTopic, StatePacket(state));
  loop.inject(kTargetTopic, ChassisTarget{1, 0, 0, 0, true});
  loop.run_for(10ms);
  EXPECT_EQ(LastCommand(env).motors[0].mode, hardware::Mode::kVelocity);
  loop.run_for(110ms);
  auto c = LastCommand(env);
  for (int i = 0; i < 8; ++i)
    EXPECT_EQ(c.motors[i].mode, hardware::Mode::kNeutral);
  state.sample_time_us += 1000;
  state.flags &= ~hardware::kEnabled;
  loop.inject(kStateTopic, StatePacket(state));
  loop.inject(kTargetTopic, ChassisTarget{1, 0, 0, 120'000'000, true});
  loop.run_for(10ms);
  EXPECT_EQ(LastCommand(env).motors[0].mode, hardware::Mode::kNeutral);
}
TEST(Drivetrain, HeadingComesFromTheImuNotWhicheverSensorSortsFirst) {
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  const auto robot_cfg = TestRobotConfig();
  const auto* devs = robot_cfg.GetDevices("drivetrain");
  DrivetrainNode node{loop, robot_cfg.hardware, TestGeometry(),
                      devs ? *devs : hardware::Devices{}};
  node.Start(event::MonotonicTime::from_nanos(1'000'000));

  uint16_t imu_id = 0, encoder_id = 0;
  for (const auto& sensor : robot_cfg.hardware.sensors) {
    if (sensor.kind == hardware::SensorKind::kPigeon2)
      imu_id = sensor.id;
    else if (encoder_id == 0)
      encoder_id = sensor.id;
  }
  ASSERT_NE(imu_id, 0);
  ASSERT_NE(encoder_id, 0);
  // The steer encoder sorts ahead of the IMU by device name, so a node reading
  // "the first valid sensor" publishes a module azimuth as the robot heading.
  ASSERT_LT(encoder_id, imu_id);

  auto state = InitialState();
  for (std::size_t i = 0; i < state.sensor_count; ++i) {
    if (state.sensors[i].id == encoder_id)
      state.sensors[i] = {encoder_id, true, 0.375, 2.0, 0, 0};
    if (state.sensors[i].id == imu_id)
      state.sensors[i] = {imu_id, true, 0.125, 0.5, 0, 0};
  }
  loop.inject(kStateTopic, StatePacket(state));
  loop.inject(kTargetTopic, ChassisTarget{0, 0, 0, 0, true});
  loop.run_for(20ms);

  auto& channel = env.channel(kDrivetrainStateTopic, sizeof(DrivetrainState));
  ASSERT_GT(channel.next_sequence(), 0u);
  DrivetrainState published{};
  channel.copy_to(
      channel.next_sequence() - 1,
      {reinterpret_cast<std::byte*>(&published), sizeof(published)});
  EXPECT_NEAR(published.yaw_rot(), 0.125, 1e-12);
  EXPECT_NEAR(published.yaw_rate_rps(), 0.5, 1e-12);
}

// Builds module states for a pure chassis rotation of `omega` rad/s: each
// module's wheel points tangentially and turns at omega * |r|.
hardware::State RotatingState(const SwerveGeometry& geometry, double omega) {
  auto state = InitialState();
  constexpr double kTau = 2 * std::numbers::pi;
  for (const auto& module : geometry.modules) {
    const double vx = -omega * module.y_m, vy = omega * module.x_m;
    const double speed_rps =
        std::hypot(vx, vy) / (kTau * module.wheel_radius_m);
    const double angle_rot = std::atan2(vy, vx) / kTau;
    for (std::size_t i = 0; i < state.motor_count; ++i) {
      if (state.motors[i].id == module.drive_id) {
        state.motors[i].velocity_rps = speed_rps;
      } else if (state.motors[i].id == module.steer_id) {
        state.motors[i].position_rot = angle_rot;
      }
    }
  }
  return state;
}

double LastPublishedYawRot(event::SimulationEnvironment& env) {
  auto& channel = env.channel(kDrivetrainStateTopic, sizeof(DrivetrainState));
  EXPECT_GT(channel.next_sequence(), 0u);
  DrivetrainState published{};
  if (channel.next_sequence() == 0) return 0;
  channel.copy_to(
      channel.next_sequence() - 1,
      {reinterpret_cast<std::byte*>(&published), sizeof(published)});
  return published.yaw_rot();
}

TEST(Drivetrain, SimulatedHeadingIntegratesTheRotationTheModulesProduce) {
  const auto robot_cfg = TestRobotConfig();
  const auto* devs = robot_cfg.GetDevices("drivetrain");
  const auto geometry = TestGeometry();
  constexpr double kOmega = 1.0;  // rad/s
  constexpr double kTau = 2 * std::numbers::pi;

  // Simulation backends leave the gyro valid and permanently zero, so the
  // sensor path cannot tell "facing downfield, not turning" from "no gyro".
  {
    event::SimulationEnvironment env;
    event::SimulatedEventLoop<> loop{env};
    DrivetrainNode node{loop, robot_cfg.hardware, geometry,
                        devs ? *devs : hardware::Devices{},
                        /*simulate_heading=*/false};
    node.Start(event::MonotonicTime::from_nanos(1'000'000));
    loop.inject(kStateTopic, StatePacket(RotatingState(geometry, kOmega)));
    loop.run_for(200ms);
    EXPECT_NEAR(LastPublishedYawRot(env), 0.0, 1e-12);
  }

  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  DrivetrainNode node{loop, robot_cfg.hardware, geometry,
                      devs ? *devs : hardware::Devices{},
                      /*simulate_heading=*/true};
  node.Start(event::MonotonicTime::from_nanos(1'000'000));
  loop.inject(kStateTopic, StatePacket(RotatingState(geometry, kOmega)));
  loop.run_for(200ms);

  // ~200 ms of rotation at 1 rad/s, less the first tick which has no interval.
  const double expected_rot = kOmega / kTau * 0.2;
  const double actual = LastPublishedYawRot(env);
  EXPECT_GT(actual, expected_rot * 0.9);
  EXPECT_LT(actual, expected_rot * 1.02);
}

TEST(Drivetrain, RecordedNetworkInputsReplayWithoutHardware) {
  const std::string path =
      ::testing::TempDir() + "/drive_" + std::to_string(::getpid()) + ".tlog";
  const auto robot_cfg = TestRobotConfig();
  const auto* devs = robot_cfg.GetDevices("drivetrain");
  {
    event::SimulationEnvironment env;
    event::SimulatedEventLoop<event::log::LogWriter> loop{
        env, event::log::LogWriter{path, "drivetrain"}};
    DrivetrainNode node{loop, robot_cfg.hardware, TestGeometry(),
                        devs ? *devs : hardware::Devices{}};
    node.Start(event::MonotonicTime::from_nanos(1'000'000));
    loop.inject(kStateTopic, StatePacket(InitialState()));
    loop.inject(kTargetTopic, ChassisTarget{1, 0.2, 0.1, 0, true});
    loop.run_for(20ms);
    loop.finish();
    ASSERT_FALSE(loop.recorder().failed());
    EXPECT_EQ(LastCommand(env).motors[0].mode, hardware::Mode::kVelocity);
  }
  event::log::LogReader reader{path};
  event::ReplayEventLoop<> replay{reader};
  DrivetrainNode node{replay, robot_cfg.hardware, TestGeometry(),
                      devs ? *devs : hardware::Devices{}};
  node.Start(event::MonotonicTime::from_nanos(1'000'000));
  EXPECT_NO_THROW(replay.run());
  EXPECT_FALSE(replay.diverged());
  EXPECT_TRUE(replay.reached_exit());
  std::remove(path.c_str());
}
}  // namespace
}  // namespace talos::drive
