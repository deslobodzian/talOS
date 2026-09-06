#include "talOS/drivetrain/node.h"

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdio>
#include <limits>

#include "common/hardware/sim_backend.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/replay_event_loop.h"
#include "talOS/events/simulated_event_loop.h"

namespace talos::drive {
namespace {
using namespace std::chrono_literals;

config::RobotConfig TestRobotConfig() {
  auto cfg = config::ParseRobotConfig("talOS/configuration/robot.toml");
  cfg.hardware.commissioned = true;
  return cfg;
}

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
  const double rps =
      1.0 / (2 * std::numbers::pi * kSwerveModules[0].wheel_radius_m);
  for (double vx : {1.0, -1.0}) {
    const auto c = SwerveCommand(config, state, vx, 0, 0);
    ASSERT_EQ(c.count, static_cast<uint16_t>(config.motors.size()));
    for (int i = 0; i < 4; ++i) {
      EXPECT_EQ(c.motors[2 * i].mode, hardware::Mode::kVelocity);
      EXPECT_NEAR(c.motors[2 * i].demand, vx * rps, 1e-12);
      EXPECT_NEAR(c.motors[2 * i + 1].demand, 0, 1e-12);
    }
  }
  const auto c = SwerveCommand(config, state, 0, 1, 0);
  for (int i = 0; i < 4; ++i)
    EXPECT_NEAR(c.motors[2 * i + 1].demand, 0.25, 1e-12);
}
TEST(Swerve, RotationDesaturationAndInvalidInput) {
  const auto robot_cfg = TestRobotConfig();
  const auto& config = robot_cfg.hardware;
  const auto state = InitialState();
  auto c = SwerveCommand(config, state, 0, 0, 1);
  EXPECT_LT(c.motors[0].demand, 0);
  EXPECT_GT(c.motors[2].demand, 0);
  EXPECT_NEAR(c.motors[1].demand, 0.125, 1e-12);
  EXPECT_NEAR(c.motors[3].demand, -0.125, 1e-12);
  c = SwerveCommand(config, state, 100, 0, 0);
  for (int i = 0; i < 4; ++i) EXPECT_NEAR(c.motors[2 * i].demand, 20, 1e-12);
  c = SwerveCommand(config, state, std::numeric_limits<double>::quiet_NaN(), 0,
                    0);
  for (int i = 0; i < 8; ++i)
    EXPECT_EQ(c.motors[i].mode, hardware::Mode::kNeutral);
}
TEST(Drivetrain, StaleTargetAndDisabledGatewayNeutralizeAllMotors) {
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  const auto robot_cfg = TestRobotConfig();
  const auto* devs = robot_cfg.GetDevices("drivetrain");
  DrivetrainNode node{loop, robot_cfg.hardware, devs ? *devs : hardware::Devices{}};
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
TEST(Drivetrain, RecordedNetworkInputsReplayWithoutHardware) {
  const std::string path =
      ::testing::TempDir() + "/drive_" + std::to_string(::getpid()) + ".tlog";
  const auto robot_cfg = TestRobotConfig();
  const auto* devs = robot_cfg.GetDevices("drivetrain");
  {
    event::SimulationEnvironment env;
    event::SimulatedEventLoop<event::log::LogWriter> loop{
        env, event::log::LogWriter{path, "drivetrain"}};
    DrivetrainNode node{loop, robot_cfg.hardware, devs ? *devs : hardware::Devices{}};
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
  DrivetrainNode node{replay, robot_cfg.hardware, devs ? *devs : hardware::Devices{}};
  node.Start(event::MonotonicTime::from_nanos(1'000'000));
  EXPECT_NO_THROW(replay.run());
  EXPECT_FALSE(replay.diverged());
  EXPECT_TRUE(replay.reached_exit());
  std::remove(path.c_str());
}
}  // namespace
}  // namespace talos::drive
