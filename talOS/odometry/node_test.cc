#include "talOS/odometry/node.h"

#include <gtest/gtest.h>
#include <unistd.h>

#include <cstdio>
#include <string>

#include "talOS/drivetrain/drive_message_generated.h"
#include "talOS/events/log/log_reader.h"
#include "talOS/events/log/log_writer.h"
#include "talOS/events/replay_event_loop.h"
#include "talOS/events/simulated_event_loop.h"
#include "talOS/odometry/odometry_message_generated.h"
#include "talOS/odometry/packet.h"
#include "talOS/shooter/shooter_message_generated.h"

namespace talos::odometry {
namespace {

using namespace std::chrono_literals;

TEST(OdometryNodeTest, PositionIntegration) {
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  OdometryNode node{loop};

  // Initial state at t = 1.0s: stationary
  talos::drive::DrivetrainState s0{
      1'000'000'000, true, 0.0, 0.0, 0.0, 0.0, 0.0};
  loop.inject(kDrivetrainStateTopic, s0);
  loop.run_for(10ms);

  EXPECT_DOUBLE_EQ(node.x(), 0.0);
  EXPECT_DOUBLE_EQ(node.y(), 0.0);
  EXPECT_DOUBLE_EQ(node.yaw_rot(), 0.0);

  // Move forward at vx = 2.0 m/s for 100ms (yaw = 0 => theta = 0)
  // dx = (2.0 * cos(0) - 0) * 0.1 = 0.2 m
  // dy = (2.0 * sin(0) + 0) * 0.1 = 0.0 m
  talos::drive::DrivetrainState s1{
      1'100'000'000, true, 2.0, 0.0, 0.0, 0.0, 0.0};
  loop.inject(kDrivetrainStateTopic, s1);
  loop.run_for(10ms);

  EXPECT_NEAR(node.x(), 0.2, 1e-6);
  EXPECT_NEAR(node.y(), 0.0, 1e-6);

  // Move with yaw = 0.25 rot (theta = pi/2 rad) at vx = 2.0 m/s for 100ms
  // dx = (2.0 * cos(pi/2) - 0) * 0.1 = 0.0 m
  // dy = (2.0 * sin(pi/2) + 0) * 0.1 = 0.2 m
  talos::drive::DrivetrainState s2{
      1'200'000'000, true, 2.0, 0.0, 0.0, 0.25, 0.0};
  loop.inject(kDrivetrainStateTopic, s2);
  loop.run_for(10ms);

  EXPECT_NEAR(node.x(), 0.2, 1e-6);
  EXPECT_NEAR(node.y(), 0.2, 1e-6);
  EXPECT_DOUBLE_EQ(node.yaw_rot(), 0.25);

  // Move laterally at vy = 1.0 m/s at yaw = 0.0 rot for 200ms
  // dx = (0 * cos(0) - 1.0 * sin(0)) * 0.2 = 0.0 m
  // dy = (0 * sin(0) + 1.0 * cos(0)) * 0.2 = 0.2 m
  talos::drive::DrivetrainState s3{
      1'400'000'000, true, 0.0, 1.0, 0.0, 0.0, 0.0};
  loop.inject(kDrivetrainStateTopic, s3);
  loop.run_for(10ms);

  EXPECT_NEAR(node.x(), 0.2, 1e-6);
  EXPECT_NEAR(node.y(), 0.4, 1e-6);

  // Verify published OdometryState message from environment channel
  auto& channel = env.channel(kOdometryTopic, sizeof(OdometryState));
  ASSERT_GT(channel.next_sequence(), 0u);
  OdometryState out{};
  channel.copy_to(channel.next_sequence() - 1,
                  {reinterpret_cast<std::byte*>(&out), sizeof(out)});

  EXPECT_EQ(out.timestamp_ns(), 1'400'000'000u);
  EXPECT_NEAR(out.x_m(), 0.2, 1e-6);
  EXPECT_NEAR(out.y_m(), 0.4, 1e-6);
  EXPECT_DOUBLE_EQ(out.yaw_rot(), 0.0);
}

TEST(OdometryNodeTest, CombinesDrivetrainAndShooterState) {
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  OdometryNode node{loop};

  // Inject shooter state
  talos::shooter::ShooterState shooter_s1{1'000'000'000, 48.5, 48.5, true, true, true};
  loop.inject(kShooterStateTopic, shooter_s1);
  loop.run_for(10ms);

  EXPECT_DOUBLE_EQ(node.flywheel_rps(), 48.5);
  EXPECT_TRUE(node.beam_broken());

  // Inject drivetrain state
  talos::drive::DrivetrainState dt_s1{
      1'020'000'000, true, 1.5, -0.5, 0.8, 0.125, 0.8};
  loop.inject(kDrivetrainStateTopic, dt_s1);
  loop.run_for(10ms);

  auto& channel = env.channel(kOdometryTopic, sizeof(OdometryState));
  ASSERT_GT(channel.next_sequence(), 0u);
  OdometryState out1{};
  channel.copy_to(channel.next_sequence() - 1,
                  {reinterpret_cast<std::byte*>(&out1), sizeof(out1)});

  EXPECT_EQ(out1.timestamp_ns(), 1'020'000'000u);
  EXPECT_DOUBLE_EQ(out1.vx_mps(), 1.5);
  EXPECT_DOUBLE_EQ(out1.vy_mps(), -0.5);
  EXPECT_DOUBLE_EQ(out1.omega_radps(), 0.8);
  EXPECT_DOUBLE_EQ(out1.yaw_rot(), 0.125);
  EXPECT_DOUBLE_EQ(out1.flywheel_rps(), 48.5);
  EXPECT_TRUE(out1.beam_broken());

  // Update shooter state
  talos::shooter::ShooterState shooter_s2{1'030'000'000, 60.0, 60.0, true, false, true};
  loop.inject(kShooterStateTopic, shooter_s2);
  loop.run_for(10ms);

  EXPECT_DOUBLE_EQ(node.flywheel_rps(), 60.0);
  EXPECT_FALSE(node.beam_broken());

  // Subsequent drivetrain state reflects updated shooter state
  talos::drive::DrivetrainState dt_s2{
      1'040'000'000, true, 1.5, -0.5, 0.8, 0.125, 0.8};
  loop.inject(kDrivetrainStateTopic, dt_s2);
  loop.run_for(10ms);

  OdometryState out2{};
  channel.copy_to(channel.next_sequence() - 1,
                  {reinterpret_cast<std::byte*>(&out2), sizeof(out2)});

  EXPECT_EQ(out2.timestamp_ns(), 1'040'000'000u);
  EXPECT_DOUBLE_EQ(out2.flywheel_rps(), 60.0);
  EXPECT_FALSE(out2.beam_broken());
}

TEST(OdometryNodeTest, RecordedNetworkInputsReplayWithoutHardware) {
  const std::string path =
      ::testing::TempDir() + "/odometry_" + std::to_string(::getpid()) + ".tlog";
  {
    event::SimulationEnvironment env;
    event::SimulatedEventLoop<event::log::LogWriter> loop{
        env, event::log::LogWriter{path, "odometry"}};
    OdometryNode node{loop};

    loop.inject(kDrivetrainStateTopic,
                talos::drive::DrivetrainState{
                    1'000'000'000, true, 1.0, 0.0, 0.0, 0.0, 0.0});
    loop.inject(kShooterStateTopic,
                talos::shooter::ShooterState{1'000'000'000, 30.0, 30.0, true, false, true});
    loop.run_for(20ms);

    loop.inject(kDrivetrainStateTopic,
                talos::drive::DrivetrainState{
                    1'050'000'000, true, 1.5, 0.5, 0.2, 0.1, 0.2});
    loop.inject(kShooterStateTopic,
                talos::shooter::ShooterState{1'050'000'000, 35.0, 35.0, true, true, true});
    loop.run_for(20ms);

    loop.finish();
    ASSERT_FALSE(loop.recorder().failed());
  }

  event::log::LogReader reader{path};
  event::ReplayEventLoop<> replay{reader};
  OdometryNode node{replay};

  EXPECT_NO_THROW(replay.run());
  EXPECT_FALSE(replay.diverged());
  EXPECT_TRUE(replay.reached_exit());
  std::remove(path.c_str());
}

}  // namespace
}  // namespace talos::odometry
