#include "talOS/driver_station/node.h"

#include <gtest/gtest.h>

#include <chrono>

#include "talOS/driver_station/driver_station_message_generated.h"
#include "talOS/drivetrain/drive_message_generated.h"
#include "talOS/drivetrain/packet.h"
#include "talOS/events/simulated_event_loop.h"
#include "talOS/shooter/packet.h"
#include "talOS/shooter/shooter_message_generated.h"

namespace talos::driver_station {
namespace {

using namespace std::chrono_literals;
using talos::drive::Packet;

Packet CreatePacket(const DriverStationData& data) {
  Packet pkt{};
  pkt.size = static_cast<uint32_t>(Encode(data, pkt.data));
  return pkt;
}

TEST(DriverStationNodeTest, DecodesPacketAndPublishesStateAndTargets) {
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  DriverStationNode node{loop};

  DriverStationData data{};
  data.sample_time_us = 12345000;
  data.fms.flags = kEnabled | kTeleop | kDsAttached;
  data.fms.alliance = Alliance::kRed;
  data.fms.station = 1;
  data.fms.match_type = MatchType::kQualification;
  data.fms.match_number = 42;
  data.fms.replay_number = 1;
  data.fms.match_time_s = 120.5f;

  data.joysticks[0].connected = true;
  data.joysticks[0].axis_count = 4;
  data.joysticks[0].button_count = 8;
  data.joysticks[0].pov_count = 1;
  data.joysticks[0].buttons = 1;  // Button 1 pressed
  data.joysticks[0].axes[0] = 0.5f;   // vy raw = -0.5
  data.joysticks[0].axes[1] = -0.75f;  // vx raw = 0.75
  data.joysticks[0].axes[2] = 0.25f;  // omega raw = -0.25
  data.joysticks[0].axes[3] = 0.0f;
  data.joysticks[0].povs[0] = 90;

  data.joysticks[1].connected = true;
  data.joysticks[1].axis_count = 2;
  data.joysticks[1].button_count = 4;
  data.joysticks[1].buttons = 4;

  loop.inject(kHwDsTopic, CreatePacket(data));
  loop.run_for(10ms);

  // 1. Verify DriverStationState on /driver_station/state
  auto& ds_channel =
      env.channel(kDsStateTopic, sizeof(DriverStationState));
  ASSERT_GT(ds_channel.next_sequence(), 0u);
  DriverStationState ds_state{};
  ds_channel.copy_to(ds_channel.next_sequence() - 1,
                     {reinterpret_cast<std::byte*>(&ds_state), sizeof(ds_state)});

  // timestamp_ns is the loop monotonic clock; the Rio stamp rides separately.
  EXPECT_EQ(ds_state.rio_sample_time_us(), 12345000ULL);
  EXPECT_NE(ds_state.timestamp_ns(), 12345000000ULL);
  EXPECT_LE(ds_state.timestamp_ns(),
            static_cast<uint64_t>(loop.monotonic_now().nanos()));
  EXPECT_EQ(ds_state.fms().flags(), kEnabled | kTeleop | kDsAttached);
  EXPECT_EQ(ds_state.fms().alliance(), static_cast<uint8_t>(Alliance::kRed));
  EXPECT_EQ(ds_state.fms().station(), 1);
  EXPECT_EQ(ds_state.fms().match_type(),
            static_cast<uint8_t>(MatchType::kQualification));
  EXPECT_EQ(ds_state.fms().match_number(), 42);
  EXPECT_EQ(ds_state.fms().replay_number(), 1);
  EXPECT_FLOAT_EQ(ds_state.fms().match_time_s(), 120.5f);

  EXPECT_TRUE(ds_state.stick0().connected());
  EXPECT_EQ(ds_state.stick0().buttons(), 1u);
  EXPECT_FLOAT_EQ(ds_state.stick0().axis0(), 0.5f);
  EXPECT_FLOAT_EQ(ds_state.stick0().axis1(), -0.75f);
  EXPECT_FLOAT_EQ(ds_state.stick0().axis2(), 0.25f);
  EXPECT_EQ(ds_state.stick0().pov0(), 90);

  EXPECT_TRUE(ds_state.stick1().connected());
  EXPECT_EQ(ds_state.stick1().buttons(), 4u);

  // 2. Verify ChassisTarget on /drivetrain/tgt
  auto& drive_channel = env.channel(talos::drive::kTargetTopic,
                                    sizeof(talos::drive::ChassisTarget));
  ASSERT_GT(drive_channel.next_sequence(), 0u);
  talos::drive::ChassisTarget chassis_tgt{};
  drive_channel.copy_to(drive_channel.next_sequence() - 1,
                        {reinterpret_cast<std::byte*>(&chassis_tgt),
                         sizeof(chassis_tgt)});

  // Magnitudes are rescaled past the 0.05 deadband: (|raw| - 0.05) / 0.95.
  // raw_vx = -(-0.75) = 0.75 -> 0.70 / 0.95 * 4.0
  // raw_vy = -(0.5) = -0.5 -> -(0.45 / 0.95) * 4.0
  // raw_omega = -(0.25) = -0.25 -> -(0.20 / 0.95) * 6.0
  EXPECT_NEAR(chassis_tgt.vx_mps(), 0.70 / 0.95 * 4.0, 1e-9);
  EXPECT_NEAR(chassis_tgt.vy_mps(), -(0.45 / 0.95) * 4.0, 1e-9);
  EXPECT_NEAR(chassis_tgt.omega_radps(), -(0.20 / 0.95) * 6.0, 1e-9);
  EXPECT_TRUE(chassis_tgt.enabled());

  // 3. Verify ShooterTarget on the topic ShooterNode actually watches
  auto& shoot_channel = env.channel(talos::shooter::kShooterTargetTopic,
                                    sizeof(talos::shooter::ShooterTarget));
  ASSERT_GT(shoot_channel.next_sequence(), 0u);
  talos::shooter::ShooterTarget shoot_tgt{};
  shoot_channel.copy_to(shoot_channel.next_sequence() - 1,
                        {reinterpret_cast<std::byte*>(&shoot_tgt),
                         sizeof(shoot_tgt)});

  EXPECT_DOUBLE_EQ(shoot_tgt.target_velocity_rps(), 60.0);
  EXPECT_TRUE(shoot_tgt.enabled());
}

TEST(DriverStationNodeTest, AppliesDeadbandAndButtonRelease) {
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  DriverStationNode node{loop};

  DriverStationData data{};
  data.fms.flags = kEnabled | kTeleop;
  data.joysticks[0].connected = true;
  data.joysticks[0].axis_count = 4;
  // Axes within 0.05 deadband
  data.joysticks[0].axes[0] = 0.03f;
  data.joysticks[0].axes[1] = -0.04f;
  data.joysticks[0].axes[2] = 0.01f;
  data.joysticks[0].buttons = 0;  // Button not pressed

  loop.inject(kHwDsTopic, CreatePacket(data));
  loop.run_for(10ms);

  auto& drive_channel = env.channel(talos::drive::kTargetTopic,
                                    sizeof(talos::drive::ChassisTarget));
  ASSERT_GT(drive_channel.next_sequence(), 0u);
  talos::drive::ChassisTarget chassis_tgt{};
  drive_channel.copy_to(drive_channel.next_sequence() - 1,
                        {reinterpret_cast<std::byte*>(&chassis_tgt),
                         sizeof(chassis_tgt)});

  EXPECT_DOUBLE_EQ(chassis_tgt.vx_mps(), 0.0);
  EXPECT_DOUBLE_EQ(chassis_tgt.vy_mps(), 0.0);
  EXPECT_DOUBLE_EQ(chassis_tgt.omega_radps(), 0.0);
  EXPECT_TRUE(chassis_tgt.enabled());

  auto& shoot_channel = env.channel(talos::shooter::kShooterTargetTopic,
                                    sizeof(talos::shooter::ShooterTarget));
  ASSERT_GT(shoot_channel.next_sequence(), 0u);
  talos::shooter::ShooterTarget shoot_tgt{};
  shoot_channel.copy_to(shoot_channel.next_sequence() - 1,
                        {reinterpret_cast<std::byte*>(&shoot_tgt),
                         sizeof(shoot_tgt)});

  EXPECT_DOUBLE_EQ(shoot_tgt.target_velocity_rps(), 0.0);
  EXPECT_FALSE(shoot_tgt.enabled());
}

TEST(DriverStationNodeTest, DeadbandIsContinuousAtTheThreshold) {
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  DriverStationNode node{loop};

  DriverStationData data{};
  data.fms.flags = kEnabled | kTeleop;
  data.joysticks[0].connected = true;
  data.joysticks[0].axis_count = 4;
  // Just outside the 0.05 deadband.
  data.joysticks[0].axes[1] = -0.06f;

  loop.inject(kHwDsTopic, CreatePacket(data));
  loop.run_for(10ms);

  auto& drive_channel = env.channel(talos::drive::kTargetTopic,
                                    sizeof(talos::drive::ChassisTarget));
  ASSERT_GT(drive_channel.next_sequence(), 0u);
  talos::drive::ChassisTarget chassis_tgt{};
  drive_channel.copy_to(drive_channel.next_sequence() - 1,
                        {reinterpret_cast<std::byte*>(&chassis_tgt),
                         sizeof(chassis_tgt)});

  // Rescaled: (0.06 - 0.05) / 0.95 * 4.0, not the 0.24 an unscaled deadband
  // would step to.
  EXPECT_NEAR(chassis_tgt.vx_mps(), 0.01 / 0.95 * 4.0, 1e-6);
  EXPECT_LT(chassis_tgt.vx_mps(), 0.05);
  EXPECT_GT(chassis_tgt.vx_mps(), 0.0);
}

TEST(DriverStationNodeTest, PublishesOnlyTheFirstTwoJoysticks) {
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  DriverStationNode node{loop};

  // The wire format carries kMaxJoysticks sticks. DriverStationState exposes
  // stick0 and stick1 only, and the teleop mapping reads stick 0; the sticks
  // past that are dropped at the state boundary on purpose.
  static_assert(kMaxJoysticks == 6);
  DriverStationData data{};
  data.fms.flags = kEnabled | kTeleop;
  for (std::size_t i = 0; i < kMaxJoysticks; ++i) {
    data.joysticks[i].connected = true;
    data.joysticks[i].axis_count = 4;
    data.joysticks[i].buttons = 1u << i;
    data.joysticks[i].axes[1] = -0.1f * static_cast<float>(i + 1);
  }

  const Packet pkt = CreatePacket(data);
  loop.inject(kHwDsTopic, pkt);
  loop.run_for(10ms);

  // The codec keeps every stick, so the narrowing is the node's choice.
  DriverStationData decoded{};
  ASSERT_TRUE(Decode(pkt.bytes(), decoded));
  EXPECT_EQ(decoded.joysticks[kMaxJoysticks - 1].buttons,
            1u << (kMaxJoysticks - 1));

  auto& ds_channel = env.channel(kDsStateTopic, sizeof(DriverStationState));
  ASSERT_GT(ds_channel.next_sequence(), 0u);
  DriverStationState ds_state{};
  ds_channel.copy_to(ds_channel.next_sequence() - 1,
                     {reinterpret_cast<std::byte*>(&ds_state), sizeof(ds_state)});

  EXPECT_EQ(ds_state.stick0().buttons(), 1u);
  EXPECT_EQ(ds_state.stick1().buttons(), 2u);
  EXPECT_FLOAT_EQ(ds_state.stick0().axis1(), -0.1f);
  EXPECT_FLOAT_EQ(ds_state.stick1().axis1(), -0.2f);

  // Targets follow stick 0 alone.
  auto& drive_channel = env.channel(talos::drive::kTargetTopic,
                                    sizeof(talos::drive::ChassisTarget));
  ASSERT_GT(drive_channel.next_sequence(), 0u);
  talos::drive::ChassisTarget chassis_tgt{};
  drive_channel.copy_to(drive_channel.next_sequence() - 1,
                        {reinterpret_cast<std::byte*>(&chassis_tgt),
                         sizeof(chassis_tgt)});
  EXPECT_NEAR(chassis_tgt.vx_mps(), (0.1 - 0.05) / 0.95 * 4.0, 1e-6);
}

TEST(DriverStationNodeTest, NoTargetsWhenDisabledOrAutonomous) {
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  DriverStationNode node{loop};

  DriverStationData data{};
  data.fms.flags = kAutonomous | kEnabled;  // Enabled but Autonomous, not Teleop
  data.joysticks[0].connected = true;
  data.joysticks[0].axes[1] = -0.5f;

  loop.inject(kHwDsTopic, CreatePacket(data));
  loop.run_for(10ms);

  // State should be published
  auto& ds_channel =
      env.channel(kDsStateTopic, sizeof(DriverStationState));
  EXPECT_GT(ds_channel.next_sequence(), 0u);

  // But no targets should be sent
  auto& drive_channel = env.channel(talos::drive::kTargetTopic,
                                    sizeof(talos::drive::ChassisTarget));
  EXPECT_EQ(drive_channel.next_sequence(), 0u);

  auto& shoot_channel = env.channel(talos::shooter::kShooterTargetTopic,
                                    sizeof(talos::shooter::ShooterTarget));
  EXPECT_EQ(shoot_channel.next_sequence(), 0u);
}

TEST(DriverStationNodeTest, NoTargetsWhenJoystick0Disconnected) {
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  DriverStationNode node{loop};

  DriverStationData data{};
  data.fms.flags = kEnabled | kTeleop;
  data.joysticks[0].connected = false;

  loop.inject(kHwDsTopic, CreatePacket(data));
  loop.run_for(10ms);

  // State published
  auto& ds_channel =
      env.channel(kDsStateTopic, sizeof(DriverStationState));
  EXPECT_GT(ds_channel.next_sequence(), 0u);

  // No targets
  auto& drive_channel = env.channel(talos::drive::kTargetTopic,
                                    sizeof(talos::drive::ChassisTarget));
  EXPECT_EQ(drive_channel.next_sequence(), 0u);

  auto& shoot_channel = env.channel(talos::shooter::kShooterTargetTopic,
                                    sizeof(talos::shooter::ShooterTarget));
  EXPECT_EQ(shoot_channel.next_sequence(), 0u);
}

TEST(DriverStationNodeTest, CorruptPacketIgnored) {
  event::SimulationEnvironment env;
  event::SimulatedEventLoop<> loop{env};
  DriverStationNode node{loop};

  Packet corrupt{};
  corrupt.size = 2;  // Too small to decode
  corrupt.data[0] = 0xFF;
  corrupt.data[1] = 0xFF;

  loop.inject(kHwDsTopic, corrupt);
  loop.run_for(10ms);

  auto& ds_channel =
      env.channel(kDsStateTopic, sizeof(DriverStationState));
  EXPECT_EQ(ds_channel.next_sequence(), 0u);
}

}  // namespace
}  // namespace talos::driver_station
