#include "2026-robot/main_processor/driver_station/node.h"

#include <gtest/gtest.h>

#include <chrono>

#include "2026-robot/main_processor/driver_station/driver_station_message_generated.h"
#include "2026-robot/main_processor/driver_station/packet.h"
#include "talOS/events/simulated_event_loop.h"

namespace talos::driver_station {
namespace {

using namespace std::chrono_literals;

Packet CreatePacket(const DriverStationData& data) {
  Packet pkt{};
  pkt.size = static_cast<uint32_t>(Encode(data, pkt.data));
  return pkt;
}

TEST(DriverStationNodeTest, DecodesPacketAndRepublishesItVerbatim) {
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

  // Requests are not this node's business; see the operator_interface
  // tests for the mapping that turns this state into targets.
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

  // What those sticks should mean is the operator interface's business.
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
