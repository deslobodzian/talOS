#include "talOS/driver_station/driver_station.h"

#include <gtest/gtest.h>
#include <array>
#include <cstdio>
#include <cstring>
#include <vector>

namespace talos::driver_station {
namespace {

TEST(DriverStationTest, RoundTripEncoding) {
  DriverStationData original{};
  original.sample_time_us = 123456789;
  original.fms.flags = kDsAttached | kFmsAttached | kEnabled | kTeleop;
  original.fms.alliance = Alliance::kRed;
  original.fms.station = 2;
  original.fms.match_type = MatchType::kQualification;
  original.fms.match_number = 42;
  original.fms.replay_number = 1;
  original.fms.match_time_s = 135.5f;
  std::snprintf(original.fms.game_specific_message, sizeof(original.fms.game_specific_message), "AMP");
  std::snprintf(original.fms.event_name, sizeof(original.fms.event_name), "CMPTX");

  // Stick 0 connected
  original.joysticks[0].connected = true;
  original.joysticks[0].axis_count = 4;
  original.joysticks[0].button_count = 10;
  original.joysticks[0].pov_count = 1;
  original.joysticks[0].buttons = (1u << 0) | (1u << 3);
  original.joysticks[0].axes[0] = 0.5f;
  original.joysticks[0].axes[1] = -0.75f;
  original.joysticks[0].axes[2] = 0.0f;
  original.joysticks[0].axes[3] = 1.0f;
  original.joysticks[0].povs[0] = 90;
  std::snprintf(original.joysticks[0].name, sizeof(original.joysticks[0].name), "Logitech Gamepad");

  // Stick 1 connected
  original.joysticks[1].connected = true;
  original.joysticks[1].axis_count = 2;
  original.joysticks[1].button_count = 4;
  original.joysticks[1].pov_count = 0;
  original.joysticks[1].buttons = (1u << 2);
  original.joysticks[1].axes[0] = -0.2f;
  original.joysticks[1].axes[1] = 0.8f;
  std::snprintf(original.joysticks[1].name, sizeof(original.joysticks[1].name), "Xbox Controller");

  // Sticks 2..5 disconnected

  std::array<uint8_t, 1024> buffer{};
  const auto size = Encode(original, buffer);
  ASSERT_GT(size, 0u);
  ASSERT_LE(size, buffer.size());

  DriverStationData decoded{};
  ASSERT_TRUE(Decode({buffer.data(), size}, decoded));

  EXPECT_EQ(decoded.sample_time_us, original.sample_time_us);
  EXPECT_EQ(decoded.fms.flags, original.fms.flags);
  EXPECT_EQ(decoded.fms.alliance, original.fms.alliance);
  EXPECT_EQ(decoded.fms.station, original.fms.station);
  EXPECT_EQ(decoded.fms.match_type, original.fms.match_type);
  EXPECT_EQ(decoded.fms.match_number, original.fms.match_number);
  EXPECT_EQ(decoded.fms.replay_number, original.fms.replay_number);
  EXPECT_FLOAT_EQ(decoded.fms.match_time_s, original.fms.match_time_s);
  EXPECT_STREQ(decoded.fms.game_specific_message, original.fms.game_specific_message);
  EXPECT_STREQ(decoded.fms.event_name, original.fms.event_name);

  // Check Stick 0
  EXPECT_TRUE(decoded.joysticks[0].connected);
  EXPECT_EQ(decoded.joysticks[0].axis_count, 4);
  EXPECT_EQ(decoded.joysticks[0].button_count, 10);
  EXPECT_EQ(decoded.joysticks[0].pov_count, 1);
  EXPECT_EQ(decoded.joysticks[0].buttons, (1u << 0) | (1u << 3));
  EXPECT_FLOAT_EQ(decoded.joysticks[0].axes[0], 0.5f);
  EXPECT_FLOAT_EQ(decoded.joysticks[0].axes[1], -0.75f);
  EXPECT_EQ(decoded.joysticks[0].povs[0], 90);
  EXPECT_STREQ(decoded.joysticks[0].name, "Logitech Gamepad");

  // Check Stick 1
  EXPECT_TRUE(decoded.joysticks[1].connected);
  EXPECT_EQ(decoded.joysticks[1].axis_count, 2);
  EXPECT_FLOAT_EQ(decoded.joysticks[1].axes[0], -0.2f);
  EXPECT_STREQ(decoded.joysticks[1].name, "Xbox Controller");

  // Check Stick 2
  EXPECT_FALSE(decoded.joysticks[2].connected);
}

TEST(DriverStationTest, UnterminatedStringsTruncateInsteadOfFailing) {
  DriverStationData original{};
  std::memset(original.fms.game_specific_message, 'G', kMaxGameMessageLength);
  std::memset(original.fms.event_name, 'E', kMaxEventNameLength);
  original.joysticks[0].connected = true;
  original.joysticks[0].axis_count = 1;
  std::memset(original.joysticks[0].name, 'N', kMaxNameLength);

  std::array<uint8_t, 1024> buffer{};
  const auto size = Encode(original, buffer);
  ASSERT_GT(size, 0u);

  DriverStationData decoded{};
  ASSERT_TRUE(Decode({buffer.data(), size}, decoded));

  EXPECT_EQ(std::strlen(decoded.fms.game_specific_message),
            kMaxGameMessageLength - 1);
  EXPECT_EQ(std::strlen(decoded.fms.event_name), kMaxEventNameLength - 1);
  EXPECT_EQ(std::strlen(decoded.joysticks[0].name), kMaxNameLength - 1);
  EXPECT_STREQ(decoded.joysticks[0].name, "NNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN");
}

TEST(DriverStationTest, CorruptPayloadRejection) {
  std::array<uint8_t, 10> short_data{};
  DriverStationData ds{};
  EXPECT_FALSE(Decode(short_data, ds));
}

}  // namespace
}  // namespace talos::driver_station
