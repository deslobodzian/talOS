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
  original.mode_since_us = 123400000;
  original.fms.flags = kDsAttached | kFmsAttached | kEnabled | kTeleop;
  original.fms.alliance = Alliance::kRed;
  original.fms.station = 2;
  original.fms.match_type = MatchType::kQualification;
  original.fms.match_number = 42;
  original.fms.replay_number = 1;
  original.fms.match_time_s = 135.5f;
  std::snprintf(original.fms.game_specific_message,
                sizeof(original.fms.game_specific_message), "AMP");
  std::snprintf(original.fms.event_name, sizeof(original.fms.event_name),
                "CMPTX");

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
  std::snprintf(original.joysticks[0].name, sizeof(original.joysticks[0].name),
                "Logitech Gamepad");

  // Stick 1 connected
  original.joysticks[1].connected = true;
  original.joysticks[1].axis_count = 2;
  original.joysticks[1].button_count = 4;
  original.joysticks[1].pov_count = 0;
  original.joysticks[1].buttons = (1u << 2);
  original.joysticks[1].axes[0] = -0.2f;
  original.joysticks[1].axes[1] = 0.8f;
  std::snprintf(original.joysticks[1].name, sizeof(original.joysticks[1].name),
                "Xbox Controller");

  // Sticks 2..5 disconnected

  std::array<uint8_t, 1024> buffer{};
  const auto size = Encode(original, buffer);
  ASSERT_GT(size, 0u);
  ASSERT_LE(size, buffer.size());

  DriverStationData decoded{};
  ASSERT_TRUE(Decode({buffer.data(), size}, decoded));

  EXPECT_EQ(decoded.sample_time_us, original.sample_time_us);
  EXPECT_EQ(decoded.mode_since_us, original.mode_since_us);
  EXPECT_EQ(decoded.fms.flags, original.fms.flags);
  EXPECT_EQ(decoded.fms.alliance, original.fms.alliance);
  EXPECT_EQ(decoded.fms.station, original.fms.station);
  EXPECT_EQ(decoded.fms.match_type, original.fms.match_type);
  EXPECT_EQ(decoded.fms.match_number, original.fms.match_number);
  EXPECT_EQ(decoded.fms.replay_number, original.fms.replay_number);
  EXPECT_FLOAT_EQ(decoded.fms.match_time_s, original.fms.match_time_s);
  EXPECT_STREQ(decoded.fms.game_specific_message,
               original.fms.game_specific_message);
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

TEST(DriverStationTest, FullyPopulatedEncodingReachesMaximumSize) {
  DriverStationData original{};
  original.sample_time_us = 123456789;
  original.mode_since_us = 123400000;
  original.fms.flags = kDsAttached | kFmsAttached | kEnabled | kTeleop;
  original.fms.alliance = Alliance::kBlue;
  original.fms.station = 3;
  original.fms.match_type = MatchType::kQualification;
  original.fms.match_number = 42;
  original.fms.replay_number = 1;
  original.fms.match_time_s = 135.5f;
  std::memset(original.fms.game_specific_message, 'G',
              kMaxGameMessageLength - 1);
  std::memset(original.fms.event_name, 'E', kMaxEventNameLength - 1);
  for (auto& stick : original.joysticks) {
    stick.connected = true;
    stick.axis_count = kMaxAxes;
    stick.button_count = kMaxButtons;
    stick.pov_count = kMaxPovs;
    stick.buttons = 0xffffffffu;
    stick.axes.fill(0.5f);
    stick.povs.fill(270);
    std::memset(stick.name, 'N', kMaxNameLength - 1);
  }

  std::array<uint8_t, kMaxEncodedSize> buffer{};
  const auto size = Encode(original, buffer);
  ASSERT_EQ(size, kMaxEncodedSize);
  DriverStationData decoded{};
  ASSERT_TRUE(Decode({buffer.data(), size}, decoded));
  EXPECT_EQ(decoded, original);
  EXPECT_EQ(Encode(original, {buffer.data(), buffer.size() - 1}), 0u);
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

DriverStationData Sample(uint64_t time_us, uint32_t flags) {
  DriverStationData ds{};
  ds.sample_time_us = time_us;
  ds.fms.flags = flags;
  return ds;
}

// The generation each sample is stamped with.
uint64_t Stamp(ModeTracker& tracker, uint64_t time_us, uint32_t flags) {
  DriverStationData ds = Sample(time_us, flags);
  tracker.Stamp(ds);
  return ds.mode_since_us;
}

constexpr uint32_t kTeleopEnabled = kDsAttached | kEnabled | kTeleop;

TEST(ModeTrackerTest, GenerationIsTheFirstSampleOfEachMode) {
  ModeTracker tracker;
  EXPECT_EQ(Stamp(tracker, 1000, kDsAttached | kTeleop), 1000u);
  EXPECT_EQ(Stamp(tracker, 1020, kDsAttached | kTeleop), 1000u);
  EXPECT_EQ(Stamp(tracker, 1040, kTeleopEnabled), 1040u);
  EXPECT_EQ(Stamp(tracker, 1060, kTeleopEnabled), 1040u);
  // Disabling and re-enabling in the same mode is a new generation: what
  // was asked before the disable must not drive after it.
  EXPECT_EQ(Stamp(tracker, 1080, kDsAttached | kTeleop), 1080u);
  EXPECT_EQ(Stamp(tracker, 1100, kTeleopEnabled), 1100u);
  EXPECT_EQ(Stamp(tracker, 1120, kDsAttached | kEnabled | kAutonomous), 1120u);
  EXPECT_EQ(Stamp(tracker, 1140, kTeleopEnabled | kEStop), 1140u);
}

TEST(ModeTrackerTest, FlapBetweenPublishedSamplesStartsANewGeneration) {
  ModeTracker tracker;
  const uint64_t before = Stamp(tracker, 20000, kTeleopEnabled);
  ASSERT_NE(before, 0u);

  // Hardware ticks observe disable/re-enable before the next 20 ms packet.
  tracker.Observe(kDsAttached | kTeleop, 25000);
  tracker.Observe(kTeleopEnabled, 30000);
  tracker.Observe(kTeleopEnabled, 35000);
  const uint64_t after = Stamp(tracker, 40000, kTeleopEnabled);
  EXPECT_EQ(after, 30000u);
  EXPECT_GT(after, before);
  EXPECT_EQ(Stamp(tracker, 60000, kTeleopEnabled), after);
}

TEST(ModeTrackerTest, SteadyHardwareTicksKeepThePublishedGeneration) {
  ModeTracker tracker;
  tracker.Observe(kTeleopEnabled, 20000);
  for (uint64_t now_us = 25000; now_us <= 40000; now_us += 5000) {
    tracker.Observe(kTeleopEnabled | kFmsAttached, now_us);
  }
  EXPECT_EQ(Stamp(tracker, 40000, kTeleopEnabled), 20000u);
}

TEST(ModeTrackerTest, EveryModeFlagObservedBetweenSamplesStartsAGeneration) {
  for (const uint32_t bit :
       {kDsAttached, kEnabled, kAutonomous, kTeleop, kTest, kEStop}) {
    ModeTracker tracker;
    const uint64_t before = Stamp(tracker, 20000, kTeleopEnabled);
    tracker.Observe(kTeleopEnabled ^ bit, 25000);
    tracker.Observe(kTeleopEnabled, 30000);
    EXPECT_GT(Stamp(tracker, 40000, kTeleopEnabled), before) << bit;
  }
}

TEST(ModeTrackerTest, EveryModeFlagStartsAGeneration) {
  for (const uint32_t bit :
       {kDsAttached, kEnabled, kAutonomous, kTeleop, kTest, kEStop}) {
    ModeTracker tracker;
    Stamp(tracker, 10, kTeleopEnabled);
    EXPECT_EQ(Stamp(tracker, 20, kTeleopEnabled ^ bit), 20u) << bit;
  }
}

TEST(ModeTrackerTest, FmsAndJoysticksDoNotStartAGeneration) {
  ModeTracker tracker;
  Stamp(tracker, 10, kTeleopEnabled);
  DriverStationData ds = Sample(20, kTeleopEnabled | kFmsAttached);
  ds.fms.match_time_s = 90.0f;
  ds.joysticks[0].connected = true;
  ds.joysticks[0].buttons = 1;
  tracker.Stamp(ds);
  EXPECT_EQ(ds.mode_since_us, 10u);
}

TEST(ModeTrackerTest, GenerationIsNeverZeroAndAlwaysMovesForward) {
  ModeTracker tracker;
  // A sampler whose clock starts at 0 still stamps a generation no target
  // mistakes for "none".
  EXPECT_EQ(Stamp(tracker, 0, kTeleopEnabled), 1u);
  // A mode change sampled at a repeated or earlier time still differs from
  // the generation before it.
  EXPECT_EQ(Stamp(tracker, 1, kDsAttached), 2u);
  EXPECT_EQ(Stamp(tracker, 1, kTeleopEnabled), 3u);
  EXPECT_EQ(Stamp(tracker, 500, kDsAttached), 500u);
}

}  // namespace
}  // namespace talos::driver_station
