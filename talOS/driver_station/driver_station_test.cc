#include "talOS/driver_station/driver_station.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <vector>

namespace talos::driver_station {
namespace {

// A Driver Station packet the RoboRIO and the main processor agree on byte
// for byte, whatever either host's byte order: the fields in the order
// docs/DATA_FLOW.md gives, every integer and float little-endian, each string
// a length byte then its bytes, a disconnected joystick one zero byte.
// REQ-CONTRACT-03
TEST(DriverStationTest, PacketIsLittleEndianOnTheWire) {
  DriverStationData ds;
  ds.sample_time_us = 0x0102030405060708ull;
  ds.mode_since_us = 0x1112131415161718ull;
  ds.boot_id = 0x2122232425262728ull;
  ds.fms.flags = 0x31323334u;
  ds.fms.alliance = Alliance::kBlue;
  ds.fms.station = 3;
  ds.fms.match_type = MatchType::kElimination;
  ds.fms.match_number = 0x0A0B;
  ds.fms.replay_number = 5;
  ds.fms.match_time_s = 97.5f;
  std::memcpy(ds.fms.game_specific_message, "ab", 2);
  std::memcpy(ds.fms.event_name, "c", 1);
  auto& stick = ds.joysticks[1];
  stick.connected = true;
  stick.axis_count = 2;
  stick.button_count = 3;
  stick.pov_count = 1;
  stick.buttons = 0x41424344u;
  stick.axes[0] = 0.5f;
  stick.axes[1] = -1.0f;
  stick.povs[0] = -1;
  std::memcpy(stick.name, "x", 1);
  const std::vector<uint8_t> expected{
      0x08, 0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01,  // sample_time_us
      0x18, 0x17, 0x16, 0x15, 0x14, 0x13, 0x12, 0x11,  // mode_since_us
      0x28, 0x27, 0x26, 0x25, 0x24, 0x23, 0x22, 0x21,  // boot_id
      0x34, 0x33, 0x32, 0x31,                          // flags
      2,    3,    3,                                   // alliance..match_type
      0x0B, 0x0A, 5,                                   // match, replay number
      0x00, 0x00, 0xC3, 0x42,                          // match_time_s 97.5f
      2,    'a',  'b',  1,    'c',                     // game message, event
      0,                                               // joystick 0
      1,    2,    3,    1,                             // joystick 1, counts
      0x44, 0x43, 0x42, 0x41,                          // buttons
      0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x80, 0xBF,  // axes 0.5f, -1.0f
      0xFF, 0xFF, 1,    'x',                           // POV -1, name
      0,    0,    0,    0};                            // joysticks 2..5
  std::array<uint8_t, kMaxEncodedSize> bytes{};
  const auto size = Encode(ds, bytes);
  ASSERT_EQ(size, expected.size());
  EXPECT_TRUE(std::equal(expected.begin(), expected.end(), bytes.begin()));
  DriverStationData decoded;
  ASSERT_TRUE(Decode(expected, decoded));
  EXPECT_EQ(decoded, ds);
}

// REQ-CONTRACT-02
TEST(DriverStationTest, FullyPopulatedEncodingReachesMaximumSize) {
  DriverStationData original{};
  original.sample_time_us = 123456789;
  original.mode_since_us = 123400000;
  original.boot_id = 0xfedcba9876543211ULL;
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

// REQ-CONTRACT-02
TEST(DriverStationTest, CorruptPayloadRejection) {
  DriverStationData base{};
  base.sample_time_us = 1000;
  base.mode_since_us = 1000;
  base.boot_id = 7;
  base.fms.flags = kDsAttached | kEnabled | kTeleop;
  auto& stick = base.joysticks[0];
  stick.connected = true;
  stick.axis_count = 2;
  stick.button_count = 3;
  stick.pov_count = 1;
  stick.buttons = 0x5u;
  stick.axes[0] = 0.25f;
  stick.axes[1] = -0.5f;
  stick.povs[0] = 90;
  std::memcpy(stick.name, "pad", 3);

  std::array<uint8_t, kMaxEncodedSize + 1> encoded{};
  const std::size_t size = Encode(base, {encoded.data(), kMaxEncodedSize});
  ASSERT_GT(size, 0u);
  DriverStationData decoded{};
  ASSERT_TRUE(Decode({encoded.data(), size}, decoded));

  // Every prefix truncation 0..size-1 is rejected.
  for (std::size_t truncated = 0; truncated < size; ++truncated) {
    EXPECT_FALSE(Decode({encoded.data(), truncated}, decoded))
        << "accepted truncated length " << truncated << " of " << size;
  }

  // Trailing byte (size + 1) is rejected.
  encoded[size] = 0x00;
  EXPECT_FALSE(Decode({encoded.data(), size + 1}, decoded));

  // Layout offsets for base (game_specific_message and event_name are empty):
  //   38: game_specific_message length (0)
  //   39: event_name length (0)
  //   40: joysticks[0].connected (1)
  //   41: axis_count (2), 42: button_count (3), 43: pov_count (1)
  //   44..47: buttons, 48..55: 2 axes, 56..57: 1 pov, 58: name length (3)
  constexpr std::size_t kGameMsgLenOffset = 38;
  constexpr std::size_t kEventNameLenOffset = 39;
  constexpr std::size_t kAxisCountOffset = 41;
  constexpr std::size_t kButtonCountOffset = 42;
  constexpr std::size_t kPovCountOffset = 43;
  constexpr std::size_t kAxesStartOffset = 48;
  constexpr std::size_t kPovsStartOffset = 56;
  constexpr std::size_t kStickNameLenOffset = 58;
  ASSERT_EQ(encoded[kGameMsgLenOffset], 0u);
  ASSERT_EQ(encoded[kEventNameLenOffset], 0u);
  ASSERT_EQ(encoded[kAxisCountOffset], 2u);
  ASSERT_EQ(encoded[kButtonCountOffset], 3u);
  ASSERT_EQ(encoded[kPovCountOffset], 1u);
  ASSERT_EQ(encoded[kStickNameLenOffset], 3u);

  const std::vector<uint8_t> valid(encoded.begin(), encoded.begin() + size);

  // Over-cap axis_count (> 12), padded with matching float bytes so only the
  // cap check rejects it.
  {
    auto bad = valid;
    bad[kAxisCountOffset] = static_cast<uint8_t>(kMaxAxes + 1);
    bad.insert(bad.begin() + static_cast<std::ptrdiff_t>(kAxesStartOffset + 8),
               (kMaxAxes + 1 - 2) * 4, 0);
    EXPECT_FALSE(Decode(bad, decoded));
  }

  // Over-cap button_count (> 32).
  {
    auto bad = valid;
    bad[kButtonCountOffset] = static_cast<uint8_t>(kMaxButtons + 1);
    EXPECT_FALSE(Decode(bad, decoded));
  }

  // Over-cap pov_count (> 4), padded with matching int16 bytes.
  {
    auto bad = valid;
    bad[kPovCountOffset] = static_cast<uint8_t>(kMaxPovs + 1);
    bad.insert(bad.begin() + static_cast<std::ptrdiff_t>(kPovsStartOffset + 2),
               (kMaxPovs + 1 - 1) * 2, 0);
    EXPECT_FALSE(Decode(bad, decoded));
  }

  // Over-cap string lengths (> 31, i.e. 32 bytes) for every wire string,
  // padded with matching string bytes so only the length cap rejects it.
  for (const auto& [offset, old_len] :
       {std::pair{kGameMsgLenOffset, std::size_t{0}},
        std::pair{kEventNameLenOffset, std::size_t{0}},
        std::pair{kStickNameLenOffset, std::size_t{3}}}) {
    auto bad = valid;
    constexpr uint8_t kTooLong = 32;  // > 31 max wire length
    bad[offset] = kTooLong;
    bad.insert(bad.begin() + static_cast<std::ptrdiff_t>(offset + 1 + old_len),
               kTooLong - old_len, static_cast<uint8_t>('A'));
    EXPECT_FALSE(Decode(bad, decoded)) << "offset " << offset;
  }
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
  // Hardware ticks observe any mode-flag change and its reversal before the
  // next 20 ms packet; the packet carries the generation of the return.
  for (const uint32_t bit :
       {kDsAttached, kEnabled, kAutonomous, kTeleop, kTest, kEStop}) {
    ModeTracker tracker;
    Stamp(tracker, 20000, kTeleopEnabled);
    tracker.Observe(kTeleopEnabled ^ bit, 25000);
    tracker.Observe(kTeleopEnabled, 30000);
    tracker.Observe(kTeleopEnabled, 35000);
    EXPECT_EQ(Stamp(tracker, 40000, kTeleopEnabled), 30000u) << bit;
    EXPECT_EQ(Stamp(tracker, 60000, kTeleopEnabled), 30000u) << bit;
  }
}

TEST(ModeTrackerTest, SteadyHardwareTicksKeepThePublishedGeneration) {
  // kFmsAttached is not a mode flag.
  ModeTracker tracker;
  tracker.Observe(kTeleopEnabled, 20000);
  for (uint64_t now_us = 25000; now_us <= 40000; now_us += 5000) {
    tracker.Observe(kTeleopEnabled | kFmsAttached, now_us);
  }
  EXPECT_EQ(Stamp(tracker, 40000, kTeleopEnabled), 20000u);
}

// REQ-SAFE-10
TEST(ModeTrackerTest, DetachedDriverStationShowsNoMode) {
  // Bench, RoboRIO 2026 image, no Driver Station attached: WPILib's HAL
  // reports teleop as its default control word, so every packet carried
  // kTeleop with kDsAttached clear (TODO RT-16). A detached packet must name
  // no mode, and the hardware ticks that observe the raw flags must agree
  // with the packets on the generation.
  ModeTracker tracker;
  DriverStationData detached = Sample(1000, kTeleop);
  tracker.Stamp(detached);
  EXPECT_EQ(detached.fms.flags & (kAutonomous | kTeleop | kTest), 0u)
      << "flags " << detached.fms.flags;
  tracker.Observe(kTeleop, 1005);
  tracker.Observe(0, 1010);
  EXPECT_EQ(Stamp(tracker, 1020, kTeleop), 1000u);
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
