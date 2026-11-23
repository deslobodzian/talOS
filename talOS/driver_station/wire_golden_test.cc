// The Driver Station packet's encoding, pinned to the hardware protocol
// version it travels under (docs/DATA_FLOW.md, "UDP frame and payload
// sizes"); talOS/hardware/wire_golden_test.cc pins the State and Command
// the same way, and says why. Every value is exactly representable, so the
// bytes are the same on every host.

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>
#include <string>

#include "talOS/driver_station/driver_station.h"
#include "talOS/protocol/types.h"

namespace talos::driver_station {
namespace {

constexpr std::uint16_t kGoldenVersion = 12;
constexpr std::size_t kPacketBytes = 678;
constexpr std::uint64_t kPacketHash = 0xd7173f183d6a229b;

std::uint64_t Fnv1a64(std::span<const std::uint8_t> bytes) {
  std::uint64_t hash = 0xcbf29ce484222325ull;
  for (const std::uint8_t byte : bytes) {
    hash ^= byte;
    hash *= 0x100000001b3ull;
  }
  return hash;
}

// Every field set, every joystick connected with every axis, POV and name
// byte in use.
DriverStationData FullPacket() {
  DriverStationData ds;
  ds.sample_time_us = 0x0102030405060708ull;
  ds.mode_since_us = 0x1112131415161718ull;
  ds.boot_id = 0x2122232425262728ull;
  ds.fms.flags = kDsAttached | kFmsAttached | kEnabled | kTeleop;
  ds.fms.alliance = Alliance::kBlue;
  ds.fms.station = 3;
  ds.fms.match_type = MatchType::kElimination;
  ds.fms.match_number = 0x0A0B;
  ds.fms.replay_number = 2;
  ds.fms.match_time_s = 97.5f;
  std::memset(ds.fms.game_specific_message, 'g', kMaxGameMessageLength - 1);
  std::memset(ds.fms.event_name, 'e', kMaxEventNameLength - 1);
  for (std::size_t j = 0; j < kMaxJoysticks; ++j) {
    auto& stick = ds.joysticks[j];
    stick.connected = true;
    stick.axis_count = kMaxAxes;
    stick.button_count = kMaxButtons;
    stick.pov_count = kMaxPovs;
    stick.buttons = 0x80000001u >> j;
    for (std::size_t a = 0; a < kMaxAxes; ++a) {
      stick.axes[a] = -1.0f + static_cast<float>(a + j) / 8.0f;
    }
    for (std::size_t p = 0; p < kMaxPovs; ++p) {
      stick.povs[p] = static_cast<std::int16_t>(p == 0 ? -1 : 45 * (p + j));
    }
    std::memset(stick.name, static_cast<int>('a' + j), kMaxNameLength - 1);
  }
  return ds;
}

// Guards: the Driver Station packet bytes of protocol kGoldenVersion. A
// failure here is a wire change: bump kProtocolVersion (docs/DATA_FLOW.md)
// and regenerate the constants above from the values this prints.
// REQ-OPS-51
TEST(WireGolden, DriverStationBytesBelongToTheirProtocolVersion) {
  ASSERT_EQ(protocol::kProtocolVersion, kGoldenVersion)
      << "kProtocolVersion changed: regenerate this file's constants for "
         "protocol "
      << protocol::kProtocolVersion;
  std::array<std::uint8_t, kMaxEncodedSize> bytes{};
  const std::size_t size = Encode(FullPacket(), bytes);
  const std::uint64_t hash = Fnv1a64(std::span{bytes}.first(size));
  char text[32];
  std::snprintf(text, sizeof text, "0x%016llx",
                static_cast<unsigned long long>(hash));
  EXPECT_TRUE(size == kPacketBytes && hash == kPacketHash)
      << "the encoding changed: bump kProtocolVersion and regenerate this "
         "constant (now "
      << size << " bytes, hash " << text << ")";
}

}  // namespace
}  // namespace talos::driver_station
