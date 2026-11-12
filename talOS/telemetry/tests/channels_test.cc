#include "talOS/telemetry/channels.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "schema_test/shapes_generated.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/hardware/packet.h"
#include "talOS/schema/field_path.h"
#include "talOS/schema/identity.h"

namespace talos::telemetry {
namespace {

// The node reads whole shared-memory slots, so its tests (node_test.cc, which
// cover names, kinds, units, labels and values) never hand the channel model
// a short message. A reader of recorded messages hands it what a log holds:
// a scalar or a Driver Station packet that ends past the bytes it was given
// reads as absent, never past them.
TEST(TelemetryChannels, AValueEndingPastTheMessageIsNotRead) {
  const schema::Identity identity = schema::Of<schema_test::Mixed>();
  const std::vector<Channel> channels =
      FlattenTopic("/demo/state", identity.name, identity.bfbs);
  ASSERT_FALSE(channels.empty());
  const auto end = [](const Channel& channel) {
    return channel.offset + schema::ScalarSize(channel.base);
  };
  const Channel& last = *std::max_element(
      channels.begin(), channels.end(),
      [&](const Channel& a, const Channel& b) { return end(a) < end(b); });
  SCOPED_TRACE(last.name);

  const schema_test::Mixed message{
      true,
      1.0,
      schema_test::Level::TOP,
      7,
      schema_test::Inner{schema_test::Mode::HIGH, 0.5f},
      0.5f};
  const std::span<const std::byte> bytes =
      std::as_bytes(std::span{&message, 1});
  std::uint64_t raw = 0;
  ASSERT_TRUE(ReadChannel(last, bytes.first(end(last)), raw));
  EXPECT_EQ(DoubleFromRaw(raw), 0.5);
  EXPECT_FALSE(ReadChannel(last, bytes.first(end(last) - 1), raw));

  hardware::Packet packet{};
  driver_station::DriverStationData data;
  data.fms.flags = driver_station::kDsAttached | driver_station::kEnabled;
  packet.size =
      static_cast<std::uint32_t>(driver_station::Encode(data, packet.data));
  ASSERT_GT(packet.size, 0u);
  const std::span<const std::byte> packet_bytes =
      std::as_bytes(std::span{&packet, 1});
  const std::size_t packet_end = offsetof(hardware::Packet, data) + packet.size;
  std::array<std::uint64_t, kDriverStationChannelCount> values{};
  ASSERT_TRUE(ReadDriverStation(packet_bytes.first(packet_end), values));
  EXPECT_FALSE(ReadDriverStation(packet_bytes.first(packet_end - 1), values));
}

}  // namespace
}  // namespace talos::telemetry
