#include "talOS/telemetry/channels.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "flatbuffers/flatbuffers.h"
#include "flatbuffers/reflection_generated.h"
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

// REQ-OPS-41: studio_review reads a recorded target lane through these
// channels. Each reads its own field of the whole envelope: the header's
// generation and active flag, and every inner field at the header's offset
// past where the bare struct has it (a channel read at the bare offset would
// show the header's bytes as the message).
TEST(TelemetryChannels, EnvelopeChannelsReadTheHeaderAndTheMessageBehindIt) {
  const schema::Identity identity = schema::Of<schema_test::Mixed>();
  const schema_test::Mixed message{
      true,
      2.5,
      schema_test::Level::TOP,
      7,
      schema_test::Inner{schema_test::Mode::HIGH, 0.25f},
      -1.5f};
  struct Envelope {
    std::uint64_t mode_since_us;
    std::uint8_t active;
    std::array<std::uint8_t, 7> padding;
    schema_test::Mixed message;
  };
  static_assert(offsetof(Envelope, active) == schema::kEnvelopeActiveOffset);
  static_assert(offsetof(Envelope, message) == schema::kEnvelopeHeaderBytes);
  const Envelope envelope{123'456, 1, {}, message};
  const std::span<const std::byte> bytes =
      std::as_bytes(std::span{&envelope, 1});

  const std::vector<Channel> inner =
      FlattenTopic("/demo/target/teleop/message", identity.name, identity.bfbs);
  const std::vector<Channel> channels =
      EnvelopeChannels("/demo/target/teleop", identity.name, identity.bfbs);
  ASSERT_EQ(channels.size(), inner.size() + 2);
  std::uint64_t raw = 0;
  ASSERT_TRUE(ReadChannel(channels[0], bytes, raw));
  EXPECT_EQ(raw, 123'456u) << channels[0].name;
  ASSERT_TRUE(ReadChannel(channels[1], bytes, raw));
  EXPECT_EQ(raw, 1u) << channels[1].name;
  const std::span<const std::byte> bare = std::as_bytes(std::span{&message, 1});
  for (std::size_t i = 0; i < inner.size(); ++i) {
    SCOPED_TRACE(inner[i].name);
    const Channel& channel = channels[i + 2];
    EXPECT_EQ(channel.name, inner[i].name);
    std::uint64_t expected = 0;
    ASSERT_TRUE(ReadChannel(inner[i], bare, expected));
    ASSERT_TRUE(ReadChannel(channel, bytes, raw));
    EXPECT_EQ(raw, expected);
  }
}

// One field of a crafted reflection struct.
struct CraftedField {
  reflection::BaseType base;
  reflection::BaseType element;
  std::int32_t index;  // object index for Obj, or Obj elements
  std::uint16_t fixed_length;
};

// A reflection schema of structs "t.o00", "t.o01", ... (so object i is at
// index i), each field at offset 0 of a 1-byte struct: offsets do not matter
// to the bounds, only the shape does.
std::string CraftSchema(const std::vector<std::vector<CraftedField>>& objects) {
  flatbuffers::FlatBufferBuilder fbb;
  std::vector<flatbuffers::Offset<reflection::Object>> made;
  for (std::size_t i = 0; i < objects.size(); ++i) {
    std::vector<flatbuffers::Offset<reflection::Field>> fields;
    for (std::size_t k = 0; k < objects[i].size(); ++k) {
      const CraftedField& f = objects[i][k];
      const auto type = reflection::CreateType(fbb, f.base, f.element, f.index,
                                               f.fixed_length, 1, 1);
      const std::string name = "f" + std::to_string(k) + "_m";
      fields.push_back(reflection::CreateFieldDirect(
          fbb, name.c_str(), type, static_cast<std::uint16_t>(k), 0));
    }
    char name[8];
    std::snprintf(name, sizeof name, "t.o%02zu", i);
    made.push_back(
        reflection::CreateObjectDirect(fbb, name, &fields, true, 1, 1));
  }
  std::vector<flatbuffers::Offset<reflection::Enum>> enums;
  reflection::FinishSchemaBuffer(
      fbb, reflection::CreateSchemaDirect(fbb, &made, &enums));
  return {reinterpret_cast<const char*>(fbb.GetBufferPointer()), fbb.GetSize()};
}

// REQ-OPS-41: studio_review flattens the schemas a recorded log carries,
// which may be damaged or crafted. Each is refused with the topic named
// (std::invalid_argument, as for a schema that does not verify) instead of
// reading out of bounds, recursing until the stack overflows, or expanding
// to billions of channels. A real schema still flattens (the test above).
TEST(TelemetryChannels, HostileSchemaIsRefusedNamingTheTopic) {
  using reflection::BaseType;
  constexpr BaseType kNone = BaseType::None;
  std::vector<std::vector<CraftedField>> deep;
  deep.reserve(19);
  for (std::int32_t i = 0; i < 18; ++i) {
    deep.push_back({{BaseType::Obj, kNone, i + 1, 0}});
  }
  deep.push_back({{BaseType::Double, kNone, -1, 0}});
  const struct {
    const char* name;
    std::vector<std::vector<CraftedField>> objects;
  } cases[] = {
      {"field names an object past the schema's",
       {{{BaseType::Obj, kNone, 99, 0}}}},
      {"array element names an object past the schema's",
       {{{BaseType::Array, BaseType::Obj, 99, 2}}}},
      {"struct contains itself", {{{BaseType::Obj, kNone, 0, 0}}}},
      {"structs nest 18 deep", deep},
      {"arrays of arrays expand to 65535 x 65535 channels",
       {{{BaseType::Array, BaseType::Obj, 1, 65535}},
        {{BaseType::Array, BaseType::Double, -1, 65535}}}},
      {"arrays of fieldless structs expand to 65535 x 65535 instances",
       {{{BaseType::Array, BaseType::Obj, 1, 65535}},
        {{BaseType::Array, BaseType::Obj, 2, 65535}},
        {}}},
  };
  for (const auto& c : cases) {
    SCOPED_TRACE(c.name);
    const std::string bfbs = CraftSchema(c.objects);
    try {
      FlattenTopic("/crafted/state", "t.o00", bfbs);
      ADD_FAILURE() << "flattened";
    } catch (const std::invalid_argument& error) {
      EXPECT_NE(std::string{error.what()}.find("/crafted/state"),
                std::string::npos)
          << error.what();
    }
  }
}

}  // namespace
}  // namespace talos::telemetry
