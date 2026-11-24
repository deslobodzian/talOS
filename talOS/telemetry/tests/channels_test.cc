#include "talOS/telemetry/channels.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <random>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "flatbuffers/flatbuffers.h"
#include "flatbuffers/reflection_generated.h"
#include "schema_test/shapes_generated.h"
#include "sim/sim_descriptor.h"
#include "sim/sim_generated.h"
#include "sim/topics.h"
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

// REQ-DEV-40: /sim/state/match reports the match to Studio through telemetry
// alone (no Studio code), so its message must flatten into channels with
// their units and the period's labels, each read at the offset the simulator
// writes. The offsets are the slot layout other readers and recorded logs
// rely on (docs/DATA_FLOW.md): a reordered or repadded field fails here, a
// size change fails sim_layout_asserts.cc, and a field without a unit suffix
// fails the build.
TEST(TelemetryChannels, MatchStateFlattensWithUnitsLabelsAndPinnedOffsets) {
  const schema::Identity identity = schema::Of<sim::MatchState>();
  const std::vector<Channel> channels =
      FlattenTopic(sim::topics::kMatchState, identity.name, identity.bfbs);

  using reflection::BaseType;
  const std::vector<std::string> period_labels = {"None", "Autonomous",
                                                  "Teleop", "Ended"};
  const struct {
    const char* name;
    const char* unit;
    std::size_t offset;
    BaseType base;
    std::uint64_t value;
  } expected[] = {
      {"sim/state/match/sample_time_ns", "ns", 0, BaseType::ULong, 101},
      {"sim/state/match/schedule_time_us", "us", 8, BaseType::ULong, 102},
      {"sim/state/match/period_remaining_us", "us", 16, BaseType::ULong, 103},
      {"sim/state/match/blue_points_count", "", 24, BaseType::UInt, 4},
      {"sim/state/match/red_points_count", "", 28, BaseType::UInt, 5},
      {"sim/state/match/blue_auto_points_count", "", 32, BaseType::UInt, 6},
      {"sim/state/match/red_auto_points_count", "", 36, BaseType::UInt, 7},
      {"sim/state/match/blue_scored_count", "", 40, BaseType::UInt, 8},
      {"sim/state/match/red_scored_count", "", 44, BaseType::UInt, 9},
      {"sim/state/match/shots_count", "", 48, BaseType::UInt, 10},
      {"sim/state/match/pieces_in_play_count", "", 52, BaseType::UShort, 11},
      {"sim/state/match/pieces_held_count", "", 54, BaseType::UShort, 12},
      {"sim/state/match/period", "", 56, BaseType::UByte, 2},
      {"sim/state/match/stage_index", "", 57, BaseType::UByte, 3},
      {"sim/state/match/blue_scoring_active", "", 58, BaseType::Bool, 1},
      {"sim/state/match/red_scoring_active", "", 59, BaseType::Bool, 0},
      {"sim/state/match/blue_inactive_first", "", 60, BaseType::Bool, 1},
      {"sim/state/match/tie_drawn", "", 61, BaseType::Bool, 1},
  };
  ASSERT_EQ(channels.size(), std::size(expected));

  const sim::MatchState message{101,
                                102,
                                103,
                                4,
                                5,
                                6,
                                7,
                                8,
                                9,
                                10,
                                11,
                                12,
                                sim::MatchPeriod::Teleop,
                                3,
                                true,
                                false,
                                true,
                                true};
  const std::span<const std::byte> bytes =
      std::as_bytes(std::span{&message, 1});
  for (std::size_t i = 0; i < channels.size(); ++i) {
    const Channel& channel = channels[i];
    SCOPED_TRACE(expected[i].name);
    EXPECT_EQ(channel.name, expected[i].name);
    EXPECT_EQ(channel.unit, expected[i].unit);
    EXPECT_EQ(channel.offset, expected[i].offset);
    EXPECT_EQ(channel.base, expected[i].base);
    EXPECT_EQ(channel.kind, KindOf(expected[i].base));
    EXPECT_EQ(channel.alert, Talos::Telemetry::AlertLevel_None);
    EXPECT_EQ(channel.labels, channel.name == "sim/state/match/period"
                                  ? period_labels
                                  : std::vector<std::string>{});
    std::uint64_t raw = 0;
    ASSERT_TRUE(ReadChannel(channel, bytes, raw));
    EXPECT_EQ(raw, expected[i].value);
  }
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

// One field of a crafted reflection struct. Stores raw int8_t bytes so hostile
// out-of-range BaseType values can be written to the FlatBuffer without UB.
struct CraftedField {
  std::int8_t base;
  std::int8_t element;
  std::int32_t index;  // object index for Obj, or Obj elements
  std::uint16_t fixed_length;

  constexpr CraftedField(reflection::BaseType b, reflection::BaseType e,
                         std::int32_t idx, std::uint16_t len)
      : base(static_cast<std::int8_t>(b)),
        element(static_cast<std::int8_t>(e)),
        index(idx),
        fixed_length(len) {}

  static constexpr CraftedField Raw(std::int8_t b, std::int8_t e,
                                    std::int32_t idx, std::uint16_t len) {
    return CraftedField{b, e, idx, len};
  }

 private:
  constexpr CraftedField(std::int8_t b, std::int8_t e, std::int32_t idx,
                         std::uint16_t len)
      : base(b), element(e), index(idx), fixed_length(len) {}
};

// A reflection schema of structs "t.o00", "t.o01", ... (so object i is at
// index i), each field at offset 0 of a struct with `bytesizes[i]` (default 1):
// offsets do not matter to the bounds, only the shape and bytesize do.
std::string CraftSchema(const std::vector<std::vector<CraftedField>>& objects,
                        const std::vector<std::int32_t>& bytesizes = {}) {
  flatbuffers::FlatBufferBuilder fbb;
  std::vector<flatbuffers::Offset<reflection::Object>> made;
  for (std::size_t i = 0; i < objects.size(); ++i) {
    std::vector<flatbuffers::Offset<reflection::Field>> fields;
    for (std::size_t k = 0; k < objects[i].size(); ++k) {
      const CraftedField& f = objects[i][k];
      const auto type_start = fbb.StartTable();
      fbb.AddElement<std::int8_t>(reflection::Type::VT_BASE_TYPE, f.base, 0);
      fbb.AddElement<std::int8_t>(reflection::Type::VT_ELEMENT, f.element, 0);
      fbb.AddElement<std::int32_t>(reflection::Type::VT_INDEX, f.index, -1);
      fbb.AddElement<std::uint16_t>(reflection::Type::VT_FIXED_LENGTH,
                                    f.fixed_length, 0);
      fbb.AddElement<std::uint32_t>(reflection::Type::VT_BASE_SIZE, 1, 4);
      fbb.AddElement<std::uint32_t>(reflection::Type::VT_ELEMENT_SIZE, 1, 0);
      const flatbuffers::Offset<reflection::Type> type{
          fbb.EndTable(type_start)};
      const std::string name = "f" + std::to_string(k) + "_m";
      fields.push_back(reflection::CreateFieldDirect(
          fbb, name.c_str(), type, static_cast<std::uint16_t>(k), 0));
    }
    char name[32];
    std::snprintf(name, sizeof name, "t.o%02zu", i);
    const std::int32_t bytesize = i < bytesizes.size() ? bytesizes[i] : 1;
    made.push_back(
        reflection::CreateObjectDirect(fbb, name, &fields, true, 1, bytesize));
  }
  std::vector<flatbuffers::Offset<reflection::Enum>> enums;
  reflection::FinishSchemaBuffer(
      fbb, reflection::CreateSchemaDirect(fbb, &made, &enums));
  return {reinterpret_cast<const char*>(fbb.GetBufferPointer()), fbb.GetSize()};
}

// REQ-OPS-41, REQ-OPS-42, REQ-CONTRACT-20: studio_review flattens the schemas a
// recorded log carries, which may be damaged or crafted. Each is refused with
// the topic named (std::invalid_argument, as for a schema that does not verify)
// instead of reading out of bounds, recursing until the stack overflows, or
// expanding to billions of channels. A real schema still flattens (the test
// above).
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
    std::vector<std::int32_t> bytesizes;
  } cases[] = {
      {"field names an object past the schema's",
       {{{BaseType::Obj, kNone, 99, 0}}},
       {}},
      {"array element names an object past the schema's",
       {{{BaseType::Array, BaseType::Obj, 99, 2}}},
       {}},
      {"struct contains itself", {{{BaseType::Obj, kNone, 0, 0}}}, {}},
      {"structs nest 18 deep", deep, {}},
      {"arrays of arrays expand to 65535 x 65535 channels",
       {{{BaseType::Array, BaseType::Obj, 1, 65535}},
        {{BaseType::Array, BaseType::Double, -1, 65535}}},
       {}},
      {"arrays of fieldless structs expand to 65535 x 65535 instances",
       {{{BaseType::Array, BaseType::Obj, 1, 65535}},
        {{BaseType::Array, BaseType::Obj, 2, 65535}},
        {}},
       {}},
      {"array element has negative out-of-range BaseType (-128)",
       {{CraftedField::Raw(static_cast<std::int8_t>(BaseType::Array), -128, -1,
                           2)}},
       {}},
      {"array element has positive out-of-range BaseType (127)",
       {{CraftedField::Raw(static_cast<std::int8_t>(BaseType::Array), 127, -1,
                           2)}},
       {}},
      {"array element is non-scalar sentinel (MaxBaseType)",
       {{{BaseType::Array, BaseType::MaxBaseType, -1, 2}}},
       {}},
      {"array element is non-scalar non-Obj (String)",
       {{{BaseType::Array, BaseType::String, -1, 2}}},
       {}},
      {"array element is BaseType::None",
       {{{BaseType::Array, BaseType::None, -1, 2}}},
       {}},
      {"array of Obj with negative bytesize",
       {{{BaseType::Array, BaseType::Obj, 1, 2}},
        {{BaseType::Double, kNone, -1, 0}}},
       {8, -1}},
      {"array of Obj with INT32_MIN bytesize",
       {{{BaseType::Array, BaseType::Obj, 1, 2}},
        {{BaseType::Double, kNone, -1, 0}}},
       {8, std::numeric_limits<std::int32_t>::min()}},
      {"array of Obj with zero bytesize",
       {{{BaseType::Array, BaseType::Obj, 1, 2}},
        {{BaseType::Double, kNone, -1, 0}}},
       {8, 0}},
      {"field has non-scalar BaseType (String)",
       {{{BaseType::String, kNone, -1, 0}}},
       {}},
      {"field has out-of-range BaseType (127)",
       {{CraftedField::Raw(127, static_cast<std::int8_t>(kNone), -1, 0)}},
       {}},
  };
  for (const auto& c : cases) {
    SCOPED_TRACE(c.name);
    const std::string bfbs = CraftSchema(c.objects, c.bytesizes);
    try {
      FlattenTopic("/crafted/state", "t.o00", bfbs);
      ADD_FAILURE() << "FlattenTopic flattened";
    } catch (const std::invalid_argument& error) {
      EXPECT_NE(std::string{error.what()}.find("/crafted/state"),
                std::string::npos)
          << error.what();
    }
    try {
      EnvelopeChannels("/crafted/target/teleop", "t.o00", bfbs);
      ADD_FAILURE() << "EnvelopeChannels flattened";
    } catch (const std::invalid_argument& error) {
      EXPECT_NE(std::string{error.what()}.find("/crafted/target/teleop"),
                std::string::npos)
          << error.what();
    }
  }

  // Non-struct table, zero bytesize, and out-of-bounds field offset are also
  // rejected naming the topic.
  for (const auto& [is_struct, bytesize, offset] :
       {std::tuple{false, 8, std::uint16_t{0}},
        std::tuple{true, 0, std::uint16_t{0}},
        std::tuple{true, 4, std::uint16_t{4}}}) {
    flatbuffers::FlatBufferBuilder fbb;
    const auto t_start = fbb.StartTable();
    fbb.AddElement<std::int8_t>(
        reflection::Type::VT_BASE_TYPE,
        static_cast<std::int8_t>(reflection::BaseType::Double), 0);
    const flatbuffers::Offset<reflection::Type> ftype{fbb.EndTable(t_start)};
    const auto fn = fbb.CreateString("x_m");
    const auto f = reflection::CreateField(fbb, fn, ftype, 0, offset);
    std::vector<flatbuffers::Offset<reflection::Field>> fields{f};
    const auto obj = reflection::CreateObjectDirect(fbb, "t.bad", &fields,
                                                    is_struct, 1, bytesize);
    std::vector<flatbuffers::Offset<reflection::Object>> objs{obj};
    std::vector<flatbuffers::Offset<reflection::Enum>> enums;
    reflection::FinishSchemaBuffer(
        fbb, reflection::CreateSchemaDirect(fbb, &objs, &enums));
    const std::string bfbs{
        reinterpret_cast<const char*>(fbb.GetBufferPointer()), fbb.GetSize()};
    EXPECT_THROW(FlattenTopic("/crafted/status", "t.bad", bfbs),
                 std::invalid_argument);
  }
}

// EnumLabels handles schemas with omitted/null enum vectors safely, and
// deterministic seeded mutation fuzzing over FlattenTopic/EnvelopeChannels/
// ReadChannel/ReadDriverStation never crashes or triggers ASan/UBSan.
TEST(TelemetryChannels, EnumLabelsNullSafeAndSeededMutationFuzz) {
  {
    flatbuffers::FlatBufferBuilder fbb;
    const auto start = fbb.StartTable();
    const flatbuffers::Offset<reflection::Schema> s{fbb.EndTable(start)};
    reflection::FinishSchemaBuffer(fbb, s);
    const auto* raw_schema = reflection::GetSchema(fbb.GetBufferPointer());
    EXPECT_TRUE(EnumLabels(*raw_schema, 0).empty());
    EXPECT_TRUE(EnumLabels(*raw_schema, -1).empty());
  }

  const schema::Identity mixed = schema::Of<schema_test::Mixed>();
  const schema::Identity arrays = schema::Of<schema_test::Arrays>();
  const std::array<std::string_view, 2> seeds = {mixed.bfbs, arrays.bfbs};
  std::mt19937_64 rng(0x4348414e4e454cULL);

  for (int iter = 0; iter < 1500; ++iter) {
    const std::string_view seed = seeds[static_cast<std::size_t>(iter) % 2];
    std::string bfbs(seed);
    const std::uint64_t mode = rng() % 4;
    if (mode == 0 && bfbs.size() > 8) {
      bfbs.resize(1 + (rng() % bfbs.size()));
    } else if (mode == 1 && !bfbs.empty()) {
      for (std::size_t k = 0; k < 3; ++k) {
        char& byte = bfbs[rng() % bfbs.size()];
        byte = static_cast<char>(static_cast<unsigned char>(byte) ^
                                 (1u << (rng() % 8)));
      }
    } else if (mode == 2) {
      bfbs.resize(rng() % 128);
      for (char& c : bfbs) c = static_cast<char>(rng() & 0xffu);
    }

    std::vector<std::byte> payload(rng() % 128);
    for (std::byte& b : payload) b = static_cast<std::byte>(rng() & 0xffu);

    for (std::string_view topic : {"/demo/status", "/demo/state"}) {
      try {
        FlattenOptions opts;
        opts.truncate = (iter & 1) != 0;
        const auto channels =
            FlattenTopic(topic, "schema_test.Mixed", bfbs, opts);
        std::uint64_t raw = 0;
        for (const auto& ch : channels) {
          (void)ReadChannel(ch, payload, raw);
        }
      } catch (const std::invalid_argument& e) {
        // Malformed fuzzed .bfbs schemas are expected to throw
        // invalid_argument.
        (void)e;
      }
      try {
        const auto channels =
            EnvelopeChannels(topic, "schema_test.Mixed", bfbs);
        std::uint64_t raw = 0;
        for (const auto& ch : channels) {
          (void)ReadChannel(ch, payload, raw);
        }
      } catch (const std::invalid_argument& e) {
        // Malformed fuzzed .bfbs schemas are expected to throw
        // invalid_argument.
        (void)e;
      }
    }
    std::array<std::uint64_t, kDriverStationChannelCount> ds{};
    (void)ReadDriverStation(payload, ds);
  }
}

}  // namespace
}  // namespace talos::telemetry
