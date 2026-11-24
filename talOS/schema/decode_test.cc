#include "talOS/schema/decode.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <vector>

#include "flatbuffers/reflection_generated.h"
#include "schema_test/shapes_generated.h"

namespace talos::schema {
namespace {

template <class T>
std::span<const std::byte> BytesOf(const T& value) {
  return std::as_bytes(std::span<const T, 1>{&value, 1});
}

TEST(Decode, NamesEveryFieldInDeclarationOrder) {
  const schema_test::Mixed mixed{
      true,
      1.5,
      schema_test::Level::TOP,
      7,
      schema_test::Inner{schema_test::Mode::LOW, 0.25f},
      0.5f};
  EXPECT_EQ(DecodeToText(Of<schema_test::Mixed>(), BytesOf(mixed)),
            "{flag: true, position_m: 1.5, level: TOP, sample_count: 7, "
            "inner: {mode: LOW, offset_m: 0.25}, speed_frac: 0.5}");
}

TEST(Decode, AnEnumValueWithNoNamePrintsItsNumber) {
  // Deliberately tests decoding an unnamed wire enum value.
  // NOLINTNEXTLINE(clang-analyzer-optin.core.EnumCastOutOfRange)
  schema_test::Inner inner{static_cast<schema_test::Mode>(9), 1.0f};
  EXPECT_EQ(DecodeToText(Of<schema_test::Inner>(), BytesOf(inner)),
            "{mode: 9, offset_m: 1}");
}

TEST(Decode, FixedArraysPrintEachElement) {
  std::vector<std::byte> bytes(sizeof(schema_test::Arrays));
  const double samples[3] = {1.0, 2.0, 3.0};
  std::memcpy(bytes.data(), samples, sizeof samples);
  const std::string text = DecodeToText(Of<schema_test::Arrays>(), bytes);
  EXPECT_NE(text.find("samples_m: [1, 2, 3]"), std::string::npos) << text;
  EXPECT_NE(text.find("modes: [OFF, OFF]"), std::string::npos) << text;
  EXPECT_NE(text.find("inners: [{mode: OFF, offset_m: 0}, {mode: OFF, "
                      "offset_m: 0}]"),
            std::string::npos)
      << text;
  EXPECT_NE(text.find("channel_id: [0, 0, 0, 0, 0]"), std::string::npos)
      << text;
}

TEST(Decode, ATargetEnvelopeDecodesItsHeaderThenTheMessage) {
  const schema_test::Inner inner{schema_test::Mode::HIGH, 2.0f};
  std::vector<std::byte> bytes(kEnvelopeHeaderBytes + sizeof inner);
  const std::uint64_t since = 1200;
  std::memcpy(bytes.data(), &since, sizeof since);
  bytes[kEnvelopeActiveOffset] = std::byte{1};
  std::memcpy(bytes.data() + kEnvelopeHeaderBytes, &inner, sizeof inner);
  EXPECT_EQ(DecodeToText(Envelope(Of<schema_test::Inner>()), bytes),
            "{mode_since_us: 1200, active: true, message: {mode: HIGH, "
            "offset_m: 2}}");
}

TEST(Decode, ALayoutWithoutSchemaBytesPrintsHex) {
  std::vector<std::byte> bytes(100);
  bytes[0] = std::byte{0xab};
  bytes[2] = std::byte{0x01};
  EXPECT_EQ(DecodeToText(Bytes(), bytes), "bytes=100 hex=ab0001");

  std::vector<std::byte> long_bytes(kMaxHexBytes + 10, std::byte{0xff});
  const std::string text = DecodeToText(Bytes(), long_bytes);
  EXPECT_EQ(text.size(), std::string{"bytes=74 hex="}.size() +
                             2 * kMaxHexBytes + std::string{"...(+10)"}.size())
      << text;
}

TEST(Decode, AWrongSizePayloadSaysSoInsteadOfReadingPastIt) {
  const std::vector<std::byte> bytes(3);
  EXPECT_EQ(DecodeToText(Of<schema_test::Inner>(), bytes),
            "<3 bytes, schema_test.Inner is 8>");
}

// ULong-backed enums print their enum label rather than falling through to raw
// decimal numbers.
TEST(Decode, ULongEnumPrintsItsLabel) {
  flatbuffers::FlatBufferBuilder builder;
  const auto enum_name = builder.CreateString("test.Flags");
  const auto low_name = builder.CreateString("LOW");
  const auto high_name = builder.CreateString("HIGH");
  reflection::EnumValBuilder ev0(builder);
  ev0.add_name(low_name);
  ev0.add_value(1);
  const auto v0 = ev0.Finish();
  reflection::EnumValBuilder ev1(builder);
  ev1.add_name(high_name);
  ev1.add_value(static_cast<std::int64_t>(std::uint64_t{1} << 63));
  const auto v1 = ev1.Finish();
  const std::vector<flatbuffers::Offset<reflection::EnumVal>> evs = {v0, v1};
  const auto values = builder.CreateVector(evs);
  reflection::TypeBuilder utb(builder);
  utb.add_base_type(reflection::BaseType::ULong);
  const auto underlying = utb.Finish();
  reflection::EnumBuilder eb(builder);
  eb.add_name(enum_name);
  eb.add_values(values);
  eb.add_underlying_type(underlying);
  const auto enum_off = eb.Finish();
  const auto enums = builder.CreateVector(&enum_off, 1);

  const auto obj_name = builder.CreateString("test.Holder");
  const auto field_name = builder.CreateString("kind");
  reflection::TypeBuilder ftb(builder);
  ftb.add_base_type(reflection::BaseType::ULong);
  ftb.add_index(0);
  const auto ftype = ftb.Finish();
  reflection::FieldBuilder fb(builder);
  fb.add_name(field_name);
  fb.add_type(ftype);
  fb.add_offset(0);
  const auto field = fb.Finish();
  const auto fields = builder.CreateVector(&field, 1);
  reflection::ObjectBuilder ob(builder);
  ob.add_name(obj_name);
  ob.add_fields(fields);
  ob.add_is_struct(true);
  ob.add_minalign(8);
  ob.add_bytesize(8);
  const auto obj = ob.Finish();
  const auto objects = builder.CreateVector(&obj, 1);
  reflection::SchemaBuilder sb(builder);
  sb.add_objects(objects);
  sb.add_enums(enums);
  reflection::FinishSchemaBuffer(builder, sb.Finish());

  Identity id;
  id.name = "test.Holder";
  id.bfbs.assign(reinterpret_cast<const char*>(builder.GetBufferPointer()),
                 builder.GetSize());

  const std::uint64_t low_val = 1;
  EXPECT_EQ(DecodeToText(id, BytesOf(low_val)), "{kind: LOW}");
  const std::uint64_t high_val = std::uint64_t{1} << 63;
  EXPECT_EQ(DecodeToText(id, BytesOf(high_val)), "{kind: HIGH}");
  const std::uint64_t unnamed_val = 42;
  EXPECT_EQ(DecodeToText(id, BytesOf(unnamed_val)), "{kind: 42}");
}

// Untrusted .bfbs with out-of-bounds indices or cyclic struct references fails
// cleanly instead of crashing.
TEST(Decode, UntrustedSchemaWithOobIndicesAndCyclesDoesNotCrash) {
  for (const auto& [base, index] :
       {std::pair{reflection::BaseType::Obj, 0},
        std::pair{reflection::BaseType::Obj, 99},
        std::pair{reflection::BaseType::ULong, 99}}) {
    flatbuffers::FlatBufferBuilder builder;
    const auto obj_name = builder.CreateString("test.Bad");
    const auto field_name = builder.CreateString("x");
    reflection::TypeBuilder tb(builder);
    tb.add_base_type(base);
    tb.add_index(index);
    const auto type = tb.Finish();
    reflection::FieldBuilder fb(builder);
    fb.add_name(field_name);
    fb.add_type(type);
    fb.add_offset(0);
    const auto field = fb.Finish();
    const auto fields = builder.CreateVector(&field, 1);
    reflection::ObjectBuilder ob(builder);
    ob.add_name(obj_name);
    ob.add_fields(fields);
    ob.add_is_struct(true);
    ob.add_minalign(8);
    ob.add_bytesize(8);
    const auto obj = ob.Finish();
    const auto objects = builder.CreateVector(&obj, 1);
    const auto enums = builder.CreateVector(
        std::vector<flatbuffers::Offset<reflection::Enum>>{});
    reflection::SchemaBuilder sb(builder);
    sb.add_objects(objects);
    sb.add_enums(enums);
    reflection::FinishSchemaBuffer(builder, sb.Finish());
    Identity id;
    id.name = "test.Bad";
    id.bfbs.assign(reinterpret_cast<const char*>(builder.GetBufferPointer()),
                   builder.GetSize());
    const std::uint64_t payload = 0;
    EXPECT_EQ(DecodeToText(id, BytesOf(payload)), "<invalid struct schema>");
  }

  // Out-of-range raw BaseType bytes (-128, 127) are rejected by VerifySchema
  // before any accessor casts them to reflection::BaseType (REQ-OPS-41).
  for (const std::int8_t bad_byte :
       {static_cast<std::int8_t>(-128), static_cast<std::int8_t>(127)}) {
    SCOPED_TRACE("raw BaseType=" + std::to_string(bad_byte));
    flatbuffers::FlatBufferBuilder builder;
    const auto obj_name = builder.CreateString("test.Bad");
    const auto field_name = builder.CreateString("x");
    const auto type_start = builder.StartTable();
    builder.AddElement<std::int8_t>(reflection::Type::VT_BASE_TYPE, bad_byte,
                                    0);
    const flatbuffers::Offset<reflection::Type> type{
        builder.EndTable(type_start)};
    reflection::FieldBuilder fb(builder);
    fb.add_name(field_name);
    fb.add_type(type);
    fb.add_offset(0);
    const auto field = fb.Finish();
    const auto fields = builder.CreateVector(&field, 1);
    reflection::ObjectBuilder ob(builder);
    ob.add_name(obj_name);
    ob.add_fields(fields);
    ob.add_is_struct(true);
    ob.add_minalign(8);
    ob.add_bytesize(8);
    const auto obj = ob.Finish();
    const auto objects = builder.CreateVector(&obj, 1);
    const auto enums = builder.CreateVector(
        std::vector<flatbuffers::Offset<reflection::Enum>>{});
    reflection::SchemaBuilder sb(builder);
    sb.add_objects(objects);
    sb.add_enums(enums);
    reflection::FinishSchemaBuffer(builder, sb.Finish());
    Identity id;
    id.name = "test.Bad";
    id.bfbs.assign(reinterpret_cast<const char*>(builder.GetBufferPointer()),
                   builder.GetSize());
    const std::uint64_t payload = 0;
    EXPECT_EQ(DecodeToText(id, BytesOf(payload)),
              "<schema bytes are not a reflection schema>");
  }
}

// Guards against exponential recursion in AppendStruct: a 22-level diamond DAG
// hits the struct-visit cap and a 2-field self-cycle hits the per-object cycle
// guard, both failing cleanly without OOM or hang (REQ-OPS-41,
// REQ-CONTRACT-20).
TEST(Decode, RejectsDiamondDagAndTwoFieldSelfCycle) {
  const std::uint64_t payload = 0;

  // 22-level diamond DAG: depth < kMaxDepth (32), no cycle, 2^22 - 1 paths.
  {
    flatbuffers::FlatBufferBuilder builder;
    constexpr int kLevels = 22;
    std::vector<flatbuffers::Offset<reflection::Object>> objs;
    objs.reserve(kLevels);
    for (int i = 0; i < kLevels; ++i) {
      const auto obj_name = builder.CreateString("dag.o" + std::to_string(i));
      std::vector<flatbuffers::Offset<reflection::Field>> fields;
      if (i + 1 == kLevels) {
        const auto field_name = builder.CreateString("leaf_m");
        reflection::TypeBuilder tb(builder);
        tb.add_base_type(reflection::BaseType::Double);
        const auto ft = tb.Finish();
        reflection::FieldBuilder fb(builder);
        fb.add_name(field_name);
        fb.add_type(ft);
        fb.add_offset(0);
        fields.push_back(fb.Finish());
      } else {
        for (int k = 0; k < 2; ++k) {
          const auto field_name = builder.CreateString(k == 0 ? "a" : "b");
          reflection::TypeBuilder tb(builder);
          tb.add_base_type(reflection::BaseType::Obj);
          tb.add_index(i + 1);
          const auto ft = tb.Finish();
          reflection::FieldBuilder fb(builder);
          fb.add_name(field_name);
          fb.add_type(ft);
          fb.add_id(static_cast<std::uint16_t>(k));
          fb.add_offset(0);
          fields.push_back(fb.Finish());
        }
      }
      const auto fvec = builder.CreateVector(fields);
      reflection::ObjectBuilder ob(builder);
      ob.add_name(obj_name);
      ob.add_fields(fvec);
      ob.add_is_struct(true);
      ob.add_minalign(8);
      ob.add_bytesize(8);
      objs.push_back(ob.Finish());
    }
    const auto objects = builder.CreateVector(objs);
    const auto enums = builder.CreateVector(
        std::vector<flatbuffers::Offset<reflection::Enum>>{});
    reflection::SchemaBuilder sb(builder);
    sb.add_objects(objects);
    sb.add_enums(enums);
    reflection::FinishSchemaBuffer(builder, sb.Finish());

    Identity id;
    id.name = "dag.o0";
    id.bfbs.assign(reinterpret_cast<const char*>(builder.GetBufferPointer()),
                   builder.GetSize());
    ASSERT_EQ(DecodeToText(id, BytesOf(payload)), "<invalid struct schema>");
  }

  // 2-field self-cycle: bad.C has two fields referencing object 0.
  {
    flatbuffers::FlatBufferBuilder builder;
    const auto obj_name = builder.CreateString("bad.C");
    std::vector<flatbuffers::Offset<reflection::Field>> fields;
    for (int k = 0; k < 2; ++k) {
      const auto field_name = builder.CreateString(k == 0 ? "a" : "b");
      reflection::TypeBuilder tb(builder);
      tb.add_base_type(reflection::BaseType::Obj);
      tb.add_index(0);
      const auto ft = tb.Finish();
      reflection::FieldBuilder fb(builder);
      fb.add_name(field_name);
      fb.add_type(ft);
      fb.add_id(static_cast<std::uint16_t>(k));
      fb.add_offset(0);
      fields.push_back(fb.Finish());
    }
    const auto fvec = builder.CreateVector(fields);
    reflection::ObjectBuilder ob(builder);
    ob.add_name(obj_name);
    ob.add_fields(fvec);
    ob.add_is_struct(true);
    ob.add_minalign(8);
    ob.add_bytesize(8);
    const auto obj = ob.Finish();
    const auto objects = builder.CreateVector(&obj, 1);
    const auto enums = builder.CreateVector(
        std::vector<flatbuffers::Offset<reflection::Enum>>{});
    reflection::SchemaBuilder sb(builder);
    sb.add_objects(objects);
    sb.add_enums(enums);
    reflection::FinishSchemaBuffer(builder, sb.Finish());

    Identity id;
    id.name = "bad.C";
    id.bfbs.assign(reinterpret_cast<const char*>(builder.GetBufferPointer()),
                   builder.GetSize());
    EXPECT_EQ(DecodeToText(id, BytesOf(payload)), "<invalid struct schema>");
  }
}

// Guards against scalar-array amplification OOM via overlapping fields or
// shallow diamond DAGs over array leaves (REQ-OPS-15, REQ-CONTRACT-20).
TEST(Decode, BoundsTotalScalarElementsAgainstArrayAmplification) {
  // 1. Two overlapping [ubyte:65535] fields at offset 0 in a 65535-byte struct.
  {
    flatbuffers::FlatBufferBuilder builder;
    const auto obj_name = builder.CreateString("bad.Amp");
    std::vector<flatbuffers::Offset<reflection::Field>> fields;
    for (int k = 0; k < 2; ++k) {
      const auto field_name = builder.CreateString(k == 0 ? "a" : "b");
      reflection::TypeBuilder tb(builder);
      tb.add_base_type(reflection::BaseType::Array);
      tb.add_element(reflection::BaseType::UByte);
      tb.add_fixed_length(65535);
      const auto ft = tb.Finish();
      reflection::FieldBuilder fb(builder);
      fb.add_name(field_name);
      fb.add_type(ft);
      fb.add_id(static_cast<std::uint16_t>(k));
      fb.add_offset(0);
      fields.push_back(fb.Finish());
    }
    const auto fvec = builder.CreateVector(fields);
    reflection::ObjectBuilder ob(builder);
    ob.add_name(obj_name);
    ob.add_fields(fvec);
    ob.add_is_struct(true);
    ob.add_minalign(1);
    ob.add_bytesize(65535);
    const auto obj = ob.Finish();
    const auto objects = builder.CreateVector(&obj, 1);
    const auto enums = builder.CreateVector(
        std::vector<flatbuffers::Offset<reflection::Enum>>{});
    reflection::SchemaBuilder sb(builder);
    sb.add_objects(objects);
    sb.add_enums(enums);
    reflection::FinishSchemaBuffer(builder, sb.Finish());

    Identity id;
    id.name = "bad.Amp";
    id.bfbs.assign(reinterpret_cast<const char*>(builder.GetBufferPointer()),
                   builder.GetSize());
    const std::vector<std::byte> payload(65535, std::byte{0});
    EXPECT_EQ(DecodeToText(id, payload), "<invalid struct schema>");
  }

  // 2. Exact boundary: 16384 scalar elements succeeds; adding one overlapping
  //    scalar (16385 total) fails cleanly.
  {
    auto build_boundary = [](bool add_extra_scalar) {
      flatbuffers::FlatBufferBuilder builder;
      const auto obj_name = builder.CreateString("bound.Arr");
      std::vector<flatbuffers::Offset<reflection::Field>> fields;
      {
        const auto field_name = builder.CreateString("a");
        reflection::TypeBuilder tb(builder);
        tb.add_base_type(reflection::BaseType::Array);
        tb.add_element(reflection::BaseType::UByte);
        tb.add_fixed_length(16384);
        const auto ft = tb.Finish();
        reflection::FieldBuilder fb(builder);
        fb.add_name(field_name);
        fb.add_type(ft);
        fb.add_id(0);
        fb.add_offset(0);
        fields.push_back(fb.Finish());
      }
      if (add_extra_scalar) {
        const auto field_name = builder.CreateString("b");
        reflection::TypeBuilder tb(builder);
        tb.add_base_type(reflection::BaseType::UByte);
        const auto ft = tb.Finish();
        reflection::FieldBuilder fb(builder);
        fb.add_name(field_name);
        fb.add_type(ft);
        fb.add_id(1);
        fb.add_offset(0);
        fields.push_back(fb.Finish());
      }
      const auto fvec = builder.CreateVector(fields);
      reflection::ObjectBuilder ob(builder);
      ob.add_name(obj_name);
      ob.add_fields(fvec);
      ob.add_is_struct(true);
      ob.add_minalign(1);
      ob.add_bytesize(16384);
      const auto obj = ob.Finish();
      const auto objects = builder.CreateVector(&obj, 1);
      const auto enums = builder.CreateVector(
          std::vector<flatbuffers::Offset<reflection::Enum>>{});
      reflection::SchemaBuilder sb(builder);
      sb.add_objects(objects);
      sb.add_enums(enums);
      reflection::FinishSchemaBuffer(builder, sb.Finish());
      Identity id;
      id.name = "bound.Arr";
      id.bfbs.assign(reinterpret_cast<const char*>(builder.GetBufferPointer()),
                     builder.GetSize());
      return id;
    };

    const std::vector<std::byte> payload(16384, std::byte{0});
    const std::string at_cap = DecodeToText(build_boundary(false), payload);
    EXPECT_NE(at_cap, "<invalid struct schema>");
    EXPECT_EQ(at_cap.rfind("{a: [0, 0, ", 0), 0U);
    EXPECT_EQ(DecodeToText(build_boundary(true), payload),
              "<invalid struct schema>");
  }

  // 3. Cumulative cap across a 10-level diamond DAG (1023 struct visits < 4096,
  //    512 leaves) over a leaf with a small [ubyte:33] array (16896 elements >
  //    16384).
  {
    flatbuffers::FlatBufferBuilder builder;
    constexpr int kLevels = 10;
    std::vector<flatbuffers::Offset<reflection::Object>> objs;
    objs.reserve(kLevels);
    for (int i = 0; i < kLevels; ++i) {
      const auto obj_name =
          builder.CreateString("ampdag.o" + std::to_string(i));
      std::vector<flatbuffers::Offset<reflection::Field>> fields;
      if (i + 1 == kLevels) {
        const auto field_name = builder.CreateString("leaf");
        reflection::TypeBuilder tb(builder);
        tb.add_base_type(reflection::BaseType::Array);
        tb.add_element(reflection::BaseType::UByte);
        tb.add_fixed_length(33);
        const auto ft = tb.Finish();
        reflection::FieldBuilder fb(builder);
        fb.add_name(field_name);
        fb.add_type(ft);
        fb.add_offset(0);
        fields.push_back(fb.Finish());
      } else {
        for (int k = 0; k < 2; ++k) {
          const auto field_name = builder.CreateString(k == 0 ? "a" : "b");
          reflection::TypeBuilder tb(builder);
          tb.add_base_type(reflection::BaseType::Obj);
          tb.add_index(i + 1);
          const auto ft = tb.Finish();
          reflection::FieldBuilder fb(builder);
          fb.add_name(field_name);
          fb.add_type(ft);
          fb.add_id(static_cast<std::uint16_t>(k));
          fb.add_offset(0);
          fields.push_back(fb.Finish());
        }
      }
      const auto fvec = builder.CreateVector(fields);
      reflection::ObjectBuilder ob(builder);
      ob.add_name(obj_name);
      ob.add_fields(fvec);
      ob.add_is_struct(true);
      ob.add_minalign(1);
      ob.add_bytesize(33);
      objs.push_back(ob.Finish());
    }
    const auto objects = builder.CreateVector(objs);
    const auto enums = builder.CreateVector(
        std::vector<flatbuffers::Offset<reflection::Enum>>{});
    reflection::SchemaBuilder sb(builder);
    sb.add_objects(objects);
    sb.add_enums(enums);
    reflection::FinishSchemaBuffer(builder, sb.Finish());

    Identity id;
    id.name = "ampdag.o0";
    id.bfbs.assign(reinterpret_cast<const char*>(builder.GetBufferPointer()),
                   builder.GetSize());
    const std::vector<std::byte> payload(33, std::byte{0});
    EXPECT_EQ(DecodeToText(id, payload), "<invalid struct schema>");
  }
}

}  // namespace
}  // namespace talos::schema
