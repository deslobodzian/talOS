#include "talOS/schema/field_path.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "flatbuffers/reflection.h"
#include "flatbuffers/reflection_generated.h"
#include "schema_test/shapes_generated.h"
#include "talOS/schema/identity.h"

namespace talos::schema {
namespace {

// REQ-OPS-22
// Reading a resolved field, including a nested one, keeps a narrow signed
// integer's sign, an unsigned one's full range and a float's value, whatever
// the caller reads it as.
TEST(FieldPath, ReadsResolvedFieldsAtTheirWidthAndSign) {
  const Identity identity = Of<schema_test::Mixed>();
  const reflection::Schema* schema = VerifySchema(identity.bfbs);
  ASSERT_NE(schema, nullptr);
  const reflection::Object* mixed =
      schema->objects()->LookupByKey(identity.name.c_str());
  ASSERT_NE(mixed, nullptr);

  const schema_test::Mixed message{
      true,
      -2.5,
      schema_test::Level::BOTTOM,
      std::numeric_limits<std::uint32_t>::max(),
      schema_test::Inner{schema_test::Mode::HIGH, 0.25f},
      0.5f};
  std::array<std::byte, sizeof message> bytes{};
  std::memcpy(bytes.data(), &message, sizeof message);
  const auto at = [&](const char* path) {
    const ResolvedField field = ResolveField(*schema, *mixed, path, "test");
    return std::pair{field.base_type, bytes.data() + field.offset};
  };

  const auto [level_type, level] = at("level");
  EXPECT_EQ(ReadInt64(level_type, level), -1);
  EXPECT_EQ(ReadDouble(level_type, level), -1.0);
  EXPECT_TRUE(ReadBool(level_type, level));

  const auto [count_type, count] = at("sample_count");
  EXPECT_EQ(ReadInt64(count_type, count), 4294967295);
  EXPECT_EQ(ReadDouble(count_type, count), 4294967295.0);

  const auto [mode_type, mode] = at("inner/mode");
  EXPECT_EQ(ReadInt64(mode_type, mode),
            static_cast<std::int64_t>(schema_test::Mode::HIGH));

  const auto [offset_type, offset] = at("inner/offset_m");
  EXPECT_EQ(ReadDouble(offset_type, offset), 0.25);

  const auto [position_type, position] = at("position_m");
  EXPECT_EQ(ReadDouble(position_type, position), -2.5);

  const auto [flag_type, flag] = at("flag");
  EXPECT_TRUE(ReadBool(flag_type, flag));
  EXPECT_EQ(ReadInt64(flag_type, flag), 1);
}

// REQ-OPS-22: integer FieldValues encode directly without double rounding,
// preserving precision above 2^53 and accepting INT64_MAX.
TEST(FieldPath, EncodeFieldPreservesFullInt64Precision) {
  std::array<std::byte, 8> bytes{};
  const ResolvedField long_field{
      .offset = 0,
      .base_type = reflection::BaseType::Long,
      .size = 8,
      .enum_def = nullptr,
      .leaf_name = "big",
      .enclosing_name = "Test",
  };
  const std::int64_t above_double_exact = (std::int64_t{1} << 53) + 1;
  for (const std::int64_t value :
       {above_double_exact, std::numeric_limits<std::int64_t>::max(),
        std::numeric_limits<std::int64_t>::min()}) {
    EncodeField(bytes.data(), long_field, FieldValue{value}, "test");
    EXPECT_EQ(ReadInt64(reflection::BaseType::Long, bytes.data()), value);
  }

  const ResolvedField ulong_field{
      .offset = 0,
      .base_type = reflection::BaseType::ULong,
      .size = 8,
      .enum_def = nullptr,
      .leaf_name = "ubig",
      .enclosing_name = "Test",
  };
  for (const std::int64_t value :
       {above_double_exact, std::numeric_limits<std::int64_t>::max()}) {
    EncodeField(bytes.data(), ulong_field, FieldValue{value}, "test");
    EXPECT_EQ(ReadInt64(reflection::BaseType::ULong, bytes.data()), value);
  }
}

// ScalarPaths lists fields in declaration order (by id()), matching
// field_path.h and DecodeToText.
TEST(FieldPath, ScalarPathsFollowsDeclarationOrder) {
  const Identity identity = Of<schema_test::Mixed>();
  const reflection::Schema* schema = VerifySchema(identity.bfbs);
  ASSERT_NE(schema, nullptr);
  const reflection::Object* mixed =
      schema->objects()->LookupByKey(identity.name.c_str());
  ASSERT_NE(mixed, nullptr);

  EXPECT_EQ(
      ScalarPaths(*schema, *mixed),
      (std::vector<std::string>{"flag", "position_m", "level", "sample_count",
                                "inner/mode", "inner/offset_m", "speed_frac"}));
}

// Trailing or leading slashes are malformed paths, not silently ignored.
TEST(FieldPath, ResolveFieldRejectsTrailingAndLeadingSlashes) {
  const Identity identity = Of<schema_test::Mixed>();
  const reflection::Schema* schema = VerifySchema(identity.bfbs);
  ASSERT_NE(schema, nullptr);
  const reflection::Object* mixed =
      schema->objects()->LookupByKey(identity.name.c_str());
  ASSERT_NE(mixed, nullptr);

  for (const char* bad : {"position_m/", "inner/offset_m/", "/position_m",
                          "inner//offset_m", "/"}) {
    EXPECT_THROW(ResolveField(*schema, *mixed, bad, "test"),
                 std::invalid_argument)
        << bad;
  }
}

// Untrusted .bfbs schemas with missing tables, out-of-bounds indices, or
// cyclic struct references must fail cleanly instead of crashing.
TEST(FieldPath, GuardsUntrustedSchemaPointersBoundsAndDepth) {
  // Cyclic struct: object 0 has a field of type Obj index 0.
  {
    flatbuffers::FlatBufferBuilder builder;
    const auto obj_name = builder.CreateString("bad.Cycle");
    const auto field_name = builder.CreateString("self");
    reflection::TypeBuilder tb(builder);
    tb.add_base_type(reflection::BaseType::Obj);
    tb.add_index(0);
    const auto type = tb.Finish();
    reflection::FieldBuilder fb(builder);
    fb.add_name(field_name);
    fb.add_type(type);
    const auto field = fb.Finish();
    const auto fields = builder.CreateVector(&field, 1);
    reflection::ObjectBuilder ob(builder);
    ob.add_name(obj_name);
    ob.add_fields(fields);
    ob.add_is_struct(true);
    ob.add_bytesize(8);
    const auto obj = ob.Finish();
    const auto objects = builder.CreateVector(&obj, 1);
    const auto enums = builder.CreateVector(
        std::vector<flatbuffers::Offset<reflection::Enum>>{});
    reflection::SchemaBuilder sb(builder);
    sb.add_objects(objects);
    sb.add_enums(enums);
    reflection::FinishSchemaBuffer(builder, sb.Finish());

    const auto* schema =
        VerifySchema({reinterpret_cast<const char*>(builder.GetBufferPointer()),
                      builder.GetSize()});
    ASSERT_NE(schema, nullptr);
    EXPECT_TRUE(ScalarPaths(*schema, *schema->objects()->Get(0)).empty());
  }

  // Out-of-bounds Obj index.
  {
    flatbuffers::FlatBufferBuilder builder;
    const auto obj_name = builder.CreateString("bad.Oob");
    const auto field_name = builder.CreateString("sub");
    reflection::TypeBuilder tb(builder);
    tb.add_base_type(reflection::BaseType::Obj);
    tb.add_index(99);
    const auto type = tb.Finish();
    reflection::FieldBuilder fb(builder);
    fb.add_name(field_name);
    fb.add_type(type);
    const auto field = fb.Finish();
    const auto fields = builder.CreateVector(&field, 1);
    reflection::ObjectBuilder ob(builder);
    ob.add_name(obj_name);
    ob.add_fields(fields);
    ob.add_is_struct(true);
    ob.add_bytesize(8);
    const auto obj = ob.Finish();
    const auto objects = builder.CreateVector(&obj, 1);
    const auto enums = builder.CreateVector(
        std::vector<flatbuffers::Offset<reflection::Enum>>{});
    reflection::SchemaBuilder sb(builder);
    sb.add_objects(objects);
    sb.add_enums(enums);
    reflection::FinishSchemaBuffer(builder, sb.Finish());

    const auto* schema =
        VerifySchema({reinterpret_cast<const char*>(builder.GetBufferPointer()),
                      builder.GetSize()});
    ASSERT_NE(schema, nullptr);
    const auto& root = *schema->objects()->Get(0);
    EXPECT_THROW(ResolveField(*schema, root, "sub/x", "test"),
                 std::invalid_argument);
    EXPECT_TRUE(ScalarPaths(*schema, root).empty());
  }

  // Out-of-range raw BaseType bytes (-128, 127) in field base_type, field
  // element, enum underlying_type, enum value union_type, root_table, and
  // service call request/response are rejected by VerifySchema before any
  // accessor casts them to reflection::BaseType.
  auto make_raw_type = [](flatbuffers::FlatBufferBuilder& builder,
                          std::int8_t base, std::int8_t element) {
    const auto start = builder.StartTable();
    builder.AddElement<std::int8_t>(reflection::Type::VT_BASE_TYPE, base, 0);
    builder.AddElement<std::int8_t>(reflection::Type::VT_ELEMENT, element, 0);
    return flatbuffers::Offset<reflection::Type>{builder.EndTable(start)};
  };
  auto make_single_field_object = [&](flatbuffers::FlatBufferBuilder& builder,
                                      const char* obj_name_str,
                                      std::int8_t base, std::int8_t element) {
    const auto obj_name = builder.CreateString(obj_name_str);
    const auto field_name = builder.CreateString("x");
    const auto type = make_raw_type(builder, base, element);
    reflection::FieldBuilder fb(builder);
    fb.add_name(field_name);
    fb.add_type(type);
    const auto field = fb.Finish();
    const auto fields = builder.CreateVector(&field, 1);
    reflection::ObjectBuilder ob(builder);
    ob.add_name(obj_name);
    ob.add_fields(fields);
    ob.add_is_struct(true);
    ob.add_bytesize(8);
    return ob.Finish();
  };
  for (const std::int8_t bad_byte :
       {static_cast<std::int8_t>(-128), static_cast<std::int8_t>(127)}) {
    for (const bool in_element : {false, true}) {
      SCOPED_TRACE("field byte=" + std::to_string(bad_byte) +
                   " in_element=" + std::to_string(in_element));
      flatbuffers::FlatBufferBuilder builder;
      const auto obj = make_single_field_object(
          builder, "bad.Type",
          in_element ? static_cast<std::int8_t>(reflection::BaseType::Array)
                     : bad_byte,
          in_element ? bad_byte : std::int8_t{0});
      const auto objects = builder.CreateVector(&obj, 1);
      const auto enums = builder.CreateVector(
          std::vector<flatbuffers::Offset<reflection::Enum>>{});
      reflection::SchemaBuilder sb(builder);
      sb.add_objects(objects);
      sb.add_enums(enums);
      reflection::FinishSchemaBuffer(builder, sb.Finish());
      EXPECT_EQ(VerifySchema(
                    {reinterpret_cast<const char*>(builder.GetBufferPointer()),
                     builder.GetSize()}),
                nullptr);
    }
    for (const bool in_union_type : {false, true}) {
      SCOPED_TRACE("enum byte=" + std::to_string(bad_byte) +
                   " in_union_type=" + std::to_string(in_union_type));
      flatbuffers::FlatBufferBuilder builder;
      const auto enum_name = builder.CreateString("bad.Enum");
      const auto val_name = builder.CreateString("V");
      const auto union_type =
          make_raw_type(builder, in_union_type ? bad_byte : std::int8_t{0}, 0);
      reflection::EnumValBuilder vb(builder);
      vb.add_name(val_name);
      vb.add_value(0);
      if (in_union_type) vb.add_union_type(union_type);
      const auto val = vb.Finish();
      const auto vals = builder.CreateVector(&val, 1);
      const auto underlying = make_raw_type(
          builder,
          in_union_type ? static_cast<std::int8_t>(reflection::BaseType::UByte)
                        : bad_byte,
          0);
      reflection::EnumBuilder eb(builder);
      eb.add_name(enum_name);
      eb.add_values(vals);
      eb.add_underlying_type(underlying);
      const auto e = eb.Finish();
      const auto objects = builder.CreateVector(
          std::vector<flatbuffers::Offset<reflection::Object>>{});
      const auto enums = builder.CreateVector(&e, 1);
      reflection::SchemaBuilder sb(builder);
      sb.add_objects(objects);
      sb.add_enums(enums);
      reflection::FinishSchemaBuffer(builder, sb.Finish());
      EXPECT_EQ(VerifySchema(
                    {reinterpret_cast<const char*>(builder.GetBufferPointer()),
                     builder.GetSize()}),
                nullptr);
    }
    {
      SCOPED_TRACE("root_table byte=" + std::to_string(bad_byte));
      flatbuffers::FlatBufferBuilder builder;
      const auto root =
          make_single_field_object(builder, "bad.Root", bad_byte, 0);
      const auto objects = builder.CreateVector(
          std::vector<flatbuffers::Offset<reflection::Object>>{});
      const auto enums = builder.CreateVector(
          std::vector<flatbuffers::Offset<reflection::Enum>>{});
      reflection::SchemaBuilder sb(builder);
      sb.add_objects(objects);
      sb.add_enums(enums);
      sb.add_root_table(root);
      reflection::FinishSchemaBuffer(builder, sb.Finish());
      EXPECT_EQ(VerifySchema(
                    {reinterpret_cast<const char*>(builder.GetBufferPointer()),
                     builder.GetSize()}),
                nullptr);
    }
    for (const bool in_response : {false, true}) {
      SCOPED_TRACE("service byte=" + std::to_string(bad_byte) +
                   " in_response=" + std::to_string(in_response));
      flatbuffers::FlatBufferBuilder builder;
      const auto good_obj = make_single_field_object(
          builder, "ok.Obj",
          static_cast<std::int8_t>(reflection::BaseType::Int), 0);
      const auto bad_obj =
          make_single_field_object(builder, "bad.Obj", bad_byte, 0);
      const auto call_name = builder.CreateString("Call");
      reflection::RPCCallBuilder cb(builder);
      cb.add_name(call_name);
      cb.add_request(in_response ? good_obj : bad_obj);
      cb.add_response(in_response ? bad_obj : good_obj);
      const auto call = cb.Finish();
      const auto calls = builder.CreateVector(&call, 1);
      const auto svc_name = builder.CreateString("bad.Svc");
      reflection::ServiceBuilder svb(builder);
      svb.add_name(svc_name);
      svb.add_calls(calls);
      const auto svc = svb.Finish();
      const auto services = builder.CreateVector(&svc, 1);
      const auto objects = builder.CreateVector(
          std::vector<flatbuffers::Offset<reflection::Object>>{});
      const auto enums = builder.CreateVector(
          std::vector<flatbuffers::Offset<reflection::Enum>>{});
      reflection::SchemaBuilder sb(builder);
      sb.add_objects(objects);
      sb.add_enums(enums);
      sb.add_services(services);
      reflection::FinishSchemaBuffer(builder, sb.Finish());
      EXPECT_EQ(VerifySchema(
                    {reinterpret_cast<const char*>(builder.GetBufferPointer()),
                     builder.GetSize()}),
                nullptr);
    }
  }
}

// VerifySchema rejects unaligned buffers without UBSan alignment faults,
// ResolveField rejects non-scalar BaseTypes and out-of-bounds offsets,
// EncodeField/EncodeNumber support UType and reject non-integer types, and
// ScalarPaths bounds cycles and diamond DAGs without exponential blowup.
TEST(FieldPath, HardensAlignmentBoundsTypesAndDagExpansion) {
  // 1. Unaligned buffer passed to VerifySchema returns nullptr cleanly.
  const Identity mixed_id = Of<schema_test::Mixed>();
  std::vector<char> unaligned(mixed_id.bfbs.size() + 4);
  std::memcpy(unaligned.data() + 1, mixed_id.bfbs.data(), mixed_id.bfbs.size());
  EXPECT_EQ(VerifySchema({unaligned.data() + 1, mixed_id.bfbs.size()}),
            nullptr);

  // 2. Non-scalar BaseTypes (String, Union, Vector64) and out-of-bounds offsets
  // are rejected by ResolveField.
  for (const auto& [base, offset, bytesize, is_struct] :
       {std::tuple{reflection::BaseType::String, std::uint16_t{0}, 8, true},
        std::tuple{reflection::BaseType::Union, std::uint16_t{0}, 8, true},
        std::tuple{reflection::BaseType::Vector64, std::uint16_t{0}, 8, true},
        std::tuple{reflection::BaseType::Double, std::uint16_t{4}, 8, true},
        std::tuple{reflection::BaseType::Double, std::uint16_t{0}, 0, true},
        std::tuple{reflection::BaseType::Double, std::uint16_t{0}, 8, false}}) {
    flatbuffers::FlatBufferBuilder fbb;
    const auto ons = fbb.CreateString("bad.Obj");
    const auto fns = fbb.CreateString("x");
    reflection::TypeBuilder tb(fbb);
    tb.add_base_type(base);
    const auto ft = tb.Finish();
    reflection::FieldBuilder fb(fbb);
    fb.add_name(fns);
    fb.add_type(ft);
    fb.add_offset(offset);
    const auto f = fb.Finish();
    const auto fvec = fbb.CreateVector(&f, 1);
    reflection::ObjectBuilder ob(fbb);
    ob.add_name(ons);
    ob.add_fields(fvec);
    ob.add_is_struct(is_struct);
    ob.add_bytesize(bytesize);
    const auto o = ob.Finish();
    const auto ovec = fbb.CreateVector(&o, 1);
    const auto evec =
        fbb.CreateVector(std::vector<flatbuffers::Offset<reflection::Enum>>{});
    reflection::SchemaBuilder sb(fbb);
    sb.add_objects(ovec);
    sb.add_enums(evec);
    reflection::FinishSchemaBuffer(fbb, sb.Finish());
    const auto* schema = VerifySchema(
        {reinterpret_cast<const char*>(fbb.GetBufferPointer()), fbb.GetSize()});
    ASSERT_NE(schema, nullptr);
    EXPECT_THROW(ResolveField(*schema, *schema->objects()->Get(0), "x", "test"),
                 std::invalid_argument);
  }

  // 3. EncodeField / EncodeNumber support UType and reject non-integer types.
  {
    std::array<std::byte, 4> buf{};
    const ResolvedField utype_field{
        .offset = 0,
        .base_type = reflection::BaseType::UType,
        .size = 1,
        .enum_def = nullptr,
        .leaf_name = "u",
        .enclosing_name = "Test",
    };
    EncodeField(buf.data(), utype_field, FieldValue{std::int64_t{200}}, "test");
    EXPECT_EQ(ReadInt64(reflection::BaseType::UType, buf.data()), 200);
    EXPECT_THROW(EncodeField(buf.data(), utype_field,
                             FieldValue{std::int64_t{256}}, "test"),
                 std::invalid_argument);
    EXPECT_THROW(
        EncodeNumber(buf.data(), reflection::BaseType::String, 1.0, "test"),
        std::invalid_argument);
  }

  // 4. Two-field self-cycle and 25-level diamond DAG terminate immediately.
  {
    flatbuffers::FlatBufferBuilder fbb;
    const auto ons = fbb.CreateString("bad.SelfCycle2");
    std::vector<flatbuffers::Offset<reflection::Field>> fields;
    for (int k = 0; k < 2; ++k) {
      const auto fns = fbb.CreateString(k == 0 ? "a" : "b");
      reflection::TypeBuilder tb(fbb);
      tb.add_base_type(reflection::BaseType::Obj);
      tb.add_index(0);
      const auto ft = tb.Finish();
      reflection::FieldBuilder fb(fbb);
      fb.add_name(fns);
      fb.add_type(ft);
      fb.add_id(static_cast<std::uint16_t>(k));
      fields.push_back(fb.Finish());
    }
    const auto fvec = fbb.CreateVector(fields);
    reflection::ObjectBuilder ob(fbb);
    ob.add_name(ons);
    ob.add_fields(fvec);
    ob.add_is_struct(true);
    ob.add_bytesize(8);
    const auto o = ob.Finish();
    const auto ovec = fbb.CreateVector(&o, 1);
    const auto evec =
        fbb.CreateVector(std::vector<flatbuffers::Offset<reflection::Enum>>{});
    reflection::SchemaBuilder sb(fbb);
    sb.add_objects(ovec);
    sb.add_enums(evec);
    reflection::FinishSchemaBuffer(fbb, sb.Finish());
    const auto* schema = VerifySchema(
        {reinterpret_cast<const char*>(fbb.GetBufferPointer()), fbb.GetSize()});
    ASSERT_NE(schema, nullptr);
    EXPECT_TRUE(ScalarPaths(*schema, *schema->objects()->Get(0)).empty());
  }
  {
    flatbuffers::FlatBufferBuilder fbb;
    constexpr int kLevels = 25;
    std::vector<flatbuffers::Offset<reflection::Object>> objs;
    for (int i = 0; i < kLevels; ++i) {
      const auto ons = fbb.CreateString("dag.o" + std::to_string(i));
      std::vector<flatbuffers::Offset<reflection::Field>> fields;
      if (i + 1 == kLevels) {
        const auto fns = fbb.CreateString("leaf_m");
        reflection::TypeBuilder tb(fbb);
        tb.add_base_type(reflection::BaseType::Double);
        const auto ft = tb.Finish();
        reflection::FieldBuilder fb(fbb);
        fb.add_name(fns);
        fb.add_type(ft);
        fields.push_back(fb.Finish());
      } else {
        for (int k = 0; k < 2; ++k) {
          const auto fns = fbb.CreateString(k == 0 ? "a" : "b");
          reflection::TypeBuilder tb(fbb);
          tb.add_base_type(reflection::BaseType::Obj);
          tb.add_index(i + 1);
          const auto ft = tb.Finish();
          reflection::FieldBuilder fb(fbb);
          fb.add_name(fns);
          fb.add_type(ft);
          fb.add_id(static_cast<std::uint16_t>(k));
          fields.push_back(fb.Finish());
        }
      }
      const auto fvec = fbb.CreateVector(fields);
      reflection::ObjectBuilder ob(fbb);
      ob.add_name(ons);
      ob.add_fields(fvec);
      ob.add_is_struct(true);
      ob.add_bytesize(8);
      objs.push_back(ob.Finish());
    }
    const auto ovec = fbb.CreateVector(objs);
    const auto evec =
        fbb.CreateVector(std::vector<flatbuffers::Offset<reflection::Enum>>{});
    reflection::SchemaBuilder sb(fbb);
    sb.add_objects(ovec);
    sb.add_enums(evec);
    reflection::FinishSchemaBuffer(fbb, sb.Finish());
    const auto* schema = VerifySchema(
        {reinterpret_cast<const char*>(fbb.GetBufferPointer()), fbb.GetSize()});
    ASSERT_NE(schema, nullptr);
    const auto paths = ScalarPaths(*schema, *schema->objects()->Get(0));
    EXPECT_LE(paths.size(), 4096u);
  }
}

// REQ-CONTRACT-07: ULong enum enumerators with bit 63 set (stored as negative
// int64_t in FlatBuffers reflection) encode their exact uint64_t bits, while
// non-integer scalars with a non-negative enum index are rejected.
TEST(FieldPath, EncodeFieldHandlesULongEnumsAndRejectsNonIntegerEnumIndices) {
  flatbuffers::FlatBufferBuilder builder;
  const auto enum_name = builder.CreateString("test.Flags");
  const auto low_name = builder.CreateString("LOW");
  const auto high_bit_name = builder.CreateString("HIGH_BIT");
  const auto max_name = builder.CreateString("MAX_VAL");

  reflection::EnumValBuilder ev0(builder);
  ev0.add_name(low_name);
  ev0.add_value(1);
  const auto v0 = ev0.Finish();

  reflection::EnumValBuilder ev1(builder);
  ev1.add_name(high_bit_name);
  ev1.add_value(static_cast<std::int64_t>(std::uint64_t{1} << 63));
  const auto v1 = ev1.Finish();

  reflection::EnumValBuilder ev2(builder);
  ev2.add_name(max_name);
  ev2.add_value(
      static_cast<std::int64_t>(std::numeric_limits<std::uint64_t>::max()));
  const auto v2 = ev2.Finish();

  const std::vector<flatbuffers::Offset<reflection::EnumVal>> evs = {v0, v1,
                                                                     v2};
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
  auto add_field = [&](const char* fname, reflection::BaseType base,
                       std::int32_t idx, std::uint16_t id,
                       std::uint16_t offset) {
    const auto fn = builder.CreateString(fname);
    reflection::TypeBuilder tb(builder);
    tb.add_base_type(base);
    tb.add_index(idx);
    const auto ft = tb.Finish();
    reflection::FieldBuilder fb(builder);
    fb.add_name(fn);
    fb.add_type(ft);
    fb.add_id(id);
    fb.add_offset(offset);
    return fb.Finish();
  };
  const std::vector<flatbuffers::Offset<reflection::Field>> fields = {
      add_field("flags", reflection::BaseType::ULong, 0, 0, 0),
      add_field("bad_bool", reflection::BaseType::Bool, 0, 1, 8),
      add_field("bad_float", reflection::BaseType::Float, 0, 2, 12),
      add_field("bad_double", reflection::BaseType::Double, 0, 3, 16),
      add_field("bad_oob_enum", reflection::BaseType::UByte, 5, 4, 24),
  };
  const auto fields_vec = builder.CreateVector(fields);
  reflection::ObjectBuilder ob(builder);
  ob.add_name(obj_name);
  ob.add_fields(fields_vec);
  ob.add_is_struct(true);
  ob.add_minalign(8);
  ob.add_bytesize(32);
  const auto obj = ob.Finish();
  const auto objects = builder.CreateVector(&obj, 1);
  reflection::SchemaBuilder sb(builder);
  sb.add_objects(objects);
  sb.add_enums(enums);
  reflection::FinishSchemaBuffer(builder, sb.Finish());

  const auto* schema =
      VerifySchema({reinterpret_cast<const char*>(builder.GetBufferPointer()),
                    builder.GetSize()});
  ASSERT_NE(schema, nullptr);
  const auto& holder = *schema->objects()->Get(0);

  const ResolvedField flags = ResolveField(*schema, holder, "flags", "test");
  ASSERT_NE(flags.enum_def, nullptr);
  std::array<std::byte, 32> buf{};

  for (const auto& [label, expected] :
       {std::pair{"LOW", std::uint64_t{1}},
        std::pair{"HIGH_BIT", std::uint64_t{1} << 63},
        std::pair{"MAX_VAL", std::numeric_limits<std::uint64_t>::max()}}) {
    EncodeField(buf.data(), flags, FieldValue{std::string{label}}, "test");
    std::uint64_t got = 0;
    std::memcpy(&got, buf.data() + flags.offset, sizeof(got));
    EXPECT_EQ(got, expected) << label;
  }
  EXPECT_THROW(
      EncodeField(buf.data(), flags, FieldValue{std::string{"NOPE"}}, "test"),
      std::invalid_argument);

  // Crafted schemas with an enum index on non-integer scalars or out-of-bounds
  // enum index are rejected at ResolveField.
  for (const char* bad_path :
       {"bad_bool", "bad_float", "bad_double", "bad_oob_enum"}) {
    EXPECT_THROW(ResolveField(*schema, holder, bad_path, "test"),
                 std::invalid_argument)
        << bad_path;
  }
}

// REQ-CONTRACT-07: EncodeField and EncodeNumber refuse out-of-range floats,
// NaN/Inf, negative unsigned values, fractional integers, and variant-type
// mismatches.
TEST(FieldPath, EncodeFieldAndEncodeNumberRefuseBoundariesAndMismatches) {
  std::array<std::byte, 8> buf{};
  auto scalar_field = [](reflection::BaseType base, std::size_t size) {
    return ResolvedField{
        .offset = 0,
        .base_type = base,
        .size = size,
        .enum_def = nullptr,
        .leaf_name = "f",
        .enclosing_name = "Test",
    };
  };

  // Float range limits, NaN, and Inf.
  const double past_flt_max =
      static_cast<double>(std::numeric_limits<float>::max()) * 2.0;
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const double bad : {past_flt_max, -past_flt_max, nan, inf, -inf}) {
    EXPECT_THROW(
        EncodeNumber(buf.data(), reflection::BaseType::Float, bad, "test"),
        std::invalid_argument);
    EXPECT_THROW(
        EncodeField(buf.data(), scalar_field(reflection::BaseType::Float, 4),
                    FieldValue{bad}, "test"),
        std::invalid_argument);
  }
  for (const double bad : {nan, inf, -inf}) {
    EXPECT_THROW(
        EncodeNumber(buf.data(), reflection::BaseType::Double, bad, "test"),
        std::invalid_argument);
    EXPECT_THROW(
        EncodeNumber(buf.data(), reflection::BaseType::Int, bad, "test"),
        std::invalid_argument);
    EXPECT_THROW(
        EncodeNumber(buf.data(), reflection::BaseType::ULong, bad, "test"),
        std::invalid_argument);
  }

  // Negative values for unsigned integer fields (both int64_t and double).
  for (const auto& [base, size] :
       {std::pair{reflection::BaseType::UType, std::size_t{1}},
        std::pair{reflection::BaseType::UByte, std::size_t{1}},
        std::pair{reflection::BaseType::UShort, std::size_t{2}},
        std::pair{reflection::BaseType::UInt, std::size_t{4}},
        std::pair{reflection::BaseType::ULong, std::size_t{8}}}) {
    EXPECT_THROW(EncodeField(buf.data(), scalar_field(base, size),
                             FieldValue{std::int64_t{-1}}, "test"),
                 std::invalid_argument);
    EXPECT_THROW(EncodeField(buf.data(), scalar_field(base, size),
                             FieldValue{-1.0}, "test"),
                 std::invalid_argument);
  }

  // Fractional values for integer fields.
  for (const reflection::BaseType base :
       {reflection::BaseType::Int, reflection::BaseType::Long,
        reflection::BaseType::ULong}) {
    EXPECT_THROW(EncodeNumber(buf.data(), base, 1.5, "test"),
                 std::invalid_argument);
    EXPECT_THROW(EncodeNumber(buf.data(), base, -0.5, "test"),
                 std::invalid_argument);
  }

  // Integer range overflow via int64_t and double.
  EXPECT_THROW(
      EncodeField(buf.data(), scalar_field(reflection::BaseType::Byte, 1),
                  FieldValue{std::int64_t{128}}, "test"),
      std::invalid_argument);
  EXPECT_THROW(
      EncodeField(buf.data(), scalar_field(reflection::BaseType::Byte, 1),
                  FieldValue{std::int64_t{-129}}, "test"),
      std::invalid_argument);
  EXPECT_THROW(
      EncodeField(buf.data(), scalar_field(reflection::BaseType::UShort, 2),
                  FieldValue{std::int64_t{65536}}, "test"),
      std::invalid_argument);
  EXPECT_THROW(EncodeNumber(buf.data(), reflection::BaseType::Long,
                            std::ldexp(1.0, 63), "test"),
               std::invalid_argument);
  EXPECT_THROW(EncodeNumber(buf.data(), reflection::BaseType::ULong,
                            std::ldexp(1.0, 64), "test"),
               std::invalid_argument);

  // Type-variant mismatches: bool, enum, and numeric fields.
  const ResolvedField bool_field = scalar_field(reflection::BaseType::Bool, 1);
  for (const FieldValue& bad : {FieldValue{std::int64_t{1}}, FieldValue{1.0},
                                FieldValue{std::string{"true"}}}) {
    EXPECT_THROW(EncodeField(buf.data(), bool_field, bad, "test"),
                 std::invalid_argument);
  }

  const Identity mixed_id = Of<schema_test::Mixed>();
  const reflection::Schema* schema = VerifySchema(mixed_id.bfbs);
  ASSERT_NE(schema, nullptr);
  const reflection::Object* mixed =
      schema->objects()->LookupByKey(mixed_id.name.c_str());
  ASSERT_NE(mixed, nullptr);
  const ResolvedField enum_field =
      ResolveField(*schema, *mixed, "level", "test");
  for (const FieldValue& bad :
       {FieldValue{true}, FieldValue{std::int64_t{0}}, FieldValue{0.0}}) {
    EXPECT_THROW(EncodeField(buf.data(), enum_field, bad, "test"),
                 std::invalid_argument);
  }

  const ResolvedField double_field =
      scalar_field(reflection::BaseType::Double, 8);
  const ResolvedField int_field = scalar_field(reflection::BaseType::Int, 4);
  for (const ResolvedField& num_field : {double_field, int_field}) {
    for (const FieldValue& bad :
         {FieldValue{true}, FieldValue{std::string{"1"}}}) {
      EXPECT_THROW(EncodeField(buf.data(), num_field, bad, "test"),
                   std::invalid_argument);
    }
  }
}

}  // namespace
}  // namespace talos::schema
