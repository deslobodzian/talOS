#include "talOS/schema/field_path.h"

#include <gtest/gtest.h>

#include <array>
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

}  // namespace
}  // namespace talos::schema
