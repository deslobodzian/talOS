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
}

}  // namespace
}  // namespace talos::schema
