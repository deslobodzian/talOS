#include "talOS/schema/field_path.h"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <utility>

#include "flatbuffers/reflection.h"
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

}  // namespace
}  // namespace talos::schema
