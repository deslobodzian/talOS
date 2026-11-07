// talos::Descriptor<M> as talos_messages generates it, and the schema hash
// over separately compiled variants of one schema (testdata/variants).

#include "talOS/schema/schema_hash.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "flatbuffers/reflection_generated.h"
#include "flatbuffers/verifier.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "schema_test/shapes_generated.h"
#include "talOS/schema/descriptor.h"

namespace {

using talos::Descriptor;

std::optional<std::uint64_t> Hash(std::span<const std::uint8_t> bfbs,
                                  const char* name) {
  return talos::schema::struct_hash(bfbs.data(), bfbs.size(), name);
}

static_assert(talos::Described<schema_test::Mixed>);
static_assert(!talos::Described<int>);

std::vector<std::uint8_t> Variant(const char* env) {
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  const char* location = std::getenv(env);
  if (!runfiles || location == nullptr) {
    throw std::runtime_error(std::string{"runfile unavailable: "} + env + " " +
                             error);
  }
  std::ifstream in(runfiles->Rlocation(location), std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

std::uint64_t VariantHash(const char* env, const char* name) {
  const std::optional<std::uint64_t> hash = Hash(Variant(env), name);
  if (!hash) throw std::runtime_error(std::string{"no struct "} + name);
  return *hash;
}

template <class M>
const reflection::Object& Reflected() {
  const reflection::Schema* schema =
      reflection::GetSchema(Descriptor<M>::kBfbs.data());
  const reflection::Object* object =
      schema->objects()->LookupByKey(std::string(Descriptor<M>::kName).c_str());
  if (object == nullptr) throw std::runtime_error("struct not in its schema");
  return *object;
}

TEST(Descriptor, NamesTheSchemaStruct) {
  EXPECT_EQ(Descriptor<schema_test::Inner>::kName, "schema_test.Inner");
  EXPECT_EQ(Descriptor<schema_test::Mixed>::kName, "schema_test.Mixed");
  EXPECT_EQ(Descriptor<schema_test::Arrays>::kName, "schema_test.Arrays");
}

TEST(Descriptor, EmbeddedSchemaMatchesTheCompiledLayout) {
  EXPECT_EQ(Reflected<schema_test::Inner>().bytesize(),
            sizeof(schema_test::Inner));
  EXPECT_EQ(Reflected<schema_test::Mixed>().bytesize(),
            sizeof(schema_test::Mixed));
  EXPECT_EQ(Reflected<schema_test::Mixed>().minalign(),
            alignof(schema_test::Mixed));
  EXPECT_EQ(Reflected<schema_test::Arrays>().bytesize(),
            sizeof(schema_test::Arrays));
}

TEST(Descriptor, HashIsTheHashOfItsEmbeddedSchema) {
  EXPECT_EQ(Hash(Descriptor<schema_test::Inner>::kBfbs, "schema_test.Inner"),
            Descriptor<schema_test::Inner>::kHash);
  EXPECT_EQ(Hash(Descriptor<schema_test::Mixed>::kBfbs, "schema_test.Mixed"),
            Descriptor<schema_test::Mixed>::kHash);
  EXPECT_EQ(Hash(Descriptor<schema_test::Arrays>::kBfbs, "schema_test.Arrays"),
            Descriptor<schema_test::Arrays>::kHash);
}

TEST(Descriptor, DistinctStructsHaveDistinctHashes) {
  EXPECT_NE(Descriptor<schema_test::Inner>::kHash,
            Descriptor<schema_test::Mixed>::kHash);
  EXPECT_NE(Descriptor<schema_test::Mixed>::kHash,
            Descriptor<schema_test::Arrays>::kHash);
  EXPECT_NE(Descriptor<schema_test::Inner>::kHash,
            Descriptor<schema_test::Arrays>::kHash);
}

TEST(SchemaHash, CommentsAndFormattingDoNotChangeIt) {
  for (const char* name : {"schema_variant.Reading", "schema_variant.Holder"}) {
    EXPECT_EQ(VariantHash("TALOS_VARIANT_FLOAT_LEVEL", name),
              VariantHash("TALOS_VARIANT_FLOAT_LEVEL_REFORMATTED", name))
        << name;
  }
}

TEST(SchemaHash, SameSizeChangesChangeItAndThoseOfStructsHoldingIt) {
  const std::vector<std::uint8_t> base = Variant("TALOS_VARIANT_FLOAT_LEVEL");
  ASSERT_EQ(reflection::GetSchema(base.data())
                ->objects()
                ->LookupByKey("schema_variant.Reading")
                ->bytesize(),
            8);
  for (const char* variant :
       {"TALOS_VARIANT_INT_LEVEL", "TALOS_VARIANT_RENAMED_LEVEL",
        "TALOS_VARIANT_RENUMBERED_KIND"}) {
    const std::vector<std::uint8_t> changed = Variant(variant);
    EXPECT_EQ(reflection::GetSchema(changed.data())
                  ->objects()
                  ->LookupByKey("schema_variant.Reading")
                  ->bytesize(),
              8)
        << variant;
    for (const char* name :
         {"schema_variant.Reading", "schema_variant.Holder"}) {
      EXPECT_NE(Hash(base, name), Hash(changed, name))
          << variant << " " << name;
    }
  }
}

TEST(SchemaHash, RejectsWhatIsNotAStructInASchema) {
  const std::vector<std::uint8_t> base = Variant("TALOS_VARIANT_FLOAT_LEVEL");
  EXPECT_FALSE(Hash(base, "schema_variant.Missing"));
  EXPECT_FALSE(Hash(base, "schema_variant.Kind"));  // an enum
  const std::vector<std::uint8_t> truncated(base.data(),
                                            base.data() + base.size() / 2);
  EXPECT_FALSE(Hash(truncated, "schema_variant.Reading"));
  const std::vector<std::uint8_t> garbage(64, 0xa5);
  EXPECT_FALSE(Hash(garbage, "schema_variant.Reading"));
}

// These buffers pass the FlatBuffers verifier: the invalid references and
// inline layouts are semantic errors, not broken table encodings.
std::vector<std::uint8_t> MalformedSchema(reflection::BaseType base,
                                          std::int32_t index,
                                          std::uint16_t offset = 0,
                                          reflection::BaseType element =
                                              reflection::BaseType::None,
                                          std::uint16_t length = 0) {
  flatbuffers::FlatBufferBuilder builder;
  const auto name = builder.CreateString("bad.Struct");
  const auto field_name = builder.CreateString("field");
  reflection::TypeBuilder type_builder(builder);
  type_builder.add_base_type(base);
  type_builder.add_index(index);
  type_builder.add_element(element);
  type_builder.add_fixed_length(length);
  const auto type = type_builder.Finish();
  reflection::FieldBuilder field_builder(builder);
  field_builder.add_name(field_name);
  field_builder.add_type(type);
  field_builder.add_offset(offset);
  const auto field = field_builder.Finish();
  const auto fields = builder.CreateVector(&field, 1);
  reflection::ObjectBuilder object_builder(builder);
  object_builder.add_name(name);
  object_builder.add_fields(fields);
  object_builder.add_is_struct(true);
  object_builder.add_minalign(8);
  object_builder.add_bytesize(8);
  const auto object = object_builder.Finish();
  const auto objects = builder.CreateVector(&object, 1);
  const auto enums = builder.CreateVector(
      std::vector<flatbuffers::Offset<reflection::Enum>>{});
  reflection::SchemaBuilder schema_builder(builder);
  schema_builder.add_objects(objects);
  schema_builder.add_enums(enums);
  reflection::FinishSchemaBuffer(builder, schema_builder.Finish());
  return {builder.GetBufferPointer(),
          builder.GetBufferPointer() + builder.GetSize()};
}

TEST(SchemaHash, RejectsInvalidReflectionIndicesLayoutsAndCycles) {
  using reflection::BaseType;
  for (const auto& bytes : {
           MalformedSchema(BaseType::Obj, -1),
           MalformedSchema(BaseType::Obj, 1),
           MalformedSchema(BaseType::Int, 0),
           MalformedSchema(BaseType::Long, -1, 1),
           MalformedSchema(BaseType::Array, -1, 0, BaseType::Long, 2),
           MalformedSchema(BaseType::Array, 1, 0, BaseType::Obj, 1),
           MalformedSchema(BaseType::Obj, 0)}) {
    flatbuffers::Verifier verifier(bytes.data(), bytes.size());
    ASSERT_TRUE(reflection::VerifySchemaBuffer(verifier));
    EXPECT_FALSE(Hash(bytes, "bad.Struct"));
    const auto* schema = reflection::GetSchema(bytes.data());
    EXPECT_TRUE(talos::schema::canonical_struct(
                    *schema, *schema->objects()->Get(0)).empty());
  }
}

}  // namespace
