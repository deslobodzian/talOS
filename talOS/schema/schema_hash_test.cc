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

// REQ-CONTRACT-07
TEST(SchemaHash, CommentsAndFormattingDoNotChangeIt) {
  for (const char* name : {"schema_variant.Reading", "schema_variant.Holder"}) {
    EXPECT_EQ(VariantHash("TALOS_VARIANT_FLOAT_LEVEL", name),
              VariantHash("TALOS_VARIANT_FLOAT_LEVEL_REFORMATTED", name))
        << name;
  }
}

// REQ-CONTRACT-07
TEST(SchemaHash, SameSizeChangesChangeItAndThoseOfStructsHoldingIt) {
  const std::vector<std::uint8_t> base = Variant("TALOS_VARIANT_FLOAT_LEVEL");
  const reflection::Object* base_reading =
      reflection::GetSchema(base.data())
          ->objects()
          ->LookupByKey("schema_variant.Reading");
  ASSERT_NE(base_reading, nullptr);
  ASSERT_EQ(base_reading->bytesize(), 8);
  ASSERT_EQ(base_reading->minalign(), 4);

  // Each variant isolates one canonical_struct component while preserving
  // bytesize == 8 and minalign == 4: scalar base type, field name, field
  // offset order, enum underlying type, enum name, enum value name, and enum
  // enumerator value.
  for (const char* variant :
       {"TALOS_VARIANT_INT_LEVEL", "TALOS_VARIANT_RENAMED_LEVEL",
        "TALOS_VARIANT_SWAPPED_FIELDS", "TALOS_VARIANT_SIGNED_KIND",
        "TALOS_VARIANT_RENAMED_ENUM", "TALOS_VARIANT_RENAMED_ENUM_VALUE",
        "TALOS_VARIANT_RENUMBERED_KIND"}) {
    const std::vector<std::uint8_t> changed = Variant(variant);
    const reflection::Object* changed_reading =
        reflection::GetSchema(changed.data())
            ->objects()
            ->LookupByKey("schema_variant.Reading");
    ASSERT_NE(changed_reading, nullptr) << variant;
    EXPECT_EQ(changed_reading->bytesize(), 8) << variant;
    EXPECT_EQ(changed_reading->minalign(), 4) << variant;
    for (const char* name :
         {"schema_variant.Reading", "schema_variant.Holder"}) {
      EXPECT_NE(Hash(base, name), Hash(changed, name))
          << variant << " " << name;
    }
  }

  // Confirm swapped_fields actually changes field offsets while keeping the
  // reflection field list (sorted by name) identical.
  const std::vector<std::uint8_t> swapped =
      Variant("TALOS_VARIANT_SWAPPED_FIELDS");
  const reflection::Object* swapped_reading =
      reflection::GetSchema(swapped.data())
          ->objects()
          ->LookupByKey("schema_variant.Reading");
  ASSERT_NE(swapped_reading, nullptr);
  EXPECT_EQ(base_reading->fields()->LookupByKey("level_m")->offset(), 0);
  EXPECT_EQ(swapped_reading->fields()->LookupByKey("level_m")->offset(), 4);
  EXPECT_EQ(base_reading->fields()->LookupByKey("kind")->offset(), 4);
  EXPECT_EQ(swapped_reading->fields()->LookupByKey("kind")->offset(), 0);
}

std::vector<std::uint8_t> SingleFieldSchema(
    std::int32_t bytesize, std::int32_t minalign, reflection::BaseType base,
    std::int32_t index, std::uint16_t offset = 0,
    reflection::BaseType element = reflection::BaseType::None,
    std::uint16_t length = 0, const char* struct_name = "bad.Struct") {
  flatbuffers::FlatBufferBuilder builder;
  const auto name = builder.CreateString(struct_name);
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
  object_builder.add_minalign(minalign);
  object_builder.add_bytesize(bytesize);
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

// REQ-CONTRACT-07
TEST(SchemaHash, AlignmentPaddingArrayLengthAndStructNameChangesAffectHash) {
  const std::vector<std::uint8_t> base = Variant("TALOS_VARIANT_FLOAT_LEVEL");

  // minalign change (4 -> 8) with identical bytesize (8) and field layout.
  const std::vector<std::uint8_t> aligned =
      Variant("TALOS_VARIANT_ALIGNED_READING");
  const reflection::Object* aligned_reading =
      reflection::GetSchema(aligned.data())
          ->objects()
          ->LookupByKey("schema_variant.Reading");
  ASSERT_NE(aligned_reading, nullptr);
  EXPECT_EQ(aligned_reading->bytesize(), 8);
  EXPECT_EQ(aligned_reading->minalign(), 8);
  for (const char* name : {"schema_variant.Reading", "schema_variant.Holder"}) {
    EXPECT_NE(Hash(base, name), Hash(aligned, name)) << name;
  }

  // Trailing padding + alignment change (bytesize 8 -> 16, minalign 4 -> 16)
  // with identical field offsets.
  const std::vector<std::uint8_t> padded =
      Variant("TALOS_VARIANT_PADDED_READING");
  const reflection::Object* padded_reading =
      reflection::GetSchema(padded.data())
          ->objects()
          ->LookupByKey("schema_variant.Reading");
  ASSERT_NE(padded_reading, nullptr);
  EXPECT_EQ(padded_reading->bytesize(), 16);
  EXPECT_EQ(padded_reading->minalign(), 16);
  for (const char* name : {"schema_variant.Reading", "schema_variant.Holder"}) {
    EXPECT_NE(Hash(base, name), Hash(padded, name)) << name;
  }

  // Isolated bytesize-only change (8 -> 16) with identical minalign (8) and
  // field layout, so deleting bytesize from canonical_struct breaks this test.
  const std::vector<std::uint8_t> size8 =
      SingleFieldSchema(8, 8, reflection::BaseType::Double, -1, 0,
                        reflection::BaseType::None, 0, "synthetic.Padded");
  const std::vector<std::uint8_t> size16 =
      SingleFieldSchema(16, 8, reflection::BaseType::Double, -1, 0,
                        reflection::BaseType::None, 0, "synthetic.Padded");
  const auto hash8 = Hash(size8, "synthetic.Padded");
  const auto hash16 = Hash(size16, "synthetic.Padded");
  ASSERT_TRUE(hash8.has_value());
  ASSERT_TRUE(hash16.has_value());
  EXPECT_NE(*hash8, *hash16);

  // Fixed-array length change (2 -> 3) inside trailing struct padding so
  // bytesize (8), minalign (4), field offsets and element type stay identical.
  const std::vector<std::uint8_t> arr2 = Variant("TALOS_VARIANT_ARRAY_LEN_2");
  const std::vector<std::uint8_t> arr3 = Variant("TALOS_VARIANT_ARRAY_LEN_3");
  const reflection::Object* series2 =
      reflection::GetSchema(arr2.data())
          ->objects()
          ->LookupByKey("schema_variant.Series");
  const reflection::Object* series3 =
      reflection::GetSchema(arr3.data())
          ->objects()
          ->LookupByKey("schema_variant.Series");
  ASSERT_NE(series2, nullptr);
  ASSERT_NE(series3, nullptr);
  EXPECT_EQ(series2->bytesize(), 8);
  EXPECT_EQ(series3->bytesize(), 8);
  EXPECT_EQ(series2->minalign(), 4);
  EXPECT_EQ(series3->minalign(), 4);
  EXPECT_EQ(series2->fields()->LookupByKey("samples_m")->type()->fixed_length(),
            2);
  EXPECT_EQ(series3->fields()->LookupByKey("samples_m")->type()->fixed_length(),
            3);
  for (const char* name :
       {"schema_variant.Series", "schema_variant.SeriesHolder"}) {
    EXPECT_NE(Hash(arr2, name), Hash(arr3, name)) << name;
  }

  // Nested struct name change (Reading -> Sample) inside Holder with identical
  // bytesize, minalign, field names/offsets and inner fields.
  const std::vector<std::uint8_t> renamed_struct =
      Variant("TALOS_VARIANT_RENAMED_STRUCT");
  EXPECT_NE(Hash(base, "schema_variant.Holder"),
            Hash(renamed_struct, "schema_variant.Holder"));
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
std::vector<std::uint8_t> MalformedSchema(
    reflection::BaseType base, std::int32_t index, std::uint16_t offset = 0,
    reflection::BaseType element = reflection::BaseType::None,
    std::uint16_t length = 0) {
  return SingleFieldSchema(8, 8, base, index, offset, element, length);
}

TEST(SchemaHash, RejectsInvalidReflectionIndicesLayoutsAndCycles) {
  using reflection::BaseType;
  for (const auto& bytes :
       {MalformedSchema(BaseType::Obj, -1), MalformedSchema(BaseType::Obj, 1),
        MalformedSchema(BaseType::Int, 0),
        MalformedSchema(BaseType::Long, -1, 1),
        MalformedSchema(BaseType::Array, -1, 0, BaseType::Long, 2),
        MalformedSchema(BaseType::Array, 1, 0, BaseType::Obj, 1),
        MalformedSchema(BaseType::Obj, 0)}) {
    flatbuffers::Verifier verifier(bytes.data(), bytes.size());
    ASSERT_TRUE(reflection::VerifySchemaBuffer(verifier));
    EXPECT_FALSE(Hash(bytes, "bad.Struct"));
    const auto* schema = reflection::GetSchema(bytes.data());
    EXPECT_TRUE(
        talos::schema::canonical_struct(*schema, *schema->objects()->Get(0))
            .empty());
  }

  // Out-of-range raw BaseType bytes (-128, 127) are rejected by struct_hash
  // without casting to reflection::BaseType.
  for (const std::int8_t bad_byte :
       {static_cast<std::int8_t>(-128), static_cast<std::int8_t>(127)}) {
    SCOPED_TRACE("raw BaseType=" + std::to_string(bad_byte));
    flatbuffers::FlatBufferBuilder builder;
    const auto obj_name = builder.CreateString("bad.Struct");
    const auto field_name = builder.CreateString("x");
    const auto type_start = builder.StartTable();
    builder.AddElement<std::int8_t>(reflection::Type::VT_BASE_TYPE, bad_byte,
                                    0);
    const flatbuffers::Offset<reflection::Type> type{
        builder.EndTable(type_start)};
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
    const std::vector<std::uint8_t> bytes(
        builder.GetBufferPointer(),
        builder.GetBufferPointer() + builder.GetSize());
    EXPECT_FALSE(Hash(bytes, "bad.Struct"));
  }
}

// Guards against exponential recursion in append_struct: a 22-level diamond DAG
// hits the struct-visit cap and a 2-field self-cycle hits the per-object cycle
// guard, both failing cleanly without OOM or hang (REQ-OPS-41,
// REQ-CONTRACT-20).
TEST(SchemaHash, RejectsDiamondDagAndTwoFieldSelfCycle) {
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

    const std::vector<std::uint8_t> bytes(
        builder.GetBufferPointer(),
        builder.GetBufferPointer() + builder.GetSize());
    flatbuffers::Verifier verifier(bytes.data(), bytes.size());
    ASSERT_TRUE(reflection::VerifySchemaBuffer(verifier));
    ASSERT_FALSE(Hash(bytes, "dag.o0"));
    const auto* schema = reflection::GetSchema(bytes.data());
    ASSERT_TRUE(
        talos::schema::canonical_struct(*schema, *schema->objects()->Get(0))
            .empty());
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

    const std::vector<std::uint8_t> bytes(
        builder.GetBufferPointer(),
        builder.GetBufferPointer() + builder.GetSize());
    flatbuffers::Verifier verifier(bytes.data(), bytes.size());
    ASSERT_TRUE(reflection::VerifySchemaBuffer(verifier));
    EXPECT_FALSE(Hash(bytes, "bad.C"));
    const auto* schema = reflection::GetSchema(bytes.data());
    EXPECT_TRUE(
        talos::schema::canonical_struct(*schema, *schema->objects()->Get(0))
            .empty());
  }
}

}  // namespace
