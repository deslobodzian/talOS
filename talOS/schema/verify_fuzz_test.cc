// Deterministic seeded mutation fuzz test for talOS/schema: VerifySchema,
// DecodeToText, struct_hash, ScalarPaths, ResolveField, EncodeField, and
// EncodeNumber.

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <random>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "flatbuffers/flatbuffers.h"
#include "flatbuffers/reflection_generated.h"
#include "schema_test/shapes_generated.h"
#include "talOS/schema/decode.h"
#include "talOS/schema/field_path.h"
#include "talOS/schema/identity.h"
#include "talOS/schema/schema_hash.h"
#include "talOS/schema/verify.h"

namespace talos::schema {
namespace {

std::vector<std::byte> MutateBytes(std::mt19937_64& rng,
                                   std::span<const std::byte> seed,
                                   std::size_t max_len) {
  std::vector<std::byte> out(seed.begin(), seed.end());
  const std::uint64_t op = rng() % 6;
  if (op == 0 && out.size() > 1) {
    // Truncate.
    out.resize(1 + (rng() % out.size()));
  } else if (op == 1 && !out.empty()) {
    // Bit/byte flips.
    const std::size_t flips = 1 + (rng() % 4);
    for (std::size_t i = 0; i < flips; ++i) {
      const std::size_t idx = rng() % out.size();
      out[idx] ^= static_cast<std::byte>(1u << (rng() % 8));
    }
  } else if (op == 2 && !out.empty()) {
    // Overwrite a 2- or 4-byte integer.
    const std::size_t idx = rng() % out.size();
    const std::uint8_t val = static_cast<std::uint8_t>(rng() & 0xffu);
    out[idx] = static_cast<std::byte>(val);
    if (idx + 1 < out.size()) {
      out[idx + 1] = static_cast<std::byte>((rng() >> 8) & 0xffu);
    }
  } else if (op == 3 && out.size() < max_len) {
    // Append random bytes.
    const std::size_t extra = 1 + (rng() % 16);
    for (std::size_t i = 0; i < extra && out.size() < max_len; ++i) {
      out.push_back(static_cast<std::byte>(rng() & 0xffu));
    }
  } else if (op == 4 && !out.empty()) {
    // Splice / duplicate a slice.
    const std::size_t at = rng() % out.size();
    const std::size_t len = std::min<std::size_t>(8, out.size() - at);
    out.insert(out.begin() + static_cast<std::ptrdiff_t>(at),
               out.begin() + static_cast<std::ptrdiff_t>(at),
               out.begin() + static_cast<std::ptrdiff_t>(at + len));
  } else {
    // Pure garbage buffer.
    const std::size_t n = rng() % 128;
    out.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
      out[i] = static_cast<std::byte>(rng() & 0xffu);
    }
  }
  return out;
}

std::string CraftHostileSchema(std::mt19937_64& rng) {
  flatbuffers::FlatBufferBuilder fbb;
  const std::size_t num_enums = rng() % 3;
  std::vector<flatbuffers::Offset<reflection::Enum>> enums;
  for (std::size_t ei = 0; ei < num_enums; ++ei) {
    const std::size_t nvals = rng() % 4;
    std::vector<flatbuffers::Offset<reflection::EnumVal>> vals;
    for (std::size_t vi = 0; vi < nvals; ++vi) {
      const auto ns = fbb.CreateString("V" + std::to_string(vi));
      reflection::EnumValBuilder evb(fbb);
      evb.add_name(ns);
      evb.add_value(static_cast<std::int64_t>(rng()));
      vals.push_back(evb.Finish());
    }
    const auto vals_vec = fbb.CreateVector(vals);
    const auto t_start = fbb.StartTable();
    fbb.AddElement<std::int8_t>(reflection::Type::VT_BASE_TYPE,
                                static_cast<std::int8_t>(rng() & 0xffu), 0);
    fbb.AddElement<std::int8_t>(reflection::Type::VT_ELEMENT,
                                static_cast<std::int8_t>(rng() & 0xffu), 0);
    const flatbuffers::Offset<reflection::Type> ut{fbb.EndTable(t_start)};
    const auto ens = fbb.CreateString("e" + std::to_string(ei));
    reflection::EnumBuilder eb(fbb);
    eb.add_name(ens);
    eb.add_values(vals_vec);
    eb.add_underlying_type(ut);
    enums.push_back(eb.Finish());
  }

  const std::size_t num_objs = 1 + (rng() % 4);
  std::vector<flatbuffers::Offset<reflection::Object>> objs;
  for (std::size_t oi = 0; oi < num_objs; ++oi) {
    const std::size_t nfields = rng() % 4;
    std::vector<flatbuffers::Offset<reflection::Field>> fields;
    for (std::size_t fi = 0; fi < nfields; ++fi) {
      const auto t_start = fbb.StartTable();
      // Bias towards valid BaseType values (0..19) plus out-of-range bytes.
      const std::int8_t base = (rng() % 4 == 0)
                                   ? static_cast<std::int8_t>(rng() & 0xffu)
                                   : static_cast<std::int8_t>(rng() % 20);
      const std::int8_t elem = (rng() % 4 == 0)
                                   ? static_cast<std::int8_t>(rng() & 0xffu)
                                   : static_cast<std::int8_t>(rng() % 20);
      fbb.AddElement<std::int8_t>(reflection::Type::VT_BASE_TYPE, base, 0);
      fbb.AddElement<std::int8_t>(reflection::Type::VT_ELEMENT, elem, 0);
      // Exercise all object/enum indices (0..3, -1, 4), including self-cycles
      // and mutual cycles.
      const std::int32_t idx = static_cast<std::int32_t>(rng() % 6) - 1;
      fbb.AddElement<std::int32_t>(reflection::Type::VT_INDEX, idx, -1);
      fbb.AddElement<std::uint16_t>(reflection::Type::VT_FIXED_LENGTH,
                                    static_cast<std::uint16_t>(rng() % 8), 0);
      const flatbuffers::Offset<reflection::Type> ftype{fbb.EndTable(t_start)};
      const auto fns = fbb.CreateString("f" + std::to_string(fi));
      reflection::FieldBuilder fb(fbb);
      fb.add_name(fns);
      fb.add_type(ftype);
      fb.add_id(static_cast<std::uint16_t>(fi));
      fb.add_offset(static_cast<std::uint16_t>(rng() % 64));
      fields.push_back(fb.Finish());
    }
    const auto fvec = fbb.CreateVector(fields);
    const auto ons = fbb.CreateString("t.o" + std::to_string(oi));
    reflection::ObjectBuilder ob(fbb);
    ob.add_name(ons);
    ob.add_fields(fvec);
    ob.add_is_struct((rng() % 5) != 0);
    ob.add_minalign(static_cast<std::int32_t>((rng() % 16)));
    ob.add_bytesize(static_cast<std::int32_t>(rng() % 40) - 4);
    objs.push_back(ob.Finish());
  }
  const auto ovec = fbb.CreateVector(objs);
  const auto evec = fbb.CreateVector(enums);
  reflection::SchemaBuilder sb(fbb);
  sb.add_objects(ovec);
  sb.add_enums(evec);
  reflection::FinishSchemaBuffer(fbb, sb.Finish());
  return {reinterpret_cast<const char*>(fbb.GetBufferPointer()), fbb.GetSize()};
}

void ExerciseSchemaAndPayload(std::string_view bfbs,
                              std::span<const std::byte> payload,
                              std::mt19937_64& rng,
                              bool exercise_decode_and_hash) {
  const reflection::Schema* schema = VerifySchema(bfbs);
  std::string struct_name = "schema_test.Mixed";
  if (schema != nullptr && schema->objects() != nullptr &&
      schema->objects()->size() > 0) {
    const auto* obj = schema->objects()->Get(
        static_cast<flatbuffers::uoffset_t>(rng() % schema->objects()->size()));
    if (obj != nullptr && obj->name() != nullptr) {
      struct_name = obj->name()->str();
    }
  }

  if (exercise_decode_and_hash) {
    Identity id{struct_name, 0, std::string{bfbs}};
    (void)DecodeToText(id, payload);
    (void)DecodeToText(Envelope(id), payload);
    (void)struct_hash(reinterpret_cast<const std::uint8_t*>(bfbs.data()),
                      bfbs.size(), struct_name);
  }

  if (schema != nullptr && schema->objects() != nullptr) {
    for (flatbuffers::uoffset_t i = 0; i < schema->objects()->size() && i < 4;
         ++i) {
      const auto* obj = schema->objects()->Get(i);
      if (obj == nullptr) continue;
      const auto paths = ScalarPaths(*schema, *obj);
      for (std::size_t p = 0; p < paths.size() && p < 6; ++p) {
        try {
          const ResolvedField rf =
              ResolveField(*schema, *obj, paths[p], "fuzz");
          if (obj->bytesize() > 0 &&
              rf.offset + rf.size <=
                  static_cast<std::size_t>(obj->bytesize())) {
            std::vector<std::byte> buf(
                static_cast<std::size_t>(obj->bytesize()), std::byte{0});
            const std::size_t n = std::min(buf.size(), payload.size());
            if (n > 0) std::memcpy(buf.data(), payload.data(), n);
            (void)ReadInt64(rf.base_type, buf.data() + rf.offset);
            (void)ReadDouble(rf.base_type, buf.data() + rf.offset);
            (void)ReadBool(rf.base_type, buf.data() + rf.offset);
            for (const FieldValue& v :
                 {FieldValue{true}, FieldValue{static_cast<std::int64_t>(7)},
                  FieldValue{1.25}, FieldValue{std::string{"OFF"}}}) {
              try {
                EncodeField(buf.data(), rf, v, "fuzz");
              } catch (const std::invalid_argument& e) {
                // Expected rejection of malformed/mismatched fuzz inputs.
                EXPECT_NE(e.what(), nullptr);
              }
            }
          }
        } catch (const std::invalid_argument& e) {
          // Expected rejection of malformed/hostile fuzz schemas.
          EXPECT_NE(e.what(), nullptr);
        }
      }
      // Also exercise malformed field paths.
      for (const char* bad : {"", "/", "a/", "/a", "a//b", "missing/x"}) {
        try {
          (void)ResolveField(*schema, *obj, bad, "fuzz");
        } catch (const std::invalid_argument& e) {
          // Expected rejection of malformed field paths.
          EXPECT_NE(e.what(), nullptr);
        }
      }
    }
  }
}

TEST(SchemaVerifyFuzz, SeededMutatedBfbsAndHostileSchemasDoNotCrash) {
  const Identity mixed_id = Of<schema_test::Mixed>();
  const Identity arrays_id = Of<schema_test::Arrays>();
  const std::array<std::string_view, 2> seeds = {mixed_id.bfbs, arrays_id.bfbs};
  std::mt19937_64 rng(0x534348454d41ULL);

  // 2,000 mutated .bfbs buffers (aligned + unaligned) + mutated payloads.
  for (int iter = 0; iter < 2000; ++iter) {
    const std::string_view seed = seeds[static_cast<std::size_t>(iter) % 2];
    const auto seed_bytes = std::as_bytes(std::span{seed.data(), seed.size()});
    const std::vector<std::byte> mutated = MutateBytes(rng, seed_bytes, 4096);
    const std::vector<std::byte> payload = MutateBytes(
        rng, seed_bytes.first(std::min<std::size_t>(64, seed.size())), 128);
    const std::string_view bfbs_sv{
        reinterpret_cast<const char*>(mutated.data()), mutated.size()};
    ExerciseSchemaAndPayload(bfbs_sv, payload, rng, true);

    // Unaligned slice check (offset + 1): VerifySchema must return nullptr
    // without triggering UBSan alignment faults.
    if (!mutated.empty()) {
      std::vector<char> unaligned(mutated.size() + 2);
      std::memcpy(unaligned.data() + 1, mutated.data(), mutated.size());
      EXPECT_EQ(VerifySchema({unaligned.data() + 1, mutated.size()}), nullptr);
    }
  }

  // 1,000 semantically hostile FlatBuffer schemas across VerifySchema,
  // DecodeToText, struct_hash, and field_path.
  for (int iter = 0; iter < 1000; ++iter) {
    const std::string crafted = CraftHostileSchema(rng);
    const std::array<std::byte, 64> payload{};
    ExerciseSchemaAndPayload(crafted, payload, rng, true);
  }
}

}  // namespace
}  // namespace talos::schema
