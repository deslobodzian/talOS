#pragma once

// A message's fields by path, from its struct's reflection schema
// (flatbuffers/reflection.h over a .bfbs): what a node configured in TOML
// uses to write a field of a message type it was not compiled against
// (teleop's assignments) or to read one (telemetry's channels).
//
//   path         "position_m", "inner/offset_m": field names joined by '/',
//                each but the last naming a nested struct
//   resolve      ResolveField: the leaf scalar's byte offset in the message,
//                base type, size and enum, or std::invalid_argument naming
//                the path segment and what it expected
//   write        EncodeField (bool, integer, float or enum label, with the
//                field's range checked) and EncodeNumber
//   read         ReadDouble, ReadInt64, ReadBool: inline, allocation-free,
//                for a per-tick call on a resolved field
//
// Resolving and EncodeField allocate and throw (construction time); the
// reads, and EncodeNumber of a value FitsFloatingPoint accepts into a
// floating-point field, do neither (per tick).

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "flatbuffers/reflection.h"

namespace talos::schema {

// A value from configuration: a bool, an integer, a float or an enum label.
using FieldValue = std::variant<bool, std::int64_t, double, std::string>;

struct ResolvedField {
  // Byte offset of the scalar from the start of the root struct.
  std::size_t offset{0};
  reflection::BaseType base_type{reflection::BaseType::None};
  // ScalarSize(base_type).
  std::size_t size{0};
  // The field's enum, or null for a plain scalar.
  const reflection::Enum* enum_def{nullptr};
  std::string leaf_name;
  std::string enclosing_name;
};

// The schema in `bfbs` once it verifies as a reflection.Schema, else null.
const reflection::Schema* VerifySchema(std::string_view bfbs);

// Bytes of a scalar base type (Bool through Double, UType included); 0 for
// anything else, so also the test for a scalar.
inline std::size_t ScalarSize(reflection::BaseType type) {
  using reflection::BaseType;
  switch (type) {
    case BaseType::UType:
    case BaseType::Bool:
    case BaseType::Byte:
    case BaseType::UByte:
      return 1;
    case BaseType::Short:
    case BaseType::UShort:
      return 2;
    case BaseType::Int:
    case BaseType::UInt:
    case BaseType::Float:
      return 4;
    case BaseType::Long:
    case BaseType::ULong:
    case BaseType::Double:
      return 8;
    default:
      return 0;
  }
}

// Resolves `field_path` against `root`, a struct of `schema`. Every error is
// std::invalid_argument prefixed with `where`: an empty path, a segment
// `root` (or the struct before it) has no field of, a non-struct segment
// before the last, or a last segment that is not a scalar.
ResolvedField ResolveField(const reflection::Schema& schema,
                           const reflection::Object& root,
                           std::string_view field_path, std::string_view where);

// The path of every field under `root` that is not itself a nested struct,
// in declaration order, nested structs expanded in place ("inner/mode").
std::vector<std::string> ScalarPaths(const reflection::Schema& schema,
                                     const reflection::Object& root);

// Writes `value` into `message` at `field` (message must hold at least
// field.offset + field.size bytes). A bool field takes a bool, an enum field
// one of its labels, any other field a number that fits it: finite and in
// range for a float, integral and in range for an integer. Otherwise throws
// std::invalid_argument prefixed with `where`.
void EncodeField(std::byte* message, const ResolvedField& field,
                 const FieldValue& value, std::string_view where);

// Whether `value` is finite and, for a Float `type`, within float's range.
bool FitsFloatingPoint(reflection::BaseType type, double value);

// Writes `value` at `dest` as `type` (a numeric scalar): floats must be
// finite and fit; integers must be integral and in range. Otherwise throws
// std::invalid_argument prefixed with `where`.
void EncodeNumber(std::byte* dest, reflection::BaseType type, double value,
                  std::string_view where = "");

namespace detail {
template <class T>
T Load(const std::byte* data) {
  T value;
  std::memcpy(&value, data, sizeof value);
  return value;
}
}  // namespace detail

// The reads below take a resolved field's type and its bytes (message +
// offset); they never allocate or throw.

// An integer or Bool scalar as its exact value (Bool as 0 or 1); a ULong
// above INT64_MAX keeps its bits (static_cast<std::uint64_t> restores it).
// 0 for a floating-point or non-scalar type.
inline std::int64_t ReadInt64(reflection::BaseType type,
                              const std::byte* data) {
  using reflection::BaseType;
  switch (type) {
    case BaseType::Bool:
      return detail::Load<std::uint8_t>(data) != 0 ? 1 : 0;
    case BaseType::UType:
    case BaseType::UByte:
      return detail::Load<std::uint8_t>(data);
    case BaseType::Byte:
      return detail::Load<std::int8_t>(data);
    case BaseType::Short:
      return detail::Load<std::int16_t>(data);
    case BaseType::UShort:
      return detail::Load<std::uint16_t>(data);
    case BaseType::Int:
      return detail::Load<std::int32_t>(data);
    case BaseType::UInt:
      return detail::Load<std::uint32_t>(data);
    case BaseType::Long:
      return detail::Load<std::int64_t>(data);
    case BaseType::ULong:
      return static_cast<std::int64_t>(detail::Load<std::uint64_t>(data));
    default:
      return 0;
  }
}

// Any scalar as a double: Bool as 0 or 1, integers converted (rounded past
// 2^53), floats widened (a NaN or infinity stays one). 0 for a non-scalar.
inline double ReadDouble(reflection::BaseType type, const std::byte* data) {
  using reflection::BaseType;
  switch (type) {
    case BaseType::Float:
      return detail::Load<float>(data);
    case BaseType::Double:
      return detail::Load<double>(data);
    case BaseType::ULong:
      return static_cast<double>(detail::Load<std::uint64_t>(data));
    default:
      return static_cast<double>(ReadInt64(type, data));
  }
}

// Any scalar as a bool: nonzero (a NaN is nonzero). false for a non-scalar.
inline bool ReadBool(reflection::BaseType type, const std::byte* data) {
  using reflection::BaseType;
  if (type == BaseType::Float || type == BaseType::Double) {
    return ReadDouble(type, data) != 0.0;
  }
  return ReadInt64(type, data) != 0;
}

}  // namespace talos::schema
