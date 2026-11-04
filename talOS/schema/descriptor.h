#pragma once

// Build-level schema descriptors of talos messages.
//
// talos_messages compiles every schema to a FlatBuffers binary schema
// (`<file>.bfbs`, a reflection.Schema) and specializes Descriptor<M> for each
// struct the file declares, in "<prefix>/<file>_descriptor.h":
//
//   #include "control/control_descriptor.h"
//   using D = talos::Descriptor<control::VelocityTarget>;
//   D::kName  // "control.VelocityTarget": the fully-qualified schema name
//   D::kHash  // talos::schema::struct_hash of that struct (schema_hash.h)
//   D::kBfbs  // the file's .bfbs bytes; reflection::GetSchema(D::kBfbs.data())
//
// kHash covers the struct's layout and meaning (field names, types, offsets,
// nested structs and enums, size and alignment), not comments or formatting:
// two same-size structs that disagree on any of those have different hashes.
// Python exposes the same values as <Struct>.TALOS_SCHEMA_NAME,
// TALOS_SCHEMA_HASH and TALOS_SCHEMA_BFBS.

#include <concepts>
#include <cstdint>
#include <span>
#include <string_view>

namespace talos {

// Specialized by talos_messages only; never by hand.
template <class M>
struct Descriptor;

// M has a generated descriptor.
template <class M>
concept Described = requires {
  { Descriptor<M>::kName } -> std::convertible_to<std::string_view>;
  { Descriptor<M>::kHash } -> std::convertible_to<std::uint64_t>;
  {
    Descriptor<M>::kBfbs
  } -> std::convertible_to<std::span<const std::uint8_t>>;
};

}  // namespace talos
