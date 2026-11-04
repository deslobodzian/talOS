#pragma once

// The schema hash of a FlatBuffers struct, from its reflection object.
//
// The hash is flatbuffers::HashFnv1a<uint64_t> (flatbuffers/hash.h: FNV-1a
// with prime 0x100000001b3 and that header's offset basis,
// 0xcbf29ce484222645) of a canonical text:
//
//   struct  := "struct " NAME " " BYTESIZE " " MINALIGN " {" field* "}"
//   field   := NAME "@" OFFSET ":" type ";"
//   type    := struct                                  (base type Obj)
//            | "[" element "*" FIXED_LENGTH "]"        (base type Array)
//            | "enum " NAME ":" BASE " {" (NAME "=" VALUE ",")* "}"
//            | BASE                                    (any other scalar)
//
// NAME is the fully-qualified schema name, BASE the reflection::BaseType
// name (Double, UByte, ...), numbers are decimal, and fields and enum values
// are taken in the order the .bfbs stores them (by name and by value).
// Nested structs and enums expand recursively, so the hash names everything
// a reader must agree on and nothing it need not: documentation, attributes
// and declaration files are left out. talos_schema.struct_hash (Python)
// computes the same value from the same bytes.
//
// C++17, like unit_suffix.h: talos_messages' generator links this in Bazel's
// exec configuration.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

#include "flatbuffers/reflection_generated.h"

namespace talos::schema {

// The canonical text above, for `object` (a struct) of `schema`.
std::string canonical_struct(const reflection::Schema& schema,
                             const reflection::Object& object);

// FNV-1a 64 of canonical_struct(schema, object).
std::uint64_t struct_hash(const reflection::Schema& schema,
                          const reflection::Object& object);

// The hash of struct `name` ("control.VelocityTarget") in `size` .bfbs
// bytes, or nullopt when the bytes are not a valid reflection.Schema or do
// not hold a struct of that name.
std::optional<std::uint64_t> struct_hash(const std::uint8_t* bfbs,
                                         std::size_t size,
                                         std::string_view name);

}  // namespace talos::schema
