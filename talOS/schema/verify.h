#pragma once

// Verified FlatBuffers reflection schema loading.
//
// Header-only so targets inside libtalos_node.so (such as
// //talOS/introspection:names) can share it without adding a compiled object
// to the shared-node link closure.

#include <cstdint>
#include <string_view>

#include "flatbuffers/reflection.h"
#include "flatbuffers/table.h"
#include "flatbuffers/verifier.h"

namespace talos::schema {

namespace detail {

inline bool ValidBaseTypeByte(std::int8_t raw) {
  return raw >= static_cast<std::int8_t>(reflection::BaseType::None) &&
         raw <= static_cast<std::int8_t>(reflection::BaseType::MaxBaseType);
}

inline bool ValidType(const reflection::Type& type) {
  // Read the raw int8_t bytes directly from the verified Table before any
  // generated accessor casts them to the unscoped enum reflection::BaseType
  // (whose valid C++ value range does not cover all int8_t values).
  static_assert(sizeof(reflection::Type) == sizeof(flatbuffers::Table));
  const auto* table = reinterpret_cast<const flatbuffers::Table*>(&type);
  return ValidBaseTypeByte(
             table->GetField<std::int8_t>(reflection::Type::VT_BASE_TYPE, 0)) &&
         ValidBaseTypeByte(
             table->GetField<std::int8_t>(reflection::Type::VT_ELEMENT, 0));
}

inline bool ValidObject(const reflection::Object& object) {
  if (object.fields() == nullptr) return true;
  for (const auto* field : *object.fields()) {
    if (field != nullptr && field->type() != nullptr &&
        !ValidType(*field->type())) {
      return false;
    }
  }
  return true;
}

}  // namespace detail

// The schema in `bfbs` once it verifies as a reflection.Schema and every
// reachable reflection::Type (in objects, root_table, enums/union_type, and
// services) has base_type and element bytes within reflection::BaseType's
// defined range, else null.
inline const reflection::Schema* VerifySchema(std::string_view bfbs) {
  flatbuffers::Verifier verifier{
      reinterpret_cast<const std::uint8_t*>(bfbs.data()), bfbs.size()};
  if (!reflection::VerifySchemaBuffer(verifier)) return nullptr;
  const reflection::Schema* schema = reflection::GetSchema(bfbs.data());
  if (schema->objects() != nullptr) {
    for (const auto* object : *schema->objects()) {
      if (object != nullptr && !detail::ValidObject(*object)) return nullptr;
    }
  }
  if (schema->root_table() != nullptr &&
      !detail::ValidObject(*schema->root_table())) {
    return nullptr;
  }
  if (schema->enums() != nullptr) {
    for (const auto* enum_def : *schema->enums()) {
      if (enum_def == nullptr) continue;
      if (enum_def->underlying_type() != nullptr &&
          !detail::ValidType(*enum_def->underlying_type())) {
        return nullptr;
      }
      if (enum_def->values() != nullptr) {
        for (const auto* val : *enum_def->values()) {
          if (val != nullptr && val->union_type() != nullptr &&
              !detail::ValidType(*val->union_type())) {
            return nullptr;
          }
        }
      }
    }
  }
  if (schema->services() != nullptr) {
    for (const auto* service : *schema->services()) {
      if (service == nullptr || service->calls() == nullptr) continue;
      for (const auto* call : *service->calls()) {
        if (call == nullptr) continue;
        if (call->request() != nullptr &&
            !detail::ValidObject(*call->request())) {
          return nullptr;
        }
        if (call->response() != nullptr &&
            !detail::ValidObject(*call->response())) {
          return nullptr;
        }
      }
    }
  }
  return schema;
}

}  // namespace talos::schema
