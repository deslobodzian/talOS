#include "talOS/schema/schema_hash.h"

#include <string>

#include "flatbuffers/hash.h"
#include "flatbuffers/reflection.h"
#include "flatbuffers/verifier.h"

namespace talos::schema {
namespace {

constexpr int kMaxDepth = 32;

// Semantic validation complements the reflection buffer's structural verifier.
bool type_size(const reflection::Schema& schema, reflection::BaseType base,
               std::int32_t index, std::size_t& size) {
  if (base == reflection::BaseType::Obj) {
    const auto* objects = schema.objects();
    if (objects == nullptr || index < 0 ||
        static_cast<std::size_t>(index) >= objects->size())
      return false;
    const auto* object = objects->Get(index);
    if (!object->is_struct() || object->bytesize() <= 0) return false;
    size = static_cast<std::size_t>(object->bytesize());
    return true;
  }
  if (base < reflection::BaseType::UType || base > reflection::BaseType::Double)
    return false;
  if (index >= 0 && (schema.enums() == nullptr ||
                     static_cast<std::size_t>(index) >= schema.enums()->size()))
    return false;
  size = flatbuffers::GetTypeSize(base);
  return true;
}

bool append_struct(const reflection::Schema& schema,
                   const reflection::Object& object, std::string& out,
                   int depth);

// A struct field's type, or an array's element: `base` and `index` (an enum
// or struct in `schema`, or -1) as reflection::Type spells them.
bool append_type(const reflection::Schema& schema, reflection::BaseType base,
                 std::int32_t index, std::string& out, int depth) {
  std::size_t size;
  if (!type_size(schema, base, index, size)) return false;
  if (base == reflection::BaseType::Obj) {
    return append_struct(schema, *schema.objects()->Get(index), out, depth);
  }
  if (index < 0) {
    out += reflection::EnumNameBaseType(base);
    return true;
  }
  const reflection::Enum& e = *schema.enums()->Get(index);
  out += "enum ";
  out += e.name()->str();
  out += ':';
  out += reflection::EnumNameBaseType(base);
  out += " {";
  for (const reflection::EnumVal* value : *e.values()) {
    out += value->name()->str();
    out += '=';
    out += std::to_string(value->value());
    out += ',';
  }
  out += '}';
  return true;
}

bool append_struct(const reflection::Schema& schema,
                   const reflection::Object& object, std::string& out,
                   int depth) {
  if (depth >= kMaxDepth || !object.is_struct() || object.bytesize() <= 0 ||
      object.fields() == nullptr)
    return false;
  out += "struct ";
  out += object.name()->str();
  out += ' ';
  out += std::to_string(object.bytesize());
  out += ' ';
  out += std::to_string(object.minalign());
  out += " {";
  for (const reflection::Field* field : *object.fields()) {
    const reflection::Type& type = *field->type();
    const bool array = type.base_type() == reflection::BaseType::Array;
    std::size_t size;
    if (!type_size(schema, array ? type.element() : type.base_type(),
                   type.index(), size))
      return false;
    const std::size_t count = array ? type.fixed_length() : 1;
    const auto bytes = static_cast<std::size_t>(object.bytesize());
    if (count == 0 || field->offset() > bytes ||
        size > (bytes - field->offset()) / count)
      return false;
    out += field->name()->str();
    out += '@';
    out += std::to_string(field->offset());
    out += ':';
    if (type.base_type() == reflection::BaseType::Array) {
      out += '[';
      if (!append_type(schema, type.element(), type.index(), out, depth + 1)) {
        return false;
      }
      out += '*';
      out += std::to_string(type.fixed_length());
      out += ']';
    } else {
      if (!append_type(schema, type.base_type(), type.index(), out,
                       depth + 1)) {
        return false;
      }
    }
    out += ';';
  }
  out += '}';
  return true;
}

}  // namespace

std::string canonical_struct(const reflection::Schema& schema,
                             const reflection::Object& object) {
  std::string out;
  if (!append_struct(schema, object, out, 0)) return {};
  return out;
}

std::uint64_t struct_hash(const reflection::Schema& schema,
                          const reflection::Object& object) {
  const std::string canonical = canonical_struct(schema, object);
  if (canonical.empty()) return 0;
  return flatbuffers::HashFnv1a<std::uint64_t>(canonical.c_str());
}

std::optional<std::uint64_t> struct_hash(const std::uint8_t* bfbs,
                                         std::size_t size,
                                         std::string_view name) {
  flatbuffers::Verifier verifier(bfbs, size);
  if (!reflection::VerifySchemaBuffer(verifier)) return std::nullopt;
  const reflection::Schema& schema = *reflection::GetSchema(bfbs);
  const reflection::Object* object =
      schema.objects()->LookupByKey(std::string(name).c_str());
  if (object == nullptr || !object->is_struct()) return std::nullopt;
  const std::string canonical = canonical_struct(schema, *object);
  if (canonical.empty()) return std::nullopt;
  return flatbuffers::HashFnv1a<std::uint64_t>(canonical.c_str());
}

}  // namespace talos::schema
