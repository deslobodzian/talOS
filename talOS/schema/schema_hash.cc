#include "talOS/schema/schema_hash.h"

#include <string>

#include "flatbuffers/hash.h"
#include "flatbuffers/verifier.h"

namespace talos::schema {
namespace {

void append_struct(const reflection::Schema& schema,
                   const reflection::Object& object, std::string& out);

// A struct field's type, or an array's element: `base` and `index` (an enum
// or struct in `schema`, or -1) as reflection::Type spells them.
void append_type(const reflection::Schema& schema, reflection::BaseType base,
                 std::int32_t index, std::string& out) {
  if (base == reflection::BaseType::Obj) {
    append_struct(schema, *schema.objects()->Get(index), out);
    return;
  }
  if (index < 0) {
    out += reflection::EnumNameBaseType(base);
    return;
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
}

void append_struct(const reflection::Schema& schema,
                   const reflection::Object& object, std::string& out) {
  out += "struct ";
  out += object.name()->str();
  out += ' ';
  out += std::to_string(object.bytesize());
  out += ' ';
  out += std::to_string(object.minalign());
  out += " {";
  for (const reflection::Field* field : *object.fields()) {
    const reflection::Type& type = *field->type();
    out += field->name()->str();
    out += '@';
    out += std::to_string(field->offset());
    out += ':';
    if (type.base_type() == reflection::BaseType::Array) {
      out += '[';
      append_type(schema, type.element(), type.index(), out);
      out += '*';
      out += std::to_string(type.fixed_length());
      out += ']';
    } else {
      append_type(schema, type.base_type(), type.index(), out);
    }
    out += ';';
  }
  out += '}';
}

}  // namespace

std::string canonical_struct(const reflection::Schema& schema,
                             const reflection::Object& object) {
  std::string out;
  append_struct(schema, object, out);
  return out;
}

std::uint64_t struct_hash(const reflection::Schema& schema,
                          const reflection::Object& object) {
  return flatbuffers::HashFnv1a<std::uint64_t>(
      canonical_struct(schema, object).c_str());
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
  return struct_hash(schema, *object);
}

}  // namespace talos::schema
