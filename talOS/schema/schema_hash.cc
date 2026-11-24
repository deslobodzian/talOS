#include "talOS/schema/schema_hash.h"

#include <cstddef>
#include <string>
#include <vector>

#include "flatbuffers/hash.h"
#include "flatbuffers/reflection.h"
#include "talOS/schema/verify.h"

namespace talos::schema {
namespace {

constexpr int kMaxDepth = 32;
constexpr std::size_t kMaxStructVisits = 4096;

bool append_struct(const reflection::Schema& schema,
                   const reflection::Object& object, std::string& out,
                   int depth, std::vector<bool>& open, std::size_t& visits);

// A struct field's type, or an array's element: `base` and `index` (an enum
// or struct in `schema`, or -1) as reflection::Type spells them.
bool append_type(const reflection::Schema& schema, reflection::BaseType base,
                 std::int32_t index, std::string& out, int depth,
                 std::vector<bool>& open, std::size_t& visits) {
  std::size_t size;
  if (!ValueSize(schema, base, index, size)) return false;
  if (base == reflection::BaseType::Obj) {
    const auto uidx = static_cast<std::size_t>(index);
    if (uidx >= open.size() || open[uidx]) return false;
    open[uidx] = true;
    const bool ok = append_struct(schema, *schema.objects()->Get(index), out,
                                  depth, open, visits);
    open[uidx] = false;
    return ok;
  }
  if (index < 0) {
    out += reflection::EnumNameBaseType(base);
    return true;
  }
  if (!IsEnumBaseType(base)) return false;
  const reflection::Enum* e = schema.enums()->Get(index);
  if (e == nullptr || e->name() == nullptr || e->values() == nullptr)
    return false;
  out += "enum ";
  out += e->name()->str();
  out += ':';
  out += reflection::EnumNameBaseType(base);
  out += " {";
  for (const reflection::EnumVal* value : *e->values()) {
    if (value == nullptr || value->name() == nullptr) return false;
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
                   int depth, std::vector<bool>& open, std::size_t& visits) {
  if (depth >= kMaxDepth || visits >= kMaxStructVisits || !object.is_struct() ||
      object.bytesize() <= 0 || object.name() == nullptr ||
      object.fields() == nullptr)
    return false;
  ++visits;
  out += "struct ";
  out += object.name()->str();
  out += ' ';
  out += std::to_string(object.bytesize());
  out += ' ';
  out += std::to_string(object.minalign());
  out += " {";
  for (const reflection::Field* field : *object.fields()) {
    if (field == nullptr || field->type() == nullptr ||
        field->name() == nullptr)
      return false;
    const reflection::Type& type = *field->type();
    const bool array = type.base_type() == reflection::BaseType::Array;
    std::size_t size;
    if (!ValueSize(schema, array ? type.element() : type.base_type(),
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
      if (!append_type(schema, type.element(), type.index(), out, depth + 1,
                       open, visits)) {
        return false;
      }
      out += '*';
      out += std::to_string(type.fixed_length());
      out += ']';
    } else {
      if (!append_type(schema, type.base_type(), type.index(), out, depth + 1,
                       open, visits)) {
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
  const std::size_t num_objects =
      schema.objects() != nullptr ? schema.objects()->size() : 0;
  std::vector<bool> open(num_objects, false);
  if (schema.objects() != nullptr) {
    for (flatbuffers::uoffset_t i = 0; i < schema.objects()->size(); ++i) {
      if (schema.objects()->Get(i) == &object) {
        open[i] = true;
        break;
      }
    }
  }
  std::size_t visits = 0;
  if (!append_struct(schema, object, out, 0, open, visits)) return {};
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
  const reflection::Schema* verified =
      VerifySchema({reinterpret_cast<const char*>(bfbs), size});
  if (verified == nullptr) return std::nullopt;
  const reflection::Schema& schema = *verified;
  const reflection::Object* object =
      schema.objects() != nullptr
          ? schema.objects()->LookupByKey(std::string(name).c_str())
          : nullptr;
  if (object == nullptr || !object->is_struct()) return std::nullopt;
  const std::string canonical = canonical_struct(schema, *object);
  if (canonical.empty()) return std::nullopt;
  return flatbuffers::HashFnv1a<std::uint64_t>(canonical.c_str());
}

}  // namespace talos::schema
