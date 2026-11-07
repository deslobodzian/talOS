#include "talOS/schema/decode.h"

#include <algorithm>
#include <charconv>
#include <cstdint>
#include <cstring>
#include <system_error>
#include <vector>

#include "flatbuffers/reflection.h"
#include "flatbuffers/verifier.h"

namespace talos::schema {
namespace {

void AppendDouble(double value, std::string& out) {
  char buffer[32];
  const auto result = std::to_chars(buffer, buffer + sizeof buffer, value);
  out.append(buffer, result.ptr);
}

void AppendFloat(float value, std::string& out) {
  char buffer[32];
  const auto result = std::to_chars(buffer, buffer + sizeof buffer, value);
  out.append(buffer, result.ptr);
}

template <class T>
std::int64_t Read(const std::uint8_t* data) {
  T value;
  std::memcpy(&value, data, sizeof value);
  return static_cast<std::int64_t>(value);
}

// An integer, bool or enum scalar of `base` at `data`.
std::int64_t ReadInteger(reflection::BaseType base, const std::uint8_t* data) {
  using reflection::BaseType;
  switch (base) {
    case BaseType::Bool:
    case BaseType::UByte:
    case BaseType::UType:
      return Read<std::uint8_t>(data);
    case BaseType::Byte:
      return Read<std::int8_t>(data);
    case BaseType::Short:
      return Read<std::int16_t>(data);
    case BaseType::UShort:
      return Read<std::uint16_t>(data);
    case BaseType::Int:
      return Read<std::int32_t>(data);
    case BaseType::UInt:
      return Read<std::uint32_t>(data);
    default:
      return Read<std::int64_t>(data);
  }
}

constexpr int kMaxDepth = 32;

bool ValueSize(const reflection::Schema& schema, reflection::BaseType base,
               std::int32_t index, std::size_t& size) {
  if (base == reflection::BaseType::Obj) {
    const auto* objects = schema.objects();
    if (objects == nullptr || index < 0 ||
        static_cast<std::size_t>(index) >= objects->size()) return false;
    const auto* object = objects->Get(index);
    if (!object->is_struct() || object->bytesize() <= 0) return false;
    size = static_cast<std::size_t>(object->bytesize());
    return true;
  }
  if (base < reflection::BaseType::UType ||
      base > reflection::BaseType::Double) return false;
  if (index >= 0 &&
      (schema.enums() == nullptr ||
       static_cast<std::size_t>(index) >= schema.enums()->size())) return false;
  size = flatbuffers::GetTypeSize(base);
  return true;
}

bool AppendStruct(const reflection::Schema& schema,
                  const reflection::Object& object, const std::uint8_t* data,
                  std::string& out, int depth);

// A scalar, enum or nested struct of `base`/`index` at `data`.
bool AppendValue(const reflection::Schema& schema, reflection::BaseType base,
                 std::int32_t index, const std::uint8_t* data,
                 std::string& out, int depth) {
  using reflection::BaseType;
  std::size_t size;
  if (!ValueSize(schema, base, index, size)) return false;
  if (base == BaseType::Obj) {
    return AppendStruct(schema, *schema.objects()->Get(index), data, out, depth);
  }
  if (base == BaseType::Float) {
    float value;
    std::memcpy(&value, data, sizeof value);
    AppendFloat(value, out);
    return true;
  }
  if (base == BaseType::Double) {
    double value;
    std::memcpy(&value, data, sizeof value);
    AppendDouble(value, out);
    return true;
  }
  if (base == BaseType::ULong) {
    std::uint64_t value;
    std::memcpy(&value, data, sizeof value);
    out += std::to_string(value);
    return true;
  }
  const std::int64_t value = ReadInteger(base, data);
  if (base == BaseType::Bool) {
    out += value != 0 ? "true" : "false";
    return true;
  }
  if (index >= 0) {
    for (const reflection::EnumVal* named :
         *schema.enums()->Get(index)->values()) {
      if (named->value() == value) {
        out += named->name()->str();
        return true;
      }
    }
  }
  out += std::to_string(value);
  return true;
}

bool AppendStruct(const reflection::Schema& schema,
                  const reflection::Object& object, const std::uint8_t* data,
                  std::string& out, int depth) {
  if (depth >= kMaxDepth || !object.is_struct() || object.bytesize() <= 0 ||
      object.fields() == nullptr) return false;
  // reflection lists fields by name; print them as declared.
  std::vector<const reflection::Field*> fields(object.fields()->begin(),
                                               object.fields()->end());
  std::sort(fields.begin(), fields.end(),
            [](const reflection::Field* a, const reflection::Field* b) {
              return a->id() < b->id();
            });
  out += '{';
  for (std::size_t i = 0; i < fields.size(); ++i) {
    const reflection::Field& field = *fields[i];
    const reflection::Type& type = *field.type();
    const bool array = type.base_type() == reflection::BaseType::Array;
    std::size_t element_bytes;
    if (!ValueSize(schema, array ? type.element() : type.base_type(),
                   type.index(), element_bytes)) return false;
    const std::size_t count = array ? type.fixed_length() : 1;
    const auto bytes = static_cast<std::size_t>(object.bytesize());
    if (count == 0 || field.offset() > bytes ||
        element_bytes > (bytes - field.offset()) / count) return false;
    if (i != 0) out += ", ";
    out += field.name()->str();
    out += ": ";
    const std::uint8_t* at = data + field.offset();
    if (type.base_type() != reflection::BaseType::Array) {
      if (!AppendValue(schema, type.base_type(), type.index(), at, out,
                       depth + 1)) return false;
      continue;
    }
    out += '[';
    for (std::uint16_t k = 0; k < type.fixed_length(); ++k) {
      if (k != 0) out += ", ";
      if (!AppendValue(schema, type.element(), type.index(),
                       at + k * element_bytes, out, depth + 1)) return false;
    }
    out += ']';
  }
  out += '}';
  return true;
}

void AppendHex(std::span<const std::byte> payload, std::string& out) {
  std::size_t stored = payload.size();
  while (stored > 0 && payload[stored - 1] == std::byte{0}) --stored;
  out += "bytes=" + std::to_string(payload.size()) + " hex=";
  static constexpr char kHex[] = "0123456789abcdef";
  const std::size_t shown = std::min(stored, kMaxHexBytes);
  for (std::size_t i = 0; i < shown; ++i) {
    const auto byte = static_cast<std::uint8_t>(payload[i]);
    out += kHex[byte >> 4];
    out += kHex[byte & 0xF];
  }
  if (shown < stored) {
    out += "...(+" + std::to_string(stored - shown) + ")";
  }
}

// `name` from `bfbs`, or why it cannot be.
void AppendMessage(const std::string& bfbs, std::string_view name,
                   std::span<const std::byte> payload, std::string& out) {
  const auto* bytes = reinterpret_cast<const std::uint8_t*>(bfbs.data());
  flatbuffers::Verifier verifier{bytes, bfbs.size()};
  if (!reflection::VerifySchemaBuffer(verifier)) {
    out += "<schema bytes are not a reflection schema>";
    return;
  }
  const reflection::Schema& schema = *reflection::GetSchema(bytes);
  const reflection::Object* object =
      schema.objects()->LookupByKey(std::string{name}.c_str());
  if (object == nullptr || !object->is_struct()) {
    out += "<schema has no struct ";
    out += name;
    out += '>';
    return;
  }
  if (payload.size() != static_cast<std::size_t>(object->bytesize())) {
    out += "<" + std::to_string(payload.size()) + " bytes, " +
           std::string{name} + " is " + std::to_string(object->bytesize()) +
           ">";
    return;
  }
  const std::size_t start = out.size();
  if (!AppendStruct(schema, *object,
                    reinterpret_cast<const std::uint8_t*>(payload.data()),
                    out, 0)) {
    out.resize(start);
    out += "<invalid struct schema>";
  }
}

}  // namespace

std::string DecodeToText(const Identity& identity,
                         std::span<const std::byte> payload) {
  std::string out;
  if (identity.bfbs.empty()) {
    AppendHex(payload, out);
    return out;
  }
  const std::string_view inner = EnvelopeInner(identity.name);
  if (inner.empty()) {
    AppendMessage(identity.bfbs, identity.name, payload, out);
    return out;
  }
  if (payload.size() < kEnvelopeHeaderBytes) {
    out += "<" + std::to_string(payload.size()) +
           " bytes, shorter than a target envelope header>";
    return out;
  }
  std::uint64_t mode_since_us;
  std::memcpy(&mode_since_us, payload.data(), sizeof mode_since_us);
  out += "{mode_since_us: " + std::to_string(mode_since_us) + ", active: ";
  out += payload[kEnvelopeActiveOffset] != std::byte{0} ? "true" : "false";
  out += ", message: ";
  AppendMessage(identity.bfbs, inner, payload.subspan(kEnvelopeHeaderBytes),
                out);
  out += '}';
  return out;
}

}  // namespace talos::schema
