#include "talOS/schema/decode.h"

#include <algorithm>
#include <charconv>
#include <cstdint>
#include <cstring>
#include <system_error>
#include <vector>

#include "flatbuffers/reflection.h"
#include "talOS/schema/verify.h"

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
constexpr std::size_t kMaxStructVisits = 4096;
constexpr std::size_t kMaxElements = 16384;

bool AppendStruct(const reflection::Schema& schema,
                  const reflection::Object& object, const std::uint8_t* data,
                  std::string& out, int depth, std::vector<bool>& open,
                  std::size_t& visits, std::size_t& elements);

// A scalar, enum or nested struct of `base`/`index` at `data`.
bool AppendValue(const reflection::Schema& schema, reflection::BaseType base,
                 std::int32_t index, const std::uint8_t* data, std::string& out,
                 int depth, std::vector<bool>& open, std::size_t& visits,
                 std::size_t& elements) {
  using reflection::BaseType;
  std::size_t size;
  if (!ValueSize(schema, base, index, size)) return false;
  if (base == BaseType::Obj) {
    const auto uidx = static_cast<std::size_t>(index);
    if (uidx >= open.size() || open[uidx]) return false;
    open[uidx] = true;
    const bool ok = AppendStruct(schema, *schema.objects()->Get(index), data,
                                 out, depth, open, visits, elements);
    open[uidx] = false;
    return ok;
  }
  if (elements >= kMaxElements) return false;
  ++elements;
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
    if (index >= 0) {
      const reflection::Enum* enum_def = schema.enums()->Get(index);
      if (enum_def != nullptr && enum_def->values() != nullptr) {
        for (const reflection::EnumVal* named : *enum_def->values()) {
          if (named != nullptr && named->name() != nullptr &&
              static_cast<std::uint64_t>(named->value()) == value) {
            out += named->name()->str();
            return true;
          }
        }
      }
    }
    out += std::to_string(value);
    return true;
  }
  const std::int64_t value = ReadInteger(base, data);
  if (base == BaseType::Bool) {
    out += value != 0 ? "true" : "false";
    return true;
  }
  if (index >= 0) {
    const reflection::Enum* enum_def = schema.enums()->Get(index);
    if (enum_def != nullptr && enum_def->values() != nullptr) {
      for (const reflection::EnumVal* named : *enum_def->values()) {
        if (named != nullptr && named->name() != nullptr &&
            named->value() == value) {
          out += named->name()->str();
          return true;
        }
      }
    }
  }
  out += std::to_string(value);
  return true;
}

bool AppendStruct(const reflection::Schema& schema,
                  const reflection::Object& object, const std::uint8_t* data,
                  std::string& out, int depth, std::vector<bool>& open,
                  std::size_t& visits, std::size_t& elements) {
  if (depth >= kMaxDepth || visits >= kMaxStructVisits || !object.is_struct() ||
      object.bytesize() <= 0 || object.fields() == nullptr)
    return false;
  ++visits;
  // reflection lists fields by name; print them as declared.
  std::vector<const reflection::Field*> fields(object.fields()->begin(),
                                               object.fields()->end());
  for (const reflection::Field* field : fields) {
    if (field == nullptr || field->type() == nullptr ||
        field->name() == nullptr)
      return false;
  }
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
                   type.index(), element_bytes))
      return false;
    const std::size_t count = array ? type.fixed_length() : 1;
    const auto bytes = static_cast<std::size_t>(object.bytesize());
    if (count == 0 || field.offset() > bytes ||
        element_bytes > (bytes - field.offset()) / count ||
        count > kMaxElements - elements)
      return false;
    if (i != 0) out += ", ";
    out += field.name()->str();
    out += ": ";
    const std::uint8_t* at = data + field.offset();
    if (type.base_type() != reflection::BaseType::Array) {
      if (!AppendValue(schema, type.base_type(), type.index(), at, out,
                       depth + 1, open, visits, elements))
        return false;
      continue;
    }
    out += '[';
    for (std::uint16_t k = 0; k < type.fixed_length(); ++k) {
      if (k != 0) out += ", ";
      if (!AppendValue(schema, type.element(), type.index(),
                       at + k * element_bytes, out, depth + 1, open, visits,
                       elements))
        return false;
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
  const reflection::Schema* verified = VerifySchema(bfbs);
  if (verified == nullptr) {
    out += "<schema bytes are not a reflection schema>";
    return;
  }
  const reflection::Schema& schema = *verified;
  const reflection::Object* object =
      schema.objects() != nullptr
          ? schema.objects()->LookupByKey(std::string{name}.c_str())
          : nullptr;
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
  const std::size_t num_objects =
      schema.objects() != nullptr ? schema.objects()->size() : 0;
  std::vector<bool> open(num_objects, false);
  if (schema.objects() != nullptr) {
    for (flatbuffers::uoffset_t i = 0; i < schema.objects()->size(); ++i) {
      if (schema.objects()->Get(i) == object) {
        open[i] = true;
        break;
      }
    }
  }
  std::size_t visits = 0;
  std::size_t elements = 0;
  if (!AppendStruct(schema, *object,
                    reinterpret_cast<const std::uint8_t*>(payload.data()), out,
                    0, open, visits, elements)) {
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
