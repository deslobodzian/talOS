#include "talOS/schema/field_path.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

#include "flatbuffers/reflection.h"

namespace talos::schema {
namespace {

using reflection::BaseType;

template <typename T>
void CheckAndCopyInt(std::byte* dest, std::int64_t val,
                     std::string_view where) {
  if (val < static_cast<std::int64_t>(std::numeric_limits<T>::lowest()) ||
      val > static_cast<std::int64_t>(std::numeric_limits<T>::max())) {
    throw std::invalid_argument(std::string{where} + ": value " +
                                std::to_string(val) +
                                " does not fit integer field");
  }
  auto v = static_cast<T>(val);
  std::memcpy(dest, &v, sizeof(v));
}

void EncodeInteger(std::byte* dest, BaseType type, std::int64_t val,
                   std::string_view where) {
  switch (type) {
    case BaseType::Byte:
      CheckAndCopyInt<std::int8_t>(dest, val, where);
      break;
    case BaseType::UType:
    case BaseType::UByte:
      CheckAndCopyInt<std::uint8_t>(dest, val, where);
      break;
    case BaseType::Short:
      CheckAndCopyInt<std::int16_t>(dest, val, where);
      break;
    case BaseType::UShort:
      CheckAndCopyInt<std::uint16_t>(dest, val, where);
      break;
    case BaseType::Int:
      CheckAndCopyInt<std::int32_t>(dest, val, where);
      break;
    case BaseType::UInt:
      CheckAndCopyInt<std::uint32_t>(dest, val, where);
      break;
    case BaseType::Long:
      std::memcpy(dest, &val, sizeof(val));
      break;
    case BaseType::ULong: {
      if (val < 0) {
        throw std::invalid_argument(std::string{where} +
                                    ": negative value for unsigned field");
      }
      auto v = static_cast<std::uint64_t>(val);
      std::memcpy(dest, &v, sizeof(v));
      break;
    }
    default:
      throw std::invalid_argument(std::string{where} +
                                  ": field is not an integer type");
  }
}

constexpr int kMaxDepth = 32;
constexpr std::size_t kMaxStructVisits = 4096;

std::string ObjectName(const reflection::Object& object) {
  return object.name() != nullptr ? object.name()->str() : "(unnamed)";
}

void AppendScalarPaths(const reflection::Schema& schema,
                       const reflection::Object& object,
                       const std::string& prefix,
                       std::vector<std::string>& paths, int depth,
                       std::vector<bool>& open, std::size_t& visits) {
  if (depth >= kMaxDepth || !object.is_struct() || object.fields() == nullptr ||
      visits >= kMaxStructVisits) {
    return;
  }
  ++visits;
  std::vector<const reflection::Field*> fields;
  fields.reserve(object.fields()->size());
  for (const auto* field : *object.fields()) {
    if (field == nullptr || field->name() == nullptr ||
        field->type() == nullptr) {
      continue;
    }
    fields.push_back(field);
  }
  std::sort(fields.begin(), fields.end(),
            [](const reflection::Field* a, const reflection::Field* b) {
              return a->id() < b->id();
            });
  for (const auto* field : fields) {
    if (visits >= kMaxStructVisits) return;
    const std::string path = prefix + field->name()->str();
    if (field->type()->base_type() == BaseType::Obj) {
      const auto* objects = schema.objects();
      const std::int32_t idx = field->type()->index();
      if (objects == nullptr || idx < 0 ||
          static_cast<std::size_t>(idx) >= objects->size()) {
        continue;
      }
      const auto uidx = static_cast<std::size_t>(idx);
      if (open[uidx]) continue;
      const auto* next = objects->Get(idx);
      if (next == nullptr || !next->is_struct()) continue;
      open[uidx] = true;
      AppendScalarPaths(schema, *next, path + "/", paths, depth + 1, open,
                        visits);
      open[uidx] = false;
    } else {
      paths.push_back(path);
    }
  }
}

}  // namespace

ResolvedField ResolveField(const reflection::Schema& schema,
                           const reflection::Object& root,
                           std::string_view field_path,
                           std::string_view where) {
  if (field_path.empty()) {
    throw std::invalid_argument(std::string{where} + ": empty field path");
  }
  if (field_path.front() == '/' || field_path.back() == '/') {
    throw std::invalid_argument(std::string{where} + ": invalid field path '" +
                                std::string{field_path} + "'");
  }
  if (!root.is_struct() || root.bytesize() <= 0) {
    throw std::invalid_argument(std::string{where} + ": '" + ObjectName(root) +
                                "' is not a positive-sized struct");
  }
  const reflection::Object* curr_obj = &root;
  std::size_t curr_offset = 0;
  std::vector<std::string> segments;
  std::size_t pos = 0;
  while (pos < field_path.size()) {
    auto next = field_path.find('/', pos);
    if (next == std::string_view::npos) {
      segments.emplace_back(field_path.substr(pos));
      break;
    }
    const std::string_view segment = field_path.substr(pos, next - pos);
    if (segment.empty()) {
      throw std::invalid_argument(std::string{where} +
                                  ": invalid field path '" +
                                  std::string{field_path} + "'");
    }
    segments.emplace_back(segment);
    pos = next + 1;
  }

  for (std::size_t s = 0; s < segments.size(); ++s) {
    const std::string& seg = segments[s];
    if (curr_obj->fields() == nullptr) {
      throw std::invalid_argument(std::string{where} + ": no fields in " +
                                  ObjectName(*curr_obj));
    }
    const reflection::Field* field = nullptr;
    for (const auto* f : *curr_obj->fields()) {
      if (f != nullptr && f->name() != nullptr && f->name()->str() == seg) {
        field = f;
        break;
      }
    }
    if (field == nullptr) {
      throw std::invalid_argument(std::string{where} + ": no field '" + seg +
                                  "' in " + ObjectName(*curr_obj));
    }
    if (field->type() == nullptr) {
      throw std::invalid_argument(std::string{where} + ": field '" + seg +
                                  "' has no type");
    }
    const auto curr_bytes = static_cast<std::size_t>(curr_obj->bytesize());
    if (s + 1 < segments.size()) {
      if (field->type()->base_type() != BaseType::Obj) {
        throw std::invalid_argument(std::string{where} + ": '" + seg +
                                    "' is not a nested struct");
      }
      const auto* objects = schema.objects();
      const std::int32_t idx = field->type()->index();
      if (objects == nullptr || idx < 0 ||
          static_cast<std::size_t>(idx) >= objects->size()) {
        throw std::invalid_argument(std::string{where} + ": '" + seg +
                                    "' has invalid struct index");
      }
      const auto* next_obj = objects->Get(idx);
      if (next_obj == nullptr || !next_obj->is_struct() ||
          next_obj->bytesize() <= 0) {
        throw std::invalid_argument(std::string{where} + ": '" + seg +
                                    "' is not a struct");
      }
      const auto next_bytes = static_cast<std::size_t>(next_obj->bytesize());
      if (field->offset() > curr_bytes ||
          next_bytes > curr_bytes - field->offset()) {
        throw std::invalid_argument(std::string{where} + ": field '" + seg +
                                    "' exceeds enclosing struct bounds");
      }
      curr_offset += field->offset();
      curr_obj = next_obj;
      continue;
    }
    const auto base = field->type()->base_type();
    const std::size_t size = ScalarSize(base);
    if (size == 0) {
      throw std::invalid_argument(std::string{where} + ": field '" + seg +
                                  "' must be a scalar");
    }
    if (field->offset() > curr_bytes || size > curr_bytes - field->offset()) {
      throw std::invalid_argument(std::string{where} + ": field '" + seg +
                                  "' exceeds enclosing struct bounds");
    }
    const reflection::Enum* enum_def = nullptr;
    const std::int32_t enum_idx = field->type()->index();
    if (enum_idx >= 0) {
      if (!IsEnumBaseType(base)) {
        throw std::invalid_argument(std::string{where} + ": field '" + seg +
                                    "' has enum index on non-integer scalar");
      }
      const auto* enums = schema.enums();
      if (enums == nullptr ||
          static_cast<std::size_t>(enum_idx) >= enums->size()) {
        throw std::invalid_argument(std::string{where} + ": field '" + seg +
                                    "' has invalid enum index");
      }
      enum_def = enums->Get(enum_idx);
      if (enum_def == nullptr) {
        throw std::invalid_argument(std::string{where} + ": field '" + seg +
                                    "' has invalid enum index");
      }
    }
    return ResolvedField{
        .offset = curr_offset + field->offset(),
        .base_type = base,
        .size = size,
        .enum_def = enum_def,
        .leaf_name = seg,
        .enclosing_name = ObjectName(*curr_obj),
    };
  }
  throw std::logic_error("unreachable");
}

std::vector<std::string> ScalarPaths(const reflection::Schema& schema,
                                     const reflection::Object& root) {
  std::vector<std::string> paths;
  const std::size_t num_objects =
      schema.objects() != nullptr ? schema.objects()->size() : 0;
  std::vector<bool> open(num_objects, false);
  if (schema.objects() != nullptr) {
    for (flatbuffers::uoffset_t i = 0; i < schema.objects()->size(); ++i) {
      if (schema.objects()->Get(i) == &root) {
        open[i] = true;
        break;
      }
    }
  }
  std::size_t visits = 0;
  AppendScalarPaths(schema, root, "", paths, 0, open, visits);
  return paths;
}

void EncodeField(std::byte* message, const ResolvedField& field,
                 const FieldValue& value, std::string_view where) {
  std::byte* const dest = message + field.offset;
  const std::string at{where};
  if (field.base_type == BaseType::Bool) {
    if (!std::holds_alternative<bool>(value)) {
      throw std::invalid_argument(at + ": field '" + field.leaf_name +
                                  "' is bool but value is not a boolean");
    }
    *dest = static_cast<std::byte>(std::get<bool>(value) ? 1 : 0);
  } else if (field.enum_def != nullptr) {
    if (!std::holds_alternative<std::string>(value)) {
      throw std::invalid_argument(at + ": field '" + field.leaf_name +
                                  "' is enum but value is not a string");
    }
    const std::string& label = std::get<std::string>(value);
    const reflection::EnumVal* ev = nullptr;
    if (field.enum_def->values() != nullptr) {
      for (const auto* val : *field.enum_def->values()) {
        if (val != nullptr && val->name() != nullptr &&
            val->name()->str() == label) {
          ev = val;
          break;
        }
      }
    }
    if (ev == nullptr) {
      const std::string enum_name = field.enum_def->name() != nullptr
                                        ? field.enum_def->name()->str()
                                        : "(unnamed)";
      throw std::invalid_argument(at + ": unknown enum label '" + label +
                                  "' in " + enum_name);
    }
    if (field.base_type == BaseType::ULong) {
      // FlatBuffers reflection stores ULong enum enumerators as raw two's
      // complement bit patterns in signed int64_t, so values >= 2^63 are
      // negative in ev->value().
      const auto v = static_cast<std::uint64_t>(ev->value());
      std::memcpy(dest, &v, sizeof(v));
    } else {
      EncodeInteger(dest, field.base_type, ev->value(), where);
    }
  } else if (field.base_type != BaseType::Float &&
             field.base_type != BaseType::Double &&
             std::holds_alternative<std::int64_t>(value)) {
    EncodeInteger(dest, field.base_type, std::get<std::int64_t>(value), where);
  } else {
    double val = 0.0;
    if (std::holds_alternative<std::int64_t>(value)) {
      val = static_cast<double>(std::get<std::int64_t>(value));
    } else if (std::holds_alternative<double>(value)) {
      val = std::get<double>(value);
    } else {
      throw std::invalid_argument(at + ": field '" + field.leaf_name +
                                  "' is numeric but value is not a number");
    }
    EncodeNumber(dest, field.base_type, val, where);
  }
}

bool FitsFloatingPoint(BaseType type, double value) {
  return std::isfinite(value) &&
         (type != BaseType::Float ||
          std::abs(value) <= std::numeric_limits<float>::max());
}

void EncodeNumber(std::byte* dest, BaseType type, double value,
                  std::string_view where) {
  if ((type == BaseType::Float || type == BaseType::Double) &&
      !FitsFloatingPoint(type, value)) {
    throw std::invalid_argument(
        std::string{where} +
        ": value must be finite and fit floating-point field");
  }
  if (type == BaseType::Float) {
    auto f = static_cast<float>(value);
    std::memcpy(dest, &f, sizeof(float));
    return;
  }
  if (type == BaseType::Double) {
    std::memcpy(dest, &value, sizeof(double));
    return;
  }
  if (!std::isfinite(value) || std::trunc(value) != value) {
    throw std::invalid_argument(
        std::string{where} +
        ": non-finite or fractional value for integer field");
  }
  if (type == BaseType::ULong) {
    // UINT64_MAX rounds up to 2^64 as a double, so use an exclusive
    // power-of-two upper bound before converting.
    if (value < 0.0 || value >= std::ldexp(1.0, 64)) {
      throw std::invalid_argument(
          std::string{where} + ": value does not fit unsigned integer field");
    }
    const auto v = static_cast<std::uint64_t>(value);
    std::memcpy(dest, &v, sizeof(v));
    return;
  }
  if (value < static_cast<double>(std::numeric_limits<std::int64_t>::min()) ||
      value >= std::ldexp(1.0, 63)) {
    throw std::invalid_argument(std::string{where} +
                                ": value does not fit integer field");
  }
  EncodeInteger(dest, type, static_cast<std::int64_t>(value), where);
}

}  // namespace talos::schema
