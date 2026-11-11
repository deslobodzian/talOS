#include "talOS/schema/field_path.h"

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
#include "flatbuffers/verifier.h"

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
      break;
  }
}

void AppendScalarPaths(const reflection::Schema& schema,
                       const reflection::Object& object,
                       const std::string& prefix,
                       std::vector<std::string>& paths) {
  for (const auto* field : *object.fields()) {
    const std::string path = prefix + field->name()->str();
    if (field->type()->base_type() == BaseType::Obj) {
      AppendScalarPaths(schema, *schema.objects()->Get(field->type()->index()),
                        path + "/", paths);
    } else {
      paths.push_back(path);
    }
  }
}

}  // namespace

const reflection::Schema* VerifySchema(std::string_view bfbs) {
  flatbuffers::Verifier verifier{
      reinterpret_cast<const std::uint8_t*>(bfbs.data()), bfbs.size()};
  if (!reflection::VerifySchemaBuffer(verifier)) return nullptr;
  return reflection::GetSchema(bfbs.data());
}

ResolvedField ResolveField(const reflection::Schema& schema,
                           const reflection::Object& root,
                           std::string_view field_path,
                           std::string_view where) {
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
    segments.emplace_back(field_path.substr(pos, next - pos));
    pos = next + 1;
  }
  if (segments.empty()) {
    throw std::invalid_argument(std::string{where} + ": empty field path");
  }

  for (std::size_t s = 0; s < segments.size(); ++s) {
    const std::string& seg = segments[s];
    const reflection::Field* field = nullptr;
    for (const auto* f : *curr_obj->fields()) {
      if (f->name()->str() == seg) {
        field = f;
        break;
      }
    }
    if (field == nullptr) {
      throw std::invalid_argument(std::string{where} + ": no field '" + seg +
                                  "' in " + curr_obj->name()->str());
    }
    if (s + 1 < segments.size()) {
      if (field->type()->base_type() != BaseType::Obj) {
        throw std::invalid_argument(std::string{where} + ": '" + seg +
                                    "' is not a nested struct");
      }
      const auto* next_obj = schema.objects()->Get(field->type()->index());
      if (next_obj == nullptr || !next_obj->is_struct()) {
        throw std::invalid_argument(std::string{where} + ": '" + seg +
                                    "' is not a struct");
      }
      curr_offset += field->offset();
      curr_obj = next_obj;
      continue;
    }
    const auto base = field->type()->base_type();
    if (base == BaseType::Obj || base == BaseType::Vector ||
        base == BaseType::Array || base == BaseType::None) {
      throw std::invalid_argument(std::string{where} + ": field '" + seg +
                                  "' must be a scalar");
    }
    const reflection::Enum* enum_def = nullptr;
    if (field->type()->index() >= 0 &&
        field->type()->index() <
            static_cast<std::int32_t>(schema.enums()->size())) {
      enum_def = schema.enums()->Get(field->type()->index());
    }
    return ResolvedField{
        .offset = curr_offset + field->offset(),
        .base_type = base,
        .size = ScalarSize(base),
        .enum_def = enum_def,
        .leaf_name = seg,
        .enclosing_name = curr_obj->name()->str(),
    };
  }
  throw std::logic_error("unreachable");
}

std::vector<std::string> ScalarPaths(const reflection::Schema& schema,
                                     const reflection::Object& root) {
  std::vector<std::string> paths;
  AppendScalarPaths(schema, root, "", paths);
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
    for (const auto* val : *field.enum_def->values()) {
      if (val->name()->str() == label) {
        ev = val;
        break;
      }
    }
    if (ev == nullptr) {
      throw std::invalid_argument(at + ": unknown enum label '" + label +
                                  "' in " + field.enum_def->name()->str());
    }
    EncodeInteger(dest, field.base_type, ev->value(), where);
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
