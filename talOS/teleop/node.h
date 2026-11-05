#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <map>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "flatbuffers/flatbuffers.h"
#include "flatbuffers/reflection.h"
#include "flatbuffers/verifier.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/devices/access.h"
#include "talOS/devices/devices.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/names.h"
#include "talOS/node/access.h"
#include "talOS/node/node.h"
#include "talOS/schema/identity.h"
#include "talOS/schema/schema_hash.h"
#include "talOS/teleop/deadband.h"

namespace talos::teleop {

namespace detail {

inline std::size_t TypeSize(reflection::BaseType type) {
  using reflection::BaseType;
  switch (type) {
    case BaseType::Bool:
    case BaseType::Byte:
    case BaseType::UByte:
      return 1;
    case BaseType::Short:
    case BaseType::UShort:
      return 2;
    case BaseType::Int:
    case BaseType::UInt:
    case BaseType::Float:
      return 4;
    case BaseType::Long:
    case BaseType::ULong:
    case BaseType::Double:
      return 8;
    default:
      return 0;
  }
}

template <typename T>
inline void CheckAndCopyInt(std::byte* dest, int64_t val,
                            std::string_view where) {
  if (val < static_cast<int64_t>(std::numeric_limits<T>::lowest()) ||
      val > static_cast<int64_t>(std::numeric_limits<T>::max())) {
    throw std::invalid_argument(std::string{where} + ": value " +
                                std::to_string(val) +
                                " does not fit integer field");
  }
  auto v = static_cast<T>(val);
  std::memcpy(dest, &v, sizeof(v));
}

inline void EncodeIntegerBytes(std::byte* dest, reflection::BaseType type,
                               int64_t val, std::string_view where = "") {
  using reflection::BaseType;
  switch (type) {
    case BaseType::Byte:
      CheckAndCopyInt<int8_t>(dest, val, where);
      break;
    case BaseType::UByte:
      CheckAndCopyInt<uint8_t>(dest, val, where);
      break;
    case BaseType::Short:
      CheckAndCopyInt<int16_t>(dest, val, where);
      break;
    case BaseType::UShort:
      CheckAndCopyInt<uint16_t>(dest, val, where);
      break;
    case BaseType::Int:
      CheckAndCopyInt<int32_t>(dest, val, where);
      break;
    case BaseType::UInt:
      CheckAndCopyInt<uint32_t>(dest, val, where);
      break;
    case BaseType::Long:
      std::memcpy(dest, &val, sizeof(val));
      break;
    case BaseType::ULong: {
      if (val < 0) {
        throw std::invalid_argument(std::string{where} +
                                    ": negative value for unsigned field");
      }
      auto v = static_cast<uint64_t>(val);
      std::memcpy(dest, &v, sizeof(v));
      break;
    }
    default:
      break;
  }
}

inline void EncodeNumericBytes(std::byte* dest, reflection::BaseType type,
                               double val, std::string_view where = "") {
  using reflection::BaseType;
  if (type == BaseType::Float) {
    auto f = static_cast<float>(val);
    std::memcpy(dest, &f, sizeof(float));
  } else if (type == BaseType::Double) {
    std::memcpy(dest, &val, sizeof(double));
  } else {
    if (std::trunc(val) != val) {
      throw std::invalid_argument(std::string{where} +
                                  ": fractional value for integer field");
    }
    EncodeIntegerBytes(dest, type, static_cast<int64_t>(val), where);
  }
}

inline void WriteScalar(std::byte* dest, reflection::BaseType type,
                        double val) {
  EncodeNumericBytes(dest, type, val);
}

struct ResolvedField {
  std::size_t offset{0};
  reflection::BaseType base_type{reflection::BaseType::None};
  const reflection::Enum* enum_def{nullptr};
  std::string leaf_name;
  std::string enclosing_name;
};

inline ResolvedField ResolveField(const reflection::Schema& schema,
                                  const reflection::Object& root_obj,
                                  std::string_view field_path,
                                  std::string_view entry_where) {
  const reflection::Object* curr_obj = &root_obj;
  std::size_t curr_offset = 0;
  std::string path_str{field_path};
  std::vector<std::string> segments;
  std::size_t pos = 0;
  while (pos < path_str.size()) {
    auto next = path_str.find('/', pos);
    if (next == std::string::npos) {
      segments.push_back(path_str.substr(pos));
      break;
    }
    segments.push_back(path_str.substr(pos, next - pos));
    pos = next + 1;
  }
  if (segments.empty()) {
    throw std::invalid_argument(std::string{entry_where} +
                                ": empty field path");
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
      throw std::invalid_argument(std::string{entry_where} + ": no field '" +
                                  seg + "' in " + curr_obj->name()->str());
    }
    if (s + 1 < segments.size()) {
      if (field->type()->base_type() != reflection::BaseType::Obj) {
        throw std::invalid_argument(std::string{entry_where} + ": '" + seg +
                                    "' is not a nested struct");
      }
      const auto* next_obj = schema.objects()->Get(field->type()->index());
      if (next_obj == nullptr || !next_obj->is_struct()) {
        throw std::invalid_argument(std::string{entry_where} + ": '" + seg +
                                    "' is not a struct");
      }
      curr_offset += field->offset();
      curr_obj = next_obj;
    } else {
      const auto base = field->type()->base_type();
      if (base == reflection::BaseType::Obj ||
          base == reflection::BaseType::Vector ||
          base == reflection::BaseType::Array ||
          base == reflection::BaseType::None) {
        throw std::invalid_argument(std::string{entry_where} + ": field '" +
                                    seg + "' must be a scalar");
      }
      const reflection::Enum* enum_def = nullptr;
      if (field->type()->index() >= 0 &&
          field->type()->index() <
              static_cast<int32_t>(schema.enums()->size())) {
        enum_def = schema.enums()->Get(field->type()->index());
      }
      return ResolvedField{
          .offset = curr_offset + field->offset(),
          .base_type = base,
          .enum_def = enum_def,
          .leaf_name = seg,
          .enclosing_name = curr_obj->name()->str(),
      };
    }
  }
  throw std::logic_error("unreachable");
}

struct ButtonFieldAction {
  uint32_t field_id{0};
  std::size_t offset{0};
  std::size_t size{0};
  std::array<std::byte, 8> bytes{};
};

struct AxisFieldAction {
  uint32_t field_id{0};
  std::size_t offset{0};
  reflection::BaseType base_type{reflection::BaseType::None};
};

struct PrecomputedBinding {
  bool is_button{true};
  int port{0};
  int button{0};
  std::vector<ButtonFieldAction> button_fields;
  int axis{0};
  double scale{1.0};
  double deadband_frac{0.0};
  AxisFieldAction axis_field;
};

struct SubsystemState {
  std::string name;
  std::size_t lane_index{0};
  std::size_t message_bytes{0};
  std::vector<PrecomputedBinding> bindings;
  std::vector<std::byte> buffer;
  std::vector<uint8_t> field_set;
  bool commanding{false};
};

}  // namespace detail

class Teleop {
 public:
  explicit Teleop(talos::Node& node) : node_{node}, ds_{node} {
    using talos::detail::NodeAccess;

    const introspect::DeclaredGraph* graph = NodeAccess::declared_graph(node);
    if (graph == nullptr) {
      return;
    }

    const config::RobotConfig& config = NodeAccess::robot_config(node);
    for (const auto& [sub_name, bindings] : config.teleop) {
      if (bindings.empty()) continue;
      BuildSubsystem(sub_name, bindings, config, *graph);
    }
  }

  void update() {
    using talos::detail::TargetAccess;

    const bool ds_ok = ds_.enabled() && ds_.teleop();
    for (auto& sub : subsystems_) {
      bool any_stick_connected = false;
      for (const auto& b : sub.bindings) {
        if (ds_.joystick(b.port).connected()) {
          any_stick_connected = true;
          break;
        }
      }
      const bool condition_ok = ds_ok && any_stick_connected;
      if (!condition_ok) {
        if (sub.commanding) {
          TargetAccess::lane_release(node_, sub.lane_index);
          sub.commanding = false;
        }
        continue;
      }

      std::memset(sub.buffer.data(), 0, sub.buffer.size());
      std::fill(sub.field_set.begin(), sub.field_set.end(), 0);

      bool any_live = false;
      for (const auto& b : sub.bindings) {
        const auto stick = ds_.joystick(b.port);
        if (!stick.connected()) continue;
        if (b.is_button) {
          if (stick.button(b.button)) {
            any_live = true;
            for (const auto& f : b.button_fields) {
              if (!sub.field_set[f.field_id]) {
                sub.field_set[f.field_id] = 1;
                std::memcpy(sub.buffer.data() + f.offset, f.bytes.data(),
                            f.size);
              }
            }
          }
        } else {
          const double raw = stick.axis(b.axis);
          if (std::abs(raw) > b.deadband_frac) {
            any_live = true;
            if (!sub.field_set[b.axis_field.field_id]) {
              sub.field_set[b.axis_field.field_id] = 1;
              const double rescaled = Deadband(raw, b.deadband_frac);
              const double val = rescaled * b.scale;
              detail::WriteScalar(sub.buffer.data() + b.axis_field.offset,
                                  b.axis_field.base_type, val);
            }
          }
        }
      }

      if (any_live) {
        sub.commanding = true;
        TargetAccess::lane_send(node_, sub.lane_index, sub.buffer.data(),
                                sub.buffer.size());
      } else {
        sub.commanding = false;
        TargetAccess::lane_release(node_, sub.lane_index);
      }
    }
  }

 private:
  void BuildSubsystem(const std::string& sub_name,
                      const std::vector<config::TeleopBinding>& bindings,
                      const config::RobotConfig& config,
                      const introspect::DeclaredGraph& graph) {
    using talos::detail::TargetAccess;

    const auto& policy = config.TargetsFor(sub_name);
    if (std::find(policy.teleop.begin(), policy.teleop.end(), "teleop") ==
        policy.teleop.end()) {
      throw std::invalid_argument(
          "[subsystems." + sub_name +
          "] teleop: 'teleop' missing from the subsystem's teleop-mode "
          "target policy");
    }

    const std::string lane_topic = "/" + sub_name + "/target/teleop";
    const introspect::naming::SourceShape* reader = nullptr;
    for (const auto& node_desc : graph) {
      for (const auto& src : node_desc.sources) {
        if (src.name == lane_topic && introspect::naming::IsReader(src.kind)) {
          reader = &src;
          break;
        }
      }
      if (reader != nullptr) break;
    }
    if (reader == nullptr) {
      throw std::invalid_argument("[subsystems." + sub_name +
                                  "] teleop: no reader of '" + lane_topic +
                                  "' in the graph");
    }

    flatbuffers::Verifier verifier{
        reinterpret_cast<const std::uint8_t*>(reader->bfbs.data()),
        reader->bfbs.size()};
    if (!reflection::VerifySchemaBuffer(verifier)) {
      throw std::invalid_argument("[subsystems." + sub_name +
                                  "] teleop: invalid reflection schema");
    }
    const reflection::Schema* schema =
        reflection::GetSchema(reader->bfbs.data());
    const std::string_view inner_name = schema::EnvelopeInner(reader->schema);
    if (inner_name.empty()) {
      throw std::invalid_argument(
          "[subsystems." + sub_name +
          "] teleop: reader schema is not a target envelope: " +
          reader->schema);
    }
    const reflection::Object* object =
        schema->objects()->LookupByKey(std::string{inner_name}.c_str());
    if (object == nullptr || !object->is_struct()) {
      throw std::invalid_argument(
          "[subsystems." + sub_name +
          "] teleop: target message is not a struct in schema: " +
          std::string(inner_name));
    }
    const std::size_t inner_bytes = object->bytesize();
    const std::size_t inner_alignment = object->minalign();
    const std::uint64_t inner_hash = schema::struct_hash(*schema, *object);
    const schema::Identity inner_identity{std::string(inner_name), inner_hash,
                                          reader->bfbs};

    const std::size_t lane_index =
        TargetAccess::open_lane(node_, sub_name, "teleop", inner_bytes,
                                inner_alignment, inner_identity);

    detail::SubsystemState state;
    state.name = sub_name;
    state.lane_index = lane_index;
    state.message_bytes = inner_bytes;
    state.buffer.resize(inner_bytes, std::byte{0});

    std::map<std::string, uint32_t> field_ids;
    auto get_field_id = [&](const std::string& path) {
      auto it = field_ids.find(path);
      if (it != field_ids.end()) return it->second;
      uint32_t id = static_cast<uint32_t>(field_ids.size());
      field_ids[path] = id;
      return id;
    };

    for (std::size_t i = 0; i < bindings.size(); ++i) {
      const auto& parsed_binding = bindings[i];
      const std::string entry_where = "[subsystems." + sub_name +
                                      "] teleop.bindings[" + std::to_string(i) +
                                      "]";
      detail::PrecomputedBinding binding;
      binding.port = parsed_binding.port;

      if (parsed_binding.button.has_value()) {
        binding.is_button = true;
        binding.button = *parsed_binding.button;

        for (const auto& [fk, fv] : parsed_binding.while_held) {
          const std::string field_where = entry_where + ".while_held." + fk;
          detail::ResolvedField rf =
              detail::ResolveField(*schema, *object, fk, field_where);

          detail::ButtonFieldAction action;
          action.field_id = get_field_id(fk);
          action.offset = rf.offset;
          action.size = detail::TypeSize(rf.base_type);

          if (rf.base_type == reflection::BaseType::Bool) {
            if (!std::holds_alternative<bool>(fv)) {
              throw std::invalid_argument(
                  field_where + ": field '" + rf.leaf_name +
                  "' is bool but value is not a boolean");
            }
            action.bytes[0] =
                static_cast<std::byte>(std::get<bool>(fv) ? 1 : 0);
          } else if (rf.enum_def != nullptr) {
            if (!std::holds_alternative<std::string>(fv)) {
              throw std::invalid_argument(
                  field_where + ": field '" + rf.leaf_name +
                  "' is enum but value is not a string");
            }
            const std::string& label = std::get<std::string>(fv);
            const reflection::EnumVal* ev = nullptr;
            for (const auto* val : *rf.enum_def->values()) {
              if (val->name()->str() == label) {
                ev = val;
                break;
              }
            }
            if (ev == nullptr) {
              throw std::invalid_argument(field_where +
                                          ": unknown enum label '" + label +
                                          "' in " + rf.enum_def->name()->str());
            }
            detail::EncodeIntegerBytes(action.bytes.data(), rf.base_type,
                                       ev->value(), field_where);
          } else {
            double val = 0.0;
            if (std::holds_alternative<int64_t>(fv)) {
              val = static_cast<double>(std::get<int64_t>(fv));
            } else if (std::holds_alternative<double>(fv)) {
              val = std::get<double>(fv);
            } else {
              throw std::invalid_argument(
                  field_where + ": field '" + rf.leaf_name +
                  "' is numeric but value is not a number");
            }
            detail::EncodeNumericBytes(action.bytes.data(), rf.base_type, val,
                                       field_where);
          }

          binding.button_fields.push_back(action);
        }
      } else {
        binding.is_button = false;
        binding.axis = *parsed_binding.axis;
        binding.scale = parsed_binding.scale;
        binding.deadband_frac = parsed_binding.deadband_frac;

        const std::string field_where = entry_where + ".field";
        detail::ResolvedField rf = detail::ResolveField(
            *schema, *object, parsed_binding.field, field_where);
        if (rf.base_type == reflection::BaseType::Bool ||
            rf.enum_def != nullptr) {
          throw std::invalid_argument(field_where + ": field '" + rf.leaf_name +
                                      "' must be a numeric scalar");
        }

        binding.axis_field.field_id = get_field_id(parsed_binding.field);
        binding.axis_field.offset = rf.offset;
        binding.axis_field.base_type = rf.base_type;
      }

      state.bindings.push_back(std::move(binding));
    }
    state.field_set.resize(field_ids.size(), 0);
    subsystems_.push_back(std::move(state));
  }

  talos::Node& node_;
  talos::DriverStation ds_;
  std::vector<detail::SubsystemState> subsystems_;
};

}  // namespace talos::teleop
