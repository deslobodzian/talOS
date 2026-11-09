#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
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

inline bool FitsFloatingPoint(reflection::BaseType type, double val) {
  return std::isfinite(val) &&
         (type != reflection::BaseType::Float ||
          std::abs(val) <= std::numeric_limits<float>::max());
}

inline void EncodeNumericBytes(std::byte* dest, reflection::BaseType type,
                               double val, std::string_view where = "") {
  using reflection::BaseType;
  if ((type == BaseType::Float || type == BaseType::Double) &&
      !FitsFloatingPoint(type, val)) {
    throw std::invalid_argument(
        std::string{where} +
        ": value must be finite and fit floating-point field");
  }
  if (type == BaseType::Float) {
    auto f = static_cast<float>(val);
    std::memcpy(dest, &f, sizeof(float));
  } else if (type == BaseType::Double) {
    std::memcpy(dest, &val, sizeof(double));
  } else {
    if (!std::isfinite(val) || std::trunc(val) != val) {
      throw std::invalid_argument(
          std::string{where} +
          ": non-finite or fractional value for integer field");
    }
    if (type == BaseType::ULong) {
      // UINT64_MAX rounds up to 2^64 as a double, so use an exclusive
      // power-of-two upper bound before converting.
      if (val < 0.0 || val >= std::ldexp(1.0, 64)) {
        throw std::invalid_argument(
            std::string{where} + ": value does not fit unsigned integer field");
      }
      const auto v = static_cast<std::uint64_t>(val);
      std::memcpy(dest, &v, sizeof(v));
      return;
    }
    if (val < static_cast<double>(std::numeric_limits<std::int64_t>::min()) ||
        val >= std::ldexp(1.0, 63)) {
      throw std::invalid_argument(std::string{where} +
                                  ": value does not fit integer field");
    }
    EncodeIntegerBytes(dest, type, static_cast<std::int64_t>(val), where);
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

inline void EncodeAssignment(ButtonFieldAction& action, const ResolvedField& rf,
                             const config::TeleopValue& fv,
                             const std::string& field_where) {
  if (rf.base_type == reflection::BaseType::Bool) {
    if (!std::holds_alternative<bool>(fv)) {
      throw std::invalid_argument(field_where + ": field '" + rf.leaf_name +
                                  "' is bool but value is not a boolean");
    }
    action.bytes[0] = static_cast<std::byte>(std::get<bool>(fv) ? 1 : 0);
  } else if (rf.enum_def != nullptr) {
    if (!std::holds_alternative<std::string>(fv)) {
      throw std::invalid_argument(field_where + ": field '" + rf.leaf_name +
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
      throw std::invalid_argument(field_where + ": unknown enum label '" +
                                  label + "' in " + rf.enum_def->name()->str());
    }
    EncodeIntegerBytes(action.bytes.data(), rf.base_type, ev->value(),
                       field_where);
  } else {
    double val = 0.0;
    if (std::holds_alternative<int64_t>(fv)) {
      val = static_cast<double>(std::get<int64_t>(fv));
    } else if (std::holds_alternative<double>(fv)) {
      val = std::get<double>(fv);
    } else {
      throw std::invalid_argument(field_where + ": field '" + rf.leaf_name +
                                  "' is numeric but value is not a number");
    }
    EncodeNumericBytes(action.bytes.data(), rf.base_type, val, field_where);
  }
}

inline void ScalarPaths(const reflection::Schema& schema,
                        const reflection::Object& object,
                        const std::string& prefix,
                        std::vector<std::string>& paths) {
  for (const auto* field : *object.fields()) {
    const std::string path = prefix + field->name()->str();
    if (field->type()->base_type() == reflection::BaseType::Obj) {
      ScalarPaths(schema, *schema.objects()->Get(field->type()->index()),
                  path + "/", paths);
    } else {
      paths.push_back(path);
    }
  }
}

struct PrecomputedBinding {
  bool is_button{true};
  int port{0};
  int button{0};
  std::optional<int> pov;
  int pov_index{0};
  bool on_toggle{false};
  bool toggled{false};
  bool was_pressed{false};
  bool connected{false};
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
  std::vector<std::byte> defaults;
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

    const auto now = node_.now();
    if (last_evaluation_ &&
        now - *last_evaluation_ >=
            talos::detail::NodeAccess::freshness_timeout(node_)) {
      armed_ = false;
    }
    last_evaluation_ = now;
    if (!ds_.enabled() || !ds_.teleop()) armed_ = false;

    // A stick that (re)appears may already be deflected; an absent spare
    // controller is no input and does not block the others.
    for (auto& sub : subsystems_) {
      for (auto& binding : sub.bindings) {
        const bool connected = ds_.joystick(binding.port).connected();
        if (connected && !binding.connected) armed_ = false;
        binding.connected = connected;
      }
    }
    if (!armed_) {
      bool neutral = ds_.enabled() && ds_.teleop();
      for (auto& sub : subsystems_) {
        for (auto& binding : sub.bindings) {
          const auto stick = ds_.joystick(binding.port);
          binding.toggled = false;
          // Seed edge history; a button held through rearm is not a press.
          const bool pressed =
              binding.is_button &&
              (binding.pov ? stick.pov(binding.pov_index) == *binding.pov
                           : stick.button(binding.button));
          binding.was_pressed = pressed;
          if (!stick.connected()) continue;
          const bool released =
              binding.pov ? stick.pov(binding.pov_index) < 0 : !pressed;
          const double axis =
              binding.is_button ? 0.0 : stick.axis(binding.axis);
          neutral = neutral && released && std::isfinite(axis) &&
                    std::abs(axis) <= binding.deadband_frac;
        }
        if (sub.commanding) {
          TargetAccess::lane_release(node_, sub.lane_index);
          sub.commanding = false;
        }
      }
      armed_ = neutral;
      return;
    }
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
        for (auto& binding : sub.bindings) {
          binding.toggled = false;
          binding.was_pressed = false;
        }
        if (sub.commanding) {
          TargetAccess::lane_release(node_, sub.lane_index);
          sub.commanding = false;
        }
        continue;
      }

      std::copy(sub.defaults.begin(), sub.defaults.end(), sub.buffer.begin());
      std::fill(sub.field_set.begin(), sub.field_set.end(), 0);

      bool any_live = false;
      bool invalid_axis = false;
      for (auto& b : sub.bindings) {
        const auto stick = ds_.joystick(b.port);
        if (!stick.connected()) {
          b.toggled = false;
          b.was_pressed = false;
          continue;
        }
        if (b.is_button) {
          const bool pressed =
              b.pov ? stick.pov(b.pov_index) == *b.pov : stick.button(b.button);
          if (b.on_toggle && !b.was_pressed && pressed) {
            b.toggled = !b.toggled;
          }
          b.was_pressed = pressed;
          if (b.on_toggle ? b.toggled : pressed) {
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
          if (!std::isfinite(raw)) {
            invalid_axis = true;
            break;
          }
          if (std::abs(raw) > b.deadband_frac) {
            any_live = true;
            if (!sub.field_set[b.axis_field.field_id]) {
              sub.field_set[b.axis_field.field_id] = 1;
              const double rescaled = Deadband(raw, b.deadband_frac);
              const double val = rescaled * b.scale;
              if (!detail::FitsFloatingPoint(b.axis_field.base_type, val)) {
                invalid_axis = true;
                break;
              }
              detail::WriteScalar(sub.buffer.data() + b.axis_field.offset,
                                  b.axis_field.base_type, val);
            }
          }
        }
      }

      if (any_live && !invalid_axis) {
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

      if (parsed_binding.button.has_value() || parsed_binding.pov.has_value()) {
        binding.is_button = true;
        binding.button = parsed_binding.button.value_or(0);
        binding.pov = parsed_binding.pov;
        binding.pov_index = parsed_binding.pov_index;
        binding.on_toggle = !parsed_binding.on_toggle.empty();
        const auto& assignments = binding.on_toggle ? parsed_binding.on_toggle
                                                    : parsed_binding.while_held;
        const std::string action_name =
            binding.on_toggle ? ".on_toggle." : ".while_held.";
        for (const auto& [fk, fv] : assignments) {
          const std::string field_where = entry_where + action_name + fk;
          detail::ResolvedField rf =
              detail::ResolveField(*schema, *object, fk, field_where);

          detail::ButtonFieldAction action;
          action.field_id = get_field_id(fk);
          action.offset = rf.offset;
          action.size = detail::TypeSize(rf.base_type);

          detail::EncodeAssignment(action, rf, fv, field_where);

          binding.button_fields.push_back(action);
        }
      } else {
        binding.is_button = false;
        binding.axis = *parsed_binding.axis;
        binding.scale = parsed_binding.scale;
        binding.deadband_frac = parsed_binding.deadband_frac;
        if (!std::isfinite(binding.scale) ||
            !std::isfinite(binding.deadband_frac)) {
          throw std::invalid_argument(
              entry_where + ": scale and deadband_frac must be finite");
        }

        const std::string field_where = entry_where + ".field";
        detail::ResolvedField rf = detail::ResolveField(
            *schema, *object, parsed_binding.field, field_where);
        if (rf.base_type == reflection::BaseType::Bool ||
            rf.enum_def != nullptr) {
          throw std::invalid_argument(field_where + ": field '" + rf.leaf_name +
                                      "' must be a numeric scalar");
        }
        if (rf.base_type != reflection::BaseType::Float &&
            rf.base_type != reflection::BaseType::Double) {
          throw std::invalid_argument(field_where + ": axis field '" +
                                      rf.leaf_name +
                                      "' must be floating-point");
        }

        binding.axis_field.field_id = get_field_id(parsed_binding.field);
        binding.axis_field.offset = rf.offset;
        binding.axis_field.base_type = rf.base_type;
      }

      state.bindings.push_back(std::move(binding));
    }
    state.defaults.resize(inner_bytes, std::byte{0});
    const auto defaults_it = config.teleop_defaults.find(sub_name);
    std::map<std::string, bool> default_fields;
    if (defaults_it != config.teleop_defaults.end()) {
      for (const auto& [path, value] : defaults_it->second) {
        const std::string where =
            "[subsystems." + sub_name + "] teleop.defaults." + path;
        const auto rf = detail::ResolveField(*schema, *object, path, where);
        detail::ButtonFieldAction action;
        action.offset = rf.offset;
        action.size = detail::TypeSize(rf.base_type);
        detail::EncodeAssignment(action, rf, value, where);
        std::memcpy(state.defaults.data() + action.offset, action.bytes.data(),
                    action.size);
        default_fields[path] = true;
      }
    }
    std::vector<std::string> paths;
    detail::ScalarPaths(*schema, *object, "", paths);
    for (const auto& path : paths) {
      // A lane publishes only while some binding is live, so a field is
      // always written when every binding writes it.
      const bool always_assigned = std::all_of(
          bindings.begin(), bindings.end(), [&](const auto& binding) {
            if (binding.axis) return binding.field == path;
            const auto& assignments = binding.on_toggle.empty()
                                          ? binding.while_held
                                          : binding.on_toggle;
            return std::any_of(
                assignments.begin(), assignments.end(),
                [&](const auto& entry) { return entry.first == path; });
          });
      if (!always_assigned && !default_fields.contains(path)) {
        throw std::invalid_argument(lane_topic + ": field '" + path +
                                    "' requires an explicit teleop default");
      }
    }
    state.field_set.resize(field_ids.size(), 0);
    subsystems_.push_back(std::move(state));
  }

  talos::Node& node_;
  talos::DriverStation ds_;
  std::vector<detail::SubsystemState> subsystems_;
  bool armed_{false};
  std::optional<talos::Time> last_evaluation_;
};

}  // namespace talos::teleop
