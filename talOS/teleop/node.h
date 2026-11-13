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

#include "flatbuffers/reflection.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/devices/access.h"
#include "talOS/devices/devices.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/introspection/describe.h"
#include "talOS/introspection/names.h"
#include "talOS/node/access.h"
#include "talOS/node/node.h"
#include "talOS/schema/field_path.h"
#include "talOS/schema/identity.h"
#include "talOS/schema/schema_hash.h"
#include "talOS/teleop/deadband.h"

namespace talos::teleop {

namespace detail {

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
  explicit Teleop(talos::Node& node)
      : node_{node},
        ds_{node},
        arming_{talos::detail::TargetAccess::open_arming(node)} {
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
    if (!ds_ok) TargetAccess::arming_disarm(node_, arming_);
    // A stick that (re)appears may already be deflected; an absent spare
    // controller is no input and does not block the others.
    for (auto& sub : subsystems_) {
      for (auto& binding : sub.bindings) {
        const bool connected = ds_.joystick(binding.port).connected();
        if (connected && !binding.connected) {
          TargetAccess::arming_disarm(node_, arming_);
        }
        binding.connected = connected;
      }
    }
    // The arming (talos::OperatorArming) also disarms on its own in every
    // new mode generation or gateway boot, after a stale Driver Station
    // link, and after this node's evaluations stall for command_timeout_us.
    if (!TargetAccess::arming_evaluate(node_, arming_,
                                       ds_ok && InputsNeutral())) {
      for (auto& sub : subsystems_) {
        for (auto& binding : sub.bindings) {
          binding.toggled = false;
          // Seed edge history; a button held through rearm is not a press.
          binding.was_pressed = Pressed(binding, ds_.joystick(binding.port));
        }
        if (sub.commanding) {
          TargetAccess::lane_release(node_, sub.lane_index);
          sub.commanding = false;
        }
      }
      return;
    }
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
          const bool pressed = Pressed(b, stick);
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
              if (!schema::FitsFloatingPoint(b.axis_field.base_type, val)) {
                invalid_axis = true;
                break;
              }
              schema::EncodeNumber(sub.buffer.data() + b.axis_field.offset,
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
  static bool Pressed(const detail::PrecomputedBinding& binding,
                      const talos::Joystick& stick) {
    return binding.is_button &&
           (binding.pov ? stick.pov(binding.pov_index) == *binding.pov
                        : stick.button(binding.button));
  }

  // Every connected bound input at rest: axes finite and inside their
  // deadband, buttons released, POVs centred (a bound POV held in another
  // direction is not at rest).
  bool InputsNeutral() const {
    for (const auto& sub : subsystems_) {
      for (const auto& binding : sub.bindings) {
        const talos::Joystick stick = ds_.joystick(binding.port);
        if (!stick.connected()) continue;
        const bool released = binding.pov ? stick.pov(binding.pov_index) < 0
                                          : !Pressed(binding, stick);
        const double axis = binding.is_button ? 0.0 : stick.axis(binding.axis);
        if (!released || !std::isfinite(axis) ||
            std::abs(axis) > binding.deadband_frac) {
          return false;
        }
      }
    }
    return true;
  }

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

    const std::string lane_topic =
        introspect::naming::TargetLaneTopic(sub_name, "teleop");
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

    const reflection::Schema* schema = schema::VerifySchema(reader->bfbs);
    if (schema == nullptr) {
      throw std::invalid_argument("[subsystems." + sub_name +
                                  "] teleop: invalid reflection schema");
    }
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
                                inner_alignment, inner_identity, arming_);

    detail::SubsystemState state;
    state.name = sub_name;
    state.lane_index = lane_index;
    state.message_bytes = inner_bytes;
    state.buffer.resize(inner_bytes, std::byte{0});

    // Each button assignment is encoded here, then its bytes kept for the
    // tick's copy.
    std::vector<std::byte> scratch(inner_bytes, std::byte{0});
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
          const schema::ResolvedField rf =
              schema::ResolveField(*schema, *object, fk, field_where);

          detail::ButtonFieldAction action;
          action.field_id = get_field_id(fk);
          action.offset = rf.offset;
          action.size = rf.size;

          schema::EncodeField(scratch.data(), rf, fv, field_where);
          std::memcpy(action.bytes.data(), scratch.data() + rf.offset, rf.size);

          binding.button_fields.push_back(action);
        }
      } else {
        if (!parsed_binding.axis.has_value()) {
          throw std::invalid_argument(
              entry_where +
              ": binding must specify exactly one of button/axis/pov");
        }
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
        const schema::ResolvedField rf = schema::ResolveField(
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
        const auto rf = schema::ResolveField(*schema, *object, path, where);
        schema::EncodeField(state.defaults.data(), rf, value, where);
        default_fields[path] = true;
      }
    }
    for (const auto& path : schema::ScalarPaths(*schema, *object)) {
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
  // A talos::OperatorArming by index: the lanes are opened by index too.
  std::size_t arming_;
  std::vector<detail::SubsystemState> subsystems_;
};

}  // namespace talos::teleop
