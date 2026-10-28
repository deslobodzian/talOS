#include "talOS/node_api/node_api.h"

#include <chrono>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>

#include "talOS/devices/access.h"
#include "talOS/devices/devices.h"
#include "talOS/hardware/messages.h"
#include "talOS/node/access.h"
#include "talOS/node/node.h"
#include "talOS/node_api/internal.h"

namespace talos::node_api {
namespace {

thread_local std::string g_last_error;
thread_local std::string g_param_string;  // TalosParam::string

using detail::HardwareAccess;
using detail::NodeAccess;

std::size_t Index(std::int64_t index) {
  if (index < 0) throw std::out_of_range("negative index");
  return static_cast<std::size_t>(index);
}

std::function<void()> Callback(TalosCallback fn, void* user,
                               const char* what) {
  if (fn == nullptr) {
    throw std::invalid_argument(std::string{what} + " is NULL");
  }
  return [fn, user, what] { Check(fn(user), what); };
}

HardwareLink& Link(TalosNode* node) {
  return AsNode(node).extension<HardwareLink>();
}

detail::DeviceKind DeviceKindOf(std::int32_t kind) {
  if (kind < TALOS_DEVICE_MOTOR || kind > TALOS_DEVICE_PWM) {
    throw std::invalid_argument("unknown device kind " + std::to_string(kind));
  }
  return static_cast<detail::DeviceKind>(kind);
}

hardware::Mode MotorMode(std::int32_t mode) {
  if (mode < TALOS_MODE_NEUTRAL || mode > TALOS_MODE_MOTION_MAGIC) {
    throw std::invalid_argument("unknown motor mode " + std::to_string(mode));
  }
  return static_cast<hardware::Mode>(mode);
}

TalosSample* Out(TalosSample* out) {
  if (out == nullptr) throw std::invalid_argument("null TalosSample");
  *out = TalosSample{};
  return out;
}

void Fill(TalosParam& out, const detail::ParamValue& value) {
  std::visit(
      [&out](const auto& v) {
        using V = std::decay_t<decltype(v)>;
        if constexpr (std::is_same_v<V, bool>) {
          out.kind = TALOS_PARAM_BOOL;
          out.boolean = v ? 1 : 0;
        } else if constexpr (std::is_same_v<V, std::int64_t>) {
          out.kind = TALOS_PARAM_INT;
          out.integer = v;
        } else if constexpr (std::is_same_v<V, double>) {
          out.kind = TALOS_PARAM_DOUBLE;
          out.number = v;
        } else {
          out.kind = TALOS_PARAM_STRING;
          g_param_string = v;
          out.string = g_param_string.c_str();
        }
      },
      value);
}

std::optional<detail::ParamValue> FindParam(const Node& node,
                                            std::string_view key,
                                            std::int32_t kind) {
  auto wrap = [](auto found) -> std::optional<detail::ParamValue> {
    if (!found) return std::nullopt;
    return detail::ParamValue{*std::move(found)};
  };
  switch (kind) {
    case TALOS_PARAM_ANY:
      return NodeAccess::find_param(node, key);
    case TALOS_PARAM_BOOL:
      return wrap(NodeAccess::find_bool(node, key));
    case TALOS_PARAM_INT:
      return wrap(NodeAccess::find_int(node, key));
    case TALOS_PARAM_DOUBLE:
      return wrap(NodeAccess::find_double(node, key));
    case TALOS_PARAM_STRING:
      return wrap(NodeAccess::find_string(node, key));
  }
  throw std::invalid_argument("unknown parameter kind " + std::to_string(kind));
}

}  // namespace

void SetError(std::string message) { g_last_error = std::move(message); }
void ClearError() { g_last_error.clear(); }

}  // namespace talos::node_api

using namespace talos::node_api;
using talos::detail::HardwareAccess;
using talos::detail::NodeAccess;

extern "C" {

uint32_t talos_abi_version(void) { return TALOS_NODE_ABI_VERSION; }

const char* talos_last_error(void) { return g_last_error.c_str(); }

int32_t talos_run(int32_t argc, char** argv, const char* node_name,
                  const char* node_target, TalosFactory factory, void* user) {
  return Guard<int32_t>(1, [&] {
    return talos::detail::RunMain(argc, argv,
                                  NonNull(node_name, "node_name"),
                                  NonNull(node_target, "node_target"),
                                  MakeFactory(factory, user));
  });
}

// --- node --------------------------------------------------------------------

const char* talos_node_name(const TalosNode* node) {
  return Guard<const char*>(
      "", [&] { return AsNode(node).name().data(); });  // a std::string
}

int32_t talos_node_simulation(const TalosNode* node) {
  return Guard<int32_t>(TALOS_ERR,
                        [&] { return AsNode(node).simulation() ? 1 : 0; });
}

int64_t talos_node_now_ns(const TalosNode* node) {
  return Guard<int64_t>(0, [&] { return AsNode(node).now().nanos(); });
}

uint64_t talos_node_tick(const TalosNode* node) {
  return Guard<uint64_t>(0, [&] { return AsNode(node).tick(); });
}

uint64_t talos_node_next_update(const TalosNode* node) {
  return Guard<uint64_t>(0,
                         [&] { return NodeAccess::next_update(AsNode(node)); });
}

int64_t talos_node_period_ns(const TalosNode* node) {
  return Guard<int64_t>(0, [&] { return AsNode(node).period().count(); });
}

const char* talos_node_config_path(const TalosNode* node) {
  return Guard<const char*>(
      "", [&] { return AsNode(node).config_path().c_str(); });
}

int32_t talos_node_param(const TalosNode* node, const char* key, int32_t kind,
                         int32_t required, TalosParam* out) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    const talos::Node& n = AsNode(node);
    const std::string_view k = NonNull(key, "key");
    if (out == nullptr) throw std::invalid_argument("null TalosParam");
    *out = TalosParam{};
    const auto value = FindParam(n, k, kind);
    if (!value) {
      if (required != 0) NodeAccess::param_error(n, k, "is missing");
      return 0;
    }
    Fill(*out, *value);
    return 1;
  });
}

int64_t talos_node_subscribe(TalosNode* node, const char* topic,
                             uint32_t message_bytes, int32_t optional,
                             TalosMessageCallback on_message, void* user) {
  return Guard<int64_t>(-1, [&] {
    talos::detail::MessageCallback callback;
    if (on_message != nullptr) {
      callback = [on_message, user, message_bytes](const void* data) {
        Check(on_message(data, message_bytes, user), "message callback");
      };
    }
    return static_cast<int64_t>(NodeAccess::add_subscription(
        AsNode(node), NonNull(topic, "topic"), message_bytes,
        talos::kMessageAlignment, optional != 0, std::move(callback)));
  });
}

int64_t talos_node_publish(TalosNode* node, const char* topic,
                           uint32_t message_bytes) {
  return Guard<int64_t>(-1, [&] {
    return static_cast<int64_t>(
        NodeAccess::add_publisher(AsNode(node), NonNull(topic, "topic"),
                                  message_bytes, talos::kMessageAlignment));
  });
}

int32_t talos_node_every(TalosNode* node, int64_t period_ns, TalosCallback fn,
                         void* user) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    AsNode(node).every(std::chrono::nanoseconds{period_ns},
                       Callback(fn, user, "every callback"));
    return TALOS_OK;
  });
}

int32_t talos_node_set_update(TalosNode* node, TalosCallback fn, void* user) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    NodeAccess::set_update(AsNode(node), Callback(fn, user, "update"));
    return TALOS_OK;
  });
}

const void* talos_subscription_value(const TalosNode* node, int64_t index) {
  return Guard<const void*>(nullptr, [&] {
    return NodeAccess::subscription_value(AsNode(node), Index(index));
  });
}

int32_t talos_subscription_has_value(const TalosNode* node, int64_t index) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    return NodeAccess::subscription_has_value(AsNode(node), Index(index)) ? 1
                                                                          : 0;
  });
}

int64_t talos_subscription_age_ns(const TalosNode* node, int64_t index) {
  return Guard<int64_t>(std::numeric_limits<int64_t>::max(), [&] {
    return NodeAccess::subscription_age(AsNode(node), Index(index)).count();
  });
}

int32_t talos_subscription_fresh(const TalosNode* node, int64_t index,
                                 int64_t max_age_ns) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    const talos::Node& n = AsNode(node);
    const auto max_age = max_age_ns < 0 ? NodeAccess::freshness_timeout(n)
                                        : std::chrono::nanoseconds{max_age_ns};
    return NodeAccess::subscription_fresh(n, Index(index), max_age) ? 1 : 0;
  });
}

int32_t talos_subscription_updated(const TalosNode* node, int64_t index) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    return NodeAccess::subscription_updated(AsNode(node), Index(index)) ? 1 : 0;
  });
}

int32_t talos_publisher_send(TalosNode* node, int64_t index, const void* data,
                             uint32_t size) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    if (data == nullptr) throw std::invalid_argument("null message");
    return NodeAccess::publisher_send(AsNode(node), Index(index), data, size)
               ? 1
               : 0;
  });
}

// --- devices -----------------------------------------------------------------

int64_t talos_device(TalosNode* node, int32_t kind, const char* name) {
  return Guard<int64_t>(-1, [&] {
    return static_cast<int64_t>(HardwareAccess::add_device(
        Link(node), DeviceKindOf(kind), NonNull(name, "device name")));
  });
}

int32_t talos_robot_enabled(TalosNode* node) {
  return Guard<int32_t>(TALOS_ERR,
                        [&] { return talos::robot_enabled(AsNode(node)); });
}

int32_t talos_motor_set(TalosNode* node, int64_t index, int32_t mode,
                        double demand, double ff_volts, int32_t slot) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    HardwareAccess::motor_set(Link(node), Index(index), MotorMode(mode), demand,
                              ff_volts, slot);
    return TALOS_OK;
  });
}

int32_t talos_digital_output_set(TalosNode* node, int64_t index,
                                 int32_t value) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    HardwareAccess::digital_output_set(Link(node), Index(index), value != 0);
    return TALOS_OK;
  });
}

int32_t talos_pwm_set(TalosNode* node, int64_t index, double output) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    HardwareAccess::pwm_set(Link(node), Index(index), output);
    return TALOS_OK;
  });
}

int32_t talos_motor_sample(TalosNode* node, int64_t index, TalosSample* out) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    TalosSample& o = *Out(out);
    const auto s = HardwareAccess::motor_sample(Link(node), Index(index));
    o.valid = s.valid;
    o.position = s.position_rot;
    o.velocity = s.velocity_rps;
    o.voltage = s.voltage;
    o.current = s.stator_current_a;
    return TALOS_OK;
  });
}

int32_t talos_sensor_sample(TalosNode* node, int64_t index, TalosSample* out) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    TalosSample& o = *Out(out);
    const auto s = HardwareAccess::sensor_sample(Link(node), Index(index));
    o.valid = s.valid;
    o.position = s.position_rot;
    o.velocity = s.velocity_rps;
    return TALOS_OK;
  });
}

int32_t talos_digital_input_sample(TalosNode* node, int64_t index,
                                   TalosSample* out) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    TalosSample& o = *Out(out);
    const auto s =
        HardwareAccess::digital_input_sample(Link(node), Index(index));
    o.valid = s.valid;
    o.value = s.value;
    return TALOS_OK;
  });
}

int32_t talos_analog_input_sample(TalosNode* node, int64_t index,
                                  TalosSample* out) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    TalosSample& o = *Out(out);
    const auto s =
        HardwareAccess::analog_input_sample(Link(node), Index(index));
    o.valid = s.valid;
    o.voltage = s.voltage;
    return TALOS_OK;
  });
}

int32_t talos_encoder_sample(TalosNode* node, int64_t index,
                             TalosSample* out) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    TalosSample& o = *Out(out);
    const auto s = HardwareAccess::encoder_sample(Link(node), Index(index));
    o.valid = s.valid;
    o.position = s.position_rot;
    o.velocity = s.velocity_rps;
    return TALOS_OK;
  });
}

}  // extern "C"
