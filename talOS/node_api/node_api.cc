#include "talOS/node_api/node_api.h"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <limits>
#include <optional>
#include <span>
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

std::function<void()> Callback(TalosCallback fn, void* user, const char* what) {
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

static_assert(TALOS_DS_MAX_JOYSTICKS == driver_station::kMaxJoysticks);
static_assert(TALOS_DS_MAX_AXES == driver_station::kMaxAxes);
static_assert(TALOS_DS_MAX_POVS == driver_station::kMaxPovs);
static_assert(TALOS_DS_NAME_LENGTH == driver_station::kMaxNameLength);
constexpr bool SameFlag(std::uint32_t abi, driver_station::DsFlags flag) {
  return abi == static_cast<std::uint32_t>(flag);
}
static_assert(SameFlag(TALOS_DS_ATTACHED, driver_station::kDsAttached) &&
              SameFlag(TALOS_DS_FMS_ATTACHED, driver_station::kFmsAttached) &&
              SameFlag(TALOS_DS_ENABLED, driver_station::kEnabled) &&
              SameFlag(TALOS_DS_AUTONOMOUS, driver_station::kAutonomous) &&
              SameFlag(TALOS_DS_TELEOP, driver_station::kTeleop) &&
              SameFlag(TALOS_DS_TEST, driver_station::kTest) &&
              SameFlag(TALOS_DS_ESTOP, driver_station::kEStop));

TalosJoystick JoystickOf(const Joystick& joystick) {
  TalosJoystick out{};
  out.connected = joystick.connected();
  out.axis_count = joystick.axis_count();
  out.button_count = joystick.button_count();
  out.pov_count = joystick.pov_count();
  for (int n = 1; n <= 32; ++n) {
    if (joystick.button(n)) out.buttons |= 1u << (n - 1);
  }
  for (int i = 0; i < TALOS_DS_MAX_AXES; ++i) {
    out.axes[i] = static_cast<float>(joystick.axis(i));
  }
  for (int i = 0; i < TALOS_DS_MAX_POVS; ++i) out.povs[i] = joystick.pov(i);
  const std::string_view name = joystick.name();
  std::memcpy(out.name, name.data(), std::min(name.size(), sizeof out.name));
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
    default:
      throw std::invalid_argument("unknown parameter kind " +
                                  std::to_string(kind));
  }
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
    return talos::detail::RunMain(argc, argv, NonNull(node_name, "node_name"),
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
  return Guard<const char*>("",
                            [&] { return AsNode(node).config_path().c_str(); });
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
    talos::Node& n = AsNode(node);
    talos::detail::MessageCallback callback;
    if (on_message != nullptr) {
      callback = [on_message, user, message_bytes](const void* data) {
        Check(on_message(data, message_bytes, user), "message callback");
      };
    }
    const std::size_t index = NodeAccess::add_subscription(
        n, NonNull(topic, "topic"), message_bytes, talos::kMessageAlignment,
        optional != 0, std::move(callback));
    if (on_message != nullptr) NodeAccess::require_immediate_delivery(n);
    return static_cast<int64_t>(index);
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

int64_t talos_node_publish_bytes(TalosNode* node, const char* topic,
                                 uint32_t max_bytes, uint32_t slots,
                                 int32_t drop_newest, int32_t external) {
  return Guard<int64_t>(-1, [&] {
    return static_cast<int64_t>(NodeAccess::add_byte_publisher(
        AsNode(node), NonNull(topic, "topic"), max_bytes,
        talos::BytePublishOptions{
            .slots = slots,
            .drop_newest = drop_newest != 0,
            .external = external != 0,
        }));
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

int32_t talos_byte_publisher_send(TalosNode* node, int64_t index,
                                  const void* data, uint32_t size) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    if (data == nullptr && size != 0) {
      throw std::invalid_argument("null message");
    }
    const auto* bytes = static_cast<const std::byte*>(data);
    return NodeAccess::byte_publisher_send(
               AsNode(node), Index(index),
               std::span<const std::byte>{bytes, size})
               ? 1
               : 0;
  });
}

int64_t talos_state_machine_create(TalosNode* node, int64_t initial) {
  return Guard<int64_t>(-1, [&] {
    return static_cast<int64_t>(
        NodeAccess::add_state_machine(AsNode(node), initial));
  });
}

int64_t talos_state_machine_state(const TalosNode* node, int64_t index) {
  return Guard<int64_t>(0, [&] {
    return NodeAccess::state_machine_state(AsNode(node), Index(index));
  });
}

int64_t talos_state_machine_previous(const TalosNode* node, int64_t index) {
  return Guard<int64_t>(0, [&] {
    return NodeAccess::state_machine_previous(AsNode(node), Index(index));
  });
}

int32_t talos_state_machine_go(TalosNode* node, int64_t index, int64_t next) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    NodeAccess::state_machine_go(AsNode(node), Index(index), next);
    return TALOS_OK;
  });
}

int64_t talos_state_machine_time_in_state_ns(const TalosNode* node,
                                             int64_t index) {
  return Guard<int64_t>(0, [&] {
    return NodeAccess::state_machine_time_in_state(AsNode(node), Index(index))
        .count();
  });
}

int32_t talos_state_machine_just_entered(const TalosNode* node, int64_t index) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    return NodeAccess::state_machine_just_entered(AsNode(node), Index(index))
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

int32_t talos_encoder_sample(TalosNode* node, int64_t index, TalosSample* out) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    TalosSample& o = *Out(out);
    const auto s = HardwareAccess::encoder_sample(Link(node), Index(index));
    o.valid = s.valid;
    o.position = s.position_rot;
    o.velocity = s.velocity_rps;
    return TALOS_OK;
  });
}

// --- Driver Station ----------------------------------------------------------

int32_t talos_driver_station(TalosNode* node, TalosCallback on_packet,
                             void* user) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    std::function<void()> callback;
    if (on_packet != nullptr) {
      callback = Callback(on_packet, user, "Driver Station callback");
    }
    // The handle is a view; the subscription and the callback live on the
    // node.
    talos::DriverStation{AsNode(node), std::move(callback)};
    return TALOS_OK;
  });
}

int32_t talos_driver_station_read(TalosNode* node, TalosDriverStation* out) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    if (out == nullptr) throw std::invalid_argument("null TalosDriverStation");
    const talos::DriverStation ds{AsNode(node)};
    *out = TalosDriverStation{
        .connected = ds.connected(),
        .enabled = ds.enabled(),
        .autonomous = ds.autonomous(),
        .teleop = ds.teleop(),
        .test = ds.test(),
        .estop = ds.estop(),
        .fms_attached = ds.fms_attached(),
        .alliance = static_cast<int32_t>(ds.alliance()),
        .station = ds.station(),
        .match_time_s = ds.match_time(),
    };
    for (std::size_t port = 0; port < TALOS_DS_MAX_JOYSTICKS; ++port) {
      out->joysticks[port] = JoystickOf(ds.joystick(port));
    }
    return TALOS_OK;
  });
}

}  // extern "C"
