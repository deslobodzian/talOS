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
#include <vector>

#include "talOS/devices/access.h"
#include "talOS/devices/devices.h"
#include "talOS/hardware/messages.h"
#include "talOS/node/access.h"
#include "talOS/node/node.h"
#include "talOS/node_api/internal.h"

namespace talos::node_api {
namespace {

thread_local std::string g_last_error;
// What TalosParam's pointers address, until the next talos_node_param call.
thread_local std::string g_param_string;
thread_local std::vector<std::string> g_param_strings;
thread_local std::vector<const char*> g_param_string_pointers;
thread_local std::vector<double> g_param_numbers;
// What talos_node_session_dir returns, until the next call.
thread_local std::string g_session_dir;

using detail::HardwareAccess;
using detail::NodeAccess;
using detail::TargetAccess;

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

// Every struct this ABI passes, field by field, as talOS/python/talos/_abi.py
// mirrors it with ctypes: talos_test.py reads these lines and checks the
// mirror against them. A layout change changes the ABI, so these numbers, the
// mirror and TALOS_NODE_ABI_VERSION change together.
static_assert(sizeof(TalosSchema) == 32);
static_assert(offsetof(TalosSchema, name) == 0);
static_assert(offsetof(TalosSchema, hash) == 8);
static_assert(offsetof(TalosSchema, bfbs) == 16);
static_assert(offsetof(TalosSchema, bfbs_size) == 24);
static_assert(sizeof(TalosParam) == 56);
static_assert(offsetof(TalosParam, kind) == 0);
static_assert(offsetof(TalosParam, boolean) == 4);
static_assert(offsetof(TalosParam, integer) == 8);
static_assert(offsetof(TalosParam, number) == 16);
static_assert(offsetof(TalosParam, string) == 24);
static_assert(offsetof(TalosParam, count) == 32);
static_assert(offsetof(TalosParam, numbers) == 40);
static_assert(offsetof(TalosParam, strings) == 48);
static_assert(sizeof(TalosSample) == 56);
static_assert(offsetof(TalosSample, valid) == 0);
static_assert(offsetof(TalosSample, value) == 4);
static_assert(offsetof(TalosSample, position) == 8);
static_assert(offsetof(TalosSample, velocity) == 16);
static_assert(offsetof(TalosSample, voltage) == 24);
static_assert(offsetof(TalosSample, current) == 32);
static_assert(offsetof(TalosSample, temperature_c) == 40);
static_assert(offsetof(TalosSample, supply_current_a) == 48);
static_assert(sizeof(TalosJoystick) == 116);
static_assert(offsetof(TalosJoystick, connected) == 0);
static_assert(offsetof(TalosJoystick, axis_count) == 4);
static_assert(offsetof(TalosJoystick, button_count) == 8);
static_assert(offsetof(TalosJoystick, pov_count) == 12);
static_assert(offsetof(TalosJoystick, buttons) == 16);
static_assert(offsetof(TalosJoystick, axes) == 20);
static_assert(offsetof(TalosJoystick, povs) == 68);
static_assert(offsetof(TalosJoystick, name) == 84);
static_assert(sizeof(TalosDriverStation) == 744);
static_assert(offsetof(TalosDriverStation, connected) == 0);
static_assert(offsetof(TalosDriverStation, enabled) == 4);
static_assert(offsetof(TalosDriverStation, autonomous) == 8);
static_assert(offsetof(TalosDriverStation, teleop) == 12);
static_assert(offsetof(TalosDriverStation, test) == 16);
static_assert(offsetof(TalosDriverStation, estop) == 20);
static_assert(offsetof(TalosDriverStation, fms_attached) == 24);
static_assert(offsetof(TalosDriverStation, alliance) == 28);
static_assert(offsetof(TalosDriverStation, station) == 32);
static_assert(offsetof(TalosDriverStation, match_time_s) == 40);
static_assert(offsetof(TalosDriverStation, joysticks) == 48);
static_assert(sizeof(TalosDriverStationPacket) == 712);
static_assert(offsetof(TalosDriverStationPacket, flags) == 0);
static_assert(offsetof(TalosDriverStationPacket, alliance) == 4);
static_assert(offsetof(TalosDriverStationPacket, station) == 8);
static_assert(offsetof(TalosDriverStationPacket, match_time_s) == 12);
static_assert(offsetof(TalosDriverStationPacket, joysticks) == 16);
static_assert(sizeof(TalosMotorRequest) == 24);
static_assert(offsetof(TalosMotorRequest, mode) == 0);
static_assert(offsetof(TalosMotorRequest, slot) == 4);
static_assert(offsetof(TalosMotorRequest, demand) == 8);
static_assert(offsetof(TalosMotorRequest, ff_volts) == 16);

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

void Fill(TalosParam& out, detail::ParamValue value) {
  std::visit(
      [&out](auto& v) {
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
        } else if constexpr (std::is_same_v<V, std::string>) {
          out.kind = TALOS_PARAM_STRING;
          g_param_string = std::move(v);
          out.string = g_param_string.c_str();
        } else if constexpr (std::is_same_v<V, std::vector<std::string>>) {
          out.kind = TALOS_PARAM_STRING_LIST;
          g_param_strings = std::move(v);
          g_param_string_pointers.clear();
          for (const std::string& s : g_param_strings) {
            g_param_string_pointers.push_back(s.c_str());
          }
          out.count = g_param_strings.size();
          out.strings = g_param_string_pointers.data();
        } else {
          static_assert(std::is_same_v<V, std::vector<double>>);
          out.kind = TALOS_PARAM_DOUBLE_LIST;
          g_param_numbers = std::move(v);
          out.count = g_param_numbers.size();
          out.numbers = g_param_numbers.data();
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
    case TALOS_PARAM_STRING_LIST:
      return wrap(NodeAccess::find_string_list(node, key));
    case TALOS_PARAM_DOUBLE_LIST:
      return wrap(NodeAccess::find_double_list(node, key));
    case TALOS_PARAM_LIST: {
      auto value = NodeAccess::find_param(node, key);
      if (value && !std::holds_alternative<std::vector<std::string>>(*value) &&
          !std::holds_alternative<std::vector<double>>(*value)) {
        NodeAccess::param_error(node, key, "must be an array");
      }
      return value;
    }
    default:
      throw std::invalid_argument("unknown parameter kind " +
                                  std::to_string(kind));
  }
}

}  // namespace

void SetError(std::string message) { g_last_error = std::move(message); }
void ClearError() { g_last_error.clear(); }
const std::string& LastError() { return g_last_error; }

}  // namespace talos::node_api

using namespace talos::node_api;
using talos::detail::HardwareAccess;
using talos::detail::NodeAccess;
using talos::detail::TargetAccess;

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

const char* talos_node_session_dir(const TalosNode* node) {
  return Guard<const char*>(nullptr, [&] {
    const auto dir = NodeAccess::session_dir(AsNode(node));
    if (!dir) return static_cast<const char*>(nullptr);
    g_session_dir = dir->string();
    return g_session_dir.c_str();
  });
}

int32_t talos_node_param(const TalosNode* node, const char* key, int32_t kind,
                         int32_t required, TalosParam* out) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    const talos::Node& n = AsNode(node);
    const std::string_view k = NonNull(key, "key");
    if (out == nullptr) throw std::invalid_argument("null TalosParam");
    *out = TalosParam{};
    auto value = FindParam(n, k, kind);
    if (!value) {
      if (required != 0) NodeAccess::param_error(n, k, "is missing");
      return 0;
    }
    Fill(*out, *std::move(value));
    return 1;
  });
}

// A TalosSchema as the identity the C++ registration takes.
static talos::schema::Identity Schema(const TalosSchema* schema) {
  if (schema == nullptr || schema->name == nullptr) {
    throw std::invalid_argument("null TalosSchema");
  }
  if (schema->bfbs == nullptr && schema->bfbs_size != 0) {
    throw std::invalid_argument("TalosSchema bfbs is null");
  }
  return talos::schema::Identity{
      schema->name, schema->hash,
      std::string{static_cast<const char*>(schema->bfbs), schema->bfbs_size}};
}

int64_t talos_node_subscribe(TalosNode* node, const char* topic,
                             uint32_t message_bytes, const TalosSchema* schema,
                             int32_t optional, TalosMessageCallback on_message,
                             void* user) {
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
        Schema(schema), optional != 0, std::move(callback));
    if (on_message != nullptr) NodeAccess::require_immediate_delivery(n);
    return static_cast<int64_t>(index);
  });
}

int64_t talos_node_publish(TalosNode* node, const char* topic,
                           uint32_t message_bytes, const TalosSchema* schema) {
  return Guard<int64_t>(-1, [&] {
    return static_cast<int64_t>(NodeAccess::add_publisher(
        AsNode(node), NonNull(topic, "topic"), message_bytes,
        talos::kMessageAlignment, Schema(schema)));
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

int32_t talos_hardware_sample_time(TalosNode* node, int64_t* out_ns) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    if (out_ns == nullptr) throw std::invalid_argument("null out_ns");
    const std::optional<talos::Time> sampled = Link(node).sample_time();
    if (!sampled) return 0;
    *out_ns = sampled->nanos();
    return 1;
  });
}

int32_t talos_hardware_fresh(TalosNode* node, int64_t max_age_ns) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    const talos::HardwareLink& link = Link(node);
    return static_cast<int32_t>(max_age_ns < 0
                                    ? link.fresh()
                                    : link.fresh(talos::Duration{max_age_ns}));
  });
}

int32_t talos_hardware_age(TalosNode* node, int64_t* out_ns) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    if (out_ns == nullptr) throw std::invalid_argument("null out_ns");
    const talos::Duration age = Link(node).age();
    if (age == talos::Duration::max()) return 0;
    *out_ns = age.count();
    return 1;
  });
}

int32_t talos_motor_set(TalosNode* node, int64_t index, int32_t mode,
                        double demand, double ff_volts, int32_t slot) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    HardwareAccess::motor_set(Link(node), Index(index), MotorMode(mode), demand,
                              ff_volts, slot);
    return TALOS_OK;
  });
}

int32_t talos_motor_rezero(TalosNode* node, int64_t index,
                           double position_rot) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    HardwareAccess::motor_rezero(Link(node), Index(index), position_rot);
    return TALOS_OK;
  });
}

int32_t talos_gyro_rezero(TalosNode* node, int64_t index, double yaw_rot) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    HardwareAccess::gyro_rezero(Link(node), Index(index), yaw_rot);
    return TALOS_OK;
  });
}

int32_t talos_motor_clamped_count(TalosNode* node, int64_t index,
                                  uint64_t* out) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    if (out == nullptr) throw std::invalid_argument("null out");
    *out = HardwareAccess::motor_clamped_count(Link(node), Index(index));
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
    o.temperature_c = s.temperature_c;
    o.supply_current_a = s.supply_current_a;
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

int32_t talos_motor_time(TalosNode* node, int64_t index, int32_t velocity,
                         int64_t* out_ns) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    if (out_ns == nullptr) throw std::invalid_argument("null out_ns");
    const auto time =
        HardwareAccess::motor_time(Link(node), Index(index), velocity != 0);
    if (!time) return 0;
    *out_ns = time->nanos();
    return 1;
  });
}

int32_t talos_sensor_time(TalosNode* node, int64_t index, int32_t velocity,
                          int64_t* out_ns) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    if (out_ns == nullptr) throw std::invalid_argument("null out_ns");
    const auto time =
        HardwareAccess::sensor_time(Link(node), Index(index), velocity != 0);
    if (!time) return 0;
    *out_ns = time->nanos();
    return 1;
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

// --- target lanes ------------------------------------------------------------

int64_t talos_target_open(TalosNode* node, uint32_t message_bytes,
                          uint32_t alignment, const TalosSchema* schema) {
  return Guard<int64_t>(-1, [&] {
    return static_cast<int64_t>(TargetAccess::open_target(
        AsNode(node), message_bytes, alignment, Schema(schema)));
  });
}

int32_t talos_target_get(TalosNode* node, int64_t target, void* out,
                         int32_t* source_index) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    talos::Node& n = AsNode(node);
    std::int32_t source = -1;
    const void* message = TargetAccess::target_get(n, Index(target), source);
    if (source_index != nullptr) *source_index = source;
    if (message == nullptr) return 0;
    if (out != nullptr) {
      std::memcpy(out, message, TargetAccess::target_bytes(n, Index(target)));
    }
    return 1;
  });
}

const char* talos_target_source_name(TalosNode* node, int64_t target,
                                     int32_t source_index) {
  return Guard<const char*>(nullptr, [&] {
    const auto sources =
        TargetAccess::target_sources(AsNode(node), Index(target));
    if (source_index < 0 ||
        static_cast<std::size_t>(source_index) >= sources.size()) {
      throw std::out_of_range("no target source " +
                              std::to_string(source_index));
    }
    return sources[static_cast<std::size_t>(source_index)].c_str();
  });
}

int64_t talos_target_lane_open(TalosNode* node, const char* subsystem,
                               const char* source, uint32_t message_bytes,
                               uint32_t alignment, const TalosSchema* schema) {
  return Guard<int64_t>(-1, [&] {
    return static_cast<int64_t>(TargetAccess::open_lane(
        AsNode(node), NonNull(subsystem, "subsystem"),
        NonNull(source, "source"), message_bytes, alignment, Schema(schema)));
  });
}

int32_t talos_target_lane_send(TalosNode* node, int64_t lane,
                               const void* message, uint32_t size) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    return TargetAccess::lane_send(AsNode(node), Index(lane), message, size)
               ? 1
               : 0;
  });
}

int32_t talos_target_lane_release(TalosNode* node, int64_t lane) {
  return Guard<int32_t>(TALOS_ERR, [&] {
    return TargetAccess::lane_release(AsNode(node), Index(lane)) ? 1 : 0;
  });
}

}  // extern "C"
