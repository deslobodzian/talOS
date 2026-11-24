#include "talOS/devices/devices.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <iterator>
#include <limits>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <utility>
#include <vector>

#include "bridge/bridge_status_generated.h"
#include "bridge/topics.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/devices/access.h"
#include "talOS/hardware/boot_history.h"
#include "talOS/hardware/feedback.h"
#include "talOS/hardware/gate.h"
#include "talOS/hardware/packet.h"
#include "talOS/introspection/names.h"
#include "talOS/node/access.h"

namespace talos {
namespace {

static_assert(std::string_view{hardware::kRequestTopicPrefix} ==
                  introspect::naming::kRequestTopicPrefix,
              "hardware::kRequestTopicPrefix must match "
              "introspect::naming::kRequestTopicPrefix");
static_assert(std::string_view{hardware::kDriverStationTopic} ==
                  introspect::naming::kDriverStationTopic,
              "hardware::kDriverStationTopic must match "
              "introspect::naming::kDriverStationTopic");

using detail::HardwareAccess;
using hardware::Packet;
using Kind = detail::DeviceKind;

bool IsOutput(Kind kind) {
  return kind == Kind::kMotor || kind == Kind::kDigitalOutput ||
         kind == Kind::kPwm;
}

const char* KindLabel(Kind kind) {
  switch (kind) {
    case Kind::kMotor:
      return "motor";
    case Kind::kCANcoder:
      return "CANcoder";
    case Kind::kGyro:
      return "gyro (Pigeon2)";
    case Kind::kDigitalInput:
      return "digital input";
    case Kind::kDigitalOutput:
      return "digital output";
    case Kind::kAnalogInput:
      return "analog input";
    case Kind::kEncoder:
      return "encoder";
    case Kind::kPwm:
      return "PWM output";
  }
  return "device";
}

const char* KindTable(Kind kind) {
  switch (kind) {
    case Kind::kMotor:
      return "motors";
    case Kind::kCANcoder:
    case Kind::kGyro:
      return "sensors";
    case Kind::kDigitalInput:
      return "digital_inputs";
    case Kind::kDigitalOutput:
      return "digital_outputs";
    case Kind::kAnalogInput:
      return "analog_inputs";
    case Kind::kEncoder:
      return "encoders";
    case Kind::kPwm:
      return "pwm_outputs";
  }
  return "devices";
}

// The sample for `id`, trying the slot it was found in last time first, so a
// state that keeps its devices in the same order from frame to frame costs one
// comparison per read instead of a scan.
template <class Sample, std::size_t N>
Sample FindSample(const std::array<Sample, N>& samples, std::size_t count,
                  std::uint16_t id, std::size_t& cached) {
  const std::size_t valid = std::min(count, N);
  if (cached < valid && samples[cached].id == id) return samples[cached];
  for (std::size_t i = 0; i < valid; ++i) {
    if (samples[i].id == id) {
      cached = i;
      return samples[i];
    }
  }
  return Sample{};
}

}  // namespace

std::uint16_t HardwareAccess::device_id(const config::RobotConfig& config,
                                        std::string_view subsystem, Kind kind,
                                        std::string_view name) {
  const hardware::Devices* owned = config.GetDevices(std::string{subsystem});
  const hardware::Config& hw = config.hardware;
  std::string have;

  auto search = [&](const auto& configs, const std::vector<std::uint16_t>& ids,
                    auto accept) -> std::optional<std::uint16_t> {
    for (const std::uint16_t id : ids) {
      for (const auto& device : configs) {
        if (device.id != id || !accept(device)) continue;
        if (device.name == name) return id;
        have += have.empty() ? "" : ", ";
        have += device.name;
      }
    }
    return std::nullopt;
  };
  auto any = [](const auto&) { return true; };
  auto sensor_of = [](hardware::SensorKind sensor) {
    return
        [sensor](const hardware::SensorConfig& c) { return c.kind == sensor; };
  };

  std::optional<std::uint16_t> found;
  if (owned != nullptr) {
    switch (kind) {
      case Kind::kMotor:
        found = search(hw.motors, owned->motors, any);
        break;
      case Kind::kCANcoder:
        found = search(hw.sensors, owned->sensors,
                       sensor_of(hardware::SensorKind::kCANcoder));
        break;
      case Kind::kGyro:
        found = search(hw.sensors, owned->sensors,
                       sensor_of(hardware::SensorKind::kPigeon2));
        break;
      case Kind::kDigitalInput:
        found = search(hw.digital_inputs, owned->digital_inputs, any);
        break;
      case Kind::kDigitalOutput:
        found = search(hw.digital_outputs, owned->digital_outputs, any);
        break;
      case Kind::kAnalogInput:
        found = search(hw.analog_inputs, owned->analog_inputs, any);
        break;
      case Kind::kEncoder:
        found = search(hw.encoders, owned->encoders, any);
        break;
      case Kind::kPwm:
        found = search(hw.pwm_outputs, owned->pwm_outputs, any);
        break;
    }
  }
  if (found) return *found;
  throw std::invalid_argument(
      std::string{subsystem} + ": no " + KindLabel(kind) + " named '" +
      std::string{name} + "' in [" + KindTable(kind) + "] of subsystem '" +
      std::string{subsystem} + "' (" +
      (have.empty() ? std::string{"it has none"} : "it has: " + have) + ")");
}

struct HardwareLink::Impl {
  struct Device {
    Kind kind;
    std::uint16_t id;
    std::string name;  // the config key the first handle named
    // NodeAccess::next_update of the update() that last commanded it (0:
    // never); commanded this tick only while it is the flushed update()'s,
    // so a command from an update() that threw never reaches a later flush.
    std::uint64_t set_update = 0;
    hardware::MotorRequest motor{};
    const hardware::MotorConfig* motor_config = nullptr;
    std::uint8_t hardware_index = 0;  // index in config.hardware.motors
    std::uint64_t clamped_count = 0;
    double position_offset = 0;  // Motor/Gyro software zero
    bool zeroed = false;         // position_offset was set by rezero
    // The position (yaw) of the last accepted State whose sample was valid,
    // in the hardware frame; 0 before one. A position read returns it while
    // the latched sample is invalid (HeldPosition).
    double held_position = 0;
    bool digital = false;
    hardware::PwmRequest pwm{};  // off: no pulse
    // Where this device's sample was in the state arrays last time it was
    // read; a hint only, checked against the id on every read.
    mutable std::size_t sample_index = 0;
  };

  Node& node;
  config::RobotConfig config;
  std::uint64_t config_id = 0;
  Duration timeout{};

  std::vector<Device> devices;
  // Held while add_device grows `devices` and while an off-loop-thread
  // setter reads a device's kind and name for its error (OffThreadLabel):
  // the only access to `devices` from another thread. The loop thread's own
  // reads take no lock, since only it writes.
  mutable std::mutex devices_growth;
  std::vector<std::uint16_t> motor_ids;    // flush order
  std::vector<std::size_t> motor_devices;  // devices[] index, same order
  bool has_outputs = false;
  std::size_t request = 0;  // publisher index on node
  // NodeAccess::next_update of the update() in which a talos::Target of this
  // node last returned a target (note_live_target; 0: never). The flush
  // drives only when it is the update() that just ran.
  std::uint64_t live_target_update = 0;
  // The stderr line a flush prints when it holds outputs neutral for want
  // of a live target, built with the first output so the tick allocates
  // nothing; printed once per enabled spell.
  std::string withheld_line;
  bool withheld_reported = false;

  hardware::State state{};
  bool have_state = false;
  hardware::BootHistory boots;
  Time state_published{};
  std::size_t state_subscription = 0;
  // The bridge's gateway-to-node clock mapping. The state is the gateway's
  // canonical wire bytes, which the bridge relays untouched (its
  // sample_time_us is what every request echoes, on the gateway clock), so
  // the mapping travels beside it on the bridge's own /hw/status, stamped
  // with the gateway boot it describes. The offset moves only with clock
  // drift and the filter's window, so the newest status (at most 20 ms old)
  // maps the newest state.
  Subscription<::bridge::BridgeStatus> bridge_status;

  explicit Impl(Node& n) : node{n} {}

  bool Fresh() const {
    return have_state && node.now() - state_published < timeout;
  }
  Duration Age() const {
    return have_state ? node.now() - state_published : Duration::max();
  }
  bool Enabled() const { return Fresh() && hardware::GatewayReady(state); }
  std::optional<Time> SampleTime() const {
    if (!Fresh() || !bridge_status.fresh(timeout)) return std::nullopt;
    const ::bridge::BridgeStatus& clock = bridge_status.value();
    if (!clock.clock_offset_valid() || clock.clock_boot_id() != state.boot_id) {
      return std::nullopt;
    }
    constexpr std::int64_t kMaxUs =
        std::numeric_limits<std::int64_t>::max() / 1000;
    constexpr std::int64_t kMinUs =
        std::numeric_limits<std::int64_t>::min() / 1000;
    const std::uint64_t sample_us = state.sample_time_us;
    const std::int64_t offset_us = clock.clock_offset_us();
    std::int64_t us = 0;
    if (offset_us >= 0) {
      const auto max_us = static_cast<std::uint64_t>(kMaxUs);
      const auto off = static_cast<std::uint64_t>(offset_us);
      if (sample_us > max_us || off > max_us - sample_us) return std::nullopt;
      us = static_cast<std::int64_t>(sample_us + off);
    } else {
      const std::uint64_t neg =
          static_cast<std::uint64_t>(-(offset_us + 1)) + 1;
      if (sample_us >= neg) {
        const std::uint64_t diff = sample_us - neg;
        if (diff > static_cast<std::uint64_t>(kMaxUs)) return std::nullopt;
        us = static_cast<std::int64_t>(diff);
      } else {
        const std::uint64_t mag = neg - sample_us;
        if (mag > static_cast<std::uint64_t>(-kMinUs)) return std::nullopt;
        us = -static_cast<std::int64_t>(mag);
      }
    }
    return Time::from_nanos(us * 1000);
  }

  Device& At(std::size_t index, Kind kind) {
    if (index >= devices.size() || devices[index].kind != kind) {
      throw std::out_of_range(std::string{node.name()} + ": no " +
                              KindLabel(kind) + " handle at index " +
                              std::to_string(index));
    }
    return devices[index];
  }
  const Device& At(std::size_t index, Kind kind) const {
    return const_cast<Impl*>(this)->At(index, kind);
  }
  // The device an output setter commands. Outputs are taken only on the
  // loop thread, inside update(): Flush() runs after each update() and sends
  // neutral for every output that update() did not set, so a command from
  // the constructor, every() or a message callback would reach the hardware
  // for at most one tick (from a timer slower than the tick, a pulse).
  // Off the loop thread a setter throws naming its device, read under
  // devices_growth: the kind and name, fixed once the device is added.
  Device& Output(std::size_t index, Kind kind, std::string_view setter) {
    try {
      detail::NodeAccess::require_loop_thread(node, setter);
    } catch (const std::logic_error&) {
      throw std::logic_error(
          std::string{node.name()} + ": " + std::string{setter} + " on " +
          OffThreadLabel(index, kind) + " must run on the node's loop thread");
    }
    Device& device = At(index, kind);
    if (!detail::NodeAccess::in_update(node)) {
      throw std::logic_error(
          std::string{node.name()} + ": " + std::string{setter} + " on " +
          KindLabel(kind) + " '" + device.name +
          "' called outside update(); set outputs in update(), since the "
          "flush after each update() sends neutral for every output it did "
          "not set");
    }
    return device;
  }
  // Called off the loop thread. The lock covers only reads and a copy into
  // capacity reserved before it was taken, never an allocation, so the loop
  // thread in add_device never waits on this thread's allocator: the first
  // pass reads the name's size, the reserve happens unlocked, and the
  // second pass assigns within that capacity (a name never changes once
  // added, so its size cannot have grown).
  std::string OffThreadLabel(std::size_t index, Kind kind) const {
    std::string name;
    bool found = false;
    for (;;) {
      std::size_t size = 0;
      {
        const std::lock_guard lock{devices_growth};
        found = index < devices.size() && devices[index].kind == kind;
        if (!found) break;
        size = devices[index].name.size();
        if (size <= name.capacity()) {
          name.assign(devices[index].name);
          break;
        }
      }
      name.reserve(size);
    }
    if (!found) {
      return std::string{KindLabel(kind)} + " handle " + std::to_string(index);
    }
    return std::string{KindLabel(kind)} + " '" + name + "'";
  }
  const Device& SensorAt(std::size_t index) const {
    if (index < devices.size() && (devices[index].kind == Kind::kGyro ||
                                   devices[index].kind == Kind::kCANcoder)) {
      return devices[index];
    }
    throw std::out_of_range(std::string{node.name()} +
                            ": no sensor handle at index " +
                            std::to_string(index));
  }

  // Decoded, then latched only when AcceptState takes it: this config's id,
  // not a recently retired gateway boot, and newer than the last sample from
  // the same boot.
  void OnState(const void* data) {
    Packet packet{};
    std::memcpy(&packet, data, sizeof packet);
    hardware::State next{};
    if (!hardware::Decode(packet.bytes(), next)) return;
    if (!hardware::AcceptState(have_state, state, config_id, next,
                               boots.retired(next.boot_id))) {
      return;
    }
    if (!boots.current(next.boot_id)) boots.Enter(next.boot_id);
    if (have_state) ForgetZeros(next);
    state = next;
    have_state = true;
    HoldPositions();
    state_published = node.now() - detail::NodeAccess::subscription_age(
                                       node, state_subscription);
  }

  // A software zero holds only in the hardware frame it was taken in, and a
  // gateway boot change or a device reset may have restarted that frame.
  // ForgetZeros runs on each accepted state, with `state` still the one
  // before it:
  //   - a boot id change drops every zero (a new boot may come with
  //     power-cycled devices);
  //   - a device_reset_count change with next.sequence == state.sequence + 1
  //     (the gateway tick right after the one seen last) drops the zeros of
  //     this node's devices that `next` masks: the gateway counts a reset
  //     and masks the device, with its followers and remote-sensor
  //     consumers, in the same State;
  //   - any other device_reset_count change drops every zero: the masking
  //     State may have been lost (a link outage, a bridge restart, dropped
  //     datagrams, a tick with no socket) and the device restored since.
  // A motor whose config sets position_frame = "absolute" keeps its zero:
  // its remote CANcoder's frame survives a reset. A dropped zero keeps its
  // offset, so dropping does not move position() or yaw(); it reads zeroed()
  // false and holds the motor's position goals neutral until the node
  // rezeroes.
  void ForgetZeros(const hardware::State& next) {
    const bool reboot = next.boot_id != state.boot_id;
    if (!reboot &&
        next.counters.device_reset_count == state.counters.device_reset_count) {
      return;
    }
    const bool consecutive = !reboot && next.sequence == state.sequence + 1;
    for (Device& device : devices) {
      if (!device.zeroed || AbsoluteFrame(device)) continue;
      if (consecutive && Valid(next, device)) continue;
      device.zeroed = false;
    }
  }

  // Records each position-reading device's position from the new state
  // while its sample there is valid. A loop over this node's handles, with
  // no allocation.
  void HoldPositions() {
    for (Device& device : devices) {
      switch (device.kind) {
        case Kind::kMotor:
          Hold(device, state.motors, state.motor_count);
          break;
        case Kind::kCANcoder:
        case Kind::kGyro:
          Hold(device, state.sensors, state.sensor_count);
          break;
        case Kind::kEncoder:
          Hold(device, state.encoders, state.encoder_count);
          break;
        default:
          break;
      }
    }
  }
  template <class Sample, std::size_t N>
  static void Hold(Device& device, const std::array<Sample, N>& samples,
                   std::size_t count) {
    const Sample sample =
        FindSample(samples, count, device.id, device.sample_index);
    if (sample.valid) device.held_position = sample.position_rot;
  }

  static bool AbsoluteFrame(const Device& device) {
    return device.motor_config != nullptr &&
           device.motor_config->absolute_position_frame.value;
  }

  static bool Valid(const hardware::State& in, Device& device) {
    return device.kind == Kind::kMotor
               ? FindSample(in.motors, in.motor_count, device.id,
                            device.sample_index)
                     .valid
               : FindSample(in.sensors, in.sensor_count, device.id,
                            device.sample_index)
                     .valid;
  }

  // Records that the running update() commanded `device`.
  void Commanded(Device& device) const {
    device.set_update = detail::NodeAccess::next_update(node);
  }

  // Whether update() `tick` set an output to something other than neutral: a
  // motor mode other than neutral, a digital output true or a PWM output to
  // a pulse (0, the centre pulse, included).
  bool SetNonNeutral(std::uint64_t tick) const {
    for (const Device& device : devices) {
      if (device.set_update != tick) continue;
      if ((device.kind == Kind::kMotor &&
           device.motor.mode != hardware::Mode::kNeutral) ||
          (device.kind == Kind::kDigitalOutput && device.digital) ||
          (device.kind == Kind::kPwm &&
           device.pwm.mode != hardware::PwmMode::kOff)) {
        return true;
      }
    }
    return false;
  }

  // RULES.md 1: a tick drives only while robot_enabled and, unless the node
  // called AllowUntargetedOutputs, a talos::Target of the node returned a
  // target during the update() that just ran (NodeAccess::next_update reads
  // that update()'s index here), and it sends only what that update() set.
  // A tick that does not drive sends every output neutral and still sends
  // the request.
  void Flush() {
    const std::uint64_t tick = detail::NodeAccess::next_update(node);
    const bool targeted = detail::NodeAccess::untargeted_outputs(node) ||
                          live_target_update == tick;
    const bool enabled = Enabled();
    if (have_state) {
      hardware::Command command = hardware::NeutralCommand(state, motor_ids);
      const bool drive = enabled && targeted;
      for (std::size_t i = 0; i < command.count; ++i) {
        const Device& motor = devices[motor_devices[i]];
        if (drive && motor.set_update == tick) command.motors[i] = motor.motor;
      }
      for (const Device& device : devices) {
        if (device.set_update != tick) continue;
        if (device.kind == Kind::kDigitalOutput &&
            command.digital_output_count < hardware::kMaxDigitalOutputs) {
          command.digital_outputs[command.digital_output_count++] = {
              device.id, drive && device.digital};
        } else if (device.kind == Kind::kPwm &&
                   command.pwm_output_count < hardware::kMaxPwmOutputs) {
          command.pwm_outputs[command.pwm_output_count++] =
              drive ? device.pwm : hardware::PwmRequest{.id = device.id};
        }
      }
      Packet packet{};
      packet.size =
          static_cast<std::uint32_t>(hardware::Encode(command, packet.data));
      if (packet.size > 0) {
        detail::NodeAccess::publisher_send(node, request, &packet,
                                           sizeof packet);
      }
    }
    // Outputs held back: enabled, not targeted, and update() set one
    // non-neutral. The node's registry row carries it while it lasts
    // (NodeAccess::set_outputs_held: a store only when it changes), and the
    // first tick of each enabled spell that holds also writes one line,
    // whole with one fwrite from the line built at registration: no
    // allocation on the tick.
    const bool held = enabled && !targeted && SetNonNeutral(tick);
    detail::NodeAccess::set_outputs_held(node, held);
    if (!enabled) {
      withheld_reported = false;
    } else if (held && !withheld_reported) {
      std::fwrite(withheld_line.data(), 1, withheld_line.size(), stderr);
      withheld_reported = true;
    }
  }
};

HardwareLink::HardwareLink(Node& node) : impl_{std::make_unique<Impl>(node)} {
  Impl& impl = *impl_;
  // A copy of the config the node already parsed, not a second parse: the
  // commissioned bit below must not leak into the node's own view.
  impl.config = detail::NodeAccess::robot_config(node);
  // The bridge treats the config as commissioned under --sim, and the
  // commissioned bit is part of the config id both ends compare.
  if (node.simulation()) impl.config.hardware.commissioned = true;
  impl.config_id = hardware::ConfigurationId(impl.config.hardware);
  impl.timeout =
      std::chrono::microseconds{impl.config.hardware.command_timeout_us};
  impl.state_subscription = detail::NodeAccess::add_subscription(
      node, hardware::kStateTopic, sizeof(Packet), alignof(Packet),
      schema::Of<Packet>(), /*optional=*/false,
      [&impl](const void* data) { impl.OnState(data); });
  impl.bridge_status =
      node.subscribe<::bridge::BridgeStatus>(::bridge::topics::kStatus);
}

HardwareLink::~HardwareLink() = default;

bool HardwareLink::enabled() const { return impl_->Enabled(); }

bool HardwareLink::fresh() const { return impl_->Fresh(); }

bool HardwareLink::fresh(Duration max_age) const {
  return impl_->have_state && impl_->Age() < max_age;
}

Duration HardwareLink::age() const { return impl_->Age(); }

std::optional<Time> HardwareLink::sample_time() const {
  return impl_->SampleTime();
}

const config::RobotConfig& HardwareLink::config() const {
  return impl_->config;
}

bool robot_enabled(Node& node) {
  return node.extension<HardwareLink>().enabled();
}

void AllowUntargetedOutputs(Node& node) {
  detail::NodeAccess::mark_untargeted_outputs(node);
}

void HardwareAccess::note_live_target(HardwareLink& link) {
  HardwareLink::Impl& impl = *link.impl_;
  impl.live_target_update = detail::NodeAccess::next_update(impl.node);
}

std::uint64_t HardwareAccess::boot_id(const HardwareLink& link) {
  const HardwareLink::Impl& impl = *link.impl_;
  return impl.have_state ? impl.state.boot_id : 0;
}

void RequireInUpdate(const Node& node, std::string_view what) {
  detail::NodeAccess::require_loop_thread(node, what);
  if (!detail::NodeAccess::in_update(node)) {
    throw std::logic_error(std::string{node.name()} + ": " + std::string{what} +
                           " called outside update(); make requests in "
                           "update(), where they apply to this tick");
  }
}

namespace {

Duration StatusPeriodsBound(double status_hz, double periods, bool ceil_round) {
  // INT64_MAX / 1000.0 = 9223372036854776.0 in IEEE-754 double, so strictly
  // less yields at most 9223372036854774.0, guaranteeing us * 1000 <=
  // INT64_MAX.
  constexpr double kMaxMicros =
      static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 1000.0;
  if (!std::isfinite(status_hz) || status_hz <= 0.0) {
    throw std::invalid_argument("status_hz must be finite and positive");
  }
  const double raw_us = periods * 1e6 / status_hz;
  const double rounded_us = ceil_round ? std::ceil(raw_us) : std::round(raw_us);
  if (!std::isfinite(rounded_us) || rounded_us >= kMaxMicros) {
    throw std::invalid_argument("status_hz is too small for age bound");
  }
  return std::chrono::microseconds{static_cast<std::int64_t>(rounded_us)};
}

}  // namespace

Duration RezeroAgeBound(const hardware::Config& config) {
  constexpr double kStatusPeriods = 2;
  return StatusPeriodsBound(config.status_hz, kStatusPeriods,
                            /*ceil_round=*/false);
}

Duration ReadinessAgeBound(const hardware::Config& config) {
  // The RoboRIO's LATE threshold, rounded as it rounds it.
  constexpr double kStatusPeriods = 4;
  return StatusPeriodsBound(config.status_hz, kStatusPeriods,
                            /*ceil_round=*/true);
}

namespace {

// Refuses post-start registration without mutating node state:
// Node::after_update checks `started` (throwing std::logic_error) before
// rejecting an empty callback (throwing std::invalid_argument).
void RequireConstruction(Node& node, std::string_view what) {
  try {
    node.after_update({});
  } catch (const std::invalid_argument&) {
    return;
  } catch (const std::logic_error&) {
    throw std::logic_error(std::string{node.name()} + ": " + std::string{what} +
                           " must be called in the constructor");
  }
}

}  // namespace

std::size_t HardwareAccess::add_device(HardwareLink& link, Kind kind,
                                       std::string_view name) {
  HardwareLink::Impl& impl = *link.impl_;
  detail::NodeAccess::require_loop_thread(impl.node, "device handle");
  const std::uint16_t id = device_id(impl.config, impl.node.name(), kind, name);
  for (std::size_t i = 0; i < impl.devices.size(); ++i) {
    if (impl.devices[i].id == id && impl.devices[i].kind == kind) return i;
  }
  RequireConstruction(impl.node, "device handle");
  if (kind == Kind::kMotor && impl.motor_ids.size() >= hardware::kMaxMotors) {
    throw std::invalid_argument(std::string{impl.node.name()} + ": more than " +
                                std::to_string(hardware::kMaxMotors) +
                                " motors");
  }
  // Input-only nodes publish no request: the bridge reads one only from
  // subsystems that own actuators, and an unread topic is a graph error.
  if (IsOutput(kind) && !impl.has_outputs) {
    // hardware::RequestTopic(name), through the one registration the node
    // layer lets write it (RULES.md 2).
    impl.request = detail::NodeAccess::add_hardware_request(
        impl.node, sizeof(Packet), alignof(Packet), schema::Of<Packet>());
    impl.node.after_update([&impl] { impl.Flush(); });
    impl.has_outputs = true;
    impl.withheld_line =
        std::string{impl.node.name()} +
        ": held every output neutral: update() set one in a tick in which no "
        "talos::Target returned a target (RULES.md 1); set each output from "
        "the target get() returned in that update(), never from a joystick "
        "or a value kept from an earlier tick\n";
  }
  if (kind == Kind::kMotor) {
    impl.motor_ids.push_back(id);
    impl.motor_devices.push_back(impl.devices.size());
  }
  {
    const std::lock_guard lock{impl.devices_growth};
    impl.devices.push_back({.kind = kind, .id = id, .name = std::string{name}});
  }
  if (kind == Kind::kMotor) {
    static_assert(hardware::kMaxMotors <= 32,
                  "motor_fault_mask is uint32_t and indexes motors by bit");
    static_assert(
        hardware::kMaxMotors <= std::numeric_limits<std::uint8_t>::max(),
        "Motor::hardware_index returns uint8_t");
    for (std::size_t i = 0; i < impl.config.hardware.motors.size(); ++i) {
      if (impl.config.hardware.motors[i].id == id) {
        impl.devices.back().motor_config = &impl.config.hardware.motors[i];
        impl.devices.back().hardware_index = static_cast<std::uint8_t>(i);
        break;
      }
    }
  }
  return impl.devices.size() - 1;
}

void HardwareAccess::motor_set(HardwareLink& link, std::size_t index,
                               hardware::Mode mode, double demand,
                               double ff_volts, int slot) {
  HardwareLink::Impl::Device& device =
      link.impl_->Output(index, Kind::kMotor, "Motor command");
  if (static_cast<std::uint8_t>(mode) >
      static_cast<std::uint8_t>(hardware::kLastMode)) {
    throw std::invalid_argument(std::string{link.impl_->node.name()} +
                                ": unknown motor mode " +
                                std::to_string(static_cast<int>(mode)));
  }
  if (slot < 0 || slot > 2) {
    throw std::invalid_argument(std::string{link.impl_->node.name()} +
                                ": motor slot " + std::to_string(slot) +
                                " is not 0, 1 or 2");
  }
  if (ff_volts != 0 && mode != hardware::Mode::kVelocity &&
      mode != hardware::Mode::kPosition &&
      mode != hardware::Mode::kMotionMagic) {
    // Only velocity, position and motion magic carry a feedforward; the
    // gateway neutralizes any other request with one (Gateway::Accept), which
    // testing::SimHardware would apply as sent. No Motor setter sends one: a
    // raw caller (the C ABI) fails here, and this call sets nothing: a motor
    // not otherwise set in the tick is sent neutral.
    throw std::invalid_argument(std::string{link.impl_->node.name()} +
                                ": motor '" + device.name +
                                "' takes no feedforward in this mode; "
                                "ff_volts must be 0");
  }
  const auto& config = *device.motor_config;
  if (mode == hardware::Mode::kTorqueCurrent &&
      !(config.foc && config.max_torque_current_a > 0)) {
    // TorqueCurrentFOC is a Phoenix Pro request bounded only by the motor's
    // own limit: a motor without both keys could only be neutralized by the
    // gateway, so a node asking for it fails in its first update(), in Sim
    // too, naming what to set.
    throw std::invalid_argument(
        std::string{link.impl_->node.name()} +
        ": Motor::set_torque_current on motor '" + device.name +
        "' needs [motors." + device.name + "] " +
        (config.foc ? "max_torque_current_a > 0"
                    : "foc = true and max_torque_current_a > 0"));
  }
  if (mode == hardware::Mode::kPosition ||
      mode == hardware::Mode::kMotionMagic) {
    // A position needs a frame that no reset has restarted since it was
    // taken: a rezero that still holds, or a remote CANcoder's absolute frame
    // ([motors.*] position_frame = "absolute"). The rotor position a TalonFX
    // powers on or resets with is no frame: neutral until the node rezeroes.
    // Other modes, torque current included, need no frame.
    if (!device.zeroed && !config.absolute_position_frame.value) {
      device.motor = {.id = device.id};
      link.impl_->Commanded(device);
      return;
    }
    demand += device.position_offset;
  }
  const double requested_demand = demand;
  const double requested_ff = ff_volts;
  // A non-finite demand is no demand: neutral, rather than handing the
  // gateway a NaN. A neutral request always carries zero demand, zero
  // feedforward and slot 0 so it matches Motor::stop() and the gateway's
  // neutral representation.
  if (!std::isfinite(demand) || !std::isfinite(ff_volts) ||
      mode == hardware::Mode::kNeutral) {
    mode = hardware::Mode::kNeutral;
    demand = 0;
    ff_volts = 0;
    slot = 0;
  }
  if (mode != hardware::Mode::kNeutral) {
    switch (mode) {
      case hardware::Mode::kDutyCycle:
        demand = std::clamp(demand, -1.0, 1.0);
        break;
      case hardware::Mode::kVoltage:
        demand = std::clamp(demand, -config.max_voltage, config.max_voltage);
        break;
      case hardware::Mode::kVelocity:
        demand = std::clamp(demand, -config.max_velocity_rps,
                            config.max_velocity_rps);
        break;
      case hardware::Mode::kPosition:
      case hardware::Mode::kMotionMagic:
        if (config.soft_limits) {
          demand = std::clamp(demand, config.reverse_limit_rot,
                              config.forward_limit_rot);
        }
        break;
      case hardware::Mode::kTorqueCurrent:  // 0 A drives: not neutral
        demand = std::clamp(demand, -config.max_torque_current_a,
                            config.max_torque_current_a);
        break;
      case hardware::Mode::kNeutral:
        break;
    }
    ff_volts = std::clamp(ff_volts, -config.max_voltage, config.max_voltage);
  }
  if (demand != requested_demand || ff_volts != requested_ff) {
    ++device.clamped_count;
  }
  device.motor = {.id = device.id,
                  .mode = mode,
                  .slot = static_cast<std::uint8_t>(slot),
                  .demand = demand,
                  .feedforward_v = ff_volts};
  link.impl_->Commanded(device);
}

std::uint64_t HardwareAccess::motor_clamped_count(const HardwareLink& link,
                                                  std::size_t index) {
  return link.impl_->At(index, Kind::kMotor).clamped_count;
}

std::uint8_t HardwareAccess::motor_hardware_index(const HardwareLink& link,
                                                  std::size_t index) {
  return link.impl_->At(index, Kind::kMotor).hardware_index;
}

void HardwareAccess::motor_rezero(HardwareLink& link, std::size_t index,
                                  double position_rot) {
  detail::NodeAccess::require_loop_thread(link.impl_->node, "Motor::rezero");
  auto& device = link.impl_->At(index, Kind::kMotor);
  const auto& impl = *link.impl_;
  const auto sample = FindSample(impl.state.motors, impl.state.motor_count,
                                 device.id, device.sample_index);
  const double offset = sample.position_rot - position_rot;
  // What a rezero pairs the position with (a switch, a stop, a constant, a
  // CANcoder its caller checked) is as of the State, so the position must
  // be too (RezeroAgeBound); an unknown age, or a disconnected sample, never
  // is.
  const std::optional<Duration> age = motor_age(link, index, false);
  const bool recent = age && *age <= RezeroAgeBound(impl.config.hardware);
  if (recent && std::isfinite(sample.position_rot) &&
      std::isfinite(position_rot) && std::isfinite(offset)) {
    device.position_offset = offset;
    device.zeroed = true;
  }
}

bool HardwareAccess::motor_zeroed(const HardwareLink& link, std::size_t index) {
  return link.impl_->At(index, Kind::kMotor).zeroed;
}

void HardwareAccess::gyro_rezero(HardwareLink& link, std::size_t index,
                                 double yaw_rot) {
  detail::NodeAccess::require_loop_thread(link.impl_->node, "Gyro::rezero");
  auto& device = link.impl_->At(index, Kind::kGyro);
  const auto& impl = *link.impl_;
  const auto sample = FindSample(impl.state.sensors, impl.state.sensor_count,
                                 device.id, device.sample_index);
  const double offset = sample.position_rot - yaw_rot;
  if (impl.Fresh() && sample.valid && std::isfinite(sample.position_rot) &&
      std::isfinite(yaw_rot) && std::isfinite(offset)) {
    device.position_offset = offset;
    device.zeroed = true;
  }
}

bool HardwareAccess::gyro_zeroed(const HardwareLink& link, std::size_t index) {
  return link.impl_->At(index, Kind::kGyro).zeroed;
}

namespace {
// A motor or CAN-sensor sample as a node reads it. An invalid one, whether
// the gateway invalidated it or the link is stale (no State younger than
// command_timeout_us, which a node cannot tell apart through connected()),
// carries the gateway's zeros (hardware::MarkInvalid: rate and effort fields
// zero, temperature and fault bits as last reported) with the last valid
// position in place of its zero. Every position reads through the rezero
// offset.
template <class Impl, class Device, class Sample>
void InvalidateUnlessFresh(const Impl& impl, const Device& device,
                           Sample& sample) {
  if (!sample.valid || !impl.Fresh()) {
    hardware::MarkInvalid(sample);
    sample.position_rot = device.held_position;
  }
  sample.position_rot -= device.position_offset;
}
}  // namespace

hardware::MotorSample HardwareAccess::motor_sample(const HardwareLink& link,
                                                   std::size_t index) {
  const auto& impl = *link.impl_;
  const auto& device = impl.At(index, Kind::kMotor);
  auto sample = FindSample(impl.state.motors, impl.state.motor_count, device.id,
                           device.sample_index);
  InvalidateUnlessFresh(impl, device, sample);
  return sample;
}

bool HardwareAccess::motor_hard_limit(const hardware::MotorSample& sample,
                                      bool forward) {
  const std::uint32_t flag = forward ? hardware::kMotorForwardHardLimitFlag
                                     : hardware::kMotorReverseHardLimitFlag;
  return sample.valid && (sample.fault_flags & flag) != 0;
}

bool HardwareAccess::motor_hard_limit(const HardwareLink& link,
                                      std::size_t index, bool forward) {
  return motor_hard_limit(motor_sample(link, index), forward);
}

hardware::SensorSample HardwareAccess::sensor_sample(const HardwareLink& link,
                                                     std::size_t index) {
  const auto& impl = *link.impl_;
  const auto& device = impl.SensorAt(index);
  auto sample = FindSample(impl.state.sensors, impl.state.sensor_count,
                           device.id, device.sample_index);
  InvalidateUnlessFresh(impl, device, sample);
  return sample;
}

namespace {
template <class Sample>
std::optional<Duration> SignalAge(const Sample& sample, bool velocity) {
  const auto age = velocity ? sample.velocity_age_us : sample.position_age_us;
  if (!sample.valid || age == UINT32_MAX) return std::nullopt;
  return std::chrono::microseconds{age};
}

template <class Sample>
std::optional<Time> AcquisitionTime(const HardwareLink& link,
                                    const Sample& sample, bool velocity) {
  const auto age = SignalAge(sample, velocity);
  if (!age) return std::nullopt;
  const auto time = link.sample_time();
  if (!time) return std::nullopt;
  if (time->nanos() < std::numeric_limits<std::int64_t>::min() + age->count()) {
    return std::nullopt;
  }
  return *time - *age;
}
}  // namespace

std::optional<Time> HardwareAccess::motor_time(const HardwareLink& link,
                                               std::size_t index,
                                               bool velocity) {
  return AcquisitionTime(link, motor_sample(link, index), velocity);
}

std::optional<Time> HardwareAccess::sensor_time(const HardwareLink& link,
                                                std::size_t index,
                                                bool velocity) {
  return AcquisitionTime(link, sensor_sample(link, index), velocity);
}

std::optional<Duration> HardwareAccess::motor_age(const HardwareLink& link,
                                                  std::size_t index,
                                                  bool velocity) {
  return SignalAge(motor_sample(link, index), velocity);
}

std::optional<Duration> HardwareAccess::sensor_age(const HardwareLink& link,
                                                   std::size_t index,
                                                   bool velocity) {
  return SignalAge(sensor_sample(link, index), velocity);
}

hardware::DigitalInputSample HardwareAccess::digital_input_sample(
    const HardwareLink& link, std::size_t index) {
  const auto& impl = *link.impl_;
  const auto& device = impl.At(index, Kind::kDigitalInput);
  auto sample =
      FindSample(impl.state.digital_inputs, impl.state.digital_input_count,
                 device.id, device.sample_index);
  if (!impl.Fresh()) sample.valid = false;
  return sample;
}

hardware::AnalogInputSample HardwareAccess::analog_input_sample(
    const HardwareLink& link, std::size_t index) {
  const auto& impl = *link.impl_;
  const auto& device = impl.At(index, Kind::kAnalogInput);
  auto sample =
      FindSample(impl.state.analog_inputs, impl.state.analog_input_count,
                 device.id, device.sample_index);
  if (!impl.Fresh()) sample.valid = false;
  return sample;
}

hardware::EncoderSample HardwareAccess::encoder_sample(const HardwareLink& link,
                                                       std::size_t index) {
  const auto& impl = *link.impl_;
  const auto& device = impl.At(index, Kind::kEncoder);
  auto sample = FindSample(impl.state.encoders, impl.state.encoder_count,
                           device.id, device.sample_index);
  // As a motor's or CAN sensor's (InvalidateUnlessFresh): invalid, or on a
  // stale link, the rate reads zero and the position its last valid value.
  if (!sample.valid || !impl.Fresh()) {
    sample.valid = false;
    sample.position_rot = device.held_position;
    sample.velocity_rps = 0;
  }
  return sample;
}

void HardwareAccess::digital_output_set(HardwareLink& link, std::size_t index,
                                        bool value) {
  auto& device =
      link.impl_->Output(index, Kind::kDigitalOutput, "DigitalOutput::set");
  device.digital = value;
  link.impl_->Commanded(device);
}

void HardwareAccess::pwm_set(HardwareLink& link, std::size_t index,
                             double output) {
  auto& device = link.impl_->Output(index, Kind::kPwm, "PwmOutput::set");
  device.pwm =
      std::isfinite(output)
          ? hardware::PwmRequest{.id = device.id,
                                 .mode = hardware::PwmMode::kPulse,
                                 .output = std::clamp(output, -1.0, 1.0)}
          : hardware::PwmRequest{.id = device.id};
  link.impl_->Commanded(device);
}

void HardwareAccess::pwm_stop(HardwareLink& link, std::size_t index) {
  auto& device = link.impl_->Output(index, Kind::kPwm, "PwmOutput::stop");
  device.pwm = hardware::PwmRequest{.id = device.id};
  link.impl_->Commanded(device);
}

// --- handles -----------------------------------------------------------------

namespace {

HardwareLink* LinkOf(Node& node) {
  detail::NodeAccess::require_loop_thread(node, "device handle");
  if (detail::NodeAccess::find_extension(node, typeid(HardwareLink)) ==
      nullptr) {
    RequireConstruction(node, "device handle");
  }
  return &node.extension<HardwareLink>();
}

HardwareLink& RequireLink(HardwareLink* link, const char* what) {
  if (link == nullptr) {
    throw std::logic_error(std::string{what} +
                           " handle is not bound to a Node");
  }
  return *link;
}

}  // namespace

using hardware::Mode;

Motor::Motor(Node& node, std::string_view name)
    : link_{LinkOf(node)},
      index_{HardwareAccess::add_device(*link_, Kind::kMotor, name)} {}
void Motor::set_duty_cycle(double fraction) {
  HardwareAccess::motor_set(RequireLink(link_, "Motor"), index_,
                            Mode::kDutyCycle, fraction, 0, 0);
}
void Motor::set_voltage(double volts) {
  HardwareAccess::motor_set(RequireLink(link_, "Motor"), index_, Mode::kVoltage,
                            volts, 0, 0);
}
void Motor::set_velocity(double rps, double ff_volts, int slot) {
  HardwareAccess::motor_set(RequireLink(link_, "Motor"), index_,
                            Mode::kVelocity, rps, ff_volts, slot);
}
void Motor::set_position(double rot, double ff_volts, int slot) {
  HardwareAccess::motor_set(RequireLink(link_, "Motor"), index_,
                            Mode::kPosition, rot, ff_volts, slot);
}
void Motor::set_motion_magic(double rot, double ff_volts, int slot) {
  HardwareAccess::motor_set(RequireLink(link_, "Motor"), index_,
                            Mode::kMotionMagic, rot, ff_volts, slot);
}
void Motor::set_torque_current(double amperes) {
  HardwareAccess::motor_set(RequireLink(link_, "Motor"), index_,
                            Mode::kTorqueCurrent, amperes, 0, 0);
}
void Motor::stop() {
  HardwareAccess::motor_set(RequireLink(link_, "Motor"), index_, Mode::kNeutral,
                            0, 0, 0);
}
std::uint64_t Motor::clamped_count() const {
  return HardwareAccess::motor_clamped_count(RequireLink(link_, "Motor"),
                                             index_);
}
std::uint8_t Motor::hardware_index() const {
  return HardwareAccess::motor_hardware_index(RequireLink(link_, "Motor"),
                                              index_);
}
void Motor::rezero(double position_rot) {
  HardwareAccess::motor_rezero(RequireLink(link_, "Motor"), index_,
                               position_rot);
}
bool Motor::zeroed() const {
  return HardwareAccess::motor_zeroed(RequireLink(link_, "Motor"), index_);
}
double Motor::position() const {
  return HardwareAccess::motor_sample(RequireLink(link_, "Motor"), index_)
      .position_rot;
}
double Motor::velocity() const {
  return HardwareAccess::motor_sample(RequireLink(link_, "Motor"), index_)
      .velocity_rps;
}
double Motor::voltage() const {
  return HardwareAccess::motor_sample(RequireLink(link_, "Motor"), index_)
      .voltage;
}
double Motor::current() const {
  return HardwareAccess::motor_sample(RequireLink(link_, "Motor"), index_)
      .stator_current_a;
}
double Motor::temperature_c() const {
  return HardwareAccess::motor_sample(RequireLink(link_, "Motor"), index_)
      .temperature_c;
}
double Motor::supply_current_a() const {
  return HardwareAccess::motor_sample(RequireLink(link_, "Motor"), index_)
      .supply_current_a;
}
bool Motor::connected() const {
  return HardwareAccess::motor_sample(RequireLink(link_, "Motor"), index_)
      .valid;
}
bool Motor::forward_limit() const {
  return HardwareAccess::motor_hard_limit(RequireLink(link_, "Motor"), index_,
                                          true);
}
bool Motor::reverse_limit() const {
  return HardwareAccess::motor_hard_limit(RequireLink(link_, "Motor"), index_,
                                          false);
}
std::optional<Time> Motor::position_time() const {
  return HardwareAccess::motor_time(RequireLink(link_, "Motor"), index_, false);
}
std::optional<Time> Motor::velocity_time() const {
  return HardwareAccess::motor_time(RequireLink(link_, "Motor"), index_, true);
}
std::optional<Duration> Motor::position_age() const {
  return HardwareAccess::motor_age(RequireLink(link_, "Motor"), index_, false);
}
std::optional<Duration> Motor::velocity_age() const {
  return HardwareAccess::motor_age(RequireLink(link_, "Motor"), index_, true);
}

CANcoder::CANcoder(Node& node, std::string_view name)
    : link_{LinkOf(node)},
      index_{HardwareAccess::add_device(*link_, Kind::kCANcoder, name)} {}
double CANcoder::position() const {
  return HardwareAccess::sensor_sample(RequireLink(link_, "CANcoder"), index_)
      .position_rot;
}
double CANcoder::velocity() const {
  return HardwareAccess::sensor_sample(RequireLink(link_, "CANcoder"), index_)
      .velocity_rps;
}
bool CANcoder::connected() const {
  return HardwareAccess::sensor_sample(RequireLink(link_, "CANcoder"), index_)
      .valid;
}
std::optional<Time> CANcoder::position_time() const {
  return HardwareAccess::sensor_time(RequireLink(link_, "CANcoder"), index_,
                                     false);
}
std::optional<Time> CANcoder::velocity_time() const {
  return HardwareAccess::sensor_time(RequireLink(link_, "CANcoder"), index_,
                                     true);
}
std::optional<Duration> CANcoder::position_age() const {
  return HardwareAccess::sensor_age(RequireLink(link_, "CANcoder"), index_,
                                    false);
}
std::optional<Duration> CANcoder::velocity_age() const {
  return HardwareAccess::sensor_age(RequireLink(link_, "CANcoder"), index_,
                                    true);
}

Gyro::Gyro(Node& node, std::string_view name)
    : link_{LinkOf(node)},
      index_{HardwareAccess::add_device(*link_, Kind::kGyro, name)} {}
void Gyro::rezero(double yaw_rot) {
  HardwareAccess::gyro_rezero(RequireLink(link_, "Gyro"), index_, yaw_rot);
}
bool Gyro::zeroed() const {
  return HardwareAccess::gyro_zeroed(RequireLink(link_, "Gyro"), index_);
}
double Gyro::yaw() const {
  return HardwareAccess::sensor_sample(RequireLink(link_, "Gyro"), index_)
      .position_rot;
}
double Gyro::yaw_rate() const {
  return HardwareAccess::sensor_sample(RequireLink(link_, "Gyro"), index_)
      .velocity_rps;
}
bool Gyro::connected() const {
  return HardwareAccess::sensor_sample(RequireLink(link_, "Gyro"), index_)
      .valid;
}
std::optional<Time> Gyro::yaw_time() const {
  return HardwareAccess::sensor_time(RequireLink(link_, "Gyro"), index_, false);
}
std::optional<Time> Gyro::yaw_rate_time() const {
  return HardwareAccess::sensor_time(RequireLink(link_, "Gyro"), index_, true);
}

DigitalInput::DigitalInput(Node& node, std::string_view name)
    : link_{LinkOf(node)},
      index_{HardwareAccess::add_device(*link_, Kind::kDigitalInput, name)} {}
bool DigitalInput::get() const {
  return HardwareAccess::digital_input_sample(
             RequireLink(link_, "DigitalInput"), index_)
      .value;
}
bool DigitalInput::connected() const {
  return HardwareAccess::digital_input_sample(
             RequireLink(link_, "DigitalInput"), index_)
      .valid;
}

DigitalOutput::DigitalOutput(Node& node, std::string_view name)
    : link_{LinkOf(node)},
      index_{HardwareAccess::add_device(*link_, Kind::kDigitalOutput, name)} {}
void DigitalOutput::set(bool value) {
  HardwareAccess::digital_output_set(RequireLink(link_, "DigitalOutput"),
                                     index_, value);
}

AnalogInput::AnalogInput(Node& node, std::string_view name)
    : link_{LinkOf(node)},
      index_{HardwareAccess::add_device(*link_, Kind::kAnalogInput, name)} {}
double AnalogInput::voltage() const {
  return HardwareAccess::analog_input_sample(RequireLink(link_, "AnalogInput"),
                                             index_)
      .voltage;
}
bool AnalogInput::connected() const {
  return HardwareAccess::analog_input_sample(RequireLink(link_, "AnalogInput"),
                                             index_)
      .valid;
}

Encoder::Encoder(Node& node, std::string_view name)
    : link_{LinkOf(node)},
      index_{HardwareAccess::add_device(*link_, Kind::kEncoder, name)} {}
double Encoder::position() const {
  return HardwareAccess::encoder_sample(RequireLink(link_, "Encoder"), index_)
      .position_rot;
}
double Encoder::velocity() const {
  return HardwareAccess::encoder_sample(RequireLink(link_, "Encoder"), index_)
      .velocity_rps;
}
bool Encoder::connected() const {
  return HardwareAccess::encoder_sample(RequireLink(link_, "Encoder"), index_)
      .valid;
}

PwmOutput::PwmOutput(Node& node, std::string_view name)
    : link_{LinkOf(node)},
      index_{HardwareAccess::add_device(*link_, Kind::kPwm, name)} {}
void PwmOutput::set(double output) {
  HardwareAccess::pwm_set(RequireLink(link_, "PwmOutput"), index_, output);
}
void PwmOutput::stop() {
  HardwareAccess::pwm_stop(RequireLink(link_, "PwmOutput"), index_);
}

// --- Driver Station ----------------------------------------------------------

namespace detail {

// One node's subscription to the forwarded Driver Station packet, shared by
// its DriverStation handles. The bridge owns the topic's layout
// (hardware::Packet, aligned as the bridge registers it), so it is
// subscribed raw rather than as a talos message.
class DriverStationLink {
 public:
  explicit DriverStationLink(Node& node) : node_{node} {
    subscription_ = NodeAccess::add_internal_driver_station_subscription(
        node, hardware::kDriverStationTopic, sizeof(Packet), alignof(Packet),
        schema::Of<Packet>(), [this](const void* data) { OnPacket(data); });
  }

  // A listener runs as each packet arrives, so the packet's publications
  // wake the node; without one the link is drained before the node's timers.
  void Listen(std::function<void()> on_packet) {
    NodeAccess::require_immediate_delivery(node_, subscription_);
    listeners_.push_back(std::move(on_packet));
  }

  bool Connected() const {
    return have_ && (data_.fms.flags & driver_station::kDsAttached) != 0 &&
           node_.now() - published_ < NodeAccess::freshness_timeout(node_);
  }
  const driver_station::DriverStationData& data() const { return data_; }

  DsMode Mode() const {
    return Connected() ? DsModeOf(data_.fms.flags) : DsMode::kNone;
  }
  // The sampler's mode generation while there is a mode, else 0.
  std::uint64_t Generation() const {
    return Mode() != DsMode::kNone ? data_.mode_since_us : 0;
  }
  std::uint64_t Recoveries() const { return recoveries_; }

 private:
  // Sample and mode times run on the sampling gateway's boot clock, so they
  // order packets only within one boot. A boot never seen is a reboot (its
  // clock restarted low): take it and start the ordering over. A boot left
  // recently is a packet delayed across that reboot: never go back to it.
  void OnPacket(const void* data) {
    Packet packet{};
    std::memcpy(&packet, data, sizeof packet);
    driver_station::DriverStationData next{};
    if (!driver_station::Decode(packet.bytes(), next)) return;
    if (boots_.retired(next.boot_id)) return;
    if (!boots_.current(next.boot_id)) {
      boots_.Enter(next.boot_id);
    } else if ((data_.sample_time_us > 0 &&
                next.sample_time_us <= data_.sample_time_us) ||
               (data_.mode_since_us > 0 &&
                next.mode_since_us < data_.mode_since_us)) {
      return;
    }
    const Time published =
        node_.now() - NodeAccess::subscription_age(node_, subscription_);
    // The link was stale in between, whether or not anyone sampled it then.
    if (have_ &&
        published - published_ >= NodeAccess::freshness_timeout(node_)) {
      ++recoveries_;
    }
    data_ = next;
    have_ = true;
    published_ = published;
    for (const auto& listener : listeners_) listener();
  }

  Node& node_;
  driver_station::DriverStationData data_{};
  bool have_ = false;
  hardware::BootHistory boots_;
  Time published_{};
  std::uint64_t recoveries_ = 0;
  std::size_t subscription_ = 0;
  std::vector<std::function<void()>> listeners_;
};

DsMode DsModeOf(std::uint32_t flags) {
  const std::uint32_t key = driver_station::ModeKey(flags);
  if ((key & driver_station::kDsAttached) == 0 ||
      (key & driver_station::kEnabled) == 0 ||
      (key & driver_station::kEStop) != 0) {
    return DsMode::kNone;
  }
  if ((key & driver_station::kAutonomous) != 0) return DsMode::kAutonomous;
  if ((key & driver_station::kTeleop) != 0) return DsMode::kTeleop;
  if ((key & driver_station::kTest) != 0) return DsMode::kTest;
  return DsMode::kNone;
}

namespace {

DriverStationLink& RequireDriverStationLink(DriverStationLink* link) {
  if (link == nullptr) {
    throw std::logic_error("DriverStation handle is not bound to a Node");
  }
  return *link;
}

}  // namespace

DriverStation DriverStationAccess::internal(Node& node) {
  return DriverStation{&node.extension<DriverStationLink>()};
}
DsMode DriverStationAccess::mode(const DriverStation& ds) {
  return RequireDriverStationLink(ds.link_).Mode();
}
std::uint64_t DriverStationAccess::generation(const DriverStation& ds) {
  return RequireDriverStationLink(ds.link_).Generation();
}
std::uint64_t DriverStationAccess::recoveries(const DriverStation& ds) {
  return RequireDriverStationLink(ds.link_).Recoveries();
}

}  // namespace detail

double Joystick::axis(int index) const {
  if (index < 0 ||
      index >= std::min<int>(data_.axis_count, driver_station::kMaxAxes)) {
    return 0.0;
  }
  return data_.axes[static_cast<std::size_t>(index)];
}

bool Joystick::button(int number) const {
  if (number < 1 || number > std::min<int>(data_.button_count, 32)) {
    return false;
  }
  return ((data_.buttons >> (number - 1)) & 1u) != 0;
}

int Joystick::pov(int index) const {
  if (index < 0 ||
      index >= std::min<int>(data_.pov_count, driver_station::kMaxPovs)) {
    return -1;
  }
  return data_.povs[static_cast<std::size_t>(index)];
}

std::string_view Joystick::name() const {
  const char* end =
      std::find(std::begin(data_.name), std::end(data_.name), '\0');
  return {data_.name, static_cast<std::size_t>(end - data_.name)};
}

namespace {

detail::DriverStationLink* AttachPublicDriverStation(Node& node) {
  // Mark (and check REQ-SAFE-14 if already started) BEFORE attaching the
  // extension so a refused late construction leaves no DriverStationLink on
  // the node.
  detail::NodeAccess::mark_driver_station_reader(node);
  return &node.extension<detail::DriverStationLink>();
}

detail::DriverStationLink& RequirePublicDriverStationLink(
    detail::DriverStationLink* link) {
  if (link == nullptr) {
    throw std::logic_error("DriverStation handle is not bound to a Node");
  }
  return *link;
}

}  // namespace

DriverStation::DriverStation(Node& node)
    : link_{AttachPublicDriverStation(node)} {}

DriverStation::DriverStation(Node& node, std::function<void()> on_packet)
    : DriverStation{node} {
  if (on_packet) link_->Listen(std::move(on_packet));
}

bool DriverStation::connected() const {
  return RequirePublicDriverStationLink(link_).Connected();
}
bool DriverStation::flag(driver_station::DsFlags flag) const {
  return connected() &&
         (RequirePublicDriverStationLink(link_).data().fms.flags & flag) != 0;
}
bool DriverStation::enabled() const { return flag(driver_station::kEnabled); }
bool DriverStation::autonomous() const {
  return flag(driver_station::kAutonomous);
}
bool DriverStation::teleop() const { return flag(driver_station::kTeleop); }
bool DriverStation::test() const { return flag(driver_station::kTest); }
bool DriverStation::estop() const { return flag(driver_station::kEStop); }
bool DriverStation::fms_attached() const {
  return flag(driver_station::kFmsAttached);
}

driver_station::Alliance DriverStation::alliance() const {
  return RequirePublicDriverStationLink(link_).data().fms.alliance;
}
int DriverStation::station() const {
  return RequirePublicDriverStationLink(link_).data().fms.station;
}
double DriverStation::match_time() const {
  return RequirePublicDriverStationLink(link_).data().fms.match_time_s;
}

Joystick DriverStation::joystick(std::size_t port) const {
  if (port >= driver_station::kMaxJoysticks) {
    throw std::out_of_range("no Driver Station joystick port " +
                            std::to_string(port) + " (ports are 0-" +
                            std::to_string(driver_station::kMaxJoysticks - 1) +
                            ")");
  }
  if (!connected()) return Joystick{};
  return Joystick{RequirePublicDriverStationLink(link_).data().joysticks[port]};
}

const driver_station::DriverStationData& DriverStation::data() const {
  return RequirePublicDriverStationLink(link_).data();
}

}  // namespace talos
