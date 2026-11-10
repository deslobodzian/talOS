#include "talOS/devices/devices.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <functional>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "bridge/bridge_status_generated.h"
#include "bridge/topics.h"
#include "talOS/bridge/boot_history.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/hardware/gate.h"
#include "talOS/hardware/packet.h"

namespace talos {
namespace {

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
    bool set = false;  // commanded in the current update()
    hardware::MotorRequest motor{};
    const hardware::MotorConfig* motor_config = nullptr;
    std::uint64_t clamped_count = 0;
    double position_offset = 0;  // Motor/Gyro software zero
    bool zeroed = false;         // position_offset was set by rezero
    // A zero was dropped (ForgetZeros) and no rezero has replaced it: the
    // offset is kept for reads, but position goals go out neutral.
    bool zero_lost = false;
    bool digital = false;
    double pwm = 0;
    // Where this device's sample was in the state arrays last time it was
    // read; a hint only, checked against the id on every read.
    mutable std::size_t sample_index = 0;
  };

  Node& node;
  config::RobotConfig config;
  std::uint64_t config_id = 0;
  Duration timeout{};

  std::vector<Device> devices;
  std::vector<std::uint16_t> motor_ids;    // flush order
  std::vector<std::size_t> motor_devices;  // devices[] index, same order
  bool has_outputs = false;
  std::size_t request = 0;  // publisher index on node

  hardware::State state{};
  bool have_state = false;
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
    const std::int64_t us = static_cast<std::int64_t>(state.sample_time_us) +
                            clock.clock_offset_us();
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
  Device& Output(std::size_t index, Kind kind, std::string_view setter) {
    detail::NodeAccess::require_loop_thread(node, setter);
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
  // and newer than the last sample from the same boot.
  void OnState(const void* data) {
    Packet packet{};
    std::memcpy(&packet, data, sizeof packet);
    hardware::State next{};
    if (!hardware::Decode(packet.bytes(), next)) return;
    if (!hardware::AcceptState(have_state, state, config_id, next)) return;
    if (have_state) ForgetZeros(next);
    state = next;
    have_state = true;
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
  // A dropped zero keeps its offset, so dropping does not move position()
  // or yaw(); it reads zeroed() false and holds the motor's position goals
  // neutral until the node rezeroes.
  void ForgetZeros(const hardware::State& next) {
    const bool reboot = next.boot_id != state.boot_id;
    if (!reboot &&
        next.counters.device_reset_count == state.counters.device_reset_count) {
      return;
    }
    const bool consecutive = !reboot && next.sequence == state.sequence + 1;
    for (Device& device : devices) {
      if (!device.zeroed) continue;
      if (consecutive && Valid(next, device)) continue;
      device.zeroed = false;
      device.zero_lost = true;
    }
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

  void Flush() {
    if (have_state) {
      hardware::Command command = hardware::NeutralCommand(state, motor_ids);
      const bool drive = Enabled();
      for (std::size_t i = 0; i < command.count; ++i) {
        const Device& motor = devices[motor_devices[i]];
        if (drive && motor.set) command.motors[i] = motor.motor;
      }
      for (const Device& device : devices) {
        if (!device.set) continue;
        if (device.kind == Kind::kDigitalOutput &&
            command.digital_output_count < hardware::kMaxDigitalOutputs) {
          command.digital_outputs[command.digital_output_count++] = {
              device.id, drive && device.digital};
        } else if (device.kind == Kind::kPwm &&
                   command.pwm_output_count < hardware::kMaxPwmOutputs) {
          command.pwm_outputs[command.pwm_output_count++] = {
              device.id, drive ? device.pwm : 0.0};
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
    for (Device& device : devices) device.set = false;
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

std::size_t HardwareAccess::add_device(HardwareLink& link, Kind kind,
                                       std::string_view name) {
  HardwareLink::Impl& impl = *link.impl_;
  const std::uint16_t id = device_id(impl.config, impl.node.name(), kind, name);
  for (std::size_t i = 0; i < impl.devices.size(); ++i) {
    if (impl.devices[i].id == id && impl.devices[i].kind == kind) return i;
  }
  if (kind == Kind::kMotor && impl.motor_ids.size() >= hardware::kMaxMotors) {
    throw std::invalid_argument(std::string{impl.node.name()} + ": more than " +
                                std::to_string(hardware::kMaxMotors) +
                                " motors");
  }
  // Input-only nodes publish no request: the bridge reads one only from
  // subsystems that own actuators, and an unread topic is a graph error.
  if (IsOutput(kind) && !impl.has_outputs) {
    impl.request = detail::NodeAccess::add_publisher(
        impl.node, hardware::RequestTopic(impl.node.name()), sizeof(Packet),
        alignof(Packet), schema::Of<Packet>());
    impl.node.after_update([&impl] { impl.Flush(); });
    impl.has_outputs = true;
  }
  if (kind == Kind::kMotor) {
    impl.motor_ids.push_back(id);
    impl.motor_devices.push_back(impl.devices.size());
  }
  impl.devices.push_back({.kind = kind, .id = id, .name = std::string{name}});
  if (kind == Kind::kMotor) {
    for (const auto& motor : impl.config.hardware.motors) {
      if (motor.id == id) {
        impl.devices.back().motor_config = &motor;
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
  if (slot < 0 || slot > 2) {
    throw std::invalid_argument(std::string{link.impl_->node.name()} +
                                ": motor slot " + std::to_string(slot) +
                                " is not 0, 1 or 2");
  }
  const auto& config = *device.motor_config;
  if (mode == hardware::Mode::kPosition ||
      mode == hardware::Mode::kMotionMagic) {
    // A position in a frame that may have restarted (a dropped zero), or
    // that nobody established ([motors.*] requires_zero): neutral until a
    // rezero holds. Other modes need no frame.
    if (!device.zeroed && (device.zero_lost || config.requires_zero.value)) {
      device.motor = {.id = device.id};
      device.set = true;
      return;
    }
    demand += device.position_offset;
  }
  const double requested_demand = demand;
  const double requested_ff = ff_volts;
  // A non-finite demand is no demand: neutral, rather than handing the
  // gateway a NaN.
  if (!std::isfinite(demand) || !std::isfinite(ff_volts)) {
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
  device.set = true;
}

std::uint64_t HardwareAccess::motor_clamped_count(const HardwareLink& link,
                                                  std::size_t index) {
  return link.impl_->At(index, Kind::kMotor).clamped_count;
}

void HardwareAccess::motor_rezero(HardwareLink& link, std::size_t index,
                                  double position_rot) {
  detail::NodeAccess::require_loop_thread(link.impl_->node, "Motor::rezero");
  auto& device = link.impl_->At(index, Kind::kMotor);
  const auto& impl = *link.impl_;
  const auto sample = FindSample(impl.state.motors, impl.state.motor_count,
                                 device.id, device.sample_index);
  const double offset = sample.position_rot - position_rot;
  if (impl.Fresh() && sample.valid && std::isfinite(sample.position_rot) &&
      std::isfinite(position_rot) && std::isfinite(offset)) {
    device.position_offset = offset;
    device.zeroed = true;
    device.zero_lost = false;
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
    device.zero_lost = false;
  }
}

bool HardwareAccess::gyro_zeroed(const HardwareLink& link, std::size_t index) {
  return link.impl_->At(index, Kind::kGyro).zeroed;
}

hardware::MotorSample HardwareAccess::motor_sample(const HardwareLink& link,
                                                   std::size_t index) {
  const auto& impl = *link.impl_;
  const auto& device = impl.At(index, Kind::kMotor);
  auto sample = FindSample(impl.state.motors, impl.state.motor_count, device.id,
                           device.sample_index);
  if (!impl.Fresh()) sample.valid = false;
  sample.position_rot -= device.position_offset;
  return sample;
}

hardware::SensorSample HardwareAccess::sensor_sample(const HardwareLink& link,
                                                     std::size_t index) {
  const auto& impl = *link.impl_;
  const auto& device = impl.SensorAt(index);
  auto sample = FindSample(impl.state.sensors, impl.state.sensor_count,
                           device.id, device.sample_index);
  if (!impl.Fresh()) sample.valid = false;
  sample.position_rot -= device.position_offset;
  return sample;
}

namespace {
template <class Sample>
std::optional<Time> AcquisitionTime(const HardwareLink& link,
                                    const Sample& sample, bool velocity) {
  const auto age = velocity ? sample.velocity_age_us : sample.position_age_us;
  if (!sample.valid || age == UINT32_MAX) return std::nullopt;
  const auto time = link.sample_time();
  if (!time) return std::nullopt;
  return *time - std::chrono::microseconds{age};
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
  if (!impl.Fresh()) sample.valid = false;
  return sample;
}

void HardwareAccess::digital_output_set(HardwareLink& link, std::size_t index,
                                        bool value) {
  auto& device =
      link.impl_->Output(index, Kind::kDigitalOutput, "DigitalOutput::set");
  device.digital = value;
  device.set = true;
}

void HardwareAccess::pwm_set(HardwareLink& link, std::size_t index,
                             double output) {
  auto& device = link.impl_->Output(index, Kind::kPwm, "PwmOutput::set");
  device.pwm = std::isfinite(output) ? std::clamp(output, -1.0, 1.0) : 0.0;
  device.set = true;
}

// --- handles -----------------------------------------------------------------

namespace {

HardwareLink* LinkOf(Node& node) { return &node.extension<HardwareLink>(); }

}  // namespace

using hardware::Mode;

Motor::Motor(Node& node, std::string_view name)
    : link_{LinkOf(node)},
      index_{HardwareAccess::add_device(*link_, Kind::kMotor, name)} {}
void Motor::set_duty_cycle(double fraction) {
  HardwareAccess::motor_set(*link_, index_, Mode::kDutyCycle, fraction, 0, 0);
}
void Motor::set_voltage(double volts) {
  HardwareAccess::motor_set(*link_, index_, Mode::kVoltage, volts, 0, 0);
}
void Motor::set_velocity(double rps, double ff_volts, int slot) {
  HardwareAccess::motor_set(*link_, index_, Mode::kVelocity, rps, ff_volts,
                            slot);
}
void Motor::set_position(double rot, double ff_volts, int slot) {
  HardwareAccess::motor_set(*link_, index_, Mode::kPosition, rot, ff_volts,
                            slot);
}
void Motor::set_motion_magic(double rot, double ff_volts, int slot) {
  HardwareAccess::motor_set(*link_, index_, Mode::kMotionMagic, rot, ff_volts,
                            slot);
}
void Motor::stop() {
  HardwareAccess::motor_set(*link_, index_, Mode::kNeutral, 0, 0, 0);
}
std::uint64_t Motor::clamped_count() const {
  return HardwareAccess::motor_clamped_count(*link_, index_);
}
void Motor::rezero(double position_rot) {
  HardwareAccess::motor_rezero(*link_, index_, position_rot);
}
bool Motor::zeroed() const {
  return HardwareAccess::motor_zeroed(*link_, index_);
}
double Motor::position() const {
  return HardwareAccess::motor_sample(*link_, index_).position_rot;
}
double Motor::velocity() const {
  return HardwareAccess::motor_sample(*link_, index_).velocity_rps;
}
double Motor::voltage() const {
  return HardwareAccess::motor_sample(*link_, index_).voltage;
}
double Motor::current() const {
  return HardwareAccess::motor_sample(*link_, index_).stator_current_a;
}
double Motor::temperature_c() const {
  return HardwareAccess::motor_sample(*link_, index_).temperature_c;
}
double Motor::supply_current_a() const {
  return HardwareAccess::motor_sample(*link_, index_).supply_current_a;
}
bool Motor::connected() const {
  return HardwareAccess::motor_sample(*link_, index_).valid;
}
std::optional<Time> Motor::position_time() const {
  return HardwareAccess::motor_time(*link_, index_, false);
}
std::optional<Time> Motor::velocity_time() const {
  return HardwareAccess::motor_time(*link_, index_, true);
}

CANcoder::CANcoder(Node& node, std::string_view name)
    : link_{LinkOf(node)},
      index_{HardwareAccess::add_device(*link_, Kind::kCANcoder, name)} {}
double CANcoder::position() const {
  return HardwareAccess::sensor_sample(*link_, index_).position_rot;
}
double CANcoder::velocity() const {
  return HardwareAccess::sensor_sample(*link_, index_).velocity_rps;
}
bool CANcoder::connected() const {
  return HardwareAccess::sensor_sample(*link_, index_).valid;
}
std::optional<Time> CANcoder::position_time() const {
  return HardwareAccess::sensor_time(*link_, index_, false);
}
std::optional<Time> CANcoder::velocity_time() const {
  return HardwareAccess::sensor_time(*link_, index_, true);
}

Gyro::Gyro(Node& node, std::string_view name)
    : link_{LinkOf(node)},
      index_{HardwareAccess::add_device(*link_, Kind::kGyro, name)} {}
void Gyro::rezero(double yaw_rot) {
  HardwareAccess::gyro_rezero(*link_, index_, yaw_rot);
}
bool Gyro::zeroed() const {
  return HardwareAccess::gyro_zeroed(*link_, index_);
}
double Gyro::yaw() const {
  return HardwareAccess::sensor_sample(*link_, index_).position_rot;
}
double Gyro::yaw_rate() const {
  return HardwareAccess::sensor_sample(*link_, index_).velocity_rps;
}
bool Gyro::connected() const {
  return HardwareAccess::sensor_sample(*link_, index_).valid;
}
std::optional<Time> Gyro::yaw_time() const {
  return HardwareAccess::sensor_time(*link_, index_, false);
}
std::optional<Time> Gyro::yaw_rate_time() const {
  return HardwareAccess::sensor_time(*link_, index_, true);
}

DigitalInput::DigitalInput(Node& node, std::string_view name)
    : link_{LinkOf(node)},
      index_{HardwareAccess::add_device(*link_, Kind::kDigitalInput, name)} {}
bool DigitalInput::get() const {
  return HardwareAccess::digital_input_sample(*link_, index_).value;
}
bool DigitalInput::connected() const {
  return HardwareAccess::digital_input_sample(*link_, index_).valid;
}

DigitalOutput::DigitalOutput(Node& node, std::string_view name)
    : link_{LinkOf(node)},
      index_{HardwareAccess::add_device(*link_, Kind::kDigitalOutput, name)} {}
void DigitalOutput::set(bool value) {
  HardwareAccess::digital_output_set(*link_, index_, value);
}

AnalogInput::AnalogInput(Node& node, std::string_view name)
    : link_{LinkOf(node)},
      index_{HardwareAccess::add_device(*link_, Kind::kAnalogInput, name)} {}
double AnalogInput::voltage() const {
  return HardwareAccess::analog_input_sample(*link_, index_).voltage;
}
bool AnalogInput::connected() const {
  return HardwareAccess::analog_input_sample(*link_, index_).valid;
}

Encoder::Encoder(Node& node, std::string_view name)
    : link_{LinkOf(node)},
      index_{HardwareAccess::add_device(*link_, Kind::kEncoder, name)} {}
double Encoder::position() const {
  return HardwareAccess::encoder_sample(*link_, index_).position_rot;
}
double Encoder::velocity() const {
  return HardwareAccess::encoder_sample(*link_, index_).velocity_rps;
}
bool Encoder::connected() const {
  return HardwareAccess::encoder_sample(*link_, index_).valid;
}

PwmOutput::PwmOutput(Node& node, std::string_view name)
    : link_{LinkOf(node)},
      index_{HardwareAccess::add_device(*link_, Kind::kPwm, name)} {}
void PwmOutput::set(double output) {
  HardwareAccess::pwm_set(*link_, index_, output);
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
    subscription_ = NodeAccess::add_subscription(
        node, hardware::kDriverStationTopic, sizeof(Packet), alignof(Packet),
        schema::Of<Packet>(), /*optional=*/false,
        [this](const void* data) { OnPacket(data); });
  }

  // A listener runs as each packet arrives, so its node drains every tick.
  void Listen(std::function<void()> on_packet) {
    NodeAccess::require_immediate_delivery(node_);
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
    } else if (next.sample_time_us > 0 && data_.sample_time_us > 0 &&
               (next.sample_time_us < data_.sample_time_us ||
                (next.mode_since_us != 0 &&
                 next.mode_since_us < data_.mode_since_us))) {
      return;
    }
    data_ = next;
    have_ = true;
    published_ =
        node_.now() - NodeAccess::subscription_age(node_, subscription_);
    for (const auto& listener : listeners_) listener();
  }

  Node& node_;
  driver_station::DriverStationData data_{};
  bool have_ = false;
  hardware::BootHistory boots_;
  Time published_{};
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

DsMode DriverStationAccess::mode(const DriverStation& ds) {
  return ds.link_->Mode();
}
std::uint64_t DriverStationAccess::generation(const DriverStation& ds) {
  return ds.link_->Generation();
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

DriverStation::DriverStation(Node& node)
    : link_{&node.extension<detail::DriverStationLink>()} {}

DriverStation::DriverStation(Node& node, std::function<void()> on_packet)
    : DriverStation{node} {
  if (on_packet) link_->Listen(std::move(on_packet));
}

bool DriverStation::connected() const { return link_->Connected(); }
bool DriverStation::flag(driver_station::DsFlags flag) const {
  return connected() && (link_->data().fms.flags & flag) != 0;
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
  return link_->data().fms.alliance;
}
int DriverStation::station() const { return link_->data().fms.station; }
double DriverStation::match_time() const {
  return link_->data().fms.match_time_s;
}

Joystick DriverStation::joystick(std::size_t port) const {
  if (port >= driver_station::kMaxJoysticks) {
    throw std::out_of_range("no Driver Station joystick port " +
                            std::to_string(port) + " (ports are 0-" +
                            std::to_string(driver_station::kMaxJoysticks - 1) +
                            ")");
  }
  if (!connected()) return Joystick{};
  return Joystick{link_->data().joysticks[port]};
}

const driver_station::DriverStationData& DriverStation::data() const {
  return link_->data();
}

}  // namespace talos
