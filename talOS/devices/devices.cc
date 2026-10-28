#include "talOS/devices/devices.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "talOS/configuration/config_parser.h"
#include "talOS/hardware/gate.h"
#include "talOS/hardware/packet.h"

namespace talos {
namespace {

using hardware::Packet;
using detail::HardwareAccess;
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

template <class Sample, std::size_t N>
Sample FindSample(const std::array<Sample, N>& samples, std::size_t count,
                  std::uint16_t id) {
  for (std::size_t i = 0; i < std::min(count, N); ++i) {
    if (samples[i].id == id) return samples[i];
  }
  return Sample{};
}

}  // namespace

std::uint16_t HardwareAccess::device_id(const config::RobotConfig& config,
                                        std::string_view subsystem,
                                        Kind kind, std::string_view name) {
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
    bool set = false;  // commanded since the last flush
    hardware::MotorRequest motor{};
    bool digital = false;
    double pwm = 0;
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
  Time state_received{};

  explicit Impl(Node& n) : node{n} {}

  bool Enabled() const {
    return have_state && hardware::GatewayReady(state) &&
           node.now() - state_received < timeout;
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
  const Device& SensorAt(std::size_t index) const {
    if (index < devices.size() && (devices[index].kind == Kind::kGyro ||
                                   devices[index].kind == Kind::kCANcoder))
      return devices[index];
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
    state = next;
    have_state = true;
    state_received = node.now();
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
              device.id, device.digital};
        } else if (device.kind == Kind::kPwm &&
                   command.pwm_output_count < hardware::kMaxPwmOutputs) {
          command.pwm_outputs[command.pwm_output_count++] = {device.id,
                                                             device.pwm};
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
  impl.config = config::ParseRobotConfig(node.config_path());
  // The bridge treats the config as commissioned under --sim, and the
  // commissioned bit is part of the config id both ends compare.
  if (node.simulation()) impl.config.hardware.commissioned = true;
  impl.config_id = hardware::ConfigurationId(impl.config.hardware);
  impl.timeout =
      std::chrono::microseconds{impl.config.hardware.command_timeout_us};
  detail::NodeAccess::add_subscription(
      node, hardware::kStateTopic, sizeof(Packet), alignof(Packet),
      /*optional=*/false, [&impl](const void* data) { impl.OnState(data); });
}

HardwareLink::~HardwareLink() = default;

bool HardwareLink::enabled() const { return impl_->Enabled(); }

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
        alignof(Packet));
    impl.node.after_update([&impl] { impl.Flush(); });
    impl.has_outputs = true;
  }
  if (kind == Kind::kMotor) {
    impl.motor_ids.push_back(id);
    impl.motor_devices.push_back(impl.devices.size());
  }
  impl.devices.push_back({.kind = kind, .id = id});
  return impl.devices.size() - 1;
}

void HardwareAccess::motor_set(HardwareLink& link, std::size_t index,
                               hardware::Mode mode, double demand,
                               double ff_volts, int slot) {
  HardwareLink::Impl::Device& device = link.impl_->At(index, Kind::kMotor);
  if (slot < 0 || slot > 2) {
    throw std::invalid_argument(std::string{link.impl_->node.name()} +
                                ": motor slot " + std::to_string(slot) +
                                " is not 0, 1 or 2");
  }
  // A non-finite demand is no demand: neutral, rather than handing the
  // gateway a NaN.
  if (!std::isfinite(demand) || !std::isfinite(ff_volts)) {
    mode = hardware::Mode::kNeutral;
    demand = 0;
    ff_volts = 0;
    slot = 0;
  }
  device.motor = {.id = device.id,
                  .mode = mode,
                  .slot = static_cast<std::uint8_t>(slot),
                  .demand = demand,
                  .feedforward_v = ff_volts};
  device.set = true;
}

hardware::MotorSample HardwareAccess::motor_sample(const HardwareLink& link,
                                                   std::size_t index) {
  const auto& impl = *link.impl_;
  const auto& device = impl.At(index, Kind::kMotor);
  return FindSample(impl.state.motors, impl.state.motor_count, device.id);
}

hardware::SensorSample HardwareAccess::sensor_sample(const HardwareLink& link,
                                                     std::size_t index) {
  const auto& impl = *link.impl_;
  const auto& device = impl.SensorAt(index);
  return FindSample(impl.state.sensors, impl.state.sensor_count, device.id);
}

hardware::DigitalInputSample HardwareAccess::digital_input_sample(
    const HardwareLink& link, std::size_t index) {
  const auto& impl = *link.impl_;
  const auto& device = impl.At(index, Kind::kDigitalInput);
  return FindSample(impl.state.digital_inputs, impl.state.digital_input_count,
                    device.id);
}

hardware::AnalogInputSample HardwareAccess::analog_input_sample(
    const HardwareLink& link, std::size_t index) {
  const auto& impl = *link.impl_;
  const auto& device = impl.At(index, Kind::kAnalogInput);
  return FindSample(impl.state.analog_inputs, impl.state.analog_input_count,
                    device.id);
}

hardware::EncoderSample HardwareAccess::encoder_sample(const HardwareLink& link,
                                                       std::size_t index) {
  const auto& impl = *link.impl_;
  const auto& device = impl.At(index, Kind::kEncoder);
  return FindSample(impl.state.encoders, impl.state.encoder_count, device.id);
}

void HardwareAccess::digital_output_set(HardwareLink& link, std::size_t index,
                                        bool value) {
  auto& device = link.impl_->At(index, Kind::kDigitalOutput);
  device.digital = value;
  device.set = true;
}

void HardwareAccess::pwm_set(HardwareLink& link, std::size_t index,
                             double output) {
  auto& device = link.impl_->At(index, Kind::kPwm);
  device.pwm = std::isfinite(output) ? output : 0.0;
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
bool Motor::connected() const {
  return HardwareAccess::motor_sample(*link_, index_).valid;
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

Gyro::Gyro(Node& node, std::string_view name)
    : link_{LinkOf(node)},
      index_{HardwareAccess::add_device(*link_, Kind::kGyro, name)} {}
double Gyro::yaw() const {
  return HardwareAccess::sensor_sample(*link_, index_).position_rot;
}
double Gyro::yaw_rate() const {
  return HardwareAccess::sensor_sample(*link_, index_).velocity_rps;
}
bool Gyro::connected() const {
  return HardwareAccess::sensor_sample(*link_, index_).valid;
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

}  // namespace talos
