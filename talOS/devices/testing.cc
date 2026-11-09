#include "talOS/devices/testing.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <map>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "bridge/bridge_status_generated.h"
#include "bridge/topics.h"
#include "talOS/bridge/clock_offset.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/devices/access.h"
#include "talOS/hardware/packet.h"

namespace talos::testing {
namespace {

using detail::SimAccess;
using hardware::Packet;
using Kind = detail::DeviceKind;

struct Axis {
  double position = 0;
  double velocity = 0;
  std::uint32_t position_age_us = 0;
  std::uint32_t velocity_age_us = 0;
};

// Neutral decay, the same first-order rotor model and constants as the
// Phoenix desktop sim (PhoenixBackend kSimNeutral*): brake shorts the windings
// and stops the rotor quickly, coast rolls on; friction snaps a nearly-stopped
// unpowered rotor to standstill.
constexpr double kNeutralBrakeTauS = 0.05;
constexpr double kNeutralCoastTauS = 1.0;
constexpr double kNeutralStandstillRps = 0.01;

// CANcoders and gyros share the sensors table, and so its ids.
Kind TableOf(Kind kind) { return kind == Kind::kGyro ? Kind::kCANcoder : kind; }

template <class Map>
auto ValueOr(const Map& map, std::uint16_t id,
             typename Map::mapped_type fallback) {
  const auto it = map.find(id);
  return it != map.end() ? it->second : fallback;
}

}  // namespace

struct SimHardware::Impl {
  Sim& sim;
  config::RobotConfig config;
  std::uint64_t config_id = 0;
  Duration period{};
  bool enabled = false;
  bool ideal = false;

  std::map<std::uint16_t, Axis> motors, sensors, encoders;
  std::map<std::uint16_t, bool> digital_inputs;
  std::map<std::uint16_t, double> analog_inputs;

  std::map<std::uint16_t, hardware::MotorRequest> motor_requests;
  std::map<std::uint16_t, hardware::MotorRequest> applied_motors;
  std::map<std::uint16_t, bool> digital_outputs;
  std::map<std::uint16_t, double> pwm_outputs;
  std::map<std::uint16_t, bool> applied_digital_outputs;
  std::map<std::uint16_t, double> applied_pwm_outputs;
  struct RequestTracker {
    std::string topic;
    std::uint64_t cursor = 0;
    std::optional<Time> last_seen;
  };
  std::map<std::string, RequestTracker> requests;
  std::set<std::pair<Kind, std::uint16_t>> disconnected;
  std::optional<driver_station::DriverStationData> ds_packet;
  driver_station::ModeTracker ds_mode;
  Duration gateway_offset{};
  Duration latency{};
  hardware::ClockOffsetEstimator clock;
  std::uint64_t states_sent = 0;
  std::uint64_t ds_packets_sent = 0;

  explicit Impl(Sim& s) : sim{s} {}

  // The simulated gateway never reboots: one boot id, carried by its State
  // and its Driver Station packets alike.
  static constexpr std::uint64_t kBootId = 1;

  // The gateway clock, in its microseconds, `before` ago.
  std::uint64_t GatewayUs(Duration before = Duration{}) const {
    return static_cast<std::uint64_t>(
        (sim.now().nanos() + (gateway_offset - before).count()) / 1000);
  }
  std::uint16_t Id(std::string_view node, Kind kind,
                   std::string_view device) const {
    return detail::HardwareAccess::device_id(config, node, kind, device);
  }
  bool Connected(Kind kind, std::uint16_t id) const {
    return !disconnected.contains({TableOf(kind), id});
  }

  void Neutralize(const hardware::Devices& devices) {
    for (const auto id : devices.motors) {
      applied_motors[id] = hardware::MotorRequest{.id = id};
    }
    for (const auto id : devices.digital_outputs) {
      applied_digital_outputs[id] = false;
    }
    for (const auto id : devices.pwm_outputs) {
      applied_pwm_outputs[id] = 0.0;
    }
  }

  // Every request each subsystem flushed since the last look, in order.
  void Collect() {
    for (const auto& [subsystem, devices] : config.subsystems) {
      auto& tracker = requests.at(subsystem);
      const std::uint64_t count =
          SimAccess::message_count(sim, tracker.topic, sizeof(Packet));
      for (; tracker.cursor < count; ++tracker.cursor) {
        Packet packet{};
        SimAccess::read_raw(sim, tracker.topic, sizeof packet, tracker.cursor,
                            &packet);
        hardware::Command command{};
        if (!hardware::Decode(packet.bytes(), command)) continue;
        tracker.last_seen = SimAccess::publish_time(
            sim, tracker.topic, sizeof packet, tracker.cursor);
        // Like the bridge, each request replaces this node's entire output.
        // Raw inspection still retains the last explicitly requested values.
        Neutralize(devices);
        for (std::size_t i = 0; i < command.count; ++i) {
          motor_requests[command.motors[i].id] = command.motors[i];
          applied_motors[command.motors[i].id] = command.motors[i];
        }
        for (std::size_t i = 0; i < command.digital_output_count; ++i) {
          digital_outputs[command.digital_outputs[i].id] =
              command.digital_outputs[i].value;
          applied_digital_outputs[command.digital_outputs[i].id] =
              command.digital_outputs[i].value;
        }
        for (std::size_t i = 0; i < command.pwm_output_count; ++i) {
          pwm_outputs[command.pwm_outputs[i].id] =
              command.pwm_outputs[i].output;
          applied_pwm_outputs[command.pwm_outputs[i].id] =
              command.pwm_outputs[i].output;
        }
      }
      if (!enabled || !tracker.last_seen ||
          sim.now() - *tracker.last_seen >=
              std::chrono::microseconds{config.hardware.command_timeout_us}) {
        Neutralize(devices);
      }
    }
  }

  void ApplyIdealMotors() {
    const double dt = std::chrono::duration<double>(period).count();
    for (const auto& motor : config.hardware.motors) {
      const auto request = ValueOr(applied_motors, motor.id,
                                   hardware::MotorRequest{.id = motor.id});
      Axis& axis = motors[motor.id];
      switch (request.mode) {
        case hardware::Mode::kNeutral: {
          const double tau = motor.brake ? kNeutralBrakeTauS : kNeutralCoastTauS;
          axis.velocity *= 1.0 - std::min(1.0, dt / tau);
          if (std::abs(axis.velocity) < kNeutralStandstillRps) {
            axis.velocity = 0;
          }
          axis.position += axis.velocity * dt;
          break;
        }
        case hardware::Mode::kVelocity:
          axis.velocity = request.demand;
          axis.position += request.demand * dt;
          break;
        case hardware::Mode::kPosition:
        case hardware::Mode::kMotionMagic:
          axis.position = request.demand;
          axis.velocity = 0;
          break;
        default:
          break;
      }
    }
  }

  void Publish() {
    Collect();
    if (ideal) ApplyIdealMotors();
    const auto& hw = config.hardware;
    hardware::State state{};
    state.config_id = config_id;
    state.boot_id = kBootId;
    state.epoch = 1;
    state.sample_time_us = GatewayUs(latency);
    state.flags = hardware::kConfigured | (enabled ? hardware::kEnabled : 0u);

    auto fill = [](auto& samples, auto& count, const auto& configs,
                   auto sample_of) {
      count =
          static_cast<std::uint16_t>(std::min(configs.size(), samples.size()));
      for (std::size_t i = 0; i < count; ++i) {
        samples[i] = sample_of(configs[i]);
      }
    };
    fill(state.motors, state.motor_count, hw.motors, [&](const auto& c) {
      if (!Connected(Kind::kMotor, c.id)) {
        return hardware::MotorSample{.id = c.id};
      }
      const Axis axis = ValueOr(motors, c.id, Axis{});
      return hardware::MotorSample{.id = c.id,
                                   .valid = true,
                                   .position_rot = axis.position,
                                   .velocity_rps = axis.velocity,
                                   .position_age_us = axis.position_age_us,
                                   .velocity_age_us = axis.velocity_age_us};
    });
    fill(state.sensors, state.sensor_count, hw.sensors, [&](const auto& c) {
      if (!Connected(Kind::kCANcoder, c.id)) {
        return hardware::SensorSample{.id = c.id};
      }
      const Axis axis = ValueOr(sensors, c.id, Axis{});
      return hardware::SensorSample{.id = c.id,
                                    .valid = true,
                                    .position_rot = axis.position,
                                    .velocity_rps = axis.velocity,
                                    .position_age_us = axis.position_age_us,
                                    .velocity_age_us = axis.velocity_age_us};
    });
    fill(state.digital_inputs, state.digital_input_count, hw.digital_inputs,
         [&](const auto& c) {
           if (!Connected(Kind::kDigitalInput, c.id)) {
             return hardware::DigitalInputSample{c.id, false, false};
           }
           return hardware::DigitalInputSample{
               c.id, true, ValueOr(digital_inputs, c.id, false)};
         });
    fill(state.digital_outputs, state.digital_output_count, hw.digital_outputs,
         [&](const auto& c) {
           if (!Connected(Kind::kDigitalOutput, c.id)) {
             return hardware::DigitalOutputSample{c.id, false, false};
           }
           return hardware::DigitalOutputSample{
               c.id, true, ValueOr(applied_digital_outputs, c.id, false)};
         });
    fill(state.analog_inputs, state.analog_input_count, hw.analog_inputs,
         [&](const auto& c) {
           if (!Connected(Kind::kAnalogInput, c.id)) {
             return hardware::AnalogInputSample{.id = c.id};
           }
           return hardware::AnalogInputSample{
               .id = c.id,
               .valid = true,
               .voltage = ValueOr(analog_inputs, c.id, 0.0)};
         });
    fill(state.encoders, state.encoder_count, hw.encoders, [&](const auto& c) {
      if (!Connected(Kind::kEncoder, c.id)) {
        return hardware::EncoderSample{.id = c.id};
      }
      const Axis axis = ValueOr(encoders, c.id, Axis{});
      return hardware::EncoderSample{.id = c.id,
                                     .valid = true,
                                     .position_rot = axis.position,
                                     .velocity_rps = axis.velocity};
    });
    fill(state.pwm_outputs, state.pwm_output_count, hw.pwm_outputs,
         [&](const auto& c) {
           if (!Connected(Kind::kPwm, c.id)) {
             return hardware::PwmSample{c.id, false, 0.0};
           }
           return hardware::PwmSample{c.id, true,
                                      ValueOr(applied_pwm_outputs, c.id, 0.0)};
         });

    Packet packet{};
    packet.size =
        static_cast<std::uint32_t>(hardware::Encode(state, packet.data));
    sim.publish(hardware::kStateTopic, packet);
    ++states_sent;
    if (ds_packet) PublishDriverStation(*ds_packet);

    // What the bridge would report having received exactly this, now.
    clock.Observe(state.boot_id, state.sample_time_us,
                  static_cast<std::uint64_t>(sim.now().nanos() / 1000));
    ::bridge::BridgeStatus status{
        states_sent,
        0,
        0,
        0,
        ds_packets_sent,
        0,
        0,
        0,
        true,
        clock.boot_id(),
        clock.offset_us(),
        clock.spread_us(),
        clock.resets_count(),
        clock.ignored_count(),
        static_cast<std::uint32_t>(clock.window_count()),
        clock.converged(),
        state.health.valid,
        state.health.battery_v,
        state.health.brownout,
        state.health.can_utilization_frac,
        0,
        state.counters.rejected_command_count,
        state.counters.neutralized_motor_count,
        state.counters.apply_failure_count,
        state.counters.device_reset_count};
    sim.publish(::bridge::topics::kStatus, status);
  }

  void PublishDriverStation(driver_station::DriverStationData data) {
    data.sample_time_us = GatewayUs();
    data.boot_id = kBootId;
    ds_mode.Stamp(data);
    Packet packet{};
    packet.size =
        static_cast<std::uint32_t>(driver_station::Encode(data, packet.data));
    if (packet.size == 0) {
      throw std::invalid_argument(
          "SimHardware: the Driver Station packet does not encode");
    }
    sim.publish(hardware::kDriverStationTopic, packet);
    ++ds_packets_sent;
  }
};

SimHardware::SimHardware(Sim& sim) : impl_{std::make_shared<Impl>(sim)} {
  Impl& impl = *impl_;
  impl.config = config::ParseRobotConfig(SimAccess::config_path(sim));
  impl.config.hardware.commissioned = true;
  impl.config_id = hardware::ConfigurationId(impl.config.hardware);
  impl.period = std::chrono::microseconds{impl.config.hardware.period_us};
  for (const auto& [name, devices] : impl.config.subsystems) {
    impl.requests.emplace(
        name, Impl::RequestTracker{.topic = hardware::RequestTopic(name),
                                  .last_seen = std::nullopt});
    impl.Neutralize(devices);
  }
  SimAccess::every(sim, impl.period, [weak = std::weak_ptr<Impl>{impl_}] {
    const auto impl = weak.lock();
    if (!impl) return false;
    impl->Publish();
    return true;
  });
}

SimHardware::~SimHardware() = default;

void SimHardware::enable(bool on) { impl_->enabled = on; }
void SimHardware::ideal_motors(bool on) { impl_->ideal = on; }

void SimHardware::set_motor(std::string_view node, std::string_view device,
                            double position, double velocity,
                            std::uint32_t position_age_us,
                            std::uint32_t velocity_age_us) {
  impl_->motors[impl_->Id(node, Kind::kMotor, device)] = {
      position, velocity, position_age_us, velocity_age_us};
}

void SimHardware::set_digital_input(std::string_view node,
                                    std::string_view device, bool value) {
  impl_->digital_inputs[impl_->Id(node, Kind::kDigitalInput, device)] = value;
}

void SimHardware::set_analog_input(std::string_view node,
                                   std::string_view device, double volts) {
  impl_->analog_inputs[impl_->Id(node, Kind::kAnalogInput, device)] = volts;
}

void SimHardware::set_encoder(std::string_view node, std::string_view device,
                              double position, double velocity) {
  impl_->encoders[impl_->Id(node, Kind::kEncoder, device)] = {position,
                                                              velocity};
}

void SimHardware::set_cancoder(std::string_view node, std::string_view device,
                               double position, double velocity,
                               std::uint32_t position_age_us,
                               std::uint32_t velocity_age_us) {
  impl_->sensors[impl_->Id(node, Kind::kCANcoder, device)] = {
      position, velocity, position_age_us, velocity_age_us};
}

void SimHardware::set_gyro(std::string_view node, std::string_view device,
                           double yaw, double yaw_rate,
                           std::uint32_t yaw_age_us,
                           std::uint32_t yaw_rate_age_us) {
  impl_->sensors[impl_->Id(node, Kind::kGyro, device)] = {
      yaw, yaw_rate, yaw_age_us, yaw_rate_age_us};
}

void SimHardware::set_gateway_clock(Duration gateway_offset, Duration latency) {
  if (latency < Duration::zero()) {
    throw std::invalid_argument("SimHardware: negative sample latency");
  }
  if (impl_->sim.now().nanos() + (gateway_offset - latency).count() < 0) {
    throw std::invalid_argument(
        "SimHardware: the gateway clock would read before 0");
  }
  impl_->gateway_offset = gateway_offset;
  impl_->latency = latency;
}

void SimHardware::set_connected(std::string_view node, std::string_view device,
                                bool connected) {
  bool found = false;
  for (const Kind kind :
       {Kind::kMotor, Kind::kCANcoder, Kind::kGyro, Kind::kDigitalInput,
        Kind::kDigitalOutput, Kind::kAnalogInput, Kind::kEncoder, Kind::kPwm}) {
    std::uint16_t id = 0;
    try {
      id = impl_->Id(node, kind, device);
    } catch (const std::invalid_argument&) {
      continue;  // not a device of this kind
    }
    found = true;
    const std::pair key{TableOf(kind), id};
    if (connected) {
      impl_->disconnected.erase(key);
    } else {
      impl_->disconnected.insert(key);
    }
  }
  if (!found) {
    throw std::invalid_argument(std::string{node} + ": no device named '" +
                                std::string{device} + "' in subsystem '" +
                                std::string{node} + "'");
  }
}

void SimHardware::set_driver_station(
    const driver_station::DriverStationData& data) {
  impl_->ds_packet = data;
}

void SimHardware::stop_driver_station() { impl_->ds_packet.reset(); }

hardware::MotorRequest SimHardware::motor_request(
    std::string_view node, std::string_view device) const {
  const std::uint16_t id = impl_->Id(node, Kind::kMotor, device);
  impl_->Collect();
  return ValueOr(impl_->motor_requests, id, hardware::MotorRequest{.id = id});
}

bool SimHardware::digital_output(std::string_view node,
                                 std::string_view device) const {
  const std::uint16_t id = impl_->Id(node, Kind::kDigitalOutput, device);
  impl_->Collect();
  return ValueOr(impl_->digital_outputs, id, false);
}

double SimHardware::pwm(std::string_view node, std::string_view device) const {
  const std::uint16_t id = impl_->Id(node, Kind::kPwm, device);
  impl_->Collect();
  return ValueOr(impl_->pwm_outputs, id, 0.0);
}

hardware::MotorRequest SimHardware::applied_motor(
    std::string_view node, std::string_view device) const {
  const std::uint16_t id = impl_->Id(node, Kind::kMotor, device);
  impl_->Collect();
  return impl_->applied_motors.at(id);
}

bool SimHardware::applied_digital_output(std::string_view node,
                                        std::string_view device) const {
  const std::uint16_t id = impl_->Id(node, Kind::kDigitalOutput, device);
  impl_->Collect();
  return impl_->applied_digital_outputs.at(id);
}

double SimHardware::applied_pwm(std::string_view node,
                               std::string_view device) const {
  const std::uint16_t id = impl_->Id(node, Kind::kPwm, device);
  impl_->Collect();
  return impl_->applied_pwm_outputs.at(id);
}

driver_station::DriverStationData DsPacket(std::uint32_t flags) {
  driver_station::DriverStationData data{};
  data.fms.flags = flags | driver_station::kDsAttached;
  return data;
}

namespace {
std::string LaneTopic(std::string_view subsystem, std::string_view source) {
  return "/" + std::string{subsystem} + "/target/" + std::string{source};
}
}  // namespace

std::uint64_t DriverStationGeneration(const Sim& sim) {
  const std::uint64_t count = SimAccess::message_count(
      sim, hardware::kDriverStationTopic, sizeof(Packet));
  if (count == 0) return 0;
  Packet packet{};
  SimAccess::read_raw(sim, hardware::kDriverStationTopic, sizeof packet,
                      count - 1, &packet);
  driver_station::DriverStationData data{};
  if (!driver_station::Decode(packet.bytes(), data)) return 0;
  return detail::DsModeOf(data.fms.flags) != detail::DsMode::kNone
             ? data.mode_since_us
             : 0;
}

void PublishTargetRaw(Sim& sim, std::string_view subsystem,
                      std::string_view source, const void* message,
                      std::size_t bytes, std::size_t alignment,
                      std::uint64_t mode_since_us) {
  std::vector<std::byte> envelope(
      detail::TargetEnvelopeBytes(bytes, alignment));
  detail::EncodeTargetEnvelope(mode_since_us, message != nullptr, message,
                               bytes, alignment, envelope);
  SimAccess::publish_raw(sim, LaneTopic(subsystem, source), envelope.data(),
                         envelope.size());
}

bool LatestTargetRaw(const Sim& sim, std::string_view subsystem,
                     std::string_view source, void* out, std::size_t bytes,
                     std::size_t alignment,
                     detail::TargetEnvelopeHeader& header) {
  const std::string topic = LaneTopic(subsystem, source);
  std::vector<std::byte> envelope(
      detail::TargetEnvelopeBytes(bytes, alignment));
  const std::uint64_t count =
      SimAccess::message_count(sim, topic, envelope.size());
  if (count == 0) return false;
  SimAccess::read_raw(sim, topic, envelope.size(), count - 1, envelope.data());
  header = detail::DecodeTargetEnvelopeHeader(envelope.data());
  std::memcpy(out, envelope.data() + detail::kTargetEnvelopeHeaderBytes, bytes);
  return true;
}

}  // namespace talos::testing
