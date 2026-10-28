#include "talOS/devices/testing.h"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <map>
#include <string>

#include "talOS/configuration/config_parser.h"
#include "talOS/devices/access.h"
#include "talOS/hardware/packet.h"

namespace talos::testing {
namespace {

using hardware::Packet;
using detail::SimAccess;
using Kind = detail::DeviceKind;

struct Axis {
  double position = 0;
  double velocity = 0;
};

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
  std::map<std::uint16_t, bool> digital_outputs;
  std::map<std::uint16_t, double> pwm_outputs;
  std::map<std::string, std::uint64_t> cursors;  // per request topic

  explicit Impl(Sim& s) : sim{s} {}

  std::uint16_t Id(std::string_view node, Kind kind,
                   std::string_view device) const {
    return detail::HardwareAccess::device_id(config, node, kind, device);
  }

  // Every request each subsystem flushed since the last look, in order.
  void Collect() {
    for (const auto& [subsystem, devices] : config.subsystems) {
      const std::string topic = hardware::RequestTopic(subsystem);
      const std::uint64_t count =
          SimAccess::message_count(sim, topic, sizeof(Packet));
      std::uint64_t& cursor = cursors[topic];
      for (; cursor < count; ++cursor) {
        Packet packet{};
        SimAccess::read_raw(sim, topic, sizeof packet, cursor, &packet);
        hardware::Command command{};
        if (!hardware::Decode(packet.bytes(), command)) continue;
        for (std::size_t i = 0; i < command.count; ++i)
          motor_requests[command.motors[i].id] = command.motors[i];
        for (std::size_t i = 0; i < command.digital_output_count; ++i)
          digital_outputs[command.digital_outputs[i].id] =
              command.digital_outputs[i].value;
        for (std::size_t i = 0; i < command.pwm_output_count; ++i)
          pwm_outputs[command.pwm_outputs[i].id] =
              command.pwm_outputs[i].output;
      }
    }
  }

  void ApplyIdealMotors() {
    const double dt = std::chrono::duration<double>(period).count();
    for (const auto& [id, request] : motor_requests) {
      Axis& axis = motors[id];
      switch (request.mode) {
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
    state.boot_id = 1;
    state.epoch = 1;
    state.sample_time_us = static_cast<std::uint64_t>(sim.now().nanos() / 1000);
    state.flags = hardware::kConfigured | (enabled ? hardware::kEnabled : 0u);

    auto fill = [](auto& samples, auto& count, const auto& configs,
                   auto sample_of) {
      count =
          static_cast<std::uint16_t>(std::min(configs.size(), samples.size()));
      for (std::size_t i = 0; i < count; ++i)
        samples[i] = sample_of(configs[i]);
    };
    fill(state.motors, state.motor_count, hw.motors, [&](const auto& c) {
      const Axis axis = ValueOr(motors, c.id, Axis{});
      return hardware::MotorSample{.id = c.id,
                                   .valid = true,
                                   .position_rot = axis.position,
                                   .velocity_rps = axis.velocity};
    });
    fill(state.sensors, state.sensor_count, hw.sensors, [&](const auto& c) {
      const Axis axis = ValueOr(sensors, c.id, Axis{});
      return hardware::SensorSample{.id = c.id,
                                    .valid = true,
                                    .position_rot = axis.position,
                                    .velocity_rps = axis.velocity};
    });
    fill(state.digital_inputs, state.digital_input_count, hw.digital_inputs,
         [&](const auto& c) {
           return hardware::DigitalInputSample{
               c.id, true, ValueOr(digital_inputs, c.id, false)};
         });
    fill(state.digital_outputs, state.digital_output_count, hw.digital_outputs,
         [&](const auto& c) {
           return hardware::DigitalOutputSample{
               c.id, true, ValueOr(digital_outputs, c.id, false)};
         });
    fill(state.analog_inputs, state.analog_input_count, hw.analog_inputs,
         [&](const auto& c) {
           return hardware::AnalogInputSample{
               .id = c.id,
               .valid = true,
               .voltage = ValueOr(analog_inputs, c.id, 0.0)};
         });
    fill(state.encoders, state.encoder_count, hw.encoders, [&](const auto& c) {
      const Axis axis = ValueOr(encoders, c.id, Axis{});
      return hardware::EncoderSample{.id = c.id,
                                     .valid = true,
                                     .position_rot = axis.position,
                                     .velocity_rps = axis.velocity};
    });
    fill(state.pwm_outputs, state.pwm_output_count, hw.pwm_outputs,
         [&](const auto& c) {
           return hardware::PwmSample{c.id, true,
                                      ValueOr(pwm_outputs, c.id, 0.0)};
         });

    Packet packet{};
    packet.size =
        static_cast<std::uint32_t>(hardware::Encode(state, packet.data));
    sim.publish(hardware::kStateTopic, packet);
  }
};

SimHardware::SimHardware(Sim& sim) : impl_{std::make_shared<Impl>(sim)} {
  Impl& impl = *impl_;
  impl.config = config::ParseRobotConfig(SimAccess::config_path(sim));
  impl.config.hardware.commissioned = true;
  impl.config_id = hardware::ConfigurationId(impl.config.hardware);
  impl.period = std::chrono::microseconds{impl.config.hardware.period_us};
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
                            double position, double velocity) {
  impl_->motors[impl_->Id(node, Kind::kMotor, device)] = {position, velocity};
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
                               double position, double velocity) {
  impl_->sensors[impl_->Id(node, Kind::kCANcoder, device)] = {position,
                                                              velocity};
}

void SimHardware::set_gyro(std::string_view node, std::string_view device,
                           double yaw, double yaw_rate) {
  impl_->sensors[impl_->Id(node, Kind::kGyro, device)] = {yaw, yaw_rate};
}

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

}  // namespace talos::testing
