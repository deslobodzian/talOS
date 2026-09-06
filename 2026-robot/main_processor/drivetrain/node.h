#pragma once

#include "talOS/hardware/messages.h"
#include "2026-robot/main_processor/drivetrain/drive_message_generated.h"
#include "2026-robot/main_processor/drivetrain/packet.h"
#include "2026-robot/main_processor/drivetrain/swerve.h"
#include "talOS/events/handles.h"

namespace talos::drive {
template <typename Loop>
class DrivetrainNode {
 public:
  explicit DrivetrainNode(Loop& loop, hardware::Config config,
                          hardware::Devices devices = {})
      : config_{std::move(config)},
        devices_{std::move(devices)},
        config_id_{hardware::ConfigurationId(config_)} {
    ValidateSwerve(config_);
    if (devices_.motors.empty()) {
      for (const auto& m : config_.motors) {
        devices_.motors.push_back(m.id);
      }
    }
    event::watch<Packet, &DrivetrainNode::OnState>(loop, kStateTopic, this);
    event::watch<ChassisTarget, &DrivetrainNode::OnTarget>(loop, kTargetTopic,
                                                           this);
    command_ = event::make_sender<Packet>(loop, kDriveRequestTopic);
    drive_state_ =
        event::make_sender<DrivetrainState>(loop, kDrivetrainStateTopic);
    timer_ = event::make_timer<&DrivetrainNode::Tick>(loop, "swerve", this);
  }
  void Start(event::MonotonicTime first) {
    timer_.setup_periodic(first, std::chrono::microseconds{config_.period_us});
  }
  uint16_t timer_id() const { return timer_.id(); }

 private:
  void OnState(const event::Context& context, const Packet& packet) {
    hardware::State state;
    if (!hardware::Decode(packet.bytes(), state)) return;
    if (config_id_ != 0 && state.config_id != config_id_) return;
    if (have_state_ && state.boot_id == state_.boot_id &&
        state.sample_time_us <= state_.sample_time_us)
      return;
    for (uint16_t id : devices_.motors) {
      bool found = false;
      for (std::size_t i = 0; i < state.motor_count; ++i) {
        if (state.motors[i].id == id) {
          found = true;
          break;
        }
      }
      if (!found) return;
    }
    state_ = state;
    state_received_ = context.now;
    have_state_ = true;
  }
  void OnTarget(const event::Context&, const ChassisTarget& target) {
    target_ = target;
  }
  void Tick(const event::Context& context) {
    if (!have_state_) return;
    const auto timeout = std::chrono::microseconds{config_.command_timeout_us};
    const auto issued = event::MonotonicTime::from_nanos(target_.issued_ns());
    bool valid =
        target_.enabled() && issued <= context.now &&
        context.now - issued < timeout &&
        context.now - state_received_ < timeout &&
        (state_.flags & (hardware::kConfigured | hardware::kEnabled)) ==
            (hardware::kConfigured | hardware::kEnabled) &&
        !(state_.flags & hardware::kHardwareFault);
    for (uint16_t id : devices_.motors) {
      for (std::size_t i = 0; i < state_.motor_count; ++i) {
        if (state_.motors[i].id == id) {
          valid &= state_.motors[i].valid;
          break;
        }
      }
    }
    hardware::Command command;
    if (valid)
      command = SwerveCommand(config_, state_, target_.vx_mps(),
                              target_.vy_mps(), target_.omega_radps());
    else {
      command.config_id = state_.config_id;
      command.boot_id = state_.boot_id;
      command.epoch = state_.epoch;
      command.observed_time_us = state_.sample_time_us;
      command.count = static_cast<uint16_t>(config_.motors.size());
      for (std::size_t i = 0; i < config_.motors.size(); ++i)
        command.motors[i].id = config_.motors[i].id;
    }
    Packet packet{};
    packet.size = hardware::Encode(command, packet.data);
    if (packet.size) command_.send(packet);

    double vx = 0, vy = 0, omega = 0;
    ComputeForwardKinematics(state_, vx, vy, omega);
    double yaw_rot = 0, yaw_rate_rps = omega;
    for (std::size_t i = 0; i < state_.sensor_count; ++i) {
      if (state_.sensors[i].valid) {
        yaw_rot = state_.sensors[i].position_rot;
        yaw_rate_rps = state_.sensors[i].velocity_rps;
        break;
      }
    }
    bool enabled = (state_.flags & (hardware::kConfigured | hardware::kEnabled)) ==
                   (hardware::kConfigured | hardware::kEnabled);
    uint64_t ts_ns = state_.sample_time_us * 1000;
    DrivetrainState dt_state{ts_ns, enabled, vx, vy, omega, yaw_rot, yaw_rate_rps};
    drive_state_.send(dt_state);
  }
  hardware::Config config_;
  hardware::Devices devices_;
  uint64_t config_id_;
  hardware::State state_{};
  bool have_state_{false};
  event::MonotonicTime state_received_{};
  ChassisTarget target_{};
  event::Sender<Loop, Packet> command_;
  event::Sender<Loop, DrivetrainState> drive_state_;
  event::Timer<Loop> timer_;
};
}  // namespace talos::drive
