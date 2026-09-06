#pragma once

#include <chrono>
#include <numbers>

#include "2026-robot/main_processor/drivetrain/drive_message_generated.h"
#include "2026-robot/main_processor/drivetrain/packet.h"
#include "2026-robot/main_processor/drivetrain/swerve.h"
#include "talOS/events/handles.h"
#include "talOS/hardware/messages.h"

namespace talos::drive {
template <typename Loop>
class DrivetrainNode {
 public:
  // simulate_heading derives the heading by integrating the rotation the
  // modules are producing, instead of reading the IMU. Simulation backends do
  // not model a gyro -- the Pigeon reports a valid, permanently zero yaw, which
  // is indistinguishable from a robot facing downfield and never turning -- so
  // in simulation the sensor cannot be believed. It stays off by default: on
  // real hardware an integrated heading has no absolute reference and drifts.
  explicit DrivetrainNode(Loop& loop, hardware::Config config,
                          SwerveGeometry geometry,
                          hardware::Devices devices = {},
                          bool simulate_heading = false)
      : config_{std::move(config)},
        geometry_{geometry},
        devices_{std::move(devices)},
        config_id_{hardware::ConfigurationId(config_)},
        imu_id_{FindImu(config_)},
        simulate_heading_{simulate_heading} {
    ValidateSwerve(config_, geometry_);
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
      command = SwerveCommand(config_, geometry_, state_, target_.vx_mps(),
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
    ComputeForwardKinematics(geometry_, state_, vx, vy, omega);
    // omega is rad/s from the module states; DrivetrainState carries a rate in
    // rotations per second, the same unit the IMU reports.
    constexpr double kTau = 2 * std::numbers::pi;
    double yaw_rot = 0, yaw_rate_rps = omega / kTau;
    if (simulate_heading_) {
      // Integrating the measured module states, not the requested target, so
      // the inverse kinematics and the drive/steer device mapping still have to
      // be right for the robot to turn the way it was asked to.
      if (have_last_tick_) {
        const double dt_s =
            std::chrono::duration<double>{context.now - last_tick_}.count();
        if (dt_s > 0) simulated_yaw_rot_ += yaw_rate_rps * dt_s;
      }
      last_tick_ = context.now;
      have_last_tick_ = true;
      yaw_rot = simulated_yaw_rot_;
    } else {
      // Heading comes from the IMU specifically. Taking the first valid sensor
      // instead reads whichever device sorts first, which on a swerve is a
      // steer encoder -- a module azimuth published as the robot's heading.
      for (std::size_t i = 0; i < state_.sensor_count && imu_id_ != 0; ++i) {
        if (state_.sensors[i].id == imu_id_ && state_.sensors[i].valid) {
          yaw_rot = state_.sensors[i].position_rot;
          yaw_rate_rps = state_.sensors[i].velocity_rps;
          break;
        }
      }
    }
    bool enabled =
        (state_.flags & (hardware::kConfigured | hardware::kEnabled)) ==
        (hardware::kConfigured | hardware::kEnabled);
    uint64_t ts_ns = state_.sample_time_us * 1000;
    DrivetrainState dt_state{ts_ns, enabled, vx,          vy,
                             omega, yaw_rot, yaw_rate_rps};
    drive_state_.send(dt_state);
  }
  // Zero when the subsystem declares no IMU, which leaves the heading at zero
  // rather than borrowing an unrelated sensor's position.
  static uint16_t FindImu(const hardware::Config& config) {
    for (const auto& sensor : config.sensors)
      if (sensor.kind == hardware::SensorKind::kPigeon2) return sensor.id;
    return 0;
  }

  hardware::Config config_;
  SwerveGeometry geometry_;
  hardware::Devices devices_;
  uint64_t config_id_;
  uint16_t imu_id_{0};
  bool simulate_heading_{false};
  double simulated_yaw_rot_{0.0};
  event::MonotonicTime last_tick_{};
  bool have_last_tick_{false};
  hardware::State state_{};
  bool have_state_{false};
  event::MonotonicTime state_received_{};
  ChassisTarget target_{};
  event::Sender<Loop, Packet> command_;
  event::Sender<Loop, DrivetrainState> drive_state_;
  event::Timer<Loop> timer_;
};
}  // namespace talos::drive
