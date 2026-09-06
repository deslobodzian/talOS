#pragma once

#include <cmath>
#include <cstdint>
#include <numbers>

#include "2026-robot/main_processor/driver_station/driver_station_message_generated.h"
#include "2026-robot/main_processor/driver_station/packet.h"
#include "2026-robot/main_processor/drivetrain/drive_message_generated.h"
#include "2026-robot/main_processor/drivetrain/packet.h"
#include "2026-robot/main_processor/intake/intake_message_generated.h"
#include "2026-robot/main_processor/intake/packet.h"
#include "2026-robot/main_processor/shooter/packet.h"
#include "2026-robot/main_processor/shooter/shooter_message_generated.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/handles.h"

// The operator interface: what the driver's inputs mean. It consumes the
// decoded Driver Station state and produces requests -- stick mapping, scaling,
// deadband, button bindings. It is one producer among several, not the
// authority: the arbiter decides whether teleop is the mode currently driving
// and is the sole writer of the topics the subsystems consume.
//
// Everything here is game logic and is expected to change through a season.
// Keeping it out of the driver_station node means the wire format has one
// reader that never changes with it.
namespace talos::oi {

struct OperatorInterfaceConfig {
  double max_linear_mps{4.0};
  double max_angular_radps{6.0};
  double deadband{0.05};
  double shooter_target_rps{60.0};
  double intake_target_rps{30.0};
  // Field-oriented driving needs a heading, so it stays off until the
  // drivetrain has published one; see OnDrivetrainState.
  bool field_oriented{true};
  // Bit 0 is button 1, matching the Driver Station's numbering.
  uint32_t shoot_button_mask{1u << 0};
  uint32_t intake_button_mask{1u << 1};
};

template <typename Loop>
class OperatorInterfaceNode {
 public:
  explicit OperatorInterfaceNode(Loop& loop,
                                 OperatorInterfaceConfig config = {})
      : config_{config},
        chassis_target_{event::make_sender<talos::drive::ChassisTarget>(
            loop, talos::drive::kTeleopTargetTopic)},
        shooter_target_{event::make_sender<talos::shooter::ShooterTarget>(
            loop, talos::shooter::kTeleopShooterTargetTopic)},
        intake_target_{event::make_sender<talos::intake::IntakeTarget>(
            loop, talos::intake::kTeleopIntakeTargetTopic)} {
    event::watch<driver_station::DriverStationState,
                 &OperatorInterfaceNode::OnDriverStation>(
        loop, driver_station::kDsStateTopic, this);
    event::watch<talos::drive::DrivetrainState,
                 &OperatorInterfaceNode::OnDrivetrainState>(
        loop, talos::drive::kDrivetrainStateTopic, this);
  }

  void Start(event::MonotonicTime = {}) {}

  bool field_oriented_active() const {
    return config_.field_oriented && have_heading_;
  }
  bool commanding() const { return commanding_; }

 private:
  // Zero inside the deadband, then rescaled so the output ramps from 0 at the
  // threshold to full scale at 1.0 instead of stepping.
  double Deadband(double value) const {
    const double magnitude = std::abs(value);
    if (magnitude <= config_.deadband || config_.deadband >= 1.0) return 0.0;
    const double scaled =
        (magnitude - config_.deadband) / (1.0 - config_.deadband);
    return std::copysign(scaled, value);
  }

  // Heading for field-oriented driving. The drivetrain owns the yaw, so this
  // node consumes it rather than reading the gyro a second time.
  void OnDrivetrainState(const event::Context&,
                         const talos::drive::DrivetrainState& state) {
    yaw_rot_ = state.yaw_rot();
    have_heading_ = true;
  }

  // Rotates a field-relative request into the robot frame. Field-oriented is
  // skipped entirely until a heading has arrived, so a missing drivetrain
  // publisher degrades to robot-relative rather than to driving on yaw 0 --
  // which would look correct only while the robot happens to face forward.
  void ToRobotFrame(double& vx, double& vy) const {
    if (!field_oriented_active()) return;
    const double yaw_rad = yaw_rot_ * 2.0 * std::numbers::pi;
    const double cos_yaw = std::cos(yaw_rad), sin_yaw = std::sin(yaw_rad);
    const double robot_vx = vx * cos_yaw + vy * sin_yaw;
    const double robot_vy = -vx * sin_yaw + vy * cos_yaw;
    vx = robot_vx;
    vy = robot_vy;
  }

  // One explicit disabled request on the falling edge, so the subsystems are
  // told to stop rather than being left to time the last request out.
  void Release(const event::Context& ctx) {
    if (!commanding_) return;
    commanding_ = false;
    chassis_target_.send(
        talos::drive::ChassisTarget{0.0, 0.0, 0.0, ctx.now.nanos(), false});
    shooter_target_.send(
        talos::shooter::ShooterTarget{0.0, ctx.now.nanos(), false});
    intake_target_.send(talos::intake::IntakeTarget{
        static_cast<uint64_t>(ctx.now.nanos()), false, 0.0f});
  }

  void OnDriverStation(const event::Context& ctx,
                       const driver_station::DriverStationState& state) {
    const uint32_t flags = state.fms().flags();
    const bool enabled = (flags & driver_station::kEnabled) != 0;
    const bool teleop = (flags & driver_station::kTeleop) != 0;
    const auto& stick = state.stick0();

    // The arbiter also gates on mode, but a producer that keeps talking while
    // disabled would still be publishing live requests onto its own topic.
    if (!enabled || !teleop || !stick.connected()) {
      Release(ctx);
      return;
    }

    const double raw_vx = -static_cast<double>(stick.axis1());
    const double raw_vy = -static_cast<double>(stick.axis0());
    const double raw_omega = -static_cast<double>(stick.axis2());

    double vx = Deadband(raw_vx) * config_.max_linear_mps;
    double vy = Deadband(raw_vy) * config_.max_linear_mps;
    const double omega = Deadband(raw_omega) * config_.max_angular_radps;
    ToRobotFrame(vx, vy);

    commanding_ = true;
    chassis_target_.send(
        talos::drive::ChassisTarget{vx, vy, omega, ctx.now.nanos(), true});

    const bool shoot = (stick.buttons() & config_.shoot_button_mask) != 0;
    shooter_target_.send(talos::shooter::ShooterTarget{
        shoot ? config_.shooter_target_rps : 0.0, ctx.now.nanos(), shoot});

    const bool intake =
        (stick.buttons() & config_.intake_button_mask) != 0;
    intake_target_.send(talos::intake::IntakeTarget{
        static_cast<uint64_t>(ctx.now.nanos()), intake,
        intake ? static_cast<float>(config_.intake_target_rps) : 0.0f});
  }

  OperatorInterfaceConfig config_;
  event::Sender<Loop, talos::drive::ChassisTarget> chassis_target_;
  event::Sender<Loop, talos::shooter::ShooterTarget> shooter_target_;
  event::Sender<Loop, talos::intake::IntakeTarget> intake_target_;

  double yaw_rot_{0.0};
  bool have_heading_{false};
  bool commanding_{false};
};

}  // namespace talos::oi
