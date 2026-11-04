#pragma once

#include <chrono>
#include <cmath>
#include <numbers>

#include "drivetrain/drive_message_generated.h"
#include "drivetrain/topics.h"
#include "intake/intake_message_generated.h"
#include "shooter/shooter_message_generated.h"
#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

namespace operator_interface {

// Maps each Driver Station packet (/hw/state/driver_station, through
// talos::DriverStation) to teleop requests as it arrives: stick scaling,
// deadband and field-oriented rotation for the chassis, buttons and setpoints
// for the shooter and intake. It writes only the teleop lanes
// (/<sub>/target/teleop); each subsystem's talos::Target decides from the
// match mode whether that lane is the one it follows.
class OperatorInterface {
 public:
  explicit OperatorInterface(talos::Node& node)
      : node_{node},
        max_linear_mps_{node.param("max_linear_mps", 4.0)},
        max_angular_radps_{node.param("max_angular_radps", 6.0)},
        deadband_{node.param("deadband", 0.05)},
        field_oriented_{node.param("field_oriented", true)},
        heading_timeout_{HeadingTimeout(node)},
        shooter_rps_{node.param("shooter_target_rps", 60.0)},
        intake_rps_{node.param("intake_target_rps", 30.0)},
        // 1-based, matching Driver Station numbering.
        shoot_button_{node.param<int>("shoot_button", 1)},
        intake_button_{node.param<int>("intake_button", 2)},
        chassis_{node, "drivetrain", "teleop"},
        shooter_{node, "shooter", "teleop"},
        intake_{node, "intake", "teleop"},
        ds_{node, [this] { OnDriver(); }} {
    // The drivetrain owns the yaw. A non-finite one carries no direction, so
    // it is dropped and the previous heading keeps aging toward stale.
    node.subscribe<drivetrain::DrivetrainState>(
        drivetrain::topics::kState,
        [this](const drivetrain::DrivetrainState& s) {
          if (!std::isfinite(s.yaw_rot())) return;
          yaw_rot_ = s.yaw_rot();
          heading_at_ = node_.now();
          have_heading_ = true;
        });
  }

  // A Driver Station that stops talking gets the same one release per lane
  // as a disable, instead of the last request aging out downstream.
  void update() {
    if (!ds_.connected()) Stop();
  }

 private:
  // Five 20 ms periods by default, so one or two dropped packets do not flap
  // the frame. A non-positive or non-finite value keeps the default.
  static talos::Duration HeadingTimeout(const talos::Node& node) {
    const double s = node.param("heading_timeout_s", 0.1);
    return std::chrono::duration_cast<talos::Duration>(
        std::chrono::duration<double>{std::isfinite(s) && s > 0 ? s : 0.1});
  }

  // Zero inside the deadband, then rescaled to ramp from 0 at the threshold
  // to full scale at 1 instead of stepping.
  double Deadband(double value) const {
    const double magnitude = std::abs(value);
    if (magnitude <= deadband_ || deadband_ >= 1.0) return 0.0;
    return std::copysign((magnitude - deadband_) / (1.0 - deadband_), value);
  }

  // One release per lane on the falling edge, so each subsystem drops
  // teleop at once rather than timing the last request out.
  void Stop() {
    if (!commanding_) return;
    commanding_ = false;
    chassis_.release();
    shooter_.release();
    intake_.release();
  }

  void OnDriver() {
    const talos::Joystick stick = ds_.joystick(0);
    // Each Target also gates on mode, but a producer that keeps talking while
    // disabled would still be publishing live requests onto its lanes.
    // enabled() and teleop() already read false for a stale or detached DS.
    if (!ds_.enabled() || !ds_.teleop() || !stick.connected()) {
      Stop();
      return;
    }

    double vx = Deadband(-stick.axis(1)) * max_linear_mps_;
    double vy = Deadband(-stick.axis(0)) * max_linear_mps_;
    const double omega = Deadband(-stick.axis(2)) * max_angular_radps_;
    // Field-relative sticks need a fresh heading; without one, drive
    // robot-relative rather than on a stale or assumed yaw.
    if (field_oriented_ && have_heading_ &&
        node_.now() - heading_at_ < heading_timeout_) {
      const double yaw = yaw_rot_ * 2.0 * std::numbers::pi;
      const double robot_vx = vx * std::cos(yaw) + vy * std::sin(yaw);
      vy = -vx * std::sin(yaw) + vy * std::cos(yaw);
      vx = robot_vx;
    }
    commanding_ = true;
    chassis_.send(drivetrain::ChassisTarget{vx, vy, omega});

    // A button held asks; released, the lane has nothing to say.
    if (stick.button(shoot_button_)) {
      shooter_.send(shooter::ShooterTarget{shooter_rps_});
    } else {
      shooter_.release();
    }
    if (stick.button(intake_button_)) {
      intake_.send(intake::IntakeTarget{static_cast<float>(intake_rps_)});
    } else {
      intake_.release();
    }
  }

  talos::Node& node_;
  double max_linear_mps_;
  double max_angular_radps_;
  double deadband_;
  bool field_oriented_;
  talos::Duration heading_timeout_;
  double shooter_rps_;
  double intake_rps_;
  int shoot_button_;
  int intake_button_;
  talos::TargetLane<drivetrain::ChassisTarget> chassis_;
  talos::TargetLane<shooter::ShooterTarget> shooter_;
  talos::TargetLane<intake::IntakeTarget> intake_;
  talos::DriverStation ds_;
  double yaw_rot_ = 0;
  talos::Time heading_at_{};
  bool have_heading_ = false;
  bool commanding_ = false;
};

}  // namespace operator_interface
