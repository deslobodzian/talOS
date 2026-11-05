#pragma once

#include <chrono>
#include <cmath>

#include "drivetrain/drive_message_generated.h"
#include "drivetrain/topics.h"
#include "talOS/devices/devices.h"
#include "talOS/node/node.h"
#include "talOS/teleop/deadband.h"

namespace driver {

// Maps each Driver Station packet (/hw/state/driver_station, through
// talos::DriverStation) to teleop requests as it arrives: stick scaling,
// deadband and field-oriented rotation for the chassis. It writes only the
// drivetrain teleop lane (/drivetrain/target/teleop); the drivetrain's
// talos::Target decides from the match mode whether that lane is the one it
// follows.
class Driver {
 public:
  explicit Driver(talos::Node& node)
      : node_{node},
        max_linear_mps_{node.param("max_linear_mps", 4.0)},
        max_angular_radps_{node.param("max_angular_radps", 6.0)},
        deadband_frac_{node.param("deadband_frac", 0.05)},
        field_oriented_{node.param("field_oriented", true)},
        heading_timeout_{HeadingTimeout(node)},
        chassis_{node, "drivetrain", "teleop"},
        ds_{node, [this] { OnDriver(); }} {
    // The drivetrain owns the heading. One it marks invalid (no gyro) or a
    // non-finite one carries no direction, so it is dropped and the previous
    // heading keeps aging toward stale.
    node.subscribe<drivetrain::DrivetrainState>(
        drivetrain::topics::kState,
        [this](const drivetrain::DrivetrainState& s) {
          if (!s.yaw_valid() || !std::isfinite(s.yaw_rad())) return;
          yaw_rad_ = s.yaw_rad();
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

  // One release on the falling edge, so the drivetrain drops teleop at once
  // rather than timing the last request out.
  void Stop() {
    if (!commanding_) return;
    commanding_ = false;
    chassis_.release();
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

    double vx = talos::teleop::Deadband(-stick.axis(1), deadband_frac_) *
                max_linear_mps_;
    double vy = talos::teleop::Deadband(-stick.axis(0), deadband_frac_) *
                max_linear_mps_;
    const double omega =
        talos::teleop::Deadband(-stick.axis(2), deadband_frac_) *
        max_angular_radps_;
    // Field-relative sticks need a fresh heading; without one, drive
    // robot-relative rather than on a stale or assumed yaw.
    if (field_oriented_ && have_heading_ &&
        node_.now() - heading_at_ < heading_timeout_) {
      const double yaw = yaw_rad_;
      const double robot_vx = vx * std::cos(yaw) + vy * std::sin(yaw);
      vy = -vx * std::sin(yaw) + vy * std::cos(yaw);
      vx = robot_vx;
    }
    commanding_ = true;
    chassis_.send(drivetrain::ChassisTarget{vx, vy, omega});
  }

  talos::Node& node_;
  double max_linear_mps_;
  double max_angular_radps_;
  double deadband_frac_;
  bool field_oriented_;
  talos::Duration heading_timeout_;
  talos::TargetLane<drivetrain::ChassisTarget> chassis_;
  talos::DriverStation ds_;
  double yaw_rad_ = 0;
  talos::Time heading_at_{};
  bool have_heading_ = false;
  bool commanding_ = false;
};

}  // namespace driver
