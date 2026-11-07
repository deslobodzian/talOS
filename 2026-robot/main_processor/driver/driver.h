#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>
#include <optional>

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
        rezero_button_id_{node.param("rezero_button_id", 1)},
        heading_hold_radps_{node.param("heading_hold_radps", 0.0)},
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
          if (armed_ && translation_neutral_) translation_armed_ = true;
        });
  }

  // A Driver Station that stops talking gets the same one release per lane
  // as a disable, instead of the last request aging out downstream.
  void update() {
    CheckEvaluation();
    if (!ds_.connected()) Stop();
    if (field_oriented_ && !FreshHeading()) translation_armed_ = false;
  }

 private:
  // Five 20 ms periods by default, so one or two dropped packets do not flap
  // the frame. A non-positive or non-finite value keeps the default.
  static talos::Duration HeadingTimeout(const talos::Node& node) {
    const double s = node.param("heading_timeout_s", 0.1);
    return std::chrono::duration_cast<talos::Duration>(
        std::chrono::duration<double>{std::isfinite(s) && s > 0 ? s : 0.1});
  }

  bool FreshHeading() const {
    return have_heading_ && node_.now() - heading_at_ < heading_timeout_;
  }

  void CheckEvaluation() {
    const auto now = node_.now();
    if (last_evaluation_ &&
        now - *last_evaluation_ >=
            talos::detail::NodeAccess::freshness_timeout(node_)) {
      Stop();
    }
    last_evaluation_ = now;
  }

  // One release on the falling edge, so the drivetrain drops teleop at once
  // rather than timing the last request out.
  void Stop() {
    armed_ = false;
    translation_armed_ = false;
    translation_neutral_ = false;
    holding_heading_ = false;
    if (!commanding_) return;
    commanding_ = false;
    chassis_.release();
  }

  void OnDriver() {
    CheckEvaluation();
    const talos::Joystick stick = ds_.joystick(0);
    // Each Target also gates on mode, but a producer that keeps talking while
    // disabled would still be publishing live requests onto its lanes.
    // enabled() and teleop() already read false for a stale or detached DS.
    if (!ds_.enabled() || !ds_.teleop() || !stick.connected()) {
      Stop();
      return;
    }

    const bool rezero =
        rezero_button_id_ > 0 && stick.button(rezero_button_id_);
    const double raw_x = -stick.axis(1);
    const double raw_y = -stick.axis(0);
    const double raw_rotation = -stick.axis(2);
    const double magnitude = std::hypot(raw_x, raw_y);
    const bool translation_neutral =
        std::isfinite(magnitude) && magnitude <= deadband_frac_;
    const bool neutral = translation_neutral &&
                         std::isfinite(raw_rotation) &&
                         std::abs(raw_rotation) <= deadband_frac_ && !rezero;
    if (!std::isfinite(magnitude) || !std::isfinite(raw_rotation)) {
      Stop();
      return;
    }
    translation_neutral_ = translation_neutral;
    if (!armed_) {
      // Seed edge history from the snapshot; a held button is not a press.
      rezero_pressed_ = rezero;
      armed_ = neutral;
      translation_armed_ = neutral && FreshHeading();
      return;
    }
    if (rezero && !rezero_pressed_ && have_heading_) {
      yaw_offset_rad_ = yaw_rad_;
      hold_yaw_rad_ = yaw_rad_;
      holding_heading_ = true;
    }
    rezero_pressed_ = rezero;
    const bool fresh_heading = FreshHeading();
    if (!fresh_heading) translation_armed_ = false;
    if (fresh_heading && translation_neutral) translation_armed_ = true;
    const double linear_scale =
        magnitude > 0 ? talos::teleop::Deadband(std::min(magnitude, 1.0),
                                                deadband_frac_) *
                            max_linear_mps_ / magnitude
                      : 0.0;
    double vx = raw_x * linear_scale;
    double vy = raw_y * linear_scale;
    double omega = talos::teleop::Deadband(raw_rotation, deadband_frac_) *
                   max_angular_radps_;
    if (fresh_heading) {
      if (omega != 0 || !holding_heading_) hold_yaw_rad_ = yaw_rad_;
      if (omega == 0 && holding_heading_) {
        const double error =
            std::remainder(hold_yaw_rad_ - yaw_rad_, 2 * std::numbers::pi);
        omega = std::clamp(heading_hold_radps_ * error, -max_angular_radps_,
                           max_angular_radps_);
      }
      holding_heading_ = true;
    } else {
      holding_heading_ = false;
    }
    // Never reinterpret a field-relative request in a different frame.
    // Recovery requires a fresh heading and a centred translation stick.
    if (field_oriented_ && !translation_armed_) {
      vx = 0;
      vy = 0;
    } else if (field_oriented_) {
      if (ds_.alliance() == talos::driver_station::Alliance::kRed) {
        vx = -vx;
        vy = -vy;
      }
      const double yaw = yaw_rad_ - yaw_offset_rad_;
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
  int rezero_button_id_;
  double heading_hold_radps_;
  talos::TargetLane<drivetrain::ChassisTarget> chassis_;
  talos::DriverStation ds_;
  double yaw_rad_ = 0;
  double yaw_offset_rad_ = 0;
  double hold_yaw_rad_ = 0;
  bool holding_heading_ = false;
  bool rezero_pressed_ = false;
  talos::Time heading_at_{};
  bool have_heading_ = false;
  bool commanding_ = false;
  bool armed_ = false;
  bool translation_armed_ = false;
  bool translation_neutral_ = false;
  std::optional<talos::Time> last_evaluation_;
};

}  // namespace driver
