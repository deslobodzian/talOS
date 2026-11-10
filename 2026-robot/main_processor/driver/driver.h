#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
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
          measured_omega_radps_ = s.omega_radps();
          heading_at_ = node_.now();
          have_heading_ = true;
          if (armed_ && translation_neutral_) SetFrame(Frame::kField);
        });
  }

  // A Driver Station that stops talking gets the same one release per lane
  // as a disable, instead of the last request aging out downstream.
  void update() {
    CheckEvaluation();
    if (!ds_.connected()) Stop();
    if (field_oriented_ && !FreshHeading() && frame_ == Frame::kField)
      SetFrame(Frame::kWaitingForNeutral);
  }

 private:
  enum class Frame { kWaitingForNeutral, kRobot, kField };

  struct ModeGeneration {
    std::uint64_t boot_id = 0;
    std::uint64_t mode_since_us = 0;
    bool operator==(const ModeGeneration&) const = default;
  };

  // Measured rotation below which a turn has stopped and the heading hold
  // latches: the slew-limited coast left from there is negligible.
  static constexpr double kTurnStoppedRadps = 0.1;

  void SetFrame(Frame next) {
    if (!field_oriented_) next = Frame::kRobot;
    if (frame_ == next) return;
    frame_ = next;
    // No driver state topic: report frame transitions without a new wire type.
    std::fprintf(stderr, "driver: translation frame %s\n",
                 next == Frame::kField   ? "driver-relative (gyro)"
                 : next == Frame::kRobot ? "robot-relative"
                                         : "waiting for neutral");
  }

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
    SetFrame(Frame::kWaitingForNeutral);
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
    // A packet lost between two others can be the only disabled one; its
    // mode generation (per gateway boot) has moved on regardless, so arming
    // holds only within the generation it was armed in.
    const ModeGeneration generation{ds_.data().boot_id,
                                    ds_.data().mode_since_us};
    if (armed_ && generation != armed_generation_) Stop();

    const bool rezero =
        rezero_button_id_ > 0 && stick.button(rezero_button_id_);
    const double raw_x = -stick.axis(1);
    const double raw_y = -stick.axis(0);
    const double raw_rotation = -stick.axis(2);
    const double magnitude = std::hypot(raw_x, raw_y);
    const bool translation_neutral =
        std::isfinite(magnitude) && magnitude <= deadband_frac_;
    const bool neutral = translation_neutral && std::isfinite(raw_rotation) &&
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
      armed_generation_ = generation;
      if (neutral) SetFrame(FreshHeading() ? Frame::kField : Frame::kRobot);
      return;
    }
    if (rezero && !rezero_pressed_ && FreshHeading()) {
      yaw_offset_rad_ = yaw_rad_;
      hold_settling_ = true;
    }
    rezero_pressed_ = rezero;
    const bool fresh_heading = FreshHeading();
    if (!fresh_heading && frame_ == Frame::kField)
      SetFrame(Frame::kWaitingForNeutral);
    if (translation_neutral)
      SetFrame(fresh_heading ? Frame::kField : Frame::kRobot);
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
      // The hold follows the heading while the driver turns and, after a
      // release, a re-zero or a stale heading, until the drivetrain measures
      // that the robot has stopped turning: its slew limit carries the robot
      // on after a release, and that coast is where the driver turned to, not
      // an error to drive back. A non-finite measurement reads as stopped.
      // Once latched, the hold's own correction never moves its target.
      if (omega != 0 || !holding_heading_) hold_settling_ = true;
      if (hold_settling_) {
        hold_yaw_rad_ = yaw_rad_;
        hold_settling_ =
            omega != 0 || std::abs(measured_omega_radps_) >= kTurnStoppedRadps;
      } else {
        const double error =
            std::remainder(hold_yaw_rad_ - yaw_rad_, 2 * std::numbers::pi);
        omega = std::clamp(heading_hold_radps_ * error, -max_angular_radps_,
                           max_angular_radps_);
      }
      holding_heading_ = true;
    } else {
      holding_heading_ = false;
    }
    // Frame changes require neutral translation: never reinterpret a held
    // stick. Rotation remains available while translation waits to re-arm.
    if (field_oriented_ && frame_ == Frame::kWaitingForNeutral) {
      vx = 0;
      vy = 0;
    } else if (field_oriented_ && frame_ == Frame::kField) {
      // Gyro yaw is boot/rezero-relative, not field-absolute. Both alliances
      // use the latched driver-forward direction without an alliance flip.
      // The drivetrain's heading re-anchors a restarted Pigeon count, so a
      // gyro reset does not move the frame; no talos::Gyro zero is involved.
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
  double measured_omega_radps_ = 0;
  double yaw_offset_rad_ = 0;
  double hold_yaw_rad_ = 0;
  bool holding_heading_ = false;
  bool hold_settling_ = false;
  bool rezero_pressed_ = false;
  talos::Time heading_at_{};
  bool have_heading_ = false;
  bool commanding_ = false;
  bool armed_ = false;
  ModeGeneration armed_generation_;
  Frame frame_ = Frame::kWaitingForNeutral;
  bool translation_neutral_ = false;
  std::optional<talos::Time> last_evaluation_;
};

}  // namespace driver
