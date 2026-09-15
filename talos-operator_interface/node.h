#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <numbers>

#include "talos_nodes/driver_station/driver_station_message_generated.h"
#include "talos_nodes/driver_station/packet.h"
#include "talos_nodes/drivetrain/drive_message_generated.h"
#include "talos_nodes/drivetrain/packet.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/events/handles.h"
#include "talOS/process/node_main.h"

// The operator interface core: stick mapping, scaling, deadband, and
// field-oriented rotation for the chassis. It consumes the decoded Driver
// Station state and produces the teleop chassis request. It is one producer
// among several, not the authority: the arbiter decides whether teleop is the
// mode currently driving and is the sole writer of the topics the subsystems
// consume.
//
// This core is game-agnostic and reusable across seasons. Year-specific
// mechanism bindings (shooter/intake buttons and setpoints) live in the year
// wrapper (2026-robot/main_processor/operator_interface/), which composes
// this core for the chassis and adds its own senders. Keeping them out of the
// driver_station node means the wire format has one reader that never changes
// with game logic.
namespace talos::oi {

struct OperatorInterfaceConfig {
  double max_linear_mps{4.0};
  double max_angular_radps{6.0};
  double deadband{0.05};
  // Field-oriented driving needs a heading, so it stays off until the
  // drivetrain has published one; see OnDrivetrainState.
  bool field_oriented{true};
  // A latched heading older than this is treated as absent: the drivetrain
  // may have stopped publishing, and rotating by a stale yaw would send the
  // robot somewhere the driver did not ask. Five periods at the 20 ms
  // subsystem rate, so one or two dropped packets do not flap the frame.
  event::Duration heading_timeout{std::chrono::milliseconds(100)};
};

template <typename Loop>
class OperatorInterfaceNode {
 public:
  explicit OperatorInterfaceNode(Loop& loop,
                                 OperatorInterfaceConfig config = {})
      : config_{config},
        chassis_target_{event::make_sender<talos::drive::ChassisTarget>(
            loop, talos::drive::kTeleopTargetTopic)} {
    event::watch<driver_station::DriverStationState,
                 &OperatorInterfaceNode::OnDriverStation>(
        loop, driver_station::kDsStateTopic, this);
    event::watch<talos::drive::DrivetrainState,
                 &OperatorInterfaceNode::OnDrivetrainState>(
        loop, talos::drive::kDrivetrainStateTopic, this);
  }

  void Start(event::MonotonicTime = {}) {}

  // Field-oriented control is live only with a valid heading fresher than
  // heading_timeout. Anything else — never received, non-finite, or stale —
  // falls back to robot-relative, never to driving on an assumed yaw of zero.
  bool field_oriented_active(event::MonotonicTime now) const {
    return config_.field_oriented && have_heading_ &&
           (now - last_heading_time_) < config_.heading_timeout;
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
  // node consumes it rather than reading the gyro a second time. A
  // non-finite yaw carries no direction, so it is dropped: the previous
  // heading (if any) keeps aging toward stale rather than being poisoned.
  void OnDrivetrainState(const event::Context& ctx,
                         const talos::drive::DrivetrainState& state) {
    if (!std::isfinite(state.yaw_rot())) return;
    yaw_rot_ = state.yaw_rot();
    last_heading_time_ = ctx.now;
    have_heading_ = true;
  }

  // Rotates a field-relative request into the robot frame. Field-oriented is
  // skipped entirely without a fresh heading, so a missing or stalled
  // drivetrain publisher degrades to robot-relative rather than to driving
  // on a stale yaw -- which would look correct only while the robot happens
  // to still face the latched direction.
  void ToRobotFrame(double& vx, double& vy, event::MonotonicTime now) const {
    if (!field_oriented_active(now)) return;
    const double yaw_rad = yaw_rot_ * 2.0 * std::numbers::pi;
    const double cos_yaw = std::cos(yaw_rad), sin_yaw = std::sin(yaw_rad);
    const double robot_vx = vx * cos_yaw + vy * sin_yaw;
    const double robot_vy = -vx * sin_yaw + vy * cos_yaw;
    vx = robot_vx;
    vy = robot_vy;
  }

  // One explicit disabled chassis request on the falling edge, so the
  // drivetrain is told to stop rather than left to time the last request out.
  // Year mechanism targets are released by the year wrapper's own edge.
  void Release(const event::Context& ctx) {
    if (!commanding_) return;
    commanding_ = false;
    chassis_target_.send(
        talos::drive::ChassisTarget{0.0, 0.0, 0.0, ctx.now.nanos(), false});
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
    ToRobotFrame(vx, vy, ctx.now);

    commanding_ = true;
    chassis_target_.send(
        talos::drive::ChassisTarget{vx, vy, omega, ctx.now.nanos(), true});
  }

  OperatorInterfaceConfig config_;
  event::Sender<Loop, talos::drive::ChassisTarget> chassis_target_;

  double yaw_rot_{0.0};
  bool have_heading_{false};
  event::MonotonicTime last_heading_time_{};
  bool commanding_{false};
};

// Factory for talos_cc_node: chassis tuning comes from the
// operator_interface subsystem table, with compiled defaults as fallback.
// Year mechanism keys (shooter/intake masks and setpoints) are owned by the
// year wrapper and intentionally not read here.
inline auto CreateNode(auto& loop, const talos::process::NodeFlags& flags) {
  const auto robot_config = talos::config::ParseRobotConfig(flags.config_path);
  OperatorInterfaceConfig oi{};
  if (const auto* tbl = robot_config.GetSubsystemTable("operator_interface")) {
    oi.max_linear_mps = (*tbl)["max_linear_mps"].value_or(oi.max_linear_mps);
    oi.max_angular_radps =
        (*tbl)["max_angular_radps"].value_or(oi.max_angular_radps);
    oi.deadband = (*tbl)["deadband"].value_or(oi.deadband);
    oi.field_oriented = (*tbl)["field_oriented"].value_or(oi.field_oriented);
    if (const auto timeout_s = (*tbl)["heading_timeout_s"].value<double>()) {
      if (std::isfinite(*timeout_s) && *timeout_s > 0.0) {
        oi.heading_timeout = std::chrono::duration_cast<event::Duration>(
            std::chrono::duration<double>(*timeout_s));
      }
    }
  }
  return OperatorInterfaceNode{loop, oi};
}

}  // namespace talos::oi
