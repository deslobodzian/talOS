#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <numbers>

#include "driver_station/driver_station_message_generated.h"
#include "driver_station/topics.h"
#include "drivetrain/drive_message_generated.h"
#include "drivetrain/topics.h"
#include "intake/intake_message_generated.h"
#include "intake/topics.h"
#include "shooter/shooter_message_generated.h"
#include "shooter/topics.h"
#include "talOS/driver_station/driver_station.h"
#include "talOS/node/node.h"

namespace operator_interface {

// Maps each Driver Station state to teleop requests as it arrives: stick
// scaling, deadband and field-oriented rotation for the chassis, button masks
// and setpoints for the shooter and intake. One producer among several, not
// the authority: the arbiter decides whether teleop is driving and is the
// sole writer of the topics the subsystems consume.
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
        // Bit 0 is button 1, matching Driver Station numbering.
        shoot_mask_{node.param<std::uint32_t>("shoot_button_mask", 1u << 0)},
        intake_mask_{node.param<std::uint32_t>("intake_button_mask", 1u << 1)},
        chassis_{node.publish<drivetrain::ChassisTarget>(
            drivetrain::kTeleopTargetTopic)},
        shooter_{node.publish<shooter::ShooterTarget>(
            shooter::kTeleopTargetTopic)},
        intake_{node.publish<intake::IntakeTarget>(intake::kTeleopTargetTopic)} {
    node.subscribe<driver_station::DriverStationState>(
        driver_station::kStateTopic,
        [this](const driver_station::DriverStationState& s) { OnDriver(s); });
    // The drivetrain owns the yaw. A non-finite one carries no direction, so
    // it is dropped and the previous heading keeps aging toward stale.
    node.subscribe<drivetrain::DrivetrainState>(
        drivetrain::kStateTopic, [this](const drivetrain::DrivetrainState& s) {
          if (!std::isfinite(s.yaw_rot())) return;
          yaw_rot_ = s.yaw_rot();
          heading_at_ = node_.now();
          have_heading_ = true;
        });
  }

 private:
  using Flags = talos::driver_station::DsFlags;

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

  void OnDriver(const driver_station::DriverStationState& state) {
    const std::int64_t now = node_.now().nanos();
    const std::uint32_t flags = state.fms().flags();
    const auto& stick = state.stick0();
    // The arbiter also gates on mode, but a producer that keeps talking while
    // disabled would still be publishing live requests onto its own topics.
    if (!(flags & Flags::kEnabled) || !(flags & Flags::kTeleop) ||
        !stick.connected()) {
      // One explicit stop per target on the falling edge, so each subsystem
      // is told to stop rather than left to time the last request out.
      if (commanding_) {
        commanding_ = false;
        chassis_.send(drivetrain::ChassisTarget{0, 0, 0, now, false});
        shooter_.send(shooter::ShooterTarget{0, now, false});
        intake_.send(
            intake::IntakeTarget{static_cast<std::uint64_t>(now), false, 0});
      }
      return;
    }

    double vx = Deadband(-stick.axis1()) * max_linear_mps_;
    double vy = Deadband(-stick.axis0()) * max_linear_mps_;
    const double omega = Deadband(-stick.axis2()) * max_angular_radps_;
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
    chassis_.send(drivetrain::ChassisTarget{vx, vy, omega, now, true});

    const bool shoot = (stick.buttons() & shoot_mask_) != 0;
    shooter_.send(shooter::ShooterTarget{shoot ? shooter_rps_ : 0.0, now, shoot});
    const bool take = (stick.buttons() & intake_mask_) != 0;
    intake_.send(intake::IntakeTarget{
        static_cast<std::uint64_t>(now), take,
        take ? static_cast<float>(intake_rps_) : 0.0f});
  }

  talos::Node& node_;
  double max_linear_mps_;
  double max_angular_radps_;
  double deadband_;
  bool field_oriented_;
  talos::Duration heading_timeout_;
  double shooter_rps_;
  double intake_rps_;
  std::uint32_t shoot_mask_;
  std::uint32_t intake_mask_;
  talos::Publisher<drivetrain::ChassisTarget> chassis_;
  talos::Publisher<shooter::ShooterTarget> shooter_;
  talos::Publisher<intake::IntakeTarget> intake_;
  double yaw_rot_ = 0;
  talos::Time heading_at_{};
  bool have_heading_ = false;
  bool commanding_ = false;
};

}  // namespace operator_interface
