#pragma once

#include <chrono>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <string>
#include <string_view>
#include <vector>

#include "drivetrain/drive_message_generated.h"
#include "drivetrain/topics.h"
#include "intake/intake_message_generated.h"
#include "intake/topics.h"
#include "shooter/shooter_message_generated.h"
#include "shooter/topics.h"
#include "talOS/configuration/config_parser.h"
#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

namespace operator_interface {

// Wire-compatible with the Target struct of any single-demand mechanism
// scaffolded by tools/new_subsystem.py (enabled:bool + 7 pad + demand:double).
struct alignas(8) GenericTarget {
  std::uint8_t enabled{0};
  std::uint8_t pad[7]{};
  double demand{0.0};
};
static_assert(sizeof(GenericTarget) == 16);
static_assert(talos::Message<GenericTarget>);

// Maps each Driver Station packet (/hw/state/driver_station, through
// talos::DriverStation) to teleop requests as it arrives: stick scaling,
// deadband and field-oriented rotation for the chassis, buttons and setpoints
// for the shooter and intake. One producer among several, not
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
        // 1-based, matching Driver Station numbering.
        shoot_button_{node.param<int>("shoot_button", 1)},
        intake_button_{node.param<int>("intake_button", 2)},
        chassis_{node.publish<drivetrain::ChassisTarget>(
            drivetrain::topics::kTeleopTarget)},
        shooter_{node.publish<shooter::ShooterTarget>(
            shooter::topics::kTeleopTarget)},
        intake_{
            node.publish<intake::IntakeTarget>(intake::topics::kTeleopTarget)},
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
    if (!node.config_path().empty()) {
      const auto cfg = talos::config::ParseRobotConfig(node.config_path());
      if (const auto* subs = cfg.toml_data["subsystems"].as_table()) {
        for (const auto& [key, val] : *subs) {
          const std::string_view sub_name = key.str();
          if (sub_name == "drivetrain" || sub_name == "shooter" ||
              sub_name == "intake") {
            continue;
          }
          const auto* tbl = val.as_table();
          if (tbl == nullptr) continue;
          const std::int64_t button =
              (*tbl)["teleop_button"].value_or(std::int64_t{0});
          if (button < 1 || button > 32) continue;
          const double demand = (*tbl)["teleop_demand"].value_or(0.0);
          const std::string topic =
              "/" + std::string{sub_name} + "/target/teleop";
          extra_bindings_.push_back(
              ExtraBinding{static_cast<int>(button), demand,
                           node.publish<GenericTarget>(topic)});
        }
      }
    }
  }

  // A Driver Station that stops talking gets the same one stop per target
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

  // One explicit stop per target on the falling edge, so each subsystem is
  // told to stop rather than left to time the last request out.
  void Stop() {
    if (!commanding_) return;
    commanding_ = false;
    chassis_.send(drivetrain::ChassisTarget{0, 0, 0, false});
    shooter_.send(shooter::ShooterTarget{0, false});
    intake_.send(intake::IntakeTarget{false, 0});
    for (auto& binding : extra_bindings_) {
      binding.publisher.send(GenericTarget{0, {}, 0.0});
    }
  }

  void OnDriver() {
    const talos::Joystick stick = ds_.joystick(0);
    // The arbiter also gates on mode, but a producer that keeps talking while
    // disabled would still be publishing live requests onto its own topics.
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
    chassis_.send(drivetrain::ChassisTarget{vx, vy, omega, true});

    const bool shoot = stick.button(shoot_button_);
    shooter_.send(shooter::ShooterTarget{shoot ? shooter_rps_ : 0.0, shoot});
    const bool take = stick.button(intake_button_);
    intake_.send(intake::IntakeTarget{
        take, take ? static_cast<float>(intake_rps_) : 0.0f});
    for (auto& binding : extra_bindings_) {
      const bool pressed = stick.button(binding.button);
      binding.publisher.send(
          GenericTarget{pressed ? std::uint8_t{1} : std::uint8_t{0},
                        {},
                        pressed ? binding.demand : 0.0});
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
  talos::Publisher<drivetrain::ChassisTarget> chassis_;
  talos::Publisher<shooter::ShooterTarget> shooter_;
  talos::Publisher<intake::IntakeTarget> intake_;
  talos::DriverStation ds_;
  struct ExtraBinding {
    int button;  // 1-based
    double demand;
    talos::Publisher<GenericTarget> publisher;
  };
  std::vector<ExtraBinding> extra_bindings_;
  double yaw_rot_ = 0;
  talos::Time heading_at_{};
  bool have_heading_ = false;
  bool commanding_ = false;
};

}  // namespace operator_interface
