#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>

#include "drivetrain/drive_message_generated.h"
#include "geometry/geometry_generated.h"
#include "intake/intake_message_generated.h"
#include "intake/topics.h"
#include "shooter/shooter_message_generated.h"
#include "shooter/topics.h"
#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

namespace autonomous {

class Autonomous {
 public:
  explicit Autonomous(talos::Node& node)
      : shooter_velocity_{node.param("shooter_velocity_rps", 60.0)},
        intake_velocity_{node.param("intake_velocity_rps", 30.0)},
        feed_velocity_{node.param("feed_velocity_rps", -20.0)},
        target_x_{node.param("target_x_m", 2.0)},
        target_y_{node.param("target_y_m", 0.0)},
        target_yaw_{node.param("target_yaw_rad", 0.0)},
        position_tolerance_{node.param("position_tolerance_m", 0.10)},
        yaw_tolerance_{node.param("yaw_tolerance_rad", 0.05)},
        max_linear_{node.param("max_linear_mps", 2.0)},
        max_angular_{node.param("max_angular_radps", 3.0)},
        feed_duration_{std::chrono::microseconds{
            node.param<int>("feed_duration_us", 500000)}},
        pose_{node.subscribe<geometry::PoseEstimate>("/pose_estimator/state")},
        shooter_state_{
            node.subscribe<shooter::ShooterState>(shooter::topics::kState)},
        intake_state_{
            node.subscribe<intake::IntakeState>(intake::topics::kState)},
        drive_{node, "drivetrain", "auto"},
        shooter_{node, "shooter", "auto"},
        intake_{node, "intake", "auto"},
        ds_{node},
        phase_{node, Phase::kIdle} {}

  void update() {
    if (!ds_.enabled() || !ds_.autonomous()) {
      phase_.go(Phase::kIdle);
      if (!released_) {
        drive_.release();
        shooter_.release();
        intake_.release();
        released_ = true;
      }
      return;
    }
    released_ = false;
    if (phase_.is(Phase::kIdle)) {
      red_ = ds_.alliance() == talos::driver_station::Alliance::kRed;
      phase_.go(Phase::kSpinup);
    }
    if (phase_.is(Phase::kSpinup) && shooter_state_.fresh() &&
        shooter_state_->at_speed()) {
      phase_.go(Phase::kFeed);
    }
    if (phase_.is(Phase::kFeed) && phase_.time_in_state() >= feed_duration_) {
      phase_.go(Phase::kDrive);
    }
    drivetrain::ChassisTarget drive{};
    double intake = 0;
    double shooter = shooter_velocity_;
    if (phase_.is(Phase::kFeed)) intake = feed_velocity_;
    if (phase_.is(Phase::kDrive)) {
      shooter = 0;
      if (pose_.fresh() && pose_->valid()) {
        const auto& p = pose_->pose();
        // Red uses the same waypoint rotated pi about the field origin.
        const double x = red_ ? -target_x_ : target_x_;
        const double y = red_ ? -target_y_ : target_y_;
        const double yaw = target_yaw_ + (red_ ? std::numbers::pi : 0.0);
        const double dx = x - p.x_m(), dy = y - p.y_m();
        const double da =
            std::remainder(yaw - p.yaw_rad(), 2 * std::numbers::pi);
        if (std::isfinite(dx) && std::isfinite(dy) && std::isfinite(da)) {
          const double distance = std::hypot(dx, dy);
          if (distance <= position_tolerance_ &&
              std::abs(da) <= yaw_tolerance_) {
            phase_.go(Phase::kHold);
          } else {
            // Unit-gain position/heading feedback, with a vector speed limit.
            const double scale =
                distance > max_linear_ ? max_linear_ / distance : 1.0;
            const double vx = dx * scale, vy = dy * scale;
            const double c = std::cos(p.yaw_rad()), s = std::sin(p.yaw_rad());
            drive = drivetrain::ChassisTarget{
                vx * c + vy * s, -vx * s + vy * c,
                std::clamp(da, -max_angular_, max_angular_)};
            if (intake_state_.fresh() && !intake_state_->beam_broken()) {
              intake = intake_velocity_;
            }
          }
        }
      }
    }
    if (phase_.is(Phase::kHold)) shooter = 0;
    drive_.send(drive);
    shooter_.send(shooter::ShooterTarget{shooter});
    intake_.send(intake::IntakeTarget{static_cast<float>(intake)});
  }

 private:
  enum class Phase { kIdle, kSpinup, kFeed, kDrive, kHold };
  double shooter_velocity_, intake_velocity_, feed_velocity_;
  double target_x_, target_y_, target_yaw_;
  double position_tolerance_, yaw_tolerance_, max_linear_, max_angular_;
  talos::Duration feed_duration_;
  talos::Subscription<geometry::PoseEstimate> pose_;
  talos::Subscription<shooter::ShooterState> shooter_state_;
  talos::Subscription<intake::IntakeState> intake_state_;
  talos::TargetLane<drivetrain::ChassisTarget> drive_;
  talos::TargetLane<shooter::ShooterTarget> shooter_;
  talos::TargetLane<intake::IntakeTarget> intake_;
  talos::DriverStation ds_;
  talos::StateMachine<Phase> phase_;
  bool released_ = false;
  bool red_ = false;
};

}  // namespace autonomous
