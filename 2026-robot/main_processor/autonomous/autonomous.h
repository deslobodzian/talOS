#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>

#include "autonomous/autonomous_message_generated.h"
#include "autonomous/topics.h"
#include "autonomous/trajectory.h"
#include "drivetrain/drive_message_generated.h"
#include "geometry/geometry_generated.h"
#include "intake/intake_message_generated.h"
#include "intake/topics.h"
#include "shooter/shooter_message_generated.h"
#include "shooter/topics.h"
#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

namespace autonomous {

// Shared-memory slot contract for /autonomous/state: docs/DATA_FLOW.md.
static_assert(sizeof(AutonomousState) == 1,
              "AutonomousState size changed; update docs/DATA_FLOW.md");

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
        field_length_{node.param("field_length_m", 16.54)},
        field_width_{node.param("field_width_m", 8.21)},
        kp_xy_{node.param("xy_feedback_per_m_mps", 1.0)},
        kp_yaw_{node.param("yaw_feedback_per_rad_radps", 1.0)},
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
        phase_{node, Phase::kIdle},
        state_{node.publish<AutonomousState>(topics::kState)} {}

  void update() {
    if (!ds_.enabled() || !ds_.autonomous()) {
      phase_.go(Phase::kIdle);
      trajectory_started_ = false;
      if (!released_) {
        drive_.release();
        shooter_.release();
        intake_.release();
        released_ = true;
      }
      state_.send(AutonomousState{WirePhase()});
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
        if (!trajectory_started_) {
          auto target = TrajectorySample{0, target_x_, target_y_, target_yaw_};
          if (red_) {
            target.x_m = field_length_ - target.x_m;
            target.y_m = field_width_ - target.y_m;
            target.yaw_rad = WrapAngle(target.yaw_rad + std::numbers::pi);
          }
          trajectory_ = Trajectory::FromPoses(
              p, geometry::Pose2d{target.x_m, target.y_m, target.yaw_rad},
              max_linear_, max_angular_);
          trajectory_started_ = true;
          trajectory_start_ = phase_.time_in_state();
        }
        const double elapsed = std::chrono::duration<double>(
                                   phase_.time_in_state() - trajectory_start_)
                                   .count();
        const auto desired = trajectory_.SampleAt(elapsed);
        const double dx = desired.x_m - p.x_m(), dy = desired.y_m - p.y_m();
        const double da = WrapAngle(desired.yaw_rad - p.yaw_rad());
        if (elapsed >= trajectory_.total_time_s() &&
            std::hypot(dx, dy) <= position_tolerance_ &&
            std::abs(da) <= yaw_tolerance_) {
          phase_.go(Phase::kHold);
        } else {
          drive =
              HolonomicController{kp_xy_, kp_yaw_, max_linear_, max_angular_}
                  .Calculate(desired, p);
          if (intake_state_.fresh() && !intake_state_->beam_broken()) {
            intake = intake_velocity_;
          }
        }
      }
    }
    if (phase_.is(Phase::kHold)) shooter = 0;
    drive_.send(drive);
    shooter_.send(shooter::ShooterTarget{shooter});
    intake_.send(intake::IntakeTarget{static_cast<float>(intake)});
    state_.send(AutonomousState{WirePhase()});
  }

 private:
  enum class Phase { kIdle, kSpinup, kFeed, kDrive, kHold };
  double shooter_velocity_, intake_velocity_, feed_velocity_;
  double target_x_, target_y_, target_yaw_;
  double position_tolerance_, yaw_tolerance_, max_linear_, max_angular_;
  double field_length_, field_width_, kp_xy_, kp_yaw_;
  Trajectory trajectory_;
  talos::Duration trajectory_start_{};
  bool trajectory_started_ = false;
  talos::Duration feed_duration_;
  talos::Subscription<geometry::PoseEstimate> pose_;
  talos::Subscription<shooter::ShooterState> shooter_state_;
  talos::Subscription<intake::IntakeState> intake_state_;
  talos::TargetLane<drivetrain::ChassisTarget> drive_;
  talos::TargetLane<shooter::ShooterTarget> shooter_;
  talos::TargetLane<intake::IntakeTarget> intake_;
  talos::DriverStation ds_;
  talos::StateMachine<Phase> phase_;
  talos::Publisher<AutonomousState> state_;
  bool released_ = false;
  bool red_ = false;

  autonomous::Phase WirePhase() const {
    switch (phase_.state()) {
      case Phase::kSpinup:
        return autonomous::Phase::SPINUP;
      case Phase::kFeed:
        return autonomous::Phase::FEED;
      case Phase::kDrive:
        return autonomous::Phase::DRIVE;
      case Phase::kHold:
        return autonomous::Phase::HOLD;
      case Phase::kIdle:
        return autonomous::Phase::IDLE;
    }
    return autonomous::Phase::IDLE;
  }
};

}  // namespace autonomous
