#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>

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
      : node_{node},
        shooter_velocity_{node.param("shooter_velocity_rps", 60.0)},
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
        spinup_timeout_s_{PositiveSeconds(node, "spinup_timeout_s", 2.0)},
        drive_timeout_s_{PositiveSeconds(node, "drive_timeout_s", 10.0)},
        pose_{node.subscribe<geometry::PoseEstimate>("/pose_estimator/state")},
        shooter_state_{
            node.subscribe<shooter::ShooterState>(shooter::topics::kState)},
        intake_state_{
            node.subscribe<intake::IntakeState>(intake::topics::kState)},
        drive_{node, "drivetrain", "auto"},
        shooter_{node, "shooter", "auto"},
        intake_{node, "intake", "auto"},
        ds_{node},
        phase_{node, Phase::kInhibited},
        state_{node.publish<AutonomousState>(topics::kState)} {
    if (feed_duration_ <= talos::Duration::zero()) {
      throw std::invalid_argument(
          "autonomous: feed_duration_us must be positive");
    }
  }

  void update() {
    const auto now = node_.now();
    if (feeding_) feed_elapsed_ += now - feed_last_update_;
    feeding_ = false;
    // Silence is not a mode transition: preserve progress and wall deadlines,
    // but never keep commanding lanes or advance feed/trajectory time.
    if (!ds_.connected()) {
      trajectory_pose_available_ = false;
      CheckDeadline(now);
      ReleaseTargets();
      state_.send(AutonomousState{WirePhase()});
      return;
    }
    if (!ds_.enabled() || !ds_.autonomous()) {
      ResetRoutine();
      ReleaseTargets();
      state_.send(AutonomousState{WirePhase()});
      return;
    }
    // A new process must observe disabled/non-auto before it can arm. Merely
    // receiving an already-running auto generation must never fire a piece.
    if (phase_.is(Phase::kInhibited)) {
      ReleaseTargets();
      state_.send(AutonomousState{WirePhase()});
      return;
    }
    if (generation_ != ds_.data().mode_since_us) ResetRoutine();
    if (phase_.is(Phase::kIdle)) {
      generation_ = ds_.data().mode_since_us;
      red_ = ds_.alliance() == talos::driver_station::Alliance::kRed;
      spinup_started_ = now;
      phase_.go(Phase::kSpinup);
    }
    CheckDeadline(now);
    if (phase_.is(Phase::kAborted) || phase_.is(Phase::kHold)) {
      ReleaseTargets();
      state_.send(AutonomousState{WirePhase()});
      return;
    }
    released_ = false;
    const bool shooter_ready =
        shooter_state_.fresh() && shooter_state_->at_speed() &&
        std::abs(shooter_state_->target_velocity_rps() - shooter_velocity_) <=
            1e-6;
    if (phase_.is(Phase::kFeed) && !shooter_ready) {
      phase_.go(Phase::kSpinup);
    }
    if (phase_.is(Phase::kSpinup) && shooter_ready) {
      phase_.go(Phase::kFeed);
    }
    if (phase_.is(Phase::kFeed) && feed_elapsed_ >= feed_duration_) {
      phase_.go(Phase::kDrive);
    }
    drivetrain::ChassisTarget drive{};
    double intake = 0;
    double shooter = shooter_velocity_;
    if (phase_.is(Phase::kFeed)) {
      intake = feed_velocity_;
      feeding_ = true;
      feed_last_update_ = now;
    }
    if (phase_.is(Phase::kDrive)) {
      shooter = 0;
      if (pose_.fresh() && pose_->valid() && pose_->localized()) {
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
          trajectory_elapsed_ = talos::Duration::zero();
          trajectory_last_update_ = phase_.time_in_state();
          trajectory_pose_available_ = false;
        }
        if (trajectory_pose_available_) {
          trajectory_elapsed_ +=
              phase_.time_in_state() - trajectory_last_update_;
        }
        trajectory_last_update_ = phase_.time_in_state();
        trajectory_pose_available_ = true;
        const double elapsed =
            std::chrono::duration<double>(trajectory_elapsed_).count();
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
      } else {
        trajectory_pose_available_ = false;
      }
    }
    if (phase_.is(Phase::kHold)) {
      ReleaseTargets();
      state_.send(AutonomousState{WirePhase()});
      return;
    }
    if (phase_.is(Phase::kDrive) && !trajectory_pose_available_) {
      drive_.release();
    } else {
      drive_.send(drive);
    }
    shooter_.send(shooter::ShooterTarget{shooter});
    intake_.send(intake::IntakeTarget{static_cast<float>(intake)});
    state_.send(AutonomousState{WirePhase()});
  }

 private:
  enum class Phase { kIdle, kSpinup, kFeed, kDrive, kHold, kAborted, kInhibited };
  talos::Node& node_;
  double shooter_velocity_, intake_velocity_, feed_velocity_;
  double target_x_, target_y_, target_yaw_;
  double position_tolerance_, yaw_tolerance_, max_linear_, max_angular_;
  double field_length_, field_width_, kp_xy_, kp_yaw_;
  Trajectory trajectory_;
  talos::Duration trajectory_elapsed_{};
  talos::Duration trajectory_last_update_{};
  bool trajectory_pose_available_ = false;
  bool trajectory_started_ = false;
  talos::Duration feed_duration_;
  double spinup_timeout_s_, drive_timeout_s_;
  std::uint64_t generation_ = 0;
  talos::Time spinup_started_{};
  talos::Duration feed_elapsed_{};
  talos::Time feed_last_update_{};
  bool feeding_ = false;
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

  static double PositiveSeconds(talos::Node& node, const char* key,
                                double fallback) {
    const double value = node.param(key, fallback);
    if (!std::isfinite(value) || value <= 0) {
      throw std::invalid_argument(std::string{"autonomous: "} + key +
                                  " must be positive and finite");
    }
    return value;
  }

  void ResetRoutine() {
    phase_.go(Phase::kIdle);
    generation_ = 0;
    trajectory_started_ = false;
    trajectory_pose_available_ = false;
    feeding_ = false;
    feed_elapsed_ = talos::Duration::zero();
  }

  void CheckDeadline(talos::Time now) {
    // One wall-clock preload budget from the FIRST spinup entry, including
    // feed and readiness retries. Flapping readiness or a DS outage cannot
    // postpone the deadline. Drive likewise includes every outage.
    if (((phase_.is(Phase::kSpinup) || phase_.is(Phase::kFeed)) &&
         now - spinup_started_ >=
             std::chrono::duration<double>{spinup_timeout_s_}) ||
        (phase_.is(Phase::kDrive) &&
         phase_.time_in_state() >=
             std::chrono::duration<double>{drive_timeout_s_})) {
      phase_.go(Phase::kAborted);
    }
  }

  void ReleaseTargets() {
    if (released_) return;
    drive_.release();
    shooter_.release();
    intake_.release();
    released_ = true;
  }

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
      case Phase::kAborted:
        return autonomous::Phase::ABORTED;
      case Phase::kInhibited:
        return autonomous::Phase::INHIBITED;
      case Phase::kIdle:
        return autonomous::Phase::IDLE;
    }
    return autonomous::Phase::IDLE;
  }
};

}  // namespace autonomous
