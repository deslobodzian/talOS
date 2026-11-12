#include "talOS/mechanisms/velocity_mechanism.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <string>

namespace talos::mechanisms {

VelocityMechanism::VelocityMechanism(Node& node, std::string_view name)
    : node_{&node},
      link_{&node.extension<HardwareLink>()},
      label_{"VelocityMechanism '" + std::string{name} + "'"},
      motor_{node, name},
      velocity_age_bound_{RezeroAgeBound(link_->config().hardware)},
      max_rps_{MotorConfigOf(node, name).max_velocity_rps} {
  const Params params{node, name};
  entry_rps_ = params.positive("at_speed_entry_rps", 2.0);
  exit_frac_ = params.positive("at_speed_exit_frac", 0.1);
  if (exit_frac_ > 1) params.fail("at_speed_exit_frac", "must be in (0, 1]");
  entry_tick_count_ = params.get<int>("at_speed_entry_tick_count", 3);
  if (entry_tick_count_ < 1) {
    params.fail("at_speed_entry_tick_count", "must be >= 1");
  }
  spin_down_rps2_ = params.number("spin_down_rps2", 0.0);
  if (spin_down_rps2_ < 0) params.fail("spin_down_rps2", "must be >= 0");
  slot_ = params.get<int>("slot", 0);
  if (slot_ < 0 || slot_ > 2) params.fail("slot", "must be 0, 1 or 2");
}

void VelocityMechanism::set_goal(double velocity_rps, double feedforward_v) {
  RequireInUpdate(*node_, label_);
  requested_ = true;
  request_rps_ = velocity_rps;
  request_ff_v_ = feedforward_v;
  request_tick_ = node_->tick();
}

void VelocityMechanism::stop() {
  RequireInUpdate(*node_, label_);
  requested_ = false;
}

void VelocityMechanism::update() {
  RequireInUpdate(*node_, label_);
  // A request lasts the update() it was made in (see set_goal()).
  const bool requested = requested_ && request_tick_ == node_->tick();
  requested_ = false;
  const Time now = node_->now();
  connected_ = motor_.connected();
  const bool spin = requested && robot_enabled(*node_) && connected_ &&
                    std::isfinite(request_rps_) && std::isfinite(request_ff_v_);
  driving_ = false;
  goal_rps_ = 0;
  commanded_rps_ = 0;
  if (!spin) {
    last_drive_.reset();
  } else {
    goal_rps_ = std::clamp(request_rps_, -max_rps_, max_rps_);
    double commanded = goal_rps_;
    // Spin-down: a live goal smaller than (or opposite to) the last command
    // is approached at no more than spin_down_rps2.
    if (spin_down_rps2_ > 0 && last_drive_) {
      const double previous = last_commanded_rps_;
      const bool winding_down =
          std::abs(goal_rps_) < std::abs(previous) || goal_rps_ * previous < 0;
      if (winding_down) {
        const double step =
            spin_down_rps2_ *
            std::chrono::duration<double>(now - *last_drive_).count();
        commanded = goal_rps_ > previous ? std::min(goal_rps_, previous + step)
                                         : std::max(goal_rps_, previous - step);
      }
    }
    commanded_rps_ = commanded;
    last_commanded_rps_ = commanded;
    last_drive_ = now;
    motor_.set_velocity(commanded, request_ff_v_, slot_);
    driving_ = true;
  }
  velocity_rps_ = motor_.velocity();
  velocity_recent_ = velocity_recent();
  current_a_ = motor_.current();
  readiness(spin ? request_rps_ : 0.0);
}

// The velocity's age at its State, judged as Motor::rezero judges a
// position's (RezeroAgeBound); unknown (no sample time, no clock mapping)
// is never recent.
bool VelocityMechanism::velocity_recent() const {
  const std::optional<Time> sampled = motor_.velocity_time();
  const std::optional<Time> state = link_->sample_time();
  return sampled && state && *state - *sampled <= velocity_age_bound_;
}

// The shooter's readiness rule: consecutive ticks inside the entry band of
// the clamped goal to enter, the wider exit band to stay.
void VelocityMechanism::readiness(double requested_rps) {
  bool at_speed = phase_ == VelocityPhase::kAtSpeed;
  if (!driving_ || std::abs(goal_rps_) <= kTargetEpsilonRps) {
    at_speed = false;
    entry_ticks_ = 0;
    readiness_goal_rps_ = 0;
  } else {
    if (std::abs(requested_rps - readiness_goal_rps_) > kTargetEpsilonRps) {
      at_speed = false;
      entry_ticks_ = 0;
      readiness_goal_rps_ = requested_rps;
    }
    const double error_rps = std::abs(velocity_rps_ - goal_rps_);
    const double exit_rps =
        std::max(entry_rps_, exit_frac_ * std::abs(goal_rps_));
    if (!velocity_recent_) {
      at_speed = false;
      entry_ticks_ = 0;
    } else if (at_speed) {
      if (error_rps > exit_rps) {
        at_speed = false;
        entry_ticks_ = 0;
      }
    } else if (error_rps <= entry_rps_) {
      at_speed = ++entry_ticks_ >= entry_tick_count_;
    } else {
      entry_ticks_ = 0;
    }
  }
  phase_ = !driving_  ? VelocityPhase::kStopped
           : at_speed ? VelocityPhase::kAtSpeed
                      : VelocityPhase::kSpinningUp;
}

control::VelocityState VelocityMechanism::state() const {
  return control::VelocityState{velocity_rps_, goal_rps_, current_a_, driving_,
                                connected_};
}

}  // namespace talos::mechanisms
