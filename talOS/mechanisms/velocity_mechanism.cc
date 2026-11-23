#include "talOS/mechanisms/velocity_mechanism.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <string>

namespace talos::mechanisms {
namespace detail {

double StepSpinDown(double goal_rps, double previous_rps, double max_rps,
                    double spin_down_rps2, double dt_s) {
  double commanded = goal_rps;
  if (spin_down_rps2 > 0) {
    const bool opposite_sign = (goal_rps < 0.0 && previous_rps > 0.0) ||
                               (goal_rps > 0.0 && previous_rps < 0.0);
    const bool winding_down =
        std::abs(goal_rps) < std::abs(previous_rps) || opposite_sign;
    if (winding_down) {
      const double step = spin_down_rps2 * (dt_s > 0.0 ? dt_s : 0.0);
      commanded = goal_rps > previous_rps
                      ? std::min(goal_rps, previous_rps + step)
                      : std::max(goal_rps, previous_rps - step);
    }
  }
  return std::clamp(commanded, -max_rps, max_rps);
}

bool StepVelocityReadiness(bool was_at_speed, int& entry_ticks,
                           double velocity_rps, double goal_rps,
                           double entry_rps, double exit_rps,
                           int entry_tick_count, bool velocity_recent) {
  const double error_rps = std::abs(velocity_rps - goal_rps);
  if (!velocity_recent) {
    entry_ticks = 0;
    return false;
  }
  if (was_at_speed) {
    if (!(error_rps <= exit_rps)) {
      entry_ticks = 0;
      return false;
    }
    return true;
  }
  if (error_rps <= entry_rps) {
    return ++entry_ticks >= entry_tick_count;
  }
  entry_ticks = 0;
  return false;
}

}  // namespace detail

VelocityMechanism::VelocityMechanism(Node& node, std::string_view name)
    : node_{&node},
      label_{"VelocityMechanism '" + std::string{name} + "'"},
      motor_{node, name},
      velocity_age_bound_{
          ReadinessAgeBound(node.extension<HardwareLink>().config().hardware)},
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
    if (last_drive_) {
      commanded = detail::StepSpinDown(
          goal_rps_, last_commanded_rps_, max_rps_, spin_down_rps2_,
          std::chrono::duration<double>(now - *last_drive_).count());
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
  readiness(goal_rps_);
}

// The velocity's age at its State against ReadinessAgeBound; an unknown age
// is never recent.
bool VelocityMechanism::velocity_recent() const {
  const std::optional<Duration> age = motor_.velocity_age();
  return age && *age <= velocity_age_bound_;
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
    const double exit_rps =
        std::max(entry_rps_, exit_frac_ * std::abs(goal_rps_));
    at_speed = detail::StepVelocityReadiness(
        at_speed, entry_ticks_, velocity_rps_, goal_rps_, entry_rps_, exit_rps,
        entry_tick_count_, velocity_recent_);
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
