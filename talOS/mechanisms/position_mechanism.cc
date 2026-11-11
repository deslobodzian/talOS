#include "talOS/mechanisms/position_mechanism.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <string>

namespace talos::mechanisms {
namespace {

constexpr double kInfinity = std::numeric_limits<double>::infinity();

Homing ParseHoming(const Params& params) {
  const std::string value = params.get<std::string>("homing", "none");
  if (value == "none") return Homing::kNone;
  if (value == "limit_switch") return Homing::kLimitSwitch;
  if (value == "stall") return Homing::kStall;
  if (value == "absolute") return Homing::kAbsolute;
  params.fail("homing",
              "must be \"none\", \"limit_switch\", \"stall\" or \"absolute\"");
}

Gravity ParseGravity(const Params& params) {
  const std::string value = params.get<std::string>("gravity", "elevator");
  if (value == "elevator") return Gravity::kElevator;
  if (value == "arm") return Gravity::kArm;
  params.fail("gravity", "must be \"elevator\" or \"arm\"");
}

bool ParseMotionMagic(const Params& params) {
  const std::string value = params.get<std::string>("control", "motion_magic");
  if (value == "motion_magic") return true;
  if (value == "position") return false;
  params.fail("control", "must be \"motion_magic\" or \"position\"");
}

}  // namespace

PositionMechanism::PositionMechanism(Node& node, std::string_view name)
    : node_{&node}, motor_{node, name}, ds_{node} {
  const Params params{node, name};
  tolerance_rot_ = params.positive("tolerance_rot");
  at_goal_debounce_ = params.micros("at_goal_debounce_us", 0);
  min_rot_ = params.limit("min_rot", -kInfinity);
  max_rot_ = params.limit("max_rot", kInfinity);
  if (!(min_rot_ < max_rot_)) params.fail("max_rot", "must be > min_rot");
  units_per_rot_ = params.positive("units_per_rot", 1.0);
  wrap_rot_ = params.number("wrap_rot", 0.0);
  if (wrap_rot_ < 0) params.fail("wrap_rot", "must be >= 0");

  gravity_ = ParseGravity(params);
  gravity_v_ = params.number("gravity_v", 0.0);
  gravity_zero_rot_ = params.number("gravity_zero_rot", 0.0);
  motion_magic_ = ParseMotionMagic(params);
  slot_ = params.get<int>("slot", 0);
  if (slot_ < 0 || slot_ > 2) params.fail("slot", "must be 0, 1 or 2");

  homing_ = ParseHoming(params);
  const hardware::MotorConfig& motor = MotorConfigOf(node, name);
  // One gravity source: the controller's slot g (with its gravity_type) or
  // gravity_v here, never both, or the mechanism gets twice the feedforward.
  if (gravity_v_ != 0 && motor.slots[static_cast<std::size_t>(slot_)].g != 0) {
    params.fail("gravity_v",
                "is set while the motor's slot" + std::to_string(slot_) +
                    " g already compensates gravity; use one, not both");
  }
  if (homing_ == Homing::kNone && !motor.absolute_position_frame.value) {
    params.fail("homing",
                "is \"none\", but the motor's position_frame is \"rezero\": "
                "it would never drive; home it or use an absolute frame");
  }
  const auto switch_name = params.get<std::string>("limit_switch", "");
  if (!switch_name.empty() && homing_ != Homing::kLimitSwitch) {
    params.fail("limit_switch", "is read only with homing = \"limit_switch\"");
  }
  if (homing_ == Homing::kLimitSwitch) {
    if (switch_name.empty()) {
      params.fail("limit_switch", "is required with homing = \"limit_switch\"");
    }
    limit_switch_.emplace(node, switch_name);
    switch_closed_reads_ = params.get<bool>("limit_switch_closed_reads", false);
    home_samples_ = params.get<int>("home_samples", 3);
    if (home_samples_ < 1) params.fail("home_samples", "must be >= 1");
    home_margin_rot_ = params.positive("home_margin_rot");
  }
  if (drives_home()) {
    homing_v_ = params.number("homing_v");
    home_rot_ = params.number("home_rot", 0.0);
    // Homing drives toward home_rot, the end of the travel homing_v points
    // at; a sign that drives away from it can never home.
    if (homing_v_ == 0 || (homing_v_ < 0 && home_rot_ > min_rot_) ||
        (homing_v_ > 0 && home_rot_ < max_rot_)) {
      params.fail("homing_v",
                  "must be nonzero and point at home_rot, the end of travel "
                  "(min_rot for a negative homing_v, max_rot for a positive)");
    }
    homing_timeout_ = params.micros("homing_timeout_us");
    if (homing_timeout_ <= Duration::zero()) {
      params.fail("homing_timeout_us", "must be > 0");
    }
    homing_stall_rps_ = params.positive("homing_stall_rps");
    homing_stall_time_ = params.micros("homing_stall_us");
    if (homing_stall_time_ <= Duration::zero()) {
      params.fail("homing_stall_us", "must be > 0");
    }
  }
  if (homing_ == Homing::kAbsolute) {
    absolute_.emplace(node, params.get<std::string>("absolute_sensor"));
    absolute_rot_per_sensor_rot_ =
        params.number("absolute_rot_per_sensor_rot", 1.0);
    if (absolute_rot_per_sensor_rot_ == 0) {
      params.fail("absolute_rot_per_sensor_rot", "must be nonzero");
    }
    absolute_offset_rot_ = params.number("absolute_offset_rot", 0.0);
  }

  stall_velocity_rps_ = params.number("stall_velocity_rps", 0.05);
  if (stall_velocity_rps_ < 0)
    params.fail("stall_velocity_rps", "must be >= 0");
  stall_current_a_ = params.number("stall_current_a", 0.0);
  if (stall_current_a_ < 0) params.fail("stall_current_a", "must be >= 0");
  stall_time_ = params.micros("stall_time_us", 0);
  move_timeout_ = params.micros("move_timeout_us", 0);

  at_goal_debounce_state_ = Debounce{at_goal_debounce_};
  stall_debounce_ = Debounce{stall_time_};
  phase_ = homed() ? PositionPhase::kReady : PositionPhase::kUnhomed;
}

bool PositionMechanism::homed() const {
  return homing_ == Homing::kNone || motor_.zeroed();
}

void PositionMechanism::set_goal(double position_rot, double feedforward_v) {
  request_ = Request::kGoal;
  request_rot_ = position_rot;
  request_ff_v_ = feedforward_v;
}

void PositionMechanism::set_goal_units(double position_units,
                                       double feedforward_v) {
  set_goal(position_units / units_per_rot_, feedforward_v);
}

void PositionMechanism::hold(double feedforward_v) {
  request_ = Request::kHold;
  request_rot_ = 0;
  request_ff_v_ = feedforward_v;
}

void PositionMechanism::stop() { request_ = Request::kStop; }

void PositionMechanism::clear_fault() {
  if (phase_ != PositionPhase::kFaulted) return;
  fault_ = PositionFault::kNone;
  phase_ = homed() ? PositionPhase::kReady : PositionPhase::kUnhomed;
}

void PositionMechanism::update() {
  // Consume this tick's request: the next tick starts at stop().
  const Request request = request_;
  const double request_rot = request_rot_;
  const double request_ff_v = request_ff_v_;
  request_ = Request::kStop;

  const Time now = node_->now();
  connected_ = motor_.connected();
  driving_ = false;
  goal_rot_ = 0;
  gravity_ff_v_ = 0;

  // The operator's reset: a Driver Station that reports disabled (not a
  // stale link) clears a fault and restarts the homing clock.
  if (ds_.connected() && !ds_.enabled()) {
    clear_fault();
    homing_since_.reset();
    homing_still_since_.reset();
  }

  // The frame. A switch zeroes only after home_samples consecutive closed
  // readings while unzeroed, so one glitch mid-travel does not; a CANcoder
  // zeroes as soon as both read.
  bool switch_closed = false;
  if (limit_switch_.has_value()) {
    switch_closed = limit_switch_->get() == switch_closed_reads_;
    home_count_ = !motor_.zeroed() && switch_closed ? home_count_ + 1 : 0;
    if (home_count_ >= home_samples_) motor_.rezero(home_rot_);
  }
  if (absolute_.has_value() && !motor_.zeroed() && connected_ &&
      absolute_->connected()) {
    motor_.rezero(absolute_->position() * absolute_rot_per_sensor_rot_ +
                  absolute_offset_rot_);
  }
  position_rot_ = motor_.position();
  velocity_rps_ = motor_.velocity();
  if (drives_home() && motor_.zeroed()) {
    homing_since_.reset();
    homing_still_since_.reset();
  }
  // A switch closed with the homed mechanism more than home_margin_rot into
  // its travel (away from home) is stuck or miswired.
  const double into_travel_rot =
      homing_v_ < 0 ? position_rot_ - home_rot_ : home_rot_ - position_rot_;
  if (switch_closed && motor_.zeroed() && connected_ &&
      into_travel_rot > home_margin_rot_) {
    fault(PositionFault::kSwitchStuck);
  }
  if (phase_ == PositionPhase::kFaulted) return;  // latched neutral

  const bool live = robot_enabled(*node_) && connected_ &&
                    request != Request::kStop && std::isfinite(request_rot) &&
                    std::isfinite(request_ff_v);
  if (!homed()) {
    if (!drives_home() || !live) {
      // Losing the target pauses homing's motion, not its clock.
      homing_still_since_.reset();
      idle(PositionPhase::kUnhomed);
      return;
    }
    if (!homing_since_) homing_since_ = now;
    if (std::abs(velocity_rps_) >= homing_stall_rps_) {
      homing_still_since_.reset();
    } else if (!homing_still_since_) {
      homing_still_since_ = now;
    }
    const bool still = homing_still_since_.has_value() &&
                       now - *homing_still_since_ >= homing_stall_time_;
    if (homing_ == Homing::kStall && still) {
      motor_.rezero(home_rot_);  // at the hard stop: that is home
      position_rot_ = motor_.position();
      homing_since_.reset();
      homing_still_since_.reset();
    } else if (now - *homing_since_ >= homing_timeout_) {
      fault(PositionFault::kHomingTimeout);
      return;
    } else if (still) {
      fault(PositionFault::kHomingStall);
      return;
    } else {
      motor_.set_voltage(homing_v_);
      driving_ = true;
      idle(PositionPhase::kHoming);
      return;
    }
  }
  if (!live) {
    idle(PositionPhase::kReady);
    return;
  }
  drive_goal(request, request_rot, request_ff_v, now);
}

void PositionMechanism::drive_goal(Request request, double request_rot,
                                   double request_ff_v, Time now) {
  double goal = request_rot;
  if (request == Request::kHold) {
    if (!held_rot_) held_rot_ = position_rot_;
    goal = *held_rot_;
  } else {
    held_rot_.reset();
  }
  goal = std::clamp(wrap(goal), min_rot_, max_rot_);
  gravity_ff_v_ = gravity_at(position_rot_);
  const double feedforward_v = request_ff_v + gravity_ff_v_;
  if (motion_magic_) {
    motor_.set_motion_magic(goal, feedforward_v, slot_);
  } else {
    motor_.set_position(goal, feedforward_v, slot_);
  }
  driving_ = true;
  goal_rot_ = goal;

  // A new goal (beyond tolerance of the last anchor) restarts the move
  // timer and the at-goal debounce.
  if (!goal_anchor_rot_ ||
      std::abs(goal - *goal_anchor_rot_) > tolerance_rot_) {
    goal_anchor_rot_ = goal;
    goal_since_ = now;
    at_goal_debounce_state_.reset();
    stall_debounce_.reset();
  }
  const bool near = std::abs(position_rot_ - goal) <= tolerance_rot_;
  phase_ = at_goal_debounce_state_.update(near, now) ? PositionPhase::kAtGoal
                                                     : PositionPhase::kMoving;
  if (near) {
    stall_debounce_.reset();
    return;
  }
  if (move_timeout_ > Duration::zero() && now - goal_since_ >= move_timeout_) {
    fault(PositionFault::kMoveTimeout);
    return;
  }
  const bool stuck = std::abs(velocity_rps_) <= stall_velocity_rps_ &&
                     std::abs(motor_.current()) >= stall_current_a_;
  if (stall_time_ > Duration::zero() && stall_debounce_.update(stuck, now)) {
    fault(PositionFault::kStall);
  }
}

control::PositionState PositionMechanism::state() const {
  return control::PositionState{position_rot_, velocity_rps_, goal_rot_,
                                driving_,      at_goal(),     connected_};
}

void PositionMechanism::fault(PositionFault reason) {
  motor_.stop();  // overrides anything set this tick
  driving_ = false;
  goal_rot_ = 0;
  gravity_ff_v_ = 0;
  idle(PositionPhase::kFaulted);
  fault_ = reason;
}

void PositionMechanism::idle(PositionPhase phase) {
  phase_ = phase;
  held_rot_.reset();
  goal_anchor_rot_.reset();
  at_goal_debounce_state_.reset();
  stall_debounce_.reset();
}

// The equivalent of `goal` (goal + k * wrap_rot) nearest the current
// position, moved inside [min_rot, max_rot] when one equivalent is.
double PositionMechanism::wrap(double goal) const {
  if (wrap_rot_ <= 0) return goal;
  double best =
      goal + std::round((position_rot_ - goal) / wrap_rot_) * wrap_rot_;
  if (best > max_rot_) {
    best -= std::ceil((best - max_rot_) / wrap_rot_) * wrap_rot_;
  }
  if (best < min_rot_) {
    best += std::ceil((min_rot_ - best) / wrap_rot_) * wrap_rot_;
  }
  return best;
}

double PositionMechanism::gravity_at(double position_rot) const {
  if (gravity_ == Gravity::kElevator) return gravity_v_;
  return gravity_v_ *
         std::cos(2 * std::numbers::pi * (position_rot - gravity_zero_rot_));
}

}  // namespace talos::mechanisms
