#include "talOS/mechanisms/position_mechanism.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <optional>
#include <string>

#include "talOS/configuration/config_parser.h"

namespace talos::mechanisms {
namespace {

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

// Both acquisition times known and at most `bound` apart.
bool Aligned(std::optional<Time> a, std::optional<Time> b, Duration bound) {
  return a.has_value() && b.has_value() &&
         (*a > *b ? *a - *b : *b - *a) <= bound;
}

Duration PositiveMicros(const Params& params, std::string_view key) {
  const Duration value = params.micros(key);
  if (value <= Duration::zero()) params.fail(key, "must be > 0");
  return value;
}

}  // namespace

PositionMechanism::PositionMechanism(Node& node, std::string_view name)
    : node_{&node},
      label_{"PositionMechanism '" + std::string{name} + "'"},
      motor_{node, name},
      ds_{node} {
  const Params params{node, name};
  tolerance_rot_ = params.positive("tolerance_rot");
  at_goal_debounce_ = params.micros("at_goal_debounce_us", 0);
  min_rot_ = params.limit("min_rot", -kInfiniteRot);
  max_rot_ = params.limit("max_rot", kInfiniteRot);
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
  // A wrapped goal or a homed frame needs a bounded travel: wrap would
  // follow a target round and round (a turret winding its cables), and a
  // homed mechanism must know where the far end is.
  if (wrap_rot_ > 0 || homing_ != Homing::kNone) {
    const char* why = wrap_rot_ > 0 ? "must be finite when wrap_rot > 0"
                                    : "must be finite when the mechanism homes";
    if (!std::isfinite(min_rot_)) params.fail("min_rot", why);
    if (!std::isfinite(max_rot_)) params.fail("max_rot", why);
  }

  const hardware::MotorConfig& motor = MotorConfigOf(node, name);
  // The fastest the motor can turn the mechanism, for telling a homing
  // push's frame jump from motion.
  max_velocity_rps_ = motor.max_velocity_rps;
  rezero_skew_ =
      RezeroAgeBound(node.extension<HardwareLink>().config().hardware);
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
    // Homing is a slow push into a stop or a switch, never full power.
    if (std::abs(homing_v_) > kMaxHomingFraction * motor.max_voltage) {
      params.fail("homing_v", "must be at most half the motor's max_voltage (" +
                                  std::to_string(motor.max_voltage) + " V)");
    }
    homing_timeout_ = PositiveMicros(params, "homing_timeout_us");
    homing_stall_rps_ = params.positive("homing_stall_rps");
    homing_stall_time_ = PositiveMicros(params, "homing_stall_us");
  }
  if (homing_ == Homing::kStall) {
    homing_stall_current_a_ = params.positive("homing_stall_current_a");
    // The controller limits stator current to stator_limit_a, so a
    // threshold at or above it could never be reached: homing never homes.
    if (homing_stall_current_a_ >= motor.stator_limit_a) {
      params.fail("homing_stall_current_a",
                  "must be below the motor's stator_limit_a (" +
                      std::to_string(motor.stator_limit_a) + " A)");
    }
    homing_min_push_ = PositiveMicros(params, "homing_min_push_us");
    homing_min_travel_rot_ = params.positive("homing_min_travel_rot");
    if (homing_min_travel_rot_ >= max_rot_ - min_rot_) {
      params.fail("homing_min_travel_rot",
                  "must be less than the travel, max_rot - min_rot");
    }
    // A stop found no sooner than the minimum push plus the still time
    // must leave time before the timeout faults, with a tick of slack at
    // each end for when the push and the stillness are first seen.
    if (homing_timeout_ <=
        homing_min_push_ + homing_stall_time_ + 2 * node.period()) {
      params.fail("homing_timeout_us",
                  "must exceed homing_min_push_us + homing_stall_us + 2 "
                  "periods");
    }
  }
  if (homing_ == Homing::kAbsolute) {
    absolute_.emplace(node, params.get<std::string>("absolute_sensor"));
    absolute_rot_per_sensor_rot_ =
        params.number("absolute_rot_per_sensor_rot", 1.0);
    absolute_offset_rot_ = params.number("absolute_offset_rot", 0.0);
    absolute_margin_rot_ = params.positive("absolute_margin_rot");
    // The CANcoder reads absolute position in [-0.5, 0.5) rotations (the
    // RoboRIO sets its discontinuity at 0.5), so the frame can name only
    // positions in the one sensor rotation around absolute_offset_rot. The
    // whole physical range, the travel plus absolute_margin_rot beyond each
    // end for hard stops and overshoot, must lie strictly inside it, or a
    // mechanism resting past an end reads across the discontinuity and
    // takes a frame a rotation away.
    const double half_rot = 0.5 * std::abs(absolute_rot_per_sensor_rot_);
    const double low = absolute_offset_rot_ - half_rot;
    const double high = absolute_offset_rot_ + half_rot;
    if (!(min_rot_ - absolute_margin_rot_ > low &&
          max_rot_ + absolute_margin_rot_ < high)) {
      params.fail("absolute_offset_rot",
                  "with absolute_rot_per_sensor_rot maps sensor readings in "
                  "[-0.5, 0.5) to (" +
                      std::to_string(low) + ", " + std::to_string(high) +
                      "); min_rot - absolute_margin_rot .. max_rot + "
                      "absolute_margin_rot must lie strictly inside it");
    }
  }

  stall_velocity_rps_ = params.number("stall_velocity_rps", 0.05);
  if (stall_velocity_rps_ < 0) {
    params.fail("stall_velocity_rps", "must be >= 0");
  }
  stall_current_a_ = params.number("stall_current_a", 0.0);
  if (stall_current_a_ < 0) params.fail("stall_current_a", "must be >= 0");
  stall_time_ = params.micros("stall_time_us", 0);
  move_timeout_ = params.micros("move_timeout_us", 0);

  at_goal_debounce_state_ = Debounce{at_goal_debounce_};
  stall_debounce_ = Debounce{stall_time_};
  phase_ = homed() ? PositionPhase::kReady : PositionPhase::kUnhomed;
}

bool PositionMechanism::homed() const {
  return homing_ == Homing::kNone || (motor_.zeroed() && !settling_);
}

void PositionMechanism::request(Request kind, double position_rot,
                                double feedforward_v) {
  RequireInUpdate(*node_, label_);
  request_ = kind;
  request_rot_ = position_rot;
  request_ff_v_ = feedforward_v;
  request_tick_ = node_->tick();
}

void PositionMechanism::set_goal(double position_rot, double feedforward_v) {
  request(Request::kGoal, position_rot, feedforward_v);
}

void PositionMechanism::set_goal_units(double position_units,
                                       double feedforward_v) {
  request(Request::kGoal, position_units / units_per_rot_, feedforward_v);
}

void PositionMechanism::hold(double feedforward_v) {
  request(Request::kHold, 0, feedforward_v);
}

void PositionMechanism::stop() { request(Request::kStop, 0, 0); }

void PositionMechanism::clear_fault() {
  RequireInUpdate(*node_, label_);
  reset_fault();
}

void PositionMechanism::reset_fault() {
  if (phase_ != PositionPhase::kFaulted) return;
  fault_ = PositionFault::kNone;
  phase_ = homed() ? PositionPhase::kReady : PositionPhase::kUnhomed;
}

void PositionMechanism::update() {
  RequireInUpdate(*node_, label_);
  // A request lasts the update() it was made in: one left from a tick whose
  // update() did not reach this call is dropped, so it never drives late.
  const Request request =
      request_tick_ == node_->tick() ? request_ : Request::kStop;
  const double request_rot = request_rot_;
  const double request_ff_v = request_ff_v_;
  request_ = Request::kStop;

  const Time now = node_->now();
  connected_ = motor_.connected();
  driving_ = false;
  goal_rot_ = 0;
  gravity_ff_v_ = 0;

  // The operator's reset: a Driver Station that reports disabled (not a
  // stale link) clears a fault and restarts the homing run.
  if (ds_.connected() && !ds_.enabled()) {
    reset_fault();
    reset_homing_run();
  }

  // The frame. A switch zeroes only after home_samples consecutive closed
  // readings while unzeroed, so one glitch mid-travel does not; a CANcoder
  // zeroes as soon as both read samples acquired within rezero_skew_ of
  // each other, so a late frame of either never sets a frame off by the
  // mechanism's motion in between. A refused rezero (Motor::rezero also
  // refuses a late motor position) leaves the motor unzeroed, so the next
  // tick tries again. A switch sample that is not connected() (an input not
  // answering, a gateway configuring) is no reading, whatever value it
  // carries: it neither counts as closed nor continues a run of closed
  // readings, as a disconnected motor ends a homing push's evidence, and it
  // is never a stuck switch.
  //
  // The switch sets the frame wherever the mechanism is when the run
  // completes, usually still moving at the homing speed (the switch need
  // not be at a hard stop). A profiled move started then would begin at
  // that speed and overshoot past the switch by v^2 / (2 * acceleration),
  // so the mechanism first settles: it is not homed, and drives nothing,
  // until its last home_samples readings with valid feedback are all slower
  // than homing_stall_rps (an invalid one breaks the run). Velocity does not
  // depend on the frame, and rest is rest whether or not the mechanism is
  // enabled or has a goal, so every tick's reading counts, before the zero
  // too: a mechanism already at rest when the switch zeroes it (on a stop,
  // or resting on its switch at power-on) is homed at once. The homing
  // timeout runs only while a live goal waits on the settling (settle()).
  bool switch_closed = false;
  if (limit_switch_.has_value()) {
    switch_closed = limit_switch_->connected() &&
                    limit_switch_->get() == switch_closed_reads_;
    home_count_ = !motor_.zeroed() && switch_closed ? home_count_ + 1 : 0;
    if (home_count_ >= home_samples_) {
      motor_.rezero(home_rot_);
      if (motor_.zeroed()) {
        settling_ = true;
        reset_homing_run();  // settling gets its own homing_timeout_us
      }
    }
  }
  if (!motor_.zeroed()) settling_ = false;  // a dropped zero homes again
  if (absolute_.has_value() && !motor_.zeroed() && connected_ &&
      absolute_->connected() &&
      Aligned(motor_.position_time(), absolute_->position_time(),
              rezero_skew_)) {
    motor_.rezero(absolute_->position() * absolute_rot_per_sensor_rot_ +
                  absolute_offset_rot_);
  }
  position_rot_ = motor_.position();
  velocity_rps_ = motor_.velocity();
  if (limit_switch_.has_value()) {
    rest_count_ = connected_ && std::abs(velocity_rps_) < homing_stall_rps_
                      ? std::min(rest_count_ + 1, home_samples_)
                      : 0;
    if (rest_count_ >= home_samples_) settling_ = false;  // at rest: homed
  }
  if (drives_home() && homed()) reset_homing_run();
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
    // Only a goal homes: hold() has no position to hold in an unknown
    // frame, so it is neutral, and so is losing the goal (which ends the
    // push and its evidence, but not the homing timeout).
    if (!drives_home() || !live || request != Request::kGoal) {
      push_since_.reset();
      homing_still_since_.reset();
      idle(PositionPhase::kUnhomed);
      return;
    }
    if (settling_) {
      settle(now);
      return;
    }
    if (home(now)) return;  // still homing, or faulted
  }
  if (!live) {
    idle(PositionPhase::kReady);
    return;
  }
  drive_goal(request, request_rot, request_ff_v, now);
}

// One tick of a live homing push. Returns false once this tick has set the
// frame (stall homing found its stop), true while homing continues or has
// faulted.
bool PositionMechanism::home(Time now) {
  if (!homing_since_) homing_since_ = now;
  // A position step faster than the motor can turn the mechanism in the
  // time since the last push tick is a frame jump (a reset, a glitch), not
  // motion: the push starts over from here, and its evidence with it.
  if (push_since_ &&
      std::abs(position_rot_ - push_last_rot_) >
          max_velocity_rps_ *
              std::chrono::duration<double>(now - push_last_at_).count()) {
    push_since_.reset();
    homing_still_since_.reset();
  }
  if (!push_since_) {
    push_since_ = now;
    push_start_rot_ = position_rot_;
  }
  push_last_rot_ = position_rot_;
  push_last_at_ = now;
  // The timeout faults, and never homes, whatever else this tick shows.
  if (now - *homing_since_ >= homing_timeout_) {
    fault(PositionFault::kHomingTimeout);
    return true;
  }
  const bool slow = std::abs(velocity_rps_) < homing_stall_rps_;
  bool still_counts = slow;
  if (homing_ == Homing::kStall) {
    // A stop is stillness under load once this push has had the chance to
    // move the mechanism: it has travelled homing_min_travel_rot since the
    // push began, or the push has lasted homing_min_push_us (a mechanism
    // that starts slowly is not yet at its stop). Motion from an earlier,
    // interrupted push does not count.
    const bool moved =
        std::abs(position_rot_ - push_start_rot_) >= homing_min_travel_rot_;
    still_counts = slow &&
                   std::abs(motor_.current()) >= homing_stall_current_a_ &&
                   (moved || now - *push_since_ >= homing_min_push_);
  }
  if (!still_counts) {
    homing_still_since_.reset();
  } else if (!homing_still_since_) {
    homing_still_since_ = now;
  }
  const bool still = homing_still_since_.has_value() &&
                     now - *homing_still_since_ >= homing_stall_time_;
  if (homing_ == Homing::kStall && still) {
    motor_.rezero(home_rot_);  // pushed still against the stop: home
    if (motor_.zeroed()) {
      position_rot_ = motor_.position();
      reset_homing_run();
      return false;
    }
    // Refused (a late position, RezeroAgeBound): push on, keeping the
    // stillness, and retry on the next tick.
  } else if (still) {  // limit-switch homing pushing a still mechanism
    fault(PositionFault::kHomingStall);
    return true;
  }
  motor_.set_voltage(homing_v_);
  driving_ = true;
  idle(PositionPhase::kHoming);
  return true;
}

// One live tick of a goal waiting while the mechanism settles after the
// limit switch set the frame (update() counts the slow readings): neutral,
// as the motor's neutral mode stops it, never driven against the motion.
void PositionMechanism::settle(Time now) {
  if (!homing_since_) homing_since_ = now;
  // A mechanism that never comes to rest (back-driven, falling) faults as
  // a push that never finds home does.
  if (now - *homing_since_ >= homing_timeout_) {
    fault(PositionFault::kHomingTimeout);
    return;
  }
  motor_.stop();
  idle(PositionPhase::kHoming);
}

void PositionMechanism::reset_homing_run() {
  homing_since_.reset();
  push_since_.reset();
  homing_still_since_.reset();
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
