#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

#include "demo/elevator_generated.h"
#include "demo/topics.h"
#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

namespace demo {

// One motion-magic motor with a bottom limit switch. Drives to the goal height
// of its target lane (/elevator/target/<source>, see [targets] in
// elevator.toml), and reports whether it has arrived. The profile (cruise,
// acceleration) and gains are the motor's, in the robot config; the travel,
// 0 to max_height_rot, is enforced here on the homed frame.
//
// Heights count from the bottom switch, not from where the TalonFX powered
// on: until the switch has zeroed the lift (Motor::zeroed()), it is HOMING.
// The lift's config sets requires_zero, so a motion-magic goal is never sent
// in an unhomed frame, and a TalonFX reset or a RoboRIO reboot drops the zero
// and starts homing again. While HOMING with a live, finite target it drives
// down at homing_v (a voltage request, which a dropped zero does not hold)
// until the switch closes, then rezeroes to 0 and resumes the goal. The switch
// zeroes an idle lift too: no motion is needed when it starts at the bottom.
//
// The switch is normally open, wired to ground, read through the input's
// pull-up: closed reads false. An open, cut or unplugged wire reads true,
// "not at the bottom", so a wiring fault can only make homing fail, never
// zero the lift mid-travel.
//
// HOMING_FAILED latches, neutral, when homing has not found the switch within
// homing_timeout_us, when the lift has not moved (|velocity| below
// homing_stall_rps) for homing_stall_us while homing pushes it, or when the
// switch reads closed while the zeroed lift is more than home_margin_rot
// above it (a stuck switch). It clears only when the Driver Station reports
// disabled, not when the hardware link merely goes stale.
//
// With no target (robot disabled, lane stale, released or never sent) it
// drives nothing, homing included: the lift is neutral, it does not hold an
// old goal. Its status then echoes goal_rot 0 (stowed) with at_goal false,
// so no reader mistakes it for having arrived anywhere, and reports STOWED
// while the lift is within tolerance_rot of 0, MOVING otherwise (coasting or
// held up by nothing it controls). While HOMING or HOMING_FAILED, goal_rot is
// 0 and at_goal false; while HOMING, height_rot is not a height (the frame is
// unhomed).
class Elevator {
 public:
  // STOWED, MOVING, AT_GOAL, HOMING, HOMING_FAILED
  using State = ElevatorState;

  explicit Elevator(talos::Node& node)
      : node_{node},
        lift_{node, "lift"},
        bottom_{node, "bottom_limit"},
        ds_{node},
        goal_{node},
        status_{node.publish<ElevatorStatus>(topics::kElevatorStatus)},
        sm_{node, State::HOMING},
        tolerance_rot_{node.param<double>("tolerance_rot")},
        max_height_rot_{node.param<double>("max_height_rot")},
        homing_volts_{node.param<double>("homing_v")},
        homing_timeout_{
            std::chrono::microseconds{node.param<int>("homing_timeout_us")}},
        stall_rps_{node.param<double>("homing_stall_rps")},
        stall_time_{
            std::chrono::microseconds{node.param<int>("homing_stall_us")}},
        home_margin_rot_{node.param<double>("home_margin_rot")} {
    Check(max_height_rot_ > 0 && std::isfinite(max_height_rot_),
          "max_height_rot must be finite and positive");
    Check(homing_volts_ < 0 && std::isfinite(homing_volts_),
          "homing_v must be negative (down, toward the switch)");
    Check(homing_timeout_ > talos::Duration::zero(),
          "homing_timeout_us must be positive");
    Check(stall_rps_ > 0 && std::isfinite(stall_rps_),
          "homing_stall_rps must be finite and positive");
    Check(stall_time_ > talos::Duration::zero(),
          "homing_stall_us must be positive");
    Check(home_margin_rot_ > 0 && std::isfinite(home_margin_rot_),
          "home_margin_rot must be finite and positive");
  }

  void update() {
    const ElevatorGoal* goal = goal_.get();
    const bool connected = lift_.connected();
    const bool at_bottom = !bottom_.get();  // normally open, to ground

    if (ds_.connected() && !ds_.enabled()) {  // the operator's reset
      failed_ = false;
      homing_since_.reset();
      stalled_since_.reset();
    }
    if (!lift_.zeroed() && at_bottom) lift_.rezero(0.0);  // no-op if stale
    if (lift_.zeroed()) {
      homing_since_.reset();
      stalled_since_.reset();
      if (connected && at_bottom && lift_.position() > home_margin_rot_) {
        failed_ = true;  // the switch says bottom, the zeroed lift says not
      }
    }

    const double height = lift_.position();
    if (!failed_ && !lift_.zeroed()) {
      home(goal != nullptr && std::isfinite(goal->height_rot()) && connected &&
           !at_bottom);
    }
    if (failed_ || !lift_.zeroed()) {
      sm_.go(failed_ ? State::HOMING_FAILED : State::HOMING);
      status_.send(ElevatorStatus{sm_.state(), height, 0.0, false});
      return;
    }

    const bool drive =
        goal != nullptr && connected && std::isfinite(goal->height_rot());
    const double goal_rot =
        drive ? std::clamp(goal->height_rot(), 0.0, max_height_rot_) : 0.0;
    if (drive) lift_.set_motion_magic(goal_rot);

    const bool near_goal =
        connected && std::abs(height - goal_rot) <= tolerance_rot_;
    const bool at_goal = drive && near_goal;
    if (!near_goal)
      sm_.go(State::MOVING);
    else if (!drive || goal_rot == 0.0)
      sm_.go(State::STOWED);
    else
      sm_.go(State::AT_GOAL);

    status_.send(ElevatorStatus{sm_.state(), height, goal_rot, at_goal});
  }

 private:
  static void Check(bool ok, const char* what) {
    if (!ok) throw std::invalid_argument(std::string{"elevator: "} + what);
  }

  // One unhomed tick: drives down while `move` (a live finite target,
  // connected feedback, switch open) unless homing has failed. The timeout
  // counts from the first homing command since the last zero or Driver
  // Station disable; losing the target pauses the motion, not that clock.
  // The stall clock counts only while homing pushes and the lift is still.
  void home(bool move) {
    if (!move) {
      stalled_since_.reset();
      return;
    }
    const talos::Time now = node_.now();
    if (!homing_since_) homing_since_ = now;
    if (std::abs(lift_.velocity()) >= stall_rps_)
      stalled_since_.reset();
    else if (!stalled_since_)
      stalled_since_ = now;
    if (now - *homing_since_ >= homing_timeout_ ||
        (stalled_since_ && now - *stalled_since_ >= stall_time_)) {
      failed_ = true;
      return;
    }
    lift_.set_voltage(homing_volts_);
  }

  talos::Node& node_;
  talos::Motor lift_;
  talos::DigitalInput bottom_;
  talos::DriverStation ds_;
  talos::Target<ElevatorGoal> goal_;
  talos::Publisher<ElevatorStatus> status_;
  talos::StateMachine<State> sm_;
  double tolerance_rot_;
  double max_height_rot_;
  double homing_volts_;
  talos::Duration homing_timeout_;
  double stall_rps_;
  talos::Duration stall_time_;
  double home_margin_rot_;
  std::optional<talos::Time> homing_since_;
  std::optional<talos::Time> stalled_since_;
  bool failed_ = false;
};

}  // namespace demo
