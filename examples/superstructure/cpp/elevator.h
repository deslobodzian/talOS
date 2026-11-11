#pragma once

#include <cmath>

#include "control/control_generated.h"
#include "demo/elevator_generated.h"
#include "demo/topics.h"
#include "talOS/devices/devices.h"
#include "talOS/mechanisms/position_mechanism.h"
#include "talOS/node/node.h"

namespace demo {

// One motion-magic motor with a bottom limit switch, as a
// talos::mechanisms::PositionMechanism named "lift": [motors.lift],
// [digital_inputs.bottom_limit] and the [lift] parameters in elevator.toml.
// Drives to the goal height of its target lane (/elevator/target/<source>,
// see [targets] in elevator.toml), and reports whether it has arrived. The
// profile (cruise, acceleration) and gains are the motor's, in the robot
// config; the travel, min_rot (0) to max_rot, is enforced on the homed frame.
//
// Heights count from the bottom switch (homing = "limit_switch"), not from
// where the TalonFX powered on: until the switch has zeroed the lift, it is
// HOMING, and a TalonFX reset or a RoboRIO reboot drops the zero and starts
// homing again. While HOMING with a live, finite target it drives down at
// homing_v until the switch has read closed on home_samples consecutive
// ticks; one closed sample mid-travel does not zero it. The switch zeroes an
// idle lift too. It is normally open, wired to ground behind the input's
// pull-up (limit_switch_closed_reads = false), so a cut wire reads "not at
// the bottom" and can only make homing fail, never zero the lift mid-travel.
//
// HOMING_FAILED is the mechanism's latched fault: homing timed out or pushed
// a still lift, or the switch read closed with the zeroed lift more than
// home_margin_rot above it (stuck). It clears only when the Driver Station
// reports disabled, not when the hardware link merely goes stale.
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
      : lift_{node, "lift"},
        goal_{node},
        status_{node.publish<ElevatorStatus>(topics::kElevatorStatus)},
        sm_{node, State::HOMING} {}

  void update() {
    if (const ElevatorGoal* goal = goal_.get()) {
      lift_.set_goal(goal->height_rot());  // non-finite: drives nothing
    } else {
      lift_.stop();
    }
    lift_.update();

    using Phase = talos::mechanisms::PositionPhase;
    const control::PositionState lift = lift_.state();
    if (lift_.phase() == Phase::kFaulted || !lift_.homed()) {
      sm_.go(lift_.phase() == Phase::kFaulted ? State::HOMING_FAILED
                                              : State::HOMING);
      status_.send(
          ElevatorStatus{sm_.state(), lift.position_rot(), 0.0, false});
      return;
    }
    // goal_rot is 0 and enabled false while the lift drives nothing.
    const bool near_goal =
        lift.connected() && std::abs(lift.position_rot() - lift.goal_rot()) <=
                                lift_.tolerance_rot();
    if (!near_goal)
      sm_.go(State::MOVING);
    else if (!lift.enabled() || lift.goal_rot() == 0.0)
      sm_.go(State::STOWED);
    else
      sm_.go(State::AT_GOAL);

    status_.send(ElevatorStatus{sm_.state(), lift.position_rot(),
                                lift.goal_rot(), lift.at_goal()});
  }

 private:
  talos::mechanisms::PositionMechanism lift_;
  talos::Target<ElevatorGoal> goal_;
  talos::Publisher<ElevatorStatus> status_;
  talos::StateMachine<State> sm_;
};

}  // namespace demo
