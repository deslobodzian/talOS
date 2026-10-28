#pragma once

#include <cmath>

#include "demo/elevator_generated.h"
#include "demo/topics.h"
#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

namespace demo {

// One motion-magic motor. Drives to the goal height, holds it there, and
// reports whether it has arrived. The profile (cruise, acceleration), gains
// and soft limits are the motor's, in the robot config.
class Elevator {
 public:
  using State = ElevatorState;  // STOWED, MOVING, AT_GOAL

  explicit Elevator(talos::Node& node)
      : lift_{node, "lift"},
        goal_{node.subscribe<ElevatorGoal>(kElevatorGoalTopic)},
        status_{node.publish<ElevatorStatus>(kElevatorStatusTopic)},
        sm_{node, State::STOWED},
        tolerance_rot_{node.param<double>("tolerance_rot")} {}

  void update() {
    // Keep the last goal if the superstructure goes quiet: an elevator that
    // drops when its commander stalls is worse than one that holds.
    if (goal_.fresh()) goal_rot_ = goal_->height_rot();
    lift_.set_motion_magic(goal_rot_);

    const double height = lift_.position();
    const bool at_goal =
        lift_.connected() && std::abs(height - goal_rot_) <= tolerance_rot_;
    if (!at_goal) sm_.go(State::MOVING);
    else if (goal_rot_ == 0.0) sm_.go(State::STOWED);
    else sm_.go(State::AT_GOAL);

    status_.send(ElevatorStatus{sm_.state(), height, goal_rot_, at_goal});
  }

 private:
  talos::Motor lift_;
  talos::Subscription<ElevatorGoal> goal_;
  talos::Publisher<ElevatorStatus> status_;
  talos::StateMachine<State> sm_;
  double tolerance_rot_;
  double goal_rot_ = 0.0;  // stowed
};

}  // namespace demo
