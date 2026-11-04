#pragma once

#include <cmath>

#include "demo/elevator_generated.h"
#include "demo/topics.h"
#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

namespace demo {

// One motion-magic motor. Drives to the goal height of its target lane
// (/elevator/target/<source>, see [targets] in elevator.toml), and reports
// whether it has arrived. The profile (cruise, acceleration), gains and soft
// limits are the motor's, in the robot config.
//
// With no target (robot disabled, lane stale, released or never sent) it
// drives nothing: the lift is neutral, it does not hold an old goal. Its
// status then echoes goal_rot 0 (stowed) with at_goal false, so no reader
// mistakes it for having arrived anywhere, and reports STOWED while the lift
// is within tolerance_rot of 0, MOVING otherwise (coasting or held up by
// nothing it controls).
class Elevator {
 public:
  using State = ElevatorState;  // STOWED, MOVING, AT_GOAL

  explicit Elevator(talos::Node& node)
      : lift_{node, "lift"},
        goal_{node},
        status_{node.publish<ElevatorStatus>(topics::kElevatorStatus)},
        sm_{node, State::STOWED},
        tolerance_rot_{node.param<double>("tolerance_rot")} {}

  void update() {
    const ElevatorGoal* goal = goal_.get();
    const bool connected = lift_.connected();
    const bool drive =
        goal != nullptr && connected && std::isfinite(goal->height_rot());
    if (drive) lift_.set_motion_magic(goal->height_rot());

    const double height = lift_.position();
    const double goal_rot = drive ? goal->height_rot() : 0.0;
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
  talos::Motor lift_;
  talos::Target<ElevatorGoal> goal_;
  talos::Publisher<ElevatorStatus> status_;
  talos::StateMachine<State> sm_;
  double tolerance_rot_;
};

}  // namespace demo
