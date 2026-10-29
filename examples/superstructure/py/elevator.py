"""One motion-magic motor. Drives to the goal height, holds it there, and
reports whether it has arrived. The profile (cruise, acceleration), gains and
soft limits are the motor's, in the robot config."""

import talos
from demo import topics
from demo.ElevatorGoal import ElevatorGoal
from demo.ElevatorState import ElevatorState as State
from demo.ElevatorStatus import ElevatorStatus


class Elevator:
    def __init__(self, node):
        self.lift = talos.Motor(node, "lift")
        self.goal = node.subscribe(ElevatorGoal, topics.ELEVATOR_GOAL)
        self.status = node.publish(ElevatorStatus, topics.ELEVATOR_STATUS)
        self.sm = talos.StateMachine(node, State.STOWED, states=State)
        self.tolerance_rot = node.param("tolerance_rot")
        self.goal_rot = 0.0  # stowed

    def update(self):
        # Keep the last goal if the superstructure goes quiet: an elevator
        # that drops when its commander stalls is worse than one that holds.
        if self.goal.fresh():
            self.goal_rot = self.goal.value.height_rot
        self.lift.set_motion_magic(self.goal_rot)

        height = self.lift.position
        at_goal = (self.lift.connected
                   and abs(height - self.goal_rot) <= self.tolerance_rot)
        if not at_goal:
            self.sm.go(State.MOVING)
        elif self.goal_rot == 0.0:
            self.sm.go(State.STOWED)
        else:
            self.sm.go(State.AT_GOAL)

        self.status.send(state=self.sm.state, height_rot=height,
                         goal_rot=self.goal_rot, at_goal=at_goal)
