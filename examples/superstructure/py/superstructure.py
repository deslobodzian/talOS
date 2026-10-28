"""Sequences the intake and the elevator, which are separate nodes: it only
reads their states and writes their goals.

  STOWED -> INTAKING -> HOLDING -> RAISING -> SCORING -> STOWED
         (INTAKE)    (has piece) (SCORE_HIGH) (at height) (piece gone)
"""

import talos
from demo.ElevatorGoal import ElevatorGoal
from demo.ElevatorStatus import ElevatorStatus
from demo.IntakeCommand import IntakeCommand
from demo.IntakeGoal import IntakeGoal
from demo.IntakeStatus import IntakeStatus
from demo.SuperstructureCommand import SuperstructureCommand as Command
from demo.SuperstructureGoal import SuperstructureGoal
from demo.SuperstructureState import SuperstructureState as State
from demo.SuperstructureStatus import SuperstructureStatus


class Superstructure:
    def __init__(self, node):
        # Nothing in this example publishes the goal (a driver or autonomous
        # node would), so it is optional to the launcher's graph check.
        self.goal = node.subscribe(SuperstructureGoal, "/superstructure/target",
                                   optional=True)
        self.intake = node.subscribe(IntakeStatus, "/intake/state")
        self.elevator = node.subscribe(ElevatorStatus, "/elevator/state")
        self.intake_goal = node.publish(IntakeGoal, "/intake/target")
        self.elevator_goal = node.publish(ElevatorGoal, "/elevator/target")
        self.status = node.publish(SuperstructureStatus, "/superstructure/state")
        self.sm = talos.StateMachine(node, State.STOWED)
        self.high_rot = node.param("high_rot")

    def update(self):
        command = self.goal.value.command if self.goal.fresh() else Command.STOW
        # Only advance on what the mechanisms report now. If either one has
        # gone quiet, hold the current step and keep sending its goals.
        if self.intake.fresh() and self.elevator.fresh():
            self.advance(command)

        state = self.sm.state
        if state in (State.STOWED, State.HOLDING):
            self.send(IntakeCommand.IDLE, 0.0)
        elif state == State.INTAKING:
            self.send(IntakeCommand.INTAKE, 0.0)
        elif state == State.RAISING:
            self.send(IntakeCommand.IDLE, self.high_rot)
        elif state == State.SCORING:
            self.send(IntakeCommand.EJECT, self.high_rot)
        self.status.send(state=state)

    def advance(self, command):
        has_piece = self.intake.value.has_piece
        sm = self.sm
        if sm.state == State.STOWED:
            if has_piece:
                sm.go(State.HOLDING)
            elif command == Command.INTAKE:
                sm.go(State.INTAKING)
        elif sm.state == State.INTAKING:
            if has_piece:
                sm.go(State.HOLDING)
            elif command == Command.STOW:
                sm.go(State.STOWED)
        elif sm.state == State.HOLDING:
            if not has_piece:
                sm.go(State.STOWED)
            elif command == Command.SCORE_HIGH:
                sm.go(State.RAISING)
        elif sm.state == State.RAISING:
            if self.elevator_at(self.high_rot):
                sm.go(State.SCORING)
            elif command == Command.STOW:
                sm.go(State.HOLDING)
        elif sm.state == State.SCORING:
            if not has_piece:
                sm.go(State.STOWED)

    def elevator_at(self, height):
        """at_goal for *this* height, not a goal the elevator reached earlier."""
        elevator = self.elevator.value
        return elevator.at_goal and elevator.goal_rot == height

    def send(self, intake, elevator_height_rot):
        self.intake_goal.send(command=intake)
        self.elevator_goal.send(height_rot=elevator_height_rot)
