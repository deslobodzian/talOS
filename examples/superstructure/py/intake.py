"""Roller + beam break. Pulls a piece in, holds it once the beam breaks, and
spits it out for a fixed time when asked."""

import talos
from demo import topics
from demo.IntakeCommand import IntakeCommand
from demo.IntakeGoal import IntakeGoal
from demo.IntakeState import IntakeState as State
from demo.IntakeStatus import IntakeStatus


class Intake:
    def __init__(self, node):
        self.roller = talos.Motor(node, "roller")
        self.beam_break = talos.DigitalInput(node, "beam_break")
        self.goal = node.subscribe(IntakeGoal, topics.INTAKE_GOAL)
        self.status = node.publish(IntakeStatus, topics.INTAKE_STATUS)
        self.sm = talos.StateMachine(node, State.IDLE, states=State)
        self.intake_volts = node.param("intake_volts")
        self.hold_volts = node.param("hold_volts")
        self.eject_volts = node.param("eject_volts")
        self.eject_s = node.param("eject_ms") / 1000

    def update(self):
        # A stale goal means nobody is driving us: treat it as IDLE.
        command = self.goal.value.command if self.goal.fresh() else IntakeCommand.IDLE
        has_piece = self.beam_break.value
        sm = self.sm

        if sm.state == State.IDLE:
            if command == IntakeCommand.INTAKE:
                sm.go(State.INTAKING)
        elif sm.state == State.INTAKING:
            self.roller.set_voltage(self.intake_volts)
            if has_piece:
                sm.go(State.HOLDING)
            elif command == IntakeCommand.IDLE:
                sm.go(State.IDLE)
            elif command == IntakeCommand.EJECT:
                sm.go(State.EJECTING)
        elif sm.state == State.HOLDING:
            self.roller.set_voltage(self.hold_volts)
            if command == IntakeCommand.EJECT:
                sm.go(State.EJECTING)
            elif not has_piece:
                sm.go(State.IDLE)  # dropped it
        elif sm.state == State.EJECTING:
            self.roller.set_voltage(self.eject_volts)
            if sm.time_in_state >= self.eject_s:
                sm.go(State.IDLE)
        # Anything not set this tick (IDLE) is neutral, and every motor is
        # neutral while the robot is disabled.
        self.status.send(state=sm.state, has_piece=has_piece)
