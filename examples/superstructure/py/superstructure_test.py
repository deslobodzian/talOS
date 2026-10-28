"""The whole example robot in one simulated process: the intake, elevator and
superstructure nodes talking over their real topics, with SimHardware in
place of the bridge, all configured by config/robot_py.toml. The same
scenarios as cpp/superstructure_test.cc."""

import os
import unittest

import talos
from demo.ElevatorGoal import ElevatorGoal
from demo.ElevatorState import ElevatorState
from demo.ElevatorStatus import ElevatorStatus
from demo.IntakeCommand import IntakeCommand
from demo.IntakeGoal import IntakeGoal
from demo.IntakeState import IntakeState
from demo.IntakeStatus import IntakeStatus
from demo.SuperstructureCommand import SuperstructureCommand as Command
from demo.SuperstructureGoal import SuperstructureGoal
from demo.SuperstructureState import SuperstructureState
from demo.SuperstructureStatus import SuperstructureStatus
from python.runfiles import runfiles
from talos.testing import Sim, SimHardware

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = runfiles.Create().Rlocation(os.environ["TALOS_TEST_CONFIG"])
intake = talos.load(os.path.join(HERE, "intake.py"))
elevator = talos.load(os.path.join(HERE, "elevator.py"))
superstructure = talos.load(os.path.join(HERE, "superstructure.py"))

# The values in config/robot_py.toml.
INTAKE_VOLTS = 8.0
HOLD_VOLTS = 1.0
EJECT_VOLTS = -10.0
HIGH_ROT = 10.0

NEUTRAL = talos.Mode.NEUTRAL
VOLTAGE = talos.Mode.VOLTAGE
MOTION_MAGIC = talos.Mode.MOTION_MAGIC


class SuperstructureTest(unittest.TestCase):
    def setUp(self):
        self.sim = Sim(CONFIG)
        self.sim.add(intake.Intake, "intake")
        self.sim.add(elevator.Elevator, "elevator")
        self.sim.add(superstructure.Superstructure, "superstructure")
        self.hw = SimHardware(self.sim)

    def hold(self, command, seconds):
        """Holds a driver command the way a driver node would: republished
        every 20 ms."""
        for _ in range(round(seconds / 0.02)):
            self.sim.publish(SuperstructureGoal, "/superstructure/target",
                             command=command)
            self.sim.step(0.02)

    def set_beam_break(self, broken):
        self.hw.set_digital_input("intake", "beam_break", broken)

    def superstructure_state(self):
        return self.sim.latest(SuperstructureStatus,
                               "/superstructure/state").state

    def intake_status(self):
        return self.sim.latest(IntakeStatus, "/intake/state")

    def elevator_status(self):
        return self.sim.latest(ElevatorStatus, "/elevator/state")

    def roller(self):
        return self.hw.motor_request("intake", "roller")

    def lift(self):
        return self.hw.motor_request("elevator", "lift")

    def collect(self):
        """Up to HOLDING: intake until the beam breaks."""
        self.hold(Command.INTAKE, 0.1)
        self.set_beam_break(True)
        self.hold(Command.INTAKE, 0.1)
        self.assertEqual(self.superstructure_state(), SuperstructureState.HOLDING)

    def test_intakes_raises_scores_and_stows(self):
        self.hw.enable()
        self.hw.ideal_motors()

        # STOWED: elevator at 0, roller off.
        self.hold(Command.STOW, 0.1)
        self.assertEqual(self.superstructure_state(), SuperstructureState.STOWED)
        self.assertEqual(self.roller().mode, NEUTRAL)
        self.assertEqual(self.lift()[:2], (MOTION_MAGIC, 0.0))
        self.assertEqual(self.elevator_status().state, ElevatorState.STOWED)
        self.assertTrue(self.elevator_status().at_goal)

        # INTAKING: roller pulls in.
        self.hold(Command.INTAKE, 0.1)
        self.assertEqual(self.superstructure_state(),
                         SuperstructureState.INTAKING)
        self.assertEqual(self.intake_status().state, IntakeState.INTAKING)
        self.assertEqual(self.roller()[:2], (VOLTAGE, INTAKE_VOLTS))

        # The beam breaks: HOLDING, with a small hold voltage.
        self.set_beam_break(True)
        self.hold(Command.INTAKE, 0.1)
        self.assertEqual(self.superstructure_state(), SuperstructureState.HOLDING)
        self.assertEqual(self.intake_status().state, IntakeState.HOLDING)
        self.assertTrue(self.intake_status().has_piece)
        self.assertEqual(self.roller().demand, HOLD_VOLTS)
        self.assertEqual(self.lift().demand, 0.0)

        # SCORE_HIGH with a lift that has not got there yet: RAISING.
        self.hw.ideal_motors(False)
        self.hold(Command.SCORE_HIGH, 0.1)
        self.assertEqual(self.superstructure_state(), SuperstructureState.RAISING)
        self.assertEqual(self.lift()[:2], (MOTION_MAGIC, HIGH_ROT))
        self.assertEqual(self.elevator_status().state, ElevatorState.MOVING)
        self.assertFalse(self.elevator_status().at_goal)
        self.assertEqual(self.roller().demand, HOLD_VOLTS)  # still holding

        # The lift arrives: at_goal, then SCORING ejects.
        self.hw.ideal_motors()
        self.hold(Command.SCORE_HIGH, 0.1)
        self.assertEqual(self.elevator_status().state, ElevatorState.AT_GOAL)
        self.assertTrue(self.elevator_status().at_goal)
        self.assertEqual(self.elevator_status().height_rot, HIGH_ROT)
        self.assertEqual(self.superstructure_state(), SuperstructureState.SCORING)
        self.assertEqual(self.intake_status().state, IntakeState.EJECTING)
        self.assertEqual(self.roller().demand, EJECT_VOLTS)

        # The piece leaves: back to STOWED, elevator down; the intake
        # finishes its timed eject and idles.
        self.set_beam_break(False)
        self.hold(Command.SCORE_HIGH, 0.1)
        self.assertEqual(self.superstructure_state(), SuperstructureState.STOWED)
        self.assertEqual(self.lift().demand, 0.0)
        self.assertEqual(self.elevator_status().state, ElevatorState.STOWED)
        self.assertEqual(self.intake_status().state, IntakeState.EJECTING)
        self.assertEqual(self.roller().demand, EJECT_VOLTS)

        self.hold(Command.SCORE_HIGH, 0.5)
        self.assertEqual(self.intake_status().state, IntakeState.IDLE)
        self.assertEqual(self.roller().mode, NEUTRAL)
        self.assertEqual(self.superstructure_state(), SuperstructureState.STOWED)

    def test_disabling_neutrals_every_motor_and_keeps_the_states(self):
        self.hw.enable()
        self.hw.ideal_motors()
        self.collect()
        self.hw.ideal_motors(False)
        self.hold(Command.SCORE_HIGH, 0.06)
        self.assertEqual(self.superstructure_state(), SuperstructureState.RAISING)

        self.hw.enable(False)
        self.hold(Command.SCORE_HIGH, 0.2)
        self.assertEqual(self.roller().mode, NEUTRAL)
        self.assertEqual(self.lift().mode, NEUTRAL)
        self.assertEqual(self.superstructure_state(), SuperstructureState.RAISING)
        self.assertEqual(self.intake_status().state, IntakeState.HOLDING)
        self.assertEqual(self.elevator_status().state, ElevatorState.MOVING)

        # Re-enabled, everything carries on from where it was.
        self.hw.enable()
        self.hw.ideal_motors()
        self.hold(Command.SCORE_HIGH, 0.1)
        self.assertEqual(self.lift()[:2], (MOTION_MAGIC, HIGH_ROT))
        self.assertEqual(self.superstructure_state(), SuperstructureState.SCORING)
        self.assertEqual(self.roller().demand, EJECT_VOLTS)


HOLDING = dict(state=IntakeState.HOLDING, has_piece=True)
DOWN = dict(state=ElevatorState.STOWED, height_rot=0.0, goal_rot=0.0,
            at_goal=True)
UP = dict(state=ElevatorState.AT_GOAL, height_rot=HIGH_ROT, goal_rot=HIGH_ROT,
          at_goal=True)


class SuperstructureAloneTest(unittest.TestCase):
    """The superstructure on its own, with the mechanism states published by
    hand so either one can go quiet."""

    def setUp(self):
        self.sim = Sim(CONFIG)
        self.sim.add(superstructure.Superstructure, "superstructure")

    def tick(self, command, intake_status, elevator_status, times=1):
        """`times` 20 ms periods: the driver's command, plus whichever
        mechanism states are still being reported (None: gone quiet)."""
        for _ in range(times):
            self.sim.publish(SuperstructureGoal, "/superstructure/target",
                             command=command)
            if intake_status is not None:
                self.sim.publish(IntakeStatus, "/intake/state", **intake_status)
            if elevator_status is not None:
                self.sim.publish(ElevatorStatus, "/elevator/state",
                                 **elevator_status)
            self.sim.step(0.02)

    def state(self):
        return self.sim.latest(SuperstructureStatus,
                               "/superstructure/state").state

    def elevator_goal_rot(self):
        return self.sim.latest(ElevatorGoal, "/elevator/target").height_rot

    def test_stale_status_holds_the_state_and_does_not_advance(self):
        self.tick(Command.INTAKE, HOLDING, DOWN, times=5)
        self.assertEqual(self.state(), SuperstructureState.HOLDING)

        # The intake goes quiet for longer than command_timeout_us (100 ms).
        # Its last word was "holding a piece", but it is stale: SCORE_HIGH
        # must not raise, and HOLDING's goals keep going out.
        self.tick(Command.INTAKE, None, DOWN, times=6)
        self.tick(Command.SCORE_HIGH, None, DOWN, times=10)
        self.assertEqual(self.state(), SuperstructureState.HOLDING)
        self.assertEqual(self.elevator_goal_rot(), 0.0)
        self.assertEqual(self.sim.latest(IntakeGoal, "/intake/target").command,
                         IntakeCommand.IDLE)

        # It reports again: RAISING.
        self.tick(Command.SCORE_HIGH, HOLDING, DOWN, times=2)
        self.assertEqual(self.state(), SuperstructureState.RAISING)
        self.assertEqual(self.elevator_goal_rot(), HIGH_ROT)

        # Now the elevator goes quiet past the timeout, and the driver lets go
        # (STOW). Without a fresh elevator state it stays RAISING and keeps
        # asking for the height rather than acting on old data.
        self.tick(Command.SCORE_HIGH, HOLDING, None, times=6)
        self.tick(Command.STOW, HOLDING, None, times=10)
        self.assertEqual(self.state(), SuperstructureState.RAISING)
        self.assertEqual(self.elevator_goal_rot(), HIGH_ROT)

        # A fresh at_goal for the high goal: SCORING.
        self.tick(Command.SCORE_HIGH, HOLDING, UP, times=2)
        self.assertEqual(self.state(), SuperstructureState.SCORING)

    def test_at_goal_for_an_earlier_height_does_not_count(self):
        self.tick(Command.INTAKE, HOLDING, DOWN, times=5)
        # The elevator is at_goal, but for 0: RAISING waits for its own height.
        self.tick(Command.SCORE_HIGH, HOLDING, DOWN, times=10)
        self.assertEqual(self.state(), SuperstructureState.RAISING)


class ElevatorAloneTest(unittest.TestCase):
    def test_holds_its_last_goal_when_the_target_goes_stale(self):
        sim = Sim(CONFIG)
        sim.add(elevator.Elevator, "elevator")
        hw = SimHardware(sim)
        hw.enable()
        hw.ideal_motors()
        sim.publish(ElevatorGoal, "/elevator/target", height_rot=HIGH_ROT)
        sim.step(0.3)  # long stale (command_timeout_us = 100 ms)
        self.assertEqual(hw.motor_request("elevator", "lift")[:2],
                         (MOTION_MAGIC, HIGH_ROT))
        self.assertEqual(sim.latest(ElevatorStatus, "/elevator/state").state,
                         ElevatorState.AT_GOAL)


if __name__ == "__main__":
    unittest.main()
