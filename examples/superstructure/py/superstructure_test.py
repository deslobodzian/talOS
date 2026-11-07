"""The whole example robot in one simulated process: the intake, elevator and
superstructure nodes talking over their real topics and target lanes, with
SimHardware in place of the bridge and the Driver Station, all configured by
config/robot_py.toml and config/subsystems/. The same scenarios as
cpp/superstructure_test.cc."""

import math
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

# The values in config/subsystems/*.toml.
INTAKE_VOLTS = 8.0
HOLD_VOLTS = 1.0
EJECT_VOLTS = -10.0
HIGH_ROT = 10.0

NEUTRAL = talos.Mode.NEUTRAL
VOLTAGE = talos.Mode.VOLTAGE
MOTION_MAGIC = talos.Mode.MOTION_MAGIC


def enable_teleop(sim, hw):
    """Enables the robot and puts the Driver Station in enabled teleop: a
    Target drives only with both. The 10 ms step lets the mode begin before
    the first lane is published."""
    hw.enable()
    hw.set_driver_station(enabled=True, teleop=True)
    sim.step(0.01)


class SuperstructureTest(unittest.TestCase):
    def setUp(self):
        self.sim = Sim(CONFIG)
        self.sim.add(intake.Intake, "intake")
        self.sim.add(elevator.Elevator, "elevator")
        self.sim.add(superstructure.Superstructure, "superstructure")
        self.hw = SimHardware(self.sim)

    def hold(self, command, seconds):
        """Holds a driver command the way a driver node would: its teleop
        lane republished every 20 ms."""
        for _ in range(round(seconds / 0.02)):
            self.sim.publish_target(SuperstructureGoal, "superstructure",
                                    "teleop", command=command)
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
        enable_teleop(self.sim, self.hw)
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
        self.assertEqual(
            self.sim.latest_target(IntakeGoal, "intake",
                                   "superstructure").command,
            IntakeCommand.INTAKE)

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

    def test_disabling_neutrals_every_motor_and_steps_back_to_holding(self):
        enable_teleop(self.sim, self.hw)
        self.hw.ideal_motors()
        self.collect()
        self.hw.ideal_motors(False)
        self.hold(Command.SCORE_HIGH, 0.06)
        self.assertEqual(self.superstructure_state(), SuperstructureState.RAISING)

        # Disabled, every motor is neutral and every state is still
        # published. The superstructure's own goal is a Target too, so it has
        # none while disabled: that counts as STOW, and RAISING steps back to
        # HOLDING (the piece stays in). The elevator, with no target, is
        # neutral and reports STOWED at its height of 0 (the lift never
        # moved).
        self.hw.enable(False)
        self.hold(Command.SCORE_HIGH, 0.2)
        self.assertEqual(self.roller().mode, NEUTRAL)
        self.assertEqual(self.lift().mode, NEUTRAL)
        self.assertEqual(self.superstructure_state(), SuperstructureState.HOLDING)
        self.assertEqual(self.intake_status().state, IntakeState.HOLDING)
        self.assertEqual(self.elevator_status().state, ElevatorState.STOWED)
        self.assertFalse(self.elevator_status().at_goal)

        # Re-enabled with SCORE_HIGH still held, it raises and scores again.
        self.hw.enable()
        self.hw.ideal_motors()
        self.hold(Command.SCORE_HIGH, 0.16)
        self.assertEqual(self.lift()[:2], (MOTION_MAGIC, HIGH_ROT))
        self.assertEqual(self.superstructure_state(), SuperstructureState.SCORING)
        self.assertEqual(self.roller().demand, EJECT_VOLTS)

    def test_no_goal_counts_as_stow(self):
        enable_teleop(self.sim, self.hw)
        self.hw.ideal_motors()
        # Never sent: STOWED, and the superstructure still drives both
        # mechanisms' lanes (elevator to 0, intake idle).
        self.sim.step(0.1)
        self.assertEqual(self.superstructure_state(), SuperstructureState.STOWED)
        self.assertEqual(self.lift()[:2], (MOTION_MAGIC, 0.0))
        self.assertEqual(self.roller().mode, NEUTRAL)

        # INTAKING, then the driver's lane is released: STOW, roller off.
        self.hold(Command.INTAKE, 0.1)
        self.assertEqual(self.superstructure_state(),
                         SuperstructureState.INTAKING)
        self.sim.release_target(SuperstructureGoal, "superstructure", "teleop")
        self.sim.step(0.06)
        self.assertEqual(self.superstructure_state(), SuperstructureState.STOWED)
        self.assertEqual(self.intake_status().state, IntakeState.IDLE)
        self.assertEqual(self.roller().mode, NEUTRAL)

        # INTAKING, then the driver's lane goes stale: STOW as well.
        self.hold(Command.INTAKE, 0.1)
        self.assertEqual(self.superstructure_state(),
                         SuperstructureState.INTAKING)
        self.sim.step(0.16)  # past command_timeout_us (100 ms)
        self.assertEqual(self.superstructure_state(), SuperstructureState.STOWED)
        self.assertEqual(self.roller().mode, NEUTRAL)


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
        self.hw = SimHardware(self.sim)
        enable_teleop(self.sim, self.hw)

    def tick(self, command, intake_status, elevator_status, times=1):
        """`times` 20 ms periods: the driver's command, plus whichever
        mechanism states are still being reported (None: gone quiet)."""
        for _ in range(times):
            self.sim.publish_target(SuperstructureGoal, "superstructure",
                                    "teleop", command=command)
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
        return self.sim.latest_target(ElevatorGoal, "elevator",
                                      "superstructure").height_rot

    def assert_lanes_released(self):
        self.assertIsNone(self.sim.latest_target(IntakeGoal, "intake",
                                                "superstructure"))
        self.assertIsNone(self.sim.latest_target(ElevatorGoal, "elevator",
                                                "superstructure"))

    def test_stow_wins_when_raising_arrives_at_scoring_height(self):
        self.tick(Command.INTAKE, HOLDING, DOWN)
        self.tick(Command.SCORE_HIGH, HOLDING, DOWN)
        self.assertEqual(self.state(), SuperstructureState.RAISING)
        self.tick(Command.STOW, HOLDING, UP)
        self.assertEqual(self.state(), SuperstructureState.HOLDING)
        self.assertEqual(self.elevator_goal_rot(), 0.0)
        self.assertEqual(
            self.sim.latest_target(IntakeGoal, "intake",
                                   "superstructure").command,
            IntakeCommand.IDLE)

    def test_scoring_stows_on_command_or_target_release(self):
        for release in (False, True):
            with self.subTest(release=release):
                self.tick(Command.INTAKE, HOLDING, DOWN)
                self.tick(Command.SCORE_HIGH, HOLDING, DOWN)
                self.tick(Command.SCORE_HIGH, HOLDING, UP)
                self.assertEqual(self.state(), SuperstructureState.SCORING)
                if release:
                    self.sim.release_target(SuperstructureGoal,
                                            "superstructure", "teleop")
                    self.sim.publish(IntakeStatus, "/intake/state", **HOLDING)
                    self.sim.publish(ElevatorStatus, "/elevator/state", **UP)
                    self.sim.step(0.02)
                else:
                    self.tick(Command.STOW, HOLDING, UP)
                self.assertEqual(self.state(), SuperstructureState.HOLDING)
                self.assertEqual(self.elevator_goal_rot(), 0.0)
                self.assertEqual(
                    self.sim.latest_target(IntakeGoal, "intake",
                                           "superstructure").command,
                    IntakeCommand.IDLE)

    def test_stale_intake_releases_lanes_until_observations_return(self):
        self.tick(Command.INTAKE, HOLDING, DOWN)
        self.assertEqual(self.state(), SuperstructureState.HOLDING)
        self.tick(Command.SCORE_HIGH, None, DOWN, times=6)
        self.assertEqual(self.state(), SuperstructureState.STOWED)
        self.assert_lanes_released()

        self.tick(Command.SCORE_HIGH, HOLDING, DOWN)
        self.assertEqual(self.state(), SuperstructureState.HOLDING)
        self.assertEqual(self.elevator_goal_rot(), 0.0)
        self.tick(Command.SCORE_HIGH, HOLDING, DOWN)
        self.assertEqual(self.state(), SuperstructureState.RAISING)
        self.assertEqual(self.elevator_goal_rot(), HIGH_ROT)

    def test_stale_status_while_raising_stows_immediately(self):
        for stale_intake in (False, True):
            with self.subTest(stale_intake=stale_intake):
                self.tick(Command.INTAKE, HOLDING, DOWN)
                self.tick(Command.SCORE_HIGH, HOLDING, DOWN)
                self.assertEqual(self.state(), SuperstructureState.RAISING)
                self.tick(Command.SCORE_HIGH,
                          None if stale_intake else HOLDING,
                          DOWN if stale_intake else None, times=4)
                self.assertEqual(self.state(), SuperstructureState.RAISING)

                # STOW coincides with the first tick seeing a stale status.
                self.tick(Command.STOW, None if stale_intake else HOLDING,
                          DOWN if stale_intake else None)
                self.assertEqual(
                    self.state(), SuperstructureState.STOWED if stale_intake
                    else SuperstructureState.HOLDING)
                self.assert_lanes_released()

                # The old high goal cannot revive the cancelled sequence.
                self.tick(Command.STOW, HOLDING, UP)
                self.assertEqual(self.state(), SuperstructureState.HOLDING)
                self.assertEqual(self.elevator_goal_rot(), 0.0)

    def test_piece_lost_while_raising_does_not_start_eject(self):
        self.tick(Command.INTAKE, HOLDING, DOWN)
        self.tick(Command.SCORE_HIGH, HOLDING, DOWN)
        self.assertEqual(self.state(), SuperstructureState.RAISING)
        self.tick(Command.SCORE_HIGH,
                  dict(state=IntakeState.IDLE, has_piece=False), UP)
        self.assertEqual(self.state(), SuperstructureState.STOWED)
        self.assertEqual(self.elevator_goal_rot(), 0.0)
        self.assertEqual(
            self.sim.latest_target(IntakeGoal, "intake",
                                   "superstructure").command,
            IntakeCommand.IDLE)

    def test_stale_scoring_releases_eject_and_does_not_replay_it(self):
        empty = dict(state=IntakeState.IDLE, has_piece=False)
        for stale_intake in (False, True):
            with self.subTest(stale_intake=stale_intake):
                self.tick(Command.INTAKE, HOLDING, DOWN)
                self.tick(Command.SCORE_HIGH, HOLDING, DOWN)
                self.tick(Command.SCORE_HIGH, HOLDING, UP)
                self.assertEqual(self.state(), SuperstructureState.SCORING)
                self.tick(Command.SCORE_HIGH,
                          None if stale_intake else HOLDING,
                          UP if stale_intake else None, times=6)
                self.assertNotEqual(self.state(), SuperstructureState.SCORING)
                self.assert_lanes_released()

                # The piece left while a dependency was silent. A held score
                # must not replay EJECT or the high goal on fresh observations.
                for _ in range(3):
                    self.tick(Command.SCORE_HIGH, empty, UP)
                    self.assertEqual(self.state(), SuperstructureState.STOWED)
                    self.assertEqual(self.elevator_goal_rot(), 0.0)
                    self.assertEqual(
                        self.sim.latest_target(IntakeGoal, "intake",
                                               "superstructure").command,
                        IntakeCommand.IDLE)

    def test_goal_loss_and_disable_cancel_with_stale_statuses(self):
        for cancellation in ("released", "disabled", "expired"):
            with self.subTest(cancellation=cancellation):
                self.tick(Command.INTAKE, HOLDING, DOWN)
                self.tick(Command.SCORE_HIGH, HOLDING, DOWN)
                self.tick(Command.SCORE_HIGH, HOLDING, UP)
                self.assertEqual(self.state(), SuperstructureState.SCORING)
                self.tick(Command.SCORE_HIGH, None, None, times=4)
                self.assertEqual(self.state(), SuperstructureState.SCORING)
                if cancellation == "released":
                    self.sim.release_target(SuperstructureGoal,
                                            "superstructure", "teleop")
                elif cancellation == "disabled":
                    self.hw.enable(False)
                self.sim.step(0.12 if cancellation == "expired" else 0.02)
                self.assertEqual(self.state(), SuperstructureState.STOWED)
                self.assert_lanes_released()

                # Fresh mechanism states cannot revive a cancelled score.
                self.sim.publish(IntakeStatus, "/intake/state", **HOLDING)
                self.sim.publish(ElevatorStatus, "/elevator/state", **UP)
                self.sim.step(0.02)
                self.assertEqual(self.state(), SuperstructureState.HOLDING)
                self.assertEqual(self.elevator_goal_rot(), 0.0)
                if cancellation == "disabled":
                    self.hw.enable()
                    self.sim.step(0.02)  # Keep the 20 ms test phase unchanged.

    def test_at_goal_for_an_earlier_height_does_not_count(self):
        self.tick(Command.INTAKE, HOLDING, DOWN, times=5)
        # The elevator is at_goal, but for 0: RAISING waits for its own height.
        self.tick(Command.SCORE_HIGH, HOLDING, DOWN, times=10)
        self.assertEqual(self.state(), SuperstructureState.RAISING)


class ElevatorAloneTest(unittest.TestCase):
    """The elevator on its own, driven through its superstructure lane by
    hand."""

    def setUp(self):
        self.sim = Sim(CONFIG)
        self.sim.add(elevator.Elevator, "elevator")
        self.hw = SimHardware(self.sim)
        enable_teleop(self.sim, self.hw)
        self.hw.ideal_motors()

    def drive(self, height_rot, seconds):
        """The superstructure's lane, republished every 10 ms."""
        for _ in range(round(seconds / 0.01)):
            self.sim.publish_target(ElevatorGoal, "elevator", "superstructure",
                                    height_rot=height_rot)
            self.sim.step(0.01)

    def lift(self):
        return self.hw.motor_request("elevator", "lift")

    def status(self):
        return self.sim.latest(ElevatorStatus, "/elevator/state")

    def assert_neutral(self):
        """Neutral, and the status says it is driving nothing."""
        self.assertEqual(self.lift().mode, NEUTRAL)
        self.assertEqual(self.status().goal_rot, 0.0)
        self.assertFalse(self.status().at_goal)

    def test_drives_to_the_lane_goal(self):
        self.drive(HIGH_ROT, 0.05)
        self.assertEqual(self.lift()[:2], (MOTION_MAGIC, HIGH_ROT))
        self.assertEqual(self.status().state, ElevatorState.AT_GOAL)
        self.assertTrue(self.status().at_goal)
        self.assertEqual(self.status().goal_rot, HIGH_ROT)

    def test_neutral_when_the_lane_goes_stale(self):
        self.drive(HIGH_ROT, 0.05)
        self.assertEqual(self.lift().mode, MOTION_MAGIC)
        # Nothing more: it does not hold the old goal past command_timeout_us.
        self.sim.step(0.11)
        self.assert_neutral()
        # Up at HIGH_ROT with nothing driving it: MOVING, not AT_GOAL.
        self.assertEqual(self.status().state, ElevatorState.MOVING)

    def test_neutral_when_the_lane_is_released(self):
        self.drive(HIGH_ROT, 0.05)
        self.assertEqual(self.lift().mode, MOTION_MAGIC)
        self.sim.release_target(ElevatorGoal, "elevator", "superstructure")
        self.sim.step(0.02)
        self.assert_neutral()

    def test_neutral_when_the_target_was_never_sent(self):
        self.sim.step(0.1)
        self.assert_neutral()
        self.assertEqual(self.status().state, ElevatorState.STOWED)  # at 0

    def test_neutral_when_disabled(self):
        self.hw.enable(False)
        self.drive(HIGH_ROT, 0.1)
        self.assert_neutral()

    def test_neutral_on_driver_station_loss(self):
        self.drive(HIGH_ROT, 0.05)
        self.assertEqual(self.lift().mode, MOTION_MAGIC)
        self.hw.stop_driver_station()
        self.drive(HIGH_ROT, 0.15)  # the lane stays fresh; the DS does not
        self.assert_neutral()

    def test_neutral_with_the_lift_disconnected(self):
        self.hw.set_connected("elevator", "lift", False)
        self.drive(HIGH_ROT, 0.05)
        self.assert_neutral()
        self.assertEqual(self.status().state, ElevatorState.MOVING)

    def test_neutral_on_a_non_finite_goal(self):
        self.drive(math.nan, 0.05)
        self.assert_neutral()


class IntakeAloneTest(unittest.TestCase):
    """The intake with no target: the state machine carries on, the roller
    is neutral."""

    def test_neutral_when_the_lane_goes_stale(self):
        sim = Sim(CONFIG)
        sim.add(intake.Intake, "intake")
        hw = SimHardware(sim)
        enable_teleop(sim, hw)

        def intake_for(periods):
            for _ in range(periods):
                sim.publish_target(IntakeGoal, "intake", "superstructure",
                                   command=IntakeCommand.INTAKE)
                sim.step(0.02)

        intake_for(5)
        hw.set_digital_input("intake", "beam_break", True)
        intake_for(5)
        self.assertEqual(hw.motor_request("intake", "roller")[:2],
                         (VOLTAGE, HOLD_VOLTS))

        sim.step(0.14)  # past command_timeout_us (100 ms)
        self.assertEqual(hw.motor_request("intake", "roller").mode, NEUTRAL)
        status = sim.latest(IntakeStatus, "/intake/state")
        self.assertEqual(status.state, IntakeState.HOLDING)
        self.assertTrue(status.has_piece)


if __name__ == "__main__":
    unittest.main()
