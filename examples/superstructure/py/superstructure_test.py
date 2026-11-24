"""The whole example robot in one simulated process: the intake, elevator and
superstructure nodes talking over their real topics and target lanes, with
SimHardware in place of the bridge and the Driver Station, all configured by
config/robot_py.toml and config/subsystems/. Detailed mechanism behavior is
covered once in cpp/superstructure_test.cc; these exercise the Python host."""

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
        # The lift starts on its bottom switch, so it zeroes without moving,
        # then leaves it as it rises. The switch is normally open to ground
        # behind a pull-up: closed reads False.
        self.hw.set_digital_input("elevator", "bottom_limit", False)
        self.sim.step(0.03)  # home_samples (3) elevator periods
        self.hw.set_digital_input("elevator", "bottom_limit", True)

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
        self.assertEqual(self.hw.applied_motor("intake", "roller").mode,
                         NEUTRAL)
        self.assertEqual(self.hw.applied_motor("elevator", "lift").mode,
                         NEUTRAL)
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

    # REQ-SAFE-46
    def test_disconnected_beam_break_clears_sensor_connected(self):
        enable_teleop(self.sim, self.hw)
        self.hold(Command.INTAKE, 0.1)
        self.assertTrue(self.intake_status().sensor_connected)
        self.assertEqual(self.roller()[:2], (VOLTAGE, INTAKE_VOLTS))

        self.hw.set_connected("intake", "beam_break", False)
        self.hold(Command.INTAKE, 0.1)
        self.assertFalse(self.intake_status().sensor_connected)
        self.assertEqual(self.roller().mode, NEUTRAL)

        self.hw.set_connected("intake", "beam_break", True)
        self.hold(Command.INTAKE, 0.1)
        self.assertTrue(self.intake_status().sensor_connected)
        self.assertEqual(self.roller()[:2], (VOLTAGE, INTAKE_VOLTS))

    # REQ-SAFE-46
    def test_disconnected_beam_break_exits_scoring_and_does_not_hold_latched_piece(
            self):
        enable_teleop(self.sim, self.hw)
        self.hw.ideal_motors()
        self.collect()

        # Disconnect while HOLDING: latched has_piece must not hold or raise.
        self.hw.set_connected("intake", "beam_break", False)
        self.hold(Command.SCORE_HIGH, 0.1)
        self.assertFalse(self.intake_status().sensor_connected)
        self.assertTrue(self.intake_status().has_piece)
        self.assertEqual(self.superstructure_state(), SuperstructureState.STOWED)
        self.assertEqual(self.lift().demand, 0.0)

        self.hold(Command.STOW, 0.06)
        self.assertEqual(self.superstructure_state(), SuperstructureState.STOWED)

        # Reconnect with beam still broken: returns to HOLDING and scores.
        self.hw.set_connected("intake", "beam_break", True)
        self.hold(Command.STOW, 0.06)
        self.assertEqual(self.superstructure_state(),
                         SuperstructureState.HOLDING)

        self.hold(Command.SCORE_HIGH, 0.12)
        self.assertEqual(self.superstructure_state(),
                         SuperstructureState.SCORING)
        self.assertEqual(self.intake_status().state, IntakeState.EJECTING)

        # Disconnect mid-score: Superstructure stows instead of trapping the
        # lift high in SCORING once the intake's timed eject finishes.
        self.hw.set_connected("intake", "beam_break", False)
        self.hold(Command.SCORE_HIGH, 0.1)
        self.assertFalse(self.intake_status().sensor_connected)
        self.assertEqual(self.superstructure_state(), SuperstructureState.STOWED)
        self.assertEqual(self.lift().demand, 0.0)

        self.hold(Command.SCORE_HIGH, 0.5)
        self.assertEqual(self.intake_status().state, IntakeState.IDLE)
        self.assertEqual(self.roller().mode, NEUTRAL)
        self.assertEqual(self.superstructure_state(), SuperstructureState.STOWED)
        self.assertEqual(self.lift().demand, 0.0)

        self.hold(Command.STOW, 0.06)
        self.assertEqual(self.superstructure_state(), SuperstructureState.STOWED)

    def test_earlier_generation_goal_is_ignored(self):
        enable_teleop(self.sim, self.hw)
        self.hw.ideal_motors()
        self.hold(Command.INTAKE, 0.1)
        self.assertEqual(self.superstructure_state(),
                         SuperstructureState.INTAKING)
        before = self.sim.driver_station_generation
        self.hw.set_driver_station(teleop=True)  # disabled
        self.sim.step(0.04)
        self.hw.set_driver_station(enabled=True, teleop=True)
        self.sim.step(0.02)
        self.assertNotEqual(self.sim.driver_station_generation, before)
        # Goal stamped with the earlier generation counts as STOW.
        self.sim.publish_target(SuperstructureGoal, "superstructure", "teleop",
                                mode_since_us=before, command=Command.INTAKE)
        self.sim.step(0.04)
        self.assertEqual(self.superstructure_state(), SuperstructureState.STOWED)
        self.assertEqual(self.hw.applied_motor("intake", "roller").mode,
                         NEUTRAL)
        # A goal stamped with the current generation drives immediately.
        self.hold(Command.INTAKE, 0.1)
        self.assertEqual(self.superstructure_state(),
                         SuperstructureState.INTAKING)
        self.assertEqual(self.hw.applied_motor("intake", "roller")[:2],
                         (VOLTAGE, INTAKE_VOLTS))

    # REQ-SAFE-04
    def test_elevator_non_finite_goal_goes_neutral(self):
        sim = Sim(CONFIG)
        sim.add(elevator.Elevator, "elevator")
        hw = SimHardware(sim)
        hw.set_digital_input("elevator", "bottom_limit", False)
        sim.step(0.03)
        hw.set_digital_input("elevator", "bottom_limit", True)
        enable_teleop(sim, hw)
        hw.ideal_motors()

        for bad in (float("nan"), float("inf"), float("-inf")):
            with self.subTest(height_rot=bad):
                for _ in range(5):
                    sim.publish_target(ElevatorGoal, "elevator",
                                       "superstructure", height_rot=HIGH_ROT)
                    sim.step(0.01)
                self.assertEqual(
                    hw.applied_motor("elevator", "lift").mode, MOTION_MAGIC)
                for _ in range(5):
                    sim.publish_target(ElevatorGoal, "elevator",
                                       "superstructure", height_rot=bad)
                    sim.step(0.01)
                self.assertEqual(
                    hw.motor_request("elevator", "lift").mode, NEUTRAL)
                self.assertEqual(
                    hw.applied_motor("elevator", "lift").mode, NEUTRAL)
                status = sim.latest(ElevatorStatus, "/elevator/state")
                self.assertIsNotNone(status)
                self.assertTrue(math.isfinite(status.height_rot))
                self.assertTrue(math.isfinite(status.goal_rot))


if __name__ == "__main__":
    unittest.main()
