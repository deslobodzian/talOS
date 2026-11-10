"""Node tests: the class on talos.testing.Sim, and the binary."""

import os
import subprocess
import tempfile
import unittest

import talos
from position_py import topics
from position_py.PositionPyHoming import PositionPyHoming
from position_py.PositionPyState import PositionPyState
from position_py.PositionPyTarget import PositionPyTarget
from python.runfiles import runfiles
from talos.testing import Sim, SimHardware

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "testdata", "robot.toml")
node_py = talos.load(os.path.join(HERE, "position_py.py"))


class PositionPyTest(unittest.TestCase):
    def setUp(self):
        self.sim = Sim(CONFIG)
        self.sim.add(node_py.PositionPy, "position_py")
        self.hw = SimHardware(self.sim)
        self.hw.set_motor("position_py", "position_py_motor", 0.0, 12.5)
        # Starts on its home switch (closed reads False, see
        # set_home): zeroed without moving.
        self.hw.set_digital_input("position_py", "position_py_home", False)

    def teleop(self, enabled=True):
        """Robot enabled (or not) in teleop: the default policy reads the
        teleop lane. Steps so the mode is current before any lane."""
        self.hw.enable(enabled)
        self.hw.set_driver_station(enabled=True, teleop=True)
        self.sim.step(0.06)  # home_samples ticks on a switch

    def hold(self, demand, seconds):
        """Republishes the teleop lane every 20 ms, as a producer does."""
        for _ in range(round(seconds / 0.02)):
            self.sim.publish_target(PositionPyTarget, "position_py", "teleop",
                                    position_rot=demand)
            self.sim.step(0.02)

    def mode(self):
        return self.hw.motor_request("position_py", "position_py_motor").mode

    def state(self):
        return self.sim.latest(PositionPyState, topics.STATE)

    def test_live_lane_drives_the_motor(self):
        self.teleop()
        self.hold(30.0, 0.06)
        request = self.hw.motor_request("position_py", "position_py_motor")
        self.assertEqual(request.mode, talos.Mode.MOTION_MAGIC)
        self.assertEqual(request.demand, 30.0)
        state = self.state()
        self.assertTrue(state.enabled)
        self.assertEqual(state.position_rot, 0.0)
        self.assertTrue(state.connected)

    def test_disabled_robot_sends_neutral(self):
        self.teleop(enabled=False)
        self.hold(30.0, 0.06)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertFalse(self.state().enabled)

    def test_stale_lane_sends_neutral(self):
        self.teleop()
        self.hold(30.0, 0.04)
        self.assertEqual(self.mode(), talos.Mode.MOTION_MAGIC)
        self.sim.step(0.15)  # past command_timeout_us
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertFalse(self.state().enabled)  # state keeps publishing

    def test_released_lane_sends_neutral(self):
        self.teleop()
        self.hold(30.0, 0.04)
        self.assertEqual(self.mode(), talos.Mode.MOTION_MAGIC)
        self.sim.release_target(PositionPyTarget, "position_py", "teleop")
        self.sim.step(0.04)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)

    def test_never_sent_sends_neutral(self):
        self.teleop()
        self.sim.step(0.06)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertFalse(self.state().enabled)

    def test_driver_station_loss_sends_neutral(self):
        self.teleop()
        self.hold(30.0, 0.04)
        self.assertEqual(self.mode(), talos.Mode.MOTION_MAGIC)
        self.hw.stop_driver_station()
        self.hold(30.0, 0.16)  # past command_timeout_us: the DS is stale
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)

    def test_non_finite_demand_sends_neutral(self):
        self.teleop()
        self.hold(float("nan"), 0.04)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertFalse(self.state().enabled)

    # Homing. The values are testdata/robot.toml's (homing_v = -1.0,
    # max_position_rot = 100.0, homing_timeout_us = 500000,
    # homing_stall_us = 200000). setUp leaves the motor moving at 12.5 rps.

    def set_home(self, closed):
        """Normally open to ground behind the input's pull-up: closed reads
        False."""
        self.hw.set_digital_input("position_py", "position_py_home", not closed)

    def test_homes_before_the_first_goal(self):
        self.set_home(False)
        self.hw.ideal_motors()
        self.teleop()
        self.hold(30.0, 0.06)  # toward the switch, not to 30 unhomed
        request = self.hw.motor_request("position_py", "position_py_motor")
        self.assertEqual(request[:2], (talos.Mode.VOLTAGE, -1.0))
        self.assertEqual(self.state().homing, PositionPyHoming.HOMING)

        # The switch closes 0.5 rotations below the power-on position: that
        # is 0, and targets go out in that frame, clamped to the travel.
        self.hw.set_motor("position_py", "position_py_motor", -0.5, 0.0)
        self.set_home(True)
        self.hold(30.0, 0.06)  # home_samples (3) consecutive closed readings
        self.set_home(False)  # the motor leaves the switch
        self.hold(30.0, 0.02)
        request = self.hw.motor_request("position_py", "position_py_motor")
        self.assertEqual(request[:2], (talos.Mode.MOTION_MAGIC, 29.5))
        self.assertEqual(self.state().homing, PositionPyHoming.HOMED)
        self.hold(1000.0, 0.04)
        self.assertEqual(
            self.hw.motor_request("position_py", "position_py_motor").demand, 99.5)
        self.hold(-5.0, 0.04)  # position 0
        self.assertEqual(
            self.hw.motor_request("position_py", "position_py_motor").demand, -0.5)

    def test_one_closed_sample_mid_travel_does_not_zero(self):
        # A glitch on the switch wire is not home: homing keeps driving.
        self.set_home(False)
        self.teleop()
        self.hold(30.0, 0.06)
        self.set_home(True)
        self.hold(30.0, 0.02)  # one tick
        self.set_home(False)
        self.hold(30.0, 0.06)
        self.assertEqual(self.mode(), talos.Mode.VOLTAGE)
        self.assertEqual(self.state().homing, PositionPyHoming.HOMING)

    def test_homing_needs_a_live_finite_target(self):
        self.set_home(False)
        self.teleop(enabled=False)
        self.hold(30.0, 0.06)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)  # disabled
        self.teleop()
        self.hold(float("nan"), 0.04)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.hold(30.0, 0.04)
        self.assertEqual(self.mode(), talos.Mode.VOLTAGE)
        self.sim.step(0.15)  # past command_timeout_us: the lane is stale
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertEqual(self.state().homing, PositionPyHoming.HOMING)

    def test_homing_fails_until_the_driver_station_disables(self):
        self.set_home(False)
        self.teleop()
        self.hold(30.0, 0.4)
        self.assertEqual(self.mode(), talos.Mode.VOLTAGE)
        self.hold(30.0, 0.2)  # past homing_timeout_us: the switch stays open
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertEqual(self.state().homing, PositionPyHoming.FAILED)
        # A stale hardware link (robot_enabled False) does not clear it.
        self.hw.lose_states(8)
        self.hold(30.0, 0.16)
        self.hold(30.0, 0.06)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertEqual(self.state().homing, PositionPyHoming.FAILED)
        self.hw.set_driver_station(enabled=False, teleop=True)
        self.sim.step(0.04)
        self.teleop()
        self.hold(30.0, 0.04)
        self.assertEqual(self.mode(), talos.Mode.VOLTAGE)

    def test_homing_that_does_not_move_fails(self):
        self.set_home(False)
        self.hw.set_motor("position_py", "position_py_motor", 0.0, 0.0)
        self.teleop()
        self.hold(30.0, 0.1)
        self.assertEqual(self.mode(), talos.Mode.VOLTAGE)
        self.hold(30.0, 0.2)  # past homing_stall_us
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertEqual(self.state().homing, PositionPyHoming.FAILED)

    def test_a_stuck_home_switch_stops_the_first_move(self):
        self.hw.ideal_motors()
        self.teleop()
        self.hold(30.0, 0.06)  # the switch still reads closed at 30
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertEqual(self.state().homing, PositionPyHoming.FAILED)


class BinaryTest(unittest.TestCase):
    def run_binary(self, *argv):
        resolver = runfiles.Create()
        binary = resolver.Rlocation(os.environ["NODE_BINARY"])
        env = dict(os.environ, **resolver.EnvVars())
        env.pop("PYTHONPATH", None)
        with tempfile.TemporaryDirectory() as cwd:
            proc = subprocess.run([binary, *argv], cwd=cwd, env=env,
                                  capture_output=True, text=True, timeout=120)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        return proc.stdout

    def test_sim_run_replays_without_divergence(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = os.path.join(tmp, "node_test.tlog")
            self.run_binary("--sim", "--duration-s", "1", "--config", CONFIG,
                            "--log", log)
            self.run_binary("--replay", log, "--config", CONFIG)


if __name__ == "__main__":
    unittest.main()
