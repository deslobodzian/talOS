"""Node tests: the class on talos.testing.Sim, and the binary.

ScaffoldPyTest is the node test matrix, docs/TESTING.md §7: the case names are
the matrix's, which //tools:check_node_matrix looks for.
"""

import os
import subprocess
import tempfile
import unittest

import talos
from python.runfiles import runfiles
from scaffold_py import topics
from scaffold_py.ScaffoldPyState import ScaffoldPyState
from scaffold_py.ScaffoldPyTarget import ScaffoldPyTarget
from talos.testing import Sim, SimHardware

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "testdata", "robot.toml")
node_py = talos.load(os.path.join(HERE, "scaffold_py.py"))


class ScaffoldPyTest(unittest.TestCase):
    def setUp(self):
        self.sim = Sim(CONFIG)
        self.sim.add(node_py.ScaffoldPy, "scaffold_py")
        self.hw = SimHardware(self.sim)
        self.hw.set_motor("scaffold_py", "scaffold_py_motor", 0.0, 12.5)

    def teleop(self, enabled=True):
        """Robot enabled (or not) in teleop: the default policy reads the
        teleop lane. Steps so the mode is current before any lane."""
        self.hw.enable(enabled)
        self.hw.set_driver_station(enabled=True, teleop=True)
        self.sim.step(0.01)

    def send(self, demand, source="teleop", mode_since_us=None):
        """One target on a lane, stamped like a producer that has drained
        the newest Driver Station packet, or with `mode_since_us`."""
        self.sim.publish_target(ScaffoldPyTarget, "scaffold_py", source,
                                mode_since_us=mode_since_us,
                                velocity_rps=demand)

    def hold(self, demand, seconds, source="teleop"):
        """Republishes a lane every 20 ms, as a producer does."""
        for _ in range(round(seconds / 0.02)):
            self.send(demand, source)
            self.sim.step(0.02)

    def mode(self):
        """What the node asked for (the flush sends neutral for an unset
        motor)."""
        return self.hw.motor_request("scaffold_py", "scaffold_py_motor").mode

    def applied(self):
        """What the gateway applies, after its enable and expiry gates."""
        return self.hw.applied_motor("scaffold_py", "scaffold_py_motor").mode

    def state(self):
        return self.sim.latest(ScaffoldPyState, topics.STATE)

    # REQ-SAFE-07
    def test_drives_at_a_live_lane(self):
        self.teleop()
        self.hold(30.0, 0.06)
        request = self.hw.applied_motor("scaffold_py", "scaffold_py_motor")
        self.assertEqual(request.mode, talos.Mode.VELOCITY)
        self.assertEqual(request.demand, 30.0)
        state = self.state()
        self.assertTrue(state.enabled)
        self.assertEqual(state.velocity_rps, 12.5)
        self.assertTrue(state.connected)

    # REQ-SAFE-02
    def test_disabled_robot_goes_neutral(self):
        self.teleop(enabled=False)
        self.hold(30.0, 0.06)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertEqual(self.applied(), talos.Mode.NEUTRAL)
        self.assertFalse(self.state().enabled)  # state keeps publishing

    # REQ-SAFE-07
    def test_stale_lane_goes_neutral(self):
        self.teleop()
        self.hold(30.0, 0.04)
        self.assertEqual(self.mode(), talos.Mode.VELOCITY)
        self.sim.step(0.15)  # past command_timeout_us
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertEqual(self.applied(), talos.Mode.NEUTRAL)
        self.assertFalse(self.state().enabled)  # state keeps publishing

    # REQ-SAFE-07
    def test_released_lane_goes_neutral(self):
        self.teleop()
        self.hold(30.0, 0.04)
        self.assertEqual(self.mode(), talos.Mode.VELOCITY)
        self.sim.release_target(ScaffoldPyTarget, "scaffold_py", "teleop")
        self.sim.step(0.04)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)

    # REQ-SAFE-07
    def test_never_sent_goes_neutral(self):
        self.teleop()
        self.sim.step(0.06)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertFalse(self.state().enabled)

    # REQ-SAFE-07
    def test_driver_station_loss_goes_neutral(self):
        self.teleop()
        self.hold(30.0, 0.04)
        self.assertEqual(self.mode(), talos.Mode.VELOCITY)
        self.hw.stop_driver_station()
        self.hold(30.0, 0.16)  # past command_timeout_us: the DS is stale
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)

    # REQ-CONTRACT-25
    def test_disconnected_feedback_goes_neutral(self):
        self.teleop()
        self.hold(30.0, 0.04)
        self.assertEqual(self.mode(), talos.Mode.VELOCITY)
        self.hw.set_connected("scaffold_py", "scaffold_py_motor", False)
        self.hold(30.0, 0.04)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        state = self.state()
        self.assertFalse(state.connected)
        self.assertFalse(state.enabled)

    # REQ-SAFE-04
    def test_non_finite_demand_goes_neutral(self):
        self.teleop()
        self.hold(float("nan"), 0.04)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertFalse(self.state().enabled)

    # matrix: n/a 9 (the scaffold_py reads no parameters)

    # REQ-SAFE-07
    def test_other_modes_lane_is_ignored(self):
        self.teleop()
        self.hold(30.0, 0.04, source="auto")  # autonomous's lane, in teleop
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.sim.release_target(ScaffoldPyTarget, "scaffold_py", "auto")
        self.hw.set_driver_station(enabled=True, autonomous=True)
        self.sim.step(0.02)
        self.hold(30.0, 0.04)  # teleop's lane, live, in autonomous
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.hold(20.0, 0.04, source="auto")
        request = self.hw.motor_request("scaffold_py", "scaffold_py_motor")
        self.assertEqual(request.mode, talos.Mode.VELOCITY)
        self.assertEqual(request.demand, 20.0)

    # REQ-SAFE-09
    def test_earlier_generation_target_is_ignored(self):
        self.teleop()
        self.hold(30.0, 0.04)
        self.assertEqual(self.mode(), talos.Mode.VELOCITY)
        before = self.sim.driver_station_generation
        self.hw.set_driver_station(teleop=True)  # disabled
        self.sim.step(0.04)
        # Re-enabled. SimHardware publishes the packet at its next 20 ms hook,
        # which the Sim runs after the node's tick at that instant, so the
        # packet waits for the node's next drain; a target its producer
        # queued under the earlier enable, published before that drain,
        # arrives in the same one.
        self.hw.set_driver_station(enabled=True, teleop=True)
        self.sim.step(0.02)
        self.assertNotEqual(self.sim.driver_station_generation, before)
        self.send(30.0, mode_since_us=before)
        self.sim.step(0.02)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        # The same target stamped with the current generation drives at once.
        self.send(30.0)
        self.sim.step(0.02)
        self.assertEqual(self.mode(), talos.Mode.VELOCITY)

    # matrix: n/a 12 (a Python tick allocates in the interpreter)


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
