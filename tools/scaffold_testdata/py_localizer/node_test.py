"""Node tests: the class on talos.testing.Sim, and the binary."""

import math
import os
import subprocess
import tempfile
import unittest

import talos
from geometry.PoseEstimate import PoseEstimate
from geometry.PoseObservation import PoseObservation
from py_localizer import topics
from python.runfiles import runfiles
from talos.testing import Sim, SimHardware

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "testdata", "robot.toml")
node_py = talos.load(os.path.join(HERE, "py_localizer.py"))


class PyLocalizerTest(unittest.TestCase):
    def setUp(self):
        self.sim = Sim(CONFIG)
        self.sim.add(node_py.PyLocalizer, "py_localizer")
        self.hw = SimHardware(self.sim)

    def observe(self, camera, x, y, valid=True, std_m=0.1):
        """A camera's observation of (x, y), taken now."""
        self.sim.publish(
            PoseObservation, f"/{camera}/state",
            sample_time_ns=round(self.sim.now * 1e9), pose_x_m=x, pose_y_m=y,
            pose_yaw_rad=0.5, x_std_m=std_m, y_std_m=std_m, yaw_std_rad=0.05,
            tag_count=2, valid=valid)

    def state(self):
        return self.sim.latest(PoseEstimate, topics.STATE)

    def test_follows_a_usable_observation(self):
        self.sim.step(0.02)
        self.observe("front_camera", 1.0, 2.0)
        self.sim.step(0.04)
        state = self.state()
        self.assertTrue(state.valid)
        self.assertEqual((state.pose_x_m, state.pose_y_m), (1.0, 2.0))
        self.assertEqual(state.observation_accepted_count, 1)  # counted once

    def test_estimates_while_the_robot_is_disabled(self):
        self.hw.enable(False)  # nothing here is actuation-gated
        self.sim.step(0.02)
        self.observe("back_camera", 3.0, 1.0)
        self.sim.step(0.04)
        self.assertTrue(self.state().valid)
        self.assertEqual(self.state().pose_x_m, 3.0)

    def test_stale_input_is_invalid_and_holds_the_pose(self):
        self.sim.step(0.02)
        self.observe("front_camera", 1.0, 2.0)
        self.sim.step(0.04)
        self.assertTrue(self.state().valid)
        self.sim.step(0.6)  # past observation_timeout_us
        self.assertFalse(self.state().valid)
        self.assertEqual(self.state().pose_x_m, 1.0)

    def test_rejects_unusable_observations(self):
        self.sim.step(0.02)
        self.observe("front_camera", 1.0, 2.0, valid=False)
        self.sim.step(0.04)
        self.observe("front_camera", 1.0, 2.0, std_m=math.nan)
        self.sim.step(0.04)
        state = self.state()
        self.assertFalse(state.valid)
        self.assertEqual((state.observation_accepted_count,
                          state.observation_rejected_count), (0, 2))

    def test_reads_only_the_listed_cameras(self):
        self.sim.step(0.02)
        self.observe("side_camera", 1.0, 2.0)  # not in `cameras`
        self.sim.step(0.04)
        self.assertFalse(self.state().valid)
        self.assertEqual(self.state().observation_accepted_count, 0)


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
