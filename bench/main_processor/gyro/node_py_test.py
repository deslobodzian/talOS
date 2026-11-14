"""gyro.py, the bench Pigeon 2 observer, on talos.testing.Sim, configured by
testdata/robot.toml: configuration/robot.toml plus subsystems/gyro.toml, so
the bench motor stays connected beside the Pigeon.

BenchGyroTest is the node test matrix for an observer, docs/TESTING.md §7
(the case names are the matrix's, which //tools:check_node_matrix looks
for), plus the gyro's own cases. The node owns no actuator and reads no
target, so the actuator cases do not apply.
"""

import math
import os
import unittest

import talos
from gyro import topics
from gyro.GyroState import GyroState
from python.runfiles import runfiles
from talos.testing import Sim, SimHardware

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = runfiles.Create().Rlocation(os.environ["TALOS_TEST_CONFIG"])
bench = talos.load(os.path.join(HERE, "gyro.py"))

# std::numeric_limits<std::uint64_t>::max(): an unknown age.
UNKNOWN = 2**64 - 1
# subsystems/gyro.toml's period_us, in seconds.
PERIOD_S = 0.01
PERIOD_NS = 10_000_000
# robot.toml's command_timeout_us, in seconds.
TIMEOUT_S = 0.1


class BenchGyroTest(unittest.TestCase):
    def setUp(self):
        self.sim = Sim(CONFIG)
        self.sim.add(bench.BenchGyro, "gyro")
        self.hw = SimHardware(self.sim)
        # Long enough for the bridge's gateway-to-node clock mapping, which
        # acquisition times (and so the ages) need.
        self.sim.step(0.05)

    def state(self):
        return self.sim.latest(GyroState, topics.STATE)

    def test_publishes_from_a_live_input(self):
        # A quarter turn and a half turn a second clockwise: a rotations
        # value (0.25, -0.5) or a flipped sign fails it.
        self.hw.set_gyro("gyro", "pigeon", 0.25, -0.5,
                         yaw_age_us=2000, yaw_rate_age_us=1000)
        self.sim.step(2 * PERIOD_S)
        state = self.state()
        self.assertTrue(state.connected)
        self.assertTrue(state.hardware_fresh)
        self.assertAlmostEqual(state.yaw_rad, math.pi / 2)
        self.assertAlmostEqual(state.yaw_rate_radps, -math.pi)
        self.assertNotEqual(state.yaw_age_us, UNKNOWN)
        self.assertEqual(state.yaw_age_us - state.yaw_rate_age_us, 1000)

    def test_stale_input_clears_validity(self):
        self.hw.set_gyro("gyro", "pigeon", 0.25, -0.5)
        self.sim.step(2 * PERIOD_S)
        self.hw.lose_states(1000)
        self.sim.step(TIMEOUT_S + 2 * PERIOD_S)
        state = self.state()  # still published, every tick
        self.assertLess(self.sim.now_ns - state.sample_time_ns, PERIOD_NS)
        self.assertFalse(state.hardware_fresh)
        self.assertFalse(state.connected)
        self.assertEqual((state.yaw_age_us, state.yaw_rate_age_us),
                         (UNKNOWN, UNKNOWN))
        self.assertAlmostEqual(state.yaw_rad, math.pi / 2)

    # matrix: n/a 9 (the gyro reads no parameters: its device is config)
    # matrix: n/a 12 (a Python tick allocates in the interpreter)

    # REQ-CONTRACT-34
    def test_invalid_sample_holds_the_last_yaw(self):
        self.hw.set_gyro("gyro", "pigeon", 0.25, -0.5)
        self.sim.step(2 * PERIOD_S)
        # The sample turns invalid with the link fresh (the motor beside it
        # stays connected), and the Pigeon's raw reading moves meanwhile.
        self.hw.set_connected("gyro", "pigeon", False)
        self.hw.set_gyro("gyro", "pigeon", -0.125, 1.0)
        self.sim.step(2 * PERIOD_S)
        state = self.state()
        self.assertTrue(state.hardware_fresh)
        self.assertFalse(state.connected)
        self.assertAlmostEqual(state.yaw_rad, math.pi / 2)
        self.assertEqual(state.yaw_rate_radps, 0.0)
        self.assertEqual(state.yaw_age_us, UNKNOWN)
        self.hw.set_connected("gyro", "pigeon", True)
        self.sim.step(2 * PERIOD_S)
        state = self.state()
        self.assertTrue(state.connected)
        self.assertAlmostEqual(state.yaw_rad, -math.pi / 4)

    def test_publishes_while_disabled(self):
        # An observer reports enabled or not: the gateway enables, then
        # disables, and a heading turned while disabled is still reported.
        self.hw.enable()
        self.hw.set_driver_station(enabled=True, teleop=True)
        self.sim.step(2 * PERIOD_S)
        self.hw.enable(False)
        self.hw.set_driver_station(teleop=True)
        self.hw.set_gyro("gyro", "pigeon", -0.125, 0.25)
        self.sim.step(2 * PERIOD_S)
        state = self.state()
        self.assertLess(self.sim.now_ns - state.sample_time_ns, PERIOD_NS)
        self.assertTrue(state.connected)
        self.assertAlmostEqual(state.yaw_rad, -math.pi / 4)
        self.assertAlmostEqual(state.yaw_rate_radps, math.pi / 2)


if __name__ == "__main__":
    unittest.main()
