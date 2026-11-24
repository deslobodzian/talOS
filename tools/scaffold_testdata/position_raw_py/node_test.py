"""Node tests: the class on talos.testing.Sim.

PositionRawPyTest is the node test matrix, docs/TESTING.md §7 in the talos
repository: the case names are the matrix's, which talos's
//tools:check_node_matrix looks for.
"""

import os
import unittest

import talos
from position_raw_py import topics
from position_raw_py.PositionRawPyHoming import PositionRawPyHoming
from position_raw_py.PositionRawPyState import PositionRawPyState
from position_raw_py.PositionRawPyTarget import PositionRawPyTarget
from talos.testing import Sim, SimHardware

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "testdata", "robot.toml")
node_py = talos.load(os.path.join(HERE, "position_raw_py.py"))


class PositionRawPyTest(unittest.TestCase):
    def setUp(self):
        self.sim = Sim(CONFIG)
        self.sim.add(node_py.PositionRawPy, "position_raw_py")
        self.hw = SimHardware(self.sim)
        self.hw.set_motor("position_raw_py", "position_raw_py", 0.0, 0.0)
        # Starts at rest on its home switch (closed reads
        # False, see set_home): zeroed without moving.
        self.hw.set_digital_input("position_raw_py", "position_raw_py_home", False)

    def teleop(self, enabled=True):
        """Robot enabled (or not) in teleop: the default policy reads the
        teleop lane. Steps so the mode is current before any lane."""
        self.hw.enable(enabled)
        self.hw.set_driver_station(enabled=True, teleop=True)
        self.sim.step(0.06)  # at rest on a switch: zeroed

    def send(self, demand, source="teleop", mode_since_us=None):
        """One target on a lane, stamped like a producer that has drained
        the newest Driver Station packet, or with `mode_since_us`."""
        self.sim.publish_target(PositionRawPyTarget, "position_raw_py", source,
                                mode_since_us=mode_since_us,
                                position_rot=demand)

    def hold(self, demand, seconds, source="teleop"):
        """Republishes a lane every 20 ms, as a producer does."""
        for _ in range(round(seconds / 0.02)):
            self.send(demand, source)
            self.sim.step(0.02)

    def mode(self):
        """What the node asked for (the flush sends neutral for an unset
        motor)."""
        return self.hw.motor_request("position_raw_py", "position_raw_py").mode

    def applied(self):
        """What the gateway applies, after its enable and expiry gates."""
        return self.hw.applied_motor("position_raw_py", "position_raw_py").mode

    def state(self):
        return self.sim.latest(PositionRawPyState, topics.STATE)

    # REQ-SAFE-07
    def test_drives_at_a_live_lane(self):
        self.teleop()
        self.set_home(False)  # zeroed on the switch, it leaves it
        self.hold(1.5, 0.06)
        request = self.hw.applied_motor("position_raw_py", "position_raw_py")
        self.assertEqual(request.mode, talos.Mode.MOTION_MAGIC)
        self.assertEqual(request.demand, 1.5)
        state = self.state()
        self.assertTrue(state.enabled)
        self.assertEqual(state.position_rot, 0.0)
        self.assertTrue(state.connected)

    # REQ-CONTRACT-25: matrix case 2, `state` still published with
    # `enabled` false while disabled. The neutral request is the
    # devices flush's (REQ-SAFE-02, proved in talOS/devices), not this
    # node's evidence.
    def test_disabled_robot_goes_neutral(self):
        self.teleop(enabled=False)
        self.hold(30.0, 0.06)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertEqual(self.applied(), talos.Mode.NEUTRAL)
        self.assertFalse(self.state().enabled)  # state keeps publishing

    # REQ-CONTRACT-25: matrix case 3, the node's `state` still published,
    # `enabled` false, once its lane is stale. The neutral request is the
    # devices flush's (Target::get returns no target: REQ-SAFE-41, proved in
    # talOS/devices), not this node's evidence.
    def test_stale_lane_goes_neutral(self):
        self.teleop()
        self.hold(30.0, 0.04)
        self.assertEqual(self.mode(), talos.Mode.MOTION_MAGIC)
        self.sim.step(0.15)  # past command_timeout_us
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertEqual(self.applied(), talos.Mode.NEUTRAL)
        self.assertFalse(self.state().enabled)  # state keeps publishing

    # Matrix case 4: the node keeps running when its lane is released. The
    # neutral request is the devices flush's (Target::get returns no
    # target: REQ-SAFE-41, proved in talOS/devices), not this node's evidence.
    def test_released_lane_goes_neutral(self):
        self.teleop()
        self.hold(30.0, 0.04)
        self.assertEqual(self.mode(), talos.Mode.MOTION_MAGIC)
        self.sim.release_target(PositionRawPyTarget, "position_raw_py", "teleop")
        self.sim.step(0.04)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)

    # REQ-CONTRACT-25: matrix case 5, the node's `state` published, `enabled`
    # false, while no target was ever sent. The neutral request is the devices
    # flush's (Target::get returns no target: REQ-SAFE-41, proved in
    # talOS/devices), not this node's evidence.
    def test_never_sent_goes_neutral(self):
        self.teleop()
        self.sim.step(0.06)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertFalse(self.state().enabled)

    # Matrix case 6: the node keeps running once the Driver Station view is
    # stale. The neutral request is the devices flush's (Target::get returns no
    # target: REQ-SAFE-41, proved in talOS/devices), not this node's evidence.
    def test_driver_station_loss_goes_neutral(self):
        self.teleop()
        self.hold(30.0, 0.04)
        self.assertEqual(self.mode(), talos.Mode.MOTION_MAGIC)
        self.hw.stop_driver_station()
        self.hold(30.0, 0.16)  # past command_timeout_us: the DS is stale
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)

    # REQ-CONTRACT-25
    def test_disconnected_feedback_goes_neutral(self):
        self.teleop()
        self.hold(30.0, 0.04)
        self.assertEqual(self.mode(), talos.Mode.MOTION_MAGIC)
        self.hw.set_connected("position_raw_py", "position_raw_py", False)
        self.hold(30.0, 0.04)
        # The neighbor's motor keeps the robot enabled (testdata/robot.toml),
        # so the node's own feedback gate is what stops it.
        self.assertTrue(talos.robot_enabled(self.sim.node("position_raw_py")))
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

    # matrix: n/a 9 (the position_raw_py reads no parameters)

    # REQ-SAFE-07: matrix case 10, the current mode's lane drives the node end
    # to end through its Target; the other mode's lane going neutral is the
    # flush's (REQ-SAFE-41).
    def test_other_modes_lane_is_ignored(self):
        self.teleop()
        self.set_home(False)  # until it opens, no farther than HOME_MARGIN_ROT
        self.hold(30.0, 0.04, source="auto")  # autonomous's lane, in teleop
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.sim.release_target(PositionRawPyTarget, "position_raw_py", "auto")
        self.hw.set_driver_station(enabled=True, autonomous=True)
        self.sim.step(0.02)
        self.hold(30.0, 0.04)  # teleop's lane, live, in autonomous
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.hold(1.0, 0.04, source="auto")
        request = self.hw.motor_request("position_raw_py", "position_raw_py")
        self.assertEqual(request.mode, talos.Mode.MOTION_MAGIC)
        self.assertEqual(request.demand, 1.0)

    # Matrix case 11: the node drives again on a target of the current
    # generation. Refusing the earlier one is Target's and the
    # flush's (REQ-SAFE-09 in talOS/devices/target_test.cc, and REQ-SAFE-41).
    def test_earlier_generation_target_is_ignored(self):
        self.teleop()
        self.hold(30.0, 0.04)
        self.assertEqual(self.mode(), talos.Mode.MOTION_MAGIC)
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
        self.assertEqual(self.mode(), talos.Mode.MOTION_MAGIC)

    # matrix: n/a 12 (a Python tick allocates in the interpreter)

    # The node's own homing bounds. The tests above are the node test
    # matrix (docs/TESTING.md §7 in the talos repository) that every
    # actuator node keeps; these guard the rules this node adds. Their
    # times and positions follow the constants in
    # position_raw_py.py: MAX_PUSH_TICKS (50 ticks, 1 s),
    # HOME_MARGIN_ROT (0.15) and MAX_ROT (2.0); change them together.
    # setUp starts the motor at rest on its switch.

    def set_home(self, closed):
        """Normally open to ground behind the input's pull-up: closed reads
        False."""
        self.hw.set_digital_input("position_raw_py", "position_raw_py_home",
                                  not closed)

    def start_off_home(self):
        """Powered on off the switch, unzeroed, and moving when pushed."""
        self.set_home(False)
        self.hw.set_motor("position_raw_py", "position_raw_py", 0.0, -1.0)

    # REQ-DEV-01
    def test_homing_that_never_reaches_the_switch_fails(self):
        self.start_off_home()  # the switch never closes, as with a cut wire
        self.teleop()
        self.hold(1.0, 0.9)
        # Still pushing toward the switch.
        self.assertEqual(self.mode(), talos.Mode.VOLTAGE)
        self.hold(1.0, 0.2)  # past the 1 s bound
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertEqual(self.state().homing, PositionRawPyHoming.FAILED)

    # REQ-DEV-01
    def test_failure_latches_until_disabled(self):
        self.start_off_home()
        self.teleop()
        self.hold(1.0, 1.1)
        self.assertEqual(self.state().homing, PositionRawPyHoming.FAILED)
        self.sim.step(0.15)  # the lane goes stale while the button is held
        self.hold(1.0, 0.2)  # live again: no second push
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertEqual(self.state().homing, PositionRawPyHoming.FAILED)
        self.hw.set_driver_station(teleop=True)  # disabled
        self.sim.step(0.04)
        self.teleop()  # enabled again: a new homing run
        self.hold(1.0, 0.04)
        self.assertEqual(self.mode(), talos.Mode.VOLTAGE)

    # REQ-DEV-01
    def test_stuck_switch_stops_within_the_margin(self):
        self.hw.ideal_motors()
        # Powered on near the top; the switch, stuck closed, zeroes it there.
        self.hw.set_motor("position_raw_py", "position_raw_py", 1.9, 0.0)
        self.teleop()
        for _ in range(10):
            self.hold(1.5, 0.02)
            # Never more than HOME_MARGIN_ROT (0.15) above the false zero.
            demand = self.hw.motor_request("position_raw_py", "position_raw_py").demand
            self.assertLessEqual(demand, 1.9 + 0.15 + 1e-9)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertEqual(self.state().homing, PositionRawPyHoming.FAILED)

        # Recover by disabling, leave the switch and drive past
        # HOME_MARGIN_ROT, then drop the zero (motor reset) at a non-zero
        # position with the switch stuck closed: the second zero must
        # enforce HOME_MARGIN_ROT again.
        self.set_home(False)
        self.hw.set_driver_station(teleop=True)  # disabled
        self.sim.step(0.04)
        self.teleop()
        self.hold(1.0, 0.06)
        self.assertEqual(self.mode(), talos.Mode.MOTION_MAGIC)
        self.assertAlmostEqual(self.state().position_rot, 1.0, places=9)

        self.set_home(True)
        self.hw.reset_device("position_raw_py", "position_raw_py")
        self.sim.step(0.08)  # past masked reset states; re-zeroes on switch
        self.assertEqual(self.state().homing, PositionRawPyHoming.HOMED)
        for _ in range(10):
            self.hold(1.5, 0.02)
            demand = self.hw.motor_request("position_raw_py", "position_raw_py").demand
            self.assertLessEqual(demand, 2.9 + 0.15 + 1e-9)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)
        self.assertEqual(self.state().homing, PositionRawPyHoming.FAILED)

    # REQ-DEV-01: off the switch, a goal past either end of the travel
    # demands that end, MAX_ROT (2.0) or 0.
    def test_goal_outside_the_travel_clamps_to_it(self):
        self.hw.ideal_motors()
        self.teleop()
        self.set_home(False)  # zeroed on the switch, it leaves it
        self.hold(5.0, 0.06)
        demand = self.hw.motor_request("position_raw_py", "position_raw_py").demand
        self.assertEqual(demand, 2.0)
        self.hold(-1.0, 0.06)
        demand = self.hw.motor_request("position_raw_py", "position_raw_py").demand
        self.assertEqual(demand, 0.0)

    # REQ-DEV-01: the switch zeroes the motor only once it moves slower
    # than REST_RPS (0.05).
    def test_zero_waits_for_rest(self):
        self.hw.set_motor("position_raw_py", "position_raw_py", 0.0, -1.0)  # on it, moving
        self.teleop()
        self.hold(1.0, 0.06)
        self.assertEqual(self.state().homing, PositionRawPyHoming.HOMING)
        self.assertEqual(self.mode(), talos.Mode.NEUTRAL)  # no push on it
        self.hw.set_motor("position_raw_py", "position_raw_py", 0.0, 0.0)  # at rest
        self.hold(1.0, 0.06)
        self.assertEqual(self.state().homing, PositionRawPyHoming.HOMED)
        self.assertEqual(self.mode(), talos.Mode.MOTION_MAGIC)


if __name__ == "__main__":
    unittest.main()
