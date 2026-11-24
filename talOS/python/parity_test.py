"""talos.parity on two toy nodes that differ on one tick of a multi-tick
step: the comparison a `# matrix: parity` claim rests on
(docs/TESTING.md §7)."""

import os
import unittest
from unittest import mock

from demo.Status import Status
from talos import _abi, parity

ROBOT_TOML = """
[robot]
period_us = 5000
command_timeout_us = 100000

[subsystems.arm]
"""

TOPIC = "/arm/status"


def _config():
    path = os.path.join(os.environ["TEST_TMPDIR"], "parity_robot.toml")
    with open(path, "w", encoding="utf-8") as f:
        f.write(ROBOT_TOML)
    return path


class Steady:
    """Reports height 1.0 on every tick."""

    def __init__(self, node):
        self.node = node
        self.status = node.publish(Status, TOPIC)

    def update(self):
        self.status.send(height_rot=1.0, at_goal=True, tick_count=0)


class Glitch(Steady):
    """Steady, except height 2.0 on its third tick."""

    def update(self):
        height = 2.0 if self.node.tick == 2 else 1.0
        self.status.send(height_rot=height, at_goal=True, tick_count=0)


class ParityTest(unittest.TestCase):
    # REQ-CONTRACT-14: a divergence on a middle tick of a step fails the
    # parity run, though the two nodes agree again by the step's end.
    def test_a_divergence_inside_a_step_fails(self):
        scenario = [("ten ticks", parity.wait(0.05))]
        with self.assertRaises(AssertionError) as raised:
            parity.assert_same_behaviour(Steady, Glitch, _config(), "arm",
                                         scenario, topics=[(Status, TOPIC)])
        message = str(raised.exception)
        self.assertIn("'ten ticks'", message)
        self.assertIn(TOPIC, message)
        self.assertIn("tick 3", message)

    # REQ-CONTRACT-14: an action that moves the clock other than through
    # Rig.advance skips the ticks it crosses, so the run fails that step
    # rather than compare only its end (where these two nodes agree), also
    # when the action advances through the rig afterwards.
    def test_a_step_that_moves_the_clock_itself_fails(self):
        def step_then_advance(rig):
            rig.sim.step(0.05)
            rig.advance(0.01)

        for name, action in (("sim.step", lambda rig: rig.sim.step(0.05)),
                             ("sim.step, then advance", step_then_advance)):
            with self.subTest(name):
                with self.assertRaises(AssertionError) as raised:
                    parity.assert_same_behaviour(
                        Steady, Glitch, _config(), "arm",
                        [("warm up", parity.wait(0.005)),
                         ("own step", action)],
                        topics=[(Status, TOPIC)])
                message = str(raised.exception)
                self.assertIn("'own step'", message)
                self.assertIn("Rig.advance", message)

    # REQ-CONTRACT-14: Rig.advance rejects negative durations instead of
    # silently doing nothing.
    def test_advance_rejects_negative_seconds(self):
        for bad in (-0.005, -1e-12, -1):
            with self.subTest(seconds=bad):
                with self.assertRaises(ValueError):
                    parity.assert_same_behaviour(
                        Steady, Steady, _config(), "arm",
                        [("negative", parity.wait(bad))],
                        topics=[(Status, TOPIC)])

        rig = parity.Rig(Steady, _config(), "arm", lambda _: [])
        rig.advance(0.01)
        before = rig.sim.now_ns
        with self.assertRaises(ValueError):
            rig.advance(-0.005)
        self.assertEqual(rig.sim.now_ns, before)

    # REQ-CONTRACT-14: a node reporting period_ns == 0 advances directly to the
    # target instant without ZeroDivisionError and still compares at step end.
    def test_event_driven_node_with_zero_period(self):
        class EventOk:
            def __init__(self, node):
                self.status = node.publish(Status, TOPIC)
                node.every(0.01, self.tick)

            def tick(self):
                self.status.send(height_rot=1.0, at_goal=True, tick_count=1)

        class EventDiff(EventOk):
            def tick(self):
                self.status.send(height_rot=2.0, at_goal=True, tick_count=1)

        with mock.patch.object(_abi.lib, "talos_node_period_ns",
                               return_value=0):
            rig = parity.Rig(EventOk, _config(), "arm", lambda r: [r.sim.now_ns])
            self.assertEqual(rig._period_ns, 0)
            rig.advance(0.025)
            self.assertEqual(rig.sim.now_ns, 25_000_000)
            self.assertEqual(rig.ticks, [])

            parity.assert_same_behaviour(
                EventOk, EventOk, _config(), "arm",
                [("wait", parity.wait(0.025), parity.require_published())],
                topics=[(Status, TOPIC)])

            with self.assertRaises(AssertionError):
                parity.assert_same_behaviour(
                    EventOk, EventDiff, _config(), "arm",
                    [("wait", parity.wait(0.025))],
                    topics=[(Status, TOPIC)])

    # REQ-CONTRACT-14: Rig.advance steps exact integer nanoseconds so a step
    # whose float64 round-trip would overshoot a boundary never misses a tick.
    def test_advance_steps_exact_integer_nanoseconds(self):
        class Quiet:
            def __init__(self, node):
                self.node = node

        rig = parity.Rig(Quiet, _config(), "arm", lambda r: [r.sim.now_ns])
        # Destroy SimHardware so stepping 50+ days does not run ~8e8 ticks.
        del rig.hw
        # (4_378_605_124_704_423 / 1e9) * 1e9 rounds up to ...424 in float64.
        boundary_ns = 4_378_605_124_704_423
        rig._period_ns = boundary_ns
        rig.advance(4_378_606)
        self.assertEqual([at for at, _ in rig.ticks], [boundary_ns])
        self.assertEqual(rig.sim.now_ns, 4_378_606_000_000_000)


if __name__ == "__main__":
    unittest.main()
