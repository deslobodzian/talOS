"""talos.parity on two toy nodes that differ on one tick of a multi-tick
step: the comparison a `# matrix: parity` claim rests on
(docs/TESTING.md §7)."""

import os
import unittest

from demo.Status import Status
from talos import parity

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


if __name__ == "__main__":
    unittest.main()
