"""The talos package, end to end through libtalos_node on talos.testing.Sim."""

import enum
import unittest

import talos
from talos.testing import Sim, SimHardware
from demo.Goal import Goal
from demo.Status import Status

ROBOT_TOML = """
[robot]
period_us = 10000
command_timeout_us = 100000

[subsystems.planner]
height_rot = 1.25

[subsystems.follower]
gain = 2.0
mode = "fast"
retries = 3
verbose = true

[subsystems.lift.motors.lift]
can_id = 1
"""


class Planner:
    """Publishes a goal every tick."""

    def __init__(self, node):
        self.node = node
        self.height = node.param("height_rot", 0.0)
        self.goal = node.publish(Goal, "/planner/goal")

    def update(self):
        self.goal.send(height_rot=self.height, enabled=True,
                       count=self.node.tick)


class Follower:
    """Reports the planner's goal, scaled by its gain."""

    def __init__(self, node):
        self.node = node
        self.gain = node.param("gain")
        self.received = []
        self.goal = node.subscribe(Goal, "/planner/goal",
                                   callback=self.received.append)
        self.status = node.publish(Status, "/follower/status")

    def update(self):
        goal = self.goal.value
        if goal is not None:
            self.status.send(height_rot=goal.height_rot * self.gain,
                             at_goal=goal.enabled, tick=self.node.tick)


class Lift:
    """One motion-magic motor, always asked for 1.5 rotations."""

    def __init__(self, node):
        self.node = node
        self.lift = talos.Motor(node, "lift")
        self.enabled = None

    def update(self):
        self.enabled = talos.robot_enabled(self.node)
        self.lift.set_motion_magic(1.5, ff_volts=0.25, slot=2)


class Stepper:
    class State(enum.Enum):
        IDLE = 1
        MOVING = 2

    def __init__(self, node):
        self.node = node
        self.sm = talos.StateMachine(node, Stepper.State.IDLE)
        self.seen = []  # (tick, state, just_entered) at each update

    def update(self):
        self.seen.append((self.node.tick, self.sm.state, self.sm.just_entered))
        if self.node.tick == 2:
            self.sm.go(Stepper.State.MOVING)


class TwoNodesTest(unittest.TestCase):
    def test_python_nodes_talk_over_a_topic(self):
        sim = Sim.from_toml(ROBOT_TOML)
        sim.add(Planner, "planner")
        follower = sim.add(Follower, "follower")
        sim.step(0.055)

        status = sim.latest(Status, "/follower/status")
        self.assertEqual(status.height_rot, 2.5)
        self.assertTrue(status.at_goal)
        self.assertEqual(status.tick, 4)  # ticks 0..4 ran by 50 ms
        self.assertEqual([g.count for g in follower.received], [0, 1, 2, 3, 4])
        self.assertTrue(follower.goal.fresh())
        self.assertFalse(follower.goal.fresh(max_age=0.001))
        self.assertLess(follower.goal.age, 0.01)
        self.assertEqual(sim.node("follower").period, 0.01)
        self.assertAlmostEqual(sim.now, 0.055)


class DevicesTest(unittest.TestCase):
    def test_neutral_when_disabled_and_requests_when_enabled(self):
        sim = Sim.from_toml(ROBOT_TOML)
        lift = sim.add(Lift, "lift")
        hw = SimHardware(sim)
        hw.set_motor("lift", "lift", 0.75, 0.0)
        sim.step(0.03)
        self.assertFalse(lift.enabled)
        self.assertEqual(hw.motor_request("lift", "lift").mode,
                         talos.Mode.NEUTRAL)
        self.assertTrue(lift.lift.connected)
        self.assertEqual(lift.lift.position, 0.75)

        hw.enable()
        sim.step(0.03)
        self.assertTrue(lift.enabled)
        self.assertEqual(hw.motor_request("lift", "lift"),
                         (talos.Mode.MOTION_MAGIC, 1.5, 0.25, 2))

    def test_unknown_device_names_it(self):
        class Wrong:
            def __init__(self, node):
                talos.Motor(node, "elbow")

        sim = Sim.from_toml(ROBOT_TOML)
        with self.assertRaisesRegex(talos.TalosError, "'elbow'"):
            sim.add(Wrong, "lift")


class StateMachineTest(unittest.TestCase):
    def test_just_entered_and_time_in_state(self):
        sim = Sim.from_toml(ROBOT_TOML)
        stepper = sim.add(Stepper, "planner")
        sim.step(0.045)
        idle, moving = Stepper.State.IDLE, Stepper.State.MOVING
        self.assertEqual(stepper.seen, [
            (0, idle, True), (1, idle, False), (2, idle, False),
            (3, moving, True)])
        self.assertEqual(stepper.sm.previous, idle)
        self.assertTrue(stepper.sm.is_(moving))
        # Entered at the 30 ms tick; now is 45 ms.
        self.assertAlmostEqual(stepper.sm.time_in_state, 0.015)


class ParamTest(unittest.TestCase):
    def test_fallback_and_missing(self):
        seen = {}

        class Reader:
            def __init__(self, node):
                seen["gain"] = node.param("gain", 1.0)
                seen["mode"] = node.param("mode")
                seen["retries"] = node.param("retries", 0)
                seen["verbose"] = node.param("verbose")
                seen["absent"] = node.param("absent", 7.5)

        sim = Sim.from_toml(ROBOT_TOML)
        sim.add(Reader, "follower")
        self.assertEqual(seen, {"gain": 2.0, "mode": "fast", "retries": 3,
                                "verbose": True, "absent": 7.5})

        class Missing:
            def __init__(self, node):
                node.param("absent")

        with self.assertRaisesRegex(talos.TalosError, "'absent'"):
            sim.add(Missing, "planner")

        class Mistyped:
            def __init__(self, node):
                node.param("height_rot", 0)

        with self.assertRaisesRegex(talos.TalosError, "must be an integer"):
            sim.add(Mistyped, "planner")


class MessageTest(unittest.TestCase):
    def test_views_and_send_round_trip(self):
        class Echo:
            def __init__(self, node):
                self.goal = node.subscribe(Goal, "/echo/in")
                self.out = node.publish(Goal, "/echo/out")

            def update(self):
                if self.goal.updated:
                    self.out.send(self.goal.value)  # re-send the view

        sim = Sim.from_toml(ROBOT_TOML)
        echo = sim.add(Echo, "planner")
        self.assertIsNone(echo.goal.value)
        sim.publish(Goal, "/echo/in", count=7)  # the rest is 0/False
        sim.step(0.015)

        goal = echo.goal.value
        self.assertEqual((goal.height_rot, goal.enabled, goal.count),
                         (0.0, False, 7))
        self.assertEqual(sim.latest(Goal, "/echo/out"), goal)
        self.assertEqual(repr(goal),
                         "Goal(height_rot=0.0, enabled=False, count=7)")
        with self.assertRaisesRegex(AttributeError, "height_rot, enabled"):
            goal.height
        with self.assertRaisesRegex(TypeError, "no field heigth_rot"):
            sim.publish(Goal, "/echo/in", heigth_rot=1.0)


if __name__ == "__main__":
    unittest.main()
