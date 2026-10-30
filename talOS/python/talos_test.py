"""The talos package, end to end through libtalos_node on talos.testing.Sim."""

import enum
import unittest

import talos
from talos.testing import Sim, SimHardware, Stick
from demo.Goal import Goal
from demo.Phase import Phase
from demo.Report import Report
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

    def test_disconnected_device_reports_it(self):
        sim = Sim.from_toml(ROBOT_TOML)
        lift = sim.add(Lift, "lift")
        hw = SimHardware(sim)
        hw.set_motor("lift", "lift", 0.75, 2.0)
        hw.set_connected("lift", "lift", False)
        sim.step(0.02)
        self.assertFalse(lift.lift.connected)
        self.assertEqual((lift.lift.position, lift.lift.velocity), (0.0, 0.0))

        hw.set_connected("lift", "lift", True)
        sim.step(0.02)
        self.assertTrue(lift.lift.connected)
        self.assertEqual(lift.lift.position, 0.75)
        with self.assertRaisesRegex(talos.TalosError, "'elbow'"):
            hw.set_connected("lift", "elbow", False)


class Pilot:
    """Reads the Driver Station, counting packets."""

    def __init__(self, node):
        self.packets = 0
        self.ds = talos.DriverStation(node, on_packet=self.count)

    def count(self):
        self.packets += 1


class DriverStationTest(unittest.TestCase):
    def test_reads_what_sim_hardware_forwards(self):
        sim = Sim.from_toml(ROBOT_TOML)
        pilot = sim.add(Pilot, "planner")
        hw = SimHardware(sim)
        sim.step(0.02)
        ds = pilot.ds
        self.assertEqual(pilot.packets, 0)  # nothing set: nothing forwarded
        self.assertFalse(ds.connected)
        self.assertFalse(ds.joystick(0).connected)

        hw.set_driver_station(
            enabled=True, teleop=True, alliance=talos.Alliance.RED,
            station=2, match_time=12.5,
            joysticks=[Stick(axes=(0.5, -0.25), buttons=(1, 3), povs=(90,),
                             name="pad")])
        sim.step(0.02)
        self.assertGreater(pilot.packets, 0)
        self.assertTrue(ds.connected)
        self.assertTrue(ds.enabled)
        self.assertTrue(ds.teleop)
        self.assertFalse(ds.autonomous or ds.test or ds.estop)
        self.assertFalse(ds.fms_attached)
        self.assertEqual(ds.alliance, talos.Alliance.RED)
        self.assertEqual(ds.station, 2)
        self.assertEqual(ds.match_time, 12.5)
        stick = ds.joystick(0)
        self.assertTrue(stick.connected)
        self.assertEqual((stick.axis_count, stick.button_count,
                          stick.pov_count), (2, 3, 1))
        self.assertEqual((stick.axis(0), stick.axis(1), stick.axis(2)),
                         (0.5, -0.25, 0.0))
        self.assertEqual([stick.button(n) for n in range(0, 5)],
                         [False, True, False, True, False])
        self.assertEqual((stick.pov(), stick.pov(1)), (90, -1))
        self.assertEqual(stick.name, "pad")
        self.assertFalse(ds.joystick(1).connected)
        with self.assertRaises(IndexError):
            ds.joystick(6)

        # A DS that is not attached reads as disabled, whatever it says.
        hw.set_driver_station(enabled=True, teleop=True, attached=False,
                              joysticks=[Stick(axes=(1.0,))])
        sim.step(0.02)
        self.assertFalse(ds.connected or ds.enabled or ds.teleop)
        self.assertFalse(ds.joystick(0).connected)
        self.assertEqual(ds.joystick(0).axis(0), 0.0)

        # Nor does a silent one stay enabled.
        hw.set_driver_station(enabled=True, teleop=True)
        sim.step(0.02)
        self.assertTrue(ds.enabled)
        del hw
        sim.step(0.15)  # past [robot] command_timeout_us
        self.assertFalse(ds.connected or ds.enabled)


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
        self.assertRegex(repr(stepper.sm),
                         r"^StateMachine\(MOVING, previous=IDLE, "
                         r"time_in_state=0\.015\d*s\)$")
        with self.assertRaisesRegex(ValueError,
                                    "Stepper.State has no state 2; it has "
                                    "IDLE, MOVING"):
            stepper.sm.go(2)

    def test_flatc_enum_states_are_named(self):
        class Mover:
            def __init__(self, node):
                self.sm = talos.StateMachine(node, Phase.IDLE, states=Phase)

            def update(self):
                self.sm.go(Phase.HOLDING)

        sim = Sim.from_toml(ROBOT_TOML)
        mover = sim.add(Mover, "planner")
        sim.step(0.015)
        self.assertEqual(mover.sm.state, Phase.HOLDING)
        self.assertIs(type(mover.sm.state), int)  # flatc values stay ints
        self.assertRegex(repr(mover.sm),
                         r"^StateMachine\(HOLDING, previous=IDLE, ")
        with self.assertRaisesRegex(
                ValueError, "Phase has no state 3; it has IDLE, MOVING, "
                            "HOLDING"):
            mover.sm.go(3)
        with self.assertRaisesRegex(ValueError, "Phase has no state 7"):
            talos.StateMachine(sim.node("planner"), 7, states=Phase)


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

    def test_enum_fields_read_as_ints_and_repr_as_names(self):
        sim = Sim.from_toml(ROBOT_TOML)
        sim.publish(Report, "/report", phase=Phase.HOLDING, height_rot=1.5)
        report = sim.latest(Report, "/report")
        self.assertEqual(report.phase, Phase.HOLDING)
        self.assertIs(type(report.phase), int)
        self.assertEqual(repr(report), "Report(phase=HOLDING, height_rot=1.5)")
        sim.publish(Report, "/report", phase=9)  # not a Phase
        self.assertEqual(repr(sim.latest(Report, "/report")),
                         "Report(phase=9, height_rot=0.0)")

    def test_byte_topics_and_relative_topic_remapping(self):
        class Telemeter:
            def __init__(self, node):
                self.node = node
                self.goal = node.subscribe(Goal, "goal")
                self.status = node.publish(Status, "status")
                self.frames = node.publish_bytes("frames", 6, slots=16,
                                                 drop_newest=True,
                                                 external=True)
                self.sent = []

            def update(self):
                if self.goal.has_value:
                    self.status.send(height_rot=self.goal.value.height_rot,
                                     at_goal=True, tick=self.node.tick)
                payload = bytearray(range(self.node.tick + 1))
                self.sent.append(self.frames.send(memoryview(payload)))

        sim = Sim.from_toml("""
[robot]
period_us = 10000
command_timeout_us = 100000

[subsystems.planner.motors.m]
can_id = 1

[subsystems.follower.remap]
goal = "/planner/goal"
"/follower/status" = "/remapped/status"
frames = "/remapped/frames"
""")
        telemeter = sim.add(Telemeter, "planner")
        remapped = sim.add(Telemeter, "follower")
        self.assertIsNone(sim.latest_bytes("/planner/frames"))

        sim.publish(Goal, "/planner/goal", height_rot=3.5)
        sim.step(0.035)  # ticks 0, 1, 2
        self.assertEqual(sim.latest(Status, "/planner/status").height_rot, 3.5)
        self.assertEqual(sim.latest(Status, "/remapped/status").height_rot, 3.5)
        self.assertEqual(sim.latest_bytes("/planner/frames"), b"\x00\x01\x02")
        self.assertEqual(sim.latest_bytes("/remapped/frames"), b"\x00\x01\x02")
        self.assertEqual(telemeter.sent, [True, True, True])

        # Oversize returns False and preserves the last valid frame; empty works.
        self.assertFalse(telemeter.frames.send(b"1234567"))
        self.assertEqual(sim.latest_bytes("/planner/frames"), b"\x00\x01\x02")
        self.assertTrue(telemeter.frames.send(b""))
        self.assertEqual(sim.latest_bytes("/planner/frames"), b"")
        self.assertIsNotNone(remapped)
        with self.assertRaises(TypeError):
            telemeter.frames.send("not bytes")

        class BadBytes:
            def __init__(self, node):
                node.publish_bytes("frames", 0)

        with self.assertRaises(talos.TalosError):
            sim.add(BadBytes, "planner")


if __name__ == "__main__":
    unittest.main()
