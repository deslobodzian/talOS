"""The talos package, end to end through libtalos_node on talos.testing.Sim."""

import ctypes
import enum
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import textwrap
import threading
from datetime import timedelta
import unittest
from unittest import mock

from python.runfiles import runfiles
import talos
from talos import _abi
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

[subsystems.vision]
cameras = ["front", "back"]
offsets_m = [0.5, 1, -2.25]
no_cameras = []
mixed = [1.0, "back"]
flags = [true]
exposure_s = 0.01

[subsystems.lift.motors.lift]
can_id = 1
[subsystems.lift.targets]
teleop = ["align", "teleop"]
"""


class Planner:
    """Publishes a goal every tick."""

    def __init__(self, node):
        self.node = node
        self.height = node.param("height_rot", 0.0)
        self.goal = node.publish(Goal, "/planner/goal")

    def update(self):
        self.goal.send(height_rot=self.height, enabled=True,
                       tick_count=self.node.tick)


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
                             at_goal=goal.enabled, tick_count=self.node.tick)


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
        self.assertEqual(status.tick_count, 4)  # ticks 0..4 ran by 50 ms
        self.assertEqual([g.tick_count for g in follower.received],
                         [0, 1, 2, 3, 4])
        self.assertTrue(follower.goal.fresh())
        self.assertFalse(follower.goal.fresh(max_age=0.001))
        self.assertLess(follower.goal.age, 0.01)
        self.assertEqual(sim.node("follower").period, 0.01)
        self.assertAlmostEqual(sim.now, 0.055)


class RunTest(unittest.TestCase):
    def test_run_freezes_the_built_node_out_of_collection(self):
        # gc.freeze is process-wide: run in a child process.
        probe = textwrap.dedent("""
            import gc, sys, tempfile
            import talos

            built = []

            class Probe:
                def __init__(self, node):
                    assert gc.get_freeze_count() == 0
                    built.append(self)

            with tempfile.NamedTemporaryFile("w", suffix=".toml") as config:
                config.write(sys.argv[1])
                config.flush()
                code = talos.run(Probe, "planner", "//talOS/python:probe",
                                 ["probe", "--describe", "--config",
                                  config.name])
            assert code == 0, code
            assert gc.get_freeze_count() > 0
            assert not any(o is built[0] for o in gc.get_objects())
        """)
        result = subprocess.run([sys.executable, "-c", probe, ROBOT_TOML],
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


class BorrowedLifetimeTest(unittest.TestCase):
    def test_wrappers_keep_sim_alive_until_native_teardown(self):
        # A bad lifetime implementation may segfault: isolate the probe and
        # check collection tears down the Sim after the last wrapper goes.
        probe = textwrap.dedent("""
            import gc
            import weakref
            import talos
            from talos.testing import Sim
            from demo.Goal import Goal

            class Endpoints:
                def __init__(self, node):
                    self.node = node
                    self.sub = node.subscribe(Goal, "/probe/input")
                    self.pub = node.publish(Goal, "/probe/output")
                    self.bytes = node.publish_bytes("/probe/bytes", 64)
                    self.motor = talos.Motor(node, "m")
                    self.hardware = talos.HardwareLink(node)
                    self.ds = talos.DriverStation(node)
                    self.target = talos.Target(node, Goal)
                    self.lane = talos.TargetLane(node, Goal, "probe", "teleop")
                    self.sm = talos.StateMachine(node, 0)

            config = '''
            [robot]
            period_us = 1000
            [subsystems.probe.motors.m]
            can_id = 1
            [subsystems.probe.targets]
            teleop = ["teleop"]
            '''
            for kind in ("node", "lookup", "sub", "pub", "bytes", "motor",
                         "hardware", "ds", "target", "lane", "sm"):
                sim = Sim.from_toml(config)
                obj = sim.add(Endpoints, "probe")
                wrapper = sim.node("probe") if kind == "lookup" else getattr(obj, kind)
                sim.step(0.002)
                owner = weakref.ref(sim)
                del obj, sim
                gc.collect()
                assert owner() is not None, kind
                if kind in ("node", "lookup"):
                    assert wrapper.name == "probe"
                elif kind == "sub":
                    assert not wrapper.has_value
                elif kind == "pub":
                    wrapper.send(height_rot=0.0, enabled=False, tick_count=1)
                elif kind == "bytes":
                    assert wrapper.send(b"offline")
                elif kind == "motor":
                    assert not wrapper.connected
                elif kind == "hardware":
                    assert not wrapper.fresh()
                elif kind == "ds":
                    assert not wrapper.enabled
                elif kind == "target":
                    assert wrapper.get() is None
                elif kind == "lane":
                    wrapper.release()
                elif kind == "sm":
                    assert wrapper.state == 0
                del wrapper
                gc.collect()
                assert owner() is None, kind
        """)
        result = subprocess.run([sys.executable, "-c", probe],
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


class SubscriptionErrorsTest(unittest.TestCase):
    def test_abi_errors_are_not_values_or_true_flags(self):
        sim = Sim.from_toml(ROBOT_TOML)
        follower = sim.add(Follower, "follower")
        subscription = follower.goal
        self.assertIsNone(subscription.value)
        with mock.patch.object(talos.lib, "talos_last_error",
                               return_value=b"subscription failed"):
            for symbol, read in (
                ("talos_subscription_has_value",
                 lambda: subscription.has_value),
                ("talos_subscription_fresh", lambda: subscription.fresh()),
                ("talos_subscription_updated", lambda: subscription.updated),
            ):
                with self.subTest(symbol=symbol):
                    with mock.patch.object(talos.lib, symbol, return_value=-1):
                        with self.assertRaises(talos.TalosError):
                            read()
            with mock.patch.object(talos.lib, "talos_subscription_has_value",
                                   return_value=1):
                with mock.patch.object(talos.lib, "talos_subscription_value",
                                       return_value=None):
                    with self.assertRaises(talos.TalosError):
                        _ = subscription.value


class CallbackErrorTest(unittest.TestCase):
    def test_a_callback_exception_is_raised_only_on_its_thread(self):
        # Like the C error string, a callback's exception belongs to the
        # thread whose C call ran it: another thread's failed call must not
        # raise it, nor take it from the call it failed.
        def boom(_):
            raise ValueError("boom")

        self.assertEqual(_abi.callback(boom)(None), 1)
        raised = []

        def other_thread():
            try:
                _abi.fail()
            except BaseException as error:  # noqa: B902 - inspected below
                raised.append(error)

        thread = threading.Thread(target=other_thread)
        thread.start()
        thread.join()
        with self.assertRaises(ValueError):
            _abi.fail()
        self.assertEqual([type(error) for error in raised], [talos.TalosError])


class AbiLayoutTest(unittest.TestCase):
    def test_ctypes_mirrors_match_the_layouts_node_api_pins(self):
        # node_api.cc pins each ABI struct's size and field offsets with
        # static_asserts; the ctypes mirror must read the same bytes.
        source = Path(runfiles.Create().Rlocation(
            os.environ["TALOS_ABI_SOURCE"])).read_text()
        sizes = {name: int(size) for name, size in re.findall(
            r"static_assert\(\s*sizeof\((Talos\w+)\)\s*==\s*(\d+)\s*\);",
            source)}
        offsets = {}
        for name, field, offset in re.findall(
                r"static_assert\(\s*offsetof\((Talos\w+),\s*(\w+)\)\s*==\s*"
                r"(\d+)\s*\);", source):
            offsets.setdefault(name, {})[field] = int(offset)
        mirrors = {"Talos" + cls.__name__: cls for cls in vars(_abi).values()
                   if isinstance(cls, type)
                   and issubclass(cls, ctypes.Structure)}
        self.assertEqual(sorted(mirrors), sorted(sizes))
        self.assertEqual(sorted(mirrors), sorted(offsets))
        for name, cls in mirrors.items():
            with self.subTest(struct=name):
                self.assertEqual(ctypes.sizeof(cls), sizes[name])
                self.assertEqual({field: getattr(cls, field).offset
                                  for field, _ in cls._fields_},
                                 offsets[name])


class DevicesTest(unittest.TestCase):
    def test_applied_output_accessors_propagate_device_errors(self):
        sim = Sim.from_toml(ROBOT_TOML)
        hw = SimHardware(sim)
        for accessor in (hw.applied_motor, hw.applied_digital_output,
                         hw.applied_pwm):
            with self.subTest(accessor=accessor.__name__):
                with self.assertRaises(talos.TalosError):
                    accessor("lift", "missing")

    def test_signal_acquisition_times_preserve_unequal_ages(self):
        class Signals:
            def __init__(self, node):
                self.motor = talos.Motor(node, "lift")
                self.encoder = talos.CANcoder(node, "cc")
                self.gyro = talos.Gyro(node, "gyro")

        toml = ROBOT_TOML + """
[subsystems.lift.sensors.cc]
type = "CANcoder"
can_id = 2
[subsystems.lift.sensors.gyro]
type = "Pigeon2"
can_id = 3
"""
        sim = Sim.from_toml(toml)
        signals = sim.add(Signals, "lift")
        node = sim.node("lift")
        hw = SimHardware(sim)
        hw.set_gateway_clock(offset=2)
        hw.set_motor("lift", "lift", 3, 2, 1000, 3000)
        hw.set_cancoder("lift", "cc", 0.2, 0.3, 2000, 4000)
        hw.set_gyro("lift", "gyro", 0.75, -0.5, 5000, 6000)
        sim.step(0.02)
        self.assertIsNone(signals.motor.position_time)
        sim.step(0.08)
        sampled = talos.hardware_sample_time_ns(node)
        self.assertIsInstance(sampled, int)
        self.assertFalse(talos.robot_enabled(node))
        for actual, age in (
                (signals.motor.position_time, 1000),
                (signals.motor.velocity_time, 3000),
                (signals.encoder.position_time, 2000),
                (signals.encoder.velocity_time, 4000),
                (signals.gyro.yaw_time, 5000),
                (signals.gyro.yaw_rate_time, 6000)):
            self.assertEqual(actual, sampled - age * 1000)
        hw.set_motor("lift", "lift", 3, 2, 0xffffffff, 0)
        hw.set_cancoder("lift", "cc", 0.2, 0.3, 0, 0xffffffff)
        hw.set_gyro("lift", "gyro", 0.75, -0.5, 0xffffffff, 0)
        sim.step(0.02)
        self.assertIsNone(signals.motor.position_time)
        self.assertIsNotNone(signals.motor.velocity_time)
        self.assertIsNone(signals.encoder.velocity_time)
        self.assertIsNone(signals.gyro.yaw_time)
        hw.set_connected("lift", "lift", False)
        sim.step(0.02)
        self.assertIsNone(signals.motor.velocity_time)
        del hw
        sim.step(0.15)
        self.assertIsNone(signals.encoder.position_time)
        self.assertIsNone(signals.gyro.yaw_rate_time)

    def test_limit_switches_read_only_from_a_connected_sample(self):
        sim = Sim.from_toml(ROBOT_TOML)
        lift = sim.add(Lift, "lift").lift
        hw = SimHardware(sim)
        hw.set_limit_switches("lift", "lift", forward=True)
        sim.step(0.02)
        self.assertEqual((lift.forward_limit, lift.reverse_limit),
                         (True, False))
        hw.set_limit_switches("lift", "lift", reverse=True)
        sim.step(0.02)
        self.assertEqual((lift.forward_limit, lift.reverse_limit),
                         (False, True))
        del hw
        sim.step(0.15)  # past [robot] command_timeout_us
        self.assertFalse(lift.connected)
        self.assertFalse(lift.reverse_limit)

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

    def test_motor_command_outside_update_raises(self):
        # The flush after each update() sends neutral for every motor that
        # update() did not set, so a command from every() would pulse.
        class Pulser:
            def __init__(self, node):
                self.lift = talos.Motor(node, "lift")
                node.every(0.02, lambda: self.lift.set_voltage(6.0))

            def update(self):
                pass

        sim = Sim.from_toml(ROBOT_TOML)
        sim.add(Pulser, "lift")
        with self.assertRaisesRegex(talos.TalosError,
                                    r"motor 'lift'.*outside update\(\)"):
            sim.step(0.05)

    def test_hardware_link_freshness_and_age(self):
        class Watcher:
            def __init__(self, node):
                self.link = talos.HardwareLink(node)

        sim = Sim.from_toml(ROBOT_TOML)
        watcher = sim.add(Watcher, "lift")
        link = watcher.link
        self.assertFalse(link.fresh())
        self.assertFalse(link.fresh(100))
        self.assertEqual(link.age, math.inf)
        self.assertIsNone(link.sample_time)
        hw = SimHardware(sim)
        sim.step(0.03)
        self.assertFalse(talos.robot_enabled(sim.node("lift")))
        self.assertTrue(link.fresh())
        self.assertIsInstance(link.age, float)
        self.assertGreaterEqual(link.age, 0)
        self.assertLess(link.age, 0.1)
        self.assertFalse(link.fresh(0))
        self.assertFalse(link.fresh(link.age))
        self.assertTrue(link.fresh(link.age + 1e-9))
        self.assertTrue(link.fresh(timedelta(seconds=1)))
        del hw
        sim.step(0.15)
        self.assertFalse(link.fresh())
        # A queued final state may drain after the publisher is gone.
        self.assertGreaterEqual(link.age, 0.1)
        self.assertTrue(link.fresh(1))
        self.assertFalse(link.fresh(timedelta(milliseconds=100)))


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


class Aimer:
    """Records what its Target selected on each update()."""

    def __init__(self, node):
        self.target = talos.Target(node, Goal)
        self.seen = None
        self.source = ""

    def update(self):
        self.seen = self.target.get()
        self.source = self.target.source


class Driver:
    """Writes the lift's teleop lane while `talking`."""

    def __init__(self, node):
        self.lane = talos.TargetLane(node, Goal, "lift", "teleop")
        self.talking = True

    def update(self):
        if self.talking:
            self.lane.send(height_rot=0.5, enabled=True, tick_count=7)
        else:
            self.lane.release()


class TargetTest(unittest.TestCase):
    """Target/TargetLane through Python; selection is target_test.cc's."""

    def setUp(self):
        self.sim = Sim.from_toml(ROBOT_TOML)
        self.lift = self.sim.add(Aimer, "lift")
        self.hw = SimHardware(self.sim)
        self.hw.enable()
        self.mode(enabled=True, teleop=True)

    def mode(self, **flags):
        self.hw.set_driver_station(**flags)
        self.sim.step(0.02)

    def hold(self, source, height, seconds=0.03):
        for _ in range(round(seconds / 0.01)):
            self.sim.publish_target(Goal, "lift", source, height_rot=height,
                                    enabled=True, tick_count=0)
            self.sim.step(0.01)

    def seen(self):
        return None if self.lift.seen is None else self.lift.seen.height_rot

    def test_teleop_drives_and_never_sent_is_none(self):
        self.sim.step(0.05)
        self.assertIsNone(self.seen())
        self.assertEqual(self.lift.source, "")
        self.hold("teleop", 1.5)
        self.assertEqual(self.seen(), 1.5)
        self.assertEqual(self.lift.source, "teleop")

    def test_stale_lane_is_none_at_publication_timeout(self):
        self.hold("teleop", 1.0)
        # hold() returns 10 ms after its final publication: 80 ms more
        # leaves the target 90 ms old, just inside the 100 ms timeout.
        self.sim.step(0.08)
        self.assertEqual(self.seen(), 1.0)
        self.sim.step(0.01)
        self.assertIsNone(self.seen())

    def test_robot_disabled_and_driver_station_loss_are_none(self):
        self.hold("teleop", 1.0)
        self.hw.enable(False)
        self.hold("teleop", 1.0)
        self.assertIsNone(self.seen())
        self.hw.enable()
        self.hold("teleop", 1.0)
        self.assertEqual(self.seen(), 1.0)
        self.hw.stop_driver_station()
        self.hold("teleop", 1.0, 0.12)
        self.assertIsNone(self.seen())

    def test_lane_sends_and_releases(self):
        sim = Sim.from_toml(ROBOT_TOML)
        driver = sim.add(Driver, "planner")
        lift = sim.add(Aimer, "lift")
        hw = SimHardware(sim)
        hw.enable()
        hw.set_driver_station(enabled=True, teleop=True)
        sim.step(0.04)
        self.assertEqual(
            sim.latest_target(Goal, "lift", "teleop").tick_count, 7)
        self.assertEqual(lift.seen.height_rot, 0.5)
        driver.talking = False
        sim.step(0.02)
        self.assertIsNone(sim.latest_target(Goal, "lift", "teleop"))
        self.assertIsNone(lift.seen)


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
        with self.assertRaises(ValueError):
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
        with self.assertRaises(ValueError):
            mover.sm.go(3)
        with self.assertRaises(ValueError):
            talos.StateMachine(sim.node("planner"), 7, states=Phase)

    def test_abi_errors_raise_rather_than_read_as_states(self):
        sim = Sim.from_toml(ROBOT_TOML)
        stepper = sim.add(Stepper, "planner")
        stepper.sm._index = 99  # no such state machine: every C call fails
        for name in ("state", "previous", "time_in_state", "just_entered"):
            with self.subTest(name=name):
                with self.assertRaises(talos.TalosError):
                    getattr(stepper.sm, name)


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

        with self.assertRaises(talos.TalosError):
            sim.add(Missing, "planner")

        class Mistyped:
            def __init__(self, node):
                node.param("height_rot", 0)

        with self.assertRaises(talos.TalosError):
            sim.add(Mistyped, "planner")

    def test_lists(self):
        seen = {}

        class Vision:
            def __init__(self, node):
                seen["cameras"] = node.param("cameras")
                seen["offsets_m"] = node.param("offsets_m")
                seen["no_cameras"] = node.param("no_cameras")
                seen["cameras_any"] = node.param("cameras", [])
                seen["offsets_typed"] = node.param("offsets_m", [0.0])
                seen["empty_over_default"] = node.param("no_cameras", ["x"])
                seen["absent"] = node.param("absent", ["left"])

        sim = Sim.from_toml(ROBOT_TOML)
        sim.add(Vision, "vision")
        self.assertEqual(seen, {
            "cameras": ["front", "back"], "offsets_m": [0.5, 1.0, -2.25],
            "no_cameras": [], "cameras_any": ["front", "back"],
            "offsets_typed": [0.5, 1.0, -2.25], "empty_over_default": [],
            "absent": ["left"]})
        self.assertEqual([type(v) for v in seen["offsets_m"]], [float] * 3)

        def reading(key, *default):
            class Reader:
                def __init__(self, node):
                    node.param(key, *default)
            return Reader

        for key, default in (
                ("mixed", ()),
                ("flags", ()),
                ("cameras", ([0.0],)),
                ("offsets_m", (["a"],)),
                ("exposure_s", ([],)),
                ("exposure_s", (["a"],)),
                ("absent", ())):
            with self.subTest(key=key, default=default):
                with self.assertRaises(talos.TalosError):
                    Sim.from_toml(ROBOT_TOML).add(reading(key, *default),
                                                  "vision")
        with self.assertRaises(TypeError):
            Sim.from_toml(ROBOT_TOML).add(reading("cameras", ["a", 1.0]),
                                          "vision")

    def test_param_of_reads_another_subsystem_only_in_init(self):
        seen = {}

        class Reader:
            def __init__(self, node):
                self.node = node
                seen["gain"] = node.param_of("follower", "gain")
                seen["offsets_m"] = node.param_of("vision", "offsets_m")
                seen["absent"] = node.param_of("follower", "absent", 7.5)

        sim = Sim.from_toml(ROBOT_TOML)
        reader = sim.add(Reader, "planner")
        self.assertEqual(seen, {"gain": 2.0, "offsets_m": [0.5, 1.0, -2.25],
                                "absent": 7.5})
        with self.assertRaises(talos.TalosError):
            reader.node.param_of("follower", "gain")

        def reading(subsystem, key, *default):
            class Reader:
                def __init__(self, node):
                    node.param_of(subsystem, key, *default)
            return Reader

        for subsystem, key, default in (("follower", "absent", ()),
                                         ("absent", "gain", (1.0,)),
                                         ("follower", "mode", (0.0,))):
            with self.subTest(subsystem=subsystem, key=key):
                with self.assertRaises(talos.TalosError):
                    Sim.from_toml(ROBOT_TOML).add(
                        reading(subsystem, key, *default), "planner")


class MessageTest(unittest.TestCase):
    def test_forwarding_checks_schema_and_payload_in_every_send_path(self):
        from talos._messages import message_type

        class Outputs:
            def __init__(self, node):
                self.publisher = node.publish(Goal, "/forward/goal")
                self.lane = talos.TargetLane(node, Goal, "lift", "teleop")

        sim = Sim.from_toml(ROBOT_TOML)
        outputs = sim.add(Outputs, "planner")
        goal_type = message_type(Goal)
        report_type = message_type(Report)
        self.assertEqual(goal_type.size, report_type.size)
        goal = goal_type.decode(goal_type.encode(
            {"height_rot": 2.5, "enabled": True, "tick_count": 0}))
        report = report_type.decode(report_type.encode(
            {"phase": Phase.IDLE, "height_rot": 9.0}))

        paths = (
            ("publisher", outputs.publisher.send, "talos_publisher_send"),
            ("lane", outputs.lane.send, "talos_target_lane_send"),
            ("sim", lambda msg: sim.publish(
                Goal, "/forward/injected", msg), "talos_sim_publish_raw"),
            ("sim_target", lambda msg: sim.publish_target(
                Goal, "lift", "align", msg), "talos_sim_publish_target"),
        )
        for name, send, symbol in paths:
            with self.subTest(path=name):
                send(goal)
                with mock.patch.object(talos.lib, symbol) as forward:
                    with self.assertRaises(TypeError):
                        send(report)
                    forward.assert_not_called()

        self.assertEqual(sim.latest(Goal, "/forward/goal"), goal)
        self.assertEqual(sim.latest(Goal, "/forward/injected"), goal)
        self.assertEqual(sim.latest_target(Goal, "lift", "teleop"), goal)
        self.assertEqual(sim.latest_target(Goal, "lift", "align"), goal)

        changed_hash = type("ChangedGoal", (Goal,), {
            "TALOS_SCHEMA_HASH": Goal.TALOS_SCHEMA_HASH ^ 1,
        })
        alternate_class = type("EquivalentGoal", (Goal,), {})
        alternate_type = message_type(alternate_class)
        equivalent = alternate_type.decode(goal._bytes)
        mismatches = (
            message_type(changed_hash).decode(goal._bytes),
            goal_type.decode(goal._bytes[:-1]),
            goal_type.decode(goal._bytes + b"\0"),
        )
        for name, send, symbol in paths:
            with self.subTest(path=name, identity="equivalent"):
                # Schema identity, not Python class identity, is the contract.
                send(equivalent)
            for bad in mismatches:
                with self.subTest(path=name, payload=bad._bytes,
                                  schema_hash=bad._type.schema_hash):
                    with mock.patch.object(talos.lib, symbol) as forward:
                        with self.assertRaises(TypeError):
                            send(bad)
                        forward.assert_not_called()
            with self.subTest(path=name, identity="not_a_message"):
                with mock.patch.object(talos.lib, symbol) as forward:
                    with self.assertRaises(TypeError):
                        send(object())
                    forward.assert_not_called()

    def test_byte_publisher_takes_buffers_and_reports_oversize(self):
        class Telemeter:
            def __init__(self, node):
                self.frames = node.publish_bytes("frames", 6)

        sim = Sim.from_toml(ROBOT_TOML)
        telemeter = sim.add(Telemeter, "planner")
        self.assertIsNone(sim.latest_bytes("/planner/frames"))
        self.assertTrue(telemeter.frames.send(memoryview(bytearray(b"\0\1\2"))))
        self.assertEqual(sim.latest_bytes("/planner/frames"), b"\0\1\2")

        # Oversize returns False and preserves the last valid frame; empty works.
        self.assertFalse(telemeter.frames.send(b"1234567"))
        self.assertEqual(sim.latest_bytes("/planner/frames"), b"\0\1\2")
        self.assertTrue(telemeter.frames.send(b""))
        self.assertEqual(sim.latest_bytes("/planner/frames"), b"")
        with self.assertRaises(TypeError):
            telemeter.frames.send("not bytes")


if __name__ == "__main__":
    unittest.main()
