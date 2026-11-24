"""The talos package, end to end through libtalos_node on talos.testing.Sim."""

import ctypes
import enum
import json
import math
import os
from pathlib import Path
import re
import struct
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
        talos.allow_untargeted_outputs(node)
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


class Clock:
    """No update(), so stepping weeks of virtual time runs no timer."""

    def __init__(self, node):
        self.node = node


class TwoNodesTest(unittest.TestCase):
    # REQ-CONTRACT-14: Python reads the node clock, and a test the Sim
    # clock, as integer nanoseconds, as C++ does with now().nanos().
    def test_now_ns_is_the_exact_node_clock(self):
        sim = Sim.from_toml(ROBOT_TOML)
        clock = sim.add(Clock, "planner")
        # 50.7 days, where float seconds no longer round-trip: each step
        # converts exactly, their sum does not.
        sim.step(4_000_000.0)
        sim.step(378_605.124704419)
        expected = 4_378_605_124_704_419
        self.assertEqual(clock.node.now_ns, expected)
        self.assertEqual(sim.now_ns, expected)
        self.assertNotEqual(round(clock.node.now * 1e9), expected)

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
        self.assertFalse(follower.goal.fresh(max_age=-0.01))
        self.assertLess(follower.goal.age, 0.01)
        self.assertEqual(sim.node("follower").period, 0.01)
        self.assertAlmostEqual(sim.now, 0.055)

    # REQ-RT-02, REQ-RT-19
    def test_an_at_drain_callback_hears_every_message_at_the_next_tick(self):
        """wake=Wake.AT_DRAIN reaches the node: goals published at 2 and 3
        ms reach the callback in order at the 10 ms tick's drain, not as
        they are published."""
        sim = Sim.from_toml(ROBOT_TOML)
        collector = sim.add(AtDrainCollector, "collector")
        sim.step(0.002)
        for tick in (7, 8):
            sim.publish(Goal, "/planner/goal", height_rot=0.0, enabled=True,
                        tick_count=tick)
            sim.step(0.001)
        self.assertEqual(collector.heard, [])
        sim.step(0.007)
        self.assertEqual(collector.heard, [(7, 10), (8, 10)])

    # REQ-RT-19: omitting wake= defaults to Wake.ON_PUBLISH; in a timed node
    # (10 ms update()), a publication at 2 ms runs the callback at now == 2 ms
    # before the 10 ms tick.
    def test_default_subscribe_callback_wakes_timed_node_on_publish(self):
        class PromptCollector:
            def __init__(self, node):
                self.node = node
                self.heard = []
                self.updates = 0
                node.subscribe(Goal, "/planner/goal", callback=self.hear)

            def hear(self, goal):
                self.heard.append((goal.tick_count, self.node.now_ns))

            def update(self):
                self.updates += 1

        sim = Sim.from_toml(ROBOT_TOML)
        collector = sim.add(PromptCollector, "collector")
        sim.step(0.002)
        sim.publish(Goal, "/planner/goal", height_rot=0.0, enabled=True,
                    tick_count=7)
        sim.step(0.001)
        self.assertEqual(collector.heard, [(7, 2_000_000)])
        self.assertEqual(collector.updates, 0)
        sim.step(0.008)
        self.assertEqual(collector.updates, 1)
        self.assertEqual(collector.heard, [(7, 2_000_000)])

    # REQ-SAFE-31: Sim.remove destroys the hosted node and invalidates its
    # Python wrappers; mid-run Sim.add starts at now_ns with no catch-up ticks
    # and delivers only messages published after attach.
    def test_remove_and_mid_run_add_restart_node_cleanly(self):
        import gc
        import weakref

        sim = Sim.from_toml(ROBOT_TOML)
        first = sim.add(Follower, "follower")
        first_node = sim.node("follower")
        first_sub = first.goal
        first_ref = weakref.ref(first)

        sim.publish(Goal, "/planner/goal", height_rot=1.0, enabled=True,
                    tick_count=1)
        sim.step(0.025)
        self.assertEqual(len(first.received), 1)
        self.assertEqual(first_node.tick, 2)

        sim.remove("follower")
        # Stale handles raise TalosError rather than touching freed memory.
        with self.assertRaises(talos.TalosError):
            _ = first_node.name
        with self.assertRaises(talos.TalosError):
            _ = first_sub.has_value
        with self.assertRaises(talos.TalosError):
            sim.remove("follower")
        with self.assertRaises(talos.TalosError):
            sim.node("follower")

        # Callback tables were cleared so the removed node object is collectible.
        del first, first_node, first_sub
        gc.collect()
        self.assertIsNone(first_ref())

        # Publications before re-add must not reach the restarted node.
        sim.publish(Goal, "/planner/goal", height_rot=2.0, enabled=True,
                    tick_count=2)
        sim.step(0.01)
        self.assertEqual(sim.now_ns, 35_000_000)

        restarted = sim.add(Follower, "follower")
        restarted_node = sim.node("follower")
        self.assertEqual(restarted_node.now_ns, 35_000_000)
        self.assertEqual(restarted_node.tick, 0)
        sim.step(0.01)
        self.assertEqual(restarted_node.tick, 1)
        self.assertEqual(restarted.received, [])
        self.assertFalse(restarted.goal.has_value)

        sim.publish(Goal, "/planner/goal", height_rot=3.0, enabled=True,
                    tick_count=3)
        sim.step(0.01)
        self.assertEqual([g.tick_count for g in restarted.received], [3])
        status = sim.latest(Status, "/follower/status")
        self.assertEqual(status.height_rot, 6.0)

        # Calling remove() or add() from inside step() raises TalosError.
        class Reenter:
            def __init__(self, node):
                self.action = None

            def update(self):
                if self.action is not None:
                    self.action()

        reenter = sim.add(Reenter, "planner")
        reenter.action = lambda: sim.remove("follower")
        with self.assertRaises(talos.TalosError):
            sim.step(0.01)
        reenter.action = lambda: sim.add(Clock, "vision")
        with self.assertRaises(talos.TalosError):
            sim.step(0.02)
        reenter.action = None
        sim.step(0.02)


class AtDrainCollector:
    """Hears every /planner/goal at the node's drains, never woken by one:
    (tick_count, node time in ms) per goal."""

    def __init__(self, node):
        self.node = node
        self.heard = []
        node.subscribe(Goal, "/planner/goal", callback=self.hear,
                       wake=talos.Wake.AT_DRAIN)

    def hear(self, goal):
        self.heard.append((goal.tick_count, round(self.node.now * 1000)))

    def update(self):
        pass


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


    # REQ-OPS-06: holds_stop, by which the launcher holds a stop through a
    # Python node's start-up, reaches its --describe as params do.
    def test_describe_reports_every_param_the_constructor_read(self):
        # The launcher refuses a subsystem key no node reads, so a Python
        # node's reads (set or not, its own table or another's) must reach
        # --describe through the C ABI.
        probe = textwrap.dedent("""
            import sys, tempfile
            import talos

            class Probe:
                def __init__(self, node):
                    node.param("height_rot")
                    node.param("absent_v", 0.0)
                    node.param_of("follower", "gain", 1.0)

            with tempfile.NamedTemporaryFile("w", suffix=".toml") as config:
                config.write(sys.argv[1])
                config.flush()
                sys.exit(talos.run(Probe, "planner", "//talOS/python:probe",
                                   ["probe", "--describe", "--config",
                                    config.name]))
        """)
        result = subprocess.run([sys.executable, "-c", probe, ROBOT_TOML],
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        described = json.loads(result.stdout[result.stdout.index("{"):])
        self.assertTrue(described["holds_stop"])
        self.assertEqual(described["params"], [
            {"subsystem": "planner", "key": "height_rot"},
            {"subsystem": "planner", "key": "absent_v"},
            {"subsystem": "follower", "key": "gain"},
        ])

    # REQ-RT-23: a Python update() reaches --describe as abi_update, by which
    # the launcher holds a Python actuator owner to 20 ms or slower.
    def test_describe_reports_a_python_update(self):
        probe = textwrap.dedent("""
            import sys, tempfile
            import talos

            class Probe:
                def __init__(self, node):
                    pass

                def update(self):
                    pass

            with tempfile.NamedTemporaryFile("w", suffix=".toml") as config:
                config.write(sys.argv[1])
                config.flush()
                sys.exit(talos.run(Probe, "planner", "//talOS/python:probe",
                                   ["probe", "--describe", "--config",
                                    config.name]))
        """)
        result = subprocess.run([sys.executable, "-c", probe, ROBOT_TOML],
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        described = json.loads(result.stdout[result.stdout.index("{"):])
        self.assertTrue(described["abi_update"])

    # REQ-SAFE-42: the opt-out reaches --describe through the C ABI, on the
    # hardware request only, which the graph lint and the registry read.
    def test_describe_reports_untargeted_outputs(self):
        probe = textwrap.dedent("""
            import sys, tempfile
            import talos

            class Probe:
                def __init__(self, node):
                    self.lift = talos.Motor(node, "lift")
                    talos.allow_untargeted_outputs(node)

            with tempfile.NamedTemporaryFile("w", suffix=".toml") as config:
                config.write(sys.argv[1])
                config.flush()
                sys.exit(talos.run(Probe, "lift", "//talOS/python:probe",
                                   ["probe", "--describe", "--config",
                                    config.name]))
        """)
        result = subprocess.run([sys.executable, "-c", probe, ROBOT_TOML],
                                capture_output=True, text=True, timeout=30)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        described = json.loads(result.stdout[result.stdout.index("{"):])
        marked = {source["name"] for source in described["sources"]
                  if source.get("untargeted_outputs")}
        self.assertEqual(marked, {"/hw/request/lift"})

    # REQ-SAFE-42: under --targeted-outputs (passed by the launcher when
    # --describe declared a hardware request without untargeted_outputs),
    # talos.allow_untargeted_outputs raises TalosError at construction so the
    # Python node exits non-zero before update() runs; without the flag, the
    # same node constructs and ticks.
    def test_targeted_outputs_refuses_allow_untargeted_outputs_at_construction(
        self,
    ):
        probe = textwrap.dedent("""
            import os, signal, sys, tempfile
            import talos

            mode = sys.argv[1]
            robot_toml = sys.argv[2]
            ticks = []

            class Probe:
                def __init__(self, node):
                    self.lift = talos.Motor(node, "lift")
                    talos.allow_untargeted_outputs(node)

                def update(self):
                    ticks.append(1)
                    os.kill(os.getpid(), signal.SIGTERM)

            with tempfile.TemporaryDirectory() as tmp:
                config = os.path.join(tmp, "robot.toml")
                log = os.path.join(tmp, "lift.tlog")
                with open(config, "w") as f:
                    f.write(robot_toml)
                argv = ["probe", "--sim", "--duration-s", "5",
                        "--log", log, "--config", config]
                if mode == "targeted":
                    argv.insert(2, "--targeted-outputs")
                    try:
                        code = talos.run(Probe, "lift", "//talOS/python:probe",
                                         argv)
                    except talos.TalosError as err:
                        assert not ticks, f"refused node ticked: {len(ticks)}"
                        print(f"TALOS_ERROR:{err}", flush=True)
                        raise
                    raise AssertionError(
                        f"expected TalosError, got exit code {code} "
                        f"(ticks={len(ticks)})"
                    )
                code = talos.run(Probe, "lift", "//talOS/python:probe", argv)
                assert code == 0, f"expected exit 0, got {code}"
                assert ticks, "expected update() to run"
        """)

        refused = subprocess.run(
            [sys.executable, "-c", probe, "targeted", ROBOT_TOML],
            capture_output=True, text=True, timeout=30,
        )
        self.assertNotEqual(refused.returncode, 0,
                            refused.stdout + refused.stderr)
        self.assertIn("TALOS_ERROR:", refused.stdout,
                      refused.stdout + refused.stderr)
        self.assertIn("lift", refused.stdout)
        self.assertIn("AllowUntargetedOutputs", refused.stdout)
        self.assertIn("--targeted-outputs", refused.stdout)
        self.assertIn("TalosError", refused.stderr)

        allowed = subprocess.run(
            [sys.executable, "-c", probe, "standalone", ROBOT_TOML],
            capture_output=True, text=True, timeout=30,
        )
        self.assertEqual(allowed.returncode, 0,
                         allowed.stdout + allowed.stderr)


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
                    self.arming = talos.OperatorArming(node)
                    self.lane = talos.TargetLane(
                        node, Goal, "probe", "teleop", arming=self.arming)
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
                         "hardware", "ds", "target", "arming", "lane", "sm"):
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
                elif kind == "arming":
                    assert not wrapper.armed
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


class AbiErrorsTest(unittest.TestCase):
    def test_abi_errors_are_not_values_states_or_true_flags(self):
        sim = Sim.from_toml(ROBOT_TOML)
        follower = sim.add(Follower, "follower")
        class LinkedStepper(Stepper):
            def __init__(self, node):
                super().__init__(node)
                self.link = talos.HardwareLink(node)

        stepper = sim.add(LinkedStepper, "planner")
        # A failed state read returns 0, a valid state: only the error
        # string set by the failed call tells them apart.
        stepper.sm._index = 99  # no such state machine: every C call fails
        for name in ("state", "previous", "time_in_state", "just_entered"):
            with self.subTest(state_machine=name):
                with self.assertRaises(talos.TalosError):
                    getattr(stepper.sm, name)
        subscription = follower.goal
        node = sim.node("follower")
        self.assertIsNone(subscription.value)
        with mock.patch.object(talos.lib, "talos_last_error",
                               return_value=b"abi call failed"):
            for symbol, read in (
                ("talos_node_simulation", lambda: node.simulation),
                ("talos_node_describing", lambda: node.describing),
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
        # Negative max_age still validates handle/index before returning False.
        subscription._index = 99
        with self.assertRaises(talos.TalosError):
            subscription.fresh(-0.01)
        node._native = None
        with self.assertRaises(talos.TalosError):
            subscription.fresh(-0.01)
        link = stepper.link
        sim.node("planner")._native = None
        with self.assertRaises(talos.TalosError):
            link.fresh(-0.01)


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

    # REQ-SAFE-03, REQ-SAFE-06: PwmOutput.set(0.0) is the centre pulse and
    # stop() no pulse (None), and set()/stop() outside update() raise naming
    # the device
    def test_pwm_stop_sends_no_pulse_and_set_zero_is_the_centre(self):
        class Servo:
            def __init__(self, node):
                talos.allow_untargeted_outputs(node)
                self.servo = talos.PwmOutput(node, "servo")
                self.stop = False

            def update(self):
                if self.stop:
                    self.servo.stop()
                else:
                    self.servo.set(0.0)

        sim = Sim.from_toml(ROBOT_TOML + """
[subsystems.lift.pwm_outputs.servo]
channel = 0
""")
        servo = sim.add(Servo, "lift")
        with self.assertRaisesRegex(talos.TalosError, r"'servo'"):
            servo.servo.set(0.5)
        with self.assertRaisesRegex(talos.TalosError, r"'servo'"):
            servo.servo.stop()
        hw = SimHardware(sim)
        hw.enable()
        sim.step(0.03)
        self.assertEqual(hw.pwm("lift", "servo"), 0.0)
        self.assertEqual(hw.applied_pwm("lift", "servo"), 0.0)
        servo.stop = True
        sim.step(0.03)
        self.assertIsNone(hw.pwm("lift", "servo"))
        self.assertIsNone(hw.applied_pwm("lift", "servo"))

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
        self.assertIsNone(signals.encoder.position_time)
        # Motor and CANcoder ages come from the State: known before the clock
        # mapping.
        self.assertEqual((signals.motor.position_age,
                          signals.motor.velocity_age), (1_000_000, 3_000_000))
        self.assertEqual((signals.encoder.position_age,
                          signals.encoder.velocity_age), (2_000_000, 4_000_000))
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
        # An unknown age invalidates the whole sample, as at the gateway.
        hw.set_motor("lift", "lift", 3, 2, 0xffffffff, 0)
        hw.set_cancoder("lift", "cc", 0.2, 0.3, 0, 0xffffffff)
        hw.set_gyro("lift", "gyro", 0.75, -0.5, 0xffffffff, 0)
        sim.step(0.02)
        self.assertFalse(signals.motor.connected)
        self.assertIsNone(signals.motor.velocity_time)
        self.assertIsNone(signals.motor.velocity_age)
        self.assertFalse(signals.encoder.connected)
        self.assertIsNone(signals.encoder.position_time)
        self.assertIsNone(signals.encoder.position_age)
        self.assertIsNone(signals.encoder.velocity_age)
        self.assertIsNone(signals.gyro.yaw_rate_time)
        hw.set_motor("lift", "lift", 3, 2, 1000, 3000)
        hw.set_cancoder("lift", "cc", 0.2, 0.3, 2000, 4000)
        hw.set_connected("lift", "lift", False)
        hw.set_connected("lift", "cc", False)
        sim.step(0.02)
        self.assertIsNone(signals.motor.velocity_time)
        self.assertIsNone(signals.encoder.position_age)
        self.assertIsNone(signals.encoder.velocity_age)
        hw.set_connected("lift", "cc", True)
        sim.step(0.02)
        self.assertEqual((signals.encoder.position_age,
                          signals.encoder.velocity_age), (2_000_000, 4_000_000))
        del hw
        sim.step(0.15)
        self.assertIsNone(signals.encoder.position_time)
        self.assertIsNone(signals.encoder.position_age)
        self.assertIsNone(signals.encoder.velocity_age)
        self.assertIsNone(signals.gyro.yaw_rate_time)

    # REQ-CONTRACT-34: before any valid sample, Motor, CANcoder, Encoder and
    # Gyro read 0.0; across disconnect and stale link, each holds its last
    # valid position/yaw through any rezero offset while rates and efforts
    # read 0.0 and connected is False.
    def test_invalid_and_stale_samples_hold_position_and_zero_rates(self):
        class Mech:
            def __init__(self, node):
                self.motor = talos.Motor(node, "lift")
                self.cc = talos.CANcoder(node, "cc")
                self.gyro = talos.Gyro(node, "gyro")
                self.enc = talos.Encoder(node, "enc")

        toml = ROBOT_TOML + """
[subsystems.lift.sensors.cc]
type = "CANcoder"
can_id = 2
[subsystems.lift.sensors.gyro]
type = "Pigeon2"
can_id = 3
[subsystems.lift.encoders.enc]
type = "DutyCycleEncoder"
dio = 2
"""
        sim = Sim.from_toml(toml)
        mech = sim.add(Mech, "lift")

        # Before first valid sample: disconnected and 0.0 across all reads.
        sim.step(0.02)
        self.assertFalse(mech.motor.connected)
        self.assertEqual(mech.motor.position, 0.0)
        self.assertEqual(mech.motor.velocity, 0.0)
        self.assertEqual(mech.motor.voltage, 0.0)
        self.assertEqual(mech.motor.current, 0.0)
        self.assertEqual(mech.motor.supply_current_a, 0.0)
        self.assertFalse(mech.cc.connected)
        self.assertEqual(mech.cc.position, 0.0)
        self.assertEqual(mech.cc.velocity, 0.0)
        self.assertFalse(mech.enc.connected)
        self.assertEqual(mech.enc.position, 0.0)
        self.assertEqual(mech.enc.velocity, 0.0)
        self.assertFalse(mech.gyro.connected)
        self.assertEqual(mech.gyro.yaw, 0.0)
        self.assertEqual(mech.gyro.yaw_rate, 0.0)

        # SimHardware.set_motor leaves MotorSample::voltage at 0.0; patch the
        # latest encoded /hw/state packet so the live sample carries 11.5 V
        # before disconnect and staleness (preventing a tautological 0.0 check).
        def publish_live_motor_voltage(expected_pos, expected_vel, voltage):
            packet_size = 1436
            wire = 4  # uint32_t size prefix in hardware::Packet
            buf = ctypes.create_string_buffer(packet_size)
            self.assertEqual(
                _abi.ok(_abi.lib.talos_sim_latest_raw(
                    sim._handle, b"/hw/state", buf, packet_size)), 1)
            raw = bytearray(buf.raw)
            (wire_size,) = struct.unpack_from("<I", raw, 0)
            self.assertGreaterEqual(wire_size, 84 + 45)
            self.assertLessEqual(wire_size, packet_size - wire)
            (motor_count,) = struct.unpack_from("<H", raw, wire + 70)
            self.assertEqual(motor_count, 1)
            self.assertEqual(raw[wire + 84 + 2], 1)  # motors[0].valid
            pos, vel = struct.unpack_from("<dd", raw, wire + 84 + 3)
            self.assertAlmostEqual(pos, expected_pos)
            self.assertAlmostEqual(vel, expected_vel)
            (sample_time_us,) = struct.unpack_from("<Q", raw, wire + 24)
            (sequence,) = struct.unpack_from("<Q", raw, wire + 48)
            struct.pack_into("<Q", raw, wire + 24, sample_time_us + 1000)
            struct.pack_into("<Q", raw, wire + 48, sequence + 1)
            struct.pack_into("<d", raw, wire + 84 + 19, voltage)
            hw.lose_states(1)
            _abi.ok(_abi.lib.talos_sim_publish_raw(
                sim._handle, b"/hw/state", bytes(raw), packet_size))
            sim.step(0.01)

        # Valid sample + rezero Motor and Gyro.
        hw = SimHardware(sim)
        hw.set_motor("lift", "lift", 3.0, 2.0,
                     temperature_c=40.0, supply_current_a=18.0,
                     stator_current_a=30.0)
        hw.set_cancoder("lift", "cc", 0.2, 0.3)
        hw.set_gyro("lift", "gyro", 0.25, -0.5)
        hw.set_encoder("lift", "enc", 0.4, 0.6)
        sim.step(0.02)
        mech.motor.rezero(1.0)
        mech.gyro.rezero(0.0)
        self.assertTrue(mech.motor.zeroed)
        self.assertTrue(mech.gyro.zeroed)
        publish_live_motor_voltage(3.0, 2.0, 11.5)
        self.assertAlmostEqual(mech.motor.position, 1.0)
        self.assertAlmostEqual(mech.motor.velocity, 2.0)
        self.assertAlmostEqual(mech.motor.voltage, 11.5)
        self.assertAlmostEqual(mech.motor.current, 30.0)
        self.assertAlmostEqual(mech.motor.supply_current_a, 18.0)
        self.assertAlmostEqual(mech.gyro.yaw, 0.0)
        self.assertAlmostEqual(mech.gyro.yaw_rate, -0.5)
        self.assertAlmostEqual(mech.cc.position, 0.2)
        self.assertAlmostEqual(mech.enc.position, 0.4)

        # Disconnect every device: positions hold through rezero offsets;
        # rates and efforts read 0.0.
        for dev in ("lift", "cc", "gyro", "enc"):
            hw.set_connected("lift", dev, False)
        sim.step(0.02)
        self.assertFalse(mech.motor.connected)
        self.assertFalse(mech.cc.connected)
        self.assertFalse(mech.gyro.connected)
        self.assertFalse(mech.enc.connected)
        self.assertAlmostEqual(mech.motor.position, 1.0)
        self.assertAlmostEqual(mech.gyro.yaw, 0.0)
        self.assertAlmostEqual(mech.cc.position, 0.2)
        self.assertAlmostEqual(mech.enc.position, 0.4)
        self.assertEqual(mech.motor.velocity, 0.0)
        self.assertEqual(mech.motor.voltage, 0.0)
        self.assertEqual(mech.motor.current, 0.0)
        self.assertEqual(mech.motor.supply_current_a, 0.0)
        self.assertEqual(mech.cc.velocity, 0.0)
        self.assertEqual(mech.gyro.yaw_rate, 0.0)
        self.assertEqual(mech.enc.velocity, 0.0)

        # Reconnect with new live positions, then drop the link past
        # command_timeout_us (stale link).
        hw.set_motor("lift", "lift", 5.0, 2.0,
                     temperature_c=45.0, supply_current_a=18.0,
                     stator_current_a=30.0)
        hw.set_cancoder("lift", "cc", 0.35, 0.3)
        hw.set_gyro("lift", "gyro", 0.5, -0.5)
        hw.set_encoder("lift", "enc", 0.55, 0.6)
        for dev in ("lift", "cc", "gyro", "enc"):
            hw.set_connected("lift", dev, True)
        sim.step(0.02)
        publish_live_motor_voltage(5.0, 2.0, 11.5)
        self.assertAlmostEqual(mech.motor.position, 3.0)
        self.assertAlmostEqual(mech.motor.voltage, 11.5)
        self.assertAlmostEqual(mech.gyro.yaw, 0.25)

        del hw
        sim.step(0.15)
        self.assertFalse(mech.motor.connected)
        self.assertFalse(mech.cc.connected)
        self.assertFalse(mech.gyro.connected)
        self.assertFalse(mech.enc.connected)
        self.assertAlmostEqual(mech.motor.position, 3.0)
        self.assertAlmostEqual(mech.gyro.yaw, 0.25)
        self.assertAlmostEqual(mech.cc.position, 0.35)
        self.assertAlmostEqual(mech.enc.position, 0.55)
        self.assertEqual(mech.motor.velocity, 0.0)
        self.assertEqual(mech.motor.voltage, 0.0)
        self.assertEqual(mech.motor.current, 0.0)
        self.assertEqual(mech.motor.supply_current_a, 0.0)
        self.assertEqual(mech.cc.velocity, 0.0)
        self.assertEqual(mech.gyro.yaw_rate, 0.0)
        self.assertEqual(mech.enc.velocity, 0.0)

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

    # REQ-CONTRACT-14, REQ-OPS-20: Motor.hardware_index returns the motor's slot
    # in configured motor order matching /hw/status motor_fault_mask.
    def test_motor_hardware_index_matches_configured_order(self):
        toml = """
[robot]
period_us = 10000
command_timeout_us = 100000

[subsystems.alpha.motors.first]
can_id = 1

[subsystems.beta.digital_inputs.beam]
dio = 0
[subsystems.beta.motors.second]
can_id = 2
[subsystems.beta.motors.third]
can_id = 3
"""

        class Alpha:
            def __init__(self, node):
                self.first = talos.Motor(node, "first")

        class Beta:
            def __init__(self, node):
                # Open a non-motor first and motors in reverse order so
                # per-node handle indices differ from hardware motor indices.
                self.beam = talos.DigitalInput(node, "beam")
                self.third = talos.Motor(node, "third")
                self.second = talos.Motor(node, "second")

        sim = Sim.from_toml(toml)
        alpha = sim.add(Alpha, "alpha")
        beta = sim.add(Beta, "beta")
        indices = (alpha.first.hardware_index, beta.second.hardware_index,
                   beta.third.hardware_index)
        self.assertEqual(set(indices), {0, 1, 2})
        self.assertNotEqual(beta.third.hardware_index, beta.third._index)
        sim.remove("alpha")
        with self.assertRaises(talos.TalosError):
            _ = alpha.first.hardware_index

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

        lift.lift.rezero(0.75)  # a rotor frame: motion magic needs a zero
        hw.enable()
        sim.step(0.03)
        self.assertTrue(lift.enabled)
        self.assertEqual(hw.motor_request("lift", "lift"),
                         (talos.Mode.MOTION_MAGIC, 1.5, 0.25, 2))

    # REQ-SAFE-42: talos.allow_untargeted_outputs raises TalosError once the
    # node's event loop has started (after __init__ or inside update()).
    def test_allow_untargeted_outputs_after_loop_start_raises(self):
        class LateOptOut:
            def __init__(self, node):
                self.node = node
                self.lift = talos.Motor(node, "lift")
                self.opt_out_in_update = False

            def update(self):
                if self.opt_out_in_update:
                    talos.allow_untargeted_outputs(self.node)

        sim = Sim.from_toml(ROBOT_TOML)
        late = sim.add(LateOptOut, "lift")
        with self.assertRaises(talos.TalosError):
            talos.allow_untargeted_outputs(sim.node("lift"))
        late.opt_out_in_update = True
        with self.assertRaises(talos.TalosError):
            sim.step(0.02)

    # REQ-SAFE-06: every motor setter outside update() (every(), __init__,
    # callback, off-thread) raises TalosError naming the motor
    def test_motor_command_outside_update_raises(self):
        # The flush after each update() sends neutral for every motor that
        # update() did not set, so a command from every() would pulse.
        torque_toml = ROBOT_TOML.replace(
            "can_id = 1\n",
            "can_id = 1\nfoc = true\nmax_torque_current_a = 5.0\n")
        setters = (
            ("set_duty_cycle", lambda m: m.set_duty_cycle(0.5)),
            ("set_voltage", lambda m: m.set_voltage(6.0)),
            ("set_velocity", lambda m: m.set_velocity(10.0)),
            ("set_position", lambda m: m.set_position(1.0)),
            ("set_motion_magic", lambda m: m.set_motion_magic(1.0)),
            ("set_torque_current", lambda m: m.set_torque_current(2.0)),
            ("stop", lambda m: m.stop()),
        )
        for name, call in setters:
            with self.subTest(setter=name):
                class Pulser:
                    def __init__(self, node):
                        self.lift = talos.Motor(node, "lift")
                        node.every(0.02, lambda: call(self.lift))

                    def update(self):
                        pass

                sim = Sim.from_toml(torque_toml)
                sim.add(Pulser, "lift")
                with self.assertRaisesRegex(talos.TalosError, r"'lift'"):
                    sim.step(0.05)

    # REQ-SAFE-06: Motor, DigitalOutput and PwmOutput setters called from
    # __init__, a message callback, outside update(), or off the loop thread
    # raise TalosError naming the device
    def test_output_setters_in_init_callback_and_off_thread_raise(self):
        toml = ROBOT_TOML + """
[subsystems.lift.digital_outputs.led]
dio = 1
[subsystems.lift.pwm_outputs.servo]
channel = 0
"""
        init_cases = (
            ("motor", "'lift'",
             lambda n: talos.Motor(n, "lift").set_voltage(6.0)),
            ("led", "'led'",
             lambda n: talos.DigitalOutput(n, "led").set(True)),
            ("servo_set", "'servo'",
             lambda n: talos.PwmOutput(n, "servo").set(0.5)),
            ("servo_stop", "'servo'",
             lambda n: talos.PwmOutput(n, "servo").stop()),
        )
        for label, device, call in init_cases:
            with self.subTest(context="init", case=label):
                class Eager:
                    def __init__(self, node):
                        call(node)

                sim = Sim.from_toml(toml)
                with self.assertRaisesRegex(talos.TalosError, device):
                    sim.add(Eager, "lift")

        # Message callback on the loop thread, outside update().
        class Listener:
            def __init__(self, node):
                talos.allow_untargeted_outputs(node)
                self.led = talos.DigitalOutput(node, "led")
                node.subscribe(Goal, "/test/poke",
                               callback=lambda _: self.led.set(True))

            def update(self):
                pass

        sim = Sim.from_toml(toml)
        sim.add(Listener, "lift")
        sim.publish(Goal, "/test/poke", height_rot=0.0, enabled=True,
                    tick_count=1)
        with self.assertRaisesRegex(talos.TalosError, r"'led'"):
            sim.step(0.02)

        # Direct call outside update() on DigitalOutput.
        class Light:
            def __init__(self, node):
                self.led = talos.DigitalOutput(node, "led")

            def update(self):
                pass

        sim = Sim.from_toml(toml)
        light = sim.add(Light, "lift")
        with self.assertRaisesRegex(talos.TalosError, r"'led'"):
            light.led.set(True)

        # Off-thread setters while update() is running.
        class Threaded:
            def __init__(self, node):
                talos.allow_untargeted_outputs(node)
                self.lift = talos.Motor(node, "lift")
                self.led = talos.DigitalOutput(node, "led")
                self.servo = talos.PwmOutput(node, "servo")
                self.errors = {}

            def update(self):
                def worker():
                    for key, fn in (
                        ("lift", lambda: self.lift.set_voltage(1.0)),
                        ("led", lambda: self.led.set(True)),
                        ("servo_set", lambda: self.servo.set(0.5)),
                        ("servo_stop", lambda: self.servo.stop()),
                    ):
                        try:
                            fn()
                            self.errors[key] = None
                        except talos.TalosError as err:
                            self.errors[key] = str(err)

                thread = threading.Thread(target=worker)
                thread.start()
                thread.join()

        sim = Sim.from_toml(toml)
        threaded = sim.add(Threaded, "lift")
        sim.step(0.02)
        self.assertIn("'lift'", threaded.errors["lift"])
        self.assertIn("'led'", threaded.errors["led"])
        self.assertIn("'servo'", threaded.errors["servo_set"])
        self.assertIn("'servo'", threaded.errors["servo_stop"])

    # REQ-SAFE-43: set_torque_current reaches the request as stator amperes,
    # clamped to the motor's limit and counted, and on a motor whose config
    # lacks foc it raises naming the motor and the key
    def test_torque_current_clamps_and_needs_the_config(self):
        class Torquer:
            def __init__(self, node):
                talos.allow_untargeted_outputs(node)
                self.lift = talos.Motor(node, "lift")

            def update(self):
                self.lift.set_torque_current(8.0)

        sim = Sim.from_toml(ROBOT_TOML.replace(
            "can_id = 1\n",
            "can_id = 1\nfoc = true\nmax_torque_current_a = 5.0\n"))
        torquer = sim.add(Torquer, "lift")
        hw = SimHardware(sim)
        hw.enable()
        sim.step(0.03)
        self.assertEqual(hw.motor_request("lift", "lift"),
                         (talos.Mode.TORQUE_CURRENT, 5.0, 0.0, 0))
        self.assertGreater(torquer.lift.clamped_count, 0)

        sim = Sim.from_toml(ROBOT_TOML)
        sim.add(Torquer, "lift")
        with self.assertRaisesRegex(talos.TalosError, r"'lift'.*foc = true"):
            sim.step(0.03)

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
        self.assertFalse(link.fresh(-0.01))
        self.assertFalse(link.fresh(timedelta(seconds=-0.01)))
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
    """Records what its Target selected on each update(), and drives the
    lift motor at its height_rot (as volts) only while get() returns one."""

    def __init__(self, node):
        self.target = talos.Target(node, Goal)
        self.motor = talos.Motor(node, "lift")
        self.seen = None
        self.source = ""

    def update(self):
        self.seen = self.target.get()
        self.source = self.target.source
        if self.seen is not None:
            self.motor.set_voltage(self.seen.height_rot)


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


class Operator:
    """Sends the lift's teleop lane whenever joystick 0's button 1 is held,
    ignoring evaluate(): its OperatorArming alone holds the lane released."""

    def __init__(self, node):
        self.ds = talos.DriverStation(node)
        self.arming = talos.OperatorArming(node)
        self.lane = talos.TargetLane(node, Goal, "lift", "teleop",
                                     arming=self.arming)

    def update(self):
        held = self.ds.joystick(0).button(1)
        self.arming.evaluate(neutral=not held)
        if held:
            self.lane.send(height_rot=0.5, enabled=True, tick_count=7)
        else:
            self.lane.release()


class TargetTest(unittest.TestCase):
    """Target/TargetLane through Python; selection is target_test.cc's.

    The safety cases (stale, disabled, earlier generation) are here because
    a marshalling fault on the way out of talos_target_get, such as a
    binding that decodes its buffer when the call found no target, would
    actuate on a target the C++ Target refused (docs/TESTING.md §4)."""

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

    def applied(self):
        """The lift motor's applied (mode, demand)."""
        request = self.hw.applied_motor("lift", "lift")
        return request.mode, request.demand

    def test_teleop_drives_and_never_sent_is_none(self):
        self.sim.step(0.05)
        self.assertIsNone(self.seen())
        self.assertEqual(self.lift.source, "")
        self.hold("teleop", 1.5)
        self.assertEqual(self.seen(), 1.5)
        self.assertEqual(self.lift.source, "teleop")
        self.assertEqual(self.applied(), (talos.Mode.VOLTAGE, 1.5))

    # REQ-SAFE-07
    def test_stale_lane_goes_neutral_at_publication_timeout(self):
        self.hold("teleop", 1.0)
        # hold() returns 10 ms after its final publication: 80 ms more
        # leaves the target 90 ms old, just inside the 100 ms timeout.
        self.sim.step(0.08)
        self.assertEqual(self.seen(), 1.0)
        self.assertEqual(self.applied(), (talos.Mode.VOLTAGE, 1.0))
        self.sim.step(0.01)
        self.assertIsNone(self.seen())
        self.assertEqual(self.applied()[0], talos.Mode.NEUTRAL)

    # REQ-SAFE-02, REQ-SAFE-07
    def test_robot_disabled_and_driver_station_loss_go_neutral(self):
        self.hold("teleop", 1.0)
        self.hw.enable(False)
        self.hold("teleop", 1.0)
        self.assertIsNone(self.seen())
        self.assertEqual(self.applied()[0], talos.Mode.NEUTRAL)
        self.hw.enable()
        self.hold("teleop", 1.0)
        self.assertEqual(self.seen(), 1.0)
        self.hw.stop_driver_station()
        self.hold("teleop", 1.0, 0.12)
        self.assertIsNone(self.seen())
        self.assertEqual(self.applied()[0], talos.Mode.NEUTRAL)

    # REQ-SAFE-09
    def test_earlier_generation_target_is_ignored(self):
        self.hold("teleop", 1.0)
        self.assertEqual(self.seen(), 1.0)
        teleop = self.sim.driver_station_generation
        self.mode(enabled=False, teleop=True)
        # SimHardware publishes on the tick grid after the lift (added
        # first) has run, so the lift drains the re-enabling packet on its
        # next tick together with whatever was queued meanwhile.
        self.hw.set_driver_station(enabled=True, teleop=True)
        self.sim.step(0.01)
        self.assertNotEqual(self.sim.driver_station_generation, teleop)
        # Said under the first teleop period, drained with the packet that
        # starts the second: only the generation tells them apart.
        self.sim.publish_target(Goal, "lift", "teleop", height_rot=2.0,
                                enabled=True, tick_count=0,
                                mode_since_us=teleop)
        self.sim.step(0.01)
        self.assertIsNone(self.seen())
        self.assertEqual(self.lift.source, "")
        self.assertEqual(self.applied()[0], talos.Mode.NEUTRAL)
        # The lift is enabled and in teleop: the current generation drives.
        self.hold("teleop", 2.0, 0.01)
        self.assertEqual(self.applied(), (talos.Mode.VOLTAGE, 2.0))

    # REQ-DEV-08
    def test_publish_target_stamps_newest_ds_generation_or_explicit_mode(self):
        from talos._messages import message_type

        goal_type = message_type(Goal)
        envelope_size = 16 + goal_type.size

        def read_envelope(sim, subsystem, source):
            buf = ctypes.create_string_buffer(envelope_size)
            topic = f"/{subsystem}/target/{source}".encode()
            found = _abi.ok(_abi.lib.talos_sim_latest_raw(
                sim._handle, topic, buf, envelope_size))
            self.assertEqual(found, 1)
            mode_since_us, active, padding = struct.unpack_from(
                "=QB7s", buf.raw, 0)
            self.assertEqual(padding, b"\0" * 7)
            return mode_since_us, active, goal_type.decode(buf.raw[16:])

        # Before any Driver Station packet is published, generation is 0.
        sim = Sim.from_toml(ROBOT_TOML)
        hw = SimHardware(sim)
        self.assertEqual(sim.driver_station_generation, 0)
        sim.publish_target(Goal, "lift", "teleop", height_rot=1.0,
                           enabled=True, tick_count=1)
        gen, active, msg = read_envelope(sim, "lift", "teleop")
        self.assertEqual((gen, active, msg.height_rot, msg.tick_count),
                         (0, 1, 1.0, 1))

        # First active mode (teleop): default publish_target stamps DS gen.
        hw.set_driver_station(enabled=True, teleop=True)
        sim.step(0.02)
        teleop_gen = sim.driver_station_generation
        self.assertGreater(teleop_gen, 0)
        sim.publish_target(Goal, "lift", "teleop", height_rot=2.0,
                           enabled=True, tick_count=2)
        gen, active, msg = read_envelope(sim, "lift", "teleop")
        self.assertEqual((gen, active, msg.height_rot, msg.tick_count),
                         (teleop_gen, 1, 2.0, 2))

        # Disabled DS packet names no active mode: default stamps 0.
        hw.set_driver_station(enabled=False, teleop=True)
        sim.step(0.02)
        self.assertEqual(sim.driver_station_generation, 0)
        sim.publish_target(Goal, "lift", "teleop", height_rot=2.5,
                           enabled=True, tick_count=3)
        gen, active, msg = read_envelope(sim, "lift", "teleop")
        self.assertEqual((gen, active, msg.height_rot, msg.tick_count),
                         (0, 1, 2.5, 3))

        # Mode transition to autonomous increments generation: default stamps
        # the newest generation (covering pre-built message forwarding too).
        hw.set_driver_station(enabled=True, autonomous=True)
        sim.step(0.02)
        auto_gen = sim.driver_station_generation
        self.assertGreater(auto_gen, teleop_gen)
        pre_built = goal_type.decode(goal_type.encode(
            {"height_rot": 3.0, "enabled": True, "tick_count": 4}))
        sim.publish_target(Goal, "lift", "auto", pre_built)
        gen, active, msg = read_envelope(sim, "lift", "auto")
        self.assertEqual((gen, active, msg.height_rot, msg.tick_count),
                         (auto_gen, 1, 3.0, 4))

        # Explicit mode_since_us overrides the newest DS generation: earlier
        # generation, explicit 0 (not treated as None), and a 64-bit value.
        for explicit_gen in (teleop_gen, 0, 0x123456789ABCDEF0):
            with self.subTest(mode_since_us=explicit_gen):
                sim.publish_target(Goal, "lift", "auto", height_rot=4.0,
                                   enabled=True, tick_count=5,
                                   mode_since_us=explicit_gen)
                gen, active, msg = read_envelope(sim, "lift", "auto")
                self.assertEqual((gen, active, msg.height_rot, msg.tick_count),
                                 (explicit_gen, 1, 4.0, 5))

        # release_target clears active and stamps the newest DS generation.
        sim.release_target(Goal, "lift", "auto")
        gen, active, _ = read_envelope(sim, "lift", "auto")
        self.assertEqual((gen, active), (auto_gen, 0))

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

    def test_operator_arming_holds_a_button_held_through_re_enable(self):
        """The arming is the C++ one (target_test.cc covers every break);
        this is the binding, on the case it exists for."""
        sim = Sim.from_toml(ROBOT_TOML)
        sim.add(Operator, "planner")
        lift = sim.add(Aimer, "lift")
        hw = SimHardware(sim)
        hw.enable()

        def driver_station(held, enabled=True):
            hw.set_driver_station(
                enabled=enabled, teleop=True,
                joysticks=[Stick(buttons=(1,) if held else ())])
            sim.step(0.04)

        driver_station(held=True)  # held from startup: never armed
        self.assertIsNone(lift.seen)
        driver_station(held=False)
        driver_station(held=True)
        self.assertEqual(lift.seen.height_rot, 0.5)
        driver_station(held=True, enabled=False)
        driver_station(held=True)  # re-enabled with the button still held
        self.assertIsNone(sim.latest_target(Goal, "lift", "teleop"))
        self.assertIsNone(lift.seen)
        driver_station(held=False)
        driver_station(held=True)
        self.assertEqual(lift.seen.height_rot, 0.5)

    # REQ-OPS-55
    def test_operator_arming_blocker_tracks_held_input_and_clears_on_arm(self):
        class BlockerOperator:
            def __init__(self, node):
                self.ds = talos.DriverStation(node)
                self.arming = talos.OperatorArming(node)
                self.last_blocker = talos.ArmingBlocker()
                self.last_armed = False

            def update(self):
                stick = self.ds.joystick(0)
                if stick.connected and stick.button(1):
                    blocker = talos.ArmingBlocker(
                        talos.ArmingInputKind.BUTTON, 0, 1)
                    self.arming.evaluate(False, blocker)
                elif stick.connected and abs(stick.axis(1)) > 0.1:
                    blocker = talos.ArmingBlocker(
                        talos.ArmingInputKind.AXIS, 0, 1)
                    self.arming.evaluate(False, blocker)
                else:
                    self.arming.evaluate(True, talos.ArmingBlocker())
                self.last_armed = self.arming.armed
                self.last_blocker = self.arming.blocker

        sim = Sim.from_toml(ROBOT_TOML)
        op = sim.add(BlockerOperator, "planner")
        hw = SimHardware(sim)
        hw.enable()

        hw.set_driver_station(
            enabled=True, teleop=True,
            joysticks=[Stick(axes=(0.0, 0.75))])
        sim.step(0.04)
        self.assertFalse(op.last_armed)
        self.assertEqual(
            op.last_blocker,
            talos.ArmingBlocker(talos.ArmingInputKind.AXIS, 0, 1))

        hw.set_driver_station(
            enabled=True, teleop=True,
            joysticks=[Stick(buttons=(1,))])
        sim.step(0.04)
        self.assertFalse(op.last_armed)
        self.assertEqual(
            op.last_blocker,
            talos.ArmingBlocker(talos.ArmingInputKind.BUTTON, 0, 1))

        hw.set_driver_station(enabled=True, teleop=True, joysticks=[Stick()])
        sim.step(0.04)
        self.assertTrue(op.last_armed)
        self.assertEqual(op.last_blocker, talos.ArmingBlocker())

        # Once armed, holding a button leaves blocker cleared.
        hw.set_driver_station(
            enabled=True, teleop=True, joysticks=[Stick(buttons=(1,))])
        sim.step(0.04)
        self.assertTrue(op.last_armed)
        self.assertEqual(op.last_blocker, talos.ArmingBlocker())

        # ArmingBlocker validates int8 port (-128..127) and int16 index
        # (-32768..32767) instead of silently wrapping in ctypes.
        self.assertEqual(
            talos.ArmingBlocker(talos.ArmingInputKind.AXIS, -128, -32768).port,
            -128)
        self.assertEqual(
            talos.ArmingBlocker(talos.ArmingInputKind.AXIS, 127, 32767).index,
            32767)
        for bad_port in (-129, 128, 256, True):
            with self.assertRaises(ValueError):
                talos.ArmingBlocker(talos.ArmingInputKind.AXIS, bad_port, 0)
        for bad_index in (-32769, 32768, 65536, False):
            with self.assertRaises(ValueError):
                talos.ArmingBlocker(talos.ArmingInputKind.AXIS, 0, bad_index)


class StateMachineTest(unittest.TestCase):
    """Transitions are StateMachine's (node_test.cc); these prove each
    accessor crosses the ABI as the state's Python type."""

    def test_enum_states_cross_the_abi(self):
        sim = Sim.from_toml(ROBOT_TOML)
        stepper = sim.add(Stepper, "planner")
        sim.step(0.045)
        idle, moving = Stepper.State.IDLE, Stepper.State.MOVING
        # go(MOVING) in tick 2's update(); tick 3 is the first in MOVING.
        self.assertEqual(stepper.seen[-2:],
                         [(2, idle, False), (3, moving, True)])
        self.assertIs(stepper.sm.previous, idle)
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


class ParamTest(unittest.TestCase):
    """Lookup, fallbacks and type checks are the C++ Node's (node_test.cc);
    these prove the kind each default asks for, and each kind's value, cross
    talos_node_param and talos_node_param_of as their Python types."""

    def test_each_kind_reads_as_its_python_type(self):
        seen = {}

        class Scalars:
            def __init__(self, node):
                seen["gain"] = node.param("gain", 1.0)
                seen["mode"] = node.param("mode")
                seen["retries"] = node.param("retries", 0)
                seen["verbose"] = node.param("verbose")
                seen["absent"] = node.param("absent", 7.5)

        class Lists:
            def __init__(self, node):
                seen["cameras"] = node.param("cameras")
                seen["offsets_m"] = node.param("offsets_m", [0.0])
                seen["no_cameras"] = node.param("no_cameras", ["x"])
                seen["absent_list"] = node.param("absent", ["left"])

        sim = Sim.from_toml(ROBOT_TOML)
        sim.add(Scalars, "follower")
        sim.add(Lists, "vision")
        self.assertEqual(seen, {
            "gain": 2.0, "mode": "fast", "retries": 3, "verbose": True,
            "absent": 7.5, "cameras": ["front", "back"],
            "offsets_m": [0.5, 1.0, -2.25], "no_cameras": [],
            "absent_list": ["left"]})
        self.assertEqual(
            [type(seen[key]) for key in ("retries", "verbose")], [int, bool])
        self.assertEqual([type(v) for v in seen["offsets_m"]], [float] * 3)

        def reading(key, *default):
            class Reader:
                def __init__(self, node):
                    node.param(key, *default)
            return Reader

        # Each refusal is C++'s, reached through the kind the default asked
        # for; a list default of mixed types never leaves Python.
        for key, default, error in (
                ("absent", (), talos.TalosError),
                ("offsets_m", (0,), talos.TalosError),
                ("cameras", ([0.0],), talos.TalosError),
                ("cameras", (["a", 1.0],), TypeError)):
            with self.subTest(key=key, default=default):
                with self.assertRaises(error):
                    Sim.from_toml(ROBOT_TOML).add(reading(key, *default),
                                                  "vision")

    def test_param_of_reads_another_subsystem(self):
        seen = {}

        class Reader:
            def __init__(self, node):
                seen["offsets_m"] = node.param_of("vision", "offsets_m")
                seen["absent"] = node.param_of("follower", "absent", 7.5)

        sim = Sim.from_toml(ROBOT_TOML)
        sim.add(Reader, "planner")
        self.assertEqual(seen, {"offsets_m": [0.5, 1.0, -2.25],
                                "absent": 7.5})

        class Missing:
            def __init__(self, node):
                node.param_of("absent", "gain", 1.0)

        with self.assertRaises(talos.TalosError):
            Sim.from_toml(ROBOT_TOML).add(Missing, "planner")


class TrajectoryPathTest(unittest.TestCase):
    """Resolution is the C++ config's (config_test.cc); this proves a Python
    node gets the path and the refusal through the C ABI.
    REQ-CONTRACT-14 REQ-CONTRACT-31"""

    def test_a_node_opens_its_configured_trajectory(self):
        traj = Path(os.environ["TEST_TMPDIR"]) / "two_piece.traj"
        traj.write_text('{"version": 3}\n')
        toml = (ROBOT_TOML + "[subsystems.planner.trajectories]\n"
                f'two_piece = "{traj}"\n')
        opened = {}

        class Auto:
            def __init__(self, node):
                with open(node.trajectory_path("two_piece")) as f:
                    opened["two_piece"] = json.load(f)

        Sim.from_toml(toml).add(Auto, "planner")
        self.assertEqual(opened, {"two_piece": {"version": 3}})

        class Undeclared:
            def __init__(self, node):
                node.trajectory_path("three_piece")

        with self.assertRaisesRegex(talos.TalosError,
                                    r"planner.*trajectories\.three_piece"):
            Sim.from_toml(toml).add(Undeclared, "planner")


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
        def tampered(raw):
            msg = goal_type.decode(goal._bytes)
            msg._bytes = raw
            return msg

        mismatches = (
            message_type(changed_hash).decode(goal._bytes),
            tampered(goal._bytes[:-1]),
            tampered(goal._bytes + b"\0"),
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

    # REQ-CONTRACT-14: passing both a received message and **fields raises
    # TypeError in every send/publish path instead of silently dropping fields;
    # publish_target still accepts mode_since_us alongside a message.
    def test_send_and_publish_reject_message_together_with_fields(self):
        from talos._messages import message_type

        class Outputs:
            def __init__(self, node):
                self.publisher = node.publish(Goal, "/combo/goal")
                self.lane = talos.TargetLane(node, Goal, "lift", "teleop")

        sim = Sim.from_toml(ROBOT_TOML)
        outputs = sim.add(Outputs, "planner")
        goal_type = message_type(Goal)
        goal = goal_type.decode(goal_type.encode(
            {"height_rot": 2.5, "enabled": True, "tick_count": 0}))

        paths = (
            ("publisher",
             lambda **kw: outputs.publisher.send(goal, **kw),
             "talos_publisher_send"),
            ("lane",
             lambda **kw: outputs.lane.send(goal, **kw),
             "talos_target_lane_send"),
            ("sim",
             lambda **kw: sim.publish(Goal, "/combo/injected", goal, **kw),
             "talos_sim_publish_raw"),
            ("sim_target",
             lambda **kw: sim.publish_target(Goal, "lift", "align", goal, **kw),
             "talos_sim_publish_target"),
        )
        for name, call, symbol in paths:
            with self.subTest(path=name):
                with mock.patch.object(talos.lib, symbol) as native:
                    with self.assertRaises(TypeError):
                        call(height_rot=9.0)
                    native.assert_not_called()

        # publish_target accepts mode_since_us with a forwarded message, and
        # still rejects extra message fields alongside both.
        sim.publish_target(Goal, "lift", "align", goal, mode_since_us=42)
        self.assertEqual(sim.latest_target(Goal, "lift", "align"), goal)
        with mock.patch.object(talos.lib, "talos_sim_publish_target") as native:
            with self.assertRaises(TypeError):
                sim.publish_target(Goal, "lift", "align", goal,
                                   mode_since_us=42, height_rot=9.0)
            native.assert_not_called()

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

    # REQ-SAFE-27: publish and publish_bytes refuse any /hw/request topic at
    # construction, direct or remapped.
    def test_publish_and_publish_bytes_refuse_hardware_request_topics(self):
        toml = ROBOT_TOML + """
[subsystems.remapped.remap]
state = "/hw/request/remapped"
[subsystems.remapped_bytes.remap]
frames = "/hw/request/remapped_bytes"
"""
        cases = (
            ("planner", lambda n: n.publish(Goal, "/hw/request/planner")),
            ("planner", lambda n: n.publish_bytes("/hw/request/planner", 64)),
            ("remapped", lambda n: n.publish(Goal, "state")),
            ("remapped_bytes", lambda n: n.publish_bytes("frames", 64)),
        )
        for subsystem, register in cases:
            with self.subTest(subsystem=subsystem):
                class BadWriter:
                    def __init__(self, node):
                        register(node)

                sim = Sim.from_toml(toml)
                with self.assertRaisesRegex(talos.TalosError, r"/hw/request"):
                    sim.add(BadWriter, subsystem)


class LoadTest(unittest.TestCase):
    # REQ-CONTRACT-14: talos.load() disambiguates same-basename files by path
    # hash, raises FileNotFoundError/ValueError on invalid paths instead of
    # AttributeError, and cleans sys.modules when exec_module fails.
    def test_load_disambiguates_same_basename_files(self):
        root = Path(os.environ["TEST_TMPDIR"]) / "load_same_basename"
        dir_a = root / "a"
        dir_b = root / "b"
        dir_a.mkdir(parents=True, exist_ok=True)
        dir_b.mkdir(parents=True, exist_ok=True)
        file_a = dir_a / "node.py"
        file_b = dir_b / "node.py"
        file_a.write_text('VALUE = "from_a"\n', encoding="utf-8")
        file_b.write_text('VALUE = "from_b"\n', encoding="utf-8")

        mod_a = talos.load(str(file_a))
        mod_b = talos.load(str(file_b))
        self.addCleanup(sys.modules.pop, mod_a.__name__, None)
        self.addCleanup(sys.modules.pop, mod_b.__name__, None)

        self.assertNotEqual(mod_a.__name__, mod_b.__name__)
        self.assertIs(sys.modules[mod_a.__name__], mod_a)
        self.assertIs(sys.modules[mod_b.__name__], mod_b)
        self.assertEqual(mod_a.VALUE, "from_a")
        self.assertEqual(mod_b.VALUE, "from_b")

    def test_load_rejects_missing_and_non_python_paths(self):
        root = Path(os.environ["TEST_TMPDIR"]) / "load_invalid"
        root.mkdir(parents=True, exist_ok=True)
        missing = root / "does_not_exist.py"
        txt = root / "notes.txt"
        txt.write_text("VALUE = 1\n", encoding="utf-8")

        with self.assertRaises(FileNotFoundError):
            talos.load(str(missing))
        with self.assertRaises(ValueError):
            talos.load(str(txt))
        with self.assertRaises(ValueError):
            talos.load(str(root))

    def test_load_removes_module_from_sys_modules_on_exec_failure(self):
        root = Path(os.environ["TEST_TMPDIR"]) / "load_fail"
        root.mkdir(parents=True, exist_ok=True)
        broken = root / "broken.py"
        broken.write_text(
            'PARTIAL = 1\nraise RuntimeError("broken module")\n',
            encoding="utf-8",
        )
        before = set(sys.modules)
        with self.assertRaisesRegex(RuntimeError, "broken module"):
            talos.load(str(broken))
        self.assertEqual(set(sys.modules), before)


if __name__ == "__main__":
    unittest.main()
