"""Safety, lifetime, and C-ABI parity regression tests for talos Python bindings.

Covers:
- RULES.md 2 / REQ-SAFE-02: talos_publisher_send refuses /hw/request/<node>
- UAF prevention: failed Sim.add() and talos.run() invalidate Node._native
- Subscription.fresh & HardwareLink.fresh: timedelta, NaN, negative, inf, huge
- Sentinel-returning ABI getters propagate C errors via _abi.value()
- DriverStation set_driver_station rejects out-of-range alliance/station/pov
- TargetLane rejects non-OperatorArming; publish_bytes rejects > 0xFFFFFFFF;
  HardwareLink.enabled parity; mechanisms.py getters raise after Sim.remove()
"""

import ctypes
from datetime import timedelta
import io
import math
import os
import tempfile
import unittest
from unittest import mock

from control.PositionTarget import PositionTarget
from control.VelocityTarget import VelocityTarget
from demo.Goal import Goal
import talos
from talos import _abi
from talos.mechanisms import (PositionMechanism, RollerWithSensor,
                              VelocityMechanism)
from talos.testing import Sim, SimHardware, Stick

ROBOT_TOML = """
[robot]
period_us = 10000
command_timeout_us = 100000

[subsystems.planner]
height_rot = 1.0

[subsystems.roller.motors.wheel]
can_id = 1
[subsystems.roller.targets]
teleop = ["teleop"]

[subsystems.arm.motors.arm]
can_id = 2
feedback = "RemoteCANcoder"
feedback_sensor = "arm_cc"
position_frame = "absolute"
[subsystems.arm.sensors.arm_cc]
type = "CANcoder"
can_id = 3
[subsystems.arm.arm]
tolerance_rot = 0.01
gravity_v = 0.5
units_per_rot = 360.0

[subsystems.shooter.motors.flywheel]
can_id = 4
max_velocity_rps = 80.0
[subsystems.shooter.flywheel]
at_speed_entry_tick_count = 3

[subsystems.intake.motors.intake_motor]
can_id = 5
[subsystems.intake.digital_inputs.beam]
dio = 0
[subsystems.intake.intake_motor]
sensor = "beam"
intake_v = 6.0
hold_v = 1.0
eject_v = -6.0
eject_us = 100000
"""


class SafetyParityTest(unittest.TestCase):
    # REQ-SAFE-02: Python/C-ABI cannot bypass the devices flush by calling
    # talos_publisher_send on the devices layer's /hw/request/<node> slot.
    def test_publisher_send_refuses_hardware_request_slot(self):
        sim = Sim.from_toml(ROBOT_TOML)
        hw = SimHardware(sim)
        results = []

        class Forged:
            def __init__(self, node):
                self.node = node
                # First output device registers /hw/request/roller at slot 0.
                self.wheel = talos.Motor(node, "wheel")

            def update(self):
                if results:
                    return
                # sizeof(talos::hardware::Packet) == 1436.
                payload = bytes(1436)
                status = _abi.lib.talos_publisher_send(
                    self.node._handle, 0, payload, len(payload))
                err = _abi.lib.talos_last_error().decode()
                results.append((status, err))

        sim.add(Forged, "roller")
        sim.step(0.02)
        self.assertEqual(len(results), 1)
        self.assertEqual(results[0][0], _abi.ERR)
        self.assertIn("hardware request", results[0][1])
        self.assertEqual(hw.motor_request("roller", "wheel").mode,
                         talos.Mode.NEUTRAL)

    # REQ-CONTRACT-14: a failed Sim.add() invalidates the Python Node handle
    # and wrappers built before the failure so surviving references raise
    # TalosError instead of reaching freed C++ memory (UAF).
    def test_failed_sim_add_invalidates_node_and_wrappers(self):
        sim = Sim.from_toml(ROBOT_TOML)
        stashed = []

        class FailingInit:
            def __init__(self, node):
                sub = node.subscribe(Goal, "/planner/goal")
                stashed.append((node, sub))
                raise ValueError("init boom")

        with self.assertRaisesRegex(ValueError, "init boom"):
            sim.add(FailingInit, "planner")

        node, sub = stashed[0]
        self.assertIsNone(node._native)
        self.assertEqual(node._callbacks, [])
        self.assertIsNone(node._owner)
        with self.assertRaises(talos.TalosError):
            _ = node.name
        with self.assertRaises(talos.TalosError):
            _ = sub.has_value

    def test_failed_set_update_invalidates_node(self):
        sim = Sim.from_toml(ROBOT_TOML)
        stashed = []

        class BadUpdateAttr:
            def __init__(self, node):
                stashed.append(node)

            @property
            def update(self):
                raise RuntimeError("update attr boom")

        with self.assertRaisesRegex(RuntimeError, "update attr boom"):
            sim.add(BadUpdateAttr, "planner")

        node = stashed[0]
        self.assertIsNone(node._native)
        self.assertIsNone(node._owner)
        with self.assertRaises(talos.TalosError):
            _ = node.name

    # REQ-CONTRACT-14: talos.run() invalidates Node._native on normal return
    # and when a factory exception propagates, preventing post-run UAF.
    def test_run_invalidates_node_after_return_and_exception(self):
        with tempfile.TemporaryDirectory() as tmp:
            cfg = os.path.join(tmp, "robot.toml")
            with open(cfg, "w", encoding="utf-8") as handle:
                handle.write(ROBOT_TOML)

            stashed_ok = []

            class OkNode:
                def __init__(self, node):
                    stashed_ok.append(node)

                def update(self):
                    pass

            with mock.patch("sys.stdout", new_callable=io.StringIO):
                code = talos.run(
                    OkNode, "planner", "//talOS/python:probe",
                    argv=["probe", "--describe", "--config", cfg])
            self.assertEqual(code, 0)
            self.assertEqual(len(stashed_ok), 1)
            self.assertIsNone(stashed_ok[0]._native)
            self.assertEqual(stashed_ok[0]._callbacks, [])
            with self.assertRaises(talos.TalosError):
                _ = stashed_ok[0].name

            stashed_err = []

            class ErrNode:
                def __init__(self, node):
                    stashed_err.append(node)
                    raise ValueError("run init boom")

            with self.assertRaisesRegex(ValueError, "run init boom"):
                talos.run(
                    ErrNode, "planner", "//talOS/python:probe",
                    argv=["probe", "--describe", "--config", cfg])
            self.assertEqual(len(stashed_err), 1)
            self.assertIsNone(stashed_err[0]._native)
            with self.assertRaises(talos.TalosError):
                _ = stashed_err[0].name

    # REQ-CONTRACT-14: Subscription.fresh and HardwareLink.fresh accept
    # timedelta, return False for NaN and negative bounds (after handle/index
    # checks), and clamp +inf / huge bounds to INT64_MAX instead of wrapping
    # negative to command_timeout_us.
    def test_fresh_handles_timedelta_nan_inf_and_huge_bounds(self):
        sim = Sim.from_toml(ROBOT_TOML)
        hw = SimHardware(sim)

        class Watcher:
            def __init__(self, node):
                self.node = node
                self.sub = node.subscribe(Goal, "/planner/goal")
                self.link = talos.HardwareLink(node)

        w = sim.add(Watcher, "planner")
        sim.publish(Goal, "/planner/goal", height_rot=1.0, enabled=True,
                    tick_count=1)
        sim.step(0.01)
        # Age both the topic message and hardware state past command_timeout_us
        # (100 ms): lose hardware states so HardwareLink is also stale (> 100 ms).
        hw.lose_states(20)
        sim.step(0.15)

        for fresh_fn in (w.sub.fresh, w.link.fresh):
            with self.subTest(fn=fresh_fn.__qualname__):
                self.assertFalse(fresh_fn())  # past 100 ms command_timeout_us
                self.assertTrue(fresh_fn(timedelta(seconds=1.0)))
                self.assertFalse(fresh_fn(timedelta(milliseconds=50)))
                self.assertFalse(fresh_fn(timedelta(seconds=-1)))
                self.assertTrue(fresh_fn(timedelta.max))
                self.assertFalse(fresh_fn(math.nan))
                self.assertFalse(fresh_fn(-1.0))
                self.assertFalse(fresh_fn(-math.inf))
                self.assertTrue(fresh_fn(math.inf))
                self.assertTrue(fresh_fn(1e15))
                self.assertTrue(fresh_fn(1 << 100))

        # Handle check must run even for negative / NaN max_age.
        sim.remove("planner")
        for fresh_fn in (w.sub.fresh, w.link.fresh):
            with self.subTest(removed_fn=fresh_fn.__qualname__):
                with self.assertRaises(talos.TalosError):
                    fresh_fn(-1.0)
                with self.assertRaises(talos.TalosError):
                    fresh_fn(math.nan)

    # REQ-CONTRACT-14: sentinel-returning ABI getters check talos_last_error()
    # via _abi.value() so C errors raise TalosError instead of returning
    # sentinels, while valid sentinel results (None session_dir, 0 clock/tick)
    # still succeed.
    def test_sentinel_getters_surface_c_errors(self):
        sim = Sim.from_toml(ROBOT_TOML)

        class Probe:
            def __init__(self, node):
                self.node = node
                self.sub = node.subscribe(Goal, "/planner/goal")

        probe = sim.add(Probe, "planner")
        node = probe.node
        # Valid sentinel values must not raise.
        self.assertIsNone(node.session_dir)
        self.assertEqual(node.now_ns, 0)
        self.assertEqual(node.now, 0.0)
        self.assertEqual(node.tick, 0)
        self.assertEqual(sim.now_ns, 0)
        self.assertEqual(sim.now, 0.0)
        self.assertEqual(sim.driver_station_generation, 0)

        # Pass a real NULL pointer (c_void_p(0), which is not None) to trigger
        # C's AsNode(nullptr) / AsSim(nullptr) error with sentinel returns.
        real_node = node._native
        try:
            node._native = ctypes.c_void_p(0)
            for attr in ("name", "now", "now_ns", "tick", "period",
                         "config_path", "session_dir"):
                with self.subTest(node_attr=attr):
                    with self.assertRaisesRegex(talos.TalosError,
                                                "null TalosNode"):
                        getattr(node, attr)
        finally:
            node._native = real_node

        # Subscription.age with an out-of-range index after has_value passes.
        real_index = probe.sub._index
        try:
            probe.sub._index = 999
            with mock.patch.object(talos.Subscription, "has_value",
                                   new_callable=mock.PropertyMock,
                                   return_value=True):
                with self.assertRaises(talos.TalosError):
                    _ = probe.sub.age
        finally:
            probe.sub._index = real_index

        real_sim = sim._handle
        try:
            sim._handle = ctypes.c_void_p(0)
            for attr in ("now", "now_ns", "driver_station_generation"):
                with self.subTest(sim_attr=attr):
                    with self.assertRaisesRegex(talos.TalosError,
                                                "null TalosSim"):
                        getattr(sim, attr)
        finally:
            sim._handle = real_sim

    # REQ-CONTRACT-13: set_driver_station rejects out-of-range alliance,
    # station, and POV values rather than silently casting/truncating.
    def test_set_driver_station_rejects_out_of_range_values(self):
        sim = Sim.from_toml(ROBOT_TOML)
        hw = SimHardware(sim)
        with self.assertRaisesRegex(talos.TalosError, "alliance"):
            hw.set_driver_station(alliance=99)
        with self.assertRaisesRegex(talos.TalosError, "station"):
            hw.set_driver_station(station=-1)
        with self.assertRaisesRegex(talos.TalosError, "station"):
            hw.set_driver_station(station=256)
        with self.assertRaisesRegex(talos.TalosError, "pov"):
            hw.set_driver_station(joysticks=[Stick(povs=(40000,))])
        with self.assertRaisesRegex(talos.TalosError, "pov"):
            hw.set_driver_station(joysticks=[Stick(povs=(-40000,))])

    # REQ-CONTRACT-14: TargetLane refuses non-OperatorArming objects even when
    # they belong to the same node (e.g. a Motor with a matching _index).
    def test_target_lane_rejects_non_operator_arming(self):
        sim = Sim.from_toml(ROBOT_TOML)

        class BadArming:
            def __init__(self, node):
                self.motor = talos.Motor(node, "wheel")
                self.lane = talos.TargetLane(
                    node, Goal, "roller", "teleop", arming=self.motor)

        with self.assertRaisesRegex(TypeError, "OperatorArming"):
            sim.add(BadArming, "roller")

    # REQ-SAFE-14, REQ-CONTRACT-14: Python operator producers writing a
    # teleop-policy target lane require OperatorArming, matching C++ semantics
    # across init order, raw publish, and lazy DriverStation reads in update().
    def test_operator_producer_teleop_lane_requires_operator_arming(self):
        class DsBeforeLane:
            def __init__(self, node):
                self.ds = talos.DriverStation(node)
                self.lane = talos.TargetLane(node, Goal, "roller", "teleop")

        class LaneBeforeDs:
            def __init__(self, node):
                self.lane = talos.TargetLane(node, Goal, "roller", "teleop")
                self.ds = talos.DriverStation(node)

        class RawPublish:
            def __init__(self, node):
                self.ds = talos.DriverStation(node)
                self.pub = node.publish(Goal, "/roller/target/teleop")

        for cls in (DsBeforeLane, LaneBeforeDs, RawPublish):
            with self.subTest(cls=cls.__name__):
                sim = Sim.from_toml(ROBOT_TOML)
                with self.assertRaisesRegex(
                        talos.TalosError,
                        r"/roller/target/teleop.*OperatorArming.*REQ-SAFE-14"):
                    sim.add(cls, "driver")

        # Lazy DriverStation construction in update() throws on every retry.
        class LazyDs:
            def __init__(self, node):
                self.node = node
                self.lane = talos.TargetLane(node, Goal, "roller", "teleop")
                self.refused = 0
                self.succeeded = 0

            def update(self):
                for _ in range(2):
                    try:
                        ds = talos.DriverStation(self.node)
                        _ = ds.enabled
                        self.succeeded += 1
                    except talos.TalosError as err:
                        if "OperatorArming" in str(err):
                            self.refused += 1

        sim_lazy = Sim.from_toml(ROBOT_TOML)
        lazy = sim_lazy.add(LazyDs, "driver")
        sim_lazy.step(0.02)
        self.assertEqual(lazy.succeeded, 0)
        self.assertGreaterEqual(lazy.refused, 2)

        # Allowed: auto lane with DriverStation, coordinator without
        # DriverStation, and armed teleop lane with DriverStation.
        class AutoProducer:
            def __init__(self, node):
                self.ds = talos.DriverStation(node)
                self.lane = talos.TargetLane(node, Goal, "roller", "auto")

        class Coordinator:
            def __init__(self, node):
                self.target = talos.Target(node, Goal)
                self.lane = talos.TargetLane(node, Goal, "roller", "teleop")

        class ArmedOperator:
            def __init__(self, node):
                self.ds = talos.DriverStation(node)
                self.arming = talos.OperatorArming(node)
                self.lane = talos.TargetLane(
                    node, Goal, "roller", "teleop", arming=self.arming)

        sim_ok = Sim.from_toml(ROBOT_TOML)
        sim_ok.add(AutoProducer, "autonomous")
        sim_ok.add(Coordinator, "superstructure")
        sim_ok.add(ArmedOperator, "driver")

    # REQ-CONTRACT-14: Node.publish_bytes rejects uint32 overflow instead of
    # silently wrapping max_bytes or slots.
    def test_publish_bytes_rejects_uint32_overflow(self):
        sim = Sim.from_toml(ROBOT_TOML)

        class BadBytes:
            def __init__(self, node):
                node.publish_bytes("frames", (1 << 32) + 64)

        class BadSlots:
            def __init__(self, node):
                node.publish_bytes("frames", 64, slots=(1 << 32) + 16)

        with self.assertRaises(ValueError):
            sim.add(BadBytes, "planner")
        with self.assertRaises(ValueError):
            sim.add(BadSlots, "planner")

    # REQ-CONTRACT-14: HardwareLink.enabled matches talos.robot_enabled(node).
    def test_hardware_link_enabled_parity(self):
        sim = Sim.from_toml(ROBOT_TOML)
        hw = SimHardware(sim)

        class Reader:
            def __init__(self, node):
                self.node = node
                self.link = talos.HardwareLink(node)

        reader = sim.add(Reader, "planner")
        sim.step(0.02)
        self.assertFalse(reader.link.enabled)
        self.assertEqual(reader.link.enabled, talos.robot_enabled(reader.node))

        hw.enable(True)
        sim.step(0.02)
        self.assertTrue(reader.link.enabled)
        self.assertEqual(reader.link.enabled, talos.robot_enabled(reader.node))

        hw.enable(False)
        sim.step(0.02)
        self.assertFalse(reader.link.enabled)

    # REQ-CONTRACT-14: mechanisms.py property getters and state() validate the
    # live node handle and raise TalosError after Sim.remove().
    def test_mechanisms_getters_raise_after_remove(self):
        sim = Sim.from_toml(ROBOT_TOML)

        class ArmNode:
            def __init__(self, node):
                self.mech = PositionMechanism(node, "arm")
                self.target = talos.Target(node, PositionTarget)

        class ShooterNode:
            def __init__(self, node):
                self.mech = VelocityMechanism(node, "flywheel")
                self.target = talos.Target(node, VelocityTarget)

        class IntakeNode:
            def __init__(self, node):
                self.mech = RollerWithSensor(node, "intake_motor")

        arm = sim.add(ArmNode, "arm").mech
        shooter = sim.add(ShooterNode, "shooter").mech
        intake = sim.add(IntakeNode, "intake").mech

        sim.remove("arm")
        for prop in ("driving", "phase", "fault", "homed", "at_goal",
                     "connected", "position_rot", "position_units",
                     "velocity_rps", "goal_rot", "gravity_v",
                     "tolerance_rot"):
            with self.subTest(arm_prop=prop):
                with self.assertRaises(talos.TalosError):
                    getattr(arm, prop)
        with self.assertRaises(talos.TalosError):
            arm.state()

        sim.remove("shooter")
        for prop in ("driving", "phase", "at_speed", "connected",
                     "velocity_rps", "goal_rps", "commanded_rps"):
            with self.subTest(shooter_prop=prop):
                with self.assertRaises(talos.TalosError):
                    getattr(shooter, prop)
        with self.assertRaises(talos.TalosError):
            shooter.state()

        sim.remove("intake")
        for prop in ("driving", "phase", "has_piece", "sensor_connected",
                     "output_v"):
            with self.subTest(intake_prop=prop):
                with self.assertRaises(talos.TalosError):
                    getattr(intake, prop)

    # REQ-CONTRACT-13, REQ-CONTRACT-14, REQ-SAFE-04: Python ctypes bindings
    # reject out-of-range integer arguments instead of silently wrapping modulo
    # 2**32 or 2**64 into valid C ABI values (e.g. slot=1<<32 -> slot=0,
    # position_age_us=1<<32 -> 0 fresh, period/step wrap -> small positive).
    def test_ctypes_integer_bounds_reject_silent_wraparound(self):
        sim = Sim.from_toml(ROBOT_TOML)
        hw = SimHardware(sim)
        hw.enable(True)
        wrap_s = (1 << 64) / 1e9 + 0.01

        class SlotOverflow:
            def __init__(self, node):
                talos.allow_untargeted_outputs(node)
                self.motor = talos.Motor(node, "wheel")
                self.sm = talos.StateMachine(node, 0)
                self.slot_exc = None

            def update(self):
                if self.slot_exc is None:
                    try:
                        self.motor.set_velocity(10.0, slot=(1 << 32))
                    except (ValueError, OverflowError) as exc:
                        self.slot_exc = exc

        roller = sim.add(SlotOverflow, "roller")
        sim.step(0.02)
        self.assertIsNotNone(roller.slot_exc)
        self.assertEqual(hw.motor_request("roller", "wheel").mode,
                         talos.Mode.NEUTRAL)

        with self.assertRaises((ValueError, OverflowError)):
            roller.sm.go(1 << 63)
        with self.assertRaises((ValueError, OverflowError)):
            roller.sm.go(-(1 << 63) - 1)

        class EveryOverflow:
            def __init__(self, node):
                node.every(wrap_s, lambda: None)

        with self.assertRaises((ValueError, OverflowError)):
            sim.add(EveryOverflow, "planner")

        class StateOverflow:
            def __init__(self, node):
                talos.StateMachine(node, 1 << 63)

        with self.assertRaises((ValueError, OverflowError)):
            sim.add(StateOverflow, "planner")

        with self.assertRaises((ValueError, OverflowError)):
            sim.step(wrap_s)
        with self.assertRaises((ValueError, OverflowError)):
            sim.pause("roller", wrap_s)
        with self.assertRaises((ValueError, OverflowError)):
            hw.set_gateway_clock(offset=wrap_s, latency=0.0)
        with self.assertRaises((ValueError, OverflowError)):
            hw.set_motor("roller", "wheel", 0.0, 0.0, position_age_us=1 << 32)
        with self.assertRaises((ValueError, OverflowError)):
            hw.set_motor("roller", "wheel", 0.0, 0.0, velocity_age_us=-1)
        with self.assertRaises((ValueError, OverflowError)):
            hw.set_cancoder("arm", "arm_cc", 0.0, 0.0, position_age_us=1 << 32)
        with self.assertRaises((ValueError, OverflowError)):
            hw.lose_states(1 << 32)
        with self.assertRaises((ValueError, OverflowError)):
            hw.set_driver_station(station=1 << 32)
        with self.assertRaises((ValueError, OverflowError)):
            hw.set_driver_station(alliance=1 << 32)
        with self.assertRaises((ValueError, OverflowError)):
            hw.set_driver_station(joysticks=[Stick(povs=(1 << 32,))])
        with self.assertRaises((ValueError, OverflowError)):
            sim.publish_target(Goal, "roller", "teleop", mode_since_us=-1,
                               height_rot=1.0, enabled=True, tick_count=1)
        with self.assertRaises((ValueError, OverflowError)):
            sim.publish_target(Goal, "roller", "teleop", mode_since_us=1 << 64,
                               height_rot=1.0, enabled=True, tick_count=1)

    # REQ-CONTRACT-14: destroying a Sim (as during cyclic GC finalization)
    # destroys and invalidates attached SimHardware wrappers so subsequent calls
    # raise TalosError instead of dereferencing freed Sim memory (UAF).
    def test_sim_destroy_invalidates_attached_sim_hardware(self):
        sim = Sim.from_toml(ROBOT_TOML)
        hw = SimHardware(sim)
        sim.__del__()
        with self.assertRaises(talos.TalosError):
            hw.enable(True)
        with self.assertRaises(talos.TalosError):
            hw.motor_request("roller", "wheel")


if __name__ == "__main__":
    unittest.main()
