"""motor.py, the Python bench motor node, on talos.testing.Sim, configured by
configuration/robot_py.toml and its subsystems/motor.toml.

BenchMotorTest is the node test matrix, docs/TESTING.md §7 (the case names
are the matrix's, which //tools:check_node_matrix looks for), plus the cases
of node_test.cc it does not already cover. ParityTest runs the C++ node
(motor.h, hosted through libbench_motor.so) and motor.py on one input script
and requires the same /motor/state bytes and the same motor requests.
"""

import math
import os
import unittest

import talos
from motor import topics
from motor.ControlMode import ControlMode
from motor.MotorState import MotorState
from motor.MotorTarget import MotorTarget
from motor.TargetSource import TargetSource
from python.runfiles import runfiles
from talos import teleop
from talos.testing import Sim, SimHardware, Stick

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = runfiles.Create().Rlocation(os.environ["TALOS_TEST_CONFIG"])
bench = talos.load(os.path.join(HERE, "motor.py"))
CppBenchMotor = talos.cpp_node(
    "bench/main_processor/motor/libbench_motor.so", "motor")

NEUTRAL = talos.Mode.NEUTRAL
VOLTAGE = talos.Mode.VOLTAGE
VELOCITY = talos.Mode.VELOCITY

# subsystems/motor.toml's max_bench_voltage_v.
MAX_VOLTS = 2.0
# robot.toml's command_timeout_us, in seconds.
TIMEOUT_S = 0.1


class Bench:
    """One simulated bench: a motor node class on a Sim with SimHardware,
    driven the way node_test.cc's MotorTest drives the C++ node."""

    def __init__(self, cls):
        self.sim = Sim(CONFIG)
        self.sim.add(cls, "motor")
        self.hw = SimHardware(self.sim)

    def enable(self):
        """Gateway enabled, Driver Station enabled in teleop."""
        self.hw.enable()
        self.hw.set_driver_station(enabled=True, teleop=True)
        self.sim.step(0.05)

    def send(self, mode, voltage_v=0.0, velocity_rps=0.0, voltage_frac=0.0,
             mode_since_us=None):
        """One teleop target, stamped like a producer that has drained the
        newest Driver Station packet (or with `mode_since_us`), then 10 ms."""
        self.sim.publish_target(MotorTarget, "motor", "teleop",
                                mode_since_us=mode_since_us, mode=mode,
                                voltage_v=voltage_v,
                                velocity_rps=velocity_rps,
                                voltage_frac=voltage_frac)
        self.sim.step(0.01)

    def release(self):
        self.sim.release_target(MotorTarget, "motor", "teleop")
        self.sim.step(0.01)

    def request(self):
        """What the node asked for."""
        return self.hw.motor_request("motor", "kraken")

    def applied(self):
        """What the gateway applies, after its enable and expiry gates."""
        return self.hw.applied_motor("motor", "kraken")

    def state(self):
        return self.sim.latest(MotorState, topics.STATE)


class BenchMotorTest(unittest.TestCase):
    def setUp(self):
        self.bench = Bench(bench.BenchMotor)
        self.sim, self.hw = self.bench.sim, self.bench.hw

    def assert_drives(self, mode, demand):
        for name, request in (("request", self.bench.request()),
                              ("applied", self.bench.applied())):
            with self.subTest(output=name):
                self.assertEqual(request[:2], (mode, demand))

    def assert_neutral(self):
        self.assertEqual(self.bench.request().mode, NEUTRAL)
        self.assertEqual(self.bench.applied().mode, NEUTRAL)
        state = self.bench.state()
        self.assertEqual(state.mode, ControlMode.NEUTRAL)
        self.assertEqual(state.source, TargetSource.NONE)

    # REQ-SAFE-07
    def test_drives_at_a_live_lane(self):
        self.bench.enable()
        self.bench.send(ControlMode.VELOCITY, velocity_rps=10.0)
        self.assert_drives(VELOCITY, 10.0)
        state = self.bench.state()
        self.assertTrue(state.enabled)
        self.assertEqual(state.mode, ControlMode.VELOCITY)
        self.assertEqual(state.source, TargetSource.TELEOP)
        # An explicit NEUTRAL target drives nothing, whatever else it says.
        self.bench.send(ControlMode.NEUTRAL, voltage_v=2.0, velocity_rps=30.0)
        self.assert_neutral()

    def test_voltage_is_clamped_and_axis_uses_node_scale(self):
        self.bench.enable()
        self.bench.send(ControlMode.VOLTAGE, voltage_v=20.0)
        self.assert_drives(VOLTAGE, MAX_VOLTS)
        self.bench.send(ControlMode.VOLTAGE, voltage_v=-20.0)
        self.assertEqual(self.bench.request().demand, -MAX_VOLTS)
        self.bench.send(ControlMode.VOLTAGE, voltage_frac=0.25)
        self.assertEqual(self.bench.request().demand, 0.25 * MAX_VOLTS)
        self.assertEqual(self.bench.state().source, TargetSource.TELEOP)

    # REQ-SAFE-02
    def test_disabled_robot_goes_neutral(self):
        self.bench.enable()
        self.hw.set_motor("motor", "kraken", 0.0, 0.0, 0, 0, 48.5, 3.25)
        self.bench.send(ControlMode.VOLTAGE, voltage_v=1.0)
        self.assert_drives(VOLTAGE, 1.0)
        self.hw.enable(False)
        self.sim.step(0.01)
        self.assert_neutral()
        state = self.bench.state()  # still published, health still visible
        self.assertFalse(state.enabled)
        self.assertTrue(state.connected)
        self.assertEqual(state.temperature_degc, 48.5)
        self.assertAlmostEqual(state.supply_current_a, 3.3)

    # REQ-SAFE-07
    def test_stale_lane_goes_neutral(self):
        self.bench.enable()
        self.hw.set_motor("motor", "kraken", 0.0, 0.0, 0, 0, 48.5, 3.25)
        self.bench.send(ControlMode.VELOCITY, velocity_rps=10.0)
        self.assert_drives(VELOCITY, 10.0)
        self.sim.step(TIMEOUT_S + 0.01)
        self.assert_neutral()
        state = self.bench.state()  # still published, health still visible
        self.assertEqual(state.temperature_degc, 48.5)
        self.assertAlmostEqual(state.supply_current_a, 3.3)

    # REQ-SAFE-07
    def test_released_lane_goes_neutral(self):
        self.bench.enable()
        self.bench.send(ControlMode.VOLTAGE, voltage_v=1.0)
        self.assert_drives(VOLTAGE, 1.0)
        self.bench.release()
        self.assert_neutral()

    # REQ-SAFE-01
    def test_never_sent_goes_neutral(self):
        self.bench.enable()
        self.assert_neutral()
        self.assertTrue(self.bench.state().enabled)

    # REQ-SAFE-07
    def test_driver_station_loss_goes_neutral(self):
        self.bench.enable()
        self.bench.send(ControlMode.VELOCITY, velocity_rps=10.0)
        self.assert_drives(VELOCITY, 10.0)
        self.hw.stop_driver_station()
        for _ in range(16):  # the lane stays live past the DS view's expiry
            self.bench.send(ControlMode.VELOCITY, velocity_rps=10.0)
        self.assert_neutral()

    # REQ-CONTRACT-25
    def test_disconnected_feedback_goes_neutral(self):
        self.bench.enable()
        self.bench.send(ControlMode.VELOCITY, velocity_rps=10.0)
        self.assert_drives(VELOCITY, 10.0)
        self.hw.set_connected("motor", "kraken", False)
        self.bench.send(ControlMode.VELOCITY, velocity_rps=10.0)
        self.assert_neutral()
        state = self.bench.state()
        self.assertFalse(state.connected)
        self.assertFalse(state.acquisition_valid)

    # REQ-SAFE-04
    def test_non_finite_demand_goes_neutral(self):
        self.bench.enable()
        for field, mode in (("voltage_v", ControlMode.VOLTAGE),
                            ("voltage_frac", ControlMode.VOLTAGE),
                            ("velocity_rps", ControlMode.VELOCITY)):
            for value in (math.nan, math.inf):
                with self.subTest(field=field, value=value):
                    self.bench.send(mode, **{field: value})
                    self.assert_neutral()

    # REQ-CONTRACT-18
    def test_bad_parameter_fails_construction(self):
        for value in ("0.0", "-1.0", "nan", "inf"):
            with self.subTest(max_bench_voltage_v=value):
                sim = Sim.from_toml(
                    "[robot]\nperiod_us = 5000\n"
                    "command_timeout_us = 100000\n"
                    f"[subsystems.motor]\nmax_bench_voltage_v = {value}\n"
                    "[subsystems.motor.motors.kraken]\n"
                    'type = "TalonFX"\ncan_id = 1\n')
                with self.assertRaisesRegex((ValueError, talos.TalosError),
                                            "max_bench_voltage_v"):
                    sim.add(bench.BenchMotor, "motor")

    # REQ-SAFE-07
    def test_other_modes_lane_is_ignored(self):
        # The bench policy lists no autonomous source, so only the teleop
        # lane exists: it must not drive in autonomous.
        self.hw.enable()
        self.hw.set_driver_station(enabled=True, autonomous=True)
        self.sim.step(0.05)
        for _ in range(4):
            self.bench.send(ControlMode.VELOCITY, velocity_rps=10.0)
        self.assert_neutral()
        self.hw.set_driver_station(enabled=True, teleop=True)
        self.sim.step(0.05)
        self.bench.send(ControlMode.VELOCITY, velocity_rps=10.0)
        self.assert_drives(VELOCITY, 10.0)

    # REQ-SAFE-09
    def test_earlier_generation_target_is_ignored(self):
        self.bench.enable()
        self.bench.send(ControlMode.VELOCITY, velocity_rps=10.0)
        self.assert_drives(VELOCITY, 10.0)
        before = self.sim.driver_station_generation
        self.hw.set_driver_station(teleop=True)  # disabled
        self.sim.step(0.04)
        self.hw.set_driver_station(enabled=True, teleop=True)
        self.sim.step(0.02)
        self.assertNotEqual(self.sim.driver_station_generation, before)
        # A target its producer queued under the earlier enable.
        self.bench.send(ControlMode.VELOCITY, velocity_rps=10.0,
                        mode_since_us=before)
        self.assert_neutral()
        # The same target stamped with the current generation drives at once.
        self.bench.send(ControlMode.VELOCITY, velocity_rps=10.0)
        self.assert_drives(VELOCITY, 10.0)

    # matrix: n/a 12 (a Python tick allocates in the interpreter)

    def test_reports_injected_feedback_and_acquisition_ages(self):
        self.bench.enable()
        self.hw.set_motor("motor", "kraken", 4.5, -7.0, 2000, 1000, 42.5, 6.25)
        self.sim.step(0.01)
        state = self.bench.state()
        self.assertEqual(state.position_rot, 4.5)
        self.assertEqual(state.velocity_rps, -7.0)
        self.assertEqual(state.temperature_degc, 42.5)
        self.assertAlmostEqual(state.supply_current_a, 6.3)
        self.assertTrue(state.connected)
        self.assertTrue(state.acquisition_valid)
        self.assertEqual(state.position_age_us - state.velocity_age_us, 1000)
        self.hw.set_connected("motor", "kraken", False)
        self.sim.step(0.01)
        state = self.bench.state()
        self.assertFalse(state.connected)
        self.assertFalse(state.acquisition_valid)
        self.assertEqual(state.temperature_degc, 0.0)
        self.assertEqual(state.supply_current_a, 0.0)

    def test_reports_limit_switches_and_clears_them_on_stale_state(self):
        self.bench.enable()
        for forward, reverse in ((True, False), (False, True)):
            with self.subTest(forward=forward, reverse=reverse):
                self.hw.set_limit_switches("motor", "kraken", forward, reverse)
                self.sim.step(0.01)
                state = self.bench.state()
                self.assertEqual(state.forward_limit, forward)
                self.assertEqual(state.reverse_limit, reverse)
        self.hw.set_limit_switches("motor", "kraken", True, True)
        self.hw.lose_states(1000)
        self.sim.step(0.2)
        state = self.bench.state()
        self.assertFalse(state.forward_limit)
        self.assertFalse(state.reverse_limit)


class BenchTeleopTest(unittest.TestCase):
    """subsystems/motor.toml's [teleop] bindings through the framework
    teleop node to motor.py, as node_test.cc's BenchTeleop does in C++."""

    def test_configured_axis_buttons_release_and_disable(self):
        sim = Sim(CONFIG)
        sim.add(bench.BenchMotor, "motor")
        sim.add(teleop.Teleop, "teleop")
        hw = SimHardware(sim)
        hw.enable()

        def push(axis=0.0, buttons=(), enabled=True):
            stick = Stick(axes=(0.0, axis), buttons=buttons, button_count=2)
            hw.set_driver_station(enabled=enabled, teleop=True,
                                  joysticks=[stick])
            sim.step(0.02)
            return hw.motor_request("motor", "kraken")

        push()  # A neutral evaluation arms the producer.
        self.assertEqual(push(axis=-1.0)[:2], (VOLTAGE, MAX_VOLTS))
        self.assertEqual(push(axis=-1.0, buttons=(2,))[:2], (VELOCITY, 30.0))
        self.assertEqual(push(axis=-1.0, buttons=(1, 2)).demand, 10.0)
        self.assertEqual(push().mode, NEUTRAL)
        push(buttons=(1,))
        hw.enable(False)
        self.assertEqual(push(buttons=(1,), enabled=False).mode, NEUTRAL)


class ParityTest(unittest.TestCase):
    """The C++ node and motor.py, each on its own Sim, given one input
    script: every step must publish the same /motor/state bytes and leave
    the same requested and applied motor output."""

    @staticmethod
    def script(cls):
        b = Bench(cls)
        hw = b.hw
        steps = [
            ("disabled", lambda: b.sim.step(0.05)),
            ("enabled, never sent", b.enable),
            ("voltage above the clamp",
             lambda: b.send(ControlMode.VOLTAGE, voltage_v=20.0)),
            ("voltage below the clamp",
             lambda: b.send(ControlMode.VOLTAGE, voltage_v=-20.0)),
            ("voltage and axis",
             lambda: b.send(ControlMode.VOLTAGE, voltage_v=0.5,
                            voltage_frac=-1.5)),
            ("velocity", lambda: b.send(ControlMode.VELOCITY,
                                        velocity_rps=10.0)),
            ("explicit neutral",
             lambda: b.send(ControlMode.NEUTRAL, voltage_v=2.0)),
            ("feedback",
             lambda: (hw.set_motor("motor", "kraken", 4.5, -7.0, 2000, 1000,
                                   42.5, 6.25, 12.0),
                      b.send(ControlMode.VELOCITY, velocity_rps=-3.0))),
            ("non-finite voltage",
             lambda: b.send(ControlMode.VOLTAGE, voltage_v=math.nan)),
            ("released", b.release),
            ("stale lane", lambda: (b.send(ControlMode.VOLTAGE,
                                           voltage_frac=0.25),
                                    b.sim.step(TIMEOUT_S + 0.01))),
            ("limit switches",
             lambda: (hw.set_limit_switches("motor", "kraken", True, False),
                      b.sim.step(0.01))),
            ("disconnected",
             lambda: (hw.set_connected("motor", "kraken", False),
                      b.send(ControlMode.VELOCITY, velocity_rps=10.0))),
            ("reconnected",
             lambda: (hw.set_connected("motor", "kraken", True),
                      b.send(ControlMode.VELOCITY, velocity_rps=10.0))),
            ("hardware state lost",
             lambda: (hw.lose_states(1000), b.sim.step(0.2))),
            ("gateway disabled",
             lambda: (hw.enable(False),
                      b.send(ControlMode.VELOCITY, velocity_rps=10.0))),
        ]
        trace = []
        for name, step in steps:
            step()
            trace.append((name, b.state(), b.request(), b.applied()))
        return trace

    def test_publishes_and_requests_what_the_cpp_node_does(self):
        cpp = self.script(CppBenchMotor)
        py = self.script(bench.BenchMotor)
        for (name, *expected), (_, *actual) in zip(cpp, py):
            with self.subTest(step=name):
                self.assertIsNotNone(expected[0])
                self.assertEqual(actual, expected)


if __name__ == "__main__":
    unittest.main()
