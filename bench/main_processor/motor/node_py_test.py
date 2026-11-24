"""motor.py, the Python bench motor node, on talos.testing.Sim, configured by
testdata/robot.toml: configuration/robot.toml (whose subsystems/motor.toml
robot_py.toml also launches) plus a neighbor motor that stays connected.

node_test.cc carries the node test matrix (docs/TESTING.md §7) for the C++
node. ParityTest runs that C++ node (motor.h, hosted through
libbench_motor.so) and motor.py on SCENARIO through talos.parity, at the
config's 5 ms period and at 20 ms (a Python motor owner's minimum on a
robot, REQ-RT-23), which
requires the same /motor/state bytes and the same requested and applied
motor output after every step; the `matrix: parity` comments name the steps
that cover each case. What a parity run cannot compare is tested here:
construction (case 9), the framework teleop node driving motor.py, and
torque mode off at the committed config's cap (the scenario's config turns
it on).
"""

import math
import os
import unittest

import talos
from motor import topics
from motor.ControlMode import ControlMode
from motor.MotorState import MotorState
from motor.MotorTarget import MotorTarget
from python.runfiles import runfiles
from swerve_module.ModuleTarget import ModuleTarget
from talos import parity, teleop
from talos.testing import Sim, SimHardware, Stick

HERE = os.path.dirname(os.path.abspath(__file__))
_RUNFILES = runfiles.Create()
# testdata/robot.toml: torque mode on (foc, max_torque_current_a 5 A,
# max_bench_torque_current_a 2 A) and max_voltage 4 V, above the node's
# 2 V cap.
CONFIG = _RUNFILES.Rlocation(os.environ["TALOS_TEST_CONFIG"])
# configuration/robot.toml as committed: torque mode off.
BENCH_CONFIG = _RUNFILES.Rlocation(os.environ["TALOS_BENCH_CONFIG"])
bench = talos.load(os.path.join(HERE, "motor.py"))
CppBenchMotor = talos.cpp_node(
    "bench/main_processor/motor/libbench_motor.so", "motor")

NEUTRAL = talos.Mode.NEUTRAL
VOLTAGE = talos.Mode.VOLTAGE
VELOCITY = talos.Mode.VELOCITY
TORQUE_CURRENT = talos.Mode.TORQUE_CURRENT

# subsystems/motor.toml's max_bench_voltage_v.
MAX_VOLTS = 2.0
# robot.toml's command_timeout_us, in seconds.
TIMEOUT_S = 0.1


class BenchMotorTest(unittest.TestCase):
    # matrix: n/a 12 (a Python tick allocates in the interpreter)

    # REQ-CONTRACT-18. Unlike node_test.cc's, no row for a positive
    # max_bench_torque_current_a on a motor without foc: motor.py cannot
    # read the motor's config (see its docstring).
    def test_bad_parameter_fails_construction(self):
        for key, value in (("max_bench_voltage_v", "0.0"),
                           ("max_bench_voltage_v", "-1.0"),
                           ("max_bench_voltage_v", "nan"),
                           ("max_bench_voltage_v", "inf"),
                           ("max_bench_voltage_v", '"two"'),
                           ("max_bench_voltage_v", "true"),
                           ("max_bench_torque_current_a", "-1.0"),
                           ("max_bench_torque_current_a", "nan"),
                           ("max_bench_torque_current_a", "inf"),
                           ("max_bench_torque_current_a", '"two"'),
                           ("max_bench_torque_current_a", "true")):
            with self.subTest(key=key, value=value):
                sim = Sim.from_toml(
                    "[robot]\nperiod_us = 5000\n"
                    "command_timeout_us = 100000\n"
                    f"[subsystems.motor]\n{key} = {value}\n"
                    "[subsystems.motor.motors.kraken]\n"
                    'type = "TalonFX"\ncan_id = 1\n'
                    "foc = true\nmax_torque_current_a = 5.0\n")
                with self.assertRaisesRegex((ValueError, talos.TalosError),
                                            key):
                    sim.add(bench.BenchMotor, "motor")

    # REQ-SAFE-43: the committed bench config drives nothing for a torque
    # target, while a voltage target still drives. (ParityTest runs with
    # torque mode on.)
    def test_torque_mode_off_at_the_committed_cap(self):
        sim = Sim(BENCH_CONFIG)
        sim.add(bench.BenchMotor, "motor")
        hw = SimHardware(sim)
        hw.enable()
        hw.set_driver_station(enabled=True, teleop=True)
        sim.step(0.05)

        def send(mode, voltage_v=0.0, torque_current_a=0.0):
            sim.publish_target(MotorTarget, "motor", "teleop", mode=mode,
                               voltage_v=voltage_v, velocity_rps=0.0,
                               voltage_frac=0.0,
                               torque_current_a=torque_current_a)
            sim.step(0.01)

        send(ControlMode.TORQUE_CURRENT, torque_current_a=1.5)
        self.assertEqual(hw.motor_request("motor", "kraken").mode, NEUTRAL)
        self.assertEqual(hw.applied_motor("motor", "kraken").mode, NEUTRAL)
        self.assertEqual(sim.latest(MotorState, topics.STATE).mode,
                         ControlMode.NEUTRAL)
        send(ControlMode.VOLTAGE, voltage_v=1.0)
        self.assertEqual(hw.applied_motor("motor", "kraken")[:2],
                         (VOLTAGE, 1.0))

    # REQ-CONTRACT-25: clock-mapping jitter putting acquisition time slightly
    # ahead of node.now_ns clamps uint64 ages to 0 instead of wrapping near 2**64.
    def test_future_clock_mapped_acquisition_clamps_age_to_zero(self):
        from bridge.BridgeStatus import BridgeStatus

        sim = Sim(CONFIG)
        sim.add(bench.BenchMotor, "motor")
        hw = SimHardware(sim)
        hw.enable()
        hw.set_driver_station(enabled=True, teleop=True)
        hw.set_motor("motor", "kraken", 1.0, 2.0, 0, 0)
        sim.step(0.05)
        mtype = talos.message_type(BridgeStatus)
        cur = sim.latest(BridgeStatus, "/hw/status")
        fields = {f: getattr(cur, f) for f in mtype.fields}
        fields["clock_offset_us"] += 50_000
        hw.lose_states(2)
        sim.publish(BridgeStatus, "/hw/status", **fields)
        sim.step(0.005)
        state = sim.latest(MotorState, topics.STATE)
        self.assertTrue(state.connected)
        self.assertTrue(state.acquisition_valid)
        self.assertEqual(state.position_age_us, 0)
        self.assertEqual(state.velocity_age_us, 0)


class ModuleLaneReader:
    """The bench config binds the swerve module's lane too, and teleop needs
    a reader of every bound lane; this stands in for the swerve module
    node."""

    def __init__(self, node):
        self.target = talos.Target(node, ModuleTarget)

    def update(self):
        pass


class BenchTeleopTest(unittest.TestCase):
    """subsystems/motor.toml's [teleop] bindings through the framework
    teleop node to motor.py, as node_test.cc's BenchTeleop does in C++."""

    def test_configured_axis_buttons_release_and_disable(self):
        sim = Sim(CONFIG)
        sim.add(bench.BenchMotor, "motor")
        sim.add(ModuleLaneReader, "swerve_module")
        sim.add(teleop.Teleop, "teleop")
        hw = SimHardware(sim)
        hw.enable()

        def push(axis=0.0, buttons=(), enabled=True):
            stick = Stick(axes=(0.0, axis), buttons=buttons, button_count=3)
            hw.set_driver_station(enabled=enabled, teleop=True,
                                  joysticks=[stick])
            sim.step(0.02)
            return hw.motor_request("motor", "kraken")

        push()  # A neutral evaluation arms the producer.
        self.assertEqual(push(axis=-1.0)[:2], (VOLTAGE, MAX_VOLTS))
        self.assertEqual(push(axis=-1.0, buttons=(2,))[:2], (VELOCITY, 30.0))
        self.assertEqual(push(axis=-1.0, buttons=(1, 2)).demand, 10.0)
        # Button 3: TORQUE_CURRENT 2 A (the test config turns torque mode on);
        # button 1 wins over it.
        self.assertEqual(push(axis=-1.0, buttons=(3,))[:2],
                         (TORQUE_CURRENT, 2.0))
        self.assertEqual(push(axis=-1.0, buttons=(1, 3)).mode, VELOCITY)
        self.assertEqual(push().mode, NEUTRAL)
        push(buttons=(1,))
        hw.enable(False)
        self.assertEqual(push(buttons=(1,), enabled=False).mode, NEUTRAL)


def target(mode, generation=None, voltage_v=0.0, velocity_rps=0.0,
           voltage_frac=0.0, torque_current_a=0.0):
    """A teleop MotorTarget stamped with the current generation, or with
    the one remembered as `generation`, then 10 ms."""
    return parity.send(MotorTarget, "teleop", generation=generation,
                       mode=mode, voltage_v=voltage_v,
                       velocity_rps=velocity_rps, voltage_frac=voltage_frac,
                       torque_current_a=torque_current_a)


# The matrix cases node_test.cc proves for the C++ node, and the steps that
# put motor.py through the same inputs:
# matrix: parity 1 node_test.cc "velocity", "teleop, current generation"
# matrix: parity 1 node_test.cc "torque current"
# matrix: parity 2 node_test.cc "gateway disabled"
# matrix: parity 3 node_test.cc "stale lane"
# matrix: parity 4 node_test.cc "released"
# matrix: parity 5 node_test.cc "enabled, never sent"
# matrix: parity 6 node_test.cc "Driver Station lost"
# matrix: parity 7 node_test.cc "disconnected"
# matrix: parity 8 node_test.cc "NaN voltage", "infinite voltage"
# matrix: parity 8 node_test.cc "NaN axis", "infinite axis"
# matrix: parity 8 node_test.cc "NaN velocity", "infinite velocity"
# matrix: parity 8 node_test.cc "NaN torque current"
# matrix: parity 8 node_test.cc "infinite torque current"
# matrix: parity 10 node_test.cc "teleop, auto lane", "autonomous, teleop lane"
# matrix: parity 11 node_test.cc "autonomous, earlier teleop generation"
# matrix: parity 11 node_test.cc "teleop, autonomous generation"
# matrix: parity 11 node_test.cc "teleop, first teleop generation"
# node_test.cc's VoltageIsClampedAndAxisUsesNodeScale: the three voltage
# steps; TorqueCurrentIsClampedToTheNodeCapAndZeroIsSent: the two torque
# current steps past the cap and "zero torque current";
# ReportsInjectedFeedbackAndAcquisitionAges: "feedback" and
# "disconnected"; ReportsLimitSwitchesAndClearsThemOnStaleState: "limit
# switches" and "hardware state lost".
SCENARIO = [
    ("disabled", parity.wait(0.05), parity.require_published()),
    ("enabled, never sent", parity.gateway(True),
     parity.driver_station(enabled=True, teleop=True, then=0.05)),
    ("teleop, auto lane",
     parity.send(MotorTarget, "auto", mode=ControlMode.VELOCITY,
                 voltage_v=0.0, velocity_rps=10.0, voltage_frac=0.0,
                 torque_current_a=0.0, then=0.02),
     parity.release(MotorTarget, "auto")),
    ("voltage above the clamp", target(ControlMode.VOLTAGE, voltage_v=20.0)),
    ("voltage below the clamp", target(ControlMode.VOLTAGE, voltage_v=-20.0)),
    ("voltage and axis",
     target(ControlMode.VOLTAGE, voltage_v=0.5, voltage_frac=-1.5)),
    ("velocity", target(ControlMode.VELOCITY, velocity_rps=10.0)),
    ("torque current",
     target(ControlMode.TORQUE_CURRENT, torque_current_a=1.5)),
    ("torque current above the cap",
     target(ControlMode.TORQUE_CURRENT, torque_current_a=8.0)),
    ("torque current below the cap",
     target(ControlMode.TORQUE_CURRENT, torque_current_a=-8.0)),
    ("zero torque current",
     target(ControlMode.TORQUE_CURRENT, torque_current_a=0.0)),
    ("explicit neutral", target(ControlMode.NEUTRAL, voltage_v=2.0)),
    ("feedback",
     parity.device("set_motor", "kraken", 4.5, -7.0, 2000, 1000, 42.5, 6.25,
                   12.0),
     target(ControlMode.VELOCITY, velocity_rps=-3.0)),
    ("released", parity.release(MotorTarget, "teleop")),
    ("NaN voltage", target(ControlMode.VOLTAGE, voltage_v=math.nan)),
    ("infinite voltage", target(ControlMode.VOLTAGE, voltage_v=math.inf)),
    ("NaN axis", target(ControlMode.VOLTAGE, voltage_frac=math.nan)),
    ("infinite axis", target(ControlMode.VOLTAGE, voltage_frac=math.inf)),
    ("NaN velocity", target(ControlMode.VELOCITY, velocity_rps=math.nan)),
    ("infinite velocity", target(ControlMode.VELOCITY, velocity_rps=math.inf)),
    ("NaN torque current",
     target(ControlMode.TORQUE_CURRENT, torque_current_a=math.nan)),
    ("infinite torque current",
     target(ControlMode.TORQUE_CURRENT, torque_current_a=math.inf)),
    ("stale lane", target(ControlMode.VOLTAGE, voltage_frac=0.25),
     parity.wait(TIMEOUT_S + 0.01)),
    ("limit switches",
     parity.device("set_limit_switches", "kraken", True, False, then=0.01)),
    ("disconnected", parity.device("set_connected", "kraken", False),
     target(ControlMode.VELOCITY, velocity_rps=10.0)),
    ("reconnected", parity.device("set_connected", "kraken", True),
     target(ControlMode.VELOCITY, velocity_rps=10.0)),
    # Teleop -> disabled -> autonomous -> teleop under the bench policy
    # (teleop ["teleop"], autonomous []): no lane drives in autonomous, and
    # a target stamped under an earlier generation never drives.
    ("Driver Station disabled", parity.remember_generation("first teleop"),
     parity.driver_station(teleop=True, then=0.04),
     target(ControlMode.VELOCITY, velocity_rps=10.0)),
    ("autonomous, teleop lane",
     parity.driver_station(enabled=True, autonomous=True, then=0.05),
     target(ControlMode.VELOCITY, velocity_rps=10.0)),
    ("autonomous, earlier teleop generation",
     target(ControlMode.VELOCITY, "first teleop", velocity_rps=10.0)),
    ("teleop, autonomous generation", parity.remember_generation("auto"),
     parity.driver_station(enabled=True, teleop=True, then=0.02),
     target(ControlMode.VELOCITY, "auto", velocity_rps=10.0)),
    ("teleop, first teleop generation",
     target(ControlMode.VELOCITY, "first teleop", velocity_rps=10.0)),
    ("teleop, current generation",
     target(ControlMode.VELOCITY, velocity_rps=10.0)),
    ("gateway disabled", parity.gateway(False),
     target(ControlMode.VELOCITY, velocity_rps=10.0)),
    ("gateway enabled again", parity.gateway(True, then=0.02),
     target(ControlMode.VELOCITY, velocity_rps=10.0)),
    ("Driver Station lost", parity.stop_driver_station(),
     *[target(ControlMode.VELOCITY, velocity_rps=10.0)] * 16),
    ("hardware state lost", parity.lose_states(1000), parity.wait(0.2)),
]


class ParityTest(unittest.TestCase):
    """The C++ node (libbench_motor.so) and motor.py on one scenario."""

    # REQ-CONTRACT-14
    def test_publishes_and_requests_what_the_cpp_node_does(self):
        parity.assert_same_behaviour(
            CppBenchMotor, bench.BenchMotor, CONFIG, "motor", SCENARIO,
            topics=[(MotorState, topics.STATE)], motors=["kraken"])

    # REQ-CONTRACT-14 at the period a Python motor owner runs at on a robot
    # (20 ms minimum, REQ-RT-23; robot_py.toml's motor_py.toml): the same
    # scenario on CONFIG with the motor's period_us 20000. (CONFIG is
    # itself an include, which cannot be included again: this is a copy.)
    # Most steps last 10 ms, half a period, so each gains a 10 ms wait:
    # otherwise every other step's target is overwritten before a tick
    # runs the nodes on it, and that step compares nothing new.
    def test_publishes_and_requests_what_the_cpp_node_does_at_20_ms(self):
        with open(CONFIG, encoding="utf-8") as f:
            text = f.read()
        bench_config = os.path.normpath(os.path.join(
            os.path.dirname(CONFIG), "../../configuration/robot.toml"))
        for old, new in (
                ('include = "../../configuration/robot.toml"',
                 f'include = "{bench_config}"'),
                ("[subsystems.motor]\n",
                 "[subsystems.motor]\nperiod_us = 20000\n")):
            self.assertEqual(text.count(old), 1, old)
            text = text.replace(old, new)
        config = os.path.join(os.environ["TEST_TMPDIR"], "motor_20ms.toml")
        with open(config, "w", encoding="utf-8") as f:
            f.write(text)
        sim = Sim(config)
        sim.add(bench.BenchMotor, "motor")
        self.assertEqual(sim.node("motor").period, 0.02)
        del sim
        scenario = [(name, *actions, parity.wait(0.01))
                    for name, *actions in SCENARIO]
        parity.assert_same_behaviour(
            CppBenchMotor, bench.BenchMotor, config, "motor", scenario,
            topics=[(MotorState, topics.STATE)], motors=["kraken"])


if __name__ == "__main__":
    unittest.main()
