"""motor.py, the Python bench motor node, on talos.testing.Sim, configured by
testdata/robot.toml: configuration/robot.toml (whose subsystems/motor.toml
robot_py.toml also launches) plus a neighbor motor that stays connected.

node_test.cc carries the node test matrix (docs/TESTING.md §7) for the C++
node. ParityTest runs that C++ node (motor.h, hosted through
libbench_motor.so) and motor.py on SCENARIO through talos.parity, which
requires the same /motor/state bytes and the same requested and applied
motor output after every step; the `matrix: parity` comments name the steps
that cover each case. What a parity run cannot compare is tested here:
construction (case 9) and the framework teleop node driving motor.py.
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


class BenchMotorTest(unittest.TestCase):
    # matrix: n/a 12 (a Python tick allocates in the interpreter)

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


def target(mode, generation=None, voltage_v=0.0, velocity_rps=0.0,
           voltage_frac=0.0):
    """A teleop MotorTarget stamped with the current generation, or with
    the one remembered as `generation`, then 10 ms."""
    return parity.send(MotorTarget, "teleop", generation=generation,
                       mode=mode, voltage_v=voltage_v,
                       velocity_rps=velocity_rps, voltage_frac=voltage_frac)


# The matrix cases node_test.cc proves for the C++ node, and the steps that
# put motor.py through the same inputs:
# matrix: parity 1 node_test.cc "velocity", "teleop, current generation"
# matrix: parity 2 node_test.cc "gateway disabled"
# matrix: parity 3 node_test.cc "stale lane"
# matrix: parity 4 node_test.cc "released"
# matrix: parity 5 node_test.cc "enabled, never sent"
# matrix: parity 6 node_test.cc "Driver Station lost"
# matrix: parity 7 node_test.cc "disconnected"
# matrix: parity 8 node_test.cc "non-finite voltage", "non-finite axis"
# matrix: parity 8 node_test.cc "infinite velocity"
# matrix: parity 10 node_test.cc "autonomous, teleop lane"
# matrix: parity 11 node_test.cc "autonomous, earlier teleop generation"
# matrix: parity 11 node_test.cc "teleop, autonomous generation"
# matrix: parity 11 node_test.cc "teleop, first teleop generation"
# node_test.cc's VoltageIsClampedAndAxisUsesNodeScale: the three voltage
# steps; ReportsInjectedFeedbackAndAcquisitionAges: "feedback" and
# "disconnected"; ReportsLimitSwitchesAndClearsThemOnStaleState: "limit
# switches" and "hardware state lost".
SCENARIO = [
    ("disabled", parity.wait(0.05), parity.require_published()),
    ("enabled, never sent", parity.gateway(True),
     parity.driver_station(enabled=True, teleop=True, then=0.05)),
    ("voltage above the clamp", target(ControlMode.VOLTAGE, voltage_v=20.0)),
    ("voltage below the clamp", target(ControlMode.VOLTAGE, voltage_v=-20.0)),
    ("voltage and axis",
     target(ControlMode.VOLTAGE, voltage_v=0.5, voltage_frac=-1.5)),
    ("velocity", target(ControlMode.VELOCITY, velocity_rps=10.0)),
    ("explicit neutral", target(ControlMode.NEUTRAL, voltage_v=2.0)),
    ("feedback",
     parity.device("set_motor", "kraken", 4.5, -7.0, 2000, 1000, 42.5, 6.25,
                   12.0),
     target(ControlMode.VELOCITY, velocity_rps=-3.0)),
    ("released", parity.release(MotorTarget, "teleop")),
    ("non-finite voltage", target(ControlMode.VOLTAGE, voltage_v=math.nan)),
    ("non-finite axis", target(ControlMode.VOLTAGE, voltage_frac=math.nan)),
    ("infinite velocity", target(ControlMode.VELOCITY, velocity_rps=math.inf)),
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


if __name__ == "__main__":
    unittest.main()
