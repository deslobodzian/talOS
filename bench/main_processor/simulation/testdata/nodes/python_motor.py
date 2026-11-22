"""SoakPythonMotor: test-only 20 ms Python actuator owner for the bench soak.

Runs alongside robot.toml's two 5 ms C++ actuator owners (swerve_module and
motor) in testdata/robot_soak.toml so the soak exercises SIGKILL/SIGSTOP
restart, request-gap bounds, and mode-generation handoffs on a Python actuator
owner as well as on both 5 ms C++ owners.
"""

import math

import talos
from motor.ControlMode import ControlMode
from motor.MotorTarget import MotorTarget


def _clamp(value, low, high):
    return low if value < low else high if high < value else value


class SoakPythonMotor:
    def __init__(self, node):
        self.node = node
        self.motor = talos.Motor(node, "python_kraken")
        self.target = talos.Target(node, MotorTarget)
        self.max_voltage_v = node.param("max_bench_voltage_v", 2.0)
        if not math.isfinite(self.max_voltage_v) or self.max_voltage_v <= 0:
            raise ValueError("max_bench_voltage_v must be finite and > 0")

    def update(self):
        enabled = talos.robot_enabled(self.node)
        target = self.target.get()
        if enabled and target is not None and self.motor.connected:
            if (target.mode == ControlMode.VOLTAGE
                    and math.isfinite(target.voltage_v)
                    and math.isfinite(target.voltage_frac)):
                volts = (target.voltage_v
                         + _clamp(target.voltage_frac, -1.0, 1.0)
                         * self.max_voltage_v)
                self.motor.set_voltage(
                    _clamp(volts, -self.max_voltage_v, self.max_voltage_v))
            elif (target.mode == ControlMode.VELOCITY
                  and math.isfinite(target.velocity_rps)):
                self.motor.set_velocity(target.velocity_rps)
