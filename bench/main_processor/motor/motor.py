"""BenchMotor: the one TalonFX of the hardware bench (docs/BENCH.md), in
Python. The twin of motor.h, statement for statement, so the bench drives and
reads the same motor through the Python path (ctypes over
//talOS/node_api, the Python devices layer) with the same subsystem file:
configuration/robot_py.toml launches it in place of the C++ node.

The motor drives only while robot_enabled, with a live teleop lane
(talos.Target) and connected feedback: VOLTAGE adds voltage_v to
voltage_frac (clamped to [-1, 1]) times max_bench_voltage_v and clamps the sum
to +-max_bench_voltage_v; VELOCITY passes velocity_rps to the onboard PID;
TORQUE_CURRENT sends torque_current_a clamped to +-max_bench_torque_current_a
as stator amperes, and nothing while that parameter is 0 (the default). A
non-finite field, a NEUTRAL target or anything else leaves it unset, which the
flush sends neutral. /motor/state is published every tick, byte for byte what
the C++ node publishes (node_py_test.py's ParityTest).

Unlike motor.h, construction does not check that a positive
max_bench_torque_current_a has [motors.kraken] foc = true and
max_torque_current_a > 0: the Python devices layer exposes no motor config.
With either missing, the first TORQUE_CURRENT target makes
Motor.set_torque_current raise TalosError naming the motor and the key, in
update().
"""

import math

import talos
from bridge.BridgeStatus import BridgeStatus
from motor import topics
from motor.ControlMode import ControlMode
from motor.MotorState import MotorState
from motor.MotorTarget import MotorTarget
from motor.TargetSource import TargetSource

# An unknown or stale age, std::numeric_limits<std::uint64_t>::max().
_UNKNOWN = 2**64 - 1
_NS = 1e9


def _clamp(value, low, high):
    """std::clamp."""
    return low if value < low else high if high < value else value


def _micros(ns):
    """Non-negative microsecond count from an int64 nanosecond duration:
    clamps negative clock-mapping jitter to 0 instead of wrapping uint64."""
    return 0 if ns <= 0 else ns // 1000


class BenchMotor:
    def __init__(self, node):
        self.node = node
        self.motor = talos.Motor(node, "kraken")
        self.link = talos.HardwareLink(node)
        self.target = talos.Target(node, MotorTarget)
        self.state = node.publish(MotorState, topics.STATE)
        self.status = node.subscribe(BridgeStatus, "/hw/status")
        self.max_voltage_v = node.param("max_bench_voltage_v", 2.0)
        if not math.isfinite(self.max_voltage_v) or self.max_voltage_v <= 0:
            raise ValueError("max_bench_voltage_v must be finite and > 0")
        self.max_torque_current_a = node.param(
            "max_bench_torque_current_a", 0.0)
        if (not math.isfinite(self.max_torque_current_a)
                or self.max_torque_current_a < 0):
            raise ValueError(
                "max_bench_torque_current_a must be finite and >= 0")

    def update(self):
        enabled = talos.robot_enabled(self.node)
        target = self.target.get()
        mode = ControlMode.NEUTRAL
        if enabled and target is not None and self.motor.connected:
            if (target.mode == ControlMode.VOLTAGE
                    and math.isfinite(target.voltage_v)
                    and math.isfinite(target.voltage_frac)):
                volts = (target.voltage_v
                         + _clamp(target.voltage_frac, -1.0, 1.0)
                         * self.max_voltage_v)
                self.motor.set_voltage(
                    _clamp(volts, -self.max_voltage_v, self.max_voltage_v))
                mode = ControlMode.VOLTAGE
            elif (target.mode == ControlMode.VELOCITY
                  and math.isfinite(target.velocity_rps)):
                self.motor.set_velocity(target.velocity_rps)
                mode = ControlMode.VELOCITY
            elif (target.mode == ControlMode.TORQUE_CURRENT
                  and self.max_torque_current_a > 0
                  and math.isfinite(target.torque_current_a)):
                self.motor.set_torque_current(
                    _clamp(target.torque_current_a,
                           -self.max_torque_current_a,
                           self.max_torque_current_a))
                mode = ControlMode.TORQUE_CURRENT
        position_time = self.motor.position_time
        velocity_time = self.motor.velocity_time
        now_ns = self.node.now_ns

        def age(time):
            return _UNKNOWN if time is None else _micros(now_ns - time)

        fault_valid = self.status.fresh() and self.status.value.health_valid
        fault_bit = 1 << self.motor.hardware_index
        self.state.send(
            sample_time_ns=now_ns,
            position_rot=self.motor.position,
            velocity_rps=self.motor.velocity,
            applied_voltage_v=self.motor.voltage,
            stator_current_a=self.motor.current,
            hardware_age_us=(_micros(round(self.link.age * _NS))
                             if self.link.fresh() else _UNKNOWN),
            position_age_us=age(position_time),
            velocity_age_us=age(velocity_time),
            acquisition_valid=(position_time is not None
                               and velocity_time is not None),
            connected=self.motor.connected,
            fault=(fault_valid
                   and (self.status.value.motor_fault_mask & fault_bit) != 0),
            fault_valid=fault_valid,
            enabled=enabled,
            mode=mode,
            source=(TargetSource.TELEOP
                    if mode != ControlMode.NEUTRAL
                    and self.target.source == "teleop"
                    else TargetSource.NONE),
            forward_limit=self.motor.forward_limit,
            reverse_limit=self.motor.reverse_limit,
            temperature_degc=self.motor.temperature_c,
            supply_current_a=self.motor.supply_current_a)
