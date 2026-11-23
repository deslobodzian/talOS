"""BenchGyro: the hardware bench's Pigeon 2 (subsystems/gyro.toml), read
through the Python devices layer (ctypes over //talOS/node_api), so the
Python Gyro path runs against real hardware.

An observer: it owns no actuator and reads no target. Every tick, enabled or
not, it publishes /gyro/state from the latest sample, converting the
device's rotations and rotations/s to radians and radians/s. It reports the
Pigeon's own yaw frame and never calls rezero. While the sample is invalid
the device reads the last valid yaw and zero rate (REQ-CONTRACT-34), which
the node reports as is, with connected false.
"""

import math

import talos
from gyro import topics
from gyro.GyroState import GyroState

# An unknown age, std::numeric_limits<std::uint64_t>::max().
_UNKNOWN = 2**64 - 1
_TAU = 2.0 * math.pi


def _micros(ns):
    """Non-negative microsecond count from an int64 nanosecond duration:
    clamps negative clock-mapping jitter to 0 instead of wrapping uint64."""
    return 0 if ns <= 0 else ns // 1000


class BenchGyro:
    def __init__(self, node):
        self.node = node
        self.gyro = talos.Gyro(node, "pigeon")
        self.link = talos.HardwareLink(node)
        self.state = node.publish(GyroState, topics.STATE)

    def update(self):
        now_ns = self.node.now_ns

        def age(time):
            return _UNKNOWN if time is None else _micros(now_ns - time)

        self.state.send(
            sample_time_ns=now_ns,
            yaw_rad=self.gyro.yaw * _TAU,
            yaw_rate_radps=self.gyro.yaw_rate * _TAU,
            yaw_age_us=age(self.gyro.yaw_time),
            yaw_rate_age_us=age(self.gyro.yaw_rate_time),
            connected=self.gyro.connected,
            hardware_fresh=self.link.fresh())
