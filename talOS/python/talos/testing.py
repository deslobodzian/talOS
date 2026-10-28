"""talos.testing: nodes on a simulated clock, for unit and multi-node tests.

    sim = talos.testing.Sim.from_toml(ROBOT_TOML)
    intake = sim.add(Intake, "intake")
    hw = talos.testing.SimHardware(sim)
    hw.enable()
    sim.publish(IntakeTarget, "/intake/target", roller_velocity_rps=30.0)
    sim.step(0.1)
    assert hw.motor_request("intake", "roller").mode == talos.Mode.VELOCITY

The same C++ talos::testing::Sim and SimHardware the C++ tests use
(talOS/node/testing.h, talOS/devices/testing.h). Durations are seconds.
"""

import collections
import ctypes

import talos
from talos import _abi
from talos._abi import lib
from talos._messages import message_type

if not hasattr(lib, "talos_sim_create"):
    raise _abi.TalosError("talos.testing needs libtalos_node_testing.so: "
                          "depend on //talOS/python:testing")

_NS = 1e9

MotorRequest = collections.namedtuple("MotorRequest",
                                      "mode demand ff_volts slot")


class Sim:
    """One simulated clock hosting several nodes; topics are in memory."""

    def __init__(self, config_path=None, *, _handle=None):
        self._handle = _handle or _abi.handle(
            lib.talos_sim_create(_abi.encode(config_path)))
        self._nodes = []  # (Node, object) per node, alive with the sim

    @classmethod
    def from_toml(cls, robot_toml):
        """A Sim over an inline single-file robot.toml."""
        return cls(_handle=_abi.handle(
            lib.talos_sim_from_toml(_abi.encode(robot_toml))))

    def __del__(self):
        if getattr(self, "_handle", None):
            lib.talos_sim_destroy(self._handle)
            self._handle = None

    def add(self, cls, node_name):
        """Builds cls on a node named node_name; returns the object."""
        factory = talos._factory(cls, self._nodes)
        _abi.handle(lib.talos_sim_add(self._handle, _abi.encode(node_name),
                                      factory, None))
        return self._nodes[-1][1]

    def node(self, node_name):
        return talos.Node(_abi.handle(
            lib.talos_sim_node(self._handle, _abi.encode(node_name))))

    def step(self, seconds):
        """Advances virtual time, running every node's timers and messages."""
        _abi.ok(lib.talos_sim_step(self._handle, round(seconds * _NS)))

    @property
    def now(self):
        return lib.talos_sim_now_ns(self._handle) / _NS

    def publish(self, msg, topic, message=None, **fields):
        """Publishes on `topic`, like Publisher.send."""
        data = message._bytes if message is not None else (
            message_type(msg).encode(fields))
        _abi.ok(lib.talos_sim_publish_raw(self._handle, _abi.encode(topic),
                                          data, len(data)))

    def latest(self, msg, topic):
        """The newest `msg` on `topic`, or None."""
        mtype = message_type(msg)
        buffer = ctypes.create_string_buffer(mtype.size)
        found = _abi.ok(lib.talos_sim_latest_raw(
            self._handle, _abi.encode(topic), buffer, mtype.size))
        return mtype.decode(buffer.raw) if found else None


class SimHardware:
    """Plays the bridge for a Sim: publishes /hw/state every robot period
    (configured; enabled after enable()) and records each node's requests."""

    def __init__(self, sim):
        self._sim = sim  # outlives this
        self._handle = _abi.handle(lib.talos_sim_hardware_create(sim._handle))

    def __del__(self):
        if getattr(self, "_handle", None):
            lib.talos_sim_hardware_destroy(self._handle)
            self._handle = None

    def enable(self, on=True):
        _abi.ok(lib.talos_sim_hardware_enable(self._handle, bool(on)))

    def ideal_motors(self, on=True):
        """Velocity/position/motion-magic requests are reached each period."""
        _abi.ok(lib.talos_sim_hardware_ideal_motors(self._handle, bool(on)))

    def set_motor(self, node, device, position, velocity):
        self._set(lib.talos_sim_hardware_set_motor, node, device, position,
                  velocity)

    def set_digital_input(self, node, device, value):
        self._set(lib.talos_sim_hardware_set_digital_input, node, device,
                  bool(value))

    def set_analog_input(self, node, device, volts):
        self._set(lib.talos_sim_hardware_set_analog_input, node, device, volts)

    def set_encoder(self, node, device, position, velocity):
        self._set(lib.talos_sim_hardware_set_encoder, node, device, position,
                  velocity)

    def set_cancoder(self, node, device, position, velocity):
        self._set(lib.talos_sim_hardware_set_cancoder, node, device, position,
                  velocity)

    def set_gyro(self, node, device, yaw, yaw_rate):
        self._set(lib.talos_sim_hardware_set_gyro, node, device, yaw, yaw_rate)

    def motor_request(self, node, device):
        """The last request `node` flushed for `device` (neutral before any):
        (mode, demand, ff_volts, slot)."""
        out = _abi.MotorRequest()
        _abi.ok(lib.talos_sim_hardware_motor_request(
            self._handle, _abi.encode(node), _abi.encode(device), out))
        return MotorRequest(talos.Mode(out.mode), out.demand, out.ff_volts,
                            out.slot)

    def digital_output(self, node, device):
        return bool(_abi.ok(lib.talos_sim_hardware_digital_output(
            self._handle, _abi.encode(node), _abi.encode(device))))

    def pwm(self, node, device):
        out = ctypes.c_double()
        _abi.ok(lib.talos_sim_hardware_pwm(self._handle, _abi.encode(node),
                                           _abi.encode(device), out))
        return out.value

    def _set(self, function, node, device, *values):
        _abi.ok(function(self._handle, _abi.encode(node), _abi.encode(device),
                         *values))
