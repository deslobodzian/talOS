"""talos.testing: nodes on a simulated clock, for unit and multi-node tests.

    sim = talos.testing.Sim.from_toml(ROBOT_TOML)
    intake = sim.add(Intake, "intake")
    hw = talos.testing.SimHardware(sim)
    hw.enable()
    hw.set_driver_station(enabled=True, teleop=True)
    sim.step(0.01)
    sim.publish_target(IntakeTarget, "intake", "teleop",
                       roller_velocity_rps=30.0)
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

# One joystick for SimHardware.set_driver_station: axes in [-1, 1], the
# pressed buttons by their 1-based numbers (button_count defaults to the
# highest), and POV angles (-1 centred).
Stick = collections.namedtuple("Stick", "axes buttons povs name button_count",
                               defaults=((), (), (), "", None))


def _generation(mode_since_us):
    """The C ABI's optional explicit generation: NULL derives it."""
    if mode_since_us is None:
        return None
    return ctypes.byref(ctypes.c_uint64(mode_since_us))


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

    def set_session_dir(self, path):
        """The session output directory (str) the nodes report via
        Node.session_dir; None clears it. Call before add()."""
        _abi.ok(lib.talos_sim_set_session_dir(
            self._handle, None if path is None else _abi.encode(path)))

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
        mtype = message_type(msg)
        data = mtype.forward(message) if message is not None else (
            mtype.encode(fields))
        _abi.ok(lib.talos_sim_publish_raw(self._handle, _abi.encode(topic),
                                          data, len(data)))

    def latest(self, msg, topic):
        """The newest `msg` on `topic`, or None."""
        mtype = message_type(msg)
        buffer = ctypes.create_string_buffer(mtype.size)
        found = _abi.ok(lib.talos_sim_latest_raw(
            self._handle, _abi.encode(topic), buffer, mtype.size))
        return mtype.decode(buffer.raw) if found else None

    @property
    def driver_station_generation(self):
        """The Driver Station mode generation a producer that has drained
        the newest /hw/state/driver_station packet stamps on its lanes: the
        packet's mode_since_us while it names a mode, else 0."""
        return lib.talos_sim_driver_station_generation(self._handle)

    def publish_target(self, msg, subsystem, source, message=None, *,
                       mode_since_us=None, **fields):
        """Publishes an active target on /<subsystem>/target/<source>, as a
        talos.TargetLane would: stamped with driver_station_generation, or
        with `mode_since_us` for a producer that has not seen the current
        mode (a target queued from an earlier one, a stale view: 0)."""
        mtype = message_type(msg)
        data = mtype.forward(message) if message is not None else (
            mtype.encode(fields))
        _abi.ok(lib.talos_sim_publish_target(
            self._handle, _abi.encode(subsystem), _abi.encode(source), data,
            mtype.size, mtype.alignment, _generation(mode_since_us)))

    def release_target(self, msg, subsystem, source):
        """Publishes a release (nothing to say) on that lane."""
        mtype = message_type(msg)
        _abi.ok(lib.talos_sim_publish_target(
            self._handle, _abi.encode(subsystem), _abi.encode(source), None,
            mtype.size, mtype.alignment, None))

    def latest_target(self, msg, subsystem, source):
        """The newest `msg` on /<subsystem>/target/<source>: the message
        when active, None when released or never sent."""
        mtype = message_type(msg)
        buffer = ctypes.create_string_buffer(mtype.size)
        active = ctypes.c_int32()
        found = _abi.ok(lib.talos_sim_latest_target(
            self._handle, _abi.encode(subsystem), _abi.encode(source), buffer,
            mtype.size, mtype.alignment, active))
        return mtype.decode(buffer.raw) if found and active.value else None

    def latest_bytes(self, topic):
        """The newest byte message on `topic`, or None."""
        topic_bytes = _abi.encode(topic)
        size = ctypes.c_uint32()
        found = _abi.ok(lib.talos_sim_latest_bytes(
            self._handle, topic_bytes, None, 0, size))
        if not found:
            return None
        buffer = ctypes.create_string_buffer(size.value)
        _abi.ok(lib.talos_sim_latest_bytes(
            self._handle, topic_bytes, buffer, size.value, size))
        return buffer.raw


class SimHardware:
    """Plays the bridge for a Sim: publishes /hw/state every robot period
    (configured; enabled after enable()) and records each node's requests.
    It also publishes /hw/status with the bridge's own gateway clock filter,
    so talos.hardware_sample_time_ns(node) is None until the 8th state, then
    each state's sim sample time plus the smallest latency the filter has
    seen (its publication time while the latency is constant)."""

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

    def set_gateway_clock(self, offset=0.0, latency=0.0):
        """From now on the gateway clock reads sim time + `offset` seconds,
        and each state is sampled `latency` seconds before it goes out."""
        _abi.ok(lib.talos_sim_hardware_set_gateway_clock(
            self._handle, round(offset * _NS), round(latency * _NS)))

    def set_connected(self, node, device, connected):
        """A disconnected device reports connected False (a motor's sample
        is invalid) and reads 0; every device of `node` named `device`."""
        self._set(lib.talos_sim_hardware_set_connected, node, device,
                  bool(connected))

    def set_driver_station(self, *, enabled=False, autonomous=False,
                           teleop=False, test=False, estop=False,
                           attached=True, fms_attached=False,
                           alliance=talos.Alliance.UNKNOWN, station=0,
                           match_time=0.0, joysticks=()):
        """The Driver Station packet to forward every period from now on.

            hw.set_driver_station(enabled=True, teleop=True,
                                  joysticks=[Stick(axes=(0.0, -1.0),
                                                   buttons=(1,))])
        """
        if len(joysticks) > _abi.DS_MAX_JOYSTICKS:
            raise ValueError(f"at most {_abi.DS_MAX_JOYSTICKS} joysticks")
        flags = 0
        for on, flag in ((attached, _abi.DS_ATTACHED),
                         (fms_attached, _abi.DS_FMS_ATTACHED),
                         (enabled, _abi.DS_ENABLED),
                         (autonomous, _abi.DS_AUTONOMOUS),
                         (teleop, _abi.DS_TELEOP), (test, _abi.DS_TEST),
                         (estop, _abi.DS_ESTOP)):
            flags |= flag if on else 0
        packet = _abi.DriverStationPacket(flags=flags, alliance=int(alliance),
                                          station=station,
                                          match_time_s=match_time)
        for port, stick in enumerate(joysticks):
            packet.joysticks[port] = _joystick(stick)
        _abi.ok(lib.talos_sim_hardware_set_driver_station(self._handle,
                                                          packet))

    def stop_driver_station(self):
        """Stops forwarding: the Driver Station goes silent, as when its link
        drops, while /hw/state keeps coming."""
        _abi.ok(lib.talos_sim_hardware_set_driver_station(self._handle, None))

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


def _joystick(stick):
    axes, buttons, povs = tuple(stick.axes), tuple(stick.buttons), tuple(
        stick.povs)
    if (len(axes) > _abi.DS_MAX_AXES or len(povs) > _abi.DS_MAX_POVS
            or not all(1 <= b <= 32 for b in buttons)):
        raise ValueError(f"{stick}: at most {_abi.DS_MAX_AXES} axes and "
                         f"{_abi.DS_MAX_POVS} POVs, buttons 1-32")
    out = _abi.Joystick(
        connected=1, axis_count=len(axes), pov_count=len(povs),
        button_count=(stick.button_count if stick.button_count is not None
                      else max(buttons, default=0)),
        buttons=sum(1 << (b - 1) for b in set(buttons)),
        name=stick.name.encode()[:_abi.DS_NAME_LENGTH])
    out.axes[:len(axes)] = axes
    out.povs[:len(povs)] = povs
    return out
