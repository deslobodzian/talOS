"""Robot hardware for talOS nodes, over talOS/devices (C++).

    self.roller = talos.Motor(node, "roller")    # [motors.roller]
    self.beam = talos.DigitalInput(node, "beam")  # [digital_inputs.beam]
    self.ds = talos.DriverStation(node)          # what the bridge forwards

Take handles in __init__. Commands apply to the next flush after update();
motors not commanded by then are neutral. Motors, digital outputs and PWM
outputs are neutral while talos.robot_enabled(node) is false. Motor demands
clamp to configured limits; non-finite input neutralizes the motor. Feedback
is in mechanism units and reads 0 (connected False) until the first state.
"""

import ctypes
import enum
import math
from datetime import timedelta

from talos import _abi
from talos._abi import lib
from talos._messages import message_type


class Mode(enum.IntEnum):
    """A motor request's control mode (talos::hardware::Mode)."""
    NEUTRAL = 0
    DUTY_CYCLE = 1
    VOLTAGE = 2
    VELOCITY = 3
    POSITION = 4
    MOTION_MAGIC = 5


def robot_enabled(node):
    """Whether this node's outputs reach the hardware."""
    return bool(_abi.ok(lib.talos_robot_enabled(node._handle)))


def hardware_sample_time_ns(node):
    """When the gateway sampled the latest accepted hardware state, as an
    int of nanoseconds on the node clock (the one node.now reads in
    seconds), or None: before any state, while it is stale, and until the
    bridge's clock mapping has converged (HardwareLink::sample_time).
    Absolute mapping error is bounded by /hw/status clock_offset_uncertainty_us
    under the documented relative oscillator drift assumption."""
    sampled = ctypes.c_int64()
    found = _abi.ok(lib.talos_hardware_sample_time(node._handle,
                                                   ctypes.byref(sampled)))
    return sampled.value if found else None


class _OnNode:
    """A wrapper over one native node's endpoint or device. It holds the
    talos.Node (`_owner`), which holds the node's owner (a talos.testing.Sim),
    so the native node lives as long as any wrapper that can reach it. Its
    handle reads raise TalosError once the node has been torn down."""

    def __init__(self, node):
        self._owner = node

    @property
    def _node(self):
        return self._owner._handle


class HardwareLink(_OnNode):
    """Accepted hardware state; ages are seconds since publication."""

    def __init__(self, node):
        super().__init__(node)
        # Attach the C++ extension during construction, before the loop starts.
        _abi.ok(lib.talos_hardware_fresh(self._node, -1))

    def fresh(self, max_age=None) -> bool:
        """Whether state is younger than max_age (default command timeout)."""
        if isinstance(max_age, timedelta):
            max_age = max_age.total_seconds()
        ns = -1 if max_age is None else round(max_age * 1e9)
        return bool(_abi.ok(lib.talos_hardware_fresh(self._node, ns)))

    @property
    def age(self) -> float:
        """Seconds since the accepted state was published; inf before any."""
        ns = ctypes.c_int64()
        found = _abi.ok(lib.talos_hardware_age(self._node, ctypes.byref(ns)))
        return ns.value / 1e9 if found else math.inf

    @property
    def sample_time(self):
        """Gateway sample time as integer node-clock nanoseconds, or None."""
        return hardware_sample_time_ns(self._owner)


class _Device(_OnNode):
    _KIND = None

    def __init__(self, node, name):
        super().__init__(node)
        self._index = _abi.index(
            lib.talos_device(self._node, self._KIND, _abi.encode(name)))


class _Input(_Device):
    _SAMPLE = None  # its talos_*_sample entry point

    def _sample(self):
        sample = _abi.Sample()
        _abi.ok(self._SAMPLE(self._node, self._index, sample))
        return sample

    @property
    def connected(self):
        return bool(self._sample().valid)


class Motor(_Input):
    _KIND = _abi.DEVICE_MOTOR
    _SAMPLE = lib.talos_motor_sample
    _TIME = lib.talos_motor_time

    def _time(self, velocity):
        sampled = ctypes.c_int64()
        found = _abi.ok(self._TIME(
            self._node, self._index, velocity, ctypes.byref(sampled)))
        return sampled.value if found else None

    @property
    def position_time(self):
        """Acquisition time, integer node-clock ns, or None if unavailable."""
        return self._time(False)

    @property
    def velocity_time(self):
        """Acquisition time, integer node-clock ns, or None if unavailable."""
        return self._time(True)

    def rezero(self, position_rot=0):
        """Set the node-shared software position on connected finite feedback.

        Position goals use this frame; soft limits remain hardware coordinates.
        Disconnected/non-finite feedback or input leaves the offset unchanged.
        """
        _abi.ok(lib.talos_motor_rezero(self._node, self._index, position_rot))

    @property
    def clamped_count(self):
        """Number of setter calls clamped or neutralized for non-finite input."""
        count = _abi.c_uint64()
        _abi.ok(lib.talos_motor_clamped_count(
            self._node, self._index, ctypes.byref(count)))
        return count.value

    def set_duty_cycle(self, fraction):  # [-1, 1]
        self._set(Mode.DUTY_CYCLE, fraction)

    def set_voltage(self, volts):
        self._set(Mode.VOLTAGE, volts)

    def set_velocity(self, rps, ff_volts=0.0, slot=0):  # onboard PID
        self._set(Mode.VELOCITY, rps, ff_volts, slot)

    def set_position(self, rot, ff_volts=0.0, slot=0):  # onboard PID
        self._set(Mode.POSITION, rot, ff_volts, slot)

    def set_motion_magic(self, rot, ff_volts=0.0, slot=0):  # onboard profile
        self._set(Mode.MOTION_MAGIC, rot, ff_volts, slot)

    def stop(self):
        self._set(Mode.NEUTRAL, 0.0)

    @property
    def position(self):  # rotations
        return self._sample().position

    @property
    def velocity(self):  # rotations/s
        return self._sample().velocity

    @property
    def voltage(self):  # applied volts
        return self._sample().voltage

    @property
    def current(self):  # stator amps
        return self._sample().current

    def _set(self, mode, demand, ff_volts=0.0, slot=0):
        _abi.ok(lib.talos_motor_set(self._node, self._index, mode, demand,
                                    ff_volts, slot))


class CANcoder(_Input):
    _KIND = _abi.DEVICE_CANCODER
    _SAMPLE = lib.talos_sensor_sample
    _TIME = lib.talos_sensor_time
    _time = Motor._time
    position_time = Motor.position_time
    velocity_time = Motor.velocity_time

    @property
    def position(self):  # rotations
        return self._sample().position

    @property
    def velocity(self):  # rotations/s
        return self._sample().velocity


class Gyro(_Input):
    """A Pigeon2. Rotations and rotations/s, CCW+."""
    _KIND = _abi.DEVICE_GYRO
    _SAMPLE = lib.talos_sensor_sample
    _TIME = lib.talos_sensor_time
    _time = Motor._time
    yaw_time = Motor.position_time
    yaw_rate_time = Motor.velocity_time

    def rezero(self, yaw_rot=0):
        """Set node-shared software yaw on connected finite feedback/input."""
        _abi.ok(lib.talos_gyro_rezero(self._node, self._index, yaw_rot))

    @property
    def yaw(self):
        return self._sample().position

    @property
    def yaw_rate(self):
        return self._sample().velocity


class DigitalInput(_Input):
    _KIND = _abi.DEVICE_DIGITAL_INPUT
    _SAMPLE = lib.talos_digital_input_sample

    @property
    def value(self):
        return bool(self._sample().value)


class AnalogInput(_Input):
    _KIND = _abi.DEVICE_ANALOG_INPUT
    _SAMPLE = lib.talos_analog_input_sample

    @property
    def voltage(self):
        return self._sample().voltage


class Encoder(_Input):
    _KIND = _abi.DEVICE_ENCODER
    _SAMPLE = lib.talos_encoder_sample

    @property
    def position(self):  # rotations
        return self._sample().position

    @property
    def velocity(self):  # rotations/s
        return self._sample().velocity


class DigitalOutput(_Device):
    _KIND = _abi.DEVICE_DIGITAL_OUTPUT

    def set(self, value):
        _abi.ok(lib.talos_digital_output_set(self._node, self._index,
                                             int(bool(value))))


class PwmOutput(_Device):
    _KIND = _abi.DEVICE_PWM

    def set(self, output):
        _abi.ok(lib.talos_pwm_set(self._node, self._index, output))


class Alliance(enum.IntEnum):
    """talos::driver_station::Alliance."""
    UNKNOWN = 0
    RED = 1
    BLUE = 2


class Joystick:
    """One Driver Station joystick as a DriverStation saw it; a snapshot."""

    def __init__(self, data):
        self._data = data  # an _abi.Joystick

    @property
    def connected(self):
        return bool(self._data.connected)

    @property
    def axis_count(self):
        return self._data.axis_count

    @property
    def button_count(self):
        return self._data.button_count

    @property
    def pov_count(self):
        return self._data.pov_count

    @property
    def name(self):
        return self._data.name.decode(errors="replace")

    def axis(self, index):
        """[-1, 1]; 0 past axis_count."""
        return self._data.axes[index] if 0 <= index < _abi.DS_MAX_AXES else 0.0

    def button(self, number):
        """1-based, as the Driver Station numbers them."""
        if not 1 <= number <= 32:
            return False
        return bool(self._data.buttons >> (number - 1) & 1)

    def pov(self, index=0):
        """Degrees clockwise from up; -1 centred or absent."""
        return self._data.povs[index] if 0 <= index < _abi.DS_MAX_POVS else -1


class DriverStation(_OnNode):
    """The Driver Station packet the bridge forwards, decoded and latched.

        self.ds = talos.DriverStation(node)
        ...
        if self.ds.teleop and self.ds.joystick(0).button(1): ...

    Mode properties are False unless connected (a packet younger than
    [robot] command_timeout_us says the DS is attached), and joystick() is
    disconnected, so a silent Driver Station reads as disabled.
    on_packet() runs as each packet arrives.
    """

    def __init__(self, node, on_packet=None):
        super().__init__(node)
        callback = _abi.CALLBACK()
        if on_packet is not None:
            callback = node._keep(_abi.CALLBACK(
                _abi.callback(lambda _: on_packet())))
        _abi.ok(lib.talos_driver_station(self._node, callback, None))

    def _read(self):
        out = _abi.DriverStation()
        _abi.ok(lib.talos_driver_station_read(self._node, out))
        return out

    connected = property(lambda self: bool(self._read().connected))
    enabled = property(lambda self: bool(self._read().enabled))
    autonomous = property(lambda self: bool(self._read().autonomous))
    teleop = property(lambda self: bool(self._read().teleop))
    test = property(lambda self: bool(self._read().test))
    estop = property(lambda self: bool(self._read().estop))
    fms_attached = property(lambda self: bool(self._read().fms_attached))
    alliance = property(lambda self: Alliance(self._read().alliance))
    station = property(lambda self: self._read().station)  # 1-3; 0 unknown
    match_time = property(lambda self: self._read().match_time_s)  # seconds

    def joystick(self, port):
        if not 0 <= port < _abi.DS_MAX_JOYSTICKS:
            raise IndexError(f"no Driver Station joystick port {port} "
                             f"(ports are 0-{_abi.DS_MAX_JOYSTICKS - 1})")
        return Joystick(self._read().joysticks[port])


class Target(_OnNode):
    """The lanes of this node's target policy, resolved (talos::Target in
    talOS/devices/target.h). Take it in __init__:

        self.target = talos.Target(node, PositionTarget)
        ...
        t = self.target.get()   # None: disabled, stale or no live lane
        if t is not None:
            self.motor.set_motion_magic(t.position_rot)

    get() returns the message of the first lane, in the current Driver
    Station mode's priority order ([robot.targets], the subsystem's
    [targets]), that is active, fresh and stamped with the current mode
    generation, while robot_enabled(node); otherwise None. `source` names
    that lane, or "".
    """

    def __init__(self, node, msg):
        super().__init__(node)
        self._type = message_type(msg)
        self._index = _abi.index(lib.talos_target_open(
            self._node, self._type.size, self._type.alignment,
            _abi.schema_of(self._type)))
        self._buffer = ctypes.create_string_buffer(self._type.size)
        self._source = ctypes.c_int32(-1)

    def get(self):
        if not _abi.ok(lib.talos_target_get(self._node, self._index,
                                            self._buffer, self._source)):
            return None
        return self._type.decode(self._buffer.raw)

    @property
    def source(self):
        if not _abi.ok(lib.talos_target_get(self._node, self._index, None,
                                            self._source)):
            return ""
        return _abi.check(lib.talos_target_source_name(
            self._node, self._index, self._source.value),
            lambda name: name is None).decode()


class TargetLane(_OnNode):
    """One producer's lane, /<subsystem>/target/<source> (talos::TargetLane):

        self.lane = talos.TargetLane(node, PositionTarget, "wrist", "auto")
        self.lane.send(position_rot=0.25)   # republish every period
        self.lane.release()                 # nothing to say

    send() stamps the mode generation this node's Driver Station shows (0,
    which no consumer accepts, while it shows no mode), so a target from an
    earlier enable or mode never drives this one.
    """

    def __init__(self, node, msg, subsystem, source):
        super().__init__(node)
        self._type = message_type(msg)
        self._index = _abi.index(lib.talos_target_lane_open(
            self._node, _abi.encode(subsystem), _abi.encode(source),
            self._type.size, self._type.alignment, _abi.schema_of(self._type)))

    def send(self, message=None, **fields):
        """send(field=value, ...) with the .fbs field names, or send(message)
        to forward a received one."""
        data = self._type.forward(message) if message is not None else (
            self._type.encode(fields))
        return bool(_abi.ok(lib.talos_target_lane_send(
            self._node, self._index, data, len(data))))

    def release(self):
        return bool(_abi.ok(lib.talos_target_lane_release(
            self._node, self._index)))
