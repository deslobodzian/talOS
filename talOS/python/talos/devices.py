"""Robot hardware for talOS nodes, over talOS/devices (C++).

    self.roller = talos.Motor(node, "roller")    # [motors.roller]
    self.beam = talos.DigitalInput(node, "beam")  # [digital_inputs.beam]
    self.ds = talos.DriverStation(node)          # what the bridge forwards

Take handles in __init__. Commands apply to the next flush after update();
a motor not commanded by then is neutral, and every motor is neutral while
talos.robot_enabled(node) is false. Feedback is in mechanism units and reads
0 (connected False) until the first hardware state.
"""

import enum

from talos import _abi
from talos._abi import lib


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


class _Device:
    _KIND = None

    def __init__(self, node, name):
        self._node = node._handle
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


class DriverStation:
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
        self._node = node._handle
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
