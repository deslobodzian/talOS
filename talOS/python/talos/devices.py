"""Robot hardware for talOS nodes, over talOS/devices (C++).

    self.roller = talos.Motor(node, "roller")    # [motors.roller]
    self.beam = talos.DigitalInput(node, "beam")  # [digital_inputs.beam]

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
