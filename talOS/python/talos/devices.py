"""Robot hardware for talOS nodes, over talOS/devices (C++).

    self.roller = talos.Motor(node, "roller")    # [motors.roller]
    self.beam = talos.DigitalInput(node, "beam")  # [digital_inputs.beam]
    self.ds = talos.DriverStation(node)          # what the bridge forwards

Take handles in __init__. Output commands (Motor.set_* and stop(),
DigitalOutput.set, PwmOutput.set and stop()) are taken only inside update()
and apply to the flush right after it; outputs that update() did not set are
neutral (a PWM output's neutral is off: no pulse; set(0.0) is the centre
pulse). From __init__, an every() timer or a callback they raise TalosError
naming the device, since such a command would reach the hardware for at most
one tick. Motors, digital outputs and PWM outputs are neutral while
talos.robot_enabled(node) is false, and in any tick in which no talos.Target
of the node returned a target from a get() (or source) made during that
update(): drive only from what get() returns. The request still goes out,
and the first such tick of an enabled spell in which update() set an output
non-neutral prints one line on the node's stderr. talos.allow_untargeted_outputs(node) lifts the
target half for a test or bench fixture. Motor demands clamp to configured
limits; non-finite input neutralizes the motor. Feedback is in mechanism
units and reads 0 (connected False) until the first state.
"""

import ctypes
import dataclasses
import enum
import math
from datetime import timedelta

from talos import _abi
from talos._abi import TalosError, lib
from talos._messages import message_type


class Mode(enum.IntEnum):
    """A motor request's control mode (talos::hardware::Mode)."""
    NEUTRAL = _abi.MODE_NEUTRAL
    DUTY_CYCLE = _abi.MODE_DUTY_CYCLE
    VOLTAGE = _abi.MODE_VOLTAGE
    VELOCITY = _abi.MODE_VELOCITY
    POSITION = _abi.MODE_POSITION
    MOTION_MAGIC = _abi.MODE_MOTION_MAGIC
    TORQUE_CURRENT = _abi.MODE_TORQUE_CURRENT


def robot_enabled(node):
    """Whether this node's outputs reach the hardware."""
    return _abi.boolean(lib.talos_robot_enabled(node._handle))


def allow_untargeted_outputs(node):
    """Lifts the live-target half of the flush's gate for this node
    (talos::AllowUntargetedOutputs, RULES.md 1): its outputs are then sent
    as set whenever talos.robot_enabled(node), whether or not a talos.Target
    returned a target that tick. For a test or bench fixture with no
    producer by design; --describe and the registry mark its hardware
    request "untargeted_outputs". Call in __init__; afterwards it raises
    TalosError."""
    _abi.ok(lib.talos_allow_untargeted_outputs(node._handle))


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


_INT64_MAX = (1 << 63) - 1


def _max_age_ns(max_age):
    """Converts a fresh(max_age) bound (seconds, timedelta, or None) to
    int64 nanoseconds for the C ABI: -1 for None (default command timeout),
    None for NaN or negative (caller returns False), clamped to INT64_MAX
    for +inf or values >= INT64_MAX ns so c_int64 never wraps negative."""
    if max_age is None:
        return -1
    if isinstance(max_age, timedelta):
        ns = ((max_age.days * 86400 + max_age.seconds) * 1_000_000
              + max_age.microseconds) * 1000
        return None if ns < 0 else min(ns, _INT64_MAX)
    if isinstance(max_age, int):
        if max_age < 0:
            return None
        return min(max_age * 1_000_000_000, _INT64_MAX)
    seconds = float(max_age)
    if math.isnan(seconds) or seconds < 0:
        return None
    if math.isinf(seconds) or seconds >= _INT64_MAX / 1e9:
        return _INT64_MAX
    return min(round(seconds * 1e9), _INT64_MAX)


class HardwareLink(_OnNode):
    """Accepted hardware state; ages are seconds since publication."""

    def __init__(self, node):
        super().__init__(node)
        # Attach the C++ extension during construction, before the loop starts.
        _abi.ok(lib.talos_hardware_fresh(self._node, -1))

    @property
    def enabled(self) -> bool:
        """Whether this node's outputs reach the hardware."""
        return robot_enabled(self._owner)

    def fresh(self, max_age=None) -> bool:
        """Whether state is younger than max_age (default command timeout)."""
        node = self._node
        ns = _max_age_ns(max_age)
        if ns is None:
            return False
        return _abi.boolean(lib.talos_hardware_fresh(node, ns))

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


class _TimedInput(_Input):
    _TIME = None  # its talos_*_time entry point

    def _time(self, velocity):
        sampled = ctypes.c_int64()
        found = _abi.ok(self._TIME(
            self._node, self._index, velocity, ctypes.byref(sampled)))
        return sampled.value if found else None


class _Rotations:
    """position and velocity of a rotational sensor's sample."""

    @property
    def position(self):  # rotations
        return self._sample().position

    @property
    def velocity(self):  # rotations/s
        return self._sample().velocity


class _PositionTimedInput(_Rotations, _TimedInput):
    _AGE = None  # its talos_*_age entry point

    def _age(self, velocity):
        age = ctypes.c_int64()
        found = _abi.ok(self._AGE(
            self._node, self._index, velocity, ctypes.byref(age)))
        return age.value if found else None

    @property
    def position_time(self):
        """Acquisition time, integer node-clock ns, or None if unavailable."""
        return self._time(False)

    @property
    def velocity_time(self):
        """Acquisition time, integer node-clock ns, or None if unavailable."""
        return self._time(True)

    @property
    def position_age(self):
        """The position sample's age at the State it arrived in, integer ns:
        read from the State, so known without the clock mapping position_time
        needs. None unless connected, or when the age is unknown."""
        return self._age(False)

    @property
    def velocity_age(self):
        """The velocity sample's age at its State, integer ns, or None, as
        position_age."""
        return self._age(True)


class _Rezeroable:
    _REZERO = None
    _ZEROED = None

    def _rezero(self, offset_rot=0):
        _abi.ok(self._REZERO(self._node, self._index, offset_rot))

    @property
    def zeroed(self):
        """Whether a rezero holds: False until one takes (a restarted node
        starts unzeroed), and again once one is dropped: on a gateway boot
        change, a reset that masks this device in the next gateway State, or
        any device reset across a sequence gap. A disconnect keeps it, and so
        does every reset of a motor whose config sets
        position_frame = "absolute". A dropped zero keeps its offset
        (dropping does not move position or yaw). Motor set_position and
        set_motion_magic go out neutral while no rezero holds, unless
        position_frame = "absolute" (a remote CANcoder's frame, usable from
        the first state)."""
        return _abi.boolean(self._ZEROED(self._node, self._index))


class Motor(_Rezeroable, _PositionTimedInput):
    _KIND = _abi.DEVICE_MOTOR
    _SAMPLE = lib.talos_motor_sample
    _TIME = lib.talos_motor_time
    _AGE = lib.talos_motor_age
    _REZERO = lib.talos_motor_rezero
    _ZEROED = lib.talos_motor_zeroed

    def rezero(self, position_rot=0):
        """Set the node-shared software position on connected finite feedback.

        Position goals use this frame; soft limits remain hardware coordinates.
        Disconnected/non-finite feedback or input, or a position sample older
        than two status periods at its State (C++ RezeroAgeBound), leaves the
        offset unchanged: zeroed stays as it was, so retry on a later tick.
        """
        self._rezero(position_rot)

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

    def set_torque_current(self, amperes):
        """Stator current, amperes, through Phoenix TorqueCurrentFOC
        (Motor::set_torque_current). Raises TalosError naming the motor and
        the key unless its config sets foc = true and
        max_torque_current_a > 0; clamped to +-max_torque_current_a
        (clamped_count counts it); non-finite is neutral. 0.0 is a driving
        request, not neutral; stop() is neutral. What the motor does at
        exactly 0 A: talOS/hardware/README.md, Supported Hardware Requests."""
        self._set(Mode.TORQUE_CURRENT, amperes)

    def stop(self):
        self._set(Mode.NEUTRAL, 0.0)

    @property
    def voltage(self):  # applied volts
        return self._sample().voltage

    @property
    def current(self):  # stator amps
        return self._sample().current

    @property
    def temperature_c(self):
        """Motor controller temperature in Celsius."""
        return self._sample().temperature_c

    @property
    def supply_current_a(self):
        """Supply current in amps, distinct from current (stator amps)."""
        return self._sample().supply_current_a

    @property
    def forward_limit(self):
        """Whether the TalonFX's forward hardware limit switch is asserted
        (Motor::forward_limit): False unless connected; otherwise read from a
        FaultField signal under 50 ms old (cleared once it is staler)."""
        return bool(self._sample().forward_limit)

    @property
    def reverse_limit(self):
        """Whether the TalonFX's reverse hardware limit switch is asserted
        (Motor::reverse_limit): False unless connected; otherwise read from a
        FaultField signal under 50 ms old (cleared once it is staler)."""
        return bool(self._sample().reverse_limit)

    def _set(self, mode, demand, ff_volts=0.0, slot=0):
        _abi.ok(lib.talos_motor_set(self._node, self._index, mode, demand,
                                    ff_volts, slot))


class CANcoder(_PositionTimedInput):
    _KIND = _abi.DEVICE_CANCODER
    _SAMPLE = lib.talos_sensor_sample
    _TIME = lib.talos_sensor_time
    _AGE = lib.talos_sensor_age


class Gyro(_Rezeroable, _TimedInput):
    """A Pigeon2. Rotations and rotations/s, CCW+."""
    _KIND = _abi.DEVICE_GYRO
    _SAMPLE = lib.talos_sensor_sample
    _TIME = lib.talos_sensor_time
    _REZERO = lib.talos_gyro_rezero
    _ZEROED = lib.talos_gyro_zeroed

    @property
    def yaw_time(self):
        """Acquisition time, integer node-clock ns, or None if unavailable."""
        return self._time(False)

    @property
    def yaw_rate_time(self):
        """Acquisition time, integer node-clock ns, or None if unavailable."""
        return self._time(True)

    def rezero(self, yaw_rot=0):
        """Set node-shared software yaw on connected finite feedback/input."""
        self._rezero(yaw_rot)

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


class Encoder(_Rotations, _Input):
    _KIND = _abi.DEVICE_ENCODER
    _SAMPLE = lib.talos_encoder_sample


class DigitalOutput(_Device):
    _KIND = _abi.DEVICE_DIGITAL_OUTPUT

    def set(self, value):
        _abi.ok(lib.talos_digital_output_set(self._node, self._index,
                                             int(bool(value))))


class PwmOutput(_Device):
    _KIND = _abi.DEVICE_PWM

    def set(self, output):
        """A pulse for `output`, clamped to [-1, 1]: 0 is the 1.5 ms centre
        pulse. A non-finite output sends no pulse."""
        _abi.ok(lib.talos_pwm_set(self._node, self._index, output))

    def stop(self):
        """No pulse in this tick's flush, the same as not calling set() in
        this update()."""
        _abi.ok(lib.talos_pwm_stop(self._node, self._index))


class Alliance(enum.IntEnum):
    """talos::driver_station::Alliance."""
    UNKNOWN = _abi.ALLIANCE_UNKNOWN
    RED = _abi.ALLIANCE_RED
    BLUE = _abi.ALLIANCE_BLUE


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
        if t is not None:       # motor needs a position frame (below)
            self.motor.set_motion_magic(t.position_rot)

    get() returns the message of the first lane, in the current Driver
    Station mode's priority order ([robot.targets], the subsystem's
    [targets]), that is active, fresh and stamped with the current mode
    generation, while robot_enabled(node); otherwise None. `source` names
    that lane, or "".

    Motion magic needs a position frame: position_frame = "absolute" on a
    remote CANcoder, or a rezero first (docs/NODE_DEVELOPMENT_GUIDE.md,
    "Rezero and position frames"); until then it goes out neutral.
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
        name = lib.talos_target_source_name(
            self._node, self._index, self._source.value)
        if name is None:
            _abi.fail()
        return name.decode()


class ArmingInputKind(enum.IntEnum):
    """Which kind of operator input blocks OperatorArming (REQ-OPS-55)."""
    NONE = _abi.ARMING_INPUT_NONE
    AXIS = _abi.ARMING_INPUT_AXIS
    BUTTON = _abi.ARMING_INPUT_BUTTON
    POV = _abi.ARMING_INPUT_POV
    OTHER = _abi.ARMING_INPUT_OTHER


@dataclasses.dataclass(frozen=True)
class ArmingBlocker:
    """Fixed-size description of the input blocking OperatorArming."""
    kind: ArmingInputKind = ArmingInputKind.NONE
    port: int = -1
    index: int = -1

    def __post_init__(self):
        if (isinstance(self.port, bool) or not isinstance(self.port, int) or
                not -128 <= self.port <= 127):
            raise ValueError(
                f"ArmingBlocker.port must be an int8 (-128..127), "
                f"got {self.port!r}")
        if (isinstance(self.index, bool) or not isinstance(self.index, int) or
                not -32768 <= self.index <= 32767):
            raise ValueError(
                f"ArmingBlocker.index must be an int16 (-32768..32767), "
                f"got {self.index!r}")


class OperatorArming(_OnNode):
    """Whether an operator-input producer may publish active targets
    (talos::OperatorArming): armed by a reading that found every input it
    maps at rest, and only for the Driver Station view it was armed under.

        self.arming = talos.OperatorArming(node)
        self.climb = talos.TargetLane(node, ClimbTarget, "climber",
                                      "operator", arming=self.arming)
        ...
        held = self.ds.joystick(1).button(3)
        blocker = (talos.ArmingBlocker(talos.ArmingInputKind.BUTTON, 1, 3)
                   if held else talos.ArmingBlocker())
        if self.arming.evaluate(neutral=not held, blocker=blocker) and held:
            self.climb.send(...)
        else:
            self.climb.release()

    evaluate() is True when this reading may publish: armed by an earlier
    one, and not disarmed since. Otherwise it is False, and a neutral
    reading under a Driver Station mode arms it at once. It disarms at
    startup; when the Driver Station shows no mode, a new mode generation or
    a new gateway boot (so after any disable or mode switch, sampled or
    not); after the link recovers from a stale spell; when readings stop
    for command_timeout_us; and on disarm(). Evaluate on every reading.
    """

    def __init__(self, node):
        super().__init__(node)
        self._index = _abi.index(lib.talos_operator_arming_open(self._node))
        self._armed_out = ctypes.c_int32(0)
        self._kind_out = ctypes.c_uint8(0)
        self._port_out = ctypes.c_int8(-1)
        self._index_out = ctypes.c_int16(-1)

    def evaluate(self, neutral, blocker=None) -> bool:
        if blocker is None:
            return _abi.boolean(lib.talos_operator_arming_evaluate(
                self._node, self._index, 1 if neutral else 0))
        if not isinstance(blocker, ArmingBlocker):
            blocker = ArmingBlocker(blocker.kind, blocker.port, blocker.index)
        _abi.ok(lib.talos_operator_arming_evaluate_blocker(
            self._node, self._index, 1 if neutral else 0,
            int(ArmingInputKind(blocker.kind)), blocker.port,
            blocker.index, ctypes.byref(self._armed_out)))
        return bool(self._armed_out.value)

    @property
    def armed(self) -> bool:
        """Armed now, without evaluating: what an armed lane's send()
        checks."""
        return _abi.boolean(lib.talos_operator_arming_armed(
            self._node, self._index))

    @property
    def blocker(self) -> ArmingBlocker:
        """The blocking input recorded while disarmed, or NONE/-1/-1 when
        armed or neutral."""
        _abi.ok(lib.talos_operator_arming_blocker(
            self._node, self._index, ctypes.byref(self._kind_out),
            ctypes.byref(self._port_out), ctypes.byref(self._index_out)))
        return ArmingBlocker(
            ArmingInputKind(self._kind_out.value),
            self._port_out.value,
            self._index_out.value)

    def disarm(self):
        _abi.ok(lib.talos_operator_arming_disarm(self._node, self._index))


class TargetLane(_OnNode):
    """One producer's lane, /<subsystem>/target/<source> (talos::TargetLane):

        self.lane = talos.TargetLane(node, PositionTarget, "wrist", "auto")
        self.lane.send(position_rot=0.25, feedforward_v=0.0)  # every period
        self.lane.release()                 # nothing to say

    send() stamps the mode generation this node's Driver Station shows (0,
    which no consumer accepts, while it shows no mode), so a target from an
    earlier enable or mode never drives this one. A lane of operator input
    takes arming=OperatorArming(node): while that is not armed, send()
    publishes a release and returns False. Any node that reads DriverStation
    (or directly subscribes to /hw/state/driver_station) and writes a lane in
    the target subsystem's teleop policy must pass arming=OperatorArming(node);
    otherwise startup and LintGraph refuse it (REQ-SAFE-14).
    """

    def __init__(self, node, msg, subsystem, source, arming=None):
        super().__init__(node)
        self._type = message_type(msg)
        if arming is not None:
            if not isinstance(arming, OperatorArming):
                raise TypeError(
                    f"{subsystem}/target/{source}: arming must be an "
                    f"OperatorArming, got {type(arming).__name__}")
            if arming._owner is not node:
                raise TalosError(f"{subsystem}/target/{source}: the lane's "
                                 "OperatorArming belongs to another node")
        self._index = _abi.index(lib.talos_target_lane_open(
            self._node, _abi.encode(subsystem), _abi.encode(source),
            self._type.size, self._type.alignment, _abi.schema_of(self._type),
            -1 if arming is None else arming._index))

    def send(self, message=None, /, **fields):
        """send(field=value, ...) with the .fbs field names, or send(message)
        to forward a received one. False when the transport dropped it or the
        lane's arming held it to a release."""
        data = self._type.payload("send", message, fields)
        return _abi.boolean(lib.talos_target_lane_send(
            self._node, self._index, data, len(data)))

    def release(self):
        return _abi.boolean(lib.talos_target_lane_release(
            self._node, self._index))
