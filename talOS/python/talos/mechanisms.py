"""The talOS/mechanisms building blocks (C++), for Python nodes.

    from talos.mechanisms import PositionMechanism

    class Arm:
        def __init__(self, node):
            self.arm = PositionMechanism(node, "arm")  # [motors.arm], [arm]
            self.target = talos.Target(node, PositionTarget)
            self.state = node.publish(PositionState, "state")

        def update(self):
            t = self.target.get()
            if t is not None:
                self.arm.set_goal(t.position_rot, t.feedforward_v)
            else:
                self.arm.stop()
            self.arm.update()
            self.state.send(self.arm.state())

Each class is the C++ one in talOS/mechanisms, run through
libtalos_node's C ABI: the same parameters ([<name>] table), the same
control, homing, readiness and faults (talOS/mechanisms/README.md). Take
them in __init__; the C++ object lives as long as the node. Requests
(set_goal, hold, stop, clear_fault) and update() are taken only inside the
node's update() and raise TalosError, naming the mechanism, anywhere else;
a request lasts that tick only. Call set_goal only with a live talos.Target.
The C++ getters are properties here, as of the last update(); state() builds
the control message the C++ state() returns.
"""

import enum

from control.PositionState import PositionState
from control.VelocityState import VelocityState
from talos import _abi
from talos._abi import lib
from talos._messages import message_type
from talos.devices import _OnNode


class PositionPhase(enum.IntEnum):
    """talos::mechanisms::PositionPhase."""
    UNHOMED = _abi.POSITION_PHASE_UNHOMED
    HOMING = _abi.POSITION_PHASE_HOMING
    READY = _abi.POSITION_PHASE_READY
    MOVING = _abi.POSITION_PHASE_MOVING
    AT_GOAL = _abi.POSITION_PHASE_AT_GOAL
    FAULTED = _abi.POSITION_PHASE_FAULTED


class PositionFault(enum.IntEnum):
    """talos::mechanisms::PositionFault."""
    NONE = _abi.POSITION_FAULT_NONE
    HOMING_TIMEOUT = _abi.POSITION_FAULT_HOMING_TIMEOUT
    HOMING_STALL = _abi.POSITION_FAULT_HOMING_STALL
    SWITCH_STUCK = _abi.POSITION_FAULT_SWITCH_STUCK
    STALL = _abi.POSITION_FAULT_STALL
    MOVE_TIMEOUT = _abi.POSITION_FAULT_MOVE_TIMEOUT


class VelocityPhase(enum.IntEnum):
    """talos::mechanisms::VelocityPhase."""
    STOPPED = _abi.VELOCITY_PHASE_STOPPED
    SPINNING_UP = _abi.VELOCITY_PHASE_SPINNING_UP
    AT_SPEED = _abi.VELOCITY_PHASE_AT_SPEED


class RollerCommand(enum.IntEnum):
    """talos::mechanisms::RollerCommand."""
    IDLE = _abi.ROLLER_COMMAND_IDLE
    INTAKE = _abi.ROLLER_COMMAND_INTAKE
    EJECT = _abi.ROLLER_COMMAND_EJECT
    FEED = _abi.ROLLER_COMMAND_FEED


class RollerPhase(enum.IntEnum):
    """talos::mechanisms::RollerPhase."""
    IDLE = _abi.ROLLER_PHASE_IDLE
    INTAKING = _abi.ROLLER_PHASE_INTAKING
    HOLDING = _abi.ROLLER_PHASE_HOLDING
    EJECTING = _abi.ROLLER_PHASE_EJECTING
    FEEDING = _abi.ROLLER_PHASE_FEEDING


class _Mechanism(_OnNode):
    _PREFIX = None  # talos_<kind>_
    _STATE = None   # its ctypes state struct

    def __init__(self, node, name):
        super().__init__(node)
        self._index = _abi.index(self._call("open", _abi.encode(name)))

    def _call(self, method, *args):
        function = getattr(lib, self._PREFIX + method)
        if method == "open":
            return function(self._node, *args)
        return _abi.ok(function(self._node, self._index, *args))

    def _state(self):
        out = self._STATE()
        self._call("state", out)
        return out

    def stop(self):
        """This tick's request: drive nothing (neutral)."""
        self._call("stop")

    def update(self):
        """Applies this tick's request; once per update()."""
        self._call("update")

    @property
    def driving(self):
        return bool(self._state().driving)


class PositionMechanism(_Mechanism):
    """talos::mechanisms::PositionMechanism: one motor to a position."""
    _PREFIX = "talos_position_mechanism_"
    _STATE = _abi.PositionMechanismState
    _MESSAGE = message_type(PositionState)

    def set_goal(self, position_rot, feedforward_v=0.0):
        self._call("set_goal", position_rot, feedforward_v)

    def set_goal_units(self, position_units, feedforward_v=0.0):
        self._call("set_goal_units", position_units, feedforward_v)

    def hold(self, feedforward_v=0.0):
        self._call("hold", feedforward_v)

    def clear_fault(self):
        self._call("clear_fault")

    phase = property(lambda self: PositionPhase(self._state().phase))
    fault = property(lambda self: PositionFault(self._state().fault))
    homed = property(lambda self: bool(self._state().homed))
    at_goal = property(lambda self: bool(self._state().at_goal))
    connected = property(lambda self: bool(self._state().connected))
    position_rot = property(lambda self: self._state().position_rot)
    position_units = property(lambda self: self._state().position_units)
    velocity_rps = property(lambda self: self._state().velocity_rps)
    goal_rot = property(lambda self: self._state().goal_rot)
    gravity_v = property(lambda self: self._state().gravity_v)
    tolerance_rot = property(lambda self: self._state().tolerance_rot)

    def state(self):
        """control.PositionState: position, velocity, goal_rot, enabled =
        driving, at_goal, connected."""
        s = self._state()
        return self._MESSAGE.decode(self._MESSAGE.encode({
            "position_rot": s.position_rot, "velocity_rps": s.velocity_rps,
            "goal_rot": s.goal_rot, "enabled": bool(s.driving),
            "at_goal": bool(s.at_goal), "connected": bool(s.connected)}))


class VelocityMechanism(_Mechanism):
    """talos::mechanisms::VelocityMechanism: one motor at a velocity."""
    _PREFIX = "talos_velocity_mechanism_"
    _STATE = _abi.VelocityMechanismState
    _MESSAGE = message_type(VelocityState)

    def set_goal(self, velocity_rps, feedforward_v=0.0):
        self._call("set_goal", velocity_rps, feedforward_v)

    phase = property(lambda self: VelocityPhase(self._state().phase))
    at_speed = property(lambda self: bool(self._state().at_speed))
    connected = property(lambda self: bool(self._state().connected))
    velocity_rps = property(lambda self: self._state().velocity_rps)
    goal_rps = property(lambda self: self._state().goal_rps)
    commanded_rps = property(lambda self: self._state().commanded_rps)

    def state(self):
        """control.VelocityState: velocity, requested = goal_rps, current,
        enabled = driving, connected."""
        s = self._state()
        return self._MESSAGE.decode(self._MESSAGE.encode({
            "velocity_rps": s.velocity_rps, "requested_rps": s.goal_rps,
            "current_a": s.current_a, "enabled": bool(s.driving),
            "connected": bool(s.connected)}))


class RollerWithSensor(_Mechanism):
    """talos::mechanisms::RollerWithSensor: a roller and a piece sensor."""
    _PREFIX = "talos_roller_with_sensor_"
    _STATE = _abi.RollerWithSensorState

    def set_goal(self, command):
        """This tick's request: a RollerCommand."""
        self._call("set_goal", int(RollerCommand(command)))

    phase = property(lambda self: RollerPhase(self._state().phase))
    has_piece = property(lambda self: bool(self._state().has_piece))
    sensor_connected = property(
        lambda self: bool(self._state().sensor_connected))
    output_v = property(lambda self: self._state().output_v)
