"""PositionPy: drives the mechanism to the position its target asks for.

Reports what it did on /position_py/state. Producers write lanes
/position_py/target/<source> with talos.TargetLane; the policy comes from
[robot.targets] or the subsystem's [targets].

The control is a talos.mechanisms.PositionMechanism (the C++
talos::mechanisms::PositionMechanism), configured by the subsystem file: the
motor [motors.position_py] and the parameter table [position_py] (keys and units in
talOS/mechanisms/README.md). It runs Motion Magic to the goal, clamped to
min_rot..max_rot, and homes on the switch position_py_home: positions count from
that switch, not from where the motor powered on. Until the switch has
zeroed the motor, and again after a motor reset or a RoboRIO reboot drops
that zero, a live goal drives toward the switch at homing_v instead. A
homing timeout, a homing push that does not move, or the switch reading
closed well into the travel (stuck) latches FAILED, neutral, until the
Driver Station reports disabled.
"""

import talos
from position_py import topics
from position_py.PositionPyHoming import PositionPyHoming
from position_py.PositionPyState import PositionPyState
from position_py.PositionPyTarget import PositionPyTarget
from talos.mechanisms import PositionMechanism, PositionPhase


class PositionPy:
    def __init__(self, node):
        self.mechanism = PositionMechanism(node, "position_py")
        self.target = talos.Target(node, PositionPyTarget)
        self.state = node.publish(PositionPyState, topics.STATE)

    def update(self):
        # The mechanism drives only on this tick's request. Without a live
        # target (disabled, stale, released, another mode's lane) stop(): the
        # motor goes neutral rather than holding an old goal.
        t = self.target.get()
        if t is not None:
            self.mechanism.set_goal(t.position_rot)  # homes first
        else:
            self.mechanism.stop()
        self.mechanism.update()
        m = self.mechanism
        if m.phase == PositionPhase.FAULTED:
            homing = PositionPyHoming.FAILED
        elif m.homed:
            homing = PositionPyHoming.HOMED
        else:
            homing = PositionPyHoming.HOMING
        self.state.send(enabled=m.driving, position_rot=m.position_rot,
                        connected=m.connected, at_goal=m.at_goal,
                        homing=homing)
