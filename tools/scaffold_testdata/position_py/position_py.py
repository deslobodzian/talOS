"""PositionPy: one motor driven to the position its target asks for.

Producers write lanes /position_py/target/<source> with talos.TargetLane; the
policy comes from [robot.targets] or the subsystem's [targets].
"""

import math

import talos
from position_py import topics
from position_py.PositionPyState import PositionPyState
from position_py.PositionPyTarget import PositionPyTarget


class PositionPy:
    def __init__(self, node):
        self.motor = talos.Motor(node, "position_py_motor")
        self.tolerance_rot = node.param("tolerance_rot", 0.05)
        self.target = talos.Target(node, PositionPyTarget)
        self.state = node.publish(PositionPyState, topics.STATE)

    def update(self):
        t = self.target.get()  # None: disabled, stale or released
        drive = (t is not None and self.motor.connected
                 and math.isfinite(t.position_rot))
        pos = self.motor.position
        at_goal = False
        if drive:
            self.motor.set_motion_magic(t.position_rot)
            at_goal = abs(pos - t.position_rot) <= self.tolerance_rot
        self.state.send(enabled=drive, position_rot=pos,
                        connected=self.motor.connected, at_goal=at_goal)
