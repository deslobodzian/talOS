"""ScaffoldPy: one motor driven at the velocity its target asks for.

Every tick the scaffold_py drives the motor at the target's velocity while a lane
of its target policy is live (robot enabled, lane active and fresh), the motor
is connected and the demand is finite, and reports what it did on
/scaffold_py/state. Gains live in the robot config ([motors.scaffold_py_motor] slot0);
the motor controller closes the velocity loop onboard.

Nothing asks yet. A teleop or autonomous producer writes its own lane,
/scaffold_py/target/<source>, with talos.TargetLane; talos.Target picks the live
lane by the policy in [robot.targets] or the subsystem's [targets].
"""

import math

import talos
from scaffold_py import topics
from scaffold_py.ScaffoldPyState import ScaffoldPyState
from scaffold_py.ScaffoldPyTarget import ScaffoldPyTarget


class ScaffoldPy:
    def __init__(self, node):
        self.motor = talos.Motor(node, "scaffold_py_motor")
        self.target = talos.Target(node, ScaffoldPyTarget)
        self.state = node.publish(ScaffoldPyState, topics.STATE)

    def update(self):
        t = self.target.get()  # None: disabled, stale or released
        drive = (t is not None and self.motor.connected
                 and math.isfinite(t.velocity_rps))
        if drive:
            # Otherwise nothing is set and the flush sends neutral.
            self.motor.set_velocity(t.velocity_rps)
        self.state.send(enabled=drive, velocity_rps=self.motor.velocity,
                        connected=self.motor.connected)
