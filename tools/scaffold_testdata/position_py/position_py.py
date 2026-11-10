"""PositionPy: one motor driven to the position its target asks for.

Producers write lanes /position_py/target/<source> with talos.TargetLane; the
policy comes from [robot.targets] or the subsystem's [targets].

Positions count from the home switch (position_py_home) at the reverse end of
travel, not from where the motor powered on. The motor sets requires_zero, so
its Motion Magic goals go out neutral until the switch has zeroed it
(Motor.rezero), and again after a motor reset or a RoboRIO reboot drops that
zero. While unzeroed with a live, finite target it homes: it drives toward
the switch at homing_v (negative: toward the reverse end; a voltage request,
which a dropped zero does not hold) until the switch has read closed on
home_samples consecutive ticks (default 3), then rezeroes to 0 and goes to
the target. One closed sample, a glitch mid-travel, does not zero it: homing
keeps driving and any open sample restarts the count. Targets clamp to 0..max_position_rot in the homed
frame; the motor has no hardware soft limits, which would act in its power-on
frame and could stop homing.

The switch is normally open, wired to ground, read through the input's
pull-up: closed reads False, and an open or cut wire reads "not home", so a
wiring fault makes homing fail rather than zero mid-travel.

FAILED latches, neutral, when homing has not found the switch within
homing_timeout_us, when the motor has not moved (|velocity| below
homing_stall_rps) for homing_stall_us while homing pushes it, or when the
switch reads closed with the zeroed motor more than home_margin_rot from it
(a stuck switch). It clears only when the Driver Station reports disabled,
not when the hardware link merely goes stale.
"""

import math

import talos
from position_py import topics
from position_py.PositionPyHoming import PositionPyHoming
from position_py.PositionPyState import PositionPyState
from position_py.PositionPyTarget import PositionPyTarget


def _positive(node, key):
    """A finite, positive [subsystem] value."""
    value = float(node.param(key))
    if not (math.isfinite(value) and value > 0):
        raise ValueError(f"position_py: {key} must be finite and positive")
    return value


class PositionPy:
    def __init__(self, node):
        self.node = node
        self.motor = talos.Motor(node, "position_py_motor")
        self.home = talos.DigitalInput(node, "position_py_home")
        self.ds = talos.DriverStation(node)
        self.tolerance_rot = node.param("tolerance_rot", 0.05)
        self.max_position_rot = _positive(node, "max_position_rot")
        self.homing_v = float(node.param("homing_v"))
        if not (math.isfinite(self.homing_v) and self.homing_v < 0):
            raise ValueError(
                "position_py: homing_v must be negative (toward the reverse end)")
        self.homing_timeout_s = _positive(node, "homing_timeout_us") / 1e6
        self.stall_rps = _positive(node, "homing_stall_rps")
        self.stall_s = _positive(node, "homing_stall_us") / 1e6
        self.home_margin_rot = _positive(node, "home_margin_rot")
        self.home_samples = int(node.param("home_samples", 3))
        if self.home_samples < 1:
            raise ValueError("position_py: home_samples must be at least 1")
        self.home_count = 0
        self.homing_since = None
        self.stalled_since = None
        self.failed = False
        self.target = talos.Target(node, PositionPyTarget)
        self.state = node.publish(PositionPyState, topics.STATE)

    def update(self):
        t = self.target.get()  # None: disabled, stale or released
        connected = self.motor.connected
        at_home = not self.home.value  # normally open, to ground
        if self.ds.connected and not self.ds.enabled:  # the operator's reset
            self.failed = False
            self.homing_since = None
            self.stalled_since = None
        # Debounced: home_samples consecutive closed readings while unzeroed.
        if not self.motor.zeroed and at_home:
            self.home_count += 1
        else:
            self.home_count = 0
        if self.home_count >= self.home_samples:
            self.motor.rezero(0.0)  # no-op while feedback is stale
        pos = self.motor.position
        if self.motor.zeroed:
            self.homing_since = None
            self.stalled_since = None
            if connected and at_home and pos > self.home_margin_rot:
                self.failed = True  # a stuck switch
        if self.failed or not self.motor.zeroed:
            drive = (not self.failed and t is not None and connected
                     and math.isfinite(t.position_rot) and self._home())
            if not drive:
                self.stalled_since = None  # counts only while pushing
            self.state.send(enabled=drive, position_rot=pos,
                            connected=connected, at_goal=False,
                            homing=(PositionPyHoming.FAILED if self.failed
                                    else PositionPyHoming.HOMING))
            return
        drive = (t is not None and connected
                 and math.isfinite(t.position_rot))
        at_goal = False
        if drive:
            goal = min(max(t.position_rot, 0.0), self.max_position_rot)
            self.motor.set_motion_magic(goal)
            at_goal = abs(pos - goal) <= self.tolerance_rot
        self.state.send(enabled=drive, position_rot=pos, connected=connected,
                        at_goal=at_goal, homing=PositionPyHoming.HOMED)

    def _home(self):
        """One homing tick: drives toward the switch unless homing has timed
        out (counted from the first homing command since the last zero or
        Driver Station disable; losing the target pauses the motion, not
        that clock) or stalled. Returns whether it drove."""
        now = self.node.now
        if self.homing_since is None:
            self.homing_since = now
        if abs(self.motor.velocity) >= self.stall_rps:
            self.stalled_since = None
        elif self.stalled_since is None:
            self.stalled_since = now
        if (now - self.homing_since >= self.homing_timeout_s
                or (self.stalled_since is not None
                    and now - self.stalled_since >= self.stall_s)):
            self.failed = True
            return False
        self.motor.set_voltage(self.homing_v)
        return True
