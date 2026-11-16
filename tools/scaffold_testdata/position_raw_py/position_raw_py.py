"""PositionRawPy: drives the mechanism to the position its target asks for.

Reports what it did on /position_raw_py/state. Producers write lanes
/position_raw_py/target/<source> with talos.TargetLane; the policy comes from
[robot.targets] or the subsystem's [targets].

Every line of the control is here, in this file: it homes on the switch
position_raw_py_home, then runs Motion Magic inside a fixed travel.
Positions count from that switch, not from where the motor powered on. Until
the switch has zeroed the motor, and again after a motor reset or a RoboRIO
reboot drops that zero, a live goal pushes toward the switch at HOMING_VOLTS
instead. A push that runs past its bound, or the switch still reading closed
after the first move off it, latches FAILED, neutral, until the Driver
Station reports disabled. "Writing the arm yourself" in the talos
repository's mechanisms README explains each rule.
"""

import math

import talos
from position_raw_py import topics
from position_raw_py.PositionRawPyHoming import PositionRawPyHoming
from position_raw_py.PositionRawPyState import PositionRawPyState
from position_raw_py.PositionRawPyTarget import PositionRawPyTarget

# PLACEHOLDER values, sized for the simulation new_subsystem's `Next:`
# describes. Set each for your mechanism before the robot drives it.
#
# Travel above the switch, in rotations: goals clamp to 0..MAX_ROT. Measure
# it on the mechanism.
MAX_ROT = 2.0
# The homing push, in volts; negative drives toward the switch. Keep it low:
# with the motor's stator_limit_a it bounds how hard the push meets the stop.
# Check its direction before the first enable.
HOMING_VOLTS = -0.5
# The push bound, in ticks of the subsystem's period_us: time a push across
# the full travel at HOMING_VOLTS and allow about twice that. 50 ticks is 1 s
# at period_us = 20000.
MAX_PUSH_TICKS = 50
# The switch must read open by here after each zero, in rotations: the
# distance from the zero at which it reliably opens, plus a little. Keep it
# small: from a wrong zero the mechanism can still move this far.
HOME_MARGIN_ROT = 0.15
# Within this of the goal, /position_raw_py/state reports at_goal.
TOLERANCE_ROT = 0.05
# Slower than this the motor is at rest, and the switch may zero it.
REST_RPS = 0.05


class PositionRawPy:
    def __init__(self, node):
        self.motor = talos.Motor(node, "position_raw_py")
        self.home = talos.DigitalInput(node, "position_raw_py_home")
        self.ds = talos.DriverStation(node)
        self.target = talos.Target(node, PositionRawPyTarget)
        self.state = node.publish(PositionRawPyState, topics.STATE)
        self.failed = False
        self.left_switch = False
        self.push_ticks = 0

    def update(self):
        motor = self.motor
        # The switch reads False when closed, True when open.
        home_connected = self.home.connected
        on_switch = home_connected and not self.home.value
        off_switch = home_connected and self.home.value
        # Zero on the switch once the motor is at rest, so the first move
        # does not start at homing speed.
        if (not motor.zeroed and on_switch
                and abs(motor.velocity) < REST_RPS):
            motor.rezero(0.0)
        zeroed = motor.zeroed
        if zeroed:
            self.push_ticks = 0  # homed: a later re-home pushes from zero
            if off_switch:
                self.left_switch = True
        else:
            self.left_switch = False  # a dropped zero homes and checks again
        # Only a Driver Station that reports disabled clears a failure: not a
        # release, a stale lane or a feedback blip, any of which would
        # otherwise start another push into the stop.
        if self.ds.connected and not self.ds.enabled:
            self.failed = False
            self.push_ticks = 0
        # Still closed after the first move off it: the switch is stuck, or
        # the zero is wrong. Stop before the move reaches HOME_MARGIN_ROT.
        if (zeroed and not self.left_switch
                and motor.position > HOME_MARGIN_ROT - TOLERANCE_ROT):
            self.failed = True
        t = self.target.get()  # None: disabled, stale or released
        connected = motor.connected
        live = (t is not None and connected
                and math.isfinite(t.position_rot) and not self.failed)
        drove = False
        goal = 0.0
        if live and zeroed:
            # Until the switch has opened, no farther than HOME_MARGIN_ROT.
            top = MAX_ROT if self.left_switch else HOME_MARGIN_ROT
            goal = min(max(t.position_rot, 0.0), top)
            motor.set_motion_magic(goal)
            drove = True
        elif live and not on_switch:
            # The push is bounded: a switch that never closes (a cut wire
            # reads open) would otherwise push into the stop while the
            # button is held.
            self.push_ticks += 1
            if self.push_ticks > MAX_PUSH_TICKS:
                # Pushed long enough to cross the travel: no switch.
                self.failed = True
            else:
                motor.set_voltage(HOMING_VOLTS)  # toward the switch
                drove = True
        # Otherwise nothing is set and the flush sends neutral.
        position = motor.position
        at_goal = drove and zeroed and abs(position - goal) < TOLERANCE_ROT
        if self.failed:
            homing = PositionRawPyHoming.FAILED
        elif zeroed:
            homing = PositionRawPyHoming.HOMED
        else:
            homing = PositionRawPyHoming.HOMING
        self.state.send(enabled=drove, position_rot=position,
                        connected=connected, at_goal=at_goal, homing=homing)
