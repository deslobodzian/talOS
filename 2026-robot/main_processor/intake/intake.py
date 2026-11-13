"""Intake: a roller wheel and a beam break.

The selected target lane (/intake/target/<source>, picked per match mode by
the robot's [robot.targets] policy) asks for a wheel speed. Every tick the intake
spins the roller for that ask (holding still once a piece breaks the beam,
neutral while collecting on a beam sample that is not valid, and permitting
negative feed only with a fresh, ready shooter) and reports what it did on
/intake/state. Gains live
in the robot config ([motors.roller] slot0); the Talon closes the velocity
loop onboard.
"""

import math

import talos
from intake import topics
from intake.Inhibit import Inhibit
from intake.IntakeState import IntakeState
from intake.IntakeTarget import IntakeTarget
from shooter import topics as shooter_topics
from shooter.ShooterState import ShooterState


def roller_demand(ask_rps, beam_broken):
    """A held piece, or a beam with no valid reading (None), stops
    collection; feeding bypasses the beam."""
    if not math.isfinite(ask_rps):
        return ask_rps
    if ask_rps > 0:
        return ask_rps if beam_broken is False else 0.0
    if ask_rps < 0:
        return ask_rps
    return 0.0


class Intake:
    def __init__(self, node):
        self.node = node
        self.roller = talos.Motor(node, "roller")
        self.beam = talos.DigitalInput(node, "intake_beam")
        self.target = talos.Target(node, IntakeTarget)
        self.shooter = node.subscribe(ShooterState, shooter_topics.STATE)
        self.state = node.publish(IntakeState, topics.STATE)
        # The last valid beam reading. An invalid sample reads false, which
        # would claim no piece; /intake/state reports this instead.
        self.beam_broken = False

    def update(self):
        t = self.target.get()  # None: disabled, stale, released or never sent
        beam = self.beam.value if self.beam.connected else None
        if beam is not None:
            self.beam_broken = beam
        demand = (roller_demand(t.roller_velocity_rps, beam)
                  if t is not None else 0.0)
        inhibit = self.inhibit(t, demand)
        if inhibit == Inhibit.DRIVE:
            self.roller.set_velocity(demand)  # otherwise the flush sends neutral
        self.state.send(
            enabled=talos.robot_enabled(self.node),
            roller_velocity_rps=self.roller.velocity,
            beam_broken=self.beam_broken,
            requested_velocity_rps=demand if math.isfinite(demand) else 0.0,
            inhibit=inhibit)

    def inhibit(self, t, demand):
        """Why the wheel must not drive this tick; the first reason wins."""
        if not talos.robot_enabled(self.node):
            return Inhibit.GATEWAY_NOT_READY
        if t is None:
            return Inhibit.TARGET_STALE
        if not self.roller.connected:
            return Inhibit.FEEDBACK_INVALID
        if not math.isfinite(demand):
            return Inhibit.DEMAND_NONFINITE
        if t.roller_velocity_rps > 0 and not self.beam.connected:
            return Inhibit.FEEDBACK_INVALID  # collecting needs the beam
        if demand < 0 and not (
                self.shooter.fresh() and self.shooter.value.at_speed and
                self.shooter.value.target_velocity_rps > 0):
            return Inhibit.SHOOTER_NOT_READY
        return Inhibit.DRIVE
