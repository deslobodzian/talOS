"""Intake: a roller wheel and a beam break.

The selected target lane (/intake/target/<source>, picked per match mode by
the robot's [robot.targets] policy) asks for a wheel speed. Every tick the intake
spins the roller for that ask (holding still once a piece breaks the beam,
and permitting negative feed only with a fresh, ready shooter) and reports
what it did on /intake/state. Gains live
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
    """A held piece stops collection; feeding bypasses the beam."""
    if not math.isfinite(ask_rps):
        return ask_rps
    if ask_rps > 0:
        return 0.0 if beam_broken else ask_rps
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

    def update(self):
        t = self.target.get()  # None: disabled, stale, released or never sent
        demand = (roller_demand(t.roller_velocity_rps, self.beam.value)
                  if t is not None else 0.0)
        inhibit = self.inhibit(t, demand)
        if inhibit == Inhibit.DRIVE:
            self.roller.set_velocity(demand)  # otherwise the flush sends neutral
        self.state.send(
            enabled=talos.robot_enabled(self.node),
            roller_velocity_rps=self.roller.velocity,
            beam_broken=self.beam.value,
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
        if demand < 0 and not (
                self.shooter.fresh() and self.shooter.value.at_speed and
                self.shooter.value.target_velocity_rps > 0):
            return Inhibit.SHOOTER_NOT_READY
        return Inhibit.DRIVE
