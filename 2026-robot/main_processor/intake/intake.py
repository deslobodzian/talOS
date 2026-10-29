"""Intake: a roller wheel and a beam break.

The arbiter asks for a wheel speed on /intake/target. Every tick the intake
spins the roller for that ask (holding still once a piece breaks the beam,
unless asked to eject) and reports what it did on /intake/state. Gains live
in the robot config ([motors.roller] slot0); the Talon closes the velocity
loop onboard.
"""

import math

import talos
from intake import topics
from intake.Inhibit import Inhibit
from intake.IntakeState import IntakeState
from intake.IntakeTarget import IntakeTarget

INTAKE_RPS = 30.0  # positive ask: pull a piece in
EJECT_RPS = -20.0  # negative ask: push it back out


def roller_demand(ask_rps, beam_broken):
    """A held piece stops the pull but can still be ejected."""
    if not math.isfinite(ask_rps):
        return ask_rps
    if ask_rps > 0:
        return 0.0 if beam_broken else INTAKE_RPS
    if ask_rps < 0:
        return EJECT_RPS
    return 0.0


class Intake:
    def __init__(self, node):
        self.node = node
        self.roller = talos.Motor(node, "roller")
        self.beam = talos.DigitalInput(node, "intake_beam")
        self.target = node.subscribe(IntakeTarget, topics.TARGET)
        self.state = node.publish(IntakeState, topics.STATE)

    def update(self):
        target = self.target.value  # None until the first
        live = self.target.fresh() and target.enabled
        demand = (roller_demand(target.roller_velocity_rps, self.beam.value)
                  if live else 0.0)
        inhibit = self.inhibit(target, live, demand)
        if inhibit == Inhibit.DRIVE:
            self.roller.set_velocity(demand)  # otherwise the flush sends neutral
        self.state.send(
            issued_ns=round(self.node.now * 1e9),
            enabled=talos.robot_enabled(self.node),
            roller_velocity_rps=self.roller.velocity,
            beam_broken=self.beam.value,
            requested_velocity_rps=demand if math.isfinite(demand) else 0.0,
            inhibit=inhibit)

    def inhibit(self, target, live, demand):
        """Why the wheel must not drive this tick; the first reason wins."""
        if not talos.robot_enabled(self.node):
            return Inhibit.GATEWAY_NOT_READY
        if target is None or not target.enabled:
            return Inhibit.TARGET_DISABLED
        if not live:
            return Inhibit.TARGET_STALE
        if not self.roller.connected:
            return Inhibit.FEEDBACK_INVALID
        if not math.isfinite(demand):
            return Inhibit.DEMAND_NONFINITE
        return Inhibit.DRIVE
