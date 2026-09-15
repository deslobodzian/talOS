"""Sim-only motor plant shared by the teaching examples.

NOT robot code: nothing here ships to the robot. It stands in for the
Talon + mechanism so students can watch tuned vs untuned behavior on a
laptop. First-order velocity servo: each ``step`` moves the velocity a
fraction of the way from here to ``gain * demand`` -- tuned means the
fraction is large (snappy), untuned means the demand barely gets
through (missing gains show as a nearly-dead motor, mirroring the RIO
sim where zero volts never snaps velocity to demand).
"""

from talos_nodes.intake import units


class VelocityPlant:
    """First-order velocity servo in mechanism rev/s."""

    def __init__(self, tuned=True, max_rps=100.0, tau_tuned_s=0.15,
                 untuned_gain=0.05, tau_untuned_s=5.0):
        self.tuned = tuned
        self.max_rps = units.check_rps(max_rps, "max_rps")
        if tau_tuned_s <= 0.0 or tau_untuned_s <= 0.0:
            raise ValueError("time constants must be > 0")
        self.tau_tuned_s = float(tau_tuned_s)
        self.tau_untuned_s = float(tau_untuned_s)
        self.untuned_gain = float(untuned_gain)
        self.velocity_rps = 0.0

    def step(self, demand_rps, dt_s):
        """Advance ``dt_s`` seconds toward the demand. Returns velocity."""
        demand = units.clamp_demand(demand_rps, self.max_rps)
        if dt_s <= 0.0:
            raise ValueError("dt_s must be > 0, got %r" % (dt_s,))
        if self.tuned:
            gain, tau = 1.0, self.tau_tuned_s
        else:
            gain, tau = self.untuned_gain, self.tau_untuned_s
        alpha = min(1.0, dt_s / tau)
        self.velocity_rps += alpha * (gain * demand - self.velocity_rps)
        return self.velocity_rps
