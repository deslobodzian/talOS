"""Example 1: roller intake (velocity + beam-break capture).

The year intake reduced to one screen: the arbiter asks for a roller
speed, ``student_policy.command_for`` maps it to a demand, the beam
sensor stops further intake once a piece is captured, and eject still
runs with the beam blocked so the piece can get back out.

Config (``subsystem.toml`` excerpt -- untuned on purpose, like the real
intake config)::

    [motors.roller]
    type = "TalonFX"
    bus = "rio"
    can_id = 10
    brake = true
    sensor_to_mechanism_ratio = 1.0
    max_velocity_rps = 100.0
    slot0 = { p = 0.0, i = 0.0, d = 0.0, s = 0.0, v = 0.0, a = 0.0, g = 0.0 }

Simulation: ``run_sim`` steps the policy against the sim plant with a
beam schedule (when the piece arrives). Diagnostics: ``diagnose``
reports requested vs measured, capture count, and whether eject cleared
the piece.

Commissioning notes (do these on the real rig, in order):

1. Wire the beam break to the configured DIO and watch ``beam_broken``
   flip by hand in ``/intake/state`` before ever enabling the wheel.
2. Set ``inverted`` so a positive demand pulls a piece IN; flip it if
   positive ejects.
3. ``brake = true`` holds a captured piece at stop; coast lets it roll
   back out.
4. Tune ``v`` (kV) first until steady-state measured ~= requested, then
   ``p`` until the step response stops lagging; leave ``i``/``d`` zero.
5. If the beam chatters on capture, raise the policy's
   ``beam_stable_ticks`` to 2-3 rather than touching the demand mapping.
"""

from talos_nodes.intake import behaviors
from talos_nodes.intake.examples.plant import VelocityPlant
from talos_nodes.intake.student_policy import Feedback, Target, command_for

CONFIG_TOML = """\
[subsystem]
name = "intake"
period_us = 20000

[motors.roller]
type = "TalonFX"
bus = "rio"
can_id = 10
brake = true
sensor_to_mechanism_ratio = 1.0
max_velocity_rps = 100.0
slot0 = { p = 0.0, i = 0.0, d = 0.0, s = 0.0, v = 0.0, a = 0.0, g = 0.0 }

[digital_inputs.intake_beam]
dio = 1
"""

DT_S = 0.02  # 20 ms policy period, like subsystem.toml period_us.


def run_sim(ask_rps, beam_at_s=None, eject_at_s=None, duration_s=2.0,
            tuned=True, beam_stable_ticks=1):
    """Run the intake policy against the sim plant.

    ``ask_rps`` is the arbiter ask (positive intakes). ``beam_at_s``
    breaks the beam from that time on (piece captured); ``eject_at_s``
    switches the ask to the eject preset from that time on (clearing the
    piece also clears the beam). Returns a result dict with the demand /
    velocity / beam traces plus ``captures``.

    Outputs by outcome (cancellation): capture holds the stop demand,
    eject drives the eject demand, and a disabled/interrupted run must
    command stop (see ``behaviors.cancellation_demand``) -- the sim
    just ends its window instead.
    """
    from talos_nodes.intake.student_policy import EJECT_RPS
    plant = VelocityPlant(tuned=tuned)
    beam_filter = behaviors.Debounce(stable_ticks=beam_stable_ticks)
    edges = behaviors.EdgeDetector()
    demands, velocities, beams = [], [], []
    captures = 0
    steps = int(duration_s / DT_S)
    for step in range(steps):
        now_s = step * DT_S
        ask = EJECT_RPS if eject_at_s is not None and now_s >= eject_at_s \
            else ask_rps
        raw_beam = beam_at_s is not None and now_s >= beam_at_s
        if eject_at_s is not None and now_s >= eject_at_s:
            raw_beam = False  # Ejecting clears the piece out of the beam.
        stable = beam_filter.update(raw_beam)
        rose, _ = edges.update(stable)
        captures += 1 if rose else 0
        demand = command_for(Target(ask), Feedback(beam_broken=stable))
        velocity = plant.step(demand, DT_S)
        demands.append(demand)
        velocities.append(velocity)
        beams.append(stable)
    return {"ask_rps": ask_rps, "demands": demands,
            "velocities": velocities, "beams": beams,
            "captures": captures, "final_rps": plant.velocity_rps}


def diagnose(result):
    """Diagnostics for a ``run_sim`` result: what happened, in words."""
    demands = result["demands"]
    velocities = result["velocities"]
    requested = max(abs(d) for d in demands)
    measured = max(abs(v) for v in velocities)
    lines = ["requested peak %.1f rps, measured peak %.1f rps"
             % (requested, measured)]
    if requested > 0.0 and measured < 0.5 * requested:
        lines.append("measured lags requested: loop untuned (raise v first) "
                     "or demand inhibited -- check the inhibit reason in "
                     "/intake/state")
    if result["captures"] > 0:
        lines.append("captured %d piece(s): intake held stop while the beam "
                     "was blocked" % result["captures"])
    if any(b for b in result["beams"]) and any(d < 0.0 for d in demands):
        lines.append("eject ran with the beam blocked: intentional ejection "
                     "is allowed")
    return {"lines": lines, "requested_peak": requested,
            "measured_peak": measured, "captures": result["captures"]}
