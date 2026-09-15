"""Example 2: flywheel (velocity + at-speed detection).

A shooter-style wheel: spin to a cruise speed as fast as possible, wait
until it is really there, then shoot. The policy reuses
``student_policy.command_for`` with flywheel presets (positive spins up,
stop holds); the new piece is at-speed detection plus
``run_until_or_timeout`` so the sequence never waits forever.

Config (``subsystem.toml`` excerpt -- TUNED, compare the intake's zeros;
mirrors ``2026-robot/main_processor/shooter/subsystem.toml``)::

    [motors.flywheel]
    type = "TalonFX"
    bus = "rio"
    can_id = 9
    brake = false
    sensor_to_mechanism_ratio = 1.0
    max_velocity_rps = 100.0
    cruise_velocity_rps = 80.0
    acceleration_rps2 = 160.0
    slot0 = { p = 0.1, i = 0.0, d = 0.0, s = 0.0, v = 0.12, a = 0.0, g = 0.0 }

Simulation: ``run_sim`` spins the tuned plant to ``target_rps`` until
within ``tolerance_rps`` or ``timeout_s``. Diagnostics: ``diagnose``
reports spin-up time, final error, and at-speed.

Commissioning notes (do these on the real rig, in order):

1. ``brake = false``: a flywheel must coast down, never brake-stop --
   braking a spinning wheel browns out the bus for no reason.
2. Tune ``v`` (kV) until the wheel holds speed under load, then add a
   touch of ``p``; overshoot here means missed shots, so back ``p`` off
   at the first oscillation.
3. Pick ``tolerance_rps`` from shot data (2 rps is the shooter node's
   starting point), not from the sim: the sim plant is kinder than air
   drag.
4. If spin-up times out, the fault is gains or current limits
   (``supply_limit_a``/``stator_limit_a``), never the timeout -- do not
   just raise the timeout.
"""

from talos_nodes.intake import behaviors
from talos_nodes.intake.examples.plant import VelocityPlant
from talos_nodes.intake.student_policy import Feedback, Target, command_for

CONFIG_TOML = """\
[subsystem]
name = "shooter"
period_us = 5000

[motors.flywheel]
type = "TalonFX"
bus = "rio"
can_id = 9
brake = false
sensor_to_mechanism_ratio = 1.0
max_velocity_rps = 100.0
cruise_velocity_rps = 80.0
acceleration_rps2 = 160.0
slot0 = { p = 0.1, i = 0.0, d = 0.0, s = 0.0, v = 0.12, a = 0.0, g = 0.0 }

[digital_inputs.beam_break]
dio = 0
"""

DT_S = 0.005  # 5 ms shooter period.
CRUISE_RPS = 80.0
TOLERANCE_RPS = 2.0


def _spin_policy(target_rps):
    def _command(ask):
        return command_for(Target(ask), Feedback(),
                           intake_rps=target_rps, eject_rps=-target_rps,
                           stop_rps=0.0)
    return _command


def run_sim(target_rps=CRUISE_RPS, tolerance_rps=TOLERANCE_RPS,
            timeout_s=3.0, tuned=True):
    """Spin the plant to ``target_rps`` until at-speed or timeout.

    Returns a result dict with ``steps``/``done`` from
    ``run_until_or_timeout``, the velocity trace, and the final error.

    Cancellation: done holds the cruise demand (completion);
    not-done commands stop (timeout -- check gains/current limits,
    never just raise the timeout); interruption propagates and the
    caller still commands stop; disable is handled as a timeout.
    Map with ``behaviors.cancellation_demand``.
    """
    command = _spin_policy(target_rps)
    plant = VelocityPlant(tuned=tuned)
    trace = []

    def _step(dt_s):
        trace.append(plant.step(command(target_rps), dt_s))

    def _at_speed():
        return bool(trace) and abs(trace[-1] - target_rps) <= tolerance_rps

    steps, done = behaviors.run_until_or_timeout(_step, _at_speed,
                                                 timeout_s, DT_S)
    final = trace[-1] if trace else 0.0
    return {"target_rps": target_rps, "steps": steps, "done": done,
            "velocities": trace, "final_rps": final,
            "spin_up_s": steps * DT_S, "error_rps": abs(final - target_rps)}


def diagnose(result, tolerance_rps=TOLERANCE_RPS):
    """Diagnostics for a ``run_sim`` result: what happened, in words."""
    lines = ["spin-up %.2f s, final error %.2f rps%s"
             % (result["spin_up_s"], result["error_rps"],
                "" if result["done"] else " (TIMED OUT)")]
    if result["done"]:
        lines.append("at speed (|error| <= %.1f rps): ready to shoot"
                     % tolerance_rps)
    else:
        lines.append("never reached speed: check gains/current limits, "
                     "not the timeout")
    if result["velocities"] and result["velocities"][-1] < 0.05 * abs(
            result["target_rps"]):
        lines.append("wheel nearly dead: loop untuned (all gains zero?)")
    return {"lines": lines, "done": result["done"],
            "spin_up_s": result["spin_up_s"],
            "error_rps": result["error_rps"]}
