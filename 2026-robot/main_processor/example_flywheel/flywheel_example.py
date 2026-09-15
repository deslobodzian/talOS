"""Example: velocity-controlled flywheel (closed loop + at-speed).

A shooter-style wheel: spin to a cruise speed as fast as the ramp
allows, wait until it is really there, then the shot is allowed. The
demand reuses ``student_policy.command_for`` with flywheel presets
(positive spins up, stop holds); the new pieces vs the roller example
are the closed-loop velocity command (``motor_commands.velocity`` on
the Talon's onboard PID, slot 0), a ``behaviors.SlewLimiter`` ramp so
the step to cruise never snaps the bus, and ``run_until_or_timeout``
so the sequence never waits forever.

Config (mirrors ``subsystem.toml`` in this directory; TUNED -- compare
the roller's all-zero slot0)::

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

Simulation: ``run_sim`` ramps to ``target_rps`` until within
``tolerance_rps`` or ``timeout_s``. Diagnostics: ``diagnose`` reports
spin-up time, final error, at-speed, and what the hold demand is.

Cancellation (every outcome states its motor output):

* completion (at speed) -> hold the cruise demand;
* timeout -> stop (check gains/current limits, never just raise it);
* interruption (step raised) -> propagates; the caller still stops;
* disable mid-spin -> treat as a timeout: stop, then re-plan on enable.

``run_sim`` returns ``hold_rps`` computed with
``behaviors.cancellation_demand`` so the mapping is code, not prose.

Commissioning order lives in ``NOTES.md`` in this directory.
"""

from talos_nodes.intake import behaviors, motor_commands, units
from talos_nodes.intake.examples.plant import VelocityPlant
from talos_nodes.intake.student_policy import Feedback, Target, command_for

MOTOR_NAME = "flywheel"
SLOT = 0
MAX_VELOCITY_RPS = 100.0
CRUISE_RPS = 80.0
TOLERANCE_RPS = 2.0
ACCELERATION_RPS2 = 160.0  # Slew rate toward cruise, like the TOML row.
DT_S = 0.005  # 5 ms shooter period.

CONFIG_TOML = """\
[subsystem]
name = "flywheel_spinup"
period_us = 5000

[motors.flywheel]
type = "TalonFX"
bus = "rio"
can_id = 9
inverted = false
brake = false
supply_limit_a = 40.0
stator_limit_a = 80.0
max_voltage = 12.0
sensor_to_mechanism_ratio = 1.0
max_velocity_rps = 100.0
cruise_velocity_rps = 80.0
acceleration_rps2 = 160.0
slot0 = { p = 0.1, i = 0.0, d = 0.0, s = 0.0, v = 0.12, a = 0.0, g = 0.0 }
"""


def spin_demand(target_rps):
    """Flywheel demand for an ask of ``target_rps`` (mechanism rev/s).

    Positive spins up to the target preset, anything else holds stop --
    a flywheel has no eject direction, so both the negative and zero
    branches map to stop through ``command_for``'s overrides. Validated
    at the ``Target`` boundary.
    """
    ask = units.check_rps(target_rps, "target_rps")
    return command_for(Target(ask), Feedback(), intake_rps=ask,
                       eject_rps=0.0, stop_rps=0.0)


def encode_velocity(state, ids, velocity_rps, name=MOTOR_NAME, slot=SLOT):
    """Encode one closed-loop velocity command for the named motor.

    ``velocity_rps`` is mechanism rev/s, clamped to +/- the config
    ``max_velocity_rps`` with its sign kept (``None`` = no clamp, same
    as the student API default). Returns the 1404-byte payload.
    """
    return motor_commands.velocity(state, ids, name, velocity_rps,
                                   slot=slot,
                                   max_velocity_rps=MAX_VELOCITY_RPS)


def run_sim(target_rps=CRUISE_RPS, tolerance_rps=TOLERANCE_RPS,
            timeout_s=3.0, tuned=True):
    """Spin the plant to ``target_rps`` until at-speed or timeout.

    Each tick maps the ask through ``spin_demand``, slews it toward the
    ramp limit, and steps the plant. Returns the trace plus ``(steps,
    done)`` from ``run_until_or_timeout`` and ``hold_rps`` from the
    cancellation mapping (cruise on completion, stop on timeout).
    """
    ask = units.check_rps(target_rps, "target_rps")
    ramp = behaviors.SlewLimiter(ACCELERATION_RPS2)
    plant = VelocityPlant(tuned=tuned, max_rps=MAX_VELOCITY_RPS)
    trace = []

    def _step(dt_s):
        demand = ramp.update(spin_demand(ask), dt_s)
        trace.append(plant.step(demand, dt_s))

    def _at_speed():
        return bool(trace) and abs(trace[-1] - ask) <= tolerance_rps

    steps, done = behaviors.run_until_or_timeout(_step, _at_speed,
                                                 timeout_s, DT_S)
    hold = behaviors.cancellation_demand(done=done, goal_rps=ask,
                                         safe_rps=0.0)
    final = trace[-1] if trace else 0.0
    return {"target_rps": ask, "steps": steps, "done": done,
            "velocities": trace, "final_rps": final, "hold_rps": hold,
            "spin_up_s": steps * DT_S, "error_rps": abs(final - ask)}


def diagnose(result, tolerance_rps=TOLERANCE_RPS):
    """Diagnostics for a ``run_sim`` result: what happened, in words."""
    lines = ["spin-up %.2f s, final error %.2f rps%s"
             % (result["spin_up_s"], result["error_rps"],
                "" if result["done"] else " (TIMED OUT)")]
    if result["done"]:
        lines.append("at speed (|error| <= %.1f rps): ready to shoot; "
                     "hold %.1f rps" % (tolerance_rps, result["hold_rps"]))
    else:
        lines.append("never reached speed: check gains/current limits, "
                     "not the timeout; command stop (hold %.1f rps)"
                     % result["hold_rps"])
    if result["velocities"] and result["velocities"][-1] < 0.05 * abs(
            result["target_rps"]):
        lines.append("wheel nearly dead: loop untuned (all gains zero?)")
    return {"lines": lines, "done": result["done"],
            "spin_up_s": result["spin_up_s"],
            "error_rps": result["error_rps"],
            "hold_rps": result["hold_rps"]}
