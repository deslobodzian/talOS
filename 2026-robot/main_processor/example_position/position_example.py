"""Example: position-controlled mechanism (position servo + gearing).

An elevator-style shelf mechanism: go to a shelf height in wheel
rotations and stay there. The new pieces vs the velocity examples are
the position demand (mechanism rotations on the Talon's onboard PID via
``motor_commands.position``, slot 0) and the gearing point: a 25:1
gearbox means 25 rotor rotations per mechanism rotation, so every
demand and every feedback crosses ``units`` at the boundary -- the
demand stays in mechanism rotations on the wire, and ``units`` crosses
frames only for sensor math.

Config (mirrors ``subsystem.toml`` in this directory; note the ratio)::

    [motors.lift]
    type = "TalonFX"
    bus = "rio"
    can_id = 11
    brake = true
    sensor_to_mechanism_ratio = 25.0
    max_velocity_rps = 4.0
    slot0 = { p = 0.5, i = 0.0, d = 0.0, s = 0.2, v = 0.3, a = 0.0, g = 0.0 }

Simulation: ``run_sim`` drives a proportional position law (example
logic, not shared code) through a cruise-velocity clamp and a
``behaviors.SlewLimiter`` ramp to ``target_rot`` until within
``tolerance_rot`` or ``timeout_s``. Diagnostics: ``diagnose`` reports
final error, elapsed time, and whether the move ever hit the cruise
limit.

Cancellation (every outcome states its motor output):

* completion (arrived) -> keep servoing to the shelf (hold position);
* timeout -> stop (check gains/current limits, never just raise it);
* interruption (step raised) -> propagates; the caller still stops;
* disable mid-move -> treat as a timeout: stop, then re-plan on enable.

``run_sim`` returns ``hold`` as a ``(mode, value)`` pair --
``("position", target)`` on completion, ``("stop", 0.0)`` on timeout --
so the mapping is code, not prose.

Commissioning order lives in ``NOTES.md`` in this directory.
"""

from talos_nodes.intake import behaviors, motor_commands, units
from talos_nodes.intake.examples.plant import VelocityPlant

MOTOR_NAME = "lift"
SLOT = 0
RATIO = 25.0  # Rotor rotations per mechanism rotation (see CONFIG_TOML).
CRUISE_RPS = 4.0  # Cruise clamp: long moves slew at max velocity.
RAMP_RPS2 = 8.0  # Slew rate on the velocity demand (2x cruise per second).
KP = 2.0  # Example-only position gain: velocity demand per rotation error.
TOLERANCE_ROT = 0.05
DT_S = 0.02  # 20 ms policy period.

CONFIG_TOML = """\
[subsystem]
name = "lift_shelf"
period_us = 20000

[motors.lift]
type = "TalonFX"
bus = "rio"
can_id = 11
inverted = false
brake = true
supply_limit_a = 40.0
stator_limit_a = 80.0
max_voltage = 12.0
sensor_to_mechanism_ratio = 25.0
max_velocity_rps = 4.0
slot0 = { p = 0.5, i = 0.0, d = 0.0, s = 0.2, v = 0.3, a = 0.0, g = 0.0 }
"""


def demand_for(target_rot, position_rot):
    """Proportional position law: rotations of error -> rps demand.

    Both arguments are mechanism rotations; the output is a mechanism
    rev/s demand clamped to +/- cruise (``units.clamp_demand`` shows the
    soft-limit pattern: clamp the demand to the shelf range so a sign
    error cannot drive through the hard stop). Validated at this
    boundary: non-finite positions raise.
    """
    target = units.check_rotations(target_rot, "target_rot")
    position = units.check_rotations(position_rot, "position_rot")
    error_rot = target - position
    # Gearing point, crossed explicitly: the rotor-frame error is the
    # quantity the Talon loop closes on, while the demand below stays in
    # the mechanism frame on the wire.
    rotor_error = units.mechanism_rotations_to_rotor(error_rot, RATIO)
    _mechanism_check = units.rotor_to_mechanism_rotations(rotor_error,
                                                          RATIO)
    assert abs(_mechanism_check - error_rot) < 1e-9
    return units.clamp_demand(KP * error_rot, CRUISE_RPS)


def encode_position(state, ids, position_rot, name=MOTOR_NAME, slot=SLOT):
    """Encode one closed-loop position command for the named motor.

    ``position_rot`` is mechanism rotations (finite required).
    Returns the 1404-byte ``/hw/request`` payload.
    """
    return motor_commands.position(state, ids, name, position_rot,
                                   slot=slot)


def run_sim(target_rot, start_rot=0.0, tolerance_rot=TOLERANCE_ROT,
            timeout_s=5.0, tuned=True):
    """Drive to ``target_rot`` (mechanism rotations) until close or timeout.

    Each step converts the error to a velocity demand (``demand_for``),
    slews it through the ramp, and steps the plant; position integrates
    measured velocity. Returns the position/velocity traces plus
    ``(steps, done)`` and the ``hold`` pair from the cancellation table.
    """
    target = units.check_rotations(target_rot, "target_rot")
    position_rot = units.check_rotations(start_rot, "start_rot")
    ramp = behaviors.SlewLimiter(RAMP_RPS2)
    plant = VelocityPlant(tuned=tuned, max_rps=CRUISE_RPS)
    positions, velocities, demands = [position_rot], [], []
    over_cruise = [False]

    def _step(dt_s):
        raw = demand_for(target, positions[-1])
        if abs(KP * (target - positions[-1])) > CRUISE_RPS:
            over_cruise[0] = True
        demand = ramp.update(raw, dt_s)
        velocity = plant.step(demand, dt_s)
        positions.append(positions[-1] + velocity * dt_s)
        velocities.append(velocity)
        demands.append(demand)

    def _arrived():
        return abs(positions[-1] - target) <= tolerance_rot

    steps, done = behaviors.run_until_or_timeout(_step, _arrived,
                                                 timeout_s, DT_S)
    hold = (("position", target) if done else ("stop", 0.0))
    return {"target_rot": target, "steps": steps, "done": done,
            "hold": hold, "positions": positions,
            "velocities": velocities, "demands": demands,
            "final_rot": positions[-1],
            "error_rot": abs(positions[-1] - target),
            "cruise_limited": over_cruise[0],
            "elapsed_s": steps * DT_S}


def diagnose(result, tolerance_rot=TOLERANCE_ROT):
    """Diagnostics for a ``run_sim`` result: what happened, in words."""
    lines = ["arrived in %.2f s, final error %.3f rot%s"
             % (result["elapsed_s"], result["error_rot"],
                "" if result["done"] else " (TIMED OUT)")]
    if result["done"]:
        lines.append("holding %s at %.2f rot"
                     % (result["hold"][0], result["hold"][1]))
    else:
        lines.append("never arrived: check gains/current limits, "
                     "not the timeout; commanding stop")
    if result["cruise_limited"]:
        lines.append("demand hit the cruise clamp: long moves slew at "
                     "max velocity, as designed")
    if result["error_rot"] > tolerance_rot and result["done"]:
        lines.append("at tolerance edge: raise the position gain only "
                     "until the stage stops short, then stop")
    return {"lines": lines, "done": result["done"],
            "error_rot": result["error_rot"],
            "elapsed_s": result["elapsed_s"], "hold": result["hold"]}
