"""Example 3: position mechanism (position servo + gearing).

An elevator-style shelf mechanism: go to a shelf height in wheel
rotations and stay there. The new pieces vs the velocity examples are
the position demand (mechanism rotations on the Talon's onboard PID via
``motor_commands.position``) and the gearing point: a 25:1 gearbox
means 25 rotor rotations per mechanism rotation, so every demand and
every feedback crosses ``units`` at the boundary.

Config (``subsystem.toml`` excerpt -- note the ratio)::

    [motors.lift]
    type = "TalonFX"
    bus = "rio"
    can_id = 11
    brake = true
    sensor_to_mechanism_ratio = 25.0
    max_velocity_rps = 4.0
    slot0 = { p = 0.5, i = 0.0, d = 0.0, s = 0.2, v = 0.3, a = 0.0, g = 0.0 }

Simulation: ``run_sim`` drives a proportional position law (example
logic, not shared code) through a cruise-velocity clamp to
``target_rot`` until within ``tolerance_rot`` or ``timeout_s``.
Diagnostics: ``diagnose`` reports final error and whether the demand
ever exceeded the cruise limit.

Commissioning notes (do these on the real rig, in order):

1. Verify the gearing FIRST: command 1 mechanism rotation by hand and
   confirm the wheel turns exactly once. A wrong
   ``sensor_to_mechanism_ratio`` multiplies every later mistake by 25.
2. ``brake = true``: a lift must hold position at stop, never coast.
3. Set soft limits before tuning gains: clamp the demand to the shelf
   range (``clamp_demand`` shows the pattern) so a sign error cannot
   drive through the hard stop.
4. Tune ``s`` until the stage just breaks loose, then ``v``, then ``p``
   -- gravity (``g``) only if the stage sags holding position.
"""

from talos_nodes.intake import behaviors, units
from talos_nodes.intake.examples.plant import VelocityPlant

CONFIG_TOML = """\
[subsystem]
name = "lift"
period_us = 20000

[motors.lift]
type = "TalonFX"
bus = "rio"
can_id = 11
brake = true
sensor_to_mechanism_ratio = 25.0
max_velocity_rps = 4.0
slot0 = { p = 0.5, i = 0.0, d = 0.0, s = 0.2, v = 0.3, a = 0.0, g = 0.0 }
"""

DT_S = 0.02  # 20 ms policy period.
RATIO = 25.0  # Rotor rotations per mechanism rotation (see CONFIG_TOML).
CRUISE_RPS = 4.0
KP = 2.0  # Example-only position gain: velocity demand per rotation error.
TOLERANCE_ROT = 0.05


def run_sim(target_rot, start_rot=0.0, tolerance_rot=TOLERANCE_ROT,
            timeout_s=5.0, tuned=True):
    """Drive to ``target_rot`` (mechanism rotations) until close or timeout.

    Each step converts the mechanism-frame error to a rotor-frame check
    (the gearing point), clamps the velocity demand to cruise, and steps
    the plant. Returns a result dict with the position/velocity traces.

    Cancellation: done holds position at the shelf (completion);
    not-done commands stop (timeout -- check gains/current limits,
    never just raise the timeout); interruption propagates and the
    caller still commands stop; disable is handled as a timeout.
    Map with ``behaviors.cancellation_demand``.
    """
    plant = VelocityPlant(tuned=tuned, max_rps=CRUISE_RPS)
    position_rot = float(start_rot)
    positions, velocities, demands = [position_rot], [], []
    over_cruise = [False]

    def _step(dt_s):
        error_rot = target_rot - positions[-1]
        # Gearing point, crossed explicitly: rotor error for logging,
        # mechanism demand for the loop.
        rotor_error = units.mechanism_rotations_to_rotor(error_rot, RATIO)
        _ = units.rotor_to_mechanism_rotations(rotor_error, RATIO)
        raw = KP * error_rot
        demand = units.clamp_demand(raw, CRUISE_RPS)
        if abs(raw) > CRUISE_RPS:
            over_cruise[0] = True
        velocity = plant.step(demand, dt_s)
        positions.append(positions[-1] + velocity * dt_s)
        velocities.append(velocity)
        demands.append(demand)

    def _arrived():
        return abs(positions[-1] - target_rot) <= tolerance_rot

    steps, done = behaviors.run_until_or_timeout(_step, _arrived,
                                                 timeout_s, DT_S)
    return {"target_rot": target_rot, "steps": steps, "done": done,
            "positions": positions, "velocities": velocities,
            "demands": demands, "final_rot": positions[-1],
            "error_rot": abs(positions[-1] - target_rot),
            "cruise_limited": over_cruise[0],
            "elapsed_s": steps * DT_S}


def diagnose(result, tolerance_rot=TOLERANCE_ROT):
    """Diagnostics for a ``run_sim`` result: what happened, in words."""
    lines = ["arrived in %.2f s, final error %.3f rot%s"
             % (result["elapsed_s"], result["error_rot"],
                "" if result["done"] else " (TIMED OUT)")]
    if not result["done"]:
        lines.append("never arrived: check gains/current limits, "
                     "not the timeout")
    if result["cruise_limited"]:
        lines.append("demand hit the cruise clamp: long moves slew at "
                     "max velocity, as designed")
    if result["error_rot"] > tolerance_rot and result["done"]:
        lines.append("at tolerance edge: raise position gain only until "
                     "the stage stops short, then stop")
    return {"lines": lines, "done": result["done"],
            "error_rot": result["error_rot"],
            "elapsed_s": result["elapsed_s"]}
