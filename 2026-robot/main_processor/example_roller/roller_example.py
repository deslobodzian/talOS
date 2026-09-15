"""Example: voltage-controlled roller (open loop bring-up).

The first thing a wheel ever does: volts in, spin out, no PID involved.
The demand path is one validated call -- ``units.check_volts`` at the
boundary, then ``motor_commands.voltage`` puts MODE_VOLTAGE on the wire
with the accepted state's header words -- so students see the whole
open-loop story (config -> validated volts -> command bytes -> motion)
before any loop is closed.

Config (mirrors ``subsystem.toml`` in this directory; UNTUNED on
purpose -- open loop uses no PID gains, so every gain stays zero and
that is correct, not a tune signal)::

    [motors.roller]
    type = "TalonFX"
    bus = "rio"
    can_id = 10
    brake = true
    sensor_to_mechanism_ratio = 1.0
    max_velocity_rps = 100.0
    slot0 = { p = 0.0, i = 0.0, d = 0.0, s = 0.0, v = 0.0, a = 0.0, g = 0.0 }

Simulation: ``run_sim`` ramps the volts-derived demand through a
``behaviors.SlewLimiter`` (a 0 -> 12 V snap browns out the bus) and
steps the sim plant, which models the motor converting volts to motion
as a first-order servo toward ``volts / max_voltage * max_rps``.
Diagnostics: ``diagnose`` reports requested vs measured and names the
wiring/limit fault when the wheel does not follow the volts.

Cancellation: the sim holds ``volts`` for ``duration_s``, then the
window ends and the next command must be stop (0 V). Map any early end
the same way -- timeout, interruption, or disable all command 0 V via
``behaviors.cancellation_demand`` (done -> hold is meaningless for a
timed open-loop push; the safe demand is always stop).

Commissioning order lives in ``NOTES.md`` in this directory.
"""

from talos_nodes.intake import behaviors, motor_commands, units
from talos_nodes.intake.examples.plant import VelocityPlant

MOTOR_NAME = "roller"
MAX_VOLTAGE = 12.0
MAX_RPS = 100.0
RATIO = 1.0  # Direct drive: rotor and mechanism frames agree.
DT_S = 0.02  # 20 ms policy period, like subsystem.toml period_us.
RAMP_RPS2 = 150.0  # Slew rate on the volts-derived rps demand.

CONFIG_TOML = """\
[subsystem]
name = "roller_bringup"
period_us = 20000

[motors.roller]
type = "TalonFX"
bus = "rio"
can_id = 10
inverted = false
brake = true
supply_limit_a = 40.0
stator_limit_a = 80.0
max_voltage = 12.0
sensor_to_mechanism_ratio = 1.0
max_velocity_rps = 100.0
slot0 = { p = 0.0, i = 0.0, d = 0.0, s = 0.0, v = 0.0, a = 0.0, g = 0.0 }
"""


def volts_to_rps(volts, max_voltage=MAX_VOLTAGE, max_rps=MAX_RPS):
    """Open-loop map: volts at the leads -> expected mechanism rev/s.

    Linear in the sim (real motors add deadband and load droop -- that
    split is what current-limit and feedforward tuning fix later).
    Validated at this boundary: over-voltage and non-finite raise.
    """
    return units.check_volts(volts, max_voltage) / max_voltage * max_rps


def encode_voltage(state, ids, volts, name=MOTOR_NAME):
    """Encode one open-loop voltage command for the named motor.

    ``state`` is the accepted-state dict from
    ``packet.decode_hw_state_minimal`` (header words ride along);
    ``ids`` maps device names to global logical IDs (see
    ``node.resolve_ids``). Returns the 1404-byte ``/hw/request`` payload.
    """
    return motor_commands.voltage(state, ids, name, volts,
                                  max_voltage=MAX_VOLTAGE)


def run_sim(volts, duration_s=1.0):
    """Push ``volts`` for ``duration_s``, then stop. Returns a result dict.

    Each tick slews the volts-derived rps demand (no step snaps on the
    bus) and steps the plant; after the window the command returns to
    0 V (timeout -> safe demand, never a held push). ``volts`` is
    validated up front, so an over-voltage ask fails here, not on the
    rig.
    """
    demand_rps = volts_to_rps(volts)  # Raises on over/non-finite volts.
    ramp = behaviors.SlewLimiter(RAMP_RPS2)
    plant = VelocityPlant(tuned=True, max_rps=MAX_RPS)
    demands, velocities = [], []
    steps = int(duration_s / DT_S)
    for _ in range(steps):
        demand = ramp.update(demand_rps, DT_S)
        demands.append(demand)
        velocities.append(plant.step(demand, DT_S))
    stop = behaviors.cancellation_demand(done=False, goal_rps=demand_rps,
                                         safe_rps=0.0)
    return {"volts": volts, "expected_rps": demand_rps, "demands": demands,
            "velocities": velocities, "final_rps": plant.velocity_rps,
            "stop_rps": stop, "elapsed_s": steps * DT_S}


def diagnose(result):
    """Diagnostics for a ``run_sim`` result: what happened, in words."""
    velocities = result["velocities"]
    requested = max([abs(d) for d in result["demands"]] + [0.0])
    measured = max([abs(v) for v in velocities] + [0.0])
    lines = ["%.1f V asked ~%.1f rps, measured peak %.1f rps"
             % (result["volts"], result["expected_rps"], measured)]
    if requested > 0.0 and measured < 0.5 * requested:
        lines.append("wheel does not follow the volts: check wiring, "
                     "current limits (supply_limit_a/stator_limit_a), "
                     "and that inverted matches the mechanism -- not gains "
                     "(open loop has none)")
    if abs(result["volts"]) < 1e-9:
        lines.append("0 V hold: wheel coasts/brakes to rest per the brake "
                     "flag (brake=true holds, false coasts)")
    return {"lines": lines, "requested_peak": requested,
            "measured_peak": measured, "final_rps": result["final_rps"]}
