"""Explicit units for student motor code (stdlib only).

Two frames show up everywhere a wheel is driven, and mixing them up is
the classic first-hardware bug:

* mechanism frame -- what the game piece feels: wheel rev/s (``rps``),
  wheel rotations, radians at the wheel;
* rotor frame -- what the Talon sees: motor-rotor rev/s and rotations.

``sensor_to_mechanism_ratio`` (one number per motor in
``subsystem.toml``) converts between them: it counts rotor rotations
per one mechanism rotation (1.0 = direct drive), so::

    rotor = mechanism * ratio
    mechanism = rotor / ratio

Other quantities and their frames:

* radians -- mechanism radians at the wheel unless the name says
  ``rotor_``; ``rps_to_rad_per_s`` / ``rad_per_s_to_rps`` stay in the
  mechanism frame, while the ``rotor_*_rad`` helpers stay in the rotor
  frame so the two never mix silently;
* volts -- always volts at the motor leads (0..``max_voltage``, 12
  nominal); see ``check_volts``;
* amps -- always amps through the motor (supply or stator, per the
  ``*_limit_a`` name in ``subsystem.toml``); see ``check_amps``.

Every converter below validates its inputs at the boundary: non-finite
or non-positive ratios raise, non-finite speeds/positions raise,
out-of-range volts/duty/amps raise, and clamps keep the sign of the
demand. Callers convert at the boundary where the frame changes
(demand out, feedback in) -- never mid-computation.
"""

import math

TWO_PI = 2.0 * math.pi


def check_ratio(ratio):
    """Validate a sensor_to_mechanism_ratio. Returns the float ratio."""
    ratio = float(ratio)
    if not math.isfinite(ratio) or ratio <= 0.0:
        raise ValueError("sensor_to_mechanism_ratio must be finite and > 0, "
                         "got %r" % (ratio,))
    return ratio


def check_rps(value, name="velocity_rps"):
    """Validate a rev/s demand. Returns the float value."""
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("%s must be finite, got %r" % (name, value))
    return value


def mechanism_rps_to_rotor_rps(mechanism_rps, ratio):
    """Wheel rev/s -> motor-rotor rev/s (what the Talon PID loop sees)."""
    return check_rps(mechanism_rps) * check_ratio(ratio)


def rotor_rps_to_mechanism_rps(rotor_rps, ratio):
    """Motor-rotor rev/s -> wheel rev/s."""
    return check_rps(rotor_rps, "rotor_rps") / check_ratio(ratio)


def mechanism_rotations_to_rotor(mechanism_rotations, ratio):
    """Wheel rotations -> rotor rotations (position demands)."""
    value = float(mechanism_rotations)
    if not math.isfinite(value):
        raise ValueError("mechanism_rotations must be finite, got %r"
                         % (mechanism_rotations,))
    return value * check_ratio(ratio)


def rotor_to_mechanism_rotations(rotor_rotations, ratio):
    """Rotor rotations -> wheel rotations (position feedback)."""
    value = float(rotor_rotations)
    if not math.isfinite(value):
        raise ValueError("rotor_rotations must be finite, got %r"
                         % (rotor_rotations,))
    return value / check_ratio(ratio)


def rps_to_rad_per_s(rps):
    """Mechanism rev/s -> mechanism rad/s."""
    return check_rps(rps) * TWO_PI


def rad_per_s_to_rps(rad_per_s):
    """Mechanism rad/s -> mechanism rev/s."""
    value = float(rad_per_s)
    if not math.isfinite(value):
        raise ValueError("rad_per_s must be finite, got %r" % (rad_per_s,))
    return value / TWO_PI


def rotor_rps_to_rotor_rad_per_s(rotor_rps):
    """Rotor rev/s -> rotor rad/s (rotor frame; same factor, explicit name)."""
    return check_rps(rotor_rps, "rotor_rps") * TWO_PI


def rotor_rad_per_s_to_rotor_rps(rotor_rad_per_s):
    """Rotor rad/s -> rotor rev/s (rotor frame; same factor, explicit name)."""
    value = float(rotor_rad_per_s)
    if not math.isfinite(value):
        raise ValueError("rotor_rad_per_s must be finite, got %r"
                         % (rotor_rad_per_s,))
    return value / TWO_PI


def mechanism_rad_per_s_to_rotor_rps(mechanism_rad_per_s, ratio):
    """Mechanism rad/s -> rotor rev/s: frame change at the demand boundary."""
    return (rad_per_s_to_rps(mechanism_rad_per_s)
            * check_ratio(ratio))


def rotor_rps_to_mechanism_rad_per_s(rotor_rps, ratio):
    """Rotor rev/s -> mechanism rad/s: frame change at the feedback boundary."""
    return rps_to_rad_per_s(rotor_rps_to_mechanism_rps(rotor_rps, ratio))


def check_rotations(value, name="position_rot"):
    """Validate a rotations demand/feedback. Returns the float value."""
    if isinstance(value, bool):
        raise ValueError("%s must be a number of rotations, got %r"
                         % (name, value))
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("%s must be finite, got %r" % (name, value))
    return result


def mechanism_rotations_to_radians(mechanism_rotations):
    """Mechanism rotations -> mechanism radians (position feedback)."""
    return check_rotations(mechanism_rotations,
                           "mechanism_rotations") * TWO_PI


def radians_to_mechanism_rotations(radians):
    """Mechanism radians -> mechanism rotations (position demands)."""
    value = check_rotations(radians, "radians")
    return value / TWO_PI


def check_amps(amps, limit_a, name="current_a"):
    """Validate an amps reading/demand against a ``*_limit_a`` ceiling.

    ``limit_a`` is the matching ``supply_limit_a`` / ``stator_limit_a``
    value from ``subsystem.toml`` (amps, finite and > 0). Returns the
    float amps; raises when non-finite or beyond +/- the limit.
    """
    if isinstance(amps, bool):
        raise ValueError("%s must be a number of amps, got %r"
                         % (name, amps))
    value = float(amps)
    ceiling = float(limit_a)
    if not math.isfinite(ceiling) or ceiling <= 0.0:
        raise ValueError("limit_a must be finite and > 0, got %r"
                         % (limit_a,))
    if not math.isfinite(value) or abs(value) > ceiling:
        raise ValueError("%s must be finite within +/- %r, got %r"
                         % (name, ceiling, amps))
    return value


def clamp_demand(demand_rps, max_velocity_rps):
    """Clamp a mechanism rev/s demand to +/- max, keeping its sign."""
    demand = check_rps(demand_rps, "demand_rps")
    limit = float(max_velocity_rps)
    if not math.isfinite(limit) or limit < 0.0:
        raise ValueError("max_velocity_rps must be finite and >= 0, got %r"
                         % (max_velocity_rps,))
    if abs(demand) <= limit:
        return demand
    return math.copysign(limit, demand)


def check_volts(volts, max_voltage=12.0):
    """Validate a voltage demand against +/- max_voltage."""
    value = float(volts)
    ceiling = float(max_voltage)
    if not math.isfinite(ceiling) or ceiling <= 0.0:
        raise ValueError("max_voltage must be finite and > 0, got %r"
                         % (max_voltage,))
    if not math.isfinite(value) or abs(value) > ceiling:
        raise ValueError("volts must be finite within +/- %r, got %r"
                         % (ceiling, volts))
    return value


def check_duty(duty):
    """Validate a duty-cycle demand (-1..1, fraction of bus voltage)."""
    value = float(duty)
    if not math.isfinite(value) or abs(value) > 1.0:
        raise ValueError("duty must be finite within [-1, 1], got %r"
                         % (duty,))
    return value
