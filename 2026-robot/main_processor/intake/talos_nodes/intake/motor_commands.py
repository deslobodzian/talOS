"""Small motor-facing command API for students (stdlib + packet only).

One function per thing a student wants a motor to do, addressed by the
device *name* from ``subsystem.toml`` (``"roller"``) rather than a
numeric ID. Each function validates its demand, stamps the accepted
state's header words, and returns the 1404-byte ``/hw/request`` payload
ready to send:

* ``duty_cycle`` -- open-loop fraction of bus voltage, -1..1;
* ``voltage`` -- open-loop volts at the leads, within +/- max_voltage;
* ``velocity`` -- closed-loop mechanism rev/s on the Talon's onboard
  PID (``slot`` selects the gain set from ``subsystem.toml``);
* ``position`` -- closed-loop mechanism rotations on the Talon's
  onboard PID;
* ``stop`` -- neutral (coast/brake follows the TOML ``brake`` flag).

Closed loop is the normal path: ``velocity``/``position`` ask the Talon
to close the loop onboard at 1 kHz using the ``slotN`` gains from the
config -- the companion only sends the demand, never the PWM. Open loop
(``duty_cycle``/``voltage``) is for characterization and bring-up. With
all gains zero the loop is untuned and the wheel lags its demand (see
``control_config``); that shows up as requested-vs-measured split in
``/intake/state``, never as an error.

``ids`` maps device names to global logical IDs (see
``node.resolve_ids``); ``state`` is the accepted-state dict from
``packet.decode_hw_state_minimal`` (only its header words are read).
"""

import math

from talos_nodes.intake import packet
from talos_nodes.intake import units


def _encode(state, motor_id, mode, slot, demand, feedforward_v=0.0):
    return packet.encode_command(state["config_id"], state["boot_id"],
                                 state["epoch"], state["sample_time_us"],
                                 motor_id, mode, slot, float(demand),
                                 float(feedforward_v))


def _motor_id(ids, name):
    try:
        motor_id = ids[name]
    except (KeyError, TypeError):
        raise KeyError("unknown motor name %r" % (name,))
    return motor_id


def duty_cycle(state, ids, name, duty):
    """Open-loop duty demand (-1..1) for the named motor."""
    return _encode(state, _motor_id(ids, name), packet.MODE_DUTY_CYCLE, 0,
                   units.check_duty(duty))


def voltage(state, ids, name, volts, max_voltage=12.0):
    """Open-loop voltage demand (volts) for the named motor."""
    return _encode(state, _motor_id(ids, name), packet.MODE_VOLTAGE, 0,
                   units.check_volts(volts, max_voltage))


def velocity(state, ids, name, velocity_rps, slot=0, feedforward_v=0.0,
             max_velocity_rps=None):
    """Closed-loop velocity demand (mechanism rev/s) on the onboard PID.

    ``velocity_rps`` is validated at this boundary (finite required) and,
    when ``max_velocity_rps`` (mechanism rev/s, e.g. from
    ``MotorControlConfig``) is given, clamped to +/- it with its sign
    kept (see ``units.clamp_demand``). ``slot`` selects the gain set
    from ``subsystem.toml``; the Talon closes the loop onboard, so this
    demand stays in the mechanism frame -- ``units`` crosses to rotor
    only for sensor math, never on this wire.
    """
    demand = units.check_rps(velocity_rps, "velocity_rps")
    if max_velocity_rps is not None:
        demand = units.clamp_demand(demand, max_velocity_rps)
    ff = float(feedforward_v)
    if not math.isfinite(ff):
        raise ValueError("feedforward_v must be finite, got %r"
                         % (feedforward_v,))
    return _encode(state, _motor_id(ids, name), packet.MODE_VELOCITY, slot,
                   demand, ff)


def position(state, ids, name, position_rot, slot=0, feedforward_v=0.0):
    """Closed-loop position demand (mechanism rotations) on the onboard PID.

    ``position_rot`` is validated at this boundary via
    ``units.check_rotations`` (finite mechanism rotations required).
    Same framing note as ``velocity``: the demand stays in mechanism
    rotations on the wire.
    """
    demand = units.check_rotations(position_rot, "position_rot")
    ff = float(feedforward_v)
    if not math.isfinite(ff):
        raise ValueError("feedforward_v must be finite, got %r"
                         % (feedforward_v,))
    return _encode(state, _motor_id(ids, name), packet.MODE_POSITION, slot,
                   demand, ff)


def stop(state, ids, name):
    """Neutral command: say stop rather than go silent."""
    return _encode(state, _motor_id(ids, name), packet.MODE_NEUTRAL, 0, 0.0)
