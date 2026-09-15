"""Intake student policy: the behavior knob students actually turn.

``command_for`` maps the arbiter's roller-velocity ask to the wheel's
demand -- positive asks run at ``INTAKE_RPS``, negative asks at
``EJECT_RPS``, zero holds at ``STOP_RPS`` -- and holds ``STOP_RPS``
against further *intake* while the beam-break sensor sees a captured
game piece. Eject still works with the beam blocked: a captured piece
is removed by running the wheel backwards, so the beam stop applies to
the intake direction only (the ask's sign is checked first).

The presets live here and only here: ``node.py`` passes no overrides,
so editing ``INTAKE_RPS`` / ``EJECT_RPS`` / ``STOP_RPS`` below changes
what the wheel does. All speeds are mechanism rev/s (wheel rev/s, not
motor-rotor rev/s -- see ``units.py`` for the rotor/mechanism split).

Stdlib only (``math``, ``dataclasses``): import-light like
``packet.py``, so probes never pay for hardware imports.
"""

import math
from dataclasses import dataclass

# Wheel-speed presets, mechanism rev/s. Sign is direction: positive pulls
# a game piece in, negative spits it back out, zero holds still. This
# module is the single source of truth -- node.py reads these through
# command_for's defaults and defines no copies.
INTAKE_RPS = 30.0
EJECT_RPS = -20.0
STOP_RPS = 0.0


@dataclass(frozen=True)
class Target:
    """The arbiter's roller-velocity ask, in mechanism rev/s.

    Positive means "pull a piece in", negative means "push it out",
    zero means "hold still". Non-finite values are allowed here and
    pass through ``command_for`` untouched so the node's
    ``INHIBIT_DEMAND_NONFINITE`` gate still coasts the wheel instead of
    commanding it.

    Typed shape: ``velocity_rps`` must be a real number (int/float,
    never bool/str/None); it is coerced to float. Transport types
    (dicts, tuples, payload bytes) are rejected with ``TypeError``.
    """
    velocity_rps: float

    def __post_init__(self):
        vel = self.velocity_rps
        if isinstance(vel, bool) or not isinstance(vel, (int, float)):
            raise TypeError("Target.velocity_rps must be a number "
                            "(mechanism rev/s), got %r" % (vel,))
        object.__setattr__(self, "velocity_rps", float(vel))


def _checked_flag(value, name):
    """Validate one boolean feedback flag. Returns the bool value."""
    if isinstance(value, bool):
        return value
    if isinstance(value, int) and value in (0, 1):
        return bool(value)
    raise TypeError("%s must be a bool, got %r" % (name, value))


def _checked_number(value, name):
    """Validate one numeric feedback reading. Returns the float value.

    Non-finite readings are allowed here (the ``*_valid`` flag, not
    finiteness, says whether the sample is trustworthy); non-numbers
    (bool/str/None/dicts) raise ``TypeError``.
    """
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError("%s must be a number, got %r" % (name, value))
    return float(value)


@dataclass(frozen=True)
class Feedback:
    """One tick's worth of typed sensor feedback -- no transport exposure.

    Fields (units in names; mechanism frame unless noted):

    * ``beam_broken`` -- True while the beam-break sensor sees a
      captured game piece; ``beam_valid`` -- whether that sample is
      trustworthy (mirrors the decoded ``/hw/state`` words);
    * ``measured_velocity_rps`` -- wheel speed in mechanism rev/s with
      ``velocity_valid`` saying whether the roller sample was present,
      finite, and fresh;
    * ``measured_position_rot`` -- wheel position in mechanism
      rotations with ``position_valid`` (False on mechanisms like this
      roller that servo velocity only and publish no position);
    * ``enabled`` -- the gateway reports configured + enabled this
      tick (robot enabled, not the arbiter's target-enable bit);
    * ``fault`` -- a hardware fault is latched this tick.

    The demand mapping in ``command_for`` reads ``beam_broken`` as
    given; validity/enabled/fault gating lives in the node (its inhibit
    table), so richer feedback never silently changes the demand law.
    Construct only from decoded values -- never from ``/hw/state``
    dicts, payload bytes, or other transport types (rejected with
    ``TypeError``).
    """
    beam_broken: bool = False
    beam_valid: bool = True
    measured_velocity_rps: float = 0.0
    velocity_valid: bool = True
    measured_position_rot: float = 0.0
    position_valid: bool = False
    enabled: bool = True
    fault: bool = False

    def __post_init__(self):
        object.__setattr__(self, "beam_broken",
                           _checked_flag(self.beam_broken, "beam_broken"))
        object.__setattr__(self, "beam_valid",
                           _checked_flag(self.beam_valid, "beam_valid"))
        object.__setattr__(self, "measured_velocity_rps",
                           _checked_number(self.measured_velocity_rps,
                                           "measured_velocity_rps"))
        object.__setattr__(self, "velocity_valid",
                           _checked_flag(self.velocity_valid,
                                         "velocity_valid"))
        object.__setattr__(self, "measured_position_rot",
                           _checked_number(self.measured_position_rot,
                                           "measured_position_rot"))
        object.__setattr__(self, "position_valid",
                           _checked_flag(self.position_valid,
                                         "position_valid"))
        object.__setattr__(self, "enabled",
                           _checked_flag(self.enabled, "enabled"))
        object.__setattr__(self, "fault",
                           _checked_flag(self.fault, "fault"))

    @property
    def drive_ok(self):
        """Advisory gate: enabled, no fault, and the velocity sample valid.

        For tests/telemetry only -- the node owns the real inhibit
        decision; ``command_for`` intentionally ignores this.
        """
        return bool(self.enabled and not self.fault and self.velocity_valid)


def command_for(target, feedback=None, *, intake_rps=None, eject_rps=None,
                stop_rps=None):
    """Map the arbiter's demand to the wheel's demand, in mechanism rev/s.

    ``target`` is a :class:`Target` (ints coerce to float; non-finite
    passes through); ``feedback`` is a :class:`Feedback` (defaults to
    "beam clear", valid, enabled, no fault). Only ``beam_broken``
    steers the demand -- the richer validity/enabled/fault fields are
    observed by the node, never by this mapping. The ``*_rps``
    overrides default to this module's presets; every in-tree caller
    uses the defaults.

    A broken beam holds ``STOP_RPS`` against further intake only: a
    positive ask still returns ``stop_rps`` (the piece is captured, so
    the wheel stops pulling rather than grinding on it), while a
    negative ask returns ``eject_rps`` -- intentional ejection with the
    beam blocked is how the piece gets out. A non-finite ask passes
    through untouched so the node's ``INHIBIT_DEMAND_NONFINITE`` gate
    still coasts the wheel instead of commanding it.
    """
    if intake_rps is None:
        intake_rps = INTAKE_RPS
    if eject_rps is None:
        eject_rps = EJECT_RPS
    if stop_rps is None:
        stop_rps = STOP_RPS
    if not isinstance(target, Target):
        raise TypeError("target must be a Target, got %r" % (target,))
    if feedback is None:
        feedback = Feedback()
    if not isinstance(feedback, Feedback):
        raise TypeError("feedback must be a Feedback, got %r" % (feedback,))
    vel = target.velocity_rps
    if not math.isfinite(vel):
        return vel
    if vel > 0.0:
        if feedback.beam_broken:
            return stop_rps
        return intake_rps
    if vel < 0.0:
        return eject_rps
    return stop_rps
