"""Reusable behavior helpers for robot code (stdlib only).

Small on purpose: only helpers that more than one robot behavior needs
live here. Each one below names its repeated users:

* ``Debounce`` -- the chattering beam-break switch; shared by the
  intake policy (``node.py``) and the roller-intake example;
* ``EdgeDetector`` -- the moment a piece arrives (capture counting);
  shared by the intake policy and the roller-intake example;
* ``SlewLimiter`` -- ramping step velocity demands (preset snaps like
  0 -> 30 rps in the node and in every teaching example) so the Talon
  and the bus never eat the full step at once;
* ``ElapsedTimer`` / ``run_until_or_timeout`` -- "hold this demand
  until something happens or time runs out"; shared by the flywheel
  (spin-up) and position (drive-to-shelf) examples.

One-off logic stays in the behavior file that needs it.

All helpers are pure step functions (no clock, no I/O): the caller
passes each tick's reading or elapsed ``dt`` so sim and hardware share
the code path and tests drive them deterministically.

Cancellation contract: every timed/conditional action built on these
helpers must state its motor output for each terminal condition --
completion, timeout, interruption, and disable. ``run_until_or_timeout``
returns ``(steps, done)`` and the caller maps it with
``cancellation_demand`` (done -> goal demand; anything else -> the
safe demand, usually stop); an exception inside ``step_fn`` propagates
and the caller must still command the safe demand. The intake node's
own table lives on ``IntakePolicy.tick``.
"""

import math


class Debounce:
    """Stable-after-N signal: output flips only after N matching reads.

    ``stable_ticks=1`` follows the input immediately (the default, so a
    lone policy tick behaves like the raw switch); raise to 2-4 on a
    chattery rig. ``update`` returns the current stable value.
    """

    def __init__(self, initial=False, stable_ticks=1):
        if int(stable_ticks) < 1:
            raise ValueError("stable_ticks must be >= 1, got %r"
                             % (stable_ticks,))
        self.stable = bool(initial)
        self.stable_ticks = int(stable_ticks)
        self._pending = bool(initial)
        self._count = 0

    def update(self, reading):
        """Fold one tick's reading. Returns the stable value."""
        reading = bool(reading)
        if reading == self.stable:
            self._pending = reading
            self._count = 0
            return self.stable
        if reading != self._pending:
            self._pending = reading
            self._count = 1
        else:
            self._count += 1
        if self._count >= self.stable_ticks:
            self.stable = self._pending
            self._count = 0
        return self.stable


class EdgeDetector:
    """Rising/falling edge counter over a boolean signal."""

    def __init__(self):
        self.previous = False
        self.rising = 0
        self.falling = 0

    def update(self, value):
        """Fold one tick. Returns (rose, fell) for this tick."""
        value = bool(value)
        rose = value and not self.previous
        fell = self.previous and not value
        self.previous = value
        self.rising += 1 if rose else 0
        self.falling += 1 if fell else 0
        return (rose, fell)


class ElapsedTimer:
    """Elapsed-time accumulator: ``update(dt)`` returns True past timeout."""

    def __init__(self, timeout_s):
        timeout = float(timeout_s)
        if not math.isfinite(timeout) or timeout < 0.0:
            raise ValueError("timeout_s must be finite and >= 0, got %r"
                             % (timeout_s,))
        self.timeout_s = timeout
        self.elapsed_s = 0.0

    @property
    def expired(self):
        """True once elapsed time reaches the timeout."""
        return self.elapsed_s >= self.timeout_s

    def update(self, dt_s):
        """Advance by ``dt_s`` seconds. Returns True when expired."""
        step = float(dt_s)
        if not math.isfinite(step) or step < 0.0:
            raise ValueError("dt_s must be finite and >= 0, got %r"
                             % (dt_s,))
        self.elapsed_s += step
        return self.expired


class SlewLimiter:
    """Symmetric rate limiter for velocity demands (mechanism rev/s).

    Step demands (0 -> intake preset, cruise snaps in the examples)
    slam the Talon and brown out the bus; this moves ``output_rps``
    toward each tick's goal by at most ``max_rate_rps2 * dt_s`` so the
    ramp is explicit and shared instead of re-spelled per behavior.
    ``update`` returns the new output demand for this tick.
    """

    def __init__(self, max_rate_rps2, initial_rps=0.0):
        rate = float(max_rate_rps2)
        if not math.isfinite(rate) or rate <= 0.0:
            raise ValueError("max_rate_rps2 must be finite and > 0, got %r"
                             % (max_rate_rps2,))
        start = float(initial_rps)
        if not math.isfinite(start):
            raise ValueError("initial_rps must be finite, got %r"
                             % (initial_rps,))
        self.max_rate_rps2 = rate
        self.output_rps = start

    def reset(self, value_rps):
        """Snap the output to ``value_rps`` (finite required)."""
        value = float(value_rps)
        if not math.isfinite(value):
            raise ValueError("value_rps must be finite, got %r"
                             % (value_rps,))
        self.output_rps = value
        return self.output_rps

    def update(self, goal_rps, dt_s):
        """Move toward ``goal_rps`` by at most rate*dt. Returns output."""
        goal = float(goal_rps)
        if not math.isfinite(goal):
            raise ValueError("goal_rps must be finite, got %r"
                             % (goal_rps,))
        step = float(dt_s)
        if not math.isfinite(step) or step < 0.0:
            raise ValueError("dt_s must be finite and >= 0, got %r"
                             % (dt_s,))
        limit = self.max_rate_rps2 * step
        error = goal - self.output_rps
        if abs(error) <= limit:
            self.output_rps = goal
        else:
            self.output_rps += limit if error > 0.0 else -limit
        return self.output_rps


def cancellation_demand(*, done, goal_rps, safe_rps=0.0):
    """Map a timed action's outcome to the motor demand to command next.

    Cancellation table (every timed/conditional action states these):

    * completion (``done`` True) -> ``goal_rps`` (hold the achieved
      demand: cruise speed, shelf hold, captured-piece stop);
    * timeout (``done`` False) -> ``safe_rps`` (default stop: the
      sequence never waits forever and never keeps driving blind);
    * interruption (``step_fn`` raised) -> ``safe_rps`` (the exception
      propagates out of ``run_until_or_timeout``; the caller commands
      safe in its cleanup);
    * disable (robot/target disabled mid-action) -> ``safe_rps``
      (treat exactly like a timeout: stop, then re-plan on enable).

    Both demands are mechanism rev/s and must be finite. Returns the
    float demand to command.
    """
    if not isinstance(done, bool):
        raise TypeError("done must be a bool, got %r" % (done,))
    for name, value in (("goal_rps", goal_rps), ("safe_rps", safe_rps)):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise TypeError("%s must be a number (mechanism rev/s), got %r"
                            % (name, value))
        if not math.isfinite(float(value)):
            raise ValueError("%s must be finite, got %r" % (name, value))
    return float(goal_rps) if done else float(safe_rps)


def run_until_or_timeout(step_fn, done_fn, timeout_s, dt_s,
                         max_steps=100000):
    """Run ``step_fn`` each ``dt_s`` until ``done_fn`` or the timeout.

    Returns ``(steps, done)``: how many steps ran and whether ``done_fn``
    (checked once before the first step and again after each step) was
    satisfied -- True means the goal was met, False means the timeout
    won. Every teaching example's simulation runs on this: the same loop
    spins up the flywheel and drives to the shelf position.

    Cancellation: map the result with ``cancellation_demand`` -- done
    holds the goal demand, timeout commands the safe (stop) demand, an
    exception inside ``step_fn`` propagates (caller still commands
    safe), and disable is handled as a timeout (stop, then re-plan).
    """
    timer = ElapsedTimer(timeout_s)
    steps = 0
    if done_fn():
        return (steps, True)
    while steps < max_steps and not timer.expired:
        step_fn(dt_s)
        steps += 1
        if done_fn():
            return (steps, True)
        timer.update(dt_s)
    return (steps, done_fn())
