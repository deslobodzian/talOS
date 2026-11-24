"""talos.parity: one scenario, run against a node's C++ and Python versions.

A node that exists in both languages (the C++ class hosted through
talos.cpp_node, and a Python class) must behave the same (REQ-CONTRACT-14).
assert_same_behaviour runs each on its own talos.testing.Sim with
SimHardware, plays the same scenario on both, and compares the newest
message of each listed topic byte for byte and each listed output,
requested and applied, at every tick of every step and again at each
step's end. The first difference raises AssertionError naming the step,
the tick within it and its simulated time, what differs, and both values,
decoded; a topic one side has published and the other has not differs. A
step that holds require_published() also fails when either side has no
message on those topics at its end.

A tick is a boundary of the node's period on the Sim clock (a multiple of
period_us from time zero): an action that advances the clock stops at each
boundary it reaches and observes both sides there. The two nodes run under
the same config on Sims that both start at zero, and the same Sim
scheduler runs a C++ and a Python node's update(), so their ticks land on
the same instants: a tick is one simulated instant on both sides, and a
phase difference is a real divergence, not a tolerance. What a node
publishes between two boundaries and overwrites before the next (an
every() timer faster than the period, a burst of callbacks) is seen only
as the newest message at the next boundary.

    SCENARIO = [
        ("disabled", parity.wait(0.05), parity.require_published()),
        ("enabled", parity.gateway(True),
                    parity.driver_station(enabled=True, teleop=True,
                                          then=0.05)),
        ("velocity", parity.send(MotorTarget, "teleop", mode=..., ...)),
        ("released", parity.release(MotorTarget, "teleop")),
    ]
    parity.assert_same_behaviour(
        CppMotor, PyMotor, CONFIG, "motor", SCENARIO,
        topics=[(MotorState, "/motor/state")], motors=["kraken"])

A step is a name and one or more actions; an action is a callable taking
the Rig (sim, hw, node). The functions below build the usual actions; each
advances the clock by its `then` seconds afterwards. An action of your own
advances the clock with rig.advance(seconds), never rig.sim.step, which
would skip the ticks it crosses: a step whose clock moved any other way
fails, naming the step.
"""

import struct

from talos import _abi
from talos.testing import Sim, SimHardware

_NS = 1_000_000_000


class Rig:
    """One language's run: its Sim, its SimHardware and the node's name.
    `generations` holds what remember_generation recorded; `ticks` the
    (time in ns, observation) of each tick advance() reached in the current
    step."""

    def __init__(self, cls, config, node, observe):
        self.sim = Sim(config)
        self.sim.add(cls, node)
        self.hw = SimHardware(self.sim)
        self.node = node
        self.generations = {}
        self.ticks = []
        self._observe = observe
        self._period_ns = _abi.value(
            _abi.lib.talos_node_period_ns(self.sim.node(node)._handle), 0)
        # Where advance() last left the clock.
        self._advanced_to_ns = self.sim.now_ns

    def check_clock(self, where):
        """Raises AssertionError when the clock is not where advance() left
        it: something else stepped the Sim, skipping the ticks it crossed."""
        if self.sim.now_ns != self._advanced_to_ns:
            raise AssertionError(
                f"{where}: the simulated clock moved from "
                f"{self._advanced_to_ns / _NS:.6f} s to "
                f"{self.sim.now_ns / _NS:.6f} s other than through "
                f"Rig.advance, so the ticks it crossed were not compared")

    def advance(self, seconds):
        """Advances the clock by `seconds`, observing at each boundary of
        the node's period it reaches."""
        self.check_clock("Rig.advance")
        delta_ns = _abi.duration_ns(seconds, "seconds")
        if seconds < 0 or delta_ns < 0:
            raise ValueError(
                f"Rig.advance: seconds must be non-negative, got {seconds!r}")
        now = self.sim.now_ns
        if self._period_ns <= 0:
            if delta_ns > 0:
                self.sim._step_ns(delta_ns)
                now = self.sim.now_ns
            self._advanced_to_ns = now
            return
        end = now + delta_ns
        while now < end:
            boundary = (now // self._period_ns + 1) * self._period_ns
            stop = min(boundary, end)
            self.sim._step_ns(stop - now)
            now = self.sim.now_ns
            if now == boundary:
                self.ticks.append((now, self.observe()))
        self._advanced_to_ns = now

    def observe(self):
        """[(what, value)] now."""
        return self._observe(self)


def _then(action, seconds):
    def run(rig):
        action(rig)
        if seconds:
            rig.advance(seconds)
    return run


def wait(seconds):
    """Advances the clock."""
    return lambda rig: rig.advance(seconds)


def gateway(enabled, then=0.0):
    """The gateway enabled or disabled (SimHardware.enable)."""
    return _then(lambda rig: rig.hw.enable(enabled), then)


def driver_station(then=0.0, **packet):
    """SimHardware.set_driver_station(**packet): the Driver Station mode,
    enable, joysticks."""
    return _then(lambda rig: rig.hw.set_driver_station(**packet), then)


def stop_driver_station(then=0.0):
    """The Driver Station link goes silent."""
    return _then(lambda rig: rig.hw.stop_driver_station(), then)


def remember_generation(name):
    """Records the current Driver Station generation under `name`, for a
    later send(..., generation=name): a target queued under that mode."""
    def run(rig):
        rig.generations[name] = rig.sim.driver_station_generation
    return run


def send(msg, source, *, generation=None, then=0.01, **fields):
    """An active target on the node's `source` lane, stamped with the current
    generation, or with the one remember_generation recorded as
    `generation`."""
    def run(rig):
        stamp = None if generation is None else rig.generations[generation]
        rig.sim.publish_target(msg, rig.node, source, mode_since_us=stamp,
                               **fields)
    return _then(run, then)


def release(msg, source, then=0.01):
    """A release on the node's `source` lane."""
    return _then(lambda rig: rig.sim.release_target(msg, rig.node, source),
                 then)


def device(method, name, *args, then=0.0, **kwargs):
    """A SimHardware device input for the node's device `name`:
    device("set_connected", "kraken", False) calls
    hw.set_connected(node, "kraken", False). Covers motor feedback, limit
    switches, sensors, disconnects and resets."""
    return _then(lambda rig: getattr(rig.hw, method)(rig.node, name, *args,
                                                    **kwargs), then)


def lose_states(count, then=0.0):
    """The next `count` hardware states are lost."""
    return _then(lambda rig: rig.hw.lose_states(count), then)


class require_published:
    """A step marker: at the end of this step each of `topics` (every listed
    topic when none is named) must have a message on both sides."""

    def __init__(self, *topics):
        self.topics = topics

    def __call__(self, rig):
        pass


def _key(value):
    """`value` with floats as their bits, so NaN equals NaN and -0.0 does
    not equal 0.0, as in a byte comparison. A message compares its bytes."""
    if isinstance(value, float):
        return struct.pack("<d", value)
    if isinstance(value, tuple):
        return tuple(_key(v) for v in value)
    return value


def _observe(rig, topics, motors, digital_outputs, pwms):
    """[(what, value)] now."""
    sim, hw, node = rig.sim, rig.hw, rig.node
    seen = [(topic, sim.latest(msg, topic)) for msg, topic in topics]
    for name in motors:
        seen.append((f"motor {name} request", hw.motor_request(node, name)))
        seen.append((f"motor {name} applied", hw.applied_motor(node, name)))
    for name in digital_outputs:
        seen.append((f"digital {name} request", hw.digital_output(node, name)))
        seen.append((f"digital {name} applied",
                     hw.applied_digital_output(node, name)))
    for name in pwms:
        seen.append((f"pwm {name} request", hw.pwm(node, name)))
        seen.append((f"pwm {name} applied", hw.applied_pwm(node, name)))
    return seen


def _compare(where, cpp_seen, py_seen, required, published):
    """Raises at the first difference between two observations, or at a
    `required` topic missing on a side; adds what was seen to
    `published`."""
    for (what, cpp), (_, py) in zip(cpp_seen, py_seen):
        if what in required and (cpp is None or py is None):
            raise AssertionError(
                f"{where}: {what} required but not published\n"
                f"  C++:    {cpp!r}\n  Python: {py!r}")
        if cpp is not None or py is not None:
            published.add(what)
        if _key(cpp) != _key(py):
            raise AssertionError(
                f"{where}: {what} differs\n"
                f"  C++:    {cpp!r}\n  Python: {py!r}")


def assert_same_behaviour(cpp_cls, py_cls, config, node, scenario, *,
                          topics=(), motors=(), digital_outputs=(), pwms=()):
    """Runs `scenario` on `cpp_cls` and on `py_cls`, each named `node` on its
    own Sim over `config`; raises AssertionError at the first tick, or step
    end, at which a topic in `topics` [(message class, topic)], or a
    requested or applied output of a device named in `motors`,
    `digital_outputs` or `pwms`, differs; at the end of a step whose
    require_published marker names a topic one side has no message on; and
    when a topic was never published by either."""
    if not scenario:
        raise ValueError("an empty scenario compares nothing")
    listed = [topic for _, topic in topics]

    def observe(rig):
        return _observe(rig, topics, motors, digital_outputs, pwms)

    rigs = {"C++": Rig(cpp_cls, config, node, observe),
            "Python": Rig(py_cls, config, node, observe)}
    published = set()
    for index, (name, *actions) in enumerate(scenario, 1):
        step = f"{node}: step {index} {name!r}"
        required = set()
        for action in actions:
            if isinstance(action, require_published):
                unknown = set(action.topics) - set(listed)
                if unknown:
                    raise ValueError(f"{step}: require_published names "
                                     f"{', '.join(sorted(unknown))}, not in "
                                     f"topics")
                required.update(action.topics or listed)
        ended = {}
        for language, rig in rigs.items():
            rig.ticks = []
            for action in actions:
                try:
                    action(rig)
                    rig.check_clock("after the action")
                except AssertionError as error:
                    raise AssertionError(f"{step} ({language}): {error}") \
                        from None
            ended[language] = rig.observe()
        cpp_ticks, py_ticks = rigs["C++"].ticks, rigs["Python"].ticks
        # The same actions advance both clocks alike, so both sides reach
        # the same boundaries.
        assert [at for at, _ in cpp_ticks] == [at for at, _ in py_ticks]
        for tick, ((at, cpp), (_, py)) in enumerate(
                zip(cpp_ticks, py_ticks), 1):
            _compare(f"{step}, tick {tick} (t = {at / _NS:.6f} s)", cpp, py,
                     (), published)
        _compare(f"{step}, end", ended["C++"], ended["Python"], required,
                 published)
    silent = [topic for topic in listed if topic not in published]
    if silent:
        raise AssertionError(f"{node}: {', '.join(silent)} never published "
                             f"in the scenario: nothing was compared")
