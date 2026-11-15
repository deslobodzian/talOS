"""talos.parity: one scenario, run against a node's C++ and Python versions.

A node that exists in both languages (the C++ class hosted through
talos.cpp_node, and a Python class) must behave the same (REQ-CONTRACT-14).
assert_same_behaviour runs each on its own talos.testing.Sim with
SimHardware, plays the same scenario on both, and after every step compares
the newest message of each listed topic byte for byte and each listed
output, requested and applied. The first step that differs raises
AssertionError naming the step and both values, decoded; a topic one side
has published and the other has not differs. A step that holds
require_published() also fails when either side has no message on those
topics after it.

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
advances the clock by its `then` seconds afterwards.
"""

import struct

from talos.testing import Sim, SimHardware


class Rig:
    """One language's run: its Sim, its SimHardware and the node's name.
    `generations` holds what remember_generation recorded."""

    def __init__(self, cls, config, node):
        self.sim = Sim(config)
        self.sim.add(cls, node)
        self.hw = SimHardware(self.sim)
        self.node = node
        self.generations = {}


def _then(action, seconds):
    def run(rig):
        action(rig)
        if seconds:
            rig.sim.step(seconds)
    return run


def wait(seconds):
    """Advances the clock."""
    return lambda rig: rig.sim.step(seconds)


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
    """A step marker: after this step each of `topics` (every listed topic
    when none is named) must have a message on both sides."""

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
    """[(what, value)] after one step."""
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


def assert_same_behaviour(cpp_cls, py_cls, config, node, scenario, *,
                          topics=(), motors=(), digital_outputs=(), pwms=()):
    """Runs `scenario` on `cpp_cls` and on `py_cls`, each named `node` on its
    own Sim over `config`; raises AssertionError at the first step after
    which a topic in `topics` [(message class, topic)], or a requested or
    applied output of a device named in `motors`, `digital_outputs` or
    `pwms`, differs, or a topic a require_published marker names has no
    message on a side, and when a topic was never published by either."""
    if not scenario:
        raise ValueError("an empty scenario compares nothing")
    listed = [topic for _, topic in topics]
    rigs = {"C++": Rig(cpp_cls, config, node),
            "Python": Rig(py_cls, config, node)}
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
        observed = {}
        for language, rig in rigs.items():
            for action in actions:
                action(rig)
            observed[language] = _observe(rig, topics, motors,
                                          digital_outputs, pwms)
        for (what, cpp), (_, py) in zip(observed["C++"], observed["Python"]):
            if what in required and (cpp is None or py is None):
                raise AssertionError(
                    f"{step}: {what} required but not published\n"
                    f"  C++:    {cpp!r}\n  Python: {py!r}")
            if cpp is not None or py is not None:
                published.add(what)
            if _key(cpp) != _key(py):
                raise AssertionError(
                    f"{step}: {what} differs\n"
                    f"  C++:    {cpp!r}\n  Python: {py!r}")
    silent = [topic for topic in listed if topic not in published]
    if silent:
        raise AssertionError(f"{node}: {', '.join(silent)} never published "
                             f"in the scenario: nothing was compared")
