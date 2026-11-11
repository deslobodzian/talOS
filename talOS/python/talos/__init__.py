"""talOS nodes in Python.

    import talos
    from elevator.ElevatorGoal import ElevatorGoal      # flatc structs
    from elevator.ElevatorStatus import ElevatorStatus

    class Elevator:
        def __init__(self, node):
            self.lift = talos.Motor(node, "lift")
            self.goal = node.subscribe(ElevatorGoal, "/elevator/goal")
            self.status = node.publish(ElevatorStatus, "/elevator/status")

        def update(self):  # every node.period seconds
            if self.goal.fresh():  # neutral until the lift is rezeroed
                self.lift.set_motion_magic(self.goal.value.height_rot)
            self.status.send(height_rot=self.lift.position)

The class is built with the node and registers everything in __init__;
update(), when present, runs every period. This is a thin binding over the
C++ talos::Node (talOS/node), so both languages behave the same. Times are
seconds, except where a name ends in _ns (integer nanoseconds, the unit
message fields such as sample_time_ns carry).
"""

import ctypes
import enum
import gc
import importlib.util
import math
import os
import sys

from talos import _abi
from talos._abi import TalosError, lib
from talos._messages import Message, enum_names, message_type
from talos.devices import (Alliance, AnalogInput, CANcoder, DigitalInput,
                           DigitalOutput, DriverStation, Encoder, Gyro, HardwareLink,
                           Joystick, Mode, Motor, OperatorArming, PwmOutput,
                           Target, TargetLane, _OnNode,
                           hardware_sample_time_ns, robot_enabled)

__all__ = [
    "Alliance", "AnalogInput", "BytePublisher", "CANcoder", "DigitalInput",
    "DigitalOutput", "DriverStation", "Encoder", "Gyro", "HardwareLink", "Joystick", "Message",
    "Mode", "Motor", "Node", "OperatorArming", "Publisher", "PwmOutput",
    "StateMachine",
    "Subscription", "TalosError", "Target", "TargetLane", "cpp_node", "load",
    "hardware_sample_time_ns", "robot_enabled", "run",
]

_NS = 1e9
_REQUIRED = object()
_PARAM_KINDS = {bool: _abi.PARAM_BOOL, int: _abi.PARAM_INT,
                float: _abi.PARAM_DOUBLE, str: _abi.PARAM_STRING}


def _param_kind(key, default):
    """The talos_node_param kind a default asks for."""
    if not isinstance(default, list):
        return _PARAM_KINDS.get(type(default), _abi.PARAM_ANY)
    if not default:
        return _abi.PARAM_LIST
    if all(isinstance(v, str) for v in default):
        return _abi.PARAM_STRING_LIST
    if all(isinstance(v, (int, float)) and not isinstance(v, bool)
           for v in default):
        return _abi.PARAM_DOUBLE_LIST
    raise TypeError(f"param {key!r}: a list default must hold only strings "
                    "or only numbers")


def _param_value(out):
    """The Python value of a found talos_node_param result."""
    if out.kind == _abi.PARAM_BOOL:
        return bool(out.boolean)
    if out.kind == _abi.PARAM_INT:
        return out.integer
    if out.kind == _abi.PARAM_DOUBLE:
        return out.number
    if out.kind == _abi.PARAM_STRING_LIST:
        return [out.strings[i].decode() for i in range(out.count)]
    if out.kind == _abi.PARAM_DOUBLE_LIST:
        return [out.numbers[i] for i in range(out.count)]
    return out.string.decode()


class Node:
    """The framework's side of one node. Made by talos, handed to __init__.

    A Node keeps its owner (the talos.testing.Sim hosting it) alive, and
    every wrapper made from it keeps the Node, so the native node outlives
    anything that can reach it.
    """

    def __init__(self, handle, owner=None):
        self._native = handle
        self._owner = owner
        self._callbacks = []  # C callbacks live as long as the node

    @property
    def _handle(self):
        if self._native is None:
            raise TalosError("this node's Sim has been destroyed")
        return self._native

    @property
    def name(self):
        return lib.talos_node_name(self._handle).decode()

    @property
    def simulation(self):
        return bool(lib.talos_node_simulation(self._handle))

    @property
    def describing(self):
        """True only while the node is built to answer --describe: skip
        building heavy native backends, but validate every parameter and
        declare the same endpoints as in a run."""
        return bool(lib.talos_node_describing(self._handle))

    @property
    def now(self):
        return lib.talos_node_now_ns(self._handle) / _NS

    @property
    def tick(self):  # update() calls completed so far
        return lib.talos_node_tick(self._handle)

    @property
    def period(self):
        return lib.talos_node_period_ns(self._handle) / _NS

    @property
    def config_path(self):
        return lib.talos_node_config_path(self._handle).decode()

    @property
    def session_dir(self):
        """The session output directory (parent of --log), or None without
        --log (tests) and in --describe, --replay and --replay-inputs."""
        path = lib.talos_node_session_dir(self._handle)
        return None if path is None else path.decode()

    def param(self, key, default=_REQUIRED):
        """A key of this subsystem's config table, typed like `default`.

        Without a default the key must exist. Dotted keys reach nested tables
        and an index reaches an array element ("geometry.modules[0].x_m";
        past the end is absent). A TOML array comes back as a list: of str,
        or of float (integer elements convert). A list default asks for an
        array; its elements pick the element type, and [] accepts either.
        """
        kind = _param_kind(key, default)
        out = _abi.Param()
        found = _abi.ok(lib.talos_node_param(
            self._handle, _abi.encode(key), kind, default is _REQUIRED, out))
        return _param_value(out) if found else default

    def param_of(self, subsystem, key, default=_REQUIRED):
        """param() of another subsystem's table, [subsystems.<subsystem>].

        For a value that subsystem owns, so it is configured once. Read-only
        and only in __init__; a subsystem the config lacks raises TalosError
        even with a default.
        """
        kind = _param_kind(key, default)
        out = _abi.Param()
        found = _abi.ok(lib.talos_node_param_of(
            self._handle, _abi.encode(subsystem), _abi.encode(key), kind,
            default is _REQUIRED, out))
        return _param_value(out) if found else default

    def subscribe(self, msg, topic, callback=None, optional=False):
        """The latest `msg` on `topic`; callback(message) on each arrival.

        optional: the graph lint tolerates no publisher.
        """
        mtype = message_type(msg)
        on_message = _abi.MESSAGE_CALLBACK()
        if callback is not None:
            on_message = self._keep(_abi.MESSAGE_CALLBACK(_abi.callback(
                lambda data, size, _: callback(
                    mtype.decode(ctypes.string_at(data, size))))))
        index = _abi.index(lib.talos_node_subscribe(
            self._handle, _abi.encode(topic), mtype.size,
            _abi.schema_of(mtype), bool(optional), on_message, None))
        return Subscription(self, mtype, index)

    def publish(self, msg, topic):
        mtype = message_type(msg)
        index = _abi.index(lib.talos_node_publish(
            self._handle, _abi.encode(topic), mtype.size,
            _abi.schema_of(mtype)))
        return Publisher(self, mtype, index)

    def publish_bytes(self, topic, max_bytes, *, slots=1024,
                      drop_newest=False, external=False):
        """A topic of variable-length messages, each at most `max_bytes`."""
        index = _abi.index(lib.talos_node_publish_bytes(
            self._handle, _abi.encode(topic), max(0, max_bytes),
            max(0, slots), bool(drop_newest), bool(external)))
        return BytePublisher(self, index)

    def every(self, period, fn):
        """Calls fn() every `period` seconds, besides update()."""
        _abi.ok(lib.talos_node_every(
            self._handle, round(period * _NS),
            self._keep(_abi.CALLBACK(_abi.callback(lambda _: fn()))), None))

    def _keep(self, c_callback):
        self._callbacks.append(c_callback)
        return c_callback


class Subscription(_OnNode):
    """Latest value on a topic, aged from publication, not delivery.

    Without a callback each drain skips to the newest message. With a callback
    the framework delivers in sequence, up to 64 messages per topic per drain.
    """

    def __init__(self, node, mtype, index):
        super().__init__(node)
        self._type, self._index = mtype, index

    @property
    def value(self):
        """The latest message, or None before the first."""
        if not self.has_value:
            return None
        data = lib.talos_subscription_value(self._node, self._index)
        if not data:
            _abi.fail()
        return self._type.decode(ctypes.string_at(data, self._type.size))

    @property
    def has_value(self):
        return bool(_abi.ok(
            lib.talos_subscription_has_value(self._node, self._index)))

    def fresh(self, max_age=None):
        """has_value and age < max_age (default [robot] command_timeout)."""
        max_age_ns = -1 if max_age is None else round(max_age * _NS)
        return bool(_abi.ok(lib.talos_subscription_fresh(
            self._node, self._index, max_age_ns)))

    @property
    def age(self):  # seconds since publication; inf before the first
        if not self.has_value:
            return math.inf
        return lib.talos_subscription_age_ns(self._node, self._index) / _NS

    @property
    def updated(self):  # a message arrived since the previous update() began
        return bool(_abi.ok(
            lib.talos_subscription_updated(self._node, self._index)))


class Publisher(_OnNode):
    def __init__(self, node, mtype, index):
        super().__init__(node)
        self._type, self._index = mtype, index

    def send(self, message=None, **fields):
        """send(field=value, ...) naming every .fbs field (an omitted one
        raises TypeError), or send(message) to re-send a received one."""
        data = self._type.forward(message) if message is not None else (
            self._type.encode(fields))
        return bool(_abi.ok(lib.talos_publisher_send(
            self._node, self._index, data, len(data))))


class BytePublisher(_OnNode):
    """A variable-length message topic; see Node.publish_bytes."""

    def __init__(self, node, index):
        super().__init__(node)
        self._index = index

    def send(self, data):
        """Publishes `data` (bytes, bytearray, or memoryview); False when
        dropped or longer than max_bytes."""
        if not isinstance(data, (bytes, bytearray, memoryview)):
            raise TypeError(f"expected bytes-like object, got "
                            f"{type(data).__name__}")
        raw = bytes(data)
        return bool(_abi.ok(lib.talos_byte_publisher_send(
            self._node, self._index, raw, len(raw))))


class StateMachine(_OnNode):
    """A state plus when it was entered, ticking with the node.

    States are enum.Enum members, or the int values of a flatc enum class
    given as `states`:

        self.sm = talos.StateMachine(node, State.STOWED, states=State)

    With either, repr names the states and go() refuses a value that is not
    one of them.
    """

    def __init__(self, node, initial, states=None):
        if states is None and isinstance(initial, enum.Enum):
            states = type(initial)
        self._states = states
        self._names = enum_names(states) if states is not None else None
        self._check(initial)
        super().__init__(node)
        self._enum_type = type(initial) if isinstance(initial, enum.Enum) else None
        self._index = _abi.index(lib.talos_state_machine_create(
            self._node, self._to_int(initial)))

    def _to_int(self, state):
        return int(state.value if isinstance(state, enum.Enum) else state)

    def _from_int(self, raw):
        return self._enum_type(raw) if self._enum_type is not None else int(raw)

    @property
    def state(self):
        return self._from_int(_abi.value(
            lib.talos_state_machine_state(self._node, self._index), 0))

    @property
    def previous(self):
        return self._from_int(_abi.value(
            lib.talos_state_machine_previous(self._node, self._index), 0))

    def is_(self, state):
        return self.state == state

    def go(self, state):
        """Switch to `state`; a no-op when already in it."""
        self._check(state)
        _abi.ok(lib.talos_state_machine_go(
            self._node, self._index, self._to_int(state)))

    def __repr__(self):
        return (f"StateMachine({self._name(self.state)}, "
                f"previous={self._name(self.previous)}, "
                f"time_in_state={self.time_in_state:g}s)")

    def _name(self, state):
        if self._names is not None and state in self._names:
            return self._names[state]
        return repr(state)

    def _check(self, state):
        if self._names is not None and state not in self._names:
            raise ValueError(f"{self._states.__qualname__} has no state "
                             f"{state!r}; it has "
                             f"{', '.join(self._names.values())}")

    @property
    def time_in_state(self):
        return _abi.value(lib.talos_state_machine_time_in_state_ns(
            self._node, self._index), 0) / _NS

    @property
    def just_entered(self):
        """True during the first update() tick that starts in this state."""
        return bool(_abi.ok(lib.talos_state_machine_just_entered(
            self._node, self._index)))

def _factory(cls, built, owner=None, freeze=False):
    """A C factory: builds cls on a new Node and installs its update().

    Appends (node, object) to `built`, which must outlive the C++ node: the
    Node holds the C callbacks. `owner` is what owns the native node (a
    talos.testing.Sim), kept alive by the Node. `freeze` calls
    _freeze_construction() once the object is built.
    """
    def build(handle, _):
        node = Node(handle, owner)
        obj = cls(node)
        update = getattr(obj, "update", None)
        if update is not None:
            _abi.ok(lib.talos_node_set_update(
                handle,
                node._keep(_abi.CALLBACK(_abi.callback(lambda _: update()))),
                None))
        built.append((node, obj))
        if freeze:
            _freeze_construction()
    return _abi.FACTORY(_abi.callback(build))


def _freeze_construction():
    """Collects construction's garbage, then moves every surviving object
    (modules, config, the node and what its constructor built) to the
    collector's permanent generation, so no later collection rescans them.
    A cycle created before this call and dropped after it is never freed.
    """
    gc.collect()
    gc.freeze()


def run(cls, node_name, node_target, argv=None):
    """Runs cls as this process's node (what talos_py_node's main calls).

    Same flags as a C++ node: --sim --describe --log --session-id --config
    --replay --duration-s. Freezes the garbage collector's heap once the node
    is built, before its loop runs (_freeze_construction). Returns the exit
    code.
    """
    argv = sys.argv if argv is None else argv
    c_argv = (ctypes.c_char_p * (len(argv) + 1))(
        *[_abi.encode(a) for a in argv], None)
    built = []
    code = lib.talos_run(len(argv), c_argv, _abi.encode(node_name),
                         _abi.encode(node_target),
                         _factory(cls, built, freeze=True), None)
    if code != 0 and _abi.pending() is not None:
        _abi.fail()
    return code


def load(path):
    """The Python file at `path`, as a module.

    A node file is loaded by path, not imported by name: intake.py shares
    its name with the `intake` message package it imports, and must not
    shadow it.
    """
    path = os.path.abspath(path)
    directory = os.path.dirname(path)
    sys.path[:] = [p for p in sys.path if os.path.abspath(p) != directory]
    stem = os.path.splitext(os.path.basename(path))[0]
    name = f"_talos_node_{stem}"
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _resolve_library(library_path):
    candidates = [library_path]
    repo_root = os.path.abspath(
        os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    candidates.append(os.path.join(repo_root, library_path))
    runfiles_dir = os.environ.get("RUNFILES_DIR")
    if runfiles_dir:
        candidates.append(os.path.join(runfiles_dir, "_main", library_path))
        candidates.append(os.path.join(runfiles_dir, library_path))
    if sys.argv and sys.argv[0]:
        candidates.append(
            os.path.join(f"{sys.argv[0]}.runfiles", "_main", library_path))
    for candidate in candidates:
        if os.path.exists(candidate):
            return candidate
    raise TalosError(f"C++ node library not found: {library_path}")


def cpp_node(library_path, name="main"):
    """Wraps a C++ node exported with TALOS_EXPORT_NODE(name, Class) as a
    Python node class.

    The library must be a talos_cc_shared_node: it then needs
    libtalos_node.so, which _abi loaded at import, and the loader reuses that
    image (macOS by install name, Linux by soname) rather than a second copy
    of the framework."""
    resolved = _resolve_library(library_path)
    cdll = ctypes.CDLL(resolved)
    fn = getattr(cdll, f"talos_cpp_node_{name}")
    fn.argtypes = [_abi.c_void_p, _abi.c_void_p]
    fn.restype = _abi.c_int32

    class CppNode:
        def __init__(self, node):
            if fn(node._handle, None) < 0:
                raise TalosError("C++ node initialization failed")

    return CppNode
