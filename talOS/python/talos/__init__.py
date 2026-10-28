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
            if self.goal.fresh():
                self.lift.set_motion_magic(self.goal.value.height_rot)
            self.status.send(height_rot=self.lift.position)

The class is built with the node and registers everything in __init__;
update(), when present, runs every period. This is a thin binding over the
C++ talos::Node (talOS/node), so both languages behave the same. Times are
seconds.
"""

import ctypes
import importlib.util
import math
import os
import sys

from talos import _abi
from talos._abi import TalosError, lib
from talos._messages import Message, message_type
from talos.devices import (AnalogInput, CANcoder, DigitalInput, DigitalOutput,
                           Encoder, Gyro, Mode, Motor, PwmOutput,
                           robot_enabled)

__all__ = [
    "AnalogInput", "CANcoder", "DigitalInput", "DigitalOutput", "Encoder",
    "Gyro", "Message", "Mode", "Motor", "Node", "Publisher", "PwmOutput",
    "StateMachine", "Subscription", "TalosError", "load",
    "robot_enabled", "run",
]

_NS = 1e9
_REQUIRED = object()
_PARAM_KINDS = {bool: _abi.PARAM_BOOL, int: _abi.PARAM_INT,
                float: _abi.PARAM_DOUBLE, str: _abi.PARAM_STRING}


class Node:
    """The framework's side of one node. Made by talos, handed to __init__."""

    def __init__(self, handle):
        self._handle = handle
        self._callbacks = []  # C callbacks live as long as the node

    @property
    def name(self):
        return lib.talos_node_name(self._handle).decode()

    @property
    def simulation(self):
        return bool(lib.talos_node_simulation(self._handle))

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

    def param(self, key, default=_REQUIRED):
        """A key of this subsystem's config table, typed like `default`.

        Without a default the key must exist. Dotted keys reach nested tables.
        """
        kind = _PARAM_KINDS.get(type(default), _abi.PARAM_ANY)
        out = _abi.Param()
        found = _abi.ok(lib.talos_node_param(
            self._handle, _abi.encode(key), kind, default is _REQUIRED, out))
        if not found:
            return default
        if out.kind == _abi.PARAM_BOOL:
            return bool(out.boolean)
        if out.kind == _abi.PARAM_INT:
            return out.integer
        if out.kind == _abi.PARAM_DOUBLE:
            return out.number
        return out.string.decode()

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
            self._handle, _abi.encode(topic), mtype.size, bool(optional),
            on_message, None))
        return Subscription(self, mtype, index)

    def publish(self, msg, topic):
        mtype = message_type(msg)
        index = _abi.index(lib.talos_node_publish(
            self._handle, _abi.encode(topic), mtype.size))
        return Publisher(self, mtype, index)

    def every(self, period, fn):
        """Calls fn() every `period` seconds, besides update()."""
        _abi.ok(lib.talos_node_every(
            self._handle, round(period * _NS),
            self._keep(_abi.CALLBACK(_abi.callback(lambda _: fn()))), None))

    def _keep(self, c_callback):
        self._callbacks.append(c_callback)
        return c_callback


class Subscription:
    """Latest value on a topic, kept by the framework."""

    def __init__(self, node, mtype, index):
        self._node, self._type, self._index = node._handle, mtype, index

    @property
    def value(self):
        """The latest message, or None before the first."""
        if not self.has_value:
            return None
        data = lib.talos_subscription_value(self._node, self._index)
        return self._type.decode(ctypes.string_at(data, self._type.size))

    @property
    def has_value(self):
        return bool(lib.talos_subscription_has_value(self._node, self._index))

    def fresh(self, max_age=None):
        """has_value and age < max_age (default [robot] command_timeout)."""
        max_age_ns = -1 if max_age is None else round(max_age * _NS)
        return bool(lib.talos_subscription_fresh(self._node, self._index,
                                                 max_age_ns))

    @property
    def age(self):  # seconds since it arrived; inf before the first
        if not self.has_value:
            return math.inf
        return lib.talos_subscription_age_ns(self._node, self._index) / _NS

    @property
    def updated(self):  # a message arrived since the previous update() began
        return bool(lib.talos_subscription_updated(self._node, self._index))


class Publisher:
    def __init__(self, node, mtype, index):
        self._node, self._type, self._index = node._handle, mtype, index

    def send(self, message=None, **fields):
        """send(field=value, ...) with the .fbs field names (missing ones
        are 0/False), or send(message) to re-send a received one."""
        data = message._bytes if message is not None else (
            self._type.encode(fields))
        return bool(_abi.ok(lib.talos_publisher_send(
            self._node, self._index, data, len(data))))


class StateMachine:
    """A state plus when it was entered, ticking with the node."""

    def __init__(self, node, initial):
        self._node = node
        self._state = self._previous = initial
        self._entered_update = lib.talos_node_next_update(node._handle)
        self._entered_at = node.now

    @property
    def state(self):
        return self._state

    @property
    def previous(self):
        return self._previous

    def is_(self, state):
        return self._state == state

    def go(self, state):
        """Switch to `state`; a no-op when already in it."""
        if state == self._state:
            return
        self._previous, self._state = self._state, state
        self._entered_at = self._node.now
        # Entered during update() k: the first update to start in it is k + 1.
        self._entered_update = lib.talos_node_next_update(self._node._handle)

    @property
    def time_in_state(self):
        return self._node.now - self._entered_at

    @property
    def just_entered(self):
        """True during the first update() tick that starts in this state."""
        return self._node.tick == self._entered_update


def _factory(cls, built):
    """A C factory: builds cls on a new Node and installs its update().

    Appends (node, object) to `built`, which must outlive the C++ node: the
    Node holds the C callbacks.
    """
    def build(handle, _):
        node = Node(handle)
        obj = cls(node)
        update = getattr(obj, "update", None)
        if update is not None:
            _abi.ok(lib.talos_node_set_update(
                handle,
                node._keep(_abi.CALLBACK(_abi.callback(lambda _: update()))),
                None))
        built.append((node, obj))
    return _abi.FACTORY(_abi.callback(build))


def run(cls, node_name, node_target, argv=None):
    """Runs cls as this process's node (what talos_py_node's main calls).

    Same flags as a C++ node: --sim --describe --log --session-id --config
    --replay --duration-s. Returns the exit code.
    """
    argv = sys.argv if argv is None else argv
    c_argv = (ctypes.c_char_p * (len(argv) + 1))(
        *[_abi.encode(a) for a in argv], None)
    built = []
    code = lib.talos_run(len(argv), c_argv, _abi.encode(node_name),
                         _abi.encode(node_target), _factory(cls, built), None)
    if code != 0 and _abi._pending is not None:
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
