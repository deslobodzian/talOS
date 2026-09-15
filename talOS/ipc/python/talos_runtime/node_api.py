"""ctypes binding over the C ground-truth library ``libtalos_node.so``.

This module is the ONLY Python door into the RTMS transport, the describe
envelope, and the state-gating predicates. It binds the exact ABI declared
in ``talOS/node_api/node_api.h`` (``TALOS_NODE_ABI_VERSION == 6``) and never
reimplements transport, envelope, or gating logic in Python:

* publish / subscribe go through ``talos_topic_open_publisher`` /
  ``talos_publish`` / ``talos_topic_open_subscriber`` / ``talos_poll_next``.
* ``describe_json`` passes raw source rows through ``talos_describe_emit``,
  which builds the envelope via ``introspect::DescribeManifest`` /
  ``DescribeToJson``. Python never hand-formats the JSON.
* ``accept_state`` / ``gateway_ready`` / ``target_fresh_ns`` call the ABI
  predicates over ``talOS/hardware/gate.h``. Python never re-spells the
  config match, the staleness check, the gateway flags, or the freshness
  timeout: the C++ is the truth, this module binds it.
* ``Config`` / ``load_config`` wrap ``TalosConfig`` from ``talos_config_load``.
* ``Loop`` / ``DispatchContext`` wrap ``TalosLoop`` and ``TalosDispatchContext``
  for realtime, simulated, and replay event loops.

Public surface (exact cross-track contract -- do not drift):

* ``Publisher`` -- single producer for a topic (owns/reclaims the layout).
* ``RTMSQueue`` -- registry of readers plus a lazy publisher handle, with
  ``register_reader`` / ``read_next`` / ``release_reader`` / ``write``.
* ``describe_json(name, target, sources)`` -- ``sources`` is a list of
  ``(kind_int, topic, message_bytes, external, optional)`` tuples; returns
  the JSON string emitted by ``talos_describe_emit``.
* ``accept_state(config_id, prev, next)`` -- ``prev``/``next`` are decoded
  state dicts (``prev`` may be None); returns True when ``next`` replaces
  ``prev``, via ``talos_accept_state``.
* ``gateway_ready(flags)`` / ``target_fresh_ns(now_ns, issued_ns, enabled,
  state_rx_ns, timeout_us)`` -- bool answers via ``talos_gateway_ready`` /
  ``talos_target_fresh_ns``.
* ``Config`` / ``load_config(config_path, simulation)`` -- parsed robot
  configuration with ``config_id``, ``period_us``, ``command_timeout_us``,
  ``get_device_id``.
* ``Loop`` -- native event loop wrapper for realtime, simulated, and replay modes.
* ``DispatchContext`` -- context passed to dispatch callbacks.

ABI policy (mirrors node_api.h, additive-only): never renumber, remove, or
re-signature an existing export; only add. The loader refuses a library
older than the version this module was built against.
"""

import ctypes
import inspect
import os
import sys

TALOS_NODE_ABI_VERSION = 6

# Reporter flag bits (talOS/introspection/registry.h kFlagSimulation/Replay).
FLAG_SIMULATION = 1 << 0
FLAG_REPLAY = 1 << 1

TALOS_OK = 0
TALOS_EMPTY = 1
TALOS_ERR = -1
TALOS_ERR_ARG = -2
TALOS_ERR_SMALL = -3

TALOS_SOURCE_TIMER = 1
TALOS_SOURCE_WATCHER = 2
TALOS_SOURCE_FETCHER = 3
TALOS_SOURCE_SENDER = 4

# Bare aliases matching event::SourceKind names; describe rows read better
# with them than with the prefixed constants.
TIMER = TALOS_SOURCE_TIMER
WATCHER = TALOS_SOURCE_WATCHER
FETCHER = TALOS_SOURCE_FETCHER
SENDER = TALOS_SOURCE_SENDER

SOURCE_FLAG_EXTERNAL = 1 << 0
SOURCE_FLAG_OPTIONAL = 1 << 1

MAX_SLOTS = 1024
MAX_READERS = 8

OVERWRITE_OLDEST = "overwrite_oldest"
DROP_NEWEST = "drop_newest"
SEQUENCE = "sequence"
LATEST = "latest"

_LIB_NAME = "libtalos_node.so"


class TalosError(RuntimeError):
    """A node_api call failed; detail is in the message (talos_last_error)."""


class TalosSource(ctypes.Structure):
    _fields_ = [
        ("kind", ctypes.c_int32),
        ("topic", ctypes.c_char_p),
        ("message_bytes", ctypes.c_uint32),
        ("flags", ctypes.c_uint32),
    ]


class TalosRtmsLayout(ctypes.Structure):
    """Frozen RTMS geometry probed from the C++ structs (ABI 3)."""
    _fields_ = [
        ("header_size", ctypes.c_uint32),
        ("off_writer_seq", ctypes.c_uint32),
        ("off_readers", ctypes.c_uint32),
        ("reader_stride", ctypes.c_uint32),
    ]


class TalosDispatchContext(ctypes.Structure):
    _fields_ = [
        ("kind", ctypes.c_uint16),
        ("source_id", ctypes.c_uint16),
        ("dispatch_index", ctypes.c_uint64),
        ("event_time_ns", ctypes.c_int64),
        ("now_ns", ctypes.c_int64),
        ("sequence", ctypes.c_uint64),
        ("dropped", ctypes.c_uint64),
    ]


TalosDispatchFn = ctypes.CFUNCTYPE(
    None, ctypes.POINTER(TalosDispatchContext), ctypes.c_void_p,
    ctypes.c_uint32, ctypes.c_void_p
)


_lib = None


def _candidate_paths():
    override = os.environ.get("TALOS_NODE_LIB")
    if override:
        yield override
        return
    try:
        try:
            from python.runfiles import runfiles
        except ImportError:
            from rules_python.python.runfiles import runfiles
        r = runfiles.Create()
        if r:
            for rpath in [
                "talos/talOS/ipc/python/talos_runtime/" + _LIB_NAME,
                "_main/talOS/ipc/python/talos_runtime/" + _LIB_NAME,
                "talos/talOS/node_api/" + _LIB_NAME,
                "_main/talOS/node_api/" + _LIB_NAME,
            ]:
                resolved = r.Rlocation(rpath)
                if resolved and os.path.isfile(resolved):
                    yield resolved
    except Exception:
        pass
    yield os.path.join(os.path.dirname(os.path.abspath(__file__)), _LIB_NAME)


def find_lib():
    """First existing libtalos_node.so candidate, or None without loading."""
    for path in _candidate_paths():
        if path and os.path.isfile(path):
            return path
    return None


def _load():
    global _lib
    if _lib is not None:
        return _lib
    tried = []
    for path in _candidate_paths():
        if not path or not os.path.isfile(path):
            continue
        try:
            lib = ctypes.CDLL(path)
        except OSError as exc:
            tried.append("%s (%s)" % (path, exc))
            continue
        _bind(lib)
        _check_version(lib)
        _lib = lib
        return lib
    raise TalosError(
        "could not load %s (tried: %s). Build it with "
        "`bazel build //talOS/node_api:libtalos_node.so` or set TALOS_NODE_LIB"
        % (_LIB_NAME, "; ".join(tried) if tried else "no candidate paths exist")
    )


def _bind(lib):
    lib.talos_abi_version.argtypes = []
    lib.talos_abi_version.restype = ctypes.c_uint32
    lib.talos_monotonic_ns.argtypes = []
    lib.talos_monotonic_ns.restype = ctypes.c_int64
    lib.talos_last_error.argtypes = []
    lib.talos_last_error.restype = ctypes.c_char_p
    lib.talos_topic_open_publisher.argtypes = [
        ctypes.c_char_p, ctypes.c_uint32, ctypes.c_uint32]
    lib.talos_topic_open_publisher.restype = ctypes.c_void_p
    lib.talos_publish.argtypes = [
        ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint32]
    lib.talos_publish.restype = ctypes.c_int32
    lib.talos_publisher_close.argtypes = [ctypes.c_void_p]
    lib.talos_publisher_close.restype = None
    lib.talos_topic_open_subscriber.argtypes = [
        ctypes.c_char_p, ctypes.c_uint32, ctypes.c_uint32]
    lib.talos_topic_open_subscriber.restype = ctypes.c_void_p
    lib.talos_poll_next.argtypes = [
        ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint32,
        ctypes.POINTER(ctypes.c_uint64), ctypes.POINTER(ctypes.c_uint64)]
    lib.talos_poll_next.restype = ctypes.c_int32
    lib.talos_subscriber_close.argtypes = [ctypes.c_void_p]
    lib.talos_subscriber_close.restype = None
    lib.talos_describe_emit.argtypes = [
        ctypes.c_char_p, ctypes.c_char_p,
        ctypes.POINTER(TalosSource), ctypes.c_uint32,
        ctypes.c_char_p, ctypes.c_uint32,
        ctypes.POINTER(ctypes.c_uint32)]
    lib.talos_describe_emit.restype = ctypes.c_int32
    lib.talos_node_register.argtypes = [
        ctypes.c_char_p, ctypes.c_char_p, ctypes.c_uint64, ctypes.c_uint32]
    lib.talos_node_register.restype = ctypes.c_void_p
    lib.talos_node_publish.argtypes = [
        ctypes.c_void_p, ctypes.POINTER(TalosSource), ctypes.c_uint32]
    lib.talos_node_publish.restype = ctypes.c_int32
    lib.talos_node_heartbeat.argtypes = [ctypes.c_void_p, ctypes.c_uint64]
    lib.talos_node_heartbeat.restype = ctypes.c_int32
    lib.talos_node_close.argtypes = [ctypes.c_void_p]
    lib.talos_node_close.restype = None
    lib.talos_rtms_layout.argtypes = [ctypes.POINTER(TalosRtmsLayout)]
    lib.talos_rtms_layout.restype = ctypes.c_uint32
    lib.talos_accept_state.argtypes = [
        ctypes.c_uint64, ctypes.c_int32,
        ctypes.c_uint64, ctypes.c_uint64,
        ctypes.c_uint64, ctypes.c_uint64, ctypes.c_uint64]
    lib.talos_accept_state.restype = ctypes.c_int32
    lib.talos_gateway_ready.argtypes = [ctypes.c_uint32]
    lib.talos_gateway_ready.restype = ctypes.c_int32
    lib.talos_target_fresh_ns.argtypes = [
        ctypes.c_int64, ctypes.c_int64, ctypes.c_int32,
        ctypes.c_int64, ctypes.c_uint64]
    lib.talos_target_fresh_ns.restype = ctypes.c_int32
    lib.talos_config_load.argtypes = [ctypes.c_char_p, ctypes.c_int32]
    lib.talos_config_load.restype = ctypes.c_void_p
    lib.talos_config_get_id.argtypes = [ctypes.c_void_p]
    lib.talos_config_get_id.restype = ctypes.c_uint64
    lib.talos_config_get_period_us.argtypes = [ctypes.c_void_p]
    lib.talos_config_get_period_us.restype = ctypes.c_uint32
    lib.talos_config_get_command_timeout_us.argtypes = [ctypes.c_void_p]
    lib.talos_config_get_command_timeout_us.restype = ctypes.c_uint32
    lib.talos_config_get_device_id.argtypes = [
        ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p]
    lib.talos_config_get_device_id.restype = ctypes.c_uint32
    lib.talos_config_close.argtypes = [ctypes.c_void_p]
    lib.talos_config_close.restype = None
    lib.talos_loop_create_realtime.argtypes = [
        ctypes.c_char_p, ctypes.c_char_p, ctypes.c_uint64]
    lib.talos_loop_create_realtime.restype = ctypes.c_void_p
    lib.talos_loop_create_simulated.argtypes = [
        ctypes.c_char_p, ctypes.c_char_p]
    lib.talos_loop_create_simulated.restype = ctypes.c_void_p
    lib.talos_loop_create_replay.argtypes = [ctypes.c_char_p]
    lib.talos_loop_create_replay.restype = ctypes.c_void_p
    lib.talos_loop_register_timer.argtypes = [
        ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint64, ctypes.c_uint64,
        TalosDispatchFn, ctypes.c_void_p]
    lib.talos_loop_register_timer.restype = ctypes.c_uint16
    lib.talos_loop_register_watcher.argtypes = [
        ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint32, ctypes.c_uint32,
        TalosDispatchFn, ctypes.c_void_p]
    lib.talos_loop_register_watcher.restype = ctypes.c_uint16
    lib.talos_loop_register_sender.argtypes = [
        ctypes.c_void_p, ctypes.c_char_p, ctypes.c_uint32, ctypes.c_uint32]
    lib.talos_loop_register_sender.restype = ctypes.c_uint16
    lib.talos_loop_arm_timer.argtypes = [
        ctypes.c_void_p, ctypes.c_uint16, ctypes.c_uint64, ctypes.c_uint64]
    lib.talos_loop_arm_timer.restype = ctypes.c_int32
    lib.talos_loop_disarm_timer.argtypes = [
        ctypes.c_void_p, ctypes.c_uint16]
    lib.talos_loop_disarm_timer.restype = ctypes.c_int32
    lib.talos_loop_send.argtypes = [
        ctypes.c_void_p, ctypes.c_uint16, ctypes.c_void_p, ctypes.c_uint32]
    lib.talos_loop_send.restype = ctypes.c_int32
    lib.talos_loop_now_ns.argtypes = [ctypes.c_void_p]
    lib.talos_loop_now_ns.restype = ctypes.c_int64
    lib.talos_loop_run.argtypes = [ctypes.c_void_p]
    lib.talos_loop_run.restype = ctypes.c_int32
    lib.talos_loop_run_for.argtypes = [ctypes.c_void_p, ctypes.c_int64]
    lib.talos_loop_run_for.restype = ctypes.c_int32
    lib.talos_loop_exit.argtypes = [ctypes.c_void_p]
    lib.talos_loop_exit.restype = None
    lib.talos_loop_diverged.argtypes = [ctypes.c_void_p]
    lib.talos_loop_diverged.restype = ctypes.c_int32
    lib.talos_loop_reached_exit.argtypes = [ctypes.c_void_p]
    lib.talos_loop_reached_exit.restype = ctypes.c_int32
    lib.talos_loop_dispatch_count.argtypes = [ctypes.c_void_p]
    lib.talos_loop_dispatch_count.restype = ctypes.c_uint64
    lib.talos_loop_sim_inject.argtypes = [
        ctypes.c_void_p, ctypes.c_char_p, ctypes.c_void_p, ctypes.c_uint32]
    lib.talos_loop_sim_inject.restype = ctypes.c_int32
    lib.talos_loop_attach_reporter.argtypes = [
        ctypes.c_void_p, ctypes.c_char_p, ctypes.c_char_p, ctypes.c_uint64,
        ctypes.c_uint32]
    lib.talos_loop_attach_reporter.restype = ctypes.c_int32
    lib.talos_loop_destroy.argtypes = [ctypes.c_void_p]
    lib.talos_loop_destroy.restype = None


def _check_version(lib):
    runtime = lib.talos_abi_version()
    if runtime < TALOS_NODE_ABI_VERSION:
        raise TalosError(
            "libtalos_node.so ABI %d is older than the version %d this "
            "module was built against; rebuild the library" % (
                runtime, TALOS_NODE_ABI_VERSION))


def _err(lib):
    raw = lib.talos_last_error()
    return raw.decode("utf-8", "replace") if raw else "unknown error"


def abi_version():
    """Runtime ABI version of the loaded library."""
    return _load().talos_abi_version()


def monotonic_ns():
    """Monotonic nanoseconds (CLOCK_MONOTONIC) via the library."""
    return _load().talos_monotonic_ns()


def last_error():
    """Last error on this thread reported by the library."""
    return _err(_load())


def accept_state(config_id, prev, next):
    """Whether the ``next`` sample replaces the last accepted one.

    ``prev``/``next`` are the decoded state dicts (``config_id``,
    ``boot_id``, ``sample_time_us``); ``prev`` is None before the first
    accepted sample. Binds ``talos_accept_state`` over
    ``hardware::AcceptState`` -- the config match and the staleness check
    live in C++, never here.
    """
    lib = _load()
    if prev is None:
        return bool(lib.talos_accept_state(
            int(config_id), 0, 0, 0,
            int(next["config_id"]), int(next["boot_id"]),
            int(next["sample_time_us"])))
    return bool(lib.talos_accept_state(
        int(config_id), 1, int(prev["boot_id"]),
        int(prev["sample_time_us"]),
        int(next["config_id"]), int(next["boot_id"]),
        int(next["sample_time_us"])))


def gateway_ready(flags):
    """Whether the gateway is ready, via ``talos_gateway_ready``."""
    return bool(_load().talos_gateway_ready(int(flags)))


def target_fresh_ns(now_ns, issued_ns, enabled, state_rx_ns, timeout_us):
    """Whether a latched target may drive, via ``talos_target_fresh_ns``."""
    return bool(_load().talos_target_fresh_ns(
        int(now_ns), int(issued_ns), 1 if enabled else 0,
        int(state_rx_ns), int(timeout_us)))


_layout = None


def rtms_layout():
    """Frozen RTMS geometry read from the library (cached).

    Importing this module never touches the ``.so``; the first call loads
    it. Callers that only need pure helpers stay ``.so``-free.
    """
    global _layout
    if _layout is None:
        layout = TalosRtmsLayout()
        if not _load().talos_rtms_layout(ctypes.byref(layout)):
            raise TalosError("talos_rtms_layout failed: %s" % last_error())
        _layout = layout
    return _layout


def _validate_layout(message_bytes, alignment):
    if not isinstance(message_bytes, int) or message_bytes <= 0:
        raise ValueError("message_bytes must be a positive int")
    if (not isinstance(alignment, int) or alignment <= 0
            or (alignment & (alignment - 1)) != 0):
        raise ValueError("alignment must be a nonzero power of two")


def _validate_slots(slots):
    if not isinstance(slots, int) or slots < 2 or (slots & (slots - 1)) != 0:
        raise ValueError("slots must be a power of 2 >= 2")


def _validate_topic(topic):
    if not isinstance(topic, str) or not topic:
        raise ValueError("topic must be a non-empty string")
    return topic.encode()


def _encode_sources(sources):
    """Validate (kind, topic, bytes, external, optional) rows into a ctypes
    array. Returns (arr, count, keepalive); arr is None when empty.
    """
    rows = list(sources) if sources else []
    encoded = []
    for row in rows:
        kind, topic, message_bytes, external, optional = row
        if kind not in (TALOS_SOURCE_TIMER, TALOS_SOURCE_WATCHER,
                        TALOS_SOURCE_FETCHER, TALOS_SOURCE_SENDER):
            raise ValueError(
                "unknown source kind %r (want 1..4)" % (kind,))
        if not isinstance(topic, str) or not topic:
            raise ValueError("source topic must be a non-empty string")
        flags = ((SOURCE_FLAG_EXTERNAL if external else 0)
                 | (SOURCE_FLAG_OPTIONAL if optional else 0))
        encoded.append((kind, topic.encode(), message_bytes, flags))
    if not encoded:
        return (None, 0, None)
    arr = (TalosSource * len(encoded))()
    for i, (kind, topic_b, message_bytes, flags) in enumerate(encoded):
        arr[i].kind = kind
        arr[i].topic = topic_b
        arr[i].message_bytes = message_bytes
        arr[i].flags = flags
    # Keep topic bytes alive across the call.
    return (arr, len(encoded), (arr, encoded))


def describe_json(name, target, sources):
    """Emit the --describe envelope via talos_describe_emit.

    ``sources`` is a list of ``(kind_int, topic, message_bytes, external,
    optional)`` tuples where ``kind_int`` is 1..4 (TIMER/WATCHER/FETCHER/
    SENDER) and ``external``/``optional`` are bools mapped to the
    naming-protocol flag bits. Returns the JSON string.
    """
    lib = _load()
    if not isinstance(name, str) or not isinstance(target, str):
        raise ValueError("name and target must be strings")
    arr, count, keepalive = _encode_sources(sources)
    need = ctypes.c_uint32(0)
    rc = lib.talos_describe_emit(
        name.encode(), target.encode(), arr, count,
        None, 0, ctypes.byref(need))
    if rc != TALOS_ERR_SMALL:
        raise TalosError("describe emit failed: %s" % _err(lib))
    buf = ctypes.create_string_buffer(need.value + 1)
    written = ctypes.c_uint32(0)
    rc = lib.talos_describe_emit(
        name.encode(), target.encode(), arr, count,
        buf, need.value + 1, ctypes.byref(written))
    del keepalive
    if rc != TALOS_OK:
        raise TalosError("describe emit failed: %s" % _err(lib))
    return buf.value.decode("utf-8")


class Node:
    """One registry claim: publish the manifest once, heartbeat per tick.

    Degraded-ok: when the registry is unavailable the handle is None and
    heartbeat() is a silent no-op, mirroring the C++ Reporter.
    """

    def __init__(self, name, target, session_id=0, flags=0, sources=()):
        lib = _load()
        if not isinstance(name, str) or not name:
            raise ValueError("name must be a non-empty string")
        if not isinstance(target, str) or not target:
            raise ValueError("target must be a non-empty string")
        self._lib = lib
        self._degraded = False
        self._handle = lib.talos_node_register(
            name.encode(), target.encode(), int(session_id), int(flags))
        if not self._handle:
            sys.stderr.write("node_api: registry unavailable (%s); "
                             "running without heartbeat\n" % _err(lib))
            self._degraded = True
            self._keepalive = None
            return
        arr, count, self._keepalive = _encode_sources(sources)
        rc = lib.talos_node_publish(self._handle, arr, count)
        if rc != TALOS_OK:
            err = _err(lib)
            lib.talos_node_close(self._handle)
            self._handle = None
            raise TalosError("node publish failed: %s" % err)

    def heartbeat(self, dispatches):
        """One refresh sample with a monotonically increasing count."""
        if self._handle is None:
            return
        rc = self._lib.talos_node_heartbeat(self._handle, int(dispatches))
        if rc != TALOS_OK:
            raise TalosError("node heartbeat failed: %s" % _err(self._lib))

    def close(self):
        handle, self._handle = self._handle, None
        if handle:
            self._lib.talos_node_close(handle)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class Publisher:
    """Single producer for a topic; owns (and reclaims) the segment layout."""

    def __init__(self, topic, message_bytes, alignment, slots=MAX_SLOTS):
        _validate_slots(slots)
        _validate_layout(message_bytes, alignment)
        lib = _load()
        handle = lib.talos_topic_open_publisher(
            _validate_topic(topic), message_bytes, alignment)
        if not handle:
            raise TalosError("open publisher %s: %s" % (topic, _err(lib)))
        self._lib = lib
        self._handle = handle
        self.topic = topic
        self.message_bytes = message_bytes
        self.message_alignment = alignment
        self.slots = slots

    def write(self, payload):
        """Publish one message. Returns None (sequence is C++-owned)."""
        data = bytes(payload)
        if len(data) != self.message_bytes:
            raise ValueError(
                "payload must be exactly message_bytes (%d), got %d" % (
                    self.message_bytes, len(data)))
        rc = self._lib.talos_publish(self._handle, data, len(data))
        if rc != TALOS_OK:
            raise TalosError("publish failed: %s" % _err(self._lib))
        return None

    def close(self):
        handle, self._handle = self._handle, None
        if handle:
            self._lib.talos_publisher_close(handle)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class RTMSQueue:
    """Reader registry plus lazy publisher handle over one topic."""

    def __init__(self, topic, message_bytes, alignment, slots=MAX_SLOTS,
                 reclaim_mismatched_segment=False):
        _validate_slots(slots)
        _validate_layout(message_bytes, alignment)
        _validate_topic(topic)
        self.topic = topic
        self.message_size = message_bytes
        self.message_alignment = alignment
        self.slots = slots
        self.reclaim_mismatched_segment = reclaim_mismatched_segment
        self._lib = None
        self._pub = None
        self._readers = {}
        self._cursors = {}

    def _ensure_lib(self):
        if self._lib is None:
            self._lib = _load()
        return self._lib

    def _ensure_pub(self):
        if self._pub is None:
            lib = self._ensure_lib()
            handle = lib.talos_topic_open_publisher(
                self.topic.encode(), self.message_size, self.message_alignment)
            if not handle:
                raise TalosError(
                    "open publisher %s: %s" % (self.topic, _err(lib)))
            self._pub = handle
        return self._pub

    def register_reader(self):
        """Open a reader snapped at the live writer; None when full."""
        for rid in range(MAX_READERS):
            if rid not in self._readers:
                break
        else:
            return None
        lib = self._ensure_lib()
        handle = lib.talos_topic_open_subscriber(
            self.topic.encode(), self.message_size, self.message_alignment)
        if not handle:
            if "no free reader" in _err(lib):
                return None
            raise TalosError(
                "open subscriber %s: %s" % (self.topic, _err(lib)))
        self._readers[rid] = handle
        self._cursors.pop(rid, None)
        return rid

    def release_reader(self, reader_id):
        handle = self._readers.pop(reader_id, None)
        self._cursors.pop(reader_id, None)
        if handle and self._lib is not None:
            self._lib.talos_subscriber_close(handle)

    def _poll_once(self, reader_id):
        lib = self._ensure_lib()
        buf = ctypes.create_string_buffer(self.message_size)
        seq = ctypes.c_uint64(0)
        dropped = ctypes.c_uint64(0)
        rc = lib.talos_poll_next(
            self._readers[reader_id], buf, self.message_size,
            ctypes.byref(seq), ctypes.byref(dropped))
        if rc == TALOS_OK:
            return ("ok", buf.raw, seq.value, dropped.value)
        if rc == TALOS_EMPTY:
            return ("empty", None, 0, 0)
        if rc == TALOS_ERR_ARG:
            return ("invalid", None, 0, 0)
        return ("torn", None, 0, 0)

    def write(self, payload):
        """Publish one message. Returns None (sequence is C++-owned)."""
        data = bytes(payload)
        if len(data) != self.message_size:
            raise ValueError(
                "payload must be exactly message_size (%d), got %d" % (
                    self.message_size, len(data)))
        lib = self._ensure_lib()
        rc = lib.talos_publish(self._ensure_pub(), data, len(data))
        if rc != TALOS_OK:
            raise TalosError("publish failed: %s" % _err(lib))
        return None

    def read_next(self, reader_id, overflow_policy=OVERWRITE_OLDEST,
                  read_mode=SEQUENCE):
        """Copy the reader's next message -> (status, payload, seq, dropped).

        ``read_mode`` SEQUENCE does a single poll; LATEST drains to the
        newest message. Only OVERWRITE_OLDEST is supported: the C library
        bakes that policy and per-call overflow selection has no ABI.
        """
        if reader_id >= MAX_READERS:
            return ("invalid", None, 0, 0)
        if reader_id not in self._readers:
            return ("inactive", None, 0, 0)
        if overflow_policy != OVERWRITE_OLDEST:
            raise ValueError(
                "overflow_policy %r unsupported: the node ABI bakes "
                "OVERWRITE_OLDEST" % (overflow_policy,))
        if read_mode not in (SEQUENCE, LATEST):
            raise ValueError("unknown read_mode %r" % (read_mode,))
        if read_mode == SEQUENCE:
            status, payload, seq, dropped = self._poll_once(reader_id)
            if status == "ok":
                self._cursors[reader_id] = seq + 1
            return (status, payload, seq, dropped)
        # LATEST: drain to the newest message.
        first = self._poll_once(reader_id)
        if first[0] != "ok":
            return first
        _, _, first_seq, first_dropped = first
        last_seq, last_payload = first_seq, first[1]
        while True:
            nxt = self._poll_once(reader_id)
            if nxt[0] != "ok":
                break
            last_seq, last_payload = nxt[2], nxt[1]
        cursor = self._cursors.get(reader_id)
        if cursor is None:
            dropped = (last_seq - first_seq) + first_dropped
        else:
            dropped = last_seq - cursor
        self._cursors[reader_id] = last_seq + 1
        return ("ok", last_payload, last_seq, dropped)

    def close(self):
        readers, self._readers = self._readers, {}
        self._cursors = {}
        pub, self._pub = self._pub, None
        if self._lib is not None:
            for handle in readers.values():
                self._lib.talos_subscriber_close(handle)
            if pub:
                self._lib.talos_publisher_close(pub)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class Config:
    """Parsed robot configuration handle over TalosConfig."""

    def __init__(self, config_path: str, simulation: bool = False):
        self._lib = None
        self._handle = None
        if not isinstance(config_path, str) or not config_path:
            raise ValueError("config_path must be a non-empty string")
        lib = _load()
        handle = lib.talos_config_load(
            config_path.encode("utf-8"), 1 if simulation else 0)
        if not handle:
            raise TalosError("load config %s: %s" % (config_path, _err(lib)))
        self._lib = lib
        self._handle = handle
        self.config_id = int(lib.talos_config_get_id(handle))
        self.period_us = int(lib.talos_config_get_period_us(handle))
        self.command_timeout_us = int(
            lib.talos_config_get_command_timeout_us(handle))

    def get_device_id(self, subsystem: str, device_name: str) -> int:
        if not self._handle:
            raise TalosError("config handle is closed")
        if not isinstance(subsystem, str) or not subsystem:
            raise ValueError("subsystem must be a non-empty string")
        if not isinstance(device_name, str) or not device_name:
            raise ValueError("device_name must be a non-empty string")
        return int(self._lib.talos_config_get_device_id(
            self._handle, subsystem.encode("utf-8"), device_name.encode("utf-8")))

    def close(self):
        handle = getattr(self, "_handle", None)
        self._handle = None
        lib = getattr(self, "_lib", None)
        if handle and lib is not None:
            lib.talos_config_close(handle)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def __del__(self):
        self.close()


def load_config(config_path: str, simulation: bool = False) -> Config:
    """Load and parse robot configuration via libtalos_node.so."""
    return Config(config_path, simulation=simulation)


class DispatchContext:
    """Context passed to dispatch callbacks."""

    def __init__(self, kind: int, source_id: int, dispatch_index: int,
                 event_time_ns: int, now_ns: int, sequence: int, dropped: int):
        self._kind = kind
        self._source_id = source_id
        self._dispatch_index = dispatch_index
        self._event_time_ns = event_time_ns
        self._now_ns = now_ns
        self._sequence = sequence
        self._dropped = dropped

    @classmethod
    def from_c(cls, c_ctx):
        c = c_ctx.contents if hasattr(c_ctx, "contents") else c_ctx
        return cls(
            kind=int(c.kind),
            source_id=int(c.source_id),
            dispatch_index=int(c.dispatch_index),
            event_time_ns=int(c.event_time_ns),
            now_ns=int(c.now_ns),
            sequence=int(c.sequence),
            dropped=int(c.dropped),
        )

    @property
    def kind(self) -> int:
        return self._kind

    @property
    def source_id(self) -> int:
        return self._source_id

    @property
    def dispatch_index(self) -> int:
        return self._dispatch_index

    @property
    def event_time_ns(self) -> int:
        return self._event_time_ns

    @property
    def now_ns(self) -> int:
        return self._now_ns

    @property
    def sequence(self) -> int:
        return self._sequence

    @property
    def dropped(self) -> int:
        return self._dropped

    @property
    def latency_ns(self) -> int:
        return self._now_ns - self._event_time_ns


def _invoke_callback(callback, ctx, data):
    try:
        sig = inspect.signature(callback)
        params = [
            p for p in sig.parameters.values()
            if p.kind in (inspect.Parameter.POSITIONAL_ONLY,
                          inspect.Parameter.POSITIONAL_OR_KEYWORD)
        ]
        has_varargs = any(p.kind == inspect.Parameter.VAR_POSITIONAL for p in sig.parameters.values())
        if has_varargs or len(params) >= 2:
            callback(ctx, data)
        elif len(params) == 1:
            callback(ctx)
        else:
            callback()
    except (ValueError, TypeError):
        try:
            callback(ctx, data)
        except TypeError:
            callback(ctx)


class Loop:
    """Event loop instance (realtime, simulated, or replay)."""

    def __init__(self, handle, lib=None):
        if not handle:
            raise ValueError("handle must not be None")
        self._lib = lib or _load()
        self._handle = handle
        self._callbacks = {}
        self._last_exception = None
        self._thunk_ref = TalosDispatchFn(self._dispatch_thunk)

    def _dispatch_thunk(self, c_ctx_ptr, data_ptr, size, user_data):
        del user_data
        try:
            c_ctx = c_ctx_ptr.contents
            py_ctx = DispatchContext.from_c(c_ctx)
            py_data = ctypes.string_at(data_ptr, size) if (data_ptr and size > 0) else b""
            callback = self._callbacks.get(c_ctx.source_id)
            if callback is not None:
                _invoke_callback(callback, py_ctx, py_data)
        except BaseException as e:
            self._last_exception = e
            self.exit()

    @classmethod
    def create_realtime(cls, log_path="", stream_name="", session_id=0):
        lib = _load()
        log_b = log_path.encode("utf-8") if log_path else None
        stream_b = stream_name.encode("utf-8") if stream_name else None
        handle = lib.talos_loop_create_realtime(log_b, stream_b, int(session_id))
        if not handle:
            raise TalosError("create_realtime loop failed: %s" % _err(lib))
        return cls(handle, lib)

    @classmethod
    def create_simulated(cls, log_path="", stream_name=""):
        lib = _load()
        log_b = log_path.encode("utf-8") if log_path else None
        stream_b = stream_name.encode("utf-8") if stream_name else None
        handle = lib.talos_loop_create_simulated(log_b, stream_b)
        if not handle:
            raise TalosError("create_simulated loop failed: %s" % _err(lib))
        return cls(handle, lib)

    @classmethod
    def create_replay(cls, replay_path):
        if not isinstance(replay_path, str) or not replay_path:
            raise ValueError("replay_path must be a non-empty string")
        lib = _load()
        handle = lib.talos_loop_create_replay(replay_path.encode("utf-8"))
        if not handle:
            raise TalosError("create_replay loop failed: %s" % _err(lib))
        return cls(handle, lib)

    def register_timer(self, name, period_ns=0, offset_ns=0, callback=None):
        if not isinstance(name, str) or not name:
            raise ValueError("name must be a non-empty string")
        if not self._handle:
            raise TalosError("loop handle is closed")
        timer_id = self._lib.talos_loop_register_timer(
            self._handle, name.encode("utf-8"), int(period_ns), int(offset_ns),
            self._thunk_ref, None)
        if timer_id == 0xFFFF:
            raise TalosError("register_timer %s failed: %s" % (name, _err(self._lib)))
        if callback is not None:
            self._callbacks[timer_id] = callback
        return timer_id

    def register_watcher(self, topic, message_bytes, alignment, callback):
        if not isinstance(topic, str) or not topic:
            raise ValueError("topic must be a non-empty string")
        _validate_layout(message_bytes, alignment)
        if callback is None:
            raise ValueError("callback must not be None")
        if not self._handle:
            raise TalosError("loop handle is closed")
        watcher_id = self._lib.talos_loop_register_watcher(
            self._handle, topic.encode("utf-8"), int(message_bytes), int(alignment),
            self._thunk_ref, None)
        if watcher_id == 0xFFFF:
            raise TalosError("register_watcher %s failed: %s" % (topic, _err(self._lib)))
        self._callbacks[watcher_id] = callback
        return watcher_id

    def register_sender(self, topic, message_bytes, alignment):
        if not isinstance(topic, str) or not topic:
            raise ValueError("topic must be a non-empty string")
        _validate_layout(message_bytes, alignment)
        if not self._handle:
            raise TalosError("loop handle is closed")
        sender_id = self._lib.talos_loop_register_sender(
            self._handle, topic.encode("utf-8"), int(message_bytes), int(alignment))
        if sender_id == 0xFFFF:
            raise TalosError("register_sender %s failed: %s" % (topic, _err(self._lib)))
        return sender_id

    def arm_timer(self, timer_id, period_ns, offset_ns=0):
        if not self._handle:
            raise TalosError("loop handle is closed")
        rc = self._lib.talos_loop_arm_timer(
            self._handle, int(timer_id), int(period_ns), int(offset_ns))
        if rc != TALOS_OK:
            raise TalosError("arm_timer failed: %s" % _err(self._lib))

    def disarm_timer(self, timer_id):
        if not self._handle:
            raise TalosError("loop handle is closed")
        rc = self._lib.talos_loop_disarm_timer(self._handle, int(timer_id))
        if rc != TALOS_OK:
            raise TalosError("disarm_timer failed: %s" % _err(self._lib))

    def send(self, sender_id, data):
        if not self._handle:
            raise TalosError("loop handle is closed")
        payload = bytes(data)
        rc = self._lib.talos_loop_send(
            self._handle, int(sender_id), payload, len(payload))
        if rc != TALOS_OK:
            raise TalosError("send failed: %s" % _err(self._lib))

    def now_ns(self):
        if not self._handle:
            raise TalosError("loop handle is closed")
        return self._lib.talos_loop_now_ns(self._handle)

    def run(self):
        if not self._handle:
            raise TalosError("loop handle is closed")
        self._last_exception = None
        rc = self._lib.talos_loop_run(self._handle)
        if self._last_exception is not None:
            exc = self._last_exception
            self._last_exception = None
            raise exc
        if rc != TALOS_OK:
            raise TalosError("loop run failed: %s" % _err(self._lib))

    def run_for(self, duration_ns):
        if not self._handle:
            raise TalosError("loop handle is closed")
        self._last_exception = None
        rc = self._lib.talos_loop_run_for(self._handle, int(duration_ns))
        if self._last_exception is not None:
            exc = self._last_exception
            self._last_exception = None
            raise exc
        if rc != TALOS_OK:
            raise TalosError("loop run_for failed: %s" % _err(self._lib))

    def exit(self):
        if self._handle:
            self._lib.talos_loop_exit(self._handle)

    @property
    def diverged(self) -> bool:
        if not self._handle:
            return False
        return bool(self._lib.talos_loop_diverged(self._handle))

    @property
    def reached_exit(self) -> bool:
        if not self._handle:
            return False
        return bool(self._lib.talos_loop_reached_exit(self._handle))

    @property
    def dispatch_count(self) -> int:
        if not self._handle:
            return 0
        return int(self._lib.talos_loop_dispatch_count(self._handle))

    def sim_inject(self, topic, data):
        if not isinstance(topic, str) or not topic:
            raise ValueError("topic must be a non-empty string")
        if not self._handle:
            raise TalosError("loop handle is closed")
        payload = bytes(data)
        rc = self._lib.talos_loop_sim_inject(
            self._handle, topic.encode("utf-8"), payload, len(payload))
        if rc != TALOS_OK:
            raise TalosError("sim_inject failed: %s" % _err(self._lib))

    def attach_reporter(self, name, target, session_id=0, flags=0):
        if not isinstance(name, str) or not name:
            raise ValueError("name must be a non-empty string")
        if not isinstance(target, str) or not target:
            raise ValueError("target must be a non-empty string")
        if not self._handle:
            raise TalosError("loop handle is closed")
        rc = self._lib.talos_loop_attach_reporter(
            self._handle, name.encode("utf-8"), target.encode("utf-8"),
            int(session_id), int(flags))
        if rc != TALOS_OK:
            raise TalosError("attach_reporter failed: %s" % _err(self._lib))

    def close(self):
        handle = getattr(self, "_handle", None)
        self._handle = None
        lib = getattr(self, "_lib", None)
        if handle and lib is not None:
            lib.talos_loop_destroy(handle)
        if hasattr(self, "_callbacks"):
            self._callbacks.clear()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def __del__(self):
        self.close()
