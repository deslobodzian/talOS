"""ctypes binding over the C ground-truth library ``libtalos_node.so``.

This module is the ONLY Python door into the RTMS transport and the
describe envelope. It binds the exact ABI declared in
``talOS/node_api/node_api.h`` (``TALOS_NODE_ABI_VERSION == 1``) and never
reimplements transport or envelope logic in Python:

* publish / subscribe go through ``talos_topic_open_publisher`` /
  ``talos_publish`` / ``talos_topic_open_subscriber`` / ``talos_poll_next``.
* ``describe_json`` passes raw source rows through ``talos_describe_emit``,
  which builds the envelope via ``introspect::DescribeManifest`` /
  ``DescribeToJson``. Python never hand-formats the JSON.

Public surface (exact cross-track contract -- do not drift):

* ``Publisher`` -- single producer for a topic (owns/reclaims the layout).
* ``RTMSQueue`` -- registry of readers plus a lazy publisher handle, with
  ``register_reader`` / ``read_next`` / ``release_reader`` / ``write``.
* ``describe_json(name, target, sources)`` -- ``sources`` is a list of
  ``(kind_int, topic, message_bytes, external, optional)`` tuples; returns
  the JSON string emitted by ``talos_describe_emit``.

ABI policy (mirrors node_api.h, additive-only): never renumber, remove, or
re-signature an existing export; only add. The loader refuses a library
older than the version this module was built against.
"""

import ctypes
import os

TALOS_NODE_ABI_VERSION = 1

TALOS_OK = 0
TALOS_EMPTY = 1
TALOS_ERR = -1
TALOS_ERR_ARG = -2
TALOS_ERR_SMALL = -3

TALOS_SOURCE_TIMER = 1
TALOS_SOURCE_WATCHER = 2
TALOS_SOURCE_FETCHER = 3
TALOS_SOURCE_SENDER = 4

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


_lib = None


def _workspace_root(start):
    """Nearest ancestor containing MODULE.bazel (the repo root)."""
    cur = os.path.abspath(start)
    while True:
        if os.path.exists(os.path.join(cur, "MODULE.bazel")):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            return None
        cur = parent


def _runfiles_roots():
    here = os.path.dirname(os.path.abspath(__file__))
    roots = []
    srcdir = os.environ.get("TEST_SRCDIR")
    if srcdir:
        roots.append(srcdir)
    runfiles = os.environ.get("RUNFILES_DIR")
    if runfiles:
        roots.append(runfiles)
    # Runfiles layout: <binary>.runfiles/<workspace>/talOS/ipc/python/....
    marker = ".runfiles" + os.sep
    if marker in here:
        roots.append(here.split(marker)[0] + "/.runfiles")
    return roots


def _candidate_paths():
    here = os.path.dirname(os.path.abspath(__file__))
    yield os.environ.get("TALOS_NODE_LIB", "")
    # Bazel test runfiles: cc_binary linkshared output wired via data=.
    # Workspace dir is the module name ("talos"); accept env/_main fallbacks.
    workspaces = [os.environ.get("TEST_WORKSPACE", ""), "talos", "_main", ""]
    seen = set()
    for root in _runfiles_roots():
        for ws in workspaces:
            key = (root, ws)
            if key in seen:
                continue
            seen.add(key)
            yield os.path.join(root, ws, "talOS", "node_api", _LIB_NAME)
    root = _workspace_root(here)
    if root:
        yield os.path.join(root, "bazel-bin", "talOS", "node_api", _LIB_NAME)


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
    if encoded:
        arr = (TalosSource * len(encoded))()
        for i, (kind, topic_b, message_bytes, flags) in enumerate(encoded):
            arr[i].kind = kind
            arr[i].topic = topic_b
            arr[i].message_bytes = message_bytes
            arr[i].flags = flags
        # Keep topic bytes alive across both calls.
        keepalive = (arr, encoded)
    else:
        arr = None
        keepalive = None
    need = ctypes.c_uint32(0)
    rc = lib.talos_describe_emit(
        name.encode(), target.encode(), arr, len(encoded),
        None, 0, ctypes.byref(need))
    if rc != TALOS_ERR_SMALL:
        raise TalosError("describe emit failed: %s" % _err(lib))
    buf = ctypes.create_string_buffer(need.value + 1)
    written = ctypes.c_uint32(0)
    rc = lib.talos_describe_emit(
        name.encode(), target.encode(), arr, len(encoded),
        buf, need.value + 1, ctypes.byref(written))
    del keepalive
    if rc != TALOS_OK:
        raise TalosError("describe emit failed: %s" % _err(lib))
    return buf.value.decode("utf-8")


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
