"""Python RTMS pub/sub client.

Thin ergonomic layer over :mod:`node_api`, the ctypes binding to the C
ground-truth library ``libtalos_node.so`` (``talOS/node_api/node_api.h``).
Transport rule: every shared-memory byte flows through a ``talos_*`` call;
no slot/header arithmetic lives in this file -- not even as constants. The
frozen RTMS geometry (``HEADER_SIZE``, ``OFF_WRITER_SEQ``, ``OFF_READERS``,
``READER_STRIDE``) is read from the library via ``talos_rtms_layout`` on
first access, so importing this module never touches the ``.so`` but no
layout fact is restated in Python.

Public API:

* ``Publisher`` / ``Subscriber`` composed over ``node_api.RTMSQueue``
  (the only ``RTMSQueue`` in the system) with ``register_reader`` /
  ``read_next`` / ``release_reader`` / ``write`` / ``close``.
* ``shm_object_name`` / ``align_up`` helpers and the ``OVERWRITE_OLDEST`` /
  ``DROP_NEWEST`` / ``SEQUENCE`` / ``LATEST`` policy strings.
* Frozen geometry names (``HEADER_SIZE``, ``OFF_WRITER_SEQ``,
  ``OFF_READERS``, ``READER_STRIDE``), resolved lazily from the library.

Differences from the pre-wrapper client, all documented in DESIGN.md:

* ``write`` pads a short payload with zeros (the old zero-filled slot did
  this implicitly); an oversized payload still raises ``ValueError``.
* ``write`` returns ``None``: the sequence counter is C++-owned and has no
  ABI export.
* Per-call ``overflow_policy`` other than ``OVERWRITE_OLDEST`` raises
  ``ValueError``: the C library bakes that policy.
"""

import ctypes
import ctypes.util
import os
import sys

try:
    import node_api
except ImportError:  # pragma: no cover - direct-script execution
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import node_api

from node_api import TalosError  # noqa: F401 - re-exported for callers

# Re-exported unchanged (single source of truth in node_api).
MAX_SLOTS = node_api.MAX_SLOTS
MAX_READERS = node_api.MAX_READERS
OVERWRITE_OLDEST = node_api.OVERWRITE_OLDEST
DROP_NEWEST = node_api.DROP_NEWEST
SEQUENCE = node_api.SEQUENCE
LATEST = node_api.LATEST

# Geometry below is C++ ground truth, read don't restate. The four names
# the layout test asserts (HEADER_SIZE, OFF_WRITER_SEQ, OFF_READERS,
# READER_STRIDE) resolve lazily from talos_rtms_layout, so import stays
# .so-free. Readers of the format should read talOS/rtms/rtms.h itself:
# a second copy drifts, and the drift is invisible.
_LAYOUT_ATTRS = {
    "HEADER_SIZE": "header_size",
    "OFF_WRITER_SEQ": "off_writer_seq",
    "OFF_READERS": "off_readers",
    "READER_STRIDE": "reader_stride",
}


def __getattr__(name):
    # PEP 562: module-level lazy binding for the frozen geometry.
    field = _LAYOUT_ATTRS.get(name)
    if field is None:
        raise AttributeError("module %r has no attribute %r"
                             % (__name__, name))
    return getattr(node_api.rtms_layout(), field)


# MAX_TOPIC_BYTES is an OS fact, not a C++ choice: macOS caps a POSIX shm
# name at PSHMNAMLEN, and rtms.cc ValidatePath enforces the same limit
# because a longer name is one the OS cannot open. Stated, not queried --
# which keeps shm_object_name pure Python and .so-free.
# Shared-memory object names map interior '/' to '.'
# (shared_memory_ptr.h ShmObjectName): '.' is injective where '_' is not
# ('/hw/state/driver_station' vs '/hw/state_driver/station'), and the
# mapping preserves length.
MAX_TOPIC_BYTES = 30


def align_up(value, alignment):
    return (value + alignment - 1) & ~(alignment - 1)


def shm_object_name(topic):
    """Map a topic to its POSIX shared-memory object name."""
    name = topic.lstrip("/")
    if len(name.encode()) > MAX_TOPIC_BYTES:
        raise ValueError(
            "Topic name '%s' exceeds limit of 30 characters after leading slash"
            % topic
        )
    return "/" + name.replace("/", ".")


def _libc():
    return ctypes.CDLL(ctypes.util.find_library("c"), use_errno=True)


def _shm_unlink(name):
    libc = _libc()
    libc.shm_unlink.argtypes = [ctypes.c_char_p]
    libc.shm_unlink.restype = ctypes.c_int
    libc.shm_unlink(name.encode())


class _ShmSegment:
    """Cleanup handle for segments owned by the C++ ground truth.

    Creation/mapping moved into libtalos_node.so; only ``unlink`` survives,
    used by tests to remove a stale topic segment before/after a run.
    """

    @staticmethod
    def unlink(shm_name):
        if sys.platform.startswith("linux") and os.path.isdir("/dev/shm"):
            try:
                os.unlink("/dev/shm/" + shm_name.lstrip("/"))
            except FileNotFoundError:
                pass
        else:
            _shm_unlink(shm_name)


def _pad_payload(payload, message_size):
    data = bytes(payload)
    if len(data) > message_size:
        raise ValueError(
            "Cannot write message larger than slot size: %d > %d"
            % (len(data), message_size)
        )
    if len(data) < message_size:
        data = data + b"\x00" * (message_size - len(data))
    return data


class Publisher:
    """Single producer for a topic; owns (and reclaims) the segment layout."""

    def __init__(self, topic, message_size, message_alignment,
                 slots=MAX_SLOTS):
        self.queue = node_api.RTMSQueue(topic, message_size,
                                        message_alignment, slots,
                                        reclaim_mismatched_segment=True)

    def write(self, payload):
        return self.queue.write(
            _pad_payload(payload, self.queue.message_size))

    def close(self):
        self.queue.close()


class Subscriber:
    """One reader slot on a topic; never reclaims a mismatched segment."""

    def __init__(self, topic, message_size, message_alignment,
                 slots=MAX_SLOTS, overflow_policy=OVERWRITE_OLDEST,
                 read_mode=SEQUENCE):
        self.queue = node_api.RTMSQueue(topic, message_size,
                                        message_alignment, slots)
        self.overflow_policy = overflow_policy
        self.read_mode = read_mode
        self.reader_id = self.queue.register_reader()

    def read_next(self):
        if self.reader_id is None:
            return ("inactive", None, 0, 0)
        return self.queue.read_next(
            self.reader_id, self.overflow_policy, self.read_mode)

    def close(self):
        if self.reader_id is not None:
            self.queue.release_reader(self.reader_id)
            self.reader_id = None
        self.queue.close()
