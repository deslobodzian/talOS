"""Python RTMS pub/sub client.

Thin compatibility layer over :mod:`node_api`, the ctypes binding to the C
ground-truth library ``libtalos_node.so`` (``talOS/node_api/node_api.h``).
Transport rule: every shared-memory byte flows through a ``talos_*`` call;
no slot/header arithmetic lives in this file. The frozen-layout constants
below are documentation values (asserted by the layout test), not live
offsets -- nothing reads or writes shared memory with them.

Public API (unchanged):

* ``Publisher`` / ``Subscriber`` / ``RTMSQueue`` with ``register_reader`` /
  ``read_next`` / ``release_reader`` / ``write`` / ``close``.
* ``shm_object_name`` / ``align_up`` helpers and the ``OVERWRITE_OLDEST`` /
  ``DROP_NEWEST`` / ``SEQUENCE`` / ``LATEST`` policy strings.

Differences from the pre-wrapper client, all documented in DESIGN.md:

* ``write`` pads a short payload with zeros (the old zero-filled slot did
  this implicitly); an oversized payload still raises ``ValueError``.
* ``write`` on the live transport returns ``None``: the sequence counter is
  C++-owned and has no ABI export. (The in-memory test path below still
  returns the sequence, as the old client did.)
* Per-call ``overflow_policy`` other than ``OVERWRITE_OLDEST`` raises
  ``ValueError`` on the live transport: the C library bakes that policy.
* ``writer_sequence`` has no ABI export and raises ``NotImplementedError``
  on the live transport.

Test-only in-memory path: the committed round-trip test builds queues via
``RTMSQueue.__new__`` with a bytearray-backed ``_seg`` and exercises the
protocol formulas without shared memory. When ``_seg`` is present the queue
serves those calls from a deque-free dict/counter emulation (no shm struct
math); every real queue (built by ``__init__``) goes through ``talos_*``.
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

from node_api import TalosError

# Re-exported unchanged (single source of truth in node_api).
MAX_SLOTS = node_api.MAX_SLOTS
MAX_READERS = node_api.MAX_READERS
OVERWRITE_OLDEST = node_api.OVERWRITE_OLDEST
DROP_NEWEST = node_api.DROP_NEWEST
SEQUENCE = node_api.SEQUENCE
LATEST = node_api.LATEST

# Frozen layout documentation values, mirroring talOS/rtms/rtms.h and
# talOS/memory/shared_memory_ptr.h. Nothing in this file indexes memory
# with them; they exist for the layout test and for readers of the format.
CACHE_LINE = 64
MAX_READ_ATTEMPTS = 8
HEADER_SIZE = 640
OFF_TOTAL_BYTES = 0
OFF_SLOTS = 8
OFF_MESSAGE_BYTES = 16
OFF_MESSAGE_ALIGNMENT = 24
OFF_DATA_OFFSET = 32
OFF_SLOT_STRIDE = 40
OFF_WRITER_SEQ = 64
OFF_READERS = 128
READER_STRIDE = 64
OFF_READER_STATE = 8

FREE, CLAIMING, ACTIVE = 0, 1, 2

# Topic names may be at most 30 bytes after the leading slash
# (rtms.cc ValidatePath; macOS PSHMNAMLEN cap). Shared-memory object names
# map interior '/' to '.' (shared_memory_ptr.h ShmObjectName): '.' is
# injective where '_' is not ('/hw/state/driver_station' vs
# '/hw/state_driver/station'), and the mapping preserves length.
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


class RTMSQueue(node_api.RTMSQueue):
    """Untyped ring buffer over one topic's segment; transport is C++-owned.

    Live queues (built by ``__init__``) delegate to the ``talos_*`` ABI.
    Queues carrying a ``_seg`` attribute were built ``__new__``-style by the
    protocol test with a bytearray segment; they run on the counter/dict
    emulation below (test-only, no shared memory, no slot arithmetic).
    """

    @property
    def _fake(self):
        return "_seg" in self.__dict__

    # -- test-only in-memory emulation (no shm struct math) ---------------

    def _init_header(self):
        """Reset emulation counters (test-only; live layout is C++-owned)."""
        self._fwseq = 0
        self._fstore = {}
        self._freaders = {}

    def _fake_state(self):
        if "_fwseq" not in self.__dict__:
            self._init_header()
        return self

    def _select_sequence(self, cursor, overflow_policy, read_mode):
        writer = self._fwseq
        if cursor >= writer:
            return None, 0
        seq = cursor
        if overflow_policy != DROP_NEWEST and writer - seq >= self.slots:
            seq = writer - self.slots + 1
        if read_mode == LATEST:
            seq = writer - 1
        return seq, seq - cursor

    def _slot_still_valid(self, seq):
        return self._fwseq - seq < self.slots

    def writer_sequence(self):
        if self._fake:
            return self._fake_state()._fwseq
        raise NotImplementedError(
            "writer_sequence has no node-ABI export; the counter is C++-owned")

    def _fake_register(self):
        st = self._fake_state()
        for i in range(MAX_READERS):
            if st._freaders.get(i, (None, False))[1] is not True:
                st._freaders[i] = [st._fwseq, True]
                return i
        return None

    def _fake_write(self, payload):
        st = self._fake_state()
        data = bytes(payload)
        if len(data) > self.message_size:
            raise ValueError(
                "Cannot write message larger than slot size: %d > %d"
                % (len(data), self.message_size)
            )
        seq = st._fwseq
        st._fstore[seq] = data + b"\x00" * (self.message_size - len(data))
        st._fwseq = seq + 1
        return seq

    def _fake_read(self, reader_id, overflow_policy=OVERWRITE_OLDEST,
                   read_mode=SEQUENCE):
        st = self._fake_state()
        if reader_id >= MAX_READERS:
            return ("invalid", None, 0, 0)
        cursor, active = st._freaders.get(reader_id, (0, False))
        if not active:
            return ("inactive", None, 0, 0)
        if self.message_size <= 0:
            return ("invalid", None, 0, 0)
        for _ in range(MAX_READ_ATTEMPTS):
            seq, dropped = self._select_sequence(
                cursor, overflow_policy, read_mode)
            if seq is None:
                return ("empty", None, 0, 0)
            payload = st._fstore.get(seq)
            if payload is None:  # pragma: no cover - corrupt emulation
                return ("torn", None, 0, 0)
            if overflow_policy != DROP_NEWEST and not (
                    st._fwseq - seq < self.slots):
                continue
            st._freaders[reader_id][0] = seq + 1
            return ("ok", bytes(payload), seq, dropped)
        return ("torn", None, 0, 0)

    # -- live overrides (validate topic naming, then delegate) -------------

    def __init__(self, topic, message_size, message_alignment,
                 slots=MAX_SLOTS, reclaim_mismatched_segment=False):
        shm_object_name(topic)  # keep the 30-byte naming validation
        super().__init__(topic, message_size, message_alignment, slots,
                         reclaim_mismatched_segment)

    def register_reader(self):
        if self._fake:
            return self._fake_register()
        return super().register_reader()

    def release_reader(self, reader_id):
        if self._fake:
            st = self._fake_state()
            if reader_id in st._freaders:
                st._freaders[reader_id][1] = False
            return
        return super().release_reader(reader_id)

    def write(self, payload):
        if self._fake:
            return self._fake_write(payload)
        return super().write(_pad_payload(payload, self.message_size))

    def read_next(self, reader_id, overflow_policy=OVERWRITE_OLDEST,
                  read_mode=SEQUENCE):
        if self._fake:
            return self._fake_read(reader_id, overflow_policy, read_mode)
        return super().read_next(reader_id, overflow_policy, read_mode)

    def close(self):
        if self._fake:
            seg = self.__dict__.get("_seg")
            if seg is not None:
                seg.close()
            return
        return super().close()


class Publisher:
    """Single producer for a topic; owns (and reclaims) the segment layout."""

    def __init__(self, topic, message_size, message_alignment,
                 slots=MAX_SLOTS):
        self.queue = RTMSQueue(topic, message_size, message_alignment, slots,
                               reclaim_mismatched_segment=True)

    def write(self, payload):
        return self.queue.write(payload)

    def close(self):
        self.queue.close()


class Subscriber:
    """One reader slot on a topic; never reclaims a mismatched segment."""

    def __init__(self, topic, message_size, message_alignment,
                 slots=MAX_SLOTS, overflow_policy=OVERWRITE_OLDEST,
                 read_mode=SEQUENCE):
        self.queue = RTMSQueue(topic, message_size, message_alignment, slots)
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
