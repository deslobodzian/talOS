"""Python RTMS pub/sub client.

Mirrors the frozen shared-memory layout owned by ``talOS/rtms/rtms.h`` and
``talOS/memory/shared_memory_ptr.h`` (PLAN section 3.4: read, never change).
Transport only: payload encode/decode lives in ``messages.py`` on top of
``flatc --python`` bindings generated from the same ``.fbs`` as the C++ peer.

Layout (all integers little-endian ``<Q``; verified against a C++ probe of
``sizeof``/``offsetof`` on this repo's headers)::

    offset  size  field
    0       8     total_bytes
    8       8     slots (power of two)
    16      8     message_bytes
    24      8     message_alignment
    32      8     data_offset (== align_up(640, message_alignment))
    40      8     slot_stride (== align_up(message_bytes, message_alignment))
    48      16    padding (Writer is alignas(64), starts at 64)
    64      8     writer.sequence, then 56 bytes padding (sizeof(Writer) == 64)
    128+i*64     reader[i].sequence (u64 @ +0), reader[i].state (u64 @ +8),
                 FREE=0 / CLAIMING=1 / ACTIVE=2, padded to 64 bytes each

    data      message slots start at data_offset; slot for sequence s lives at
              data_offset + (s & (slots - 1)) * slot_stride

Sequence protocol (same as ``RTMSQueue``): the writer counter holds the count
of messages published, so the next message takes sequence ``writer`` and the
store publishes it as ``writer + 1`` (release). A reader cursor holds the next
sequence it wants; ``register_reader`` snapshots the current writer counter.
"""

import ctypes
import ctypes.util
import errno
import fcntl
import mmap
import os
import struct
import sys
import tempfile

CACHE_LINE = 64
MAX_SLOTS = 1024
MAX_READERS = 8
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

OVERWRITE_OLDEST = "overwrite_oldest"
DROP_NEWEST = "drop_newest"
SEQUENCE = "sequence"
LATEST = "latest"

_U64 = struct.Struct("<Q")


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


def _shm_open(name, oflag, mode=0o666):
    libc = _libc()
    libc.shm_open.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_int]
    libc.shm_open.restype = ctypes.c_int
    fd = libc.shm_open(name.encode(), oflag, mode)
    if fd == -1:
        errno = ctypes.get_errno()
        raise OSError(errno, "shm_open %s: %s" % (name, os.strerror(errno)))
    return fd


def _shm_unlink(name):
    libc = _libc()
    libc.shm_unlink.argtypes = [ctypes.c_char_p]
    libc.shm_unlink.restype = ctypes.c_int
    libc.shm_unlink(name.encode())


class _ShmSegment:
    """One mapped shared-memory object, created or attached like SharedMemoryPtr.

    Linux opens /dev/shm/<name> directly; other POSIX systems (macOS) go
    through libc shm_open. Creation uses O_CREAT|O_EXCL so exactly one peer
    wins, matching SharedMemoryPtr; attach refuses a segment smaller than the
    layout needs, matching its fstat guard.
    """

    def __init__(self, shm_name, size):
        self.shm_name = shm_name
        self.size = size
        self.created = False
        fd = self._open()
        try:
            if self.created:
                os.ftruncate(fd, size)
            else:
                st_size = os.fstat(fd).st_size
                if st_size < size:
                    raise RuntimeError(
                        "shared memory %s is %d bytes, need %d; it was created "
                        "by a process built against a different message layout"
                        % (shm_name, st_size, size)
                    )
            self.buf = mmap.mmap(
                fd, size, flags=mmap.MAP_SHARED, prot=mmap.PROT_READ | mmap.PROT_WRITE
            )
        finally:
            os.close(fd)

    def _open(self):
        if sys.platform.startswith("linux") and os.path.isdir("/dev/shm"):
            path = "/dev/shm/" + self.shm_name.lstrip("/")
            try:
                fd = os.open(path, os.O_CREAT | os.O_EXCL | os.O_RDWR, 0o666)
                self.created = True
                return fd
            except FileExistsError:
                return os.open(path, os.O_RDWR)
        flags = os.O_CREAT | os.O_EXCL | os.O_RDWR
        try:
            fd = _shm_open(self.shm_name, flags)
            self.created = True
            return fd
        except OSError as exc:
            if exc.errno != errno.EEXIST:
                raise
            return _shm_open(self.shm_name, os.O_RDWR)

    def close(self):
        self.buf.close()

    @staticmethod
    def unlink(shm_name):
        if sys.platform.startswith("linux") and os.path.isdir("/dev/shm"):
            try:
                os.unlink("/dev/shm/" + shm_name.lstrip("/"))
            except FileNotFoundError:
                pass
        else:
            _shm_unlink(shm_name)


class RTMSQueue:
    """Untyped ring buffer over one topic's segment; mirrors RTMSQueue."""

    def __init__(self, topic, message_size, message_alignment, slots=MAX_SLOTS,
                 reclaim_mismatched_segment=False):
        if slots < 2 or (slots & (slots - 1)):
            raise ValueError("slots must be a power of 2 >= 2")
        self.topic = topic
        self.shm_name = shm_object_name(topic)
        self.message_size = message_size
        self.message_alignment = message_alignment
        self.slots = slots
        self.data_offset = align_up(HEADER_SIZE, message_alignment)
        self.stride = align_up(message_size, message_alignment)
        self.total_bytes = self.data_offset + self.stride * slots
        seg = _ShmSegment(self.shm_name, self.total_bytes)
        if not seg.created and not self._layout_matches(seg.buf):
            # Publisher owns the topic layout, so it reclaims a segment left
            # behind by an older build; a reader reports the skew instead
            # (RTMSOptions::reclaim_mismatched_segment).
            if not reclaim_mismatched_segment:
                seg.close()
                raise RuntimeError(
                    "RTMS shared-memory layout mismatch on %s: this process "
                    "was built against a different message layout than the "
                    "publisher that created the segment" % self.shm_name
                )
            seg.close()
            _ShmSegment.unlink(self.shm_name)
            seg = _ShmSegment(self.shm_name, self.total_bytes)
        self._seg = seg
        if seg.created:
            self._init_header()

    def _u64(self, off):
        return _U64.unpack_from(self._seg.buf, off)[0]

    def _set_u64(self, off, value):
        _U64.pack_into(self._seg.buf, off, value)

    def _layout_matches(self, buf):
        want = (
            self.total_bytes,
            self.slots,
            self.message_size,
            self.message_alignment,
        )
        got = (
            _U64.unpack_from(buf, OFF_TOTAL_BYTES)[0],
            _U64.unpack_from(buf, OFF_SLOTS)[0],
            _U64.unpack_from(buf, OFF_MESSAGE_BYTES)[0],
            _U64.unpack_from(buf, OFF_MESSAGE_ALIGNMENT)[0],
        )
        return want == got

    def _init_header(self):
        buf = self._seg.buf
        for i in range(self.total_bytes):
            buf[i] = 0
        self._set_u64(OFF_TOTAL_BYTES, self.total_bytes)
        self._set_u64(OFF_SLOTS, self.slots)
        self._set_u64(OFF_MESSAGE_BYTES, self.message_size)
        self._set_u64(OFF_MESSAGE_ALIGNMENT, self.message_alignment)
        self._set_u64(OFF_DATA_OFFSET, self.data_offset)
        self._set_u64(OFF_SLOT_STRIDE, self.stride)
        # Writer sequence and all reader slots are already zero (FREE).

    def close(self):
        self._seg.close()

    def writer_sequence(self):
        return self._u64(OFF_WRITER_SEQ)

    def _reader_base(self, reader_id):
        return OFF_READERS + reader_id * READER_STRIDE

    def _slot_address(self, sequence):
        return self.data_offset + (sequence & (self.slots - 1)) * self.stride

    @staticmethod
    def _lock_path(shm_name):
        digest = shm_name.lstrip("/").replace(".", "_")
        return os.path.join(tempfile.gettempdir(), "rtms-%s.lock" % digest)

    def register_reader(self):
        """Claim a FREE reader slot; the cursor starts at the live writer."""
        with open(self._lock_path(self.shm_name), "a+b") as lock:
            fcntl.flock(lock.fileno(), fcntl.LOCK_EX)
            try:
                for i in range(MAX_READERS):
                    base = self._reader_base(i)
                    if self._u64(base + OFF_READER_STATE) != FREE:
                        continue
                    self._set_u64(base + OFF_READER_STATE, CLAIMING)
                    self._set_u64(base, self.writer_sequence())
                    self._set_u64(base + OFF_READER_STATE, ACTIVE)
                    return i
            finally:
                fcntl.flock(lock.fileno(), fcntl.LOCK_UN)
        return None

    def release_reader(self, reader_id):
        self._set_u64(self._reader_base(reader_id) + OFF_READER_STATE, FREE)

    def write(self, payload):
        """Publish one message; returns its sequence number."""
        if len(payload) > self.message_size:
            raise ValueError(
                "Cannot write message larger than slot size: %d > %d"
                % (len(payload), self.message_size)
            )
        writer = self.writer_sequence()
        addr = self._slot_address(writer)
        self._seg.buf[addr:addr + len(payload)] = payload
        # Release: slot bytes are visible before the sequence bump.
        self._set_u64(OFF_WRITER_SEQ, writer + 1)
        return writer

    def _select_sequence(self, cursor, overflow_policy, read_mode):
        writer = self.writer_sequence()
        if cursor >= writer:
            return None, 0
        seq = cursor
        if overflow_policy != DROP_NEWEST and writer - seq >= self.slots:
            seq = writer - self.slots + 1
        if read_mode == LATEST:
            seq = writer - 1
        return seq, seq - cursor

    def _slot_still_valid(self, seq):
        writer = self.writer_sequence()
        return writer - seq < self.slots

    def read_next(self, reader_id, overflow_policy=OVERWRITE_OLDEST,
                  read_mode=SEQUENCE):
        """Copy the reader's next message.

        Returns (status, payload, sequence, dropped) with status in
        ok/empty/inactive/invalid/torn, mirroring RTMSQueue::read_next
        including the 8-attempt seqlock lap recovery.
        """
        if reader_id >= MAX_READERS:
            return ("invalid", None, 0, 0)
        base = self._reader_base(reader_id)
        if self._u64(base + OFF_READER_STATE) != ACTIVE:
            return ("inactive", None, 0, 0)
        if self.message_size <= 0:
            return ("invalid", None, 0, 0)
        for _ in range(MAX_READ_ATTEMPTS):
            cursor = self._u64(base)
            selected = self._select_sequence(cursor, overflow_policy, read_mode)
            if selected[0] is None:
                return ("empty", None, 0, 0)
            seq, dropped = selected
            addr = self._slot_address(seq)
            payload = bytes(self._seg.buf[addr:addr + self.message_size])
            if overflow_policy != DROP_NEWEST and not self._slot_still_valid(seq):
                continue
            self._set_u64(base, seq + 1)
            return ("ok", payload, seq, dropped)
        return ("torn", None, 0, 0)


class Publisher:
    """Single producer for a topic; owns (and reclaims) the segment layout."""

    def __init__(self, topic, message_size, message_alignment, slots=MAX_SLOTS):
        self.queue = RTMSQueue(topic, message_size, message_alignment, slots,
                               reclaim_mismatched_segment=True)

    def write(self, payload):
        return self.queue.write(payload)

    def close(self):
        self.queue.close()


class Subscriber:
    """One reader slot on a topic; never reclaims a mismatched segment."""

    def __init__(self, topic, message_size, message_alignment, slots=MAX_SLOTS,
                 overflow_policy=OVERWRITE_OLDEST, read_mode=SEQUENCE):
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
