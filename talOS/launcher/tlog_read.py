"""Binary .tlog header/record reader and node clock helper for Python tools.

Mirrors talOS/events/log/format.h (versions 7 and 8).
"""

import struct
import sys
import time

# talOS/events/log/format.h (versions 7 and 8). EventKind (events/context.h)
# and SourceKind (events/manifest.h) both number TIMER 1; EventKind SEND is 4.
# Only TIMER and SEND records are read, so a GAP record (10, version 8) is
# passed over like any other kind.
FILE = struct.Struct("<8sIIQqqII64sQ")
ENTRY = struct.Struct("<HHIIIqq64sQII")
RECORD = struct.Struct("<IHHQqqqQIIII")
RECORD_MAGIC = 0x54524543
VERSIONS = (7, 8)  # OLDEST_READABLE_VERSION to FORMAT_VERSION
TIMER, SEND = 1, 4


def node_clock_ns():
    """The clock a node stamps its event log with (talOS/events Poller)."""
    clock = time.CLOCK_MONOTONIC_RAW if sys.platform == "darwin" else time.CLOCK_MONOTONIC
    return time.clock_gettime_ns(clock)


def _file_size(stream):
    pos = stream.tell()
    stream.seek(0, 2)
    size = stream.tell()
    stream.seek(pos)
    return size


def read_tlog_header(stream, path):
    """Reads and validates a .tlog file header.

    Returns (header_bytes, origin_ns, manifest_count, process_name, schema_bytes).
    """
    file_size = _file_size(stream)
    header = stream.read(FILE.size)
    if len(header) < FILE.size:
        raise ValueError(f"{path}: truncated file header")
    (magic, version, header_bytes, _, origin, _, count, _, process,
     schema_bytes) = FILE.unpack(header)
    if magic != b"TALOSLOG" or version not in VERSIONS:
        raise ValueError(f"{path}: not a version 7 or 8 .tlog")
    if header_bytes < FILE.size or header_bytes > file_size:
        raise ValueError(f"{path}: invalid header_bytes {header_bytes}")
    manifest_end = header_bytes + count * ENTRY.size
    if manifest_end > file_size:
        raise ValueError(f"{path}: truncated manifest")
    if manifest_end + schema_bytes > file_size:
        raise ValueError(f"{path}: truncated schema section")
    try:
        process_name = process.split(b"\0", 1)[0].decode("utf-8")
    except UnicodeDecodeError:
        raise ValueError(f"{path}: invalid UTF-8 process name") from None
    return header_bytes, origin, count, process_name, schema_bytes


def iter_tlog_records(stream, path, *, allow_truncated=True):
    """Yields (kind, source_id, event_ns, now_ns) from record headers.

    Payloads are skipped by seeking `stored` bytes after each header. When
    `allow_truncated` is True, a truncated or damaged tail ends iteration (as a
    SIGKILLed run leaves); when False, a short record header or truncated
    payload raises ValueError.
    """
    end = _file_size(stream)
    while raw := stream.read(RECORD.size):
        if len(raw) < RECORD.size:
            if allow_truncated:
                break
            raise ValueError(f"truncated log header: {path}")
        magic, kind, source_id, _, event, now, *_, stored = RECORD.unpack(raw)
        if magic != RECORD_MAGIC:
            if allow_truncated:
                break
            raise ValueError(f"bad record magic: {path}")
        if stream.tell() + stored > end:
            if allow_truncated:
                break
            raise ValueError(f"truncated record payload: {path}")
        yield kind, source_id, event, now
        stream.seek(stored, 1)


def read_tlog(path):
    """A .tlog's origin, manifest and record headers; payloads are skipped.

    A truncated or damaged tail ends the records, as a SIGKILLed run leaves.
    """
    with open(path, "rb") as stream:
        header_bytes, origin, count, process, schema_bytes = read_tlog_header(
            stream, path)
        stream.seek(header_bytes)
        sources = {}
        for _ in range(count):
            raw = stream.read(ENTRY.size)
            if len(raw) < ENTRY.size:
                raise ValueError(f"{path}: truncated manifest")
            source_id, kind, _, _, armed, period, offset, name, *_ = ENTRY.unpack(raw)
            try:
                source_name = name.split(b"\0", 1)[0].decode("utf-8")
            except UnicodeDecodeError:
                raise ValueError(f"{path}: invalid UTF-8 source name") from None
            sources[source_id] = {
                "kind": kind,
                "name": source_name,
                "armed": armed != 0,
                "period_ns": period,
                "offset_ns": offset,
            }
        stream.seek(schema_bytes, 1)
        records = list(iter_tlog_records(stream, path, allow_truncated=True))
    return {"process": process, "origin_ns": origin,
            "sources": sources, "records": records}
