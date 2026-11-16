"""What the Studio bridge's Python tests share: a stdlib WebSocket client, the
system graph's fast-value order, and a bridge process's port and stop."""
import base64
import os
import signal
import struct
import subprocess


def receive(sock, count):
    data = b""
    while len(data) < count:
        chunk = sock.recv(count - len(data))
        if not chunk:
            raise EOFError("WebSocket closed")
        data += chunk
    return data


# The socket carries two message families: telemetry frames as binary, and the
# system graph as text. Returning the opcode rather than asserting one keeps
# the caller in charge of which it is waiting for.
TEXT, BINARY = 0x81, 0x82


def frame(sock):
    first, length = receive(sock, 2)
    assert first in (TEXT, BINARY), first
    # A server never masks its frames (RFC 6455 section 5.1).
    assert not length & 0x80, "server frame is masked"
    length &= 127
    if length == 126:
        length = struct.unpack("!H", receive(sock, 2))[0]
    elif length == 127:
        length = struct.unpack("!Q", receive(sock, 8))[0]
    return first, receive(sock, length)


# The system graph's fast values in the order studio/bridge/system.h
# BuildSystemGraph appends them, the one source of that order. An update's
# `values` are written over these slots of the client's document.
# fast_slots_test holds this walk to the bridge's golden files.
def fast_slots(document):
    slots = [(document, "wall_ns")]
    slots += [(document["bridge"], key) for key in
              ("clients", "frames_forwarded", "invalid", "ws_drops", "refused", "egress_bytes")]
    for node in document["nodes"]:
        slots += [(node, key) for key in ("heartbeat_wall_ns", "dispatch_count", "alive")]
        slots += [(node["delivery"], key) for key in
                  ("wake_failures", "stalled_arms", "wake_fallbacks", "requests_skipped")]
        for source in node["sources"]:
            slots += [(source, key) for key in
                      ("events", "dropped", "sequence", "last_monotonic_ns",
                       "last_latency_ns", "max_latency_ns")]
        slots += [(selection, "selected") for selection in node["selections"]]
        if "outputs_held" in node:
            slots.append((node, "outputs_held"))
    for topic in document["topics"]:
        slots += [(topic, key) for key in ("published", "received", "dropped", "health")]
    return slots


def frame_of(sock, opcode, limit=20):
    """The next frame of one family, skipping the other."""
    for _ in range(limit):
        first, payload = frame(sock)
        if first == opcode:
            return payload
    raise AssertionError(f"no frame with opcode {opcode:#x} in {limit} frames")


def bound_port(bridge):
    """The port a bridge started with port 0 reports once it is listening.

    The OS picks the port at bind time, so no other process can take it
    between choosing and listening, as it could a port reserved and closed by
    the test first.
    """
    line = bridge.stdout.readline()
    assert line.startswith("listening "), f"bridge exited before listening: {line!r}"
    return int(line.split()[1])


def stop(processes):
    """SIGTERM every process still running, then reap each (SIGKILL after 5 s).

    All are signalled before any is waited for, so their stops overlap.
    """
    for process in processes:
        if process.poll() is None:
            process.send_signal(signal.SIGTERM)
    for process in reversed(processes):
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        if process.stdout:
            process.stdout.close()


def upgrade(sock, path="/telemetry"):
    """Sends a WebSocket upgrade for `path` and returns the reply headers."""
    key = base64.b64encode(os.urandom(16)).decode()
    sock.sendall((f"GET {path} HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\n"
                  f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\n"
                  f"Sec-WebSocket-Version: 13\r\n\r\n").encode())
    headers = b""
    while not headers.endswith(b"\r\n\r\n"):
        headers += receive(sock, 1)
    return headers


CLOSE = 0x88


def any_frame(sock):
    """The next frame of any kind, close frames included: (opcode, payload)."""
    first, length = receive(sock, 2)
    length &= 127
    if length == 126:
        length = struct.unpack("!H", receive(sock, 2))[0]
    elif length == 127:
        length = struct.unpack("!Q", receive(sock, 8))[0]
    return first, receive(sock, length)
