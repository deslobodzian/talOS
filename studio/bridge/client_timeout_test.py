"""A Studio client that stops answering gives up its slot within the idle timeout."""
import json
import os
from pathlib import Path
import select
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from integration_test import TEXT, bound_port, fast_slots, receive, stop, upgrade

# Control frames (RFC 6455 section 5.5).
PING, PONG = 0x89, 0x8A

# The idle timeout this test runs the bridge with, the smallest uWebSockets
# takes (main.cc kIdleTimeoutS derives the production 16 s). Its margin is
# 4 s: the bridge pings a client that has sent nothing for 8 - 4 s and
# closes it if nothing arrives in the next 4. uSockets checks timeouts every
# 4 s and rounds each phase up to whole checks, so a client that stops
# answering is closed at most 4 + 4 = 8 s after the last data it sent.
IDLE_TIMEOUT_S = 8
# The bridge's 4 s timeout check is a loop timer, and this test sees the
# close when its own wait returns: both may run late on a busy host.
LATE_S = 0.5
# Bounds only a hang.
HANG_S = 10


class Healthy(threading.Thread):
    """A client that reads everything it is sent and answers every ping.

    Runs on its own thread, so it answers pings while the test waits on the
    other clients, and records each graph message's client count.
    """

    def __init__(self, sock):
        super().__init__(daemon=True)
        self.sock = sock
        self.pings = 0
        self.graph = None
        self.counts = []  # (monotonic s, the bridge's client count)
        self.error = None
        self.condition = threading.Condition()

    def message(self):
        """The next data message, answering any ping before it."""
        while True:
            first, length = receive(self.sock, 2)
            length &= 127
            if length == 126:
                length = struct.unpack("!H", receive(self.sock, 2))[0]
            elif length == 127:
                length = struct.unpack("!Q", receive(self.sock, 8))[0]
            payload = receive(self.sock, length) if length else b""
            if first != PING:
                return first, payload
            self.pings += 1
            # A client masks its frames; a zero key leaves the payload as is.
            self.sock.sendall(bytes([PONG, 0x80 | len(payload)]) + bytes(4) + payload)

    def run(self):
        try:
            while True:
                first, payload = self.message()
                if first != TEXT:
                    continue
                document = json.loads(payload)
                if document["kind"] == "talos.system_graph":
                    self.graph = document
                elif document["kind"] == "talos.system_graph.update" and self.graph:
                    for (owner, key), value in zip(fast_slots(self.graph), document["values"]):
                        owner[key] = value
                else:
                    continue
                with self.condition:
                    self.counts.append((time.monotonic(), self.graph["bridge"]["clients"]))
                    self.condition.notify_all()
        except (EOFError, OSError) as error:
            with self.condition:
                self.error = error
                self.condition.notify_all()

    def wait_for(self, predicate):
        """Waits for a recorded (time, count) to satisfy `predicate`; False if closed first."""
        with self.condition:
            self.condition.wait_for(
                lambda: self.error is not None or any(map(predicate, self.counts)), HANG_S)
            return self.error is None and any(map(predicate, self.counts))


def connect(port):
    sock = socket.create_connection(("127.0.0.1", port), timeout=HANG_S)
    return sock, upgrade(sock)


def closed_by_peer(socks, deadline):
    """When the peer closed each socket ({fd: monotonic s}), seen without reading it."""
    closed = {}
    if sys.platform == "darwin":
        queue = select.kqueue()
        queue.control([select.kevent(s.fileno(), select.KQ_FILTER_READ,
                                     select.KQ_EV_ADD | select.KQ_EV_CLEAR) for s in socks], 0)
        while len(closed) < len(socks) and (remaining := deadline - time.monotonic()) > 0:
            for event in queue.control(None, len(socks), remaining):
                if event.flags & select.KQ_EV_EOF:
                    closed.setdefault(event.ident, time.monotonic())
        queue.close()
        return closed
    poller = select.poll()
    for sock in socks:
        poller.register(sock, select.POLLRDHUP)
    while len(closed) < len(socks) and (remaining := deadline - time.monotonic()) > 0:
        for fd, _ in poller.poll(remaining * 1000):
            closed.setdefault(fd, time.monotonic())
            poller.unregister(fd)
    return closed


class ClientTimeout(unittest.TestCase):
    # REQ-OPS-23, REQ-OPS-46
    def test_dead_client_slot_is_reclaimed_and_a_quiet_client_stays(self):
        """Clients that stop answering lose their slots; one answering pings keeps its own.

        A Driver Station laptop that drops off the network leaves sockets
        that never read or answer again. With the four slots full, a fifth
        connection is refused until the bridge closes the dead ones, which
        it must do within the idle timeout, and a client that only receives
        and answers pings must outlive it.
        """
        workspace = Path(os.environ["TEST_SRCDIR"]) / os.environ.get("TEST_WORKSPACE", "_main")
        binaries = workspace / "studio/bridge"
        processes, sockets = [], []
        healthy = None
        with tempfile.TemporaryDirectory() as webroot:
            try:
                # No publisher: the graph alone keeps the healthy client fed.
                bridge = subprocess.Popen(
                    [str(binaries / "studio_bridge"), f"/test/telemetry/i{os.getpid()}",
                     "0", webroot, "--idle-timeout-s", str(IDLE_TIMEOUT_S)],
                    stdout=subprocess.PIPE, text=True)
                processes.append(bridge)
                port = bound_port(bridge)
                opened = time.monotonic()
                sock, headers = connect(port)
                sockets.append(sock)
                self.assertIn(b"101 ", headers)
                healthy = Healthy(sock)
                healthy.start()
                # Three clients that send the upgrade and nothing more, and
                # never read: the bridge's sends land in their receive buffers.
                dead = []
                for _ in range(3):
                    last_data = time.monotonic()
                    sock, headers = connect(port)
                    sockets.append(sock)
                    dead.append(sock)
                    self.assertIn(b"101 ", headers)
                self.assertTrue(healthy.wait_for(lambda count: count[1] == 4), healthy.error)
                sock, headers = connect(port)
                sock.close()
                self.assertIn(b"503 ", headers, "four clients should fill the default limit")

                # The bridge closes every dead client within the bound.
                closed = closed_by_peer(dead, last_data + IDLE_TIMEOUT_S + LATE_S)
                self.assertEqual(len(closed), len(dead),
                                 f"{len(dead) - len(closed)} dead clients still open "
                                 f"{time.monotonic() - last_data:.2f} s after their last "
                                 f"data; bound {IDLE_TIMEOUT_S} s idle timeout + {LATE_S} s late")
                print(f"dead clients closed {max(closed.values()) - last_data:.2f} s after "
                      f"their last data", flush=True)
                sock, headers = connect(port)
                sockets.append(sock)
                self.assertIn(b"101 ", headers, "a fifth connection should take a freed slot")

                # The healthy client, which sent nothing but pongs since its
                # upgrade, is still sent graphs after the bound has passed.
                self.assertTrue(healthy.wait_for(
                    lambda count: count[0] > opened + IDLE_TIMEOUT_S + LATE_S),
                    f"healthy client closed: {healthy.error}")
                self.assertGreaterEqual(healthy.pings, 1, "the bridge never pinged")
            finally:
                for sock in sockets:
                    try:
                        sock.shutdown(socket.SHUT_RDWR)
                    except OSError:
                        pass
                    sock.close()
                if healthy:
                    healthy.join(HANG_S)
                stop(processes)
                self.assertEqual(bridge.returncode, 0)


if __name__ == "__main__":
    unittest.main()
