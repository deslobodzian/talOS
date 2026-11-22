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
from bridge_client import TEXT, bound_port, fast_slots, receive, stop, upgrade

# Control frames (RFC 6455 section 5.5).
PING, PONG = 0x89, 0x8A

# The production idle timeout, which this test runs the bridge with (main.cc
# kIdleTimeoutS, REQ-OPS-46's 16 s). Its margin is 8 s: the bridge pings a
# client that has sent nothing for 16 - 8 s and closes it if nothing arrives
# in the next 8. uSockets checks timeouts every 4 s and rounds each phase up
# to whole checks, so a client that stops answering is closed 12 to 16 s
# after the last data it sent. A bridge whose default were smaller closes
# sooner than EARLIEST_CLOSE_S (12 s less 1 s of slack).
IDLE_TIMEOUT_S = 16
EARLIEST_CLOSE_S = 11
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


def connect(port, path="/telemetry"):
    sock = socket.create_connection(("127.0.0.1", port), timeout=HANG_S)
    return sock, upgrade(sock, path)


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
    # REQ-OPS-23, REQ-OPS-36, REQ-OPS-46
    def test_dead_client_slot_is_reclaimed_and_a_quiet_client_stays(self):
        """Clients that stop answering lose their slots; ones answering pings keep theirs.

        A Driver Station laptop that drops off the network leaves sockets
        that never read or answer again. With the four slots full, a fifth
        connection is refused until the bridge closes the dead ones, which
        it must do within the default 16 s idle timeout (and not sooner), and
        a client that only receives and answers pings must outlive it. The
        same holds for /drive clients, which take no telemetry slot.
        """
        workspace = Path(os.environ["TEST_SRCDIR"]) / os.environ.get("TEST_WORKSPACE", "_main")
        binaries = workspace / "studio/bridge"
        processes, sockets = [], []
        healthy = drive = None
        with tempfile.TemporaryDirectory() as webroot, \
                tempfile.TemporaryDirectory() as configs:
            Path(configs, "robot.toml").write_text(
                "[robot]\nperiod_us = 5000\n\n[studio]\ndashboard = \"dashboard.toml\"\n\n"
                "[subsystems.alpha]\nnode = \"//test:alpha\"\ncritical = false\n")
            Path(configs, "dashboard.toml").write_text(
                "[dashboard]\ntitle = \"Pit\"\ndrive_clients = 2\n\n"
                "[[widgets]]\nkind = \"number\"\nat = [0, 0, 2, 1]\n"
                "channel = \"drive/velocity\"\n")
            try:
                # No publisher: the graph alone keeps the healthy client fed.
                bridge = subprocess.Popen(
                    [str(binaries / "studio_bridge"), f"/test/telemetry/i{os.getpid()}",
                     "0", webroot, "--config", str(Path(configs, "robot.toml"))],
                    stdout=subprocess.PIPE, text=True)
                processes.append(bridge)
                port = bound_port(bridge)
                opened = time.monotonic()
                sock, headers = connect(port)
                sockets.append(sock)
                self.assertIn(b"101 ", headers)
                healthy = Healthy(sock)
                healthy.start()
                # A /drive client that answers pings, outside the four slots.
                sock, headers = connect(port, "/drive")
                sockets.append(sock)
                self.assertIn(b"101 ", headers)
                drive = Healthy(sock)
                drive.start()
                # Three telemetry clients and one /drive client that send the
                # upgrade and nothing more, and never read: the bridge's sends
                # land in their receive buffers.
                dead = []
                for path in ("/telemetry",) * 3 + ("/drive",):
                    last_data = time.monotonic()
                    sock, headers = connect(port, path)
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
                self.assertGreaterEqual(
                    min(closed.values()) - last_data, EARLIEST_CLOSE_S,
                    "a dead client closed before the 16 s default idle timeout")
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
                self.assertIsNone(drive.error, f"/drive client closed: {drive.error}")
                self.assertGreaterEqual(drive.pings, 1, "the bridge never pinged /drive")
            finally:
                # Shut down, join, then close: a shutdown wakes the healthy
                # thread's recv at once, but on macOS closing the descriptor
                # under it does not, and a close racing its wake left it
                # waiting out the socket's HANG_S timeout.
                for sock in sockets:
                    try:
                        sock.shutdown(socket.SHUT_RDWR)
                    except OSError:
                        pass
                for reader in (healthy, drive):
                    if reader:
                        reader.join(HANG_S)
                for sock in sockets:
                    sock.close()
                stop(processes)
                self.assertEqual(bridge.returncode, 0)


if __name__ == "__main__":
    unittest.main()
