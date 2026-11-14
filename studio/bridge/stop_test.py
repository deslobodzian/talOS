"""The bridge stops well inside the launcher's stop timeout whatever its clients do."""
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import tempfile
import time
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from integration_test import bound_port, receive, upgrade

# The launcher SIGKILLs a member still alive kStopTimeout (2 s,
# talOS/launcher/launcher.h; REQ-OPS-06) after its stop signal, kStopSignal
# (SIGINT). The bridge's stop is one loop pass that waits on no peer and no
# timer, so it is bounded at a quarter of that, leaving the rest to a late
# host. The bound is also half of uWebSockets' one-second date timer, the
# shortest loop timer a stop could otherwise wait for: a bridge started just
# before its signal next sees that timer about a second later, so a stop that
# waits for the loop's next timer fails here.
STOP_BOUND_S = 0.5
# Bounds only a hang, and lets a held stop report how long it took: uWS
# closes an idle HTTP connection 10 s after its last data, swept every 4 s.
HANG_S = 20


class BridgeStop(unittest.TestCase):
    def setUp(self):
        workspace = Path(os.environ["TEST_SRCDIR"]) / os.environ.get("TEST_WORKSPACE", "_main")
        webroot = tempfile.TemporaryDirectory()
        self.addCleanup(webroot.cleanup)
        Path(webroot.name, "index.html").write_text("<!doctype html>\n")
        self.sockets = []
        # No publisher: the bridge serves HTTP and waits for the segment.
        self.bridge = subprocess.Popen(
            [str(workspace / "studio/bridge/studio_bridge"),
             f"/test/telemetry/s{os.getpid()}", "0", webroot.name],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        self.addCleanup(self.reap)
        # "listening" is printed after the stop handlers are installed.
        self.port = bound_port(self.bridge)

    def reap(self):
        for sock in self.sockets:
            sock.close()
        if self.bridge.poll() is None:
            self.bridge.kill()
        self.bridge.communicate()

    def connect(self):
        sock = socket.create_connection(("127.0.0.1", self.port), timeout=HANG_S)
        self.sockets.append(sock)
        return sock

    def accepted(self):
        """Returns once the bridge has accepted every earlier connection.

        It accepts in order on one thread, so a later connection's answered
        request means the earlier ones are accepted. That connection asks to
        be closed and is read to its end, so it holds nothing open itself.
        """
        probe = self.connect()
        probe.sendall(b"GET /system.json HTTP/1.1\r\nHost: localhost\r\n"
                      b"Connection: close\r\n\r\n")
        reply = b""
        while chunk := probe.recv(4096):
            reply += chunk
        self.assertTrue(reply.startswith(b"HTTP/1.1 200"), reply[:40])

    def assert_stops(self, sent_at=None):
        """SIGINTs the bridge (unless sent at `sent_at`) and requires a clean exit in the bound."""
        if sent_at is None:
            sent_at = time.monotonic()
            self.bridge.send_signal(signal.SIGINT)
        try:
            self.bridge.wait(timeout=HANG_S)
        except subprocess.TimeoutExpired:
            self.fail(f"the bridge did not stop within {HANG_S} s of SIGINT")
        elapsed_s = time.monotonic() - sent_at
        print(f"{self.id()}: stopped {elapsed_s:.3f} s after SIGINT", flush=True)
        self.assertLessEqual(elapsed_s, STOP_BOUND_S,
                             f"stopped {elapsed_s:.3f} s after SIGINT; bound {STOP_BOUND_S} s "
                             f"(a quarter of the launcher's 2 s stop timeout)")
        self.assertEqual(self.bridge.returncode, 0, self.bridge.stderr.read())

    # REQ-OPS-06, REQ-OPS-30
    def test_idle_keep_alive_connection(self):
        """A page's keep-alive connection, idle after a complete request, does not hold the stop."""
        sock = self.connect()
        sock.sendall(b"GET / HTTP/1.1\r\nHost: localhost\r\n\r\n")
        headers = b""
        while not headers.endswith(b"\r\n\r\n"):
            headers += receive(sock, 1)
        self.assertIn(b"200 OK", headers)
        length = next(int(line.split(b":")[1]) for line in headers.split(b"\r\n")
                      if line.lower().startswith(b"content-length:"))
        receive(sock, length)
        self.assert_stops()

    # REQ-OPS-06, REQ-OPS-30
    def test_connection_that_sent_nothing(self):
        """A connection the bridge accepted that never sent a byte does not hold the stop."""
        self.connect()
        self.accepted()
        self.assert_stops()

    # REQ-OPS-06, REQ-OPS-30
    def test_half_sent_request(self):
        """A request whose headers never finish does not hold the stop."""
        sock = self.connect()
        sock.sendall(b"GET / HTTP/1.1\r\nHost: localhost\r\n")
        self.accepted()
        self.assert_stops()

    # REQ-OPS-06, REQ-OPS-30
    def test_websocket_that_ignores_the_close(self):
        """A WebSocket client that never reads or answers a close does not hold the stop."""
        sock = self.connect()
        self.assertIn(b"101 ", upgrade(sock))
        self.assert_stops()

    # REQ-OPS-06, REQ-OPS-30
    def test_connect_arriving_with_the_stop(self):
        """A connection still in the listener's backlog when the stop lands does not hold it.

        The bridge is suspended while the connection completes in the
        listener's backlog and the stop signal is queued, so the stop and the
        connection are both pending when it resumes.
        """
        self.bridge.send_signal(signal.SIGSTOP)
        os.waitid(os.P_PID, self.bridge.pid, os.WSTOPPED)
        self.connect()
        sent_at = time.monotonic()
        self.bridge.send_signal(signal.SIGINT)
        self.bridge.send_signal(signal.SIGCONT)
        self.assert_stops(sent_at)


if __name__ == "__main__":
    unittest.main()
