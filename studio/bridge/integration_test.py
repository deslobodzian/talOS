"""Exercise real IPC -> UDP/WebSocket bytes without a third-party WS client."""
import base64
import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import tempfile
import time
import unittest
import urllib.error
import urllib.request


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
    length &= 127
    if length == 126:
        length = struct.unpack("!H", receive(sock, 2))[0]
    elif length == 127:
        length = struct.unpack("!Q", receive(sock, 8))[0]
    return first, receive(sock, length)


def frame_of(sock, opcode, limit=20):
    """The next frame of one family, skipping the other."""
    for _ in range(limit):
        first, payload = frame(sock)
        if first == opcode:
            return payload
    raise AssertionError(f"no frame with opcode {opcode:#x} in {limit} frames")


class BridgeIntegration(unittest.TestCase):
    def test_transports(self):
        workspace = Path(os.environ["TEST_SRCDIR"]) / os.environ.get("TEST_WORKSPACE", "_main")
        binaries = workspace / "studio/bridge"
        # A multi-segment name, like every real topic (talOS/NAMING.md).
        #
        # This was a flat "/studio_test_<pid>", which meant the test never
        # exercised the one thing that makes a real topic name special: the
        # interior slash. POSIX does not allow it in a shared-memory name, so
        # RTMS maps the topic to a segment name, and a direct shm_open on the
        # unmapped topic in the bridge's publisher pre-check went unnoticed
        # because no test used a name that needed mapping. Keep this shaped
        # like a real topic.
        topic = f"/test/telemetry/p{os.getpid()}"
        processes = []
        with tempfile.TemporaryDirectory() as webroot, \
                tempfile.TemporaryDirectory() as outputs, \
                socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
            Path(webroot, "index.html").write_text("studio integration")
            # The launcher's declared graph, deliberately outside the webroot:
            # a copy inside it would be served by the static file handler at the
            # same path, and the test could not tell the route from the file.
            #
            # The bridge serves these bytes verbatim and never parses them, so
            # the content only has to be JSON. Asserting on a real graph.json
            # shape here would be testing the launcher.
            declared = Path(outputs, "graph.json")
            declared_bytes = (b'{"session_id":"7","config_path":"robot.toml",'
                              b'"nodes":[{"name":"drivetrain",'
                              b'"target":"//2026-robot/main_processor/drivetrain:node",'
                              b'"sources":[{"kind":"SENDER","name":"/drivetrain/state",'
                              b'"message_bytes":48,"external":false,"optional":false}]}],'
                              b'"diagnostics":[]}')
            declared.write_bytes(declared_bytes)
            udp.bind(("127.0.0.1", 0))
            udp.settimeout(5)
            # A second destination nobody reads, for the bridge that runs
            # without --declared: two bridges aiming at one port would
            # interleave chunks of two documents that share a document id, and
            # the reassembly below would splice them into nonsense.
            quiet_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            quiet_udp.bind(("127.0.0.1", 0))
            with socket.socket() as reservation:
                reservation.bind(("127.0.0.1", 0))
                http_port = reservation.getsockname()[1]
            with socket.socket() as reservation:
                reservation.bind(("127.0.0.1", 0))
                bare_port = reservation.getsockname()[1]
            try:
                mock = subprocess.Popen([str(binaries / "studio_mock"), topic])
                processes.append(mock)
                # Creation of the shared-memory header precedes the first HTTP bind.
                time.sleep(0.2)
                bridge = subprocess.Popen([str(binaries / "studio_bridge"), "--drop-newest-publisher", topic,
                                           "127.0.0.1", str(udp.getsockname()[1]), str(http_port), webroot,
                                           "--declared", str(declared)])
                processes.append(bridge)
                # The same bridge with no declared graph, to pin down what
                # /declared.json does then. Started here so both are up while
                # the deadline loop below waits.
                bare = subprocess.Popen([str(binaries / "studio_bridge"), "--drop-newest-publisher", topic,
                                         "127.0.0.1", str(quiet_udp.getsockname()[1]), str(bare_port), webroot])
                processes.append(bare)
                deadline = time.monotonic() + 10
                while True:
                    self.assertIsNone(mock.poll(), "mock exited")
                    self.assertIsNone(bridge.poll(), "bridge exited")
                    self.assertIsNone(bare.poll(), "bridge without --declared exited")
                    try:
                        ws = socket.create_connection(("127.0.0.1", http_port), timeout=1)
                        break
                    except OSError:
                        if time.monotonic() > deadline:
                            raise
                        time.sleep(0.02)
                with urllib.request.urlopen(f"http://127.0.0.1:{http_port}/", timeout=3) as response:
                    self.assertEqual(response.read(), b"studio integration")
                with ws:
                    ws.settimeout(5)
                    key = base64.b64encode(os.urandom(16)).decode()
                    ws.sendall((f"GET /telemetry HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\n"
                                f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n").encode())
                    headers = b""
                    while not headers.endswith(b"\r\n\r\n"):
                        headers += receive(ws, 1)
                    self.assertIn(b"101 Switching Protocols", headers)
                    # A client is sent the system graph on connect, so it does
                    # not open on an empty view while waiting for the next
                    # refresh.
                    graph = json.loads(frame_of(ws, TEXT).decode())
                    self.assertEqual(graph["kind"], "talos.system_graph")
                    self.assertEqual(graph["bridge"]["topic"], topic)
                    # The bridge holds a reader slot on the topic, so it must
                    # appear as a subscriber of it rather than leaving the topic
                    # reading as one nobody consumes.
                    self.assertIn("studio_bridge", [n["name"] for n in graph["nodes"]])
                    self.assertIn(topic, [t["name"] for t in graph["topics"]])

                    packet = frame_of(ws, BINARY)
                    self.assertEqual(struct.unpack_from("<I", packet)[0], len(packet) - 4)
                    self.assertEqual(packet[8:12], b"TLMS")

                    # UDP may contain older packets captured before the WS
                    # upgrade, and now also carries system-graph chunks.
                    chunks = {}
                    seen_frame = False
                    for _ in range(500):
                        datagram = udp.recv(65536)
                        if datagram.startswith(b"TSYS"):
                            # tag(4) + document id(2) + index(1) + count(1).
                            document = datagram[4] | (datagram[5] << 8)
                            chunks.setdefault(document, {})[datagram[6]] = datagram[8:]
                            continue
                        if datagram == packet:
                            seen_frame = True
                            break
                    self.assertTrue(seen_frame,
                                    "identical WebSocket frame absent from UDP stream")

                    # A graph too large for one datagram is split, so the
                    # desktop transport has to reassemble; check the framing
                    # over the real socket rather than only in a unit test.
                    complete = [parts for parts in chunks.values()
                                if parts and len(parts) == max(parts) + 1]
                    self.assertTrue(complete, "no system graph on the UDP stream")
                    document = b"".join(
                        parts[i] for parts in complete[:1] for i in sorted(parts))
                    self.assertEqual(
                        json.loads(document.decode())["kind"], "talos.system_graph")

                with urllib.request.urlopen(
                        f"http://127.0.0.1:{http_port}/system.json", timeout=3) as response:
                    graph = json.loads(response.read())
                    self.assertEqual(graph["kind"], "talos.system_graph")
                    self.assertEqual(graph["version"], 2)
                    # Every source and topic carries the endpoint attributes,
                    # so a viewer never has to guess whether a missing end is a
                    # fault or the design.
                    for node in graph["nodes"]:
                        for source in node["sources"]:
                            self.assertIn("external", source)
                            self.assertIn("optional", source)
                    for entry in graph["topics"]:
                        self.assertIn(
                            entry["health"],
                            ["ok", "orphaned", "unread", "bridged",
                             "unconnected", "lossy", "idle"])

                # Served verbatim: the bridge has no JSON parser and re-emitting
                # the launcher's bytes unchanged is the one transformation that
                # cannot be subtly wrong.
                with urllib.request.urlopen(
                        f"http://127.0.0.1:{http_port}/declared.json", timeout=3) as response:
                    self.assertEqual(response.read(), declared_bytes)
                    self.assertEqual(response.headers["Content-Type"], "application/json")

                # Without --declared it is a 404, not an empty document: "no
                # declared graph was given" and "the declared graph is empty"
                # are different statements, and a viewer that could not tell
                # them apart would draw a session with no nodes in it.
                with self.assertRaises(urllib.error.HTTPError) as absent:
                    urllib.request.urlopen(
                        f"http://127.0.0.1:{bare_port}/declared.json", timeout=3)
                self.assertEqual(absent.exception.code, 404)
                with socket.create_connection(("127.0.0.1", http_port), timeout=3) as http:
                    http.sendall(b"GET /%2e%2e/secret HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n")
                    self.assertIn(b"400 Bad Request", http.recv(1024))
            finally:
                for process in reversed(processes):
                    if process.poll() is None:
                        process.send_signal(signal.SIGTERM)
                        try:
                            process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait()
                for process in processes:
                    self.assertEqual(process.returncode, 0)
                quiet_udp.close()


if __name__ == "__main__":
    unittest.main()
