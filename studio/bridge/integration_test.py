"""Exercise real IPC -> WebSocket bytes with a minimal stdlib WebSocket client."""
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
    # A server never masks its frames (RFC 6455 section 5.1).
    assert not length & 0x80, "server frame is masked"
    length &= 127
    if length == 126:
        length = struct.unpack("!H", receive(sock, 2))[0]
    elif length == 127:
        length = struct.unpack("!Q", receive(sock, 8))[0]
    return first, receive(sock, length)


# The system graph's fast values in document order (studio/bridge/system.h
# BuildSystemGraph; studio/src/system.ts walks the same order). An update's
# `values` are written over these slots of the client's document.
def fast_slots(document):
    slots = [(document, "wall_ns")]
    slots += [(document["bridge"], key) for key in
              ("clients", "frames_forwarded", "invalid", "ws_drops", "refused", "egress_bytes")]
    for node in document["nodes"]:
        slots += [(node, key) for key in ("heartbeat_wall_ns", "dispatch_count", "alive")]
        for source in node["sources"]:
            slots += [(source, key) for key in
                      ("events", "dropped", "sequence", "last_monotonic_ns",
                       "last_latency_ns", "max_latency_ns")]
        slots += [(selection, "selected") for selection in node["selections"]]
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
                tempfile.TemporaryDirectory() as outputs:
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
            # The launcher's session manifest, like the declared graph, lives
            # outside the webroot so the route cannot be confused with the
            # static file handler. The bridge re-reads it per request, so the
            # test rewrites it below to prove /manifest.json is live.
            manifest = Path(outputs, "manifest.json")
            manifest_bytes = (b'{"session_id":"7","config_path":"robot.toml",'
                              b'"nodes":[{"name":"drivetrain","restart_count":0,'
                              b'"exit_code":0}]}')
            manifest.write_bytes(manifest_bytes)
            try:
                # The bridges start before the publisher: each serves HTTP and
                # waits for the telemetry segment, then attaches to it.
                bridge = subprocess.Popen([str(binaries / "studio_bridge"), topic,
                                           "0", webroot, "--declared", str(declared),
                                           "--manifest", str(manifest)],
                                          stdout=subprocess.PIPE, text=True)
                processes.append(bridge)
                # The same bridge with no declared graph, to pin down what
                # /declared.json does then. Started here so both are up while
                # the deadline loop below waits.
                bare = subprocess.Popen([str(binaries / "studio_bridge"), topic,
                                         "0", webroot],
                                        stdout=subprocess.PIPE, text=True)
                processes.append(bare)
                http_port = bound_port(bridge)
                bare_port = bound_port(bare)
                # Wait until both bridges listen. A bridge signalled before it
                # is listening has not installed its SIGTERM handler yet and
                # dies with -15 instead of exiting 0.
                deadline = time.monotonic() + 10
                for port in (bare_port, http_port):
                    while True:
                        self.assertIsNone(bridge.poll(), "bridge exited")
                        self.assertIsNone(bare.poll(), "bridge without --declared exited")
                        try:
                            ws = socket.create_connection(("127.0.0.1", port), timeout=1)
                            break
                        except OSError:
                            if time.monotonic() > deadline:
                                raise
                            time.sleep(0.02)
                    if port == bare_port:
                        ws.close()
                mock = subprocess.Popen([str(binaries / "studio_mock"), topic],
                                        stdout=subprocess.PIPE, text=True)
                processes.append(mock)
                self.assertEqual(mock.stdout.readline().strip(), "ready")
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

                    # The first binary frame is the dictionary: the bridge
                    # hands a new client the dictionary and then a keyframe
                    # on open, so values are decodable from the start.
                    packet = frame_of(ws, BINARY)
                    self.assertEqual(struct.unpack_from("<I", packet)[0], len(packet) - 4)
                    self.assertEqual(packet[8:12], b"TLM3")
                    self.assertIn(b"drive/velocity", packet)
                    self.assertIn(b"intake/inhibit", packet)
                    # Value pages decode against that dictionary: they
                    # carry no channel names and are far smaller.
                    following = frame_of(ws, BINARY)
                    self.assertEqual(following[8:12], b"TLM3")
                    self.assertNotIn(b"drive/velocity", following)
                    self.assertLess(len(following), len(packet))
                    self.assertNotEqual(following, packet)

                    # The self-describing JSON views of the same feed.
                    with urllib.request.urlopen(
                            f"http://127.0.0.1:{http_port}/channels.json", timeout=3) as response:
                        channels_doc = json.loads(response.read())
                    self.assertEqual(channels_doc["kind"], "talos.telemetry.channels")
                    self.assertEqual(channels_doc["version"], 3)
                    names = [c["name"] for c in channels_doc["channels"]]
                    self.assertIn("drive/velocity", names)
                    inhibit = next(c for c in channels_doc["channels"]
                                   if c["name"] == "intake/inhibit")
                    self.assertIn("TARGET_STALE", inhibit["labels"])

                    with urllib.request.urlopen(
                            f"http://127.0.0.1:{http_port}/telemetry.json", timeout=3) as response:
                        telemetry_doc = json.loads(response.read())
                    self.assertEqual(telemetry_doc["kind"], "talos.telemetry")
                    self.assertEqual(telemetry_doc["version"], 3)
                    # uint64 leaves as decimal strings (JSON numbers are
                    # doubles and would lose precision past 2^53).
                    self.assertIsInstance(telemetry_doc["stream_id"], str)
                    self.assertIsInstance(telemetry_doc["sequence_id"], str)
                    self.assertIsInstance(telemetry_doc["timestamp_ns"], str)
                    self.assertIn("age_ms", telemetry_doc)
                    channels_now = telemetry_doc["channels"]
                    self.assertIn("value", channels_now["drive/velocity"])
                    self.assertEqual(channels_now["drive/velocity"]["unit"], "m/s")

                    # After the whole graph, the open socket gets only its
                    # fast values (counters, heartbeats), for the structure it
                    # holds: written over the document's fast fields in
                    # order, value for value of the same JSON type, they make
                    # the current graph, which counts the producer ticks the
                    # bridge has assembled by then.
                    # (A node registering meanwhile changes the structure, which
                    # brings the whole graph again.) Applied in turn, updates
                    # reach a graph counting the ticks forwarded since.
                    for _ in range(20):
                        update = json.loads(frame_of(ws, TEXT, limit=2000).decode())
                        if update["kind"] == "talos.system_graph":
                            graph = update
                            continue
                        self.assertEqual(update["kind"], "talos.system_graph.update")
                        self.assertEqual(update["structure"], graph["structure"])
                        slots = fast_slots(graph)
                        self.assertEqual(len(update["values"]), len(slots))
                        for (owner, key), value in zip(slots, update["values"]):
                            if owner[key] is not None and value is not None:
                                self.assertIs(type(value), type(owner[key]), key)
                            owner[key] = value
                        if int(graph["bridge"]["frames_forwarded"]) > 0:
                            break
                    self.assertGreater(int(graph["bridge"]["frames_forwarded"]), 0)
                    self.assertEqual(graph["bridge"]["clients"], 1)

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

                # The session manifest is served the same way, but re-read per
                # request: rewriting the file changes what /manifest.json
                # returns, because the launcher keeps it current in place.
                with urllib.request.urlopen(
                        f"http://127.0.0.1:{http_port}/manifest.json", timeout=3) as response:
                    self.assertEqual(response.read(), manifest_bytes)
                    self.assertEqual(response.headers["Content-Type"], "application/json")
                updated_manifest_bytes = manifest_bytes.replace(b'"restart_count":0',
                                                                b'"restart_count":1')
                manifest.write_bytes(updated_manifest_bytes)
                with urllib.request.urlopen(
                        f"http://127.0.0.1:{http_port}/manifest.json", timeout=3) as response:
                    self.assertEqual(response.read(), updated_manifest_bytes)

                # Without --declared it is a 404, not an empty document: "no
                # declared graph was given" and "the declared graph is empty"
                # are different statements, and a viewer that could not tell
                # them apart would draw a session with no nodes in it.
                with self.assertRaises(urllib.error.HTTPError) as absent:
                    urllib.request.urlopen(
                        f"http://127.0.0.1:{bare_port}/declared.json", timeout=3)
                self.assertEqual(absent.exception.code, 404)
                # Without --manifest the manifest is likewise 404.
                with self.assertRaises(urllib.error.HTTPError) as absent_manifest:
                    urllib.request.urlopen(
                        f"http://127.0.0.1:{bare_port}/manifest.json", timeout=3)
                self.assertEqual(absent_manifest.exception.code, 404)
                with socket.create_connection(("127.0.0.1", http_port), timeout=3) as http:
                    http.sendall(b"GET /%2e%2e/secret HTTP/1.1\r\nHost: localhost\r\nConnection: close\r\n\r\n")
                    self.assertIn(b"400 Bad Request", http.recv(1024))
                # Static files are read synchronously on the polling thread,
                # so one past the 4 MiB cap is refused rather than read.
                oversized = Path(webroot, "oversized.js")
                with oversized.open("wb") as handle:
                    handle.truncate(4 * 1024 * 1024 + 1)
                with self.assertRaises(urllib.error.HTTPError) as too_large:
                    urllib.request.urlopen(
                        f"http://127.0.0.1:{http_port}/oversized.js", timeout=3)
                self.assertEqual(too_large.exception.code, 413)
            finally:
                for process in reversed(processes):
                    if process.poll() is None:
                        process.send_signal(signal.SIGTERM)
                        try:
                            process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait()
                    if process.stdout:
                        process.stdout.close()
                for process in processes:
                    self.assertEqual(process.returncode, 0)


if __name__ == "__main__":
    unittest.main()
