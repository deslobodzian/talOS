"""Exercise real IPC -> WebSocket bytes with a minimal stdlib WebSocket client."""
import base64
import json
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import sys
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

    All are signalled before any is waited for: a bridge exits at its next
    loop wake, up to a second after the signal, and waiting for each in turn
    added those seconds up.
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


# An RTMS segment's header (talOS/rtms/rtms.h RTMSHeader, version 9, 2368 B):
# the magic at 0 and the version at 8; ten u64 fields, the 128 B schema name
# and the lifetime word, then the 64 B-aligned Writer, whose owner process
# word is at 256 + 8; reader slot i of 32 at 320 + 64 i, its state word at
# + 8, holding its claimant's process word above two state bits. A process
# word's low 22 bits are the pid (talOS/rtms/process_liveness.h
# kProcessPidBits). A segment of another version is skipped: no process of
# this build can hold it, and its reader table is not at these offsets.
RTMS_MAGIC = 0x32534D54534F4C41
RTMS_VERSION = 9
RTMS_HEADER_BYTES = 2368
RTMS_WRITER_OWNER = 264
RTMS_READERS, RTMS_MAX_READERS = 320, 32
PID_MASK = (1 << 22) - 1


def rtms_segments():
    """(name, writer pid, reader pids) of every RTMS segment this user can read."""
    segments = []
    for entry in os.scandir("/dev/shm"):
        try:
            with open(entry.path, "rb") as handle:
                header = handle.read(RTMS_HEADER_BYTES)
        except OSError:
            continue
        if (len(header) < RTMS_HEADER_BYTES
                or struct.unpack_from("<QQ", header) != (RTMS_MAGIC, RTMS_VERSION)):
            continue
        owner = struct.unpack_from("<Q", header, RTMS_WRITER_OWNER)[0]
        readers = {(struct.unpack_from("<Q", header, RTMS_READERS + 64 * i + 8)[0] >> 2) & PID_MASK
                   for i in range(RTMS_MAX_READERS)}
        segments.append((entry.name, owner & PID_MASK if owner else None, readers - {0}))
    return segments


def socket_inodes(pid):
    """The inode of every socket process `pid` holds open."""
    inodes = set()
    for fd in os.scandir(f"/proc/{pid}/fd"):
        try:
            target = os.readlink(fd.path)
        except OSError:
            continue
        if target.startswith("socket:["):
            inodes.add(int(target[len("socket:["):-1]))
    return inodes


def table_inodes(*tables):
    """The socket inodes /proc/net/<table> lists, for each table given."""
    inodes = set()
    for table in tables:
        path = Path("/proc/net", table)
        if not path.exists():
            continue
        lines = path.read_text().splitlines()[1:]
        # The inode is column 9 of the inet tables and column 6 of unix.
        column = 6 if table == "unix" else 9
        inodes |= {int(line.split()[column]) for line in lines if len(line.split()) > column}
    return inodes


class BridgeIntegration(unittest.TestCase):
    # REQ-SAFE-35
    @unittest.skipUnless(sys.platform.startswith("linux"),
                         "reads /dev/shm and /proc, which only Linux has")
    def test_running_bridge_opens_no_writer_and_no_datagram_socket(self):
        """A bridge serving a client holds no RTMS writer and no UDP or raw socket.

        A viewer that wrote a topic (a target lane, /hw/request/<node>) or
        sent a frame to the RoboRIO (UDP) could move the robot. Every RTMS
        segment on the host records its live writer's process, so none may
        name the bridge; every socket the bridge holds must be TCP (HTTP and
        WebSocket) or Unix-domain (the wake channel). Neither check can pass
        by reading nothing: the telemetry segment must show the mock as its
        writer and the bridge as a reader, and the bridge's listener must be
        found among its TCP sockets.
        """
        workspace = Path(os.environ["TEST_SRCDIR"]) / os.environ.get("TEST_WORKSPACE", "_main")
        binaries = workspace / "studio/bridge"
        topic = f"/test/telemetry/s{os.getpid()}"
        processes = []
        with tempfile.TemporaryDirectory() as webroot:
            try:
                mock = subprocess.Popen([str(binaries / "studio_mock"), topic],
                                        stdout=subprocess.PIPE, text=True)
                processes.append(mock)
                self.assertEqual(mock.stdout.readline().strip(), "ready")
                bridge = subprocess.Popen([str(binaries / "studio_bridge"), topic, "0", webroot],
                                          stdout=subprocess.PIPE, text=True)
                processes.append(bridge)
                port = bound_port(bridge)
                with socket.create_connection(("127.0.0.1", port), timeout=10) as ws:
                    self.assertIn(b"101 Switching Protocols", upgrade(ws))
                    # A telemetry page reached the client: the bridge has
                    # attached, read the topic and sent on the socket.
                    self.assertEqual(frame_of(ws, BINARY)[8:12], b"TLM3")

                    segments = rtms_segments()
                    written = [name for name, writer, _ in segments if writer == bridge.pid]
                    self.assertEqual(written, [], f"bridge pid {bridge.pid} writes {written}")
                    telemetry = [(writer, readers) for _, writer, readers in segments
                                 if bridge.pid in readers]
                    self.assertIn((mock.pid, {bridge.pid}), telemetry,
                                  f"no segment written by the mock ({mock.pid}) and read by "
                                  f"the bridge ({bridge.pid}): {segments}")

                    held = socket_inodes(bridge.pid)
                    datagram = held & table_inodes("udp", "udp6", "udplite", "udplite6",
                                                   "raw", "raw6")
                    self.assertEqual(datagram, set(), f"bridge holds UDP or raw sockets {datagram}")
                    tcp, unix = table_inodes("tcp", "tcp6"), table_inodes("unix")
                    self.assertTrue(held & tcp, f"none of {held} is TCP: the listener is missing")
                    self.assertEqual(held - tcp - unix, set(),
                                     f"bridge sockets neither TCP nor Unix: {held - tcp - unix}")
            finally:
                stop(processes)
                for process in processes:
                    self.assertEqual(process.returncode, 0)

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

                # Served verbatim: the bridge never parses it, and re-emitting
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
                stop(processes)
                for process in processes:
                    self.assertEqual(process.returncode, 0)

    # REQ-OPS-35
    def test_drive_feed_slots_and_layout(self):
        """/drive is served outside max_clients, evicts its oldest, and needs a layout.

        With the only programmer slot taken, a driver dashboard still
        connects; it is sent the dictionary, its masked values and the drive
        summary, never the system graph; a second dashboard past
        drive_clients evicts the first with a close the page can show. A
        bridge without a layout answers 404 to both /drive endpoints.
        """
        workspace = Path(os.environ["TEST_SRCDIR"]) / os.environ.get("TEST_WORKSPACE", "_main")
        binaries = workspace / "studio/bridge"
        topic = f"/test/telemetry/d{os.getpid()}"
        processes = []
        with tempfile.TemporaryDirectory() as webroot, \
                tempfile.TemporaryDirectory() as configs:
            Path(configs, "robot.toml").write_text(
                "[robot]\nperiod_us = 5000\n\n"
                "[studio]\nmax_clients = 1\ndashboard = \"dashboard.toml\"\n\n"
                "[subsystems.alpha]\nnode = \"//test:alpha\"\ncritical = false\n")
            Path(configs, "dashboard.toml").write_text(
                "[dashboard]\ntitle = \"Pit\"\ndrive_clients = 1\n\n"
                "[[widgets]]\nkind = \"number\"\nat = [0, 0, 2, 1]\n"
                "channel = \"drive/velocity\"\n")
            try:
                mock = subprocess.Popen([str(binaries / "studio_mock"), topic],
                                        stdout=subprocess.PIPE, text=True)
                processes.append(mock)
                self.assertEqual(mock.stdout.readline().strip(), "ready")
                bridge = subprocess.Popen([str(binaries / "studio_bridge"), topic, "0", webroot,
                                           "--config", str(Path(configs, "robot.toml"))],
                                          stdout=subprocess.PIPE, text=True)
                processes.append(bridge)
                bare = subprocess.Popen([str(binaries / "studio_bridge"), topic, "0", webroot],
                                        stdout=subprocess.PIPE, text=True)
                processes.append(bare)
                port, bare_port = bound_port(bridge), bound_port(bare)

                with urllib.request.urlopen(
                        f"http://127.0.0.1:{port}/drive/layout.json", timeout=3) as response:
                    layout = json.loads(response.read())
                self.assertEqual(layout["title"], "Pit")
                self.assertEqual(layout["channels"], ["drive/velocity"])
                with self.assertRaises(urllib.error.HTTPError) as absent:
                    urllib.request.urlopen(f"http://127.0.0.1:{bare_port}/drive/layout.json",
                                           timeout=3)
                self.assertEqual(absent.exception.code, 404)
                with socket.create_connection(("127.0.0.1", bare_port), timeout=10) as ws:
                    self.assertIn(b" 404 ", upgrade(ws, "/drive").split(b"\r\n")[0])

                with socket.create_connection(("127.0.0.1", port), timeout=10) as programmer, \
                        socket.create_connection(("127.0.0.1", port), timeout=10) as refused, \
                        socket.create_connection(("127.0.0.1", port), timeout=10) as first, \
                        socket.create_connection(("127.0.0.1", port), timeout=10) as second:
                    self.assertIn(b"101 Switching Protocols", upgrade(programmer))
                    self.assertIn(b"503", upgrade(refused).split(b"\r\n")[0])
                    self.assertIn(b"101 Switching Protocols", upgrade(first, "/drive"))

                    binary, summary = [], None
                    while summary is None or len(binary) < 3:
                        opcode, payload = any_frame(first)
                        if opcode == BINARY:
                            self.assertEqual(payload[8:12], b"TLM3")
                            binary.append(payload)
                            continue
                        self.assertEqual(opcode, TEXT)
                        document = json.loads(payload.decode())
                        if document["kind"] == "talos.drive.heartbeat":
                            continue  # before it holds a tick
                        self.assertEqual(document["kind"], "talos.drive.summary",
                                         "a drive client was sent another text message")
                        summary = document
                    # The dictionary first, whole, then value pages.
                    self.assertIn(b"drive/velocity", binary[0])
                    self.assertLessEqual(len(json.dumps(summary)), 4096)
                    self.assertFalse(summary["supervised"])
                    self.assertEqual([(n["name"], n["critical"], n["alive"])
                                      for n in summary["nodes"]],
                                     [("hardware_node", True, False), ("alpha", False, False)])
                    self.assertEqual(summary["omitted"], 0)

                    # drive_clients = 1: the newer dashboard takes the slot.
                    self.assertIn(b"101 Switching Protocols", upgrade(second, "/drive"))
                    while True:
                        opcode, payload = any_frame(first)
                        if opcode == CLOSE:
                            break
                    code = struct.unpack("!H", payload[:2])[0]
                    self.assertEqual(code, 4000)
                    self.assertEqual(payload[2:], b"replaced by a newer driver dashboard")
                    self.assertEqual(any_frame(second)[0], BINARY)
            finally:
                stop(processes)
                for process in processes:
                    self.assertEqual(process.returncode, 0)

    # REQ-OPS-34
    def test_cache_headers(self):
        """Hashed assets are cached for good; pages and live JSON are not.

        A browser that cached index.html or drive.html would keep loading
        the asset names of an old build after a deploy; a hashed asset
        never changes under its name, so it is cached for a year. A file
        that is not hashed (a favicon, an unhashed file under assets/) is
        revalidated like the pages, and the JSON the bridge generates is
        never stored.
        """
        workspace = Path(os.environ["TEST_SRCDIR"]) / os.environ.get("TEST_WORKSPACE", "_main")
        binaries = workspace / "studio/bridge"
        topic = f"/test/telemetry/c{os.getpid()}"
        processes = []
        with tempfile.TemporaryDirectory() as webroot, \
                tempfile.TemporaryDirectory() as configs:
            Path(webroot, "index.html").write_text("studio")
            Path(webroot, "drive.html").write_text("drive")
            Path(webroot, "favicon.svg").write_text("<svg/>")
            Path(webroot, "assets").mkdir()
            # Vite's default name, `<name>-<8-character hash>.<ext>`; the
            # hash's alphabet includes '-' (a deployed bundle's real name).
            Path(webroot, "assets", "index-3-nMjUae.js").write_text("hashed")
            Path(webroot, "assets", "logo.svg").write_text("<svg/>")
            Path(configs, "robot.toml").write_text(
                "[robot]\nperiod_us = 5000\n\n"
                "[studio]\ndashboard = \"dashboard.toml\"\n\n"
                "[subsystems.alpha]\nnode = \"//test:alpha\"\n")
            Path(configs, "dashboard.toml").write_text(
                "[dashboard]\ntitle = \"Pit\"\n\n"
                "[[widgets]]\nkind = \"number\"\nat = [0, 0, 2, 1]\n"
                "channel = \"drive/velocity\"\n")
            try:
                mock = subprocess.Popen([str(binaries / "studio_mock"), topic],
                                        stdout=subprocess.PIPE, text=True)
                processes.append(mock)
                self.assertEqual(mock.stdout.readline().strip(), "ready")
                bridge = subprocess.Popen([str(binaries / "studio_bridge"), topic, "0", webroot,
                                           "--config", str(Path(configs, "robot.toml"))],
                                          stdout=subprocess.PIPE, text=True)
                processes.append(bridge)
                port = bound_port(bridge)

                def cache_control(path):
                    # The live JSON answers 503 until the mock's dictionary
                    # and first tick arrive.
                    deadline = time.monotonic() + 10
                    while True:
                        try:
                            with urllib.request.urlopen(f"http://127.0.0.1:{port}{path}",
                                                        timeout=3) as response:
                                return response.headers.get("Cache-Control")
                        except urllib.error.HTTPError as error:
                            if error.code != 503 or time.monotonic() > deadline:
                                raise
                            time.sleep(0.02)

                immutable = "public, max-age=31536000, immutable"
                expected = {
                    "/": "no-cache",
                    "/index.html": "no-cache",
                    "/drive": "no-cache",
                    "/drive.html": "no-cache",
                    "/favicon.svg": "no-cache",
                    "/assets/logo.svg": "no-cache",
                    "/assets/index-3-nMjUae.js": immutable,
                    "/channels.json": "no-store",
                    "/telemetry.json": "no-store",
                    "/system.json": "no-store",
                    "/drive/layout.json": "no-store",
                }
                for path, header in expected.items():
                    with self.subTest(path=path):
                        self.assertEqual(cache_control(path), header)
            finally:
                stop(processes)
                for process in processes:
                    self.assertEqual(process.returncode, 0)

    # REQ-RT-21
    def test_sleeps_until_a_publication_or_a_deadline(self):
        """An idle bridge wakes only for its deadlines; a publication wakes it.

        Before this, a 1 ms timer drained the topic, so the bridge woke about
        1000 times a second with nothing published, and a page waited for the
        next tick of that timer.
        """
        workspace = Path(os.environ["TEST_SRCDIR"]) / os.environ.get("TEST_WORKSPACE", "_main")
        binaries = workspace / "studio/bridge"
        topic = f"/test/telemetry/w{os.getpid()}"
        processes = []
        with tempfile.TemporaryDirectory() as webroot:
            try:
                # The publisher first, so the bridge attaches at once and the
                # idle window below is the attached, idle state; it publishes
                # only when the test writes a line.
                mock = subprocess.Popen([str(binaries / "studio_mock"), topic, "--on-stdin"],
                                        stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                        text=True)
                processes.append(mock)
                self.assertEqual(mock.stdout.readline().strip(), "ready")
                bridge = subprocess.Popen([str(binaries / "studio_bridge"), topic, "0", webroot],
                                          stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                          text=True)
                processes.append(bridge)
                started = time.monotonic()
                port = bound_port(bridge)
                with socket.create_connection(("127.0.0.1", port), timeout=5) as ws:
                    key = base64.b64encode(os.urandom(16)).decode()
                    ws.sendall((f"GET /telemetry HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\n"
                                f"Connection: Upgrade\r\nSec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n\r\n").encode())
                    headers = b""
                    while not headers.endswith(b"\r\n\r\n"):
                        headers += receive(ws, 1)
                    self.assertIn(b"101 Switching Protocols", headers)
                    # The idle window is measured in the bridge's own graph
                    # updates (one per GraphFeed::kUpdateNs), not slept out.
                    for _ in range(4):
                        self.assertIn(json.loads(frame_of(ws, TEXT).decode())["kind"],
                                      ("talos.system_graph", "talos.system_graph.update"))
                    # One tick published into the idle topic reaches the
                    # client: its dictionary, then its keyframe.
                    mock.stdin.write("tick\n")
                    mock.stdin.flush()
                    self.assertEqual(mock.stdout.readline().strip(), "published 0")
                    self.assertEqual(frame_of(ws, BINARY)[8:12], b"TLM3")
                    self.assertEqual(frame_of(ws, BINARY)[8:12], b"TLM3")
                bridge.send_signal(signal.SIGTERM)
                _, errors = bridge.communicate(timeout=5)
                elapsed_s = time.monotonic() - started
                self.assertEqual(bridge.returncode, 0, errors)
                summary = dict(field.split("=", 1) for field in errors.split()
                               if field.count("=") == 1)
                # The bridge's deadlines, idle with one client: the graph
                # refresh (4/s), the segment check (1/s), the client's graph
                # update (2/s), and uWebSockets' own date (1/s) and socket
                # sweep (1/4 s) timers: under 9 a second. 25 a second and 20
                # for the connection, the publication and the stop leave
                # room for an early or doubled wake; the 1 ms timer made
                # about 1000 a second.
                bound = 25 * elapsed_s + 20
                self.assertLessEqual(int(summary["wakes"]), bound,
                                     f"{summary['wakes']} wakes in {elapsed_s:.2f} s; "
                                     f"bound {bound:.0f} (25/s + 20): {errors}")
                # The page was read because its publication woke the loop.
                self.assertGreaterEqual(int(summary["publication_wakes"]), 1, errors)
            finally:
                if mock.stdin:
                    mock.stdin.close()
                stop(processes)
                for process in processes:
                    self.assertEqual(process.returncode, 0)


if __name__ == "__main__":
    unittest.main()
