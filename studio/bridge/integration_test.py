"""Exercise real IPC -> WebSocket bytes with bridge_client's stdlib WebSocket client."""
import base64
import ctypes
import json
import mmap
import os
from pathlib import Path
import re
import shutil
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

sys.path.insert(0, str(Path(__file__).resolve().parent))
from bridge_client import (BINARY, CLOSE, TEXT, any_frame, bound_port, fast_slots,
                           frame_of, receive, stop, upgrade)
from talOS.memory.shm_name import shm_name


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


def parse_rtms_header(header):
    """(writer_pid_or_None, reader_pids) from an RTMS v9 header, or None."""
    if (len(header) < RTMS_HEADER_BYTES
            or struct.unpack_from("<QQ", header) != (RTMS_MAGIC, RTMS_VERSION)):
        return None
    owner = struct.unpack_from("<Q", header, RTMS_WRITER_OWNER)[0]
    readers = {(struct.unpack_from("<Q", header, RTMS_READERS + 64 * i + 8)[0] >> 2) & PID_MASK
               for i in range(RTMS_MAX_READERS)}
    return (owner & PID_MASK if owner else None, readers - {0})


def rtms_segments():
    """(name, writer pid, reader pids) of every RTMS segment this user can read."""
    segments = []
    for entry in os.scandir("/dev/shm"):
        try:
            with open(entry.path, "rb") as handle:
                header = handle.read(RTMS_HEADER_BYTES)
        except OSError:
            continue
        parsed = parse_rtms_header(header)
        if parsed is None:
            continue
        segments.append((entry.name, parsed[0], parsed[1]))
    return segments


VMMAP_REGION_RE = re.compile(r"\]\s+([r-][w-][x-])/([r-][w-][x-])\s+SM=(\S+)")
DARWIN_NON_SOCKET_TYPES = frozenset({
    "REG", "VREG", "DIR", "VDIR", "CHR", "VCHR", "BLK", "VBLK",
    "FIFO", "VFIFO", "PIPE", "KQUEUE", "VNODE", "NPOLICY", "GUARD",
    "PSXSHM", "PSXSEM", "FSEVENTS", "NEXUS", "CHANNEL",
})


def darwin_rtms_segment(topic):
    """(writer pid, reader pids) of `topic`'s POSIX shm segment on macOS."""
    if "TALOS_SHM_NAMESPACE" in os.environ:
        ns = os.environ["TALOS_SHM_NAMESPACE"]
    else:
        ns = os.environ.get("TEST_TMPDIR", "")
    segment = shm_name(topic, ns) if ns else "/" + topic.lstrip("/").replace("/", ".")
    libc = ctypes.CDLL(None, use_errno=True)
    # Declare only the two fixed parameters: O_RDONLY never reads the variadic
    # mode_t argument, keeping arm64 macOS ABI exact.
    libc.shm_open.argtypes = [ctypes.c_char_p, ctypes.c_int]
    libc.shm_open.restype = ctypes.c_int
    fd = libc.shm_open(segment.encode(), os.O_RDONLY)
    if fd < 0:
        err = ctypes.get_errno()
        raise OSError(err, os.strerror(err), segment)
    try:
        with mmap.mmap(fd, RTMS_HEADER_BYTES, access=mmap.ACCESS_READ) as mapping:
            header = mapping[:RTMS_HEADER_BYTES]
    finally:
        os.close(fd)
    parsed = parse_rtms_header(header)
    if parsed is None:
        raise AssertionError(f"segment {segment} for {topic} is not an RTMS v{RTMS_VERSION} header")
    return parsed


def darwin_tool(name, fallback):
    """Locates a macOS inspection tool even when Bazel's PATH omits /usr/sbin."""
    return shutil.which(name) or fallback


def darwin_writable_shm_regions(pid):
    """Writable `SM=SHM` region lines from `vmmap -wide <pid>` on macOS.

    Catches closed-fd `SharedMemoryPtr` mappings that `lsof` cannot see.
    """
    proc = subprocess.run(
        [darwin_tool("vmmap", "/usr/bin/vmmap"), "-wide", str(pid)],
        capture_output=True, text=True, timeout=15,
    )
    if proc.returncode != 0:
        raise AssertionError(f"vmmap failed ({proc.returncode}): {proc.stderr}")
    writable = []
    for line in proc.stdout.splitlines():
        match = VMMAP_REGION_RE.search(line)
        if match and "w" in match.group(1) and match.group(3) == "SHM":
            writable.append(line.strip())
    return writable


def parse_lsof_fields(text):
    """Parses `lsof -F ftPTan` output into a list of per-fd dicts."""
    records = []
    current = None
    for line in text.splitlines():
        if not line:
            continue
        tag, value = line[0], line[1:]
        if tag == "f":
            current = {"fd": value}
            records.append(current)
        elif current is not None:
            if tag == "T":
                if value.startswith("ST="):
                    current["state"] = value[3:]
            else:
                current[tag] = value
    return records


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
    @unittest.skipUnless(
        sys.platform.startswith("linux") or sys.platform == "darwin",
        "inspects open sockets and shared memory via /proc+/dev/shm (Linux) or lsof+shm_open (macOS)",
    )
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

                    if sys.platform.startswith("linux"):
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
                        self.assertEqual(datagram, set(),
                                         f"bridge holds UDP or raw sockets {datagram}")
                        tcp, unix = table_inodes("tcp", "tcp6"), table_inodes("unix")
                        self.assertTrue(held & tcp,
                                        f"none of {held} is TCP: the listener is missing")
                        self.assertEqual(held - tcp - unix, set(),
                                         f"bridge sockets neither TCP nor Unix: {held - tcp - unix}")
                    else:
                        # macOS: verify the RTMS header directly via
                        # shm_open(O_RDONLY), check writable SM=SHM VM regions
                        # via vmmap (catches closed-fd SharedMemoryPtr maps),
                        # and enforce a strict socket allowlist via lsof.
                        writer, readers = darwin_rtms_segment(topic)
                        self.assertNotEqual(writer, bridge.pid,
                                            f"bridge pid {bridge.pid} owns writer on {topic}")
                        self.assertEqual(writer, mock.pid,
                                         f"expected mock pid {mock.pid} as writer, got {writer}")
                        self.assertIn(bridge.pid, readers,
                                      f"bridge pid {bridge.pid} not listed in readers {readers}")

                        # RegistryReader maps O_RDONLY (r--/r--) and --session-id
                        # is unset, so only the subscribed telemetry topic is
                        # mapped rw- SM=SHM.
                        writable_shm = darwin_writable_shm_regions(bridge.pid)
                        self.assertEqual(
                            len(writable_shm), 1,
                            f"expected exactly 1 writable SM=SHM region (telemetry topic), "
                            f"got {len(writable_shm)}: {writable_shm}",
                        )

                        lsof_bin = darwin_tool("lsof", "/usr/sbin/lsof")
                        udp = subprocess.run(
                            [lsof_bin, "-n", "-P", "-a", "-p", str(bridge.pid), "-i", "UDP"],
                            capture_output=True, text=True, timeout=10,
                        )
                        self.assertIn(udp.returncode, (0, 1), udp.stderr)
                        if udp.returncode == 1:
                            self.assertEqual(udp.stderr, "",
                                             f"lsof -i UDP failed: {udp.stderr}")
                        self.assertEqual(udp.stdout.strip(), "",
                                         f"bridge holds UDP sockets:\n{udp.stdout}")

                        all_fds = subprocess.run(
                            [lsof_bin, "-n", "-P", "-a", "-p", str(bridge.pid), "-F", "ftPTan"],
                            capture_output=True, text=True, timeout=10,
                        )
                        self.assertEqual(all_fds.returncode, 0, all_fds.stderr)
                        records = parse_lsof_fields(all_fds.stdout)
                        self.assertTrue(records, "lsof returned no descriptors for bridge")

                        port_re = re.compile(rf":{port}(?:$|[^0-9])")
                        ws_port_re = re.compile(rf":{ws.getsockname()[1]}(?:$|[^0-9])")
                        tcp_listeners = []
                        tcp_clients = []
                        for rec in records:
                            fd_type = rec.get("t", "")
                            proto = rec.get("P", "")
                            state = rec.get("state", "")
                            access = rec.get("a", "")
                            name = rec.get("n", "")
                            if fd_type == "PSXSHM":
                                self.assertNotIn(access, ("w", "u"),
                                                 f"bridge holds writable shm descriptor: {rec}")
                                continue
                            if fd_type in DARWIN_NON_SOCKET_TYPES and not proto:
                                continue
                            if fd_type == "unix":
                                continue
                            # macOS libnetwork/getaddrinfo opens an internal
                            # com.apple.netsrc kernel-control socket for
                            # address-selection queries (cannot transmit).
                            if (fd_type == "systm"
                                    and name.startswith("[ctl com.apple.netsrc ")):
                                continue
                            # Strict socket allowlist: reject every non-unix
                            # socket (IPv4/IPv6 non-TCP, systm, KCTL, ROUTE,
                            # RAW, NDRV, etc.) except the single TCP LISTEN
                            # socket on :{port} and this test's ESTABLISHED ws.
                            self.assertIn(fd_type, ("IPv4", "IPv6"),
                                          f"bridge holds disallowed socket type: {rec}")
                            self.assertEqual(proto, "TCP",
                                             f"bridge holds non-TCP socket: {rec}")
                            local, sep, remote = name.partition("->")
                            if not sep and state == "LISTEN" and port_re.search(local):
                                tcp_listeners.append(rec)
                            elif (sep and state == "ESTABLISHED"
                                  and port_re.search(local)
                                  and ws_port_re.search(remote)):
                                tcp_clients.append(rec)
                            else:
                                self.fail(f"bridge holds unexpected TCP socket: {rec}")
                        self.assertEqual(
                            len(tcp_listeners), 1,
                            f"expected exactly 1 TCP LISTEN socket on port {port}, "
                            f"got {tcp_listeners} in {records}",
                        )
                        self.assertEqual(
                            len(tcp_clients), 1,
                            f"expected 1 ESTABLISHED ws client socket, "
                            f"got {tcp_clients} in {records}",
                        )
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
                # A bridge reports its port only once it listens, with its
                # SIGTERM handler installed, so each accepts at once.
                socket.create_connection(("127.0.0.1", bare_port), timeout=1).close()
                ws = socket.create_connection(("127.0.0.1", http_port), timeout=1)
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
                    self.assertEqual(graph["version"], 3)
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
                # The RoboRIO link's alert is carried whatever the widgets.
                self.assertEqual(layout["channels"],
                                 ["drive/velocity", "hw/status/alert_error_link_lost"])
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
                # The live JSON answers 503 until the mock's dictionary and
                # first tick arrive; a client is sent both, as a dictionary
                # and then a value page, once the bridge holds them.
                with socket.create_connection(("127.0.0.1", port), timeout=5) as ws:
                    self.assertIn(b"101 ", upgrade(ws))
                    self.assertEqual(frame_of(ws, BINARY)[8:12], b"TLM3")
                    self.assertEqual(frame_of(ws, BINARY)[8:12], b"TLM3")

                def cache_control(path):
                    with urllib.request.urlopen(f"http://127.0.0.1:{port}{path}",
                                                timeout=3) as response:
                        return response.headers.get("Cache-Control")

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

    # REQ-OPS-34, REQ-OPS-35, REQ-OPS-36, TODO.md DEPLOY-12
    def test_refusal_file_serves_drive_summary_layout_and_ws_without_config(self):
        """With --refusal-file, the bridge binds port 0, serves /drive,
        a minimal /drive/layout.json, /drive/summary.json, and ws /drive with
        launch_refused + refusal_reason (plus heartbeats), and fails fast if
        the requested port is already held."""
        workspace = Path(os.environ["TEST_SRCDIR"]) / os.environ.get("TEST_WORKSPACE", "_main")
        bridge_bin = str(workspace / "studio/bridge/studio_bridge")
        with tempfile.TemporaryDirectory() as tmp:
            webroot = Path(tmp, "webroot")
            webroot.mkdir()
            (webroot / "drive.html").write_text("<html>drive</html>")
            refusal_path = Path(tmp, "last_refusal.json")
            refusal_path.write_text(json.dumps({
                "refused": True,
                "reason": "error: motor_py: unread parameter key unread.value_m",
                "config_path": "/opt/talos/current/robot.toml",
                "wall_ns": 123456789,
            }))
            bridge = subprocess.Popen(
                [bridge_bin, "/talos/telemetry", "0", str(webroot),
                 "--refusal-file", str(refusal_path)],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            processes = [bridge]
            try:
                port = bound_port(bridge)
                with urllib.request.urlopen(
                        f"http://127.0.0.1:{port}/drive", timeout=3) as resp:
                    self.assertEqual(resp.status, 200)
                    self.assertIn(b"drive", resp.read())
                with urllib.request.urlopen(
                        f"http://127.0.0.1:{port}/drive/layout.json", timeout=3) as resp:
                    layout = json.loads(resp.read())
                    self.assertEqual(layout["columns"], 1)
                    self.assertEqual(layout["rows"], 1)
                    self.assertEqual(layout["widgets"], [])
                with urllib.request.urlopen(
                        f"http://127.0.0.1:{port}/drive/summary.json", timeout=3) as resp:
                    summary = json.loads(resp.read())
                    self.assertEqual(summary["kind"], "talos.drive.summary")
                    self.assertTrue(summary["launch_refused"])
                    self.assertIn("unread.value_m", summary["refusal_reason"])
                with socket.create_connection(("127.0.0.1", port), timeout=3) as ws:
                    self.assertIn(b"101 Switching Protocols", upgrade(ws, "/drive"))
                    ws.settimeout(3)
                    got_summary = None
                    got_heartbeat = False
                    for _ in range(10):
                        opcode, payload = any_frame(ws)
                        self.assertEqual(opcode, TEXT)
                        doc = json.loads(payload)
                        if doc["kind"] == "talos.drive.summary":
                            got_summary = doc
                        elif doc["kind"] == "talos.drive.heartbeat":
                            got_heartbeat = True
                        if got_summary and got_heartbeat:
                            break
                    self.assertIsNotNone(got_summary)
                    self.assertTrue(got_summary["launch_refused"])
                    self.assertIn("unread.value_m", got_summary["refusal_reason"])
                    self.assertTrue(got_heartbeat)

                # A second refusal bridge on the same port fails immediately.
                conflict = subprocess.run(
                    [bridge_bin, "/talos/telemetry", str(port), str(webroot),
                     "--refusal-file", str(refusal_path)],
                    capture_output=True, text=True, timeout=5)
                self.assertNotEqual(conflict.returncode, 0)
                self.assertNotIn("listening", conflict.stdout)
            finally:
                stop(processes)
                for process in processes:
                    self.assertEqual(process.returncode, 0)


if __name__ == "__main__":
    unittest.main()
