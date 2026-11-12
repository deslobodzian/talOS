"""Real 200 Hz IPC/WebSocket load probe; run directly with --measure."""
import argparse
import concurrent.futures
import json
import os
from pathlib import Path
import selectors
import signal
import socket
import sys
import struct
import subprocess
import tempfile
import time
import threading
import unittest
import urllib.request

sys.path.insert(0, str(Path(__file__).resolve().parent))
from integration_test import BINARY, TEXT, bound_port, receive


def connect(port, stalled=False):
    sock = socket.socket()
    if stalled:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024)
    sock.settimeout(2)
    sock.connect(("127.0.0.1", port))
    sock.sendall(b"GET /telemetry HTTP/1.1\r\nHost: localhost\r\nUpgrade: websocket\r\n"
                 b"Connection: Upgrade\r\nSec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==\r\n"
                 b"Sec-WebSocket-Version: 13\r\n\r\n")
    headers = b""
    while not headers.endswith(b"\r\n\r\n"):
        headers += receive(sock, 1)
    return sock, headers


# Vtable offsets of Frame's sequence_id and timestamp_ns (talOS/telemetry/telemetry.fbs).
SEQUENCE, TIMESTAMP = 6, 8


def field(payload, voffset):
    table = 4 + struct.unpack_from("<I", payload, 4)[0]
    vtable = table - struct.unpack_from("<i", payload, table)[0]
    # FlatBuffers omits a scalar equal to its default (0) from the table.
    if voffset >= struct.unpack_from("<H", payload, vtable)[0]:
        return 0
    offset = struct.unpack_from("<H", payload, vtable + voffset)[0]
    return struct.unpack_from("<Q", payload, table + offset)[0] if offset else 0


def frames_in(buffer):
    """Removes and yields each whole WebSocket frame at the front of `buffer`."""
    while len(buffer) >= 2:
        first, length = buffer[0], buffer[1]
        assert first in (TEXT, BINARY), first
        # A server never masks its frames (RFC 6455 section 5.1).
        assert not length & 0x80, "server frame is masked"
        header, length = 2, length & 127
        if length == 126:
            if len(buffer) < 4:
                return
            header, length = 4, struct.unpack_from("!H", buffer, 2)[0]
        elif length == 127:
            if len(buffer) < 10:
                return
            header, length = 10, struct.unpack_from("!Q", buffer, 2)[0]
        if len(buffer) < header + length:
            return
        payload = bytes(buffer[header:header + length])
        del buffer[:header + length]
        yield first, payload, header + length


def read_all(sockets, stop, served, end):
    """Drains every reader's socket on one thread until `stop` is readable.

    One thread reads whichever socket has bytes, so no reader falls behind
    because another reader's thread was scheduled first. Returns, per
    reader, what arrived in order: ("binary", sequence) for each telemetry
    update after the dictionary page, ("text", None) for each graph message.
    """
    selector = selectors.DefaultSelector()
    selector.register(stop, selectors.EVENT_READ)
    readers = []
    for sock in sockets:
        sock.setblocking(False)
        reader = {"socket": sock, "buffer": bytearray(), "bytes": 0, "frames": 0,
                  "ages": [], "received": []}
        selector.register(sock, selectors.EVENT_READ, reader)
        readers.append(reader)
    while time.monotonic() < end:
        events = selector.select(timeout=max(0, end - time.monotonic()))
        if any(key.data is None for key, _ in events):
            break
        for key, _ in events:
            reader = key.data
            chunk = reader["socket"].recv(65536)
            if not chunk:
                raise EOFError("WebSocket closed")
            reader["buffer"] += chunk
            for opcode, payload, size in frames_in(reader["buffer"]):
                reader["bytes"] += size
                if opcode != BINARY:
                    reader["received"].append(("text", None))
                    continue
                # The first binary is the dictionary page, whose
                # timestamp is the tick it was published on.
                if reader["frames"]:
                    reader["ages"].append((time.monotonic_ns() - field(payload, TIMESTAMP)) / 1e6)
                    reader["received"].append(("binary", field(payload, SEQUENCE)))
                reader["frames"] += 1
        if all(reader["frames"] >= 16 for reader in readers):
            served.set()
    selector.close()
    results = []
    for reader in readers:
        ages = sorted(reader["ages"])
        results.append({"bytes": reader["bytes"], "frames": reader["frames"],
                        "age_p99_ms": round(ages[int(len(ages) * .99)], 2) if ages else None,
                        "age_max_ms": round(max(ages), 2) if ages else None,
                        "received": reader["received"]})
    return results


def count_window(results):
    """Each reader's sends in the window every reader was served through.

    The window is the producer ticks after the last reader's first update
    (its keyframe, sent as it joined, outside the rotation) through the
    oldest tick any reader ended on. The bridge stamps each update with the
    latest tick, so the updates stamped inside the window are one unbroken
    run of the bridge's sends, whenever the test got around to reading them.
    `updates` counts a reader's updates in that run; `graph` its graph
    messages from its first update in the window to its first one past it,
    the turns in which it may have been sent a graph message instead.
    """
    sequences = [[s for kind, s in r["received"] if kind == "binary"] for r in results]
    complete = all(sequences)
    if complete:
        low, high = max(s[0] for s in sequences), min(s[-1] for s in sequences)
    for result in results:
        updates = graph = 0
        inside = False
        for kind, sequence in result.pop("received"):
            if not complete:
                break
            if kind == "binary":
                if sequence > high:
                    break
                inside = sequence > low
                updates += inside
            elif inside:
                graph += 1
        result["updates"] = updates if complete else None
        result["graph"] = graph if complete else None


def probe(binaries, clients, seconds=None, stalled=False, excess=False, limit=4):
    processes, sockets = [], []
    with tempfile.TemporaryDirectory() as root:
        topic = f"/test/telemetry/p{os.getpid()}"
        config = Path(root, "robot.toml")
        config.write_text(f"[studio]\nmax_clients = {limit}\n"
                          "# Placeholder CAN id; this probe never opens hardware.\n"
                          "[subsystems.alpha.motors.m1]\n"
                          "type = \"TalonFX\"\nbus = \"rio\"\ncan_id = 1\n")
        try:
            mock = subprocess.Popen([str(binaries / "studio_mock"), topic, "64"],
                                    stdout=subprocess.PIPE, text=True)
            processes.append(mock)
            assert mock.stdout.readline().strip() == "ready"
            started = time.monotonic()
            bridge = subprocess.Popen([str(binaries / "studio_bridge"),
                                       topic, "0", root,
                                       "--config", str(config)],
                                      stdout=subprocess.PIPE, text=True)
            processes.append(bridge)
            port = bound_port(bridge)
            deadline = time.monotonic() + 10
            while True:
                try:
                    with urllib.request.urlopen(f"http://127.0.0.1:{port}/system.json", timeout=1):
                        break
                except OSError:
                    if time.monotonic() > deadline:
                        raise
                    time.sleep(.02)
            if stalled:
                sock, headers = connect(port, True)
                sockets.append(sock)
                assert b"101 " in headers, headers
            active = clients - int(stalled)
            for _ in range(active):
                sock, headers = connect(port)
                sockets.append(sock)
                assert b"101 " in headers, headers
            refused = False
            if excess:
                sock, headers = connect(port)
                sockets.append(sock)
                refused = b"503 " in headers
            served = threading.Event()
            stop, stopped = socket.socketpair()
            end = time.monotonic() + (seconds if seconds is not None else 5)
            with concurrent.futures.ThreadPoolExecutor(max_workers=1) as pool:
                reading = pool.submit(read_all, sockets[int(stalled):int(stalled) + active],
                                      stopped, served, end)
                served_ns = None
                try:
                    while True:
                        with urllib.request.urlopen(f"http://127.0.0.1:{port}/system.json") as response:
                            document = json.load(response)
                        stats = document["bridge"]
                        if seconds is None:
                            # Finish on evidence, not an eight-second latency sample:
                            # every healthy reader has kept receiving beside the
                            # non-reading client.
                            if served_ns is None and served.is_set():
                                served_ns = time.time_ns()
                            # /system.json is the bridge's last refresh, taken
                            # every 250 ms (main.cc), so it can predate the
                            # clients connecting; finish on the first refresh
                            # taken after every reader was served.
                            if served_ns is not None and int(document["wall_ns"]) >= served_ns:
                                break
                        if time.monotonic() >= end:
                            if seconds is None:
                                raise AssertionError("bridge made no progress before deadline")
                            break
                        time.sleep(.005)
                finally:
                    stop.send(b"x")
                results = reading.result()
            stop.close()
            stopped.close()
            elapsed = time.monotonic() - started
            count_window(results)
            for result in results:
                result["bytes_s"] = round(result["bytes"] / elapsed)
            rss = None
            status = Path(f"/proc/{bridge.pid}/status")
            if status.exists():
                rss = next(int(line.split()[1]) for line in status.read_text().splitlines()
                           if line.startswith("VmRSS:"))
            return {"clients": clients, "elapsed_s": elapsed,
                    "stalled": stalled, "excess_refused": refused,
                    "rss_kib": rss, "readers": results, "stats": stats}
        finally:
            for sock in sockets:
                sock.close()
            for process in reversed(processes):
                process.send_signal(signal.SIGTERM)
                process.wait(timeout=5)
                if process.stdout:
                    process.stdout.close()


class BandwidthTest(unittest.TestCase):
    # REQ-OPS-23
    def test_limit_budget_and_stalled_isolation(self):
        workspace = Path(os.environ["TEST_SRCDIR"]) / os.environ.get("TEST_WORKSPACE", "_main")
        result = probe(workspace / "studio/bridge", 4, stalled=True, excess=True)
        print(json.dumps(result), flush=True)
        self.assertTrue(result["excess_refused"])
        self.assertEqual(result["stats"]["clients"], 4)
        self.assertGreaterEqual(int(result["stats"]["refused"]), 1)
        self.assertLessEqual(int(result["stats"]["egress_bytes"]),
                             160000 * result["elapsed_s"] + 64000)
        for reader in result["readers"]:
            self.assertGreaterEqual(reader["frames"], 16)
        # The shared bucket is granted in rotation: of the readers, each able
        # to take a send throughout the window, none is sent to twice before
        # every other one is sent to once. So two readers' updates differ by
        # at most one, plus the turns the one with fewer spent on a graph
        # message instead.
        readers = result["readers"]
        for a in readers:
            for b in readers:
                self.assertLessEqual(a["updates"], b["updates"] + 1 + b["graph"], readers)

    def test_configured_client_limit(self):
        workspace = Path(os.environ["TEST_SRCDIR"]) / os.environ.get("TEST_WORKSPACE", "_main")
        result = probe(workspace / "studio/bridge", 1, excess=True, limit=1)
        self.assertTrue(result["excess_refused"])


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--measure", type=Path)
    parser.add_argument("--seconds", type=float, default=8)
    args, remaining = parser.parse_known_args()
    if args.measure:
        for count, stalled, excess in [(1, False, False), (4, False, False),
                                        (4, False, True), (4, True, True)]:
            print(json.dumps(probe(args.measure.resolve(), count, args.seconds, stalled, excess)), flush=True)
    else:
        unittest.main(argv=[__file__] + remaining)
