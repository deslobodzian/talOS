"""Real 200 Hz IPC/WebSocket load probe; run directly with --measure."""
import argparse
import concurrent.futures
import json
import os
from pathlib import Path
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
from integration_test import BINARY, frame, receive


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


def timestamp(payload):
    table = 4 + struct.unpack_from("<I", payload, 4)[0]
    vtable = table - struct.unpack_from("<i", payload, table)[0]
    offset = struct.unpack_from("<H", payload, vtable + 8)[0]
    return struct.unpack_from("<Q", payload, table + offset)[0]


def probe(binaries, clients, seconds=None, stalled=False, excess=False, limit=4):
    processes, sockets = [], []
    with tempfile.TemporaryDirectory() as root:
        topic = f"/test/telemetry/p{os.getpid()}"
        config = Path(root, "robot.toml")
        config.write_text(f"[studio]\nmax_clients = {limit}\n"
                          "# Placeholder CAN id; this probe never opens hardware.\n"
                          "[subsystems.alpha.motors.m1]\n"
                          "type = \"TalonFX\"\nbus = \"rio\"\ncan_id = 1\n")
        with socket.socket() as reservation:
            reservation.bind(("127.0.0.1", 0))
            port = reservation.getsockname()[1]
        try:
            mock = subprocess.Popen([str(binaries / "studio_mock"), topic, "64"],
                                    stdout=subprocess.PIPE, text=True)
            processes.append(mock)
            assert mock.stdout.readline().strip() == "ready"
            started = time.monotonic()
            bridge = subprocess.Popen([str(binaries / "studio_bridge"),
                                       "--drop-newest-publisher", topic, str(port), root,
                                       "--config", str(config)])
            processes.append(bridge)
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
            stop = threading.Event()
            ready = [threading.Event() for _ in range(active)]
            end = time.monotonic() + (seconds if seconds is not None else 5)

            def consume(index):
                sock = sockets[int(stalled) + index]
                count, total, ages = 0, 0, []
                while not stop.is_set() and time.monotonic() < end:
                    opcode, payload = frame(sock)
                    total += len(payload) + (4 if len(payload) >= 126 else 2)
                    if opcode == BINARY:
                        # First binary is the cached dictionary, deliberately old.
                        if count:
                            ages.append((time.monotonic_ns() - timestamp(payload)) / 1e6)
                        count += 1
                        if count >= 16:
                            ready[index].set()
                ages.sort()
                return {"bytes": total, "frames": count,
                        "age_p99_ms": round(ages[int(len(ages) * .99)], 2) if ages else None,
                        "age_max_ms": round(max(ages), 2) if ages else None}

            with concurrent.futures.ThreadPoolExecutor(max_workers=active) as pool:
                readers = [pool.submit(consume, index) for index in range(active)]
                try:
                    while True:
                        with urllib.request.urlopen(f"http://127.0.0.1:{port}/system.json") as response:
                            stats = json.load(response)["bridge"]
                        if seconds is None:
                            # Finish on evidence, not an eight-second latency sample:
                            # the budget has dropped traffic and every healthy reader
                            # has continued receiving despite the non-reading client.
                            if all(event.is_set() for event in ready) and (
                                    not stalled or int(stats["ws_drops"]) > 0):
                                break
                        if time.monotonic() >= end:
                            if seconds is None:
                                raise AssertionError("bridge made no progress before deadline")
                            break
                        time.sleep(.005)
                finally:
                    stop.set()
                results = [reader.result() for reader in readers]
            elapsed = time.monotonic() - started
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
    def test_limit_budget_and_stalled_isolation(self):
        workspace = Path(os.environ["TEST_SRCDIR"]) / os.environ.get("TEST_WORKSPACE", "_main")
        result = probe(workspace / "studio/bridge", 4, stalled=True, excess=True)
        print(json.dumps(result), flush=True)
        self.assertTrue(result["excess_refused"])
        self.assertEqual(result["stats"]["clients"], 4)
        self.assertGreaterEqual(int(result["stats"]["refused"]), 1)
        self.assertGreater(int(result["stats"]["ws_drops"]), 0)
        self.assertLessEqual(int(result["stats"]["egress_bytes"]),
                             160000 * result["elapsed_s"] + 64000)
        for reader in result["readers"]:
            self.assertGreaterEqual(reader["frames"], 16)

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
