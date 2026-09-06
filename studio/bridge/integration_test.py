"""Exercise real IPC -> UDP/WebSocket bytes without a third-party WS client."""
import base64
import os
from pathlib import Path
import signal
import socket
import struct
import subprocess
import tempfile
import time
import unittest
import urllib.request


def receive(sock, count):
    data = b""
    while len(data) < count:
        chunk = sock.recv(count - len(data))
        if not chunk:
            raise EOFError("WebSocket closed")
        data += chunk
    return data


def frame(sock):
    first, length = receive(sock, 2)
    assert first == 0x82, first
    length &= 127
    if length == 126:
        length = struct.unpack("!H", receive(sock, 2))[0]
    elif length == 127:
        length = struct.unpack("!Q", receive(sock, 8))[0]
    return receive(sock, length)


class BridgeIntegration(unittest.TestCase):
    def test_transports(self):
        workspace = Path(os.environ["TEST_SRCDIR"]) / os.environ.get("TEST_WORKSPACE", "_main")
        binaries = workspace / "studio/bridge"
        topic = f"/studio_test_{os.getpid()}"
        processes = []
        with tempfile.TemporaryDirectory() as webroot, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as udp:
            Path(webroot, "index.html").write_text("studio integration")
            udp.bind(("127.0.0.1", 0))
            udp.settimeout(5)
            with socket.socket() as reservation:
                reservation.bind(("127.0.0.1", 0))
                http_port = reservation.getsockname()[1]
            try:
                mock = subprocess.Popen([str(binaries / "studio_mock"), topic])
                processes.append(mock)
                # Creation of the shared-memory header precedes the first HTTP bind.
                time.sleep(0.2)
                bridge = subprocess.Popen([str(binaries / "studio_bridge"), "--drop-newest-publisher", topic,
                                           "127.0.0.1", str(udp.getsockname()[1]), str(http_port), webroot])
                processes.append(bridge)
                deadline = time.monotonic() + 10
                while True:
                    self.assertIsNone(mock.poll(), "mock exited")
                    self.assertIsNone(bridge.poll(), "bridge exited")
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
                    packet = frame(ws)
                    self.assertEqual(struct.unpack_from("<I", packet)[0], len(packet) - 4)
                    self.assertEqual(packet[8:12], b"TLMS")
                    # UDP may contain older packets captured before the WS upgrade.
                    for _ in range(500):
                        datagram = udp.recv(65536)
                        if datagram == packet:
                            break
                    else:
                        self.fail("identical WebSocket frame absent from UDP stream")
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


if __name__ == "__main__":
    unittest.main()
