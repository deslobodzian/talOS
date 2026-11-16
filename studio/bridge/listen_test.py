"""A bridge refuses a port another listener already holds, and says so."""
import os
from pathlib import Path
import socket
import subprocess
import tempfile
import unittest

# Bounds only a hang: a refused listen ends the bridge in milliseconds.
HANG_S = 20


def held_port():
    """A listening socket on the wildcard address, with the options uSockets
    gives every listener it is not told to keep exclusive (SO_REUSEADDR and
    SO_REUSEPORT, dual-stack where the host has IPv6): another bridge, as the
    kernel sees it. Returns (socket, port)."""
    try:
        holder = socket.socket(socket.AF_INET6, socket.SOCK_STREAM)
        holder.setsockopt(socket.IPPROTO_IPV6, socket.IPV6_V6ONLY, 0)
        address = ("::", 0)
    except OSError:
        holder = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        address = ("0.0.0.0", 0)
    holder.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    holder.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    holder.bind(address)
    holder.listen()
    return holder, holder.getsockname()[1]


class Listen(unittest.TestCase):
    # REQ-OPS-34
    def test_a_port_another_listener_holds_is_refused(self):
        """uSockets sets SO_REUSEPORT on a listener unless told
        LIBUS_LISTEN_EXCLUSIVE_PORT, so a second bridge on the robot's port
        (a developer's ./sim.sh on the robot computer) bound beside the first
        and the kernel split /drive connections between two sessions. The
        bridge now listens exclusively: its listen fails, it exits nonzero
        and names the port, and the launcher's restart bound ends it."""
        workspace = Path(os.environ["TEST_SRCDIR"]) / os.environ.get("TEST_WORKSPACE", "_main")
        holder, port = held_port()
        with holder, tempfile.TemporaryDirectory() as webroot:
            bridge = subprocess.Popen(
                [str(workspace / "studio/bridge/studio_bridge"),
                 f"/test/telemetry/l{os.getpid()}", str(port), webroot],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            try:
                out, err = bridge.communicate(timeout=HANG_S)
            except subprocess.TimeoutExpired:
                bridge.kill()
                out, err = bridge.communicate()
                self.fail(f"bridge listened on held port {port}: {out!r}")
            self.assertNotEqual(bridge.returncode, 0, out)
            self.assertNotIn("listening", out)
            self.assertIn(f"cannot listen on port {port}", err)


if __name__ == "__main__":
    unittest.main()
