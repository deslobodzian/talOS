"""unit (REQ-DEV-25, REQ-DEV-30): UDP port helpers pick two distinct free
0.0.0.0 ports below the OS ephemeral range (20000..31999) and reject any port
already held on a non-loopback address."""

import errno
import socket
import unittest
from unittest import mock

from talOS.build import sim_check
from talOS.build import udp_ports

PORT_BASE = 20000
PORT_MAX = 31999


def _bind_holder(port):
    """Bind a UDP socket on 127.0.0.2:port when the host permits it (Linux
    allows 127.0.0.1:port to bind alongside 127.0.0.2:port, while 0.0.0.0:port
    fails EADDRINUSE), falling back to 127.0.0.1:port on hosts where 127.0.0.2
    is unassigned (macOS)."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(("127.0.0.2", port))
        return sock, "127.0.0.2"
    except OSError as exc:
        if exc.errno != errno.EADDRNOTAVAIL:
            sock.close()
            raise
    sock.bind(("127.0.0.1", port))
    return sock, "127.0.0.1"


def _find_free_in_range():
    """Find a currently free UDP port on 0.0.0.0 in [PORT_BASE, PORT_MAX]."""
    for port in range(PORT_BASE, PORT_MAX + 1):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            try:
                probe.bind(("0.0.0.0", port))
            except OSError:
                continue
            return port
    raise RuntimeError(f"no free UDP port in {PORT_BASE}-{PORT_MAX}")


class UdpPortsTest(unittest.TestCase):
    HELPERS = (
        ("udp_ports", udp_ports.udp_ports),
        ("sim_check", sim_check.udp_ports),
    )

    # REQ-DEV-25, REQ-DEV-30
    def test_returns_two_distinct_free_wildcard_ports_below_ephemeral(self):
        for name, helper in self.HELPERS:
            with self.subTest(helper=name):
                first, second = helper()
                self.assertNotEqual(first, second)
                for port in (first, second):
                    self.assertGreaterEqual(port, PORT_BASE)
                    self.assertLessEqual(port, PORT_MAX)
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s1, \
                        socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s2:
                    s1.bind(("0.0.0.0", first))
                    s2.bind(("0.0.0.0", second))

    # REQ-DEV-25, REQ-DEV-30
    def test_rejects_port_held_on_non_loopback_address(self):
        held_port = _find_free_in_range()
        holder, bound_ip = _bind_holder(held_port)
        self.addCleanup(holder.close)
        if bound_ip == "127.0.0.2":
            # Confirm the exact Linux asymmetry that caused hardware_node's
            # 0.0.0.0 bind to fail after a 127.0.0.1 probe succeeded.
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as loopback:
                loopback.bind(("127.0.0.1", held_port))
        start_offset = held_port - PORT_BASE
        for name, helper in self.HELPERS:
            with self.subTest(helper=name):
                with mock.patch("random.randrange", return_value=start_offset):
                    first, second = helper()
                self.assertNotEqual(first, second)
                self.assertNotIn(held_port, (first, second))
                for port in (first, second):
                    self.assertGreaterEqual(port, PORT_BASE)
                    self.assertLessEqual(port, PORT_MAX)


if __name__ == "__main__":
    unittest.main()
