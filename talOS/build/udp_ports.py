"""Free loopback UDP port selection for test and tool launches."""

import socket


def udp_ports():
    """Two distinct free loopback UDP ports (docs/TESTING.md §6.4: bound at
    0 and released before the launch, since the launcher refuses port 0)."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as gateway, \
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as bridge:
        gateway.bind(("127.0.0.1", 0))
        bridge.bind(("127.0.0.1", 0))
        return gateway.getsockname()[1], bridge.getsockname()[1]
