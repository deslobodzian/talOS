"""Free wildcard UDP port selection for test and tool launches."""

import random
import socket

# Below Linux/macOS OS ephemeral port ranges (32768..65535) so transient
# DNS/connect() lookups on the host do not grab a port between release and
# launch.
_PORT_BASE = 20000
_PORT_SPAN = 12000


def udp_ports():
    """Two distinct free UDP ports on 0.0.0.0 below the OS ephemeral range
    (20000..31999; docs/TESTING.md §6.4: probed and released before the
    launch, since the launcher refuses port 0).

    talOS/build/sim_check.py keeps an inline copy with the same probe logic
    because talos_sim_test (contract.bzl) and standalone py_test targets run
    sim_check.py directly via srcs without depending on //talOS/build:udp_ports;
    udp_ports_test.py tests both implementations together.
    """
    start = random.randrange(_PORT_SPAN)
    held = []
    ports = []
    try:
        for offset in range(_PORT_SPAN):
            port = _PORT_BASE + (start + offset) % _PORT_SPAN
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.bind(("0.0.0.0", port))
            except OSError:
                sock.close()
                continue
            held.append(sock)
            ports.append(port)
            if len(ports) == 2:
                return ports[0], ports[1]
    finally:
        for sock in held:
            sock.close()
    raise RuntimeError(
        f"fewer than 2 free UDP ports in "
        f"{_PORT_BASE}-{_PORT_BASE + _PORT_SPAN - 1}")
