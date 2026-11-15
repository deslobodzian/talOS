"""talOS's sim_gateway sending its State to the wrong port.

The simulator a session runs in sim_check_no_state_test's config: it execs
//talOS/simulation:sim_gateway with every argument the launcher gives it,
except that `--remote-port` (where it sends State: the bridge's port) names
a loopback socket this process opened and nobody reads. So the gateway
describes, registers, runs and exits 0 on the session's stop as in any
session, and the bridge never receives a State, as with a simulator given
the wrong ports.
"""

import os
import socket
import sys

from python.runfiles import runfiles


def main(argv):
    gateway = runfiles.Create().Rlocation(
        "_main/talOS/simulation/sim_gateway")
    if "--remote-port" in argv and "--describe" not in argv:
        # Kept open across the exec, so State lands in its buffer rather than
        # on a closed port.
        void = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        void.bind(("127.0.0.1", 0))
        void.set_inheritable(True)
        argv[argv.index("--remote-port") + 1] = str(void.getsockname()[1])
    os.execv(gateway, [gateway] + argv)


if __name__ == "__main__":
    main(sys.argv[1:])
