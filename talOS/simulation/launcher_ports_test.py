"""Integration: the launcher starts the simulator on the config's gateway
ports, with no port written by hand, so the bridge receives its State."""

import json
import os
import signal
import socket
import subprocess
import tempfile
import threading
import unittest

from _posixshmem import shm_unlink  # what multiprocessing.shared_memory uses
from python.runfiles import runfiles

# Each bounds a hang, not the run (docs/TESTING.md §6): the test waits for
# the bridge's first accepted State, then stops the session. Together below
# Bazel's 60 s `short` timeout, so a hang fails with the launcher's output.
FIRST_STATE_TIMEOUT_S = 15
STOP_TIMEOUT_S = 20
LOG_DUMP_TIMEOUT_S = 10
# What hardware_node prints on the first State it accepts from a gateway
# boot (HardwareNode::ReportGatewayBuild); the launcher echoes it.
FIRST_STATE_LINE = "hardware_node: gateway boot "
# talOS/introspection/registry.h kRegistryPath. The launcher leaves the
# registry for later sessions, so the test unlinks its own.
REGISTRY = "/talos_registry.6"


def shm_name(topic, namespace):
    """talos::ShmObjectName (talOS/memory/shared_memory_ptr.h)."""
    mapped = "/" + topic.lstrip("/").replace("/", ".")
    value = 0xCBF29CE484222325
    for byte in namespace.encode() + b"\0" + mapped.encode():
        value = ((value ^ byte) * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return f"/~{value:016x}"


def unlink_registry(namespace):
    try:
        shm_unlink(shm_name(REGISTRY, namespace))
    except FileNotFoundError:
        pass


def udp_ports():
    """Two distinct free loopback UDP ports, neither 5802 nor 5803
    (docs/TESTING.md §6: bound at 0 and released before the launch, since
    the launcher refuses port 0)."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as gateway, \
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as bridge:
        gateway.bind(("127.0.0.1", 0))
        bridge.bind(("127.0.0.1", 0))
        return gateway.getsockname()[1], bridge.getsockname()[1]


def states_published(log_dump, session):
    """How many /hw/state the bridge published: one per State it accepted
    from the simulator."""
    dumped = subprocess.run(
        [log_dump, "--format", "jsonl", "--node", "hardware_node", "--topic",
         "/hw/state", session],
        capture_output=True, text=True, timeout=LOG_DUMP_TIMEOUT_S,
        check=True)
    rows = [json.loads(line) for line in dumped.stdout.splitlines()
            if line.startswith("{")]
    return sum(1 for row in rows if row["kind"] == "SEND")


class LauncherPortsTest(unittest.TestCase):
    # REQ-DEV-31
    def test_simulator_takes_the_config_ports_from_the_launcher(self):
        r = runfiles.Create()
        launcher = r.Rlocation(os.environ["LAUNCHER"])
        log_dump = r.Rlocation(os.environ["LOG_DUMP"])
        # The launcher resolves //pkg:name to ./bazel-bin/pkg/name; the
        # runfiles tree has the same layout under the main repository.
        root = os.path.dirname(os.path.dirname(os.path.dirname(launcher)))
        controller_port, local_port = udp_ports()
        namespace = os.environ["TEST_TMPDIR"]
        self.addCleanup(unlink_registry, namespace)
        with tempfile.TemporaryDirectory() as work:
            os.symlink(root, os.path.join(work, "bazel-bin"))
            config = os.path.join(work, "robot.toml")
            with open(config, "w", encoding="utf-8") as f:
                f.write("[robot]\n"
                        'name = "simulator_ports"\n'
                        "period_us = 5000\n"
                        'sim_gateway = "//talOS/simulation:sim_gateway"\n'
                        'controller_address = "127.0.0.1"\n'
                        f"controller_port = {controller_port}\n"
                        f"local_port = {local_port}\n"
                        # No node: the session is the bridge and the
                        # simulator alone.
                        "[subsystems]\n")
            output = os.path.join(work, "sessions")
            args = [launcher, "--config", config, "--sim",
                    "--start-sim-gateway", "--output-dir", output]
            lines = []
            first_state = threading.Event()

            def drain(stream):
                for line in stream:
                    lines.append(line)
                    if line.startswith(FIRST_STATE_LINE):
                        first_state.set()

            with subprocess.Popen(args, cwd=work, stdout=subprocess.PIPE,
                                  stderr=subprocess.STDOUT,
                                  text=True) as run:
                reader = threading.Thread(target=drain, args=(run.stdout,))
                reader.start()
                received = first_state.wait(FIRST_STATE_TIMEOUT_S)
                # SIGINT stops the session in order, as Ctrl-C does.
                run.send_signal(signal.SIGINT)
                try:
                    run.wait(STOP_TIMEOUT_S)
                except subprocess.TimeoutExpired:
                    run.kill()
                    run.wait()
                reader.join()
            diagnostics = (f"ports {controller_port}, {local_port} "
                           f"(released before the launch; another process "
                           f"may have taken one)\n{''.join(lines)}")
            self.assertTrue(received,
                            f"the bridge accepted no State within "
                            f"{FIRST_STATE_TIMEOUT_S} s: {diagnostics}")
            self.assertEqual(run.returncode, 0, diagnostics)
            (session,) = os.listdir(output)
            published = states_published(log_dump,
                                         os.path.join(output, session))
        # The bridge publishes each State it accepts; one proves the State
        # crossed the config's ports.
        self.assertGreater(published, 0,
                           f"no /hw/state published: {diagnostics}")


if __name__ == "__main__":
    unittest.main()
