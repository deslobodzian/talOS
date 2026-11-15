"""e2e: `bazel run //:sim -- --duration-s 3` runs this robot repository's
config against talOS's simulator, from @talos labels alone: the launcher
exits 0 and every member of the session ran and exited 0."""

import json
import os
import socket
import subprocess
import tempfile
import unittest

from _posixshmem import shm_unlink  # what multiprocessing.shared_memory uses
from python.runfiles import runfiles

DURATION_S = 3
# Bounds a hang, not the run, which lasts DURATION_S: below Bazel's 60 s
# `short` timeout, so a hang fails with the launcher's output.
LAUNCH_TIMEOUT_S = 40
MEMBERS = ["gate", "hardware_node", "roller", "sim_gateway", "telemetry",
           "teleop"]
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
    """Two distinct free loopback UDP ports (docs/TESTING.md §6: bound at 0
    and released before the launch, since the launcher refuses port 0)."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as gateway, \
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as bridge:
        gateway.bind(("127.0.0.1", 0))
        bridge.bind(("127.0.0.1", 0))
        return gateway.getsockname()[1], bridge.getsockname()[1]


class SimSmokeTest(unittest.TestCase):
    # REQ-DEV-30
    def test_sim_runs_every_member(self):
        sim = runfiles.Create().Rlocation(os.environ["SIM"])
        gateway_port, bridge_port = udp_ports()
        # TEST_TMPDIR is the shared-memory namespace (talOS ARCHITECTURE.md
        # "Topics and messages"); only the registry outlives the session.
        namespace = os.environ["TEST_TMPDIR"]
        self.addCleanup(unlink_registry, namespace)
        with tempfile.TemporaryDirectory() as out:
            args = [sim, "--duration-s", str(DURATION_S), "--output-dir", out,
                    "--remote", "127.0.0.1",
                    "--remote-port", str(gateway_port),
                    "--local-port", str(bridge_port)]
            try:
                run = subprocess.run(args, capture_output=True, text=True,
                                     timeout=LAUNCH_TIMEOUT_S, check=False)
            except subprocess.TimeoutExpired as hung:
                self.fail(f"sim still running after {hung.timeout:.0f} s "
                          f"(ports {gateway_port}, {bridge_port}): "
                          f"{hung.stdout!r} {hung.stderr!r}")
            diagnostics = (f"ports {gateway_port}, {bridge_port} (released "
                           f"before the launch; another process may have "
                           f"taken one)\n{run.stdout}{run.stderr}")
            self.assertEqual(run.returncode, 0, diagnostics)
            # The launcher writes into <output-dir>/<session_id>/.
            sessions = [d for d in os.listdir(out)
                        if os.path.isdir(os.path.join(out, d))]
            self.assertEqual(len(sessions), 1, sessions)
            with open(os.path.join(out, sessions[0], "manifest.json"),
                      encoding="utf-8") as f:
                nodes = json.load(f)["nodes"]
        self.assertEqual(sorted(n["name"] for n in nodes), MEMBERS,
                         diagnostics)
        for node in nodes:
            self.assertTrue(node["runs"], f"{node['name']} never ran")
            for run_ in node["runs"]:
                self.assertEqual(
                    (run_["exit_code"], run_["exit_signal"]), (0, 0),
                    f"{node['name']}: {run_} {diagnostics}")


if __name__ == "__main__":
    unittest.main()
