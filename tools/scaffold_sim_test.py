"""e2e: a robot scaffolded with a C++ velocity mechanism (gripper) and a
position mechanism (position_cpp), edited exactly as their `Next:` output
says and run the way it says (the launcher with sim_gateway --physics and
its mode and joystick schedules, as `./sim.sh --sim-gateway` starts them),
moves each motor to the goal its teleop binding holds while button 3 is
held. The node binaries are new_subsystem's goldens, which
new_subsystem_test keeps equal to its output."""

import json
import os
import socket
import subprocess
import tempfile
import unittest

from _posixshmem import shm_unlink  # what multiprocessing.shared_memory uses
from python.runfiles import runfiles

from tools import new_robot_test as harness
from tools import new_subsystem
from tools import sync_manifest

MECHANISMS = (("gripper", "velocity"), ("position_cpp", "position"))

# talOS/introspection/registry.h kRegistryPath. The launcher creates the
# registry and deliberately leaves it for later sessions, so the test
# unlinks its own after the run.
REGISTRY = "/talos_registry.6"

# Bounds on a hang, not on the run (which lasts SIM_DURATION_S): together
# they stay below Bazel's 60 s `short` timeout, so a hang fails with the
# test's diagnostics instead of being killed by Bazel.
LAUNCH_TIMEOUT_S = 40
LOG_DUMP_TIMEOUT_S = 5


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
    """Two distinct free loopback UDP ports (TESTING.md §6: bound at 0)."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as gateway, \
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as bridge:
        gateway.bind(("127.0.0.1", 0))
        bridge.bind(("127.0.0.1", 0))
        return gateway.getsockname()[1], bridge.getsockname()[1]


def scaffold(work):
    """The robot new_robot writes with the gripper, plus position_cpp from
    new_subsystem, with their node binaries standing in for a build."""
    harness.generate(work, "gripper", "cpp")
    writes, _ = new_subsystem.plan(work, "my_robot", "position_cpp", "cpp",
                                   "position")
    for path, _, content in writes:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            f.write(content)
    harness.stand_in_binary(work, "position_cpp", "cpp")


def follow_next(work):
    """Appends each mechanism's simulation block to the simulation config,
    as its `Next:` says; returns that config, the one `Next:` runs."""
    for mechanism, kind in MECHANISMS:
        harness.append_simulation_block(work, mechanism, kind)
    return harness.SIM_CONFIG


def states(log_dump, session, mechanism):
    """Every /<mechanism>/state the node published, decoded."""
    dumped = subprocess.run(
        [log_dump, "--format", "jsonl", "--node", mechanism, "--topic",
         f"/{mechanism}/state", session],
        capture_output=True, text=True, timeout=LOG_DUMP_TIMEOUT_S,
        check=True)
    rows = [json.loads(line) for line in dumped.stdout.splitlines()
            if line.startswith("{")]
    decoded = []
    for row in rows:
        fields = {}
        for item in row["decoded"].strip("{}").split(", "):
            key, value = item.split(": ", 1)
            fields[key] = value
        decoded.append((row["publish_ms"], fields))
    return decoded


class ScaffoldSimTest(unittest.TestCase):
    # REQ-DEV-23
    def test_next_steps_move_each_mechanism_to_its_goal(self):
        r = runfiles.Create()
        log_dump = r.Rlocation("_main/talOS/launcher/log_dump")
        with tempfile.TemporaryDirectory() as tmp:
            work, launcher, env = harness.workspace(tmp)
            scaffold(work)
            config = follow_next(work)
            subsystems, _ = sync_manifest.effective(work, config)

            gateway_port, bridge_port = udp_ports()
            # Bazel's TEST_TMPDIR is the shared-memory namespace
            # (ARCHITECTURE.md "Topics and messages") and holds the lock
            # files, which Bazel removes with it; only the registry segment
            # is left to unlink.
            namespace = os.environ.get("TEST_TMPDIR") or tmp
            env.pop("TALOS_SHM_NAMESPACE", None)
            env["TEST_TMPDIR"] = namespace
            self.addCleanup(unlink_registry, namespace)
            gateway_args = [
                "--physics", "--port", str(gateway_port), "--remote-port",
                str(bridge_port), "--mode-schedule",
                new_subsystem.MODE_SCHEDULE, "--joystick-schedule",
                new_subsystem.JOYSTICK_SCHEDULE]
            args = [launcher, "--config", config, "--sim",
                    "--start-sim-gateway"]
            for arg in gateway_args:
                args += ["--sim-gateway-arg", arg]
            output = os.path.join(tmp, "sessions")
            args += ["--remote", "127.0.0.1", "--remote-port",
                     str(gateway_port), "--local-port", str(bridge_port),
                     "--output-dir", output, "--duration-s",
                     str(new_subsystem.SIM_DURATION_S)]
            # The timeout only bounds a hang; the run lasts SIM_DURATION_S.
            try:
                run = subprocess.run(args, cwd=work, env=env,
                                     capture_output=True, text=True,
                                     timeout=LAUNCH_TIMEOUT_S)
            except subprocess.TimeoutExpired as hung:
                self.fail(f"launcher still running after {hung.timeout:.0f} s:"
                          f" {hung.stdout!r} {hung.stderr!r}")
            diagnostics = run.stdout + run.stderr
            self.assertEqual(run.returncode, 0, diagnostics)
            (session,) = os.listdir(output)
            session = os.path.join(output, session)

            for mechanism, kind in MECHANISMS:
                with self.subTest(mechanism=mechanism):
                    (binding,) = subsystems[mechanism]["teleop"]["bindings"]
                    driven = [fields for _, fields in
                              states(log_dump, session, mechanism)
                              if fields["enabled"] == "true"]
                    self.assertTrue(driven, f"{mechanism} never drove: "
                                    f"{diagnostics}")
                    if kind == "position":
                        goal = binding["while_held"]["position_rot"]
                        tolerance = subsystems[mechanism][mechanism][
                            "tolerance_rot"]
                        positions = [float(f["position_rot"])
                                     for f in driven]
                        at_goal = [f for f in driven
                                   if f["at_goal"] == "true"]
                        self.assertTrue(
                            at_goal,
                            f"{mechanism} never reached {goal} rot within "
                            f"{tolerance}: positions "
                            f"{min(positions)}..{max(positions)}, homing "
                            f"{sorted({f['homing'] for f in driven})}")
                        self.assertLessEqual(
                            abs(float(at_goal[0]["position_rot"]) - goal),
                            tolerance)
                    else:
                        goal = binding["while_held"]["velocity_rps"]
                        fastest = max(float(f["velocity_rps"])
                                      for f in driven)
                        # The velocity loop holds its demand: within 1%.
                        self.assertGreaterEqual(
                            fastest, 0.99 * goal,
                            f"{mechanism} reached {fastest} of {goal} rps")


if __name__ == "__main__":
    unittest.main()
