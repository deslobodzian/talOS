"""Gateway startup, full robot sessions, and transient producer-stall recovery."""

import json
import os
import signal
import subprocess
import socket
import struct
import tempfile
import time
from pathlib import Path
import unittest
import uuid


def workspace():
    return Path(os.environ["TEST_SRCDIR"]) / os.environ.get(
        "TEST_WORKSPACE", "_main")


def binary():
    return workspace() / "2026-robot/main_processor/simulation/sim_gateway"


def udp_ports():
    # Hold both reservations until distinct ephemeral ports have been selected.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as gateway, \
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as bridge:
        gateway.bind(("127.0.0.1", 0))
        bridge.bind(("127.0.0.1", 0))
        return gateway.getsockname()[1], bridge.getsockname()[1]


class SimGatewayStartup(unittest.TestCase):
    def test_describe_answers_without_hardware(self):
        proc = subprocess.run(
            [str(binary()), "--describe"], capture_output=True, text=True,
            timeout=30)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        doc = json.loads(proc.stdout)
        self.assertEqual(doc["name"], "sim_gateway")
        self.assertEqual(doc["target"],
                         "//2026-robot/main_processor/simulation:sim_gateway")

    def test_missing_config_fails_fast(self):
        proc = subprocess.run(
            [str(binary()), "--duration-s", "1"], capture_output=True,
            text=True, timeout=30)
        self.assertNotEqual(proc.returncode, 0)
        self.assertIn("--config", proc.stderr)

    def test_scheduled_modes_and_generations(self):
        with tempfile.TemporaryDirectory() as tmp, socket.socket(
                socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            config = Path(tmp) / "robot.toml"
            config.write_text(
                '[robot]\nname = "schedule_test"\nperiod_us = 5000\n'
                'controller_address = "127.0.0.1"\n'
                '[subsystems.test.motors.motor]\ncan_id = 1\n',
                encoding="utf-8")
            receiver.bind(("127.0.0.1", 5803))
            receiver.settimeout(0.1)
            with subprocess.Popen(
                    [str(binary()), "--config", str(config),
                     "--mode-schedule", "disabled:0.1,auto:0.2,teleop:0.2",
                     "--duration-s", "0.6", "--alliance", "red"],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    text=True) as proc:
                phases = []
                deadline = time.monotonic() + 10
                try:
                    while time.monotonic() < deadline:
                        try:
                            packet = receiver.recv(2048)
                        except socket.timeout:
                            if proc.poll() is not None:
                                break
                            continue
                        # Protocol v2: 40-byte frame header; DS payload begins
                        # with sample time, mode generation, flags, alliance.
                        frame_type = struct.unpack_from("<H", packet, 12)[0]
                        if frame_type != 24:
                            continue
                        sample, generation, flags, alliance = struct.unpack_from(
                            "<QQIB", packet, 40)
                        self.assertEqual(alliance, 1)
                        self.assertGreater(generation, 0)
                        self.assertLessEqual(generation, sample)
                        if not phases or phases[-1][0] != flags:
                            phases.append((flags, generation))
                        else:
                            self.assertEqual(phases[-1][1], generation)
                    stdout, stderr = proc.communicate(timeout=5)
                finally:
                    if proc.poll() is None:
                        proc.kill()
                        proc.communicate()
                self.assertEqual(proc.returncode, 0, stdout + stderr)
                self.assertEqual([flags for flags, _ in phases], [1, 13, 21])
                generations = [generation for _, generation in phases]
                self.assertEqual(generations, sorted(set(generations)))


class FullStackSimulation(unittest.TestCase):
    def run_session(self, stall_producer=False):
        root = workspace()
        config = root / "2026-robot/main_processor/configuration/robot.toml"
        gw_port, bridge_port = udp_ports()
        env = dict(os.environ, TALOS_SHM_NAMESPACE=f"full-stack-{uuid.uuid4().hex}")
        with tempfile.TemporaryDirectory() as tmp:
            # The launcher resolves labels under bazel-bin; map that layout
            # onto the test's runfiles without changing the real workspace.
            (Path(tmp) / "bazel-bin").symlink_to(root, target_is_directory=True)
            output = Path(tmp) / "sessions"
            processes = []
            stopped_pid = None
            with open(Path(tmp) / "gateway.txt", "w+") as gateway_log, \
                    open(Path(tmp) / "launcher.txt", "w+") as launcher_log:
                try:
                    started = time.monotonic()
                    gateway = subprocess.Popen(
                        [str(binary()), "--config", str(config),
                         "--port", str(gw_port), "--remote-port", str(bridge_port),
                         "--mode-schedule",
                         "disabled:0.2,auto:1.0,teleop:0.6,disabled:0.2",
                         "--duration-s", "2.5"],
                        cwd=root, env=env, stdout=gateway_log,
                        stderr=subprocess.STDOUT, start_new_session=True)
                    processes.append(gateway)
                    launcher = subprocess.Popen(
                        [str(root / "talOS/launcher/launcher"),
                         "--config", str(config), "--sim",
                         "--remote", "127.0.0.1",
                         "--remote-port", str(gw_port),
                         "--local-port", str(bridge_port),
                         "--output-dir", str(output), "--duration-s", "2"],
                        cwd=tmp, env=env, stdout=launcher_log,
                        stderr=subprocess.STDOUT, start_new_session=True)
                    processes.append(launcher)
                    if stall_producer:
                        # The driver publishes teleop targets. Stop it within
                        # the scheduled teleop phase, beyond target freshness
                        # (100 ms) but below the heartbeat stall floor (500 ms).
                        deadline = started + 1.45
                        producer = None
                        while time.monotonic() < deadline:
                            manifests = list(output.glob("*/manifest.json"))
                            if manifests:
                                manifest = json.loads(manifests[0].read_text())
                                producer = next(
                                    (node for node in manifest["nodes"]
                                     if node["name"] == "driver"
                                     and node["pid"] > 0), None)
                            if producer and time.monotonic() >= started + 1.3:
                                break
                            self.assertIsNone(launcher.poll(),
                                              "launcher exited before injection")
                            time.sleep(0.01)
                        self.assertIsNotNone(producer,
                                             "driver did not start during teleop")
                        self.assertLess(time.monotonic(), started + 1.5,
                                        "missed the teleop injection window")
                        stopped_pid = producer["pid"]
                        os.kill(stopped_pid, signal.SIGSTOP)
                        time.sleep(0.25)
                        os.kill(stopped_pid, signal.SIGCONT)
                        stopped_pid = None
                    launcher.wait(timeout=30)
                    gateway.wait(timeout=10)
                    gateway_log.seek(0)
                    launcher_log.seek(0)
                    diagnostics = gateway_log.read() + launcher_log.read()
                    self.assertEqual(gateway.returncode, 0, diagnostics)
                    self.assertEqual(launcher.returncode, 0, diagnostics)
                    manifests = list(output.glob("*/manifest.json"))
                    self.assertEqual(len(manifests), 1, diagnostics)
                    manifest = json.loads(manifests[0].read_text())
                    nodes = {node["name"]: node for node in manifest["nodes"]}
                    self.assertEqual(set(nodes), {
                        "hardware_node", "autonomous", "driver", "drivetrain",
                        "intake", "pose_estimator", "shooter", "telemetry",
                        "teleop", "front_left_camera", "front_right_camera",
                        "rear_left_camera", "rear_right_camera",
                    }, diagnostics)
                    for name, node in nodes.items():
                        with self.subTest(node=name):
                            self.assertEqual(node["exit_code"], 0, diagnostics)
                            self.assertFalse(node["recorder_failed"], diagnostics)
                            self.assertEqual(node["restart_count"], 0, diagnostics)
                    for name in ("hardware_node", "autonomous", "drivetrain",
                                 "pose_estimator", "front_left_camera",
                                 "front_right_camera", "rear_left_camera",
                                 "rear_right_camera"):
                        with self.subTest(log=name):
                            self.assertGreater(nodes[name]["record_count"], 0,
                                               diagnostics)
                            log = manifests[0].parent / f"{name}.tlog"
                            self.assertGreater(log.stat().st_size, 0)
                finally:
                    if stopped_pid is not None:
                        try:
                            os.kill(stopped_pid, signal.SIGCONT)
                        except ProcessLookupError:
                            pass  # Still reap the remaining process group.
                    # Kill the whole launcher process group on a failed
                    # assertion/timeout so its children cannot leak into tests.
                    for proc in reversed(processes):
                        if proc.poll() is None:
                            os.killpg(proc.pid, signal.SIGKILL)
                        proc.wait(timeout=10)

    def test_full_stack_closed_loop(self):
        self.run_session()

    def test_transient_producer_stall(self):
        self.run_session(stall_producer=True)


if __name__ == "__main__":
    unittest.main()
