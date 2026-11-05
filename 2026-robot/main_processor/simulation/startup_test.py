"""sim_gateway must fail fast without --config instead of running dead.

Regression guard: an unconfigured Gateway stamps config_id 0, which every
mechanism's AcceptState gate rejects, so the graph runs green and motionless.
The binary therefore requires --config; this test pins that contract plus
the --describe envelope the launcher probes.
"""

import json
import os
import subprocess
import socket
import struct
import tempfile
import time
from pathlib import Path
import unittest


def binary():
    workspace = Path(os.environ["TEST_SRCDIR"]) / os.environ.get(
        "TEST_WORKSPACE", "_main")
    return workspace / "2026-robot/main_processor/simulation/sim_gateway"


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


if __name__ == "__main__":
    unittest.main()
