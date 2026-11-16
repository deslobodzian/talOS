"""Integration: the simulator alone, with no launcher or robot graph.

Each case runs `//talOS/simulation:sim_gateway` as a child process: its
`--describe` output and flag validation, a run without a config, and the
Driver Station and State datagrams it sends over loopback UDP for a config
the case writes itself.
"""

import json
import os
import socket
import struct
import subprocess
import tempfile
import time
import unittest
from pathlib import Path

from python.runfiles import runfiles


def binary():
    return runfiles.Create().Rlocation(os.environ["SIM_GATEWAY"])


class SimGatewayStartup(unittest.TestCase):
    def test_physics_truth_message_preserves_wire_size(self):
        proc = subprocess.run(
            [str(binary()), "--describe", "--physics"], capture_output=True,
            text=True, timeout=30)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertEqual(json.loads(proc.stdout)["sources"][0]["message_bytes"], 56)

    def test_physics_flags_validate_before_starting(self):
        for flag, value in (
                ("--start-pose", "2,2,0"),
                ("--wheel-radius-error", "0.1"),
                ("--wheel-slip-gain", "0.2"),
                ("--gyro-drift-radps", "0.01")):
            with self.subTest(flag=flag):
                proc = subprocess.run(
                    [str(binary()), "--describe", flag, value],
                    capture_output=True, text=True, timeout=30)
                self.assertNotEqual(proc.returncode, 0)
        for flag, value in (
                ("--start-pose", "1,2"), ("--start-pose", "1,2,3,4"),
                ("--start-pose", "nan,2,0"), ("--start-pose", "1,inf,0"),
                ("--start-pose", "1,2,-inf"),
                ("--wheel-radius-error", "-0.5"),
                ("--wheel-radius-error", "0.5"),
                ("--wheel-radius-error", "nan"),
                ("--wheel-slip-gain", "-0.01"),
                ("--wheel-slip-gain", "inf"),
                ("--gyro-drift-radps", "nan")):
            with self.subTest(flag=flag, value=value):
                proc = subprocess.run(
                    [str(binary()), "--describe", "--physics", flag, value],
                    capture_output=True, text=True, timeout=30)
                self.assertNotEqual(proc.returncode, 0)

    def test_missing_config_fails_fast(self):
        proc = subprocess.run(
            [str(binary()), "--duration-s", "1"], capture_output=True,
            text=True, timeout=30)
        self.assertNotEqual(proc.returncode, 0)

    def test_scheduled_modes_and_generations(self):
        with tempfile.TemporaryDirectory() as tmp, socket.socket(
                socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            config = Path(tmp) / "robot.toml"
            config.write_text(
                '[robot]\nname = "schedule_test"\nperiod_us = 5000\n'
                'controller_address = "127.0.0.1"\n'
                '[subsystems.test.motors.motor]\ncan_id = 1\n',
                encoding="utf-8")
            # Ephemeral ports: the default 5802/5803 belong to whatever
            # session or test is already running on the host.
            receiver.bind(("127.0.0.1", 0))
            receiver.settimeout(0.1)
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserve:
                reserve.bind(("127.0.0.1", 0))
                gateway_port = reserve.getsockname()[1]
            with subprocess.Popen(
                    [str(binary()), "--config", str(config),
                     "--port", str(gateway_port),
                     "--remote-port", str(receiver.getsockname()[1]),
                     "--mode-schedule", "disabled:0.1,auto:0.2,teleop:0.2",
                     "--joystick-schedule",
                     "0:axes=0,0,0;buttons=0;pov=-1|"
                     "0.3:axes=0,-0.5,0;buttons=5;pov=90",
                     "--duration-s", "0.6", "--alliance", "red"],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    text=True) as proc:
                phases = []
                joystick_samples = []
                boots = set()
                previous_sample = None
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
                        # with sample time, mode generation, boot id, flags,
                        # alliance.
                        frame_type = struct.unpack_from("<H", packet, 12)[0]
                        if frame_type != 24:
                            continue
                        sample, generation, boot, flags, alliance = (
                            struct.unpack_from("<QQQIB", packet, 40))
                        # The gateway's random boot id (odd), one per run.
                        self.assertEqual(boot & 1, 1)
                        boots.add(boot)
                        # Empty FMS strings end at byte 40. Joystick axes and
                        # POVs are variable-length, not native struct arrays.
                        connected, axes, buttons, povs, mask = struct.unpack_from(
                            "<BBBBI", packet, 40 + 40)
                        axis = struct.unpack_from("<f", packet, 40 + 52)[0]
                        pov = struct.unpack_from("<h", packet, 40 + 60)[0]
                        joystick_samples.append(
                            (connected, axes, buttons, povs, mask, axis, pov))
                        self.assertEqual(alliance, 1)
                        self.assertGreater(generation, 0)
                        self.assertLessEqual(generation, sample)
                        if not phases or phases[-1][0] != flags:
                            # A new mode goes out on the tick that observes
                            # it, not at the next 20 ms slot.
                            self.assertEqual(generation, sample)
                            phases.append((flags, generation))
                        else:
                            self.assertEqual(phases[-1][1], generation)
                            self.assertGreaterEqual(sample - previous_sample,
                                                    20000)
                        previous_sample = sample
                    stdout, stderr = proc.communicate(timeout=5)
                finally:
                    if proc.poll() is None:
                        proc.kill()
                        proc.communicate()
                self.assertEqual(proc.returncode, 0, stdout + stderr)
                self.assertEqual([flags for flags, _ in phases], [1, 13, 21])
                self.assertEqual(len(boots), 1)
                generations = [generation for _, generation in phases]
                self.assertEqual(generations, sorted(set(generations)))
                self.assertIn((1, 3, 32, 1, 5, -0.5, 90), joystick_samples)

    # REQ-DEV-10
    def test_physics_home_switches_read_from_the_config(self):
        """[digital_inputs.*] sim_motor binds a home switch in --physics: with
        the lift motor at rest at 0 rotations, a switch closed at or below
        -0.1 reads open (true, normally open) and one closed at or below 0.1
        reads closed (false); closed_above and closed_reads apply; an input
        with no sim_motor reads false."""
        with tempfile.TemporaryDirectory() as tmp, socket.socket(
                socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            config = Path(tmp) / "robot.toml"
            config.write_text(
                '[robot]\nname = "home_switch_test"\nperiod_us = 5000\n'
                'controller_address = "127.0.0.1"\n'
                '[subsystems.test.motors.lift]\ncan_id = 1\n'
                '[subsystems.test.digital_inputs]\n'
                'a_away = { dio = 0, sim_motor = "lift", '
                'sim_closed_at_rot = -0.1 }\n'
                'b_at = { dio = 1, sim_motor = "lift", '
                'sim_closed_at_rot = 0.1 }\n'
                'c_above = { dio = 2, sim_motor = "lift", '
                'sim_closed_at_rot = -0.1, sim_closed_above = true, '
                'sim_closed_reads = true }\n'
                'd_unbound = { dio = 3 }\n',
                encoding="utf-8")
            receiver.bind(("127.0.0.1", 0))
            receiver.settimeout(0.1)
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reserve:
                reserve.bind(("127.0.0.1", 0))
                gateway_port = reserve.getsockname()[1]
            with subprocess.Popen(
                    [str(binary()), "--config", str(config), "--physics",
                     "--port", str(gateway_port),
                     "--remote-port", str(receiver.getsockname()[1]),
                     "--duration-s", "0.3"],
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    text=True) as proc:
                inputs = None
                deadline = time.monotonic() + 10
                try:
                    while inputs is None and time.monotonic() < deadline:
                        try:
                            packet = receiver.recv(4096)
                        except socket.timeout:
                            if proc.poll() is not None:
                                break
                            continue
                        if struct.unpack_from("<H", packet, 12)[0] != 20:
                            continue
                        # talOS/hardware/messages.cc Encode(State): an 84-byte
                        # base after the 40-byte frame header, whose counts
                        # start at 70; 45 bytes per motor; then (id, valid,
                        # value) per digital input.
                        motors, sensors, digital_inputs = struct.unpack_from(
                            "<HHH", packet, 40 + 70)
                        self.assertEqual((motors, sensors), (1, 0))
                        offset = 40 + 84 + 45 * motors
                        inputs = [struct.unpack_from("<H??", packet,
                                                     offset + 4 * i)
                                  for i in range(digital_inputs)]
                    stdout, stderr = proc.communicate(timeout=5)
                finally:
                    if proc.poll() is None:
                        proc.kill()
                        proc.communicate()
                self.assertEqual(proc.returncode, 0, stdout + stderr)
                # Logical ids follow the sorted device names (a..d, then lift).
                self.assertEqual(inputs, [(1, True, True), (2, True, False),
                                          (3, True, True), (4, True, False)])


if __name__ == "__main__":
    unittest.main()
