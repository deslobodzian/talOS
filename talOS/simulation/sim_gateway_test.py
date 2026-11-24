"""Integration: the simulator alone, with no launcher or robot graph.

Each case runs `//talOS/simulation:sim_gateway` as a child process: its
`--describe` output and flag validation, a run without a config, the
Driver Station and State datagrams it sends over loopback UDP for a config
the case writes itself, and its `/sim/state` RTMS publication under `--physics`.
"""

import json
import math
import mmap
import os
import signal
import socket
import struct
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

from _posixshmem import shm_open, shm_unlink
from python.runfiles import runfiles
from talOS.build.udp_ports import udp_ports
from talOS.memory.shm_name import shm_name

# talOS/hardware/messages.h GatewayProgram::kSimGateway.
GATEWAY_PROGRAM_SIM_GATEWAY = 3

# talOS/rtms/rtms.h shared-memory ring header layout (version 9).
RTMS_MAGIC = 0x32534D54534F4C41
RTMS_VERSION = 9
RTMS_HEADER_BYTES = 2368
RTMS_WRITER_SEQ_OFFSET = 256

# geometry::OdometrySample (talOS/msgs/geometry.fbs): u64 sample_time_ns,
# Pose2d (x_m, y_m, yaw_rad), Twist2d (vx_mps, vy_mps, omega_radps).
ODOMETRY_SAMPLE_FORMAT = "<Qdddddd"
ODOMETRY_SAMPLE_BYTES = struct.calcsize(ODOMETRY_SAMPLE_FORMAT)

SIM_STATE_TOPIC = "/sim/state"
REGISTRY_TOPIC = "/talos_registry.6"


def binary():
    return runfiles.Create().Rlocation(os.environ["SIM_GATEWAY"])


def _unlink_shm(topic, namespace):
    try:
        shm_unlink(shm_name(topic, namespace))
    except FileNotFoundError:
        pass


def _isolated_env(test_case, namespace):
    test_case.addCleanup(_unlink_shm, SIM_STATE_TOPIC, namespace)
    test_case.addCleanup(_unlink_shm, REGISTRY_TOPIC, namespace)
    env = os.environ.copy()
    env["TALOS_SHM_NAMESPACE"] = namespace
    return env


def _decode_state_build(packet):
    """Return (revision, program, dirty) from a State frame (type 20)."""
    return struct.unpack_from("<QBB", packet, 40 + 56)


class _RtmsRingView:
    """Read-only view of a live RTMS shared-memory segment."""

    def __init__(self, mm):
        self._mm = mm
        (self.magic, self.version, self.segment_id, self.total_bytes,
         self.slots, self.message_bytes, self.message_alignment,
         self.data_offset, self.slot_stride, self.schema_hash) = (
             struct.unpack_from("<10Q", mm, 0))
        self.schema_name = mm[80:208].split(b"\0", 1)[0].decode("ascii")

    def writer_sequence(self):
        return struct.unpack_from("<Q", self._mm, RTMS_WRITER_SEQ_OFFSET)[0]

    def read_odometry(self, seq):
        slot_index = seq & (self.slots - 1)
        offset = self.data_offset + slot_index * self.slot_stride
        return struct.unpack_from(ODOMETRY_SAMPLE_FORMAT, self._mm, offset)

    def close(self):
        self._mm.close()


def _try_open_rtms_readonly(topic, namespace):
    """Map an existing initialized RTMS segment read-only, or return None."""
    try:
        fd = shm_open(shm_name(topic, namespace), os.O_RDONLY, 0)
    except FileNotFoundError:
        return None
    try:
        size = os.fstat(fd).st_size
        if size < RTMS_HEADER_BYTES:
            return None
        mm = mmap.mmap(fd, size, mmap.MAP_SHARED, mmap.PROT_READ)
    finally:
        os.close(fd)
    magic = struct.unpack_from("<Q", mm, 0)[0]
    if magic != RTMS_MAGIC:
        mm.close()
        return None
    view = _RtmsRingView(mm)
    if size < view.total_bytes:
        view.close()
        return None
    return view


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

    # REQ-CONTRACT-30
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
            gateway_port, _ = udp_ports()
            with subprocess.Popen(
                    [str(binary()), "--config", str(config),
                     "--port", str(gateway_port),
                     "--remote-port", str(receiver.getsockname()[1]),
                     "--mode-schedule", "disabled:0.1,auto:0.2,teleop:0.2",
                     "--joystick-schedule",
                     "0:axes=0,0,0;buttons=0;pov=-1|"
                     "0.3:axes=0,-0.5,0;buttons=5;pov=90",
                     "--duration-s", "0.6", "--alliance", "red"],
                    env=_isolated_env(self, tmp),
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    text=True) as proc:
                phases = []
                joystick_samples = []
                boots = set()
                state_builds = set()
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
                        if frame_type == 20:
                            state_builds.add(_decode_state_build(packet))
                            continue
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
                self.assertEqual(state_builds,
                                 {(0, GATEWAY_PROGRAM_SIM_GATEWAY, 0)})
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
            gateway_port, _ = udp_ports()
            with subprocess.Popen(
                    [str(binary()), "--config", str(config), "--physics",
                     "--port", str(gateway_port),
                     "--remote-port", str(receiver.getsockname()[1]),
                     "--duration-s", "0.3"],
                    env=_isolated_env(self, tmp),
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

    # REQ-DEV-05, REQ-DEV-23
    def test_physics_reads_simulation_geometry_from_config(self):
        """[simulation] wheel_radius_m and [[simulation.swerve_modules]] in an
        overlay config drive sim_gateway --physics chassis motion, overriding
        both the 0.0508 m default radius and ±0.30 m name inference."""
        wheel_radius_m = 0.08
        x_m = 0.40
        y_m = 0.25
        with tempfile.TemporaryDirectory() as tmp, socket.socket(
                socket.AF_INET, socket.SOCK_DGRAM) as peer:
            base = Path(tmp) / "robot.toml"
            base.write_text(
                '[robot]\nname = "sim_geometry_test"\nperiod_us = 5000\n'
                'command_timeout_us = 100000\n'
                'controller_address = "127.0.0.1"\n'
                '[subsystems.drive.motors]\n'
                'back_left_drive = { can_id = 1 }\n'
                'back_left_steer = { can_id = 2 }\n'
                'back_right_drive = { can_id = 3 }\n'
                'back_right_steer = { can_id = 4 }\n'
                'front_left_drive = { can_id = 5 }\n'
                'front_left_steer = { can_id = 6 }\n'
                'front_right_drive = { can_id = 7 }\n'
                'front_right_steer = { can_id = 8 }\n'
                '[subsystems.drive.sensors.imu]\n'
                'type = "Pigeon2"\ncan_id = 9\n',
                encoding="utf-8")
            overlay = Path(tmp) / "robot_sim.toml"
            overlay.write_text(
                'include = "robot.toml"\n'
                '[simulation]\n'
                f'wheel_radius_m = {wheel_radius_m}\n'
                '[[simulation.swerve_modules]]\n'
                'drive = "front_left_drive"\nsteer = "front_left_steer"\n'
                f'x_m = {x_m}\ny_m = {y_m}\n'
                '[[simulation.swerve_modules]]\n'
                'drive = "front_right_drive"\nsteer = "front_right_steer"\n'
                f'x_m = {x_m}\ny_m = {-y_m}\n'
                '[[simulation.swerve_modules]]\n'
                'drive = "back_left_drive"\nsteer = "back_left_steer"\n'
                f'x_m = {-x_m}\ny_m = {y_m}\n'
                '[[simulation.swerve_modules]]\n'
                'drive = "back_right_drive"\nsteer = "back_right_steer"\n'
                f'x_m = {-x_m}\ny_m = {-y_m}\n',
                encoding="utf-8")
            peer.bind(("127.0.0.1", 0))
            peer.settimeout(0.1)
            gateway_port, _ = udp_ports()
            # Sorted device names assign logical ids:
            # 1: back_left_drive, 2: back_left_steer, 3: back_right_drive,
            # 4: back_right_steer, 5: front_left_drive, 6: front_left_steer,
            # 7: front_right_drive, 8: front_right_steer, 9: imu.
            # With all steer angles at 0, driving right wheels (+y = -0.25)
            # forward and left wheels (+y = +0.25) backward produces pure CCW
            # rotation whose ratio yaw_rate_rps / drive_rps equals
            # (y_m / (x_m^2 + y_m^2)) * wheel_radius_m.
            motor_demands = [
                (1, 1, -0.15), (2, 0, 0.0),
                (3, 1, 0.15), (4, 0, 0.0),
                (5, 1, -0.15), (6, 0, 0.0),
                (7, 1, 0.15), (8, 0, 0.0),
            ]
            with subprocess.Popen(
                    [str(binary()), "--config", str(overlay), "--physics",
                     "--port", str(gateway_port),
                     "--remote-port", str(peer.getsockname()[1]),
                     "--duration-s", "0.4"],
                    env=_isolated_env(self, tmp),
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    text=True) as proc:
                cmd_seq = 0
                observed = None
                try:
                    for _ in range(100):
                        try:
                            packet = peer.recv(4096)
                        except socket.timeout:
                            if proc.poll() is not None:
                                break
                            continue
                        if struct.unpack_from("<H", packet, 12)[0] != 20:
                            continue
                        config_id, boot_id, epoch, sample_time_us = (
                            struct.unpack_from("<QQQQ", packet, 40))
                        motors, sensors = struct.unpack_from(
                            "<HH", packet, 40 + 70)
                        self.assertEqual((motors, sensors), (8, 1))
                        # Motor index 2 is back_right_drive (id 3, +0.15 duty).
                        m_off = 40 + 84 + 45 * 2
                        _, _, _, drive_vel_rps = struct.unpack_from(
                            "<HBdd", packet, m_off)
                        s_off = 40 + 84 + 45 * motors
                        _, _, _, pigeon_vel_rps = struct.unpack_from(
                            "<HBdd", packet, s_off)
                        if drive_vel_rps > 1.0:
                            observed = (drive_vel_rps, pigeon_vel_rps)
                        cmd_seq += 1
                        payload = bytearray(struct.pack(
                            "<QQQQHHH", config_id, boot_id, epoch,
                            sample_time_us, len(motor_demands), 0, 0))
                        for motor_id, mode, demand in motor_demands:
                            payload += struct.pack(
                                "<HBBdd", motor_id, mode, 0, demand, 0.0)
                        frame = (
                            packet[:12]
                            + struct.pack("<HHQQII", 21, 0, cmd_seq,
                                          sample_time_us, len(payload), 0)
                            + payload
                        )
                        peer.sendto(frame, ("127.0.0.1", gateway_port))
                    stdout, stderr = proc.communicate(timeout=5)
                finally:
                    if proc.poll() is None:
                        proc.kill()
                        proc.communicate()
                self.assertEqual(proc.returncode, 0, stdout + stderr)
                self.assertIsNotNone(
                    observed, "drive motor never accelerated under commands")
                drive_vel_rps, pigeon_vel_rps = observed
                expected_ratio = (y_m / (x_m * x_m + y_m * y_m)) * wheel_radius_m
                self.assertAlmostEqual(
                    pigeon_vel_rps / drive_vel_rps, expected_ratio, places=5)

    # REQ-DEV-05, REQ-DEV-10
    def test_physics_reads_battery_chassis_and_mechanisms_from_config(self):
        """[simulation.battery], [simulation.chassis], and
        [[simulation.mechanisms]] in an overlay config drive sim_gateway
        --physics end-to-end: State.health.battery_v reflects resting and
        under-load sag, flywheel inertia limits spin-up well below legacy 50 ms
        lag, and chassis mass/traction limits yaw rate with physical wheel
        slip."""
        wheel_radius_m = 0.0508
        y_m = 0.30
        nominal_v = 12.6
        r_int_ohm = 0.03
        base_current_a = 5.0
        expected_resting_v = nominal_v - r_int_ohm * base_current_a
        with tempfile.TemporaryDirectory() as tmp, socket.socket(
                socket.AF_INET, socket.SOCK_DGRAM) as peer:
            base = Path(tmp) / "robot.toml"
            base.write_text(
                '[robot]\nname = "sim_physics_models_test"\nperiod_us = 5000\n'
                'command_timeout_us = 100000\n'
                'controller_address = "127.0.0.1"\n'
                '[subsystems.sim.motors]\n'
                'flywheel = { can_id = 1, stator_limit_a = 40.0, '
                'supply_limit_a = 40.0 }\n'
                'left_drive = { can_id = 2, sensor_to_mechanism_ratio = 6.0, '
                'stator_limit_a = 80.0, supply_limit_a = 60.0 }\n'
                'left_steer = { can_id = 3 }\n'
                'right_drive = { can_id = 4, sensor_to_mechanism_ratio = 6.0, '
                'stator_limit_a = 80.0, supply_limit_a = 60.0 }\n'
                'right_steer = { can_id = 5 }\n'
                '[subsystems.sim.sensors.imu]\n'
                'type = "Pigeon2"\ncan_id = 6\n',
                encoding="utf-8")
            overlay = Path(tmp) / "robot_sim.toml"
            overlay.write_text(
                'include = "robot.toml"\n'
                '[simulation]\n'
                f'wheel_radius_m = {wheel_radius_m}\n'
                '[simulation.battery]\n'
                f'nominal_voltage_v = {nominal_v}\n'
                f'internal_resistance_ohm = {r_int_ohm}\n'
                f'base_current_a = {base_current_a}\n'
                'brownout_voltage_v = 6.75\n'
                '[simulation.chassis]\n'
                'mass_kg = 60.0\n'
                'moment_of_inertia_kg_m2 = 6.0\n'
                'traction_coefficient = 0.3\n'
                '[[simulation.swerve_modules]]\n'
                'drive = "left_drive"\nsteer = "left_steer"\n'
                f'x_m = 0.0\ny_m = {y_m}\n'
                '[[simulation.swerve_modules]]\n'
                'drive = "right_drive"\nsteer = "right_steer"\n'
                f'x_m = 0.0\ny_m = {-y_m}\n'
                '[[simulation.mechanisms]]\n'
                'motor = "flywheel"\n'
                'inertia_kg_m2 = 0.05\n'
                'load = "flywheel"\n'
                'viscous_damping_nm_per_radps = 0.001\n',
                encoding="utf-8")
            peer.bind(("127.0.0.1", 0))
            peer.settimeout(0.1)
            gateway_port, _ = udp_ports()
            with subprocess.Popen(
                    [str(binary()), "--config", str(overlay), "--physics",
                     "--port", str(gateway_port),
                     "--remote-port", str(peer.getsockname()[1]),
                     "--duration-s", "0.5"],
                    env=_isolated_env(self, tmp),
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    text=True) as proc:
                cmd_seq = 0
                resting_v = None
                min_battery_v = None
                first_cmd_sample_us = None
                final_sample = None
                try:
                    for _ in range(150):
                        try:
                            packet = peer.recv(4096)
                        except socket.timeout:
                            if proc.poll() is not None:
                                break
                            continue
                        if struct.unpack_from("<H", packet, 12)[0] != 20:
                            continue
                        (config_id, boot_id, epoch, sample_time_us,
                         last_cmd_seq) = struct.unpack_from(
                             "<QQQQQ", packet, 40)
                        motors, sensors = struct.unpack_from(
                            "<HH", packet, 40 + 70)
                        self.assertEqual((motors, sensors), (5, 1))
                        # Motor order is alphabetical: flywheel(0),
                        # left_drive(1), left_steer(2), right_drive(3),
                        # right_steer(4).
                        motor_ids = []
                        motor_vels = []
                        for i in range(motors):
                            m_off = 40 + 84 + 45 * i
                            m_id, _, _, vel_rps = struct.unpack_from(
                                "<HBdd", packet, m_off)
                            motor_ids.append(m_id)
                            motor_vels.append(vel_rps)
                        s_off = 40 + 84 + 45 * motors
                        _, _, _, pigeon_vel_rps = struct.unpack_from(
                            "<HBdd", packet, s_off)
                        health_off = s_off + 27 * sensors
                        health_valid, battery_v, _ = struct.unpack_from(
                            "<?d?", packet, health_off)
                        self.assertTrue(health_valid)
                        if last_cmd_seq == 0 and resting_v is None:
                            resting_v = battery_v
                        if last_cmd_seq > 0:
                            if first_cmd_sample_us is None:
                                first_cmd_sample_us = sample_time_us
                            if min_battery_v is None or battery_v < min_battery_v:
                                min_battery_v = battery_v
                            driven_s = (sample_time_us - first_cmd_sample_us) * 1e-6
                            if driven_s >= 0.15:
                                final_sample = (
                                    driven_s, motor_vels[0], motor_vels[3],
                                    pigeon_vel_rps)
                                break
                        # Drive flywheel at +1.0 duty and rotate chassis CCW
                        # (left_drive = -1.0, right_drive = +1.0, steer = 0).
                        demands = [
                            (motor_ids[0], 1, 1.0),
                            (motor_ids[1], 1, -1.0),
                            (motor_ids[2], 0, 0.0),
                            (motor_ids[3], 1, 1.0),
                            (motor_ids[4], 0, 0.0),
                        ]
                        cmd_seq += 1
                        payload = bytearray(struct.pack(
                            "<QQQQHHH", config_id, boot_id, epoch,
                            sample_time_us, len(demands), 0, 0))
                        for motor_id, mode, demand in demands:
                            payload += struct.pack(
                                "<HBBdd", motor_id, mode, 0, demand, 0.0)
                        frame = (
                            packet[:12]
                            + struct.pack("<HHQQII", 21, 0, cmd_seq,
                                          sample_time_us, len(payload), 0)
                            + payload
                        )
                        peer.sendto(frame, ("127.0.0.1", gateway_port))
                    stdout, stderr = proc.communicate(timeout=5)
                finally:
                    if proc.poll() is None:
                        proc.kill()
                        proc.communicate()
                self.assertEqual(proc.returncode, 0, stdout + stderr)
                self.assertIsNotNone(resting_v)
                self.assertAlmostEqual(resting_v, expected_resting_v, places=5)
                self.assertIsNotNone(min_battery_v)
                self.assertLess(
                    min_battery_v, expected_resting_v - 0.3,
                    f"expected battery sag under load below {expected_resting_v - 0.3:.2f} V, got {min_battery_v:.3f} V")
                self.assertIsNotNone(
                    final_sample, "gateway did not reach 0.15 s of driven ticks")
                driven_s, flywheel_vel_rps, drive_vel_rps, pigeon_vel_rps = final_sample
                # Flywheel spin-up is current-limited by J = 0.05 kg*m^2 and
                # 40 A stator limit (~2.47 rps/s), orders of magnitude slower
                # than legacy 50 ms first-order lag (> 40 rps/s).
                self.assertGreater(flywheel_vel_rps, 0.15)
                self.assertLess(flywheel_vel_rps / driven_s, 4.0)
                # Chassis mass/MOI/traction limits yaw rate and causes physical
                # wheel slip (pigeon yaw rate well below rigid kinematic ratio
                # drive_vel_rps * wheel_radius_m / y_m).
                self.assertGreater(drive_vel_rps, 5.0)
                self.assertGreater(pigeon_vel_rps, 0.05)
                kinematic_yaw_rps = drive_vel_rps * (wheel_radius_m / y_m)
                self.assertLess(
                    pigeon_vel_rps, 0.5 * kinematic_yaw_rps,
                    f"expected traction-limited wheel slip: pigeon={pigeon_vel_rps:.3f} rps vs kinematic={kinematic_yaw_rps:.3f} rps")
                # Theoretical traction-limited angular acceleration ceiling is
                # (2 * mu * m * g / 2 * y_m / MOI) / (2*pi) ~= 1.405 rps/s.
                self.assertLess(pigeon_vel_rps / driven_s, 1.8)

    # REQ-DEV-05, REQ-DEV-31
    def test_pause_realigns_ticks_on_node_clock_without_catchup_burst(self):
        """After a host pause (SIGSTOP/SIGCONT), sim_gateway drops missed
        deadlines and aligns the next tick to end + period_us instead of firing
        a zero-sleep UDP burst; State and Driver Station sample timestamps use
        the node clock (Poller::now())."""
        period_us = 10000
        node_clock = (
            time.CLOCK_MONOTONIC_RAW
            if sys.platform == "darwin"
            else time.CLOCK_MONOTONIC
        )
        with tempfile.TemporaryDirectory() as tmp, socket.socket(
                socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            config = Path(tmp) / "robot.toml"
            config.write_text(
                f'[robot]\nname = "pause_test"\nperiod_us = {period_us}\n'
                'controller_address = "127.0.0.1"\n'
                '[subsystems.test.motors.motor]\ncan_id = 1\n',
                encoding="utf-8")
            receiver.bind(("127.0.0.1", 0))
            receiver.settimeout(0.1)
            gateway_port, _ = udp_ports()
            before_us = time.clock_gettime_ns(node_clock) // 1000
            with subprocess.Popen(
                    [str(binary()), "--config", str(config),
                     "--port", str(gateway_port),
                     "--remote-port", str(receiver.getsockname()[1]),
                     "--duration-s", "0.5"],
                    env=_isolated_env(self, tmp),
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    text=True) as proc:
                state_samples = []
                ds_samples = []
                paused = False
                try:
                    for _ in range(200):
                        try:
                            packet = receiver.recv(4096)
                        except socket.timeout:
                            if proc.poll() is not None:
                                break
                            continue
                        frame_type = struct.unpack_from("<H", packet, 12)[0]
                        if frame_type == 20:
                            sample_us = struct.unpack_from("<Q", packet, 40 + 24)[0]
                            state_samples.append(sample_us)
                            if not paused and len(state_samples) >= 3:
                                paused = True
                                os.kill(proc.pid, signal.SIGSTOP)
                                # Drain any in-flight datagram, then wait out
                                # 150 ms of UDP silence while SIGSTOPped.
                                receiver.settimeout(0.15)
                                while True:
                                    try:
                                        in_flight = receiver.recv(4096)
                                    except socket.timeout:
                                        break
                                    ft = struct.unpack_from("<H", in_flight, 12)[0]
                                    if ft == 20:
                                        state_samples.append(
                                            struct.unpack_from("<Q", in_flight, 40 + 24)[0]
                                        )
                                    elif ft == 24:
                                        ds_samples.append(
                                            struct.unpack_from("<Q", in_flight, 40)[0]
                                        )
                                receiver.settimeout(0.1)
                                os.kill(proc.pid, signal.SIGCONT)
                        elif frame_type == 24:
                            sample_us = struct.unpack_from("<Q", packet, 40)[0]
                            ds_samples.append(sample_us)
                    stdout, stderr = proc.communicate(timeout=5)
                finally:
                    if proc.poll() is None:
                        try:
                            os.kill(proc.pid, signal.SIGCONT)
                        except ProcessLookupError:
                            pass
                        proc.kill()
                        proc.communicate()
            after_us = time.clock_gettime_ns(node_clock) // 1000
            self.assertEqual(proc.returncode, 0, stdout + stderr)
            self.assertTrue(paused, "gateway did not emit initial states")
            self.assertGreater(len(state_samples), 10)
            self.assertGreater(len(ds_samples), 2)
            # Both State and Driver Station timestamps come from Poller::now().
            for sample_us in state_samples + ds_samples:
                self.assertGreaterEqual(sample_us, before_us)
                self.assertLessEqual(sample_us, after_us)
            gaps = [b - a for a, b in zip(state_samples, state_samples[1:])]
            pause_indices = [i for i, g in enumerate(gaps) if g >= 100_000]
            self.assertTrue(
                pause_indices,
                f"expected >= 100 ms gap from SIGSTOP, got max {max(gaps)} us",
            )
            post_pause_gaps = gaps[pause_indices[0] + 1:]
            self.assertGreaterEqual(len(post_pause_gaps), 5)
            # Without realignment, ~15 missed ticks fire back-to-back (< 1 ms
            # apart) right after SIGCONT. With NextTickDeadline's end + period
            # realignment, the first post-pause tick waits a full period, and a
            # later single-tick host stall (< 1 period late) may legitimately
            # recover phase on the very next tick (e.g. 15.8 ms + 4.2 ms =
            # 20.0 ms). Assert both the realigned first gap and that no
            # multi-tick catch-up burst occurs (5-gap span and every 2-gap
            # pair stay on period).
            self.assertGreaterEqual(
                post_pause_gaps[0],
                period_us // 2,
                f"immediate catch-up after host pause: post-pause gaps = {post_pause_gaps[:15]}",
            )
            self.assertGreaterEqual(
                sum(post_pause_gaps[:5]),
                4 * period_us,
                f"catch-up burst across first 5 post-pause ticks: {post_pause_gaps[:15]}",
            )
            # Any single late tick delayed by d < period recovers on the next
            # tick (gap = period - d), so the following pair (period - d) +
            # period can be just above period (e.g. 4.8 ms + 10.0 ms = 14.8 ms)
            # without any catch-up burst. For d >= period, NextTickDeadline
            # realigns to now + period (the SIGSTOP pause case guarded by the
            # first assertion above), so two consecutive post-pause ticks still
            # span at least one full period (vs < 1 ms in a catch-up burst).
            two_tick_spans = [
                a + b for a, b in zip(post_pause_gaps, post_pause_gaps[1:])
            ]
            self.assertGreaterEqual(
                min(two_tick_spans),
                period_us,
                f"catch-up burst in post-pause pair: gaps = {post_pause_gaps[:15]}",
            )

    # REQ-DEV-10, REQ-CONTRACT-30
    def test_physics_sim_state_publishes_start_pose_and_modelled_errors(self):
        """`sim_gateway --physics` publishes `geometry.OdometrySample` truth on
        `/sim/state` starting at `--start-pose`, applies `--wheel-radius-error`,
        `--wheel-slip-gain`, and `--gyro-drift-radps` deterministically relative
        to UDP `State`, and stamps `GatewayProgram::kSimGateway` on every State.
        """
        wheel_radius_m = 0.0508
        start_x_m = 3.25
        start_y_m = -1.5
        start_yaw_rad = 0.5
        wheel_radius_error_frac = 0.2
        wheel_slip_gain = 0.5
        gyro_drift_radps = 0.25
        with tempfile.TemporaryDirectory() as tmp, socket.socket(
                socket.AF_INET, socket.SOCK_DGRAM) as peer:
            config = Path(tmp) / "robot.toml"
            config.write_text(
                '[robot]\nname = "sim_truth_errors_test"\nperiod_us = 5000\n'
                'command_timeout_us = 100000\n'
                'controller_address = "127.0.0.1"\n'
                '[subsystems.drive.motors]\n'
                'back_left_drive = { can_id = 1 }\n'
                'back_left_steer = { can_id = 2 }\n'
                'back_right_drive = { can_id = 3 }\n'
                'back_right_steer = { can_id = 4 }\n'
                'front_left_drive = { can_id = 5 }\n'
                'front_left_steer = { can_id = 6 }\n'
                'front_right_drive = { can_id = 7 }\n'
                'front_right_steer = { can_id = 8 }\n'
                '[subsystems.drive.sensors.imu]\n'
                'type = "Pigeon2"\ncan_id = 9\n'
                '[simulation]\n'
                f'wheel_radius_m = {wheel_radius_m}\n'
                '[[simulation.swerve_modules]]\n'
                'drive = "front_left_drive"\nsteer = "front_left_steer"\n'
                'x_m = 0.30\ny_m = 0.30\n'
                '[[simulation.swerve_modules]]\n'
                'drive = "front_right_drive"\nsteer = "front_right_steer"\n'
                'x_m = 0.30\ny_m = -0.30\n'
                '[[simulation.swerve_modules]]\n'
                'drive = "back_left_drive"\nsteer = "back_left_steer"\n'
                'x_m = -0.30\ny_m = 0.30\n'
                '[[simulation.swerve_modules]]\n'
                'drive = "back_right_drive"\nsteer = "back_right_steer"\n'
                'x_m = -0.30\ny_m = -0.30\n',
                encoding="utf-8")
            peer.bind(("127.0.0.1", 0))
            peer.settimeout(0.1)
            gateway_port, _ = udp_ports()
            # Sorted motor ids: 1, 3, 5, 7 are drives; 2, 4, 6, 8 are steers.
            # Equal forward duty on all four drives with zero steer drives
            # straight along start_yaw_rad with zero true angular velocity.
            motor_demands = [
                (1, 1, 0.25), (2, 0, 0.0),
                (3, 1, 0.25), (4, 0, 0.0),
                (5, 1, 0.25), (6, 0, 0.0),
                (7, 1, 0.25), (8, 0, 0.0),
            ]
            ring = None
            state_builds = set()
            states_by_seq = {}
            try:
                with subprocess.Popen(
                        [str(binary()), "--config", str(config), "--physics",
                         "--start-pose",
                         f"{start_x_m},{start_y_m},{start_yaw_rad}",
                         "--wheel-radius-error", str(wheel_radius_error_frac),
                         "--wheel-slip-gain", str(wheel_slip_gain),
                         "--gyro-drift-radps", str(gyro_drift_radps),
                         "--port", str(gateway_port),
                         "--remote-port", str(peer.getsockname()[1]),
                         "--duration-s", "0.5"],
                        env=_isolated_env(self, tmp),
                        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                        text=True) as proc:
                    cmd_seq = 0
                    try:
                        for _ in range(400):
                            try:
                                packet = peer.recv(4096)
                            except socket.timeout:
                                if proc.poll() is not None:
                                    break
                                continue
                            if struct.unpack_from("<H", packet, 12)[0] != 20:
                                continue
                            if ring is None:
                                ring = _try_open_rtms_readonly(
                                    SIM_STATE_TOPIC, tmp)
                            state_builds.add(_decode_state_build(packet))
                            (config_id, boot_id, epoch, sample_time_us,
                             last_cmd_seq, _, state_seq) = struct.unpack_from(
                                 "<QQQQQQQ", packet, 40)
                            motors, sensors = struct.unpack_from(
                                "<HH", packet, 40 + 70)
                            self.assertEqual((motors, sensors), (8, 1))
                            _, _, drive_pos_rot, drive_vel_rps = (
                                struct.unpack_from("<HBdd", packet, 40 + 84))
                            s_off = 40 + 84 + 45 * motors
                            _, s_valid, pigeon_pos_rot, pigeon_vel_rps = (
                                struct.unpack_from("<HBdd", packet, s_off))
                            self.assertEqual(s_valid, 1)
                            states_by_seq[state_seq] = (
                                sample_time_us, last_cmd_seq, drive_pos_rot,
                                drive_vel_rps, pigeon_pos_rot, pigeon_vel_rps)
                            cmd_seq += 1
                            payload = bytearray(struct.pack(
                                "<QQQQHHH", config_id, boot_id, epoch,
                                sample_time_us, len(motor_demands), 0, 0))
                            for motor_id, mode, demand in motor_demands:
                                payload += struct.pack(
                                    "<HBBdd", motor_id, mode, 0, demand, 0.0)
                            frame = (
                                packet[:12]
                                + struct.pack("<HHQQII", 21, 0, cmd_seq,
                                              sample_time_us, len(payload), 0)
                                + payload
                            )
                            peer.sendto(frame, ("127.0.0.1", gateway_port))
                        stdout, stderr = proc.communicate(timeout=5)
                    finally:
                        if proc.poll() is None:
                            proc.kill()
                            proc.communicate()
                self.assertEqual(proc.returncode, 0, stdout + stderr)
                self.assertEqual(state_builds,
                                 {(0, GATEWAY_PROGRAM_SIM_GATEWAY, 0)})
                self.assertIsNotNone(
                    ring, "/sim/state RTMS segment was never published")
                self.assertEqual(ring.magic, RTMS_MAGIC)
                self.assertEqual(ring.version, RTMS_VERSION)
                self.assertEqual(ring.message_bytes, ODOMETRY_SAMPLE_BYTES)
                self.assertEqual(ring.schema_name, "geometry.OdometrySample")
                truth_count = ring.writer_sequence()
                self.assertGreaterEqual(truth_count, 20)
                self.assertLess(truth_count, ring.slots)
                truth = [ring.read_odometry(i) for i in range(truth_count)]
            finally:
                if ring is not None:
                    ring.close()

            # Initial /sim/state sample (tick 1, before any command arrived)
            # carries the exact --start-pose with zero velocity.
            t0_ns, x0, y0, yaw0, vx0, vy0, omega0 = truth[0]
            self.assertGreater(t0_ns, 0)
            self.assertAlmostEqual(x0, start_x_m, places=9)
            self.assertAlmostEqual(y0, start_y_m, places=9)
            self.assertAlmostEqual(yaw0, start_yaw_rad, places=9)
            self.assertAlmostEqual(vx0, 0.0, places=9)
            self.assertAlmostEqual(vy0, 0.0, places=9)
            self.assertAlmostEqual(omega0, 0.0, places=9)
            for earlier, later in zip(truth, truth[1:]):
                self.assertGreater(later[0], earlier[0])

            # Gateway::Tick increments state.sequence from 0 to 1 on tick 1
            # and truth_publisher writes slot 0 on that same tick, so State
            # sequence k pairs 1:1 with truth[k - 1].
            driven_seqs = sorted(
                seq for seq, row in states_by_seq.items()
                if row[3] > 1.0 and seq <= truth_count)
            self.assertTrue(
                driven_seqs, "drive motors never accelerated under commands")
            first_seq = driven_seqs[0]
            last_seq = driven_seqs[-1]
            driven_s = (
                states_by_seq[last_seq][0] - states_by_seq[first_seq][0]
            ) * 1e-6
            self.assertGreaterEqual(
                driven_s, 0.25,
                f"expected >= 0.25 s of driven ticks, got {driven_s:.3f} s")

            # During initial acceleration, --wheel-slip-gain suppresses true
            # ground speed well below surface speed.
            _, _, _, first_drive_vel, _, _ = states_by_seq[first_seq]
            _, _, _, _, first_vx, first_vy, _ = truth[first_seq - 1]
            first_surface_speed = (
                first_drive_vel * (2.0 * math.pi) * wheel_radius_m
                * (1.0 + wheel_radius_error_frac))
            self.assertLess(
                math.hypot(first_vx, first_vy), 0.5 * first_surface_speed)

            # At steady state, --wheel-radius-error scales true ground speed
            # above wheel odometry, cumulative slip leaves true travel strictly
            # below no-slip scaled odometry, motion follows start_yaw_rad
            # (catching swapped x/y or ignored start yaw), and --gyro-drift-radps
            # diverges UDP Pigeon yaw/rate while true yaw stays fixed.
            (_, _, drive_pos_end, drive_vel_end,
             pigeon_pos_end, pigeon_vel_end) = states_by_seq[last_seq]
            _, x_end, y_end, yaw_end, vx_end, vy_end, omega_end = (
                truth[last_seq - 1])
            dx = x_end - start_x_m
            dy = y_end - start_y_m
            self.assertGreater(dx, 0.5)
            self.assertGreater(dy, 0.25)
            self.assertAlmostEqual(math.atan2(dy, dx), start_yaw_rad, places=5)
            self.assertAlmostEqual(yaw_end, start_yaw_rad, places=6)
            self.assertAlmostEqual(omega_end, 0.0, places=6)

            self.assertGreater(drive_vel_end, 20.0)
            odom_speed = drive_vel_end * (2.0 * math.pi) * wheel_radius_m
            truth_speed = math.hypot(vx_end, vy_end)
            self.assertAlmostEqual(
                truth_speed / odom_speed,
                1.0 + wheel_radius_error_frac,
                delta=0.02)

            odom_dist = drive_pos_end * (2.0 * math.pi) * wheel_radius_m
            no_slip_dist = odom_dist * (1.0 + wheel_radius_error_frac)
            truth_dist = math.hypot(dx, dy)
            self.assertLess(
                truth_dist, 0.95 * no_slip_dist,
                f"expected wheel-slip deficit: truth={truth_dist:.4f} m vs "
                f"no_slip={no_slip_dist:.4f} m")

            self.assertAlmostEqual(
                pigeon_vel_end, gyro_drift_radps / (2.0 * math.pi), places=5)
            self.assertGreater(pigeon_pos_end * (2.0 * math.pi), 0.05)

    # REQ-DEV-05, REQ-CONTRACT-30
    def test_physics_fault_schedule_injects_and_recovers_faults_in_state(self):
        """`sim_gateway --physics --fault-schedule` injects scheduled
        motor_disconnect and supply_voltage faults into emitted UDP `State`
        datagrams and clears them when their intervals end."""
        fault_voltage_v = 6.2
        with tempfile.TemporaryDirectory() as tmp, socket.socket(
                socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            config = Path(tmp) / "robot.toml"
            config.write_text(
                '[robot]\nname = "fault_schedule_test"\nperiod_us = 5000\n'
                'controller_address = "127.0.0.1"\n'
                '[subsystems.test.motors.lift]\ncan_id = 1\n'
                '[subsystems.test.sensors.imu]\ntype = "Pigeon2"\ncan_id = 2\n',
                encoding="utf-8")
            receiver.bind(("127.0.0.1", 0))
            receiver.settimeout(0.1)
            gateway_port, _ = udp_ports()
            with subprocess.Popen(
                    [str(binary()), "--config", str(config), "--physics",
                     "--port", str(gateway_port),
                     "--remote-port", str(receiver.getsockname()[1]),
                     "--fault-schedule",
                     f"0.08:0.20:motor_disconnect:lift|"
                     f"0.12:0.26:supply_voltage:all:{fault_voltage_v}",
                     "--duration-s", "0.45"],
                    env=_isolated_env(self, tmp),
                    stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                    text=True) as proc:
                state_builds = set()
                motor_valid_transitions = []
                supply_drop_transitions = []
                normal_voltages = []
                try:
                    for _ in range(400):
                        try:
                            packet = receiver.recv(4096)
                        except socket.timeout:
                            if proc.poll() is not None:
                                break
                            continue
                        if struct.unpack_from("<H", packet, 12)[0] != 20:
                            continue
                        state_builds.add(_decode_state_build(packet))
                        motors, sensors = struct.unpack_from(
                            "<HH", packet, 40 + 70)
                        self.assertEqual((motors, sensors), (1, 1))
                        _, m_valid_byte, m_pos, m_vel, m_volts = (
                            struct.unpack_from("<HBddd", packet, 40 + 84))
                        m_valid = m_valid_byte == 1
                        if not m_valid:
                            self.assertEqual((m_pos, m_vel, m_volts),
                                             (0.0, 0.0, 0.0))
                        health_off = 40 + 84 + 45 * motors + 27 * sensors
                        health_valid, battery_v = struct.unpack_from(
                            "<?d", packet, health_off)
                        self.assertTrue(health_valid)
                        if (not motor_valid_transitions
                                or motor_valid_transitions[-1] != m_valid):
                            motor_valid_transitions.append(m_valid)
                        dropped = abs(battery_v - fault_voltage_v) < 1e-6
                        if not dropped:
                            normal_voltages.append(battery_v)
                        if (not supply_drop_transitions
                                or supply_drop_transitions[-1] != dropped):
                            supply_drop_transitions.append(dropped)
                    stdout, stderr = proc.communicate(timeout=5)
                finally:
                    if proc.poll() is None:
                        proc.kill()
                        proc.communicate()
                self.assertEqual(proc.returncode, 0, stdout + stderr)
                self.assertEqual(state_builds,
                                 {(0, GATEWAY_PROGRAM_SIM_GATEWAY, 0)})
                self.assertEqual(motor_valid_transitions, [True, False, True])
                self.assertEqual(supply_drop_transitions, [False, True, False])
                self.assertTrue(normal_voltages)
                for volts in normal_voltages:
                    self.assertGreater(volts, 12.0)


if __name__ == "__main__":
    unittest.main()
