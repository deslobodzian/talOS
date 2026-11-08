"""Gateway startup and a real, bounded match with observable safety assertions."""

import json
import os
import re
import shutil
import signal
import subprocess
import socket
import struct
import tempfile
import time
import sys
import tomllib
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
        self.assertEqual(doc["sources"], [])
        proc = subprocess.run(
            [str(binary()), "--describe", "--physics"], capture_output=True,
            text=True, timeout=30)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        sources = json.loads(proc.stdout)["sources"]
        self.assertEqual(len(sources), 1)
        self.assertEqual(sources[0]["name"], "/sim/state")
        self.assertEqual(sources[0]["kind"], "SENDER")
        self.assertEqual(sources[0]["schema"], "geometry.OdometrySample")
        self.assertEqual(sources[0]["message_bytes"], 56)
        self.assertTrue(sources[0]["bfbs"])

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
                self.assertIn("require --physics", proc.stderr)
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
                        if previous_sample is not None:
                            self.assertGreaterEqual(sample - previous_sample,
                                                    20000)
                        previous_sample = sample
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
                self.assertEqual(len(boots), 1)
                generations = [generation for _, generation in phases]
                self.assertEqual(generations, sorted(set(generations)))
                self.assertIn((1, 3, 32, 1, 5, -0.5, 90), joystick_samples)


def packet_bytes(row):
    """Unwrap the fixed IPC Packet; decode only its canonical wire bytes."""
    payload = bytes.fromhex(row["payload_hex"])
    size, = struct.unpack_from("<I", payload)
    if size > len(payload) - 4:
        raise ValueError("truncated hardware Packet")
    return payload[4:4 + size]


def motor_requests(row):
    data = packet_bytes(row)
    count, = struct.unpack_from("<H", data, 32)
    return {motor[0]: motor[1:] for motor in (
        struct.unpack_from("<HBBdd", data, 38 + 20 * i)
        for i in range(count))}


def ds_snapshot(row):
    data = packet_bytes(row)
    flags, = struct.unpack_from("<I", data, 24)
    offset = 38
    for _ in range(2):  # Length-prefixed game message and event name.
        offset += 1 + data[offset]
    if not data[offset]:
        raise ValueError("scripted joystick disconnected")
    axis1, = struct.unpack_from("<f", data, offset + 12)
    return flags, axis1


def ds_buttons(row):
    data = packet_bytes(row)
    offset = 38
    for _ in range(2):
        offset += 1 + data[offset]
    return struct.unpack_from("<I", data, offset + 4)[0]


def log_baseline_ns(session):
    """Find log_dump's exact origin without assuming a Python/C++ clock epoch.

    Walk v6 record headers only (events/log/format.h); payload decoding remains
    log_dump's job. On macOS Node time is MONOTONIC_RAW, not steady_clock.
    """
    baseline = None
    for path in session.glob("*.tlog"):
        with path.open("rb") as stream:
            header = stream.read(120)
            magic, version, header_bytes = struct.unpack_from("<8sII", header)
            if magic != b"TALOSLOG" or version != 6:
                raise ValueError(f"unsupported log format: {path}")
            manifests, = struct.unpack_from("<I", header, 40)
            schema_bytes, = struct.unpack_from("<Q", header, 112)
            stream.seek(header_bytes + 112 * manifests + schema_bytes)
            while record := stream.read(64):
                if len(record) != 64:
                    raise ValueError(f"truncated log header: {path}")
                timestamp, = struct.unpack_from("<q", record, 16)
                baseline = timestamp if baseline is None else min(baseline, timestamp)
                stored, = struct.unpack_from("<I", record, 60)
                stream.seek(stored, 1)
    if baseline is None:
        raise ValueError("no recorded events")
    return baseline


def node_clock_ns():
    clock = time.CLOCK_MONOTONIC_RAW if sys.platform == "darwin" else time.CLOCK_MONOTONIC
    return time.clock_gettime_ns(clock)


def device_layout(config):
    """Mirror the config's documented sorted logical-ID and motor-bit order."""
    robot = tomllib.loads(config.read_text())
    devices = []
    for subsystem in robot["subsystems"]:
        document = tomllib.loads((config.parent / subsystem["path"]).read_text())
        for kind in ("motors", "sensors", "digital_inputs", "digital_outputs",
                     "analog_inputs", "encoders", "pwm_outputs"):
            devices.extend((subsystem["name"], name, kind)
                           for name in document.get(kind, {}))
    return [(i, *device) for i, device in enumerate(sorted(devices), 1)]


class FullStackSimulation(unittest.TestCase):
    def window(self, rows, start, end):
        samples = [row for row in rows if start <= row["now_ms"] < end]
        self.assertGreater(len(samples), 2, f"missing samples in [{start}, {end})")
        # A quiet/missing recorder cannot masquerade as safe outputs.
        times = [start] + [row["now_ms"] for row in samples] + [end]
        self.assertLessEqual(max(b - a for a, b in zip(times, times[1:])), 30,
                             f"output gap in [{start}, {end})")
        return samples

    def assert_driving(self, rows, start, end, drive_ids):
        samples = self.window(rows, start, end)
        self.assertTrue(all(all(
            motor_requests(row)[i][0] == 3 and
            abs(motor_requests(row)[i][2]) > 0.1 for i in drive_ids)
                            for row in samples), "missing nonzero velocity requests")

    def assert_neutral(self, rows, start, end, motor_ids):
        for row in self.window(rows, start, end):
            requests = motor_requests(row)
            for motor in motor_ids:
                self.assertEqual(requests[motor], (0, 0, 0.0, 0.0),
                                 f"motor {motor} active at {row['now_ms']} ms")

    def assert_neutral_by(self, rows, start, deadline, end, motor_ids):
        neutral = [row for row in rows if start <= row["now_ms"] <= deadline and
                   all(motor_requests(row)[i] == (0, 0, 0.0, 0.0)
                       for i in motor_ids)]
        self.assertTrue(neutral, f"no neutral request by {deadline} ms")
        self.assert_neutral(rows, neutral[0]["now_ms"], end, motor_ids)

    def test_scripted_match_neutralizes_stalls_blackouts_and_disconnects(self):
        root = workspace()
        config_dir = root / "2026-robot/main_processor/configuration"
        config = config_dir / "robot_physics_sim.toml"
        robot = tomllib.loads((config_dir / "robot.toml").read_text())
        bound_ms = (robot["robot"]["command_timeout_us"] +
                    2 * robot["robot"]["period_us"]) / 1000
        layout = device_layout(config_dir / "robot.toml")
        motor_ids = [i for i, sub, _, kind in layout
                     if sub == "drivetrain" and kind == "motors"]
        drive_ids = [i for i, sub, name, kind in layout
                     if sub == "drivetrain" and kind == "motors" and name.endswith("_drive")]
        fault_id = next(i for i, _, name, kind in layout
                        if kind == "motors" and name == "back_left_drive")
        roller_id = next(i for i, sub, name, kind in layout
                         if sub == "intake" and name == "roller" and kind == "motors")
        motor_order = [i for i, _, _, kind in layout if kind == "motors"]
        fault_bit = 1 << motor_order.index(fault_id)
        gw_port, bridge_port = udp_ports()
        env = dict(os.environ, TALOS_SHM_NAMESPACE=f"full-stack-{uuid.uuid4().hex}")
        tmp = Path(tempfile.mkdtemp(prefix="talos-scripted-match-"))
        output = tmp / "sessions"
        launcher = None
        stopped_pid = None
        success = False
        try:
            (tmp / "bazel-bin").symlink_to(root, target_is_directory=True)
            with (tmp / "launcher.txt").open("w+") as launcher_log:
                args = [str(root / "talOS/launcher/launcher"),
                        "--config", str(config), "--sim", "--start-sim-gateway"]
                gateway_args = [
                    "--physics", "--port", str(gw_port), "--remote-port", str(bridge_port),
                    "--mode-schedule", "disabled:0.5,auto:0.5,teleop:6,disabled:1",
                    "--joystick-schedule",
                    "0:axes=0,0,0;buttons=0;pov=-1|"
                    "1.4:axes=0,-0.6,0;buttons=4;pov=-1|"
                    "1.8:axes=0,-0.6,0;buttons=5;pov=-1|"
                    "3:axes=0,0,0;buttons=0;pov=-1|"
                    "3.4:axes=0,-0.6,0;buttons=0;pov=-1|"
                    "5:axes=0,0,0;buttons=0;pov=-1|"
                    "5.3:axes=0,-0.6,0;buttons=0;pov=-1",
                    "--fault-schedule",
                    "4.3:4.8:udp_blackout:bridge|6:6.7:motor_disconnect:back_left_drive"]
                for arg in gateway_args:
                    args.extend(["--sim-gateway-arg", arg])
                args.extend(["--remote", "127.0.0.1", "--remote-port", str(gw_port),
                             "--local-port", str(bridge_port), "--output-dir", str(output),
                             "--duration-s", "8"])
                launcher = subprocess.Popen(
                    args, cwd=tmp, env=env, stdout=launcher_log,
                    stderr=subprocess.STDOUT, start_new_session=True)
                deadline = time.monotonic() + 15
                while time.monotonic() < deadline:
                    manifests = list(output.glob("*/manifest.json"))
                    if manifests:
                        manifest = json.loads(manifests[0].read_text())
                        nodes = {node["name"]: node for node in manifest["nodes"]}
                        if nodes.get("sim_gateway", {}).get("pid", 0) > 0:
                            break
                    self.assertIsNone(launcher.poll(), "launcher exited before gateway")
                    time.sleep(0.01)
                else:
                    self.fail("gateway did not start")
                # Anchor to the gateway manifest, not slow --describe probes.
                # Recorded preconditions below reject a missed injection window.
                time.sleep(2.0)
                manifest = json.loads(manifests[0].read_text())
                driver = next(n for n in manifest["nodes"] if n["name"] == "driver")
                self.assertGreater(driver["pid"], 0)
                stopped_pid = driver["pid"]
                os.kill(stopped_pid, signal.SIGSTOP)
                stop_ns = node_clock_ns()
                time.sleep(0.25)  # >100 ms freshness; <500 ms stall supervision.
                resume_ns = node_clock_ns()
                os.kill(stopped_pid, signal.SIGCONT)
                stopped_pid = None
                launcher.wait(timeout=20)
                launcher_log.seek(0)
                diagnostics = launcher_log.read()
            self.assertEqual(launcher.returncode, 0, diagnostics)
            manifests = list(output.glob("*/manifest.json"))
            self.assertEqual(len(manifests), 1, diagnostics)
            session = manifests[0].parent
            manifest = json.loads(manifests[0].read_text())
            nodes = {node["name"]: node for node in manifest["nodes"]}
            self.assertEqual(set(nodes), {
                "hardware_node", "autonomous", "driver", "drivetrain", "intake",
                "pose_estimator", "shooter", "telemetry", "teleop", "sim_gateway",
                "front_left_camera", "front_right_camera",
                "rear_left_camera", "rear_right_camera"}, diagnostics)
            for name, node in nodes.items():
                self.assertEqual(node["exit_code"], 0, f"{name}: {diagnostics}")
                self.assertFalse(node["recorder_failed"], f"{name}: {diagnostics}")
                self.assertEqual(node["restart_count"], 0, f"{name}: {diagnostics}")
                if name != "sim_gateway":
                    self.assertGreater(node["record_count"], 0, name)
            dumped = subprocess.run(
                [str(root / "talOS/launcher/log_dump"), "--format", "jsonl",
                 "--node", "drivetrain,hardware_node,intake,shooter",
                 "--topic", "/hw/request/drivetrain,/hw/request/intake,/shooter/state,"
                 "/hw/state/driver_station,/hw/state,/hw/status",
                 str(session)], capture_output=True, text=True, timeout=10, check=True)
            (tmp / "match.jsonl").write_text(dumped.stdout)
            rows = [json.loads(line) for line in dumped.stdout.splitlines()]
            baseline = log_baseline_ns(session)
            stop_ms = (stop_ns - baseline) / 1e6
            resume_ms = (resume_ns - baseline) / 1e6
            requests = [r for r in rows if r["process"] == "drivetrain" and
                        r["topic"] == "/hw/request/drivetrain" and r["kind"] == "SEND"]
            ds = [r for r in rows if r["process"] == "drivetrain" and
                  r["topic"] == "/hw/state/driver_station" and r["kind"] == "MESSAGE"]
            self.assertGreater(len(ds), 100)
            intake_requests = [
                r for r in rows if r["process"] == "intake" and
                r["topic"] == "/hw/request/intake" and r["kind"] == "SEND"]
            feed_only = next(r["now_ms"] for r in ds if ds_buttons(r) == 4)
            spinup = next(r["now_ms"] for r in ds if ds_buttons(r) == 5)
            # Button 3 alone must not feed a stopped flywheel. Require dense
            # request samples so a missing recorder cannot pass as safe.
            self.assert_neutral(intake_requests, feed_only + 80, spinup, [roller_id])
            ready = [
                r for r in rows if r["process"] == "shooter" and
                r["topic"] == "/shooter/state" and r["kind"] == "SEND" and
                spinup < r["now_ms"] < spinup + 1000 and
                struct.unpack_from("<?", bytes.fromhex(r["payload_hex"]), 24)[0] and
                struct.unpack_from("<d", bytes.fromhex(r["payload_hex"]), 16)[0] > 0]
            self.assertTrue(ready, "flywheel never reached speed while 1+3 held")
            feeding = self.window(intake_requests, ready[0]["now_ms"] + 80,
                                  ready[0]["now_ms"] + 200)
            self.assertTrue(all(
                motor_requests(r)[roller_id][0] == 3 and
                motor_requests(r)[roller_id][2] == -20.0 for r in feeding),
                "ready shooter plus held buttons 1+3 did not feed")
            centers = []
            previous_axis = None
            for row in ds:
                flags, axis = ds_snapshot(row)
                if flags & 16 and axis == 0 and previous_axis is not None and previous_axis < -0.5:
                    centers.append(row["now_ms"])
                previous_axis = axis
            self.assertEqual(len(centers), 2, "missing scheduled release/rearm snapshots")
            center, blackout_center = centers
            self.assertLess(resume_ms + 200, center, "missed held-stick injection window")
            self.assert_driving(requests, stop_ms - 150, stop_ms, drive_ids)
            self.assert_neutral_by(requests, stop_ms, stop_ms + bound_ms,
                                   resume_ms, motor_ids)
            # Resuming must not replay the still-held forward stick.
            self.assert_neutral(requests, resume_ms, center, motor_ids)
            self.assert_driving(requests, center + 650, center + 1000, drive_ids)
            gaps = [(a, b) for a, b in zip(ds, ds[1:])
                    if b["now_ms"] - a["now_ms"] > 300]
            self.assertEqual(len(gaps), 1, "missing or unexpected DS blackout")
            last_ds, recovered_ds = gaps[0]
            blackout = last_ds["now_ms"]
            self.assert_driving(requests, blackout - 150, blackout, drive_ids)
            # Last receipt + timeout is at least as strict as blackout + timeout.
            self.assert_neutral_by(requests, blackout, blackout + bound_ms,
                                   recovered_ds["now_ms"], motor_ids)
            self.assert_neutral(requests, recovered_ds["now_ms"],
                                blackout_center, motor_ids)
            self.assert_driving(requests, blackout_center + 500,
                                blackout_center + 850, drive_ids)
            invalid_states = []
            for row in rows:
                if row["process"] != "hardware_node" or row["topic"] != "/hw/state":
                    continue
                data = packet_bytes(row)
                count, = struct.unpack_from("<H", data, 44)
                for i in range(count):
                    motor, valid = struct.unpack_from("<HB", data, 58 + 47 * i)
                    if motor == fault_id and not valid:
                        invalid_states.append(row)
            self.assertGreater(len(invalid_states), 20, "motor disconnect not observed")
            disconnected = invalid_states[0]["now_ms"]
            restored = invalid_states[-1]["now_ms"]
            self.assert_driving(requests, disconnected - 150, disconnected, drive_ids)
            self.assert_neutral_by(requests, disconnected, disconnected + bound_ms,
                                   restored, [fault_id])
            statuses = [r for r in rows if r["process"] == "drivetrain" and
                        r["topic"] == "/hw/status" and r["kind"] == "MESSAGE" and
                        disconnected + 50 <= r["now_ms"] < restored]
            self.assertGreater(len(statuses), 10, "missing bridge fault status")
            for row in statuses:
                match = re.search(r"\bmotor_fault_mask:\s*(\d+)", row["decoded"])
                self.assertIsNotNone(match, row["decoded"])
                self.assertNotEqual(int(match[1]) & fault_bit, 0, row["decoded"])
            final_disabled = next(r["now_ms"] for r in ds
                                  if r["now_ms"] > restored and
                                  not ds_snapshot(r)[0] & 4)
            self.assert_neutral(requests, final_disabled + 10,
                                final_disabled + 200, motor_ids)
            success = True
        finally:
            if stopped_pid is not None:
                try:
                    os.kill(stopped_pid, signal.SIGCONT)
                except ProcessLookupError:
                    pass
            if launcher is not None:
                if not success:
                    try:
                        os.killpg(launcher.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                launcher.wait(timeout=5)
            if success:
                shutil.rmtree(tmp)
            else:
                print(f"Scripted match artifacts retained: {tmp}; sessions: {output}",
                      file=sys.stderr, flush=True)


if __name__ == "__main__":
    unittest.main()
