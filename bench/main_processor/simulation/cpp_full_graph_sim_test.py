"""e2e: pure-C++ bench full-graph simulation under plain, ASan and TSan.

Runs the launcher on testdata/robot_cpp_sim.toml with every member a C++ binary:
hardware_node, sim_gateway --physics, studio_bridge, swerve_module, motor,
teleop and telemetry (no Python nodes, so every launched process runs
instrumented inside tools/sanitizer_lane.sh asan|tsan).

Schedules count from a node-clock time zero written via --schedule-start-file
only after wait_ready() observes the bridge's first accepted State, every
member's loop running, and studio_bridge bound and settled, so sanitizer
start-up overhead never races the scripted phases.
"""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest
import urllib.request

from python.runfiles import runfiles

import support
from talOS.build import sim_check

READY_TIMEOUT_S = 45
STOP_WAIT_S = 15
EXIT_WAIT_S = 5
LOG_DUMP_TIMEOUT_S = 15
SESSION_DURATION_S = 3.2
BACKSTOP_S = READY_TIMEOUT_S + int(SESSION_DURATION_S) + 30
NODE_CLOCK = (time.CLOCK_MONOTONIC_RAW if sys.platform == "darwin"
              else time.CLOCK_MONOTONIC)

# Teleop rescales axis -0.6 through deadband_frac = 0.1 -> (0.6 - 0.1) / 0.9.
TELEOP_DRIVE_RPS = (0.6 - 0.1) / (1.0 - 0.1) * 10.0  # ~5.56 rps
TELEOP_MOTOR_V = (0.6 - 0.1) / (1.0 - 0.1) * 2.0  # ~1.11 V

EXPECTED_MEMBERS = frozenset({
    "hardware_node",
    "sim_gateway",
    "studio_bridge",
    "motor",
    "swerve_module",
    "teleop",
    "telemetry",
})


def rlocation(path):
    found = runfiles.Create().Rlocation(path, source_repo="")
    if not found or not os.path.exists(found):
        raise FileNotFoundError(f"missing runfile: {path}")
    return found


def start_schedule(path):
    pending = path + ".pending"
    with open(pending, "w", encoding="utf-8") as f:
        f.write(str(time.clock_gettime_ns(NODE_CLOCK)))
    os.rename(pending, path)


def dump_jsonl(log_dump, session, topic, node=None):
    args = [log_dump, "--format", "jsonl"]
    if node:
        args += ["--node", node]
    args += ["--topic", topic, session]
    proc = subprocess.run(
        args, capture_output=True, text=True, timeout=LOG_DUMP_TIMEOUT_S,
        check=True)
    return [json.loads(line) for line in proc.stdout.splitlines()
            if line.startswith("{")]


def parse_decoded_fields(row):
    fields = {}
    for item in row["decoded"].strip("{}").split(", "):
        key, value = item.split(": ", 1)
        fields[key] = value
    return fields


class CppFullGraphSimTest(unittest.TestCase):
    # REQ-OPS-30, REQ-SAFE-31: full-graph C++ composition evidence under plain,
    # ASan and TSan (disabled -> teleop actuation -> disabled, with supervised
    # Studio bridge).
    def test_cpp_full_graph_drives_and_neutralizes_cleanly(self):
        launcher = rlocation(os.environ["LAUNCHER"])
        log_dump = rlocation(os.environ["LOG_DUMP"])
        registry_unlink = rlocation(os.environ["REGISTRY_UNLINK"])
        session_report_path = rlocation("_main/talOS/launcher/session_report.py")
        config_path = Path(rlocation(os.environ["CONFIG"]))
        root = Path(launcher).parents[2]
        report = sim_check.load_session_report(session_report_path)

        layout = support.device_layout(config_path)
        drive_ids = [i for i, sub, name, kind in layout
                     if sub == "swerve_module" and kind == "motors"
                     and name == "drive"]
        motor_ids = [i for i, sub, _, kind in layout
                     if sub == "motor" and kind == "motors"]
        self.assertEqual(len(drive_ids), 1, layout)
        self.assertEqual(len(motor_ids), 1, layout)
        drive_id = drive_ids[0]
        motor_id = motor_ids[0]

        gateway_port, bridge_port = sim_check.udp_ports()
        with tempfile.TemporaryDirectory(
                dir=os.environ.get("TEST_TMPDIR")) as tmp:
            work = os.path.join(tmp, "work")
            os.makedirs(work)
            os.symlink(str(root), os.path.join(work, "bazel-bin"))
            output = os.path.join(tmp, "sessions")
            os.makedirs(output)
            log = os.path.join(tmp, "launch.log")
            start_file = os.path.join(tmp, "schedule_start")

            env = os.environ.copy()
            env.pop("TALOS_SHM_NAMESPACE", None)
            env["TEST_TMPDIR"] = os.environ.get("TEST_TMPDIR") or tmp

            gateway_args = [
                "--physics",
                "--mode-schedule", "disabled:0.3,teleop:2.0,disabled:0.5",
                "--joystick-schedule",
                "0:axes=0,0,0,0,0,0;buttons=0;pov=-1|"
                "0.6:axes=0,-0.6,0,0,0,-0.6;buttons=32;pov=-1|"
                "2.0:axes=0,0,0,0,0,0;buttons=0;pov=-1",
                "--schedule-start-file", start_file,
            ]
            args = [
                launcher, "--config", str(config_path),
                "--sim", "--start-sim-gateway", "--start-studio-bridge",
                "--studio-http-port", "0",
                "--remote", "127.0.0.1",
                "--remote-port", str(gateway_port),
                "--local-port", str(bridge_port),
                "--output-dir", output,
                "--duration-s", str(BACKSTOP_S),
            ]
            for arg in gateway_args:
                args += ["--sim-gateway-arg", arg]

            failed = True
            channels_doc = None
            system_doc = None
            try:
                try:
                    with open(log, "wb") as out, \
                            sim_check.Launch(args, out, cwd=work,
                                             env=env) as launch:
                        unready = sim_check.wait_ready(
                            launch, output, READY_TIMEOUT_S, studio=True)
                        ended = launch.process.poll()
                        if not unready and ended is None:
                            start_schedule(start_file)
                            try:
                                ended = launch.process.wait(
                                    timeout=SESSION_DURATION_S)
                            except subprocess.TimeoutExpired:
                                pass
                        if not unready and ended is None:
                            # Exercise studio_bridge's live HTTP endpoints
                            # (/channels.json and /system.json) while the
                            # instrumented session is running.
                            live_sessions = [
                                os.path.join(output, d)
                                for d in os.listdir(output)
                                if os.path.isdir(os.path.join(output, d))
                            ]
                            if len(live_sessions) == 1:
                                with open(
                                    os.path.join(live_sessions[0],
                                                 "manifest.json"),
                                    encoding="utf-8",
                                ) as mf:
                                    live_manifest = json.load(mf)
                                bridge_entry = next(
                                    (n for n in live_manifest.get("nodes", [])
                                     if n.get("name") == "studio_bridge"),
                                    None,
                                )
                                if bridge_entry and sim_check.valid_http_port(
                                        bridge_entry.get("http_port")):
                                    base_url = (
                                        f"http://127.0.0.1:"
                                        f"{bridge_entry['http_port']}"
                                    )
                                    with urllib.request.urlopen(
                                            f"{base_url}/channels.json",
                                            timeout=5) as resp:
                                        channels_doc = json.loads(
                                            resp.read().decode("utf-8"))
                                    with urllib.request.urlopen(
                                            f"{base_url}/system.json",
                                            timeout=5) as resp:
                                        system_doc = json.loads(
                                            resp.read().decode("utf-8"))
                        in_order, killed = launch.finish(
                            STOP_WAIT_S, EXIT_WAIT_S)
                finally:
                    subprocess.run([registry_unlink], env=env, check=False)

                with open(log, encoding="utf-8", errors="replace") as f:
                    diagnostics = f.read()
                self.assertEqual(unready, "", diagnostics)
                self.assertIsNone(
                    ended, f"launch exited early ({ended}):\n{diagnostics}")
                self.assertTrue(
                    in_order,
                    f"launch did not stop in order:\n{diagnostics}")
                self.assertEqual(killed, [], diagnostics)
                self.assertEqual(launch.process.returncode, 0, diagnostics)

                sessions = [os.path.join(output, d)
                            for d in os.listdir(output)
                            if os.path.isdir(os.path.join(output, d))]
                self.assertEqual(len(sessions), 1, os.listdir(output))
                session = sessions[0]

                problems = sim_check.judge(
                    session, log_dump, report, studio=True)
                self.assertEqual(
                    problems, [],
                    f"session judge failed: {problems}\n{diagnostics}")

                with open(os.path.join(session, "manifest.json"),
                          encoding="utf-8") as f:
                    manifest = json.load(f)
                names = {n["name"] for n in manifest["nodes"]}
                self.assertEqual(names, EXPECTED_MEMBERS, diagnostics)

                # Hardware link health at end of session.
                status_rows = dump_jsonl(log_dump, session, "/hw/status")
                self.assertTrue(status_rows, f"no /hw/status:\n{diagnostics}")
                last_status = parse_decoded_fields(status_rows[-1])
                self.assertEqual(last_status["configured"], "true", last_status)
                for field, value in last_status.items():
                    if field.startswith("alert_"):
                        self.assertEqual(
                            value, "false", f"{field}: {last_status}")
                    if field in ("motor_fault_mask",
                                 "gateway_rejected_command_count",
                                 "gateway_apply_failure_count",
                                 "gateway_device_reset_count"):
                        self.assertEqual(value, "0", f"{field}: {last_status}")

                # Both C++ actuator owners drive under teleop and return to
                # neutral when disabled.
                swerve_reqs = [
                    (r["now_ms"], support.motor_requests(r)[drive_id])
                    for r in dump_jsonl(log_dump, session,
                                        "/hw/request/swerve_module",
                                        "swerve_module")
                    if r["kind"] == "SEND"
                ]
                motor_reqs = [
                    (r["now_ms"], support.motor_requests(r)[motor_id])
                    for r in dump_jsonl(log_dump, session,
                                        "/hw/request/motor", "motor")
                    if r["kind"] == "SEND"
                ]
                swerve_active = [
                    t for t, req in swerve_reqs
                    if req[0] == "velocity"
                    and abs(abs(req[2]) - TELEOP_DRIVE_RPS) < 0.1
                ]
                motor_active = [
                    t for t, req in motor_reqs
                    if req[0] == "voltage"
                    and abs(req[2] - TELEOP_MOTOR_V) < 0.1
                ]
                self.assertGreater(
                    len(swerve_active), 10,
                    f"swerve_module did not drive in teleop "
                    f"({len(swerve_reqs)} total requests)\n{diagnostics}")
                self.assertGreater(
                    len(motor_active), 10,
                    f"motor did not drive in teleop "
                    f"({len(motor_reqs)} total requests)\n{diagnostics}")

                swerve_tail_neutral = [
                    t for t, req in swerve_reqs
                    if t > swerve_active[-1] and req == support.NEUTRAL
                ]
                motor_tail_neutral = [
                    t for t, req in motor_reqs
                    if t > motor_active[-1] and req == support.NEUTRAL
                ]
                self.assertGreater(
                    len(swerve_tail_neutral), 5,
                    "swerve_module did not return to neutral after teleop")
                self.assertGreater(
                    len(motor_tail_neutral), 5,
                    "motor did not return to neutral after teleop")

                # Physics simulator applied the commands and returned nonzero
                # motor motion in /hw/state.
                state_rows = [
                    r for r in dump_jsonl(log_dump, session, "/hw/state",
                                          "hardware_node")
                    if r["kind"] == "SEND"
                ]
                self.assertTrue(state_rows, "no /hw/state records")
                max_vel = {drive_id: 0.0, motor_id: 0.0}
                for row in state_rows:
                    for motor in support.hardware(row)["motors"]:
                        mid = motor["id"]
                        if mid in max_vel:
                            max_vel[mid] = max(
                                max_vel[mid], abs(motor["velocity_rps"]))
                self.assertGreater(
                    max_vel[drive_id], 0.5,
                    f"swerve drive motor never moved in physics sim: {max_vel}")
                self.assertGreater(
                    max_vel[motor_id], 0.5,
                    f"bench motor never moved in physics sim: {max_vel}")

                # Telemetry node received live subsystem states and published
                # its dictionary over /talos/telemetry to studio_bridge.
                telemetry_inputs = [
                    r for r in dump_jsonl(
                        log_dump, session,
                        "/motor/state,/swerve_module/state", "telemetry")
                    if r["kind"] == "MESSAGE"
                ]
                telemetry_topics = {r["topic"] for r in telemetry_inputs}
                self.assertEqual(
                    telemetry_topics, {"/motor/state", "/swerve_module/state"},
                    f"telemetry missed expected subsystem state topics: "
                    f"{telemetry_topics}\n{diagnostics}")
                self.assertIsNotNone(
                    channels_doc,
                    f"failed to fetch /channels.json from studio_bridge\n"
                    f"{diagnostics}")
                channel_names = {
                    ch["name"] for ch in channels_doc.get("channels", [])
                }
                self.assertTrue(
                    any(name.startswith("motor/state/")
                        for name in channel_names),
                    f"/channels.json missing motor/state/*: "
                    f"{sorted(channel_names)[:20]}")
                self.assertTrue(
                    any(name.startswith("swerve_module/state/")
                        for name in channel_names),
                    f"/channels.json missing swerve_module/state/*: "
                    f"{sorted(channel_names)[:20]}")
                self.assertIsNotNone(
                    system_doc,
                    f"failed to fetch /system.json from studio_bridge\n"
                    f"{diagnostics}")
                self.assertTrue(
                    system_doc.get("nodes"),
                    f"/system.json reported no live nodes: {system_doc}")
                failed = False
            finally:
                if failed:
                    kept = sim_check.preserve_failure_artifacts(log, output)
                    if kept:
                        print(f"failure artifacts preserved: {kept}",
                              file=sys.stderr)


if __name__ == "__main__":
    unittest.main()
