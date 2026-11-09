"""A recorded driver/match rehearsal, including process death and stale input."""

import bisect
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import tempfile
import time
import unittest
import uuid

import startup_test as support


class MatchRehearsal(unittest.TestCase):
    window = support.FullStackSimulation.window
    assert_driving = support.FullStackSimulation.assert_driving
    assert_neutral = support.FullStackSimulation.assert_neutral
    assert_neutral_by = support.FullStackSimulation.assert_neutral_by

    def test_match_restarts_only_killed_node_and_rejects_stale_targets(self):
        root = support.workspace()
        config_dir = root / "2026-robot/main_processor/configuration"
        config = config_dir / "robot_physics_sim.toml"
        robot = support.tomllib.loads((config_dir / "robot.toml").read_text())
        # The consumer tick, not two ticks, is the allowed scheduling margin.
        bound = (robot["robot"]["command_timeout_us"] +
                 robot["robot"]["period_us"]) / 1000
        layout = support.device_layout(config_dir / "robot.toml")
        motor_ids = [i for i, sub, _, kind in layout
                     if sub == "drivetrain" and kind == "motors"]
        drive_ids = [i for i, sub, name, kind in layout
                     if sub == "drivetrain" and kind == "motors" and
                     name.endswith("_drive")]
        gw_port, bridge_port = support.udp_ports()
        tmp = Path(tempfile.mkdtemp(
            prefix="talos-match-rehearsal-",
            dir=os.environ.get("TEST_UNDECLARED_OUTPUTS_DIR")))
        output = tmp / "sessions"
        env = dict(os.environ, TALOS_SHM_NAMESPACE=f"rehearsal-{uuid.uuid4().hex}")
        launcher = None
        paused = None
        success = False
        try:
            (tmp / "bazel-bin").symlink_to(root, target_is_directory=True)
            args = [str(root / "talOS/launcher/launcher"), "--config", str(config),
                    "--sim", "--start-sim-gateway", "--remote", "127.0.0.1",
                    "--remote-port", str(gw_port), "--local-port", str(bridge_port),
                    "--output-dir", str(output), "--duration-s", "17"]
            gateway_args = [
                "--physics", "--port", str(gw_port), "--remote-port", str(bridge_port),
                "--mode-schedule", "disabled:1,auto:4,disabled:1,teleop:9,disabled:2",
                "--joystick-schedule",
                "0:axes=0,0,0;buttons=0;pov=-1|"
                "6.5:axes=0,-0.6,0;buttons=5;pov=-1|"
                "10:axes=0,0,0;buttons=0;pov=-1|"
                "10.4:axes=0,-0.6,0;buttons=0;pov=-1|"
                "13:axes=0,0,0;buttons=0;pov=-1|"
                "13.4:axes=0,-0.6,0;buttons=0;pov=-1"]
            for arg in gateway_args:
                args.extend(["--sim-gateway-arg", arg])
            with (tmp / "launcher.txt").open("w+") as log:
                launcher = subprocess.Popen(args, cwd=tmp, env=env, stdout=log,
                                            stderr=subprocess.STDOUT,
                                            start_new_session=True)
                deadline = time.monotonic() + 20
                while time.monotonic() < deadline:
                    manifests = list(output.glob("*/manifest.json"))
                    if manifests:
                        manifest_path = manifests[0]
                        manifest = json.loads(manifest_path.read_text())
                        nodes = {n["name"]: n for n in manifest["nodes"]}
                        if nodes.get("sim_gateway", {}).get("pid", 0) > 0:
                            break
                    self.assertIsNone(launcher.poll(), "launcher exited during startup")
                    time.sleep(0.01)
                else:
                    self.fail("gateway never started")
                anchor = time.monotonic()

                def at(seconds):
                    time.sleep(max(0, anchor + seconds - time.monotonic()))
                    self.assertIsNone(launcher.poll(), "launcher exited during match")

                # Freeze the producer before killing the consumer: its restarted
                # process must not actuate from a pre-restart shared-memory lane.
                at(8)
                driver_pid = nodes["driver"]["pid"]
                self.assertGreater(driver_pid, 0)
                paused = driver_pid
                os.kill(paused, signal.SIGSTOP)
                kill_ns = support.node_clock_ns()
                killed_pid = nodes["drivetrain"]["pid"]
                self.assertGreater(killed_pid, 0)
                os.kill(killed_pid, signal.SIGKILL)
                time.sleep(0.3)  # stale target, but below driver stall supervision
                first_resume_ns = support.node_clock_ns()
                os.kill(paused, signal.SIGCONT)
                paused = None
                at(12)
                paused = driver_pid
                os.kill(paused, signal.SIGSTOP)
                stop_ns = support.node_clock_ns()
                time.sleep(0.25)
                resume_ns = support.node_clock_ns()
                os.kill(paused, signal.SIGCONT)
                paused = None
                launcher.wait(timeout=15)
                log.seek(0)
                diagnostics = log.read()
            self.assertEqual(launcher.returncode, 0, diagnostics)
            session = manifest_path.parent
            manifest = json.loads(manifest_path.read_text())
            self.assertGreater(manifest["end_wall_ns"], manifest["start_wall_ns"])
            final_nodes = {n["name"]: n for n in manifest["nodes"]}
            self.assertEqual(set(final_nodes), set(nodes))
            for name, node in final_nodes.items():
                self.assertEqual(node["exit_code"], 0, name)
                self.assertFalse(node["recorder_failed"], name)
                self.assertFalse(node["quarantined"], name)
                self.assertEqual(node["restart_count"], int(name == "drivetrain"), name)
                if name != "sim_gateway":
                    self.assertGreater(node["record_count"], 0, name)
            self.assertNotEqual(final_nodes["drivetrain"]["pid"], killed_pid)
            self.assertEqual(len(final_nodes["drivetrain"]["log_paths"]), 2)

            dumped = subprocess.run(
                [str(root / "talOS/launcher/log_dump"), "--format", "jsonl",
                 "--node", "drivetrain,intake,shooter,autonomous,teleop",
                 "--topic", "/hw/request/*,/hw/state/driver_station,/drivetrain/target/*",
                 str(session)], capture_output=True, text=True, check=True, timeout=15)
            (tmp / "match.jsonl").write_text(dumped.stdout)
            rows = [json.loads(line) for line in dumped.stdout.splitlines()]
            baseline = support.log_baseline_ns(session)
            kill, first_resume, stop, resume = [
                (stamp - baseline) / 1e6 for stamp in
                (kill_ns, first_resume_ns, stop_ns, resume_ns)]
            requests = [r for r in rows if r["process"] == "drivetrain" and
                        r["topic"] == "/hw/request/drivetrain" and r["kind"] == "SEND"]
            ds = [r for r in rows if r["process"] == "intake" and
                  r["topic"] == "/hw/state/driver_station" and r["kind"] == "MESSAGE"]
            self.assertGreater(len(ds), 500)
            phases = []
            for row in ds:
                flags, _ = support.ds_snapshot(row)
                if not phases or phases[-1][1] != flags:
                    phases.append((row["now_ms"], flags))
            self.assertEqual([f for _, f in phases], [1, 13, 1, 21, 1])
            # For each node, join each outgoing request to that node's actual
            # latest DS receipt. This includes transitions, not a hand-picked
            # settled subset, and checks every motor (including mechanisms).
            for name in ("drivetrain", "intake", "shooter"):
                snapshots = [r for r in rows if r["process"] == name and
                             r["topic"] == "/hw/state/driver_station" and
                             r["kind"] == "MESSAGE"]
                times = [r["now_ms"] for r in snapshots]
                outgoing = [r for r in rows if r["process"] == name and
                            r["topic"] == f"/hw/request/{name}" and r["kind"] == "SEND"]
                subsystem = support.tomllib.loads(
                    (config_dir / "subsystems" / f"{name}.toml").read_text())
                period_us = subsystem["subsystem"].get(
                    "period_us", robot["robot"]["period_us"])
                self.assertGreater(len(outgoing), 16_000_000 / period_us, name)
                for row in outgoing:
                    index = bisect.bisect_right(times, row["now_ms"]) - 1
                    enabled = index >= 0 and support.ds_snapshot(snapshots[index])[0] & 4
                    if not enabled:
                        self.assertTrue(all(m == (0, 0, 0.0, 0.0) for m in
                                            support.motor_requests(row).values()), row)
            auto_start, auto_end = phases[1][0], phases[2][0]
            self.assertTrue(any(auto_start < r["now_ms"] < auto_end and
                                any(m[0] != 0 and abs(m[2]) > 0.1 for m in
                                    support.motor_requests(r).values()) for r in requests),
                            "autonomous routine never drove")
            intake = [r for r in rows if r["process"] == "intake" and
                      r["topic"] == "/hw/request/intake" and r["kind"] == "SEND"]
            self.assertTrue(any(phases[3][0] + 800 < r["now_ms"] < kill and
                                any(m[0] != 0 and abs(m[2]) > 0.1 for m in
                                    support.motor_requests(r).values()) for r in intake),
                            "teleop mechanism buttons never actuated intake")
            self.assert_driving(requests, kill - 150, kill, drive_ids)
            restarted = [r for r in requests if kill + 10 < r["now_ms"] < first_resume]
            self.assertGreater(len(restarted), 5, "consumer did not restart during pause")
            self.assert_neutral(requests, restarted[0]["now_ms"], first_resume, motor_ids)
            # Every non-neutral post-restart output must follow a freshly
            # received active target, published after the crash (not retained).
            targets = [r for r in rows if r["process"] == "drivetrain" and
                       r["topic"] == "/drivetrain/target/teleop" and
                       r["kind"] == "MESSAGE" and r["now_ms"] > kill]
            for row in requests:
                if row["now_ms"] <= kill or not any(m[0] != 0 for m in
                                                                  support.motor_requests(row).values()):
                    continue
                fresh = [t for t in targets if t["now_ms"] <= row["now_ms"] and
                         0 <= row["now_ms"] - t["publish_ms"] <= bound and
                         t["publish_ms"] > kill and
                         bytes.fromhex(t["payload_hex"])[8] != 0]
                self.assertTrue(fresh, f"output without fresh post-crash target: {row}")
            self.assert_driving(requests, stop - 150, stop, drive_ids)
            self.assert_neutral_by(requests, stop, stop + bound, resume, motor_ids)
            center = next(r["now_ms"] for r in ds if r["now_ms"] > resume and
                          support.ds_snapshot(r)[1] == 0)
            self.assert_neutral(requests, resume, center, motor_ids)
            self.assert_driving(requests, center + 700, center + 1000, drive_ids)
            self.assert_neutral(requests, phases[-1][0] + 30,
                                phases[-1][0] + 500, motor_ids)

            # Only non-fatal host real-time privilege warnings are allowed.
            # Deliberate SIGKILL is a launcher diagnostic, never a node error.
            allowlist = (
                re.compile(r".*(?:mlockall|sched_setscheduler|SCHED_FIFO|"
                           r"real.time|affinity).*(?:permitted|permission|failed)", re.I),
                re.compile(r"  • \w+ \(pid=\d+, exit=0, records=\d+, restarts=[01], "
                           r"failed=no, log=.*\)"),
            )
            error = re.compile(r"\b(?:error|failed|fatal|exception|traceback|segmentation|abort)\b", re.I)
            # Ordinary nodes inherit the launcher's stdout/stderr; optional
            # host-side processes may also have separate plaintext logs.
            logs = [("launcher.txt", diagnostics)] + [
                (path.name, path.read_text()) for path in sorted(session.glob("*.log"))]
            for name, text in logs:
                for line in text.splitlines():
                    if error.search(line):
                        self.assertTrue(any(p.fullmatch(line) for p in allowlist),
                                        f"unexpected {name}: {line}")
            success = True
        finally:
            if paused is not None:
                try:
                    os.kill(paused, signal.SIGCONT)
                except ProcessLookupError:
                    pass
            if launcher is not None:
                if not success:
                    try:
                        os.killpg(launcher.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                launcher.wait(timeout=5)
            (tmp / "bazel-bin").unlink(missing_ok=True)
            if success:
                shutil.rmtree(tmp)
            else:
                print(f"Match rehearsal artifacts retained: {tmp}", flush=True)
                print((tmp / "launcher.txt").read_text(), flush=True)


if __name__ == "__main__":
    unittest.main()
