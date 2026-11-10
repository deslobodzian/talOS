"""A recorded driver/match rehearsal, including process death and stale input."""

import bisect
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import tempfile
import time
import unittest
import uuid

import autonomous_progress
import startup_test as support


class MatchRehearsal(unittest.TestCase):
    window = support.FullStackSimulation.window
    assert_driving = support.FullStackSimulation.assert_driving
    assert_neutral = support.FullStackSimulation.assert_neutral
    assert_neutral_by = support.FullStackSimulation.assert_neutral_by

    def print_restart_timeline(self, manifest_path, kill_ns):
        """Show whether the old exit/spawn or the new node's startup was slow."""
        try:
            nodes = {n["name"]: n for n in
                     json.loads(manifest_path.read_text())["nodes"]}
            origin, first, send = support.restart_timeline(
                nodes["drivetrain"]["log_paths"][-1], "/hw/request/drivetrain")
            ms = lambda a, b: "n/a" if a is None or b is None else f"{(b - a) / 1e6:.1f} ms"
            print(f"drivetrain restart: kill->loop origin {ms(kill_ns, origin)}, "
                  f"origin->run() {ms(origin, first)}, "
                  f"run()->first request {ms(first, send)}", flush=True)
        except Exception as error:  # diagnostics must not mask the failure
            print(f"drivetrain restart timeline unavailable: {error!r}", flush=True)

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
        env = support.launcher_env(TALOS_SHM_NAMESPACE=f"rehearsal-{uuid.uuid4().hex}")
        launcher = None
        manifest_path = None
        kill_ns = None
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
            support.assert_realtime(self, diagnostics)
            session = manifest_path.parent
            manifest = json.loads(manifest_path.read_text())
            self.assertGreater(manifest["end_wall_ns"], manifest["start_wall_ns"])
            final_nodes = {n["name"]: n for n in manifest["nodes"]}
            for name, node in final_nodes.items():
                self.assertEqual(node["exit_code"], 0, name)
                self.assertFalse(node["recorder_failed"], name)
                self.assertFalse(node["quarantined"], name)
                self.assertEqual(node["restart_count"], int(name == "drivetrain"), name)
            self.assertNotEqual(final_nodes["drivetrain"]["pid"], killed_pid)
            self.assertEqual(len(final_nodes["drivetrain"]["log_paths"]), 2)

            dumped = subprocess.run(
                [str(root / "talOS/launcher/log_dump"), "--format", "jsonl",
                 "--node", "hardware_node,drivetrain,intake,shooter,autonomous,teleop",
                 "--topic", "/hw/command,/hw/request/*,/hw/state/driver_station,"
                 "/drivetrain/target/*",
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
            # Coverage, not a count: a slow host drains fewer packets. A
            # loaded 3-vCPU macOS runner at timeshare priority (Linux still
            # runs so) delayed the intake's drains by 260 ms, so the
            # bound is an outage (1 s), not jitter; the neutral/driving windows
            # below check behaviour, and session_report measures jitter on a
            # real host.
            ds_times = [r["now_ms"] for r in ds]
            self.assertGreater(len(ds_times), 2)
            gap, at = max((b - a, a) for a, b in zip(ds_times, ds_times[1:]))
            print(f"largest Driver Station receipt gap: {gap:.1f} ms at {at:.1f} ms",
                  flush=True)
            self.assertLess(gap, 1000, "Driver Station stream outage")
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
            # SIGKILL loses the killed process's unflushed recording (up to
            # one recorder chunk), so pre-kill actuation is read from what the
            # bridge actually sent to the gateway.
            commands = [r for r in rows if r["process"] == "hardware_node" and
                        r["topic"] == "/hw/command" and r["kind"] == "SEND"]
            self.assert_driving(commands, kill - 150, kill, drive_ids)
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
            autonomous_progress.remove(env["TALOS_SHM_NAMESPACE"])
            if success:
                shutil.rmtree(tmp)
            else:
                print(f"Match rehearsal artifacts retained: {tmp}", flush=True)
                self.print_restart_timeline(manifest_path, kill_ns)
                print((tmp / "launcher.txt").read_text(), flush=True)


if __name__ == "__main__":
    unittest.main()
