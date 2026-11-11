import json
import os
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

from python.runfiles import runfiles
from talOS.launcher import session_report


class SessionReportTest(unittest.TestCase):
    def test_recorded_session(self):
        files = runfiles.Create()
        fixture = files.Rlocation("_main/talOS/launcher/session_report_fixture")
        binary = files.Rlocation("_main/talOS/launcher/session_report")
        with tempfile.TemporaryDirectory() as directory:
            subprocess.run([fixture, directory], check=True)
            result = subprocess.run([binary, "--json", "--strict", directory],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 2, result.stderr)
            report = json.loads(result.stdout)
            self.assertEqual(report["config_sha256"], "fixture-sha256")
            self.assertEqual(report["state_overall"],
                             {"count": 7, "median_ms": 5, "p99_ms": 200, "max_ms": 200})
            self.assertEqual(report["state_enabled"],
                             {"count": 5, "median_ms": 5, "p99_ms": 120, "max_ms": 120})
            self.assertEqual(report["enabled_gaps_ge_100_ms"], 1)
            self.assertEqual(report["timer_lateness"]["fixture:update"]["max_ms"], 1)
            self.assertEqual(report["target_receipt_age"]["fixture:/fixture/target/test"]["median_ms"], 3)
            verdicts = {v["measurement"]: v["verdict"] for v in report["verdicts"]}
            self.assertEqual(verdicts["enabled state median"], "PASS")
            self.assertEqual(verdicts["enabled state gaps"], "FAIL")
            self.assertEqual(report["target_age_at_tick"],
                             {"fixture:/fixture/target/test":
                                  {"count": 1, "median_ms": 4, "p99_ms": 4, "max_ms": 4}})
            # The manifest's signal death reaches the node summary.
            self.assertEqual(report["nodes"][0]["exit_signal"], 9)
            self.assertEqual(verdicts["target age at tick"], "PASS")

    # REQ-RT-20 (the bridge acts on a request when it arrives), REQ-OPS-16.
    def test_disabled_session_of_an_event_driven_bridge(self):
        files = runfiles.Create()
        fixture = files.Rlocation("_main/talOS/launcher/session_report_fixture")
        binary = files.Rlocation("_main/talOS/launcher/session_report")
        with tempfile.TemporaryDirectory() as directory:
            subprocess.run([fixture, directory, "disabled-bridge"], check=True)
            result = subprocess.run([binary, "--strict", directory],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 2, result.stderr)
            report = json.loads(subprocess.run(
                [binary, "--json", directory], capture_output=True, text=True,
                check=True).stdout)
        # The bridge has no timer: it is judged on the time from a request to
        # the first command carrying its observation; the heartbeat that
        # repeats an observation 20 ms later is not a reaction to a request.
        bridge = report["bridge_request_to_command"]
        self.assertEqual(bridge["process"], "hardware_node")
        self.assertEqual(bridge["latency"]["count"], 5)
        self.assertAlmostEqual(bridge["latency"]["max_ms"], 0.25)
        self.assertNotIn("hardware_node timer lateness: NOT MEASURED", result.stdout)
        # Never enabled: only the enabled-only rows stay unmeasured, each
        # still failing --strict (REQ-RT-15), and the report says why.
        self.assertEqual(sorted(line for line in result.stderr.splitlines()
                                if line.startswith("session_report: strict:")),
                         ["session_report: strict: enabled state gaps: NOT MEASURED",
                          "session_report: strict: enabled state median: NOT MEASURED",
                          "session_report: strict: enabled state p99: NOT MEASURED",
                          "session_report: strict: target age at tick: NOT MEASURED"])
        self.assertIn("robot never enabled", result.stdout)

    def test_strict_names_a_damaged_log_and_its_offset(self):
        files = runfiles.Create()
        fixture = files.Rlocation("_main/talOS/launcher/session_report_fixture")
        binary = files.Rlocation("_main/talOS/launcher/session_report")
        with tempfile.TemporaryDirectory() as directory:
            subprocess.run([fixture, directory], check=True)
            log = os.path.join(directory, "fixture.tlog")
            with open(log, "r+b") as file:
                data = file.read()
                # FileHeader (format.h): header_bytes at 12, manifest_count
                # at 40, schema_bytes at 112; then 112-byte manifest entries
                # and the schema block.
                (header,) = struct.unpack_from("<I", data, 12)
                (count,) = struct.unpack_from("<I", data, 40)
                (schemas,) = struct.unpack_from("<Q", data, 112)
                offset = header + 112 * count + schemas
                # Damage the second record's dispatch index; its checksum no
                # longer matches, and intact records follow it.
                (stored,) = struct.unpack_from("<I", data, offset + 60)
                offset += 64 + stored
                file.seek(offset + 8)
                file.write(bytes([data[offset + 8] ^ 0xFF]))
            result = subprocess.run([binary, "--json", "--strict", directory],
                                    capture_output=True, text=True)
            damaged = [issue for issue in json.loads(result.stdout)["session_issues"]
                       if issue.startswith("damaged log " + log)]
            self.assertEqual(len(damaged), 1, result.stderr)
            self.assertIn(f"first at byte {offset})", damaged[0])

    def test_descriptor_status_and_no_duplicate_readers(self):
        def record(process, kind, topic, now, decoded=""):
            return dict(process=process, kind=kind, topic=topic, now_ms=now,
                        event_ms=now, publish_ms=now, decoded=decoded)
        records = [record("bridge", "SEND", "/hw/state", t) for t in (0, 5, 10)]
        records += [record("consumer", "MESSAGE", "/hw/state", t) for t in (1, 6, 11)]
        records += [record("bridge", "SEND", "/hw/status", i, value) for i, value in enumerate((
            "{identity_rejected_count: 7, clock_offset_valid: false, clock_offset_us: -30, clock_offset_spread_us: 10, clock_uncertainty_us: 9}",
            "{identity_rejected_count: 9, clock_offset_valid: true, clock_offset_us: -20, clock_offset_spread_us: 20, clock_uncertainty_us: 4}",
            "{identity_rejected_count: 1, clock_offset_valid: true, clock_offset_us: -10, clock_offset_spread_us: 30, clock_uncertainty_us: 2}"))]
        report = session_report.analyze({"nodes": []}, records)
        self.assertEqual(report["state_overall"]["count"], 2)
        self.assertIsNone(report["state_enabled"])
        self.assertEqual(report["status"]["identity_rejected_count"]["delta"], 2)
        self.assertEqual(report["status"]["identity_rejected_count"]["resets"], 1)
        self.assertEqual(report["status"]["clock_offset_valid"]["fraction"], 2 / 3)
        self.assertEqual(report["status"]["clock_uncertainty_us"]["min"], 2)

    def test_target_age_at_tick_limit(self):
        def record(kind, topic, now, age_ms=0, **extra):
            return dict(process="lift", kind=kind, topic=topic, now_ms=now, event_ms=now,
                        publish_ms=now - age_ms, decoded="", **extra)
        enabled = [record("SEND", "/hw/state/driver_station", 0, ds_enabled=True)]
        def lane(mode_since_us, active):
            message = record("MESSAGE", "/lift/target/teleop", 1)
            message["decoded"] = (f"{{mode_since_us: {mode_since_us}, active: {active}, "
                                  "message: {position_rot: 0.5}}")
            return [message]
        live = lane(1200, "true")
        def read(age_ms):
            return record("READ", "/lift/target/teleop", 2, age_ms)
        for session, verdict in ((live + [read(0), read(25)], "PASS"),
                                 (live + [read(25.001)], "FAIL"),
                                 # Active while enabled, never selected: a
                                 # real finding such as a generation mismatch.
                                 (enabled + live, "FAIL"),
                                 # Released (neutral declarative teleop) or
                                 # unstamped: nothing a Target could select.
                                 (enabled + lane(1200, "false"), "NOT MEASURED"),
                                 (enabled + lane(0, "true"), "NOT MEASURED"),
                                 # Nothing to select from.
                                 (enabled, "NOT MEASURED")):
            report = session_report.analyze({"nodes": []}, session)
            self.assertEqual({v["measurement"]: v["verdict"] for v in report["verdicts"]}
                             ["target age at tick"], verdict, session)

    def test_runs_after_dropped_ones_keep_their_spawn_numbers(self):
        # The launcher keeps the first run and the newest; the ones between
        # are counted in runs_dropped, so the newest kept are runs 3 and 4.
        def run(pid, reason, end=2_000_000_000):
            return {"pid": pid, "start_wall_ns": 1_000_000_000, "end_wall_ns": end,
                    "exit_code": -1, "exit_signal": 9, "reason": reason}
        manifest = {"nodes": [{"name": "arm", "runs_dropped": 2, "runs": [
            run(10, "stalled"), run(13, "crash"), run(14, "running", end=0)]}]}
        text = session_report.markdown(session_report.analyze(manifest, []))
        runs = text[text.index("## Runs"):text.index("## Session completeness")]
        rows = [line for line in runs.splitlines() if line.startswith("| arm |")]
        self.assertEqual([row.split(" | ")[2:4] for row in rows[:3]],
                         [["0", "10"], ["3", "13"], ["4", "14"]])
        self.assertIn("1.000 | -1 | 9 | stalled", rows[0])
        self.assertIn("running | -1 | 9 | running", rows[2])
        self.assertIn("2 runs after the first dropped", rows[3])

    def test_target_lanes_follow_the_naming_grammar(self):
        # Mirrors naming::IsTargetLane: only a lane's records count as target
        # traffic.
        for topic in ("/lift/target/teleop", "/drivetrain/target/align_2"):
            self.assertTrue(session_report.is_target_lane(topic), topic)
        for topic in ("/Lift/target/teleop", "/lift/target/tele__op",
                      "/lift/target/teleop_", "/lift/target/2nd",
                      "/lift/target/" + "s" * 19, "/lift/target/teleop/x"):
            self.assertFalse(session_report.is_target_lane(topic), topic)

    def test_empty_session_has_no_pass(self):
        report = session_report.analyze({"nodes": []}, [])
        self.assertTrue(all(v["verdict"] == "NOT MEASURED" for v in report["verdicts"]))

    def test_strict_rejects_unmeasured_and_incomplete_sessions(self):
        records = [dict(process="bridge", kind="SEND", topic="/hw/state/driver_station",
                        now_ms=0, event_ms=0, publish_ms=0, decoded="", ds_enabled=True)]
        records += [dict(process="bridge", kind="SEND", topic="/hw/state", now_ms=t,
                         event_ms=t, publish_ms=t, decoded="") for t in range(0, 55, 5)]
        flags = {"quarantined": True, "recorder_failed": True, "log_capped": True}
        bridge = {"name": "bridge", "log_path": "/s/bridge.tlog"}
        # A failed viewer without an event log is not acceptance evidence.
        studio = {"name": "studio_bridge", "log_path": "", **flags}
        cases = (
            # No target lane was dispatched: only that required row is
            # NOT MEASURED; timer lateness is informational.
            ({"end_wall_ns": 1, "nodes": [bridge, studio]}, records, 1),
            # The hardware node recorded no /hw/state: six required verdicts.
            ({"end_wall_ns": 1, "nodes": [bridge]}, [], 6),
            ({"end_wall_ns": 0, "nodes": [{**bridge, **flags}, studio]}, records, 5),
        )
        for manifest, session, failures in cases:
            self.assertEqual(len(session_report.strict_failures(
                session_report.analyze(manifest, session))), failures, manifest)

    def test_strict_rejects_an_actuation_loop_without_its_realtime_grant(self):
        # The registry row's outcome, as the launcher records it.
        def node(name, tier, grant, refused=""):
            return {"name": name, "log_path": f"/s/{name}.tlog", "loop_tier": tier,
                    "realtime": grant, "realtime_refused": refused}
        def failing(manifest):
            # The nodes whose realtime issue --strict rejects the session for.
            report = session_report.analyze({"end_wall_ns": 1, **manifest}, [])
            failures = session_report.strict_failures(report)
            return {issue.split(":", 1)[0] for issue in report["realtime_issues"]
                    if issue in failures}
        robot = [node("hardware_node", "bridge", "full"),
                 node("shooter", "control", "refused", "policy"),
                 node("intake", "control", "partial", "ring mlock"),
                 # An observer, a node that never reached its loop, or one in
                 # a manifest that predates the outcome is reported but not
                 # judged.
                 node("telemetry", "observer", "refused", "policy"),
                 node("camera", "", ""),
                 {"name": "old", "log_path": "/s/old.tlog"}]
        self.assertEqual(failing({"simulation": False, "nodes": robot}), {"shooter", "intake"})
        # On the robot, not asking (TALOS_REALTIME=0) is no excuse.
        self.assertEqual(failing({"simulation": False, "nodes": [
            node("shooter", "control", "not_requested")]}), {"shooter"})
        # Under --sim a loop asks only with TALOS_REALTIME=1; one that asked
        # and was refused still fails.
        self.assertEqual(failing({"simulation": True, "nodes": [
            node("shooter", "control", "not_requested"),
            node("hardware_node", "bridge", "refused", "policy")]}), {"hardware_node"})

    def test_missing_log_dump_is_an_input_error(self):
        with tempfile.TemporaryDirectory() as directory, \
                mock.patch.object(session_report.runfiles, "Create", return_value=None), \
                mock.patch.object(sys, "argv", ["session_report", directory]):
            with open(directory + "/manifest.json", "w") as manifest:
                manifest.write('{"nodes": []}')
            self.assertEqual(session_report.main(), 1)


if __name__ == "__main__":
    unittest.main()
