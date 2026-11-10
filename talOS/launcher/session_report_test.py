import json
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

    def test_missing_log_dump_is_an_input_error(self):
        with tempfile.TemporaryDirectory() as directory, \
                mock.patch.object(session_report.runfiles, "Create", return_value=None), \
                mock.patch.object(sys, "argv", ["session_report", directory]):
            with open(directory + "/manifest.json", "w") as manifest:
                manifest.write('{"nodes": []}')
            self.assertEqual(session_report.main(), 1)


if __name__ == "__main__":
    unittest.main()
