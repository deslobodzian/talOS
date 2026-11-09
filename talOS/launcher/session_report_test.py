import json
import subprocess
import tempfile
import unittest

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
            self.assertEqual(verdicts["target age at tick"], "NOT MEASURED")
            text = subprocess.run([binary, directory], capture_output=True, text=True, check=True).stdout
            self.assertIn("**FAIL** enabled state gaps: 1", text)
            self.assertIn("fixture-sha256", text)

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

    def test_empty_session_has_no_pass(self):
        report = session_report.analyze({"nodes": []}, [])
        self.assertTrue(all(v["verdict"] == "NOT MEASURED" for v in report["verdicts"]))


if __name__ == "__main__":
    unittest.main()
