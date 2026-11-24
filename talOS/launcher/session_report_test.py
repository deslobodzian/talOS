import contextlib
import io
import json
import os
import pathlib
import re
import shutil
import struct
import subprocess
import sys
import tempfile
import unittest
from unittest import mock
import zlib

from python.runfiles import runfiles
from talOS.launcher import session_report

# What the launcher records from the bench config's [robot]: the bounds the
# /hw/state cadence rows are judged against.
BENCH_BOUNDS = {"period_us": 5000, "command_timeout_us": 100000}


class SessionReportTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        files = runfiles.Create()
        cls._fixture = files.Rlocation("_main/talOS/launcher/session_report_fixture")
        cls._binary = files.Rlocation("_main/talOS/launcher/session_report")
        cls._probe = files.Rlocation("_main/talOS/launcher/target_lane_probe")
        cls._default_fixture_tmp = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls._default_fixture_tmp.cleanup)
        cls._default_fixture_dir = cls._default_fixture_tmp.name
        subprocess.run([cls._fixture, cls._default_fixture_dir], check=True)

    @staticmethod
    def run_in_process(*args):
        """Runs session_report.main() in-process (still executing the real
        log_dump binary) to avoid redundant Python interpreter startups while
        preserving full CLI argument, stdout/stderr, and exit-code coverage."""
        stdout, stderr = io.StringIO(), io.StringIO()
        with mock.patch.object(sys, "argv", ["session_report", *args]), \
                contextlib.redirect_stdout(stdout), \
                contextlib.redirect_stderr(stderr):
            returncode = session_report.main()
        return subprocess.CompletedProcess(
            ["session_report", *args], returncode, stdout.getvalue(), stderr.getvalue())

    def test_recorded_session(self):
        result = self.run_in_process("--json", "--strict", self._default_fixture_dir)
        self.assertEqual(result.returncode, 2, result.stderr)
        report = json.loads(result.stdout)
        self.assertEqual(report["config_sha256"], "fixture-sha256")
        self.assertEqual(report["state_overall"],
                         {"count": 7, "min_ms": 5, "median_ms": 5, "p99_ms": 200, "max_ms": 200})
        self.assertEqual(report["state_enabled"],
                         {"count": 5, "min_ms": 5, "median_ms": 5, "p99_ms": 120, "max_ms": 120})
        self.assertEqual((report["enabled_gaps"], report["enabled_gap_bound_ms"]), (1, 100))
        self.assertEqual(report["state_p99_bound_ms"], 10)
        self.assertEqual(report["timer_lateness"]["fixture:update"]["max_ms"], 1)
        self.assertEqual(report["target_receipt_age"]["fixture:/fixture/target/test"]["median_ms"], 3)
        verdicts = {v["measurement"]: v["verdict"] for v in report["verdicts"]}
        self.assertEqual(verdicts["enabled state median"], "PASS")
        self.assertEqual(verdicts["enabled state gaps"], "FAIL")
        self.assertEqual(report["target_age_at_tick"],
                         {"fixture:/fixture/target/test":
                              {"count": 1, "min_ms": 4, "median_ms": 4, "p99_ms": 4, "max_ms": 4}})
        # The manifest's signal death reaches the node summary.
        self.assertEqual(report["nodes"][0]["exit_signal"], 9)
        self.assertEqual(verdicts["target age at tick"], "PASS")

    # REQ-RT-15, REQ-OPS-16 (TODO.md SPEC-4): the /hw/state cadence is judged
    # against the recorded config, not the bench's. A robot with period_us
    # 10000 and command_timeout_us 250000 publishes a State every 10 ms with
    # one 150 ms enabled hole: p99 10 ms is under 2 x 10 ms, and 150 ms is
    # under its 250 ms timeout, so both pass where the bench's 10 ms and
    # 100 ms would fail them.
    def test_cadence_is_judged_against_the_manifests_period_and_timeout(self):
        with tempfile.TemporaryDirectory() as directory:
            subprocess.run([self._fixture, directory, "slow-gateway"], check=True)
            report = json.loads(subprocess.run(
                [self._binary, "--json", directory], capture_output=True, text=True,
                check=True).stdout)
        self.assertEqual(report["state_enabled"],
                         {"count": 116, "min_ms": 10, "median_ms": 10, "p99_ms": 10,
                          "max_ms": 150})
        verdicts = {v["measurement"]: v for v in report["verdicts"]}
        for name in ("overall state p99", "enabled state p99", "enabled state gaps"):
            with self.subTest(verdict=name):
                self.assertEqual(verdicts[name]["verdict"], "PASS", verdicts[name])
        # 2 x period_us (20 ms), not a tenth of the timeout (25 ms) or a literal.
        self.assertEqual(report["state_p99_bound_ms"], 20)
        self.assertEqual((report["enabled_gaps"], report["enabled_gap_bound_ms"]), (0, 250))

    # REQ-OPS-16: a manifest from before the launcher recorded its config's
    # period_us and command_timeout_us has no bound to judge the cadence
    # against: the rows are NOT MEASURED, never judged against a bench
    # constant, and --strict names each.
    def test_a_manifest_without_the_bounds_does_not_judge_the_cadence(self):
        with tempfile.TemporaryDirectory() as directory:
            subprocess.run([self._fixture, directory, "unrecorded-bounds"], check=True)
            result = self.run_in_process("--json", "--strict", directory)
        self.assertEqual(result.returncode, 2, result.stderr)
        report = json.loads(result.stdout)
        verdicts = {v["measurement"]: v for v in report["verdicts"]}
        for name, field in (("overall state p99", "period_us"),
                            ("enabled state p99", "period_us"),
                            ("enabled state gaps", "command_timeout_us")):
            with self.subTest(verdict=name):
                self.assertEqual(verdicts[name]["verdict"], "NOT MEASURED")
                self.assertIn(f"manifest records no {field}", verdicts[name]["detail"])
                self.assertIn(f"session_report: strict: {name}: NOT MEASURED",
                              result.stderr.splitlines())
        # The samples are still reported; only the judgement needs the bound.
        self.assertEqual(report["state_enabled"]["p99_ms"], 120)
        self.assertIsNone(report["state_p99_bound_ms"])
        self.assertEqual((report["enabled_gaps"], report["enabled_gap_bound_ms"]),
                         (None, None))

    # REQ-RT-20 (the bridge acts on a request when it arrives), REQ-OPS-16.
    def test_disabled_session_of_an_event_driven_bridge(self):
        with tempfile.TemporaryDirectory() as directory:
            subprocess.run([self._fixture, directory, "disabled-bridge"], check=True)
            result = subprocess.run([self._binary, "--strict", directory],
                                    capture_output=True, text=True)
            self.assertEqual(result.returncode, 2, result.stderr)
            json_result = self.run_in_process("--json", directory)
            self.assertEqual(json_result.returncode, 0, json_result.stderr)
            report = json.loads(json_result.stdout)
        # The bridge has no timer: it is judged on the time from a request to
        # the first command carrying its observation; the heartbeat that
        # repeats an observation 20 ms later is not a reaction to a request.
        bridge = report["bridge_request_to_command"]
        self.assertEqual(bridge["process"], "hardware_node")
        self.assertEqual(bridge["disabled"]["count"], 5)
        self.assertAlmostEqual(bridge["disabled"]["max_ms"], 0.25)
        self.assertIsNone(bridge["enabled"])
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

    # REQ-OPS-16, REQ-OPS-51: a session recorded under another hardware
    # protocol (its States rejected by this build's codec; its Driver Station
    # packets would still decode) is judged on no hardware packet: every
    # verdict that needs one is NOT MEASURED, not a PASS from bytes read with
    # this build's layout, and --strict names log_dump's one line, which
    # names this build's protocol. Verdicts that read only record times (the
    # /hw/state cadence) are still measured. log_dump's own tests cover how
    # it decides (the manifest's version, then the codecs).
    def test_strict_judges_no_hardware_packet_of_another_protocol(self):
        with tempfile.TemporaryDirectory() as directory:
            subprocess.run([self._fixture, directory, "foreign-protocol"], check=True)
            result = self.run_in_process("--json", "--strict", directory)
        self.assertEqual(result.returncode, 2, result.stderr)
        report = json.loads(result.stdout)
        verdicts = {v["measurement"]: v["verdict"] for v in report["verdicts"]}
        for name in ("enabled state median", "enabled state p99",
                     "enabled state gaps", "target age at tick"):
            with self.subTest(verdict=name):
                self.assertEqual(verdicts[name], "NOT MEASURED")
        self.assertEqual(verdicts["overall state median"], "PASS")
        named = [line for line in result.stderr.splitlines()
                 if line.startswith("session_report: strict: hardware packets not decoded")]
        self.assertEqual(len(named), 1, result.stderr)
        self.assertRegex(named[0], r"protocol \d+ ")
        self.assertIsNone(report["never_enabled"])

    # REQ-OPS-51: when the bridge's /hw/status protocol_version_id and
    # manifest.json's hardware_protocol_version disagree, session_report prints
    # one warning naming both versions and reads the session under the bridge's
    # version.
    def test_bridge_and_manifest_protocol_disagreement_warns_and_uses_bridges_version(self):
        # Verified reflection.Schema .bfbs for:
        #   namespace bridge; struct BridgeStatus { protocol_version_id:uint16; }
        bfbs = bytes([
            0x1c, 0x00, 0x00, 0x00, 0x42, 0x46, 0x42, 0x53, 0x14, 0x00, 0x1c, 0x00,
            0x04, 0x00, 0x08, 0x00, 0x0c, 0x00, 0x10, 0x00, 0x00, 0x00, 0x14, 0x00,
            0x00, 0x00, 0x18, 0x00, 0x14, 0x00, 0x00, 0x00, 0x30, 0x00, 0x00, 0x00,
            0x28, 0x00, 0x00, 0x00, 0x1c, 0x00, 0x00, 0x00, 0x10, 0x00, 0x00, 0x00,
            0x08, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
            0x30, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
            0x94, 0xff, 0xff, 0xff, 0x3c, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x14, 0x00, 0x1c, 0x00, 0x08, 0x00, 0x0c, 0x00,
            0x07, 0x00, 0x10, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x18, 0x00,
            0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x28, 0x00, 0x00, 0x00,
            0x1c, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00,
            0x04, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x2f, 0x2f, 0x73, 0x2e,
            0x66, 0x62, 0x73, 0x00, 0x01, 0x00, 0x00, 0x00, 0x24, 0x00, 0x00, 0x00,
            0x13, 0x00, 0x00, 0x00, 0x62, 0x72, 0x69, 0x64, 0x67, 0x65, 0x2e, 0x42,
            0x72, 0x69, 0x64, 0x67, 0x65, 0x53, 0x74, 0x61, 0x74, 0x75, 0x73, 0x00,
            0x08, 0x00, 0x0c, 0x00, 0x04, 0x00, 0x08, 0x00, 0x08, 0x00, 0x00, 0x00,
            0x28, 0x00, 0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x10, 0x00, 0x10, 0x00,
            0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x08, 0x00, 0x0c, 0x00,
            0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x02, 0x00, 0x00, 0x00,
            0x01, 0x00, 0x00, 0x00, 0x13, 0x00, 0x00, 0x00, 0x70, 0x72, 0x6f, 0x74,
            0x6f, 0x63, 0x6f, 0x6c, 0x5f, 0x76, 0x65, 0x72, 0x73, 0x69, 0x6f, 0x6e,
            0x5f, 0x69, 0x64, 0x00,
        ])

        def write_bridge_status_log(path, session_id, version):
            schema_name = b"bridge.BridgeStatus"
            schema_block = schema_name + bfbs
            manifest_entry = struct.pack(
                "<HHIIIqq64sQII",
                1, 4, 2, 2, 0, 0, 0, b"/hw/status", 1, len(schema_name), len(bfbs))
            header_crc = zlib.crc32(manifest_entry + schema_block) & 0xFFFFFFFF
            file_header = struct.pack(
                "<8sIIQqqII64sQ",
                b"TALOSLOG", 8, 120, session_id, 1_000_000_000, 0,
                1, header_crc, b"hardware_node", len(schema_block))
            payload = struct.pack("<H", version)
            stored = payload.rstrip(b"\x00")
            rec_zero = struct.pack(
                "<IHHQqqqQIIII",
                0x54524543, 4, 1, 1, 1_000_000_000, 1_000_000_000,
                1_000_000_000, 1, 0, len(payload), 0, len(stored))
            rec_crc = zlib.crc32(stored, zlib.crc32(rec_zero)) & 0xFFFFFFFF
            rec_header = struct.pack(
                "<IHHQqqqQIIII",
                0x54524543, 4, 1, 1, 1_000_000_000, 1_000_000_000,
                1_000_000_000, 1, 0, len(payload), rec_crc, len(stored))
            with open(path, "wb") as f:
                f.write(file_header + manifest_entry + schema_block + rec_header + stored)

        with tempfile.TemporaryDirectory() as directory:
            shutil.copytree(self._default_fixture_dir, directory, dirs_exist_ok=True)
            manifest_path = os.path.join(directory, "manifest.json")
            with open(manifest_path) as f:
                manifest = json.load(f)
            build_version = manifest["hardware_protocol_version"]
            other_version = build_version - 1
            bridge_log = os.path.join(directory, "hardware_node.tlog")

            # Bridge matches build, manifest names another: warns once naming
            # both and decodes hardware packets under the bridge's version.
            manifest["hardware_protocol_version"] = other_version
            with open(manifest_path, "w") as f:
                json.dump(manifest, f)
            write_bridge_status_log(bridge_log, manifest["session_id"], build_version)
            result = self.run_in_process("--json", directory)
            self.assertEqual(result.returncode, 0, result.stderr)
            expected = (
                f"session_report: warning: the session's bridge recorded hardware protocol "
                f"{build_version} (/hw/status protocol_version_id), its launcher "
                f"{other_version} (manifest.json hardware_protocol_version): read under the bridge's"
            )
            self.assertEqual(result.stderr.splitlines().count(expected), 1, result.stderr)
            report = json.loads(result.stdout)
            self.assertIsNone(report["hardware_not_decoded"])
            self.assertEqual(report["state_enabled"]["count"], 5)

            # Bridge names another version, manifest matches build: warns once
            # naming both and refuses hardware packets under the bridge's version.
            manifest["hardware_protocol_version"] = build_version
            with open(manifest_path, "w") as f:
                json.dump(manifest, f)
            write_bridge_status_log(bridge_log, manifest["session_id"], other_version)
            result = self.run_in_process("--json", "--strict", directory)
            self.assertEqual(result.returncode, 2, result.stderr)
            expected = (
                f"session_report: warning: the session's bridge recorded hardware protocol "
                f"{other_version} (/hw/status protocol_version_id), its launcher "
                f"{build_version} (manifest.json hardware_protocol_version): read under the bridge's"
            )
            self.assertEqual(result.stderr.splitlines().count(expected), 1, result.stderr)
            report = json.loads(result.stdout)
            self.assertIsNotNone(report["hardware_not_decoded"])
            verdicts = {v["measurement"]: v["verdict"] for v in report["verdicts"]}
            self.assertEqual(verdicts["enabled state median"], "NOT MEASURED")

    # REQ-OPS-16: the default session with its enabled States at
    # 5 to 135 ms lost in a log gap. The 0 -> 140 ms interval across the hole
    # is the disk's, not the robot's: it is not measured, so the enabled rows,
    # left with no interval, are NOT MEASURED rather than a 140 ms FAIL; the
    # gap is listed in the text and JSON, and --strict names it.
    def test_a_log_gap_is_listed_not_measured_and_fails_strict(self):
        with tempfile.TemporaryDirectory() as directory:
            subprocess.run([self._fixture, directory, "log-gap"], check=True)
            result = self.run_in_process("--json", "--strict", directory)
            text_result = self.run_in_process(directory)
            self.assertEqual(text_result.returncode, 0, text_result.stderr)
            text = text_result.stdout
        self.assertEqual(result.returncode, 2, result.stderr)
        report = json.loads(result.stdout)
        verdicts = {v["measurement"]: v["verdict"] for v in report["verdicts"]}
        for name in ("enabled state median", "enabled state p99", "enabled state gaps"):
            with self.subTest(verdict=name):
                self.assertEqual(verdicts[name], "NOT MEASURED")
        self.assertIsNone(report["state_enabled"])
        self.assertEqual(report["state_overall"]["count"], 2)
        # log_dump's times count from the first record's event time, 1 ms
        # before the fixture's 0; the gap starts at the 5 ms State's event time.
        self.assertEqual(report["log_gaps"], [{
            "process": "fixture", "start_ms": 5, "end_ms": 141, "first_lost_ms": 6,
            "last_lost_ms": 136, "first_dispatch_index": 3, "last_dispatch_index": 6,
            "records": 4, "bytes": report["log_gaps"][0]["bytes"], "dispatches": 0,
            "sends": 4, "cause": "pool_full", "splits_dispatch": False,
            "closed_at_flush": False}])
        line = "fixture: log gap from 5 to 141 ms, 4 records lost (pool_full)"
        self.assertIn("- " + line, text.splitlines())
        self.assertIn("session_report: strict: " + line, result.stderr.splitlines())

    # REQ-OPS-16: a gap makes unmeasured only the intervals of the logs a
    # measurement reads (a stream's own log; for request to command, the
    # bridge's and the requesters'), and fails --strict on its own.
    def test_a_gap_suppresses_only_the_intervals_of_its_own_logs(self):
        def record(process, kind, topic, now, age_ms=0, **extra):
            return dict(process=process, kind=kind, topic=topic, now_ms=now,
                        event_ms=now, publish_ms=now - age_ms, decoded="", **extra)
        def gap(process, first, last, records):
            return record(process, "GAP", "?", last + 1, gap={
                "start_ms": first, "end_ms": last + 1, "first_lost_ms": first,
                "last_lost_ms": last, "records": records, "cause": "pool_full",
                "closed_at_flush": False})
        def strict(records):
            return session_report.strict_failures(
                session_report.analyze({"end_wall_ns": 1, "nodes": [], **BENCH_BOUNDS},
                                       records))
        lane = record("lift", "MESSAGE", "/lift/target/teleop", 50)
        lane["decoded"] = "{mode_since_us: 1200, active: true, message: {position_rot: 0.5}}"
        passing = [record("hardware_node", "SEND", "/hw/state/driver_station", 0,
                          ds_enabled=True), lane,
                   record("lift", "READ", "/lift/target/teleop", 51, 1)]
        passing += [record("hardware_node", "SEND", "/hw/state", t) for t in range(0, 105, 5)]
        self.assertEqual(strict(passing), [])
        camera = "camera: log gap from 20 to 61 ms, 3 records lost (pool_full)"
        # The bridge lost its 25 ms State: the 10 ms interval across the
        # hole is not measured, so it fails no p99; the camera's gap at the
        # same time is in another log and changes no measurement.
        lost_state = [r for r in passing if r["now_ms"] != 25 or r["topic"] != "/hw/state"]
        for label, records, failures, count in (
                ("another log", passing + [gap("camera", 20, 60, 3)], [camera], 20),
                ("the bridge's log", lost_state + [gap("hardware_node", 25, 25, 1)],
                 ["hardware_node: log gap from 25 to 26 ms, 1 records lost (pool_full)"], 18)):
            with self.subTest(label):
                self.assertEqual(strict(records), failures)
                self.assertEqual(session_report.analyze({"nodes": []}, records)
                                 ["state_overall"]["count"], count)

        # Request to command spans two logs. lift's request at 20 ms carrying
        # observation 2 is lost, so the next one, 25 ms, would read as a 1 ms
        # reaction to the 26 ms command; with that sample unmeasured, only
        # observation 1's 0.25 ms remains, whatever the camera lost.
        def hardware(process, topic, now, observed):
            return record(process, "SEND", topic, now,
                          hardware={"boot_id": 1, "observed_time_us": observed})
        traffic = [record("hardware_node", "SEND", "/hw/state", 0, hardware={
                       "boot_id": 1, "sample_time_us": 0, "configured": True,
                       "enabled": False, "hardware_fault": False}),
                   hardware("lift", "/hw/request/lift", 10, 1),
                   hardware("hardware_node", "/hw/command", 10.25, 1),
                   hardware("lift", "/hw/request/lift", 25, 2),
                   hardware("hardware_node", "/hw/command", 26, 2)]
        for label, gaps, latency in (
                ("requester's log", [gap("lift", 20, 20, 1), gap("camera", 0, 30, 9)],
                 {"count": 1, "min_ms": 0.25, "median_ms": 0.25, "p99_ms": 0.25, "max_ms": 0.25}),
                ("both logs", [gap("lift", 20, 20, 1), gap("hardware_node", 10, 10, 1)],
                 None)):
            with self.subTest(label):
                report = session_report.analyze({"nodes": []}, traffic + gaps)
                self.assertEqual(report["bridge_request_to_command"]["disabled"], latency)
        self.assertIn("| bridge request to command hardware_node disabled | NOT MEASURED "
                      "| — | — | — | — |", session_report.markdown(report).splitlines())

        # A gap at the head of lift's log, before its first kept request:
        # a request carrying observation 1 could be lost there, so the
        # 0.25 ms sample is no measurement.
        report = session_report.analyze({"nodes": []}, traffic[:3] + [gap("lift", 2, 2, 1)])
        self.assertIsNone(report["bridge_request_to_command"]["disabled"])

        # The bridge drops only the Driver Station packet disabling the robot
        # at 50 ms and keeps its States; until its next kept packet, at 70 ms,
        # the bit is unknown, so 55 -> 70 ms is neither enabled nor disabled,
        # and a lane dispatched at 60 ms is not one dispatched while enabled
        # (Target never selecting it is no FAIL).
        def ds(now, enabled):
            return record("hardware_node", "SEND", "/hw/state/driver_station", now,
                          ds_enabled=enabled)
        hidden = [ds(0, True), ds(70, False), gap("hardware_node", 50, 50, 1)]
        hidden += [record("hardware_node", "SEND", "/hw/state", t) for t in range(0, 105, 5)]
        lane_at_60 = dict(lane, now_ms=60, event_ms=60, publish_ms=60)
        report = session_report.analyze({"nodes": []}, hidden + [lane_at_60])
        # 0 -> 45 ms only: 45 -> 55 ms meets the gap in the State log too.
        self.assertEqual(report["state_enabled"]["count"], 9)
        self.assertEqual({v["measurement"]: v["verdict"] for v in report["verdicts"]}
                         ["target age at tick"], "NOT MEASURED")

        # An alert's active span across its stream's gap still counts as an
        # activation, but its clearing record could be lost: it is not timed.
        def status(publish, active):
            return record("watchdog", "SEND", "/watchdog/status", publish, alerts=[
                {"name": "alert_error_link_lost", "level": "error", "active": active}])
        alert = session_report.analyze({"nodes": []}, [
            status(20, True), gap("watchdog", 30, 30, 1), status(40, False)])["alerts"]
        self.assertEqual(alert["/watchdog/status/alert_error_link_lost"], {
            "level": "error", "stream": "watchdog/SEND", "first_active_ms": 20,
            "activations": 1, "longest_ms": None, "total_ms": None,
            "active_at_end": False, "spans_across_gap": 1})

    # REQ-OPS-16: a row of one-record samples spans no gap, but the samples a
    # gap dropped are missing: lift lost the READs of a slow stretch, so the
    # worst age it shows is not the worst it had, and the row says so where
    # it is read. A manifest's log_gap (a run killed during a gap leaves no
    # GAP record) still names the member under --strict.
    def test_rows_of_a_process_that_lost_records_say_so(self):
        def record(kind, topic, now, age_ms=0):
            return dict(process="lift", kind=kind, topic=topic, now_ms=now, event_ms=now - 1,
                        publish_ms=now - age_ms, decoded="")
        def gap(first, last, records):
            return dict(process="lift", kind="GAP", topic="?", now_ms=last + 1,
                        event_ms=first, publish_ms=0, decoded="", gap={
                            "start_ms": first, "end_ms": last + 1, "first_lost_ms": first,
                            "last_lost_ms": last, "records": records, "cause": "pool_full",
                            "closed_at_flush": False})
        session = [record("READ", "/lift/target/teleop", 10, 12),
                   record("TIMER", "update", 10), gap(20, 40, 5), gap(60, 60, 2)]
        manifest = {"end_wall_ns": 1, "nodes": [
            {"name": "lift", "log_path": "/s/lift.tlog", "log_gap": True,
             "recorder_failed": False}]}
        report = session_report.analyze(manifest, session)
        details = {v["measurement"]: v["detail"] for v in report["verdicts"]}
        lost = "; lift lost 7 records in 2 log gaps"
        for name in ("target age at tick", "timer lateness"):
            with self.subTest(row=name):
                self.assertTrue(details[name].endswith(lost), details[name])
        self.assertIn("lift lost 7 records in 2 log gaps: its target receipt, target at "
                      "tick and timer rows lack the samples those held.",
                      session_report.markdown(report).splitlines())
        self.assertIn("lift: log_gap", session_report.strict_failures(report))
        # Without a gap, nothing is appended.
        quiet = session_report.analyze({"nodes": []}, session[:2])
        self.assertNotIn("lost", {v["measurement"]: v["detail"]
                                  for v in quiet["verdicts"]}["timer lateness"])

    # REQ-RT-20, REQ-OPS-16: an owner timeout's neutral command adopts State
    # S; the node resumes with a request observing S after that command. The
    # request is not what the command reacted to, so it yields no sample (and
    # never a negative one).
    def test_a_request_after_its_command_is_no_reaction(self):
        def send(topic, now):
            return dict(process="hardware_node", kind="SEND", topic=topic, now_ms=now,
                        hardware={"boot_id": 1, "observed_time_us": 5000})
        _, delays, _ = session_report.request_to_command(
            [send("/hw/command", 10.0), send("/hw/request/lift", 12.0)])
        self.assertEqual(delays, [])

    # REQ-RT-25: each motor's and CAN sensor's feedback age is the older of
    # its position and velocity ages, as the RoboRIO measured them, over
    # every State of the bridge's own stream (a reader's copies are not
    # pooled), skipping a sample the State marks invalid. Phoenix's
    # unknown-stamp sentinel (UINT32_MAX, about 4295 s) is no age: it is
    # counted apart, never in the statistics.
    def test_feedback_age_per_device(self):
        def state(process, kind, now, motor_ages, sensor_valid=True):
            return dict(process=process, kind=kind, topic="/hw/state", now_ms=now,
                        event_ms=now, publish_ms=now, decoded="", hardware={
                            "type": "state", "boot_id": "7", "sample_time_us": now * 1000,
                            "motors": [{"id": 3, "valid": True, "position_age_us": motor_ages[0],
                                        "velocity_age_us": motor_ages[1]}],
                            "sensors": [{"id": 9, "valid": sensor_valid,
                                         "position_age_us": 400, "velocity_age_us": 900}]})
        records = [state("hardware_node", "SEND", 0, (1200, 300)),
                   state("hardware_node", "SEND", 5, (2500, 2600), sensor_valid=False),
                   state("hardware_node", "SEND", 10, (100, 4000)),
                   state("hardware_node", "SEND", 15, (2**32 - 1, 300)),
                   state("lift", "MESSAGE", 1, (99999, 99999))]
        report = session_report.analyze({"nodes": []}, records)
        self.assertEqual(report["feedback_age_us"], {
            "motor 3": {"count": 3, "min_us": 1200, "median_us": 2600, "p99_us": 4000, "max_us": 4000,
                        "sentinel": 1},
            "sensor 9": {"count": 3, "min_us": 900, "median_us": 900, "p99_us": 900, "max_us": 900,
                         "sentinel": 0}})

    # REQ-RT-25: State to request joins a State's (boot_id, sample_time_us),
    # a RoboRIO time, to an owner's first request carrying it as
    # (boot_id, observed_time_us), and subtracts only main-processor times:
    # the State's publication from the request's tick. The RoboRIO's clock
    # reads 7000 s here, so a cross-clock difference would be seconds, not
    # the 2 ms and 4 ms the owners took. A later request repeating the
    # observation, and one of another boot with the same sample time, are
    # not that State's first request.
    def test_state_to_request_joins_on_observation_and_one_clock(self):
        def send(process, topic, now, **hardware):
            return dict(process=process, kind="SEND", topic=topic, now_ms=now, event_ms=now,
                        publish_ms=now, decoded="", hardware=hardware)
        sample_us = 7_000_000_000
        records = [send("hardware_node", "/hw/state", 10, boot_id="7", sample_time_us=sample_us),
                   send("lift", "/hw/request/lift", 12, boot_id="7", observed_time_us=sample_us),
                   send("lift", "/hw/request/lift", 17, boot_id="7", observed_time_us=sample_us),
                   send("arm", "/hw/request/arm", 3, boot_id="8", observed_time_us=sample_us),
                   send("arm", "/hw/request/arm", 14, boot_id="7", observed_time_us=sample_us)]
        report = session_report.analyze({"nodes": []}, records)
        self.assertEqual(report["state_to_request_ms"], {
            "arm": {"count": 1, "min_ms": 4, "median_ms": 4, "p99_ms": 4, "max_ms": 4},
            "lift": {"count": 1, "min_ms": 2, "median_ms": 2, "p99_ms": 2, "max_ms": 2}})

    # REQ-RT-25, REQ-OPS-16: a sample whose span meets a gap of the owner's
    # log is NOT MEASURED, not a pass and not a shorter figure: lift's log
    # lost a record between the State and its kept request, which could be
    # an earlier request carrying the observation. The row stays listed.
    def test_a_gap_makes_state_to_request_not_measured(self):
        def send(process, topic, now, **hardware):
            return dict(process=process, kind="SEND", topic=topic, now_ms=now, event_ms=now,
                        publish_ms=now, decoded="", hardware=hardware)
        records = [send("hardware_node", "/hw/state", 10, boot_id="7", sample_time_us=5000),
                   send("lift", "/hw/request/lift", 18, boot_id="7", observed_time_us=5000),
                   dict(process="lift", kind="GAP", topic="?", now_ms=16, event_ms=16,
                        publish_ms=0, decoded="", gap={
                            "start_ms": 12, "end_ms": 16, "first_lost_ms": 13,
                            "last_lost_ms": 13, "records": 1, "cause": "pool_full",
                            "closed_at_flush": False})]
        report = session_report.analyze({"nodes": []}, records)
        self.assertEqual(report["state_to_request_ms"], {"lift": None})
        self.assertEqual(report["intervals_across_gaps"]["state_to_request"], 1)

    # REQ-RT-25: a Driver Station packet's publication (its publish time,
    # which a reader's copy carries; teleop received this one 3 ms later) to
    # the producer's next lane publication, a release included.
    def test_driver_station_to_lane_counts_from_publication(self):
        def record(kind, topic, publish, now, decoded=""):
            return dict(process="teleop", kind=kind, topic=topic, now_ms=now, event_ms=now,
                        publish_ms=publish, decoded=decoded)
        records = [record("MESSAGE", "/hw/state/driver_station", 0, 3),
                   record("MESSAGE", "/hw/state/driver_station", 20, 23),
                   record("SEND", "/lift/target/teleop", 5, 5),
                   record("SEND", "/lift/target/teleop", 10, 10),
                   record("SEND", "/lift/target/teleop", 26, 26)]
        report = session_report.analyze({"nodes": []}, records)
        self.assertEqual(report["driver_station_to_lane_ms"],
                         {"teleop": {"count": 2, "min_ms": 5, "median_ms": 5.5, "p99_ms": 6, "max_ms": 6}})

    # REQ-RT-25: Driver Station to lane is the reaction of
    # a producer that is publishing. teleop released its lane once, 3 ms
    # after the 0 ms packet, published nothing while disabled, and from
    # 82 ms, enabled, sent every 5 ms tick but stalled from 97 to 182 ms.
    # The disabled spell's packets at 20, 40 and 60 ms, and the enabled
    # packets at 100 to 160 ms, which it did not answer before the next
    # packet, are no samples of up to 82 ms: each is counted
    # unanswered, apart by the packet's enabled bit, so an enabled stall
    # stays visible among disabled silence. A gap in the packet stream's log
    # could hide the next packet, so the window of the packet before it is
    # not measured.
    def test_driver_station_to_lane_takes_only_packets_answered_before_the_next(self):
        def record(kind, topic, publish, process="teleop", **extra):
            return dict(process=process, kind=kind, topic=topic, now_ms=publish + 1,
                        event_ms=publish + 1, publish_ms=publish, decoded="", **extra)
        packets = [record("SEND", "/hw/state/driver_station", t, "hardware_node",
                          ds_enabled=t >= 80) for t in range(0, 200, 20)]
        sends = [record("SEND", "/lift/target/teleop", t)
                 for t in [3] + list(range(82, 98, 5)) + list(range(182, 200, 5))]
        report = session_report.analyze({"nodes": []}, packets + sends)
        self.assertEqual(report["driver_station_to_lane_ms"],
                         {"teleop": {"count": 3, "min_ms": 2, "median_ms": 2, "p99_ms": 3, "max_ms": 3}})
        self.assertEqual(report["driver_station_to_lane_unanswered"],
                         {"teleop": {"total": 7, "enabled": 4, "other": 3}})
        line = next(line for line in session_report.markdown(report).splitlines()
                    if line.startswith("Driver station to lane teleop:"))
        # The enabled count first.
        self.assertEqual(re.findall(r"\d+", line), ["4", "3"])
        # Packets recorded under another hardware protocol carry no enabled
        # bit: the split is not measured, never an enabled count of 0.
        undecoded = [{k: v for k, v in p.items() if k != "ds_enabled"} for p in packets]
        report = session_report.analyze({"nodes": []}, undecoded + sends,
                                        hardware_not_decoded="not decoded")
        self.assertEqual(report["driver_station_to_lane_unanswered"],
                         {"teleop": {"total": 7, "enabled": None, "other": None}})
        # The bridge's log lost the 80 ms packet: 60 -> 82 ms is no sample.
        gap = dict(record("GAP", "?", 80, "hardware_node"), gap={
            "start_ms": 79, "end_ms": 81, "first_lost_ms": 80, "last_lost_ms": 80,
            "records": 1, "cause": "pool_full", "closed_at_flush": False})
        hidden = [p for p in packets if p["publish_ms"] != 80] + [gap]
        report = session_report.analyze({"nodes": []}, hidden + sends)
        self.assertEqual(report["driver_station_to_lane_ms"]["teleop"]["count"], 2)
        self.assertEqual(report["intervals_across_gaps"]["driver_station_to_lane"], 1)

    # Driver Station to command's records: a packet, a tick in which a
    # Target selected a lane and its node sent a request, a merged command.
    @staticmethod
    def chain_record(process, kind, topic, now, publish=None, index=0, **extra):
        return dict(process=process, kind=kind, topic=topic, now_ms=now, event_ms=now,
                    publish_ms=now if publish is None else publish, decoded="",
                    dispatch_index=index, **extra)

    def ds_packet(self, publish, enabled, mode="teleop"):
        return self.chain_record("hardware_node", "SEND", "/hw/state/driver_station", publish,
                                 ds_enabled=enabled, hardware={"type": "driver_station",
                                                               "mode": mode})

    def selecting_tick(self, now, lane_publish, index, observed, lane="teleop"):
        return [self.chain_record("lift", "READ", f"/lift/target/{lane}", now, lane_publish,
                                  index),
                self.chain_record("lift", "SEND", "/hw/request/lift", now, index=index,
                                  hardware={"boot_id": "7", "observed_time_us": observed})]

    def merged_command(self, now, observed):
        return self.chain_record("hardware_node", "SEND", "/hw/command", now,
                                 hardware={"boot_id": "7", "observed_time_us": observed})

    # REQ-RT-25: Driver Station to command follows one teleop packet to the
    # first command built from it, in enabled time only. The packet
    # published at 0 ms; lift's tick at 6 ms selected a lane published at
    # 4 ms and sent a request observing (7, 5000). The bridge's command at
    # 6.1 ms still carries (7, 4000): with two owners it merges the older
    # observation, so lift's request reaches a command only once the other
    # owner observes a newer State, at 6.3 ms: 6.3 ms. A tick selecting a
    # lane published before the packet is not built from it. The packet at
    # 20 ms shows the robot disabled and gives no sample; the one at 40 ms
    # is enabled, but the robot is disabled (60 ms) by the command its chain
    # reaches (at 62 ms); the autonomous one at 80 ms reaches a command
    # through the auto lane, which is no operator path.
    def test_driver_station_to_command_follows_one_packet_while_enabled(self):
        ds, tick, command = self.ds_packet, self.selecting_tick, self.merged_command
        records = [ds(0, True), *tick(1, -2, 1, 4000), command(1.2, 4000),
                   *tick(6, 4, 2, 5000), command(6.1, 4000), command(6.3, 5000),
                   ds(20, False), *tick(26, 24, 3, 6000), command(26.2, 6000),
                   ds(40, True), ds(60, False), *tick(61, 45, 4, 7000), command(62, 7000),
                   ds(80, True, "autonomous"), *tick(81, 80.5, 5, 8000, "auto"),
                   command(81.2, 8000)]
        report = session_report.analyze({"nodes": []}, records)
        self.assertEqual(report["driver_station_to_command_ms"],
                         {"count": 1, "min_ms": 6.3, "median_ms": 6.3, "p99_ms": 6.3, "max_ms": 6.3})

    # REQ-RT-25: a tick is built from a packet only if its lane was
    # published before the next packet. Enabled with the sticks neutral the
    # lane is released and Target logs no READ for a second; the first
    # selection, at 1005 ms of a lane published at 1003 ms, follows only the
    # packet at 1000 ms, not the 50 before it.
    def test_driver_station_to_command_takes_no_tick_after_the_next_packet(self):
        records = [self.ds_packet(t, True) for t in range(0, 1001, 20)]
        records += [*self.selecting_tick(1005, 1003, 1, 5000),
                    self.merged_command(1005.25, 5000)]
        self.assertEqual(session_report.analyze({"nodes": []}, records)
                         ["driver_station_to_command_ms"],
                         {"count": 1, "min_ms": 5.25, "median_ms": 5.25, "p99_ms": 5.25, "max_ms": 5.25})

    # REQ-RT-25: a restarted node logs under its name again from dispatch 0,
    # so a READ joins a request only within one run. lift's first run
    # selects and requests at dispatch 2 (21 ms) and runs to dispatch 3; its
    # second, a minute later, starts again at dispatch 1 and reuses 2: each
    # packet follows its own run's request (1.25 ms and 11.25 ms), never the
    # other's.
    def test_driver_station_to_command_joins_within_one_run(self):
        records = [self.ds_packet(20, True), *self.selecting_tick(21, 20.5, 2, 2000),
                   self.merged_command(21.25, 2000),
                   self.chain_record("lift", "TIMER", "update", 30, index=3),
                   self.ds_packet(59990, True),
                   self.chain_record("lift", "TIMER", "update", 59995, index=1),
                   *self.selecting_tick(60001, 60000.5, 2, 9000),
                   self.merged_command(60001.25, 9000)]
        row = session_report.analyze({"nodes": []}, records)["driver_station_to_command_ms"]
        self.assertEqual((row["count"], row["median_ms"], row["max_ms"]), (2, 6.25, 11.25))

    # REQ-OPS-16, REQ-RT-25: a Driver Station to command sample whose span meets
    # a log gap in the bridge's log or in the selecting/requesting node's log is
    # excluded (NOT MEASURED) and counted in intervals_across_gaps, while a gap
    # in an unrelated node's log leaves the sample measured.
    def test_driver_station_to_command_excludes_samples_across_gaps(self):
        def gap(process, first, last):
            return self.chain_record(process, "GAP", "?", last + 1, gap={
                "start_ms": first, "end_ms": last + 1, "first_lost_ms": first,
                "last_lost_ms": last, "records": 1, "cause": "pool_full",
                "closed_at_flush": False})
        # Packet at 0 ms, selecting tick at 6 ms (lane published at 4 ms, in
        # packet 0's window [0, 5)), a fresh enabled packet at 5 ms (so
        # enabled_at(6.3) stays known after a bridge gap at 2 ms), and the
        # merged command at 6.3 ms.
        base = [self.ds_packet(0, True), self.ds_packet(5, True),
                *self.selecting_tick(6, 4, 2, 5000), self.merged_command(6.3, 5000)]
        for label, process in (("bridge", "hardware_node"), ("node", "lift")):
            with self.subTest(label):
                report = session_report.analyze({"nodes": []}, base + [gap(process, 2, 2)])
                self.assertIsNone(report["driver_station_to_command_ms"])
                self.assertEqual(report["intervals_across_gaps"]["driver_station_to_command"], 1)
        unrelated = session_report.analyze({"nodes": []}, base + [gap("camera", 2, 2)])
        self.assertEqual(unrelated["driver_station_to_command_ms"]["count"], 1)
        self.assertEqual(unrelated["intervals_across_gaps"]["driver_station_to_command"], 0)

    # REQ-RT-25: bridge request to command is split as the bridge decides
    # its send: by the State it holds (GatewayReady: configured, enabled, no
    # hardware fault), held over the whole sample. Ready, it sends a changed
    # merge at once; not ready, only on its heartbeat. The Driver Station
    # shows enabled throughout, but the State latches a fault at 50 ms, so
    # 60 -> 80 ms is not-ready time, and a sample across the change
    # (49 -> 51 ms) is in neither.
    def test_request_to_command_is_split_into_enabled_and_disabled_time(self):
        def send(topic, now, observed=None, **extra):
            hardware = None if observed is None else {"boot_id": "7",
                                                      "observed_time_us": observed}
            return dict(process="lift" if topic.startswith("/hw/request/") else "hardware_node",
                        kind="SEND", topic=topic, now_ms=now, event_ms=now, publish_ms=now,
                        decoded="", hardware=hardware, **extra)
        def state(now, fault):
            return dict(send("/hw/state", now), hardware={
                "type": "state", "boot_id": "7", "sample_time_us": now * 1000,
                "configured": True, "enabled": True, "hardware_fault": fault})
        records = [send("/hw/state/driver_station", 0, ds_enabled=True),
                   state(0, False), state(50, True),
                   send("/hw/request/lift", 10, 1), send("/hw/command", 10.25, 1),
                   send("/hw/request/lift", 49, 2), send("/hw/command", 51, 2),
                   send("/hw/request/lift", 60, 3), send("/hw/command", 80, 3)]
        bridge = session_report.analyze({"nodes": []}, records)["bridge_request_to_command"]
        self.assertEqual(bridge, {
            "process": "hardware_node",
            "enabled": {"count": 1, "min_ms": 0.25, "median_ms": 0.25, "p99_ms": 0.25, "max_ms": 0.25},
            "disabled": {"count": 1, "min_ms": 20, "median_ms": 20, "p99_ms": 20, "max_ms": 20}})

    # REQ-RT-25: the clocks' relative drift is the slope of the bridge's
    # valid clock_offset_us over its /hw/status publication times, fitted
    # within one estimator: offsets rising 50 µs per second are 50 ppm,
    # whatever the jump a gateway reboot (a new clock_boot_id) makes, and an
    # invalid offset is ignored.
    def test_clock_drift_is_the_offset_slope_within_one_boot(self):
        def status(publish, boot, offset, valid=True):
            return dict(process="hardware_node", kind="SEND", topic="/hw/status",
                        now_ms=publish, event_ms=publish, publish_ms=publish,
                        decoded=f"{{clock_boot_id: {boot}, clock_offset_us: {offset}, "
                                f"clock_resets_count: {boot}, "
                                f"clock_offset_valid: {'true' if valid else 'false'}}}")
        records = [status(t, 1, -3000 + t * 0.05) for t in range(0, 10_000, 1000)]
        records += [status(t, 2, 900_000 + t * 0.05) for t in range(10_000, 20_000, 1000)]
        records += [status(4500, 1, 77_777, valid=False)]
        records.sort(key=lambda r: r["publish_ms"])
        report = session_report.analyze({"nodes": []}, records)
        self.assertAlmostEqual(report["clock_drift_ppm"], 50)
        self.assertIsNone(session_report.analyze({"nodes": []}, records[:1])["clock_drift_ppm"])

    def test_strict_names_a_damaged_log_and_its_offset(self):
        with tempfile.TemporaryDirectory() as directory:
            shutil.copytree(self._default_fixture_dir, directory, dirs_exist_ok=True)
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
            result = self.run_in_process("--json", "--strict", directory)
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

    # REQ-OPS-37: per alert, its first activation, activation count, and
    # longest and total active time, from the `alerts` log_dump put on the
    # publisher's status records, timed by publication: each record here is
    # received 0 to 9 ms after it was published, and a reader's copies of the
    # topic, received later still, are never pooled with the publisher's.
    def test_alerts_report_activations_and_active_time_by_publication(self):
        def record(process, kind, publish, delay, link_lost, hot):
            return dict(process=process, kind=kind, topic="/watchdog/status",
                        publish_ms=publish, now_ms=publish + delay, event_ms=publish + delay,
                        # An `alert_` field naming no level: log_dump gives
                        # it no `alerts` entry, and the report reads none.
                        decoded="{alert_critical_overheat: true}",
                        alerts=[{"name": "alert_error_link_lost", "level": "error",
                                 "active": link_lost},
                                {"name": "health.alert_warning_hot", "level": "warning",
                                 "active": hot},
                                {"name": "alert_info_homed", "level": "info",
                                 "active": False}])
        # link_lost rises at 20, falls at 40, rises at 55 and is still active
        # at the topic's last record, 60; hot is active from 35 to 40.
        timeline = ((10, 1, False, False), (20, 9, True, False), (35, 0, True, True),
                    (40, 6, False, False), (55, 2, True, False), (60, 3, True, False))
        records = [record("watchdog", "SEND", *row) for row in timeline]
        records += [record("telemetry", "MESSAGE", publish, 50, True, True)
                    for publish, *_ in timeline]
        manifest = {"end_wall_ns": 1, "nodes": []}
        report = session_report.analyze(manifest, records)
        self.assertEqual(report["alerts"], {
            "/watchdog/status/alert_error_link_lost": {
                "level": "error", "stream": "watchdog/SEND", "first_active_ms": 20,
                "activations": 2, "longest_ms": 20, "total_ms": 25, "active_at_end": True},
            "/watchdog/status/health.alert_warning_hot": {
                "level": "warning", "stream": "watchdog/SEND", "first_active_ms": 35,
                "activations": 1, "longest_ms": 5, "total_ms": 5, "active_at_end": False}})
        self.assertIn("| /watchdog/status/alert_error_link_lost | error | 20 | 2 | 20 | 25 | yes |",
                      session_report.markdown(report).splitlines())
        # Informational: an error alert does not change what --strict rejects.
        self.assertEqual(session_report.strict_failures(report),
                         session_report.strict_failures(session_report.analyze(manifest, [])))

    # REQ-OPS-06, REQ-OPS-16: under the launcher a bounded session ends by
    # its stop, which the bridge exits 0 for whether or not the controller
    # ever answered, so the report judges the link from the log: the
    # bridge's /hw/state (it publishes only a State it accepted), else
    # /hw/status (state_age_us 0 and alert_error_link_lost true until the
    # first), else NOT MEASURED.
    def test_controller_link_fails_a_session_that_never_heard_a_state(self):
        def record(process, kind, topic, publish, decoded=""):
            return dict(process=process, kind=kind, topic=topic, publish_ms=publish,
                        now_ms=publish + 1, event_ms=publish + 1, decoded=decoded)
        def status(publish, age_us, link_lost):
            return record("drivetrain", "MESSAGE", "/hw/status", publish,
                          f"{{states_received_count: 0, state_age_us: {age_us}, "
                          f"alert_error_link_lost: {link_lost}}}")
        unheard = [status(t, 0, "true") for t in range(0, 8000, 20)]
        heard_at_120 = [status(t, 0 if t < 120 else 3000, "true" if t < 120 else "false")
                        for t in range(0, 8000, 20)]
        state = [record("hardware_node", "SEND", "/hw/state", t) for t in (118, 123)]
        for label, session, verdict, first in (
                ("no State", unheard, "FAIL", None),
                ("a State", heard_at_120 + state, "PASS", 118),
                ("a State, /hw/state unlogged", heard_at_120, "PASS", 117),
                ("no evidence", [], "NOT MEASURED", None)):
            with self.subTest(label):
                report = session_report.analyze({"end_wall_ns": 1, "nodes": []}, session)
                link = {v["measurement"]: v for v in report["verdicts"]}["controller link"]
                self.assertEqual((link["verdict"], report["first_state_ms"]), (verdict, first))
                self.assertEqual("controller link: " + verdict in
                                 session_report.strict_failures(report), verdict != "PASS")

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

    # REQ-RT-15: only a lane's records count as target traffic, so
    # session_report's copy of the lane grammar must agree with
    # naming::IsTargetLane (talOS/introspection/names.h), the authority, run
    # here through target_lane_probe on the same names.
    def test_target_lanes_agree_with_names_h(self):
        agree = [
            "/lift/target/teleop", "/drivetrain/target/align_2",
            "/a/target/b", "/x1/target/y2", "/hw/target/teleop",
            "/lift/target/" + "s" * 18,  # 31 bytes, kMaxTopicBytes
            "/" + "o" * 21 + "/target/s",  # 31 bytes
            "/lift/target/" + "s" * 19, "/" + "o" * 22 + "/target/s",
            "/Lift/target/teleop", "/lift/target/Teleop",
            "/lift/target/tele__op", "/lift/target/teleop_",
            "/lift/target/_teleop", "/lift/target/2nd", "/lift/target/tele-op",
            "/lift/target/teleop/x", "/lift/target/teleop/", "/lift/target/",
            "/lift/target", "/lift/targets/teleop", "/lift/state/teleop",
            "lift/target/teleop", "//target/teleop", "/lift//target/teleop",
            "/lift/target//teleop", "", "/", "/lift/target/télé",
        ]
        # names.h refuses these for an abbreviation or a direction suffix;
        # session_report.py does not re-check those, since the launcher
        # refuses a graph that uses them.
        relaxed = ["/lift/target/cmd", "/drive/target/teleop",
                   "/lift/target/teleop_out", "/lift/target/align_tx"]
        names = agree + relaxed
        verdicts = subprocess.run([self._probe], input="\n".join(names) + "\n",
                                  capture_output=True, text=True,
                                  check=True).stdout.split()
        self.assertEqual(len(verdicts), len(names))
        authority = dict(zip(names, (v == "1" for v in verdicts)))
        self.assertEqual(sum(authority[t] for t in agree), 7)
        for topic in agree:
            self.assertEqual(session_report.is_target_lane(topic),
                             authority[topic], repr(topic))
        for topic in relaxed:
            self.assertFalse(authority[topic], repr(topic))

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
            ({"end_wall_ns": 1, "nodes": [bridge, studio], **BENCH_BOUNDS}, records, 1),
            # The hardware node recorded no /hw/state: seven required
            # verdicts, the controller link among them.
            ({"end_wall_ns": 1, "nodes": [bridge], **BENCH_BOUNDS}, [], 7),
            ({"end_wall_ns": 0, "nodes": [{**bridge, **flags}, studio], **BENCH_BOUNDS},
             records, 5),
        )
        for manifest, session, failures in cases:
            self.assertEqual(len(session_report.strict_failures(
                session_report.analyze(manifest, session))), failures, manifest)

    # REQ-RT-09: session_report --strict rejects a bridge or control loop that
    # ran without its full realtime grant.
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

    # REQ-OPS-43, REQ-OPS-16, REQ-RT-19, REQ-RT-20 (TODO.md RT-7): a wake a writer lost
    # left a reader asleep until its backstop, a stalled arm left a watcher's
    # publications waiting for timers or the backstop, a request the bridge
    # passed over is one its subsystem never merged, and a wake fallback
    # polled; each rejects the session, naming the node and the count. The
    # registry row's counts as the launcher records them: absent in a
    # manifest that predates them, recorded zeros, and recorded losses.
    def test_delivery_loss_is_reported_and_strict_rejects_a_loss(self):
        def strict(nodes):
            report = session_report.analyze({"end_wall_ns": 1, "nodes": nodes}, [])
            failures = session_report.strict_failures(report)
            section = session_report.markdown(report).split("## Delivery loss\n")[1]
            return ([issue for issue in report["delivery_issues"] if issue in failures],
                    section.split("\n\n")[0].splitlines())
        def node(name, **counters):
            return {"name": name, "log_path": f"/s/{name}.tlog", **counters}
        zeros = dict(wake_failures=0, stalled_arms=0, wake_fallbacks=0,
                     requests_skipped=0, requests_skipped_by_subsystem={})
        for label, nodes, failures, shown in (
                ("not recorded", [node("drivetrain")], [],
                 ["- drivetrain: NOT RECORDED"]),
                ("recorded zeros", [node("drivetrain", **zeros)], [],
                 ["- drivetrain: none"]),
                # A stalled arm alone, as the bridge counts a request topic
                # whose failed arms its idle-pass guard stopped holding.
                ("a stalled arm", [
                    node("drivetrain", **zeros),
                    node("hardware_node", **{**zeros, "stalled_arms": 1})],
                 ["hardware_node: stalled_arms=1"], ["- drivetrain: none",
                                                      "- hardware_node: stalled_arms=1"]),
                ("recorded losses", [
                    node("drivetrain", **{**zeros, "wake_failures": 3, "stalled_arms": 1}),
                    node("intake", **{**zeros, "wake_fallbacks": 1}),
                    node("hardware_node", **{**zeros, "requests_skipped": 2,
                                             "requests_skipped_by_subsystem": {"intake": 2}})],
                 ["drivetrain: wake_failures=3", "drivetrain: stalled_arms=1",
                  "intake: wake_fallbacks=1",
                  "hardware_node: requests_skipped=2 (intake=2)"],
                 ["- drivetrain: wake_failures=3, stalled_arms=1",
                  "- intake: wake_fallbacks=1",
                  "- hardware_node: requests_skipped=2 (intake=2)"])):
            with self.subTest(label):
                self.assertEqual(strict(nodes), (failures, shown))

    # REQ-OPS-10: a line the launcher could not keep leaves the terminal
    # record incomplete, so the report names each output that lost bytes
    # from the launcher's notes in its talos.log, read beside the manifest.
    def test_reports_the_output_talos_log_says_was_dropped(self):
        with tempfile.TemporaryDirectory() as directory:
            manifest = {"launcher_log": "/robot/logs/7/talos.log"}
            self.assertEqual(session_report.launcher_drops(directory, {}), None)
            with open(os.path.join(directory, "talos.log"), "w") as log:
                log.write("[launcher] Starting session 7 with 2 nodes...\n"
                          "[launcher] 812 bytes of output dropped here: the "
                          "launcher's log writer fell behind\n"
                          "[launcher] arm: 4096 bytes of output dropped (marked in "
                          "/robot/logs/7/arm.log)\n"
                          "[launcher] 300 bytes for the launcher's stdout were not "
                          "written: its reader fell behind\n"
                          "[launcher] 9000 bytes for the launcher's stderr were not "
                          "written, at most: the last 512 were in a write that may "
                          "still deliver them; it did not drain within 1000 ms of "
                          "the end\n")
            self.assertEqual(session_report.launcher_drops(directory, manifest),
                             ["812 bytes dropped from talos.log (the launcher's log "
                              "writer fell behind)",
                              "4096 bytes dropped from arm.log",
                              "300 bytes dropped from the launcher's stdout",
                              "up to 9000 bytes dropped from the launcher's stderr"])

    def test_missing_log_dump_is_an_input_error(self):
        with tempfile.TemporaryDirectory() as directory, \
                mock.patch.object(session_report.runfiles, "Create", return_value=None), \
                mock.patch.object(sys, "argv", ["session_report", directory]):
            with open(directory + "/manifest.json", "w") as manifest:
                manifest.write('{"nodes": []}')
            self.assertEqual(session_report.main(), 1)

    # REQ-RT-25 (TODO.md TIME-4): stats() carries min alongside median/p99/max
    # so state-to-request and other latency rows show whether samples swept a
    # full period.
    def test_stats_includes_min_and_markdown_table_shows_it(self):
        self.assertEqual(session_report.stats([10, 2, 6, 4]),
                         {"count": 4, "min_ms": 2, "median_ms": 5.0,
                          "p99_ms": 10, "max_ms": 10})
        self.assertEqual(session_report.stats([300, 100, 200], "us"),
                         {"count": 3, "min_us": 100, "median_us": 200,
                          "p99_us": 300, "max_us": 300})
        self.assertIsNone(session_report.stats([]))
        self.assertEqual(
            session_report.stats_row("row", session_report.stats([10, 2, 6, 4])),
            "| row | 4 | 2 | 5 | 10 | 10 |")
        self.assertEqual(
            session_report.stats_row("row", None),
            "| row | NOT MEASURED | — | — | — | — |")

    # TODO.md RT-28: count missed /hw/state sequences within each gateway boot,
    # reporting total missed states, gap count, and the longest consecutive gap
    # (ignoring boot transitions and intervals across a /hw/state log gap).
    def test_state_sequence_gaps_within_boot_and_excluding_log_gaps(self):
        def state(now_ms, boot_id, sequence, process="hardware_node"):
            return dict(process=process, kind="SEND", topic="/hw/state",
                        now_ms=now_ms, event_ms=now_ms, publish_ms=now_ms,
                        decoded="", hardware={"type": "state",
                                              "boot_id": str(boot_id),
                                              "sequence": sequence})
        def gap(first, last, records):
            return dict(process="hardware_node", kind="GAP", topic="?",
                        now_ms=last + 1, event_ms=first, publish_ms=0,
                        decoded="", gap={
                            "start_ms": first, "end_ms": last + 1,
                            "first_lost_ms": first, "last_lost_ms": last,
                            "records": records, "cause": "pool_full",
                            "closed_at_flush": False})
        # Boot 1: seq 10->11 (0 missed), 11->14 (2 missed at 5 ms),
        # 14->15 (0 missed). Boot 2 restarts at seq 1: boot jump is NOT a gap!
        # Boot 2: seq 1->6 (4 missed at 30 ms), then a log gap hides 6->100
        # (excluded, disk drop not link loss), then 100->101 (0 missed).
        records = [
            state(0, 1, 10),
            state(5, 1, 11),
            state(20, 1, 14),
            state(25, 1, 15),
            state(30, 2, 1),
            state(55, 2, 6),
            gap(60, 100, 8),
            state(110, 2, 100),
            state(115, 2, 101),
        ]
        report = session_report.analyze({"nodes": []}, records)
        self.assertEqual(report["state_sequence_gaps"], {
            "intervals": 5,
            "missed": 6,
            "gaps": 2,
            "longest_run": 4,
            "longest_at_ms": 30,
        })
        self.assertEqual(report["intervals_across_gaps"]["state_sequence"], 1)
        md = session_report.markdown(report)
        self.assertIn(
            "State sequence gaps: 6 missed states across 2 gaps "
            "(longest consecutive gap: 4 states at 30 ms).",
            md.splitlines())

        # Contiguous sequences report 0 missed states.
        clean = [state(0, 1, 1), state(5, 1, 2), state(10, 1, 3)]
        clean_report = session_report.analyze({"nodes": []}, clean)
        self.assertEqual(clean_report["state_sequence_gaps"], {
            "intervals": 2,
            "missed": 0,
            "gaps": 0,
            "longest_run": 0,
            "longest_at_ms": None,
        })

        # Unsequenced/undecoded states report None (NOT MEASURED).
        undecoded = [dict(process="hardware_node", kind="SEND", topic="/hw/state",
                          now_ms=t, event_ms=t, publish_ms=t, decoded="")
                     for t in (0, 5, 10)]
        self.assertIsNone(
            session_report.analyze({"nodes": []}, undecoded)["state_sequence_gaps"])

    # REQ-OPS-20 (TODO.md RT-17, RT-28): BridgeStatus socket/State loss counters
    # are surfaced in both JSON (`bridge_socket_counters`) and markdown output.
    def test_bridge_socket_and_state_loss_counters(self):
        def status(t, send_fail, reopens, seq_gaps, decode_fail, would_block):
            decoded = (
                f"{{states_received_count: 10, "
                f"socket_send_failures_count: {send_fail}, "
                f"socket_reopens_count: {reopens}, "
                f"state_sequence_gaps_count: {seq_gaps}, "
                f"state_decode_failures_count: {decode_fail}, "
                f"socket_send_would_block_count: {would_block}}}"
            )
            return dict(process="hardware_node", kind="SEND", topic="/hw/status",
                        now_ms=t, event_ms=t, publish_ms=t, decoded=decoded)

        # Include a bridge restart mid-session so reset/delta accounting is exercised.
        records = [
            status(0, 1, 0, 2, 0, 0),
            status(20, 4, 1, 5, 1, 2),
            status(40, 0, 0, 1, 0, 0),
            status(60, 2, 1, 3, 2, 1),
        ]
        report = session_report.analyze({"nodes": []}, records)
        counters = report["bridge_socket_counters"]
        self.assertIsNotNone(counters)
        self.assertEqual(set(counters.keys()), set(session_report.BRIDGE_SOCKET_COUNTERS))
        self.assertEqual(counters["socket_send_failures_count"]["delta"], 5)
        self.assertEqual(counters["socket_send_failures_count"]["last"], 2)
        self.assertEqual(counters["socket_send_failures_count"]["resets"], 1)
        self.assertEqual(counters["socket_reopens_count"]["delta"], 2)
        self.assertEqual(counters["state_sequence_gaps_count"]["delta"], 5)
        self.assertEqual(counters["state_decode_failures_count"]["delta"], 3)
        self.assertEqual(counters["socket_send_would_block_count"]["delta"], 3)

        md = session_report.markdown(report)
        self.assertIn(
            "Bridge UDP/State loss counters: "
            "socket_send_failures_count=delta:5/last:2 (resets=1), "
            "socket_reopens_count=delta:2/last:1 (resets=1), "
            "state_sequence_gaps_count=delta:5/last:3 (resets=1), "
            "state_decode_failures_count=delta:3/last:2 (resets=1), "
            "socket_send_would_block_count=delta:3/last:1 (resets=1)",
            md.splitlines(),
        )

        # Older logs without these fields report None and omit the summary line.
        legacy = [dict(process="hardware_node", kind="SEND", topic="/hw/status",
                       now_ms=0, event_ms=0, publish_ms=0,
                       decoded="{states_received_count: 5}")]
        legacy_report = session_report.analyze({"nodes": []}, legacy)
        self.assertIsNone(legacy_report["bridge_socket_counters"])
        self.assertNotIn("Bridge UDP/State loss counters:",
                         session_report.markdown(legacy_report))

    # REQ-OPS-10 (REPO-4): session_report prints `talOS revision:` right after
    # `Source revision:` when manifest.json carries `talos_git_revision`, and
    # omits both JSON key and markdown line when absent.
    def test_reports_talos_revision_after_source_revision_when_present(self):
        plain = session_report.analyze(
            {"session_id": 1, "git_revision": "aaaa1111", "nodes": []}, [])
        self.assertNotIn("talos_git_revision", plain)
        self.assertNotIn("talOS revision:", session_report.markdown(plain))

        for dirty, suffix in ((False, ""), (True, " (dirty)")):
            with self.subTest(dirty=dirty):
                report = session_report.analyze({
                    "session_id": 2,
                    "git_revision": "aaaa1111",
                    "git_dirty": False,
                    "talos_git_revision": "bbbb2222",
                    "talos_git_dirty": dirty,
                    "nodes": [],
                }, [])
                self.assertEqual(report["talos_git_revision"],
                                 "bbbb2222" + suffix)
                lines = session_report.markdown(report).splitlines()
                src_idx = lines.index("Source revision: aaaa1111")
                self.assertEqual(lines[src_idx + 1],
                                 "talOS revision: bbbb2222" + suffix)

    # REQ-RT-15, REQ-OPS-16 (TODO.md SPEC-4): exact verdict boundaries for
    # overall/enabled /hw/state median ([4.0, 6.0] ms), p99 (< 2 x period_us),
    # and enabled state gaps (< command_timeout_us), across bench bounds,
    # tighter-than-bench bounds, and a non-1:10 period/timeout ratio.
    def test_state_cadence_and_gap_verdict_boundaries(self):
        def record(topic, now, **extra):
            return dict(process="hardware_node", kind="SEND", topic=topic,
                        now_ms=now, event_ms=now, publish_ms=now, decoded="",
                        **extra)

        def verdicts_for(times, bounds=BENCH_BOUNDS):
            records = [record("/hw/state/driver_station", 0, ds_enabled=True)]
            records += [record("/hw/state", t) for t in times]
            report = session_report.analyze(
                {"end_wall_ns": 1, "nodes": [], **bounds}, records)
            return report, {v["measurement"]: v["verdict"] for v in report["verdicts"]}

        # Overall and enabled state median: 3.99 FAIL, 4.0/6.0 PASS, 6.01 FAIL.
        for interval, expected in ((3.99, "FAIL"), (4.0, "PASS"),
                                   (6.0, "PASS"), (6.01, "FAIL")):
            with self.subTest(median_ms=interval):
                _, verdicts = verdicts_for([0.0, interval, 2 * interval, 3 * interval])
                self.assertEqual(verdicts["overall state median"], expected)
                self.assertEqual(verdicts["enabled state median"], expected)

        # Overall and enabled state p99: 9.99 PASS, 10.0 FAIL at bench bounds.
        # Put the peak interval first so peak - 0.0 is exact in IEEE 754; nine
        # 5 ms intervals follow (10 total: median = 5 ms, nearest-rank p99 = peak).
        for peak, expected in ((9.99, "PASS"), (10.0, "FAIL")):
            with self.subTest(p99_ms=peak):
                times = [0.0] + [peak + 5.0 * i for i in range(10)]
                report, verdicts = verdicts_for(times)
                self.assertEqual(report["state_p99_bound_ms"], 10.0)
                self.assertEqual(verdicts["overall state median"], "PASS")
                self.assertEqual(verdicts["enabled state median"], "PASS")
                self.assertEqual(verdicts["overall state p99"], expected)
                self.assertEqual(verdicts["enabled state p99"], expected)

        # Enabled state gaps: 99.9 PASS (0 gaps >= 100 ms), 100.0 FAIL (1 gap).
        for gap_ms, expected, count in ((99.9, "PASS", 0), (100.0, "FAIL", 1)):
            with self.subTest(gap_ms=gap_ms):
                times = [0.0, gap_ms, gap_ms + 5.0, gap_ms + 10.0]
                report, verdicts = verdicts_for(times)
                self.assertEqual(report["enabled_gaps"], count)
                self.assertEqual(report["enabled_gap_bound_ms"], 100.0)
                self.assertEqual(verdicts["enabled state gaps"], expected)

        # Tighter-than-bench config (3000 / 60000 -> p99 < 6 ms, gaps < 60 ms):
        # 7 ms p99 and 70 ms gap both FAIL where bench literals 10 / 100 pass.
        tight = {"period_us": 3000, "command_timeout_us": 60000}
        for peak, expected in ((5.99, "PASS"), (6.0, "FAIL"), (7.0, "FAIL")):
            with self.subTest(tight_p99_ms=peak):
                times = [0.0] + [peak + 3.0 * i for i in range(10)]
                report, verdicts = verdicts_for(times, bounds=tight)
                self.assertEqual(report["state_p99_bound_ms"], 6.0)
                self.assertEqual(verdicts["overall state p99"], expected)
                self.assertEqual(verdicts["enabled state p99"], expected)
        for gap_ms, expected, count in ((59.9, "PASS", 0), (60.0, "FAIL", 1),
                                        (70.0, "FAIL", 1)):
            with self.subTest(tight_gap_ms=gap_ms):
                times = [0.0, gap_ms, gap_ms + 3.0, gap_ms + 6.0]
                report, verdicts = verdicts_for(times, bounds=tight)
                self.assertEqual(report["enabled_gaps"], count)
                self.assertEqual(report["enabled_gap_bound_ms"], 60.0)
                self.assertEqual(verdicts["enabled state gaps"], expected)

        # Non-1:10 ratio (10000 / 250000 -> p99 < 20 ms != 250000 / 10000 = 25 ms):
        # 20 ms and 22 ms p99 both FAIL, catching timeout-derived p99 mutants.
        wide = {"period_us": 10000, "command_timeout_us": 250000}
        for peak, expected in ((19.99, "PASS"), (20.0, "FAIL"), (22.0, "FAIL")):
            with self.subTest(wide_p99_ms=peak):
                times = [0.0] + [peak + 10.0 * i for i in range(10)]
                report, verdicts = verdicts_for(times, bounds=wide)
                self.assertEqual(report["state_p99_bound_ms"], 20.0)
                self.assertEqual(verdicts["overall state p99"], expected)
                self.assertEqual(verdicts["enabled state p99"], expected)
        for gap_ms, expected, count in ((249.9, "PASS", 0), (250.0, "FAIL", 1)):
            with self.subTest(wide_gap_ms=gap_ms):
                times = [0.0, gap_ms, gap_ms + 10.0, gap_ms + 20.0]
                report, verdicts = verdicts_for(times, bounds=wide)
                self.assertEqual(report["enabled_gaps"], count)
                self.assertEqual(report["enabled_gap_bound_ms"], 250.0)
                self.assertEqual(verdicts["enabled state gaps"], expected)

    # REQ-OPS-16 (TODO.md SPEC-4): invalid, out-of-uint32-range, or partially
    # absent manifest bounds fail closed per field as NOT MEASURED without
    # raising (e.g. OverflowError on huge ints) or conflating the two keys.
    def test_recorded_bound_validation_and_partial_manifests(self):
        records = [
            dict(process="hardware_node", kind="SEND",
                 topic="/hw/state/driver_station",
                 now_ms=0.0, event_ms=0.0, publish_ms=0.0, decoded="",
                 ds_enabled=True),
            *[dict(process="hardware_node", kind="SEND", topic="/hw/state",
                   now_ms=t, event_ms=t, publish_ms=t, decoded="")
              for t in (0.0, 5.0, 10.0, 15.0)],
        ]

        for bad in (0, -5000, True, False, 5000.0, "5000", None, 2**32, 10**400):
            with self.subTest(bad_value=repr(bad)):
                report = session_report.analyze(
                    {"end_wall_ns": 1, "nodes": [],
                     "period_us": bad, "command_timeout_us": bad}, records)
                verdicts = {v["measurement"]: v["verdict"] for v in report["verdicts"]}
                self.assertEqual(verdicts["overall state p99"], "NOT MEASURED")
                self.assertEqual(verdicts["enabled state p99"], "NOT MEASURED")
                self.assertEqual(verdicts["enabled state gaps"], "NOT MEASURED")
                self.assertIsNone(report["state_p99_bound_ms"])
                self.assertIsNone(report["enabled_gaps"])
                self.assertIsNone(report["enabled_gap_bound_ms"])

        # Largest valid uint32_t is accepted without overflow.
        max_u32 = 0xFFFFFFFF
        report = session_report.analyze(
            {"end_wall_ns": 1, "nodes": [],
             "period_us": max_u32, "command_timeout_us": max_u32}, records)
        verdicts = {v["measurement"]: v["verdict"] for v in report["verdicts"]}
        self.assertEqual(verdicts["overall state p99"], "PASS")
        self.assertEqual(verdicts["enabled state p99"], "PASS")
        self.assertEqual(verdicts["enabled state gaps"], "PASS")
        self.assertAlmostEqual(report["state_p99_bound_ms"], 2 * max_u32 / 1000)
        self.assertEqual(report["enabled_gaps"], 0)
        self.assertAlmostEqual(report["enabled_gap_bound_ms"], max_u32 / 1000)

        # Partial manifest: only period_us present.
        only_period = session_report.analyze(
            {"end_wall_ns": 1, "nodes": [], "period_us": 5000}, records)
        v_period = {v["measurement"]: v for v in only_period["verdicts"]}
        self.assertEqual(v_period["overall state p99"]["verdict"], "PASS")
        self.assertEqual(v_period["enabled state p99"]["verdict"], "PASS")
        self.assertEqual(only_period["state_p99_bound_ms"], 10.0)
        self.assertEqual(v_period["enabled state gaps"]["verdict"], "NOT MEASURED")
        self.assertIn("manifest records no command_timeout_us",
                      v_period["enabled state gaps"]["detail"])
        self.assertIsNone(only_period["enabled_gaps"])
        self.assertIsNone(only_period["enabled_gap_bound_ms"])

        # Partial manifest: only command_timeout_us present.
        only_timeout = session_report.analyze(
            {"end_wall_ns": 1, "nodes": [], "command_timeout_us": 100000}, records)
        v_timeout = {v["measurement"]: v for v in only_timeout["verdicts"]}
        self.assertEqual(v_timeout["overall state p99"]["verdict"], "NOT MEASURED")
        self.assertEqual(v_timeout["enabled state p99"]["verdict"], "NOT MEASURED")
        self.assertIn("manifest records no period_us",
                      v_timeout["overall state p99"]["detail"])
        self.assertIsNone(only_timeout["state_p99_bound_ms"])
        self.assertEqual(v_timeout["enabled state gaps"]["verdict"], "PASS")
        self.assertEqual(only_timeout["enabled_gaps"], 0)
        self.assertEqual(only_timeout["enabled_gap_bound_ms"], 100.0)

    # REQ-OPS-16: hostile manifest.json inputs (non-object roots, deeply nested
    # JSON, NaN/Infinity literals, wrong field types) and non-finite decoded
    # scalars must never raise uncaught tracebacks or emit invalid NaN JSON.
    def test_adversarial_manifests_and_non_finite_scalars(self):
        hostile_cli_manifests = [
            ("list root", "[]"),
            ("null root", "null"),
            ("int root", "123"),
            ("deep nesting", '{"a":' * 1500 + "1" + "}" * 1500),
            ("nan literal", '{"nodes": [], "period_us": NaN}'),
            ("inf literal", '{"nodes": [], "period_us": Infinity}'),
            ("nodes null", '{"nodes": null}'),
            ("nodes non-dict item", '{"nodes": ["not_a_dict"]}'),
        ]
        with tempfile.TemporaryDirectory() as directory:
            session = pathlib.Path(directory)
            for index, (label, text) in enumerate(hostile_cli_manifests):
                with self.subTest(manifest=label):
                    (session / "manifest.json").write_text(text)
                    # Prove the real py_binary CLI exits 1 without a traceback
                    # on the first hostile input; run the rest in-process.
                    if index == 0:
                        result = subprocess.run([self._binary, str(session)],
                                                capture_output=True, text=True)
                    else:
                        result = self.run_in_process(str(session))
                    self.assertEqual(result.returncode, 1)
                    self.assertNotIn("Traceback", result.stderr)
                    self.assertNotIn("no .tlog files", result.stderr)
                    self.assertIn("session_report:", result.stderr)

        # Wrong-typed optional node/session fields in analyze/markdown must not
        # raise TypeError or AttributeError.
        hostile_manifest = {
            "session_id": 1,
            "end_wall_ns": 100,
            "launcher_log": 12345,
            "git_revision": 42,
            "talos_git_revision": 99,
            "nodes": [{
                "name": None,
                "binary_sha256": 12345,
                "requests_skipped": 2,
                "requests_skipped_by_subsystem": "not_a_dict",
                "runs": [{"pid": 1, "start_wall_ns": "bad", "end_wall_ns": 10}],
                "runs_dropped": "bad",
            }],
        }
        report = session_report.analyze(hostile_manifest, [])
        self.assertIsNone(
            session_report.launcher_drops(pathlib.Path("/nonexistent"), hostile_manifest))
        rendered = session_report.markdown(report)
        self.assertIn("# Session report", rendered)
        # Valid strict JSON (no NaN/Infinity).
        json.dumps(report, allow_nan=False)

        # Overflowing float literal (1e500 -> inf) in decoded /hw/status must
        # never poison clock_drift_ppm or JSON serialization with NaN/Infinity.
        status_records = [
            dict(process="hardware_node", kind="SEND", topic="/hw/status",
                 now_ms=0.0, event_ms=0.0, publish_ms=0.0,
                 decoded="{clock_offset_valid: true, clock_boot_id: 1, "
                         "clock_resets_count: 0, clock_offset_us: 1e500}"),
            dict(process="hardware_node", kind="SEND", topic="/hw/status",
                 now_ms=20.0, event_ms=20.0, publish_ms=20.0,
                 decoded="{clock_offset_valid: true, clock_boot_id: 1, "
                         "clock_resets_count: 0, clock_offset_us: 1e500}"),
        ]
        self.assertIsNone(session_report.clock_drift_ppm(status_records, {}))
        overflow_report = session_report.analyze(
            {"end_wall_ns": 1, "nodes": []}, status_records)
        json.dumps(overflow_report, allow_nan=False)

        # Merged sorted spans keep across_gap exact across thousands of gaps.
        many_gaps = [
            {"process": "p", "start_ms": float(i * 10), "end_ms": float(i * 10 + 2),
             "first_lost_ms": float(i * 10), "last_lost_ms": float(i * 10 + 1)}
            for i in range(2000)
        ]
        spans = session_report.lost_spans(many_gaps)
        self.assertTrue(session_report.across_gap(spans, {"p"}, 5000.5, 5000.9))
        self.assertFalse(session_report.across_gap(spans, {"p"}, 5002.0, 5009.0))


if __name__ == "__main__":
    unittest.main()
