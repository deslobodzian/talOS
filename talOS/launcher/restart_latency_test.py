import io
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest
from unittest import mock

from python.runfiles import runfiles
from talOS.launcher import restart_latency
from talOS.launcher import tlog_read


class RestartStagesTest(unittest.TestCase):
    def test_stages_of_a_recorded_restart(self):
        # The fixture's real LogWriter output: run() is the tick deadline
        # less one period, and the request counts from the first tick even
        # when that tick sends nothing.
        fixture = runfiles.Create().Rlocation("_main/talOS/launcher/restart_latency_fixture")
        with tempfile.TemporaryDirectory() as directory:
            path = f"{directory}/drivetrain.restart1.tlog"
            subprocess.run([fixture, path], check=True)
            log = restart_latency.read_tlog(path)
        stages = restart_latency.restart_stages(log, 996_000_000, "drivetrain")
        expected = {"kill_to_origin": 4.0, "origin_to_run": 1.5, "run_to_first_tick": 5.1,
                    "first_tick_to_request": 5.0, "total": 15.6}
        self.assertEqual(stages.keys(), expected.keys())
        for key, value in expected.items():
            self.assertAlmostEqual(stages[key], value, places=6, msg=key)
        with self.assertRaisesRegex(ValueError, "no /hw/request/intake send"):
            log["sources"][0]["name"] = "intake"
            restart_latency.restart_stages(log, 0, "intake")

    # REQ-OPS-12, REQ-OPS-15: tlog_read must reject hostile headers (schema_bytes
    # >= 2**63 causing OverflowError, header_bytes < FILE.size, truncated record
    # payloads) with ValueError rather than OverflowError or yielding partial data.
    def test_tlog_read_rejects_hostile_headers_and_truncated_payloads(self):
        with tempfile.TemporaryDirectory() as directory:
            # schema_bytes = 2**63 previously raised OverflowError in seek().
            huge_schema = Path(directory) / "huge_schema.tlog"
            huge_schema.write_bytes(tlog_read.FILE.pack(
                b"TALOSLOG", 8, tlog_read.FILE.size, 1, 0, 0, 0, 0,
                b"node\0".ljust(64, b"\0"), 1 << 63))
            with self.assertRaises(ValueError):
                tlog_read.read_tlog(huge_schema)

            # header_bytes smaller than FILE.size previously sought backwards.
            bad_header_len = Path(directory) / "bad_header_len.tlog"
            bad_header_len.write_bytes(tlog_read.FILE.pack(
                b"TALOSLOG", 8, 8, 1, 0, 0, 0, 0,
                b"node\0".ljust(64, b"\0"), 0))
            with self.assertRaises(ValueError):
                tlog_read.read_tlog(bad_header_len)

            # Truncated payload: must not be yielded, and allow_truncated=False
            # must raise ValueError instead of silently seeking past EOF.
            rec = tlog_read.RECORD.pack(
                tlog_read.RECORD_MAGIC, tlog_read.TIMER, 1, 1, 100, 100, 100,
                1, 0, 64, 0, 64)
            truncated_payload = rec + b"\x00" * 10
            self.assertEqual(
                list(tlog_read.iter_tlog_records(
                    io.BytesIO(truncated_payload), "trunc.tlog",
                    allow_truncated=True)),
                [])
            with self.assertRaises(ValueError):
                list(tlog_read.iter_tlog_records(
                    io.BytesIO(truncated_payload), "trunc.tlog",
                    allow_truncated=False))

    # REQ-SAFE-30: Session.kill must never pass pid <= 0 (e.g. manifest default
    # -1) to os.kill, and remove_segments must survive corrupt/deep graph.json.
    def test_session_kill_refuses_nonpositive_pid_and_cleanup_survives_bad_graph(self):
        session = object.__new__(restart_latency.Session)
        for bad_pid in (-1, 0, None):
            with self.subTest(pid=bad_pid):
                session.nodes = lambda p=bad_pid: {"alpha": {"pid": p, "restart_count": 0}}
                with mock.patch.object(restart_latency.os, "kill") as fake_kill:
                    with self.assertRaises(RuntimeError):
                        session.kill("alpha")
                    fake_kill.assert_not_called()

        with tempfile.TemporaryDirectory() as directory:
            d = Path(directory)
            (d / "graph.json").write_text("{corrupt")
            restart_latency.remove_segments(d, "ns-test-corrupt")
            (d / "graph.json").write_text("[" * 1500 + "]" * 1500)
            restart_latency.remove_segments(d, "ns-test-deep")


if __name__ == "__main__":
    unittest.main()
