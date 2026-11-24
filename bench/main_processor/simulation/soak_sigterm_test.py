"""Regression tests for soak_test.py SIGTERM cleanup (REQ-DEV-25)."""

import json
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import soak_test
import support
from talOS.launcher import tlog_read

# Runs inside a child Python process so a missing SIGTERM handler actually
# terminates the process with -SIGTERM before finally: runs.
_CHILD_RUNNER = r"""
import json, os, signal, subprocess, sys, unittest
from pathlib import Path
from unittest import mock
import soak_test

real_popen = subprocess.Popen
ready_fd = int(os.environ["READY_FD"])
gate_fd = int(os.environ["GATE_FD"])
child_eof_fd = int(os.environ["CHILD_EOF_FD"])
workspace_dir = Path(os.environ["FAKE_WORKSPACE"])

_DUMMY_SESSION = r'''
import os, signal, subprocess, sys
child_eof_fd = int(os.environ["CHILD_EOF_FD"])
child = subprocess.Popen(
    [sys.executable, "-c", "import os; os.write(1, b'c\\n'); os.read(0, 1)"],
    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
    pass_fds=(child_eof_fd,),
)
os.close(child_eof_fd)
assert child.stdout.readline() == b"c\n"
os.kill(child.pid, signal.SIGSTOP)
os.write(1, f"{os.getpid()} {child.pid}\n".encode())
os.read(0, 1)
'''

class InterceptSoak(soak_test.Soak):
    def read_nodes(self, manifest_path):
        os.write(ready_fd, f"{self._session_pids} {manifest_path.parent.parent.parent}\n".encode())
        os.close(ready_fd)
        # Wait until the parent sends SIGTERM (or closes the gate on timeout).
        os.read(gate_fd, 1)
        return {}

_opened_procs = []

def fake_popen(args, **kwargs):
    out_idx = args.index("--output-dir") + 1
    out_dir = Path(args[out_idx]) / "s0"
    out_dir.mkdir(parents=True, exist_ok=True)
    proc = real_popen(
        [sys.executable, "-c", _DUMMY_SESSION],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        pass_fds=(child_eof_fd,),
        start_new_session=kwargs.get("start_new_session", False),
    )
    os.close(child_eof_fd)
    _opened_procs.append(proc)
    pids_line = proc.stdout.readline().decode().strip()
    assert pids_line, f"dummy session exited with {proc.poll()}"
    InterceptSoak._session_pids = pids_line
    (out_dir / "manifest.json").write_text(json.dumps({"nodes": []}))
    return proc

fake_robot = {"command_timeout_us": 200000, "period_us": 20000}
sub_file = workspace_dir / "sub.toml"
sub_file.write_text("[subsystem]\nperiod_us = 5000\n")
fake_subs = {"swerve_module": sub_file, "motor": sub_file, "python_motor": sub_file}
fake_layout = [
    (1, "swerve_module", "drive", "motors"),
    (2, "motor", "motor", "motors"),
    (3, "python_motor", "python_kraken", "motors"),
]

os.environ["TALOS_SOAK_SECONDS"] = "3"
try:
    with mock.patch.object(soak_test.support, "workspace", return_value=workspace_dir), \
         mock.patch.object(soak_test.support, "load_config", return_value=(fake_robot, fake_subs)), \
         mock.patch.object(soak_test.support, "device_layout", return_value=fake_layout), \
         mock.patch.object(soak_test.support, "udp_ports", return_value=(1, 2)), \
         mock.patch.object(soak_test.subprocess, "Popen", side_effect=fake_popen):
        suite = unittest.TestSuite([InterceptSoak("test_graph_survives_kills_and_stalls")])
        result = unittest.TextTestRunner(verbosity=2).run(suite)
        sys.exit(0 if result.wasSuccessful() else 1)
finally:
    for proc in _opened_procs:
        if proc.stdin is not None:
            proc.stdin.close()
        if proc.stdout is not None:
            proc.stdout.close()
"""


def _pipe_closed(fd, timeout_s=5.0):
    """Wait until the sole write-end holder has exited and closed `fd`."""
    readable, _, _ = select.select([fd], [], [], timeout_s)
    return bool(readable) and os.read(fd, 1) == b""


class SoakSigtermCleanupTest(unittest.TestCase):
    # REQ-DEV-25: a SIGTERM delivered to soak_test.py (e.g. Bazel timeout)
    # must unwind into finally: so the launcher's process group (including any
    # SIGSTOPped child) is killed and the bazel-bin symlink is removed.
    def test_sigterm_runs_finally_and_kills_session(self):
        ready_r, ready_w = os.pipe()
        gate_r, gate_w = os.pipe()
        child_eof_r, child_eof_w = os.pipe()
        os.set_inheritable(ready_w, True)
        os.set_inheritable(gate_r, True)
        os.set_inheritable(child_eof_w, True)
        leader_pid = None
        child_pid = None
        with tempfile.TemporaryDirectory(prefix="soak-sigterm-") as work:
            env = dict(
                os.environ,
                PYTHONPATH=os.pathsep.join(sys.path),
                READY_FD=str(ready_w),
                GATE_FD=str(gate_r),
                CHILD_EOF_FD=str(child_eof_w),
                FAKE_WORKSPACE=work,
                TEST_UNDECLARED_OUTPUTS_DIR=work,
            )
            runner = subprocess.Popen(
                [sys.executable, "-c", _CHILD_RUNNER],
                env=env,
                pass_fds=(ready_w, gate_r, child_eof_w),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            os.close(ready_w)
            os.close(gate_r)
            os.close(child_eof_w)
            try:
                with os.fdopen(ready_r, "r") as ready:
                    line = ready.readline().strip()
                if not line:
                    out, _ = runner.communicate(timeout=5)
                    self.fail(f"child never entered soak_test body:\n{out}")
                leader_str, child_str, soak_tmp_str = line.split(" ", 2)
                leader_pid = int(leader_str)
                child_pid = int(child_str)
                soak_tmp = Path(soak_tmp_str)

                # While the dummy session runs, the SIGSTOPped child holds
                # child_eof_w open so child_eof_r is not readable.
                readable, _, _ = select.select([child_eof_r], [], [], 0)
                self.assertEqual(readable, [], "dummy SIGSTOPped child exited early")

                os.kill(runner.pid, signal.SIGTERM)
                output, _ = runner.communicate(timeout=10)

                self.assertNotEqual(
                    runner.returncode,
                    -signal.SIGTERM,
                    f"soak_test died to default SIGTERM without running finally:\n{output}",
                )
                self.assertIn(f"SystemExit: {128 + signal.SIGTERM}", output)
                self.assertTrue(
                    _pipe_closed(child_eof_r, timeout_s=5.0),
                    f"SIGSTOPped child {child_pid} leaked after SIGTERM",
                )
                self.assertFalse(
                    (soak_tmp / "bazel-bin").exists() or (soak_tmp / "bazel-bin").is_symlink(),
                    "bazel-bin symlink left behind after SIGTERM",
                )
            finally:
                os.close(gate_w)
                os.close(child_eof_r)
                if runner.poll() is None:
                    runner.kill()
                    runner.wait(timeout=5)
                if runner.stdout is not None:
                    runner.stdout.close()
                for pid in (child_pid, leader_pid):
                    if pid is not None:
                        try:
                            os.kill(pid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass

    # REQ-DEV-25: a second SIGTERM arriving during finally: cleanup must not
    # abort killpg / bazel-bin unlink / handler restoration.
    def test_second_sigterm_during_cleanup_does_not_abort_finally(self):
        sentinel = lambda _signum, _frame: None  # noqa: E731
        original = signal.signal(signal.SIGTERM, sentinel)
        try:
            with tempfile.TemporaryDirectory(prefix="soak-restore-") as work:
                work_path = Path(work)
                sub_file = work_path / "sub.toml"
                sub_file.write_text("[subsystem]\nperiod_us = 5000\n")
                fake_robot = {"command_timeout_us": 200000, "period_us": 20000}
                fake_subs = {"swerve_module": sub_file, "motor": sub_file, "python_motor": sub_file}
                fake_layout = [
                    (1, "swerve_module", "drive", "motors"),
                    (2, "motor", "motor", "motors"),
                    (3, "python_motor", "python_kraken", "motors"),
                ]
                captured_tmp = []
                killpg_calls = []

                class FakeProc:
                    pid = 424242

                    def poll(self):
                        return None

                    def wait(self, timeout=None):
                        return -signal.SIGKILL

                def fake_popen(args, **_kwargs):
                    out_idx = args.index("--output-dir") + 1
                    captured_tmp.append(Path(args[out_idx]).parent)
                    os.kill(os.getpid(), signal.SIGTERM)
                    raise AssertionError("SIGTERM did not raise SystemExit")

                def fake_killpg(pgid, sig):
                    killpg_calls.append((pgid, sig))
                    # Deliver a second SIGTERM mid-cleanup; must be ignored.
                    os.kill(os.getpid(), signal.SIGTERM)

                with mock.patch.dict(
                    os.environ,
                    {"TALOS_SOAK_SECONDS": "3", "TEST_UNDECLARED_OUTPUTS_DIR": work},
                ), mock.patch.object(
                    soak_test.support, "workspace", return_value=work_path
                ), mock.patch.object(
                    soak_test.support, "load_config", return_value=(fake_robot, fake_subs)
                ), mock.patch.object(
                    soak_test.support, "device_layout", return_value=fake_layout
                ), mock.patch.object(
                    soak_test.support, "udp_ports", return_value=(1, 2)
                ), mock.patch.object(
                    soak_test.subprocess, "Popen", side_effect=fake_popen
                ), mock.patch.object(
                    soak_test.os, "killpg", side_effect=fake_killpg
                ):
                    with self.assertRaises(SystemExit) as ctx:
                        soak_test.Soak("test_graph_survives_kills_and_stalls").test_graph_survives_kills_and_stalls()
                    self.assertEqual(ctx.exception.code, 128 + signal.SIGTERM)

                # Also exercise a second SIGTERM during killpg on an active proc.
                killpg_calls.clear()
                captured_tmp.clear()

                def fake_popen_returns(args, **_kwargs):
                    out_idx = args.index("--output-dir") + 1
                    captured_tmp.append(Path(args[out_idx]).parent)
                    return FakeProc()

                def fail_after_spawn(*_args, **_kwargs):
                    os.kill(os.getpid(), signal.SIGTERM)
                    raise AssertionError("SIGTERM did not raise SystemExit")

                with mock.patch.dict(
                    os.environ,
                    {"TALOS_SOAK_SECONDS": "3", "TEST_UNDECLARED_OUTPUTS_DIR": work},
                ), mock.patch.object(
                    soak_test.support, "workspace", return_value=work_path
                ), mock.patch.object(
                    soak_test.support, "load_config", return_value=(fake_robot, fake_subs)
                ), mock.patch.object(
                    soak_test.support, "device_layout", return_value=fake_layout
                ), mock.patch.object(
                    soak_test.support, "udp_ports", return_value=(1, 2)
                ), mock.patch.object(
                    soak_test.subprocess, "Popen", side_effect=fake_popen_returns
                ), mock.patch.object(
                    soak_test.os, "killpg", side_effect=fake_killpg
                ), mock.patch.object(
                    soak_test.time, "monotonic", side_effect=fail_after_spawn
                ):
                    with self.assertRaises(SystemExit) as ctx:
                        soak_test.Soak("test_graph_survives_kills_and_stalls").test_graph_survives_kills_and_stalls()
                    self.assertEqual(ctx.exception.code, 128 + signal.SIGTERM)

                self.assertEqual(killpg_calls, [(FakeProc.pid, signal.SIGKILL)])
                self.assertTrue(captured_tmp)
                self.assertFalse(
                    (captured_tmp[0] / "bazel-bin").exists() or (captured_tmp[0] / "bazel-bin").is_symlink(),
                    "second SIGTERM aborted bazel-bin unlink during cleanup",
                )
                self.assertIs(
                    signal.getsignal(signal.SIGTERM),
                    sentinel,
                    "previous SIGTERM handler was not restored after SIGTERM",
                )
        finally:
            signal.signal(signal.SIGTERM, original)


def _tlog_header(process=b"node"):
    return tlog_read.FILE.pack(
        b"TALOSLOG",
        tlog_read.VERSIONS[-1],
        tlog_read.FILE.size,
        0,
        1000,
        0,
        0,
        0,
        process.ljust(64, b"\0"),
        0,
    )


def _tlog_record(event_ns, payload=b"", stored=None):
    if stored is None:
        stored = len(payload)
    return (
        tlog_read.RECORD.pack(
            tlog_read.RECORD_MAGIC,
            tlog_read.TIMER,
            1,
            0,
            event_ns,
            event_ns,
            0,
            0,
            0,
            0,
            0,
            stored,
        )
        + payload
    )


class LogBaselineNsTest(unittest.TestCase):
    # REQ-OPS-12, REQ-OPS-15, REQ-SAFE-31: SIGKILL mid-write can leave a
    # truncated header or truncated trailing record; log_baseline_ns skips
    # incomplete files/tails like log_dump and returns the earliest intact
    # record timestamp.
    def test_skips_truncated_header_and_truncated_tail_records(self):
        with tempfile.TemporaryDirectory(prefix="tlog-baseline-") as work:
            root = Path(work)
            # Incomplete file header (SIGKILL mid-header write).
            (root / "truncated_header.tlog").write_bytes(_tlog_header()[:12])
            # Valid header + two intact records + truncated payload tail whose
            # timestamp (1000) must not be counted.
            (root / "truncated_payload.tlog").write_bytes(
                _tlog_header(b"motor")
                + _tlog_record(5000, b"abcd")
                + _tlog_record(2000, b"xy")
                + _tlog_record(1000, b"123", stored=64)
            )
            # Valid header + intact record + incomplete record header tail.
            (root / "truncated_rec_hdr.tlog").write_bytes(
                _tlog_header(b"gyro")
                + _tlog_record(3000)
                + _tlog_record(500)[:7]
            )
            self.assertEqual(support.log_baseline_ns(root), 2000)

    # REQ-OPS-12, REQ-OPS-15, REQ-SAFE-31: a session containing only an
    # incomplete-header .tlog returns None instead of raising ValueError.
    def test_only_truncated_header_returns_none(self):
        with tempfile.TemporaryDirectory(prefix="tlog-baseline-empty-") as work:
            root = Path(work)
            (root / "incomplete.tlog").write_bytes(b"TALOS")
            self.assertIsNone(support.log_baseline_ns(root))


if __name__ == "__main__":
    unittest.main()
