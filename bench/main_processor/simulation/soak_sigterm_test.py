"""Regression tests for soak_test.py SIGTERM cleanup (REQ-DEV-25)."""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

import soak_test

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
workspace_dir = Path(os.environ["FAKE_WORKSPACE"])

_DUMMY_SESSION = r'''
import os, signal, subprocess, sys
child = subprocess.Popen(
    [sys.executable, "-c", "import os; os.write(1, b'c\\n'); os.read(0, 1)"],
    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
)
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

def fake_popen(args, **kwargs):
    out_idx = args.index("--output-dir") + 1
    out_dir = Path(args[out_idx]) / "s0"
    out_dir.mkdir(parents=True, exist_ok=True)
    proc = real_popen(
        [sys.executable, "-c", _DUMMY_SESSION],
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        start_new_session=kwargs.get("start_new_session", False),
    )
    pids_line = proc.stdout.readline().decode().strip()
    assert pids_line, f"dummy session exited with {proc.poll()}"
    InterceptSoak._session_pids = pids_line
    (out_dir / "manifest.json").write_text(json.dumps({"nodes": []}))
    return proc

fake_robot = {"command_timeout_us": 200000, "period_us": 20000}
sub_file = workspace_dir / "sub.toml"
sub_file.write_text("[subsystem]\nperiod_us = 5000\n")
fake_subs = {"swerve_module": sub_file, "motor": sub_file}
fake_layout = [
    (1, "swerve_module", "drive", "motors"),
    (2, "motor", "motor", "motors"),
]

os.environ["TALOS_SOAK_SECONDS"] = "3"
with mock.patch.object(soak_test.support, "workspace", return_value=workspace_dir), \
     mock.patch.object(soak_test.support, "load_config", return_value=(fake_robot, fake_subs)), \
     mock.patch.object(soak_test.support, "device_layout", return_value=fake_layout), \
     mock.patch.object(soak_test.support, "udp_ports", return_value=(1, 2)), \
     mock.patch.object(soak_test.subprocess, "Popen", side_effect=fake_popen):
    suite = unittest.TestSuite([InterceptSoak("test_graph_survives_kills_and_stalls")])
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
"""


def _pid_alive(pid):
    try:
        os.kill(pid, 0)
    except ProcessLookupError:
        return False
    stat = Path(f"/proc/{pid}/stat")
    try:
        # Format: "<pid> (<comm>) <state> ..."; comm may contain spaces/parens.
        state = stat.read_text().rsplit(")", 1)[-1].split()[0]
    except FileNotFoundError:
        return not Path("/proc").is_dir()
    return state != "Z"


class SoakSigtermCleanupTest(unittest.TestCase):
    # REQ-DEV-25: a SIGTERM delivered to soak_test.py (e.g. Bazel timeout)
    # must unwind into finally: so the launcher's process group (including any
    # SIGSTOPped child) is killed and the bazel-bin symlink is removed.
    def test_sigterm_runs_finally_and_kills_session(self):
        ready_r, ready_w = os.pipe()
        gate_r, gate_w = os.pipe()
        os.set_inheritable(ready_w, True)
        os.set_inheritable(gate_r, True)
        leader_pid = None
        child_pid = None
        with tempfile.TemporaryDirectory(prefix="soak-sigterm-") as work:
            env = dict(
                os.environ,
                PYTHONPATH=os.pathsep.join(sys.path),
                READY_FD=str(ready_w),
                GATE_FD=str(gate_r),
                FAKE_WORKSPACE=work,
                TEST_UNDECLARED_OUTPUTS_DIR=work,
            )
            runner = subprocess.Popen(
                [sys.executable, "-c", _CHILD_RUNNER],
                env=env,
                pass_fds=(ready_w, gate_r),
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
            )
            os.close(ready_w)
            os.close(gate_r)
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

                self.assertTrue(_pid_alive(leader_pid), "dummy session leader not alive")
                self.assertTrue(_pid_alive(child_pid), "dummy SIGSTOPped child not alive")

                os.kill(runner.pid, signal.SIGTERM)
                output, _ = runner.communicate(timeout=10)

                self.assertNotEqual(
                    runner.returncode,
                    -signal.SIGTERM,
                    f"soak_test died to default SIGTERM without running finally:\n{output}",
                )
                self.assertIn(f"SystemExit: {128 + signal.SIGTERM}", output)
                self.assertFalse(
                    _pid_alive(leader_pid),
                    f"launcher session leader {leader_pid} leaked after SIGTERM",
                )
                self.assertFalse(
                    _pid_alive(child_pid),
                    f"SIGSTOPped child {child_pid} leaked after SIGTERM",
                )
                self.assertFalse(
                    (soak_tmp / "bazel-bin").exists() or (soak_tmp / "bazel-bin").is_symlink(),
                    "bazel-bin symlink left behind after SIGTERM",
                )
            finally:
                os.close(gate_w)
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

    # REQ-DEV-25: soak_test saves and restores the previous SIGTERM handler in
    # finally:, both on normal exceptions and on SIGTERM itself.
    def test_restores_previous_sigterm_handler(self):
        sentinel = lambda _signum, _frame: None  # noqa: E731
        original = signal.signal(signal.SIGTERM, sentinel)
        try:
            with tempfile.TemporaryDirectory(prefix="soak-restore-") as work:
                work_path = Path(work)
                sub_file = work_path / "sub.toml"
                sub_file.write_text("[subsystem]\nperiod_us = 5000\n")
                fake_robot = {"command_timeout_us": 200000, "period_us": 20000}
                fake_subs = {"swerve_module": sub_file, "motor": sub_file}
                fake_layout = [
                    (1, "swerve_module", "drive", "motors"),
                    (2, "motor", "motor", "motors"),
                ]

                def trigger_sigterm(*_args, **_kwargs):
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
                    soak_test.subprocess, "Popen", side_effect=trigger_sigterm
                ):
                    with self.assertRaises(SystemExit) as ctx:
                        soak_test.Soak("test_graph_survives_kills_and_stalls").test_graph_survives_kills_and_stalls()
                    self.assertEqual(ctx.exception.code, 128 + signal.SIGTERM)

                self.assertIs(
                    signal.getsignal(signal.SIGTERM),
                    sentinel,
                    "previous SIGTERM handler was not restored after SIGTERM",
                )
        finally:
            signal.signal(signal.SIGTERM, original)


if __name__ == "__main__":
    unittest.main()
