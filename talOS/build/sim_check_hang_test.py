"""integration: a launch talos_sim_test's check stops as a hang leaves no
process of its session behind (REQ-DEV-30).

The stand-in launch ignores SIGINT and SIGTERM, as a hung launcher may,
and has two children that ignore them too: one in its process group, one
leading a group of its own, as every launcher member does. The check gets
the stand-in's pids from it once all three run, checks that its listing of
the session finds exactly those, stops the launch with no wait for SIGINT
or SIGTERM to work, and then finds no process of the session. Killing only
the launch's own process (what `subprocess.run(timeout=...)` does) leaves
both children running.

RunningInSessionTest is a unit test of the /proc stat reading that listing
uses, and FailureArtifactsTest verifies that check() preserves launch.log and
the session directory in TEST_UNDECLARED_OUTPUTS_DIR on failure without
writing there on pass (docs/TESTING.md §8.2, REQ-DEV-25).
"""

import json
import os
import select
import shlex
import signal
import sys
import tempfile
import unittest
from unittest import mock

from talOS.build import sim_check

# Bounds the stand-in's start; only a hang reaches it (docs/TESTING.md §6.3).
READY_TIMEOUT_S = 10

STAND_IN = r"""
import os, signal, sys
signal.signal(signal.SIGINT, signal.SIG_IGN)
signal.signal(signal.SIGTERM, signal.SIG_IGN)
pids = [os.getpid()]
for own_group in (False, True):
    pid = os.fork()
    if pid == 0:
        if own_group:
            os.setpgid(0, 0)
        while True:
            signal.pause()
    pids.append(pid)
os.write(int(sys.argv[1]), (" ".join(map(str, pids)) + "\n").encode())
while True:
    signal.pause()
"""


def kill_all(pids):
    for pid in pids:
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass


class SimCheckHangTest(unittest.TestCase):
    # REQ-DEV-30
    def test_hung_launch_leaves_no_process(self):
        read_fd, write_fd = os.pipe()
        self.addCleanup(os.close, read_fd)
        with tempfile.TemporaryFile() as output:
            launch = sim_check.Launch(
                [sys.executable, "-c", STAND_IN, str(write_fd)], output,
                pass_fds=(write_fd,))
            os.close(write_fd)
            self.addCleanup(
                lambda: kill_all(sim_check.session_processes(launch.sid)))
            ready, _, _ = select.select([read_fd], [], [], READY_TIMEOUT_S)
            self.assertTrue(ready, f"no pids from the stand-in after "
                            f"{READY_TIMEOUT_S} s")
            pids = {int(pid) for pid in os.read(read_fd, 4096).split()}
            self.assertEqual(len(pids), 3, pids)
            self.assertEqual(sim_check.session_processes(launch.sid), pids)
            in_order, _ = launch.finish(stop_wait_s=0, exit_wait_s=0)
        self.assertFalse(in_order)
        self.assertEqual(sim_check.session_processes(launch.sid), set(),
                         f"left running of {sorted(pids)}")

    # REQ-DEV-30
    def test_a_check_that_raises_leaves_no_process(self):
        # The launch has no duration of its own that would end it: whatever
        # goes wrong while the check waits, leaving the `with` stops it.
        read_fd, write_fd = os.pipe()
        self.addCleanup(os.close, read_fd)
        with tempfile.TemporaryFile() as output:
            with self.assertRaisesRegex(RuntimeError, "the wait broke"):
                with sim_check.Launch(
                        [sys.executable, "-c", STAND_IN, str(write_fd)],
                        output, pass_fds=(write_fd,)) as launch:
                    os.close(write_fd)
                    self.addCleanup(lambda: kill_all(
                        sim_check.session_processes(launch.sid)))
                    ready, _, _ = select.select([read_fd], [], [],
                                                READY_TIMEOUT_S)
                    self.assertTrue(ready, "no pids from the stand-in")
                    pids = {int(p) for p in os.read(read_fd, 4096).split()}
                    self.assertEqual(
                        sim_check.session_processes(launch.sid), pids)
                    # The stand-in ignores SIGINT and SIGTERM: do not wait
                    # the real bounds for them.
                    launch.finish = lambda finish=launch.finish: finish(
                        stop_wait_s=0, exit_wait_s=0)
                    raise RuntimeError("the wait broke")
        self.assertEqual(sim_check.session_processes(launch.sid), set(),
                         f"left running of {sorted(pids)}")


def stat_line(state, sid, comm=b"python3"):
    """A /proc/<pid>/stat line of pid 19 in session `sid` (proc_pid_stat(5):
    pid (comm) state ppid pgrp session tty_nr ...)."""
    return (b"19 (" + comm + b") " + state + b" 1 18 " + str(sid).encode() +
            b" 0 -1 4194560 120 0 0 0 0 0 20 0 1 0 1234 0 0\n")


class RunningInSessionTest(unittest.TestCase):
    # REQ-DEV-30: a process the reaper has taken but the kernel has not yet
    # removed reads `X` (EXIT_DEAD); counting it left a reaped orphan
    # "left running" in the sandbox's pid namespace.
    def test_an_exited_process_is_not_running(self):
        for state in (b"Z", b"X"):
            with self.subTest(state=state):
                self.assertFalse(
                    sim_check.running_in_session(stat_line(state, 18), 18))

    # REQ-DEV-30
    def test_a_live_process_of_the_session_is_running(self):
        for state in (b"R", b"S", b"D", b"T", b"t", b"I"):
            with self.subTest(state=state):
                self.assertTrue(
                    sim_check.running_in_session(stat_line(state, 18), 18))
        self.assertFalse(
            sim_check.running_in_session(stat_line(b"R", 17), 18))

    # REQ-DEV-30: comm is the executable's name, which may hold anything.
    def test_a_comm_with_spaces_and_parentheses_parses(self):
        stat = stat_line(b"S", 18, comm=b"a) b (c) 7 8 9")
        self.assertTrue(sim_check.running_in_session(stat, 18))
        self.assertFalse(sim_check.running_in_session(stat, 9))

    # REQ-DEV-30: empty or truncated /proc/<pid>/stat bytes from a concurrently
    # exiting process return False instead of raising and aborting SIGKILL
    # cleanup in Launch.finish().
    def test_truncated_or_malformed_stat_returns_false(self):
        for raw in (
                b"",
                b"19 (py",
                b"19 (py)",
                b"19 (py) ",
                b"19 (py) S 1 18",
                b"19 (py) S 1 18 not_a_sid",
                b"19 (py)S 1 18 18",
        ):
            with self.subTest(raw=raw):
                self.assertFalse(sim_check.running_in_session(raw, 18))


FAILING_SIM_STUB = """import json, os, sys
out = sys.argv[sys.argv.index("--output-dir") + 1]
session = os.path.join(out, "2026-fail-session")
os.makedirs(session, exist_ok=True)
with open(os.path.join(session, "manifest.json"), "w", encoding="utf-8") as f:
    json.dump({"stopped_before_start": True, "nodes": []}, f)
with open(os.path.join(session, "graph.json"), "w", encoding="utf-8") as f:
    json.dump({"nodes": []}, f)
with open(os.path.join(session, "talos.log"), "w", encoding="utf-8") as f:
    f.write("talos log line\\n")
with open(os.path.join(session, "hardware_node.log"), "w", encoding="utf-8") as f:
    f.write("bridge log line\\n")
with open(os.path.join(session, "telemetry.tlog"), "wb") as f:
    f.write(b"\\x00TLOG\\x01\\x02\\xff")
print("stub-launcher: failing on purpose", flush=True)
sys.exit(1)
"""

PASSING_SIM_STUB = """import json, os, signal, sys
out = sys.argv[sys.argv.index("--output-dir") + 1]
session = os.path.join(out, "2026-pass-session")
manifest_path = os.path.join(session, "manifest.json")

def write_manifest(ended):
    nodes = []
    for name in ("hardware_node", "sim_gateway", "telemetry"):
        run = {"pid": os.getpid(), "start_wall_ns": 1, "exit_code": 0,
               "exit_signal": 0, "reason": "stop"}
        if ended:
            run["end_wall_ns"] = 2
        nodes.append({"name": name, "realtime": {"granted": True},
                      "runs_dropped": 0, "runs": [run]})
    tmp = manifest_path + (".end.tmp" if ended else ".start.tmp")
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump({"stopped_before_start": False, "nodes": nodes}, f)
    os.replace(tmp, manifest_path)

def on_sigint(*_):
    write_manifest(ended=True)
    sys.exit(0)

signal.signal(signal.SIGINT, on_sigint)
os.makedirs(session, exist_ok=True)
with open(os.path.join(session, "hardware_node.log"), "w", encoding="utf-8") as f:
    f.write("hardware_node: gateway boot 1\\n")
write_manifest(ended=False)
print("stub-launcher: ready", flush=True)
while True:
    signal.pause()
"""

SESSION_REPORT_STUB = """def stream(records, topic):
    return None, []

def controller_link(states, state_source, statuses, status_source):
    return True, True, None, "ok"
"""

NOOP_SCRIPT = "import sys\nsys.exit(0)\n"


def _write_executable(directory, name, body):
    # Wrap via #!/bin/sh instead of #!{sys.executable} so long Bazel
    # output-base paths do not exceed Linux's 255-byte shebang limit.
    path = os.path.join(directory, name)
    with open(path, "w", encoding="utf-8") as f:
        f.write(
            "#!/bin/sh\n"
            f'exec {shlex.quote(sys.executable)} -c {shlex.quote(body)} "$@"\n')
    os.chmod(path, 0o755)
    return path


class FailureArtifactsTest(unittest.TestCase):
    def setUp(self):
        self._work = tempfile.TemporaryDirectory(
            dir=os.environ.get("TEST_TMPDIR"))
        self.addCleanup(self._work.cleanup)
        self.noop = _write_executable(self._work.name, "noop.py", NOOP_SCRIPT)
        self.report = os.path.join(self._work.name, "session_report_stub.py")
        with open(self.report, "w", encoding="utf-8") as f:
            f.write(SESSION_REPORT_STUB)
        self.failing_sim = _write_executable(
            self._work.name, "failing_sim.py", FAILING_SIM_STUB)
        self.passing_sim = _write_executable(
            self._work.name, "passing_sim.py", PASSING_SIM_STUB)

    # REQ-DEV-25, REQ-DEV-30
    def test_check_preserves_launch_log_and_session_dir_on_failure(self):
        outputs_dir = os.path.join(self._work.name, "undeclared_outputs")
        os.makedirs(outputs_dir)
        with mock.patch.dict(os.environ,
                             {"TEST_UNDECLARED_OUTPUTS_DIR": outputs_dir}):
            problems, diagnostics = sim_check.check(
                self.failing_sim, 0, self.noop, self.report, self.noop,
                ready_s=READY_TIMEOUT_S)
            self.assertTrue(problems, diagnostics)
            first_dir = os.path.join(outputs_dir, "sim_check_failure")
            self.assertTrue(os.path.isdir(first_dir), os.listdir(outputs_dir))
            self.assertIn(first_dir, diagnostics)
            with open(os.path.join(first_dir, "launch.log"),
                      encoding="utf-8") as f:
                self.assertIn("stub-launcher: failing on purpose", f.read())
            session = os.path.join(first_dir, "sessions", "2026-fail-session")
            for filename in ("manifest.json", "graph.json", "talos.log",
                             "hardware_node.log"):
                self.assertTrue(
                    os.path.isfile(os.path.join(session, filename)),
                    f"missing {filename} in {session}")
            with open(os.path.join(session, "telemetry.tlog"), "rb") as f:
                self.assertEqual(f.read(), b"\x00TLOG\x01\x02\xff")

            # A second failure in the same test run preserves into a distinct
            # subdirectory without clobbering the first.
            problems2, _ = sim_check.check(
                self.failing_sim, 0, self.noop, self.report, self.noop,
                ready_s=READY_TIMEOUT_S)
            self.assertTrue(problems2)
            second_dir = os.path.join(outputs_dir, "sim_check_failure_2")
            self.assertTrue(os.path.isdir(second_dir), os.listdir(outputs_dir))
            self.assertTrue(os.path.isfile(
                os.path.join(second_dir, "launch.log")))

    # REQ-DEV-25, REQ-DEV-30
    def test_check_skips_undeclared_outputs_on_pass(self):
        outputs_dir = os.path.join(self._work.name, "undeclared_outputs_pass")
        os.makedirs(outputs_dir)
        with mock.patch.dict(os.environ,
                             {"TEST_UNDECLARED_OUTPUTS_DIR": outputs_dir}):
            problems, diagnostics = sim_check.check(
                self.passing_sim, 0, self.noop, self.report, self.noop,
                ready_s=READY_TIMEOUT_S)
        self.assertEqual(problems, [], diagnostics)
        self.assertEqual(os.listdir(outputs_dir), [])

    # REQ-DEV-25
    def test_unwritable_outputs_dir_does_not_mask_check_failure(self):
        blocker = os.path.join(self._work.name, "not_a_directory")
        with open(blocker, "w", encoding="utf-8") as f:
            f.write("file blocking mkdir")
        with mock.patch.dict(os.environ,
                             {"TEST_UNDECLARED_OUTPUTS_DIR": blocker}):
            problems, diagnostics = sim_check.check(
                self.failing_sim, 0, self.noop, self.report, self.noop,
                ready_s=READY_TIMEOUT_S)
        self.assertTrue(problems, diagnostics)
        self.assertIn("stub-launcher: failing on purpose", diagnostics)


if __name__ == "__main__":
    unittest.main()
