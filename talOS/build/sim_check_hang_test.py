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
"""

import os
import select
import signal
import sys
import tempfile
import unittest

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


if __name__ == "__main__":
    unittest.main()
