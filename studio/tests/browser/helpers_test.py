"""The browser test's two helpers, without a browser or a robot: children.py
(a child's process group is signalled only while the child is unreaped) and
watch.py (a wait on files that polls nothing). They keep
//studio:drive_browser_test (REQ-OPS-36) from failing, or hanging, for a
reason that is not the dashboard's; this target is not e2e, so the macOS
lane runs the kqueue paths too.
"""

import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).parent))
import children  # noqa: E402
import watch  # noqa: E402


# REQ-OPS-36
class ChildSignalsItsGroupOnlyUnreaped(unittest.TestCase):
    """The cleanup above may run twice (tearDownClass, then atexit or a
    SIGTERM): a group whose leader was reaped is never signalled again,
    since its pid may by then lead an unrelated group."""

    def test_no_signal_after_reap(self):
        sent = []

        def killpg(pid, sig):
            sent.append((pid, sig))
            os.killpg(pid, sig)

        child = children.Child([sys.executable, "-c", "import sys; sys.stdin.read()"],
                               killpg=killpg, stdin=subprocess.PIPE)
        self.addCleanup(child.proc.stdin.close)
        self.assertFalse(child.exited())
        # A child that does not exit ends the wait at its timeout: a lost
        # SIGKILL must fail below in seconds, not hang until Bazel's timeout.
        self.assertFalse(child.wait_exited(0.05), "a live child read as exited")
        self.assertTrue(child.signal_group(signal.SIGKILL))
        # exited() sees the exit and leaves it unreaped: reap() still reads
        # the SIGKILL status, which a reaped pid would have lost.
        self.assertTrue(child.wait_exited(10), "the killed child never read as exited")
        self.assertTrue(child.exited())
        self.assertEqual(child.reap(), -signal.SIGKILL)
        self.assertFalse(child.signal_group(signal.SIGKILL))
        child.reap()
        self.assertEqual(sent, [(child.pid, signal.SIGKILL)])


class WatchSeesAFileCreatedBeforeItsDirectoryIsWatched(unittest.TestCase):
    """wait_for lists the paths, then watches them: a file created between
    the two is in no list and raised no event on its directory, so the write
    that satisfies the check would wake nothing (a session's
    studio_bridge.log, created while the session directory was being
    watched, timed the startup out)."""

    def test_a_write_to_a_file_created_in_that_window_wakes_the_wait(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = Path(tmp) / "member.log"
            listed = []
            checked = []

            def paths():
                found = [Path(tmp), *Path(tmp).glob("*.log")]
                if not listed:
                    log.touch()  # created after the listing, before the watch
                listed.append(found)
                return found

            def check():
                text = log.read_text()
                if not checked:
                    log.write_text("listening\n")  # the only later change
                checked.append(text)
                return text

            # 10 s only bounds a hang.
            self.assertEqual(watch.wait_for(check, paths, 10, "the write"), "listening\n")
            self.assertEqual(checked[0], "", "the check ran after the write: nothing proved")


if __name__ == "__main__":
    unittest.main()
