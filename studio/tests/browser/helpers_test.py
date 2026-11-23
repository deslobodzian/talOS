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

    def test_in_place_write_after_atomic_replace_wakes_the_wait(self):
        """Replacing a watched file atomically (rename over it, as the launcher
        updates manifest.json) swaps its inode; the watch must detect the
        replacement and re-attach to the new inode so a subsequent in-place
        write wakes wait_for rather than watching the unlinked inode until
        timeout."""
        with tempfile.TemporaryDirectory() as tmp:
            target = Path(tmp) / "manifest.json"
            target.write_text("v0")
            staged = Path(tmp) / "manifest.json.tmp"
            step = [0]

            def paths():
                # Watch only the file itself (not its parent dir) so both
                # atomic-replace detection and inode re-watch are exercised
                # natively on inotify and kqueue alike.
                return [target]

            def check():
                text = target.read_text()
                if text == "v2":
                    return text
                if step[0] == 0:
                    step[0] = 1
                    staged.write_text("v1")
                    os.replace(staged, target)
                    return None
                if step[0] == 1 and text == "v1":
                    step[0] = 2
                    with target.open("w") as handle:
                        handle.write("v2")
                        handle.flush()
                return None

            self.assertEqual(watch.wait_for(check, paths, 10, "post-replace write"), "v2")

    def test_kqueue_add_raises_on_non_vanished_error_and_tracks_inode_replace(self):
        """_Kqueue.add must fail fast on non-ENOENT/ENOTDIR open errors (never
        silently leaving wait_for blind until its bound) and replace the old
        watched fd when a file's inode changes."""
        import errno as _errno
        from unittest import mock

        class FakeKqueue:
            def __init__(self):
                self.registered = []
                self.drained = 0
                self.pending_batches = 2
                self.closed = False

            def fileno(self):
                return 99

            def control(self, changelist, max_events, timeout):
                if changelist is not None:
                    self.registered.extend(changelist)
                    return []
                self.drained += 1
                if self.pending_batches > 0:
                    self.pending_batches -= 1
                    return [object()]
                return []

            def close(self):
                self.closed = True

        fake_kq = FakeKqueue()
        with tempfile.TemporaryDirectory() as tmp:
            target = Path(tmp) / "manifest.json"
            target.write_text("v0")
            staged = Path(tmp) / "manifest.json.tmp"
            with mock.patch.object(watch.select, "kqueue", create=True, return_value=fake_kq), \
                    mock.patch.object(watch.select, "kevent", create=True,
                                      side_effect=lambda fd, **kw: (fd, kw)), \
                    mock.patch.object(watch.select, "KQ_FILTER_VNODE", -4, create=True), \
                    mock.patch.object(watch.select, "KQ_EV_ADD", 1, create=True), \
                    mock.patch.object(watch.select, "KQ_EV_CLEAR", 2, create=True), \
                    mock.patch.object(watch.select, "KQ_NOTE_DELETE", 1, create=True), \
                    mock.patch.object(watch.select, "KQ_NOTE_WRITE", 2, create=True), \
                    mock.patch.object(watch.select, "KQ_NOTE_EXTEND", 4, create=True), \
                    mock.patch.object(watch.select, "KQ_NOTE_RENAME", 8, create=True):
                kq = watch._Kqueue()
                try:
                    self.assertTrue(kq.add(str(target)))
                    self.assertFalse(kq.add(str(target)))
                    # Atomic rename changes inode: add() must re-watch and close old fd.
                    staged.write_text("v1")
                    os.replace(staged, target)
                    self.assertTrue(kq.add(str(target)))
                    self.assertEqual(len(fake_kq.registered), 2)
                    # Missing path returns False; EMFILE raises instead of going blind.
                    self.assertFalse(kq.add(str(Path(tmp) / "missing.json")))
                    other = Path(tmp) / "other.json"
                    other.write_text("x")
                    with mock.patch.object(
                        watch.os, "open",
                        side_effect=OSError(_errno.EMFILE, "Too many open files"),
                    ):
                        with self.assertRaises(OSError) as err:
                            kq.add(str(other))
                        self.assertEqual(err.exception.errno, _errno.EMFILE)
                    # drain() loops until kqueue has no more pending batches.
                    kq.drain()
                    self.assertEqual(fake_kq.drained, 3)
                finally:
                    kq.close()
                self.assertTrue(fake_kq.closed)


if __name__ == "__main__":
    unittest.main()
