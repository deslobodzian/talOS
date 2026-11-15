"""A child process in a session of its own, signalled as a group, never after it is reaped.

A process group is named by its leader's pid, which the kernel can hand to a
new process once the leader is reaped and the group is empty. So `Child`
signals its group only while the leader is unreaped (running, or a zombie
this process has not waited for), checks for an exit without reaping it
(waitid with WNOWAIT; on macOS, whose Python has no os.waitid, a kqueue
NOTE_EXIT registration, which an exited, unreaped child answers at once),
and records the reap: after `reap()` every `signal_group` is a no-op,
whatever the caller does twice.
"""

import os
import select
import subprocess
import threading


def _exited_unreaped(pid):
    """Whether child `pid` exited, without reaping it."""
    if hasattr(os, "waitid"):
        info = os.waitid(os.P_PID, pid, os.WEXITED | os.WNOHANG | os.WNOWAIT)
        return info is not None and info.si_pid == pid
    kq = select.kqueue()
    try:
        event = select.kevent(pid, select.KQ_FILTER_PROC,
                              select.KQ_EV_ADD | select.KQ_EV_ONESHOT, select.KQ_NOTE_EXIT)
        try:
            return bool(kq.control([event], 1, 0))
        except ProcessLookupError:
            return True  # an unreaped child's pid is never reused: it exited
    finally:
        kq.close()


def _wait_exited_unreaped(pid, timeout):
    """Waits up to `timeout` seconds for child `pid` to exit, without reaping
    it or polling: a pidfd on Linux; where Python lacks pidfd_open, a waitid
    with WNOWAIT blocking in a helper thread whose completion is awaited for
    `timeout`; the kqueue NOTE_EXIT event where it lacks waitid too (macOS).
    A child already reaped elsewhere counts as exited, until its pid is
    reused; only reap() reaps a child of these tests."""
    if hasattr(os, "pidfd_open"):
        try:
            fd = os.pidfd_open(pid)
        except ProcessLookupError:
            return True
        try:
            return bool(select.select([fd], [], [], timeout)[0])
        finally:
            os.close(fd)
    if hasattr(os, "waitid"):
        done = threading.Event()

        def block():
            try:
                os.waitid(os.P_PID, pid, os.WEXITED | os.WNOWAIT)
            except ChildProcessError:
                pass  # reaped elsewhere: it exited
            done.set()

        # A daemon: when the child outlives `timeout` the thread stays blocked
        # in waitid and must not keep the interpreter alive.
        threading.Thread(target=block, daemon=True).start()
        return done.wait(timeout)
    kq = select.kqueue()
    try:
        event = select.kevent(pid, select.KQ_FILTER_PROC,
                              select.KQ_EV_ADD | select.KQ_EV_ONESHOT, select.KQ_NOTE_EXIT)
        try:
            return bool(kq.control([event], 1, timeout))
        except ProcessLookupError:
            return True
    finally:
        kq.close()


class Child:
    def __init__(self, args, killpg=os.killpg, **popen):
        """`killpg` is the group signal (a test passes a recorder)."""
        self._killpg = killpg
        self.proc = subprocess.Popen(args, start_new_session=True, **popen)
        self.pid = self.proc.pid
        self.reaped = False

    @property
    def returncode(self):
        return self.proc.returncode

    def exited(self):
        """Whether the leader exited, leaving it unreaped (so its pid, and
        the group's, cannot be reused yet)."""
        if self.reaped:
            return True
        return _exited_unreaped(self.pid)

    def wait_exited(self, timeout):
        """Whether the leader exits within `timeout` seconds; never reaps."""
        if self.reaped:
            return True
        return _wait_exited_unreaped(self.pid, timeout)

    def signal_group(self, sig):
        """Signals the group; False once reaped, when it signals nothing."""
        if self.reaped:
            return False
        try:
            self._killpg(self.pid, sig)
        except ProcessLookupError:
            pass  # every member gone; the unreaped leader keeps the pid unused
        return True

    def reap(self, timeout=None):
        """Waits for the leader and reaps it (subprocess.TimeoutExpired when
        it outlives `timeout`, leaving it unreaped)."""
        if not self.reaped:
            self.proc.wait(timeout=timeout)
            self.reaped = True
        return self.proc.returncode
