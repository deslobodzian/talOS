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
