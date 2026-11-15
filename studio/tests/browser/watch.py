"""Wait for a condition on files without polling: re-check it on each change.

`wait_for(check, paths, bound_s, what)` re-runs `check()` whenever one of
`paths()` changes (an entry created, renamed in or written), and returns its
first truthy result. The change comes from the kernel (inotify on Linux,
kqueue on macOS), so nothing sleeps; `bound_s` only bounds a hang, through a
timer that writes to a pipe the wait also watches, and reads no clock.
`paths()` is asked again after every change, so a file or directory that
appears later (a session directory, a log) is watched from then on.
"""

import ctypes
import os
from pathlib import Path
import select
import sys
import threading

_IN_MODIFY = 0x002
_IN_CLOSE_WRITE = 0x008
_IN_MOVED_TO = 0x080
_IN_CREATE = 0x100


class _Inotify:
    def __init__(self):
        self.libc = ctypes.CDLL(None, use_errno=True)
        self.fd = self.libc.inotify_init1(os.O_NONBLOCK | os.O_CLOEXEC)
        if self.fd < 0:
            raise OSError(ctypes.get_errno(), "inotify_init1")
        self.watched = set()

    def add(self, path):
        if path in self.watched:
            return
        mask = _IN_MODIFY | _IN_CLOSE_WRITE | _IN_MOVED_TO | _IN_CREATE
        if self.libc.inotify_add_watch(self.fd, os.fsencode(path), mask) >= 0:
            self.watched.add(path)

    def drain(self):
        try:
            while os.read(self.fd, 65536):
                pass
        except BlockingIOError:
            pass

    def close(self):
        os.close(self.fd)


class _Kqueue:
    def __init__(self):
        self.queue = select.kqueue()
        self.fd = self.queue.fileno()
        self.files = {}

    def add(self, path):
        if path in self.files:
            return
        try:
            fd = os.open(path, getattr(os, "O_EVTONLY", os.O_RDONLY))
        except OSError:
            return
        self.files[path] = fd
        event = select.kevent(fd, filter=select.KQ_FILTER_VNODE,
                              flags=select.KQ_EV_ADD | select.KQ_EV_CLEAR,
                              fflags=select.KQ_NOTE_WRITE | select.KQ_NOTE_EXTEND)
        self.queue.control([event], 0, 0)

    def drain(self):
        self.queue.control(None, 64, 0)

    def close(self):
        for fd in self.files.values():
            os.close(fd)
        self.queue.close()


def wait_for(check, paths, bound_s, what):
    """Returns check()'s first truthy result; raises TimeoutError after bound_s."""
    notify = _Kqueue() if sys.platform == "darwin" else _Inotify()
    expired_r, expired_w = os.pipe()
    lock = threading.Lock()
    done = []

    def expire():
        with lock:  # never writes to a closed (or reused) descriptor
            if not done:
                os.write(expired_w, b"x")

    timer = threading.Timer(bound_s, expire)
    timer.daemon = True
    timer.start()
    try:
        while True:
            # Watch before checking, so a change between the two wakes us.
            for path in paths():
                if Path(path).exists():
                    notify.add(str(path))
            result = check()
            if result:
                return result
            ready, _, _ = select.select([notify.fd, expired_r], [], [])
            if expired_r in ready:
                raise TimeoutError(f"{what}: not within {bound_s:.0f} s")
            notify.drain()
    finally:
        timer.cancel()
        with lock:
            done.append(True)
        os.close(expired_r)
        os.close(expired_w)
        notify.close()
