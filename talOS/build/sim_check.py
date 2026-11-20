"""The check talos_sim_test (contract.bzl) runs: a talos_sim target, judged
from the session it records.

    sim_check.py --sim RLOCATION --settle-s N --log-dump RLOCATION
        --session-report RLOCATION --registry-unlink RLOCATION
        [--studio-bridge]

It runs `<sim> --output-dir <tmp> --remote 127.0.0.1 --remote-port P
--local-port Q --duration-s <backstop>` (plus `--start-studio-bridge
--studio-http-port 0` when `--studio-bridge` is given) on free loopback
ports (the duration
lies past every bound below and only ends a launch whose check was killed
outright; the check stops the launch itself), and waits for
two events: the bridge has accepted a State (its `hardware_node: gateway
boot ` line in `<session>/hardware_node.log`, printed on the first State it
accepts from each gateway boot), and every member of the session's manifest
has a run whose loop has started (the manifest's per-node `realtime`, which
the launcher records from the run's registry row once the node reports it,
right before its loop runs, and posts as a manifest version). With
`--studio-bridge`, when `studio_bridge` is in the manifest, it also waits
for `studio_bridge` to record a bound `http_port` and for the launcher's
`Started studio_bridge` line in `<session>/talos.log` (printed only once
the bridge has run past `kStudioBridgeSettleInterval`, so stopping it sends
SIGINT rather than SIGKILL). It wakes on
file events (inotify on Linux, kqueue on macOS) and on the launch's exit,
never on a timer. Once ready, the session runs `settle_s` more and then
gets SIGINT, so the launcher stops it in order. A member whose run ends
or restarts before then ends the wait at once. So a slow-starting member (a
Python node importing a large library, a loaded host) delays the stop, and
only the hang bound below can fail a session for being slow.

A caller of check() that expects no State (a test of this check) can bound
the wait for the State alone by `state_s`, counted from the moment every
member's loop has started rather than from the launch, so a slow start-up
(a sanitizer build, a loaded host) still gets the whole hang bound to start
the session; the expiry is a problem of its own.

It passes only when:

- the events above came before the hang bound;
- the launch kept running until the check stopped it and then exited 0;
- the session has a readable manifest (a launcher that refuses its graph
  writes none);
- every member in the session's manifest ran exactly once (a second run is
  a restart) and exited with code 0 and no signal;
- the session holds the simulator and at least one node besides it and the
  hardware bridge;
- the bridge received State from the simulator: session_report's
  "controller link" verdict (talOS/launcher/session_report.py,
  controller_link), from the session's /hw/state and /hw/status records,
  is PASS. The exit statuses cannot say this: a stopped bridge exits 0
  whether or not a State arrived;
- no process of the launch is left once it has ended.

The members come from the session's manifest, never from a list here.

A hang. The launch runs in a session of its own (setsid). The wait for the
events is bounded by Bazel's TEST_TIMEOUT less `settle_s` and the bounds of
the steps after it (HANG_MARGIN_S); a launch that does not stop within
STOP_WAIT_S of its SIGINT gets SIGTERM to its process group (the runner and
the launcher), then SIGKILL after STOP_WAIT_S more. Each member leads a
process group of its own in the same session and dies with the launcher by
its guardian (talOS/launcher, REQ-SAFE-30), so the check then waits, on
the kernel's exit notifications, for every process left in the launch's
session, kills any still there after EXIT_WAIT_S, and fails naming them.
A process that left the session (a grandchild calling setsid) is not seen.
A failure prints the launch's output.

Shared memory. The launch runs in the test's namespace (TEST_TMPDIR, which
ShmObjectName maps every name into), so it never touches a robot's. The
launcher unlinks its session's topics when it stops in order, and in a
namespace also the live registry once its members are reaped
(talOS/launcher/README.md "One session per namespace"). A launch killed
past its stop leaves both: registry_unlink, which takes the name from
ShmObjectName itself, removes the registry after every check, and the
topics stay in the namespace.
"""

import argparse
import ctypes
import importlib.util
import json
import os
import random
import select
import signal
import socket
import subprocess
import sys
import tempfile
import time

from python.runfiles import runfiles

# talOS/launcher/roster.h kHardwareNodeName, kSimGatewayName and
# kStudioBridgeName: the names the launcher gives the bridge, the simulator
# and Studio's bridge in every session.
BRIDGE = "hardware_node"
SIMULATOR = "sim_gateway"
STUDIO_BRIDGE = "studio_bridge"
# What the bridge prints on the first State it accepts from a gateway boot
# (HardwareNode::ReportGatewayBuild), which the launcher writes to its log.
FIRST_STATE_LINE = "hardware_node: gateway boot "
# What the launcher writes to talos.log once studio_bridge has both bound its
# HTTP port and run past kStudioBridgeSettleInterval (StudioBridge::Supervise /
# Watch). Stopping before this line SIGKILLs the settling bridge.
STUDIO_STARTED_LINE = "[launcher] Started studio_bridge (pid "
# What readiness() waits for until FIRST_STATE_LINE is in the bridge's log.
WAITING_FOR_STATE = "a State accepted by the bridge"

# Bounds on waits that end on an event; each only bounds a hang
# (docs/TESTING.md §6.3). The launcher's stop gives each member 2 s after
# its SIGINT, then reaps; a killed process exits at once.
STOP_WAIT_S = 10
EXIT_WAIT_S = 5
KILL_WAIT_S = 5
LOG_DUMP_TIMEOUT_S = 10
# What the steps after the wait for the events may take, so that a hang
# fails with the launch's output instead of being killed by Bazel.
HANG_MARGIN_S = 2 * STOP_WAIT_S + EXIT_WAIT_S + KILL_WAIT_S + \
    LOG_DUMP_TIMEOUT_S + 5
# The shortest wait for the events worth running: a timeout or settle_s
# leaving less is refused before anything starts.
MIN_READY_S = 10
# Bazel's `short` timeout, for a run outside `bazel test`.
DEFAULT_TIMEOUT_S = 60


def udp_ports():
    """Two distinct free UDP ports on 0.0.0.0 below the OS ephemeral range
    (20000..31999; docs/TESTING.md §6.4: probed and released before the
    launch, since the launcher refuses port 0).

    Kept inline here because talos_sim_test (contract.bzl) runs sim_check.py
    directly via srcs with only @rules_python//python/runfiles in deps.
    """
    port_base, port_span = 20000, 12000
    start = random.randrange(port_span)
    held = []
    ports = []
    try:
        for offset in range(port_span):
            port = port_base + (start + offset) % port_span
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.bind(("0.0.0.0", port))
            except OSError:
                sock.close()
                continue
            held.append(sock)
            ports.append(port)
            if len(ports) == 2:
                return ports[0], ports[1]
    finally:
        for sock in held:
            sock.close()
    raise RuntimeError(
        f"fewer than 2 free UDP ports in "
        f"{port_base}-{port_base + port_span - 1}")


def running_in_session(stat, sid):
    """Whether `stat`, the bytes of one /proc/<pid>/stat, is a process of
    session `sid` that has not exited. A zombie (`Z`) has exited, and so
    has a process its reaper has taken but the kernel has not yet removed
    (`X`, EXIT_DEAD), which a reader can see in the instant between."""
    # pid (comm) state ppid pgrp session ...; comm may hold spaces and ")".
    fields = stat[stat.rindex(b")") + 2:].split()
    return fields[0] not in (b"Z", b"X") and int(fields[3]) == sid


def session_processes(sid):
    """The pids of session `sid`'s processes that have not exited (one that
    is a zombie, or is being reaped, has: it runs nothing; see
    running_in_session). macOS's ps(1) prints no state for a process being
    reaped but `Z`."""
    found = set()
    if os.path.isdir("/proc/self"):
        for entry in os.listdir("/proc"):
            if not entry.isdigit():
                continue
            try:
                with open(f"/proc/{entry}/stat", "rb") as f:
                    stat = f.read()
            except OSError:
                continue
            if running_in_session(stat, sid):
                found.add(int(entry))
        return found
    listing = subprocess.run(["ps", "-A", "-o", "pid=,stat="],
                             capture_output=True, text=True, check=True)
    for line in listing.stdout.splitlines():
        pid, state = line.split(None, 1)
        if state.startswith("Z"):
            continue
        try:
            if os.getsid(int(pid)) == sid:
                found.add(int(pid))
        except OSError:
            pass
    return found


LINUX = sys.platform.startswith("linux")
# pidfd_open(2)'s number on every Linux architecture. Python builds
# against an older glibc (the hermetic interpreter's) lack os.pidfd_open.
SYS_PIDFD_OPEN = 434
# inotify(7): a file written, a directory entry created or renamed in.
IN_MODIFY = 0x2
IN_MOVED_TO = 0x80
IN_CREATE = 0x100


def libc():
    return ctypes.CDLL(None, use_errno=True)


def raise_errno():
    code = ctypes.get_errno()
    raise OSError(code, os.strerror(code))  # ESRCH: ProcessLookupError


def pidfd_open(pid):
    if hasattr(os, "pidfd_open"):
        return os.pidfd_open(pid)
    fd = libc().syscall(SYS_PIDFD_OPEN, ctypes.c_int(pid), ctypes.c_uint(0))
    if fd < 0:
        raise_errno()
    return fd


def wait_exited(pids, timeout_s):
    """Waits until every one of `pids` has exited, or `timeout_s`; returns
    those still running. Waits on the kernel's exit notification (a pidfd on
    Linux, a kqueue process filter on macOS), not by polling."""
    deadline = time.monotonic() + timeout_s
    if LINUX:
        poller = select.poll()
        open_fds = {}
        try:
            for pid in pids:
                try:
                    fd = pidfd_open(pid)
                except ProcessLookupError:
                    continue
                open_fds[fd] = pid
                poller.register(fd, select.POLLIN)
            while open_fds:
                left = deadline - time.monotonic()
                if left <= 0:
                    break
                for fd, _ in poller.poll(left * 1000):
                    poller.unregister(fd)
                    os.close(fd)
                    del open_fds[fd]
            return set(open_fds.values())
        finally:
            for fd in open_fds:
                os.close(fd)
    queue = select.kqueue()
    try:
        alive = set()
        for pid in pids:
            try:
                queue.control([select.kevent(
                    pid, select.KQ_FILTER_PROC,
                    select.KQ_EV_ADD | select.KQ_EV_ONESHOT,
                    select.KQ_NOTE_EXIT)], 0, 0)
            except ProcessLookupError:
                continue
            alive.add(pid)
        while alive:
            left = deadline - time.monotonic()
            if left <= 0:
                break
            for event in queue.control(None, len(alive), left):
                alive.discard(event.ident)
        return alive
    finally:
        queue.close()


class Watch:
    """Wakes on a change to a watched directory's entries or a watched
    file's contents, or on process `pid`'s exit (inotify and a pidfd on
    Linux, kqueue on macOS)."""

    def __init__(self, pid):
        self._fds = []
        if LINUX:
            self._inotify = libc().inotify_init1(os.O_NONBLOCK | os.O_CLOEXEC)
            if self._inotify < 0:
                raise_errno()
            self._fds.append(self._inotify)
            self._poll = select.poll()
            self._poll.register(self._inotify, select.POLLIN)
            try:
                pidfd = pidfd_open(pid)
            except ProcessLookupError:
                return  # already exited: the caller sees it
            self._fds.append(pidfd)
            self._poll.register(pidfd, select.POLLIN)
        else:
            self._queue = select.kqueue()
            try:
                self._queue.control([select.kevent(
                    pid, select.KQ_FILTER_PROC,
                    select.KQ_EV_ADD | select.KQ_EV_ONESHOT,
                    select.KQ_NOTE_EXIT)], 0, 0)
            except ProcessLookupError:
                pass

    def add(self, path, directory):
        if LINUX:
            mask = IN_CREATE | IN_MOVED_TO if directory else IN_MODIFY
            if libc().inotify_add_watch(self._inotify, os.fsencode(path),
                                        ctypes.c_uint32(mask)) < 0:
                raise_errno()
            return
        fd = os.open(path, getattr(os, "O_EVTONLY", os.O_RDONLY))
        self._fds.append(fd)
        self._queue.control([select.kevent(
            fd, select.KQ_FILTER_VNODE, select.KQ_EV_ADD | select.KQ_EV_CLEAR,
            select.KQ_NOTE_WRITE | select.KQ_NOTE_EXTEND)], 0, 0)

    def wait(self, timeout_s):
        if LINUX:
            for fd, _ in self._poll.poll(max(0, timeout_s) * 1000):
                if fd == self._inotify:
                    try:
                        while os.read(self._inotify, 65536):
                            pass
                    except BlockingIOError:
                        pass
            return
        self._queue.control(None, 16, max(0, timeout_s))

    def close(self):
        for fd in self._fds:
            os.close(fd)
        if not LINUX:
            self._queue.close()


def valid_http_port(value):
    """Whether `value` is a bound TCP port number in [1, 65535]."""
    return (isinstance(value, int) and not isinstance(value, bool)
            and 1 <= value <= 65535)


def readiness(session, studio=False):
    """(done, waiting for): done once the bridge has accepted a State, every
    member's loop has started, and (with `studio`, when studio_bridge is in
    the manifest) studio_bridge has recorded a bound http_port and settled
    past kStudioBridgeSettleInterval; or as soon as any member's run has
    ended or restarted, which no wait can mend. When `studio` is True and
    studio_bridge is absent from a populated manifest (dropped because it was
    not configured or did not answer --describe), readiness does not wait on
    it so judge() fails it after the stop rather than hanging until the
    timeout."""
    waiting = []
    try:
        with open(os.path.join(session, f"{BRIDGE}.log"), "rb") as f:
            if FIRST_STATE_LINE.encode() not in f.read():
                waiting.append(WAITING_FOR_STATE)
    except OSError:
        waiting.append(WAITING_FOR_STATE)
    try:
        with open(os.path.join(session, "manifest.json"),
                  encoding="utf-8") as f:
            nodes = json.load(f)["nodes"]
    except (OSError, ValueError, KeyError, TypeError):
        return False, waiting + ["the session manifest"]
    if not nodes:
        return False, waiting + ["the session roster"]
    starting = []
    studio_node = None
    for node in nodes:
        if node.get("name") == STUDIO_BRIDGE:
            studio_node = node
        runs = node.get("runs") or []
        if (len(runs) > 1 or node.get("runs_dropped", 0) > 0
                or (runs and runs[-1].get("end_wall_ns"))):
            return True, []
        if not runs or not node.get("realtime"):
            starting.append(node["name"])
    if starting:
        waiting.append(f"the loops of {', '.join(starting)} to start")
    if studio and studio_node is not None:
        if not valid_http_port(studio_node.get("http_port")):
            waiting.append(f"{STUDIO_BRIDGE} to bind its HTTP port")
        try:
            with open(os.path.join(session, "talos.log"), "rb") as f:
                if STUDIO_STARTED_LINE.encode() not in f.read():
                    waiting.append(f"{STUDIO_BRIDGE} to settle")
        except OSError:
            waiting.append(f"{STUDIO_BRIDGE} to settle")
    return not waiting, waiting


def wait_ready(launch, out, timeout_s, state_s=None, studio=False):
    """Waits for readiness() of the session the launch writes under `out`,
    for the launch's exit, or `timeout_s`: returns the problem, empty when
    ready or the launch exited. With `state_s`, once every member's loop has
    started and only the State is missing, the wait ends `state_s` after
    that moment (never past `timeout_s`)."""
    deadline = time.monotonic() + timeout_s
    state_deadline = None
    watch = Watch(launch.process.pid)
    try:
        watch.add(out, directory=True)
        session = None
        log_watched = False
        talos_log_watched = False
        waiting = ["the session directory"]
        while launch.process.poll() is None:
            # Watches are added before the files are read, so a change
            # after a read wakes the next wait.
            if session is None:
                found = [os.path.join(out, d) for d in os.listdir(out)
                         if os.path.isdir(os.path.join(out, d))]
                if found:
                    session = found[0]
                    watch.add(session, directory=True)
            if session is not None:
                log = os.path.join(session, f"{BRIDGE}.log")
                if not log_watched and os.path.exists(log):
                    watch.add(log, directory=False)
                    log_watched = True
                if studio and not talos_log_watched:
                    talos_log = os.path.join(session, "talos.log")
                    if os.path.exists(talos_log):
                        watch.add(talos_log, directory=False)
                        talos_log_watched = True
                done, waiting = readiness(session, studio=studio)
                if done:
                    return ""
                if (state_s is not None and state_deadline is None
                        and waiting == [WAITING_FOR_STATE]):
                    state_deadline = time.monotonic() + state_s
            now = time.monotonic()
            if state_deadline is not None and now >= state_deadline:
                return (f"no State accepted by the bridge within {state_s} "
                        "s of every member's loop starting")
            if now >= deadline:
                return (f"after {timeout_s} s, still waiting for "
                        f"{'; '.join(waiting)}")
            watch.wait(min(deadline, state_deadline or deadline) - now)
        return ""
    finally:
        watch.close()


class Launch:
    """A command run in a session of its own, its output to `output` (a
    file: a pipe would stay open while any process holds it). Used as a
    context manager it is stopped on the way out, whatever happened inside:
    a check that raises, or is ended by a signal, leaves no process of the
    launch behind."""

    def __init__(self, args, output, pass_fds=(), cwd=None, env=None):
        self.process = subprocess.Popen(
            args, stdin=subprocess.DEVNULL, stdout=output,
            stderr=subprocess.STDOUT, start_new_session=True,
            pass_fds=pass_fds, cwd=cwd, env=env)
        # setsid made the child its session's and process group's leader.
        self.sid = self.process.pid
        self.stopped = None

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.finish()
        return False

    def _signal_group(self, signum):
        try:
            os.killpg(self.sid, signum)
        except ProcessLookupError:
            pass

    def finish(self, stop_wait_s=STOP_WAIT_S, exit_wait_s=EXIT_WAIT_S):
        """Stops the launch with SIGINT, as Ctrl-C does; one that does not
        exit within `stop_wait_s` is stopped as a hang (module docstring).
        Returns (whether it stopped on SIGINT, or had already exited; the
        pids of its session that had to be killed after it ended). Leaves
        no process of the session behind either way. A second call returns
        the first one's result."""
        if self.stopped is not None:
            return self.stopped
        in_order = True
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=stop_wait_s)
            except subprocess.TimeoutExpired:
                in_order = False
                self._signal_group(signal.SIGTERM)
                try:
                    self.process.wait(timeout=stop_wait_s)
                except subprocess.TimeoutExpired:
                    self._signal_group(signal.SIGKILL)
                    self.process.wait()
        left = wait_exited(session_processes(self.sid), exit_wait_s)
        left |= session_processes(self.sid)
        for pid in left:
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        wait_exited(left, KILL_WAIT_S)
        self.stopped = (in_order, sorted(left))
        return self.stopped


def load_session_report(path):
    spec = importlib.util.spec_from_file_location("session_report", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def controller_link(session, log_dump, report):
    """(passes, verdict text): session_report's controller link verdict
    over the session's /hw/state and /hw/status records."""
    dump = subprocess.run(
        [log_dump, "--format", "jsonl", "--topic", "/hw/state,/hw/status",
         "--", session],
        capture_output=True, text=True, check=False,
        timeout=LOG_DUMP_TIMEOUT_S)
    if dump.returncode != 0:
        return False, f"log_dump exited {dump.returncode}: {dump.stderr}"
    records = [json.loads(line) for line in dump.stdout.splitlines()]
    state_source, states = report.stream(records, "/hw/state")
    status_source, statuses = report.stream(records, "/hw/status")
    measured, passes, _, detail = report.controller_link(
        states, state_source, statuses, status_source)
    verdict = "PASS" if passes else "FAIL" if measured else "NOT MEASURED"
    return passes, f"{verdict}: {detail}"


def judge(session, log_dump, report, studio=False):
    """The session's problems (module docstring), empty if it passes."""
    try:
        with open(os.path.join(session, "manifest.json"),
                  encoding="utf-8") as f:
            manifest = json.load(f)
        nodes = manifest["nodes"]
    except (OSError, ValueError, KeyError, TypeError) as error:
        # The launcher writes the manifest before it starts a member, so a
        # launch that refused its graph leaves none; its output says why.
        what = ("no manifest" if isinstance(error, FileNotFoundError)
                else f"an unreadable manifest ({error})")
        return [f"the session {os.path.basename(session)} has {what}: the "
                "launch ended before it started the robot; its output above "
                "says why"]
    problems = []
    if manifest.get("stopped_before_start"):
        problems.append("the session stopped before every member started")
    names = []
    by_name = {}
    for node in nodes:
        name = node["name"]
        names.append(name)
        by_name[name] = node
        runs = node.get("runs", [])
        count = len(runs) + node.get("runs_dropped", 0)
        if count != 1:
            problems.append(f"{name} ran {count} times; it must run exactly "
                            "once (a second run is a restart)")
        for run in runs:
            if (run["exit_code"], run["exit_signal"]) != (0, 0):
                problems.append(
                    f"{name} (pid {run['pid']}) ended with exit code "
                    f"{run['exit_code']}, signal {run['exit_signal']} "
                    f"({run['reason']})")
    if SIMULATOR not in names:
        problems.append(f"no {SIMULATOR} in the session: it ran without a "
                        "simulator")
    if not set(names) - {BRIDGE, SIMULATOR, STUDIO_BRIDGE}:
        problems.append(f"no node besides {BRIDGE} and {SIMULATOR} ran: "
                        f"{names}")
    if studio:
        bridge = by_name.get(STUDIO_BRIDGE)
        if bridge is None:
            problems.append(
                f"no {STUDIO_BRIDGE} in the session manifest: it was not "
                "configured, missing from runfiles, or did not answer "
                "--describe")
        else:
            port = bridge.get("http_port")
            if not valid_http_port(port):
                problems.append(
                    f"{STUDIO_BRIDGE} did not record a valid http_port "
                    f"(got {port!r})")
            if ((bridge.get("exit_code", 0), bridge.get("exit_signal", 0))
                    != (0, 0) and not bridge.get("runs")):
                problems.append(
                    f"{STUDIO_BRIDGE} ended with exit code "
                    f"{bridge.get('exit_code')}, signal "
                    f"{bridge.get('exit_signal')}")
            restarts = bridge.get("restart_count", 0)
            if restarts != 0:
                problems.append(
                    f"{STUDIO_BRIDGE} restarted {restarts} times "
                    "(restart_count must be 0)")
            if bridge.get("recorder_failed", False):
                problems.append(
                    f"{STUDIO_BRIDGE} is marked recorder_failed in the "
                    "session manifest")
    passes, verdict = controller_link(session, log_dump, report)
    if not passes:
        problems.append(f"controller link {verdict}")
    return problems


def ready_bound(settle_s):
    """Seconds the wait for the events may take: Bazel's timeout less
    `settle_s` and HANG_MARGIN_S. Raises ValueError, naming both, when that
    leaves less than MIN_READY_S."""
    timeout = int(os.environ.get("TEST_TIMEOUT") or DEFAULT_TIMEOUT_S)
    bound = timeout - settle_s - HANG_MARGIN_S
    if bound < MIN_READY_S:
        raise ValueError(
            f"settle_s = {settle_s} leaves {bound} s of the test's {timeout} "
            f"s timeout to wait for the session to start (at least "
            f"{MIN_READY_S} s are needed after {HANG_MARGIN_S} s for the "
            "stop): lower settle_s or give the test a longer timeout")
    return bound


def check(sim, settle_s, log_dump, session_report, registry_unlink,
          ready_s, state_s=None, studio=False):
    """Runs `sim`, waits up to `ready_s` for it to be ready (with `state_s`,
    for the State only `state_s` past every member's loop starting; with
    `studio`, also starts Studio's bridge on an ephemeral HTTP port and
    judges it in the manifest; module docstring), lets it run `settle_s`
    more, stops it and judges it: (problems, diagnostics); no problems is a
    pass."""
    report = load_session_report(session_report)
    gateway_port, bridge_port = udp_ports()
    with tempfile.TemporaryDirectory(
            dir=os.environ.get("TEST_TMPDIR")) as tmp:
        out = os.path.join(tmp, "sessions")
        os.makedirs(out)
        output_path = os.path.join(tmp, "launch.log")
        # The check stops the launch itself. The launcher's own duration is
        # only a backstop for a check killed outright (SIGKILL after Bazel's
        # timeout), which could stop nothing: it lies past every bound the
        # check waits on, and a launch that ends on its own fails below.
        backstop_s = ready_s + settle_s + HANG_MARGIN_S
        args = [sim, "--output-dir", out, "--remote", "127.0.0.1",
                "--remote-port", str(gateway_port),
                "--local-port", str(bridge_port),
                "--duration-s", str(backstop_s)]
        if studio:
            args += ["--start-studio-bridge", "--studio-http-port", "0"]
        problems = []
        try:
            with open(output_path, "wb") as output, \
                    Launch(args, output) as launch:
                unready = wait_ready(launch, out, ready_s, state_s,
                                     studio=studio)
                if unready:
                    problems.append(unready)
                elif launch.process.poll() is None:
                    try:
                        launch.process.wait(timeout=settle_s)
                    except subprocess.TimeoutExpired:
                        pass
                ended = launch.process.poll()
                in_order, killed = launch.finish()
        finally:
            subprocess.run([registry_unlink], check=False)
        with open(output_path, encoding="utf-8", errors="replace") as f:
            diagnostics = (f"{' '.join(args)}\nports {gateway_port}, "
                           f"{bridge_port} (released before the launch; "
                           "another process may have taken one)\n"
                           f"launch output:\n{f.read()}")
        status = launch.process.returncode
        if ended is not None:
            problems.append(f"the launch ended on its own (exit {ended}) "
                            "before the check stopped it")
        elif not in_order:
            problems.append(f"the launch did not stop within {STOP_WAIT_S} s "
                            "of SIGINT and was killed")
        elif status != 0:
            problems.append(f"the launch exited {status}")
        if killed:
            problems.append("processes of the launch outlived it and were "
                            f"killed: {killed}")
        if in_order:
            sessions = [os.path.join(out, d) for d in os.listdir(out)
                        if os.path.isdir(os.path.join(out, d))]
            if len(sessions) != 1:
                problems.append(f"expected one session directory, found "
                                f"{sessions}")
            else:
                problems += judge(sessions[0], log_dump, report,
                                  studio=studio)
    return problems, diagnostics


def rlocation(path):
    found = runfiles.Create().Rlocation(path, source_repo="")
    if not found or not os.path.exists(found):
        sys.exit(f"sim_check: missing runfile {path}")
    return found


def main(argv):
    parser = argparse.ArgumentParser(prog="sim_check", allow_abbrev=False)
    for flag in ("--sim", "--log-dump", "--session-report",
                 "--registry-unlink"):
        parser.add_argument(flag, required=True)
    parser.add_argument("--settle-s", type=int, required=True)
    parser.add_argument(
        "--studio-bridge", action="store_true",
        help="start Studio's bridge on an ephemeral HTTP port and require "
             "it to run cleanly in the manifest")
    args = parser.parse_args(argv)
    # Bazel ends a test at its timeout with SIGTERM: leave through the
    # finally blocks, which stop the launch, as Ctrl-C does.
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(128 + signal.SIGTERM))
    try:
        ready_s = ready_bound(args.settle_s)
    except ValueError as error:
        sys.exit(f"sim_check: {error}")
    problems, diagnostics = check(
        rlocation(args.sim), args.settle_s, rlocation(args.log_dump),
        rlocation(args.session_report), rlocation(args.registry_unlink),
        ready_s, studio=args.studio_bridge)
    if problems:
        print(diagnostics, file=sys.stderr)
        print("sim_check: FAILED\n" + "\n".join(f"  {p}" for p in problems),
              file=sys.stderr)
        return 1
    print(f"sim_check: passed (stopped {args.settle_s} s after the session "
          "was ready)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
