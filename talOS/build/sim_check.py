"""The check talos_sim_test (contract.bzl) runs: a talos_sim target for a
bounded time, judged from the session it records.

    sim_check.py --sim RLOCATION --duration-s N --log-dump RLOCATION
        --session-report RLOCATION --registry-unlink RLOCATION

It runs `<sim> --duration-s N --output-dir <tmp> --remote 127.0.0.1
--remote-port P --local-port Q` on free loopback ports and passes only when:

- the launch exits 0 before its hang bound;
- every member in the session's manifest ran exactly once (a second run is
  a restart) and exited with code 0 and no signal;
- the session holds the simulator and at least one node besides it and the
  hardware bridge;
- the bridge received State from the simulator: session_report's
  "controller link" verdict (talOS/launcher/session_report.py,
  controller_link), from the session's /hw/state and /hw/status records,
  is PASS. The exit statuses cannot say this: the launcher ends a bounded
  session by stopping every member, and a stopped bridge exits 0 whether or
  not a State arrived;
- no process of the launch is left once it has ended.

The members come from the session's manifest, never from a list here.

A hang. The launch runs in a session of its own (setsid). Still running at
its bound, which is Bazel's TEST_TIMEOUT less the time the stop below may
take, it gets SIGTERM to its process group (the runner and the launcher,
which stops its session in order), then SIGKILL after STOP_WAIT_S. Each
member leads a process group of its own in the same session and dies with
the launcher by its guardian (talOS/launcher, REQ-SAFE-30), so the check
then waits, on the kernel's exit notifications, for every process left in
the launch's session, kills any still there after EXIT_WAIT_S, and fails
naming them. A failure prints the launch's output.

Shared memory. The launch runs in the test's namespace (TEST_TMPDIR, which
ShmObjectName maps every name into), so it never touches a robot's. The
launcher unlinks its session's topics when it stops in order; the live
registry it leaves for later sessions is unlinked by registry_unlink,
which takes the name from ShmObjectName itself. A launch killed past its
stop leaves its topics; the next run in the same namespace replaces them.
"""

import argparse
import ctypes
import importlib.util
import json
import os
import select
import signal
import socket
import subprocess
import sys
import tempfile
import time

from python.runfiles import runfiles

# talOS/launcher/roster.h kHardwareNodeName and kSimGatewayName: the names
# the launcher gives the bridge and the simulator in every session.
BRIDGE = "hardware_node"
SIMULATOR = "sim_gateway"

# Bounds on waits that end on an event; each only bounds a hang
# (docs/TESTING.md §6.3). The launcher's stop gives each member 2 s after
# its SIGINT, then reaps; a killed process exits at once.
STOP_WAIT_S = 10
EXIT_WAIT_S = 10
KILL_WAIT_S = 10
LOG_DUMP_TIMEOUT_S = 30
# Bazel's timeout less what a hang's stop may take, so a hang fails with
# the launch's output instead of being killed by Bazel.
HANG_MARGIN_S = STOP_WAIT_S + EXIT_WAIT_S + KILL_WAIT_S + 5


def udp_ports():
    """Two distinct free loopback UDP ports (docs/TESTING.md §6.4: bound at
    0 and released before the launch, since the launcher refuses port 0)."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as gateway, \
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as bridge:
        gateway.bind(("127.0.0.1", 0))
        bridge.bind(("127.0.0.1", 0))
        return gateway.getsockname()[1], bridge.getsockname()[1]


def session_processes(sid):
    """The pids of session `sid`'s processes that have not exited (a zombie
    has: it runs nothing)."""
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
            # pid (comm) state ppid pgrp session ...; comm may hold spaces.
            fields = stat[stat.rindex(b")") + 2:].split()
            if fields[0] != b"Z" and int(fields[3]) == sid:
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


# pidfd_open(2)'s number on every Linux architecture. Python builds
# against an older glibc (the hermetic interpreter's) lack os.pidfd_open.
SYS_PIDFD_OPEN = 434


def pidfd_open(pid):
    if hasattr(os, "pidfd_open"):
        return os.pidfd_open(pid)
    libc = ctypes.CDLL(None, use_errno=True)
    fd = libc.syscall(SYS_PIDFD_OPEN, ctypes.c_int(pid), ctypes.c_uint(0))
    if fd < 0:
        code = ctypes.get_errno()
        raise OSError(code, os.strerror(code))  # ESRCH: ProcessLookupError
    return fd


def wait_exited(pids, timeout_s):
    """Waits until every one of `pids` has exited, or `timeout_s`; returns
    those still running. Waits on the kernel's exit notification (a pidfd on
    Linux, a kqueue process filter on macOS), not by polling."""
    deadline = time.monotonic() + timeout_s
    if sys.platform.startswith("linux"):
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


class Launch:
    """A command run in a session of its own, its output to `output` (a
    file: a pipe would stay open while any process holds it)."""

    def __init__(self, args, output, pass_fds=()):
        self.process = subprocess.Popen(
            args, stdin=subprocess.DEVNULL, stdout=output,
            stderr=subprocess.STDOUT, start_new_session=True,
            pass_fds=pass_fds)
        # setsid made the child its session's and process group's leader.
        self.sid = self.process.pid

    def _signal_group(self, signum):
        try:
            os.killpg(self.sid, signum)
        except ProcessLookupError:
            pass

    def finish(self, hang_s, stop_wait_s=STOP_WAIT_S,
               exit_wait_s=EXIT_WAIT_S):
        """Waits up to `hang_s` for the launch; one still running is stopped
        as a hang (module docstring). Returns (its exit status, None if it
        hung; the pids of its session that had to be killed after it
        ended). Leaves no process of the session behind either way."""
        status = None
        try:
            status = self.process.wait(timeout=hang_s)
        except subprocess.TimeoutExpired:
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
        return status, sorted(left)


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


def judge(session, log_dump, report):
    """The session's problems (module docstring), empty if it passes."""
    with open(os.path.join(session, "manifest.json"), encoding="utf-8") as f:
        manifest = json.load(f)
    problems = []
    if manifest.get("stopped_before_start"):
        problems.append("the session stopped before every member started")
    names = []
    for node in manifest["nodes"]:
        name = node["name"]
        names.append(name)
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
    if not set(names) - {BRIDGE, SIMULATOR}:
        problems.append(f"no node besides {BRIDGE} and {SIMULATOR} ran: "
                        f"{names}")
    passes, verdict = controller_link(session, log_dump, report)
    if not passes:
        problems.append(f"controller link {verdict}")
    return problems


def hang_bound(duration_s):
    """Seconds the launch may run before it is stopped as a hang."""
    timeout = os.environ.get("TEST_TIMEOUT")
    if timeout:
        return max(duration_s, int(timeout) - HANG_MARGIN_S)
    return duration_s + HANG_MARGIN_S


def check(sim, duration_s, log_dump, session_report, registry_unlink,
          hang_s):
    """Runs `sim` for `duration_s` and judges it: (problems, diagnostics);
    no problems is a pass."""
    report = load_session_report(session_report)
    gateway_port, bridge_port = udp_ports()
    with tempfile.TemporaryDirectory(
            dir=os.environ.get("TEST_TMPDIR")) as tmp:
        out = os.path.join(tmp, "sessions")
        os.makedirs(out)
        output_path = os.path.join(tmp, "launch.log")
        args = [sim, "--duration-s", str(duration_s), "--output-dir", out,
                "--remote", "127.0.0.1", "--remote-port", str(gateway_port),
                "--local-port", str(bridge_port)]
        try:
            with open(output_path, "wb") as output:
                status, killed = Launch(args, output).finish(hang_s)
        finally:
            subprocess.run([registry_unlink], check=False)
        with open(output_path, encoding="utf-8", errors="replace") as f:
            diagnostics = (f"{' '.join(args)}\nports {gateway_port}, "
                           f"{bridge_port} (released before the launch; "
                           "another process may have taken one)\n"
                           f"launch output:\n{f.read()}")
        problems = []
        if status is None:
            problems.append(f"the launch was still running after {hang_s} s "
                            "and was stopped")
        elif status != 0:
            problems.append(f"the launch exited {status}")
        if killed:
            problems.append("processes of the launch outlived it and were "
                            f"killed: {killed}")
        if status is not None:
            sessions = [os.path.join(out, d) for d in os.listdir(out)
                        if os.path.isdir(os.path.join(out, d))]
            if len(sessions) != 1:
                problems.append(f"expected one session directory, found "
                                f"{sessions}")
            else:
                problems += judge(sessions[0], log_dump, report)
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
    parser.add_argument("--duration-s", type=int, required=True)
    args = parser.parse_args(argv)
    hang_s = hang_bound(args.duration_s)
    problems, diagnostics = check(
        rlocation(args.sim), args.duration_s, rlocation(args.log_dump),
        rlocation(args.session_report), rlocation(args.registry_unlink),
        hang_s)
    if problems:
        print(diagnostics, file=sys.stderr)
        print("sim_check: FAILED\n" + "\n".join(f"  {p}" for p in problems),
              file=sys.stderr)
        return 1
    print(f"sim_check: passed ({args.duration_s} s session)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
