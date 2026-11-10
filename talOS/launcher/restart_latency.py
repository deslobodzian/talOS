"""Restart latency of a supervised node, per stage, from its event logs.

Starts the launcher on a config with --sim, SIGKILLs one node --kills times,
spaced past --restart-window-ms so each is a first restart, and reads each
restarted run's .tlog: kill -> event-loop origin -> run() -> first tick ->
first /hw/request/<node> send. Kills are stamped on the node clock
(CLOCK_MONOTONIC; CLOCK_MONOTONIC_RAW on macOS). Then it SIGKILLs each Python
node --python-kills times and times its spawn (the process start time) to the
restarted run's event-loop origin, which the node sets right before it claims
its registry row. The config's nodes must already be built (bazel build).
See talOS/launcher/README.md#restart-latency.
"""

from _posixshmem import shm_unlink  # what multiprocessing.shared_memory uses
import argparse
import ctypes
import json
import os
from pathlib import Path
import platform
import shutil
import signal
import socket
import statistics
import struct
import subprocess
import sys
import tempfile
import time
import uuid

from python.runfiles import runfiles

# talOS/events/log/format.h (version 6). EventKind (events/context.h) and
# SourceKind (events/manifest.h) both number TIMER 1; EventKind SEND is 4.
_FILE = struct.Struct("<8sIIQqqII64sQ")
_ENTRY = struct.Struct("<HHIIIqq64sQII")
_RECORD = struct.Struct("<IHHQqqqQIIII")
_RECORD_MAGIC = 0x54524543
TIMER, SEND = 1, 4

STAGES = [("kill_to_origin", "SIGKILL → event-loop origin"),
          ("origin_to_run", "origin → `run()`"),
          ("run_to_first_tick", "`run()` → first tick"),
          ("first_tick_to_request", "first tick → first `/hw/request/{node}`"),
          ("total", "**total**")]

# One writer's rewrite-and-sync loop for --disk-load: event logs on the same
# disk sync with a plain fsync (LogWriter), so this does too.
_DISK_LOAD = r"""
import os, sys
path, size = sys.argv[1], int(sys.argv[2])
block = os.urandom(1 << 20)
while True:
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    for _ in range(size >> 20):
        os.write(fd, block)
    os.fsync(fd)
    os.close(fd)
"""


def node_clock_ns():
    """The clock a node stamps its event log with (talOS/events Poller)."""
    clock = time.CLOCK_MONOTONIC_RAW if sys.platform == "darwin" else time.CLOCK_MONOTONIC
    return time.clock_gettime_ns(clock)


def read_tlog(path):
    """A .tlog's origin, manifest and record headers; payloads are skipped.

    A truncated or damaged tail ends the records, as a SIGKILLed run leaves.
    """
    with open(path, "rb") as stream:
        header = stream.read(_FILE.size)
        if len(header) < _FILE.size:
            raise ValueError(f"{path}: truncated file header")
        (magic, version, header_bytes, _, origin, _, count, _, process,
         schema_bytes) = _FILE.unpack(header)
        if magic != b"TALOSLOG" or version != 6:
            raise ValueError(f"{path}: not a version 6 .tlog")
        stream.seek(header_bytes)
        sources = {}
        for _ in range(count):
            raw = stream.read(_ENTRY.size)
            if len(raw) < _ENTRY.size:
                raise ValueError(f"{path}: truncated manifest")
            source_id, kind, _, _, armed, period, offset, name, *_ = _ENTRY.unpack(raw)
            sources[source_id] = {"kind": kind, "name": name.split(b"\0", 1)[0].decode(),
                                  "armed": armed != 0, "period_ns": period,
                                  "offset_ns": offset}
        stream.seek(schema_bytes, 1)
        records = []
        while len(raw := stream.read(_RECORD.size)) == _RECORD.size:
            magic, kind, source_id, _, event, now, *_, stored = _RECORD.unpack(raw)
            if magic != _RECORD_MAGIC:
                break
            records.append((kind, source_id, event, now))
            stream.seek(stored, 1)
    return {"process": process.split(b"\0", 1)[0].decode(), "origin_ns": origin,
            "sources": sources, "records": records}


def restart_stages(log, kill_ns, node):
    """Milliseconds per stage from a kill stamp to the run's first request."""
    ticks = [i for i, s in log["sources"].items()
             if s["kind"] == TIMER and s["name"] == node and s["armed"]]
    if len(ticks) != 1:
        raise ValueError(f"{node}: no armed tick timer named {node}")
    tick = log["sources"][ticks[0]]
    requests = {i for i, s in log["sources"].items() if s["name"] == f"/hw/request/{node}"}
    origin = log["origin_ns"]
    # run() shifts the schedule built before it by its own start: the first
    # tick's deadline is run() plus the manifest's origin-relative offset
    # (talOS/events/realtime_event_loop.h run_until).
    first_tick = next(((event, now) for kind, source, event, now in log["records"]
                       if kind == TIMER and source == ticks[0]), None)
    first_send = next((now for kind, source, _, now in log["records"]
                       if kind == SEND and source in requests), None)
    if first_tick is None:
        raise ValueError(f"{node}: no tick recorded")
    if first_send is None:
        raise ValueError(f"{node}: no /hw/request/{node} send recorded")
    deadline, first_tick = first_tick
    run = deadline - tick["offset_ns"]
    def ms(start, end):
        return (end - start) / 1e6
    return {"kill_to_origin": ms(kill_ns, origin), "origin_to_run": ms(origin, run),
            "run_to_first_tick": ms(run, first_tick),
            "first_tick_to_request": ms(first_tick, first_send),
            "total": ms(kill_ns, first_send)}


def process_start_ns(pid):
    """A live process's fork time on the node clock, and its resolution (ns).

    macOS: sysctl KERN_PROC_PID p_starttime (wall clock, microseconds), as
    talOS/rtms reads it. Linux: /proc/<pid>/stat start, in clock ticks since
    boot. Returns None once the process is gone.
    """
    if sys.platform == "darwin":
        libc = ctypes.CDLL(None, use_errno=True)
        libc.sysctl.argtypes = [ctypes.POINTER(ctypes.c_int), ctypes.c_uint, ctypes.c_void_p,
                                ctypes.POINTER(ctypes.c_size_t), ctypes.c_void_p,
                                ctypes.c_size_t]
        mib = (ctypes.c_int * 4)(1, 14, 1, pid)  # CTL_KERN, KERN_PROC, KERN_PROC_PID
        info = ctypes.create_string_buffer(1024)  # struct kinfo_proc is 648 bytes
        size = ctypes.c_size_t(len(info))
        if libc.sysctl(mib, 4, info, ctypes.byref(size), None, 0) != 0 or size.value == 0:
            return None
        seconds, microseconds = struct.unpack_from("=qi", info.raw, 0)  # kp_proc.p_starttime
        wall = seconds * 1_000_000_000 + microseconds * 1000
        return wall - (time.time_ns() - node_clock_ns()), 1000
    try:
        with open(f"/proc/{pid}/stat") as stream:
            fields = stream.read().rsplit(")", 1)[1].split()
    except OSError:
        return None
    hz = os.sysconf("SC_CLK_TCK")
    boot = time.clock_gettime_ns(time.CLOCK_BOOTTIME) - time.clock_gettime_ns(time.CLOCK_MONOTONIC)
    return int(fields[19]) * 1_000_000_000 // hz - boot, 1_000_000_000 // hz  # field 22


def is_python(binary):
    """A py_binary's entry point is a Python script."""
    try:
        with open(binary, "rb") as stream:
            first = stream.readline(256)
    except OSError:
        return False
    return first.startswith(b"#!") and b"python" in first


def evict_cache(binary):
    """Drops a node's files from the page cache; False when it could not.

    Linux evicts every file in the binary's runfiles (interpreter, packages,
    native libraries). macOS has no per-file eviction: `purge` drops the whole
    cache and needs root, so it runs only with cached sudo credentials.
    Either way, pages another live process maps (a second Python node's
    interpreter and shared packages) stay cached.
    """
    if sys.platform == "darwin":
        return subprocess.run(["sudo", "-n", "/usr/sbin/purge"],
                              capture_output=True).returncode == 0
    done = set()
    for directory, _, names in os.walk(os.path.realpath(binary) + ".runfiles",
                                       followlinks=True):
        for name in names:
            path = os.path.realpath(os.path.join(directory, name))
            if path in done or not os.path.isfile(path):
                continue
            done.add(path)
            try:
                fd = os.open(path, os.O_RDONLY)
            except OSError:
                continue
            try:
                os.posix_fadvise(fd, 0, 0, os.POSIX_FADV_DONTNEED)
            finally:
                os.close(fd)
    return True


def shm_name(topic, namespace):
    """talos::ShmObjectName (talOS/memory/shared_memory_ptr.h) in a namespace."""
    mapped = "/" + topic.lstrip("/").replace("/", ".")
    value = 0xCBF29CE484222325
    for byte in namespace.encode() + b"\0" + mapped.encode():
        value = ((value ^ byte) * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return f"/~{value:016x}"


def remove_segments(session, namespace):
    """Unlinks the session's topics and registry left in its namespace."""
    topics = {"/talos_registry.3"}  # talOS/introspection/registry.h kRegistryPath
    def collect(value):
        if isinstance(value, dict):
            for key, item in value.items():
                if key == "name" and isinstance(item, str) and item.startswith("/"):
                    topics.add(item)
                collect(item)
        elif isinstance(value, list):
            for item in value:
                collect(item)
    if session is not None and (session / "graph.json").is_file():
        collect(json.loads((session / "graph.json").read_text()))
    for topic in topics:
        try:
            shm_unlink(shm_name(topic, namespace))
        except OSError:
            pass
    lock = Path("/tmp/talos-registry-" + shm_name("/talos_registry.3", namespace)[1:] + ".lock")
    lock.unlink(missing_ok=True)


def udp_ports():
    # Hold both reservations until distinct ephemeral ports have been selected.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as gateway, \
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as bridge:
        gateway.bind(("127.0.0.1", 0))
        bridge.bind(("127.0.0.1", 0))
        return gateway.getsockname()[1], bridge.getsockname()[1]


def launcher_path():
    files = runfiles.Create()
    binary = files.Rlocation("_main/talOS/launcher/launcher") if files else None
    if not binary or not Path(binary).is_file():
        raise FileNotFoundError(
            "launcher runfile not found; run with bazel run //talOS/launcher:restart_latency")
    return binary


def summary(values):
    return f"{statistics.median(values):.1f} ({min(values):.1f}–{max(values):.1f})"


class Session:
    """A launcher session and its manifest, polled while nodes come and go."""

    def __init__(self, args, workspace, tmp):
        self.output = tmp / "sessions"
        self.namespace = f"restart-latency-{uuid.uuid4().hex}"
        self.directory = None
        remote_port, local_port = udp_ports()
        gateway_args = [arg.replace("{remote_port}", str(remote_port))
                        .replace("{local_port}", str(local_port))
                        for arg in args.sim_gateway_arg]
        command = [launcher_path(), "--config", str(args.config), "--sim",
                   "--remote", "127.0.0.1", "--remote-port", str(remote_port),
                   "--local-port", str(local_port), "--output-dir", str(self.output),
                   "--restart-window-ms", str(args.restart_window_ms)]
        if args.start_sim_gateway:
            command.append("--start-sim-gateway")
        for arg in gateway_args:
            command.extend(["--sim-gateway-arg", arg])
        # The nodes resolve their own runfiles; this binary's must not leak in.
        env = {key: value for key, value in os.environ.items()
               if not key.startswith(("RUNFILES_", "PYTHON")) and key != "JAVA_RUNFILES"}
        env["TALOS_SHM_NAMESPACE"] = self.namespace
        self.log = (tmp / "launcher.log").open("w")
        self.process = subprocess.Popen(command, cwd=workspace, env=env, stdout=self.log,
                                        stderr=subprocess.STDOUT, start_new_session=True)

    def nodes(self):
        if self.process.poll() is not None:
            raise RuntimeError(f"launcher exited with {self.process.returncode}")
        if self.directory is None:
            found = list(self.output.glob("*/manifest.json"))
            if not found:
                return {}
            self.directory = found[0].parent
        try:
            manifest = json.loads((self.directory / "manifest.json").read_text())
        except (OSError, ValueError):
            return {}  # replaced by rename, so only absent before the first write
        return {node["name"]: node for node in manifest["nodes"]}

    def wait(self, predicate, timeout_s, what):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            nodes = self.nodes()
            if predicate(nodes):
                return nodes
            time.sleep(0.002)
        raise RuntimeError(f"timed out after {timeout_s} s waiting for {what}")

    def kill(self, name, while_down=None):
        """SIGKILLs a node's current run; returns (kill stamp, restart, new pid).

        while_down runs after the run has exited and before its restart: the
        launcher is stopped (SIGSTOP) until it returns.
        """
        node = self.nodes()[name]
        pid, count = node["pid"], node["restart_count"]
        if while_down is not None:
            os.kill(self.process.pid, signal.SIGSTOP)
        try:
            stamp = node_clock_ns()
            os.kill(pid, signal.SIGKILL)
            if while_down is not None:
                time.sleep(0.2)  # the kernel tears down the run's mappings
                while_down()
        finally:
            if while_down is not None:
                os.kill(self.process.pid, signal.SIGCONT)
        nodes = self.wait(lambda n: n[name]["restart_count"] > count and
                          n[name]["pid"] > 0 and n[name]["pid"] != pid, 10,
                          f"{name} to restart")
        return stamp, nodes[name]["restart_count"], nodes[name]["pid"]

    def stop(self):
        if self.process.poll() is None:
            self.process.send_signal(signal.SIGTERM)
            try:
                self.process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait()
        self.log.close()
        remove_segments(self.directory, self.namespace)

    def tlog(self, name, restart):
        return self.directory / (f"{name}.tlog" if restart == 0 else
                                 f"{name}.restart{restart}.tlog")


def measure(args, workspace, tmp):
    """Runs the session; returns (kills, python spawns, failures)."""
    session = Session(args, workspace, tmp)
    kills, spawns, failures, loads = [], [], [], []
    python = {}
    spacing_s = (args.restart_window_ms + 500) / 1000
    try:
        seen = set()
        def started(nodes):
            for name, node in nodes.items():
                if node["pid"] > 0 and name not in seen:
                    seen.add(name)
                    binary = workspace / node["binary_path"]
                    if is_python(binary):
                        python[name] = binary
                        start = process_start_ns(node["pid"])
                        if start is not None:
                            spawns.append({"node": name, "restart": 0,
                                           "cache": "warm (after --describe)",
                                           "spawn_ns": start[0], "resolution_ns": start[1]})
            return bool(nodes) and len(seen) == len(nodes)
        nodes = session.wait(started, 120, "every node to start")
        if args.node not in nodes:
            raise RuntimeError(f"{args.node} is not in the config's roster")
        time.sleep(args.settle_s)
        for index in range(args.disk_load):
            loads.append(subprocess.Popen(
                [sys.executable, "-c", _DISK_LOAD, str(tmp / f"load{index}"), str(256 << 20)],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL))
        if loads:
            time.sleep(1)
        for _ in range(args.kills):
            stamp, restart, _ = session.kill(args.node)
            kills.append((stamp, restart))
            time.sleep(max(0.0, spacing_s - (node_clock_ns() - stamp) / 1e9))
        for load in loads:
            load.kill()
            load.wait()
        loads = []
        for name, binary in sorted(python.items()):
            if name == args.node:
                continue
            for _ in range(args.python_kills):
                evicted = []
                evict = (lambda: evicted.append(evict_cache(binary))) if args.evict_cache else None
                stamp, restart, pid = session.kill(name, evict)
                cache = ("warm" if not evicted else "evicted" if evicted[0]
                         else "warm (eviction failed)")
                start = process_start_ns(pid)
                if start is None:
                    failures.append(f"{name} restart {restart}: exited before its start was read")
                else:
                    spawns.append({"node": name, "restart": restart, "cache": cache,
                                   "spawn_ns": start[0], "resolution_ns": start[1]})
                time.sleep(max(0.0, spacing_s - (node_clock_ns() - stamp) / 1e9))
    finally:
        for load in loads:
            load.kill()
            load.wait()
        session.stop()
    stages = []
    for stamp, restart in kills:
        try:
            stages.append(restart_stages(read_tlog(session.tlog(args.node, restart)),
                                         stamp, args.node))
        except (OSError, ValueError) as error:
            failures.append(f"{args.node} restart {restart}: {error}")
    for spawn in spawns:
        try:
            origin = read_tlog(session.tlog(spawn["node"], spawn["restart"]))["origin_ns"]
            spawn["ms"] = (origin - spawn["spawn_ns"]) / 1e6
        except (OSError, ValueError) as error:
            failures.append(f"{spawn['node']} run {spawn['restart']}: {error}")
    return stages, [s for s in spawns if "ms" in s], failures


def report(args, tmp, stages, spawns, failures):
    clock = "CLOCK_MONOTONIC_RAW" if sys.platform == "darwin" else "CLOCK_MONOTONIC"
    load = f"{args.disk_load} writers rewriting and syncing 256 MiB files" if args.disk_load else "idle"
    print(f"# Restart latency: {args.node}, {len(stages)}/{args.kills} SIGKILLs measured")
    print(f"Config: {args.config}; --restart-window-ms {args.restart_window_ms}; disk: {load}")
    print(f"Session and disk load in {tmp} (TMPDIR selects the disk)")
    print(f"Host: {platform.system()} {platform.release()} {platform.machine()}, "
          f"{os.cpu_count()} CPUs; kill stamps on {clock}")
    if stages:
        print("\n| Stage | ms, median (range) |\n|---|---|")
        for key, label in STAGES:
            print(f"| {label.format(node=args.node)} | {summary([s[key] for s in stages])} |")
        print("\nPer kill, total ms: " + ", ".join(f"{s['total']:.1f}" for s in stages))
    if spawns:
        resolution = max(s["resolution_ns"] for s in spawns) / 1e6
        print("\n## Python node spawn → event-loop origin (registry row claimed next)")
        print(f"Spawn is the process start time, resolution {resolution:g} ms.")
        if any(s["cache"] == "evicted" for s in spawns):
            print("'evicted': dropped from the page cache between the kill and the "
                  "restart, except pages other live processes map.")
        print("\n| Node | Run | Page cache | ms |\n|---|---|---|---|")
        for s in spawns:
            run = "first launch" if s["restart"] == 0 else f"restart {s['restart']}"
            print(f"| {s['node']} | {run} | {s['cache']} | {s['ms']:.1f} |")
    for failure in failures:
        print(f"restart_latency: {failure}", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Gateway arguments take '=' (--sim-gateway-arg=--physics); "
               "{remote_port} and {local_port} in them become the session's "
               "loopback controller and bridge ports.")
    parser.add_argument("--config", type=Path, required=True,
                        help="robot config, absolute or workspace-relative")
    parser.add_argument("--node", default="drivetrain",
                        help="critical node to SIGKILL; it must send /hw/request/<node>")
    parser.add_argument("--kills", type=int, default=10)
    parser.add_argument("--restart-window-ms", type=int, default=1500,
                        help="launcher restart window; kills are spaced 500 ms past it")
    parser.add_argument("--settle-s", type=float, default=3.0,
                        help="seconds after every node started before the first kill")
    parser.add_argument("--python-kills", type=int, default=3,
                        help="SIGKILLs of each Python node for its spawn time")
    parser.add_argument("--evict-cache", action="store_true",
                        help="evict each Python node's files from the page cache "
                             "between its kill and its restart (macOS: `purge`, "
                             "needs cached sudo credentials)")
    parser.add_argument("--disk-load", type=int, default=0, metavar="N",
                        help="N processes rewrite and fsync 256 MiB files beside "
                             "the session (in TMPDIR) during the kills")
    parser.add_argument("--start-sim-gateway", action="store_true")
    parser.add_argument("--sim-gateway-arg", action="append", default=[])
    parser.add_argument("--keep", action="store_true", help="keep the session directory")
    args = parser.parse_args()
    workspace = Path(os.environ.get("BUILD_WORKSPACE_DIRECTORY", os.getcwd()))
    args.config = workspace / args.config
    tmp = Path(tempfile.mkdtemp(prefix="talos-restart-latency-"))
    try:
        stages, spawns, failures = measure(args, workspace, tmp)
    except (OSError, RuntimeError, ValueError) as error:
        print(f"restart_latency: {error}; launcher log and session kept in {tmp}",
              file=sys.stderr)
        return 1
    report(args, tmp, stages, spawns, failures)
    if failures or args.keep:
        print(f"Session kept in {tmp}", file=sys.stderr)
    else:
        shutil.rmtree(tmp)
    return 1 if failures or len(stages) != args.kills else 0


if __name__ == "__main__":
    sys.exit(main())
