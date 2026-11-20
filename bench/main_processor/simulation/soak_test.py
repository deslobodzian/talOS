"""Soak the bench's simulated graph for minutes while killing and stalling nodes.

The subject is the bench's robot_physics_sim.toml: the C++ swerve_module (an
actuator owner at 5 ms), the Python motor (an actuator owner at 20 ms), the
framework teleop node (the operator producer of both lanes, with
OperatorArming), the Python gyro observer, telemetry, the hardware bridge,
and the Studio bridge (--start-studio-bridge, every [studio] key at its
default: the bridge target, the webroot under bazel-bin/ of the working
directory, and the telemetry topic).

The scripted Driver Station holds teleop for TALOS_SOAK_SECONDS (default 180,
at most 600 so the run fits Bazel's long timeout)
with the stick centred for 0.5 s at the start of every 3 s cycle and driving
the rest (swerve hold-to-run button 6 and drive axis 5, the motor's voltage
axis 1). Each cycle, 1.8 s in, one fault is injected into a running process.
The first cycle's fault is always the owner restart under a frozen producer:
teleop is SIGSTOPped, swerve_module SIGKILLed, and teleop resumed 300 ms
later, so the owner restarts (at once: its first crash) while its lane holds
only targets published before the crash. Every later cycle's fault follows a
seeded order (TALOS_SOAK_SEED, default 0) that cycles through every kind:
SIGKILL (restarted by the launcher) or a 300 ms SIGSTOP (below the 500 ms
progress floor, so the launcher must not restart it). The Studio bridge is
launched too and only stalled, never killed.

Checked from the recorded session, over the whole run:
Each owner's records are read from each run's own event log (the manifest's
log_paths: restart k writes <name>.restart<k>.tlog), so every request is
judged by the inputs and crash of the process that sent it; a kill is
stamped before and after, and no stamp decides which process sent what. A
crash is the instant the launcher reaped it (the manifest run's
end_wall_ns, mapped to the node clock at the kill): after the death, before
the restart's spawn.

- every actuator owner's /hw/request gaps stay within its bridge owner
  timeout except across a fault injected into that node, and a killed owner
  sends again within RESTART_LIMIT_MS;
- no request actuates while the newest Driver Station view its process
  received is disabled, or before that process received one, and no
  swerve_module request actuates without an active teleop target received
  by the same process, published after that process's crash and younger
  than command_timeout;
- across a stalled teleop (or hardware bridge) every swerve_module request is
  neutral from command_timeout after the newest active target (or /hw/state)
  it received, and the first was due on its first tick after that; a stalled
  swerve_module is neutralized by the bridge's /hw/command within its owner
  timeout plus a period of its last request;
- swerve_module, restarted under the frozen producer, sends more than five
  requests before the producer resumes, all neutral from the first and at
  most 30 ms apart. The lane's newest target (the frozen producer's last,
  which must be active) is at most command_timeout less REGIME_TICKS of the
  owner's periods old at the restarted process's first request with an
  enabled Driver Station view, so Target::get would still accept it by its
  age: the requests are neutral because a restarted node's subscription
  receives only what is published after it attaches. A restart too slow for
  that fails the run as no evidence. The run prints the restart latency and
  that target's age;
- the launcher exits 0, nothing is quarantined, and each node's restart_count
  equals the number of times it was killed.
"""

import bisect
import json
import os
from pathlib import Path
import random
import shutil
import signal
import subprocess
import tempfile
import time
import unittest
import uuid

import support

CYCLE_S = 3.0
CENTER_S = 0.5
FAULT_AT_S = 1.8
STALL_S = 0.3
LEAD_S = 2.0  # disabled before and after teleop
RESTART_LIMIT_MS = 5000
# The restarted owner's output gap while its producer is frozen: its 5 ms
# tick with margin, as the match rehearsal bounds it.
RESTARTED_GAP_MS = 30
# The frozen-restart phase is evidence only while a retained target would
# still be accepted by its age for this many of the restarted owner's ticks
# from its first enabled request.
REGIME_TICKS = 4
# Teleop length cap: with startup, shutdown and log analysis the run stays
# inside Bazel's "long" timeout (900 s).
MAX_SOAK_S = 600

DRIVE = "swerve_module"  # the C++ owner whose lane the checks follow
PRODUCER = "teleop"  # the operator producer of every lane
# The first cycle's fault: SIGSTOP the producer, SIGKILL the owner, resume
# the producer STALL_S later.
FROZEN_KILL = ("frozen_kill", DRIVE)
# (action, node). Kills of non-critical telemetry stay under max_restarts (5)
# per rolling minute: one kind in this list runs once per len(FAULTS) cycles.
FAULTS = [
    ("stop", PRODUCER),
    ("kill", DRIVE),
    ("stop", "hardware_node"),
    ("kill", "telemetry"),
    ("stop", DRIVE),
    ("stop", "studio_bridge"),
    ("kill", PRODUCER),
    ("stop", "telemetry"),
    ("kill", "motor"),
    ("kill", "hardware_node"),
    ("stop", "motor"),
]
OWNERS = (DRIVE, "motor")
NEUTRAL = support.NEUTRAL


def joystick_schedule(teleop_s):
    entries = []
    cycle = 0
    while cycle * CYCLE_S < teleop_s:
        start = LEAD_S + cycle * CYCLE_S
        entries.append(f"{start:.3f}:axes=0,0,0,0,0,0;buttons=0;pov=-1")
        entries.append(f"{start + CENTER_S:.3f}:axes=0,-0.6,0,0,0,-0.6;buttons=32;pov=-1")
        cycle += 1
    entries.append(f"{LEAD_S + teleop_s:.3f}:axes=0,0,0,0,0,0;buttons=0;pov=-1")
    return "0:axes=0,0,0,0,0,0;buttons=0;pov=-1|" + "|".join(entries)


def _sigterm_to_exit(signum, _frame):
    """Turn SIGTERM into SystemExit so finally: cleanup always runs."""
    # Ignore a second SIGTERM while unwinding into finally: so cleanup cannot
    # be aborted mid-way; the inner finally: restores the previous handler.
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    raise SystemExit(128 + signum)


class Soak(unittest.TestCase):
    def read_nodes(self, manifest_path):
        return {n["name"]: n for n in json.loads(manifest_path.read_text())["nodes"]}

    def dump(self, root, logs, nodes, topics, since=None, until=None):
        """log_dump's rows of `logs` (the session directory, or a list of
        log files: then its times count from those files' own origin)."""
        args = [str(root / "talOS/launcher/log_dump"), "--format", "jsonl",
                "--node", nodes, "--topic", topics]
        if since is not None:
            args += ["--since-ms", f"{since:.3f}", "--until-ms", f"{until:.3f}"]
        logs = [logs] if isinstance(logs, Path) else logs
        with subprocess.Popen(args + [str(p) for p in logs], stdout=subprocess.PIPE,
                              text=True) as proc:
            for line in proc.stdout:
                yield json.loads(line)
        self.assertEqual(proc.returncode, 0, "log_dump failed")

    def neutral_by(self, rows, start, deadline, end, motor_ids, gap_ms, what):
        """rows: sorted (now_ms, motors). Neutral from some row <= deadline to end."""
        window = [r[:2] for r in rows if start <= r[0] < end]
        times = [start] + [t for t, _ in window] + [end]
        gap = max(b - a for a, b in zip(times, times[1:]))
        self.assertLessEqual(gap, gap_ms, f"{what}: output gap in [{start:.1f}, {end:.1f})")
        first = next((t for t, m in window if t <= deadline and
                      all(m[i] == NEUTRAL for i in motor_ids)), None)
        self.assertIsNotNone(first, f"{what}: no neutral output by {deadline:.1f} ms")
        for t, motors in window:
            if t >= first:
                self.assertTrue(all(motors[i] == NEUTRAL for i in motor_ids),
                                f"{what}: active output at {t:.1f} ms")

    def neutral_from(self, rows, expiry, end, motor_ids, gap_ms, period_ms, what):
        """rows: sorted (now_ms, motors, event_ms, ...). An input expired at
        `expiry`: every output from then to end is neutral, and the first was
        due (event_ms) on the node's first tick after it. A skipped flush
        would leave the bridge holding the last demand; a late tick still
        reports the deadline it missed, so host lateness is not bounded."""
        window = [r for r in rows if expiry <= r[0] < end]
        self.assertTrue(window, f"{what}: no output in [{expiry:.1f}, {end:.1f})")
        times = [expiry] + [r[0] for r in window] + [end]
        gap = max(b - a for a, b in zip(times, times[1:]))
        self.assertLessEqual(gap, gap_ms, f"{what}: output gap in [{expiry:.1f}, {end:.1f})")
        self.assertLess(window[0][2], expiry + period_ms,
                        f"{what}: the first output after the expiry at {expiry:.1f} ms "
                        f"was due at {window[0][2]:.1f} ms")
        for t, motors, *_ in window:
            self.assertTrue(all(motors[i] == NEUTRAL for i in motor_ids),
                            f"{what}: active output at {t:.1f} ms, after the expiry "
                            f"at {expiry:.1f} ms")

    @staticmethod
    def wall_minus_node_ns():
        """CLOCK_REALTIME less the node clock, read between two node-clock
        reads: maps a manifest wall time to the node clock to within those
        reads' microseconds (and the clocks' slew since)."""
        before = support.node_clock_ns()
        wall = time.time_ns()
        after = support.node_clock_ns()
        return wall - (before + after) // 2

    def driving(self, rows, start, end, drive_ids, what):
        window = [r[1] for r in rows if start <= r[0] < end]
        self.assertGreater(len(window), 2, f"{what}: no outputs in [{start:.1f}, {end:.1f})")
        self.assertTrue(all(m[i][0] == "velocity" and abs(m[i][2]) > 0.1
                            for m in window for i in drive_ids),
                        f"{what}: not driving in [{start:.1f}, {end:.1f})")

    # REQ-SAFE-31: composition evidence (the frozen restart).
    def test_graph_survives_kills_and_stalls(self):
        teleop_s = float(os.environ.get("TALOS_SOAK_SECONDS", "180"))
        self.assertTrue(CYCLE_S <= teleop_s <= MAX_SOAK_S,
                        f"TALOS_SOAK_SECONDS must be in [{CYCLE_S:.0f}, {MAX_SOAK_S}]")
        seed = int(os.environ.get("TALOS_SOAK_SEED", "0"))
        rng = random.Random(seed)
        root = support.workspace()
        config = root / "bench/main_processor/configuration/robot_physics_sim.toml"
        robot, subsystem_files = support.load_config(config)
        timeout_ms = robot["command_timeout_us"] / 1000
        period_ms = {}
        owner_timeout = {}
        for name in OWNERS:
            sub = support.tomllib.loads(subsystem_files[name].read_text())
            period_ms[name] = sub["subsystem"].get("period_us", robot["period_us"]) / 1000
            owner_timeout[name] = min(max(2 * period_ms[name], timeout_ms / 2), timeout_ms)
        # Its motor file takes its devices from motor.toml (`defaults`),
        # so the device order is robot.toml's.
        layout = support.device_layout(root / "bench/main_processor/configuration/robot.toml")
        motor_ids = {name: [i for i, sub, _, kind in layout
                            if sub == name and kind == "motors"] for name in OWNERS}
        drive_ids = [i for i, sub, name, kind in layout
                     if sub == DRIVE and kind == "motors" and name == "drive"]
        self.assertEqual(len(drive_ids), 1, layout)

        cycles = int(teleop_s // CYCLE_S)
        order = [FROZEN_KILL]
        while len(order) < cycles:
            batch = list(FAULTS)
            rng.shuffle(batch)
            order.extend(batch)
        order = order[:cycles]

        gw_port, bridge_port = support.udp_ports()
        tmp = Path(tempfile.mkdtemp(prefix="talos-soak-",
                                    dir=os.environ.get("TEST_UNDECLARED_OUTPUTS_DIR")))
        output = tmp / "sessions"
        env = support.launcher_env(TALOS_SHM_NAMESPACE=f"soak-{uuid.uuid4().hex}")
        print(f"soak: {teleop_s:.0f} s teleop, seed {seed}, "
              f"TALOS_SHM_NAMESPACE={env['TALOS_SHM_NAMESPACE']}", flush=True)
        launcher = None
        stopped = None
        success = False
        previous_sigterm = signal.signal(signal.SIGTERM, _sigterm_to_exit)
        try:
            (tmp / "bazel-bin").symlink_to(root, target_is_directory=True)
            duration = 2 * LEAD_S + teleop_s
            args = [str(root / "talOS/launcher/launcher"), "--config", str(config),
                    "--sim", "--start-sim-gateway", "--start-studio-bridge",
                    "--studio-http-port", "0",
                    "--remote", "127.0.0.1", "--remote-port", str(gw_port),
                    "--local-port", str(bridge_port), "--output-dir", str(output),
                    "--duration-s", f"{duration:.0f}"]
            gateway_args = [
                "--physics",
                "--mode-schedule",
                f"disabled:{LEAD_S:.0f},teleop:{teleop_s:.0f},disabled:{LEAD_S:.0f}",
                "--joystick-schedule", joystick_schedule(teleop_s)]
            for arg in gateway_args:
                args.extend(["--sim-gateway-arg", arg])
            events = []  # (action, node, start_ns, end_ns)
            kills = {}
            # DRIVE's kills in order: the wall-to-node clock offset read
            # right after each, to map its run's reaped instant.
            drive_offsets = []
            with (tmp / "launcher.txt").open("w+") as log:
                launcher = subprocess.Popen(args, cwd=tmp, env=env, stdout=log,
                                            stderr=subprocess.STDOUT,
                                            start_new_session=True)
                deadline = time.monotonic() + 30
                while time.monotonic() < deadline:
                    manifests = list(output.glob("*/manifest.json"))
                    if manifests:
                        manifest_path = manifests[0]
                        if self.read_nodes(manifest_path).get(
                                "sim_gateway", {}).get("pid", 0) > 0:
                            break
                    self.assertIsNone(launcher.poll(), "launcher exited during startup")
                    time.sleep(0.01)
                else:
                    self.fail("gateway never started")
                anchor = time.monotonic()

                for cycle, (action, name) in enumerate(order):
                    at = anchor + LEAD_S + cycle * CYCLE_S + FAULT_AT_S
                    time.sleep(max(0, at - time.monotonic()))
                    self.assertIsNone(launcher.poll(), "launcher exited during soak")
                    nodes = self.read_nodes(manifest_path)

                    def send(target, sig):
                        # A pid the manifest still lists but that has exited
                        # is a failure of the run, named, not a skip.
                        pid = nodes.get(target, {}).get("pid", 0)
                        self.assertGreater(pid, 0, f"{target} not running at cycle {cycle}")
                        try:
                            os.kill(pid, sig)
                        except ProcessLookupError:
                            self.fail(f"{target} (pid {pid}) gone at cycle {cycle} "
                                      f"({action}, {signal.Signals(sig).name})")
                        return pid

                    # A kill is stamped on both sides: the crash lies between
                    # the stamps, however long the harness is preempted
                    # there. Which process sent a request is read from the
                    # run log that recorded it, never from these stamps.
                    if action == "kill":
                        start_ns = support.node_clock_ns()
                        send(name, signal.SIGKILL)
                        end_ns = support.node_clock_ns()
                        kills[name] = kills.get(name, 0) + 1
                        if name == DRIVE:
                            drive_offsets.append(self.wall_minus_node_ns())
                    elif action == "frozen_kill":
                        # Freeze the producer before killing the owner: the
                        # restarted owner must not actuate from a lane
                        # published before its crash.
                        stopped = send(PRODUCER, signal.SIGSTOP)
                        start_ns = support.node_clock_ns()
                        send(name, signal.SIGKILL)
                        kills[name] = kills.get(name, 0) + 1
                        drive_offsets.append(self.wall_minus_node_ns())
                        time.sleep(STALL_S)
                        end_ns = support.node_clock_ns()
                        os.kill(stopped, signal.SIGCONT)
                        stopped = None
                    else:
                        # Stamped before the stop: the neutral deadline
                        # counts from no later than the stall's start.
                        start_ns = support.node_clock_ns()
                        stopped = send(name, signal.SIGSTOP)
                        time.sleep(STALL_S)
                        end_ns = support.node_clock_ns()
                        os.kill(stopped, signal.SIGCONT)
                        stopped = None
                    events.append((action, name, start_ns, end_ns))
                    print(f"cycle {cycle}: {action} {name}", flush=True)
                launcher.wait(timeout=duration + 30)
                log.seek(0)
                diagnostics = log.read()
            self.assertEqual(launcher.returncode, 0, diagnostics[-20000:])
            support.assert_realtime(self, diagnostics)

            session = manifest_path.parent
            final = self.read_nodes(manifest_path)
            for name, node in final.items():
                self.assertEqual(node["exit_code"], 0, name)
                self.assertFalse(node["quarantined"], name)
                self.assertEqual(node["restart_count"], kills.get(name, 0), name)

            baseline = support.log_baseline_ns(session)
            self.assertIsNotNone(baseline, "no recorded events")
            events = [(a, n, (s - baseline) / 1e6, (e - baseline) / 1e6)
                      for a, n, s, e in events]
            # Each owner's records are read run by run, so every row carries
            # the restart number of the process that recorded it, and is
            # judged by that process's own inputs and crash.
            requests = {name: [] for name in OWNERS}  # (now, motors, event, run)
            ds = {name: [] for name in OWNERS}  # (now, enabled, run)
            targets = []  # DRIVE's received active targets: (now, publish, run)
            lane = f"/{DRIVE}/target/{PRODUCER}"
            empty_logs = {name: {} for name in OWNERS}  # run: its files
            for name in OWNERS:
                for run, logs in support.run_logs(session, final[name]):
                    origin = support.log_baseline_ns(logs)
                    if origin is None:
                        # Killed before it recorded anything.
                        empty_logs[name][run] = logs
                        continue
                    shift = (origin - baseline) / 1e6
                    for row in self.dump(root, logs, name, f"/hw/request/{name},"
                                         f"/hw/state/driver_station,{lane}"):
                        topic, now = row["topic"], row["now_ms"] + shift
                        if topic == f"/hw/request/{name}" and row["kind"] == "SEND":
                            requests[name].append((now, support.motor_requests(row),
                                                   row["event_ms"] + shift, run))
                        elif (topic == "/hw/state/driver_station" and
                              row["kind"] == "MESSAGE"):
                            ds[name].append((now, support.ds_snapshot(row)[0] & 4, run))
                        elif (name == DRIVE and topic == lane and
                              row["kind"] == "MESSAGE" and
                              bytes.fromhex(row["payload_hex"])[8] != 0):
                            targets.append((now, row["publish_ms"] + shift, run))
            for rows in (*requests.values(), *ds.values(), targets):
                rows.sort(key=lambda r: r[0])

            for name in OWNERS:
                rows = requests[name]
                self.assertGreater(len(rows), 100, name)
                times = [r[0] for r in rows]
                own = [(s, e) for _, n, s, e in events if n == name]
                # Gaps, excluding the span from a fault on this node to its
                # next request.
                for a, b in zip(times, times[1:]):
                    if any(s <= b and e >= a for s, e in own):
                        continue
                    self.assertLessEqual(b - a, owner_timeout[name],
                                         f"{name} request gap at {a:.1f} ms")
                gaps = sorted(b - a for a, b in zip(times, times[1:]))
                print(f"{name}: {len(rows)} requests, gap p99 "
                      f"{gaps[int(.99 * len(gaps))]:.1f} ms, max {gaps[-1]:.1f} ms",
                      flush=True)
                for action, n, s, e in events:
                    if n == name:
                        index = bisect.bisect_right(times, e)
                        self.assertLess(index, len(times), f"{name} never resumed")
                        self.assertLessEqual(times[index] - e, RESTART_LIMIT_MS,
                                             f"{name} slow to resume after {action}")
                # Disabled Driver Station view -> neutral request: the newest
                # view the sending process received, and none (disabled)
                # before its first.
                for run in {r[3] for r in rows}:
                    views = [(t, enabled) for t, enabled, r in ds[name] if r == run]
                    view_times = [t for t, _ in views]
                    for t, motors, _, _ in (r for r in rows if r[3] == run):
                        index = bisect.bisect_right(view_times, t) - 1
                        if index < 0 or not views[index][1]:
                            self.assertTrue(all(m == NEUTRAL for m in motors.values()),
                                            f"{name} active while disabled at {t:.1f} ms")

            drive_rows = requests[DRIVE]
            # Restart k follows the k-th kill (restart_count is checked
            # above): `stamps` orders the kills, and run k-1's manifest entry
            # holds its crash, the instant the launcher reaped it.
            stamps = sorted(s for a, n, s, _ in events
                            if a in ("kill", "frozen_kill") and n == DRIVE)
            runs = final[DRIVE]["runs"]
            self.assertEqual(final[DRIVE]["runs_dropped"], 0, "runs dropped")
            self.assertEqual(len(runs), len(stamps) + 1, runs)
            crashes = []
            for k, (run, offset) in enumerate(zip(runs, drive_offsets)):
                self.assertEqual(run["exit_signal"], signal.SIGKILL,
                                 f"{DRIVE} run {k} did not end by the kill: {run}")
                crashes.append((run["end_wall_ns"] - offset - baseline) / 1e6)
                self.assertGreaterEqual(crashes[-1], stamps[k],
                                        f"{DRIVE} run {k} reaped before its kill")
            for action, name, s, e in events:
                what = f"{action} {name} at {s:.1f} ms"

                def commands(since, until):
                    rows = [(r["now_ms"], support.motor_requests(r)) for r in self.dump(
                        root, session, "hardware_node", "/hw/command", since, until)
                        if r["kind"] == "SEND"]
                    return sorted(rows, key=lambda r: r[0])

                if action == "frozen_kill":
                    # SIGKILL loses the killed process's unflushed recording,
                    # so pre-kill actuation is read from what the bridge sent.
                    self.driving(commands(s - 300, s), s - 150, s, drive_ids, what)
                    # The restarted process's requests are those its own run
                    # log recorded before the producer resumed, from its
                    # first: no harness stamp decides which they are.
                    run = stamps.index(s) + 1
                    self.assertNotIn(run, empty_logs[DRIVE],
                                     f"{what}: the restarted owner's run log "
                                     f"{[str(p) for p in empty_logs[DRIVE].get(run, [])]} "
                                     "holds no record")
                    every = [r for r in drive_rows if r[3] == run]
                    self.assertTrue(every, f"{what}: the restarted owner's run log "
                                    "holds no request")
                    restarted = [r for r in every if r[0] < e]
                    self.assertGreater(
                        len(restarted), 5,
                        f"{what}: the restarted owner sent {len(restarted)} requests "
                        f"before the producer resumed (its first {every[0][0] - s:.1f} "
                        "ms after the kill)")
                    first = restarted[0][0]
                    # The lane's newest value is the last target the frozen
                    # producer published (its own record: it was stopped,
                    # not killed), and it must be an active one.
                    sent = [(r["now_ms"], bytes.fromhex(r["payload_hex"])[8] != 0)
                            for r in self.dump(root, session, PRODUCER, lane, s - 300, e)
                            if r["kind"] == "SEND" and r["now_ms"] < e]
                    self.assertTrue(sent, f"{what}: no target published")
                    published, active = max(sent)
                    self.assertTrue(active, f"{what}: the frozen lane's newest target "
                                    f"at {published:.1f} ms is a release")
                    # A retained target would drive from the restarted
                    # owner's first request after its first enabled Driver
                    # Station packet until the target's age reaches
                    # command_timeout. The phase is evidence only if that
                    # window held REGIME_TICKS of its ticks; a slower
                    # restart proves nothing, and fails as such.
                    enabled = [t for t, on, r in ds[DRIVE] if r == run and on and t < e]
                    armed = next((t for t, *_ in restarted
                                  if enabled and t >= enabled[0]), None)
                    self.assertIsNotNone(armed, f"{what}: the restarted owner received no "
                                         "enabled Driver Station packet before the resume")
                    # s precedes the kill, so each latency is an upper bound;
                    # the crash is the instant the launcher reaped it.
                    crash = crashes[run - 1]
                    print(f"{what}: crash reaped {crash - s:.1f} ms after the pre-kill stamp; "
                          f"restarted owner's first request {first - crash:.1f} ms "
                          f"after it, its first enabled one {armed - crash:.1f} ms; the "
                          f"lane's newest target was then {armed - published:.1f} ms old "
                          f"(command_timeout {timeout_ms:.0f} ms)", flush=True)
                    limit = timeout_ms - REGIME_TICKS * period_ms[DRIVE]
                    self.assertLessEqual(
                        armed - published, limit,
                        f"{what}: not evidence: the restarted owner's first request "
                        f"with an enabled Driver Station view came {armed - s:.1f} ms "
                        f"after the kill, when the frozen target was "
                        f"{armed - published:.1f} ms old, over the {limit:.0f} ms "
                        f"(command_timeout less {REGIME_TICKS} ticks) that leaves "
                        "Target::get accepting it by its age, so neutral requests do "
                        "not show that it was never delivered")
                    self.neutral_by(restarted, first, first, e, motor_ids[DRIVE],
                                    RESTARTED_GAP_MS, what)
                if action != "stop":
                    continue
                # Each stall is judged from the input it starves, as recorded,
                # not from the harness's SIGSTOP: the last target (or State)
                # can precede the stop by a producer tick or more.
                if name == PRODUCER:
                    self.driving(drive_rows, s - 150, s, drive_ids, what)
                    # The lane expires a timeout after the newest active
                    # target the owner received before the resume.
                    newest = max((p for r, p, _ in targets if r < e), default=None)
                    self.assertIsNotNone(newest, f"{what}: no active target received")
                    self.neutral_from(drive_rows, newest + timeout_ms, e,
                                      motor_ids[DRIVE], owner_timeout[DRIVE],
                                      period_ms[DRIVE], what)
                elif name == "hardware_node":
                    self.driving(drive_rows, s - 150, s, drive_ids, what)
                    # robot_enabled() turns false a timeout after the
                    # publication of the newest /hw/state the owner
                    # received before the resume.
                    states = [r["publish_ms"] for r in self.dump(
                        root, session, DRIVE, "/hw/state", s - 300, e)
                        if r["kind"] == "MESSAGE" and r["now_ms"] < e]
                    self.assertTrue(states, f"{what}: no /hw/state received")
                    self.neutral_from(drive_rows, max(states) + timeout_ms, e,
                                      motor_ids[DRIVE], owner_timeout[DRIVE],
                                      period_ms[DRIVE], what)
                elif name == DRIVE:
                    sent = commands(s - 300, e + 300)
                    self.driving(sent, s - 150, s, drive_ids, what)
                    # The bridge times the owner out an owner timeout after
                    # it read the last request, on its 1 ms check; it does
                    # not record that read, so an owner period past the
                    # request's send covers the read and the check.
                    last = drive_rows[bisect.bisect_left(
                        [r[0] for r in drive_rows], e) - 1][0]
                    self.neutral_by(sent, s, last + owner_timeout[DRIVE] +
                                    period_ms[DRIVE], e, motor_ids[DRIVE],
                                    owner_timeout[DRIVE], what)

            # No swerve_module actuation without an active target received
            # by the process that sent the request, published after the
            # previous run's crash (reaped, so no target published from then
            # on was that run's) and younger than the timeout when the request
            # was sent, as Target::get() judges. "After the crash" holds
            # whatever the restart latency: a restarted owner's reader starts
            # at the lane's next publication (RTMSQueue::register_reader).
            received = {}
            for r, p, run in targets:
                received.setdefault(run, []).append((r, p))
            for t, motors, _, run in drive_rows:
                if all(m[0] == "neutral" for m in motors.values()):
                    continue
                crash = crashes[run - 1] if run > 0 else float("-inf")
                own = received.get(run, [])
                j = bisect.bisect_right(own, (t, float("inf"))) - 1
                fresh = False
                # A target received before t - timeout was published before it.
                while j >= 0 and own[j][0] >= t - timeout_ms:
                    if 0 <= t - own[j][1] < timeout_ms and own[j][1] > crash:
                        fresh = True
                        break
                    j -= 1
                self.assertTrue(fresh, f"{DRIVE} active without fresh target at {t:.1f} ms")
            success = True
        finally:
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
            try:
                if stopped is not None:
                    try:
                        os.kill(stopped, signal.SIGCONT)
                    except ProcessLookupError:
                        pass
                if launcher is not None:
                    if launcher.poll() is None and not success:
                        try:
                            os.killpg(launcher.pid, signal.SIGKILL)
                        except ProcessLookupError:
                            pass
                    launcher.wait(timeout=10)
                (tmp / "bazel-bin").unlink(missing_ok=True)
                if success:
                    shutil.rmtree(tmp)
                else:
                    print(f"Soak artifacts retained: {tmp}", flush=True)
                    launcher_log = tmp / "launcher.txt"
                    if launcher_log.exists():
                        print(launcher_log.read_text(errors="replace")[-20000:], flush=True)
                    # The launcher reports only how each bridge run ended (exit
                    # code or signal); the bridge's own reason (webroot, HTTP
                    # listen, config) is in its log.
                    for bridge_log in output.glob("*/studio_bridge.log"):
                        print(f"{bridge_log}:\n" +
                              bridge_log.read_text(errors="replace")[-4000:], flush=True)
            finally:
                signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == "__main__":
    unittest.main()
