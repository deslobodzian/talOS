"""Soak the bench's simulated graph for minutes while killing and stalling nodes.

The subject is testdata/robot_soak.toml (built on bench/main_processor/configuration/
robot.toml): two 5 ms C++ actuator owners (swerve_module and motor), a test-only
20 ms Python actuator owner (python_motor), the framework teleop node (the
operator producer of all three teleop lanes, with OperatorArming), the test-only
C++ auto_producer node (20 ms, producing all three autonomous lanes), the Python
gyro observer, telemetry, the hardware bridge, and the Studio bridge
(--start-studio-bridge, every [studio] key at its default: the bridge target,
the webroot under bazel-bin/ of the working directory, and the telemetry topic).

The scripted Driver Station runs TALOS_SOAK_SECONDS of enabled 3 s cycles
(default 180, at most 600 so the run fits Bazel's long timeout) framed by
disabled lead/tail legs (2 s each). Whenever at least four cycles run (including
48 s in prepush and 180 s in CI), the schedule passes through an initial teleop
leg, a 1 s disabled transition, an autonomous leg (~20% of enabled time, at
least 6 s, or 9 s at 36 s+), a 1 s disabled transition, and the remaining teleop
leg. During teleop cycles the stick is centred for the first 0.5 s and driving
the rest (swerve hold-to-run button 6 and drive axis 5 at -0.6, deadband-rescaled
to ~5.56 rps, and motor/python_motor voltage axis 1 at -0.6, deadband-rescaled
to ~1.11 V); during autonomous auto_producer drives swerve at 4.0 rps and both
motor and python_motor in velocity mode at 8.0 rps.

Each cycle, 1.8 s in, one fault is injected into a running process.
The first cycle's fault is always the owner restart under a frozen producer:
teleop is SIGSTOPped, swerve_module SIGKILLed, and teleop resumed 300 ms
later, so the owner restarts (at once: its first crash) while its lane holds
only targets published before the crash. Every later cycle's fault follows a
seeded order (TALOS_SOAK_SEED, default 0) that cycles through every kind:
SIGKILL (restarted by the launcher) or a 300 ms SIGSTOP (below the 500 ms
progress floor, so the launcher must not restart it), with both kill and stop
of auto_producer guaranteed inside the autonomous leg. The Studio bridge is
launched too and only stalled, never killed.

Checked from the recorded session, over the whole run:
Each owner's records are read from each run's own event log (the manifest's
log_paths: restart k writes <name>.restart<k>.tlog), so every request is
judged by the inputs and crash of the process that sent it; a kill is
stamped before and after, and no stamp decides which process sent what. A
crash is the instant the launcher reaped it (the manifest run's
end_wall_ns, mapped to the node clock at the kill): after the death, before
the restart's spawn.

- all three actuator owners' (two 5 ms C++ owners and the 20 ms Python owner)
  /hw/request gaps stay within their bridge owner timeout except across a fault
  injected into that node, and a killed or stalled owner sends again within
  RESTART_LIMIT_MS;
- swerve_module, restarted under the frozen producer, sends more than five
  requests before the producer resumes, all neutral from the first and at
  most RESTARTED_GAP_MS apart. The lane's newest target (the frozen producer's
  last, which must be active) is at most command_timeout less REGIME_TICKS of
  the owner's periods old at the restarted process's first request with an
  enabled Driver Station view, so Target::get would still accept it by its
  age: the requests are neutral because a restarted node's subscription
  receives only what is published after it attaches (REQ-SAFE-31). A restart
  too slow for that fails the run as no evidence. The run prints the restart
  latency and that target's age;
- across mode-generation handoffs (teleop -> disabled -> autonomous -> disabled
  -> teleop), all three actuator owners (swerve_module, motor, python_motor)
  produce sustained active auto-driven actuation during the autonomous leg (with
  no teleop leakage) and sustained active teleop-driven actuation in both teleop
  legs (with no auto leakage);
- the launcher exits 0, nothing is quarantined, and each node's restart_count
  equals the number of times it was killed (300 ms stalls stay below the
  500 ms progress floor and must not restart or quarantine a node).
  Per-tick disabled-neutral, stall-to-neutral, and fresh-target bounds are
  proven at component tier (talOS/devices, talOS/bridge, talOS/hardware) and
  are not re-asserted here (RULES.md 12).
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
LEAD_S = 2.0  # disabled before first leg and after last leg
TRANSITION_S = 1.0  # disabled gap between teleop and autonomous legs
# REQ-OPS-04, REQ-OPS-05: 5000 ms matches the launcher's startup grace
# (kStartupGrace in talOS/launcher/launcher.h); a restarted or resumed owner
# must publish its first /hw/request within that startup window.
RESTART_LIMIT_MS = 5000
# REQ-SAFE-31 (no end-to-end gap budget yet: TODO SPEC-4): 6 x swerve_module's
# 5 ms period_us (30 ms, inside its 50 ms owner_timeout under the bench's
# 100 ms command_timeout_us), proving the restarted owner flushes neutral
# continuously across the 300 ms frozen-producer window.
RESTARTED_GAP_MS = 30
# REQ-SAFE-31: 4 x swerve_module's 5 ms period_us (20 ms, leaving 80 ms under
# the bench's 100 ms command_timeout_us), requiring at least 4 enabled owner
# ticks while a retained pre-crash target would still be accepted by age.
REGIME_TICKS = 4
# Enabled soak length cap: with startup, transitions, shutdown and log analysis
# the run stays inside Bazel's "long" timeout (900 s).
MAX_SOAK_S = 600

# Driver Station flag bits (talOS/driver_station/driver_station.h).
DS_ENABLED = 1 << 2
DS_AUTONOMOUS = 1 << 3
DS_TELEOP = 1 << 4

# Distinct setpoints for teleop vs autonomous so mode-generation handoffs are
# directly observable in /hw/request/<owner>. Teleop rescales axis -0.6 through
# deadband_frac = 0.1 -> (0.6 - 0.1) / (1.0 - 0.1) = 5/9.
AUTO_DRIVE_RPS = 4.0  # auto_producer.cc kAutoDriveVelocityRps
AUTO_MOTOR_RPS = 8.0  # auto_producer.cc kAutoMotorVelocityRps
TELEOP_DRIVE_RPS = (0.6 - 0.1) / (1.0 - 0.1) * 10.0  # ~5.56 rps
TELEOP_MOTOR_V = (0.6 - 0.1) / (1.0 - 0.1) * 2.0  # ~1.11 V

DRIVE = "swerve_module"  # the C++ owner whose lane the frozen-kill check follows
PY_MOTOR = "python_motor"  # the 20 ms Python actuator owner
PRODUCER = "teleop"  # the operator producer of every teleop lane
AUTO_PRODUCER = "auto_producer"  # the test-only autonomous target producer
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
    ("kill", PY_MOTOR),
    ("stop", PY_MOTOR),
    ("kill", AUTO_PRODUCER),
    ("stop", AUTO_PRODUCER),
]
OWNERS = (DRIVE, "motor", PY_MOTOR)
NEUTRAL = support.NEUTRAL


def mode_legs(soak_s):
    """Split `soak_s` of enabled cycles into (teleop1, auto, teleop2) counts."""
    cycles = int(soak_s // CYCLE_S)
    if cycles < 4:
        return cycles, 0, 0
    auto_cycles = min(max(2, round(cycles * 0.2)), cycles - 2)
    if cycles >= 12:
        auto_cycles = max(auto_cycles, 3)
    remaining = cycles - auto_cycles
    teleop1_cycles = min(2, remaining - 1) if remaining >= 2 else remaining
    teleop2_cycles = remaining - teleop1_cycles
    return teleop1_cycles, auto_cycles, teleop2_cycles


def mode_schedule(teleop1_cycles, auto_cycles, teleop2_cycles):
    """Return (mode_schedule_str, duration_s, cycle_starts_s)."""
    teleop1_s = teleop1_cycles * CYCLE_S
    if auto_cycles == 0:
        schedule = f"disabled:{LEAD_S:.0f},teleop:{teleop1_s:.0f},disabled:{LEAD_S:.0f}"
        duration = 2 * LEAD_S + teleop1_s
        starts = [LEAD_S + i * CYCLE_S for i in range(teleop1_cycles)]
        return schedule, duration, starts
    auto_s = auto_cycles * CYCLE_S
    teleop2_s = teleop2_cycles * CYCLE_S
    schedule = (
        f"disabled:{LEAD_S:.0f},teleop:{teleop1_s:.0f},"
        f"disabled:{TRANSITION_S:.0f},autonomous:{auto_s:.0f},"
        f"disabled:{TRANSITION_S:.0f},teleop:{teleop2_s:.0f},"
        f"disabled:{LEAD_S:.0f}"
    )
    duration = 2 * LEAD_S + 2 * TRANSITION_S + teleop1_s + auto_s + teleop2_s
    auto_start = LEAD_S + teleop1_s + TRANSITION_S
    teleop2_start = auto_start + auto_s + TRANSITION_S
    starts = (
        [LEAD_S + i * CYCLE_S for i in range(teleop1_cycles)]
        + [auto_start + i * CYCLE_S for i in range(auto_cycles)]
        + [teleop2_start + i * CYCLE_S for i in range(teleop2_cycles)]
    )
    return schedule, duration, starts


def joystick_schedule(teleop1_cycles, auto_cycles, teleop2_cycles):
    """Centered during disabled/autonomous and first CENTER_S of each teleop cycle."""
    centered = "axes=0,0,0,0,0,0;buttons=0;pov=-1"
    driving = "axes=0,-0.6,0,0,0,-0.6;buttons=32;pov=-1"
    entries = []
    for cycle in range(teleop1_cycles):
        start = LEAD_S + cycle * CYCLE_S
        entries.append(f"{start:.3f}:{centered}")
        entries.append(f"{start + CENTER_S:.3f}:{driving}")
    teleop1_end = LEAD_S + teleop1_cycles * CYCLE_S
    entries.append(f"{teleop1_end:.3f}:{centered}")
    if teleop2_cycles > 0:
        teleop2_start = teleop1_end + TRANSITION_S + auto_cycles * CYCLE_S + TRANSITION_S
        for cycle in range(teleop2_cycles):
            start = teleop2_start + cycle * CYCLE_S
            entries.append(f"{start:.3f}:{centered}")
            entries.append(f"{start + CENTER_S:.3f}:{driving}")
        teleop2_end = teleop2_start + teleop2_cycles * CYCLE_S
        entries.append(f"{teleop2_end:.3f}:{centered}")
    return f"0:{centered}|" + "|".join(entries)


def fault_order(teleop1_cycles, auto_cycles, teleop2_cycles, rng):
    """Build seeded fault list with cycle-0 FROZEN_KILL and auto faults in auto."""
    cycles = teleop1_cycles + auto_cycles + teleop2_cycles
    if auto_cycles < 2:
        order = [FROZEN_KILL]
        while len(order) < cycles:
            batch = list(FAULTS)
            rng.shuffle(batch)
            order.extend(batch)
        return order[:cycles]
    auto_faults = [("kill", AUTO_PRODUCER), ("stop", AUTO_PRODUCER)]
    pool = [f for f in FAULTS if f[1] != AUTO_PRODUCER]
    rng.shuffle(pool)
    while len(pool) < cycles:
        batch = list(FAULTS)
        rng.shuffle(batch)
        pool.extend(batch)
    idx = 0
    teleop1_order = [FROZEN_KILL]
    need_t1 = teleop1_cycles - 1
    teleop1_order.extend(pool[idx:idx + need_t1])
    idx += need_t1
    need_auto_extra = auto_cycles - len(auto_faults)
    # Keep auto_faults within the first batch so long soaks (>= 234 s) preserve
    # per-batch stratification (one fault kind per len(FAULTS) cycles).
    first_auto_extra = min(need_auto_extra, len(FAULTS) - len(auto_faults))
    auto_first = list(auto_faults) + pool[idx:idx + first_auto_extra]
    idx += first_auto_extra
    rng.shuffle(auto_first)
    rem_auto = need_auto_extra - first_auto_extra
    auto_order = auto_first + pool[idx:idx + rem_auto]
    idx += rem_auto
    teleop2_order = pool[idx:idx + teleop2_cycles]
    return teleop1_order + auto_order + teleop2_order


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

    # REQ-SAFE-31: composition evidence (the frozen restart and mode handoff).
    def test_graph_survives_kills_and_stalls(self):
        soak_s = float(os.environ.get("TALOS_SOAK_SECONDS", "180"))
        self.assertTrue(CYCLE_S <= soak_s <= MAX_SOAK_S,
                        f"TALOS_SOAK_SECONDS must be in [{CYCLE_S:.0f}, {MAX_SOAK_S}]")
        seed = int(os.environ.get("TALOS_SOAK_SEED", "0"))
        rng = random.Random(seed)
        root = support.workspace()
        config = root / "bench/main_processor/simulation/testdata/robot_soak.toml"
        robot, subsystem_files = support.load_config(config)
        timeout_ms = robot["command_timeout_us"] / 1000
        period_ms = {}
        owner_timeout = {}
        for name in OWNERS:
            sub = support.tomllib.loads(subsystem_files[name].read_text())
            period_ms[name] = sub["subsystem"].get("period_us", robot["period_us"]) / 1000
            owner_timeout[name] = min(max(2 * period_ms[name], timeout_ms / 2), timeout_ms)
        layout = support.device_layout(config)
        motor_ids = {name: [i for i, sub, _, kind in layout
                            if sub == name and kind == "motors"] for name in OWNERS}
        drive_ids = [i for i, sub, name, kind in layout
                     if sub == DRIVE and kind == "motors" and name == "drive"]
        self.assertEqual(len(drive_ids), 1, layout)
        self.assertEqual(len(motor_ids["motor"]), 1, layout)
        self.assertEqual(len(motor_ids[PY_MOTOR]), 1, layout)

        teleop1_cycles, auto_cycles, teleop2_cycles = mode_legs(soak_s)
        schedule, duration, cycle_starts = mode_schedule(
            teleop1_cycles, auto_cycles, teleop2_cycles)
        order = fault_order(teleop1_cycles, auto_cycles, teleop2_cycles, rng)

        gw_port, bridge_port = support.udp_ports()
        tmp = Path(tempfile.mkdtemp(prefix="talos-soak-",
                                    dir=os.environ.get("TEST_UNDECLARED_OUTPUTS_DIR")))
        output = tmp / "sessions"
        env = support.launcher_env(TALOS_SHM_NAMESPACE=f"soak-{uuid.uuid4().hex}")
        print(f"soak: {soak_s:.0f} s enabled ({schedule}), seed {seed}, "
              f"TALOS_SHM_NAMESPACE={env['TALOS_SHM_NAMESPACE']}", flush=True)
        launcher = None
        stopped = None
        success = False
        previous_sigterm = signal.signal(signal.SIGTERM, _sigterm_to_exit)
        try:
            (tmp / "bazel-bin").symlink_to(root, target_is_directory=True)
            args = [str(root / "talOS/launcher/launcher"), "--config", str(config),
                    "--sim", "--start-sim-gateway", "--start-studio-bridge",
                    "--studio-http-port", "0",
                    "--remote", "127.0.0.1", "--remote-port", str(gw_port),
                    "--local-port", str(bridge_port), "--output-dir", str(output),
                    "--duration-s", f"{duration:.0f}"]
            gateway_args = [
                "--physics",
                "--mode-schedule", schedule,
                "--joystick-schedule",
                joystick_schedule(teleop1_cycles, auto_cycles, teleop2_cycles)]
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
                manifest_path = None
                schedule_start_ns = None
                while time.monotonic() < deadline:
                    manifests = list(output.glob("*/manifest.json"))
                    if manifests:
                        manifest_path = manifests[0]
                        nodes = self.read_nodes(manifest_path)
                        start_path = manifest_path.parent / "sim_schedule_start"
                        # Wait for the launcher's sim_schedule_start anchor
                        # (written atomically once every non-observer node's
                        # loop has started) so fault injection shares
                        # sim_gateway's exact node-clock time zero.
                        if (nodes.get("sim_gateway", {}).get("pid", 0) > 0 and
                                start_path.is_file()):
                            schedule_start_ns = int(start_path.read_text().strip())
                            break
                    self.assertIsNone(launcher.poll(), "launcher exited during startup")
                    time.sleep(0.01)
                else:
                    self.fail("gateway schedule never started")

                for cycle, (start_s, (action, name)) in enumerate(
                        zip(cycle_starts, order)):
                    at_ns = schedule_start_ns + round((start_s + FAULT_AT_S) * 1e9)
                    while True:
                        remaining_s = (at_ns - support.node_clock_ns()) / 1e9
                        if remaining_s <= 0:
                            break
                        time.sleep(remaining_s)
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
            self.assertIn(AUTO_PRODUCER, final, "auto_producer missing from manifest")
            if auto_cycles >= 2:
                self.assertGreaterEqual(
                    kills.get(AUTO_PRODUCER, 0), 1,
                    "auto_producer was never killed during autonomous leg")
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
            drive_ds = []  # DRIVE's received Driver Station views: (now, flags, run)
            lane = f"/{DRIVE}/target/{PRODUCER}"
            empty_logs = {name: {} for name in OWNERS}  # run: its files
            for name in OWNERS:
                topics = f"/hw/request/{name}"
                if name == DRIVE:
                    topics += ",/hw/state/driver_station"
                for run, logs in support.run_logs(session, final[name]):
                    origin = support.log_baseline_ns(logs)
                    if origin is None:
                        # Killed before it recorded anything.
                        empty_logs[name][run] = logs
                        continue
                    shift = (origin - baseline) / 1e6
                    for row in self.dump(root, logs, name, topics):
                        topic, now = row["topic"], row["now_ms"] + shift
                        if topic == f"/hw/request/{name}" and row["kind"] == "SEND":
                            requests[name].append((now, support.motor_requests(row),
                                                   row["event_ms"] + shift, run))
                        elif (name == DRIVE and
                              topic == "/hw/state/driver_station" and
                              row["kind"] == "MESSAGE"):
                            drive_ds.append((now, support.ds_snapshot(row)[0], run))
            for rows in (*requests.values(), drive_ds):
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

            if auto_cycles > 0:
                # Verify system-level mode-generation handoff across
                # teleop -> disabled -> autonomous -> disabled -> teleop:
                # all three actuator owners (two 5 ms C++ owners and the 20 ms
                # Python owner) actuate from auto_producer during autonomous
                # (with no teleop leakage) and from teleop in both teleop legs
                # (with no auto leakage), sustained across the leg minus faults.
                auto_ds = [t for t, f, _ in drive_ds
                           if (f & DS_ENABLED) and (f & DS_AUTONOMOUS)]
                self.assertTrue(auto_ds, "no enabled autonomous Driver Station packets")
                auto_start_ms, auto_end_ms = auto_ds[0], auto_ds[-1]
                self.assertGreaterEqual(
                    auto_end_ms - auto_start_ms, auto_cycles * CYCLE_S * 500,
                    "autonomous leg shorter than expected")
                teleop1_ds = [t for t, f, _ in drive_ds
                              if t < auto_start_ms and (f & DS_ENABLED) and (f & DS_TELEOP)]
                teleop2_ds = [t for t, f, _ in drive_ds
                              if t > auto_end_ms and (f & DS_ENABLED) and (f & DS_TELEOP)]
                self.assertTrue(teleop1_ds, "no enabled teleop packets before autonomous")
                self.assertTrue(teleop2_ds, "no enabled teleop packets after autonomous")

                owner_check = {
                    DRIVE: (
                        drive_ids[0],
                        lambda req: req[0] == "velocity" and abs(abs(req[2]) - AUTO_DRIVE_RPS) < 0.1,
                        lambda req: req[0] == "velocity" and abs(abs(req[2]) - TELEOP_DRIVE_RPS) < 0.1,
                    ),
                    "motor": (
                        motor_ids["motor"][0],
                        lambda req: req[0] == "velocity" and abs(req[2] - AUTO_MOTOR_RPS) < 0.1,
                        lambda req: req[0] == "voltage" and abs(req[2] - TELEOP_MOTOR_V) < 0.1,
                    ),
                    PY_MOTOR: (
                        motor_ids[PY_MOTOR][0],
                        lambda req: req[0] == "velocity" and abs(req[2] - AUTO_MOTOR_RPS) < 0.1,
                        lambda req: req[0] == "voltage" and abs(req[2] - TELEOP_MOTOR_V) < 0.1,
                    ),
                }

                def min_active_hits(owner, span, driving_fraction, leg_producer):
                    """Floor on expected active requests over `span`: half of the
                    driving ticks remaining after conservatively budgeting for
                    faults in `span` that can pause `owner`."""
                    affected = {owner, leg_producer, "hardware_node"}
                    fault_ms = 0.0
                    for action, target, s_ms, _ in events:
                        if not (span[0] <= s_ms <= span[1]):
                            continue
                        if leg_producer == PRODUCER and (
                                action == "frozen_kill"
                                or target in (PRODUCER, "hardware_node")):
                            # A 300 ms SIGSTOP (> 100 ms command_timeout_us) or
                            # SIGKILL of teleop or hardware_node disarms
                            # OperatorArming while deflected at FAULT_AT_S (1.8 s),
                            # leaving actuation off until next cycle's CENTER_S
                            # (CYCLE_S - FAULT_AT_S = 1200 ms).
                            fault_ms += 1500.0
                        elif target in affected:
                            fault_ms += 1500.0 if action in ("kill", "frozen_kill") else 500.0
                    driving_ms = (span[1] - span[0]) * driving_fraction
                    expected = max(0.0, driving_ms - fault_ms) / period_ms[owner]
                    return max(20, int(0.5 * expected))

                teleop_fraction = (CYCLE_S - CENTER_S) / CYCLE_S
                leg_spans = (
                    ("teleop1", (teleop1_ds[0], teleop1_ds[-1]), False, teleop_fraction, PRODUCER),
                    ("auto", (auto_start_ms, auto_end_ms), True, 1.0, AUTO_PRODUCER),
                    ("teleop2", (teleop2_ds[0], teleop2_ds[-1]), False, teleop_fraction, PRODUCER),
                )
                leg_counts = {}
                for leg, span, is_auto, frac, leg_producer in leg_spans:
                    counts = {}
                    for owner in OWNERS:
                        dev_id, auto_pred, teleop_pred = owner_check[owner]
                        active_pred = auto_pred if is_auto else teleop_pred
                        leak_pred = teleop_pred if is_auto else auto_pred
                        leg_reqs = [r[1][dev_id] for r in requests[owner]
                                    if span[0] <= r[0] <= span[1]]
                        hits = sum(active_pred(m) for m in leg_reqs)
                        floor = min_active_hits(owner, span, frac, leg_producer)
                        self.assertGreaterEqual(
                            hits, floor,
                            f"{owner}: expected >= {floor} active requests during "
                            f"{leg}, got {hits} (of {len(leg_reqs)})")
                        self.assertFalse(
                            any(leak_pred(m) for m in leg_reqs),
                            f"{owner}: wrong-mode target leaked into {leg} leg")
                        counts[owner] = hits
                    leg_counts[leg] = counts
                summary = ", ".join(
                    f"{leg} (" + ", ".join(f"{o}={leg_counts[leg][o]}" for o in OWNERS) + ")"
                    for leg, *_ in leg_spans)
                print(f"mode handoff: {summary} active requests", flush=True)

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
                if action != "frozen_kill":
                    continue
                what = f"{action} {name} at {s:.1f} ms"

                def commands(since, until):
                    rows = [(r["now_ms"], support.motor_requests(r)) for r in self.dump(
                        root, session, "hardware_node", "/hw/command", since, until)
                        if r["kind"] == "SEND"]
                    return sorted(rows, key=lambda r: r[0])

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
                enabled = [t for t, flags, r in drive_ds
                           if r == run and (flags & DS_ENABLED) and t < e]
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
