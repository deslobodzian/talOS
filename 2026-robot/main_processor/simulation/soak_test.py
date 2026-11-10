"""Soak the full simulated graph for minutes while killing and stalling nodes.

The scripted Driver Station holds teleop for TALOS_SOAK_SECONDS (default 180,
at most 600 so the run fits Bazel's long timeout)
with the stick centred for 0.5 s at the start of every 3 s cycle and driving
the rest. Each cycle, 1.8 s in, one fault is injected into a running process,
in a seeded order (TALOS_SOAK_SEED, default 0) that cycles through every kind:
SIGKILL (restarted by the launcher) or a 300 ms SIGSTOP (below the 500 ms
heartbeat floor, so the launcher must not restart it). The Studio bridge is
launched too and only stalled: the launcher never restarts a stable bridge.

Checked from the recorded session, over the whole run:
- every actuator owner's /hw/request gaps stay within its bridge owner
  timeout except across a fault injected into that node, and a killed owner
  sends again within RESTART_LIMIT_MS;
- no request actuates while that node's latest Driver Station view is
  disabled, and no drivetrain request actuates without an active teleop target
  published after its latest crash and within command_timeout + one period;
- a stalled driver or hardware bridge neutralizes the drivetrain's own
  requests within the stale-input bound, and a stalled drivetrain is
  neutralized by the bridge's /hw/command within its owner timeout;
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
import socket
import subprocess
import tempfile
import time
import unittest
import uuid

import autonomous_progress
import startup_test as support

CYCLE_S = 3.0
CENTER_S = 0.5
FAULT_AT_S = 1.8
STALL_S = 0.3
LEAD_S = 2.0  # disabled before and after teleop
RESTART_LIMIT_MS = 5000
# Teleop length cap: with startup, shutdown and log analysis the run stays
# inside Bazel's "long" timeout (900 s).
MAX_SOAK_S = 600

# (action, node). Kills of non-critical telemetry stay under max_restarts (5)
# per rolling minute: one kind in this list runs once per len(FAULTS) cycles.
FAULTS = [
    ("stop", "driver"),
    ("kill", "drivetrain"),
    ("stop", "hardware_node"),
    ("kill", "telemetry"),
    ("stop", "drivetrain"),
    ("stop", "studio_bridge"),
    ("kill", "driver"),
    ("stop", "telemetry"),
    ("kill", "intake"),
    ("kill", "hardware_node"),
    ("stop", "shooter"),
    ("kill", "teleop"),
]
OWNERS = ("drivetrain", "intake", "shooter")
NEUTRAL = (0, 0, 0.0, 0.0)


def tcp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(("127.0.0.1", 0))
        return probe.getsockname()[1]


def joystick_schedule(teleop_s):
    entries = []
    cycle = 0
    while cycle * CYCLE_S < teleop_s:
        start = LEAD_S + cycle * CYCLE_S
        entries.append(f"{start:.3f}:axes=0,0,0;buttons=0;pov=-1")
        entries.append(f"{start + CENTER_S:.3f}:axes=0,-0.6,0;buttons=5;pov=-1")
        cycle += 1
    entries.append(f"{LEAD_S + teleop_s:.3f}:axes=0,0,0;buttons=0;pov=-1")
    return "0:axes=0,0,0;buttons=0;pov=-1|" + "|".join(entries)


class Soak(unittest.TestCase):
    def read_nodes(self, manifest_path):
        return {n["name"]: n for n in json.loads(manifest_path.read_text())["nodes"]}

    def dump(self, root, session, nodes, topics, since=None, until=None):
        args = [str(root / "talOS/launcher/log_dump"), "--format", "jsonl",
                "--node", nodes, "--topic", topics]
        if since is not None:
            args += ["--since-ms", f"{since:.3f}", "--until-ms", f"{until:.3f}"]
        with subprocess.Popen(args + [str(session)], stdout=subprocess.PIPE,
                              text=True) as proc:
            for line in proc.stdout:
                yield json.loads(line)
        self.assertEqual(proc.returncode, 0, "log_dump failed")

    def neutral_by(self, rows, start, deadline, end, motor_ids, gap_ms, what):
        """rows: sorted (now_ms, motors). Neutral from some row <= deadline to end."""
        window = [r for r in rows if start <= r[0] < end]
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

    def driving(self, rows, start, end, drive_ids, what):
        window = [m for t, m in rows if start <= t < end]
        self.assertGreater(len(window), 2, f"{what}: no outputs in [{start:.1f}, {end:.1f})")
        self.assertTrue(all(m[i][0] == 3 and abs(m[i][2]) > 0.1
                            for m in window for i in drive_ids),
                        f"{what}: not driving in [{start:.1f}, {end:.1f})")

    def test_graph_survives_kills_and_stalls(self):
        teleop_s = float(os.environ.get("TALOS_SOAK_SECONDS", "180"))
        self.assertTrue(CYCLE_S <= teleop_s <= MAX_SOAK_S,
                        f"TALOS_SOAK_SECONDS must be in [{CYCLE_S:.0f}, {MAX_SOAK_S}]")
        seed = int(os.environ.get("TALOS_SOAK_SEED", "0"))
        print(f"soak: {teleop_s:.0f} s teleop, seed {seed}", flush=True)
        rng = random.Random(seed)
        root = support.workspace()
        config_dir = root / "2026-robot/main_processor/configuration"
        config = config_dir / "robot_physics_sim.toml"
        robot = support.tomllib.loads((config_dir / "robot.toml").read_text())
        timeout_ms = robot["robot"]["command_timeout_us"] / 1000
        period_ms = robot["robot"]["period_us"] / 1000
        # A consumer tick past the timeout (ARCHITECTURE.md "The tick").
        stale_bound = timeout_ms + period_ms
        owner_timeout = {}
        for name in OWNERS:
            sub = support.tomllib.loads(
                (config_dir / "subsystems" / f"{name}.toml").read_text())
            period = sub["subsystem"].get("period_us", robot["robot"]["period_us"]) / 1000
            owner_timeout[name] = min(max(2 * period, timeout_ms / 2), timeout_ms)
        layout = support.device_layout(config_dir / "robot.toml")
        motor_ids = {name: [i for i, sub, _, kind in layout
                            if sub == name and kind == "motors"] for name in OWNERS}
        drive_ids = [i for i, sub, name, kind in layout
                     if sub == "drivetrain" and kind == "motors" and name.endswith("_drive")]

        cycles = int(teleop_s // CYCLE_S)
        order = []
        while len(order) < cycles:
            batch = list(FAULTS)
            rng.shuffle(batch)
            order.extend(batch)
        order = order[:cycles]

        gw_port, bridge_port = support.udp_ports()
        tmp = Path(tempfile.mkdtemp(prefix="talos-soak-",
                                    dir=os.environ.get("TEST_UNDECLARED_OUTPUTS_DIR")))
        output = tmp / "sessions"
        env = dict(os.environ, TALOS_SHM_NAMESPACE=f"soak-{uuid.uuid4().hex}")
        launcher = None
        stopped = None
        success = False
        try:
            (tmp / "bazel-bin").symlink_to(root, target_is_directory=True)
            duration = 2 * LEAD_S + teleop_s
            args = [str(root / "talOS/launcher/launcher"), "--config", str(config),
                    "--sim", "--start-sim-gateway", "--start-studio-bridge",
                    "--studio-http-port", str(tcp_port()),
                    "--remote", "127.0.0.1", "--remote-port", str(gw_port),
                    "--local-port", str(bridge_port), "--output-dir", str(output),
                    "--duration-s", f"{duration:.0f}"]
            gateway_args = [
                "--physics", "--port", str(gw_port), "--remote-port", str(bridge_port),
                "--mode-schedule",
                f"disabled:{LEAD_S:.0f},teleop:{teleop_s:.0f},disabled:{LEAD_S:.0f}",
                "--joystick-schedule", joystick_schedule(teleop_s)]
            for arg in gateway_args:
                args.extend(["--sim-gateway-arg", arg])
            events = []  # (action, node, start_ns, end_ns)
            kills = {}
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
                    pid = self.read_nodes(manifest_path).get(name, {}).get("pid", 0)
                    self.assertGreater(pid, 0, f"{name} not running at cycle {cycle}")

                    def send(sig):
                        # A pid the manifest still lists but that has exited
                        # is a failure of the run, named, not a skip.
                        try:
                            os.kill(pid, sig)
                        except ProcessLookupError:
                            self.fail(f"{name} (pid {pid}) gone at cycle {cycle} "
                                      f"({action}, {signal.Signals(sig).name})")

                    if action == "kill":
                        send(signal.SIGKILL)
                        # Stamped after the kill: the crash bound for the
                        # fresh-target check must not precede a request the
                        # dying process sent just before it.
                        start_ns = end_ns = support.node_clock_ns()
                        kills[name] = kills.get(name, 0) + 1
                    else:
                        # Stamped before the stop: the neutral deadline
                        # counts from no later than the stall's start.
                        start_ns = support.node_clock_ns()
                        stopped = pid
                        send(signal.SIGSTOP)
                        time.sleep(STALL_S)
                        end_ns = support.node_clock_ns()
                        send(signal.SIGCONT)
                        stopped = None
                    events.append((action, name, start_ns, end_ns))
                    print(f"cycle {cycle}: {action} {name}", flush=True)
                launcher.wait(timeout=duration + 30)
                log.seek(0)
                diagnostics = log.read()
            self.assertEqual(launcher.returncode, 0, diagnostics[-20000:])

            session = manifest_path.parent
            final = self.read_nodes(manifest_path)
            for name, node in final.items():
                self.assertEqual(node["exit_code"], 0, name)
                self.assertFalse(node["quarantined"], name)
                self.assertEqual(node["restart_count"], kills.get(name, 0), name)

            baseline = support.log_baseline_ns(session)
            events = [(a, n, (s - baseline) / 1e6, (e - baseline) / 1e6)
                      for a, n, s, e in events]
            requests = {name: [] for name in OWNERS}
            ds = {name: [] for name in OWNERS}
            targets = []
            for row in self.dump(root, session, ",".join(OWNERS),
                                 "/hw/request/*,/hw/state/driver_station,"
                                 "/drivetrain/target/teleop"):
                name, topic = row["process"], row["topic"]
                if topic == f"/hw/request/{name}" and row["kind"] == "SEND":
                    requests[name].append((row["now_ms"], support.motor_requests(row)))
                elif topic == "/hw/state/driver_station" and row["kind"] == "MESSAGE":
                    ds[name].append((row["now_ms"], support.ds_snapshot(row)[0] & 4))
                elif (name == "drivetrain" and topic == "/drivetrain/target/teleop" and
                      row["kind"] == "MESSAGE" and
                      bytes.fromhex(row["payload_hex"])[8] != 0):
                    targets.append((row["now_ms"], row["publish_ms"]))
            for rows in (*requests.values(), *ds.values(), targets):
                rows.sort(key=lambda r: r[0])

            for name in OWNERS:
                rows = requests[name]
                self.assertGreater(len(rows), 100, name)
                times = [t for t, _ in rows]
                own = [(s, e) for _, n, s, e in events if n == name]
                # Gaps, excluding the span from a fault on this node to its
                # next request.
                for (a, _), (b, _) in zip(rows, rows[1:]):
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
                # Disabled Driver Station view -> neutral request.
                views = ds[name]
                view_times = [t for t, _ in views]
                for t, motors in rows:
                    index = bisect.bisect_right(view_times, t) - 1
                    if index < 0 or not views[index][1]:
                        self.assertTrue(all(m == NEUTRAL for m in motors.values()),
                                        f"{name} active while disabled at {t:.1f} ms")

            # No drivetrain actuation without a fresh, post-crash active target.
            target_times = [t for t, _ in targets]
            crashes = sorted(s for a, n, s, _ in events
                             if a == "kill" and n == "drivetrain")
            for t, motors in requests["drivetrain"]:
                if all(m[0] == 0 for m in motors.values()):
                    continue
                crash = crashes[bisect.bisect_right(crashes, t) - 1] if (
                    crashes and crashes[0] <= t) else float("-inf")
                j = bisect.bisect_right(target_times, t) - 1
                fresh = False
                while j >= 0 and targets[j][0] >= t - stale_bound:
                    if t - targets[j][1] <= stale_bound and targets[j][1] > crash:
                        fresh = True
                        break
                    j -= 1
                self.assertTrue(fresh, f"drivetrain active without fresh target at {t:.1f} ms")

            drive_rows = requests["drivetrain"]
            for action, name, s, e in events:
                what = f"{action} {name} at {s:.1f} ms"
                if action != "stop":
                    continue
                if name == "driver":
                    self.driving(drive_rows, s - 150, s, drive_ids, what)
                    self.neutral_by(drive_rows, s, s + stale_bound, e,
                                    motor_ids["drivetrain"], owner_timeout["drivetrain"], what)
                elif name == "hardware_node":
                    # The last State can be one gateway tick old when the
                    # bridge stops; the node then needs a tick to notice.
                    self.driving(drive_rows, s - 150, s, drive_ids, what)
                    self.neutral_by(drive_rows, s, s + stale_bound + period_ms, e,
                                    motor_ids["drivetrain"], owner_timeout["drivetrain"], what)
                elif name == "drivetrain":
                    commands = [(r["now_ms"], support.motor_requests(r)) for r in self.dump(
                        root, session, "hardware_node", "/hw/command", s - 300, e + 300)
                        if r["kind"] == "SEND"]
                    commands.sort(key=lambda r: r[0])
                    self.driving(commands, s - 150, s, drive_ids, what)
                    # Owner timeout plus the bridge's 1 ms check, with a
                    # consumer period of margin for the stop's delivery.
                    self.neutral_by(commands, s, s + owner_timeout["drivetrain"] + period_ms,
                                    e, motor_ids["drivetrain"], owner_timeout["drivetrain"],
                                    what)
            success = True
        finally:
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
            autonomous_progress.remove(env["TALOS_SHM_NAMESPACE"])
            if success:
                shutil.rmtree(tmp)
            else:
                print(f"Soak artifacts retained: {tmp}", flush=True)
                launcher_log = tmp / "launcher.txt"
                if launcher_log.exists():
                    print(launcher_log.read_text(errors="replace")[-20000:], flush=True)


if __name__ == "__main__":
    unittest.main()
