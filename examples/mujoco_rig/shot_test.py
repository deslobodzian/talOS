"""e2e: the rig's own nodes shoot its preloads into the blue HUB.

A launcher session of main_processor/configuration/robot_sim.toml against
the MuJoCo gateway, without --allow-graph-errors: disabled, then TELEOP; the
sticks neutral first (teleop's OperatorArming arms), then button 3 held (the
flywheel spins up), then buttons 3 and 4 held (the feeder runs). The match is
read from the session's own record: telemetry's event log holds
/sim/state/match, the hardware bridge's holds /hw/state.

Nothing here assumes the host keeps real time. The gateway drops plant time
when the host is slow, so the session ends on the record, not the clock:
once every preload has scored, or the schedule clock (which the match
periods run on) is far enough past the last launch, or past the end of
SHIFT 1. The launcher's --duration-s (Bazel's timeout less what the stop
needs), READY_TIMEOUT_S and LOG_DUMP_TIMEOUT_S are wall-clock backstops
against a hang only; none of them decides a pass.
"""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import unittest

from python.runfiles import runfiles

from talOS.build import sim_check

# The operator's schedule (README.md). The rig's place on the field is the
# config's [simulation.start_pose], not passed here: a config without it
# starts the rig at (2, 2, 0), where no shot scores.
MODE_SCHEDULE = "disabled:1,teleop:60"
SPIN_BUTTONS = 1 << 2  # button 3: the flywheel to the shot's speed
FEED_BUTTONS = SPIN_BUTTONS | 1 << 3  # and button 4: the feeder
JOYSTICK_SCHEDULE = "|".join([
    "0:axes=0;buttons=0;pov=-1",
    f"2:axes=0;buttons={SPIN_BUTTONS};pov=-1",
    f"4:axes=0;buttons={FEED_BUTTONS};pov=-1",
])
PRELOADS = 8
# Every one of 90 measured runs of the scene without processes scored all 8
# (README.md "The shot"); one piece of margin, the twin's bound
# (rig_shot_test).
MIN_POINTS = 7
# Schedule-clock seconds: after the last launch, for its flight; and the
# latest the session runs, inside SHIFT 1 (TELEOP starts at 1 s, TRANSITION
# lasts 10 s, SHIFT 1 25 s), when seed 1's draw keeps the blue HUB counting.
FLIGHT_S = 3.0
LATEST_S = 30.0
POLL_S = 0.5
READY_TIMEOUT_S = 60
LOG_DUMP_TIMEOUT_S = 30
MATCH = "/sim/state/match"
# The clock the gateway's schedule start file is read on (talOS/events).
NODE_CLOCK = (time.CLOCK_MONOTONIC_RAW if sys.platform == "darwin"
              else time.CLOCK_MONOTONIC)


def rlocation(path):
    found = runfiles.Create().Rlocation(path, source_repo="")
    if not found or not os.path.exists(found):
        raise FileNotFoundError(f"missing runfile: {path}")
    return found


def start_schedule(path):
    pending = path + ".pending"
    with open(pending, "w", encoding="utf-8") as f:
        f.write(str(time.clock_gettime_ns(NODE_CLOCK)))
    os.rename(pending, path)


def dump(log_dump, inputs, topic, node):
    """The topic's records in `node`'s event log, decoded (log_dump). While
    the session runs, the log's last record may be partly written: log_dump
    stops before it."""
    if isinstance(inputs, (str, os.PathLike)):
        inputs = [str(inputs)]
    proc = subprocess.run(
        [log_dump, "--format", "jsonl", "--node", node, "--topic", topic,
         "--", *inputs],
        capture_output=True, text=True, timeout=LOG_DUMP_TIMEOUT_S,
        check=False)
    if proc.returncode != 0:
        raise RuntimeError(f"log_dump exited {proc.returncode}: {proc.stderr}")
    return [json.loads(line) for line in proc.stdout.splitlines()
            if line.startswith("{")]


def fields(record):
    """log_dump's decoded struct, `{a: 1, b: true}`, as a dict of strings."""
    out = {}
    body = record["decoded"].strip("{}").strip()
    if not body:
        return out
    for item in body.split(", "):
        key, value = item.split(": ", 1)
        out[key] = value
    return out


def matches(log_dump, session, live=False):
    if live:
        # While the session runs, read only telemetry's .tlog files (skipping
        # the 200 Hz hardware_node log) and treat an absent log as no records
        # yet instead of failing log_dump before the writer creates it.
        telemetry_logs = sorted(
            str(p) for p in Path(session).glob("telemetry*.tlog"))
        if not telemetry_logs:
            return []
        inputs = telemetry_logs
    else:
        inputs = [session]
    return [fields(r) for r in dump(log_dump, inputs, MATCH, "telemetry")
            if r["kind"] == "MESSAGE"]


def finished(match_log):
    """Whether the record shows the shot over: every preload scored; or
    every one launched and FLIGHT_S of schedule time since; or LATEST_S."""
    if not match_log:
        return False
    last = match_log[-1]
    now_s = int(last["schedule_time_us"]) * 1e-6
    if int(last["blue_scored_count"]) >= PRELOADS or now_s >= LATEST_S:
        return True
    launched = [m for m in match_log if int(m["shots_count"]) >= PRELOADS]
    return bool(launched) and (
        now_s - int(launched[0]["schedule_time_us"]) * 1e-6 >= FLIGHT_S)


class ShotTest(unittest.TestCase):
    # REQ-DEV-37, REQ-DEV-39, REQ-DEV-40, REQ-DEV-41: a robot's own nodes,
    # through the launcher, the bridge and its target lanes, command the
    # MuJoCo gateway's generated model, whose scripted shooter launches the
    # preloads the field's game placed, and the game scores them.
    def test_rig_scores_its_preloads_in_teleop(self):
        launcher = rlocation(os.environ["LAUNCHER"])
        log_dump = rlocation(os.environ["LOG_DUMP"])
        registry_unlink = rlocation(os.environ["REGISTRY_UNLINK"])
        report = sim_check.load_session_report(
            rlocation(os.environ["SESSION_REPORT"]))
        config = rlocation(os.environ["CONFIG"])
        root = Path(launcher).parents[2]
        # The launcher's own --duration-s: the one wall-clock bound, so a
        # session the record never ends stops before Bazel's timeout.
        timeout_s = int(os.environ.get("TEST_TIMEOUT") or 300)
        backstop_s = timeout_s - sim_check.HANG_MARGIN_S
        if backstop_s < sim_check.MIN_READY_S:
            raise ValueError(
                f"TEST_TIMEOUT={timeout_s} leaves {backstop_s} s after "
                f"{sim_check.HANG_MARGIN_S} s stop margin")
        ready_timeout_s = min(READY_TIMEOUT_S, backstop_s)

        # Bazel ends a test at its timeout with SIGTERM: unwind through the
        # with/finally blocks so Launch stops its session, registry_unlink
        # runs, and failure artifacts are preserved.
        previous_sigterm = signal.signal(
            signal.SIGTERM, lambda *_: sys.exit(128 + signal.SIGTERM))
        try:
            with tempfile.TemporaryDirectory(
                    dir=os.environ.get("TEST_TMPDIR")) as tmp:
                work = os.path.join(tmp, "work")
                os.makedirs(work)
                os.symlink(str(root), os.path.join(work, "bazel-bin"))
                output = os.path.join(tmp, "sessions")
                os.makedirs(output)
                log = os.path.join(tmp, "launch.log")
                start_file = os.path.join(tmp, "schedule_start")
                # Isolate shared-memory segments and lock files into this
                # run's temp directory even when TEST_TMPDIR is unset or
                # shared outside Bazel.
                env = os.environ.copy()
                env.pop("TALOS_SHM_NAMESPACE", None)
                env["TEST_TMPDIR"] = tmp

                gateway_args = [
                    "--mode-schedule", MODE_SCHEDULE,
                    "--joystick-schedule", JOYSTICK_SCHEDULE,
                    "--schedule-start-file", start_file,
                ]
                # Reserve and release UDP ports immediately before Launch to
                # narrow the bind TOCTOU window (docs/TESTING.md §6.4).
                gateway_port, bridge_port = sim_check.udp_ports()
                args = [
                    launcher, "--config", config, "--sim",
                    "--start-sim-gateway",
                    "--remote", "127.0.0.1",
                    "--remote-port", str(gateway_port),
                    "--local-port", str(bridge_port),
                    "--output-dir", output,
                    "--duration-s", str(backstop_s),
                ]
                for arg in gateway_args:
                    args += ["--sim-gateway-arg", arg]

                failed = True
                try:
                    sessions = []
                    session = None
                    match_log = []
                    try:
                        with open(log, "wb") as out, \
                                sim_check.Launch(args, out, cwd=work,
                                                 env=env) as launch:
                            unready = sim_check.wait_ready(
                                launch, output, ready_timeout_s)
                            ended = launch.process.poll()
                            if not unready and ended is None:
                                sessions = [
                                    os.path.join(output, d)
                                    for d in os.listdir(output)
                                    if os.path.isdir(os.path.join(output, d))
                                ]
                                if len(sessions) == 1:
                                    session = sessions[0]
                                    start_schedule(start_file)
                                    while launch.process.poll() is None:
                                        match_log = matches(
                                            log_dump, session, live=True)
                                        if finished(match_log):
                                            break
                                        try:
                                            launch.process.wait(timeout=POLL_S)
                                        except subprocess.TimeoutExpired:
                                            pass
                                ended = launch.process.poll()
                            in_order, killed = launch.finish()
                    finally:
                        subprocess.run([registry_unlink], env=env, check=False)

                    with open(log, encoding="utf-8", errors="replace") as f:
                        launch_output = f.read()
                    diagnostics = (
                        f"{' '.join(args)}\nports {gateway_port}, "
                        f"{bridge_port} (released before the launch; "
                        "another process may have taken one)\n"
                        f"launch output:\n{launch_output}"
                    )
                    self.assertEqual(unready, "", diagnostics)
                    if ended is not None and ended != 0:
                        self.fail(
                            f"launch exited early ({ended}):\n{diagnostics}")
                    self.assertTrue(in_order, diagnostics)
                    self.assertEqual(killed, [], diagnostics)
                    self.assertEqual(launch.process.returncode, 0, diagnostics)
                    self.assertEqual(
                        len(sessions), 1,
                        f"expected one session directory, found {sessions}\n"
                        f"{diagnostics}")
                    self.assertTrue(
                        finished(match_log),
                        f"the shot did not end before the launch's "
                        f"--duration-s: {match_log[-1:]}\n"
                        f"{diagnostics}")
                    self.assertIsNone(
                        ended,
                        f"launch exited early ({ended}):\n{diagnostics}")
                    self.assertEqual(
                        sim_check.judge(session, log_dump, report), [],
                        diagnostics)

                    final_matches = matches(log_dump, session)
                    self.assertTrue(
                        final_matches,
                        f"no {MATCH} records in session\n{diagnostics}")
                    last = final_matches[-1]
                    summary = (f"{last}\n" + "\n".join(
                        line for line in launch_output.splitlines()
                        if "mujoco_gateway:" in line))
                    self.assertEqual(last["period"], "Teleop", summary)
                    self.assertEqual(
                        int(last["shots_count"]), PRELOADS, summary)
                    self.assertGreaterEqual(
                        int(last["blue_points_count"]), MIN_POINTS, summary)

                    # Each beam break closed (read false: sim_closed_reads is
                    # false) at least once, and was open at least once.
                    readings = {}
                    for record in dump(log_dump, session, "/hw/state",
                                       "hardware_node"):
                        if record["kind"] != "SEND":
                            continue
                        for beam in record["hardware"]["digital_inputs"]:
                            readings.setdefault(beam["name"], set()).add(
                                beam["value"])
                    self.assertEqual(readings, {"feeder_beam": {False, True},
                                                "shooter_beam": {False, True}})
                    failed = False
                finally:
                    if failed:
                        kept = sim_check.preserve_failure_artifacts(
                            log, output)
                        if kept:
                            print(f"failure artifacts preserved: {kept}",
                                  file=sys.stderr)
        finally:
            signal.signal(signal.SIGTERM, previous_sigterm)


if __name__ == "__main__":
    unittest.main()
