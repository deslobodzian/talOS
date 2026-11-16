"""e2e: a robot scaffolded with a C++ velocity mechanism (gripper) and a
position mechanism (position_cpp), edited exactly as their `Next:` output
says and run with its schedules (the launcher with sim_gateway --physics
and its mode and joystick schedules, as `./sim.sh --sim-gateway` starts
them; the test supplies the schedules' start itself, below), moves each
motor to the goal its teleop binding holds while the button its `Next:`
names is held, the position mechanism after homing on its simulated home
switch. The joystick schedule holds each mechanism's button in turn, so it
also shows that position_cpp's button does not move the gripper.
The node binaries are new_subsystem's goldens, which
new_subsystem_test keeps equal to its output.

A robot scaffolded with --no-mechanism runs the same way: every member
exits 0, and the bridge's last /hw/status is configured with no alert,
reject or fault.

The test times the match itself (sim_gateway's --schedule-start-file)
rather than with the `--duration-s 8` the printed command passes, which
counts from the launcher's first spawn while the schedules count from the
gateway's start, spawned last: every second of start-up (a sanitizer
build, a loaded host) would come off the last mechanism's window (TODO.md
TOOLS-18). Once every member's loop has started and the bridge has
accepted a State (talOS/build/sim_check.py wait_ready), it writes the
schedules' time zero for sim_gateway's --schedule-start-file, and stops the
session with SIGINT, as Ctrl-C does, the schedule's length after it."""

import json
import os
import socket
import subprocess
import sys
import tempfile
import time
import unittest

from _posixshmem import shm_unlink  # what multiprocessing.shared_memory uses
from python.runfiles import runfiles

from talOS.build import sim_check
from talOS.memory.shm_name import shm_name
from tools import new_robot_test as harness
from tools import new_subsystem
from tools import sync_manifest

# position_cpp first: the gripper's check uses when it reached its goal.
MECHANISMS = (("position_cpp", "position"), ("gripper", "velocity"))

# talOS/introspection/registry.h kRegistryPath. A launch in a namespace
# removes the registry once its members are reaped (talOS/launcher/README.md
# "One session per namespace"); one killed before then leaves it, so the
# test unlinks the namespace's after the run.
REGISTRY = "/talos_registry.6"

# Bounds on a hang, not on a run (which lasts SIM_DURATION_S or
# NO_MECHANISM_DURATION_S from the schedules' time zero), so that one hang
# fails with the test's diagnostics instead of being killed at Bazel's 60 s
# `short` timeout. Per case: the wait for the session to be up, then the
# stop: STOP_WAIT_S for the launcher to exit on SIGINT (its members get
# 2 s, launcher.h kStopTimeout), as long again after SIGTERM to its group,
# then EXIT_WAIT_S for its session's processes and sim_check.KILL_WAIT_S
# after SIGKILL. A launcher that neither starts nor stops: 15 + 8 + 8 + 3
# + 5 = 39 s; one that starts and then ignores SIGINT: start-up + 8 + 24
# s. Either, with the other case's ordinary run (about 10 s plain), leaves
# at least 10 s of the timeout on a plain build.
READY_TIMEOUT_S = 15
STOP_WAIT_S = 8
EXIT_WAIT_S = 3
LOG_DUMP_TIMEOUT_S = 5
# The no-mechanism run: past the mode schedule's 1 s disabled spell, so
# the gateway is enabled for the last records.
NO_MECHANISM_DURATION_S = 3
# The launcher's --duration-s: only a backstop for a test killed outright,
# which could stop nothing; it lies past every bound above.
BACKSTOP_S = READY_TIMEOUT_S + new_subsystem.SIM_DURATION_S + 10
# The main-processor node clock sim_gateway reads a schedule start on
# (talOS/simulation/README.md): CLOCK_MONOTONIC_RAW on macOS.
NODE_CLOCK = (time.CLOCK_MONOTONIC_RAW if sys.platform == "darwin"
              else time.CLOCK_MONOTONIC)


def unlink_registry(namespace):
    try:
        shm_unlink(shm_name(REGISTRY, namespace))
    except FileNotFoundError:
        pass


def udp_ports():
    """Two distinct free loopback UDP ports (TESTING.md §6: bound at 0)."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as gateway, \
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as bridge:
        gateway.bind(("127.0.0.1", 0))
        bridge.bind(("127.0.0.1", 0))
        return gateway.getsockname()[1], bridge.getsockname()[1]


def scaffold(work):
    """The robot new_robot writes with the gripper, plus position_cpp from
    new_subsystem, with their node binaries standing in for a build; returns
    each mechanism's button, teleop_button's choice before its scaffold (the
    one its `Next:` prints)."""
    buttons = {}
    harness.generate(work, "gripper", "cpp")
    buttons["gripper"] = new_subsystem.TELEOP_BUTTONS[0]  # a new robot's
    buttons["position_cpp"] = new_subsystem.teleop_button(work, "my_robot")
    writes, _ = new_subsystem.plan(work, "my_robot", "position_cpp", "cpp",
                                   "position")
    for path, _, content in writes:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            f.write(content)
    harness.stand_in_binary(work, "position_cpp", "cpp")
    return buttons


def follow_next(work, buttons):
    """Appends each mechanism's simulation block to the simulation config,
    as its `Next:` says; returns that config, the one `Next:` runs."""
    for mechanism, kind in MECHANISMS:
        harness.append_simulation_block(work, mechanism, kind,
                                        buttons[mechanism])
    return harness.SIM_CONFIG


# Each mechanism's `Next:` holds its own button from second 3 to 6
# (new_subsystem.joystick_schedule). Run together, position_cpp's is held
# over those seconds, its time to home and reach its goal, then only the
# gripper's to the end of the run.
def joystick_schedule(buttons):
    def held(mechanism):
        return f"axes=0;buttons={1 << (buttons[mechanism] - 1)};pov=-1"
    return (f"0:axes=0;buttons=0;pov=-1|3:{held('position_cpp')}"
            f"|6:{held('gripper')}"
            f"|{new_subsystem.SIM_DURATION_S}:axes=0;buttons=0;pov=-1")


def records(log_dump, session, topic, node=None):
    """Every `topic` record in the session's logs (`node`'s alone if
    given), decoded, as their fields."""
    return [fields for _, fields in timed_records(log_dump, session, topic,
                                                  node)]


def states(log_dump, session, mechanism):
    """Every /<mechanism>/state the node published, decoded."""
    return timed_records(log_dump, session, f"/{mechanism}/state", mechanism)


def timed_records(log_dump, session, topic, node):
    """(publish_ms, fields) of every `topic` record log_dump decodes."""
    selection = ["--node", node] if node else []
    dumped = subprocess.run(
        [log_dump, "--format", "jsonl"] + selection + ["--topic", topic,
                                                       session],
        capture_output=True, text=True, timeout=LOG_DUMP_TIMEOUT_S,
        check=True)
    rows = [json.loads(line) for line in dumped.stdout.splitlines()
            if line.startswith("{")]
    decoded = []
    for row in rows:
        fields = {}
        for item in row["decoded"].strip("{}").split(", "):
            key, value = item.split(": ", 1)
            fields[key] = value
        decoded.append((row["publish_ms"], fields))
    return decoded


def start_schedule(path):
    """Writes now on the node clock as the schedules' time zero, atomically
    (by rename), as sim_gateway's --schedule-start-file reads it."""
    pending = path + ".pending"
    with open(pending, "w", encoding="utf-8") as f:
        f.write(str(time.clock_gettime_ns(NODE_CLOCK)))
    os.rename(pending, path)


class ScaffoldSimTest(unittest.TestCase):
    def launch(self, tmp, work, launcher, env, config, gateway_extra,
               duration_s):
        """Runs `config` the way `./sim.sh --sim-gateway` does: the
        launcher starting sim_gateway --physics on its mode schedule, whose
        time zero is when the session is up; stops it `duration_s` after
        that (module docstring). Asserts the launcher stops in order and
        exits 0; returns (session directory, launcher output)."""
        gateway_port, bridge_port = udp_ports()
        # Bazel's TEST_TMPDIR is the shared-memory namespace
        # (ARCHITECTURE.md "Topics and messages") and holds the lock files,
        # which Bazel removes with it; only the registry segment, which a
        # launch killed before its members are reaped leaves, may remain to
        # unlink.
        namespace = os.environ.get("TEST_TMPDIR") or tmp
        env.pop("TALOS_SHM_NAMESPACE", None)
        env["TEST_TMPDIR"] = namespace
        self.addCleanup(unlink_registry, namespace)
        start_file = os.path.join(tmp, "schedule_start")
        gateway_args = [
            "--physics", "--mode-schedule", new_subsystem.MODE_SCHEDULE,
            "--schedule-start-file", start_file] + gateway_extra
        args = [launcher, "--config", config, "--sim", "--start-sim-gateway"]
        for arg in gateway_args:
            args += ["--sim-gateway-arg", arg]
        output = os.path.join(tmp, "sessions")
        args += ["--remote", "127.0.0.1", "--remote-port", str(gateway_port),
                 "--local-port", str(bridge_port), "--output-dir", output,
                 "--duration-s", str(BACKSTOP_S)]
        os.makedirs(output)
        log = os.path.join(tmp, "launch.log")
        with open(log, "wb") as out, \
                sim_check.Launch(args, out, cwd=work, env=env) as launch:
            unready = sim_check.wait_ready(launch, output, READY_TIMEOUT_S)
            ended = launch.process.poll()
            if not unready and ended is None:
                start_schedule(start_file)
                try:
                    ended = launch.process.wait(timeout=duration_s)
                except subprocess.TimeoutExpired:
                    pass
            in_order, killed = launch.finish(STOP_WAIT_S, EXIT_WAIT_S)
        with open(log, encoding="utf-8", errors="replace") as f:
            diagnostics = f.read()
        self.assertEqual(unready, "", diagnostics)
        self.assertIsNone(ended, f"the launch ended on its own (exit "
                          f"{ended}) before the test stopped it: "
                          f"{diagnostics}")
        self.assertTrue(in_order, f"the launch did not stop within "
                        f"{STOP_WAIT_S} s of SIGINT: "
                        f"{diagnostics}")
        self.assertEqual(killed, [], diagnostics)
        self.assertEqual(launch.process.returncode, 0, diagnostics)
        sessions = [d for d in os.listdir(output)
                    if os.path.isdir(os.path.join(output, d))]
        self.assertEqual(len(sessions), 1, os.listdir(output))
        (session,) = sessions
        return os.path.join(output, session), diagnostics

    # REQ-CONTRACT-35
    def test_robot_with_no_mechanism_runs_against_the_simulator(self):
        # new_robot --no-mechanism (bridge, teleop, telemetry, no device)
        # under sim_gateway: every member ends cleanly, and the bridge
        # reports a configured link with no alert, reject or fault.
        r = runfiles.Create()
        log_dump = r.Rlocation("_main/talOS/launcher/log_dump")
        with tempfile.TemporaryDirectory() as tmp:
            work, launcher, env = harness.workspace(tmp)
            harness.generate(work, None, None)
            session, diagnostics = self.launch(
                tmp, work, launcher, env, harness.SIM_CONFIG, [],
                NO_MECHANISM_DURATION_S)
            with open(os.path.join(session, "manifest.json"),
                      encoding="utf-8") as f:
                nodes = json.load(f)["nodes"]
            self.assertEqual(
                sorted(n["name"] for n in nodes),
                ["hardware_node", "sim_gateway", "telemetry", "teleop"])
            for node in nodes:
                for run in node["runs"]:
                    self.assertEqual(
                        (run["exit_code"], run["exit_signal"]), (0, 0),
                        f"{node['name']}: {run} {diagnostics}")
            statuses = records(log_dump, session, "/hw/status")
            self.assertTrue(statuses, f"no /hw/status logged: {diagnostics}")
            last = statuses[-1]
            self.assertEqual(last["configured"], "true", last)
            for field, value in last.items():
                if field.startswith("alert_"):
                    self.assertEqual(value, "false", f"{field}: {last}")
                if field.endswith("rejected_count") or field in (
                        "motor_fault_mask",
                        "gateway_rejected_command_count",
                        "gateway_neutralized_motor_count",
                        "gateway_apply_failure_count",
                        "gateway_device_reset_count"):
                    self.assertEqual(value, "0", f"{field}: {last}")

    # REQ-DEV-01, REQ-DEV-23
    def test_next_steps_move_each_mechanism_to_its_goal(self):
        r = runfiles.Create()
        log_dump = r.Rlocation("_main/talOS/launcher/log_dump")
        with tempfile.TemporaryDirectory() as tmp:
            work, launcher, env = harness.workspace(tmp)
            buttons = scaffold(work)
            # Two mechanisms on one press is what teleop_button prevents.
            self.assertNotEqual(buttons["gripper"], buttons["position_cpp"])
            config = follow_next(work, buttons)
            subsystems, _ = sync_manifest.effective(work, config)

            session, diagnostics = self.launch(
                tmp, work, launcher, env, config,
                ["--joystick-schedule", joystick_schedule(buttons)],
                new_subsystem.SIM_DURATION_S)

            reached = []  # when position_cpp first reached its goal
            for mechanism, kind in MECHANISMS:
                with self.subTest(mechanism=mechanism):
                    (binding,) = subsystems[mechanism]["teleop"]["bindings"]
                    rows = states(log_dump, session, mechanism)
                    driven = [fields for _, fields in rows
                              if fields["enabled"] == "true"]
                    self.assertTrue(driven, f"{mechanism} never drove: "
                                    f"{diagnostics}")
                    if kind == "position":
                        goal = binding["while_held"]["position_rot"]
                        tolerance = subsystems[mechanism][mechanism][
                            "tolerance_rot"]
                        positions = [float(f["position_rot"])
                                     for f in driven]
                        at_goal = [f for f in driven
                                   if f["at_goal"] == "true"]
                        self.assertTrue(
                            at_goal,
                            f"{mechanism} never reached {goal} rot within "
                            f"{tolerance}: positions "
                            f"{min(positions)}..{max(positions)}, homing "
                            f"{sorted({f['homing'] for f in driven})}")
                        reached.append(next(
                            ms for ms, f in rows if f["enabled"] == "true"
                            and f["at_goal"] == "true"))
                        self.assertLessEqual(
                            abs(float(at_goal[0]["position_rot"]) - goal),
                            tolerance)
                        # It homed by driving to the simulated switch, not
                        # by zeroing where it started on a switch that
                        # reads closed everywhere.
                        (switch,) = subsystems[mechanism][
                            "digital_inputs"].values()
                        homing = [float(f["position_rot"]) for f in driven
                                  if f["homing"] == "HOMING"]
                        self.assertTrue(homing, f"{mechanism} never homed")
                        self.assertLessEqual(min(homing),
                                             switch["sim_closed_at_rot"])
                    else:
                        goal = binding["while_held"]["velocity_rps"]
                        fastest = max(float(f["velocity_rps"])
                                      for f in driven)
                        # The velocity loop holds its demand: within 1%.
                        self.assertGreaterEqual(
                            fastest, 0.99 * goal,
                            f"{mechanism} reached {fastest} of {goal} rps")
                        # It stood still until position_cpp, under
                        # its own button alone, reached its goal.
                        (moved,) = reached
                        before = [abs(float(f["velocity_rps"]))
                                  for ms, f in rows if ms <= moved]
                        self.assertLess(
                            max(before), 0.1 * goal,
                            f"{mechanism} moved at {max(before)} rps while "
                            f"only position_cpp's button was held")


if __name__ == "__main__":
    unittest.main()
