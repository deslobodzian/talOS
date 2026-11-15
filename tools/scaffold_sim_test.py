"""e2e: a robot scaffolded with a C++ velocity mechanism (gripper) and a
position mechanism (position_cpp), edited exactly as their `Next:` output
says and run the way it says (the launcher with sim_gateway --physics and
its mode schedule, as `./sim.sh --sim-gateway` starts them), moves each
motor to the goal its teleop binding holds while the button its `Next:`
names is held, the position mechanism after homing on its simulated home
switch. The joystick schedule holds each mechanism's button in turn, so it
also shows that position_cpp's button does not move the gripper.
The node binaries are new_subsystem's goldens, which
new_subsystem_test keeps equal to its output.

A robot scaffolded with --no-mechanism runs the same way: every member
exits 0, and the bridge's last /hw/status is configured with no alert,
reject or fault."""

import json
import os
import socket
import subprocess
import tempfile
import unittest

from _posixshmem import shm_unlink  # what multiprocessing.shared_memory uses
from python.runfiles import runfiles

from tools import new_robot_test as harness
from tools import new_subsystem
from tools import sync_manifest

# position_cpp first: the gripper's check uses when it reached its goal.
MECHANISMS = (("position_cpp", "position"), ("gripper", "velocity"))

# talOS/introspection/registry.h kRegistryPath. The launcher creates the
# registry and deliberately leaves it for later sessions, so the test
# unlinks its own after the run.
REGISTRY = "/talos_registry.6"

# Bounds on a hang, not on a run (which lasts SIM_DURATION_S or
# NO_MECHANISM_DURATION_S): a hang in one case, with the other case's
# ordinary run, stays below Bazel's 60 s `short` timeout, so it fails with
# the test's diagnostics instead of being killed by Bazel.
LAUNCH_TIMEOUT_S = 40
NO_MECHANISM_LAUNCH_TIMEOUT_S = 12
LOG_DUMP_TIMEOUT_S = 5
# The no-mechanism run: past the mode schedule's 1 s disabled spell, so
# the gateway is enabled for the last records.
NO_MECHANISM_DURATION_S = 3


def shm_name(topic, namespace):
    """talos::ShmObjectName (talOS/memory/shared_memory_ptr.h)."""
    mapped = "/" + topic.lstrip("/").replace("/", ".")
    value = 0xCBF29CE484222325
    for byte in namespace.encode() + b"\0" + mapped.encode():
        value = ((value ^ byte) * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return f"/~{value:016x}"


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


class ScaffoldSimTest(unittest.TestCase):
    def launch(self, tmp, work, launcher, env, config, gateway_extra,
               duration_s, timeout_s=LAUNCH_TIMEOUT_S):
        """Runs `config` for `duration_s` the way `./sim.sh --sim-gateway`
        does: the launcher starting sim_gateway --physics on its mode
        schedule. Asserts the launcher exits 0; returns (session directory,
        launcher output)."""
        gateway_port, bridge_port = udp_ports()
        # Bazel's TEST_TMPDIR is the shared-memory namespace
        # (ARCHITECTURE.md "Topics and messages") and holds the lock files,
        # which Bazel removes with it; only the registry segment is left to
        # unlink.
        namespace = os.environ.get("TEST_TMPDIR") or tmp
        env.pop("TALOS_SHM_NAMESPACE", None)
        env["TEST_TMPDIR"] = namespace
        self.addCleanup(unlink_registry, namespace)
        gateway_args = [
            "--physics", "--port", str(gateway_port), "--remote-port",
            str(bridge_port), "--mode-schedule",
            new_subsystem.MODE_SCHEDULE] + gateway_extra
        args = [launcher, "--config", config, "--sim", "--start-sim-gateway"]
        for arg in gateway_args:
            args += ["--sim-gateway-arg", arg]
        output = os.path.join(tmp, "sessions")
        args += ["--remote", "127.0.0.1", "--remote-port", str(gateway_port),
                 "--local-port", str(bridge_port), "--output-dir", output,
                 "--duration-s", str(duration_s)]
        # The timeout only bounds a hang; the run lasts duration_s.
        try:
            run = subprocess.run(args, cwd=work, env=env,
                                 capture_output=True, text=True,
                                 timeout=timeout_s)
        except subprocess.TimeoutExpired as hung:
            self.fail(f"launcher still running after {hung.timeout:.0f} s: "
                      f"{hung.stdout!r} {hung.stderr!r}")
        diagnostics = run.stdout + run.stderr
        self.assertEqual(run.returncode, 0, diagnostics)
        (session,) = os.listdir(output)
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
                NO_MECHANISM_DURATION_S, NO_MECHANISM_LAUNCH_TIMEOUT_S)
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
