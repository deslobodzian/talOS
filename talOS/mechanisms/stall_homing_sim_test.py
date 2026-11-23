"""e2e: PositionMechanism stall homing completes in a sim_gateway --physics
session against [[simulation.hard_stops]] (with soft_limits = false), then
drives to a teleop goal position.

Runs the launcher with hardware_node, sim_gateway --physics, teleop, and the
testonly lift node hosting PositionMechanism (homing = "stall"). Schedules
count from a node-clock time zero written via --schedule-start-file once every
member's loop has started and the bridge has accepted a State.
"""

import json
import os
import subprocess
import sys
import tempfile
import time
import unittest

from _posixshmem import shm_unlink
from python.runfiles import runfiles

from talOS.build import sim_check
from talOS.memory.shm_name import shm_name

REGISTRY = "/talos_registry.6"
READY_TIMEOUT_S = 15
STOP_WAIT_S = 8
EXIT_WAIT_S = 3
LOG_DUMP_TIMEOUT_S = 10
SESSION_DURATION_S = 3.0
BACKSTOP_S = READY_TIMEOUT_S + int(SESSION_DURATION_S) + 15
NODE_CLOCK = (time.CLOCK_MONOTONIC_RAW if sys.platform == "darwin"
              else time.CLOCK_MONOTONIC)

GOAL_ROT = 2.0
TOLERANCE_ROT = 0.05
HARD_STOP_MIN_ROT = -0.3


def unlink_registry(namespace):
    try:
        shm_unlink(shm_name(REGISTRY, namespace))
    except FileNotFoundError:
        pass


def start_schedule(path):
    pending = path + ".pending"
    with open(pending, "w", encoding="utf-8") as f:
        f.write(str(time.clock_gettime_ns(NODE_CLOCK)))
    os.rename(pending, path)


def decoded_records(log_dump, session, topic, node=None):
    selection = ["--node", node] if node else []
    dumped = subprocess.run(
        [log_dump, "--format", "jsonl"] + selection + ["--topic", topic,
                                                       session],
        capture_output=True, text=True, timeout=LOG_DUMP_TIMEOUT_S,
        check=True)
    out = []
    for line in dumped.stdout.splitlines():
        if not line.startswith("{"):
            continue
        row = json.loads(line)
        fields = {}
        for item in row["decoded"].strip("{}").split(", "):
            key, value = item.split(": ", 1)
            fields[key] = value
        out.append((row["publish_ms"], fields))
    return out


class StallHomingSimTest(unittest.TestCase):
    # REQ-DEV-10, REQ-SAFE-36
    def test_stall_homing_against_hard_stop_reaches_goal_in_physics_session(
            self):
        r = runfiles.Create()
        launcher = r.Rlocation(os.environ["LAUNCHER"])
        log_dump = r.Rlocation(os.environ["LOG_DUMP"])
        root = os.path.dirname(os.path.dirname(os.path.dirname(launcher)))
        gateway_port, bridge_port = sim_check.udp_ports()
        with tempfile.TemporaryDirectory() as tmp:
            work = os.path.join(tmp, "work")
            os.makedirs(work)
            os.symlink(root, os.path.join(work, "bazel-bin"))
            config = os.path.join(work, "robot.toml")
            with open(config, "w", encoding="utf-8") as f:
                f.write(
                    "[robot]\n"
                    'name = "stall_homing_sim"\n'
                    "period_us = 5000\n"
                    "command_timeout_us = 100000\n"
                    'controller_address = "127.0.0.1"\n'
                    'sim_gateway = "//talOS/simulation:sim_gateway"\n'
                    "\n"
                    "[simulation]\n"
                    "[[simulation.hard_stops]]\n"
                    'motor = "lift"\n'
                    f"min_rot = {HARD_STOP_MIN_ROT}\n"
                    "max_rot = 5.0\n"
                    "\n"
                    "[subsystems.teleop]\n"
                    'node = "//talOS/teleop:node"\n'
                    "\n"
                    "[subsystems.lift]\n"
                    'node = "//talOS/mechanisms:lift"\n'
                    "period_us = 10000\n"
                    "\n"
                    "[subsystems.lift.lift]\n"
                    f"tolerance_rot = {TOLERANCE_ROT}\n"
                    "min_rot = 0.0\n"
                    "max_rot = 5.0\n"
                    'homing = "stall"\n'
                    "homing_v = -1.5\n"
                    "home_rot = 0.0\n"
                    "homing_timeout_us = 4000000\n"
                    "homing_stall_rps = 0.2\n"
                    "homing_stall_us = 150000\n"
                    "homing_stall_current_a = 20.0\n"
                    "homing_min_push_us = 100000\n"
                    "homing_min_travel_rot = 0.1\n"
                    "\n"
                    "[subsystems.lift.teleop]\n"
                    "defaults = { feedforward_v = 0.0 }\n"
                    "\n"
                    "[[subsystems.lift.teleop.bindings]]\n"
                    "button = 1\n"
                    f"while_held = {{ position_rot = {GOAL_ROT} }}\n"
                    "\n"
                    "[subsystems.lift.motors.lift]\n"
                    "can_id = 1\n"
                    "soft_limits = false\n"
                    'position_frame = "rezero"\n'
                    'motor_type = "kraken_x60"\n'
                    "stator_limit_a = 80.0\n"
                    "supply_limit_a = 40.0\n"
                    "max_voltage = 12.0\n"
                    "sensor_to_mechanism_ratio = 5.0\n"
                    "max_velocity_rps = 20.0\n"
                    "cruise_velocity_rps = 6.0\n"
                    "acceleration_rps2 = 24.0\n"
                    "slot0 = { p = 6.0, d = 0.1, v = 0.6 }\n"
                )
            env = os.environ.copy()
            namespace = os.environ.get("TEST_TMPDIR") or tmp
            env.pop("TALOS_SHM_NAMESPACE", None)
            env["TEST_TMPDIR"] = namespace
            self.addCleanup(unlink_registry, namespace)
            start_file = os.path.join(tmp, "schedule_start")
            output = os.path.join(tmp, "sessions")
            os.makedirs(output)
            log = os.path.join(tmp, "launch.log")
            gateway_args = [
                "--physics",
                "--mode-schedule", "disabled:0.3,teleop:4.0",
                "--joystick-schedule",
                "0:axes=0;buttons=0;pov=-1|0.5:axes=0;buttons=1;pov=-1|"
                "2.8:axes=0;buttons=0;pov=-1",
                "--schedule-start-file", start_file,
            ]
            args = [
                launcher, "--config", config, "--sim", "--start-sim-gateway",
                "--remote", "127.0.0.1",
                "--remote-port", str(gateway_port),
                "--local-port", str(bridge_port),
                "--output-dir", output,
                "--duration-s", str(BACKSTOP_S),
            ]
            for arg in gateway_args:
                args += ["--sim-gateway-arg", arg]

            with open(log, "wb") as out, \
                    sim_check.Launch(args, out, cwd=work, env=env) as launch:
                unready = sim_check.wait_ready(launch, output, READY_TIMEOUT_S)
                ended = launch.process.poll()
                if not unready and ended is None:
                    start_schedule(start_file)
                    try:
                        ended = launch.process.wait(timeout=SESSION_DURATION_S)
                    except subprocess.TimeoutExpired:
                        pass
                in_order, killed = launch.finish(STOP_WAIT_S, EXIT_WAIT_S)

            with open(log, encoding="utf-8", errors="replace") as f:
                diagnostics = f.read()
            self.assertEqual(unready, "", diagnostics)
            self.assertIsNone(ended, f"launch exited early ({ended}): "
                              f"{diagnostics}")
            self.assertTrue(in_order, f"launch did not stop in order: "
                            f"{diagnostics}")
            self.assertEqual(killed, [], diagnostics)
            self.assertEqual(launch.process.returncode, 0, diagnostics)

            sessions = [d for d in os.listdir(output)
                        if os.path.isdir(os.path.join(output, d))]
            self.assertEqual(len(sessions), 1, os.listdir(output))
            session = os.path.join(output, sessions[0])

            with open(os.path.join(session, "manifest.json"),
                      encoding="utf-8") as f:
                nodes = json.load(f)["nodes"]
            for node in nodes:
                self.assertEqual(len(node["runs"]), 1,
                                 f"{node['name']} restarted: {diagnostics}")
                run = node["runs"][0]
                self.assertEqual((run["exit_code"], run["exit_signal"]),
                                 (0, 0), f"{node['name']}: {run}\n{diagnostics}")

            statuses = decoded_records(log_dump, session, "/hw/status")
            self.assertTrue(statuses, f"no /hw/status logged: {diagnostics}")
            last_status = statuses[-1][1]
            self.assertEqual(last_status["configured"], "true", last_status)
            for field, value in last_status.items():
                if field.startswith("alert_"):
                    self.assertEqual(value, "false", f"{field}: {last_status}")
                if field in ("motor_fault_mask",
                             "gateway_rejected_command_count",
                             "gateway_neutralized_motor_count",
                             "gateway_apply_failure_count",
                             "gateway_device_reset_count"):
                    self.assertEqual(value, "0", f"{field}: {last_status}")

            rows = decoded_records(log_dump, session, "/lift/state", "lift")
            driven = [fields for _, fields in rows
                      if fields["enabled"] == "true"]
            self.assertTrue(driven, f"lift never drove: {diagnostics}")

            # While homing, PositionMechanism drives with goal_rot == 0 and
            # travels down to the physical hard stop at HARD_STOP_MIN_ROT.
            homing_positions = [float(f["position_rot"]) for f in driven
                                if float(f["goal_rot"]) == 0.0]
            self.assertTrue(homing_positions,
                            f"lift never entered homing push: {driven[:10]}")
            self.assertAlmostEqual(min(homing_positions), HARD_STOP_MIN_ROT,
                                   places=3)

            # After stall homing zeroes the frame at the hard stop, lift drives
            # to GOAL_ROT and reports at_goal within tolerance.
            at_goal = [f for f in driven
                       if f["at_goal"] == "true"
                       and abs(float(f["goal_rot"]) - GOAL_ROT) < 1e-6]
            self.assertTrue(
                at_goal,
                f"lift never reached {GOAL_ROT} rot after stall homing; "
                f"goals={sorted({float(f['goal_rot']) for f in driven})}, "
                f"positions={min(float(f['position_rot']) for f in driven)}.."
                f"{max(float(f['position_rot']) for f in driven)}")
            self.assertLessEqual(
                abs(float(at_goal[0]["position_rot"]) - GOAL_ROT),
                TOLERANCE_ROT)


if __name__ == "__main__":
    unittest.main()
