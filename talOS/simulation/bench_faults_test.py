"""Integration: replay observed physical-bench faults through sim_gateway and
hardware_node in a launched session.

1. RESET-3 (65 ms CAN/feedback silence): a 65 ms motor feedback freeze stays
   strictly inside command_timeout_us (100 ms); the gateway and bridge keep
   feedback valid, raise no motor/hardware/link/owner-timeout alert, never
   neutralize the active velocity command, and resume fresh feedback cleanly.
2. RT-28 (410 ms link blackout hole): a 410 ms outbound UDP blackout exceeds
   command_timeout_us (100 ms) while enabled and driving; /hw/status raises
   alert_error_link_lost as state_age_us climbs past 100 ms, the actuator node
   neutralizes its request while the link is stale, the gateway invalidates
   the active command (advancing epoch and clearing command_active while
   State.sequence keeps stepping across the ~82 missed ticks), and once frames
   resume the link clears alert_error_link_lost and recovers cleanly to active
   actuation.
"""

import json
import os
import subprocess
import sys
import tempfile
import time
import unittest
import uuid

from _posixshmem import shm_unlink
from python.runfiles import runfiles

from talOS.build import sim_check
from talOS.memory.shm_name import shm_name

REGISTRY = "/talos_registry.6"
READY_TIMEOUT_S = 15
STOP_WAIT_S = 8
EXIT_WAIT_S = 3
LOG_DUMP_TIMEOUT_S = 10
COMMAND_TIMEOUT_US = 100_000
PERIOD_US = 5_000
TARGET_VELOCITY_RPS = 10.0
NODE_CLOCK = (time.CLOCK_MONOTONIC_RAW if sys.platform == "darwin"
              else time.CLOCK_MONOTONIC)


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


def dump_jsonl(log_dump, session, topic, node=None):
    selection = ["--node", node] if node else []
    dumped = subprocess.run(
        [log_dump, "--format", "jsonl"] + selection + ["--topic", topic,
                                                       session],
        capture_output=True, text=True, timeout=LOG_DUMP_TIMEOUT_S,
        check=True)
    return [json.loads(line) for line in dumped.stdout.splitlines()
            if line.startswith("{")]


def parse_decoded(row):
    fields = {}
    for item in row["decoded"].strip("{}").split(", "):
        key, value = item.split(": ", 1)
        fields[key] = value
    return fields


def write_config(path):
    with open(path, "w", encoding="utf-8") as f:
        f.write(
            "[robot]\n"
            'name = "bench_faults"\n'
            f"period_us = {PERIOD_US}\n"
            f"command_timeout_us = {COMMAND_TIMEOUT_US}\n"
            'controller_address = "127.0.0.1"\n'
            'sim_gateway = "//talOS/simulation:sim_gateway"\n'
            "\n"
            "[subsystems.teleop]\n"
            'node = "//talOS/teleop:node"\n'
            "\n"
            "[subsystems.bench_motor]\n"
            'node = "//talOS/simulation:bench_motor"\n'
            f"period_us = {PERIOD_US}\n"
            "\n"
            "[subsystems.bench_motor.teleop]\n"
            "defaults = { feedforward_v = 0.0 }\n"
            "\n"
            "[[subsystems.bench_motor.teleop.bindings]]\n"
            "button = 1\n"
            f"while_held = {{ velocity_rps = {TARGET_VELOCITY_RPS} }}\n"
            "\n"
            "[subsystems.bench_motor.motors.kraken]\n"
            "can_id = 1\n"
            "soft_limits = false\n"
            'position_frame = "rezero"\n'
            'motor_type = "kraken_x60"\n'
            "stator_limit_a = 80.0\n"
            "supply_limit_a = 40.0\n"
            "max_voltage = 12.0\n"
            "sensor_to_mechanism_ratio = 5.0\n"
            "max_velocity_rps = 20.0\n"
            "slot0 = { p = 1.0, v = 0.5 }\n"
        )


class BenchFaultsTest(unittest.TestCase):
    def _run_session(self, name, mode_schedule, joystick_schedule,
                     fault_schedule, duration_s, check):
        r = runfiles.Create()
        launcher = r.Rlocation(os.environ["LAUNCHER"])
        log_dump = r.Rlocation(os.environ["LOG_DUMP"])
        root = os.path.dirname(os.path.dirname(os.path.dirname(launcher)))
        gateway_port, bridge_port = sim_check.udp_ports()
        backstop_s = READY_TIMEOUT_S + int(duration_s) + 15

        with tempfile.TemporaryDirectory() as tmp:
            work = os.path.join(tmp, "work")
            os.makedirs(work)
            os.symlink(root, os.path.join(work, "bazel-bin"))
            config = os.path.join(work, "robot.toml")
            write_config(config)

            namespace = f"bench-faults-{name}-{uuid.uuid4().hex}"
            env = os.environ.copy()
            env["TALOS_SHM_NAMESPACE"] = namespace
            self.addCleanup(unlink_registry, namespace)

            start_file = os.path.join(tmp, "schedule_start")
            output = os.path.join(tmp, "sessions")
            os.makedirs(output)
            log = os.path.join(tmp, "launch.log")

            gateway_args = [
                "--physics",
                "--mode-schedule", mode_schedule,
                "--joystick-schedule", joystick_schedule,
                "--fault-schedule", fault_schedule,
                "--schedule-start-file", start_file,
            ]
            args = [
                launcher, "--config", config, "--sim", "--start-sim-gateway",
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
                with open(log, "wb") as out, \
                        sim_check.Launch(args, out, cwd=work, env=env) as run:
                    unready = sim_check.wait_ready(
                        run, output, READY_TIMEOUT_S)
                    ended = run.process.poll()
                    if not unready and ended is None:
                        start_schedule(start_file)
                        try:
                            ended = run.process.wait(timeout=duration_s)
                        except subprocess.TimeoutExpired:
                            pass
                    in_order, killed = run.finish(STOP_WAIT_S, EXIT_WAIT_S)

                with open(log, encoding="utf-8", errors="replace") as f:
                    diagnostics = f.read()
                self.assertEqual(unready, "", diagnostics)
                self.assertIsNone(
                    ended, f"launch exited early ({ended}): {diagnostics}")
                self.assertTrue(
                    in_order, f"launch did not stop in order: {diagnostics}")
                self.assertEqual(killed, [], diagnostics)
                self.assertEqual(run.process.returncode, 0, diagnostics)

                sessions = [d for d in os.listdir(output)
                            if os.path.isdir(os.path.join(output, d))]
                self.assertEqual(len(sessions), 1, os.listdir(output))
                session = os.path.join(output, sessions[0])

                with open(os.path.join(session, "manifest.json"),
                          encoding="utf-8") as f:
                    nodes = json.load(f)["nodes"]
                for node in nodes:
                    self.assertEqual(
                        len(node["runs"]), 1,
                        f"{node['name']} restarted: {diagnostics}")
                    run_info = node["runs"][0]
                    self.assertEqual(
                        (run_info["exit_code"], run_info["exit_signal"]),
                        (0, 0), f"{node['name']}: {run_info}\n{diagnostics}")

                check(log_dump, session, diagnostics)
                failed = False
            finally:
                if failed:
                    sim_check.preserve_failure_artifacts(
                        log, output, subdir=f"bench_faults_{name}")

    # REQ-SAFE-22, REQ-OPS-20
    def test_65ms_feedback_silence_stays_fresh_and_active(self):
        """RESET-3: a 65 ms motor feedback silence stays strictly inside
        command_timeout_us (100 ms); the bridge raises no fault or timeout
        alert and actuation continues without neutralization."""
        def verify(log_dump, session, diagnostics):
            statuses = [
                (row["publish_ms"], parse_decoded(row))
                for row in dump_jsonl(log_dump, session, "/hw/status",
                                      "hardware_node")
                if row["kind"] == "SEND"
            ]
            self.assertTrue(statuses, f"no /hw/status logged: {diagnostics}")
            configured = [
                (t, s) for t, s in statuses if s["configured"] == "true"
            ]
            self.assertGreater(
                len(configured), 20,
                f"bridge never reached configured state: {statuses[:5]}")

            # Once configured, a 65 ms feedback silence must raise zero link,
            # motor, hardware or owner-timeout alerts and zero gateway faults.
            for t, s in configured:
                for field in ("alert_error_link_lost",
                              "alert_error_motor_fault",
                              "alert_error_hardware_fault",
                              "alert_warning_owner_timed_out",
                              "alert_error_not_configured"):
                    self.assertEqual(
                        s[field], "false",
                        f"unexpected {field} at {t:.1f} ms during 65 ms "
                        f"silence: {s}")
                for field in ("motor_fault_mask",
                              "gateway_neutralized_motor_count",
                              "gateway_apply_failure_count",
                              "gateway_device_reset_count",
                              "gateway_rejected_command_count"):
                    self.assertEqual(
                        s[field], "0",
                        f"unexpected {field} at {t:.1f} ms during 65 ms "
                        f"silence: {s}")

            states = [
                (row["publish_ms"], row["hardware"])
                for row in dump_jsonl(log_dump, session, "/hw/state",
                                      "hardware_node")
                if row["kind"] == "SEND"
            ]
            self.assertTrue(states, f"no /hw/state logged: {diagnostics}")

            # Confirm the 65 ms freeze actually occurred while enabled &
            # driving, stayed strictly below command_timeout_us, and kept
            # feedback valid and command_active throughout.
            frozen_indices = [
                idx for idx, (_, st) in enumerate(states)
                if st["motors"][0]["position_age_us"] > 0
                or st["motors"][0]["velocity_age_us"] > 0
            ]
            self.assertGreaterEqual(
                len(frozen_indices), 5,
                f"65 ms freeze never produced aged feedback samples: "
                f"total states={len(states)}")
            max_pos_age_us = max(
                states[i][1]["motors"][0]["position_age_us"]
                for i in frozen_indices)
            max_vel_age_us = max(
                states[i][1]["motors"][0]["velocity_age_us"]
                for i in frozen_indices)
            self.assertGreaterEqual(
                max_pos_age_us, 45_000,
                f"expected ~65 ms feedback silence, got max position_age_us="
                f"{max_pos_age_us}")
            self.assertLess(
                max_pos_age_us, COMMAND_TIMEOUT_US,
                f"65 ms silence exceeded command_timeout_us "
                f"({COMMAND_TIMEOUT_US}): position_age_us={max_pos_age_us}")
            self.assertGreaterEqual(
                max_vel_age_us, 45_000,
                f"expected ~65 ms feedback silence, got max velocity_age_us="
                f"{max_vel_age_us}")
            self.assertLess(
                max_vel_age_us, COMMAND_TIMEOUT_US,
                f"65 ms silence exceeded command_timeout_us "
                f"({COMMAND_TIMEOUT_US}): velocity_age_us={max_vel_age_us}")

            first_freeze_idx = frozen_indices[0]
            last_freeze_idx = frozen_indices[-1]
            for idx in range(first_freeze_idx, last_freeze_idx + 1):
                t, st = states[idx]
                motor = st["motors"][0]
                self.assertTrue(
                    st["enabled"],
                    f"robot not enabled during freeze at {t:.1f} ms: {st}")
                self.assertTrue(
                    st["command_active"],
                    f"command_active dropped during 65 ms silence at "
                    f"{t:.1f} ms: {st}")
                self.assertFalse(
                    st["hardware_fault"],
                    f"hardware_fault raised during 65 ms silence at "
                    f"{t:.1f} ms: {st}")
                self.assertTrue(
                    motor["valid"],
                    f"motor feedback invalidated during 65 ms silence at "
                    f"{t:.1f} ms (age={motor['position_age_us']} us): {motor}")

            # After the silence ends, feedback age returns to 0 and velocity
            # stays live without interruption.
            post_freeze = states[last_freeze_idx + 1:]
            self.assertGreaterEqual(
                len(post_freeze), 20,
                "too few states after 65 ms silence to confirm recovery")
            for t, st in post_freeze[:20]:
                motor = st["motors"][0]
                self.assertEqual(motor["position_age_us"], 0, f"at {t:.1f} ms")
                self.assertEqual(motor["velocity_age_us"], 0, f"at {t:.1f} ms")
                self.assertTrue(motor["valid"], f"at {t:.1f} ms")
                self.assertTrue(st["command_active"], f"at {t:.1f} ms")
                self.assertGreater(
                    motor["velocity_rps"], 5.0,
                    f"motor lost velocity across 65 ms silence at {t:.1f} ms")

            # Verify /hw/command stayed in velocity mode across the entire
            # silence (never neutralized).
            freeze_start_ms = states[first_freeze_idx][0]
            freeze_end_ms = states[last_freeze_idx][0]
            commands = [
                (row["publish_ms"], row["hardware"])
                for row in dump_jsonl(log_dump, session, "/hw/command",
                                      "hardware_node")
                if row["kind"] == "SEND"
            ]
            during_and_after = [
                (t, c) for t, c in commands
                if freeze_start_ms <= t <= freeze_end_ms + 100.0
            ]
            self.assertTrue(
                during_and_after,
                "no /hw/command logged across the 65 ms silence window")
            for t, cmd in during_and_after:
                motor_req = cmd["motors"][0]
                self.assertEqual(
                    motor_req["mode"], "velocity",
                    f"motor neutralized during 65 ms silence at {t:.1f} ms: "
                    f"{motor_req}")
                self.assertAlmostEqual(
                    motor_req["demand"], TARGET_VELOCITY_RPS, places=4)

        self._run_session(
            name="reset3_65ms_silence",
            mode_schedule="disabled:0.2,teleop:2.0",
            joystick_schedule="0:axes=0;buttons=0;pov=-1|"
                              "0.35:axes=0;buttons=1;pov=-1",
            fault_schedule="0.700:0.765:freeze:kraken",
            duration_s=1.5,
            check=verify,
        )

    # REQ-SAFE-15, REQ-SAFE-22, REQ-OPS-20
    def test_410ms_blackout_stale_neutralizes_and_recovers(self):
        """RT-28: a 410 ms outbound UDP blackout exceeds command_timeout_us
        (100 ms) while enabled; the link goes stale (alert_error_link_lost),
        outputs neutralize on both ends, and once frames resume the link
        clears the alert and recovers cleanly to active actuation."""
        def verify(log_dump, session, diagnostics):
            states = [
                (row["publish_ms"], row["hardware"])
                for row in dump_jsonl(log_dump, session, "/hw/state",
                                      "hardware_node")
                if row["kind"] == "SEND"
            ]
            self.assertGreater(
                len(states), 40, f"too few /hw/state records: {diagnostics}")

            # Locate the 410 ms blackout hole in /hw/state.
            holes = [
                (i, states[i], states[i + 1],
                 states[i + 1][1]["sample_time_us"]
                 - states[i][1]["sample_time_us"])
                for i in range(len(states) - 1)
                if states[i + 1][1]["sample_time_us"]
                - states[i][1]["sample_time_us"] >= 300_000
            ]
            self.assertEqual(
                len(holes), 1,
                f"expected exactly one >= 300 ms blackout hole, found "
                f"{[(i, dt) for i, _, _, dt in holes]}")
            hole_idx, (pre_ms, pre), (post_ms, post), gap_us = holes[0]

            # Match RT-28: ~410 ms step in sample_time_us and ~81 missing
            # States while the gateway kept ticking at 5 ms.
            self.assertGreaterEqual(
                gap_us, 350_000,
                f"blackout hole smaller than expected: {gap_us} us")
            seq_delta = post["sequence"] - pre["sequence"]
            self.assertGreaterEqual(
                seq_delta, 60,
                f"expected >= 60 missed gateway States across 410 ms hole, "
                f"got sequence step {seq_delta}")

            # Before the hole, the robot was enabled and actively driving.
            self.assertTrue(pre["enabled"], f"pre-hole state: {pre}")
            self.assertTrue(pre["command_active"], f"pre-hole state: {pre}")
            self.assertTrue(pre["motors"][0]["valid"], f"pre-hole state: {pre}")
            self.assertGreater(
                pre["motors"][0]["velocity_rps"], 5.0,
                f"motor was not driving before blackout: {pre['motors'][0]}")

            # Across the hole, the gateway expired the active command,
            # advanced its control epoch, and neutralized outputs without
            # latching a hardware or motor fault.
            self.assertGreater(
                post["epoch"], pre["epoch"],
                f"gateway epoch did not advance across 410 ms blackout: "
                f"pre={pre['epoch']} post={post['epoch']}")
            self.assertFalse(
                post["command_active"],
                f"gateway did not neutralize active command during 410 ms "
                f"blackout: {post}")
            self.assertFalse(post["hardware_fault"], f"post-hole state: {post}")
            self.assertTrue(
                post["motors"][0]["valid"], f"post-hole state: {post}")

            # During the blackout, /hw/status raised alert_error_link_lost as
            # state_age_us climbed past command_timeout_us.
            statuses = [
                (row["publish_ms"], parse_decoded(row))
                for row in dump_jsonl(log_dump, session, "/hw/status",
                                      "hardware_node")
                if row["kind"] == "SEND"
            ]
            blackout_statuses = [
                (t, s) for t, s in statuses if pre_ms < t < post_ms
            ]
            self.assertTrue(
                blackout_statuses,
                f"bridge published no /hw/status during [{pre_ms:.1f}, "
                f"{post_ms:.1f}] ms blackout")
            max_age_us = max(
                int(s["state_age_us"]) for _, s in blackout_statuses)
            self.assertGreaterEqual(
                max_age_us, 300_000,
                f"expected state_age_us >= 300 ms during 410 ms blackout, got "
                f"{max_age_us} us")
            self.assertTrue(
                any(s["alert_error_link_lost"] == "true"
                    for _, s in blackout_statuses),
                f"alert_error_link_lost never raised during 410 ms blackout: "
                f"{blackout_statuses}")

            # While the link was stale during the blackout, the actuator node
            # neutralized its /hw/request/bench_motor.
            requests = [
                (row["publish_ms"], row["hardware"])
                for row in dump_jsonl(log_dump, session,
                                      "/hw/request/bench_motor", "bench_motor")
                if row["kind"] == "SEND"
            ]
            stale_requests = [
                (t, req) for t, req in requests
                if pre_ms + 130.0 <= t < post_ms
            ]
            self.assertTrue(
                stale_requests,
                "no /hw/request/bench_motor logged while link was stale")
            for t, req in stale_requests:
                self.assertEqual(
                    req["motors"][0]["mode"], "neutral",
                    f"actuator node did not neutralize during stale blackout "
                    f"at {t:.1f} ms: {req['motors'][0]}")

            # After frames resume, /hw/status clears alert_error_link_lost
            # and stays healthy through the end of the session.
            post_statuses = [(t, s) for t, s in statuses if t > post_ms + 40.0]
            self.assertGreaterEqual(
                len(post_statuses), 10,
                "too few /hw/status records after blackout recovery")
            for t, s in post_statuses:
                self.assertEqual(
                    s["configured"], "true",
                    f"lost configured after blackout recovery at {t:.1f} ms")
                self.assertEqual(
                    s["alert_error_link_lost"], "false",
                    f"alert_error_link_lost stayed set after recovery at "
                    f"{t:.1f} ms: {s}")
                self.assertEqual(
                    s["alert_error_hardware_fault"], "false",
                    f"hardware fault after blackout recovery at {t:.1f} ms")
                self.assertEqual(
                    s["alert_error_motor_fault"], "false",
                    f"motor fault after blackout recovery at {t:.1f} ms")
                self.assertEqual(
                    s["alert_warning_owner_timed_out"], "false",
                    f"owner timeout after blackout recovery at {t:.1f} ms")
                self.assertLess(
                    int(s["state_age_us"]), COMMAND_TIMEOUT_US,
                    f"state_age_us stayed stale at {t:.1f} ms: {s}")
            last_status = post_statuses[-1][1]
            for field in ("motor_fault_mask",
                          "gateway_apply_failure_count",
                          "gateway_device_reset_count"):
                self.assertEqual(
                    last_status[field], "0", f"{field}: {last_status}")

            # After operator re-arming (button released then pressed again),
            # /hw/state recovers to command_active == true with live velocity.
            post_states = states[hole_idx + 1:]
            recovered_driving = [
                (t, st) for t, st in post_states
                if st["command_active"]
                and st["motors"][0]["valid"]
                and st["motors"][0]["velocity_rps"] > 5.0
            ]
            self.assertGreaterEqual(
                len(recovered_driving), 10,
                f"actuation did not recover cleanly after 410 ms blackout; "
                f"post_states={len(post_states)}, "
                f"active={sum(1 for _, s in post_states if s['command_active'])}")

        self._run_session(
            name="rt28_410ms_blackout",
            mode_schedule="disabled:0.2,teleop:3.0",
            joystick_schedule="0:axes=0;buttons=0;pov=-1|"
                              "0.35:axes=0;buttons=1;pov=-1|"
                              "1.35:axes=0;buttons=0;pov=-1|"
                              "1.55:axes=0;buttons=1;pov=-1",
            fault_schedule="0.700:1.110:udp_blackout:bridge",
            duration_s=2.2,
            check=verify,
        )


if __name__ == "__main__":
    unittest.main()
