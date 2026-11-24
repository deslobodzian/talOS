"""End-to-end REQ-RT-15 check: session_report on a real launched session."""

import contextlib
import io
import json
import os
import select
import subprocess
import sys
import tempfile
import threading
import time
import unittest
from unittest import mock
import uuid

from _posixshmem import shm_unlink
from python.runfiles import runfiles
from talOS.build import sim_check
from talOS.launcher import session_report
from talOS.memory.shm_name import shm_name


class SessionReportLiveTest(unittest.TestCase):
    @staticmethod
    def _run_session_report(*args):
        stdout, stderr = io.StringIO(), io.StringIO()
        with mock.patch.object(sys, "argv", ["session_report", *args]), \
                contextlib.redirect_stdout(stdout), \
                contextlib.redirect_stderr(stderr):
            returncode = session_report.main()
        return subprocess.CompletedProcess(
            ["session_report", *args], returncode, stdout.getvalue(), stderr.getvalue())

    # REQ-RT-15: verify target-age and /hw/state cadence verdicts on a real
    # recorded --sim --start-sim-gateway session with an active teleop target
    # (PASS, never NOT MEASURED).
    def test_real_launched_session_passes_cadence_and_target_age_verdicts(self):
        files = runfiles.Create()
        launcher = files.Rlocation(os.environ["LAUNCHER"])
        root = os.path.dirname(os.path.dirname(os.path.dirname(launcher)))
        gateway_port, bridge_port = sim_check.udp_ports()
        node_clock = (time.CLOCK_MONOTONIC_RAW if sys.platform == "darwin"
                      else time.CLOCK_MONOTONIC)

        def unlink_registry(namespace):
            try:
                shm_unlink(shm_name("/talos_registry.6", namespace))
            except FileNotFoundError:
                pass

        with tempfile.TemporaryDirectory() as tmp:
            work = os.path.join(tmp, "work")
            os.makedirs(work)
            os.symlink(root, os.path.join(work, "bazel-bin"))
            config = os.path.join(work, "robot.toml")
            with open(config, "w", encoding="utf-8") as handle:
                handle.write(
                    "[robot]\n"
                    'name = "session_report_real"\n'
                    "period_us = 5000\n"
                    "command_timeout_us = 100000\n"
                    'controller_address = "127.0.0.1"\n'
                    'sim_gateway = "//talOS/simulation:sim_gateway"\n'
                    "\n"
                    "[subsystems.teleop]\n"
                    'node = "//talOS/teleop:node"\n'
                    "\n"
                    "[subsystems.bench_motor]\n"
                    'node = "//talOS/simulation:bench_motor"\n'
                    "period_us = 20000\n"
                    "\n"
                    "[subsystems.bench_motor.teleop]\n"
                    "defaults = { feedforward_v = 0.0 }\n"
                    "\n"
                    "[[subsystems.bench_motor.teleop.bindings]]\n"
                    "button = 1\n"
                    "while_held = { velocity_rps = 10.0 }\n"
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

            namespace = f"session-report-real-{uuid.uuid4().hex}"
            env = os.environ.copy()
            env["TALOS_SHM_NAMESPACE"] = namespace
            self.addCleanup(unlink_registry, namespace)

            start_file = os.path.join(tmp, "schedule_start")
            output = os.path.join(tmp, "sessions")
            os.makedirs(output)
            log = os.path.join(tmp, "launch.log")

            # Arm OperatorArming with neutral sticks on the teleop enable edge,
            # then hold button 1 for ~0.55 s (~110 enabled /hw/state ticks) so
            # both /hw/state cadence and selected target age at tick are measured.
            gateway_args = [
                "--physics",
                "--mode-schedule", "disabled:0.05,teleop:2.0",
                "--joystick-schedule",
                "0:axes=0;buttons=0;pov=-1|0.10:axes=0;buttons=1;pov=-1",
                "--schedule-start-file", start_file,
            ]
            args = [
                launcher, "--config", config, "--sim", "--start-sim-gateway",
                "--remote", "127.0.0.1",
                "--remote-port", str(gateway_port),
                "--local-port", str(bridge_port),
                "--output-dir", output,
                "--duration-s", "30",
            ]
            for arg in gateway_args:
                args += ["--sim-gateway-arg", arg]

            stall_max_ms = 0.0
            stall_r, stall_w = os.pipe()

            def measure_host_stalls():
                nonlocal stall_max_ms
                interval_s = 0.005
                while True:
                    t0_ns = time.clock_gettime_ns(node_clock)
                    ready, _, _ = select.select([stall_r], [], [], interval_s)
                    dt_ms = (time.clock_gettime_ns(node_clock) - t0_ns) / 1e6
                    if ready:
                        break
                    overrun_ms = dt_ms - interval_s * 1000.0
                    if overrun_ms > stall_max_ms:
                        stall_max_ms = overrun_ms

            stall_thread = threading.Thread(target=measure_host_stalls, daemon=True)
            with open(log, "wb") as out, \
                    sim_check.Launch(args, out, cwd=work, env=env) as run:
                unready = sim_check.wait_ready(run, output, 15)
                ended = run.process.poll()
                if not unready and ended is None:
                    stall_thread.start()
                    try:
                        pending = start_file + ".pending"
                        with open(pending, "w", encoding="utf-8") as handle:
                            handle.write(str(time.clock_gettime_ns(node_clock)))
                        os.rename(pending, start_file)
                        try:
                            ended = run.process.wait(timeout=0.65)
                        except subprocess.TimeoutExpired:
                            pass
                    finally:
                        os.write(stall_w, b"x")
                        stall_thread.join(timeout=5)
                in_order, killed = run.finish(8, 3)
            os.close(stall_r)
            os.close(stall_w)

            with open(log, encoding="utf-8", errors="replace") as handle:
                diagnostics = handle.read()
            self.assertEqual(unready, "", diagnostics)
            self.assertIsNone(ended, f"launch exited early ({ended}): {diagnostics}")
            self.assertTrue(in_order, f"launch did not stop in order: {diagnostics}")
            self.assertEqual(killed, [], diagnostics)
            self.assertEqual(run.process.returncode, 0, diagnostics)

            sessions = [os.path.join(output, d) for d in os.listdir(output)
                        if os.path.isdir(os.path.join(output, d))]
            self.assertEqual(len(sessions), 1, os.listdir(output))
            session = sessions[0]

            result = self._run_session_report("--json", "--strict", session)
            if result.returncode != 0:
                sim_check.preserve_failure_artifacts(
                    "session_report_live_test", log, output)
            host_tag = (
                f"[HOST_UNQUALIFIED] host scheduler stall {stall_max_ms:.1f} ms "
                f"exceeded 10.0 ms during 5 ms cadence window\n"
                if stall_max_ms > 10.0 else ""
            )
            self.assertEqual(
                result.returncode, 0,
                f"{host_tag}session_report --strict failed:\n{result.stderr}\n"
                f"{result.stdout}\n{diagnostics}")
            report = json.loads(result.stdout)
            verdicts = {v["measurement"]: v for v in report["verdicts"]}
            for name in (
                "controller link",
                "overall state median",
                "overall state p99",
                "enabled state median",
                "enabled state p99",
                "enabled state gaps",
                "target age at tick",
            ):
                with self.subTest(verdict=name):
                    self.assertEqual(verdicts[name]["verdict"], "PASS", verdicts[name])
            self.assertGreater(report["state_overall"]["count"], 50)
            self.assertGreater(report["state_enabled"]["count"], 50)
            self.assertEqual(report["enabled_gaps"], 0)
            self.assertEqual(report["state_p99_bound_ms"], 10.0)
            self.assertEqual(report["enabled_gap_bound_ms"], 100.0)
            lane = "bench_motor:/bench_motor/target/teleop"
            self.assertIn(lane, report["target_age_at_tick"], report)
            self.assertGreater(report["target_age_at_tick"][lane]["count"], 0)
            self.assertLessEqual(
                report["target_age_at_tick"][lane]["max_ms"],
                session_report.TARGET_AGE_LIMIT_MS)
            self.assertIn(lane, report["target_receipt_age"], report)
            self.assertGreater(report["target_receipt_age"][lane]["count"], 0)


if __name__ == "__main__":
    unittest.main()
