"""Intake node tests: the Intake class on talos.testing.Sim, and the binary."""

import json
import math
import os
import subprocess
import tempfile
import unittest

import talos
from intake.IntakeState import IntakeState
from intake.IntakeTarget import IntakeTarget
from python.runfiles import runfiles
from talos.testing import Sim, SimHardware

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "testdata", "robot.toml")
intake = talos.load(os.path.join(HERE, "intake.py"))


class IntakeTest(unittest.TestCase):
    def run_intake(self, ask, enabled=True, beam=False, target_enabled=True,
                   seconds=0.05):
        """Publishes one target at t0, runs, returns (request, last state)."""
        sim = Sim(CONFIG)
        sim.add(intake.Intake, "intake")
        hw = SimHardware(sim)
        hw.enable(enabled)
        hw.set_motor("intake", "roller", 0.0, 12.5)
        hw.set_digital_input("intake", "intake_beam", beam)
        sim.publish(IntakeTarget, "/intake/target", enabled=target_enabled,
                    roller_velocity_rps=ask)
        sim.step(seconds)
        return (hw.motor_request("intake", "roller"),
                sim.latest(IntakeState, "/intake/state"))

    def test_ask_maps_to_velocity_command(self):
        cases = [
            (25.0, False, intake.INTAKE_RPS),
            (-5.0, False, intake.EJECT_RPS),
            (25.0, True, 0.0),  # piece held: stop pulling
            (-5.0, True, intake.EJECT_RPS),  # eject still gets it out
        ]
        for ask, beam, demand in cases:
            with self.subTest(ask=ask, beam=beam):
                request, state = self.run_intake(ask, beam=beam)
                self.assertEqual(request.mode, talos.Mode.VELOCITY)
                self.assertEqual(request.demand, demand)
                self.assertEqual(request.slot, 0)
                self.assertEqual(state.inhibit, intake.DRIVE)
                self.assertEqual(state.requested_velocity_rps, demand)

    def test_failed_gate_sends_neutral_and_names_it(self):
        cases = [
            ("disabled robot", dict(ask=25.0, enabled=False), intake.GATEWAY),
            ("disabled target", dict(ask=25.0, target_enabled=False),
             intake.TARGET_DISABLED),
            ("stale target", dict(ask=25.0, seconds=0.3), intake.TARGET_STALE),
            ("nonfinite ask", dict(ask=math.nan), intake.DEMAND_NONFINITE),
        ]
        for name, inputs, inhibit in cases:
            with self.subTest(name):
                request, state = self.run_intake(**inputs)
                self.assertEqual(request.mode, talos.Mode.NEUTRAL)
                self.assertEqual(state.inhibit, inhibit)
        # Only a live, finite ask is echoed as requested.
        _, state = self.run_intake(25.0, target_enabled=False)
        self.assertEqual(state.requested_velocity_rps, 0.0)

    def test_state_reports_feedback(self):
        _, state = self.run_intake(0.0, beam=True)
        self.assertTrue(state.enabled)
        self.assertEqual(state.roller_velocity_rps, 12.5)
        self.assertTrue(state.beam_broken)
        self.assertEqual(state.requested_velocity_rps, 0.0)
        self.assertGreater(state.issued_ns, 0)
        _, state = self.run_intake(0.0, enabled=False)
        self.assertFalse(state.enabled)


class BinaryTest(unittest.TestCase):
    def run_binary(self, *argv):
        resolver = runfiles.Create()
        binary = resolver.Rlocation(os.environ["INTAKE_BINARY"])
        env = dict(os.environ, **resolver.EnvVars())
        env.pop("PYTHONPATH", None)
        with tempfile.TemporaryDirectory() as cwd:
            proc = subprocess.run([binary, *argv], cwd=cwd, env=env,
                                  capture_output=True, text=True, timeout=120)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        return proc.stdout

    def test_describe_lists_the_intake_topics(self):
        doc = json.loads(self.run_binary("--describe", "--config", CONFIG))
        self.assertEqual(doc["name"], "intake")
        self.assertEqual(
            sorted((s["kind"], s["name"], s["message_bytes"])
                   for s in doc["sources"]),
            sorted([("TIMER", "intake", 0),
                    ("WATCHER", "/hw/state", 1404),
                    ("WATCHER", "/intake/target", IntakeTarget.SizeOf()),
                    ("SENDER", "/hw/request/intake", 1404),
                    ("SENDER", "/intake/state", IntakeState.SizeOf())]))

    def test_sim_run_replays_without_divergence(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = os.path.join(tmp, "intake.tlog")
            self.run_binary("--sim", "--duration-s", "1", "--config", CONFIG,
                            "--log", log)
            self.run_binary("--replay", log, "--config", CONFIG)


if __name__ == "__main__":
    unittest.main()
