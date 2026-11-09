"""Intake node tests: the Intake class on talos.testing.Sim, and the binary."""

import math
import os
import subprocess
import tempfile
import unittest

import talos
from intake import topics
from intake.Inhibit import Inhibit
from intake.IntakeState import IntakeState
from intake.IntakeTarget import IntakeTarget
from python.runfiles import runfiles
from shooter import topics as shooter_topics
from shooter.ShooterState import ShooterState
from talos.testing import Sim, SimHardware

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "testdata", "robot.toml")
intake = talos.load(os.path.join(HERE, "intake.py"))


class IntakeTest(unittest.TestCase):
    def run_intake(self, ask, enabled=True, beam=False, lane="send",
                   seconds=0.05, roller_connected=True, ds_loss=False,
                   shooter=None, shooter_age=0.0):
        """Teleop DS, then one lane envelope ("send", "release" after a
        send, or None for never sent); runs, returns (request, last state)."""
        sim = Sim(CONFIG)
        sim.add(intake.Intake, "intake")
        hw = SimHardware(sim)
        hw.enable(enabled)
        hw.set_driver_station(enabled=True, teleop=True)
        hw.set_motor("intake", "roller", 0.0, 12.5)
        hw.set_connected("intake", "roller", roller_connected)
        hw.set_digital_input("intake", "intake_beam", beam)
        sim.step(0.01)  # the teleop mode is current before the lane speaks
        if shooter is not None:
            sim.publish(ShooterState, shooter_topics.STATE, **shooter)
        sim.step(shooter_age)
        if lane is not None:
            sim.publish_target(IntakeTarget, "intake", "teleop",
                               roller_velocity_rps=ask)
        if lane == "release":
            sim.step(0.02)
            sim.release_target(IntakeTarget, "intake", "teleop")
        if ds_loss:
            sim.step(0.02)
            hw.stop_driver_station()
        sim.step(seconds)
        return (hw.motor_request("intake", "roller"),
                sim.latest(IntakeState, topics.STATE))

    def test_ask_maps_to_velocity_command(self):
        cases = [
            (25.0, False, 25.0),
            (-5.0, False, -5.0),
            (25.0, True, 0.0),  # piece held: stop pulling
            (-5.0, True, -5.0),  # feeding bypasses the beam
            (0.0, False, 0.0),
            (0.0, True, 0.0),
        ]
        for ask, beam, demand in cases:
            with self.subTest(ask=ask, beam=beam):
                request, state = self.run_intake(
                    ask, beam=beam,
                    shooter=dict(at_speed=True, target_velocity_rps=60.0))
                self.assertEqual(request.mode, talos.Mode.VELOCITY)
                self.assertEqual(request.demand, demand)
                self.assertEqual(state.inhibit, Inhibit.DRIVE)
                self.assertEqual(state.requested_velocity_rps, demand)
                self.assertEqual(state.roller_velocity_rps, 12.5)
                self.assertEqual(state.beam_broken, beam)

    def test_failed_gate_sends_neutral_and_names_it(self):
        cases = [
            ("disabled robot", dict(ask=25.0, enabled=False),
             Inhibit.GATEWAY_NOT_READY),
            ("stale lane", dict(ask=25.0, seconds=0.3),
             Inhibit.TARGET_STALE),
            ("released lane", dict(ask=25.0, lane="release"),
             Inhibit.TARGET_STALE),
            ("never sent", dict(ask=25.0, lane=None), Inhibit.TARGET_STALE),
            ("driver station lost", dict(ask=25.0, ds_loss=True, seconds=0.3),
             Inhibit.TARGET_STALE),
            ("nonfinite ask", dict(ask=math.nan), Inhibit.DEMAND_NONFINITE),
        ]
        for name, inputs, inhibit in cases:
            with self.subTest(name):
                request, state = self.run_intake(**inputs)
                self.assertEqual(request.mode, talos.Mode.NEUTRAL)
                self.assertEqual(state.inhibit, inhibit)
                if inputs.get("lane") == "release":
                    self.assertEqual(state.requested_velocity_rps, 0.0)

    def test_disconnected_roller_reports_feedback_invalid(self):
        request, state = self.run_intake(25.0, roller_connected=False)
        self.assertEqual(request.mode, talos.Mode.NEUTRAL)
        self.assertEqual(state.inhibit, Inhibit.FEEDBACK_INVALID)
        self.assertEqual(state.roller_velocity_rps, 0.0)  # no sample
        self.assertTrue(state.enabled)

    def test_feed_requires_continuously_fresh_ready_shooter(self):
        ready = dict(at_speed=True, target_velocity_rps=60.0,
                     flywheel_velocity_rps=60.0, enabled=True)
        cases = [
            ("absent", None, 0.0),
            ("stale", ready, 0.15),
            ("spinning up", dict(ready, at_speed=False), 0.0),
            ("stopped target", dict(ready, target_velocity_rps=0.0), 0.0),
            ("reverse target", dict(ready, target_velocity_rps=-60.0), 0.0),
        ]
        for name, shooter, age in cases:
            with self.subTest(name=name):
                request, state = self.run_intake(
                    -20.0, beam=True, shooter=shooter, shooter_age=age)
                self.assertEqual(request.mode, talos.Mode.NEUTRAL)
                self.assertEqual(state.inhibit, Inhibit.SHOOTER_NOT_READY)
                self.assertEqual(state.requested_velocity_rps, -20.0)
        request, state = self.run_intake(-20.0, beam=True, shooter=ready)
        self.assertEqual(request.mode, talos.Mode.VELOCITY)
        self.assertEqual(request.demand, -20.0)
        self.assertEqual(state.inhibit, Inhibit.DRIVE)

    def test_feed_stops_when_shooter_readiness_expires(self):
        sim = Sim(CONFIG)
        sim.add(intake.Intake, "intake")
        hw = SimHardware(sim)
        hw.enable()
        hw.set_driver_station(enabled=True, teleop=True)
        sim.step(0.01)
        sim.publish(ShooterState, shooter_topics.STATE,
                    at_speed=True, target_velocity_rps=60.0)
        for _ in range(8):
            sim.publish_target(IntakeTarget, "intake", "teleop",
                               roller_velocity_rps=-20.0)
            sim.step(0.02)
        self.assertEqual(hw.motor_request("intake", "roller").mode,
                         talos.Mode.NEUTRAL)
        self.assertEqual(sim.latest(IntakeState, topics.STATE).inhibit,
                         Inhibit.SHOOTER_NOT_READY)

    def test_feed_obeys_robot_and_target_gates(self):
        ready = dict(at_speed=True, target_velocity_rps=60.0)
        for inputs in (dict(enabled=False), dict(lane=None),
                       dict(seconds=0.3)):
            with self.subTest(inputs=inputs):
                request, _ = self.run_intake(-20.0, shooter=ready, **inputs)
                self.assertEqual(request.mode, talos.Mode.NEUTRAL)

    def test_collection_does_not_require_shooter(self):
        for beam in (False, True):
            for shooter in (None, dict(at_speed=False, target_velocity_rps=0.0)):
                with self.subTest(beam=beam, shooter=shooter):
                    request, state = self.run_intake(30.0, beam=beam, shooter=shooter)
                    self.assertEqual(request.mode, talos.Mode.VELOCITY)
                    self.assertEqual(request.demand, 0.0 if beam else 30.0)
                    self.assertEqual(state.inhibit, Inhibit.DRIVE)


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

    def test_sim_run_replays_without_divergence(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = os.path.join(tmp, "intake.tlog")
            self.run_binary("--sim", "--duration-s", "1", "--config", CONFIG,
                            "--log", log)
            self.run_binary("--replay", log, "--config", CONFIG)


if __name__ == "__main__":
    unittest.main()
