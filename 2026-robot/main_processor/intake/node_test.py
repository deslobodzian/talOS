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
    """The node test matrix (docs/TESTING.md §7), under the names
    //tools:check_node_matrix looks for, then the intake's own feed gate."""

    def setUp(self):
        self.sim = Sim(CONFIG)
        self.sim.add(intake.Intake, "intake")
        self.hw = SimHardware(self.sim)
        self.hw.set_motor("intake", "roller", 0.0, 12.5)

    def teleop(self, enabled=True):
        """Robot enabled (or not) in teleop; steps so the mode is current
        before any lane speaks."""
        self.hw.enable(enabled)
        self.hw.set_driver_station(enabled=True, teleop=True)
        self.sim.step(0.02)

    def send(self, ask, source="teleop", mode_since_us=None):
        self.sim.publish_target(IntakeTarget, "intake", source,
                                mode_since_us=mode_since_us,
                                roller_velocity_rps=ask)

    def hold(self, ask, seconds=0.04, source="teleop"):
        """A producer republishing its lane every 20 ms."""
        for _ in range(round(seconds / 0.02)):
            self.send(ask, source)
            self.sim.step(0.02)

    def request(self):
        """What the intake asked for (the flush sends neutral for an unset
        motor)."""
        return self.hw.motor_request("intake", "roller")

    def applied(self):
        """What the gateway applies, after its enable and expiry gates."""
        return self.hw.applied_motor("intake", "roller")

    def state(self):
        return self.sim.latest(IntakeState, topics.STATE)

    def assertNeutral(self, inhibit):
        """Neutral as asked and as applied, with /intake/state still
        published and naming why."""
        self.assertEqual(self.request().mode, talos.Mode.NEUTRAL)
        self.assertEqual(self.applied().mode, talos.Mode.NEUTRAL)
        state = self.state()
        self.assertEqual(state.inhibit, inhibit)
        self.assertEqual(state.requested_velocity_rps, 0.0)

    # REQ-SAFE-07
    def test_drives_at_a_live_lane(self):
        ready = dict(timestamp_ns=0, flywheel_velocity_rps=60.0,
                     target_velocity_rps=60.0, at_speed=True,
                     beam_broken=False, enabled=True)
        cases = [
            (25.0, False, None, 25.0),  # collecting needs no shooter
            (25.0, True, None, 0.0),  # piece held: stop pulling
            (-5.0, False, ready, -5.0),
            (-5.0, True, ready, -5.0),  # feeding bypasses the beam
            (0.0, False, None, 0.0),
        ]
        for ask, beam, shooter, demand in cases:
            with self.subTest(ask=ask, beam=beam):
                self.setUp()
                self.hw.set_digital_input("intake", "intake_beam", beam)
                self.teleop()
                if shooter is not None:
                    self.sim.publish(ShooterState, shooter_topics.STATE,
                                     **shooter)
                self.hold(ask)
                applied = self.applied()
                self.assertEqual(applied.mode, talos.Mode.VELOCITY)
                self.assertEqual(applied.demand, demand)
                state = self.state()
                self.assertTrue(state.enabled)
                self.assertEqual(state.inhibit, Inhibit.DRIVE)
                self.assertEqual(state.requested_velocity_rps, demand)
                self.assertEqual(state.roller_velocity_rps, 12.5)
                self.assertEqual(state.beam_broken, beam)

    # REQ-SAFE-02
    def test_disabled_robot_goes_neutral(self):
        self.teleop(enabled=False)
        self.hold(25.0)
        self.assertNeutral(Inhibit.GATEWAY_NOT_READY)
        self.assertFalse(self.state().enabled)  # state keeps publishing

    # REQ-SAFE-07
    def test_stale_lane_goes_neutral(self):
        self.teleop()
        self.hold(25.0)
        self.assertEqual(self.request().mode, talos.Mode.VELOCITY)
        # The last send was one period ago: this reaches the tick one period
        # past command_timeout_us (100 ms) since it.
        self.sim.step(0.1)
        self.assertNeutral(Inhibit.TARGET_STALE)

    # REQ-SAFE-07
    def test_released_lane_goes_neutral(self):
        self.teleop()
        self.hold(25.0)
        self.assertEqual(self.request().mode, talos.Mode.VELOCITY)
        self.sim.release_target(IntakeTarget, "intake", "teleop")
        self.sim.step(0.02)
        self.assertNeutral(Inhibit.TARGET_STALE)

    # REQ-SAFE-07
    def test_never_sent_goes_neutral(self):
        self.teleop()
        self.sim.step(0.06)
        self.assertNeutral(Inhibit.TARGET_STALE)

    # REQ-SAFE-07
    def test_driver_station_loss_goes_neutral(self):
        self.teleop()
        self.hold(25.0)
        self.assertEqual(self.request().mode, talos.Mode.VELOCITY)
        self.hw.stop_driver_station()
        self.hold(25.0, 0.16)  # the lane stays live; the DS view goes stale
        self.assertNeutral(Inhibit.TARGET_STALE)

    # REQ-CONTRACT-25
    def test_disconnected_feedback_goes_neutral(self):
        self.teleop()
        self.hold(25.0)
        self.assertEqual(self.request().mode, talos.Mode.VELOCITY)
        self.hw.set_connected("intake", "roller", False)
        self.hold(25.0)
        # The neighbor's motor keeps the robot enabled (testdata/robot.toml),
        # so the intake's own feedback gate is what stops it.
        self.assertTrue(talos.robot_enabled(self.sim.node("intake")))
        self.assertEqual(self.request().mode, talos.Mode.NEUTRAL)
        self.assertEqual(self.applied().mode, talos.Mode.NEUTRAL)
        self.assertEqual(self.state().inhibit, Inhibit.FEEDBACK_INVALID)

    # REQ-CONTRACT-25
    def test_invalid_beam_stops_collection_not_feeding(self):
        """An invalid beam sample (connected False) reads false: collecting
        on it would pull into a piece the intake may already hold."""
        ready = dict(timestamp_ns=0, flywheel_velocity_rps=60.0,
                     target_velocity_rps=60.0, at_speed=True,
                     beam_broken=False, enabled=True)
        self.hw.set_digital_input("intake", "intake_beam", True)
        self.teleop()
        self.hold(0.0)
        self.assertTrue(self.state().beam_broken)
        # (beam valid, broken, ask) -> applied mode, demand, inhibit,
        # reported beam_broken (the last valid reading while invalid).
        steps = [
            (False, True, 25.0, talos.Mode.NEUTRAL, 0.0,
             Inhibit.FEEDBACK_INVALID, True),
            (True, False, 25.0, talos.Mode.VELOCITY, 25.0, Inhibit.DRIVE,
             False),
            (False, False, 25.0, talos.Mode.NEUTRAL, 0.0,
             Inhibit.FEEDBACK_INVALID, False),
            (True, True, 25.0, talos.Mode.VELOCITY, 0.0, Inhibit.DRIVE, True),
            (False, True, -5.0, talos.Mode.VELOCITY, -5.0, Inhibit.DRIVE,
             True),  # feeding bypasses the beam, valid or not
        ]
        for valid, broken, ask, mode, demand, inhibit, reported in steps:
            with self.subTest(valid=valid, broken=broken, ask=ask):
                self.hw.set_digital_input("intake", "intake_beam", broken)
                self.hw.set_connected("intake", "intake_beam", valid)
                self.sim.publish(ShooterState, shooter_topics.STATE, **ready)
                self.hold(ask)
                self.assertTrue(talos.robot_enabled(self.sim.node("intake")))
                self.assertEqual(self.request().mode, mode)
                applied = self.applied()
                self.assertEqual(applied.mode, mode)
                if mode == talos.Mode.VELOCITY:
                    self.assertEqual(applied.demand, demand)
                state = self.state()
                self.assertEqual(state.inhibit, inhibit)
                self.assertEqual(state.requested_velocity_rps, demand)
                self.assertEqual(state.beam_broken, reported)

    # REQ-SAFE-04
    def test_non_finite_demand_goes_neutral(self):
        for ask in (math.nan, math.inf, -math.inf):
            with self.subTest(ask=ask):
                self.setUp()
                self.teleop()
                self.hold(ask)
                self.assertNeutral(Inhibit.DEMAND_NONFINITE)
                self.assertTrue(math.isfinite(self.state().roller_velocity_rps))

    # matrix: n/a 9 (the intake reads no parameters: its devices are config)

    # REQ-SAFE-07
    def test_other_modes_lane_is_ignored(self):
        self.teleop()
        self.hold(25.0, source="auto")  # autonomous's lane, live, in teleop
        self.assertNeutral(Inhibit.TARGET_STALE)
        self.sim.release_target(IntakeTarget, "intake", "auto")
        self.hw.set_driver_station(enabled=True, autonomous=True)
        self.sim.step(0.02)
        # Stamped with the autonomous generation: only the policy rejects it.
        self.hold(25.0)
        self.assertNeutral(Inhibit.TARGET_STALE)
        self.hold(15.0, source="auto")
        applied = self.applied()
        self.assertEqual(applied.mode, talos.Mode.VELOCITY)
        self.assertEqual(applied.demand, 15.0)

    # REQ-SAFE-09
    def test_earlier_generation_target_is_ignored(self):
        self.teleop()
        self.hold(25.0)
        self.assertEqual(self.request().mode, talos.Mode.VELOCITY)
        before = self.sim.driver_station_generation
        self.hw.set_driver_station(teleop=True)  # disabled
        self.sim.step(0.04)
        # Re-enabled off the 20 ms tick grid: the packet lands just after the
        # intake's tick and waits for its next drain, together with a target
        # its producer queued under the earlier enable.
        self.sim.step(0.001)
        self.hw.set_driver_station(enabled=True, teleop=True)
        self.sim.step(0.019)
        self.assertNotEqual(self.sim.driver_station_generation, before)
        self.send(25.0, mode_since_us=before)
        self.sim.step(0.02)
        self.assertNeutral(Inhibit.TARGET_STALE)
        # The same word stamped with the current generation drives at once.
        self.send(25.0)
        self.sim.step(0.02)
        self.assertEqual(self.request().mode, talos.Mode.VELOCITY)

    # matrix: n/a 12 (a Python tick allocates in the interpreter)

    # REQ-SAFE-32
    def test_feed_requires_continuously_fresh_ready_shooter(self):
        ready = dict(timestamp_ns=0, flywheel_velocity_rps=60.0,
                     target_velocity_rps=60.0, at_speed=True,
                     beam_broken=False, enabled=True)
        cases = [
            ("absent", None, 0.0),
            ("stale", ready, 0.15),
            ("spinning up", dict(ready, at_speed=False), 0.0),
            ("stopped target", dict(ready, target_velocity_rps=0.0), 0.0),
            ("reverse target", dict(ready, target_velocity_rps=-60.0), 0.0),
        ]
        for name, shooter, age in cases:
            with self.subTest(name=name):
                self.setUp()
                self.hw.set_digital_input("intake", "intake_beam", True)
                self.teleop()
                if shooter is not None:
                    self.sim.publish(ShooterState, shooter_topics.STATE,
                                     **shooter)
                self.sim.step(age)
                self.send(-20.0)
                self.sim.step(0.04)
                self.assertEqual(self.request().mode, talos.Mode.NEUTRAL)
                self.assertEqual(self.state().inhibit,
                                 Inhibit.SHOOTER_NOT_READY)


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
