"""Pose estimator tests: both backends on talos.testing.Sim, and the binary.

Every scenario runs once per backend. A simulated swerve chassis publishes
DrivetrainState the way the drivetrain does (module positions, gyro yaw,
sample time); the expected poses are closed-form, not another estimator.
"""

import json
import math
import os
import subprocess
import tempfile
import unittest

import talos
from drivetrain import topics as drivetrain_topics
from drivetrain.DrivetrainState import DrivetrainState
from geometry.PoseEstimate import PoseEstimate
from geometry.PoseObservation import PoseObservation
from pose_estimator import topics
from python.runfiles import runfiles
from talos.testing import Sim, SimHardware

HERE = os.path.dirname(os.path.abspath(__file__))
CONFIG = os.path.join(HERE, "testdata", "robot.toml")
estimator = talos.load(os.path.join(HERE, "pose_estimator.py"))

BACKENDS = ("kalman", "factor_graph")
DT = 0.02
# 2026 robot layout, in [subsystems.drivetrain.geometry].modules order.
MODULES = ((-0.30, 0.30), (-0.30, -0.30), (0.30, 0.30), (0.30, -0.30))


def robot_toml(backend, *, cameras=("front_camera", "back_camera"),
               extra=""):
    names = ", ".join(f'"{c}"' for c in cameras)
    return f"""
[robot]
name = "pose_estimator_test"
period_us = 20000
command_timeout_us = 100000

[subsystems.pose_estimator]
period_us = 20000
backend = "{backend}"
cameras = [{names}]
module_x_m = [{", ".join(str(x) for x, _ in MODULES)}]
module_y_m = [{", ".join(str(y) for _, y in MODULES)}]
max_observation_age_us = 500000
{extra}

# A robot config needs one motor: the drivetrain's, whose state the tests
# publish themselves.
[subsystems.drivetrain.motors.front_left_drive]
type = "TalonFX"
bus = "test"
can_id = 1
"""


class Chassis:
    """A swerve chassis on the Sim: integrates robot-frame (vx, vy, omega)
    in closed form and publishes what the drivetrain would report."""

    def __init__(self, sim):
        self.sim = sim
        self.x = self.y = self.yaw = 0.0
        self.distances = [0.0] * len(MODULES)
        self.angles = [0.0] * len(MODULES)
        self.velocity = (0.0, 0.0, 0.0)

    def advance(self, vx, vy, omega, slip=1.0):
        """One DT of constant robot-frame velocity; `slip` scales what the
        wheels report (1: no slip)."""
        if omega == 0.0:
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            self.x += (vx * c - vy * s) * DT
            self.y += (vx * s + vy * c) * DT
        else:  # exact constant-twist arc
            yaw_end = self.yaw + omega * DT
            self.x += (vx * (math.sin(yaw_end) - math.sin(self.yaw))
                       + vy * (math.cos(yaw_end) - math.cos(self.yaw))) / omega
            self.y += (vx * (math.cos(self.yaw) - math.cos(yaw_end))
                       + vy * (math.sin(yaw_end) - math.sin(self.yaw))) / omega
        self.yaw += omega * DT
        for i, (mx, my) in enumerate(MODULES):
            wx, wy = vx - omega * my, vy + omega * mx
            speed = math.hypot(wx, wy)
            if speed > 0.0:
                self.angles[i] = math.atan2(wy, wx)
            self.distances[i] += speed * DT * slip
        self.velocity = (vx, vy, omega)

    def publish(self, *, sample_time_ns=None, modules_valid=True,
                yaw_valid=True, distance_m=None):
        fields = dict(
            sample_time_ns=(round(self.sim.now * 1e9)
                            if sample_time_ns is None else sample_time_ns),
            enabled=False, modules_valid=modules_valid, yaw_valid=yaw_valid,
            vx_mps=self.velocity[0], vy_mps=self.velocity[1],
            omega_radps=self.velocity[2], yaw_rad=self.yaw)
        for i in range(len(MODULES)):
            fields[f"module{i}_distance_m"] = (
                self.distances[i] if distance_m is None else distance_m)
            fields[f"module{i}_angle_rad"] = self.angles[i]
        self.sim.publish(DrivetrainState, drivetrain_topics.STATE, **fields)

    def drive(self, vx, vy, omega, seconds, slip=1.0, **publish):
        for _ in range(round(seconds / DT)):
            self.advance(vx, vy, omega, slip)
            self.publish(**publish)
            self.sim.step(DT)

    def hold(self, seconds, **publish):
        """Stationary, still publishing every DT."""
        self.drive(0.0, 0.0, 0.0, seconds, **publish)


class EstimatorTest(unittest.TestCase):
    def start(self, backend, *, enabled=False, driver_station=True,
              attached=True, **config):
        self.sim = Sim.from_toml(robot_toml(backend, **config))
        self.sim.add(estimator.PoseEstimator, "pose_estimator")
        self.hw = SimHardware(self.sim)
        self.hw.enable(enabled)
        if driver_station:
            self.hw.set_driver_station(
                enabled=enabled, teleop=True, attached=attached)
        self.chassis = Chassis(self.sim)
        self.sim.step(DT)  # sample time 0 means "unknown"
        self.chassis.publish()
        self.sim.step(DT)
        return self.chassis

    def assertPoseHeld(self, held):
        for value, expected in zip(self.pose(), held):
            self.assertAlmostEqual(value, expected, delta=1e-9)

    def estimate(self):
        return self.sim.latest(PoseEstimate, topics.STATE)

    def pose(self):
        e = self.estimate()
        return (e.pose_x_m, e.pose_y_m, e.pose_yaw_rad)

    def observe(self, camera="front_camera", *, age_s=0.0, x=None, y=None,
                yaw=None, std=(0.05, 0.05, 0.05), valid=True, time_ns=None,
                tag_count=2):
        """Publishes one observation taken `age_s` before the newest
        drivetrain sample, by default of the chassis' current true pose."""
        if time_ns is None:
            latest = self.estimate().sample_time_ns
            time_ns = latest - round(age_s * 1e9)
        self.sim.publish(
            PoseObservation, f"/{camera}/state", sample_time_ns=time_ns,
            pose_x_m=self.chassis.x if x is None else x,
            pose_y_m=self.chassis.y if y is None else y,
            pose_yaw_rad=self.chassis.yaw if yaw is None else yaw,
            x_std_m=std[0], y_std_m=std[1], yaw_std_rad=std[2], tag_count=tag_count,
            valid=valid)

    def counts(self):
        e = self.estimate()
        return (e.observation_accepted_count, e.observation_rejected_count)

    def test_configured_initial_pose(self):
        for backend in BACKENDS:
            with self.subTest(backend):
                chassis = self.start(backend, extra=(
                    "initial_x_m = 2.0\ninitial_y_m = 3.0\n"
                    "initial_yaw_rad = 1.0"))
                self.assertPoseHeld((2.0, 3.0, 1.0))
                self.assertTrue(self.estimate().valid)
                self.assertTrue(self.estimate().localized)
                chassis.hold(DT)
                self.assertPoseHeld((2.0, 3.0, 1.0))

    def test_enabled_restart_accepts_vision_before_field_localization(self):
        for backend in BACKENDS:
            for x, yaw in ((2.1, 1.05), (8.0, 2.0)):
                with self.subTest(backend=backend, x=x, yaw=yaw):
                    chassis = self.start(backend, enabled=True, extra=(
                        "initial_x_m = 2.0\ninitial_y_m = 3.0\n"
                        "initial_yaw_rad = 1.0"))
                    self.assertPoseHeld((2.0, 3.0, 1.0))
                    self.assertTrue(self.estimate().valid)
                    self.assertFalse(self.estimate().localized)
                    for index, camera in enumerate(
                            ("front_camera", "back_camera", "front_camera")):
                        self.observe(camera, x=x, y=3.0, yaw=yaw,
                                     std=(0.001, 0.001, 0.001))
                        chassis.hold(DT)
                        self.assertEqual(self.counts(), (index + 1, 0))
                        self.assertTrue(self.estimate().valid)
                        self.assertEqual(self.estimate().localized, index == 2)
                    self.assertAlmostEqual(self.pose()[0], x, delta=0.1)
                    self.assertAlmostEqual(self.pose()[2], yaw, delta=0.02)
                    # Localization restores the normal innovation gate.
                    self.observe(x=x, y=3.0, yaw=yaw - 1.0)
                    chassis.hold(DT)
                    self.assertEqual(self.counts(), (3, 1))

    def test_restart_needs_independent_camera_confirmation_even_after_disable(self):
        for backend in BACKENDS:
            with self.subTest(backend=backend):
                chassis = self.start(backend, enabled=True)
                self.hw.set_driver_station(enabled=False, teleop=True)
                for index in range(5):
                    self.observe(x=6.0, std=(0.001, 0.001, 0.001))
                    chassis.hold(DT)
                    self.assertFalse(self.estimate().localized)
                    self.assertEqual(self.counts(), (index + 1, 0))
                self.observe("back_camera", x=6.0,
                             std=(0.001, 0.001, 0.001))
                chassis.hold(DT)
                self.assertTrue(self.estimate().localized)
                self.assertEqual(self.counts(), (6, 0))

    def test_one_configured_camera_can_confirm_startup_localization(self):
        for backend in BACKENDS:
            with self.subTest(backend=backend):
                chassis = self.start(
                    backend, enabled=True, cameras=("front_camera",))
                for index in range(3):
                    self.observe(x=6.0)
                    chassis.hold(DT)
                    self.assertEqual(self.counts(), (index + 1, 0))
                    self.assertEqual(self.estimate().localized, index == 2)

    def test_unlocalized_measurements_still_require_usable_field_observations(self):
        for backend in BACKENDS:
            for fields in (dict(x=-0.01), dict(y=8.22), dict(yaw=math.nan),
                           dict(std=(0.0, 0.05, 0.05))):
                with self.subTest(backend=backend, fields=fields):
                    chassis = self.start(backend, enabled=True)
                    self.observe(**fields)
                    chassis.hold(DT)
                    self.assertEqual(self.counts(), (0, 1))
                    self.assertFalse(self.estimate().localized)
                    self.assertTrue(self.estimate().valid)
                    self.assertPoseHeld((0.0, 0.0, 0.0))

    def test_startup_localization_requires_consistent_recent_distinct_evidence(self):
        for backend in BACKENDS:
            for interruption in ("inconsistent", "capture_timeout",
                                 "arrival_timeout", "replay"):
                with self.subTest(backend=backend, interruption=interruption):
                    chassis = self.start(backend, enabled=True)
                    timestamp = self.estimate().sample_time_ns
                    self.observe(x=6.0, time_ns=(
                        timestamp + round(DT * 1e9)
                        if interruption == "arrival_timeout" else None))
                    chassis.hold(DT)
                    if interruption == "capture_timeout":
                        chassis.hold(0.52)
                    elif interruption == "arrival_timeout":
                        chassis.hold(0.50)
                    self.observe("back_camera",
                                 x=10.0 if interruption == "inconsistent" else 6.0,
                                 time_ns=(timestamp if interruption == "replay"
                                          else None))
                    chassis.hold(DT)
                    self.observe(x=6.0, time_ns=(
                        timestamp if interruption == "replay" else None))
                    chassis.hold(DT)
                    self.assertFalse(self.estimate().localized)

    def test_unseen_or_detached_ds_does_not_trust_the_seed(self):
        for backend in BACKENDS:
            for driver_station in (False, True):
                with self.subTest(backend=backend, driver_station=driver_station):
                    chassis = self.start(
                        backend, driver_station=driver_station, attached=False)
                    self.assertTrue(self.estimate().valid)
                    self.assertFalse(self.estimate().localized)
                    self.hw.set_driver_station(enabled=True, teleop=True)
                    chassis.hold(DT)
                    self.assertFalse(self.estimate().localized)
                    self.hw.set_driver_station(enabled=False, teleop=True)
                    chassis.hold(DT)
                    self.assertFalse(self.estimate().localized)

    def test_disabled_first_ds_can_arrive_after_odometry(self):
        for backend in BACKENDS:
            with self.subTest(backend=backend):
                chassis = self.start(backend, driver_station=False)
                self.assertFalse(self.estimate().localized)
                self.hw.set_driver_station(enabled=False, teleop=True)
                # A DS packet published on a tick boundary can be polled just
                # after that update; allow the following update to publish it.
                chassis.hold(2 * DT)
                self.assertTrue(self.estimate().localized)

    def test_localization_survives_enable_disable_ds_loss_and_stale_odometry(self):
        for backend in BACKENDS:
            for enabled in (False, True):
                with self.subTest(backend=backend, enabled=enabled):
                    chassis = self.start(backend, enabled=enabled)
                    if enabled:
                        for camera in ("front_camera", "back_camera",
                                       "front_camera"):
                            self.observe(camera, x=6.0)
                            chassis.hold(DT)
                    self.assertTrue(self.estimate().localized)
                    for mode in (True, False, True):
                        self.hw.set_driver_station(enabled=mode, teleop=True)
                        chassis.hold(DT)
                        self.assertTrue(self.estimate().localized)
                    self.hw.stop_driver_station()
                    self.sim.step(0.2)
                    self.assertFalse(self.estimate().valid)
                    self.assertTrue(self.estimate().localized)
                    chassis.hold(DT)
                    self.assertTrue(self.estimate().valid)
                    self.assertTrue(self.estimate().localized)

    def test_tag_bounds_and_innovation_gates(self):
        for backend in BACKENDS:
            for observation in (dict(x=-0.01), dict(x=16.55),
                                dict(y=-0.01), dict(y=8.22),
                                dict(x=4.0, tag_count=1),
                                dict(x=4.0, tag_count=2)):
                with self.subTest(backend=backend, observation=observation):
                    chassis = self.start(backend)
                    self.observe(**observation)
                    chassis.hold(DT)
                    self.assertEqual(self.counts(), (0, 1))
                    self.assertPoseHeld((0.0, 0.0, 0.0))

    def test_yaw_innovation_gate_applies_to_all_tag_counts(self):
        for backend in BACKENDS:
            for yaw, count, accepted in ((0.2, 1, True), (0.3, 1, False),
                                         (-0.3, 1, False), (0.6, 2, False)):
                with self.subTest(backend=backend, yaw=yaw, tag_count=count):
                    chassis = self.start(backend)
                    before = self.pose()
                    self.observe(yaw=yaw, tag_count=count)
                    chassis.hold(DT)
                    self.assertEqual(self.counts(), (1, 0) if accepted else (0, 1))
                    if not accepted:
                        self.assertPoseHeld(before)

    def test_three_consistent_measurements_relocalize_while_disabled(self):
        for backend in BACKENDS:
            for tag_count in (1, 2):
                for cameras in (("front_camera",),
                                ("front_camera", "back_camera")):
                    with self.subTest(backend=backend, tag_count=tag_count,
                                      cameras=cameras):
                        chassis = self.start(
                            backend, cameras=cameras)
                        for index in range(3):
                            self.observe(cameras[index % len(cameras)],
                                         x=4.0, yaw=0.6, tag_count=tag_count,
                                         std=(0.001, 0.001, 0.001))
                            chassis.hold(DT)
                            if index < 2:
                                self.assertEqual(self.counts(), (0, index + 1))
                                self.assertPoseHeld((0.0, 0.0, 0.0))
                        self.assertEqual(self.counts(), (1, 2))
                        self.assertAlmostEqual(self.pose()[0], 4.0, delta=0.1)
                        self.assertAlmostEqual(self.pose()[2], 0.6, delta=0.02)

    def test_multiple_cameras_require_independent_relocalization_confirmation(self):
        for backend in BACKENDS:
            with self.subTest(backend=backend):
                chassis = self.start(backend)
                # Repeated, stable errors from one camera cannot relocalize,
                # even after more than the three required observations.
                for index in range(5):
                    self.observe(x=4.0, std=(0.001, 0.001, 0.001))
                    chassis.hold(DT)
                    self.assertEqual(self.counts(), (0, index + 1))
                    self.assertPoseHeld((0.0, 0.0, 0.0))
                self.observe("back_camera", x=4.0, std=(0.001, 0.001, 0.001))
                chassis.hold(DT)
                self.assertEqual(self.counts(), (1, 5))
                self.assertAlmostEqual(self.pose()[0], 4.0, delta=0.1)

    def test_inconsistent_relocalization_candidates_do_not_move_pose(self):
        for backend in BACKENDS:
            # The last sequence agrees with its neighbours but not pairwise.
            for observations in ((4.0, 8.0, 4.0), (4.0, 6.0, 8.0)):
                with self.subTest(backend=backend, observations=observations):
                    chassis = self.start(backend)
                    for index, x in enumerate(observations):
                        self.observe("back_camera" if index == 1
                                     else "front_camera", x=x)
                        chassis.hold(DT)
                    self.assertEqual(self.counts(), (0, 3))
                    self.assertPoseHeld((0.0, 0.0, 0.0))
            with self.subTest(backend=backend, inconsistent_yaw=True):
                chassis = self.start(backend)
                for index, yaw in enumerate((0.6, 1.2, 0.6)):
                    self.observe("back_camera" if index == 1
                                 else "front_camera", x=4.0, yaw=yaw)
                    chassis.hold(DT)
                self.assertEqual(self.counts(), (0, 3))
                self.assertPoseHeld((0.0, 0.0, 0.0))

    def test_relocalization_requires_recent_distinct_consecutive_evidence(self):
        for backend in BACKENDS:
            for interruption in ("timeout", "replay", "accepted", "rejected"):
                with self.subTest(backend=backend, interruption=interruption):
                    chassis = self.start(backend, cameras=("front_camera",))
                    timestamp = self.estimate().sample_time_ns
                    self.observe(x=4.0)
                    chassis.hold(DT)
                    if interruption == "timeout":
                        chassis.hold(0.52)
                    elif interruption == "accepted":
                        self.observe()
                        chassis.hold(DT)
                    elif interruption == "rejected":
                        self.observe(std=(0.0, 0.05, 0.05))
                        chassis.hold(DT)
                    for _ in range(2):
                        self.observe(x=4.0, time_ns=(
                            timestamp if interruption == "replay" else None))
                        chassis.hold(DT)
                    self.assertEqual(self.counts(),
                                     (1, 3) if interruption == "accepted"
                                     else (0, 4) if interruption == "rejected"
                                     else (0, 3))
                    self.assertPoseHeld((0.0, 0.0, 0.0))

    def test_relocalization_compensates_for_odometry_between_candidates(self):
        for backend in BACKENDS:
            with self.subTest(backend=backend):
                chassis = self.start(backend)
                for index in range(3):
                    self.observe("back_camera" if index == 1
                                 else "front_camera",
                                 x=4.0, yaw=chassis.yaw + 0.6)
                    chassis.drive(0.0, 0.0, 3.0, 0.1)
                    if index < 2:
                        self.assertEqual(self.counts(), (0, index + 1))
                self.assertEqual(self.counts(), (1, 2))

    def test_invalid_and_no_tag_camera_states_are_not_measurements(self):
        for backend in BACKENDS:
            with self.subTest(backend=backend):
                chassis = self.start(backend)
                for fields in (dict(valid=False), dict(tag_count=0),
                               dict(valid=False, tag_count=0, y=math.nan)):
                    self.observe(**fields)
                    chassis.hold(DT)
                    self.assertEqual(self.counts(), (0, 0))
                    self.assertPoseHeld((0.0, 0.0, 0.0))

    def test_single_tag_yaw_innovation_wraps_at_pi(self):
        for backend in BACKENDS:
            for sign in (-1, 1):
                with self.subTest(backend=backend, sign=sign):
                    initial = sign * (math.pi - 0.03)
                    chassis = self.start(
                        backend, extra=f"initial_yaw_rad = {initial}")
                    self.observe(yaw=-initial, tag_count=1)
                    chassis.hold(DT)
                    self.assertEqual(self.counts(), (1, 0))

    def test_delayed_single_tag_matches_historical_yaw_during_fast_rotation(self):
        for backend in BACKENDS:
            with self.subTest(backend=backend):
                chassis = self.start(backend)
                chassis.drive(0.0, 0.0, 3.5, 0.2)
                historical_yaw = chassis.yaw
                historical_ns = self.estimate().sample_time_ns
                chassis.drive(0.0, 0.0, 3.5, 0.1)
                self.assertEqual(self.estimate().sample_time_ns - historical_ns,
                                 100_000_000)
                self.assertGreater(chassis.yaw - historical_yaw,
                                   math.radians(20))
                self.observe(time_ns=historical_ns, yaw=historical_yaw,
                             tag_count=1)
                chassis.drive(0.0, 0.0, 3.5, DT)
                self.assertEqual(self.counts(), (1, 0))
                self.assertAlmostEqual(self.pose()[2], chassis.yaw, places=4)

    def test_yaw_innovation_limit_is_configurable_and_positive_finite(self):
        for backend in BACKENDS:
            with self.subTest(backend=backend, configured=True):
                chassis = self.start(
                    backend, extra="max_yaw_innovation_rad = 0.1")
                self.observe(yaw=0.11, tag_count=1)
                chassis.hold(DT)
                self.assertEqual(self.counts(), (0, 1))
            for value in ("0.0", "-0.1", "nan", "inf"):
                with self.subTest(backend=backend, value=value):
                    sim = Sim.from_toml(robot_toml(
                        backend, extra=f"max_yaw_innovation_rad = {value}"))
                    with self.assertRaisesRegex(Exception, "max_yaw_innovation_rad"):
                        sim.add(estimator.PoseEstimator, "pose_estimator")

    def test_odometry_tracks_a_straight_line_then_an_arc(self):
        for backend in BACKENDS:
            with self.subTest(backend):
                chassis = self.start(backend)
                chassis.drive(1.0, 0.0, 0.0, 1.0)
                x, y, yaw = self.pose()
                self.assertAlmostEqual(x, 1.0, places=4)
                self.assertAlmostEqual(y, 0.0, places=4)
                self.assertAlmostEqual(yaw, 0.0, places=4)

                chassis.drive(1.0, 0.0, 1.0, 1.0)  # radius 1 m, 1 rad
                x, y, yaw = self.pose()
                self.assertAlmostEqual(x, 1.0 + math.sin(1.0), places=3)
                self.assertAlmostEqual(y, 1.0 - math.cos(1.0), places=3)
                self.assertAlmostEqual(yaw, 1.0, places=4)
                e = self.estimate()
                self.assertTrue(e.valid)
                self.assertEqual(e.sample_time_ns,
                                 round((self.sim.now - DT) * 1e9))
                self.assertEqual((e.velocity_vx_mps, e.velocity_omega_radps),
                                 (1.0, 1.0))
                self.assertEqual(self.counts(), (0, 0))

    def test_a_delayed_observation_corrects_the_estimate(self):
        for backend in BACKENDS:
            with self.subTest(backend):
                chassis = self.start(backend)
                # The wheels report 90% of the travel: odometry falls behind.
                chassis.drive(1.0, 0.0, 0.0, 1.5, slip=0.9)
                truth_then = chassis.x
                chassis.drive(1.0, 0.0, 0.0, 0.3, slip=0.9)
                before = self.pose()
                error_before = chassis.x - before[0]
                self.assertAlmostEqual(error_before, 0.18, places=3)

                # The camera saw the true pose 0.3 s ago (15 samples back).
                self.observe(age_s=0.3, x=truth_then, y=0.0, yaw=0.0)
                chassis.drive(1.0, 0.0, 0.0, DT, slip=0.9)
                after = self.pose()
                error_after = chassis.x - after[0]
                self.assertEqual(self.counts(), (1, 0))
                self.assertLess(error_after, 0.6 * error_before)
                self.assertGreater(error_after, -0.05)  # no overshoot
                self.assertAlmostEqual(after[1], 0.0, places=2)

                # Processed once: later ticks neither recount nor reapply.
                chassis.hold(0.1)
                self.assertEqual(self.counts(), (1, 0))

    def test_an_observation_between_samples_is_placed_at_its_own_time(self):
        # Exact odometry and an exact, confident observation taken 9 ms
        # after a sample: placed at its own time it agrees with the
        # estimate. Placed at the nearer sample without moving it by the
        # odometry in between, it would drag the pose 9 mm back.
        for backend in BACKENDS:
            with self.subTest(backend):
                chassis = self.start(backend)
                chassis.drive(1.0, 0.0, 0.0, 0.6)
                age_s = 0.2 - 0.009
                self.observe(age_s=age_s, x=chassis.x - 1.0 * age_s,
                             std=(0.001, 0.001, 0.001))
                chassis.drive(1.0, 0.0, 0.0, DT)
                self.assertEqual(self.counts(), (1, 0))
                self.assertAlmostEqual(self.pose()[0], chassis.x, delta=1e-3)

    def test_bad_observations_are_rejected_and_leave_the_pose_alone(self):
        nan, inf = math.nan, math.inf
        cases = [
            ("non-finite pose", dict(y=nan)),
            ("infinite yaw", dict(yaw=inf)),
            ("non-finite std", dict(std=(0.05, nan, 0.05))),
            ("zero std", dict(std=(0.05, 0.05, 0.0))),
            ("negative std", dict(std=(-0.05, 0.05, 0.05))),
            ("from the future", dict(age_s=-0.1)),
            ("older than the age limit", dict(age_s=0.55)),
            ("before the first odometry sample", dict(time_ns=1)),
        ]
        for backend in BACKENDS:
            for name, observation in cases:
                with self.subTest(backend=backend, case=name):
                    chassis = self.start(backend)
                    chassis.drive(1.0, 0.0, 0.0, 0.6)
                    chassis.hold(0.1)
                    before = self.pose()
                    self.observe(x=chassis.x + 1.0, **observation)
                    chassis.hold(0.1)
                    self.assertEqual(self.counts(), (0, 1))
                    self.assertPoseHeld(before)
                    self.assertTrue(self.estimate().valid)

    def test_an_observation_inside_the_limits_is_accepted(self):
        # The edges the rejection cases sit just past.
        for backend in BACKENDS:
            for age_s in (0.45, -0.01):
                with self.subTest(backend=backend, age_s=age_s):
                    chassis = self.start(backend)
                    chassis.drive(1.0, 0.0, 0.0, 0.6)
                    chassis.hold(0.1)
                    before = self.pose()
                    self.observe(x=chassis.x + 0.5, age_s=age_s)
                    chassis.hold(DT)
                    self.assertEqual(self.counts(), (1, 0))
                    self.assertGreater(self.pose()[0], before[0] + 0.1)

    def test_an_observation_before_any_odometry_is_rejected(self):
        for backend in BACKENDS:
            with self.subTest(backend):
                self.sim = Sim.from_toml(robot_toml(backend))
                self.sim.add(estimator.PoseEstimator, "pose_estimator")
                self.hw = SimHardware(self.sim)
                self.hw.enable(False)
                self.hw.set_driver_station(enabled=False, teleop=True)
                self.chassis = Chassis(self.sim)
                self.sim.step(DT)
                self.observe(time_ns=round(self.sim.now * 1e9))
                self.sim.step(DT)
                e = self.estimate()
                self.assertFalse(e.valid)
                self.assertFalse(e.localized)
                self.assertEqual(self.counts(), (0, 1))
                self.assertEqual(self.pose(), (0.0, 0.0, 0.0))

    def test_estimation_continues_while_the_robot_is_disabled(self):
        for backend in BACKENDS:
            with self.subTest(backend):
                chassis = self.start(backend)
                # Gateway disabled, Driver Station disabled: an actuator
                # would send neutral; the estimator keeps estimating.
                chassis.drive(0.5, 0.0, 0.0, 1.0)  # pushed by hand
                self.assertAlmostEqual(self.pose()[0], 0.5, places=4)
                self.observe(x=chassis.x + 0.2)
                chassis.hold(DT)
                self.assertEqual(self.counts(), (1, 0))
                self.assertTrue(self.estimate().valid)

    def test_stale_drivetrain_input_is_invalid_and_holds_the_pose(self):
        for backend in BACKENDS:
            with self.subTest(backend):
                chassis = self.start(backend)
                chassis.drive(1.0, 0.0, 0.0, 0.4)
                held = self.pose()
                self.assertTrue(self.estimate().valid)

                # The drivetrain falls silent (past drivetrain_timeout_us).
                for _ in range(10):
                    chassis.advance(1.0, 0.0, 0.0)
                    self.sim.step(DT)
                e = self.estimate()
                self.assertFalse(e.valid)
                self.assertPoseHeld(held)
                self.assertEqual((e.velocity_vx_mps, e.velocity_omega_radps),
                                 (0.0, 0.0))

                # Back: the wheel travel of the gap is folded in at once.
                chassis.drive(1.0, 0.0, 0.0, DT)
                self.assertTrue(self.estimate().valid)
                self.assertAlmostEqual(self.pose()[0], chassis.x, places=4)

    def test_a_sample_the_gateway_stopped_refreshing_goes_invalid(self):
        for backend in BACKENDS:
            with self.subTest(backend):
                chassis = self.start(backend)
                chassis.drive(1.0, 0.0, 0.0, 0.2)
                held = self.pose()
                stuck = self.estimate().sample_time_ns
                # Fresh messages, but the hardware sample never advances.
                chassis.drive(1.0, 0.0, 0.0, 0.2, sample_time_ns=stuck)
                self.assertFalse(self.estimate().valid)
                self.assertPoseHeld(held)

    def test_invalid_drivetrain_samples_are_not_integrated(self):
        cases = [("modules invalid", dict(modules_valid=False)),
                 ("yaw invalid", dict(yaw_valid=False)),
                 ("non-finite module", dict(distance_m=math.nan)),
                 ("no sample time", dict(sample_time_ns=0))]
        for backend in BACKENDS:
            for name, publish in cases:
                with self.subTest(backend=backend, case=name):
                    chassis = self.start(backend)
                    chassis.drive(1.0, 0.0, 0.0, 0.2)
                    held = self.pose()
                    chassis.drive(1.0, 0.0, 0.0, 0.1, **publish)
                    self.assertFalse(self.estimate().valid)
                    self.assertPoseHeld(held)
                    chassis.drive(1.0, 0.0, 0.0, DT)
                    self.assertTrue(self.estimate().valid)
                    self.assertAlmostEqual(self.pose()[0], chassis.x,
                                           places=4)

    def test_a_repeated_sample_is_integrated_once(self):
        for backend in BACKENDS:
            with self.subTest(backend):
                chassis = self.start(backend)
                chassis.drive(1.0, 0.0, 0.0, 0.2)
                held = self.pose()
                for _ in range(3):  # the same sample, republished
                    chassis.publish(
                        sample_time_ns=self.estimate().sample_time_ns)
                    self.sim.step(DT)
                self.assertTrue(self.estimate().valid)
                self.assertPoseHeld(held)

    def test_only_the_listed_cameras_are_used(self):
        for backend in BACKENDS:
            with self.subTest(backend):
                chassis = self.start(backend)
                chassis.drive(1.0, 0.0, 0.0, 0.4)
                self.observe("side_camera", x=chassis.x + 1.0)  # not listed
                chassis.hold(DT)
                self.assertEqual(self.counts(), (0, 0))
                self.observe("back_camera", x=chassis.x + 0.3)
                chassis.hold(DT)
                self.observe("front_camera", x=chassis.x + 0.3)
                chassis.hold(DT)
                self.assertEqual(self.counts(), (2, 0))

    def test_no_cameras_is_odometry_only(self):
        for backend in BACKENDS:
            with self.subTest(backend):
                chassis = self.start(backend, cameras=())
                chassis.drive(1.0, 0.0, 0.0, 0.4)
                self.observe("front_camera", x=chassis.x + 1.0)
                chassis.hold(DT)
                self.assertEqual(self.counts(), (0, 0))
                self.assertAlmostEqual(self.pose()[0], chassis.x, places=4)

    def test_bad_config_fails_at_startup_naming_the_key(self):
        def module_list(key, values):
            return robot_toml("kalman").replace(
                next(line for line in robot_toml("kalman").splitlines()
                     if line.startswith(key)), f"{key} = {values}")
        cases = [
            ("backend", robot_toml("particle_filter")),
            ("max_observation_age_us",
             robot_toml("kalman").replace("max_observation_age_us = 500000",
                                          "max_observation_age_us = 2000000")),
            ("module_x_m", module_list("module_x_m", "[0.3, 0.3, -0.3]")),
            ("cameras", robot_toml("kalman").replace(
                'cameras = ["front_camera", "back_camera"]', "cameras = [1]")),
            ("kalman.state_std_x_m",
             robot_toml("kalman", extra="kalman.state_std_x_m = 0.0")),
            ("factor_graph.odometry_std_yaw_rad",
             robot_toml("factor_graph",
                        extra="factor_graph.odometry_std_yaw_rad = -1.0")),
        ]
        for key in ("initial_x_m", "initial_y_m", "initial_yaw_rad",
                    "field_length_m", "field_width_m", "max_innovation_m"):
            cases.append((key, robot_toml("kalman", extra=f"{key} = nan")))
        for key, toml in cases:
            with self.subTest(key):
                sim = Sim.from_toml(toml)
                with self.assertRaises(Exception) as caught:
                    sim.add(estimator.PoseEstimator, "pose_estimator")
                self.assertIn(key, str(caught.exception))


class BinaryTest(unittest.TestCase):
    def run_binary(self, *argv):
        resolver = runfiles.Create()
        binary = resolver.Rlocation(os.environ["POSE_ESTIMATOR_BINARY"])
        env = dict(os.environ, **resolver.EnvVars())
        env.pop("PYTHONPATH", None)
        with tempfile.TemporaryDirectory() as cwd:
            proc = subprocess.run([binary, *argv], cwd=cwd, env=env,
                                  capture_output=True, text=True, timeout=120)
        self.assertEqual(proc.returncode, 0, proc.stderr)
        return proc.stdout

    def test_describe_lists_the_drivetrain_and_camera_inputs(self):
        doc = json.loads(self.run_binary("--describe", "--config", CONFIG))
        self.assertEqual(doc["name"], "pose_estimator")
        sources = {(s["kind"], s["name"]): s for s in doc["sources"]}
        self.assertEqual(
            sources[("SENDER", topics.STATE)]["message_bytes"],
            PoseEstimate.SizeOf())
        self.assertEqual(PoseEstimate.SizeOf(), 72)
        self.assertFalse(
            sources[("WATCHER", drivetrain_topics.STATE)]["optional"])
        for camera in ("/front_camera/state", "/back_camera/state"):
            self.assertTrue(sources[("WATCHER", camera)]["optional"])
            self.assertEqual(sources[("WATCHER", camera)]["message_bytes"],
                             PoseObservation.SizeOf())
        self.assertNotIn(("SENDER", "/hw/request/pose_estimator"), sources)

    def test_sim_run_replays_without_divergence(self):
        with tempfile.TemporaryDirectory() as tmp:
            log = os.path.join(tmp, "pose_estimator.tlog")
            self.run_binary("--sim", "--duration-s", "1", "--config", CONFIG,
                            "--log", log)
            self.run_binary("--replay", log, "--config", CONFIG)

    def test_replay_inputs_determinism_and_observation_handling(self):
        resolver = runfiles.Create()
        tool_binary = resolver.Rlocation(
            os.environ["REPLAY_SESSION_TOOL_BINARY"])
        binary = resolver.Rlocation(os.environ["POSE_ESTIMATOR_BINARY"])
        env = dict(os.environ, **resolver.EnvVars())
        env.pop("PYTHONPATH", None)

        with tempfile.TemporaryDirectory() as tmp:
            session_dir = os.path.join(tmp, "session")
            proc = subprocess.run([tool_binary, "generate", session_dir],
                                  capture_output=True, text=True, timeout=30)
            self.assertEqual(proc.returncode, 0, proc.stderr)

            for backend in BACKENDS:
                with self.subTest(backend=backend):
                    cfg_path = os.path.join(tmp, f"robot_{backend}.toml")
                    with open(cfg_path, "w") as f:
                        f.write(robot_toml(backend))

                    log1 = os.path.join(tmp, f"{backend}_1.tlog")
                    log2 = os.path.join(tmp, f"{backend}_2.tlog")

                    for log_path in (log1, log2):
                        proc = subprocess.run(
                            [binary, "--replay-inputs", session_dir,
                             "--config", cfg_path, "--log", log_path,
                             "--session-id", "42"],
                            cwd=tmp, env=env, capture_output=True, text=True,
                            timeout=30)
                        self.assertEqual(proc.returncode, 0, proc.stderr)

                    # (a) outputs are bit-identical for both estimates and record stream
                    records1 = subprocess.run(
                        [tool_binary, "records", log1],
                        capture_output=True, text=True, check=True).stdout
                    records2 = subprocess.run(
                        [tool_binary, "records", log2],
                        capture_output=True, text=True, check=True).stdout
                    self.assertTrue(records1)
                    self.assertEqual(records1, records2)

                    estimates1_raw = subprocess.run(
                        [tool_binary, "estimates", log1],
                        capture_output=True, text=True, check=True).stdout
                    estimates2_raw = subprocess.run(
                        [tool_binary, "estimates", log2],
                        capture_output=True, text=True, check=True).stdout
                    self.assertEqual(estimates1_raw, estimates2_raw)

                    estimates = [json.loads(line) for line in estimates1_raw.splitlines() if line.strip()]
                    self.assertGreater(len(estimates), 40)
                    self.assertTrue(all(
                        not estimate["localized"] for estimate in estimates))

                    # (b) estimated pose changes over the run (non-constant)
                    first_pose = (estimates[0]["pose_x_m"], estimates[0]["pose_y_m"],
                                  estimates[0]["pose_yaw_rad"])
                    last_pose = (estimates[-1]["pose_x_m"], estimates[-1]["pose_y_m"],
                                 estimates[-1]["pose_yaw_rad"])
                    self.assertNotEqual(first_pose, last_pose)
                    self.assertGreater(estimates[-1]["pose_x_m"], 0.5)
                    self.assertGreater(estimates[-1]["pose_yaw_rad"], 0.4)

                    # (c) Delayed vision shifts the estimate toward observation.
                    # The DS listener uses POLL delivery: a publication on an
                    # update boundary may be observed on the next update.
                    # Locate the acceptance transition, not a fixed tick index.
                    accepted_index = next(
                        (index for index, estimate in enumerate(estimates)
                         if estimate["observation_accepted_count"] > 0), None)
                    self.assertIsNotNone(accepted_index)
                    self.assertGreater(accepted_index, 0)
                    before_obs = estimates[accepted_index - 1]
                    after_obs = estimates[accepted_index]
                    self.assertEqual(before_obs["observation_accepted_count"], 0)
                    self.assertEqual(after_obs["observation_accepted_count"], 1)
                    step_forward = after_obs["pose_x_m"] - before_obs["pose_x_m"]
                    self.assertGreater(step_forward, 0.04)

                    # (d) stale measurements are rejected; invalid states are skipped
                    stale_step = estimates[accepted_index + 1]
                    self.assertEqual(stale_step["observation_accepted_count"], 1)
                    self.assertEqual(stale_step["observation_rejected_count"], 1)

                    invalid_step = estimates[accepted_index + 2]
                    self.assertEqual(invalid_step["observation_accepted_count"], 1)
                    self.assertEqual(invalid_step["observation_rejected_count"], 1)

                    self.assertLess(stale_step["pose_x_m"], 1.0)
                    self.assertLess(stale_step["pose_y_m"], 0.1)
                    self.assertLess(invalid_step["pose_x_m"], 1.0)
                    self.assertLess(invalid_step["pose_y_m"], 0.1)

                    self.assertEqual(estimates[-1]["observation_accepted_count"], 1)
                    self.assertEqual(estimates[-1]["observation_rejected_count"], 1)


if __name__ == "__main__":
    unittest.main()
