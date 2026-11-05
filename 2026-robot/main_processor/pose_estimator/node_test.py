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
    def start(self, backend, *, disabled_hardware=False, **config):
        self.sim = Sim.from_toml(robot_toml(backend, **config))
        self.sim.add(estimator.PoseEstimator, "pose_estimator")
        if disabled_hardware:
            # A live gateway and Driver Station, both saying disabled.
            self.hw = SimHardware(self.sim)
            self.hw.enable(False)
            self.hw.set_driver_station(enabled=False, teleop=True)
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
                chassis.hold(DT)
                self.assertPoseHeld((2.0, 3.0, 1.0))

    def test_tag_bounds_and_innovation_gates(self):
        for backend in BACKENDS:
            for observation in (dict(tag_count=0), dict(x=-0.01),
                                dict(x=16.55), dict(y=-0.01),
                                dict(y=8.22), dict(x=4.0, tag_count=1)):
                with self.subTest(backend=backend, observation=observation):
                    chassis = self.start(backend)
                    self.observe(**observation)
                    chassis.hold(DT)
                    self.assertEqual(self.counts(), (0, 1))
                    self.assertPoseHeld((0.0, 0.0, 0.0))
            with self.subTest(backend=backend, relocalization=True):
                chassis = self.start(backend)
                self.observe(x=4.0, tag_count=2)
                chassis.hold(DT)
                self.assertEqual(self.counts(), (1, 0))
                self.assertGreater(self.pose()[0], 0.1)

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
            ("marked invalid", dict(valid=False)),
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
                self.chassis = Chassis(self.sim)
                self.sim.step(DT)
                self.observe(time_ns=round(self.sim.now * 1e9))
                self.sim.step(DT)
                e = self.estimate()
                self.assertFalse(e.valid)
                self.assertEqual(self.counts(), (0, 1))
                self.assertEqual(self.pose(), (0.0, 0.0, 0.0))

    def test_estimation_continues_while_the_robot_is_disabled(self):
        for backend in BACKENDS:
            with self.subTest(backend):
                chassis = self.start(backend, disabled_hardware=True)
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

                    # (b) estimated pose changes over the run (non-constant)
                    first_pose = (estimates[0]["pose_x_m"], estimates[0]["pose_y_m"],
                                  estimates[0]["pose_yaw_rad"])
                    last_pose = (estimates[-1]["pose_x_m"], estimates[-1]["pose_y_m"],
                                 estimates[-1]["pose_yaw_rad"])
                    self.assertNotEqual(first_pose, last_pose)
                    self.assertGreater(estimates[-1]["pose_x_m"], 0.5)
                    self.assertGreater(estimates[-1]["pose_yaw_rad"], 0.4)

                    # (c) delayed vision measurement shifts estimate toward observation
                    before_obs = estimates[16]
                    after_obs = estimates[17]
                    self.assertEqual(before_obs["observation_accepted_count"], 0)
                    self.assertEqual(after_obs["observation_accepted_count"], 1)
                    step_forward = after_obs["pose_x_m"] - before_obs["pose_x_m"]
                    self.assertGreater(step_forward, 0.04)

                    # (d) stale/invalid observations are rejected and don't change estimate
                    stale_step = estimates[18]
                    self.assertEqual(stale_step["observation_accepted_count"], 1)
                    self.assertEqual(stale_step["observation_rejected_count"], 1)

                    invalid_step = estimates[19]
                    self.assertEqual(invalid_step["observation_accepted_count"], 1)
                    self.assertEqual(invalid_step["observation_rejected_count"], 2)

                    self.assertLess(stale_step["pose_x_m"], 1.0)
                    self.assertLess(stale_step["pose_y_m"], 0.1)
                    self.assertLess(invalid_step["pose_x_m"], 1.0)
                    self.assertLess(invalid_step["pose_y_m"], 0.1)

                    self.assertEqual(estimates[-1]["observation_accepted_count"], 1)
                    self.assertEqual(estimates[-1]["observation_rejected_count"], 2)


if __name__ == "__main__":
    unittest.main()
