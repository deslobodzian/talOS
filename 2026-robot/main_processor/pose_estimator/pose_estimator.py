"""Pose estimator: swerve odometry fused with camera pose observations.

Every tick the estimator folds the newest drivetrain sample (module positions
and gyro yaw, taken at sample_time_ns) into its estimate, adds each newly
arrived camera observation that passes the checks below, and publishes the
field pose on /pose_estimator/state. It owns no actuators and runs whether
the robot is enabled or not.

Two backends share this shell, its checks and its tests; `backend` picks one:

- "kalman": wpimath's SwerveDrive4PoseEstimator, a steady-state Kalman gain
  with latency compensation by odometry replay. Odometry goes in through
  updateWithTime(sample time), never update(), which reads a wall clock.
- "factor_graph": a GTSAM fixed-lag smoother (iSAM2) over one Pose2 per
  integrated drivetrain sample: wheel-odometry BetweenFactors, a prior on the
  first pose, and one PriorFactor per accepted observation. The lag window
  is max_observation_age_us.

An observation is rejected (and counted) when it is marked invalid, carries
a non-finite value or a non-positive standard deviation, arrives before any
odometry, was taken before the first odometry sample, is newer than the
latest odometry sample by more than observation_future_tolerance_us, or is
older than it by more than max_observation_age_us.
"""

import collections
import importlib
import math

import numpy as np
from drivetrain import topics as drivetrain_topics
from drivetrain.DrivetrainState import DrivetrainState
from geometry.PoseEstimate import PoseEstimate
from geometry.PoseObservation import PoseObservation
from pose_estimator import topics
from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import (Pose2d, Rotation2d, Transform2d, Translation2d,
                              Twist2d)
from wpimath.kinematics import SwerveDrive4Kinematics, SwerveModulePosition

# wpimath's pose buffer (SwerveDrivePoseEstimator kBufferDuration): an older
# observation could not be replayed against odometry.
MAX_OBSERVATION_AGE_LIMIT_US = 1_500_000
MODULE_COUNT = 4
_NS = 1e9
_US = 1e6


def _positive(node, key, default):
    value = node.param(key, default)
    if not (math.isfinite(value) and value > 0):
        raise ValueError(f"pose_estimator: {key} must be positive and "
                         f"finite, got {value}")
    return value


def _std_devs(node, table, prefix, defaults):
    return tuple(_positive(node, f"{table}.{prefix}_{axis}", default)
                 for axis, default in zip(("x_m", "y_m", "yaw_rad"), defaults))


def _module_offsets(node):
    """(x_m, y_m) per module, in drivetrain geometry module order."""
    lists = {}
    for key in ("module_x_m", "module_y_m"):
        values = node.param(key)
        if (not isinstance(values, list) or len(values) != MODULE_COUNT
                or not all(math.isfinite(v) for v in values)):
            raise ValueError(f"pose_estimator: {key} must list "
                             f"{MODULE_COUNT} finite offsets, got {values!r}")
        lists[key] = values
    return list(zip(lists["module_x_m"], lists["module_y_m"]))


def module_positions(state):
    """The four SwerveModulePositions of a DrivetrainState, or None when a
    value is non-finite."""
    positions = []
    for i in range(MODULE_COUNT):
        distance = getattr(state, f"module{i}_distance_m")
        angle = getattr(state, f"module{i}_angle_rad")
        if not (math.isfinite(distance) and math.isfinite(angle)):
            return None
        positions.append(SwerveModulePosition(distance, Rotation2d(angle)))
    return tuple(positions)


class KalmanBackend:
    """wpimath.estimator.SwerveDrive4PoseEstimator."""

    def __init__(self, node, kinematics):
        self.kinematics = kinematics
        self.state_std = _std_devs(node, "kalman", "state_std",
                                   (0.1, 0.1, 0.1))
        self.estimator = None

    def start(self, time_s, yaw, positions):
        # The field heading starts at the gyro's; x and y at the origin.
        self.estimator = SwerveDrive4PoseEstimator(
            self.kinematics, yaw, positions, Pose2d(0.0, 0.0, yaw),
            self.state_std, (1.0, 1.0, 1.0))
        self.estimator.updateWithTime(time_s, yaw, positions)

    def add_odometry(self, time_s, yaw, positions):
        self.estimator.updateWithTime(time_s, yaw, positions)

    def add_observation(self, time_s, pose, std_devs):
        self.estimator.addVisionMeasurement(pose, time_s, std_devs)

    def pose(self):
        return self.estimator.getEstimatedPosition()


# One integrated drivetrain sample: its graph key, when it was taken and the
# dead-reckoned pose there (odometry alone, before any observation).
_Sample = collections.namedtuple("_Sample", "key time_s odometry")


class FactorGraphBackend:
    """gtsam.IncrementalFixedLagSmoother over one Pose2 per odometry sample.

    An observation taken between two samples is attached to the nearer one
    as a PriorFactorPose2, after moving it by the odometry between its own
    time and that sample's. The dead-reckoned pose at the observation time
    follows the constant twist between the two samples (the same arc the
    odometry step assumes), so the attachment adds no error for motion of
    constant velocity across one sample interval; otherwise it adds the
    deviation from that arc over at most half an interval (centimetres only
    under extreme acceleration at 20 ms samples), plus nothing of the time
    offset, which nearest-key attachment alone would cost: speed times half
    an interval, 4 cm at 4 m/s.
    """

    def __init__(self, node, kinematics, lag_s):
        # Loaded here, for this backend only: gtsam is 150 MB of native code
        # that takes seconds to load on a cold start, longer than the
        # launcher's --describe probe allows.
        self.gtsam = importlib.import_module("gtsam")
        self.kinematics = kinematics
        self.lag_s = lag_s
        self.odometry_noise = self.gtsam.noiseModel.Diagonal.Sigmas(np.array(
            _std_devs(node, "factor_graph", "odometry_std",
                      (0.01, 0.01, 0.005))))
        self.prior_noise = self.gtsam.noiseModel.Diagonal.Sigmas(np.array(
            _std_devs(node, "factor_graph", "prior_std", (1.0, 1.0, 0.5))))
        self.smoother = self.gtsam.IncrementalFixedLagSmoother(
            lag_s, self.gtsam.ISAM2Params())
        # The samples whose keys are still in the smoother, plus the newest
        # marginalized one as the interpolation anchor just past the window.
        self.samples = collections.deque()
        self.next_key = 0
        self.yaw = Rotation2d()
        self.positions = None
        self.estimate = Pose2d()

    def start(self, time_s, yaw, positions):
        origin = Pose2d(0.0, 0.0, yaw)
        graph = self.gtsam.NonlinearFactorGraph()
        graph.add(self.gtsam.PriorFactorPose2(0, self._pose2(origin),
                                              self.prior_noise))
        self._add_sample(graph, time_s, origin, origin)
        self.yaw, self.positions = yaw, positions

    def add_odometry(self, time_s, yaw, positions):
        # wpimath SwerveDriveOdometry's step: the wheels' twist, with the
        # gyro's heading change in place of the wheels'.
        wheels = self.kinematics.toTwist2d(self.positions, positions)
        twist = Twist2d(wheels.dx, wheels.dy,
                        (yaw - self.yaw).radians())
        step = Pose2d().exp(twist)
        previous = self.samples[-1]
        graph = self.gtsam.NonlinearFactorGraph()
        graph.add(self.gtsam.BetweenFactorPose2(
            previous.key, self.next_key, self._pose2(step),
            self.odometry_noise))
        self._add_sample(graph, time_s, previous.odometry.exp(twist),
                         self.estimate.exp(twist))
        self.yaw, self.positions = yaw, positions

    def add_observation(self, time_s, pose, std_devs):
        sample = self._attach(time_s)
        # The observed pose moved by the odometry from time_s to the sample.
        moved = pose.transformBy(
            Transform2d(self._odometry_at(time_s), sample.odometry))
        graph = self.gtsam.NonlinearFactorGraph()
        graph.add(self.gtsam.PriorFactorPose2(
            sample.key, self._pose2(moved),
            self.gtsam.noiseModel.Diagonal.Sigmas(np.array(std_devs))))
        self.smoother.update(graph, self.gtsam.Values(),
                             self.gtsam.FixedLagSmootherKeyTimestampMap())
        self._read_estimate()

    def pose(self):
        return self.estimate

    def _pose2(self, pose):
        return self.gtsam.Pose2(pose.X(), pose.Y(), pose.rotation().radians())

    def _add_sample(self, graph, time_s, odometry, guess):
        values = self.gtsam.Values()
        values.insert(self.next_key, self._pose2(guess))
        stamps = self.gtsam.FixedLagSmootherKeyTimestampMap()
        stamps.insert((self.next_key, time_s))
        self.smoother.update(graph, values, stamps)
        self.samples.append(_Sample(self.next_key, time_s, odometry))
        self.next_key += 1
        # The smoother marginalizes keys stamped before latest - lag.
        cutoff = time_s - self.lag_s
        while len(self.samples) > 1 and self.samples[1].time_s < cutoff:
            self.samples.popleft()
        self._read_estimate()

    def _alive(self, sample):
        return sample.time_s >= self.samples[-1].time_s - self.lag_s

    def _bracket(self, time_s):
        """(before, after): the samples around time_s (equal at the ends)."""
        after = next((s for s in self.samples if s.time_s >= time_s),
                     self.samples[-1])
        index = self.samples.index(after)
        before = self.samples[index - 1] if index > 0 else after
        return before, after

    def _attach(self, time_s):
        before, after = self._bracket(time_s)
        nearer = (before if time_s - before.time_s <= after.time_s - time_s
                  else after)
        return nearer if self._alive(nearer) else after

    def _odometry_at(self, time_s):
        before, after = self._bracket(time_s)
        span = after.time_s - before.time_s
        if span <= 0:
            return after.odometry
        fraction = min(max((time_s - before.time_s) / span, 0.0), 1.0)
        twist = before.odometry.log(after.odometry)
        return before.odometry.exp(Twist2d(twist.dx * fraction,
                                           twist.dy * fraction,
                                           twist.dtheta * fraction))

    def _read_estimate(self):
        latest = self.smoother.calculateEstimatePose2(self.samples[-1].key)
        self.estimate = Pose2d(latest.x(), latest.y(),
                               Rotation2d(latest.theta()))


class PoseEstimator:
    def __init__(self, node):
        self.node = node
        kinematics = SwerveDrive4Kinematics(*(
            Translation2d(x, y) for x, y in _module_offsets(node)))
        max_age_us = node.param("max_observation_age_us",
                                MAX_OBSERVATION_AGE_LIMIT_US)
        if not 0 < max_age_us <= MAX_OBSERVATION_AGE_LIMIT_US:
            raise ValueError(
                f"pose_estimator: max_observation_age_us must be in "
                f"(0, {MAX_OBSERVATION_AGE_LIMIT_US}] (wpimath's pose "
                f"buffer), got {max_age_us}")
        self.max_age_s = max_age_us / _US
        self.future_tolerance_s = _positive(
            node, "observation_future_tolerance_us", 20_000) / _US
        self.drivetrain_timeout_s = _positive(
            node, "drivetrain_timeout_us", 100_000) / _US
        backend = node.param("backend")
        if backend == "kalman":
            self.backend = KalmanBackend(node, kinematics)
        elif backend == "factor_graph":
            self.backend = FactorGraphBackend(node, kinematics, self.max_age_s)
        else:
            raise ValueError(f"pose_estimator: backend must be \"kalman\" or "
                             f"\"factor_graph\", got {backend!r}")
        self.drive = node.subscribe(DrivetrainState, drivetrain_topics.STATE)
        cameras = node.param("cameras")
        if (not isinstance(cameras, list)
                or not all(isinstance(c, str) and c for c in cameras)):
            raise ValueError(f"pose_estimator: cameras must list camera node "
                             f"names, got {cameras!r}")
        self.cameras = [
            node.subscribe(PoseObservation, f"/{camera}/state", optional=True)
            for camera in cameras]
        self.state = node.publish(PoseEstimate, topics.STATE)
        self.first_ns = None  # first integrated sample; None before it
        self.latest_ns = 0
        self.accepted = 0
        self.rejected = 0

    def update(self):
        drive = self.integrate()
        for camera in self.cameras:
            if camera.updated:
                self.observe(camera.value)
        pose = self.backend.pose() if self.first_ns is not None else Pose2d()
        self.state.send(
            sample_time_ns=self.latest_ns,
            pose_x_m=pose.X(), pose_y_m=pose.Y(),
            pose_yaw_rad=pose.rotation().radians(),
            velocity_vx_mps=drive.vx_mps if drive else 0.0,
            velocity_vy_mps=drive.vy_mps if drive else 0.0,
            velocity_omega_radps=drive.omega_radps if drive else 0.0,
            valid=drive is not None,
            observation_accepted_count=self.accepted,
            observation_rejected_count=self.rejected)

    def integrate(self):
        """Folds in a new drivetrain sample; returns the drivetrain state
        when the estimate is valid this tick (fresh, usable, integrated),
        else None."""
        if not self.drive.fresh(self.drivetrain_timeout_s):
            return None
        d = self.drive.value
        positions = module_positions(d)
        if not (d.modules_valid and d.yaw_valid and d.sample_time_ns > 0
                and positions is not None and math.isfinite(d.yaw_rad)):
            return None
        if d.sample_time_ns > self.latest_ns:
            time_s, yaw = d.sample_time_ns / _NS, Rotation2d(d.yaw_rad)
            if self.first_ns is None:
                self.backend.start(time_s, yaw, positions)
                self.first_ns = d.sample_time_ns
            else:
                self.backend.add_odometry(time_s, yaw, positions)
            self.latest_ns = d.sample_time_ns
        # A sample the gateway stopped refreshing is as stale as no message.
        if self.node.now - self.latest_ns / _NS > self.drivetrain_timeout_s:
            return None
        return d

    def observe(self, o):
        if self.acceptable(o):
            self.backend.add_observation(
                o.sample_time_ns / _NS,
                Pose2d(o.pose_x_m, o.pose_y_m, Rotation2d(o.pose_yaw_rad)),
                (o.x_std_m, o.y_std_m, o.yaw_std_rad))
            self.accepted += 1
        else:
            self.rejected += 1

    def acceptable(self, o):
        values = (o.pose_x_m, o.pose_y_m, o.pose_yaw_rad)
        std_devs = (o.x_std_m, o.y_std_m, o.yaw_std_rad)
        if not o.valid or not all(map(math.isfinite, values + std_devs)):
            return False
        if min(std_devs) <= 0 or self.first_ns is None:
            return False
        if o.sample_time_ns < self.first_ns:
            return False  # nothing to attach it to
        latest_s, time_s = self.latest_ns / _NS, o.sample_time_ns / _NS
        return (time_s - latest_s <= self.future_tolerance_s
                and latest_s - time_s <= self.max_age_s)
