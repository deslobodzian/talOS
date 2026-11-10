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
  is max_observation_age_us. No GTSAM exception leaves update(): the
  smoother restarts at the last good estimate, and an observation whose
  update failed is rejected.

Invalid/no-tag camera states carry no measurement and are skipped. A measurement
is rejected (and counted) when it carries a non-finite value or a standard
deviation below MIN_OBSERVATION_STD_M / MIN_OBSERVATION_STD_RAD, arrives before
any odometry, was taken before the first odometry sample, is too old or too far
in the future, or is outside the field.
Once localized, measurements pass position and wrapped-yaw innovation gates
against the estimate at their sample time. An untrusted startup seed does not
gate measurements. Localization (and large innovations once localized) requires
three consecutive, mutually consistent measurements within 0.5 s; agreement
uses the same gates after compensating for intervening odometry. With multiple
configured cameras, at least two distinct cameras must supply that evidence.
A confirmed large innovation resets the backend to that measurement, so the
evidence for the old pose cannot outvote it.
"""

import bisect
import collections
import importlib
import math
import operator
import sys

import numpy as np
import talos
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
# Relocalization requires sustained evidence, independent of reported confidence.
RELOCALIZATION_COUNT = 3
RELOCALIZATION_WINDOW_S = 0.5
# A smaller observation standard deviation is no camera's real precision; in
# the factor graph it would only make the linear system ill-conditioned.
MIN_OBSERVATION_STD_M = 1e-4
MIN_OBSERVATION_STD_RAD = 1e-4
# GTSAM's C++ exceptions as pybind11 raises them: its own (an indeterminant
# linear system, a missing or duplicate key) as RuntimeError, std::out_of_range
# key lookups as IndexError. Either leaves the smoother unusable.
_GTSAM_ERRORS = (RuntimeError, IndexError)
# One stderr line per factor-graph failure kind per this much sample time.
_FAILURE_REPORT_INTERVAL_S = 5.0
_NS = 1e9
_US = 1e6
_SAMPLE_TIME = operator.attrgetter("time_s")


def _positive(node, key, default):
    value = node.param(key, default)
    if not (math.isfinite(value) and value > 0):
        raise ValueError(f"pose_estimator: {key} must be positive and "
                         f"finite, got {value}")
    return value


def _initial_pose(node):
    values = []
    for key in ("initial_x_m", "initial_y_m", "initial_yaw_rad"):
        value = node.param(key, 0.0)
        if not math.isfinite(value):
            raise ValueError(f"pose_estimator: {key} must be finite, got {value}")
        values.append(value)
    return Pose2d(values[0], values[1], Rotation2d(values[2]))


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


def _bracket(samples, time_s):
    """(before, after): the retained samples around time_s (equal at the
    ends). Sample times strictly increase, so a binary search finds them."""
    index = min(bisect.bisect_left(samples, time_s, key=_SAMPLE_TIME),
                len(samples) - 1)
    after = samples[index]
    return (samples[index - 1] if index else after), after


def _odometry_at(samples, time_s):
    """Interpolate retained odometry with the samples' constant-twist arc."""
    before, after = _bracket(samples, time_s)
    span = after.time_s - before.time_s
    if span <= 0:
        return after.odometry
    fraction = min(max((time_s - before.time_s) / span, 0.0), 1.0)
    twist = before.odometry.log(after.odometry)
    return before.odometry.exp(Twist2d(twist.dx * fraction,
                                      twist.dy * fraction,
                                      twist.dtheta * fraction))


class KalmanBackend:
    """wpimath.estimator.SwerveDrive4PoseEstimator."""

    def __init__(self, node, kinematics):
        self.kinematics = kinematics
        self.state_std = _std_devs(node, "kalman", "state_std",
                                   (0.1, 0.1, 0.1))
        self.estimator = None
        self.initial_pose = _initial_pose(node)
        self.samples = collections.deque()
        self.yaw = Rotation2d()
        self.positions = None

    def start(self, time_s, yaw, positions):
        self.estimator = SwerveDrive4PoseEstimator(
            self.kinematics, yaw, positions, self.initial_pose,
            self.state_std, (1.0, 1.0, 1.0))
        self.estimator.updateWithTime(time_s, yaw, positions)
        self.samples.append(_Sample(0, time_s, self.initial_pose))
        self.yaw, self.positions = yaw, positions

    def add_odometry(self, time_s, yaw, positions):
        self.estimator.updateWithTime(time_s, yaw, positions)
        wheels = self.kinematics.toTwist2d(self.positions, positions)
        twist = Twist2d(wheels.dx, wheels.dy, (yaw - self.yaw).radians())
        self.samples.append(_Sample(
            0, time_s, self.samples[-1].odometry.exp(twist)))
        cutoff = time_s - MAX_OBSERVATION_AGE_LIMIT_US / _US
        while len(self.samples) > 1 and self.samples[1].time_s < cutoff:
            self.samples.popleft()
        self.yaw, self.positions = yaw, positions

    def add_observation(self, time_s, pose, std_devs):
        self.estimator.addVisionMeasurement(pose, time_s, std_devs)
        return True

    def reset(self, yaw, positions, pose=None):
        """Adopts (yaw, positions) as the odometry baseline without
        integrating the step to them; with `pose`, the estimate continues
        from there. wpimath clears its pose buffer either way, so an
        observation added before the next odometry sample is dropped."""
        self.estimator.resetPosition(
            yaw, positions, self.pose() if pose is None else pose)
        self.yaw, self.positions = yaw, positions

    def pose(self):
        return self.estimator.getEstimatedPosition()

    def pose_at(self, time_s):
        sample_at = getattr(self.estimator, "sampleAt", None)
        if sample_at is not None:
            sampled = sample_at(time_s)
            if isinstance(sampled, Pose2d):
                return sampled
        return self.pose().transformBy(
            Transform2d(self.samples[-1].odometry,
                        _odometry_at(self.samples, time_s)))


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

    A GTSAM exception leaves iSAM2 unusable (the failed factor stays in its
    graph and every later update fails), so none escapes: the smoother
    restarts (_restart) at the last good estimate. A failed odometry step
    is kept; a failed observation is dropped and reported as not fused.
    """

    def __init__(self, node, kinematics, lag_s):
        # Loaded here, for this backend only, so a kalman configuration never
        # loads gtsam's ~150 MB of native code.
        self.gtsam = importlib.import_module("gtsam")
        self.kinematics = kinematics
        self.lag_s = lag_s
        self.odometry_noise = self.gtsam.noiseModel.Diagonal.Sigmas(np.array(
            _std_devs(node, "factor_graph", "odometry_std",
                      (0.01, 0.01, 0.005))))
        self.prior_noise = self.gtsam.noiseModel.Diagonal.Sigmas(np.array(
            _std_devs(node, "factor_graph", "prior_std", (1.0, 1.0, 0.5))))
        self.smoother = None
        # The samples whose keys are still in the smoother, plus the newest
        # marginalized one as the interpolation anchor just past the window
        # and, after a restart, older ones kept for their odometry only.
        self.samples = collections.deque()
        self.origin = None  # the sample the smoother (re)started at
        self.next_key = 0
        self.yaw = Rotation2d()
        self.positions = None
        self.estimate = Pose2d()
        self.initial_pose = _initial_pose(node)
        # kind: (failures since the last report, that report's sample time)
        self.failures = {}

    def start(self, time_s, yaw, positions):
        self._append(time_s, self.initial_pose)
        self._restart(self.initial_pose)
        self.yaw, self.positions = yaw, positions

    def add_odometry(self, time_s, yaw, positions):
        # wpimath SwerveDriveOdometry's step: the wheels' twist, with the
        # gyro's heading change in place of the wheels'.
        wheels = self.kinematics.toTwist2d(self.positions, positions)
        twist = Twist2d(wheels.dx, wheels.dy,
                        (yaw - self.yaw).radians())
        previous = self.samples[-1]
        guess = self.estimate.exp(twist)
        sample = self._append(time_s, previous.odometry.exp(twist))
        graph = self.gtsam.NonlinearFactorGraph()
        graph.add(self.gtsam.BetweenFactorPose2(
            previous.key, sample.key, self._pose2(Pose2d().exp(twist)),
            self.odometry_noise))
        values = self.gtsam.Values()
        values.insert(sample.key, self._pose2(guess))
        stamps = self.gtsam.FixedLagSmootherKeyTimestampMap()
        stamps.insert((sample.key, time_s))
        if not self._update(graph, values, stamps, "odometry"):
            self._restart(guess)
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
        if self._update(graph, self.gtsam.Values(),
                        self.gtsam.FixedLagSmootherKeyTimestampMap(),
                        "observation"):
            return True
        self._restart(self.estimate)
        return False

    def reset(self, yaw, positions, pose=None):
        """Adopts (yaw, positions) as the odometry baseline: each step's
        factor is built from differences, so the graph needs nothing else.
        With `pose`, the smoother restarts there."""
        if pose is not None:
            self._restart(pose)
        self.yaw, self.positions = yaw, positions

    def pose(self):
        return self.estimate

    def pose_at(self, time_s):
        return self.estimate.transformBy(
            Transform2d(self.samples[-1].odometry, self._odometry_at(time_s)))

    def _pose2(self, pose):
        return self.gtsam.Pose2(pose.X(), pose.Y(), pose.rotation().radians())

    def _append(self, time_s, odometry):
        sample = _Sample(self.next_key, time_s, odometry)
        self.samples.append(sample)
        self.next_key += 1
        # The smoother marginalizes keys stamped before latest - lag.
        cutoff = time_s - self.lag_s
        while len(self.samples) > 1 and self.samples[1].time_s < cutoff:
            self.samples.popleft()
        return sample

    def _restart(self, pose):
        """A new smoother holding only the newest sample, at `pose` under the
        prior_std prior, as at startup. Every older key and observation
        prior is gone; older samples keep only their odometry."""
        self.origin = self.samples[-1]
        self.smoother = self.gtsam.IncrementalFixedLagSmoother(
            self.lag_s, self.gtsam.ISAM2Params())
        graph = self.gtsam.NonlinearFactorGraph()
        graph.add(self.gtsam.PriorFactorPose2(
            self.origin.key, self._pose2(pose), self.prior_noise))
        values = self.gtsam.Values()
        values.insert(self.origin.key, self._pose2(pose))
        stamps = self.gtsam.FixedLagSmootherKeyTimestampMap()
        stamps.insert((self.origin.key, self.origin.time_s))
        self.smoother.update(graph, values, stamps)
        self._read_estimate()

    def _update(self, graph, values, stamps, kind):
        """Folds the factors into the smoother and reads the new estimate;
        False, after a report, when GTSAM raised."""
        try:
            self.smoother.update(graph, values, stamps)
            self._read_estimate()
        except _GTSAM_ERRORS as error:
            self._report(kind, error)
            return False
        return True

    def _report(self, kind, error):
        """At most one stderr line per kind and _FAILURE_REPORT_INTERVAL_S
        of sample time, counting the failures since the last one."""
        now_s = self.samples[-1].time_s
        count, reported_s = self.failures.get(kind, (0, -math.inf))
        count += 1
        if now_s - reported_s >= _FAILURE_REPORT_INTERVAL_S:
            message = str(error).strip().split("\n", 1)[0]
            print(f"pose_estimator: {count} factor_graph {kind} update "
                  f"failure(s), smoother restarted at the last good "
                  f"estimate: {message}", file=sys.stderr)
            count, reported_s = 0, now_s
        self.failures[kind] = (count, reported_s)

    def _alive(self, sample):
        """Whether sample's key is in the smoother: neither older than its
        (re)start nor marginalized."""
        return (sample.key >= self.origin.key
                and sample.time_s >= self.samples[-1].time_s - self.lag_s)

    def _attach(self, time_s):
        before, after = _bracket(self.samples, time_s)
        nearer = (before if time_s - before.time_s <= after.time_s - time_s
                  else after)
        if self._alive(nearer):
            return nearer
        # nearer was marginalized, or the smoother restarted after it: the
        # next live sample stands in (the restart sample when after
        # predates the restart too).
        return after if self._alive(after) else self.origin

    def _odometry_at(self, time_s):
        return _odometry_at(self.samples, time_s)

    def _read_estimate(self):
        latest = self.smoother.calculateEstimatePose2(self.samples[-1].key)
        x, y, theta = latest.x(), latest.y(), latest.theta()
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(theta)):
            raise RuntimeError(f"non-finite estimate ({x}, {y}, {theta})")
        self.estimate = Pose2d(x, y, Rotation2d(theta))


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
        # 2026 wheel free speed is 4.63 m/s; default includes >2x margin.
        self.max_odometry_speed_mps = _positive(
            node, "max_odometry_speed_mps", 10.0)
        self.max_odometry_yaw_rate_radps = _positive(
            node, "max_odometry_yaw_rate_radps", 25.0)
        self.field_length_m = _positive(node, "field_length_m", 16.541)
        self.field_width_m = _positive(node, "field_width_m", 8.069)
        self.max_innovation_m = _positive(node, "max_innovation_m", 3.0)
        self.max_yaw_innovation_rad = _positive(
            node, "max_yaw_innovation_rad", 0.2618)
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
        self.relocalization_camera_count = min(2, len(set(cameras)))
        self.cameras = [
            (camera, node.subscribe(
                PoseObservation, f"/{camera}/state", optional=True))
            for camera in cameras]
        self.state = node.publish(PoseEstimate, topics.STATE)
        self.first_ns = None  # first integrated sample; None before it
        self.latest_ns = 0
        self.accepted = 0
        self.rejected = 0
        self.odometry_rejected_count = 0
        self.checked_ns = 0
        self.relocalization_candidates = []
        self.localized = False
        self.seed_trusted = None  # first fresh, attached DS observation only
        self.ds = talos.DriverStation(node, on_packet=self.observe_driver_station)

    def update(self):
        self.observe_driver_station()
        drive = self.integrate()
        if self.first_ns is not None and self.seed_trusted:
            self.localized = True
        for name, camera in self.cameras:
            if camera.updated:
                self.observe(camera.value, name)
        pose = self.backend.pose() if self.first_ns is not None else Pose2d()
        self.state.send(
            sample_time_ns=self.latest_ns,
            pose_x_m=pose.X(), pose_y_m=pose.Y(),
            pose_yaw_rad=pose.rotation().radians(),
            velocity_vx_mps=drive.vx_mps if drive else 0.0,
            velocity_vy_mps=drive.vy_mps if drive else 0.0,
            velocity_omega_radps=drive.omega_radps if drive else 0.0,
            valid=drive is not None,
            localized=self.localized,
            observation_accepted_count=self.accepted,
            observation_rejected_count=self.rejected)

    def observe_driver_station(self):
        # Missing/stale/detached DS data reads disabled, but cannot authorize
        # the configured seed. A later disable cannot trust an enabled-first
        # restart's seed, nor can a mode change revoke established localization.
        if self.seed_trusted is None and self.ds.connected:
            self.seed_trusted = not self.ds.enabled

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
        if (d.sample_time_ns < self.checked_ns
                or (d.sample_time_ns == self.checked_ns
                    and self.checked_ns > self.latest_ns)):
            return None
        if d.sample_time_ns > self.latest_ns:
            time_s, yaw = d.sample_time_ns / _NS, Rotation2d(d.yaw_rad)
            if self.first_ns is None:
                self.backend.start(time_s, yaw, positions)
                self.first_ns = d.sample_time_ns
            else:
                dt = (d.sample_time_ns - self.checked_ns) / _NS
                distance = max(abs(new.distance - old.distance)
                               for old, new in zip(self.backend.positions, positions))
                yaw_step = abs(math.remainder(
                    (yaw - self.backend.yaw).radians(), 2 * math.pi))
                if (distance > self.max_odometry_speed_mps * dt
                        or yaw_step > self.max_odometry_yaw_rate_radps * dt):
                    self.odometry_rejected_count += 1
                    # Adopt the new frame without integrating its jump.
                    self.backend.reset(yaw, positions)
                    self.checked_ns = d.sample_time_ns
                    return None
                self.backend.add_odometry(time_s, yaw, positions)
            self.latest_ns = d.sample_time_ns
            self.checked_ns = d.sample_time_ns
        # A sample the gateway stopped refreshing is as stale as no message.
        if self.node.now - self.latest_ns / _NS > self.drivetrain_timeout_s:
            return None
        return d

    def observe(self, o, camera):
        if not o.valid or o.tag_count < 1:
            return
        if self.acceptable(o, camera) and self.backend.add_observation(
                o.sample_time_ns / _NS,
                Pose2d(o.pose_x_m, o.pose_y_m, Rotation2d(o.pose_yaw_rad)),
                (o.x_std_m, o.y_std_m, o.yaw_std_rad)):
            self.accepted += 1
        else:
            self.rejected += 1

    def acceptable(self, o, camera):
        values = (o.pose_x_m, o.pose_y_m, o.pose_yaw_rad)
        std_devs = (o.x_std_m, o.y_std_m, o.yaw_std_rad)
        if not all(map(math.isfinite, values + std_devs)):
            self.relocalization_candidates.clear()
            return False
        if (min(o.x_std_m, o.y_std_m) < MIN_OBSERVATION_STD_M
                or o.yaw_std_rad < MIN_OBSERVATION_STD_RAD
                or self.first_ns is None):
            self.relocalization_candidates.clear()
            return False
        if (not 0.0 <= o.pose_x_m <= self.field_length_m
                or not 0.0 <= o.pose_y_m <= self.field_width_m):
            self.relocalization_candidates.clear()
            return False
        if o.sample_time_ns < self.first_ns:
            self.relocalization_candidates.clear()
            return False  # nothing to attach it to
        latest_s, time_s = self.latest_ns / _NS, o.sample_time_ns / _NS
        if (time_s - latest_s > self.future_tolerance_s
                or latest_s - time_s > self.max_age_s):
            self.relocalization_candidates.clear()
            return False
        reference_time = min(time_s, latest_s)
        pose = Pose2d(o.pose_x_m, o.pose_y_m, Rotation2d(o.pose_yaw_rad))
        if not self.localized:
            if self.relocalizes(o.sample_time_ns, reference_time, pose, camera):
                self.localized = True
            return True  # never gate a field observation against an untrusted seed
        reference = self.backend.pose_at(reference_time)
        if self.poses_agree(pose, reference):
            self.relocalization_candidates.clear()
            return True
        if not self.relocalizes(o.sample_time_ns, reference_time, pose, camera):
            return False
        # A confirmed new pose: the backend restarts there (moved by the
        # odometry since, to the newest sample), so the evidence for the old
        # pose cannot outvote it.
        samples = self.backend.samples
        self.backend.reset(self.backend.yaw, self.backend.positions,
                           pose.transformBy(Transform2d(
                               _odometry_at(samples, reference_time),
                               samples[-1].odometry)))
        return True

    def poses_agree(self, pose, reference):
        yaw_delta = (pose.rotation() - reference.rotation()).radians()
        return (math.hypot(pose.X() - reference.X(),
                           pose.Y() - reference.Y()) <= self.max_innovation_m
                and abs(math.atan2(math.sin(yaw_delta), math.cos(yaw_delta)))
                <= self.max_yaw_innovation_rad)

    def relocalizes(self, sample_ns, time_s, pose, camera):
        """Confirm distinct, mutually consistent candidates, not a replayed frame.

        Every candidate must agree with every other one, not just its neighbour,
        so a chain of individually small disagreements cannot authorize a jump.
        Both capture and arrival times must fit the confirmation window.
        With multiple configured cameras, at least two must confirm the jump.
        Independent cameras may capture at the same instant, or arrive out of
        capture order; only a camera repeating (or going back to) a capture
        time it already contributed is a replay, and it adds no evidence.
        """
        candidates = self.relocalization_candidates
        now = self.node.now
        if any(c[4] == camera and sample_ns <= c[0] for c in candidates):
            return False
        if candidates:
            times = [c[0] for c in candidates] + [sample_ns]
            if ((max(times) - min(times)) / _NS > RELOCALIZATION_WINDOW_S
                    or now - candidates[0][2] > RELOCALIZATION_WINDOW_S):
                candidates.clear()
        odometry = _odometry_at(self.backend.samples, time_s)
        for _, candidate_odometry, _, candidate_pose, _ in candidates:
            moved = candidate_pose.transformBy(Transform2d(
                candidate_odometry, odometry))
            if not self.poses_agree(pose, moved):
                candidates.clear()
                break
        candidates.append((sample_ns, odometry, now, pose, camera))
        if len(candidates) > RELOCALIZATION_COUNT:
            candidates.pop(0)
        if (len(candidates) < RELOCALIZATION_COUNT
                or len({candidate[4] for candidate in candidates})
                < self.relocalization_camera_count):
            return False
        candidates.clear()
        return True
