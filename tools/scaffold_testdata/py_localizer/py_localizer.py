"""PyLocalizer: a pose estimator.

Reads geometry.PoseObservation from each camera node listed in `cameras`
(/<camera>/state) and publishes geometry.PoseEstimate on its own state
topic every tick. As generated it follows the newest usable observation
(the most certain one when several arrive in one tick) and is valid while
one arrived within observation_timeout_us; fuse odometry here (see
2026-robot/main_processor/pose_estimator, which runs wpimath's
SwerveDrive4PoseEstimator or a GTSAM smoother). It drives nothing, so it
runs whether the robot is enabled or not.
"""

import math

from py_localizer import topics
from geometry.PoseEstimate import PoseEstimate
from geometry.PoseObservation import PoseObservation


def usable(o):
    values = (o.pose_x_m, o.pose_y_m, o.pose_yaw_rad, o.x_std_m, o.y_std_m,
              o.yaw_std_rad)
    return (o.valid and o.sample_time_ns > 0
            and all(map(math.isfinite, values))
            and min(o.x_std_m, o.y_std_m, o.yaw_std_rad) > 0)


class PyLocalizer:
    def __init__(self, node):
        self.node = node
        cameras = node.param("cameras")
        if (not isinstance(cameras, list)
                or not all(isinstance(c, str) and c for c in cameras)):
            raise ValueError(f"py_localizer: cameras must list camera node names, "
                             f"got {cameras!r}")
        self.cameras = [
            node.subscribe(PoseObservation, f"/{camera}/state", optional=True)
            for camera in cameras]
        self.timeout_s = node.param("observation_timeout_us", 500_000) / 1e6
        self.state = node.publish(PoseEstimate, topics.STATE)
        self.pose = (0.0, 0.0, 0.0)  # held while no usable observation arrives
        self.sample_time_ns = 0
        self.last_accepted_s = None
        self.accepted = self.rejected = 0

    def update(self):
        best = None
        for camera in self.cameras:
            if not camera.updated:  # each observation counts once
                continue
            o = camera.value
            if not usable(o):
                self.rejected += 1
                continue
            self.accepted += 1
            if best is None or (o.x_std_m + o.y_std_m
                                < best.x_std_m + best.y_std_m):
                best = o
        if best is not None:
            self.pose = (best.pose_x_m, best.pose_y_m, best.pose_yaw_rad)
            self.sample_time_ns = best.sample_time_ns
            self.last_accepted_s = self.node.now
        valid = (self.last_accepted_s is not None
                 and self.node.now - self.last_accepted_s <= self.timeout_s)
        x, y, yaw = self.pose
        self.state.send(sample_time_ns=self.sample_time_ns, pose_x_m=x,
                        pose_y_m=y, pose_yaw_rad=yaw, valid=valid,
                        observation_accepted_count=self.accepted,
                        observation_rejected_count=self.rejected)
