# pose_estimator

The field pose of the 2026 robot: swerve odometry from `/drivetrain/state`,
corrected by camera pose observations, published every tick (20 ms) as
`geometry.PoseEstimate` on `/pose_estimator/state`. Python
(`talos_py_node`); it owns no devices and keeps estimating while the robot is
disabled. Telemetry draws it through `[subsystems.telemetry] pose_topic`.

## Inputs and output

| Topic | Type | |
|---|---|---|
| `/drivetrain/state` | `drivetrain.DrivetrainState` | `sample_time_ns`, `modules_valid`, `yaw_valid`, `yaw_rad`, `module0`..`module3` (distance, angle), chassis velocity |
| `/<camera>/state`, per name in `cameras` | `geometry.PoseObservation` | optional subscriptions: a listed camera that is not running is no graph error |
| `/pose_estimator/state` | `geometry.PoseEstimate` | pose at the newest integrated `sample_time_ns`, the drivetrain's chassis velocity, `valid`, accepted/rejected observation counts |

The configured sources are `front_left_camera`, `front_right_camera`,
`rear_left_camera` and `rear_right_camera`, built from the shared C++ vision
node. Each also publishes `vision.VisionStatus` on `/<camera>/status`.
Live AVFoundation/V4L2 capture starts on the camera's first update and runs
through a capture callback and processing worker. Under `--sim`, observations
come from rendered/projected tags using drivetrain-integrated truth, never
this estimator's output. Camera intrinsics, distortion, mounts and devices
live in each camera's subsystem file; see [vision](../vision/README.md).

## Backends

`backend` in `configuration/subsystems/pose_estimator.toml` (an unknown value
fails at startup):

- `kalman`: Python `wpimath.estimator.SwerveDrive4PoseEstimator`, a steady-state
  Kalman gain with latency compensation by odometry replay. Tuned by `[kalman]
  state_std_x_m/y_m/yaw_rad`. Odometry goes in through
  `updateWithTime(sample_time_s, ...)` only.
- `factor_graph`: `gtsam.IncrementalFixedLagSmoother` (iSAM2) over one
  `Pose2` per integrated drivetrain sample. `BetweenFactorPose2` from
  `SwerveDrive4Kinematics.toTwist2d` (dx, dy) and the gyro's yaw change
  (dtheta), sigma `[factor_graph] odometry_std_*` per sample; a prior
  (`prior_std_*`) on the configured initial pose; each
  accepted observation is a `PriorFactorPose2` on the sample nearer its
  time, moved by the odometry between the two along the constant twist
  between the bracketing samples (exact for constant velocity within a
  sample interval; nearest-key attachment alone would be off by speed × half
  an interval). Lag window = `max_observation_age_us`. gtsam is imported
  only for this backend: it is 150 MB of native code with a slow cold start.

## Rules (both backends)

- Both backends seed the first integrated sample with `initial_x_m`,
  `initial_y_m`, and `initial_yaw_rad` (finite, default zero), independently
  of the gyro's initial reading; subsequent gyro deltas change that heading.
- Field dimensions `field_length_m` (16.54) and `field_width_m` (8.21)
  must be finite and positive. Observations outside the inclusive field
  rectangle or with no tags are rejected. A single-tag observation more
  than `max_innovation_m` (3.0, finite and positive) from the current estimate
  is rejected; multi-tag solves may relocalize across larger wheel-slip jumps.
- A drivetrain sample is integrated once, when `sample_time_ns` increased,
  `modules_valid` and `yaw_valid` hold and every value is finite.
- `valid` = odometry integrated at least once, the drivetrain message fresh
  (`drivetrain_timeout_us`) and usable, and its sample no older than that
  timeout. Otherwise the pose is held and the velocity reads zero.
- An observation is read once (`updated`) and rejected (counted) when
  `valid` is false, a value is non-finite, a std dev is not positive, no
  odometry has been integrated, it predates the first odometry sample, it
  is more than `observation_future_tolerance_us` newer than the latest
  odometry sample, or older than `max_observation_age_us` (at most 1.5 s,
  wpimath's pose buffer).
- `module_x_m` / `module_y_m` repeat `[subsystems.drivetrain.geometry]
  .modules` (same order); `config_golden_test` fails if they drift.

## Tests

`bazel test //2026-robot/main_processor/pose_estimator:node_test` runs every
scenario against both backends on `talos.testing.Sim`: a straight line then
an arc against the closed-form pose; a delayed observation correcting
wheel-slip drift; an observation between samples placed at its own time;
each rejection case leaving the pose alone; estimation while disabled;
stale, frozen and invalid drivetrain input; repeated samples; `cameras`
wiring; startup errors naming the bad key; `--describe` and a logged sim run
replaying without divergence.
