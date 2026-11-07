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
| `/pose_estimator/state` | `geometry.PoseEstimate` | pose at the newest integrated `sample_time_ns`, the drivetrain's chassis velocity, `valid`, `localized`, accepted/rejected observation counts |

The configured sources are `front_left_camera`, `front_right_camera`,
`rear_left_camera` and `rear_right_camera`, built from the shared C++ vision
node. Each also publishes `vision.VisionStatus` on `/<camera>/status`.
Live AVFoundation/V4L2 capture starts on the camera's first update and runs
through a capture callback and processing worker. Under `--sim`, observations
come from rendered/projected tags using absolute `/sim/state` truth from
`sim_gateway --physics`, never this estimator's output or measured odometry.
WPILib sim publishes no truth, so simulated camera observations remain invalid.
The gateway owns the world's `--start-pose`; the estimator's independent
`initial_*` seed may differ and be corrected by vision. Camera intrinsics,
distortion, mounts and devices live in each camera's subsystem file;
see [vision](../vision/README.md).

The estimator also reads the Driver Station through `talos.DriverStation`.
Only a fresh, attached packet can classify startup; an absent, stale or
detached Driver Station is not evidence of a normal disabled boot.

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
  The first fresh, attached Driver Station observation decides whether that
  seed is trusted: disabled means normal pre-match boot, so the first
  integrated sample is localized; enabled means a mid-match restart, so the
  seed is untrusted. A later disable never changes that startup decision.
- Field dimensions `field_length_m` (16.54) and `field_width_m` (8.21)
  must be finite and positive. Observations outside the inclusive field
  rectangle are rejected. Once localized, measurements use the position
  (`max_innovation_m`, 3.0 m) and wrapped-yaw (`max_yaw_innovation_rad`,
  0.2618 rad) innovation gates against the estimate at their sample time;
  both limits must be finite and positive. Before localization, these gates
  never compare observations against the untrusted seed: otherwise usable
  observations are accepted and fused immediately, but do not authorize
  field-relative driving until confirmed. Establishing localization from
  vision, or accepting a large innovation once localized, requires three
  consecutive, distinct, mutually consistent measurements within 0.5 s of
  capture and arrival time, compared after odometry compensation; with
  multiple configured cameras they must include at least two distinct
  cameras, with exactly one camera three consistent observations suffice.
  Invalid or no-tag camera states are skipped without incrementing either
  counter; `observation_rejected_count` counts offered measurements refused.
- A drivetrain sample is integrated once, when `sample_time_ns` increased,
  `modules_valid` and `yaw_valid` hold and every value is finite.
- `valid` = odometry integrated at least once, the drivetrain message fresh
  (`drivetrain_timeout_us`) and usable, and its sample no older than that
  timeout. Otherwise the pose is held and the velocity reads zero.
- `localized` = the integrated estimate has a trusted field frame, either
  from the disabled-first startup decision or the camera confirmation above.
  It is false before the first integrated sample. Once true, it stays true
  for the process lifetime, including enable/disable transitions, Driver
  Station loss and stale or invalid odometry. It is not a freshness flag:
  field-pose consumers must require both `valid` and `localized`.
- An observation is read once (`updated`). Invalid or no-tag camera states
  are skipped; offered measurements are rejected (counted) when a value is
  non-finite, a std dev is not positive, no odometry has been integrated, it
  predates the first odometry sample, is more than
  `observation_future_tolerance_us` newer than the latest
  odometry sample, or older than `max_observation_age_us` (at most 1.5 s,
  wpimath's pose buffer).
- `module_x_m` / `module_y_m` repeat `[subsystems.drivetrain.geometry]
  .modules` (same order); `config_golden_test` fails if they drift.

`PoseEstimate` remains 72 bytes with 8-byte alignment: `localized` occupies
existing padding next to `valid`. Telemetry gains one double-valued channel
(8 bytes per value frame); even the documented 1176-byte physics graph before
the additional hardware/simulator exclusions remains below the 1200-byte cap
at 1184 bytes. No additional exclusion is required.

## Tests

`bazel test //2026-robot/main_processor/pose_estimator:node_test` runs every
scenario against both backends on `talos.testing.Sim`: a straight line then
an arc against the closed-form pose; a delayed observation correcting
wheel-slip drift; an observation between samples placed at its own time;
each rejection case leaving the pose alone; yaw acceptance, rejection and ±π
wrapping, including multi-tag outlier rejection, sustained (multi-camera)
relocalization, inconsistent/expired/replayed evidence, motion compensation,
and skipped non-measurements; disabled-first seed trust, enabled-first restart
recovery without seed innovation gating, cross-camera startup confirmation,
missing/detached DS startup and process-lifetime localization latching;
estimation while disabled; stale, frozen and invalid drivetrain input; repeated samples; `cameras`
wiring; startup errors naming the bad key; `--describe` and a logged sim run
replaying without divergence.
