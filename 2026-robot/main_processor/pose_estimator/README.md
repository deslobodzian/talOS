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
| `/<camera>/state`, per name in `cameras` | `geometry.PoseObservation` | optional callback subscriptions, delivered in sequence: a listed camera that is not running is no graph error |
| `/pose_estimator/state` | `geometry.PoseEstimate` | pose at the newest integrated `sample_time_ns`, the drivetrain's chassis velocity, `valid`, `localized`, accepted/rejected observation counts |

Camera calibration and capture behavior are documented in
[vision](../vision/README.md).

The estimator also reads the Driver Station through `talos.DriverStation`,
in `update()` (no packet listener). Only a fresh, attached packet can
classify startup; an absent, stale or detached Driver Station is not evidence
of a normal disabled boot.

Every camera observation is delivered: each camera subscription has a
callback, so the framework delivers its messages in sequence, and the
callback queues each valid, tagged frame (at most 64 per camera; on overflow
the oldest is dropped and counted in `observation_rejected_count`). The next
`update()` fuses the whole queue in capture-time order. A latest-value read
would lose all but each camera's newest frame per 20 ms tick (cameras publish
every frame, up to 60 fps) and let an invalid frame hide a valid one. The
callbacks are opened with `wake=talos.Wake.AT_DRAIN`: nothing reads the
queues before `update()`, so a camera publication wakes nothing and its
frames are delivered by the drain right before the next `update()` (see
[targets](../../../ARCHITECTURE.md#targets)), at most 64 per camera per
drain; each drain is native, and Python runs only per delivered frame.

## Backends

`backend` in `configuration/subsystems/pose_estimator.toml` selects the
implementation (an unknown value fails at startup). The robot config selects
`"factor_graph"`. The launcher's `--describe` probe (`node.describing`)
validates the configuration and declares the same endpoints but builds no
backend and imports neither wpimath nor gtsam, which are most of a cold
probe's time.

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
  an interval). Lag window = `max_observation_age_us`. GTSAM is imported
  only for this backend.
  No GTSAM exception (`RuntimeError`; `IndexError` for a missing key)
  leaves `update()`. A failed update leaves iSAM2 unusable, so the smoother
  restarts holding only the newest sample, at the last good estimate under
  the `prior_std_*` prior: a failed odometry step is kept, a failed
  observation is rejected (counted). Older keys and observation priors are
  dropped; an observation taken before the restart attaches to the restart
  sample. stderr gets at most one line per failure kind (odometry,
  observation) per 5 s of sample time.

## Rules (both backends)

- Both backends seed the first integrated sample with `initial_x_m`,
  `initial_y_m`, and `initial_yaw_rad` (finite, default zero), independently
  of the gyro's initial reading; subsequent gyro deltas change that heading.
  That seed is the blue-origin pose. The Driver Station decides whether it is
  trusted: a fresh, attached, disabled view naming the blue or red alliance
  trusts that alliance's seed, and a disabled view with no known alliance
  decides nothing. On red the seed is field-flipped the way autonomous flips
  its blue-origin targets, a rotation by π about the field centre:
  (`field_length_m` − x, `field_width_m` − y, yaw + π). Trusting a seed
  restarts the backend at it, moved by all odometry integrated since startup
  (so a robot pushed while disabled keeps that motion, mirrored on red);
  observations fused before it are discarded with the old estimate. Until the
  first enabled view, an alliance change re-seeds, unless cameras have
  confirmed a pose. The first fresh, attached enabled view closes the
  decision: a seed trusted before it stays, an enabled-first restart never
  trusts one, and a later disable changes neither.
  Localization becomes true once both a trusted seed and an integrated sample
  exist.
- Field dimensions `field_length_m` and `field_width_m` (set in
  [`pose_estimator.toml`](../configuration/subsystems/pose_estimator.toml)
  from the field layout `config_golden_test` checks them against)
  must be finite and positive. Observations outside the inclusive field
  rectangle are rejected. Once localized, measurements use the position
  (`max_innovation_m`) and wrapped-yaw (`max_yaw_innovation_rad`) innovation
  gates from the same file against the estimate at their sample time;
  both limits must be finite and positive. Before localization, these gates
  never compare observations against the untrusted seed: otherwise usable
  observations are accepted and fused immediately, but do not authorize
  field-relative driving until confirmed. Establishing localization from
  vision, or accepting a large innovation once localized, requires three
  consecutive, distinct, mutually consistent measurements within 0.5 s of
  capture and arrival time, compared after odometry compensation; with
  multiple configured cameras they must include at least two distinct
  cameras, with exactly one camera three consistent observations suffice.
  Accepting a large innovation resets the backend to that measurement,
  moved by odometry to the newest sample: wpimath's `resetPosition` there,
  or the factor-graph restart above at that pose plus the measurement's own
  prior. Evidence for the old pose, up to a full lag window of factor-graph
  priors, cannot outvote it. After a Kalman reset (a relocalization, a
  trusted seed, a rejected odometry step) wpimath's pose buffer starts at the
  newest sample, so an observation taken earlier is moved by the odometry
  since its time and fused there, as the factor graph attaches it to its
  restart sample.
  Invalid or no-tag camera states are skipped without incrementing either
  counter; `observation_rejected_count` counts offered measurements refused.
- A drivetrain sample is integrated once, when `sample_time_ns` increased,
  `modules_valid` and `yaw_valid` hold and every value is finite.
  Each wheel's implied speed must be at most `max_odometry_speed_mps` (10 m/s);
  wrapped gyro steps must be at most
  `max_odometry_yaw_rate_radps` (25 rad/s). Both limits must be positive and
  finite. Rejected steps hold the pose, invalidate that tick, and increment
  the node's `odometry_rejected_count` (not an observation rejection or an
  extra telemetry channel). The new encoder/yaw frame becomes the baseline
  so later physically plausible motion recovers without integrating the jump.
  Repeated rejected frames and older timestamps stay invalid without counting
  again or changing that baseline.
- `valid` = odometry integrated at least once, the drivetrain message fresh
  (`drivetrain_timeout_us`) and usable, and its sample no older than that
  timeout. Otherwise the pose is held and the velocity reads zero.
- `localized` = the integrated estimate has a trusted field frame, either
  from a trusted alliance seed or the camera confirmation above.
  It is false before the first integrated sample. Once true, it stays true
  for the process lifetime, including enable/disable transitions, Driver
  Station loss and stale or invalid odometry. It is not a freshness flag:
  field-pose consumers must require both `valid` and `localized`.
- Each observation is fused once, in the `update()` after it arrives.
  Invalid or no-tag camera states are skipped; offered measurements are
  rejected (counted) when their camera's queue overflowed, a value is
  non-finite, a std dev is below 1e-4 m (x, y) or 1e-4 rad (yaw), finer
  than any camera resolves (in the factor graph such a sigma only
  ill-conditions the system), no odometry has been integrated, it
  predates the first odometry sample, is more than
  `observation_future_tolerance_us` newer than the latest
  odometry sample, or older than `max_observation_age_us` (at most 1.5 s,
  wpimath's pose buffer).
- The module offsets are the drivetrain's own, read with
  `node.param_of("drivetrain", "geometry.modules[i].x_m")` (and `y_m`) from
  `[subsystems.drivetrain.geometry].modules`, in that order; it must list
  four modules.

## Update benchmark

```sh
bazel run //2026-robot/main_processor/pose_estimator:update_benchmark -- \
  [--backend kalman|factor_graph|both] [--seconds N]
```

runs the `PoseEstimator` class on `talos.testing.Sim` over the production
`robot.toml` with only `backend` replaced (default both, one after the
other), and times every `update()` with `time.perf_counter_ns`. The
workload, with a fixed seed, lasts `--seconds` simulated seconds per backend
(default 900):

- the drivetrain state at the drivetrain's configured period, from a
  Lissajous sweep that starts at the configured initial pose (up to
  ~3.7 m/s wheel speed, 1.35 rad/s), with per-wheel slip and gyro noise;
- every configured camera capturing at ~60 Hz, each frame the true pose plus
  noise (2–8 cm, 0.01–0.03 rad) that it reports as its std devs, arriving
  20–35 ms after capture, 15 % of frames a further 20–100 ms late (after
  newer frames, so they attach to past keys);
- a disabled Driver Station on the blue alliance, so the estimate is
  localized from the first sample and every observation is fused.

As in production, a tick fuses every observation that arrived since the
previous one (about five with four 60 fps cameras at 20 ms). The report
gives the host (platform, CPU model, Python, CPU count) and package
versions, then per backend: ticks, p50, p99, p99.9 and max `update()`
duration, the fraction of ticks longer than the period (`>period`),
observations fused per tick (mean, and the fraction of ticks that fused at
least four), the maximum during the first lag window (warm-up, excluded from
the other statistics) and the final position error. It
exits non-zero when an observation or odometry step was rejected or the
final estimate is not valid and localized, i.e. the workload was not the
one described; it never judges timing. `update_benchmark_test` runs it for
3 s per backend.

It measures `update()` alone in one process, not the camera callbacks that
queue observations. Like `talos.run`, it freezes the
garbage collector's heap once the estimator is built (see
[Node API: Python garbage collection](../../../docs/NODE_DEVELOPMENT_GUIDE.md#python-garbage-collection)).
It does not measure the node loop's dispatch or lateness, real-time
scheduling, other nodes competing for the CPU, the production process's
allocation pattern (the harness's allocations also trigger collections, which
land in the measured ticks), or real camera timing.

## Tests

`bazel test //2026-robot/main_processor/pose_estimator:node_test` runs each
scenario against both backends on `talos.testing.Sim`: a straight line then
an arc against the closed-form pose; a delayed observation correcting
wheel-slip drift; an observation between samples placed at its own time;
each rejection case leaving the pose alone; yaw acceptance, rejection and ±π
wrapping, including multi-tag outlier rejection, sustained (multi-camera)
relocalization, relocalization against a full 1.5 s window of observations
of the old pose, inconsistent/expired/replayed evidence, motion compensation,
and skipped non-measurements; every observation arriving between two ticks
fused, past a trailing invalid frame; the alliance seed (untrusted before an
alliance is known, red field-flipped, re-seeded on an alliance change while
disabled with odometry since startup kept, fixed once enabled),
enabled-first restart recovery without seed innovation gating, cross-camera
startup confirmation,
missing/detached DS startup and process-lifetime localization latching;
estimation while disabled; stale, frozen and invalid drivetrain input; repeated samples; `cameras`
wiring; startup errors naming the bad key; `--describe` and a logged sim run
replaying without divergence. For `factor_graph` alone (Kalman has no such
failure), a raising smoother update rejects that observation or keeps that
odometry step, and stale input still invalidates the estimate afterwards.
