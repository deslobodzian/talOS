# Simulation Gateway

Standalone hardware gateway for host-side simulation. It speaks the same UDP
frames as the RoboRIO program, so the companion bridge and subsystem nodes
run unmodified against either.

```sh
# Ideal actuators (fast logic tests); defaults to enabled teleop, blue alliance.
bazel run //2026-robot/main_processor/simulation:sim_gateway -- \
  --config "$PWD/2026-robot/main_processor/configuration/robot.toml"

# Physics-backed ground truth with optional odometry error, bounded to ten seconds.
bazel run //2026-robot/main_processor/simulation:sim_gateway -- \
  --config "$PWD/2026-robot/main_processor/configuration/robot.toml" \
  --physics --start-pose 2,2,0 --wheel-radius-error 0.1 \
  --wheel-slip-gain 0.2 --gyro-drift-radps 0.01 --duration-s 10

# Script a disabled / autonomous / teleop cycle on the red alliance.
bazel run //2026-robot/main_processor/simulation:sim_gateway -- \
  --config "$PWD/2026-robot/main_processor/configuration/robot.toml" \
  --mode-schedule disabled:0.2,auto:0.5,teleop:0.5,disabled:0.2 \
  --alliance red --duration-s 1.5
```

The summary line (`active_ticks=... first_wheel_rotations=...`) keeps its
exact format in both modes: `tools/test_drivetrain.py` parses it. The backend
choice is logged to stderr (`backend=ideal|physics`).

`--mode-schedule` accepts comma-separated `mode:seconds` phases with positive,
finite fractional durations. Modes are `disabled`, `auto` (or `autonomous`),
`teleop`, and `test`. After the schedule finishes, the final mode is held until
stopped or `--duration-s` expires (zero or omitted means unbounded).
`--alliance blue|red` defaults to blue. Driver Station packets carry the
scheduled mode and a new `mode_since_us` generation at each mode transition;
the gateway's enable gate follows that same mode. The bridge publishes these
packets on `/hw/state/driver_station`, just as it does for the RoboRIO.
Packets are emitted at most every 20000 µs (50 Hz), matching the RoboRIO,
even when physics and hardware snapshots run at 200 Hz.
`--port N` selects the gateway's loopback UDP listening port (default 5802);
`--remote-port N` selects the bridge's receiving port (default 5803). Tests
choose ephemeral ports so independent simulated stacks do not share sockets.

## Scripted joystick and fault schedules

Both schedules use seconds since the gateway's loop starts and run alongside
`--mode-schedule`. Quote the argument so the shell does not interpret `|` or `;`.

`--joystick-schedule 'time:axes=A0,A1,...;buttons=MASK;pov=DEG|...'`
sets complete, connected joystick-0 snapshots. Times must be nonnegative and
strictly increasing. Each snapshot is held until the next event; before the
first event the joystick is disconnected. All three fields are required.
Axes are one to twelve finite values in `[-1,1]`; omitted axes are zero.
Buttons are a decimal uint32 mask (bit 0 is button 1). POV is `-1` (released)
or an integer in `[0,359]` degrees. Other joysticks remain disconnected.
For the 2026 driver, negative axis 1 is forward; center the stick in teleop
before pushing it, and center again after stale DS input or a driver stall.

`--fault-schedule 'start:end:kind:target[:volts]|...'` requires `--physics`.
Intervals are half-open `[start,end)`, with finite `0 <= start < end`.
Device targets use logical TOML device names, not physical CAN IDs.

| Kind | Target | Effect |
|---|---|---|
| `motor_disconnect` | Motor name | Invalid feedback until the interval ends |
| `sensor_disconnect` | Sensor name | Invalid feedback until the interval ends |
| `freeze` | Motor name | Hold feedback until the interval ends |
| `supply_voltage` | `all` | Require fifth field, volts in `[0,16]`; restore 12 V at end |
| `reset` | Motor or sensor name | One device-reset edge on the first tick at/after start |
| `udp_blackout` | `bridge` | Drop **all outbound UDP**, including DS, hardware state and config ACKs |

Unknown devices and overlapping intervals of the same kind/target are rejected.
Blackouts do not stop incoming commands, physics or gateway safety ticks;
suppressed packets are discarded, never replayed after recovery.
The bridge's `/hw/status.motor_fault_mask` uses configured motor order and
marks invalid feedback as well as nonzero controller fault flags.

```sh
bazel run //2026-robot/main_processor/simulation:sim_gateway -- \
  --config "$PWD/2026-robot/main_processor/configuration/robot_physics_sim.toml" \
  --physics --mode-schedule disabled:0.5,teleop:5 --duration-s 6 \
  --joystick-schedule '0:axes=0,0,0;buttons=0;pov=-1|1:axes=0,-0.6,0;buttons=0;pov=-1|3:axes=0,0,0;buttons=0;pov=-1|3.3:axes=0,-0.6,0;buttons=0;pov=-1' \
  --fault-schedule '2:2.5:udp_blackout:bridge|4:5:motor_disconnect:back_left_drive'
```

`./sim.sh --sim-gateway` selects the physics backend. Its repeatable
`--gateway-arg ARG` passes one argument verbatim through the launcher's
repeatable `--sim-gateway-arg ARG` interface, preserving order:

```sh
./sim.sh --sim-gateway --gateway-arg --start-pose --gateway-arg 3,2,0.5 \
  --gateway-arg --wheel-radius-error --gateway-arg 0.1
```

`--gateway-arg` requires `--sim-gateway`; the launcher forwarding flag requires
`--start-sim-gateway`. The launcher forwards these arguments to both the
gateway's `--describe` probe and runtime, so truth is declared exactly when
`--physics` is selected. The running gateway publishes its live registry row
with the session ID; heartbeat/stall supervision remains disabled as before.

## True world and injected odometry error

Only `--physics` publishes `/sim/state` (`geometry.OdometrySample`), every
physics tick (200 Hz in the robot config). Its timestamp uses the
main-processor node clock; pose uses the blue-alliance field frame (x
downfield, y left, yaw radians CCW+). Velocity is **field-frame** m/s and
rad/s. Cameras render from this physical pose, not drivetrain odometry.
The estimator retains its own independent initial pose; a seed mismatch is
corrected by vision like on a real robot.

`./sim.sh --sim-gateway`, the accuracy scenario, and full-stack startup tests
select `configuration/robot_physics_sim.toml`. It includes `robot.toml` and
only sets `[subsystems.drivetrain] integrate_heading = false`, so odometry
uses the physics gateway's modeled Pigeon, including gyro drift. WPILib and
ideal simulation keep `robot.toml`'s module-integrated simulated heading,
because those backends report a permanently zero Pigeon yaw.

Physics-only flags (all values must be finite; supplying any without
`--physics` is a usage error):

| Flag | Default | Meaning |
|---|---|---|
| `--start-pose X_M,Y_M,YAW_RAD` | `2,2,0` | Physical field start pose, independent of estimator parameters |
| `--wheel-radius-error FRAC` | `0` | True radius `0.0508 × (1 + FRAC)` m; strictly `-0.5 < FRAC < 0.5` |
| `--wheel-slip-gain G` | `0` | Nonnegative acceleration-dependent traction loss |
| `--gyro-drift-radps R` | `0` | Reported yaw bias grows by `R × elapsed_seconds`, yaw rate gains `R` |

Encoders truthfully measure wheel rotations; radius error changes ground
travel only. Each wheel's slip fraction is
`min(1, G × abs(wheel_surface_acceleration_mps2) / 9.80665)` and ground speed is
surface speed times `1 − slip`. It is deterministic and vanishes at constant
speed. This simplified model only reduces ground travel while accelerating
**or braking**, so nominal-radius odometry over-reports travel during either.
Gyro drift never rotates physical truth. All-zero errors retain nominal
behavior.

The ideal backend and WPILib/RoboRIO simulation publish **no** `/sim/state`.
Their simulated camera nodes emit invalid observations rather than inventing
truth from odometry.

## What is ideal vs physics-backed

See `talOS/hardware/README.md` ("Simulation Backends") for the full table.
Short version: the ideal backend teleports to demands (even with zero gains),
while the physics backend drives a reusable motor/mechanism model from
controller voltage output and writes rotor position/velocity back. Closed-loop
demands with missing gains honestly stall; neutral coasts/brakes per the
motor's brake flag instead of snapping to zero.

MotionMagic uses trapezoidal position/velocity setpoints when both cruise
velocity and acceleration are positive, including velocity/acceleration
feedforward and velocity-error damping. Without those limits it uses the
ordinary position loop.

## Deterministic stepping

Physics has no wall-clock or randomness: `Read()` derives `dt` from
`sample_time_us` deltas, so Gateway ticks with explicit timestamps reproduce
exactly. `PhysicsSimBackend::Step(dt)` advances the models without a Gateway
for unit tests. Scheduling (`period_us` pacing) and UDP timing are endpoint
concerns, intentionally outside the model and tested separately.

## Controlled inputs and faults

Beam breaks and limit-switch inputs via `SetDigitalInput`; soft limits come
from `MotorConfig` and are enforced in physics. `FreezeFeedback`,
`SetMotorConnected`/`SetSensorConnected` (a disconnect invalidates only that
sample; `Read` fails only when all configured motors and sensors disconnect),
`SetSupplyVoltage`, and `RestartController` cover the rest. Controller restart
zeroes rotor motion, commands, fault flags and the reported gyro (including
drift elapsed time), but preserves the physical field pose and configuration.
Pigeon yaw is relative to the heading at boot/restart, so a nonzero start yaw
still boots with reported yaw zero, before mounting correction. Truth solves
the planar rigid-body motion of every named `*_drive`/`*_steer` pair;
`front`/`back` (`rear`) and `left`/`right` locate modules at ±0.30 m.
Tests with other names or dimensions register pairs with
`SetSwerveModuleGeometry`. `SetGyro` and `SetChassisYawRate` explicitly
override only gyro reporting until reconfiguration or controller restart;
they do not teleport the robot or override its ground motion.

## Full-stack regression test

```sh
bazel test --test_output=errors //2026-robot/main_processor/simulation:all
```

`startup_test` runs a bounded eight-second scripted match using the real
launcher, robot processes and physics gateway, with ephemeral UDP ports and
unique shared-memory names. It checks disabled/autonomous/teleop operation,
clean process exits, no restarts/recorder failures, and nonempty node logs.
It then decodes the complete JSONL `payload_hex` bytes from `log_dump` (the
human-readable `decoded` packet preview is intentionally truncated).
Additional JSONL fields, including publication timestamps, are tolerated.

The match proves nonzero drivetrain velocity requests before each injection:

- `SIGSTOP driver` for 250 ms: every drivetrain motor must request actual
  neutral mode within `command_timeout + 2 periods` (110 ms in this config).
- `SIGCONT` with forward still held: neutral must persist until the scheduled
  centered stick; a later forward push must drive again.
- A 500 ms outbound UDP blackout: the same 110 ms bound is measured from the
  last received DS sample; recovery while held cannot rearm driving.
- Disconnect `back_left_drive`: that motor must request neutral and its
  configured-order fault bit must appear in `/hw/status`.

Assertions require dense recorded output throughout the safety windows,
not merely one eventual zero demand. Without stale-target neutralization,
the pre-proven forward request remains active past the deadline and fails;
without rearming, resumed held input fails the continued-neutral window.
Zero-demand velocity mode is **not** accepted as neutral. The test uses the
node clock and v6 log record timestamps to align signal injection precisely
on macOS as well as Linux. Process groups are always reaped. On failure it
prints and retains the temporary directory containing `sessions/`, launcher
diagnostics and (when export succeeds) `match.jsonl`; successful runs remove it.

`match_rehearsal_test` adds a 17-second full-graph rehearsal: disabled,
four seconds of the configured autonomous routine, disabled, teleop with
drive and shooter/feed buttons, then disabled. It kills drivetrain with
`SIGKILL` while briefly pausing its driver producer, checks that only
drivetrain restarts, and proves its resumed output follows a fresh,
post-crash active target. A separate 250 ms driver pause must neutralize
all drivetrain motors within `command_timeout_us + period_us`; held input
cannot rearm until centered. The recording checks every motor request
against its consumer's latest disabled DS snapshot, nonzero autonomous
and teleop mechanism actuation, clean manifest closure, and all nodes'
recorder/exit status. Node-log error lines are rejected except explicit
host real-time privilege warnings. Failure artifacts are retained.

Run the independent rehearsal three times for a stability check:

```sh
bazel test --test_output=errors --runs_per_test=3 \
  //2026-robot/main_processor/simulation:match_rehearsal_test
```

## Pose-estimator accuracy scenario

```sh
# Three independent twelve-second scenarios; print each report and mean/min/max.
bazel run //2026-robot/main_processor/simulation:accuracy_scenario

# Retain the original input logs, odometry replay, launcher diagnostics and report.
bazel run //2026-robot/main_processor/simulation:accuracy_scenario -- \
  --duration-s 12 --output-dir /tmp/talos-accuracy

# Alternatively retain logs in a newly allocated temporary directory.
bazel run //2026-robot/main_processor/simulation:accuracy_scenario -- --keep

# One scenario when iterating (CI also runs the shared runner only once).
bazel run //2026-robot/main_processor/simulation:accuracy_scenario -- --repeat 1

# Override only the live cameras' yaw floor for a repeatable calibration sweep.
bazel run //2026-robot/main_processor/simulation:accuracy_scenario -- \
  --min-yaw-std-rad 0.035 --keep

bazel test --test_output=errors \
  //2026-robot/main_processor/simulation:accuracy_test
```

The tool runs the existing 2026 autonomous routine, from physical and estimator
start `(2, 2, 0)` toward the in-field blue waypoint `(6, 4, 0)`; Red still
rotates the waypoint about the field centre. The moderate preset is
`--wheel-radius-error -0.02 --wheel-slip-gain 0.05 --gyro-drift-radps 0.002`.
The launcher owns the physics gateway and forwards the preset plus
`--mode-schedule disabled:1,auto:N`. Duration and `--repeat` must be positive
integers; the report tool defaults to three independent scenarios because
single-session accuracy varies. Each retains its five-row and timing tables,
followed by mean/min/max position and heading RMS for each row. `--output-dir`
also retains `summary.md`. CI stays one short scenario, not the CLI repeat loop.
Loopback ports and shared-memory namespaces are unique per physical session.
Negative radius error and acceleration-dependent slip both reduce ground
travel relative to encoder-derived odometry, rather than partly cancelling.

The derived live config also injects true-minus-believed per-camera mount
errors: yaw ±1 degree, pitch ±0.5 degrees and translation ±0.01 m per axis.
The sole preset definition is `MOUNT_ERROR_PRESET` in `accuracy.py`; the sign
pattern is:

| Camera | x | y | z | pitch | yaw |
|---|---:|---:|---:|---:|---:|
| front_left_camera | + | − | + | + | + |
| front_right_camera | − | + | + | − | + |
| rear_left_camera | + | + | − | + | − |
| rear_right_camera | − | − | − | − | − |

Only the renderer/projector sees the true mounting transform; localization
keeps the configured mount. These deterministic camera-specific errors model
calibration bias separately from pixel noise.

The report has five rows:

- **odometry-only** and **fused** both re-run the baseline estimator with
  `launcher --replay-inputs SESSION --only pose_estimator`, on the same
  virtual-clock timer schedule and recorded drivetrain inputs. Only
  **odometry-only** disables camera subscriptions.
- **fused-live** is the original live session, including the mount-error preset.
- **fused-lag-uncorrected** uses the same moderate/mount-error presets plus
  `camera.sim_timestamp_lag_us = 40000`, with `latency_offset_us = 0`.
- **fused-lag-corrected** uses the same lag and
  `camera.latency_offset_us = 40000`, restoring exposure timestamps.

The 40 ms lag corresponds to 8 cm at the routine's configured 2 m/s maximum,
four times the 2 cm x/y uncertainty floor. The lag cases each run a separate
physical session. Both delay delivery until exposure time + 40 ms, so the
uncorrected case measures time-alignment error rather than future-stamp
rejection. The normal physics config injects no lag. The sole lag constant is
`TIMESTAMP_LAG_US` in `accuracy.py`; the overlay tool owns all three live configs.
The existing per-camera consistency table is scored from the baseline fused run.
The lag-session RMS comparison is reported, not gated: independent real-time
sessions have scheduling variance larger than the correction gain. CI requires
accepted observations in both lag runs; it gates the paired virtual-clock
fused-vs-odometry improvement and absolute 0.15 m / 3-degree RMS limits for both
the fused replay and its live source.
Timestamp correction arithmetic is covered by
`VisionTest.CaptureTimestampCorrectionIsSharedWithLiveFrames`.

Live and replay configs flatten `robot_physics_sim.toml` onto `robot.toml`,
preserving all non-include override tables with `tomllib`. Live overlays add
mount errors and timing settings; the fused replay reuses that live config,
while odometry replay sets `[subsystems.pose_estimator] cameras = []` with no
camera-error overlay. Mount/timing errors cannot affect odometry-only replay.
Physics overrides have one source and no whole robot/subsystem files are copied.
Replay reuses the session ID solely so the log merger aligns records with
the original camera/drivetrain logs; output logs stay in separate `*-replay`
directories, never overwriting the live estimator log.
All three physical runs use the requested autonomous duration (12 s by default),
and the report includes total wall-clock runtime, including startup/replay/scoring.
It also prints final `frame_period_us`, dropped, captured and processed counters
for each camera in each physical session. These are actual camera status
records (including disabled startup), not rates inferred from estimator accepts.
Sim capture deadlines preserve the configured mean frame period despite polling
quantization; missed deadlines and real queue overflow remain counted drops.

### Scoring

All logs are read through `//talOS/launcher:log_dump --format jsonl`; the
scenario does not parse `.tlog` files itself. Ground truth is `/sim/state`
recorded as an input in `front_left_camera.tlog`. Estimates are matched to
truth by integer **`sample_time_ns`**, not receipt time: x/y are linearly
interpolated between bracketing truth samples and yaw is interpolated along
the shortest wrapped arc. Exact timestamps use the exact truth sample;
unbracketed samples are excluded, never extrapolated.

Only valid estimates published while the latest `/drivetrain/state` SEND
record in `drivetrain.tlog` has `enabled = true` and is younger than 100 ms
are scored. This **drivetrain-enabled window is autonomous** because the
scenario's only phases are disabled and auto; it never enables teleop.
The enabled flag comes from the schema-decoded state, without decoding wire
packets in Python. Repeated estimator publications with the same odometry
timestamp count once, taking the last publication. Each scored sample has
equal weight. Position error is Euclidean field-frame distance; heading error
is the absolute angle wrapped to `[-pi, pi]`, converted to degrees. RMS means
`sqrt(mean(error**2))`, max is the largest sample error, and final is the
last scored sample's error. Accepted/rejected counts are the estimator's
cumulative observation counters at that final sample (including disabled
startup observations).

The report command exits nonzero only for tool/session/scoring failures,
not poor accuracy. The exclusive CI test uses the **same runner and scorer**
with eight autonomous seconds for the baseline and four seconds for each lag
session. Fused replay must improve on odometry-only RMS with equal scored
sample counts. Both replayed and live fusion must meet 0.15 m position and
3-degree heading RMS limits and accept observations. Comparing live fusion
against offline odometry alone is not a same-input experiment: live polling
can skip drivetrain samples and process cameras at different odometry-buffer
positions when the host is delayed. Both paired replays instead consume the
same published inputs on the same simulated schedule, without changing the
product's timestamp or latency compensation. The lag sessions must both accept
observations; their RMS difference and ratio remain a report, not a CI gate.
Scoring unit tests cover wrapped interpolation, timestamp deduplication,
phase/validity filtering and all timing overlays.

### Measured baseline

Measured on 2026-11-06 (macOS arm64, kalman backend, four rasterized cameras),
with the mount-error preset above, measured `pixel_std_px = 0.33`, single-tag
ambiguity limit 0.2 and the estimator's 15-degree single-tag yaw gate.

Two twelve-second runs per floor, with no startup failures in this sweep:

| Yaw floor (rad) | Run | Fused position RMS (m) | Fused heading RMS (deg) | Heading max (deg) | Accepted | Rejected |
|---|---:|---:|---:|---:|---:|---:|
| 0.018 | 1 | 0.0510 | 0.678 | 2.453 | 621 | 592 |
| 0.018 | 2 | 0.0371 | 1.171 | 2.272 | 604 | 578 |
| 0.035 | 1 | 0.0514 | 0.638 | 2.093 | 613 | 578 |
| 0.035 | 2 | 0.0306 | 0.746 | 2.109 | 607 | 590 |
| 0.070 | 1 | 0.0760 | 0.571 | 1.914 | 421 | 802 |
| 0.070 | 2 | 0.0531 | 1.177 | 2.443 | 425 | 783 |
| 0.100 | 1 | 0.0490 | 0.591 | 1.984 | 432 | 753 |
| 0.100 | 2 | 0.0412 | 1.152 | 2.290 | 598 | 575 |

**Selected 0.035 rad**: the lowest measured two-run mean fused heading RMS
(0.692 degrees), with the best mean position RMS (0.0410 m). Correlated mount
bias does not average away like independent frame noise; selection uses fused
outcomes, not a requirement that per-observation normalized RMS equal one.
The x/y floor remains 0.02 m; floors were not inflated around rare outliers.

Selected-floor run 1:

| Run | Samples | Position RMS (m) | Position max (m) | Heading RMS (deg) | Heading max (deg) | Final position (m) | Final heading (deg) | Accepted | Rejected |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| odometry-only | 601 | 0.1018 | 0.1151 | 0.889 | 1.481 | 0.1151 | 1.481 | 0 | 0 |
| fused | 601 | 0.0514 | 0.1198 | 0.638 | 2.093 | 0.0723 | 0.440 | 613 | 578 |

Selected-floor run 2:

| Run | Samples | Position RMS (m) | Position max (m) | Heading RMS (deg) | Heading max (deg) | Final position (m) | Final heading (deg) | Accepted | Rejected |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| odometry-only | 601 | 0.1016 | 0.1149 | 0.889 | 1.481 | 0.1149 | 1.481 | 0 | 0 |
| fused | 601 | 0.0306 | 0.1197 | 0.746 | 2.109 | 0.0547 | 0.412 | 607 | 590 |

Per-camera consistency before/after the floor change (RMS and interpolated
95th percentile of absolute normalized errors; yaw errors are wrapped):

| Floor (rad) | Run | Camera | Samples | x RMS / p95 | y RMS / p95 | yaw RMS / p95 | Raw yaw RMS (deg) |
|---|---:|---|---:|---:|---:|---:|---:|
| 0.100 | 1 | front_left | 302 | 0.207 / 0.461 | 0.278 / 0.334 | 0.092 / 0.149 | 0.525 |
| 0.100 | 1 | front_right | 22 | 0.269 / 0.485 | 0.250 / 0.464 | 0.227 / 0.375 | 1.302 |
| 0.100 | 1 | rear_left | 59 | 0.931 / 1.209 | 1.381 / 0.714 | 0.565 / 0.209 | 3.238 |
| 0.100 | 1 | rear_right | 80 | 0.751 / 1.701 | 0.771 / 1.548 | 0.202 / 0.361 | 1.155 |
| 0.100 | 2 | front_left | 303 | 0.325 / 0.340 | 0.121 / 0.214 | 0.202 / 0.217 | 1.156 |
| 0.100 | 2 | front_right | 259 | 0.071 / 0.104 | 0.246 / 0.254 | 0.226 / 0.229 | 1.297 |
| 0.100 | 2 | rear_left | 59 | 0.845 / 1.199 | 0.385 / 0.487 | 0.176 / 0.200 | 1.010 |
| 0.100 | 2 | rear_right | 79 | 0.798 / 1.827 | 0.842 / 1.914 | 0.195 / 0.302 | 1.119 |
| 0.035 | 1 | front_left | 294 | 0.181 / 0.305 | 0.277 / 0.305 | 0.256 / 0.426 | 0.514 |
| 0.035 | 1 | front_right | 248 | 0.406 / 0.423 | 0.242 / 0.251 | 0.253 / 0.231 | 0.518 |
| 0.035 | 1 | rear_left | 59 | 0.899 / 1.200 | 2.487 / 0.802 | 2.327 / 0.604 | 4.965 |
| 0.035 | 1 | rear_right | 77 | 0.757 / 1.649 | 0.786 / 1.705 | 0.554 / 0.977 | 1.111 |
| 0.035 | 2 | front_left | 306 | 0.159 / 0.333 | 0.290 / 0.313 | 0.258 / 0.425 | 0.519 |
| 0.035 | 2 | front_right | 253 | 0.198 / 0.210 | 0.058 / 0.144 | 0.399 / 0.404 | 0.824 |
| 0.035 | 2 | rear_left | 57 | 0.891 / 1.199 | 1.807 / 0.605 | 1.723 / 0.583 | 3.670 |
| 0.035 | 2 | rear_right | 78 | 0.725 / 1.739 | 0.755 / 1.497 | 0.579 / 0.997 | 1.161 |

These camera tables include all valid camera outputs, including those the
estimator rejects. Rear-left RMS versus p95 shows remaining rare heavy tails;
the yaw innovation gate prevents those from being fused. A local Gaussian
covariance is not evidence that every planar hypothesis is correct.

The two pinned tag-32 regressions demonstrate why both gates are needed:
the old floor heuristic switched candidates without decisive evidence at
3.94 m (ambiguity 0.414), while the flipped primary itself had the better
sub-noise fit at 3.73 m (ambiguity 0.277). Both now fail the 0.2 camera gate;
the separate estimator yaw gate protects against other gross single-tag jumps.

`PoseObservation` does not record the invalidity reason or per-tag ambiguity,
and `VisionStatus` only distinguishes range rejections. Consequently the
earlier approximately 750 estimator rejections cannot be split exactly into
new unknown-ambiguity single-tag rejections versus other causes from these
logs. The selected runs report 578/590 total rejections; no finer attribution
is claimed.

Worst selected-floor twelve-second RMS has 2.9x position and 4.0x heading
margin against the unchanged absolute CI bounds (0.15 m / 3 degrees).
The actual eight-second CI run passed with fused RMS 0.0580 m / 0.656 degrees:
2.59x / 4.57x margins against those absolute bounds. Its same-input odometry
was 0.0944 m / 0.625 degrees RMS, so position improved while heading was
slightly worse in that CI run.

The historical slice-3 baseline, without injected mount bias, was
0.0358–0.0499 m / 0.075–0.105 degrees RMS; the harder mount-error preset is not
an equivalent comparison. These measured results still exceed that historical
heading range. Physics is deterministic, but process/UDP/camera scheduling is
not; two-run sweep means are measurements, not golden performance guarantees.

A prior, pre-policy sweep session had rear-right startup watchdog failure:
the launcher reported `heartbeat missing`, sent SIGKILL, and restarted it.
The first log was empty; the restart exited zero. No exception or frame/solver
crash was observed. Logs cannot establish the blocking startup operation or
prove machine load; all eight sessions in the reported final sweep were clean.


### Historical capture-latency correction measurements

Before the region-limited blur and capture-deadline changes, two default
twelve-second scenarios on 2026-11-06, with the unchanged presets above and
a 40 ms timestamp lag in the two lag rows, measured:

Run 1 (42.2 s total wall time):

| Run | Samples | Position RMS (m) | Position max (m) | Heading RMS (deg) | Heading max (deg) | Final position (m) | Final heading (deg) | Accepted | Rejected |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| odometry-only | 600 | 0.1014 | 0.1148 | 0.888 | 1.481 | 0.1148 | 1.481 | 0 | 0 |
| fused | 600 | 0.1128 | 0.1745 | 0.560 | 2.161 | 0.1366 | 0.225 | 364 | 847 |
| fused-lag-uncorrected | 601 | 0.0476 | 0.1222 | 0.895 | 1.767 | 0.0436 | 0.591 | 492 | 487 |
| fused-lag-corrected | 600 | 0.0426 | 0.1715 | 1.111 | 1.719 | 0.0436 | 1.217 | 326 | 628 |

Run 2 (42.2 s total wall time):

| Run | Samples | Position RMS (m) | Position max (m) | Heading RMS (deg) | Heading max (deg) | Final position (m) | Final heading (deg) | Accepted | Rejected |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| odometry-only | 601 | 0.1015 | 0.1148 | 0.889 | 1.481 | 0.1148 | 1.481 | 0 | 0 |
| fused | 600 | 0.0436 | 0.1482 | 0.663 | 2.066 | 0.0502 | 0.594 | 619 | 607 |
| fused-lag-uncorrected | 600 | 0.0767 | 0.1297 | 0.558 | 2.077 | 0.0916 | 0.120 | 178 | 762 |
| fused-lag-corrected | 600 | 0.0367 | 0.1737 | 1.038 | 1.891 | 0.0345 | 1.075 | 331 | 642 |

Correction reduced lag-session position RMS by 0.0050 m (1.12×) and 0.0400 m
(2.09×). These are separate physical sessions, not identical-input replays;
process/UDP scheduling varies. The first run's normal fused row was worse than
its odometry baseline; these measurements do not establish improvement in every
session, nor an improvement in lag-session heading RMS.

The unchanged-bound CI gate passed in 22.1 s: baseline fused position/heading
RMS 0.0443 m / 1.163 degrees versus odometry position RMS 0.0939 m; margins to
the 0.15 m / 3-degree limits were 3.39× / 2.58×. Its four-second lag sessions
gave 0.0677 m uncorrected versus 0.0353 m corrected position RMS, an improvement
of 0.0325 m (1.92×, using unrounded scores). Both accepted observations
(170 uncorrected, 167 corrected). No previous accuracy bounds were loosened.

### Region-limited exposure and capture-deadline measurements

The renderer now initializes the static background once and integrates only
disjoint tag regions, including projected quiet-zone and rasterization margins.
Intermediate projections extend endpoint bounds when needed. The old full-frame
accumulation survives only as a test oracle (maximum allowed discrepancy: one
gray level). Shared cell vertices are projected once per 11×11 lattice.
Capture deadlines stay phase-anchored at 60 fps; lag delivery queues are sized
`ceil(lag * fps) + 2` and publish every due frame, rather than discarding all
but the latest. Slow-tick missed deadlines remain genuine counted drops.

Temporary front-left timers on three-second autonomous profiles isolated the
remaining detector bottleneck; all timers and experimental overrides were removed:

| Detector setting | Execute mean (ms) | median (ms) | p95 (ms) | max (ms) | Camera frame period (µs) |
|---|---:|---:|---:|---:|---:|
| decimate 1 / threads 1 / auto→Metal | 25.628 | 12.841 | 81.148 | 112.696 | 33601 |
| decimate 1 / threads 1 / CPU overlay | 26.323 | 10.893 | 82.975 | 124.553 | 33877 |
| decimate 1 / threads 1 / auto, `TALOS_REALTIME=0` | 25.392 | 13.209 | 80.731 | 90.967 | 33687 |
| **decimate 2 / threads 2 / auto→Metal** | **6.186** | **3.547** | **14.142** | **158.687** | **17601** |

With the old detector settings, render mean/p95 was 2.632/2.850 ms, per-tag
pose estimation 2.091/2.718 ms and joint localization 0.006/0.014 ms. CPU and
realtime-disabled experiments did not remove the Execute tail. For a 10 ms
node period, Mach requests a 5 ms computation budget and 10 ms constraint;
the experiment did not establish that policy as the cause. The selected
half-resolution quad search and two detector workers retain full-resolution
edge refinement, margin 30, Hamming limit 1 and backend auto. Its measured
Execute p95 is below the 16.667 ms frame period, although startup/tail misses
are still visible and counted, not hidden.

Three clean fastbuild twelve-second scenarios with these final settings:

Run 1 (43.2 s):

| Run | Samples | Position RMS (m) | Position max (m) | Heading RMS (deg) | Heading max (deg) | Final position (m) | Final heading (deg) | Accepted | Rejected |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| odometry-only | 599 | 0.1012 | 0.1144 | 0.886 | 1.478 | 0.1144 | 1.478 | 0 | 0 |
| fused | 600 | 0.0716 | 0.1633 | 0.666 | 2.291 | 0.0810 | 0.440 | 718 | 1594 |
| fused-lag-uncorrected | 599 | 0.0736 | 0.1899 | 1.365 | 2.465 | 0.0729 | 1.491 | 734 | 1596 |
| fused-lag-corrected | 600 | 0.0265 | 0.1451 | 0.997 | 2.339 | 0.0109 | 0.945 | 753 | 1596 |

Run 2 (42.5 s):

| Run | Samples | Position RMS (m) | Position max (m) | Heading RMS (deg) | Heading max (deg) | Final position (m) | Final heading (deg) | Accepted | Rejected |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| odometry-only | 601 | 0.1016 | 0.1150 | 0.889 | 1.481 | 0.1150 | 1.481 | 0 | 0 |
| fused | 600 | 0.0293 | 0.1567 | 1.041 | 2.161 | 0.0189 | 1.045 | 1109 | 1240 |
| fused-lag-uncorrected | 600 | 0.0756 | 0.2271 | 0.902 | 2.547 | 0.0651 | 1.183 | 528 | 1913 |
| fused-lag-corrected | 600 | 0.0381 | 0.1607 | 0.741 | 2.345 | 0.0335 | 0.586 | 730 | 1581 |

Run 3 (42.4 s):

| Run | Samples | Position RMS (m) | Position max (m) | Heading RMS (deg) | Heading max (deg) | Final position (m) | Final heading (deg) | Accepted | Rejected |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| odometry-only | 600 | 0.1014 | 0.1147 | 0.888 | 1.481 | 0.1147 | 1.481 | 0 | 0 |
| fused | 600 | 0.0424 | 0.1927 | 1.030 | 2.582 | 0.0389 | 1.471 | 346 | 1976 |
| fused-lag-uncorrected | 600 | 0.0654 | 0.2499 | 1.367 | 2.008 | 0.0614 | 1.480 | 523 | 1810 |
| fused-lag-corrected | 601 | 0.0411 | 0.1641 | 1.543 | 2.554 | 0.0383 | 2.168 | 372 | 2047 |

Final camera status counters, as **frame_period_us / frames_dropped_count**:

| Run | Camera | fused | lag uncorrected | lag corrected |
|---|---|---:|---:|---:|
| 1 | front_left | 16956 / 13 | 16929 / 12 | 16938 / 12 |
| 1 | front_right | 16956 / 13 | 16951 / 13 | 16938 / 12 |
| 1 | rear_left | 16868 / 9 | 16885 / 10 | 16893 / 10 |
| 1 | rear_right | 16868 / 9 | 16885 / 10 | 16850 / 8 |
| 2 | front_left | 16932 / 12 | 16933 / 12 | 16977 / 14 |
| 2 | front_right | 16938 / 12 | 16956 / 13 | 16977 / 14 |
| 2 | rear_left | 16894 / 10 | 16867 / 9 | 16890 / 10 |
| 2 | rear_right | 16850 / 8 | 16867 / 9 | 16888 / 10 |
| 3 | front_left | 16932 / 12 | 16955 / 13 | 16932 / 12 |
| 3 | front_right | 16976 / 14 | 16977 / 14 | 16954 / 13 |
| 3 | rear_left | 16890 / 10 | 16889 / 10 | 16888 / 10 |
| 3 | rear_right | 16890 / 10 | 16889 / 10 | 16888 / 10 |

Cadence is now 16.850–16.977 ms, with 8–14 missed deadlines per camera over
13 seconds, rather than approximately 33 ms/frame. The zero-loss steady-stream
unit regression isolates delivery-queue behavior from real runtime overruns.
All three corrected lag sessions improved position RMS, by 2.78× / 1.98× /
1.59×; heading did not improve in every run.

Plain fused position RMS spans 0.0561–0.0953 m (**1.70×**); consequently the
report tool now defaults to `--repeat 3`, with an explicit `--repeat 1` escape
for iteration and unchanged single-scenario CI. These three sessions average
0.0754 m fused position RMS and 0.652 degrees heading RMS. Each scenario took
42.5–42.8 s; three default repeats therefore take about 128 s, excluding build.

The three-run table above predates the multi-tag seed change and the mount
covariance change; the span and average quoted here are the post-fix
re-measurement. The seed change (score every decode-valid tag's primary and
secondary candidate by the joint reprojection residual over all tags' corners
before LM) makes the rear cameras localize the grazing end-pose tag pair
instead of rejecting it, raising fused position RMS (0.0478→0.0804 m) because
those grazing observations carry amplified mount bias. Adding
`camera.mount_angle_std_rad` / `camera.mount_translation_std_m` to the published
covariance lets the estimator down-weight that bias, lowering fused position RMS
to 0.0754 m while heading RMS stays at 0.652 deg. The isotropic lever-arm term
is a worst-case bound, so per-camera normalized x/y RMS falls below one
(~0.4–0.5 for the rear cameras) instead of landing at one; it is reported, not
retuned. All rows remain well inside the unchanged CI bounds (0.15 m / 3 deg).
The repeat CLI was also exercised with two one-second scenarios, printing both
individual reports and the four-row mean/min/max summary.

The updated production-effects corner-noise invariant (decimate 2, two workers,
full-resolution refinement) measured **0.324317 px** standard deviation and
0.008764 px signed mean over 1280 coordinate errors. This remains within the
unchanged ±30% tolerance of `pixel_std_px = 0.33`; no covariance setting changed.
The moving-pose region/full-frame equivalence and all eleven narrow test targets
passed. The final CI scenario took 22.0 s: fused RMS 0.0681 m / 1.354 degrees
versus odometry position RMS 0.0940 m; the unchanged absolute bounds retain
2.20× position / 2.22× heading margins. Its lag sessions measured 0.0662 m
uncorrected and 0.0473 m corrected RMS, a 0.0189 m improvement (1.40×).
