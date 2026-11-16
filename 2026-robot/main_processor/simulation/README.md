# 2026 simulation tests

The 2026 robot's end-to-end tests against the simulator,
`//talOS/simulation:sim_gateway`, which stands in for the RoboRIO for any
robot config: its flags, schedules, physics and `/sim/state` are in
[its README](../../../talOS/simulation/README.md).

For the 2026 driver, negative axis 1 is forward in a
[joystick schedule](../../../talOS/simulation/README.md#scripted-joystick-and-fault-schedules);
center the stick in teleop before pushing it, and center again after stale
DS input or a driver stall.

## Simulation configs

`./sim.sh --sim-gateway`, the accuracy scenario, and full-stack startup tests
select `configuration/robot_physics_sim.toml`. It includes `robot.toml` and
sets only `[subsystems.drivetrain] integrate_heading = false` (and restates
the base's `[robot] require_target_coverage = true`, and sets
`[robot] simulation_only = true`, so the launcher and `hardware_node` refuse
it without `--sim`), so odometry
uses the physics gateway's modeled Pigeon, including gyro drift. WPILib and
ideal simulation keep `robot.toml`'s module-integrated simulated heading,
because those backends report a permanently zero Pigeon yaw.

Under WPILib, which publishes no `/sim/state`, the pose estimator therefore
localizes only from its trusted
seed ([seed rules](../pose_estimator/README.md)), which needs a disabled
Driver Station view naming an alliance. The headless RoboRIO stand-in
(`controller_processor/rio/src/main/include/SimDriverStation.h`) always names
one: blue station 1, or red with `TALOS_SIM_DS_ALLIANCE=red`
(`./sim.sh --headless --alliance red`). Start the mode schedule with a
disabled phase (`--mode-schedule disabled:3,auto:15`) so the seed is trusted
before autonomous drives.

## Full-stack regression test

```sh
bazel test --test_output=errors //2026-robot/main_processor/simulation:all
```

`startup_test` runs a bounded eight-second scripted match using the real
launcher, robot processes and physics gateway, with ephemeral UDP ports
(given to the launcher, which passes them to the gateway) and unique
shared-memory names. It checks disabled/autonomous/teleop operation,
clean process exits and no restarts/recorder failures.
It then reads hardware requests, States and Driver Station packets from the
JSONL `hardware` objects `log_dump` decodes with the wire codecs
([log_dump export](../../../talOS/launcher/README.md#filtering-and-exporting-logs));
`match_rehearsal_test` uses the same helpers.

The match proves nonzero drivetrain velocity requests before each injection:

- A 500 ms outbound UDP blackout: neutralization must occur within
  `command_timeout + 2 periods` (110 ms in this config) of the last received
  DS sample; recovery while held cannot rearm driving.
- Disconnect `back_left_drive`: that motor must request neutral and its
  configured-order fault bit must appear in `/hw/status`.

Assertions require dense recorded output throughout the safety windows,
not merely one eventual zero demand. Without stale-target neutralization,
the pre-proven forward request remains active past the deadline and fails;
without rearming, resumed held input fails the continued-neutral window.
Zero-demand velocity mode is **not** accepted as neutral. The test uses the
node clock and v7 log record timestamps to align signal injection precisely
on macOS as well as Linux. Process groups are always reaped. On failure it
prints and retains the temporary directory containing `sessions/`, launcher
diagnostics and (when export succeeds) `match.jsonl`; successful runs remove it.

`match_rehearsal_test` adds a 17-second full-graph rehearsal: disabled,
four seconds of the configured autonomous routine, disabled, teleop with
drive and shooter/feed buttons, then disabled. It kills drivetrain with
`SIGKILL` while briefly pausing its driver producer, checks that only
drivetrain restarts, and proves each resumed actuating request follows a
post-crash active target younger than `command_timeout_us`. In a separate
250 ms driver pause, every drivetrain request from `command_timeout_us` after
the last active target the drivetrain received (not after the harness's
`SIGSTOP`) must be neutral, the first of them due on the drivetrain's first
tick after that expiry; held input cannot rearm until centered. The
recording checks every motor request against its consumer's latest disabled
DS snapshot, nonzero autonomous and teleop mechanism actuation, clean
manifest closure, and all nodes' recorder/exit status. Each actuator owner
must send one request per period from the first Driver Station receipt to
the bridge's last command, less 1 s (the recording a `SIGKILL` discards)
and, for drivetrain, its restart gap.
Once every node's loop runs (its log exists) and the gateway is up, the
harness writes a node-clock start 1 s ahead to the gateway's
`--schedule-start-file`, times its signals from it and stops the session 17 s
after it. It fails naming the delay if the first Driver Station receipt misses
the opening disabled second, if autonomous does not begin within 1 s after
its scheduled time, or if the second pause does not end at least 100 ms
before the stick centers, and fails with `host slept N ms; rerun` if the host
slept during the match (on macOS node time counts sleep and the harness's
does not). Failure artifacts are retained.

Run the independent rehearsal three times for a stability check:

```sh
bazel test --test_output=errors --runs_per_test=3 \
  //2026-robot/main_processor/simulation:match_rehearsal_test
```

The soak, which uses the bench robot as its subject, is
[`//bench/main_processor/simulation:soak_test`](../../../bench/main_processor/simulation/README.md).

## Realtime scheduling in the full-graph tests on macOS

`--sim` does not request realtime scheduling, so on macOS `startup_test` and
`match_rehearsal_test` set `TALOS_REALTIME=1`, the production
policy (see the node development guide's realtime scheduling paragraph). The
graph alone uses about 2.5 cores (p90 about 4.6), more than a 3-vCPU macOS
runner has free; at timeshare priority 5 ms loops then waited 10–50 ms and
`/hw/request` gaps passed the tests' 30 ms bound. A macOS run fails if the
launcher log reports `talos: realtime scheduling incomplete`, so a silent
fallback cannot pass. Linux runs keep timeshare: they pass without it, and
`SCHED_FIFO` has no per-thread fail-safe. A failed `match_rehearsal_test`
also prints the restarted drivetrain's kill → loop origin, origin → first
dispatch (`run()`) and first dispatch → first request times, separating a
slow exit and respawn from a slow node startup.

`sim_gateway` honours the same variable and runs in the bridge's tier
([simulator realtime](../../../talOS/simulation/README.md#realtime-scheduling)),
so `startup_test` holds it to a full grant as it does `hardware_node`.

## Pose-estimator accuracy scenario

```sh
# Three independent twelve-second scenarios; print each report and mean/min/max.
bazel run //2026-robot/main_processor/simulation:accuracy_scenario

# Retain the original input logs, every replay, launcher diagnostics and report.
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
single-session accuracy varies. Each retains its run/backend and timing
tables, followed by mean/min/max position and heading RMS for each run and
backend. `--output-dir` also retains `summary.md`. CI stays one short
scenario, not the CLI repeat loop.
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

The report has one row per run and `pose_estimator` backend (`kalman` and
`factor_graph`, `BACKENDS` in `accuracy.py`). Every backend re-runs the
estimator on the same recorded sessions with
`launcher --replay-inputs SESSION --only pose_estimator`, on the same
virtual-clock timer schedule and recorded inputs, so backend rows of one run
compare identical inputs. Nothing in a replay depends on host timing, so the
replays run at once after the last live session, not between them, where
they would compete with its cameras for the CPU:

- **odometry-only** and **fused** replay the baseline session. Only
  **odometry-only** withholds camera observations: it keeps the estimator's
  camera subscriptions, so its graph matches the fused replay's, but remaps
  them to unrecorded topics.
- **fused-live** is the original live baseline session, including the
  mount-error preset. It runs the backend configured in
  `subsystems/pose_estimator.toml` and has only that backend's row.
- **fused-lag-uncorrected** replays a session with the same moderate/mount-error
  presets plus `camera.sim_timestamp_lag_us = 40000`, with `latency_offset_us = 0`.
- **fused-lag-corrected** replays a session with the same lag and
  `camera.latency_offset_us = 40000`, restoring exposure timestamps.

The 40 ms lag corresponds to 8 cm at the routine's configured 2 m/s maximum,
four times the 2 cm x/y uncertainty floor. The lag cases each run a separate
physical session. Both delay delivery until exposure time + 40 ms, so the
uncorrected case measures time-alignment error rather than future-stamp
rejection. The normal physics config injects no lag. The sole lag constant is
`TIMESTAMP_LAG_US` in `accuracy.py`; the overlay tool owns all three live configs.
The existing per-camera consistency table is scored from the baseline live session.
The lag-session RMS comparison is reported, not gated: independent real-time
sessions have scheduling variance larger than the correction gain. For every
backend, CI requires accepted observations in both lag runs; it gates paired
virtual-clock fused-vs-odometry improvement when camera throughput is
sufficient, and always gates absolute 0.15 m / 3-degree RMS limits for replay
and (for the configured backend) live fusion.
Timestamp correction arithmetic is covered by
`VisionTest.CaptureTimestampCorrectionIsSharedWithLiveFrames`.

Live and replay configs flatten `robot_physics_sim.toml` onto `robot.toml`,
preserving all non-include override tables with `tomllib`. Live overlays add
mount errors and timing settings; each fused replay reuses its session's live
overlay plus `[subsystems.pose_estimator] backend`, while odometry replay sets
`backend` and `[subsystems.pose_estimator.remap]` `/<camera>/state` to
`/<camera>/state/none`, with no camera-error overlay. Mount/timing errors cannot affect odometry-only replay.
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
session. For each backend, fused replay must improve on that backend's
odometry-only RMS when all baseline
cameras capture at least two thirds of scheduled deadlines (at least 40 of the
requested 60 fps; a deadline ends either processed or dropped, so the
scheduled count is processed + dropped), report a positive mean frame period no larger than 25 ms,
and process at least 95% of captured frames. These are input-health prerequisites,
chosen to require a majority of the requested stream and near-complete processing,
not fitted to position RMS or CPU architecture. Every run reports each backend's
paired RMS difference; insufficient throughput prints **Fusion improvement NOT EVALUATED**
with per-camera period/capture/drop/processing evidence in the report and test
output. This is not a skipped test: equal paired sample counts, accepted
observations and both replayed/live 0.15 m position and 3-degree heading RMS
limits remain unconditional, and the two backends' fused rows must differ.
A host provisioned to sustain the camera pipeline can require health as well with
`--test_env=TALOS_ACCURACY_REQUIRE_CAMERA_HEALTH=1`; generic CI does not infer
host capacity from the OS or architecture. Comparing live fusion
against offline odometry alone is not a same-input experiment: live polling
can skip drivetrain samples and process cameras at different odometry-buffer
positions when the host is delayed. All replays of a session instead consume the
same published inputs on the same simulated schedule, without changing the
product's timestamp or latency compensation. Each backend's lag replays must both accept
observations; their RMS difference and ratio remain a report, not a CI gate.
The deterministic estimator regression
`EstimatorTest.test_a_delayed_observation_corrects_the_estimate` remains an
unconditional fusion-benefit gate for both backends: an exact delayed camera
observation must remove at least 40% of wheel-slip error, without overshoot.
The real-time scenario additionally tests the rendered, mounting-biased stream;
it cannot guarantee improvement from an undersustained, view-biased subset.

Scoring unit tests cover wrapped interpolation, timestamp deduplication,
phase/validity filtering and all timing overlays.

