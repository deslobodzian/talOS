# Camera localization nodes

One `vision::Vision` implementation is built as four independent nodes:
`front_left_camera`, `front_right_camera`, `rear_left_camera`, and
`rear_right_camera`. Each publishes `geometry.PoseObservation` on its own
`/<name>/state` and `vision.VisionStatus` on `/<name>/status`. The pose estimator
lists all four camera names. Nodes run every 10000 µs (100 Hz) and publish
status each tick, whether enabled or disabled. Live observations publish once
per processed frame, up to the requested 60 fps capture rate; neither capture
nor processing throughput is guaranteed. There are no targets or actuation gates.

Each `configuration/subsystems/<name>.toml` holds just that camera's
calibration and device. Its `[subsystem] defaults = "vision.toml"` layers the
shared tag size, covariance solver settings, node period, detector options,
capture rate, and simulation renderer selection.
The official 2026 welded layout is embedded in the binary;
installed bundles need no vision runfiles. Mounts map OpenCV optical coordinates
to robot NWU using `Rz(yaw) Ry(pitch) Rx(roll)`. Positive pitch points a forward
lens downward, preserving the previous calibration convention. The checked-in
mounts are season-plausible defaults, not measured calibration.

Shared detector and scheduling defaults (overridable per camera):

| Key | Default | Units and meaning |
| --- | --- | --- |
| `subsystem.period_us` | `10000` | Node/status period in microseconds (100 Hz), not the capture period. |
| `camera.fps` | `60` | Requested live capture rate and maximum simulated capture rate, in frames per second. Must be finite and positive. |
| `camera.latency_offset_us` | `0` | Subtracted only from live delivery/PTS timestamps, and from simulated timestamps. Never applied again to SOE/EOF or known midpoint stamps. |
| `camera.sim_timestamp_lag_us` | `0` | Sim only: stamp the rendered truth instant this many microseconds later, modeling a timestamp taken after exposure. Ignored live, but validated in both modes. |
| `camera.readout_us` | `0` | Known sensor readout duration subtracted from live EOF timestamps (unknown readout is zero). |
| `camera.frame_timeout_us` | `500000` | Live silence deadline; positive, at most 10,000,000 µs. Reopens a connected but silent camera on the worker. |
| `camera.exposure_ms` | `0` | Manual exposure duration; zero leaves duration unspecified. Also controls simulated exposure blur. |
| `camera.gain` | `0` | Backend-native manual gain; zero leaves gain unspecified. |
| `camera.auto_exposure`, `camera.auto_gain` | `true` when duration/gain is zero | Live automatic-control selection. A positive manual value defaults its automatic mode to false. |
| `camera.mount_angle_std_rad` | `0.0175` | Extrinsic mount-rotation calibration uncertainty in radians (≈1 deg), live **and** sim, not sim-only. Added to the published covariance: field x/y variance gains `(range × value)²`, yaw variance gains `value²`. |
| `camera.mount_translation_std_m` | `0.01` | Extrinsic mount-translation calibration uncertainty in metres, live **and** sim, not sim-only. Added to the published field x/y variance. |
| `detector.decimate` | `2.0` | Dimensionless quad-search downsampling factor; 2.0 searches at half resolution, then refines edges at full resolution. |
| `detector.threads` | `2` | Integer detector worker count. |
| `detector.sigma` | `0.0` | Quad-search Gaussian blur sigma in pixels; zero disables blur. |
| `detector.refine_edges` | `true` | Refine detected quad edges before decoding. |
| `detector.hamming_threshold` | `1` | Maximum corrected tag-code bits (inclusive), passed to the decoder and checked for validity. |
| `detector.min_decision_margin` | `30.0` | Minimum AprilTag decode-margin score (inclusive), measuring image-intensity contrast, not a probability or pose error. |

Timing offsets and readout must be finite, nonnegative and strictly below
1,000,000 µs; construction failures name the node and key. The single conversion
boundary is observation publication (`CaptureMidpoint`): mapped V4L2 SOE adds
half the successfully configured manual exposure; EOF subtracts half exposure
plus known readout. Unknown/automatic exposure contributes zero, never a guessed
duration. Delivery/host receipt and AVFoundation mapped PTS (which does not
guarantee exposure semantics) subtract only `latency_offset_us`. Known midpoint
stamps are unchanged. V4L2 non-monotonic timestamps fall back to host receipt;
AVFoundation without a synchronized sample clock does likewise. Backends retain
the source kind rather than claiming these are all exposure timestamps.
Measure delivery/PTS bias per camera; do not subtract processing/transport delay
already excluded by a hardware source timestamp. Underflow or future stamps
publish invalid observations, never an unsigned-wrapped future timestamp.

V4L2 applies automatic/manual exposure mode, manual exposure in 100 µs units,
and requested gain controls before streaming. macOS AVFoundation supports
automatic exposure mode, but not custom duration/ISO; manual requests remain
visible as failed rather than silently ignored. Capture continues on failure:
`controls_applied` is false and `control_failure_count` increments per open
whose requested controls could not be applied. Check these before commissioning
timing. The exposure correction is omitted after a control failure.

Both mount-uncertainty keys must be finite and nonnegative. They model the
residual calibration error left after mounting each camera, which the corner
noise cannot see: a mount rotation of `mount_angle_std_rad` tilts the optical
axis and shifts the tag lever arm by `range × angle`, and `mount_translation_std_m`
shifts it directly, so the published covariance adds both to field x/y (and the
angle to yaw) before the uncertainty floors. Set them from the deployed robot's
post-mounting calibration residual — a mount measured to ±0.5 deg and ±5 mm
would use roughly half the defaults — not from the sim's synthetic error keys.

The typed pipeline configuration carries `apriltag::DetectorOptions` and the
minimum decision margin directly; there is no string parameter map. A detection
must have a decision margin at least the configured minimum and a Hamming
distance no greater than the threshold, as well as passing geometric validity
checks. Failed decode-quality checks mark the detection
invalid, excluding it from both visibility counts and localization before range
filtering or pose solving. A high margin alone does not guarantee a valid pose.

## Frame recording

When the node has a session directory (it was started by the launcher, which
passes `--log <session>/<node>.tlog`; tests, `--describe` and `--replay` have
none) and `recording.enabled` is true, each camera records frames under
`<session>/frames/<camera>/`. Every recorded frame is two files sharing a stem
`<capture_ns>_<reason>` (20-digit zero-padded capture time): a lossless 8-bit
grayscale PNG and a JSON sidecar carrying the camera's intrinsics, mounting,
detector settings, every decoded detection (with its `reject_reason`), the
published observation, and — under `--sim` — the `/sim/state` pose the frame
was rendered from. `reason` is `rejected` when a frame decoded at least one
detection but produced no valid observation, or any detection was invalid,
range-rejected, ambiguity-rejected or not in the layout; otherwise `sampled`.

Recording never blocks the node loop or the live worker: frames are converted
to gray once into a preallocated slot of a bounded queue (a full queue drops
and counts) and a writer thread encodes PNG + JSON, deleting the oldest pairs
first when a camera exceeds `max_bytes`.
Null-data frames and frames whose dimensions differ from the configured slot
are dropped and counted before conversion. A writer error removes temporary
files, counts the failed and queued pairs as dropped, logs once for the camera,
and stops recording; later frames are also counted as dropped. Failure to
initialize the recorder logs once and leaves the camera running without it.

| Key | Default | Meaning |
| --- | --- | --- |
| `recording.enabled` | `true` | Record frames when a session directory is present. |
| `recording.sample_hz` | `1.0` | Frames with a valid observation per second; `0` records none. |
| `recording.rejected_hz` | `1.0` | Rejected frames per second; `0` records none. |
| `recording.max_bytes` | `268435456` | Per-camera disk budget (256 MiB); oldest pairs deleted first. |

`max_bytes` must be a nonnegative integer; the rates must be finite and
nonnegative. Disk math: at ~2 frames/s (one sampled + one rejected at the
defaults) and ~0.8 MB per PNG, 256 MiB holds roughly 160 s of recording per
camera. Tune offline with
`bazel run //talOS/vision/recording:frame_replay -- --frames <dir> --layout
2026-robot/main_processor/vision/data/2026-rebuilt-welded.json`.

Shared pose-solver settings live in `[subsystem]` in `vision.toml`, with the
same per-camera override mechanism. Both single-tag and multi-tag observations
use the joint corner solver and publish its returned standard deviations
directly.

| Key | Default | Units and meaning |
| --- | --- | --- |
| `pixel_std_px` | `0.33` | Sim-measured corner-coordinate standard deviation in pixels; real cameras should use their calibration RMS. Independent of direct projection's `camera.pixel_noise_std`. |
| `floor_z_std_m` | `0.05` | Robot-height prior standard deviation in metres about the field floor. |
| `floor_tilt_std_rad` | `0.05` | Robot roll/pitch prior standard deviation in radians about level. |
| `huber_delta_px` | `2.0` | Huber robust-loss transition in pixels for corner residuals. |
| `min_xy_std_m` | `0.02` | Minimum published x/y standard deviation in metres. |
| `min_yaw_std_rad` | `0.035` | Minimum published yaw standard deviation in radians (about 2 degrees), selected against the accuracy preset's ±1-degree yaw mounting bias. |

The raster/detect invariant test measures the sim pixel noise with fixed seed
1000, production sensor/exposure settings, five offsets at each of 1.5, 2.5,
4 and 6 m, and eight images per pose (640×480, 500 px focal length).
Its 1280 corner-coordinate errors have standard deviation **0.3243 px** and
signed mean 0.0088 px with decimation 2, two detector workers and full-resolution
edge refinement. The sim uncertainty default remains **0.33 px**.
The test requires the measured standard deviation to stay within ±30% of
`PoseSolverOptions`' default. It uses named fixture effects matching
`vision.toml`, not a robot dependency from the framework.
This is a simulator calibration, not a claim about physical camera noise:
set each real camera's `pixel_std_px` from its measured calibration RMS.

Pixel-coordinate residuals are divided by `pixel_std_px`; the Huber transition
is divided by the same value. Floor z and roll/pitch residuals are divided by
their respective prior standard deviations, so all objective rows are
dimensionless. The reported reprojection error remains unweighted RMS pixels.
The covariance is the inverse final undamped `Jᵀ W J`, including the floor
prior and final Huber IRLS weights. It is mapped from right-increment SE(3)
coordinates into field-frame x/y/yaw before applying uncertainty floors.
Non-finite or non-positive-definite covariance invalidates the observation.
The yaw floor deliberately lets the gyro dominate heading.
The yaw floor was selected by a two-run-per-floor sweep with deterministic
±1-degree yaw, ±0.5-degree pitch and ±0.01 m translation mounting errors:
0.035 rad gave the lowest measured mean fused heading RMS without worsening
mean position RMS among 0.018, 0.035, 0.07 and 0.1 rad.
Mount errors are correlated bias, not independent per-frame noise; normalized
observation RMS alone is not a calibration objective for the fused estimator.
Rare ambiguous single-tag observations can still have heavy-tailed errors;
the estimator's separate 15-degree single-tag yaw innovation gate rejects
those jumps rather than inflating uncertainty floors to disguise them.

The accuracy scenario's second table scores each camera's valid SEND
observations once per sample timestamp during the same enabled window.
Truth is interpolated at that timestamp. It reports RMS and the interpolated
95th percentile of absolute normalized x/y/wrapped-yaw errors, plus raw yaw
RMS in degrees. A consistent Gaussian uncertainty model has normalized RMS
near 1 and absolute p95 near 1.96. When yaw normalized RMS exceeds 1, the table
also reports the smallest additional yaw floor that would make that session
consistent while retaining covariance standard deviations already above it.
This diagnostic does not change the configured floor.

With two pose candidates, ambiguity is the lower/higher reprojection-error
ratio. The single-tag ambiguity limit is **0.2**, defined by
`PoseValidationOptions`; multi-tag joint solves do not inherit this single-tag
limit. The patched AprilTag ambiguity search reports its exact count of new
minima: no alternate minimum means ambiguity `0`, whereas multiple new minima
without a returned second pose mean unknown ambiguity `1`. Single-tag
observations above the limit are rejected unless the floor-disambiguation
check resolves them; an absent second pose alone cannot resolve that check.
The floor check uses the norm of `(z/floor_z_std_m,
roll/floor_tilt_std_rad, pitch/floor_tilt_std_rad)`. It is decisive only when
one candidate is within a 3-sigma prior radius and the other outside 6 sigma.
Only then may it replace the lower-reprojection primary or override the
ambiguity rejection. A merely smaller height/tilt residual cannot select the
other planar minimum. This prevents the pinned tag-32 near-edge observation
from switching to a 24-degree-flipped pose under small mounting errors.

Only `--sim` enables synthetic observations. `rasterize = true` (default) uses
a preallocated image buffer, AprilTag detection, and 3D localization;
`rasterize = false` projects visible tag corners directly into localization.
Each camera optionally subscribes to `/sim/state` (`geometry.OdometrySample`),
published every physics tick only by `sim_gateway --physics`. Tags are rendered
from its latest absolute true field pose, not integrated drivetrain odometry.
The gateway's `--start-pose X_M,Y_M,YAW_RAD` sets the world's starting pose
(default `2,2,0`); cameras have no start-pose parameters or integration state.
The estimator's independent `initial_*` seed may differ and be corrected by
vision. No camera subscribes to the estimator's pose: estimator drift cannot
move the simulated field or confirm itself, including after a camera restart.
Under `--sim`, the renderer and direct projector use a separate true mount.
Optional `[camera]` keys `sim_mount_error_x_m`, `sim_mount_error_y_m`,
`sim_mount_error_z_m`, `sim_mount_error_roll_rad`,
`sim_mount_error_pitch_rad` and `sim_mount_error_yaw_rad` default to zero and
must be finite. The true mount composes the configured NWU extrinsic transform
with this translation and `Rz(yaw) Ry(pitch) Rx(roll)` error, then applies the
usual optical-axis mapping. Localization always uses the configured,
believed mount. Live cameras ignore these sim-only keys entirely, including
their values and validation.

With both timing keys zero, simulated observations carry the truth sample's
`sample_time_ns`, the node-clock instant its pose describes, rather than the
camera update time. With timing effects, the published stamp is
`truth.sample_time_ns + sim_timestamp_lag_us * 1000 - latency_offset_us * 1000`.
Rendering still uses the original truth instant. Lagged observations wait until
`truth.sample_time_ns + sim_timestamp_lag_us * 1000` before delivery, even when
the offset cancels the lag; this prevents unrealistic future-stamped fusion.
The lag window uses a preallocated, bounded queue sized from the configured
capture period (`ceil(lag * fps) + 2` slots); overflow drops the oldest pending
observation and increments the dropped-frame count. Every due observation is
published in order, including batches after a late update. Stale/missing truth
clears pending observations and publishes invalid immediately.
The first fresh truth captures immediately and anchors a `1 / camera.fps`
deadline grid. Subsequent captures use the first drained truth at or after the
next deadline, advancing the grid rather than resetting it to the actual truth
time. At 100 Hz polling, a 60 fps stream alternates 10/20 ms intervals with
approximately 16.7 ms mean period. Identical truth cannot create duplicate
frames. If a slow tick misses several deadlines, it captures once from real
truth, advances past that truth time and counts skipped deadlines as drops;
it never invents intermediate poses. Between captures only status publishes,
unless lagged observations become due.
Truth must pass `Subscription::fresh()` (younger than
`[robot] command_timeout_us` since receipt).
Missing or stale truth publishes `valid = false` and unhealthy status every
update, with exactly one diagnostic naming each camera per outage. Fresh truth
clears that outage. WPILib/RoboRIO simulation and the ideal gateway publish no
truth: simulated cameras need `sim_gateway --physics`, and otherwise remain
invalid rather than inventing a world.

Simulation camera keys live under `[camera]` (shared defaults in
`vision.toml`, with per-camera overrides in the named camera files):

| Key | Units and meaning |
| --- | --- |
| `pixel_noise_std` | Corner-coordinate Gaussian standard deviation in pixels, only for `rasterize = false`; never grayscale noise. Per-camera default: 0.5 px. |
| `sensor_noise_std_gray` | Independent sensor Gaussian standard deviation in 8-bit grayscale levels; rendered images only. Shared default: 2.0. |
| `exposure_gain_std` | Dimensionless frame-wide multiplicative gain standard deviation; rendered images only. Shared default: 0.02. |
| `exposure_ms` | Exposure duration in milliseconds for rendered motion blur. Shared default: 5.0 ms. |
| `rng_seed` | Nonnegative integer replay seed; each `Vision` instance explicitly owns a persistent `std::mt19937_64`, seeded after camera configuration loads and passed to both simulation paths, so successive frames differ but replay remains repeatable. |

All four noise/exposure magnitudes must be finite and nonnegative; zero disables
the corresponding effect. These are plausible simulation settings, not measured
camera specifications: 2 gray levels are about 0.8% of the 8-bit range, 0.02 gain
represents 2% frame exposure variation, and a short 5 ms exposure limits rather
than eliminates motion smear at robot speeds. Tune them against captured frames.
Rasterization receives true field-frame linear velocity (`vx_mps`, `vy_mps`)
and yaw rate (`omega_radps`) directly from `/sim/state`; it never rotates those
field axes again or differentiates a noisy localization estimate. Stationary
images still have sensor/exposure variation. The direct-corner path does not
model exposure or image sensor noise. Neither path adds arbitrary pose jitter
after solving; observation standard deviations describe uncertainty rather than
perturbing the published pose a second time.

The renderer also receives caller-owned `RenderScratch` retained beside the
camera's pixel buffer. Motion integration reuses its detection vectors, instant
image and accumulator across frames rather than hiding mutable state in camera
configuration or allocating temporary frame buffers on each render.
Motion blur initializes the static background once, then accumulates deviations
only within disjoint regions covering visible tags' start/end projected bounds.
Bounds include the projected 1.25× quiet zone and raster rounding margin, and
extend for intermediate curved motion or newly visible tags. Pixels outside
these regions retain the static background. A test-only full-frame integration
oracle requires moving-pose results to agree within one gray level. Adjacent
cells share a preprojected 11×11 vertex lattice rather than repeating four
projections per cell.
Image noise, gain and exposure accumulation use OpenCV kernels over reused
matrices. The third-party AprilTag target is compiled with `-O2` in every Bazel
mode, including `fastbuild`: noisy-image thresholding otherwise exceeds the
camera period even when the surrounding simulation code is unoptimized.

Without `--sim`, construction validates calibration and registers topics; it
does not open a camera. Live capture requires a nonempty `camera.device` set to
a stable device id, never a camera index: an AVFoundation uniqueID on macOS or
a `/dev/v4l/by-id/...` path on Linux. List macOS ids and names with
`bazel run //talOS/vision/calibrate:calibrate_camera -- --list-devices`
(uses `DiscoverCaptureDevices()`); on Linux use `ls -l /dev/v4l/by-id/`.
An empty device logs one diagnostic naming the node, disables live capture,
and publishes invalid observations and unhealthy status on each update.
Simulation ignores this requirement. With a configured device, construction
starts a worker waiting for the first node-clock tick.
The worker owns all device open/start/stop calls and all detection/localization.
A missing or disconnected configured device is recreated with node-clock
backoff of 0.1, 0.2, 0.4, 0.8, 1.6, 3.2, 6.4, then at most 8 seconds,
resetting after a processed frame. This also recovers permission denial,
capture-loop exits, and connected cameras with no frame for `frame_timeout_us`.
The silence timer starts at successful open and resets on frame receipt;
`stall_count` counts silence retirements and `reconnect_count` counts successful
opens after the first successful connection. The loop passes
time atomically and uses a nonblocking result-slot try-lock; it never opens a
device, spawns a thread, or waits for worker work. Capture replaces one pending
frame, and processing replaces one fixed-size completed observation. Replacements
in either slot count as dropped frames. Shutdown wakes and joins the worker,
which stops capture before exiting. `--describe` never opens a camera.

One observation is published per processed frame delivered to the loop, stamped
with the best-estimate mid-exposure time under the source-kind rule above.
Missing tags and rejected estimates publish
`valid = false`, never a held valid pose. Invalid detections are excluded from
visibility and localization before range checking; valid camera-to-tag ranges
above `max_range_m` are counted and rejected before solving. Estimates then pass
field-bound and reprojection gates. Standard deviations come directly from the
robust solver covariance described above. Status uses uncorrected capture-time
mean frame periods and node-clock corrected-stamp-to-publication latency in
integer microseconds (`last_latency_us`). Simulated latency is elapsed node time
since the corrected truth stamp, not necessarily zero; it includes delivery lag
when that bias is corrected. Invalid underflow/future timestamps report zero
latency and never publish a valid pose. Simulation replay uses
`--sim --replay <log>`; no wall clock enters its outputs.

Framework telemetry supports every scalar in the status schema and observes all
four camera streams. Its observers intentionally do not count as control
consumers: graph lint reports the four status-only warnings. The robot telemetry
excludes cumulative capture/process/range counters and the new recovery/control
fields, retaining its existing byte budget. All fields remain available in
shared-memory status and dispatch logs; status uses the enclosing log timestamp
instead of duplicating it as a scalar. `range_rejected_count` counts detections
culled before localization.
There is no side-channel vision server.
