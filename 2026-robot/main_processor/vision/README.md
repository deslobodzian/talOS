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
lens downward. Calibrate the mounts for the deployed robot.

Shared configuration defaults and code fallbacks (overridable per camera):

| Key | Default | Units and meaning |
| --- | --- | --- |
| `subsystem.period_us` | `10000` | Node/status period in microseconds (100 Hz), not the capture period. |
| `camera.fps` | `60` | Requested live capture rate and maximum simulated capture rate, in frames per second. Must be finite and positive. |
| `camera.latency_offset_us` | `0` | The camera's measured stamp bias, subtracted from every live timestamp kind (after its exposure/readout correction) and from simulated timestamps. |
| `camera.sim_timestamp_lag_us` | `0` | Sim only: stamp the rendered truth instant this many microseconds later, modeling a timestamp taken after exposure. Ignored live, but validated in both modes. |
| `camera.readout_us` | `0` | Known sensor readout duration subtracted from live EOF timestamps (unknown readout is zero). |
| `camera.frame_timeout_us` | `500000` | Live silence deadline; positive, at most 10,000,000 µs. Reopens a connected but silent camera on the worker, and is the maximum live stamp age (see timestamps below). |
| `camera.exposure_ms` | `5.0` | Shared manual exposure duration; also controls simulated exposure blur. Zero leaves duration unspecified. |
| `camera.gain` | `0` | Backend-native manual gain; zero leaves gain unspecified. |
| `camera.auto_exposure`, `camera.auto_gain` | `false`, `true` | Fallbacks are true only when duration/gain is zero; the shared exposure is positive and gain is zero. |
| `camera.mount_angle_std_rad` | `0.0175` | Extrinsic mount-rotation calibration uncertainty in radians (≈1 deg), live **and** sim, not sim-only. Added to the published covariance: field x/y variance gains `(range × value)²`, yaw variance gains `value²`. |
| `camera.mount_translation_std_m` | `0.01` | Extrinsic mount-translation calibration uncertainty in metres, live **and** sim, not sim-only. Added to the published field x/y variance. |
| `detector.decimate` | `2.0` | Dimensionless quad-search downsampling factor; 2.0 searches at half resolution, then refines edges at full resolution. |
| `detector.threads` | `2` | Integer detector worker count. |
| `detector.sigma` | `0.0` | Quad-search Gaussian blur sigma in pixels; zero disables blur. |
| `detector.refine_edges` | `true` | Refine detected quad edges before decoding. |
| `detector.hamming_threshold` | `1` | Maximum corrected tag-code bits (inclusive), passed to the decoder and checked for validity. |
| `detector.min_decision_margin` | `30.0` | Minimum AprilTag decode-margin score (inclusive), measuring image-intensity contrast, not a probability or pose error. |
| `subsystem.backend` | `"auto"` | Use Metal when available and supported, otherwise CPU (always CPU off macOS). On macOS, Metal pipelines compile on a background thread and the camera detects on the CPU until they are ready. `"cpu"` forces CPU; `"metal"` requires Metal, rejects execution fallback, and fails construction if the pipelines are not compiled within 5 s. On macOS, with `auto` or `metal`, the node logs the backend it starts on and each switch. |

Timing offsets and readout must be finite, nonnegative and strictly below
1,000,000 µs; construction failures name the node and key. The single conversion
boundary is observation publication (`CaptureMidpoint`): mapped V4L2 SOE adds
half the successfully configured manual exposure; EOF subtracts half exposure
plus known readout. Unknown/automatic exposure contributes zero, never a guessed
duration. Delivery/host receipt, AVFoundation mapped PTS (which does not
guarantee exposure semantics) and known midpoint stamps get no exposure
correction. Every kind then subtracts `latency_offset_us`. uvcvideo stamps are
always treated as delivery stamps: the driver labels its buffers SOE, but with
its default `hwtimestamps=0` stamps the host arrival of the frame's first USB
packet. With `hwtimestamps=1` its bias is still absorbed only by
`latency_offset_us`.
V4L2 non-monotonic timestamps fall back to host receipt;
AVFoundation without a synchronized sample clock does likewise. Backends retain
the source kind rather than claiming these are all exposure timestamps.
Measure each camera's remaining stamp bias; do not subtract processing/transport
delay already excluded by a hardware source timestamp. Underflow or future stamps
publish invalid observations, never an unsigned-wrapped future timestamp. So
does a live stamp older than `frame_timeout_us` when published; one diagnostic
names the camera when its stamps turn stale and one when they recover.

V4L2 applies automatic/manual exposure mode, manual exposure in 100 µs units,
and requested gain controls before streaming. macOS AVFoundation supports
automatic exposure mode, but not custom duration/ISO; manual requests remain
visible as failed rather than silently ignored. Capture continues on failure:
`controls_applied` is false and `control_failure_count` increments per open
whose requested controls could not be applied. Check these before commissioning
timing. The exposure correction is omitted after a control failure.


The typed pipeline configuration carries `apriltag::DetectorOptions` and the
minimum decision margin directly; there is no string parameter map. A detection
must have a decision margin at least the configured minimum and a Hamming
distance no greater than the threshold, as well as passing geometric validity
checks. Failed decode-quality checks mark the detection
invalid, excluding it from both visibility counts and localization before range
filtering or pose solving. A high margin alone does not guarantee a valid pose.
A tag id decoded more than once in one frame (a false decode, or a printed copy
of a field tag elsewhere in view) enters localization once, as one of its three
strongest decodes (higher decision margin, then lower per-tag reprojection
error). Beside other ids it is the decode consistent with them, whatever its
margin: each detection's single-tag poses propose robot poses, each scored by
the joint reprojection residual with every id at its best-fitting decode, and
the best proposal decides. A repeated id alone in the frame has nothing to
check against and keeps its strongest decode.

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
nonnegative. Replay captured frames with:

```sh
bazel run //talOS/vision/recording:frame_replay -- --frames <dir> --layout 2026-robot/main_processor/vision/data/2026-rebuilt-welded.json
```

Shared pose-solver settings live in `[subsystem]` in `vision.toml`, with the
same per-camera override mechanism. Both single-tag and multi-tag observations
use the joint corner solver and publish its returned standard deviations
directly.

| Key | Default | Units and meaning |
| --- | --- | --- |
| `pixel_std_px` | `0.33` | Nominal corner-coordinate standard deviation in pixels, the lower bound of the residual scaling below; real cameras should use their calibration RMS. Independent of direct projection's `camera.pixel_noise_std`. |
| `floor_z_std_m` | `0.05` | Robot-height prior standard deviation in metres about the field floor. |
| `floor_tilt_std_rad` | `0.05` | Robot roll/pitch prior standard deviation in radians about level. |
| `huber_delta_px` | `2.0` | Huber robust-loss transition in pixels for corner residuals. |
| `min_xy_std_m` | `0.02` | Minimum published x/y standard deviation in metres. |
| `min_yaw_std_rad` | `0.035` | Minimum published yaw standard deviation in radians (about 2 degrees). |

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

The solver's information matrix assumes `pixel_std_px` corner noise. It scales
the resulting covariance by the fit's a-posteriori variance factor
`s² = χ² / (m − 6)`, where `χ²` sums the squared Huber-weighted whitened
residuals over its `m` rows (two per corner plus three floor rows), clamped to
`[1, max_reprojection_error_px² / (2 · pixel_std_px²)]`: a fit worse than the
nominal noise publishes proportionally larger uncertainty, never less than
nominal and at most the per-coordinate variance ratio of a fit at the 3.5 px
joint reprojection reject threshold (about 56, a 7.5× standard deviation, at
the defaults). It then returns field-frame x/y/yaw standard deviations after
applying the configured uncertainty floors and mount-calibration uncertainty.
Non-finite or non-positive-definite covariance invalidates the observation.
Single-tag ambiguity rejection and floor-prior disambiguation are implemented
in [localization.h](../../../talOS/vision/localization.h).

Only `--sim` enables synthetic observations. `rasterize = true` (default)
renders into a preallocated image buffer, then runs AprilTag detection, 3D
localization and frame recording, all on the camera's worker thread, never in
`update()`; `rasterize = false` projects visible tag corners directly into
localization inside `update()`, which is bounded by the visible tag count and
deterministic tick for tick.
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
unless lagged observations become due (or, rasterized, a worker result arrives).
With `rasterize = true` a due capture posts one request, a copy of its
`/sim/state` sample, into a single slot the worker takes from; a request still
untaken when the next capture is due is replaced and counted in
`frames_dropped_count`, as is a finished result the worker replaces before the
loop collected it. Each update collects at most one finished result with a
nonblocking try-lock and handles it like a direct-projection capture (counted
processed, then the lag queue). Its observation therefore arrives one or more
updates after its capture, still stamped with the truth instant it was rendered
from; `last_latency_us` includes the render and detection time.
Truth must pass `Subscription::fresh()` (younger than
`[robot] command_timeout_us` since receipt).
Missing or stale truth publishes `valid = false` and unhealthy status every
update, with exactly one diagnostic naming each camera per outage. Fresh truth
clears that outage. An outage also discards the captures the worker still
holds: their results carry the outage epoch they were posted in, and the loop
drops and counts any result from before the latest outage, so no observation
is published from truth captured before stale truth was seen.
WPILib/RoboRIO simulation and the ideal gateway publish no truth: simulated
cameras need `sim_gateway --physics`, and otherwise remain invalid rather than
inventing a world.

Simulation camera keys live under `[camera]` (shared defaults in
`vision.toml`, with per-camera overrides in the named camera files):

| Key | Units and meaning |
| --- | --- |
| `pixel_noise_std` | Corner-coordinate Gaussian standard deviation in pixels, only for `rasterize = false`; never grayscale noise. Per-camera default: 0.5 px. |
| `sensor_noise_std_gray` | Independent sensor Gaussian standard deviation in 8-bit grayscale levels; rendered images only. Shared default: 2.0. |
| `exposure_gain_std` | Dimensionless frame-wide multiplicative gain standard deviation; rendered images only. Shared default: 0.02. |
| `exposure_ms` | Exposure duration in milliseconds for rendered motion blur. Shared default: 5.0 ms. |
| `rng_seed` | Nonnegative integer seed; each `Vision` instance explicitly owns a persistent `std::mt19937_64`, seeded after camera configuration loads and passed to both simulation paths, so successive frames differ. Replay is repeatable only with `rasterize = false`: which raster captures the worker renders, and when their results arrive, depend on its speed, as for live cameras. |

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
backoff of 0.1, 0.2, 0.4, 0.8, then at most 1 second, resetting after a
processed frame. This also recovers permission denial,
capture-loop exits, and connected cameras with no frame for `frame_timeout_us`.
The silence timer starts at successful open and resets on frame receipt;
`stall_count` counts silence retirements and `reconnect_count` counts successful
opens after the first successful connection. Each connection state change
(`Connected`, `Disconnected`, `Error`) logs one line naming the node and the
reason, the source's own (missing device, permission, unsupported mode, a
failed V4L2 call with its errno) when it gives one; a retry that fails the same
way logs nothing. The loop passes
time atomically and uses a nonblocking result-slot try-lock; it never opens a
device, spawns a thread, or waits for worker work.

Capture hands frames to the worker through one newest-wins triple buffer
(`talos::vision::FrameMailbox`): the source's callback copies the driver
buffer it borrows (V4L2 mmap buffer, locked CoreVideo buffer) into the filling
slot and swaps it with the ready slot; the worker swaps the ready slot with the
one it processes. The worker therefore always gets the newest complete frame,
and the frame it is processing is never overwritten. Each slot is sized by
the first frame it holds, in the format the source negotiated when it opened
(V4L2 delivers at most 2 B/px, AVFoundation BGRA 4 B/px), and grows only for a
larger frame, so steady-state capture does not allocate.
Processing replaces one fixed-size completed observation. `frames_dropped_count`
is the mailbox's single count of every frame that never reached a published
observation: a ready frame replaced before the worker took it, a frame without
an image, errored or short driver buffers the source reports, frames discarded
with a retired capture session, and a completed observation replaced before
the loop published it. Shutdown wakes and joins the worker,
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
latency, stale live stamps report their age, and neither publishes a valid
pose. Simulation replay uses
`--sim --replay <log>`; with `rasterize = false` no wall clock enters its
outputs.

See [DATA_FLOW](../../../docs/DATA_FLOW.md) for telemetry transport and budgets.
`range_rejected_count` counts detections culled before localization.
