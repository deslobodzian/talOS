# Camera localization nodes

One `vision::Vision` implementation is built as four independent nodes:
`front_left_camera`, `front_right_camera`, `rear_left_camera`, and
`rear_right_camera`. Each publishes `geometry.PoseObservation` on its own
`/<name>/state` and `vision.VisionStatus` on `/<name>/status`. The pose estimator
lists all four camera names. Sensing runs at 20 Hz whether enabled or disabled;
there are no targets or actuation gates.

Each `configuration/subsystems/<name>.toml` holds just that camera's
calibration and device. Its `[subsystem] defaults = "vision.toml"` layers the
shared tag size, uncertainty bases, period, simulation renderer selection, and
field starting pose. The official 2026 welded layout is embedded in the binary;
installed bundles need no vision runfiles. Mounts map OpenCV optical coordinates
to robot NWU using `Rz(yaw) Ry(pitch) Rx(roll)`. Positive pitch points a forward
lens downward, preserving the previous calibration convention. The checked-in
mounts are season-plausible defaults, not measured calibration.

Only `--sim` enables synthetic observations. `rasterize = true` (default) uses
a preallocated image buffer, AprilTag detection, and 3D localization;
`rasterize = false` projects visible tag corners directly into localization.
`SimTruth` consumes each `/drivetrain/state` sample, integrates its measured
robot-relative chassis motion through `SE2::Exp`, and applies its gyro heading
plus the configured starting yaw. Duplicate/invalid samples and gaps longer
than 100 ms cannot advance truth. No camera subscribes to the estimator's pose:
estimator drift cannot move the simulated field or confirm itself. Golden
config tests keep camera and estimator starting poses equal.

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
Rasterization receives field-relative measured linear velocity and yaw rate from
`SimTruth`; it never differentiates a noisy localization estimate. Stationary
images still have sensor/exposure variation. The direct-corner path does not
model exposure or image sensor noise. Neither path adds arbitrary pose jitter
after solving; observation standard deviations describe uncertainty rather than
perturbing the published pose a second time.

The renderer also receives caller-owned `RenderScratch` retained beside the
camera's pixel buffer. Motion integration reuses its detection vectors, instant
image and accumulator across frames rather than hiding mutable state in camera
configuration or allocating temporary frame buffers on each render.
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
backoff of 1, 2, 4, then at most 8 seconds, resetting after a processed frame. This also
recovers first-launch permission denial and capture-loop exits. The loop passes
time atomically and uses a nonblocking result-slot try-lock; it never opens a
device, spawns a thread, or waits for worker work. Capture replaces one pending
frame, and processing replaces one fixed-size completed observation. Replacements
in either slot count as dropped frames. Shutdown wakes and joins the worker,
which stops capture before exiting. `--describe` never opens a camera.

One observation is published per processed frame delivered to the loop, stamped
with that frame's capture time. Missing tags and rejected estimates publish
`valid = false`, never a held valid pose. Invalid detections are excluded from
visibility and localization before range checking; valid camera-to-tag ranges
above `max_range_m` are counted and rejected before solving. Estimates then pass
field-bound and reprojection gates. Standard deviations scale as
`base * distance^2 / sqrt(tag_count) *
(1 + 2*ambiguity)`. Status uses capture-time mean frame periods and node-clock
capture-to-publication latency in integer microseconds, with zero simulated
latency. Simulation replay uses `--sim --replay <log>`; no wall clock enters its
outputs.

Framework telemetry supports every scalar in the status schema and observes all
four camera streams. Its observers intentionally do not count as control
consumers: graph lint reports the four status-only warnings. Eight scalar status
channels per camera preserve the robot's radio-budget check; the status uses the
enclosing log/telemetry timestamp instead of duplicating it as a scalar.
`range_rejected_count` counts detections culled before localization.
There is no side-channel vision server.
