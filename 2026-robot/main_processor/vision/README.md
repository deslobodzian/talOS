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

Without `--sim`, construction validates calibration, registers topics and starts
a worker waiting for the first node-clock tick; it does not open a camera.
The worker owns all device open/start/stop calls and all detection/localization.
A missing or disconnected device is recreated with node-clock backoff of
1, 2, 4, then at most 8 seconds, resetting after a processed frame. This also
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
