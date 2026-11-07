# Robot bring-up

## Network and Mini build

- [ ] Restrain the robot, clear mechanisms, keep Driver Station disabled, and have an operator at the disable control. Expected: no actuator motion throughout the disabled checks.
- [ ] Give the Mac Mini a static `10.56.87.11` address and the RoboRIO `10.56.87.2`; verify bidirectional connectivity on the robot network. Expected: the RIO can send hardware state to the Mini address compiled into `kCompanionAddress` in `2026-robot/controller_processor/rio/src/main/cpp/Robot.cpp`.
- [ ] Inspect `2026-robot/main_processor/configuration/robot_field.toml` before building. Expected: it includes the simulation/base config but overrides `[robot] controller_address = "10.56.87.2"`; commissioning remains false. Do not rely on a hand-run `--remote` override: boot passes no arguments.
- [ ] On the Mini, from the repository root, run `bazel build //...` and `bazel test //...`. Expected: both succeed on the actual deployment architecture; Linux CI is not a substitute for this check.

## Bundle and deploy

- [ ] Run `bazel build //2026-robot/main_processor:robot_bundle`. Expected: the field-config bundle is generated at `bazel-bin/2026-robot/main_processor/robot_bundle/`.
- [ ] Deploy the RoboRIO program using `./gradlew deploy` from `2026-robot/controller_processor/rio`. Expected: the intended gateway program starts on the RIO with the Mini's companion address.
- [ ] On the Mini run `./tools/deploy.sh install bazel-bin/2026-robot/main_processor/robot_bundle`. Expected: `/opt/talos/current` selects a new read-only bundle and the previous bundle is retained. Use the documented `--prefix` option if the operator needs a different writable installation location.
- [ ] Start exactly one launcher with `/opt/talos/current/run.sh --output-dir /opt/talos/logs`; stop any boot-managed instance first. Expected: nodes stay running and a unique `/opt/talos/logs/<session-id>/` logging session is created. The launcher appends the session id; pass the output root, not a session path. Do not edit the installed bundle; rebuild and reinstall configuration changes.
- [ ] Run `./tools/deploy.sh logs` (add `--prefix` for a custom installation). Expected: session directories are listed and the newest manifest identifies the deployed config hash and node log paths. Retain that session and the RIO console output for every check below.
- [ ] For live inspection, run `bazel build //studio:web_dist`, then in another terminal `bazel run //studio/bridge:studio_bridge -- --drop-newest-publisher /talos/telemetry 5800 bazel-bin/studio/dist`. Open `http://127.0.0.1:5800` on the Mini and select Studio's system view. Expected: live node rows, source selections and loop timing are visible. Run only one Studio bridge.

## Camera intrinsics calibration

1. With the robot disabled and the camera at its deployment focus and resolution,
   generate a ChArUco board:
   `bazel run //talOS/vision/calibrate:calibrate_camera -- --board /tmp/charuco.png`.
   Print at exactly 200 pixels per 40 mm square (280 × 200 mm total), without
   scaling or cropping; mount it flat on rigid backing. The board uses 7 × 5
   squares, 40 mm square length, 30 mm markers and `DICT_4X4_50`.
2. Capture sharp, well-lit views at varied distances and tilts, moving the board
   across all image edges and the center. Keep the camera focus fixed. Use at
   least 12 distinct views, preferably 20–30; avoid motion blur and reflections.
   Existing image files must all have the same dimensions.
3. For an image directory run
   `bazel run //talOS/vision/calibrate:calibrate_camera -- --images /absolute/path/images`.
   Alternatively use `--device DEVICE_ID --seconds 120 --interval 1` for native
   AVFoundation (macOS) or V4L2 (Linux) capture. On macOS, run
   `bazel run //talOS/vision/calibrate:calibrate_camera -- --list-devices` to
   print each camera's `DiscoverCaptureDevices()` uniqueID and name; camera
   permission is required. On Linux, use `ls -l /dev/v4l/by-id/` to find the
   stable `/dev/v4l/by-id/...` path. Set each camera's `camera.device` to its
   own stable id and confirm it matches that camera's physical mount.
   Live robot vision refuses an empty device (no index fallback), logs one
   diagnostic, and publishes invalid observations and unhealthy status until
   configured and restarted. `--sim` does not require device ids.
   Live capture requests the existing source default mode (1280 × 720). Match the resulting
   width/height in deployment. Frames are sampled at the interval and accepted
   only when their corresponding board points differ by at least 2.5% of image
   width from previously accepted views.
4. Check the reported view count and RMS reprojection error in pixels. The tool
   refuses fewer than 12 views, coverage below 10 of 12 image grid cells, or RMS
   above 1 pixel. Aim below 0.5 px; inspect/repeat captures rather than trusting a
   low RMS alone. Verify coverage, focus and held-out images.
5. Re-run with `--update 2026-robot/main_processor/configuration/subsystems/front_left_camera.toml`
   (choose the actual camera file). The tool prints a `[camera]` TOML fragment
   and updates only fx, fy, cx, cy, k1, k2, k3, p1, p2, width and height within
   that table, preserving unrelated lines and comments. Unsupported/ambiguous
   table syntax is refused; keep a backup and review the diff. It never changes
   commissioning, CAN ids, device selection or mount extrinsics. Measure the
   robot-frame mount separately and repeat for every camera.
6. Measure each camera's exposure-to-timestamp bias, separately from mount and
   intrinsics calibration. Under a safe, supervised constant-speed translation
   or rotation, align camera observations against independent odometry/gyro
   history; repeat both directions and several speeds to distinguish a time
   shift from fixed mounting bias. A timestamped LED/blink exposure test is an
   alternative. Set `[camera] latency_offset_us` in that camera's TOML to the
   measured nonnegative delay (default 0, finite and below 1,000,000 µs).
   Published `sample_time_ns = capture_ns - latency_offset_us * 1000`;
   do not subtract processing delay if the source already timestamps exposure.
   Rebuild/redeploy and repeat the motion check: timestamp alignment should
   improve, and status `last_latency_us` now includes the correction. Leave
   `sim_timestamp_lag_us = 0` on the robot; it is a sim-only injection, ignored
   live, not a calibration setting.

The tool uses OpenCV 4.12's ChArUco detector: the BCR module exposes both
`@opencv//:objdetect` and `@opencv//:calib3d`. Calibration uses the five-coefficient
Brown–Conrady model (no rational/thin-prism extensions).

## Vision frame recording and replay

Every launched camera records frames while a session directory is present
(always, under the launcher): `recording.enabled = true` writes lossless 8-bit
PNG frames and a detection-context JSON sidecar to
`<session>/frames/<camera>/`, rate-limited by `recording.sample_hz` /
`recording.rejected_hz` and capped by `recording.max_bytes` (oldest pairs
deleted first). Fetch a session with `./tools/deploy.sh fetch` as usual; the
frames come along. Re-run the detector and localization offline over a camera's
frames with:

```sh
bazel run //talOS/vision/recording:frame_replay -- \
  --frames <session>/frames/front_left_camera \
  --layout 2026-robot/main_processor/vision/data/2026-rebuilt-welded.json
```

The tool prints a Markdown summary: valid observations and per-reason rejection
counts before/after, plus position/yaw error RMS against the recorded truth.
Override the recorded detector settings with `--decimate`, `--threads`,
`--sigma`, `--refine-edges`, `--hamming`, `--min-decision-margin` and
`--backend`; pass `--max-range-m` to match a non-default `camera.max_range_m`.

## Uncommissioned, then disabled

- [ ] With the uncommissioned bundle and DS disabled, run `bazel run //2026-robot/main_processor/drivetrain:monitor -- --config 2026-robot/main_processor/configuration/robot_field.toml`. Expected: RIO logs `commissioned=0`, monitor shows not configured, and every actuator stays neutral. This deliberate refusal is not a CAN fault to bypass.
- [ ] Inspect `/hw/status` (Studio channels use `hw/status/*`; field definitions are in `talOS/bridge/bridge_status.fbs`). Expected: `states_received_count` advances, `state_age_us` stays low, and `identity_rejected_count` / `ownership_rejected_count` do not grow. Before the first frame, `state_age_us = 0` alone does not prove connectivity.

## Human commissioning

- [ ] In Phoenix Tuner X, compare every physical CAN id against `2026-robot/main_processor/configuration/subsystems/drivetrain.toml`, `2026-robot/main_processor/configuration/subsystems/shooter.toml` and `2026-robot/main_processor/configuration/subsystems/intake.toml`. Expected: each device resolves to the intended mechanism; a human corrects any mismatch.
- [ ] Check each motor and encoder `inverted`; mechanically align all four swerve modules and measure all four CANcoder `offset_rot` against Phoenix Tuner X. Expected: positive demands and measured directions agree, with measured offsets rather than unchecked zero defaults.
- [ ] Tune intake `slot0` before demanding roller velocity, and validate drivetrain gains and `sensor_to_mechanism_ratio`. Expected: intake gains are not all zero; drive feedforward is in mechanism units (the existing `v = 0.12` with ratio `6.75` needs checking; about `0.81` would be the corresponding mechanism-side value, not an automatic replacement).
- [ ] Only after those checks, have a human set `commissioned = true` and update `2026-robot/main_processor/configuration/config_golden_test.cc` in the same change. Repeat Mini build/test, bundle build and install above. Expected: RIO accepts the new configuration; `/hw/status.configured` and monitor's configured flag become true.
- [ ] Keep DS disabled while exercising sticks and mechanism buttons. Expected: monitor reports disabled, Studio shows no selected actuation source, and all actuators remain neutral. Then enable with controls released. Expected: no motion without a live target.

## Drivetrain on blocks

- [ ] Lift and secure all drive wheels. With a spotter, enable teleop and apply small forward, strafe and rotation inputs separately, then release and disable. Expected: module angles and wheel directions agree with the requested chassis motion; release stops the demand and disable always neutralizes it.
- [ ] If isolating the chassis from the driver, pause the `driver` process first (use its PID in Studio) so there is only one writer on the teleop lane. Bench only, while enabled in teleop, run `bazel run //2026-robot/main_processor/drivetrain:send_target -- 0.1 0 0 2`. Expected: a small forward demand for two seconds, then lane release and neutral. Never use this tool at a match; it bypasses the operator interface and is not in the bundle. End the bench test disabled before resuming the driver.
- [ ] Watch drivetrain monitor and Studio while repeating release/disable. Expected: measured velocities respond in the correct direction, no hardware fault appears, and the selected target disappears when disabled or stale.

## Shooter, then intake

- [ ] Secure the shooter, clear its discharge path, enable teleop and use the shooter binding in `2026-robot/main_processor/configuration/subsystems/shooter.toml` (button 1). Expected: the flywheel follows the configured demand in the correct direction, the beam input changes when interrupted, and release/disable removes drive (coast-down is not continued actuation).
- [ ] Secure the intake, enable teleop and use the intake binding in `2026-robot/main_processor/configuration/subsystems/intake.toml` (button 2). Expected: the roller intakes in the correct direction, beam detection changes its state as implemented, and release/disable neutralizes it. Observe both mechanisms in Studio; drivetrain monitor is not a shooter/intake monitor.

## Link timing and fault injection

- [ ] Collect representative enabled sessions with `./tools/deploy.sh fetch /media/usb/robot-sessions` on the Mini (add `--prefix` for a custom installation), using a fresh destination to avoid merging existing sessions. Expected: each complete session folder, its manifest and all restarted-node log attempts are copied. Stop the stack before the final fetch for a stable completed recording. Transfer the folders to the analysis machine and decode each with `bazel run //talOS/launcher:log_dump -- <session-dir>` (replace the placeholder with a copied session directory). Expected: timestamped topic records and named payload fields are available; preserve the raw `.tlog` files and record the config hash.
- [ ] Measure `/hw/state` inter-arrival from the launcher logs. Pass: median about **5 ms**, **p99 under 10 ms**, and **no gap of 100 ms while enabled**.
- [ ] Correlate target receipts with drivetrain ticks. Pass: target age at each tick is **about 25 ms at most**, **never `command_timeout_us`**. Freshness starts at node receipt, not a sender timestamp; account for the consumer's drain/tick when interpreting a stalled consumer.
- [ ] Check `DrivetrainState.timestamp_ns` against node time, not the RIO sample clock. Expected: odometry's integration `dt` uses node-time state timestamps; it must not interpret RIO uptime as node time.
- [ ] Record loop lateness for every node in Studio's system view on the Mini. Expected: the Mach time-constraint policy is accepted for every node period and no node logs `realtime scheduling incomplete`. Record measured lateness; no numeric lateness threshold has been specified. `TALOS_REALTIME=0` disables the realtime request and is not the acceptance configuration.
- [ ] Inspect `/hw/status` alongside the timing trace. Expected: `configured` stays true, receive/send counters advance, and rejection counters stay flat. Record `state_age_us`, `timed_out_mask` and `subsystem_count`; timeout bit positions follow actuator-owning subsystem names in sorted order. Record `clock_offset_valid`, `clock_offset_us`, `clock_offset_spread_us`, `clock_window_count`, `clock_boot_id`, `clock_resets_count` and `clock_ignored_count` when relating RIO sample times to node time; this is a bounded latency estimate, not clock synchronization.
- [ ] On blocks, use the PIDs in Studio to send `SIGSTOP` then `SIGCONT` to `driver` (chassis) and, separately, `teleop` (mechanism buttons), with measured pauses of **50 ms** and **200 ms**. Pass for **200 ms**: each subsystem goes neutral within **`command_timeout_us` plus a tick**, timing out its own lane. Record the 50 ms behavior without inventing an extra pass threshold; disable between trials and verify safe recovery before re-enabling.
- [ ] Pause drivetrain with `SIGSTOP` for **80 ms**, then resume with `SIGCONT`. Expected: record hardware response, target receipt/tick timing and recovery; no separate numeric pass threshold is specified for this pause. Keep the spotter ready to disable.
- [ ] With wheels spinning on blocks, unplug RIO Ethernet. Expected: actuation goes neutral on link loss, stale hardware samples become invalid, and odometry does not run away; `state_age_us` grows and timeout diagnostics identify the affected owners. Disable before reconnecting and verify recovery without unintended motion.
- [ ] Measure stick-to-drive-request median on the real link. Expected: report the measured median alongside the **2.99 ms simulation** result and **4.98 ms old-arbiter baseline**; neither reference is a specified real-link pass threshold.

## Boot handoff

- [ ] Disable, stop the manually launched stack, then run `./tools/deploy.sh enable-boot --user <mini account>`. Expected: `/Library/LaunchDaemons/org.talos.robot.plist` is loaded (`sudo launchctl print system/org.talos.robot`), and after a reboot with nobody logged in a single installed stack starts with no command-line addressing override. Recheck configuration, logs and disabled neutrality after restart. For a regression, `./tools/deploy.sh rollback` selects the previous whole bundle; stop/restart the stack and recheck its config hash before use.
