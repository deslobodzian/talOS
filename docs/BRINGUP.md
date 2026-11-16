# Robot bring-up

The procedure every robot on talOS goes through, from a new main processor
to boot operation. A robot's own repository adds the checks of its own
mechanisms and routines (what each binding drives, its autonomous
selection) to the [mechanism checks](#mechanisms-one-at-a-time). For the
one-Kraken RoboRIO/Mini test bench, use [BENCH.md](BENCH.md). Before each
match, the drive team runs [PRE_MATCH.md](PRE_MATCH.md).

`tools/deploy.sh` below is talOS's: from a robot repository it is the one
in the talOS checkout its `MODULE.bazel` names (`../talos` beside the
repository, or `talos/` as a submodule; [robot repository](GLOSSARY.md)),
`../talos/tools/deploy.sh` for a repository made the
[documented way](FIRST_MECHANISM.md#2-make-your-robot-repository).

## Network and Mini build

- [ ] Restrain the robot, clear mechanisms (and the discharge path of any mechanism that ejects or launches), keep Driver Station disabled, and have an operator at the disable control. Expected: no actuator motion throughout the disabled checks.
- [ ] Give the Mac Mini a static `10.56.87.11` address and the RoboRIO `10.56.87.2`; verify bidirectional connectivity on the robot network. Expected: the RIO can send hardware state to the Mini address the program was built with (`10.56.87.11` by default; `-Pcompanion`, [RoboRIO](../talOS/hardware/README.md#roborio)).
- [ ] Inspect the robot's field config, the `config` of its `talos_bundle`, before building ([DEPLOYMENT.md](DEPLOYMENT.md#build-a-bundle); the bench's is `bench/main_processor/configuration/robot_bench.toml`). Expected: it includes the simulation/base config but overrides `[robot] controller_address = "10.56.87.2"`; commissioning remains false. Do not rely on a hand-run `--remote` override: boot passes no arguments.
- [ ] On the Mini, from the root of the talOS checkout the robot repository's `MODULE.bazel` names, run `tools/mac_qualify.sh`: host information, the [boot settings](DEPLOYMENT.md#on-the-main-processor) (`deploy.sh boot-check`), `bazel build //...`, `bazel test --test_output=errors //...`, launcher `--describe-only` on every robot and example config in the checkout, and [restart latency](../talOS/launcher/README.md#restart-latency) on the bench robot's physics simulation (idle and under disk load, with Python node spawn times; it asks for `sudo` once to evict the page cache). The robot's own nodes, such as the pose_estimator `update()` benchmark, are qualified from the robot's repository. Stop any boot-managed launcher first. Expected: every step `PASS` in its summary on the actual deployment architecture (boot settings pass only once FileVault is off and `enable-boot` has set `pmset`; rerun the step after the boot items below); Linux CI is not a substitute. Retain its log (path printed last) and record the results, dated, in [BENCH_RESULTS.md](BENCH_RESULTS.md) for TODO.md MAC-1 and INC-3.

## Bundle and deploy

- [ ] From the robot's repository, run `bazel build --config=release //:robot_bundle` (the template's `talos_bundle`; a repository that keeps it elsewhere uses its own label). Expected: the field-config bundle is generated at `bazel-bin/robot_bundle/`.
- [ ] Deploy the RoboRIO program using `./gradlew deploy` from the `controller_processor/rio` directory of the talOS checkout the robot repository's `MODULE.bazel` names (`../talos/controller_processor/rio` beside the repository, `talos/controller_processor/rio` as a submodule). Expected: the intended gateway program starts on the RIO, and its console line `[<s>.<ms>] hardware gateway build <commit> companion <address>:5803` names the Mini's address.
- [ ] On the Mini, from the robot repository's root, run `../talos/tools/deploy.sh install bazel-bin/robot_bundle` (the talOS checkout's `tools/deploy.sh`, as above). Expected: it warms each member without a `warning: warm-up` line ([warm-up](DEPLOYMENT.md#warm-up)), prints that the launcher accepts the graph ([graph check](DEPLOYMENT.md#graph-check)), `/opt/talos/current` selects a new read-only bundle and the previous bundle is retained. Use the documented `--prefix` option if the operator needs a different writable installation location.
- [ ] Turn FileVault off (System Settings > Privacy & Security) and macOS automatic update installs off (System Settings > General > Software Update > Automatic updates). Expected: `fdesetup status` prints `FileVault is Off.`; `./tools/deploy.sh boot-check` reports no `WARN` for `AutomaticallyInstallMacOSUpdates`.
- [ ] Run `./tools/deploy.sh enable-boot --user <service account>` (use `--dry-run` first to inspect; it uses sudo, including `pmset -a autorestart 1 sleep 0`). Expected: it ends with `boot-check` passing; bundle read/traversal checks pass as that account, `/opt/talos/logs` is owned by it with mode 0755, and the installed bundle remains read-only. Start as that account and verify a session manifest appears without permission repair.
- [ ] Start exactly one launcher with `/opt/talos/current/run.sh --output-dir /opt/talos/logs`; stop any boot-managed instance first. Expected: nodes stay running and a unique `/opt/talos/logs/<session-id>/` logging session is created. The launcher appends the session id; pass the output root, not a session path. Do not edit the installed bundle; rebuild and reinstall configuration changes.
- [ ] Run `./tools/deploy.sh logs` (add `--prefix` for a custom installation). Expected: session directories are listed oldest first by start time, each with its revision, match class and run outcomes, and the newest manifest identifies the deployed config hash, revision and node log paths. Retain that session and the RIO console output for every check below.
- [ ] On a disposable session with a small `[robot] log_max_bytes`, exceed each event writer's share. Expected: control processes continue, each log keeps its newest records in whole-record segments within its share (the oldest `<stem>.segment<k>.tlog` deleted), the completed manifest reports `log_capped: true` for incomplete captures, and every capped member's kept records span between 7/8 of the session's log window and the window, a member storing more than its expected rate less (REQ-OPS-32; `log_dump` on the session directory: each process's first record; the window is `talos.log`'s `Event-log window` line). Every roster member, the Studio bridge included, also writes `<name>.log` (its stdout and stderr), up to 64 MiB per member per session outside the event-log share (and the launcher's own `talos.log` up to 64 MiB), and each recording camera up to its `[recording] max_bytes` of frames. Older sessions count all of it within the other half of `log_max_bytes`, except the 16 newest matches, which only a disk without room for the next session (retention's free floor) removes: size the disk by the [retention sizing](../talOS/launcher/README.md#session-segments-and-logs), and after a session in which retention removed anything read its `retention.txt`.
- [ ] For live inspection, use the Studio the installed bundle serves: the boot service's `run.sh` starts its bridge ([DEPLOYMENT.md](DEPLOYMENT.md#build-a-bundle)). The bundle carries the `studio_bridge` binary and the Studio webroot ([DEPLOYMENT.md](DEPLOYMENT.md#build-a-bundle)); a session log line saying the bundle carries no Studio means it was built without them. Open `http://127.0.0.1:5800` and select Studio's system view. Expected: live node rows, source selections and loop timing are visible. Run only one bridge; follow its [operating limits and load probe](../studio/bridge/README.md#websocket-operating-limits) on the field link with other traffic.

## Camera capture check

Before calibrating, check what each camera delivers through talOS's own
capture backend, at the mode its config will ask for:

```sh
bazel run //talOS/vision/capture:capture_check -- --device /dev/v4l/by-id/... --list
bazel run //talOS/vision/capture:capture_check -- --device /dev/v4l/by-id/... \
  --width 1280 --height 800 --fps 120 --seconds 20 --detect --decimate 2 --threads 2
```

`--list` prints the modes the source can open. A run prints the mode the
source opened (it fails naming the mode when the device does not offer it),
delivered and steady frame rate, frame-interval percentiles, frames missing
from interval gaps and those the source dropped, capture-thread CPU per frame
with that thread's scheduling, and process CPU; `--detect` adds the CPU
AprilTag backend's stage times and capture-to-result latency on the newest
frame, `--save DIR --save-count N` saves one gray PNG per second. Expected:
the steady rate within 1% of the requested one, no missing or dropped frames
on an idle host, and the capture thread below the node loops (`nice 10`) when
run inside a node. Timing from a busy host is not evidence (TESTING.md §6).

## Camera intrinsics calibration

1. With the robot disabled and the camera at its deployment focus and resolution,
   generate a ChArUco board:
   `bazel run //talOS/vision/calibrate:calibrate_camera -- --board /tmp/charuco.png`.
   Print at exactly 200 pixels per 40 mm square (280 × 200 mm total), without
   scaling or cropping; mount it flat on rigid backing. The board uses 7 × 5
   squares, 40 mm square length, 30 mm markers and `DICT_4X4_50`. The PNG
   carries no print size: print it at 127 pixels per inch (e.g.
   `magick charuco.png -units PixelsPerInch -density 127 charuco.pdf`, then
   "actual size"). 280 mm does not fit US Letter (279.4 mm); use A4, A3 or
   tabloid. Measure five squares along each axis: both must read the same
   (200.0 mm); an overall scale error leaves the intrinsics unchanged, an
   aspect error does not.
2. Capture sharp, well-lit views at varied distances and tilts, moving the board
   across all image edges and the center. Keep the camera focus fixed. Use at
   least 12 distinct views, preferably 20–30; avoid motion blur and reflections.
   Existing image files must all have the same dimensions.
3. For an image directory run
   `bazel run //talOS/vision/calibrate:calibrate_camera -- --images /absolute/path/images`.
   Alternatively use `--device DEVICE_ID --width W --height H --fps F
   --seconds 120 --interval 1` (optionally `--exposure-ms MS --gain G`, Linux
   only) for native AVFoundation (macOS) or V4L2 (Linux) capture, at the mode
   the camera's config will run (an OV9281: 1280 × 800). On macOS, run
   `bazel run //talOS/vision/calibrate:calibrate_camera -- --list-devices` to
   print each camera's `DiscoverCaptureDevices()` uniqueID and name; camera
   permission is required. On Linux, use `ls -l /dev/v4l/by-id/` to find the
   stable `/dev/v4l/by-id/...` path. Set each camera's `camera.device` to its
   own stable id and confirm it matches that camera's physical mount.
   Live robot vision refuses an empty device (no index fallback), logs one
   diagnostic, and publishes invalid observations and unhealthy status until
   configured and restarted. `--sim` does not require device ids.
   Live capture opens exactly that mode or fails naming it; set the camera
   file's `width`/`height` to it (`--update` does). Frames are sampled at the
   interval.
   Near-duplicate rejection compares the RMS
   displacement of corresponding board points when at least eight overlap;
   the threshold is 2.5% of image width.
4. Check the reported view count and RMS reprojection error in pixels. The tool
   refuses fewer than 12 views, coverage below 10 of 12 image grid cells, or RMS
   above 1 pixel. Inspect coverage, focus and held-out images rather than
   trusting a low RMS alone.
5. Re-run with `--update <robot>/configuration/subsystems/<camera>.toml`
   (the camera's own subsystem file). The tool prints a `[camera]` TOML fragment
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
   This offset applies to every stamp kind, after the kind's own correction:
   V4L2 SOE adds half exposure; EOF subtracts half exposure plus configured
   `readout_us` (default zero if unknown). Exposure contributes only when
   requested controls succeeded and auto exposure is off. uvcvideo cameras'
   SOE-labelled stamps are host arrival times and get no exposure correction.
   Do not subtract processing delay already excluded by a hardware timestamp.
   Rebuild/redeploy and repeat the motion check: timestamp alignment should
   improve, and status `last_latency_us` now includes the correction. Leave
   `sim_timestamp_lag_us = 0` on the robot; it is a sim-only injection, ignored
   live, not a calibration setting.
7. Set positive `camera.exposure_ms` / `camera.gain` for manual capture (or
   explicitly set `auto_exposure` / `auto_gain`). On Linux a control the
   camera lacks, refuses or cannot hold fails the open, naming it in the
   node's log; `v4l2-ctl -d <device> -L` lists what it offers. Check with
   `v4l2-ctl -d <device> -C auto_exposure,exposure_time_absolute,gain` while
   the node runs. On macOS confirm status `controls_applied = true` and
   `control_failure_count = 0`; AVFoundation does not expose custom
   duration/ISO, so manual requests report failure there. Verify with a
   blink/motion test rather than assuming a configured value reached hardware.
8. Test a USB stall/unplug while disabled. With a connected source,
   `frame_timeout_us` of silence (default 500,000 µs) increments `stall_count`
   and closes/reopens the source. A detected disconnect enters recovery
   directly, without necessarily counting as a stall. Retries use 100 ms–1 s
   exponential backoff, and each connection state change logs one line with its
   reason; `reconnect_count` counts successful recovery, not failed attempts. Confirm frames resume without restarting the node. These
   recovery/control fields are excluded from the robot's radio telemetry.

The tool uses OpenCV 4.12's ChArUco detector: the BCR module exposes both
`@opencv//:objdetect` and `@opencv//:calib3d`. Calibration uses the five-coefficient
Brown–Conrady model (no rational/thin-prism extensions).


## Vision frame recording and replay

With `recording.enabled = true` (the default) and a session directory, cameras
record available frames as 8-bit grayscale PNGs with detection-context JSON
sidecars in `<session>/frames/<camera>/`. Capture must be configured and recorder
initialization must succeed. Recording is rate-limited by `recording.sample_hz`
and `recording.rejected_hz` and capped by `recording.max_bytes` (oldest pairs
deleted first). Fetch a session with `./tools/deploy.sh fetch` as usual; the
frames come along. Re-run the detector and localization offline over a camera's
frames with:

```sh
bazel run //talOS/vision/recording:frame_replay -- \
  --frames <session>/frames/<camera> \
  --layout "$PWD/talOS/vision/testdata/2026-rebuilt-welded.json"
```

from a talOS checkout's root, with an absolute `<session>` (`bazel run`
runs the tool in its runfiles); the layout is the 2026 field's.

The tool prints valid observations and per-reason rejection counts before/after.
Position/yaw RMS uses only frames carrying truth and valid replay observations;
physical captures have no truth. With no eligible samples, printed zero RMS
does not indicate measured zero error.
Override recorded detector settings with `--decimate`, `--threads`, `--sigma`,
`--refine-edges`, `--hamming` and `--min-decision-margin`. Backend selection
defaults to `auto`, not the recorded backend; choose it with `--backend`.
Pass `--max-range-m` to match a non-default `camera.max_range_m`.

## Uncommissioned, then disabled

- [ ] With the uncommissioned bundle running and DS disabled, watch the RoboRIO console and Studio. Expected: RIO logs `commissioned=0`, `/hw/status` `commissioned` reads false, and every actuator stays neutral. This deliberate refusal is not a CAN fault to bypass.
- [ ] Inspect `/hw/status` (Studio channels use `hw/status/*`; field definitions are in `talOS/bridge/bridge_status.fbs`). Expected: `states_received_count` advances, `state_age_us` stays low, and `identity_rejected_count` / `ownership_rejected_count` do not grow. Before the first frame, `state_age_us = 0` alone does not prove connectivity.

## Human commissioning

- [ ] In Phoenix Tuner X, compare every physical CAN id against the robot's subsystem files (every `[motors.*]`, CANcoder, gyro and other CAN device). Expected: each device resolves to the intended mechanism; a human corrects any mismatch.
- [ ] Check each motor and encoder `inverted`; mechanically align each mechanism an absolute encoder zeroes and measure each CANcoder's `offset_rot` against Phoenix Tuner X. Expected: positive demands and measured directions agree, with measured offsets rather than unchecked zero defaults.
- [ ] Tune each closed-loop motor's `slot0` before demanding velocity or position from it, and validate its `sensor_to_mechanism_ratio`. Expected: no closed-loop motor has all-zero gains; feedforward is in mechanism units (V per mechanism rps, the motor's V per rotor rps times the ratio; confirm against a measured free-speed run).
- [ ] Measure and replace every provisional limit the robot's configs mark as derived rather than measured (accelerations, current limits). Expected: full-demand steps and reversals stay within traction and out of sustained current limiting.
- [ ] Only after those checks, have a human set `commissioned = true` and update the config's golden test, if the robot has one (the bench's `bench/main_processor/configuration/config_golden_test.cc`), in the same change. A commissioned config also requires [target coverage](../ARCHITECTURE.md#targets), so its launcher `--describe-only` must still pass. Repeat Mini build/test, bundle build and install above. Expected: RIO accepts the new configuration; the `/hw/state` configured flag, which requires `commissioned`, becomes true, and so does `/hw/status` `commissioned`. `/hw/status.configured` was already true: it only reports the RoboRIO's acknowledgement.
- [ ] Keep DS disabled while exercising sticks and mechanism buttons. Expected: Studio shows the Driver Station disabled and no selected actuation source, and all actuators remain neutral. Then enable with controls released. Expected: no motion without a live target.

## Mechanisms, one at a time

- [ ] Lift or secure each mechanism in turn, with a spotter at the disable control; for a mechanism that ejects or launches, clear its discharge path before enabling. Enable teleop with controls released, exercise one binding at a time with small demands, then release and disable. Expected: each mechanism moves in the commanded direction, its sensors change as implemented, release stops the demand and disable always neutralizes it (a flywheel's coast-down is not continued actuation). Observe each in Studio.
- [ ] With the robot disabled, check the autonomous routine selection the robot provides, then run each routine with mechanisms safely restrained. Expected: the selected routine runs; disable, a Driver Station outage or a stale input releases its lanes.
- [ ] For each producer that performs one-shot actions (an autonomous routine's shot or drive), kill it in the middle of an action while enabled; the launcher restarts it. Pass condition: no completed action repeats. Expected: the restarted producer resumes or ends the interrupted action, never redoes one it completed.

## Link timing and fault injection

- [ ] Collect representative enabled sessions with `./tools/deploy.sh fetch /media/usb/robot-sessions` on the Mini (add `--prefix` for a custom installation), using a fresh destination to avoid merging existing sessions. Expected: each complete session folder, its manifest and all restarted-node log attempts are copied. Stop the stack before the final fetch for a stable completed recording. Transfer the folders to the analysis machine and run `bazel run //talOS/launcher:session_report -- --strict <session-dir>` (replace the placeholder with a copied session directory). Preserve the Markdown report, raw `.tlog` files and config hash; add `--json` for structured results. `--strict` exits 2 unless every required verdict passes and the session finished with no quarantined, recorder-failed or log-capped recording node, no damaged log and no node whose "Delivery loss" shows a `wake_failures`, `requests_skipped` or `wake_fallbacks` above 0; the "Launcher output" section names any output whose bytes `talos.log` says were dropped ([semantics](../talOS/launcher/README.md#session-measurements)); NOT MEASURED is not acceptance.
- [ ] Review the report's `/hw/state` inter-arrival statistics. Pass: median about **5 ms**, **p99 under 10 ms**, and **no gap of 100 ms while enabled**. The tool interprets “about” as 4–6 ms and labels this analysis tolerance explicitly. It selects bridge IPC publication when available, otherwise one consumer dispatch stream, never pools duplicated reader observations, and uses the logged Driver Station enabled bit.
- [ ] Review per-consumer target receipt ages where MESSAGE dispatches exist. Pass: target age at each tick is **about 25 ms at most**, **never `command_timeout_us`**. The report's "target age at tick" verdict measures this from the READ record `Target::get` logs for each selected lane (worst `process:lane` named); an age at or past `command_timeout_us` is never selected, so a stalled producer shows as ages rising toward the timeout. Receipt age is not tick age. Freshness starts at the writer's publication timestamp; draining an expired message cannot revive it. Include paused-consumer cases when measuring age.
- [ ] Check every hardware-sample timestamp a node publishes (odometry's) against `HardwareLink::sample_time()`, not tick time or raw RIO uptime. Expected: it uses mapped hardware sample time; unconverged/stale mapping clears validity. Independently measure mapping error and CAN acquisition ages. The two-way uncertainty must contain observed mapping error; validate the <=1000 ppm relative oscillator assumption and compare the bound to fusion requirements (no numeric fusion threshold is specified).
- [ ] Review per-node/per-timer lateness (`now - deadline`) in the session report; use Studio's system view on the Mini for live inspection. Expected: the Mach time-constraint policy is accepted for the bridge's and every control node's period, observers take `QOS_CLASS_USER_INTERACTIVE`, and no node logs `realtime scheduling incomplete` (the session report's realtime section lists each loop's grant; `--strict` fails a bridge or control loop without its full grant). Record measured lateness; no numeric lateness threshold has been specified. Missing TIMER dispatches are NOT MEASURED; the bridge has none and is reported by its request-to-command delay instead. `TALOS_REALTIME=0` disables the realtime request and is not the acceptance configuration.
- [ ] Inspect `/hw/status` alongside the timing trace. Expected: `configured` stays true, counters advance, rejections stay flat. Record `state_age_us`, `timed_out_mask`, `subsystem_count` and the [clock mapping diagnostics](../talOS/hardware/README.md#runtime-contract). The robot's radio telemetry excludes `clock_offset_uncertainty_us` and `clock_min_round_trip_us`; inspect them in local logs.
- [ ] On blocks, use the PIDs in Studio to send `SIGSTOP` then `SIGCONT` to each operator-input producer in turn (`teleop`, and any node of the robot's own that writes a teleop lane), with measured pauses of **50 ms** and **200 ms**. Pass for **200 ms**: each subsystem goes neutral within **`command_timeout_us` plus a tick**, timing out its own lane. Record the 50 ms behavior without inventing an extra pass threshold; disable between trials and verify safe recovery before re-enabling.
- [ ] Pause an actuator-owning node with `SIGSTOP` for **80 ms**, then resume with `SIGCONT`. Expected: record hardware response, target receipt/tick timing and recovery; no separate numeric pass threshold is specified for this pause. Keep the spotter ready to disable.
- [ ] With a mechanism running on blocks, unplug RIO Ethernet. Expected: actuation goes neutral on link loss, stale hardware samples become invalid, and odometry, if the robot has one, does not run away; `state_age_us` grows and timeout diagnostics identify the affected owners. Disable before reconnecting and verify recovery without unintended motion.
- [ ] Disabled, stop the stack on the Mini, then start it again. Expected: within a second of the stop the RoboRIO's `companion/connected` turns false and the Driver Station log shows one `companion lost` error; on restart `companion/connected` returns true with one `companion connected` warning, and no repeated messages while the link holds ([RoboRIO link report](../talOS/hardware/README.md#roborio)). The driver dashboard stops with the stack, so read `companion/connected` as [main processor not reachable](PRE_MATCH.md#main-processor-not-reachable) says.
- [ ] On blocks, repeat the [RoboRIO loop stall check](BENCH.md#fault-injection-and-timing) with the robot's devices, a mechanism driven slowly. Expected: every motor goes neutral, every DIO output false and every PWM output disabled within one loop period plus 5 ms of the command's expiry, with one `talos loop watchdog` Driver Station error per stall; record the full thread priority listing, what the Driver Station showed, and whether it disabled the robot. Disable and release before any re-enable.
- [ ] Measure and record stick-to-hardware-request median on the real link; no real-link pass threshold is specified here.

## Boot operation

- [ ] Disable, stop the manually launched stack, then run `./tools/deploy.sh enable-boot --user <mini account>`. Expected: `/Library/LaunchDaemons/org.talos.robot.plist` is loaded (`sudo launchctl print system/org.talos.robot`), and after a reboot with nobody logged in a single installed stack starts with no command-line addressing override. Recheck configuration, logs and disabled neutrality after restart. If no new session appears, `./tools/deploy.sh logs` prints the failed launch's errors from `/Library/Logs/talos/launchd.stderr.log` ([boot service output](DEPLOYMENT.md#boot-service-output)). For a regression, `./tools/deploy.sh rollback` selects the previous whole bundle; stop/restart the stack and recheck its config hash before use.
- [ ] Camera permission under the daemon: macOS asks for camera access interactively, which cannot appear with nobody logged in, and a privacy (PPPC) configuration profile can only deny camera access, not grant it. With the boot service running and nobody logged in, check every camera node's frames arrive (Studio, or the session's camera captures). Expected: frames from every camera. If capture is denied, record the node's error and the TCC entry (`log show --predicate 'subsystem == "com.apple.TCC"' --last 10m`) and choose a fix with TODO.md DEPLOY-1 (for example, granting the service account's binaries once from a logged-in session, then checking that a reinstalled bundle, whose binaries have a new path, keeps the grant); do not field cameras that need a login.
- [ ] Power the robot from cold several times (RoboRIO and Mini together, as at a match) and note on the Driver Station clock when robot code turns green and when the driver dashboard's Hardware configured check turns `OK` (the RoboRIO has then heard the Mini and acknowledged its configuration; [PRE_MATCH.md](PRE_MATCH.md#driver-dashboard-top-to-bottom)). Expected: the Mini connects within the RoboRIO reporter's 30 s startup grace (`kStartupGraceUs`, `CompanionLink.h`) of robot code start, so no `companion not heard` error appears; record the times. If it routinely takes longer, the error fires on every cold boot: speed up the Mini's boot or raise the grace.
