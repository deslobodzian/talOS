# TODO

What's left needs the robot, a human step, or a decision. `[read]` marks a finding from reading the code that nobody has reproduced or measured yet. The bring-up procedure is [docs/BRINGUP.md](docs/BRINGUP.md).

## Design review roadmap (2026-11-07, branch `hardening/match-readiness`)

The design review rated the architecture 6/10: the transport fail-safes are
sound, but "fresh", "restart is safe", "subsystem safety is local" and "pose
valid" promised more than the code guaranteed. Each fix below is one commit on
the branch; `git log` carries the Problem/Fix/Why. Deferred items keep the
reason they were deferred so the next pass can pick them up.

Re-rated 7.5/10 after the readiness pass. Every match-losing gap the audit found in software is fixed and tested in simulation: one CAN device failing configure or resetting now costs only its mechanism, the drivetrain drives on three modules, odometry survives device reboots, optional nodes cannot block boot, Red-alliance driving is correct, autonomous is selectable at the field, feeding is interlocked to a ready shooter. Verified: default `bazel test //...` 139/139, `--config=asan` and `--config=tsan` 122/122 each, `./gradlew phoenixTest buildRio`, describe-only on all three 2026 configs. What keeps it below 9 is unmeasured hardware: CAN bus load, CAN ids/offsets, real-link timing and fault injection, camera calibration ("On the robot" below).

Landed in the competition-readiness pass (2026-11-07; `git log` has each Problem/Fix/Why):

- [x] **Camera timestamp kind** (`2c4a32e`): SOE/EOF/delivery kinds with one midpoint conversion, live exposure/gain controls, silent-camera reconnect with backoff. AVFoundation path compiled on macOS only, unbuilt on Linux.
- [x] **Autonomous routine selection** (`745e5b5`): `preload`, `none`, `shoot_only`, `taxi` as action sequences; joystick 0 POV while disabled (incl. FMS pre-match disabled-auto), latched at arm, persisted per session. Loaded trajectories remain open.
- [x] **Feed permit** (`cade9f4`): the intake refuses every negative (feed) demand without a fresh, at-speed shooter; teleop feed on button 3. Robot-level, not yet a framework concept.
- [x] **Registry crash windows** (`495601e`): flock-serialized init/claim; dead initializer or claimant recovered.
- [x] **Simulated transport fidelity** (`73c5c88`): RTMS-sized rings and drop accounting; `Sim::pause/resume` (C++, C ABI, Python). Dispatch is still serial.
- [x] **Studio trust** (`d06ba4a`): per-sample staleness, worker failure surfaced, bounded updates, fault set vs clear.
- [x] **Driver frame** (`e770ae2`): driver-relative heading on both alliances (Red flip removed), robot-relative fallback after neutral on heading loss.
- [x] **Gateway device isolation** (`5514a3e`): asynchronous configure that may leave individual devices unavailable (they and their dependents neutral, retried with backoff); per-device reset restore; bridge push backoff; RIO socket-open retry. Phoenix keeps MotorVoltage/StatorCurrent at 200 Hz (shared status frames); the 5 ms neutral refresh stays.
- [x] **Drivetrain degrade** (`6831392`): module live only with drive, steer and CANcoder; drives on any well-conditioned subset of 2+ live modules; reset-continuous wheel distance and yaw; pose estimator rejects implausible odometry steps.
- [x] **Shooter readiness hysteresis** (`c67a061`): entry band for N ticks, wider exit band, against the clamped demand; clears on target change, release, disable, disconnect. Bands need tuning on the real flywheel.
- [x] **Launcher boot resilience** (`4f2cbb1`): `[subsystem] critical = false` nodes (cameras, pose_estimator, telemetry) cannot block boot and may not own actuators or write target lanes; critical nodes and the bridge are never quarantined (backoff capped at 1 s).
- [x] **Sanitizers and decoder robustness in CI** (`bf30d9d`, `99dcff1`): ASan+UBSan and TSan configs and CI jobs; malformed-input tests for frame, State/Command/config, Driver Station and TOML decoders. Found and fixed: toml++ 3.4.0 abort on `[#x]`-style headers and UB on non-ASCII comments (our own `vision.toml` hit it every parse; patched), and a data race on the event loop's dispatch counter read by the registry reporter. Python-host tests are excluded under sanitizers (ctypes cannot load the runtime).

Deferred (not started; in priority order):

- [ ] **CAN bus load** (audit, needs the robot): all 15 devices on the RIO bus at 200 Hz status plus per-tick control frames; measure `can_utilization_frac` before commissioning.
- [ ] **Per-signal acquisition time.** One `sample_time` per `/hw/state` covers CAN signals cached at different instants (`PhoenixBackend.cpp` Read); odometry treats sensor skew as motion. Needs Phoenix per-signal timestamps carried through the devices API. Large; touches the hardware wire format.
- [ ] **Clock-offset accuracy.** `talOS/bridge/clock_offset` is one-way: a constant transport delay looks like perfect convergence. Needs a bidirectional exchange or exported timestamp uncertainty propagated into fusion.
- [ ] **One config resolver.** `config_parser.h` and `tools/sync_manifest.py` are two interpreters kept equal by fixtures. Make the C++ resolver emit canonical JSON + dependency closure and have sync_manifest consume it.
- [ ] **Replay modes.** Label exact dispatch replay vs counterfactual input replay; add a consumer-receipt/drop trace mode; replay adapters so an appended enum value does not make every earlier log un-replayable (schema hash covers enum labels).
- [ ] **Telemetry beyond one frame.** The 1200 B single value frame is now the limiting budget (2026 graph needed exclusions). Schedule channels across bounded frames by priority/rate.
- [ ] **Chassis intent coordinator.** Priority lanes hand over whole commands; "driver translates while align rotates" and heading-hold handoff cannot be expressed.
- [ ] **Autonomous trajectories.** Routines are action sequences with one waypoint each; loaded trajectory files and more routines remain.
- [ ] **Coupled-action permits as a framework concept.** The intake enforces the shooter-ready feed permit itself; generalize when a second coupled mechanism exists.
- [ ] **Neutral requested vs achieved.** `Backend::Neutral()` returns nothing and Phoenix discards neutral-send status; expose delivery uncertainty and rely on documented controller-side command expiry.
- [ ] **Python on actuator loops.** Intake is Python; a GC pause > 2 periods trips the bridge watchdog. Measure jitter on the coprocessor or move actuator loops to C++; `docs/METHODOLOGY.md` must not claim "memory, not jitter".

## On the robot

Human-only steps (`.omp/RULES.md`: agents never set `commissioned` or change CAN ids):

- [x] `controller_address`: `robot_bundle` is built from `configuration/robot_field.toml` (an `include` overlay setting `10.56.87.2`); `robot.toml` stays loopback for sim. `bundle_test` fails if the bundled config does not resolve to `10.56.87.2`.
- [ ] Give the Mac Mini the static address `10.56.87.11`. It is compiled into the RIO program (`kCompanionAddress` in `2026-robot/controller_processor/rio/src/main/cpp/Robot.cpp`), which only sends state there.
- [ ] Check the CAN ids in `subsystems/*.toml` against Phoenix Tuner X; measure the four CANcoder `offset_rot` (all `0.0`); check each `inverted`; verify and tune the sim-consistent intake roller `slot0` (`p = 0.1`, `v = 0.12`) and drivetrain defaults (`slot0.v = 0.81`, `max_velocity_rps = 14.5`, `cruise_velocity_rps = 14.5`) on the real robot. Only then set `commissioned = true`, and update `config_golden_test` in the same change.
- [ ] Build and test on the Mini first (`bazel test //...`), then build `robot_bundle` there (there is no cross-compile config), `tools/deploy.sh install`, `enable-boot --user <account>`, and check the drivetrain, shooter and intake in the order in docs/BRINGUP.md. `enable-boot` installs a LaunchDaemon (`/Library/LaunchDaemons/org.talos.robot.plist`) with sudo; only its `--dry-run` has run.
- [ ] Calibrate each camera's intrinsics and Brown-Conrady distortion with `//talOS/vision/calibrate:calibrate_camera` (see docs/BRINGUP.md), measure its robot-frame mount, and set `device` in each camera subsystem file.
- [ ] On the robot, verify that live capture starts at the first camera update and each `/<camera>/status` stays healthy; the checked-in calibrations and mounts are not measured robot calibration.
- [ ] Confirm two timing changes behave on the real link (sim cannot: the node and the RIO share one clock and the link has no latency):
  - target freshness is measured from when the node receives the target (`Subscription::fresh()`, `talOS/node/node.cc`), not from a sender timestamp (`issued_ns` has been removed);
  - `DrivetrainState.timestamp_ns` is node time, not RoboRIO sample time (only odometry reads it, as its integration `dt`).
  - Measure from the launcher's `.tlog`s (`bazel run //talOS/launcher:log_dump -- <dir>`): `/hw/state` inter-arrival (median about 5 ms, p99 under 10 ms, no gap of 100 ms while enabled); target age at each drivetrain tick (about 25 ms at most, never `command_timeout_us`); loop lateness in Studio's system view.
  - Then inject faults: `SIGSTOP` the `driver` node (chassis) and the `teleop` node (mechanism buttons) for 50 ms and 200 ms (200 ms must go neutral within `command_timeout_us` plus a tick; each subsystem times out its own lane), `SIGSTOP` the drivetrain for 80 ms, unplug the RIO Ethernet while the wheels spin. Also measure stick → drive request median on the real link (sim: 2.99 ms; old arbiter baseline 4.98 ms).
- [ ] Measure loop lateness on the Mini. Node binaries and the bridge request realtime scheduling (`talos::process::ConfigureRealtime`: Mach time-constraint policy on macOS, `SCHED_FIFO` + `mlockall` on Linux; `TALOS_REALTIME=0` disables) and nodes without message callbacks drain only before their own timers; neither has been measured on the robot. Check that the Mach policy is accepted for every node period, also when started by launchd, and that no node prints the "realtime scheduling incomplete" line.
- [ ] Check the Tier 0 hardening on the real RIO and CAN bus (all proven in tests and sim only):
  - the persisted config at `/home/lvuser/talos_config.bin` restores at boot before enable, and a RIO program restart while enabled resumes driving;
  - the async device-reset restore and its retry with backoff after pulling and replugging a Talon's CAN while enabled;
  - `battery_v`, `brownout`, `can_utilization_frac` and `motor_fault_mask` in `/hw/status` against the DS and Phoenix Tuner, and CAN utilization after `OptimizeBusUtilization`;
  - PWM neutral now disables the pulse, so check what each PWM device actually does when disabled.

## Merge `vision`

- [x] PR #2 `vision` → `main` (https://github.com/deslobodzian/talOS/pull/2): CI green at `8064165` (Linux after one rerun, see the `talos` package entry below). CI on x86-64 Linux found what the arm64 container could not: gcc rejecting out-of-order designated initializers, libjpeg-turbo's SIMD table missing from Bazel's test shared objects (`build:linux --@@libjpeg_turbo+//:noasm=yes`), `SO3::Log` losing accuracy near pi, and an x86-only unused NEON helper. All fixed on the branch. Not merged.
- [ ] `PhoenixBackendPigeon.MountYawConfiguredAndInversionFlipsReadYaw` (GradleRIO `phoenixTest`) failed once in CI with "CAN frame not received/too-stale" on a Pigeon config refresh, then passed on the next run with no RIO change: a Phoenix sim timing flake. If it recurs, wait for the device before refreshing.
- [ ] `phoenixTest` on macOS still needs the `MacOSX26.sdk` GradleRIO wants; it passes on Linux.

## Sim vision slices

Slices 1-6 (sim truth world, accuracy scenario, typed detector settings, estimate quality, latency offset, frame recording) are committed on `vision` (PR #2).

- [x] Slice 6, frame recording: `Node::session_dir()` (C++, C ABI `talos_node_session_dir`, Python `node.session_dir`, `Sim::set_session_dir`); `talOS/vision/recording` writes `<session>/frames/<camera>/<capture_ns:020>_<reason>.{png,json}` (1 Hz sampled + ≤1 Hz rejected, 256 MiB per camera, oldest pairs deleted); `//talOS/vision/recording:frame_replay` re-runs detection/localization offline; Studio's Frames tab opens a recorded folder offline. Sim PNGs measured ~200-260 KB each (15 s session: 15 pairs per camera, 2.9-3.9 MB).
- [x] Slice 7 (report only, no code change): CPU vs Metal `Execute` on 360 realistic 1280×720 frames (300 rendered noisy/moving + 60 recorded), Apple M3 Max laptop (not the Mini). Robot settings (decimate 2, threads 2), `-c opt`: CPU mean/median/p95 3.48/1.65/10.6 ms, Metal 3.72/2.20/10.2 ms; fastbuild CPU 3.55/1.69/10.9, Metal 4.59/2.81/12.9. Four concurrent 60 fps processes: both sustain 60 fps (p95 CPU 11.6-11.9 ms, Metal 12.1-12.5 ms). Decimate 1/threads 1: Metal ~10% faster (mean 21.3 vs 23.6 ms). Detection parity exact (same tag ids on all frames; corner RMS mean 0.0007 px). Metal construction + first frame ~31 ms. Decision pending: keep, delete, or switch the camera default to `backend = "cpu"`; rerun on the Mini first.
- [x] Multi-tag fix (found via Studio + frame recorder): the 0.2 ambiguity limit now applies only to single-tag solves; multi-tag seeds from the candidate with the lowest joint reprojection residual. Replay of a recorded rear_right session: 9 → 894 valid of 894. Observation covariance now includes camera mount-calibration uncertainty (`camera.mount_angle_std_rad` 0.0175, `camera.mount_translation_std_m` 0.01, isotropic `range × σ_angle` bound). Accuracy `--repeat 3`: fused 0.075 m (0.056-0.095) / 0.652° vs odometry-only 0.101 m / 0.888°; the earlier 0.03-0.05 m came from rejecting the grazing two-tag views, not from better estimates.
- [ ] The isotropic mount term is conservative (rear cameras' normalized x/y RMS ~0.4-0.5 under ±1° injected mount error); a direction-aware projection of mount yaw error (perpendicular to the camera-to-tag ray) would be tighter. Set the two mount std params from the real robot's post-mounting calibration residual.

## Studio revamp (done, uncommitted)

- [x] Architecture: the worker keeps decode/timeline/cursor/RPC; a main-thread columnar store (`studio/src/data/`) feeds hooks; panels are self-contained modules registered in `panels/registry.ts`; workspaces are a persisted split-tree of pages (`layout/`), opened from presets on first launch (Robot, Vision, Mechanisms, Hardware, System, Signals; Spatial and Tools on demand). `studio/README.md` documents the contract, adding a panel, and CSS conventions.
- [x] Panels: Robot (at-a-glance), Subsystems (targets, lanes, demand vs measured), Vision (camera health/rates/disagreement, estimator acceptance), Events (filterable fact log), Field (trails, truth, camera observations with uncertainty, follow), Plot (unit axes, enum steps, event markers), Inspector, Hardware link (power/link/clock/counters/faults), Session (manifest + registry + declared processes), System, Timing, Health, 3D pose, Agent, Frames. Status strip, event/mode-band timeline, command palette (Cmd/Ctrl+K), shortcuts (?).
- [x] Robot-side data added for it: `hw/state/driver_station/alliance` and `/autonomous/state` (`phase`) telemetry channels (physics-sim graph 136 channels / 1176 B of 1200 B), and the bridge's `GET /manifest.json` (launcher passes `--manifest`).
- [ ] Telemetry headroom is now 24 B (3 channels) on the physics-sim graph; per-motor demand/current/fault channels do not fit without a float32 value-frame change or selective excludes.
- [ ] Studio has no live camera image view (frames are offline only) and no command/actuation path by design; a match-replay mode from a session's `.tlog`s through the bridge (`--replay-dir`) is still open (below).

## Found, not fixed

- [ ] **The `talos` Python package intermittently loses its contents.** Locally, `talOS/python/talos/__init__.py` was truncated to 0 bytes twice on 2026-11-06 (once between 16:12 and 16:15, once at 16:48:19) while workers ran tests and tools. On PR #2's Linux CI run at `8064165` every test importing `talos` failed the same way (`module 'talos' has no attribute 'Alliance'` / `'load'` / `'cpp_node'` / `'message_type'`; 12 tests), C++ and Studio tests passed, and a rerun of the same commit passed. The previous commit (`c4a0455`) differs only in C++ tests and passed. Runfiles map `_main/talOS/python/talos/__init__.py` to the source file alone (no second `talos` package). Not reproduced: macOS full suite with the package's `.py` files made read-only (no permission error; the darwin sandbox already blocks test writes), all 35 `py_test`s in an arm64 Ubuntu container, nor anything in the earlier list (bundle tests, startup/accuracy, bridge integration, launcher tests, `//tools/...`, `//examples/...`, `sync_manifest`, `--describe-only`, `./sim.sh`). Next: make the checkout read-only in CI before `bazel test` (`git ls-files -z | xargs -0 chmod a-w`, `MODULE.bazel.lock` writable) so the writer fails with a traceback instead of corrupting the file.
- [ ] rules_js `js_test` needs `patch_node_fs = False` for Studio tests (`studio/BUILD.bazel`): with the default fs patches, tsx cannot match runfiles-resolved sources against `tsconfig.json` and compiles `.tsx` with the classic JSX transform ("React is not defined"). `tests/run.mjs` also passes the tsconfig's real path. Revisit if rules_js or tsx changes.
- [ ] The bundled omp headless browser stopped producing screenshots mid-session (every page, including `data:` URLs, timed out); Studio was verified by driving the installed Google Chrome over CDP instead.
- [x] Resolved: simulated cameras render from the physics gateway's absolute `/sim/state` truth (`sim_gateway --physics`), so a restarted camera follows the true pose; `SimTruth` is deleted. Under WPILib sim no truth exists and cameras publish invalid observations.
- [x] Resolved: `sim_gateway` registers in the live registry (`NodeRegistration`), so Studio's system view lists it running. Verified live with `./sim.sh --sim-gateway`.
- [x] Resolved: channel-level telemetry excludes drop three cumulative counters per camera. Measured over the real 2026 graph with `MeasureValueFrameBytes`: 139 channels / 1200 B before, 127 channels / 1104 B after. Studio's alliance and autonomous-phase channels since brought it to 129 channels / 1120 B (80 B headroom; `robot_physics_sim.toml` 136 / 1176 B).
- [x] Resolved: `sim_gateway` emits Driver Station frames at the RoboRIO's ≤50 Hz rate.
- [x] Resolved: the 2026 autonomous target moved in-field to (6.0, 4.0, 0.0); it previously drove from the default start pose (2, 2, 0) to the field's y = 0 edge. The accuracy scenario (`//2026-robot/main_processor/simulation:accuracy_scenario`) uses this routine.
- [x] Resolved: `@apriltag` is patched (`third_party/apriltag_pose_ambiguity.patch`) to return the ambiguity search's new-minima count and drop its stderr print; multiple minima without a second pose now map to ambiguity 1 (unknown), a unique solution to 0.
- [ ] Single-tag observations still show rare gross errors below the 0.2 ambiguity limit (rear-left camera, mount-error accuracy runs: normalized yaw RMS 2.3/1.7 against p95 0.60/0.58). The estimator's single-tag yaw (15°) and position (3 m) innovation gates keep them out of the fused pose; rejection reasons are not logged, so their frequency per cause is unknown.
- [ ] With the moderate preset's ±1° per-camera mount bias, fused heading in the 8 s accuracy CI session (0.656° RMS) was slightly worse than odometry-only (0.625°); the 12 s scenario averages 0.692° fused against 0.889° odometry-only. Real mount calibration bounds how much vision can improve heading.
- [ ] A simulated camera was SIGKILLed by the launcher at startup ("heartbeat missing": no ACTIVE registry row within the 5 s startup grace) during a back-to-back accuracy-scenario run; its first log was 0 bytes and the restart succeeded. The node runtime creates its Reporter only after `Vision` construction (`talOS/node/node.cc:~1119-1125`), and `backend = "auto"` initializes Metal shaders/pipelines in that constructor. Cause unproven (Metal init vs machine load); slice 7 (Metal benchmark) and registering before heavy construction are candidate fixes.
- [ ] [read] Protocol `AckRequested` / `Ack` / `Fragment` flags are declared but unused by runtime send paths; delete them or use them explicitly (`talOS/protocol/types.h`, `talOS/hardware/endpoint.h`, `talOS/bridge/node.cc`).

## Fixed on 2026-11-05

- [x] `deploy.sh install` used `cp -r`, which on macOS dereferences the bundle's relative symlinks: the venv `python3` could not load `libpython3.11.dylib`, so every Python node (and the nodes whose graph needs them) failed to describe in an installed bundle. Now `cp -RP` on the resolved bundle path; `//2026-robot/main_processor:bundle_test` installs with `deploy.sh` and fails on the old copy.
- [x] `deploy.sh enable-boot` on macOS wrote a per-user LaunchAgent (starts only after a login, ignored `--user`). It now installs a LaunchDaemon running as `--user`.
- [x] `tools/setup.sh` was committed without its executable bit (`./tools/setup.sh` failed); it missed a WPILib-installer roborio toolchain (`~/wpilib/2026/roborio`); it warned falsely when `/usr/local/sbin` is merged into `/usr/local/bin` (Fedora).
- [x] `DrivetrainTest.UpdateDoesNotAllocate` failed on macOS once `SimulatedChannel::publish` was inlined into `SimulatedEventLoop::send_impl`; the sim-transport filter matches both frames.
- [x] Earlier: odometry runaway on a quiet link, `deploy.sh logs` session search, bundle-relative paths, D1/D5 acceptance, `talos_graph_test` in CI, selected target source in Studio.

## Tier 0 hardening (2026-11-05)

- [x] A RIO program restart while enabled no longer leaves the robot dead: the gateway persists the last applied config (`talos_config.bin`, CRC-checked) and restores it at startup before reporting ready, and an identical push succeeds while enabled. The config assembler no longer re-acknowledges every chunk after completion.
- [x] One out-of-range demand no longer stops the robot: the devices layer clamps each motor to its configured limits (`Motor::clamped_count()` in C++, C ABI and Python), and the gateway neutralizes only the offending motor (`neutralized_motor_count`) instead of rejecting the whole command.
- [x] Apply failures are counted, neutralized and retried. Only 3 consecutive failures latch, and re-enabling or any successful reconfigure clears the latch. Device-reset restores run off the 5 ms loop and retry with backoff (250 ms doubling to 2 s), so a failed restore no longer needs a disable.
- [x] `State` carries battery voltage, brownout, CAN utilization, per-motor fault flags and the gateway counters. `/hw/status` publishes them, so Studio shows them as `hw/status/*`. Protocol version 2; the payload is now 1432 bytes, so a frame is 1472 bytes and fits a 1500-byte MTU.
- [x] Logs survive: every launch writes `<output-dir>/<session_id>/`; old sessions are pruned (`[robot] log_keep_sessions` / `log_max_bytes`); `.tlog`s are fsynced about every second; `manifest.json` is written at start and updated atomically. `hardware_node --log` records every command sent and state received. `deploy.sh logs` lists sessions and `deploy.sh fetch` copies them.
- [x] Commissioning: only a loopback remote forces `commissioned`; `--sim` with a non-loopback remote is refused by the launcher and the bridge.
- [x] The launcher restarts nodes whose registry heartbeat or dispatch progress stalls, and `max_restarts` now applies per rolling window instead of per session. A framework node that never dispatches anything counts as stalled after its startup grace period.
- [x] DIO and PWM are gated on `robot_enabled` in the nodes. Outputs a subsystem omits from its request go neutral in the bridge merge. PWM neutral disables the pulse rather than sending a centre pulse.
- [x] RTMS enforces a single writer at runtime (dead writers are reclaimable). Reader-slot exhaustion is an explicit error, and the graph lint rejects a topic with more than 8 declared readers.
- [x] Tests no longer share host-global shared-memory names: `ShmObjectName` hashes `TALOS_SHM_NAMESPACE` / `TEST_TMPDIR` into `/~<16 hex>` under `bazel test`. The `exclusive` tags are removed from `continuity_test` and `replay_inputs_test`, and `hardware_node_test.cc` uses OS-assigned ephemeral UDP ports.

## Control, simulation & autonomous (2026-11-05)

- [x] Per-motor feedback faults are isolated in `Gateway::Accept`, `PhoenixBackend::Read` and `PhysicsSimBackend::Read`: a disconnected/stale motor or remote CANcoder neutralizes only that motor; whole-robot unhealthy triggers only on total CAN bus loss.
- [x] Swerve kinematics uses `talos::geometry::SE2::Discretize` (the desired finite-period pose's logarithm divided by the period) and scales drive speed by the cosine of steer error (`2026-robot/main_processor/drivetrain/swerve.h`).
- [x] Driver controls use a circular deadband and radial speed cap, a Red-alliance field-frame flip, rising-edge gyro re-zero (`rezero_button_id`) and proportional heading hold (`heading_hold_radps`) in `2026-robot/main_processor/driver/driver.h`.
- [x] Sim-consistent config defaults: drivetrain drive `slot0.v = 0.81`, `max_velocity_rps = 14.5` and `cruise_velocity_rps = 14.5` in `drivetrain.toml`; intake roller `slot0` uses `p = 0.1`, `v = 0.12` in `intake.toml`. Real-robot verification/tuning remains required before commissioning.
- [x] `PhysicsSimBackend` implements MotionMagic trapezoidal profiling and automatic swerve-to-Pigeon2 rotational coupling (`talOS/hardware/sim_physics.h`).
- [x] `//2026-robot/main_processor/autonomous:node` writes `/drivetrain/target/auto`, `/shooter/target/auto` and `/intake/target/auto`: preload spinup → feed → alliance-aware waypoint + intake → hold. All six missing-auto graph warnings on `2026-robot` are eliminated.

## 2027 platform / Wave 3 (2026-11-05)

- [x] Hardware supports follower motors and `gravity_type`, with hardware sanity lints for invalid or physically inconsistent configurations.
- [x] `HardwareLink.fresh()` / age are exposed through C ABI v17 and Python; `//talOS/python:api_parity_test` checks API parity.
- [x] Teleop bindings support declarative `pov` inputs and `on_toggle` actions.
- [x] `pose_estimator` supports initial pose seeding and vision `tag_count`, field-bounds and innovation outlier gating; reusable `Trajectory` and `HolonomicController` support autonomous motion.
- [x] `sim_gateway` supports scriptable `--mode-schedule` and `--alliance`.
- [x] `examples/external_subsystem/bundle_test.py` now installs through `deploy.sh`, exercising the real deployment path rather than relocating with `copytree`.
- [x] CI includes a `macos-latest` job, and documentation drift is corrected for the current hardware, simulation, control and deployment APIs.

## Full-stack validation & runtime control / Wave 4 (2026-11-05)

- [x] `//2026-robot/main_processor/simulation:startup_test` covers the full nine-node closed loop across disabled → auto → teleop → disabled and `SIGSTOP`/`SIGCONT` fault injection. `.gitignore` no longer ignores `MODULE.bazel.lock`.
- [x] Pigeon2 `offset_rot` (`MountPoseYaw`) and `inverted` are supported across `config.cc`, `PhoenixBackend`, `PhysicsSimBackend` and `SimBackend`. Physics simulation preserves rotor positions across resets; runtime `Motor::rezero` and `Gyro::rezero` are exposed in C++, C ABI v18 and Python.
- [x] Drivetrain translational and rotational acceleration limits are configurable with `max_linear_accel_mps2` and `max_angular_accel_radps2`, with immediate neutral/reset on disable or target release.
- [x] `//talOS/launcher:log_dump` supports `--node`, `--topic`, `--since-ms` and `--until-ms` filtering and `--format text|jsonl|csv` export, covered by `//talOS/launcher:log_dump_test`.

## Decisions taken

- [x] Odometry when the RIO link drops: hold. `Drivetrain` clears `modules_valid`/`yaw_valid`, zeroes speeds and keeps the last module positions while samples are stale.
- [x] Freshness counts from the writer's publication timestamp in RTMS v5 / `.tlog` v6 for C++ and Python nodes alike (`ARCHITECTURE.md`, "Targets"), while plain subscriptions read latest-value and callback subscriptions read sequence.
- [x] `/hw/state` stays out of telemetry, silently (`/hw/status` covers link health).
- [x] Python subsystems can run the framework teleop node in a `talos.testing` Sim (`talos.teleop.Teleop`, via `TALOS_EXPORT_NODE`).
- [x] The intake runs at the asked `roller_velocity_rps` (stops on the beam unless ejecting).
- [x] The headless agent server stays, on port 5804 (was 5802, the gateway's number).
- [x] `.gitattributes`: `* text=auto eol=lf`.
- [x] A real second repo: `examples/wrist_repo` (`@wrist`, dev dependency + `local_path_override`) consumed and bundled by `examples/external_subsystem`; its tests run from `//...` through `:wrist_tests`.
- [x] Earlier: Studio slimmed in place, teleop split into `driver` + `[[teleop.bindings]]`, `include` overlays.
- [x] Initial waypoint/preload autonomous now publishes all three robot `auto` lanes; the robot no longer idles because those producers are missing. Multi-routine selection and trajectory files remain future work.

## Open decisions

- [ ] Studio post-match replay (`studio_bridge --replay-dir <session>`) and a topic inspector/publisher, which would replace the per-subsystem `monitor`/`send_target` binaries. Until decided, shooter and intake have no monitor (only the drivetrain has `monitor` and `send_target`, and neither is in the bundle).
- [ ] `talOS/` still knows about Studio by label: `config_parser.h` defaults the bridge label and webroot, and the launcher has a `StudioBridge` class. There is no build dependency.

## Written but never run

- [ ] `tools/setup.ps1` install mode (only `-Check` ran), including the stop when the WSL default user is root. `tools/setup.sh` has now run on Ubuntu 24.04, macOS 15 (Homebrew), Arch (amd64) and Fedora 45; Fedora 45 no longer packages JDK 17 (docs/INSTALL.md says to install Temurin 17).
- [ ] `./sim.sh` with the WPILib RoboRIO sim ran on macOS (link configured, 11,913 states, 5,692 commands, no timeouts), but nobody enabled the robot in the sim GUI, so the enabled path is unexercised there.
- [ ] `deploy.sh enable-boot` without `--dry-run` (needs sudo), on the Mini and on Linux.
