# TODO

What's left needs the robot, a human step, or a decision. `[read]` marks a finding from reading the code that nobody has reproduced or measured yet. The bring-up procedure is [docs/BRINGUP.md](docs/BRINGUP.md).

## On the robot

Human-only steps (`.omp/RULES.md`: agents never set `commissioned` or change CAN ids):

- [x] `controller_address`: `robot_bundle` is built from `configuration/robot_field.toml` (an `include` overlay setting `10.56.87.2`); `robot.toml` stays loopback for sim. `bundle_test` fails if the bundled config does not resolve to `10.56.87.2`.
- [ ] Give the Mac Mini the static address `10.56.87.11`. It is compiled into the RIO program (`kCompanionAddress` in `2026-robot/controller_processor/rio/src/main/cpp/Robot.cpp`), which only sends state there.
- [ ] Check the CAN ids in `subsystems/*.toml` against Phoenix Tuner X; measure the four CANcoder `offset_rot` (all `0.0`); check each `inverted`; verify and tune the sim-consistent intake roller `slot0` (`p = 0.1`, `v = 0.12`) and drivetrain defaults (`slot0.v = 0.81`, `max_velocity_rps = 14.5`, `cruise_velocity_rps = 14.5`) on the real robot. Only then set `commissioned = true`, and update `config_golden_test` in the same change.
- [ ] Build and test on the Mini first (`bazel test //...`), then build `robot_bundle` there (there is no cross-compile config), `tools/deploy.sh install`, `enable-boot --user <account>`, and check the drivetrain, shooter and intake in the order in docs/BRINGUP.md. `enable-boot` installs a LaunchDaemon (`/Library/LaunchDaemons/org.talos.robot.plist`) with sudo; only its `--dry-run` has run.
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

## Merge `agent/automatic-wiring`

- [ ] Open the draft PR to `main` and get CI green; do not merge red. Reproduced locally on 2026-11-05: Linux arm64 container `bazel build //...` + `bazel test //...` 83/83 at `a54428e`; `phoenixTest` (8/8 per variant) and `buildRio` in a Linux amd64 container; `clang-format-18` (449 violations in 23 files, fixed), `buildifier` 8.2.1 (2 files, fixed) and `clang-tidy-18` (93 errors in 18 files, fixed) on the files changed since `main`. Not yet run: CI itself (amd64 Bazel build with the `linuxx86-64` wpimath artifacts).
- [ ] `phoenixTest` on macOS still needs the `MacOSX26.sdk` GradleRIO wants; it passes on Linux.

## Found, not fixed

- [ ] Studio's system view lists `sim_gateway` as "declared, never registered — it crashed at startup, or its binary is missing" while it runs: the gateway is not a talos node and never registers. Seen live with `./sim.sh --sim-gateway`.
- [ ] `examples/external_subsystem/bundle_test.py` still relocates with `copytree(symlinks=True)`; the 2026 `bundle_test` installs through `deploy.sh`, which is what caught the `cp -r` bug below.

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
- [x] Swerve kinematics uses second-order `ChassisSpeeds::Discretize` and scales drive speed by the cosine of steer error (`2026-robot/main_processor/drivetrain/swerve.h`).
- [x] Driver controls use a circular deadband and radial speed cap, a Red-alliance field-frame flip, rising-edge gyro re-zero (`rezero_button_id`) and proportional heading hold (`heading_hold_radps`) in `2026-robot/main_processor/driver/driver.h`.
- [x] Sim-consistent config defaults: drivetrain drive `slot0.v = 0.81`, `max_velocity_rps = 14.5` and `cruise_velocity_rps = 14.5` in `drivetrain.toml`; intake roller `slot0` uses `p = 0.1`, `v = 0.12` in `intake.toml`. Real-robot verification/tuning remains required before commissioning.
- [x] `PhysicsSimBackend` implements MotionMagic trapezoidal profiling and automatic swerve-to-Pigeon2 rotational coupling (`talOS/hardware/sim_physics.h`).
- [x] `//2026-robot/main_processor/autonomous:node` writes `/drivetrain/target/auto`, `/shooter/target/auto` and `/intake/target/auto`: preload spinup → feed → alliance-aware waypoint + intake → hold. All six missing-auto graph warnings on `2026-robot` are eliminated.

## Decisions taken

- [x] Odometry when the RIO link drops: hold. `Drivetrain` clears `modules_valid`/`yaw_valid`, zeroes speeds and keeps the last module positions while samples are stale.
- [x] Freshness counts from receipt for C++ and Python nodes alike (ARCHITECTURE.md, "The tick").
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
