# TODO

What's left needs the robot, a human step, or a decision. `[read]` marks a finding from reading the code that nobody has reproduced or measured yet.

## On the robot

Human-only steps (`.omp/RULES.md`: agents never set `commissioned` or change CAN ids):

- [ ] Set `[robot] controller_address` (`2026-robot/main_processor/configuration/robot.toml`, loopback today) to the RoboRIO, `10.56.87.2`, before building the bundle. `run.sh` forwards `--remote` only when run by hand; the boot unit passes no arguments and an installed bundle is read-only. Missed, the nodes reject `/hw/state` and `robot_enabled` stays false with nothing logged. [read]
- [ ] Give the Mac Mini the static address `10.56.87.11`. It is compiled into the RIO program (`kCompanionAddress` in `2026-robot/controller_processor/rio/src/main/cpp/Robot.cpp`), which only sends state there.
- [ ] Check the CAN ids in `subsystems/*.toml` against Phoenix Tuner X; measure the four CANcoder `offset_rot` (all `0.0`); check each `inverted`; tune the intake `slot0` (all zero: the roller will not follow its demand) and the drivetrain gains (drive `v = 0.12` with `sensor_to_mechanism_ratio = 6.75` looks like a rotor-side value; mechanism-side would be about 0.81 [read]). Then `commissioned = true`, and update `config_golden_test` in the same change.
- [ ] Build and test on the Mini first (`bazel test //...`; CI is Linux-only), then build `robot_bundle` there (there is no cross-compile config), `tools/deploy.sh install`, and check the drivetrain, shooter and intake. Suggested order: uncommissioned and disabled (the RIO logs `commissioned=0`, `monitor` shows not configured), drivetrain on blocks, shooter, intake. No bring-up checklist exists yet.
- [ ] Confirm two timing changes behave on the real link (sim cannot: the node and the RIO share one clock and the link has no latency):
  - target freshness is measured from when the node receives the target (`Subscription::fresh()`, `talOS/node/node.cc`), not from a sender timestamp (`issued_ns` has been removed);
  - `DrivetrainState.timestamp_ns` is node time, not RoboRIO sample time (only odometry reads it, as its integration `dt`).
  - Measure from the launcher's `.tlog`s (`bazel run //talOS/launcher:log_dump -- <dir>`): `/hw/state` inter-arrival (median about 5 ms, p99 under 10 ms, no gap of 100 ms while enabled); target age at each drivetrain tick (about 25 ms at most, never `command_timeout_us`); loop lateness in Studio's system view.
  - Then inject faults: `SIGSTOP` the `driver` node (chassis) and the `teleop` node (mechanism buttons) for 50 ms and 200 ms (200 ms must go neutral within `command_timeout_us` plus a tick; each subsystem times out its own lane), `SIGSTOP` the drivetrain for 80 ms, unplug the RIO Ethernet while the wheels spin. Also measure stick → drive request median on the real link (sim: 2.99 ms; old arbiter baseline 4.98 ms).
- [ ] Measure loop lateness on the Mini. Node binaries and the bridge now request realtime scheduling (`talos::process::ConfigureRealtime`: Mach time-constraint policy on macOS, `SCHED_FIFO` + `mlockall` on Linux; `TALOS_REALTIME=0` disables) and nodes without message callbacks drain only before their own timers; neither has been measured on the robot. Check that the Mach policy is accepted for every node period and that no node prints the "realtime scheduling incomplete" line.

## Found, not fixed

- [x] Odometry runs away when the RIO link goes quiet. Fixed: `HardwareAccess::*_sample` marks samples invalid once `HardwareLink::fresh()` is false (`command_timeout_us`), and `HardwareLink` exposes `fresh()` and `age()`. Covered by `SimHardware` tests in `talOS/devices/devices_test.cc` and `2026-robot/main_processor/drivetrain/node_test.cc`.
- [x] `tools/deploy.sh logs` never finds a session. Fixed: `run.sh` forwards `--output-dir "$TALOS_LOG_DIR"`, boot configs set `TALOS_LOG_DIR=$PREFIX/logs`, and `deploy.sh logs` searches `$PREFIX/logs`, the bundle directory, and `/tmp/talos_logs`.
- [x] `deploy.sh install` may break a bundle's Python node. Fixed: `talOS/build/bundle_builder.py` no longer writes build-tree absolute paths into `MANIFEST` or symlinks, and `//2026-robot/main_processor:bundle_test` copies `robot_bundle` to a temporary directory and runs `run.sh --describe-only`.
- [x] Complete D5 acceptance. Done: shared parser/`sync_manifest` fixtures, effective-config scaffold checks, relocated-bundle test, scaffold goldens. See `docs/HANDOFF_AUTOMATIC_WIRING.md`.
- [x] CI never runs `--describe-only` on the real configs. Fixed: added `talos_graph_test` in `talOS/build/contract.bzl` and wired `graph_test` targets in `2026-robot/main_processor/configuration/BUILD` and `examples/superstructure/config/BUILD`.
- [x] Complete D1 acceptance. Done: mode generation stamps every target, regressions in C++ and Python, live sim fault table, stick-to-request median 2.99 ms. `GenericTarget` is gone (D4). See `docs/HANDOFF_AUTOMATIC_WIRING.md`.
- [x] The selected target source is in the node's registry row and shown in Studio's system view.
- [ ] Automatic wiring (D1-D6) is verified only on macOS in simulation. A draft PR from `agent/automatic-wiring` to `main` runs CI: the Linux build and tests (wpimath Linux artifacts never built), `clang-format-18` (local formatting used 21), and `buildifier` on changed files. Run `clang-tidy-18` by hand if CI does not. Fix what fails, then merge. `phoenixTest` still needs a machine with the macOS SDK GradleRIO wants (or Linux).

## Next for agents (automatic-wiring follow-ups)

Read `docs/HANDOFF_AUTOMATIC_WIRING.md` "Status at the 2026-11-04 evening stop" first.

- [ ] Get the draft PR's CI green (item above). Do not merge red.
- [ ] Python subsystem authors cannot exercise their `[[teleop.bindings]]` in a Python `Sim`: `//talOS/teleop:node` is C++ only. If wanted, expose it the sanctioned way (`TALOS_EXPORT_NODE` + `talos.cpp_node`, as `examples/superstructure/py` does) with a `talos.testing` test, and note that the declared-graph API it needs is C++-only by design (handoff "Decisions"). An unused `libteleop.so` was removed rather than kept without a consumer.
- [ ] `//talOS/telemetry:node` skips topics without a schema (today `/hw/state`, the raw hardware packet) and prints that once to stderr. Decide whether `/hw/state` gets a descriptor or stays out of telemetry.
- [ ] Studio: `studio/bridge/telemetry.h:167` still cites the old channel name `pose/vx` in a comment; channels are now `<owner>/<role>/<field>` (e.g. `pose_estimator/state/velocity/vx_mps`). Check Studio views for any remaining assumption of the old names on the actual page.
- [ ] Lint warnings left on purpose: `target/auto` lanes nothing writes (see Decisions), unit-suffix warnings on `examples/superstructure` intake params (`eject_ms`, `eject_volts`, `hold_volts`, `intake_volts`). Renaming those params touches the example's C++ and Python nodes together.
- [ ] Local cleanup, ask the user first: worktrees under `.muse/` and the local `wiring/*` branches are merged and no longer needed.

## Tooling

- [ ] Timing measurement needs a telemetry channel for the DS link offset. Done: `log_dump` decodes payloads to named fields (D2), hardware-state age (`HardwareLink::age()`, `/hw/status` `state_age_us`) and per-subsystem bridge timeouts (`/hw/status` `timed_out_mask`, shown in Studio as `hw/status/*`).
- [x] `tools/deploy.sh enable-boot` now generates/installs a macOS `launchd` plist (`/Library/LaunchDaemons/org.talos.robot.plist`) on Darwin.
- [ ] Shooter and intake monitors (only the drivetrain has `monitor` and `send_target`, and neither is in the bundle) and a bring-up checklist in `docs/`.
- [x] `tools/new_subsystem.py` unit tests (`//tools:new_subsystem_test`) added for `check_name`, `plan` (with custom `--robot`, checking the regenerated manifest), and `--standalone` Bzlmod repository generation.
- [x] `talOS/process` unit test (`//talOS/process:process_test`) added.

## Subsystems as separate repos

Done: `bazel run //tools:new_subsystem -- <name> --lang cpp|py`; `talos_messages(deps = [...])` (a schema includes another package's types) with a guard against an empty package path; nested structs in the Python message codec; `@repo//` node labels through the names lint, `node_target` and `ResolveBinary`, with bundle-manifest overrides matching both label spellings. Proven with scratch modules only (an external node built, its baked label passed the lint, `--describe-only` was clean); no real second repo exists yet.

- [ ] Prove it with a real second repo (a wrist, say) consumed through `bazel_dep` + `local_path_override`, launched as `@wrist//:node` and bundled. A throwaway `@wrist//:node` repo has been built and launched (D5); a committed one, and bundling an external node (`bundle_builder.py`, the manifest `label`), are not.
- [x] Split subsystem defaults from robot overrides: `ParseRobotConfig` supports `defaults = "..."` in `[[subsystems]]` and `[subsystem]`, deep-merging subsystem device/param defaults with robot-level overrides (`//talOS/configuration:test`).
- [x] Shared messages package (`//talOS/msgs:geometry` and `//talOS/msgs:control`, ROS `*_msgs` style) added with C++ and Python `Sim` tests.
- [x] `tools/new_subsystem.py` scaffolds a `talos::Target`/`talos.Target` consumer of its `/<sub>/target/<source>` lanes, the §11 test matrix, and a commented `[targets]` policy, plus `--standalone DIR` for standalone Bzlmod subsystem repos.
- [x] Added `publish_bytes` (`BytePublisher`) and `Sim.latest_bytes` to `talOS/node_api` and `talOS/python`, plus relative topic resolution (`"state"` -> `"/<node>/state"`) and `[subsystems.<name>.remap]` in `talOS/node`.

## Decisions

- [x] Studio slimmed in place: the Tauri shell, `rules_rust`/crate_universe, the UDP path (port 5801, `[studio] udp_port`) and `package-lock.json` are gone; the bridge serves the web UI and WebSocket only. The schema moved to `//talOS/telemetry`, so robot code no longer depends on `studio/`.
- [ ] Keep the headless agent server (`//studio:agent`, port 5802) and `ws`, or remove them?
- [ ] Studio post-match replay (`studio_bridge --replay-dir <session>`) and a topic inspector/publisher, which would replace the per-subsystem `monitor`/`send_target` binaries.
- [x] Teleop is split: the `driver` node (`2026-robot/main_processor/driver`) drives the chassis from `talos::DriverStation`; mechanism buttons are `[[teleop.bindings]]` in each subsystem's TOML, written by `//talOS/teleop:node`.
- [x] `examples/superstructure/config/robot_py.toml` now uses top-level `include = "robot.toml"` and overrides only `node =` on `[[subsystems]]`; unknown top-level keys in `robot.toml` are rejected.
- [ ] When the RIO link drops, should odometry hold the pose (the old behaviour) or keep integrating the last velocities? (See "Found, not fixed".)
- [ ] Freshness counts from receipt, so a stalled consumer (Python GC, CPU starvation) that drains a backlog sees old targets as fresh for another `command_timeout_us`; only the RIO's observed-age check bounds the hardware sample, not the target's age. Nodes without message callbacks now drain only before their own timers, so receipt is the drain and the bound loosens by at most one node period. Acceptable for Python nodes? [read]
- [ ] Nothing publishes `/<sub>/target/auto`, so the robot idles in autonomous. Fine for bring-up, not for a match.

## Small

- [ ] Windows checkouts get CRLF for everything except `*.sh` and `gradlew` (`* text=auto` plus `core.autocrlf`). Tests that edit TOML as strings (`ConfigWith` in `2026-robot/main_processor/drivetrain/node_test.cc`) then fail when run from WSL against `/mnt/c`. Either `* text=auto eol=lf`, or always clone inside WSL (`docs/INSTALL.md` recommends it).
- [x] Fixed doc drift in `2026-robot/main_processor/drivetrain/README.md` and `subsystems/intake.toml`.
- [ ] The intake uses a fixed `INTAKE_RPS` whatever the ask (`intake.py`), so the `roller_velocity_rps` in its `[[teleop.bindings]]` only switches it on. [read]
- [ ] `talOS/` still knows about Studio by label: `config_parser.h` defaults the bridge label and webroot, and the launcher has a `StudioBridge` class. There is no build dependency.

## Written but never run

- [ ] `tools/setup.ps1` install mode (only `-Check` ran), including the stop when the WSL default user is root; `tools/setup.sh` on macOS (Homebrew), Fedora (dnf) and Arch (pacman). `setup.sh` ran end to end on Ubuntu 24.04 only.
- [ ] `./sim.sh` with the WPILib RoboRIO sim (only `--sim-gateway`, with and without Studio, has run since automatic wiring) and `phoenixTest`; `buildRio` compiles.
- [ ] `deploy.sh` on a Mac. The build, tests, bundle (`bundle_test`) and sim run on macOS arm64.
