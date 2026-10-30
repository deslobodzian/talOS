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
  - Then inject faults: `SIGSTOP` the arbiter for 50 ms and 200 ms (200 ms must go neutral within `command_timeout_us` plus a tick), `SIGSTOP` the drivetrain for 80 ms, unplug the RIO Ethernet while the wheels spin.
- [ ] Measure loop lateness on the Mini. Node binaries and the bridge now request realtime scheduling (`talos::process::ConfigureRealtime`: Mach time-constraint policy on macOS, `SCHED_FIFO` + `mlockall` on Linux; `TALOS_REALTIME=0` disables) and nodes without message callbacks drain only before their own timers; neither has been measured on the robot. Check that the Mach policy is accepted for every node period and that no node prints the "realtime scheduling incomplete" line.

## Found, not fixed

- [x] Odometry runs away when the RIO link goes quiet. Fixed: `HardwareAccess::*_sample` marks samples invalid once `HardwareLink::fresh()` is false (`command_timeout_us`), and `HardwareLink` exposes `fresh()` and `age()`. Covered by `SimHardware` tests in `talOS/devices/devices_test.cc` and `2026-robot/main_processor/drivetrain/node_test.cc`.
- [x] `tools/deploy.sh logs` never finds a session. Fixed: `run.sh` forwards `--output-dir "$TALOS_LOG_DIR"`, boot configs set `TALOS_LOG_DIR=$PREFIX/logs`, and `deploy.sh logs` searches `$PREFIX/logs`, the bundle directory, and `/tmp/talos_logs`.
- [x] `deploy.sh install` may break a bundle's Python node. Fixed: `talOS/build/bundle_builder.py` no longer writes build-tree absolute paths into `MANIFEST` or symlinks, and `//2026-robot/main_processor:bundle_test` copies `robot_bundle` to a temporary directory and runs `run.sh --describe-only`.
- [ ] The bundle's `nodes` and `subsystems` lists (`2026-robot/main_processor/BUILD`) are a second copy of the manifest, so a subsystem added to one and not the other fails only on the robot. `new_subsystem` edits both; derive them from `robot.toml` instead. (`sim_gateway` removed from `robot_bundle`.)
- [x] CI never runs `--describe-only` on the real configs. Fixed: added `talos_graph_test` in `talOS/build/contract.bzl` and wired `graph_test` targets in `2026-robot/main_processor/configuration/BUILD` and `examples/superstructure/config/BUILD`.

## Tooling

- [ ] Timing measurement needs: `log_dump` payload decoding (it prints sizes only) and a telemetry channel for the DS link offset. Done: hardware-state age (`HardwareLink::age()`, `/hw/status` `state_age_us`) and per-subsystem bridge timeouts (`/hw/status` `timed_out_mask`, shown in Studio as `bridge/*`).
- [x] `tools/deploy.sh enable-boot` now generates/installs a macOS `launchd` plist (`/Library/LaunchDaemons/org.talos.robot.plist`) on Darwin.
- [ ] Shooter and intake monitors (only the drivetrain has `monitor` and `send_target`, and neither is in the bundle) and a bring-up checklist in `docs/`.
- [x] `tools/new_subsystem.py` unit tests (`//tools:new_subsystem_test`) added for `check_name`, `insert_label`, `plan` (with custom `--robot`), and `--standalone` Bzlmod repository generation.
- [x] `talOS/process` unit test (`//talOS/process:process_test`) added.

## Subsystems as separate repos

Done: `bazel run //tools:new_subsystem -- <name> --lang cpp|py`; `talos_messages(deps = [...])` (a schema includes another package's types) with a guard against an empty package path; nested structs in the Python message codec; `@repo//` node labels through the names lint, `node_target` and `ResolveBinary`, with bundle-manifest overrides matching both label spellings. Proven with scratch modules only (an external node built, its baked label passed the lint, `--describe-only` was clean); no real second repo exists yet.

- [ ] Prove it with a real second repo (a wrist, say) consumed through `bazel_dep` + `local_path_override`, launched as `@wrist//:node` and bundled. Bundling an external node (`bundle_builder.py`, the manifest `label`) is untested.
- [x] Split subsystem defaults from robot overrides: `ParseRobotConfig` supports `defaults = "..."` in `[[subsystems]]` and `[subsystem]`, deep-merging subsystem device/param defaults with robot-level overrides (`//talOS/configuration:test`).
- [x] Shared messages package (`//talOS/msgs:geometry` and `//talOS/msgs:control`, ROS `*_msgs` style) added with C++ and Python `Sim` tests.
- [x] `tools/new_subsystem.py` scaffolds `/<sub>/target/teleop` and `/<sub>/target/auto` producer lanes in `talos_messages`, plus `--standalone DIR` for standalone Bzlmod subsystem repos.
- [x] Added `publish_bytes` (`BytePublisher`) and `Sim.latest_bytes` to `talOS/node_api` and `talOS/python`, plus relative topic resolution (`"state"` -> `"/<node>/state"`) and `[subsystems.<name>.remap]` in `talOS/node`.

## Decisions

- [x] Studio slimmed in place: the Tauri shell, `rules_rust`/crate_universe, the UDP path (port 5801, `[studio] udp_port`) and `package-lock.json` are gone; the bridge serves the web UI and WebSocket only. The schema moved to `//talOS/telemetry`, so robot code no longer depends on `studio/`.
- [ ] Keep the headless agent server (`//studio:agent`, port 5802) and `ws`, or remove them?
- [ ] Studio post-match replay (`studio_bridge --replay-dir <session>`) and a topic inspector/publisher, which would replace the per-subsystem `monitor`/`send_target` binaries.
- [x] `operator_interface` and `arbiter` run on `talos::DriverStation` (`/hw/state/driver_station`); the arbiter drops to no source when the DS goes silent, and button masks became 1-based `shoot_button`/`intake_button`.
- [x] `examples/superstructure/config/robot_py.toml` now uses top-level `include = "robot.toml"` and overrides only `node =` on `[[subsystems]]`; unknown top-level keys in `robot.toml` are rejected.
- [ ] When the RIO link drops, should odometry hold the pose (the old behaviour) or keep integrating the last velocities? (See "Found, not fixed".)
- [ ] Freshness counts from receipt, so a stalled consumer (Python GC, CPU starvation) that drains a backlog sees old targets as fresh for another `command_timeout_us`; only the RIO's observed-age check bounds the hardware sample, not the target's age. Nodes without message callbacks now drain only before their own timers, so receipt is the drain and the bound loosens by at most one node period. Acceptable for Python nodes? [read]
- [ ] Nothing publishes `/<sub>/target/auto`, so the robot idles in autonomous. Fine for bring-up, not for a match.

## Small

- [ ] Windows checkouts get CRLF for everything except `*.sh` and `gradlew` (`* text=auto` plus `core.autocrlf`). Tests that edit TOML as strings (`ConfigWith` in `2026-robot/main_processor/drivetrain/node_test.cc`) then fail when run from WSL against `/mnt/c`. Either `* text=auto eol=lf`, or always clone inside WSL (`docs/INSTALL.md` recommends it).
- [x] Fixed doc drift in `2026-robot/main_processor/drivetrain/README.md` and `subsystems/intake.toml`.
- [ ] The intake uses a fixed `INTAKE_RPS` whatever the ask, so the operator interface's `intake_target_rps` has no effect (`intake.py`). [read]
- [ ] `talOS/` still knows about Studio by label: `config_parser.h` defaults the bridge label and webroot, and the launcher has a `StudioBridge` class. There is no build dependency.

## Written but never run

- [ ] `tools/setup.ps1` install mode (only `-Check` ran), including the stop when the WSL default user is root; `tools/setup.sh` on macOS (Homebrew), Fedora (dnf) and Arch (pacman). `setup.sh` ran end to end on Ubuntu 24.04 only.
- [ ] `./sim.sh` end to end (WPILib sim, Studio) and the RoboRIO Gradle tasks (`phoenixTest`, `buildRio`): untouched, not re-run.
- [ ] Anything on a Mac: the build, the tests, the bundle, `deploy.sh`.
