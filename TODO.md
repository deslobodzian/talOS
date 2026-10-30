# TODO

What's left needs the robot, a human step, or a decision. `[read]` marks a finding from reading the code that nobody has reproduced or measured yet.

## On the robot

Human-only steps (`.omp/RULES.md`: agents never set `commissioned` or change CAN ids):

- [ ] Set `[robot] controller_address` (`2026-robot/main_processor/configuration/robot.toml`, loopback today) to the RoboRIO, `10.56.87.2`, before building the bundle. `run.sh` forwards `--remote` only when run by hand; the boot unit passes no arguments and an installed bundle is read-only. Missed, the nodes reject `/hw/state` and `robot_enabled` stays false with nothing logged. [read]
- [ ] Give the Mac Mini the static address `10.56.87.11`. It is compiled into the RIO program (`kCompanionAddress` in `2026-robot/controller_processor/rio/src/main/cpp/Robot.cpp`), which only sends state there.
- [ ] Check the CAN ids in `subsystems/*.toml` against Phoenix Tuner X; measure the four CANcoder `offset_rot` (all `0.0`); check each `inverted`; tune the intake `slot0` (all zero: the roller will not follow its demand) and the drivetrain gains (drive `v = 0.12` with `sensor_to_mechanism_ratio = 6.75` looks like a rotor-side value; mechanism-side would be about 0.81 [read]). Then `commissioned = true`, and update `config_golden_test` in the same change.
- [ ] Build and test on the Mini first (`bazel test //...`; CI is Linux-only), then build `robot_bundle` there (there is no cross-compile config), `tools/deploy.sh install`, and check the drivetrain, shooter and intake. Suggested order: uncommissioned and disabled (the RIO logs `commissioned=0`, `monitor` shows not configured), drivetrain on blocks, shooter, intake. No bring-up checklist exists yet.
- [ ] Confirm two timing changes behave on the real link (sim cannot: the node and the RIO share one clock and the link has no latency):
  - target freshness is measured from when the node receives the target (`Subscription::fresh()`, `talOS/node/node.cc`), not from the sender's `issued_ns` (still written, never read);
  - `DrivetrainState.timestamp_ns` is node time, not RoboRIO sample time (only odometry reads it, as its integration `dt`).
  - Measure from the launcher's `.tlog`s (`bazel run //talOS/launcher:log_dump -- <dir>`): `/hw/state` inter-arrival (median about 5 ms, p99 under 10 ms, no gap of 100 ms while enabled); target age at each drivetrain tick (about 25 ms at most, never `command_timeout_us`); loop lateness in Studio's system view; the `/driver_station/state` offset `timestamp_ns / 1000 - rio_sample_time_us` (its spread is link plus drain jitter, its slope is clock drift).
  - Then inject faults: `SIGSTOP` the arbiter for 50 ms and 200 ms (200 ms must go neutral within `command_timeout_us` plus a tick), `SIGSTOP` the drivetrain for 80 ms, unplug the RIO Ethernet while the wheels spin.
- [ ] Measure loop lateness on the Mini: `talOS/events` uses kqueue there (epoll + timerfd on Linux), delivery is a 1 ms drain, and nothing in `talOS/` sets `SCHED_FIFO`, a Mach `thread_policy`, `pthread_setschedparam` or `mlockall` on either platform.

## Found, not fixed

- [ ] Odometry runs away when the RIO link goes quiet. [read] `Motor::connected()` is only `sample.valid`, with no age check (`talOS/devices/devices.cc`), so the drivetrain keeps publishing the last measured velocities with a fresh `timestamp_ns` and odometry keeps integrating them; telemetry's `drivetrain/stale` stays false and the operator interface keeps a stale yaw marked fresh. Motors still go neutral (`robot_enabled` goes false), so this is pose drift, not actuation. First write the `SimHardware` test that silences `/hw/state` while the drivetrain moves (`hw.reset()`, as in `talOS/devices/devices_test.cc`), then fix it one of two ways: make `connected()` false once the sample is older than `command_timeout_us` (expose the hardware-state age through `talOS/devices`, `node_api` and `talos/devices.py`), or add `rio_sample_time_us` to `DrivetrainState` like `DriverStationState` and have odometry use `dt = 0` on a repeated sample (a wire-size change).
- [ ] `tools/deploy.sh logs` never finds a session. [read] It looks for `manifest.json` under the installed bundle, but the launcher writes sessions to `/tmp/talos_logs/<id>/` unless given `--output-dir`. Pass `--output-dir <prefix>/logs` from `run.sh` or the boot unit and look there.
- [ ] `deploy.sh install` may break a bundle's Python node. [read] `talOS/build/bundle_builder.py` writes absolute build-machine paths into the runfiles `MANIFEST` and symlinks, which the intake (hermetic interpreter plus `libtalos_node.so`) relies on, and the bundle is then copied elsewhere. Nothing tests `bundle_builder.py` or `talos_bundle`: add a test that copies a built bundle and runs `run.sh --describe-only`.
- [ ] The bundle's `nodes` and `subsystems` lists (`2026-robot/main_processor/BUILD`) are a second copy of the manifest, so a subsystem added to one and not the other fails only on the robot. `new_subsystem` edits both; derive them from `robot.toml` instead. The robot bundle also ships `sim_gateway`, which it does not need.
- [ ] CI never runs `--describe-only` on the real configs (the 2026 `robot.toml`, `examples/superstructure/config/robot.toml` and `robot_py.toml`). Add it as a Bazel test so a graph error fails `bazel test //...`. Add a macOS CI job too (even `bazel build //talOS/...`): the coprocessor is macOS and CI is Linux-only.

## Tooling

- [ ] Timing measurement needs: `log_dump` payload decoding (it prints sizes only), a telemetry channel for the DS link offset, an accessor for hardware-state age, and per-subsystem bridge timeout counters (`hardware_node` prints `timed_out` only at exit).
- [ ] `tools/deploy.sh enable-boot` only prints a systemd unit on macOS; write a launchd plist (`/Library/LaunchDaemons`) for the Mini. The rest of `deploy.sh` has not been run on macOS.
- [ ] Shooter and intake monitors (only the drivetrain has `monitor` and `send_target`, and neither is in the bundle) and a bring-up checklist in `docs/`.
- [ ] `tools/new_subsystem.py` was smoke-tested only (a C++ `wrist` and a Python `climber` scaffolded in a scratch copy, then their tests, `--describe-only` and `robot_bundle`). Add unit tests for `check_name`, `insert_label` and `plan`, and try `--robot` with something other than `2026-robot`.
- [ ] `talOS/process` has no tests.

## Subsystems as separate repos

Done: `bazel run //tools:new_subsystem -- <name> --lang cpp|py`; `talos_messages(deps = [...])` (a schema includes another package's types) with a guard against an empty package path; nested structs in the Python message codec; `@repo//` node labels through the names lint, `node_target` and `ResolveBinary`, with bundle-manifest overrides matching both label spellings. Proven with scratch modules only (an external node built, its baked label passed the lint, `--describe-only` was clean); no real second repo exists yet.

- [ ] Prove it with a real second repo (a wrist, say) consumed through `bazel_dep` + `local_path_override`, launched as `@wrist//:node` and bundled. Bundling an external node (`bundle_builder.py`, the manifest `label`) is untested.
- [ ] Split subsystem defaults from robot overrides: a subsystem repo ships device types and params, the robot repo supplies CAN ids, gains and `commissioned`. This generalizes the config-include decision below.
- [ ] A shared messages repo (ROS `*_msgs` style) for types several subsystems use, such as a `Pose2d` inside `DrivetrainState`, so no subsystem repo depends on another's code. `deps` makes it possible; the repo and the type moves do not exist.
- [ ] A scaffolded subsystem has no producer: it needs a `/<sub>/target/<source>` lane and a line in the arbiter. Teach the scaffolder to add them?
- [ ] More languages. The wire is FlatBuffers structs, so nodes in different languages already interoperate; a new language needs C ABI bindings, a `talos_<lang>_node` rule, `talos_messages` codegen and a Sim harness. Java is the obvious third (WPILib). The Python node API still lacks `publish_bytes` (`docs/NODE_DEVELOPMENT_GUIDE.md`).

## Decisions

- [ ] Studio is about 10k lines plus the Tauri, uWebSockets and agent JSON-RPC toolchains. Slim it down, or move it to its own repo?
  - The robot and `sim.sh` use only the bridge (`//studio/bridge:studio_bridge`), `//studio/schema` and the web UI. Nothing uses the Tauri shell, its UDP path (port 5801; only Tauri listens), the headless agent server (`//studio:agent`, port 5802) or `studio/package-lock.json` (dead; `pnpm-lock.yaml` is the used one).
  - Recommendation: slim in place, in four steps. (1) Delete `studio/src-tauri` and the Rust and `crate_universe` modules in `MODULE.bazel`, with the matching CI, `setup.sh`/`setup.ps1` and docs. (2) Remove the UDP path (bridge, `[studio] udp_port`, launcher arguments, tests; `studio/bridge/integration_test.py` moves to WebSocket, which needs a client library). (3) Remove the agent server and its examples; keep `agent/rpc.ts` (the in-browser Agent tab uses it) and drop `ws`. (4) Delete `package-lock.json`.
  - That removes the Rust toolchain, about 435 crates and the GTK/WebKit packages from CI and every Linux dev box (`tools/setup.sh` installs them even with `--no-studio`, because `bazel build //...` builds the shell). Moving Studio out saves little more: the bridge links talOS internals and the telemetry node depends on `//studio/schema`.
  - Questions: keep the headless agent server? Any non-browser consumer of the UDP stream (a driver-station laptop)? Move `studio/schema` to a talOS-neutral path, since a robot package depends on it?
- [ ] `operator_interface` and `arbiter` decode the Driver Station topic themselves (about 20 lines each); they would be shorter on `talos::DriverStation`.
  - Recommendation: move both onto it, reading the bridge topic `/hw/state/driver_station`. The C ABI and Python mirror already exist, so no node-API change, and the `driver_station` node stays alive because `telemetry` still reads `/driver_station/state`. It also fixes the arbiter keeping its last mode when the DS goes silent: add an `update()` that publishes `kNone` when `!ds.connected()`.
  - Behaviour changes to accept: a new `kDsAttached` gate; button masks become button numbers (`shoot_button_mask` and `intake_button_mask` become `shoot_button` and `intake_button`, so no "any of" chords); buttons above the controller's `button_count` read false.
  - Touch list: `operator_interface.h` and its TOML, `arbiter.h` (declare the handle before the lanes), their BUILD deps, tests through `SimHardware::set_driver_station` (plus stale-DS and `kDsAttached` cases), `2026-robot/README.md`, the "one reader" comment in `talOS/driver_station/driver_station_node.h`.
  - Questions: also move `telemetry` and delete the node, its `.fbs` and its TOML (cleaner, but `/driver_station/state` disappears from logs and Studio)? Are single button numbers enough?
- [ ] `examples/superstructure/config/robot.toml` and `robot_py.toml` repeat the `[robot]` block and the three `[[subsystems]]` name/path pairs; only the `node =` labels differ.
  - Smallest fix: a top-level `include = "robot_common.toml"`, resolved in `ParseRobotConfig` (`talOS/configuration/config_parser.h`) before the manifest expands. The included file may hold only `[robot]` and `[studio]`; a key set in both files is an error; no nested includes. `talos_bundle` needs a `config_includes` attribute and `bundle_builder.py` must copy and hash the file; tests go in `config_test.cc`. An unknown top-level key is silently ignored today, so a misspelled `include` would do nothing: consider rejecting unknown keys.
  - Alternative: `extends` (`robot_py.toml` extends `robot.toml`): no duplication left, about twice the parser code.
  - Questions: only `[robot]`, or `extends`? Key conflicts an error (recommended) or the manifest wins? `[studio]` in the include? Bundle support now (recommended) or reject `include` in bundled configs?
- [ ] When the RIO link drops, should odometry hold the pose (the old behaviour) or keep integrating the last velocities? (See "Found, not fixed".)
- [ ] Keep `issued_ns` on targets as log-only sender time (it measures in-host pipeline latency), or remove it (frozen wire sizes change)? `hardware::TargetFresh` (`talOS/hardware/gate.h`) is dead code outside `gate_test.cc` either way.
- [ ] Freshness counts from receipt, so a stalled consumer (Python GC, CPU starvation) that drains a backlog sees old targets as fresh for another `command_timeout_us`; only the RIO's observed-age check bounds the hardware sample, not the target's age. Acceptable for Python nodes? [read]
- [ ] Nothing publishes `/<sub>/target/auto`, so the robot idles in autonomous. Fine for bring-up, not for a match.

## Small

- [ ] Windows checkouts get CRLF for everything except `*.sh` and `gradlew` (`* text=auto` plus `core.autocrlf`). Tests that edit TOML as strings (`ConfigWith` in `2026-robot/main_processor/drivetrain/node_test.cc`) then fail when run from WSL against `/mnt/c`. Either `* text=auto eol=lf`, or always clone inside WSL (`docs/INSTALL.md` recommends it).
- [ ] The old `//2026-robot/main_processor/driver_station:node` label (now `//talOS/driver_station:node`) is still in `studio/tests/launcher_graph.json` and `launcher_graph_faults.json` (the launcher README calls them real `--describe-only` output, so regenerate them), `studio/src/demo.ts`, and fixture strings in `talOS/bridge/hardware_node_test.cc` and `talOS/introspection/names_test.cc`.
- [ ] Doc drift: `2026-robot/main_processor/drivetrain/README.md` says "in `robot/`" and `simulateNativeRelease` (the directory is `2026-robot/controller_processor/rio`, the task is `simulateNative`) and that there is "no network configuration upload" (the bridge pushes it; `talOS/hardware/README.md`); `subsystems/intake.toml` cites `talos_nodes/intake/node.py` (now `main_processor/intake/intake.py`).
- [ ] The intake uses a fixed `INTAKE_RPS` whatever the ask, so the operator interface's `intake_target_rps` has no effect (`intake.py`). [read]
- [ ] `talOS/` knows about Studio by label: `config_parser.h` defaults the bridge label and webroot, and the launcher has a `StudioBridge` class. There is no build dependency, but `talOS/` is not usable without a `studio/` (part of the Studio decision).
- [ ] `talOS/rtms` has two TODOs (`rtms.cc`: `void*` vs `std::byte*`; `rtms.h`: `RTMSMessage` is redundant with `std::span`).

## Written but never run

- [ ] `tools/setup.ps1` install mode (only `-Check` ran), including the stop when the WSL default user is root; `tools/setup.sh` on macOS (Homebrew), Fedora (dnf) and Arch (pacman). `setup.sh` ran end to end on Ubuntu 24.04 only.
- [ ] `./sim.sh` end to end (WPILib sim, Studio) and the RoboRIO Gradle tasks (`phoenixTest`, `buildRio`): untouched, not re-run.
- [ ] Anything on a Mac: the build, the tests, the bundle, `deploy.sh`.
