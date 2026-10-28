# TODO

Open work left after the cleanup and the node-API cutover.

## Verify on a machine with WPILib
- [ ] Build the RoboRIO program (`cd 2026-robot/controller_processor/rio && ./gradlew build phoenixTest`). It has not been compiled since `ZeroPosition`/`GetHealth` were removed and `build.gradle` switched to an explicit talOS source list.
- [ ] Run `python3 tools/test_drivetrain.py --wpilib` and `./sim.sh` (WPILib sim, not `--sim-gateway`).
- [ ] Deploy to the robot: `tools/deploy.sh install`, then check the drivetrain, shooter and intake on real hardware.

## Behavior changes to confirm on the robot
- [ ] Freshness of a target is now measured from when the node received it (`Subscription::fresh()`), not from the sender's `issued_ns`. This affects shooter, drivetrain and intake.
- [ ] `DrivetrainState.timestamp_ns` is node time, not RoboRIO sample time.
- [ ] The drivetrain setting `integrate_heading` defaults to on under `--sim`. Confirm it is off on the robot.

## Test coverage lost with tools/delivery
- [ ] Port the gateway failure-scenario test (`scenario_matrix_test.cc`) into `talOS/hardware/gateway_test.cc`.
- [ ] Port the launcher session/shutdown test (`supervision_test.cc`) into `talOS/launcher/launcher_test.cc`.
  Both can be recovered with `git show <cleanup-commit>^:tools/delivery/<file>`.

## Python API gaps
- [ ] Topic names exist only in C++ (`topics.h`), so Python nodes write the strings out. Add a `topics=` option to `talos_messages` that generates both a C++ header and a Python module.
- [ ] Message enum fields print as numbers. Look up enum names in `Message.__repr__` and `StateMachine`.
- [ ] `examples/superstructure/config/robot_py.toml` duplicates `robot.toml` except for the `node =` labels (`config_test.py` guards the drift). Let the manifest or the launcher override a subsystem's node label so one config serves both languages.
- [ ] `SimHardware` always reports devices as valid, so intake's `FEEDBACK_INVALID` path is untested. Add a way to mark a device disconnected.

## Leftovers worth simplifying
- [ ] Telemetry still runs on the low-level `process::RunNode`, because its Studio feed is a variable-length frame. Consider a variable-size publisher in `talOS/node` so it can move to `talos::Node`.
- [ ] The driver-station node subscribes to `/hw/state/driver_station` through `detail::NodeAccess`, because the bridge's packet alignment differs from user topics. Add a driver-station handle to `talOS/devices`.
- [ ] `talOS/introspection/names.h` (558 LOC) still accepts `@repo`/`talos-` target spellings from the old module split. Trim it to the topic grammar.
- [ ] `talOS/launcher/launcher.h` (~1.4k LOC) hand-parses bundle JSON with `string::find`, and Studio supervision is spread across it. Simplify.
- [ ] `config_parser.h` still accepts two layouts (`[[subsystems]]` manifest and single-file `[subsystems.<name>]`). Pick one.
- [ ] `hardware::Packet` is 1404 bytes per tick per node for a handful of motors. Consider a tighter request encoding.
- [ ] Studio was left as-is, at about 10.5k LOC plus the Tauri, uWebSockets and agent JSON-RPC toolchains. Decide whether to slim it down or move it to its own repo.

## Housekeeping
- [ ] CI: confirm the GitHub workflow passes with the standalone-modules job removed.
