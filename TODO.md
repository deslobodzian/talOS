# TODO

What's left needs the robot or a decision.

## On the robot
- [ ] Deploy (`tools/deploy.sh install`), then check the drivetrain, shooter and intake on real hardware.
- [ ] Confirm two timing changes behave on the real link:
  - target freshness is measured from when the node receives the target (`Subscription::fresh()`), not from the sender's `issued_ns`;
  - `DrivetrainState.timestamp_ns` is node time, not RoboRIO sample time.

## Decisions
- [ ] Studio is still about 10.5k LOC, plus the Tauri, uWebSockets and agent JSON-RPC toolchains. Slim it down, or move it to its own repo?
- [ ] `operator_interface` and `arbiter` decode the Driver Station topic themselves. They would be shorter on `talos::DriverStation` (staleness built in, `joystick(0).axis(1)`, `button(n)`). The catch: they would read the bridge topic directly instead of through the driver_station node, and the button-mask params would become button numbers.

## Small
- [ ] `examples/superstructure/config/robot.toml` and `robot_py.toml` still repeat the `[robot]` block. A manifest-level include would remove that.
