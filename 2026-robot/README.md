# 2026-robot

| | runs | built with |
|---|---|---|
| `main_processor/` | the talOS nodes | Bazel |
| `controller_processor/rio/` | the RoboRIO program: the hardware layer, over UDP | GradleRIO |

## Nodes

| Node | Lang | Does | Reads | Writes |
|---|---|---|---|---|
| `drivetrain` | C++ | swerve kinematics and module control | `/drivetrain/target/teleop`, `/drivetrain/target/auto` | `/drivetrain/state` |
| `shooter` | C++ | flywheel to target speed, beam break | `/shooter/target/teleop`, `/shooter/target/auto` | `/shooter/state` |
| `intake` | Python | roller and beam break | `/intake/target/teleop`, `/intake/target/auto` | `/intake/state` |
| `operator_interface` | C++ | joystick 0 sticks and buttons (`shoot_button`, `intake_button`, per-subsystem `teleop_button`, 1-based) to teleop target lanes | `/hw/state/driver_station`, `/drivetrain/state` | `/drivetrain/target/teleop`, `/shooter/target/teleop`, `/intake/target/teleop` |
| `pose_estimator` | Python | field pose from swerve odometry and camera observations; backend `kalman` (wpimath) or `factor_graph` (GTSAM); runs while disabled | `/drivetrain/state`, `/<camera>/state` per `cameras` | `/pose_estimator/state` |
| `telemetry` | C++ | Studio frames; the pose from `pose_topic` | the state topics, `/hw/state/driver_station`, `/hw/status` | `/talos/telemetry` |
| `hardware_node` (`//talOS/bridge`) | C++ | UDP courier to the RoboRIO | `/hw/request/<node>` | `/hw/state`, `/hw/status` |

Simulation stands in for the RoboRIO with `main_processor/simulation:sim_gateway`.

## Config

`main_processor/configuration/robot.toml` is the manifest (`[robot]`,
`[robot.targets]`, `[studio]`, and a `[[subsystems]]` list); each subsystem's devices and
parameters are in `subsystems/<name>.toml`. `[robot.targets]` is the target
policy: which lanes (`/<sub>/target/<source>`) each match mode reads, in
priority order; each subsystem's `talos::Target` follows the first fresh,
active one, and nothing while the Driver Station is disabled, e-stopped or
silent. `config_golden_test` pins them.
Devices are referred to by name; ids are assigned by sorting
(subsystem, device).

## Run in simulation

From the repository root:

```sh
./sim.sh --sim-gateway --no-studio   # nodes + lightweight simulator
./sim.sh                             # WPILib simulation (needs WPILib) + Studio
./sim.sh --headless                  # WPILib without its GUI
python3 tools/test_drivetrain.py     # drivetrain smoke test with log replay
```

With the WPILib simulation, its GUI is the Driver Station: enable Teleoperated.
Logs land in `/tmp/talos_logs/<session-id>/`. Deploying:
[docs/DEPLOYMENT.md](../docs/DEPLOYMENT.md).

## Add a subsystem

Follow [examples/superstructure/README.md](../examples/superstructure/README.md):
messages and their `topics` with `talos_messages`, a class built with `talos_cc_node`
or `talos_py_node` in `main_processor/<name>/`, then a `[[subsystems]]` entry in
`robot.toml` and `subsystems/<name>.toml` with
`node = "//2026-robot/main_processor/<name>:node"`, and the node in
`main_processor/BUILD`'s `robot_bundle`. If it takes teleop or auto targets,
read them with `talos::Target` / `talos.Target` and have each producer write
its own `talos::TargetLane` (`/<name>/target/<source>`).
