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
| `driver` | C++ | joystick 0 sticks to drivetrain teleop target lane (deadband, field-oriented) | `/hw/state/driver_station`, `/drivetrain/state` | `/drivetrain/target/teleop` |
| `teleop` (`//talOS/teleop:node`) | C++ | framework teleop node; maps joystick buttons and axes declared in `[[teleop.bindings]]` to mechanism target lanes | `/hw/state/driver_station` | `/shooter/target/teleop`, `/intake/target/teleop` |
| `pose_estimator` | Python | field pose from swerve odometry and camera observations; backend `kalman` (wpimath) or `factor_graph` (GTSAM); runs while disabled | `/drivetrain/state`, `/<camera>/state` per `cameras` | `/pose_estimator/state` |
| `telemetry` (`//talOS/telemetry:node`) | C++ | framework telemetry node; Studio frames; the pose from `[studio] pose_topic` | the state and status topics, `/hw/state/driver_station`, `/hw/status` | `/talos/telemetry` |
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

The deployable `//2026-robot/main_processor:robot_bundle` instead uses
`main_processor/configuration/robot_field.toml`: it includes `robot.toml`
and overrides only the controller address to the RoboRIO's `10.56.87.2`.
Simulation and tests retain `robot.toml`'s loopback address. For a different
controller, launch the bundle with `./run.sh --remote <ip>`; see deployment
instructions below.

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
