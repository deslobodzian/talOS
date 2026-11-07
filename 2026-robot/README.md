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
| `intake` | Python | roller, beam break and shooter-ready feed permit | `/intake/target/teleop`, `/intake/target/auto`, `/shooter/state` | `/intake/state` |
| `driver` | C++ | joystick 0 sticks to drivetrain teleop target lane (deadband, field-oriented) | `/hw/state/driver_station`, `/drivetrain/state` | `/drivetrain/target/teleop` |
| `autonomous` | C++ | preload spinup/feed, alliance-relative waypoint and collection | `/hw/state/driver_station`, `/pose_estimator/state`, `/shooter/state`, `/intake/state` | `/drivetrain/target/auto`, `/shooter/target/auto`, `/intake/target/auto` |
| `teleop` (`//talOS/teleop:node`) | C++ | framework teleop node; maps joystick buttons and axes declared in `[[teleop.bindings]]` to mechanism target lanes | `/hw/state/driver_station` | `/shooter/target/teleop`, `/intake/target/teleop` |
| `pose_estimator` | Python | field pose from swerve odometry and camera observations; backend `kalman` (wpimath) or `factor_graph` (GTSAM); runs while disabled | `/drivetrain/state`, `/<camera>/state` per `cameras` | `/pose_estimator/state` |
| `front_left_camera`, `front_right_camera`, `rear_left_camera`, `rear_right_camera` | C++ | independent AprilTag localization at 20 Hz; shared vision implementation; runs while disabled | `/drivetrain/state` (optional simulated truth) | `/<camera>/state`, `/<camera>/status` |
| `telemetry` (`//talOS/telemetry:node`) | C++ | framework telemetry node; Studio frames; the pose from `[studio] pose_topic` | the state and status topics, `/hw/state/driver_station`, `/hw/status` | `/talos/telemetry` |
| `hardware_node` (`//talOS/bridge`) | C++ | UDP courier to the RoboRIO | `/hw/request/<node>` | `/hw/state`, `/hw/status` |

Simulation stands in for the RoboRIO with `main_processor/simulation:sim_gateway`.

Camera calibration is in `configuration/subsystems/<name>_camera.toml`, layered
over shared defaults in `subsystems/vision.toml`. The field layout is embedded
into each node binary, so deployed bundles require no vision runfiles.
Only `--sim` enables synthetic images (`rasterize = true`) or fast projection
(`rasterize = false`). Simulated ground truth integrates measured drivetrain
motion, never estimator output. Without `--sim`, cameras capture real images
on a bounded worker thread; unavailable devices report unhealthy status rather
than synthetic observations. Describing the graph never opens cameras.

Select autonomous on joystick 0 **POV right/left** while DS-connected and
disabled in any mode, including FMS pre-match disabled-autonomous (unused by
driver/teleop bindings). Release between
presses; a held POV at startup/reconnect is not a press. The configured order is
`preload`, `none`, `shoot_only`, `taxi` (default `preload`). Selection survives a
node restart within the launcher session; a new session or corrupt selection
file uses the default. Studio is read-only: `autonomous/state/routine_index`
reports the selected index while disabled and the latched index during auto.
Enabled inputs never change it. Tune the list, default, POV binding and
`routine.<name>` action parameters in `subsystems/autonomous.toml`.

`none` releases all lanes throughout; `taxi` only drives to its waypoint;
`shoot_only` shoots the preload then holds. `preload` preserves the full sequence:
it waits for fresh shooter `at_speed`, then feeds for 500 ms at
-20 rps: negative roller demand is allowed through the intake beam interlock,
whereas positive collection stops when a piece breaks the beam. It then stops
the shooter and drives toward the configured waypoint with bounded field-frame
position/heading feedback rotated into robot coordinates. The waypoint is in
blue-origin coordinates; Red rotates both translation and heading by pi about
the field centre (configure the estimator's starting pose consistently). Invalid,
unlocalized or stale pose stops drive and collection immediately; stale intake
feedback inhibits collection. Reaching the waypoint releases all lanes until mode
exit. Feed counts only commanded feeding time and continuously requires matching
fresh shooter readiness. Spinup/feed share a wall-clock deadline; drive has its
own, neither extended by outages. Disable, teleop or DS loss releases all lanes.
A DS outage resumes the same generation without re-firing; a restarted node
starts INHIBITED and must observe non-auto/disabled before a new auto can arm.

Teleop joystick 0: hold button **1** for the 60 rps flywheel, **2** to collect
at +30 rps (stops at the intake beam), and **3** to feed at -20 rps past the
beam. Hold **1+3** to score; button 3 alone cannot feed a stopped shooter.
If 2 and 3 are held together, collection wins (first binding takes precedence).
Every negative intake target, autonomous or teleop, requires fresh shooter
`at_speed` with a positive shooter target on every tick; otherwise the roller
is neutral and `/intake/state.inhibit` reports `SHOOTER_NOT_READY`.
`requested_velocity_rps` retains the withheld feed ask for diagnosis.
Release the buttons and enable with controls neutral to rearm after a disable
or link loss.

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
