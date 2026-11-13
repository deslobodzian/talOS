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
| `driver` | C++ | joystick 0 sticks to drivetrain teleop target lane (deadband, gyro-relative driver frame) | `/hw/state/driver_station`, `/drivetrain/state` | `/drivetrain/target/teleop` |
| `autonomous` | C++ | preload spinup/feed, alliance-relative waypoint and collection | `/hw/state/driver_station`, `/pose_estimator/state`, `/shooter/state`, `/intake/state` | `/drivetrain/target/auto`, `/shooter/target/auto`, `/intake/target/auto`, `/autonomous/state` |
| `teleop` (`//talOS/teleop:node`) | C++ | framework teleop node; maps joystick buttons and axes declared in `[[teleop.bindings]]` to mechanism target lanes | `/hw/state/driver_station` | `/shooter/target/teleop`, `/intake/target/teleop` |
| `pose_estimator` | Python | field pose from swerve odometry and camera observations; backend `kalman` (wpimath) or `factor_graph` (GTSAM); runs while disabled | `/drivetrain/state`, `/<camera>/state` per `cameras` | `/pose_estimator/state` |
| `front_left_camera`, `front_right_camera`, `rear_left_camera`, `rear_right_camera` | C++ | independent AprilTag localization; capture rate from `camera.fps`, node/status tick from `period_us`; shared vision implementation; runs while disabled | `/sim/state` (optional, under `--sim`) | `/<camera>/state`, `/<camera>/status` |
| `telemetry` (`//talOS/telemetry:node`) | C++ | framework telemetry node; Studio frames; the pose from `[studio] pose_topic` | the state and status topics, `/hw/state/driver_station`, `/hw/status` | `/talos/telemetry` |
| `hardware_node` (`//talOS/bridge:hardware_node`) | C++ | UDP courier to the RoboRIO | `/hw/request/<node>` | `/hw/state`, `/hw/state/driver_station`, `/hw/status`, `/hw/command` |

Simulation stands in for the RoboRIO with `main_processor/simulation:sim_gateway`.

Camera calibration is in `main_processor/configuration/subsystems/<name>_camera.toml`,
layered over shared defaults in `subsystems/vision.toml`. The field layout is embedded
into each node binary, so deployed bundles require no vision runfiles.
Only `--sim` enables synthetic images (`rasterize = true`) or fast projection
(`rasterize = false`). Simulated ground truth comes from `/sim/state`,
published by `sim_gateway --physics`, never estimator output. Without `--sim`, cameras capture real images
on a bounded worker thread; unavailable devices report unhealthy status rather
than synthetic observations. Describing the graph never opens cameras.

Select autonomous on joystick 0 **POV right/left** while DS-connected and
disabled in any mode, including FMS pre-match disabled-autonomous (unused by
driver/teleop bindings). Release between
presses; a held POV at startup/reconnect is not a press. The configured order is
`preload`, `none`, `shoot_only`, `taxi` (default `preload`). Selection survives a
node restart within the launcher session; a new session or corrupt selection
file uses the default. Studio is read-only: `autonomous/state/routine_index`
reports the selected index, or the latched one from the routine's arm until a
new match (below); a selection made meanwhile applies from that match. The
telemetry dictionary labels it with `routines` (`[[subsystem.labels]]` in
`subsystems/telemetry.toml`), so viewers show the routine's name; keep
`routines` an ordered string array whose index is `routine_index`.
Enabled inputs never change it. Tune the list, default, POV binding and
`routine.<name>` action parameters in `subsystems/autonomous.toml`.

`none` releases all lanes throughout; `taxi` only drives to its waypoint;
`shoot_only` shoots the preload then holds. `preload` preserves the full sequence:
it waits for fresh shooter `at_speed`, then feeds for 500 ms at
-20 rps: negative roller demand is allowed through the intake beam interlock,
whereas positive collection stops when a piece breaks the beam, and is
neutral (`FEEDBACK_INVALID`) while the beam has no valid sample. It then stops
the shooter and drives toward the configured waypoint with bounded field-frame
position/heading feedback rotated into robot coordinates. The waypoint is in
blue-origin coordinates; Red rotates both translation and heading by pi about
the field centre, as the estimator does with its blue-origin starting pose. Invalid,
unlocalized or stale pose stops drive and collection immediately; stale intake
feedback inhibits collection. Reaching the waypoint releases all lanes until mode
exit. Feed counts only commanded feeding time and continuously requires matching
fresh shooter readiness. Spinup/feed share a wall-clock deadline; drive has its
own, neither extended by outages. Disable, teleop or DS loss releases all lanes.

A routine runs once per match. A DS outage, or a disable and re-enable in
autonomous within the same match (a new mode generation), resumes the step it
was on, so no action fires twice in a match: a resumed feed runs only its
remaining time, a resumed drive re-plans from the current pose, and both
deadlines keep counting through the pause. Leaving enabled autonomous counts as
another match, and resets the routine to its first step, once any of these
holds after the routine armed: the DS, attached, reports teleop or test (FMS
runs teleop after auto); the FMS match type, number, replay or event differs
from the routine's, both present (a routine armed before match data arrives
adopts the first it sees, and missing data never counts); or
`autonomous_period_us` (20 s, the game's autonomous period) has passed.

The same holds across a restart of the node or of the launcher. The node
records the armed routine's progress (match, arm time, step, feed time fed,
spinup and drive start) in the shared-memory object `/autonomous/progress`
(named through `ShmObjectName`, so `/autonomous.progress` in production). It
is not a topic, so the launcher, which unlinks only declared topics' segments,
leaves it in place; it lasts until reboot, whatever `--output-dir` a new
session uses, and storing into it never waits on a disk. Times are on the
boot's monotonic clock (Linux `CLOCK_MONOTONIC`, macOS `mach_continuous_time`)
and the record names its boot (Linux `boot_id`, macOS `kern.bootsessionuuid`).
A restarted node compares the record with its first Driver Station packet by
the rules above: from the same match, it resumes the recorded step as after a
disable; with no record, or one from another match or boot, it starts
INHIBITED and must observe non-auto/disabled before a new auto can arm. While
feeding, the record counts each commanded tick as fed for `command_timeout_us`,
since a killed node's last feed lane drives the intake until it goes stale; a
restarted feed therefore runs only what remains of the 500 ms even in that
worst case, and may stop up to that much short of it. A record it cannot read
(both of its two checksummed copies damaged, or naming routines the current
list lacks) holds the routine in HOLD until another match: it may hide a shot
already fired. Without a session directory (tests, `--describe`, `--replay`)
nothing is recorded; sim harnesses that run under their own
`TALOS_SHM_NAMESPACE` remove the record when done
(`main_processor/simulation/autonomous_progress.py`).

Teleop joystick 0: hold button **1** for the 60 rps flywheel, **2** to collect
at +30 rps (stops at the intake beam), and **3** to feed at -20 rps past the
beam. Hold **1+3** to score; button 3 alone cannot feed a stopped shooter.
If 2 and 3 are held together, collection wins (first binding takes precedence).
Every negative intake target, autonomous or teleop, requires fresh shooter
`at_speed` with a positive shooter target on every tick; otherwise the roller
is neutral and `/intake/state.inhibit` reports `SHOOTER_NOT_READY`.
`requested_velocity_rps` retains the withheld feed ask for diagnosis.
Press **4** to re-zero driver-forward to the robot's current heading
(`rezero_button_id` in `subsystems/driver.toml`).
Release the buttons and enable with controls neutral to rearm after a disable
or link loss.

## Config

`main_processor/configuration/robot.toml` is the robot manifest.
See [Configuration](../ARCHITECTURE.md#configuration) and
[Targets](../ARCHITECTURE.md#targets) for the shared format and target policy.
Devices are referred to by name; ids are assigned by sorting
(subsystem, device).

The deployable `//2026-robot/main_processor:robot_bundle` instead uses
`main_processor/configuration/robot_field.toml`: it includes `robot.toml`
and selects the RoboRIO address `10.56.87.2`.
Simulation and tests retain `robot.toml`'s loopback address. For a different
controller, launch the bundle with `./run.sh --remote <ip>`; see deployment
instructions below.

## Run in simulation

From the repository root:

```sh
./sim.sh --sim-gateway --no-studio   # nodes + lightweight simulator
./sim.sh                             # WPILib simulation (needs WPILib) + Studio
./sim.sh --headless                  # WPILib without its GUI, teleop enabled
./sim.sh --headless --no-studio --mode-schedule teleop:60,auto:20,disabled:5
                                     # scripted Driver Station modes
./sim.sh --headless --no-studio --mode-schedule teleop:30,disabled:5 \
  --joystick-schedule '0:axes=0,0,0;buttons=0;pov=-1|3:axes=0,-0.5,0;buttons=0;pov=-1'
                                     # ... and a scripted joystick 0
python3 tools/test_drivetrain.py     # drivetrain smoke test with log replay
```

With the WPILib simulation, its GUI is the Driver Station: enable Teleoperated.
Headless, the RoboRIO program's desktop-only stand-in
(`controller_processor/rio/src/main/include/SimDriverStation.h`) names the blue
alliance (`--alliance red` for red), enables teleop once the gateway is
configured, or runs `--mode-schedule` phases from that
moment (the lightweight simulator's schedule runs from its start).
`--joystick-schedule` scripts joystick 0 on the same clock, in the lightweight
simulator's syntax ([simulation/README.md](main_processor/simulation/README.md#scripted-joystick-and-fault-schedules));
headless it goes through WPILib's simulated Driver Station, so the gateway
reads it as it would a real stick. The stick keeps its scripted value when the
schedule disables; the robot goes neutral because it is disabled.
Logs land in `/tmp/talos_logs/<session-id>/`. Deploying:
[docs/DEPLOYMENT.md](../docs/DEPLOYMENT.md).

## Add a subsystem

Follow [examples/superstructure/README.md](../examples/superstructure/README.md)
and [Where a new thing goes](../ARCHITECTURE.md#where-a-new-thing-goes).
The bundle reads generated `NODES` and `CONFIG_FILES`; do not add each
subsystem to `main_processor/BUILD` by hand.
