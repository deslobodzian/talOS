# 2026-robot

One robot, built on [talOS](../talOS). Two processors, one wire contract.

| | runs | built with |
|---|---|---|
| `main_processor/` | the talOS node graph — drivetrain, shooter, odometry, driver station | Bazel |
| `controller_processor/rio/` | the HAL that owns the actuators | GradleRIO / WPILib |

They communicate over the UDP framing in `talOS/protocol`, so the controller
processor is replaceable: a SystemCore or Jetson would be a new directory
beside `rio/` speaking the same frames, with no change to `main_processor/`.

## Configuration

`main_processor/configuration/robot.toml` is the single declaration of this
robot's hardware. It names every motor, sensor and subsystem, which talOS node
binary implements each subsystem, and the `sim_gateway` used in simulation.
`config_golden_test` pins that file against a reference config, so an
accidental edit to the hardware declaration fails the build.

Devices are addressed by **name**, never by logical id. The parser assigns ids
by sorting every device in the robot on (subsystem name, device name) and
numbering from one, so adding or renaming a device -- in any subsystem -- can
renumber every other one. `[subsystems.drivetrain.geometry]` names the drive,
steer and encoder of each swerve module, and `drivetrain/geometry.h` resolves
those names to ids at startup.

## Targets and arbitration

Every topic the drivetrain and shooter consume has exactly **one** writer:
`arbiter/`. Producers publish to their own topic and never coordinate; the
arbiter picks which one reaches the actuators, based on the match mode the FMS
reports on `/driver_station/state`.

```
/drivetrain/tgt/teleop --.
/drivetrain/tgt/auto ----+--> [arbiter] --> /drivetrain/tgt --> [drivetrain]
/driver_station/state --'
```

Two writers on one topic would resolve differently on replay than they did on
the field, which is why this is a rule and not a convention. Adding a producer
(an auto routine, a driver assist) is a new topic plus one line in the arbiter;
no subsystem node changes. On every mode change the arbiter emits one disabled
target, so no request outlives the mode that produced it. Anything it does not
recognise -- test mode, e-stop, disabled -- drives nothing.

## Teleop

Two nodes, split along "what the hardware said" versus "what we want it to do":

| | does | changes when |
|---|---|---|
| `driver_station/` | decodes the Driver Station packet and republishes it on `/driver_station/state` | the wire format does |
| `operator_interface/` | stick mapping, scaling, deadband, button bindings; publishes `/drivetrain/tgt/teleop` and `/shooter/tgt/teleop` | the game does |

`[subsystems.operator_interface]` holds the mapping policy: speed ceilings,
`deadband`, `shoot_button_mask`, and `field_oriented`. Field-oriented rotates
the request by the heading the drivetrain publishes on `/drivetrain/state`, and
falls back to robot-relative until a heading arrives.

Bind a new control by editing `operator_interface/` alone. Nothing there touches
the Driver Station wire format, and the subsystems never see a joystick.

Drive the whole chain without a Driver Station or a controller processor:

```
bazel build //talOS/bridge:hardware_node //2026-robot/main_processor/...

bazel-bin/2026-robot/main_processor/drivetrain/sim_gateway --duration-s 16 &
bazel-bin/talOS/bridge/hardware_node --duration-s 14 &
bazel-bin/2026-robot/main_processor/drivetrain/node --sim --duration-s 13 &
bazel-bin/2026-robot/main_processor/driver_station/node --duration-s 13 &
bazel-bin/2026-robot/main_processor/arbiter/node --duration-s 13 &

# Push the stick, then watch the modules respond.
bazel-bin/2026-robot/main_processor/driver_station/send_joystick --y 0.5 --rot 0.4 --duration-s 9 &
bazel-bin/2026-robot/main_processor/drivetrain/monitor --duration-s 4
```

`send_joystick` publishes the same wire encoding the Rio sends, so nothing
downstream can tell it apart from a real Driver Station. Add `--disabled` and
the modules stay put: the arbiter forwards nothing, so the same stick
deflection produces zero wheel travel.

## Running the robot against Studio

The WPILib simulation stands in for the RoboRIO, the launcher starts the node
graph, and the Studio bridge serves the telemetry. Three terminals:

```sh
# 1. Controller processor. The simulation GUI is the Driver Station: enable it
#    and pick Teleoperated there, and its joystick reaches the node graph.
cd 2026-robot/controller_processor/rio && ./gradlew simulateNative

# 2. Every node in robot.toml, plus the hardware bridge. No --start-sim-gateway:
#    the simulated Rio is the gateway.
bazel run //talOS/launcher:launcher -- --sim

# 3. Studio.
bazel build //studio/bridge:studio_bridge //studio:web_dist
bazel-bin/studio/bridge/studio_bridge --drop-newest-publisher \
    /talos_studio 127.0.0.1 5801 5800 bazel-bin/studio/dist
# http://localhost:5800
```

`telemetry/` is what makes the robot visible there: it turns `/odometry`,
`/drivetrain/state` and `/driver_station/state` into Studio frames on
`/talos_studio`. It is the only producer on that topic, which the bridge
requires -- see `studio/bridge/README.md` for why that topic must use
DROP_NEWEST and must never be an existing control-loop topic.

For headless runs, `TALOS_SIM_DS=1` enables teleop without the GUI, and
`send_joystick` supplies stick deflection.

### Heading in simulation

No simulation backend models a gyro: the Pigeon reports a valid, permanently
zero yaw, which reads exactly like a robot facing downfield and never turning.
So under `--sim` the drivetrain derives its heading by integrating the rotation
the modules are *measuring*, not the rotation that was requested -- the inverse
kinematics and the drive/steer device mapping still have to be right for the
robot to turn correctly. On real hardware the IMU is used and this is off, since
an integrated heading has no absolute reference and drifts.

Simulated wheels reach roughly a seventh of their commanded velocity, so both
translation and rotation run slow against the stick. That is the motor model in
`PhoenixBackend::UpdateSimulation`, not the kinematics: `drivetrain/omega` and
the integrated heading agree with each other, and both disagree with the
target.

## Adding a subsystem

Add a `[subsystems.<name>]` block with a `node = "//2026-robot/main_processor/<name>:node"`
target and its devices; the launcher discovers it from the config and the
hardware bridge starts tracking its requests. Subsystems that own no actuators
(the driver station, for instance) are input-only and get no request tracker.
