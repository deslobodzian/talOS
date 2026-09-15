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
reports on `/driver_station/state`. That rule, and the grammar every name here
follows, are normative in `talOS/NAMING.md`.

```
/drivetrain/target/teleop --.
/drivetrain/target/auto ----+--> [arbiter] --> /drivetrain/target --> [drivetrain]
/driver_station/state ------'
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
| `talos-driver_station/` (standalone module at the repo root; wire codec in `talOS/driver_station/`) | decodes the Driver Station packet and republishes it on `/driver_station/state` | the wire format does |
| `operator_interface/` | stick mapping, scaling, deadband, button bindings; publishes `/drivetrain/target/teleop` and `/shooter/target/teleop` | the game does |

`[subsystems.operator_interface]` holds the mapping policy: speed ceilings,
`deadband`, `shoot_button_mask`, and `field_oriented`. Field-oriented rotates
the request by the heading the drivetrain publishes on `/drivetrain/state`, and
falls back to robot-relative until a heading arrives.

Bind a new control by editing `operator_interface/` alone. Nothing there touches
the Driver Station wire format, and the subsystems never see a joystick.

Drive the whole chain without a Driver Station or a controller processor:

```
bazel build //talOS/bridge:hardware_node //2026-robot/main_processor/... @talos_driver_station//:node @talos_driver_station//:send_joystick

bazel-bin/2026-robot/main_processor/simulation/sim_gateway --duration-s 16 &
bazel-bin/talOS/bridge/hardware_node --config 2026-robot/main_processor/configuration/robot.toml --duration-s 14 &
bazel-bin/2026-robot/main_processor/drivetrain/node --sim --duration-s 13 &
bazel-bin/external/talos_driver_station+/node --duration-s 13 &
bazel-bin/2026-robot/main_processor/arbiter/node --duration-s 13 &

# Push the stick, then watch the modules respond.
bazel-bin/external/talos_driver_station+/send_joystick --y 0.5 --rot 0.4 --duration-s 9 &
bazel-bin/2026-robot/main_processor/drivetrain/monitor --duration-s 4
```

`send_joystick` publishes the same wire encoding the Rio sends, so nothing
downstream can tell it apart from a real Driver Station. Add `--disabled` and
the modules stay put: the arbiter forwards nothing, so the same stick
deflection produces zero wheel travel.

## Running the robot against Studio

The WPILib simulation stands in for the RoboRIO, the launcher starts the node
graph, and the Studio bridge serves the telemetry. One command from the
repository root starts all three and opens Studio:

```sh
./sim.sh
```

Then enable the robot and pick Teleoperated in the sim GUI, which is the Driver
Station: its joystick reaches the node graph. Ctrl-C stops everything.

```sh
./sim.sh --headless       # no sim GUI; teleop through TALOS_SIM_DS instead
./sim.sh --no-studio      # robot only, no bridge and no browser
./sim.sh --no-rio         # node graph only, against a simulation already up
./sim.sh --duration-s 30  # stop the node graph after 30 seconds
```

The ordering is not cosmetic. The simulated Rio is the sim gateway, so the
hardware bridge fails immediately if the node graph starts first; and the
Studio bridge refuses to start until the telemetry node is publishing, rather
than create an empty ring. The launcher owns the bridge (`[studio]` in
`robot.toml`, `--start-studio-bridge`, which `sim.sh` passes unless
`--no-studio`): it writes `graph.json` with the bridge declared in it, then
starts the bridge with that graph as `--declared`, so Studio can compare what
was declared against what actually registered.

Logs for a session land in `/tmp/talos_logs/<session-id>/`, with the three
process logs under `sim/`.

To drive one piece on its own -- debugging the bridge, or running the graph
under a debugger -- the underlying commands are:

```sh
# 1. Controller processor.
cd 2026-robot/controller_processor/rio && ./gradlew simulateNative

# 2. Every node in robot.toml, plus the hardware bridge. No --start-sim-gateway:
#    the simulated Rio is the gateway.
bazel run //talOS/launcher:launcher -- --config 2026-robot/main_processor/configuration/robot.toml --sim

# 3. Studio. Prefer the launcher's supervised bridge over a manual one, so
# the bridge stays a declared node: add --start-studio-bridge to the step-2
# launcher invocation (needs //studio:web_dist built). A bare manual bridge
# works for bridge debugging but is undeclared; pass the running session's
# graph as --declared to keep the declared-against-actual comparison:
#   bazel-bin/studio/bridge/studio_bridge --drop-newest-publisher \
#       /talos/telemetry 127.0.0.1 5801 5800 bazel-bin/studio/dist \
#       --declared /tmp/talos_logs/<session-id>/graph.json
# http://localhost:5800
```

`telemetry/` is what makes the robot visible there: it turns `/odometry/state`,
`/drivetrain/state` and `/driver_station/state` into Studio frames on
`/talos/telemetry`. It is the only producer on that topic, which the bridge
requires -- see `studio/bridge/README.md` for why that topic must use
DROP_NEWEST and must never be an existing control-loop topic.

For headless runs, `TALOS_SIM_DS=1` enables teleop without the GUI, and
`send_joystick` supplies stick deflection.

## Your first intake (beginner walkthrough)

Five steps from nothing to hardware, each one checkable. Every command below
was run against this tree; the two `bazel test` lines are the integration
gates that catch a broken step.

1. **Generate.** Scaffold a Python hardware node next to the checkout (any
   `--name` works; `my_intake` avoids colliding with the year intake):

   ```sh
   python3 tools/node_scaffold/main.py --name my_intake --lang python \
       --hardware --out ../my_intake
   ```

   The generated `packet.py` already speaks the canonical hardware wire
   format: `encode_command(..., motor_id, MODE_VELOCITY, slot, 30.0, ...)`
   produces a 1404-byte envelope the C++ decoder accepts (a 30 rps ask
   decodes back to exactly 30.0), and the generated hardware acceptance
   test injects hw state plus a target and checks the motor request.

2. **Configure the motor.** In the generated `subsystem.toml`, set the
   physical device under `[motors.*]`: `bus`/`can_id` to match the
   wiring, `inverted`/`brake` to match the mechanism, and the `slot0` gains
   for the onboard velocity PID (the year intake documents each knob in
   `2026-robot/main_processor/intake/subsystem.toml`, `[motors.roller]`).

3. **Simulate.** Standalone first, then in the graph:

   ```sh
   cd ../my_intake && bazel test //...          # generated node + hw test
   cd ../talOS && bazel test //2026-robot/main_processor/intake/...  # year intake
   ./sim.sh --sim-gateway --no-open --duration-s 45                  # full graph
   ```

4. **Inspect requested vs measured.** In Studio, `/intake/state` shows
   `requested_rps` (what the policy asked for), the measured roller
   velocity, and the inhibit reason naming the first failed gate
   (`gateway`, `disabled`, `stale`, ...). If requested moves but measured
   does not, the fault is in the motor/gains from step 2, not the policy:
   the student's only behavior knob is `command_for(target, feedback)` in
   `student_policy.py`, where positive asks run at `INTAKE_RPS`, negative
   at `EJECT_RPS`, and a broken beam holds stop against further intake
   while eject still runs (that is how a captured piece gets back out).
   Three worked examples with config, simulation, diagnostics, and
   commissioning notes live in
   `2026-robot/main_processor/intake/talos_nodes/intake/examples/`
   (roller intake, flywheel, position mechanism); run their tests with
   `bazel test //2026-robot/main_processor/intake:examples_test
   //2026-robot/main_processor/intake:student_api_test`.

5. **Deploy.** Follow "Real robot" below (companion install, controller
   address, `--remote` for one-off runs). Nothing in steps 1-4 changes:
   the same node binary and the same `subsystem.toml` ship.

## Real robot (companion + RoboRIO)

On the field the node graph runs on the companion computer and the controller
processor (RoboRIO) owns the actuators, joined by the UDP contract in
`talOS/protocol` (gateway port 5802, companion port 5803). Both ends must point
at each other:

1. **Controller processor.** Set `kCompanionAddress` in
   `controller_processor/rio/src/main/cpp/Robot.cpp` to the companion's static
   address (the example `10.56.87.11` fits team 5687), then deploy from the rio
   directory (`./gradlew deploy`). The RIO side needs no per-launch flags: on
   hardware it binds `0.0.0.0:5802` and sends to the companion address.
2. **Companion install.** From the repository root, build the launcher, the
   bridge and the node graph, then copy the binaries plus the configuration
   directory to the companion:

   ```sh
   bazel build //talOS/launcher:launcher //talOS/bridge:hardware_node //2026-robot/main_processor/...
   ```

   Ship `bazel-bin/talOS/launcher/launcher`,
   `bazel-bin/talOS/bridge/hardware_node`, the `bazel-bin/2026-robot/...`
   node binaries, and `2026-robot/main_processor/configuration/` (the manifest
   plus every `subsystems/*.toml` it references).
3. **Set the controller address.** In the companion's copy of `robot.toml`,
   set `[robot] controller_address` to the RIO address (e.g. `"10.56.87.2"`).
   The shipped default is loopback for simulation; a one-off run can instead
   pass `--remote 10.56.87.2` to the launcher, which wins over the config.
4. **Run.** On the companion, launch without `--sim` and enable from the real
   Driver Station:

   ```sh
   ./launcher --config <path-to>/robot.toml
   # or, without editing the config:
   ./launcher --config <path-to>/robot.toml --remote 10.56.87.2
   ```

   The launcher prints the resolved endpoint
   (`[launcher] Controller 10.56.87.2:5802 ...`) and starts `hardware_node`
   with an explicit `--remote/--remote-port/--local-port` triple, so what it
   reports is what the bridge dials. Commands stay refused until the hardware
   `commissioned` flag is set in the deployed config: match the manifest to
   the physical drivetrain and tune it first (see `talOS/hardware/README.md`),
   then commission explicitly.

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
