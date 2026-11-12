# Glossary

Terms in the order a newcomer meets them. Each links to the section that
defines it; the code and [NAMING.md](../talOS/NAMING.md) are the authority
when a sentence here is thinner than they are. A worked example that uses most
of them is [FIRST_MECHANISM.md](FIRST_MECHANISM.md).

## The robot

**Main processor.** The Linux coprocessor (a laptop in simulation) that runs
every node. [Two processors](../ARCHITECTURE.md#two-processors).

**Controller processor, RoboRIO.** The robot controller. It owns motors,
sensors and the Driver Station link and runs no robot logic.
[Two processors](../ARCHITECTURE.md#two-processors),
[RoboRIO](../talOS/hardware/README.md#roborio).

**Driver Station (DS).** The laptop software that enables the robot, picks its
mode and carries joystick input. Nodes read it through `talos::DriverStation`.
[Hardware API](NODE_DEVELOPMENT_GUIDE.md#hardware).

**Mode.** The Driver Station's current state: disabled, autonomous, teleop or
test. A target policy lists lanes per mode.
[Targets](../ARCHITECTURE.md#targets).

**2026-robot.** The project's simulation testbed robot, a sibling of the
platform in `talOS/`, with its own nodes and config.
[Where a new thing goes](../ARCHITECTURE.md#where-a-new-thing-goes).

## Nodes and messages

**Node.** One program, C++ or Python, that does one job on the main processor
and talks to others only through topics.
[Node API](NODE_DEVELOPMENT_GUIDE.md#node),
[anatomy of a node](METHODOLOGY.md#2-anatomy-of-a-node).

**Subsystem.** A node together with its configuration: a `[[subsystems]]` entry
in `robot.toml` and a subsystem file. Its name is the node name.
[Configuration](../ARCHITECTURE.md#configuration),
[names of nodes](../talOS/NAMING.md#nodes).

**Topic.** A named shared-memory slot holding the latest message of one fixed
size. Names are `/owner/role[/qualifier]`.
[Topics and messages](../ARCHITECTURE.md#topics-and-messages),
[names of topics](../talOS/NAMING.md#topics).

**Message.** A FlatBuffers struct (fixed size, no strings or vectors) declared
in a `.fbs` file. Every number field carries a unit suffix such as `_rps`, `_m`
or `_us`. [Topics and messages](../ARCHITECTURE.md#topics-and-messages),
[units](METHODOLOGY.md#6-units).

**Role.** The second segment of a topic name (`state`, `status`, `event`,
`telemetry`, `target`, `request`, `command`); it fixes who may write the topic.
[Names of topics](../talOS/NAMING.md#topics).

**Schema hash.** The 64-bit identity of a message type. A topic attach with a
different hash fails. [Topics and messages](../ARCHITECTURE.md#topics-and-messages).

**RTMS.** The shared-memory topic layer (`talOS/rtms`) under every topic; one
writer, up to eight declared readers per topic.
[Layers](../ARCHITECTURE.md#layers),
[names](../talOS/NAMING.md#topics).

**Fresh.** A subscription value younger than `[robot] command_timeout_us`,
counted from the writer's publication time.
[Node API](NODE_DEVELOPMENT_GUIDE.md#node),
[the tick](../ARCHITECTURE.md#the-tick).

## Targets: what a subsystem is asked to do

**Target lane.** The topic `/<subsystem>/target/<source>`, one per producer.
[Targets](../ARCHITECTURE.md#targets).

**Source.** The last segment of a lane (`teleop`, `auto`, `align`, ...): who
produces it. The policy lists sources per mode.
[Targets](../ARCHITECTURE.md#targets).

**Target policy.** `[robot.targets]` and a subsystem's `[targets]`: which
sources count in each mode, highest priority first.
[Targets](../ARCHITECTURE.md#targets).

**`TargetLane`, `Target`.** A producer writes its lane with `TargetLane`
(`send`, or `release()` when it has nothing to say); a subsystem reads with
`Target::get()`, which returns the first live lane or null.
[Node guide: targets](NODE_DEVELOPMENT_GUIDE.md#targets).

**Generation.** The time the Driver Station entered its current mode
(`mode_since_us`). A target stamped with an earlier generation never actuates.
[Targets](../ARCHITECTURE.md#targets).

**Operator arming.** `OperatorArming`: a joystick producer's lane sends nothing
until every input has been seen neutral, so a control held through a disable
does not drive.
[Targets](../ARCHITECTURE.md#targets).

**`robot_enabled(node)`.** True while the robot is enabled, with a hardware
state younger than `command_timeout_us` and no fault. Every motor is neutral
while it is false. [The tick](../ARCHITECTURE.md#the-tick).

**Neutral.** A motor with no drive: coasts or brakes per its config. What a
motor gets when it is not set, or when anything gating it fails.
[The tick](../ARCHITECTURE.md#the-tick).

**Teleop bindings.** `[[teleop.bindings]]` in a subsystem file: a Driver
Station button, axis or hat mapped to fields of that subsystem's teleop lane,
with no code. [Binding joystick buttons](NODE_DEVELOPMENT_GUIDE.md#binding-joystick-buttons-and-axes-via-toml).

## Time

**Tick.** One run of a node's `update()`, every `period_us`. After it, the
framework sends the node's hardware request.
[The tick](../ARCHITECTURE.md#the-tick).

**Drain.** A node reading the messages waiting on its topics. It drains
before each of its timers. [Targets](../ARCHITECTURE.md#targets).

**Wake.** A publication that rouses a sleeping node to drain. `kOnPublish` (the
default for a callback) wakes the node; `kAtDrain` waits for the next drain.
[When messages arrive](NODE_DEVELOPMENT_GUIDE.md#when-messages-arrive).

**Event loop.** The per-node loop that sleeps, drains and runs timers
(`talOS/events`). [Layers](../ARCHITECTURE.md#layers).

**Tier (loop tier).** Realtime priority class of a node's loop: the bridge,
control nodes (those that own an output or write a lane), then observers.
[The targets section's scheduling paragraph](../ARCHITECTURE.md#targets),
[realtime scheduling](NODE_DEVELOPMENT_GUIDE.md#realtime-scheduling).

**Tier (test tier).** What a test may touch: unit, component, contract,
integration, e2e or soak. [Tiers](TESTING.md#2-tiers-by-what-the-test-may-touch).

## Hardware link

**Bridge.** `//talOS/bridge:hardware_node`: the courier between the nodes and
the RoboRIO. It merges each node's `/hw/request/<node>`, sends commands over
UDP and publishes `/hw/state`, `/hw/state/driver_station` and `/hw/status`.
[Two processors](../ARCHITECTURE.md#two-processors).

**Gateway.** The program that owns the hardware on the controller side: the
RoboRIO program, or the simulator in its place.
[Hardware gateway](../talOS/hardware/README.md).

**Hardware request.** `/hw/request/<node>`: one node's motor, digital and PWM
demands for a tick. Only that node's devices layer writes it.
[The tick](../ARCHITECTURE.md#the-tick),
[names](../talOS/NAMING.md#topics).

**Owner timeout.** How long the bridge keeps an actuator node's last request
once it goes silent: `max(2 x period_us, command_timeout_us / 2)`, never more
than `command_timeout_us`. [The tick](../ARCHITECTURE.md#the-tick).

**`command_timeout_us`.** The age after which hardware state, Driver Station
packets and targets stop counting; the RoboRIO neutralizes outputs within it
of the state a command was built from.
[The tick](../ARCHITECTURE.md#the-tick).

**Boot id, epoch.** Counters in the gateway's state. A new boot (a RoboRIO
restart) invalidates sample times and generations; the control epoch advances
at enable transitions, timeouts and stops, so no earlier request actuates.
[Runtime contract](../talOS/hardware/README.md#runtime-contract).

**Commissioned.** `[robot] commissioned = true`: a person has checked the
physical CAN ids at the robot. Until then the RoboRIO refuses commands. Never
set by code or agents. [Bring-up](BRINGUP.md#human-commissioning),
[RULES.md](../RULES.md) rule 4.

**Rezero.** `Motor::rezero`: a software offset that makes a motor's position
read a known value (a limit switch, a hard stop). Position demands are neutral
until one holds, unless the motor's position frame is absolute.
[Rezero and position frames](NODE_DEVELOPMENT_GUIDE.md#rezero-and-position-frames).

## Configuration and build

**`robot.toml`.** A robot's top-level config: timing, controller address,
target policy and the `[[subsystems]]` list.
[Configuration](../ARCHITECTURE.md#configuration).

**Subsystem file.** `configuration/subsystems/<name>.toml`: the node's label,
period, devices (`[motors.*]`) and parameters.
[Configuration](../ARCHITECTURE.md#configuration).

**Parameter.** A subsystem key the framework does not own; the node reads it
with `node.param`. The launcher refuses a key no node read.
[Parameters](NODE_DEVELOPMENT_GUIDE.md#parameters).

**Manifest.** `robot_manifest.bzl`: the generated list of a config's node
labels and files, which Bazel reads because it cannot read TOML.
Regenerated by `//tools:sync_manifest`.
[Configuration](../ARCHITECTURE.md#configuration).

**Bazel, target, label.** The build tool; a target (`//pkg:name`) is
something it builds or runs; a label is its name. `BUILD` files declare them.
[Install](INSTALL.md#what-bazel-provides-vs-what-the-host-needs).

**Scaffold.** `bazel run //tools:new_subsystem`: writes a node package, its
subsystem file and manifest entry. `bazel run //tools:new_robot` writes a
whole robot around its first one.
[Where a new thing goes](../ARCHITECTURE.md#where-a-new-thing-goes).

**Bundle.** One directory holding every node binary and config for a robot,
built with `talos_bundle` and installed on the main processor.
[Deployment](DEPLOYMENT.md#build-a-bundle).

## Running and observing

**Launcher.** Spawns and supervises a config's nodes, and lints the graph
first. [Two phases](../talOS/launcher/README.md#two-phases).

**Describe.** `--describe`: a node binary builds itself, prints the topics and
parameters it declares, and exits. The launcher builds its graph from these.
`--describe-only` stops after that check.
[Two phases](../talOS/launcher/README.md#two-phases),
[describe format](../talOS/introspection/README.md#describe).

**Graph (lint).** Nodes and the topics between them; the launcher refuses
a graph that breaks a naming or graph rule.
[Names](../talOS/NAMING.md#topics).

**Session.** One launcher run: `<output-dir>/<session-id>/` with each node's
log, output, `graph.json` and `manifest.json`.
[Session manifest](../talOS/launcher/README.md#session-manifest).

**Registry.** The live shared-memory table of nodes, endpoints, heartbeats and
selected targets, which the launcher and Studio read.
[Live node registry](../talOS/introspection/README.md).

**Dispatch log (`.tlog`).** A node's recording of every send, read and
message, read with `log_dump`.
[Dispatch logs](DATA_FLOW.md#dispatch-logs),
[filtering and exporting logs](../talOS/launcher/README.md#filtering-and-exporting-logs).

**Replay.** Feeding a recorded session's inputs back into one node.
[Replaying a session](../talOS/launcher/README.md#replaying-a-session-into-one-node).

**Studio.** talOS's own browser viewer (`studio/`): live topics, the system
view, session panels. [Studio](../studio/README.md).

## Simulation

**`Sim`.** `talos::testing::Sim`: runs nodes in a test on one simulated clock
with in-memory topics. No process, no robot.
[Testing](NODE_DEVELOPMENT_GUIDE.md#testing).

**`SimHardware`.** `talos::testing::SimHardware`: inside a `Sim`, stands in for
the bridge and the RoboRIO (set a motor's feedback, the Driver Station mode).
[Testing](NODE_DEVELOPMENT_GUIDE.md#testing).

**Sim gateway.** The lightweight simulator process
(`//2026-robot/main_processor/simulation:sim_gateway`) that plays the RoboRIO
for a running graph; `./sim.sh --sim-gateway`. Its backend is ideal (motors
reach demands at once) or physics (`--physics`).
[Lightweight simulation gateway](../talOS/launcher/README.md#lightweight-simulation-gateway),
[backends](../talOS/hardware/README.md#simulation-backends-ideal-vs-physics-backed).

**Node test matrix.** The cases every node's tests cover (disabled, stale,
released, ...). [Node test matrix](TESTING.md#7-the-node-test-matrix).

**REQ ID.** `REQ-<AREA>-<n>`: a numbered requirement in
[REQUIREMENTS.md](REQUIREMENTS.md) that a test or change cites.
