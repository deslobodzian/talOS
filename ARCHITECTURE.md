# Architecture

## Two processors

The **main processor** (a coprocessor running Linux, or a laptop in
simulation) runs every node. The **controller processor** (the RoboRIO,
`2026-robot/controller_processor/rio`) is only the hardware layer: it owns the
motors, sensors and the Driver Station link and runs no robot logic.

They talk over UDP (`talOS/protocol`; gateway port 5802, main-processor port
5803). On the main processor `//talOS/bridge:hardware_node` is the courier: it
collects each node's `/hw/request/<node>`, sends the commands to the RoboRIO,
and publishes what comes back on `/hw/state` and `/hw/state/driver_station`.
In simulation `//2026-robot/main_processor/simulation:sim_gateway` (or the
RoboRIO program under WPILib simulation) takes the RoboRIO's place.

## Layers

```
robot code        2026-robot/main_processor/*, examples/*
talOS/devices     Motor, CANcoder, Gyro, DigitalInput, ..., robot_enabled, Target, TargetLane, testing::SimHardware
talOS/node        Node, Subscription, Publisher, StateMachine, TALOS_NODE, testing::Sim
talOS/python      the same API in Python, ctypes over talOS/node_api (libtalos_node.so)
talOS/events      event loop, dispatch log, replay
talOS/rtms        shared-memory topics, on talOS/memory and talOS/ipc
```

Allowed dependencies:

- Each layer depends only on layers below it. `talOS/node` knows nothing about
  hardware; `talOS/devices` is built on `Node`'s public API
  (`extension<T>()`, `after_update()`).
- `talOS/` never depends on a robot, an example or `studio/`.
- Python reaches talOS only through the C ABI in `talOS/node_api`; it does not
  reimplement a wire format, a memory layout, a state machine, or a C++ node
  (C++ nodes exposed to Python use `TALOS_EXPORT_NODE` and `talos.cpp_node`).
- Adding a node in one language never requires modifying code in another
  language: cross-cutting robot wiring (target policies, `operator_interface`,
  `telemetry`) is driven by TOML configuration.
- Nodes share only messages and topic names (`talos_messages`), never each
  other's classes.

Beside the stack:

| Package | Role |
|---|---|
| `talOS/configuration` | parses `robot.toml` and the subsystem files |
| `talOS/hardware` | hardware config, request/state messages, the gateway the RoboRIO and sim run |
| `talOS/protocol` | frame and UDP wire format |
| `talOS/bridge` | `:hardware_node`; publishes link health and the gateway-to-node clock mapping (`:clock_offset`, read by `HardwareLink::sample_time()`) on `/hw/status` (`bridge::BridgeStatus`) |
| `talOS/driver_station` | Driver Station codec |
| `talOS/launcher` | spawns and supervises the nodes in a config (restarts crashed nodes up to `[robot] max_restarts`); `--describe-only` lints the graph |
| `talOS/introspection` | `--describe`, the naming rules (`names.h`), the live registry Studio reads |
| `talOS/process` | stop handlers and `ConfigureRealtime` for node binaries |
| `talOS/msgs` | shared geometry and control message structs |
| `talOS/telemetry` | the Studio telemetry frame schema and byte-topic wire format |
| `talOS/schema` | `talos::Descriptor<M>` (`.bfbs` bytes, name, schema hash), `talos::schema::struct_hash`, the unit-suffix table; Python `talos_schema` and upstream `reflection` classes |
| `talOS/build/contract.bzl` | `talos_cc_node`, `talos_py_node`, `talos_messages`, `talos_bundle`, `talos_graph_test` (both fed by a config's generated `robot_manifest.bzl`, `tools/sync_manifest.py`) |

## The tick

A node's `update()` runs every `period_us` (the subsystem's, else `[robot]`).
After each tick the devices layer flushes one hardware request for the node:

- a motor not set during that tick is sent neutral;
- every motor is neutral while `robot_enabled(node)` is false (no accepted
  hardware state, gateway not enabled, a fault, or a state older than
  `[robot] command_timeout_us`);
- digital and PWM outputs set during the tick are sent as set.

A subscription's `fresh()` means a value younger than `command_timeout_us`, so
a node that stops hearing its target goes neutral on its own.

## Targets

What a subsystem is asked to do arrives on target lanes,
`/<sub>/target/<source>`, one per producer (`teleop`, `auto`, `align`, ...).
Each carries a `TargetEnvelope<M>`: a 16-byte header (the producer's Driver
Station mode generation, `active`), then the producer's message. A producer
writes its lane with `talos::TargetLane<M>` (`send`, or `release()` when it
has nothing to say). The consumer resolves its own lanes with
`talos::Target<M>`: `get()` returns the first lane, in the priority order the
config gives for the current Driver Station mode, that is active, fresh and
stamped with the consumer's current mode generation, and only while
`robot_enabled(node)`; otherwise null, and the node goes neutral. The policy
is `[robot.targets]` (defaults: teleop `["teleop"]`, autonomous `["auto"]`,
test none) with a per-subsystem `[targets]` override. No process sits between
a producer and its subsystem, so a subsystem's safety depends only on its own
process and the bridge. Each `get()` records the selected source in the node's
live registry row, which is how Studio's system view shows who drives each
subsystem.

The mode generation is what keeps a target issued under an earlier enable or
mode from ever actuating. The RoboRIO program (and `SimHardware`,
`send_joystick`) stamps each Driver Station packet with `mode_since_us`, the
sample time of the first packet in the current mode, via one
`driver_station::ModeTracker`; disabling, re-enabling, switching mode,
e-stopping or re-attaching starts a new generation, and the bridge forwards it
unchanged. `TargetLane::send` stamps the generation its own node last drained
(0 while that view is stale, disabled or in no mode); `Target::get` accepts a
lane only when its stamp is nonzero and equals the generation the consumer
last drained. Receipt time cannot do this: a target queued before a mode
change and the packet that changes the mode can be drained together, at one
instant. A Driver Station link that drops and returns in the same mode keeps
its generation, so the producer drives again once its lane is fresh.

Messages reach a node when its loop drains its topics. A node without message
callbacks drains right before each of its own timers (`update()`, `every()`)
and otherwise sleeps; a node with a `subscribe` callback or a Driver Station
listener, or with no timer, drains every 1 ms. Ages count from the drain, so
for the first kind a message can be up to one period older than `age()`
reports, and a silent target is judged stale at most one period after
`command_timeout_us`. The simulated loop uses the same rule, and replay
reproduces either from the log. Off-simulation the loop thread runs under
`SCHED_FIFO` (Linux) or the Mach time-constraint policy (macOS) with memory
locked, best effort (`talOS/process`).

## Topics and messages

- A topic is a shared-memory slot holding the latest message of one fixed size.
- Messages are FlatBuffers **structs** (no tables, strings or vectors), declared
  in a `.fbs` per package and built with `talos_messages`, which generates C++
  and Python, a `.bfbs` schema and a schema descriptor per struct, and fails
  the build on a numeric field without a unit suffix.
- Every user topic's slot is 8-byte aligned, so a C++ and a Python endpoint
  agree on the layout from the size alone.
- The FlatBuffers namespace is the short subsystem name (`shooter`, `intake`,
  `drivetrain`); C++ includes `<package>/<file>_generated.h`, Python imports
  `<namespace>.<Struct>`.
- Topic names are declared once, in the package's `topics.h`, and follow
  [talOS/NAMING.md](talOS/NAMING.md).
- One writer per topic. Several producers of a target each write their own
  lane, and the consumer picks one (see "Targets").
- The launcher refuses to start a graph that breaks a naming or graph rule
  (`--allow-graph-errors` overrides).

## Configuration

```
configuration/robot.toml        [robot] period, timeouts, controller address
                                [robot.targets] target lanes per Driver Station mode
                                [studio] viewer bridge
                                [[subsystems]] name + path, one per subsystem
configuration/subsystems/*.toml [subsystem] name, node = "//...:node", period_us,
                                custom keys (node.param), [motors.*] and other devices,
                                [targets] lane priorities per mode
```

A manifest entry may also set `node = "//...:target"`, which overrides the
subsystem file's `node`, so one set of subsystem files can serve two manifests
(see `examples/superstructure/config/`). A single-file `robot.toml` with
`[subsystems.<name>]` tables also works; most test configs use it. Devices are
addressed by name.
Example configs carry placeholder CAN ids; the robot's config carries the
physical ones and sets `commissioned = true` only after checking them.

`[[subsystems]]` is the only list of a robot's nodes. Bazel cannot read TOML,
so `bazel run //tools:sync_manifest -- <dir>/robot.toml` writes
`<dir>/robot_manifest.bzl` (`<stem>_manifest.bzl` for any other config name):
`NODES`, every node label the config launches, and `CONFIG_FILES`, every file
it reads besides itself (`include`, `path`, `defaults`). `talos_bundle` and
`talos_graph_test` take those constants; no BUILD file lists nodes or
subsystem files. Each config package has a `manifest_sync_test`
(`tools/sync_manifest.bzl`) that fails, printing the regenerate command, when
the config changes and the manifest does not.

## Where a new thing goes

| New thing | Where |
|---|---|
| a mechanism | a node package under `<robot>/main_processor/<name>/`, plus a subsystem file and a `[[subsystems]]` entry, then regenerate the manifest; `bazel run //tools:new_subsystem -- <name> --lang cpp\|py` does all of it (walkthrough in [examples/superstructure](examples/superstructure/README.md)) |
| a mechanism in its own repository | `bazel run //tools:new_subsystem -- <name> --lang cpp\|py --standalone DIR` writes a Bzlmod repo (overriding `@talos` with this checkout); the robot adds `bazel_dep` + `local_path_override`, a subsystem file naming `@<name>//:node`, and its `[[subsystems]]` entry |
| a message or topic | the owning package's `.fbs` and `topics.h` |
| a producer of an existing target | a `talos::TargetLane` on `/<sub>/target/<source>`, and `<source>` in that subsystem's policy (`[robot.targets]` or its `[targets]`); the launcher refuses a lane no mode lists |
| a device type | `talOS/hardware` (config + wire), `talOS/devices`, `talOS/python/talos/devices.py`, and the RoboRIO program |
| a node-API feature | `talOS/node`, mirrored in `talOS/node_api` and `talOS/python/talos` |
| a robot | a sibling of `2026-robot/` |
| a controller processor | a sibling of `controller_processor/rio/` speaking `talOS/protocol` |
| a viewer tab | `studio/` |
