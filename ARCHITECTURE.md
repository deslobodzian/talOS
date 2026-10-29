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
talOS/devices     Motor, CANcoder, Gyro, DigitalInput, ..., robot_enabled, testing::SimHardware
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
  reimplement a wire format or a memory layout.
- Nodes share only messages and topic names (`talos_messages`), never each
  other's classes.

Beside the stack:

| Package | Role |
|---|---|
| `talOS/configuration` | parses `robot.toml` and the subsystem files |
| `talOS/hardware` | hardware config, request/state messages, the gateway the RoboRIO and sim run |
| `talOS/protocol` | frame and UDP wire format |
| `talOS/bridge` | `:hardware_node` |
| `talOS/driver_station` | Driver Station codec and `:node` |
| `talOS/launcher` | spawns and supervises the nodes in a config; `--describe-only` lints the graph |
| `talOS/introspection` | `--describe`, the naming rules (`names.h`), the live registry Studio reads |
| `talOS/process` | `InstallStopHandlers`: SIGINT/SIGTERM handling for node binaries |
| `talOS/build/contract.bzl` | `talos_cc_node`, `talos_py_node`, `talos_messages`, `talos_bundle` |

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

## Topics and messages

- A topic is a shared-memory slot holding the latest message of one fixed size.
- Messages are FlatBuffers **structs** (no tables, strings or vectors), declared
  in a `.fbs` per package and built with `talos_messages`, which generates C++
  and Python.
- Every user topic's slot is 8-byte aligned, so a C++ and a Python endpoint
  agree on the layout from the size alone.
- The FlatBuffers namespace is the short subsystem name (`shooter`, `intake`,
  `drivetrain`); C++ includes `<package>/<file>_generated.h`, Python imports
  `<namespace>.<Struct>`.
- Topic names are declared once, in the package's `topics.h`, and follow
  [talOS/NAMING.md](talOS/NAMING.md).
- One writer per topic. Several producers of a target publish
  `/<sub>/target/<source>` and an arbiter node writes `/<sub>/target`.
- The launcher refuses to start a graph that breaks a naming or graph rule
  (`--allow-graph-errors` overrides).

## Configuration

```
configuration/robot.toml        [robot] period, timeouts, controller address
                                [studio] viewer bridge
                                [[subsystems]] name + path, one per subsystem
configuration/subsystems/*.toml [subsystem] name, node = "//...:node", period_us,
                                custom keys (node.param), [motors.*] and other devices
```

A manifest entry may also set `node = "//...:target"`, which overrides the
subsystem file's `node`, so one set of subsystem files can serve two manifests
(see `examples/superstructure/config/`). A single-file `robot.toml` with
`[subsystems.<name>]` tables also works; most test configs use it. Devices are
addressed by name.
Example configs carry placeholder CAN ids; the robot's config carries the
physical ones and sets `commissioned = true` only after checking them.

## Where a new thing goes

| New thing | Where |
|---|---|
| a mechanism | a node package under `<robot>/main_processor/<name>/`, plus a subsystem file; walkthrough in [examples/superstructure](examples/superstructure/README.md) |
| a message or topic | the owning package's `.fbs` and `topics.h` |
| a producer of an existing target | a `/<sub>/target/<source>` topic and a line in the arbiter |
| a device type | `talOS/hardware` (config + wire), `talOS/devices`, `talOS/python/talos/devices.py`, and the RoboRIO program |
| a node-API feature | `talOS/node`, mirrored in `talOS/node_api` and `talOS/python/talos` |
| a robot | a sibling of `2026-robot/` |
| a controller processor | a sibling of `controller_processor/rio/` speaking `talOS/protocol` |
| a viewer tab | `studio/` |
