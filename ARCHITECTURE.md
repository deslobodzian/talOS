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

See [Data flow and byte budgets](docs/DATA_FLOW.md) for every transport,
producer/consumer, rate, dynamic payload, and shared-memory slot size.

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
  language: cross-cutting robot wiring (target policies, `driver` / `teleop`,
  `telemetry`) is driven by TOML configuration.
- Nodes share only messages and topic names (`talos_messages`), never each
  other's classes.
- Geometry and vision are reusable libraries beside the node stack:
  robot camera nodes depend on `talOS/vision`, which depends on
  `talOS/geometry`, never the reverse.

Beside the stack:

| Package | Role |
|---|---|
| `talOS/configuration` | parses `robot.toml` and the subsystem files |
| `talOS/hardware` | hardware config, request/state messages, the gateway the RoboRIO and sim run |
| `talOS/protocol` | frame and UDP wire format |
| `talOS/bridge` | `:hardware_node`; publishes link health, RoboRIO power/CAN health and gateway counters, and the gateway-to-node clock mapping (`:clock_offset`, read by `HardwareLink::sample_time()`) on `/hw/status` (`bridge::BridgeStatus`); `--log` records every command sent and state received |
| `talOS/driver_station` | Driver Station codec |
| `talOS/launcher` | spawns and supervises the nodes in a config (restarts crashed or stalled nodes per its [restart policy](talOS/launcher/README.md#restarts)); writes each session to `<output-dir>/<session_id>/` with a manifest kept current and old sessions pruned; `--describe-only` lints the graph |
| `talOS/introspection` | `--describe`, the naming rules (`names.h`), the live registry Studio reads |
| `talOS/process` | stop handlers and `ConfigureRealtime` for node binaries; `DurableSync`, the power-loss-safe sync of the launcher's session files and event logs |
| `talOS/msgs` | shared geometry and control message structs (field and chassis angles in radians; `SwerveModulePosition`, `PoseObservation`, `PoseEstimate`) |
| `talOS/telemetry` | `:node` framework node, telemetry frame schema, byte-topic wire format; schema-less `/hw/state` is excluded because `/hw/status` (`bridge::BridgeStatus`) already reports link health. |
| `talOS/teleop` | `:node` framework teleop node; `:deadband` stick deadband calculation; declarative button/axis-to-lane mapping |
| `talOS/schema` | `talos::Descriptor<M>` (`.bfbs` bytes, name, schema hash), `talos::schema::struct_hash`, the unit-suffix table; Python `talos_schema` and upstream `reflection` classes |
| `talOS/build/contract.bzl` | `talos_cc_node`, `talos_py_node`, `talos_messages`, `talos_bundle`, `talos_graph_test` (both fed by a config's generated `robot_manifest.bzl`, `tools/sync_manifest.py`) |
| `talOS/geometry` | header-only SO2/SE2/SO3/SE3 on Eigen 3.4 (`@eigen3`); C++ drivetrain kinematics and vision transforms |
| `talOS/vision` | camera/frame/backend contracts, calibration, localization and pipeline; AprilTag 3.4.5 CPU detection, macOS Metal preprocessing/CCL, AVFoundation/V4L2 capture, field simulation and fixtures; OpenCV 4.12 image conversion/IO |
| `talOS/python/requirements.txt` | Python `robotpy-wpimath` pose estimation and GTSAM factor graphs; `wpilib_lockstep_test` aligns GradleRIO with the RobotPy wheels, not C++ dependencies |

Live node identity is registered before user construction, with the
`kFlagConstructing` lifecycle flag and independent bookkeeping heartbeats.
Endpoints and active-reader liveness appear only after the event loop starts.
The launcher enforces a fixed 30-second construction deadline from spawn;
processes with no registry row retain the ordinary 5-second startup grace.
Neither initializing nor failed optional nodes block control peers.


## The tick

A node's `update()` runs every `period_us` (the subsystem's, else `[robot]`).
After each tick the devices layer flushes one hardware request for the node:

- a motor not set during that tick is sent neutral;
- every motor is neutral while `robot_enabled(node)` is false (no accepted
  hardware state, gateway not enabled, a fault, or a state older than
  `[robot] command_timeout_us`);
- digital and PWM outputs set during the tick are sent as set only while
  `robot_enabled(node)` is true; otherwise DIO is false and PWM is zero;
- motor demand and feedforward are clamped to that motor's configured limits;
  non-finite values neutralize only that motor. `Motor::clamped_count()` counts
  adjusted setter calls for the lifetime of the device.

A subscription's `fresh()` means a value younger than `command_timeout_us`, so
a node that stops hearing its target goes neutral on its own.

Config load refuses a `command_timeout_us` shorter than two Driver Station
packet gaps, `2 × (20000 + [robot] period_us)` µs (a node's Driver Station view
expires at the timeout, and consecutive packets can be 20 ms plus one gateway
tick apart), or longer than 500000 µs. It also refuses a subsystem owning a
motor, digital output or PWM output whose `period_us` exceeds half the timeout.

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
subsystem. A missing optional policy lane remains a warning. With
`[robot] require_target_coverage = true` (default `false`), every described
subsystem that reads its own target lanes must have at least one writer among
each nonempty Driver Station mode's policy sources; missing coverage is a graph
error naming the subsystem, mode, and lanes.

The mode generation is what keeps a target issued under an earlier enable or
mode from ever actuating. The RoboRIO program (and `SimHardware`,
`send_joystick`) stamps each Driver Station packet with `mode_since_us` via one
`driver_station::ModeTracker`: the time of the first observation in the current
mode; the RoboRIO observes every hardware tick independently of the 20 ms packet
publication cadence. Disabling, re-enabling, switching mode,
e-stopping or re-attaching starts a new generation, and the bridge forwards it
unchanged. Each packet also carries the gateway's boot id (the one its State
carries): generations and sample times order packets only within one boot, so
a consumer starts over on a RoboRIO reboot (whose clock restarts low) and
ignores packets of recently retired boots, and the bridge forwards only the
boot of its accepted State. `TargetLane::send` stamps the generation its own
node last drained
(0 while that view is stale, disabled or in no mode); `Target::get` accepts a
lane only when its stamp is nonzero and equals the generation the consumer
last drained. Receipt time cannot do this: a target queued before a mode
change and the packet that changes the mode can be drained together, at one
instant. A Driver Station link that drops and returns in the same mode keeps
its generation. Driver and declarative teleop producers nevertheless require
one neutral-input evaluation before publishing active targets after DS
recovery, startup, or an evaluation gap reaching `command_timeout_us`.

Messages reach a node when its loop drains its topics. A node without message
callbacks drains right before each of its own timers (`update()`, `every()`)
and otherwise sleeps; a node with a `subscribe` callback or a Driver Station
listener, or with no timer, drains every 1 ms. Off-simulation the loop thread
runs under `SCHED_FIFO` (Linux) or the Mach time-constraint policy (macOS) with
memory locked, best effort (`talOS/process`).

Messages become visible when the consumer drains its topics, but `age()` and
`fresh()` count from the writer's publication timestamp. Draining an expired
message cannot revive it. Subscriptions without per-message callbacks dispatch
only the newest available message, counting skipped messages as dropped;
callback subscriptions retain sequence delivery, capped at 64 messages per
topic per drain. Each realtime timer dispatch samples its own clock. Simulation
and replay preserve publication-age semantics. Simulated topics retain RTMS's
1024-slot overwrite window (1023 guaranteed readable messages), with identical
latest-value skips and sequence-drop accounting. `testing::Sim::pause(name,
duration)` stops one node's drains and timers while its clock and peers advance;
`resume(name)` resumes early. Overdue periodic timers coalesce on their original
phase, as after SIGSTOP/SIGCONT. Simulation dispatches nodes serially on one
chronological frontier: each step runs every node's loop up to the earliest
pending deadline of any of them, so no node sees a publication from its future,
and at one instant nodes run in the order they were added. It does not model
concurrent copies, torn reads, OS scheduling jitter, or the CPU cost of
handlers.

## Topics and messages

- A topic is a shared-memory slot holding the latest message of one fixed size.
- Messages are FlatBuffers **structs** (no tables, strings or vectors), declared
  in a `.fbs` per package and built with `talos_messages`, which generates C++
  and Python, a `.bfbs` schema and a schema descriptor per struct, and fails
  the build on a numeric field without a unit suffix.
- Topics are typed by schema identity (`talos::Descriptor<M>`, name and
  64-bit schema hash), not size alone. An attach with a mismatched schema
  fails, naming both types.
- Under the launcher, session-owned segments are created for every declared
  topic before nodes spawn and unlinked at session end. Outside a session, the
  last queue to close unlinks, and dead readers are reclaimed.
- Shared-memory OS names use `ShmObjectName`: interior topic slashes become
  dots. `TALOS_SHM_NAMESPACE`, when set, instead selects a namespace; otherwise
  Bazel's `TEST_TMPDIR` isolates each test run and its child processes. A
  namespaced name is `/~` plus 16 lowercase hex digits of FNV-1a-64 over the
  namespace, a NUL separator, and the mapped topic (18 bytes total). Mapping
  an existing `/~` name is idempotent. Production runs with neither variable
  retain their ordinary names; publishers, subscribers, launcher session
  cleanup, the registry and Studio all use the same mapping.
- Dispatch logs (`.tlog` format version 6) carry each logged topic's schema
  descriptor, allowing `log_dump` to decode payloads to named fields without
  the recording build.
- Every user topic's slot is 8-byte aligned, matching FlatBuffers struct rules.
- The FlatBuffers namespace is the short subsystem name (`shooter`, `intake`,
  `drivetrain`); C++ includes `<package>/<file>_generated.h`, Python imports
  `<namespace>.<Struct>`.
- Topic names are declared once in the package's `BUILD` via
  `talos_messages(topics = ...)`, which generates the package's `topics.h`,
  and follow [talOS/NAMING.md](talOS/NAMING.md).
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
(`tools/sync_manifest.bzl`). `talos_bundle` additionally runs the same check as
a build action and consumes its success stamp before assembly, so a stale
manifest fails the build with the regenerate command.

## Where a new thing goes

| New thing | Where |
|---|---|
| a mechanism | a node package under `<robot>/main_processor/<name>/`, plus a subsystem file and a `[[subsystems]]` entry, then regenerate the manifest; `bazel run //tools:new_subsystem -- <name> --lang cpp\|py` does all of it (walkthrough in [examples/superstructure](examples/superstructure/README.md)) |
| a mechanism in its own repository | `bazel run //tools:new_subsystem -- <name> --lang cpp\|py --standalone DIR` writes a Bzlmod repo (overriding `@talos` with this checkout); the robot adds `bazel_dep` + `local_path_override`, a subsystem file naming `@<name>//:node`, and its `[[subsystems]]` entry |
| a camera | its calibrated `configuration/subsystems/<camera>.toml`, a name in `vision/BUILD`'s `CAMERAS`, a `[[subsystems]]` entry and regenerated manifest; list it in the pose estimator's `cameras` |
| a message or topic | the owning package's `.fbs` and `BUILD` declaration via `talos_messages(topics = ...)`, which generates `topics.h` |
| a producer of an existing target | a `talos::TargetLane` on `/<sub>/target/<source>`, and `<source>` in that subsystem's policy (`[robot.targets]` or its `[targets]`); the launcher refuses a lane no mode lists |
| a device type | `talOS/hardware` (config + wire), `talOS/devices`, `talOS/python/talos/devices.py`, and the RoboRIO program |
| a node-API feature | `talOS/node`, mirrored in `talOS/node_api` and `talOS/python/talos` |
| a robot | a sibling of `2026-robot/` |
| a controller processor | a sibling of `controller_processor/rio/` speaking `talOS/protocol` |
| a viewer tab | `studio/` |
