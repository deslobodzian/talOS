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
talOS/devices     Motor, CANcoder, Gyro, DigitalInput, ..., robot_enabled, Target, TargetLane, OperatorArming, testing::SimHardware
talOS/node        Node, Subscription, Publisher, StateMachine, TALOS_NODE, testing::Sim
talOS/python      the same API in Python, ctypes over talOS/node_api (libtalos_node.so)
talOS/events      event loop, dispatch log, replay
talOS/rtms        shared-memory topics, on talOS/memory and talOS/ipc
```

Allowed dependencies:

- Each layer depends only on layers below it. `talOS/node` drives no
  hardware: it knows the hardware request topic, `/hw/request/<node>`, only to
  open it for the devices layer's flush and refuse it to every other publisher
  ([NAMING.md](talOS/NAMING.md#topics)), and reads the hardware config only
  for timeouts and actuator ownership (its loop tier). `talOS/devices` is
  built on `Node`'s public API (`extension<T>()`, `after_update()`) and the
  framework hooks in `detail::NodeAccess`.
- `talOS/` never depends on a robot, an example or `studio/`.
- Python reaches talOS only through the C ABI in `talOS/node_api`; it does not
  reimplement a wire format, a memory layout, a state machine, or a C++ node
  (C++ nodes exposed to Python use `TALOS_EXPORT_NODE` and `talos.cpp_node`).
- Adding a node in one language never requires modifying code in another
  language: cross-cutting robot wiring (target policies, `driver` / `teleop`,
  `telemetry`) is driven by TOML configuration.
- Nodes share only messages and topic names (`talos_messages`), never each
  other's classes.
- Geometry, vision, trajectory and aiming are reusable libraries beside the
  node stack: robot camera nodes depend on `talOS/vision`, and an autonomous
  routine may depend on `talOS/trajectory`; both depend on `talOS/geometry`,
  never the reverse; `talOS/aiming` depends only on `talOS/geometry` and
  `talOS/msgs`.

Beside the stack:

| Package | Role |
|---|---|
| `talOS/configuration` | parses `robot.toml` and the subsystem files |
| `talOS/hardware` | hardware config, request/state messages, the gateway the RoboRIO and sim run |
| `talOS/protocol` | frame and UDP wire format |
| `talOS/bridge` | `:hardware_node`; publishes link health, RoboRIO power/CAN health, gateway counters and build, and the gateway-to-node clock mapping (`:clock_offset`, read by `HardwareLink::sample_time()`) on `/hw/status` (`bridge::BridgeStatus`); `:merge` holds the request merge rules (owner timeout, merged observation) that `testing::SimHardware` also runs; `--log` records every command sent and state received |
| `talOS/driver_station` | Driver Station codec |
| `talOS/launcher` | spawns and supervises the nodes in a config (restarts crashed or stalled nodes per its [restart policy](talOS/launcher/README.md#restarts)); writes each session to `<output-dir>/<session_id>/` with a manifest kept current and old sessions pruned; `--describe-only` lints the graph; `:log_dump` prints its `.tlog`s and `:tlog_to_wpilog` converts them to `.wpilog` |
| `talOS/introspection` | `--describe`, the naming rules (`names.h`), the live registry Studio reads |
| `talOS/process` | stop handlers, `ConfigureRealtime` for node binaries and `ConfigureBackgroundThread` for their helper threads; `DurableSync`, the power-loss-safe sync of the launcher's session files and event logs |
| `talOS/msgs` | shared geometry and control message structs (field and chassis angles in radians; `SwerveModulePosition`, `PoseObservation`, `PoseEstimate`) |
| `talOS/telemetry` | `:node` framework node, telemetry frame schema, byte-topic wire format; schema-less `/hw/state` is excluded because `/hw/status` (`bridge::BridgeStatus`) already reports link health. |
| `talOS/teleop` | `:node` framework teleop node; `:deadband` stick deadband calculation; declarative button/axis-to-lane mapping |
| `talOS/schema` | `talos::Descriptor<M>` (`.bfbs` bytes, name, schema hash), `talos::schema::struct_hash`, the unit-suffix table, field paths (`:field_path`: a struct's scalar by `a/b` path, encoded from TOML by `teleop`, read per tick by `telemetry`); Python `talos_schema` and upstream `reflection` classes |
| `talOS/build/contract.bzl` | `talos_cc_node`, `talos_py_node`, `talos_cc_shared_node` (C++ nodes for `talos.cpp_node`, linked against `libtalos_node.so`), `talos_messages`, `talos_bundle`, `talos_graph_test` (both fed by a config's generated `robot_manifest.bzl`, `tools/sync_manifest.py`) |
| `talOS/mechanisms` | configured building blocks for robot nodes: `PositionMechanism`, `VelocityMechanism`, `RollerWithSensor`; depends only on `talOS/devices`' public API (handles, `robot_enabled`, `RequireInUpdate`), `talOS/node`, `talOS/configuration`, `//talOS/hardware:config` (a motor's own config: limits, frame, gains) and `//talOS/msgs:control`, and is used by robot code; a node still gates on its own `Target` |
| `talOS/geometry` | header-only SO2/SE2/SO3/SE3 on Eigen 3.4 (`@eigen3`); C++ drivetrain kinematics and vision transforms |
| `talOS/aiming` | shooter aiming library (no node): `ShotTable` (distance → flywheel, hood, time of flight; clamped and flagged outside its range), `TargetSolver` (static and shoot-on-the-move aim for a fixed shooter or a turret, bounded time-of-flight iteration) and `ReadinessGate` (ready to fire, first failing reason, tick debounce) |
| `talOS/trajectory` | time-indexed holonomic trajectories (`sample(t)`, event markers, splits), the Choreo `.traj` loader (`@nlohmann_json`), straight-line moves between two poses (`MakeStraightLine`, trapezoidal, allocation-free into a reserved trajectory), alliance flips (rotate or mirror) and `HolonomicFollower` (feedforward plus PID to a chassis speed); depends only on `talOS/geometry` |
| `talOS/vision` | camera/frame/backend contracts, calibration, localization and pipeline; AprilTag 3.4.5 CPU detection, macOS Metal preprocessing/CCL, AVFoundation/V4L2 capture, field simulation and fixtures; OpenCV 4.12 image conversion/IO |
| `talOS/python/requirements.txt` | Python `robotpy-wpimath` pose estimation and GTSAM factor graphs; `wpilib_lockstep_test` aligns GradleRIO with the RobotPy wheels, not C++ dependencies |

Live node identity is registered before user construction, with the
`kFlagConstructing` lifecycle flag and independent bookkeeping heartbeats.
Endpoints and active-reader liveness appear only after the event loop starts;
the launcher then compares each run's endpoints with its `--describe` probe
once ([live endpoints](talOS/launcher/README.md#live-endpoints)).
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
  adjusted setter calls for the lifetime of the device;
- a motor's position and motion-magic demands are sent neutral while no
  `Motor::rezero` holds for it (never taken, or dropped on a gateway boot or
  device reset that may have restarted its frame), unless its config sets
  `position_frame = "absolute"` (a remote CANcoder's frame, which survives a
  reset while one CANcoder rotation covers the travel; see
  [position frames](docs/NODE_DEVELOPMENT_GUIDE.md#rezero-and-position-frames)).

Output setters (motor, digital, PWM) are therefore accepted only inside
`update()`; from the constructor, `every()` or a message callback they throw
`std::logic_error` naming the device (`TalosError` in Python), since such a
command would reach the hardware for at most one tick.

A subscription's `fresh()` means a value younger than `command_timeout_us`, so
a node that stops hearing its target goes neutral on its own.

A node that stops sending requests goes neutral at the bridge once its last
accepted request is older than its owner timeout,
`max(2 × period_us, command_timeout_us / 2)` and never more than
`command_timeout_us` (50 ms for every 2026-robot subsystem), checked on the
bridge's 1 ms tick; a shorter gap, such as a late or missed tick, keeps its
last demand. Whatever the bridge does, the RoboRIO neutralizes it within
`command_timeout_us` of the State its last request was built from (plus
the RoboRIO loop's lateness; if that loop stalls, its
[watchdog](talOS/hardware/README.md) acts two loop periods and one 5 ms
poll later), but by invalidating every output: the merged command carries the oldest observation
of any owner still merged. Because an actuator owner's `period_us` is at most
`command_timeout_us / 4` (below), its owner timeout is half the timeout, so
the owner timeout keeps one silent node from stopping the others while the
State its last request was built from was younger than half the timeout
(50 ms at 100 ms) when the bridge accepted it. A silent owner whose last
request already observed an older State stops every output; that is the cost
of tolerating request gaps up to half the timeout.

Config load refuses a `command_timeout_us` shorter than two Driver Station
packet gaps, `2 × (20000 + [robot] period_us)` µs (a node's Driver Station view
expires at the timeout, and consecutive packets can be 20 ms plus one gateway
tick apart), or longer than 500000 µs. It also refuses a subsystem owning a
motor, digital output or PWM output whose `period_us` exceeds a quarter of the
timeout.

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
its generation.

The generation does not stop a producer from turning a joystick held through
a disable into a new target of the next generation. A producer of operator
input therefore opens its lanes with a `talos::OperatorArming` (`target.h`;
`talos.OperatorArming` in Python), as `driver` and the declarative `teleop`
do, and evaluates it on every reading of its inputs. An evaluation that
finds every input neutral, while the Driver Station shows a mode, arms it;
until then the lane's `send()` goes out as a release. It disarms at startup;
when the Driver Station shows no mode, a new generation or a new gateway
boot (so after a disable, however brief, even one no evaluation sampled);
when the link recovers from a stale spell (sampled or not); when the
producer's evaluations stop for `command_timeout_us`; and on `disarm()`. A
`TargetLane` without an `OperatorArming` is never held: the graph lint cannot tell operator input
from an autonomous producer, so using `OperatorArming` is the producer's
job.

Messages reach a node when its loop drains its topics. Every node drains right
before each of its own timers (`update()`, `every()`), so a timer sees what was
published by its deadline whether or not the node has callbacks, and otherwise
sleeps. A `subscribe` callback, or a Driver Station listener, makes that topic
wake the node: its publisher notifies the sleeping loop, which drains every
subscription at once. In a node with no timer every subscription wakes it.
While it waits on such a topic the loop also drains at least once a second, in
case a publisher died before notifying. `testing::Sim` models a wake as a 1 ms
drain.
Off-simulation the loop thread runs realtime, best effort, in one of three
tiers the process picks itself: the bridge above control nodes (a node whose
own subsystem owns a motor, digital or PWM output, or that writes a target
lane) above observers (every other node). Linux gives each tier its own
`SCHED_FIFO` priority below PREEMPT_RT's interrupt threads; macOS runs the
bridge and control nodes under the Mach time-constraint policy, which has no
priority among them, and observers timeshare below it. The loop's
shared-memory rings and stack are prefaulted and locked; Linux also locks
all other memory (`mlockall`), while macOS only prefaults the talOS code
(`talOS/process`; the priorities, what is locked on each OS and
`TALOS_REALTIME`'s override are in the node development guide's realtime
scheduling paragraph). Helper threads, such as camera capture
and AprilTag detection, run below every loop thread
(`ConfigureBackgroundThread`).

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
- Dispatch logs (`.tlog` format version 7) carry each logged topic's schema
  descriptor, allowing `log_dump` to decode payloads to named fields without
  the recording build, and `tlog_to_wpilog` to convert a session to a WPILib
  `.wpilog` that AdvantageScope opens (one entry per message field;
  [talOS/launcher/README.md](talOS/launcher/README.md#converting-a-session-to-wpilog)).
- Every user topic's slot is 8-byte aligned, matching FlatBuffers struct rules.
- The FlatBuffers namespace is the short subsystem name (`shooter`, `intake`,
  `drivetrain`); C++ includes `<package>/<file>_generated.h`, Python imports
  `<namespace>.<Struct>`.
- Topic names are declared once in the package's `BUILD` via
  `talos_messages(topics = ...)`, which generates the package's `topics.h`,
  and follow [talOS/NAMING.md](talOS/NAMING.md).
- One writer per topic. Several producers of a target each write their own
  lane, and the consumer picks one (see "Targets").
- Who may write the `hw` namespace (the bridge, and each node's devices
  layer for its own request) is in [NAMING.md](talOS/NAMING.md#topics).
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
                                [targets] lane priorities per mode,
                                [trajectories] name = "path" data files
```

A manifest entry may also set `node = "//...:target"`, which overrides the
subsystem file's `node`, so one set of subsystem files can serve two manifests
(see `examples/superstructure/config/`). A single-file `robot.toml` with
`[subsystems.<name>]` tables also works; most test configs use it. Devices are
addressed by name. A subsystem key the framework does not own
(`config::kFrameworkSubsystemKeys`) is a node's parameter, so config load
refuses only a framework section spelled in the singular (`target`, `motor`,
`sensor`, ...), naming the key and its subsystem file (or
`[subsystems.<name>]` in a single-file config). A node declares its
parameters by reading them in its constructor (`param`, `param_of`), which
its `--describe` reports, and the launcher refuses a key no node read
([parameter keys](talOS/launcher/README.md#parameter-keys)).
`[trajectories]` maps a name to a data file (a Choreo `.traj`), its path
relative to the file that sets it (a subsystem file's entry overrides its
defaults file's, and only the effective entries are required and bundled);
config load refuses a missing file, a non-string or empty path, a name that
is not a topic segment, a section that is not a table and a
`[subsystem.trajectories]` under a subsystem file's header, naming the file
and `trajectories.<name>`, and a node opens it by name
(`RobotConfig::TrajectoryPath`, [talOS/trajectory](talOS/trajectory/README.md#using-it-from-a-node)).
Example configs carry placeholder CAN ids; the robot's config carries the
physical ones and sets `commissioned = true` only after checking them.

`[[subsystems]]` is the only list of a robot's nodes. Bazel cannot read TOML,
so `bazel run //tools:sync_manifest -- <dir>/robot.toml` writes
`<dir>/robot_manifest.bzl` (`<stem>_manifest.bzl` for any other config name):
`NODES`, every node label the config launches, and `CONFIG_FILES`, every file
it reads besides itself (`include`, `path`, `defaults`, `[trajectories]`
files). `talos_bundle`, which ships them at the same relative layout, and
`talos_graph_test` take those constants; no BUILD file lists nodes or
subsystem files. Each config package has a `manifest_sync_test`
(`tools/sync_manifest.bzl`). `talos_bundle` additionally runs the same check as
a build action and consumes its success stamp before assembly, so a stale
manifest fails the build with the regenerate command.

## Where a new thing goes

| New thing | Where |
|---|---|
| a mechanism | a node package under `<robot>/main_processor/<name>/`, plus a subsystem file and a `[[subsystems]]` entry, then regenerate the manifest; `bazel run //tools:new_subsystem -- <name> --lang cpp\|py` does all of it, generating a self-contained node (walkthrough in [examples/superstructure](examples/superstructure/README.md)). In C++ the node can instead be a thin wrapper over a [`talOS/mechanisms`](talOS/mechanisms/README.md) building block, as `examples/superstructure/cpp/elevator.h` is |
| a mechanism in its own repository | `bazel run //tools:new_subsystem -- <name> --lang cpp\|py --standalone DIR` writes a Bzlmod repo (overriding `@talos` with this checkout); the robot adds `bazel_dep` + `local_path_override`, a subsystem file naming `@<name>//:node`, and its `[[subsystems]]` entry |
| a camera | its calibrated `configuration/subsystems/<camera>.toml`, a name in `vision/BUILD`'s `CAMERAS`, a `[[subsystems]]` entry and regenerated manifest; list it in the pose estimator's `cameras` |
| a trajectory | a Choreo `.traj` file (beside its `.chor` project) in `<robot>/main_processor/configuration/trajectories/`, named in the following node's subsystem file under `[trajectories]` and loaded with `talOS/trajectory`; regenerate the manifest so `CONFIG_FILES`, and the bundle, carry it |
| a message or topic | the owning package's `.fbs` and `BUILD` declaration via `talos_messages(topics = ...)`, which generates `topics.h` |
| a producer of an existing target | a `talos::TargetLane` on `/<sub>/target/<source>` (opened with a `talos::OperatorArming` when it maps operator input), and `<source>` in that subsystem's policy (`[robot.targets]` or its `[targets]`); the launcher refuses a lane no mode lists |
| a device type | `talOS/hardware` (config + wire), `talOS/devices`, `talOS/python/talos/devices.py`, and the RoboRIO program |
| a node-API feature | `talOS/node`, mirrored in `talOS/node_api` and `talOS/python/talos` |
| a robot | a sibling of `2026-robot/` |
| a controller processor | a sibling of `controller_processor/rio/` speaking `talOS/protocol` |
| a viewer tab | `studio/` |
