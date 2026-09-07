# talOS
Named after the first automaton, talOS is a robotics control architecture aimed to work with the NI RobotRIO or SystemCore acting as a HAL layer with a seperate main computer handling logic

## How it fits together
[ARCHITECTURE.md](ARCHITECTURE.md) is the normative statement of the stack: the
layers bottom to top, what may depend on what and what may never, where a new
thing goes — a subsystem, a topic, a message type, a viewer tab, a controller
processor — and why each boundary is where it is. Read it before adding a node,
a topic or a dependency. `talOS/NAMING.md` is the authority on names.

## Where this is going
[PLAN.md](PLAN.md) is the implementation plan for the subsystem architecture:
hardware declared in one `robot.toml`, a generic RoboRIO program that never
changes again, and subsystems that are one config block plus one node. Read it
before adding a subsystem or touching the hardware path.

## Repository layout

talOS is the framework; a robot is a separate tree that depends on it. The
build enforces the direction — `talOS/` has zero dependencies on any robot, so
a second robot is a new sibling of `2026-robot/`, not a fork of the framework.

```
talOS/                     the framework. Knows nothing about any robot.
  events/  ipc/  memory/   event loop, dispatch-log replay, shared-memory IPC
  rtms/    process/        real-time message store, node lifetime/signals
  introspection/           the live registry: which nodes are running and
                           which topics each one publishes and subscribes to
  protocol/                frame + UDP wire format
  hardware/                hardware wire contract: config, messages, gateway,
                           endpoint, and the Packet IPC envelope
  driver_station/          Driver Station wire codec
  geometry/                header-only Lie groups (SO2/SE2/SO3/SE3) on Eigen
  bridge/                  the IPC <-> UDP node that fronts a controller
  configuration/           robot.toml parser and schema
  launcher/                spawns a robot's nodes from its config

2026-robot/                one robot.
  main_processor/          the node graph that runs talOS
    configuration/         robot.toml: this robot's hardware and subsystems
    drivetrain/ shooter/ odometry/ driver_station/
  controller_processor/
    rio/                   RoboRIO program (GradleRIO/WPILib)
```

`main_processor` and `controller_processor` are the two halves of one robot:
the main computer runs the logic on talOS, the controller processor is the HAL
that owns the actuators. They talk over the `talOS/protocol` UDP framing.
Today the controller is a RoboRIO; a SystemCore or Jetson would be a sibling
directory under `controller_processor/` speaking the same wire contract.

That is the shape of the tree. The rules that keep it that shape —
the layer order, the dependency edges a reviewer should reject, and the exact
files and targets to create for a new subsystem — are in
[ARCHITECTURE.md](ARCHITECTURE.md).

To see what is actually running at any moment, ask the system graph. Every node
publishes its own topics and traffic counters into a shared-memory registry
(`talOS/introspection/`), and the Studio bridge serves the whole picture at
`http://localhost:5800/system.json` — every node, every topic, and who is on
which end. `curl` it, open Studio's System tab, or call `get_system_graph` on
the agent JSON-RPC service. Nothing else answers this: a telemetry frame shows
what one node chose to publish, not the shape of the system that produced it.

## Building
Build used bazel for all projects in the repo to build all. </b>

The one exception is `2026-robot/controller_processor/rio/`, the RoboRIO program, which still builds through
GradleRIO because that is what supplies the WPILib and Phoenix 6 native
libraries. It is deliberately tiny — a Driver Station loop and the hardware
gateway, on `frc::RobotBase` rather than `frc::TimedRobot`, with no
command-based framework and no test suite.

**TODO (next step): drop GradleRIO.** Cross-compile that single executable from
Bazel against the WPILib/HAL and Phoenix 6 native libraries so the whole
repository builds with one tool, in seconds. The robot program was cut down to
the bare minimum specifically to make this a small job.


Build all: `bazle build //...`</b>

Example for building a specific module:

`bazel build //.talos`

## Building on macOS

`.bazelrc` carries two extra flags for macOS. They are not optional and they are
not tuning: without them the C++ tree does not compile at all on a Mac, while
building fine on the Linux CI runner, which is a confusing way to lose an
afternoon.

```
build:macos --cxxopt=-fexperimental-library --linkopt=-fexperimental-library
build:macos --macos_minimum_os=13.4
```

Apple's libc++ still keeps `std::jthread` and `std::stop_token` behind
`-fexperimental-library`, and every node's shutdown path uses both, so without
it nothing that starts a node compiles. The flag is needed at link time as well
as compile time. Separately, both of those types are annotated as available only
from macOS 11, and the floating-point path of `std::format` — reached from
`talOS/memory/shared_memory_ptr.h` — needs 13.4; Bazel's default deployment
target is older than either, so the deployment target has to be raised or the
same code fails as unavailable rather than missing.

## Debugging
To build with debug:
`bazel build -c dbg //path/to:target`</b>
To run:
`lldb bazel-bin/path/to/target`</b>

To generate `compile_comands.json` for clangd or other LSPs in c++: </b>

`bazel run @hedron_compile_commands//:refresh_all`
