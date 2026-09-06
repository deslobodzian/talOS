# talOS
Named after the first automaton, talOS is a robotics control architecture aimed to work with the NI RobotRIO or SystemCore acting as a HAL layer with a seperate main computer handling logic

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
  protocol/                frame + UDP wire format
  hardware/                hardware wire contract: config, messages, gateway,
                           endpoint, and the Packet IPC envelope
  driver_station/          Driver Station wire codec
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

## Debugging
To build with debug:
`bazel build -c dbg //path/to:target`</b>
To run:
`lldb bazel-bin/path/to/target`</b>

To generate `compile_comands.json` for clangd or other LSPs in c++: </b>

`bazel run @hedron_compile_commands//:refresh_all`
