# talOS
Named after the first automaton, talOS is a robotics control architecture aimed to work with the NI RobotRIO or SystemCore acting as a HAL layer with a seperate main computer handling logic

## Where this is going
[PLAN.md](PLAN.md) is the implementation plan for the subsystem architecture:
hardware declared in one `robot.toml`, a generic RoboRIO program that never
changes again, and subsystems that are one config block plus one node. Read it
before adding a subsystem or touching the hardware path.

## Building
Build used bazel for all projects in the repo to build all. </b>

The one exception is `robot/`, the RoboRIO program, which still builds through
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
