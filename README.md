# talOS

talOS runs C++ and Python robot nodes on a main processor, with the RoboRIO as
the hardware layer. See [ARCHITECTURE.md](ARCHITECTURE.md) for processors,
layers, targets and configuration, and [talOS/NAMING.md](talOS/NAMING.md) for names.

API reference: [docs/NODE_DEVELOPMENT_GUIDE.md](docs/NODE_DEVELOPMENT_GUIDE.md).
Terms: [docs/GLOSSARY.md](docs/GLOSSARY.md).

## Start here

**Adding a mechanism (a student).**

1. [Install](docs/INSTALL.md), then [your first mechanism](docs/FIRST_MECHANISM.md): clone to a simulated mechanism.
2. [Glossary](docs/GLOSSARY.md), when a word is new.
3. [Node anatomy](docs/METHODOLOGY.md#2-anatomy-of-a-node) and [units](docs/METHODOLOGY.md#6-units).
4. [examples/superstructure](examples/superstructure/README.md): three mechanisms that cooperate.
5. [The node test matrix](docs/TESTING.md#7-the-node-test-matrix): the tests your node needs.
6. [Node API reference](docs/NODE_DEVELOPMENT_GUIDE.md).

**Debugging a match (a programmer).**

1. [Architecture](ARCHITECTURE.md): the tick, targets, topics.
2. [Reading a session](talOS/launcher/README.md#filtering-and-exporting-logs): `log_dump`, and [session measurements](talOS/launcher/README.md#session-measurements).
3. [Studio](studio/README.md), the live viewer.
4. [Who drives a subsystem](talOS/introspection/README.md#who-drives-a-subsystem) and [replaying a session into one node](talOS/launcher/README.md#replaying-a-session-into-one-node).
5. [Data flow](docs/DATA_FLOW.md): every link, rate and size.

**Maintaining the platform (a mentor).**

1. [Architecture](ARCHITECTURE.md) and [names](talOS/NAMING.md).
2. [RULES.md](RULES.md) and [the development workflow](docs/DEVELOPMENT.md).
3. [Requirements](docs/REQUIREMENTS.md) and [testing](docs/TESTING.md).
4. [Design notes](docs/DESIGN.md).
5. [Deployment](docs/DEPLOYMENT.md), [bring-up](docs/BRINGUP.md) and [the bench](docs/BENCH.md).
6. [TODO.md](TODO.md): open work.

## Install

Linux, macOS, or Windows inside WSL2: `./tools/setup.sh`. Details, flags and troubleshooting: [docs/INSTALL.md](docs/INSTALL.md).

## Build and test

```sh
bazel build //...
bazel test //...
bazel test //examples/superstructure/...
```

The RoboRIO program (`2026-robot/controller_processor/rio`) builds with
GradleRIO, not Bazel.

How tests are chosen and run: [docs/TESTING.md](docs/TESTING.md). How a
change is made and verified: [docs/DEVELOPMENT.md](docs/DEVELOPMENT.md).

## Run in simulation

```sh
./sim.sh --sim-gateway --no-studio   # lightweight simulator, no WPILib needed
./sim.sh                             # WPILib simulation of the RoboRIO + Studio
./sim.sh --help
```

Check a robot's graph without starting it:

```sh
bazel run //talOS/launcher -- --config $PWD/2026-robot/main_processor/configuration/robot.toml --describe-only
```

## Debugging

```sh
bazel build -c dbg //path/to:target && lldb bazel-bin/path/to/target
bazel run @hedron_compile_commands//:refresh_all   # compile_commands.json for clangd
```
