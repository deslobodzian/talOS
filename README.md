# talOS

talOS runs C++ and Python robot nodes on a main processor, with the RoboRIO as
the hardware layer. See [ARCHITECTURE.md](ARCHITECTURE.md) for processors,
layers, targets and configuration, and [talOS/NAMING.md](talOS/NAMING.md) for names.

To write a mechanism, follow
[examples/superstructure/README.md](examples/superstructure/README.md).
API reference: [docs/NODE_DEVELOPMENT_GUIDE.md](docs/NODE_DEVELOPMENT_GUIDE.md).
Deploying: [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md).
Robot bring-up checklist: [docs/BRINGUP.md](docs/BRINGUP.md).

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
