# talOS

talOS is a small ROS-like stack for FRC robots: nodes in C++ or Python run on a
main processor and talk over shared-memory topics, and the RoboRIO is only the
hardware layer, reached over UDP.

```
your node (C++ class or Python class)
   │  talos::Motor{node, "flywheel"}, robot_enabled(node)
talOS/devices    hardware handles; neutral unless set, neutral when disabled
   │  subscribe / publish / param / update() / StateMachine / TALOS_NODE
talOS/node       generic node: topics, timers, parameters, the tick
   │
talOS/events  talOS/rtms  talOS/memory  talOS/ipc   loop, log/replay, shared-memory topics
```

A C++ node (`2026-robot/main_processor/shooter/shooter.h`, abridged):

```cpp
class Shooter {
 public:
  explicit Shooter(talos::Node& node)
      : node_{node}, flywheel_{node, "flywheel"},
        target_{node.subscribe<ShooterTarget>(kTargetTopic)},
        state_{node.publish<ShooterState>(kStateTopic)} {}
  void update() {
    const bool spin = talos::robot_enabled(node_) && target_.fresh() && target_->enabled();
    if (spin) flywheel_.set_velocity(target_->target_velocity_rps());  // else neutral
    state_.send(ShooterState{/* ... */});
  }
 private:
  talos::Node& node_;
  talos::Motor flywheel_;
  talos::Subscription<ShooterTarget> target_;
  talos::Publisher<ShooterState> state_;
};
```

`shooter.cc` is `TALOS_NODE(shooter::Shooter)`. A Python node
(`2026-robot/main_processor/intake/intake.py`, abridged):

```python
import talos
from intake.IntakeState import IntakeState
from intake.IntakeTarget import IntakeTarget

class Intake:
    def __init__(self, node):
        self.node = node
        self.roller = talos.Motor(node, "roller")
        self.beam = talos.DigitalInput(node, "intake_beam")
        self.target = node.subscribe(IntakeTarget, "/intake/target")
        self.state = node.publish(IntakeState, "/intake/state")

    def update(self):
        target = self.target.value  # None until the first message
        if self.target.fresh() and target.enabled:  # otherwise the flush sends neutral
            self.roller.set_velocity(roller_demand(target.roller_velocity_rps, self.beam.value))
        self.state.send(beam_broken=self.beam.value, enabled=talos.robot_enabled(self.node))
```

To write your own mechanism, follow
[examples/superstructure/README.md](examples/superstructure/README.md). API
reference: [docs/NODE_DEVELOPMENT_GUIDE.md](docs/NODE_DEVELOPMENT_GUIDE.md).
Layers and rules: [ARCHITECTURE.md](ARCHITECTURE.md). Names:
[talOS/NAMING.md](talOS/NAMING.md). Deploying: [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md).

## Layout

| Path | What |
|---|---|
| `talOS/node`, `talOS/devices` | the node API and hardware handles (C++) |
| `talOS/python/talos` | the same API in Python (ctypes over `talOS/node_api`) |
| `talOS/events`, `rtms`, `memory`, `ipc`, `process` | event loop, log/replay, shared-memory topics, process plumbing |
| `talOS/hardware`, `protocol`, `bridge` | hardware wire contract, UDP framing, `:hardware_node` (the courier to the RoboRIO) |
| `talOS/driver_station` | Driver Station codec and node |
| `talOS/configuration`, `launcher`, `introspection` | `robot.toml` parser, the launcher, graph description and names |
| `talOS/build/contract.bzl` | `talos_cc_node`, `talos_py_node`, `talos_messages`, `talos_bundle` |
| `2026-robot/` | this season's robot ([README](2026-robot/README.md)) |
| `examples/superstructure/` | intake + elevator + superstructure example |
| `studio/` | the telemetry viewer |
| `sim.sh`, `tools/deploy.sh`, `tools/test_drivetrain.py` | run in sim, deploy, drivetrain smoke test |

## Install

Linux/macOS: `./tools/setup.sh`. Windows: `powershell -ExecutionPolicy Bypass -File tools\setup.ps1`
(talOS builds inside WSL2). Details, flags and troubleshooting: [docs/INSTALL.md](docs/INSTALL.md).

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

## macOS

`.bazelrc` adds two macOS-only flags; without them the C++ tree does not build:

```
build:macos --cxxopt=-fexperimental-library --linkopt=-fexperimental-library  # std::jthread, std::stop_token
build:macos --macos_minimum_os=13.4                                            # std::format floating point
```

## Debugging

```sh
bazel build -c dbg //path/to:target && lldb bazel-bin/path/to/target
bazel run @hedron_compile_commands//:refresh_all   # compile_commands.json for clangd
```
