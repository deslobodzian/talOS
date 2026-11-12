# Your first mechanism

From a fresh clone to a new mechanism, a "wrist" with one motor, running in
simulation and following a scripted joystick button. Every command here was
run on Linux; macOS differs only in how `tools/setup.sh` installs tools. Two
tracks share the setup: the Python track comes first because it is shorter;
the C++ track follows and reuses everything but the node.

Terms are explained where they first matter; [the glossary](GLOSSARY.md) has
all of them.

## 1. Set up

```sh
./tools/setup.sh          # Linux or macOS; on Windows run it inside WSL2
./tools/setup.sh --check  # lists each tool and whether it is acceptable
```

`setup.sh` installs the compiler, Bazel (through Bazelisk), Python, Node and
the linters ([INSTALL.md](INSTALL.md) has flags and troubleshooting). Bazel is
the build tool: a *target* such as `//tools:new_subsystem` names something it
can build or run, and the first build of a fresh clone compiles the framework,
which takes the longest of anything here. Later builds reuse that work.

All commands below run from the repository root.

## 2. Make a scratch robot

`2026-robot/` is the project's simulation testbed, and its graph is full: the
launcher refuses more than eight readers of one topic, and `/hw/state/driver_station`
already has eight. A ninth node there makes `--describe-only` fail with
`/hw/state/driver_station: has 9 readers; RTMS reader limit is 8`. So the
walkthrough builds on a small robot of its own. Create three files.

`my_robot/main_processor/configuration/robot.toml`:

```toml
[robot]
name = "my_robot"
period_us = 5000
command_timeout_us = 100000
status_hz = 200.0
commissioned = false
require_target_coverage = true
sim_gateway = "//2026-robot/main_processor/simulation:sim_gateway"

controller_address = "127.0.0.1"
controller_port = 5802
local_port = 5803

[robot.targets]
teleop = ["teleop"]
autonomous = []
test = []

[[subsystems]]
name = "teleop"
path = "subsystems/teleop.toml"
```

`my_robot/main_processor/configuration/subsystems/teleop.toml`:

```toml
[subsystem]
name = "teleop"
node = "//talOS/teleop:node"
period_us = 20000
```

`my_robot/main_processor/configuration/BUILD`:

```python
package(default_visibility = ["//visibility:public"])

exports_files(["robot.toml"] + glob(["subsystems/*.toml"]))
```

What these say:

- A **node** is one program (one process) that does one job; a **subsystem**
  is the robot's name for a node plus its configuration. `[[subsystems]]` in
  `robot.toml` is the only list of a robot's nodes. `teleop` is the framework's
  node that turns Driver Station buttons and sticks into targets; it needs no
  code from you ([architecture: configuration](../ARCHITECTURE.md#configuration)).
- `[robot] period_us` is the default **tick**: how often a node's `update()`
  runs. `command_timeout_us` is how old data may be before the robot goes
  neutral ([architecture: the tick](../ARCHITECTURE.md#the-tick)).
- `[robot.targets]` says which **target lanes** each Driver Station mode
  listens to. Here a subsystem obeys the `teleop` lane in teleop and nothing in
  autonomous. `require_target_coverage = true` makes the launcher refuse a
  subsystem nobody can drive.
- `sim_gateway` names the program that stands in for the RoboRIO, the robot's
  hardware layer, when you simulate.

## 3. Python track

### Generate the node

```sh
bazel run //tools:new_subsystem -- wrist --lang py --robot my_robot
```

```text
wrote my_robot/main_processor/wrist/wrist_message.fbs
wrote my_robot/main_processor/wrist/BUILD
wrote my_robot/main_processor/wrist/wrist.py
wrote my_robot/main_processor/wrist/node_test.py
wrote my_robot/main_processor/wrist/testdata/robot.toml
wrote my_robot/main_processor/configuration/subsystems/wrist.toml
wrote my_robot/main_processor/configuration/robot.toml
wrote my_robot/main_processor/configuration/robot_manifest.bzl
```

Run it through `bazel run`; `python3 tools/new_subsystem.py` alone fails with
`No module named 'tools'`. `--dry-run` prints every file and edit without
writing. The tool also added a `[[subsystems]]` entry for `wrist` to
`robot.toml` and rewrote `robot_manifest.bzl`: the **manifest**, a generated
list of the node binaries and config files this robot needs. Bazel cannot read
TOML, so it reads this file instead. Edit `robot.toml` or a subsystem file in a
way that adds or removes a node or file, and regenerate it with
`bazel run //tools:sync_manifest -- my_robot/main_processor/configuration/robot.toml`
(a stale manifest fails the build with that command).

### The message

`wrist_message.fbs`:

```fbs
namespace wrist;

struct WristTarget {
  velocity_rps:double;
}

struct WristState {
  enabled:bool;
  velocity_rps:double;
  connected:bool;
}
```

A **topic** is a named shared-memory slot, `/wrist/state`, holding the latest
**message** of one fixed type; nodes talk only through topics. Messages are
FlatBuffers *structs*: fixed-size records, which is what lets them sit in
shared memory. Every number carries its unit as a suffix (`_rps`, `_m`, `_us`,
`_v`), and the build fails on one without. Topic names follow
[NAMING.md](../talOS/NAMING.md#topics): `/owner/role[/qualifier]`, where the
role says who may write it (`state` is published by its owner; `target` is
written by anyone else).

### The node

`wrist.py`, without the comments:

```python
class Wrist:
    def __init__(self, node):
        self.motor = talos.Motor(node, "wrist_motor")
        self.target = talos.Target(node, WristTarget)
        self.state = node.publish(WristState, topics.STATE)

    def update(self):
        t = self.target.get()  # None: disabled, stale or released
        drive = (t is not None and self.motor.connected
                 and math.isfinite(t.velocity_rps))
        if drive:
            self.motor.set_velocity(t.velocity_rps)
        self.state.send(enabled=drive, velocity_rps=self.motor.velocity,
                        connected=self.motor.connected)
```

- The constructor opens everything the node uses: a `Motor` named by the key
  in its subsystem file, a `Target` for incoming goals, a publisher for
  `/wrist/state`.
- `update()` runs every tick. After it returns, the framework sends one
  hardware request for the node: a motor not set during that tick is sent
  neutral. That is why **motor setters work only inside `update()`**: a command
  from anywhere else would reach the motor for at most one tick, so the
  framework refuses it with an error
  ([node guide: threads](NODE_DEVELOPMENT_GUIDE.md#threads)).
- A **target lane** is `/wrist/target/<source>`, one per producer (`teleop`,
  `auto`, ...). `Target.get()` returns the message from the first lane in the
  mode's priority list that is active, fresh, and issued under the current
  Driver Station mode, and only while `robot_enabled(node)` holds (the robot is
  enabled, with no fault and a hardware state younger than
  `command_timeout_us`). Otherwise it returns `None`, nothing is set, and the motor goes
  neutral. Safety needs no code from you; it is the shape of the node
  ([architecture: targets](../ARCHITECTURE.md#targets), and rule 1 of
  [RULES.md](../RULES.md)).

### Configure the subsystem

`my_robot/main_processor/configuration/subsystems/wrist.toml` describes the
motor and the node's inputs. Two edits make it move. Uncomment the teleop
binding, which tells the framework's `teleop` node to write a target on
`/wrist/target/teleop` while Driver Station joystick button 3 is held:

```toml
[[teleop.bindings]]
button = 3
while_held = { velocity_rps = 20.0 }
```

And give the motor's velocity loop a feedforward. The scaffold leaves every
gain at zero ("untuned"), and the physics simulation then stalls the motor
([simulation backends](../talOS/hardware/README.md#simulation-backends-ideal-vs-physics-backed)):

```toml
slot0 = { p = 0.0, i = 0.0, d = 0.0, s = 0.0, v = 0.12, a = 0.0, g = 0.0 }
```

The `can_id` there is a placeholder. On a real robot a person sets the
physical id and `commissioned = true` at the robot; nobody does it from a
desk ([bring-up](BRINGUP.md#human-commissioning)).

Without the binding, nothing writes the wrist's lane and the launcher refuses
the graph (`target coverage required for mode 'teleop'`).

### Run the generated test

```sh
bazel test //my_robot/main_processor/wrist:all
```

```text
//my_robot/main_processor/wrist:node_test    PASSED in 1.3s
```

The scaffold already tests the case list every node must pass: a disabled
robot, a stale or released lane, a lost Driver Station, a disconnected motor, a
NaN demand, another mode's lane, a target left over from an earlier enable
([node test matrix](TESTING.md#7-the-node-test-matrix)). The test runs your
class on `talos.testing.Sim`, a simulated clock with in-memory topics, and
`SimHardware`, a stand-in for the hardware bridge; neither needs a robot or a
process. When you change what the node does, change `node_test.py` with it.

### Check the graph

```sh
bazel build //talOS/launcher //talOS/bridge:hardware_node \
  //2026-robot/main_processor/simulation:sim_gateway //my_robot/main_processor/...
bazel run //talOS/launcher -- --config $PWD/my_robot/main_processor/configuration/robot.toml --describe-only
```

```text
================ Declared Graph ==================
Nodes described: 3/3, topics: 7
  • hardware_node (5 sources)
  • teleop (3 sources)
  • wrist (7 sources)
  warning: /wrist/state: is published but nothing subscribes to it
==================================================
[launcher] describe-only: nothing was started
```

The launcher runs every node binary with `--describe`, which makes the node
build itself, report the topics it reads and writes, and exit, then checks the
whole graph against the naming rules
([launcher: two phases](../talOS/launcher/README.md#two-phases)). The build
comes first because the launcher runs the built binaries; before it, you see
`did not answer --describe (execv bazel-bin/...: No such file or directory)`.
A warning is fine; an `error:` line means the launcher would refuse to start.

### Run it in simulation

```sh
mkdir -p /tmp/talos_tutorial
./bazel-bin/talOS/launcher/launcher \
  --config $PWD/my_robot/main_processor/configuration/robot.toml \
  --sim --session-id 1 --output-dir /tmp/talos_tutorial --duration-s 8 \
  --start-sim-gateway --sim-gateway-arg --physics \
  --sim-gateway-arg --mode-schedule --sim-gateway-arg 'disabled:1,teleop:10' \
  --sim-gateway-arg --joystick-schedule \
  --sim-gateway-arg '0:axes=0;buttons=0;pov=-1|3:axes=0;buttons=4;pov=-1|6:axes=0;buttons=0;pov=-1'
```

This is what `./sim.sh --sim-gateway --no-studio` runs for `2026-robot`, with
the same scripting options spelled for the launcher; `sim.sh` is fixed to the
2026 config, so a robot of your own uses the launcher directly. The simulator
plays the Driver Station and the RoboRIO. The mode schedule is `mode:seconds`
phases (disabled for 1 s, then teleop). The joystick schedule is
`seconds:axes=...;buttons=MASK;pov=DEG` snapshots of joystick 0, each held
until the next: button 3 is bit 2 of the mask, so `buttons=4` presses it from
second 3 to second 6 ([syntax](../2026-robot/main_processor/simulation/README.md#scripted-joystick-and-fault-schedules)).
Use a new `--session-id` each run, or delete `/tmp/talos_tutorial/1`.

### See the result

A **session** is one run of the launcher; its directory,
`<output-dir>/<session-id>/`, holds each node's log (`wrist.tlog`), its text
output (`wrist.log`) and the graph. `log_dump` reads the logs and decodes
every message:

```sh
./bazel-bin/talOS/launcher/log_dump --node wrist --topic /wrist/state /tmp/talos_tutorial/1 \
  | grep WristState | awk 'NR%50==1'
```

```text
      wrist.WristState {enabled: false, velocity_rps: 0, connected: true}
      wrist.WristState {enabled: false, velocity_rps: 0, connected: true}
      wrist.WristState {enabled: true, velocity_rps: 10.433849447220124, connected: true}
      wrist.WristState {enabled: true, velocity_rps: 19.999745921323143, connected: true}
      wrist.WristState {enabled: true, velocity_rps: 19.999999993251436, connected: true}
      wrist.WristState {enabled: false, velocity_rps: 2.66967771064453, connected: true}
      wrist.WristState {enabled: false, velocity_rps: 0, connected: true}
```

The wrist sat neutral while disabled, spun up to 20 rps while button 3 was
held, and coasted down when the button released. Remove the `buttons=4`
snapshot, or the mode schedule's `teleop`, and it stays neutral: that is the
safety path working. [Studio](../studio/README.md) is the live viewer;
`./sim.sh --sim-gateway` starts it for `2026-robot`. Reading a log is the
same on every robot ([launcher: filtering and exporting logs](../talOS/launcher/README.md#filtering-and-exporting-logs)).

## 4. C++ track

Everything in sections 1 and 2 is the same. Use a different name if the Python
track already took `wrist`; this track uses `claw`.

```sh
bazel run //tools:new_subsystem -- claw --lang cpp --robot my_robot
```

```text
wrote my_robot/main_processor/claw/claw_message.fbs
wrote my_robot/main_processor/claw/BUILD
wrote my_robot/main_processor/claw/claw.h
wrote my_robot/main_processor/claw/claw.cc
wrote my_robot/main_processor/claw/node_test.cc
wrote my_robot/main_processor/claw/testdata/robot.toml
wrote my_robot/main_processor/claw/testdata/claw.toml
wrote my_robot/main_processor/configuration/subsystems/claw.toml
...
```

The message file has the same shape as the Python one. The node is a header
plus a one-line entry point, `TALOS_NODE(claw::Claw)` in `claw.cc`:

```cpp
class Claw {
 public:
  explicit Claw(talos::Node& node)
      : motor_{node, "claw_motor"},
        target_{node},
        state_{node.publish<State>(topics::kState)} {}

  void update() {
    const Target* t = target_.get();  // null: disabled, stale or released
    const bool drive =
        t != nullptr && motor_.connected() && std::isfinite(t->velocity_rps());
    if (drive) motor_.set_velocity(t->velocity_rps());
    state_.send(State{drive, motor_.velocity(), motor_.connected()});
  }
  ...
};
```

It is the Python node line for line; the framework is written once and
wrapped for Python ([node guide](NODE_DEVELOPMENT_GUIDE.md#node) lists each
call in both languages). Do the same two edits to
`subsystems/claw.toml` (the teleop binding and `v = 0.12`), then:

```sh
bazel test //my_robot/main_processor/claw:all
bazel build //talOS/launcher //talOS/bridge:hardware_node \
  //2026-robot/main_processor/simulation:sim_gateway //my_robot/main_processor/...
bazel run //talOS/launcher -- --config $PWD/my_robot/main_processor/configuration/robot.toml --describe-only
```

The launch and `log_dump` commands from the Python track work unchanged; read
the `claw` node instead:

```sh
./bazel-bin/talOS/launcher/log_dump --node claw --topic /claw/state /tmp/talos_tutorial/1 \
  | grep ClawState | awk 'NR%50==1'
```

```text
      claw.ClawState {enabled: false, velocity_rps: 0, connected: true}
      claw.ClawState {enabled: true, velocity_rps: 19.999999976104935, connected: true}
      claw.ClawState {enabled: false, velocity_rps: 0, connected: true}
```

## 5. Next

- Real behaviour: build on a ready-made block, `bazel run //tools:new_subsystem
  -- <name> --lang cpp --type position`
  ([mechanisms](../talOS/mechanisms/README.md)), and read the worked example
  [examples/superstructure](../examples/superstructure/README.md).
- A producer other than a button binding (autonomous, an aligner): a
  `TargetLane` ([node guide: targets](NODE_DEVELOPMENT_GUIDE.md#targets)).
- Settings read from the config (`node.param`), the full API, and the
  rules every node follows: [node guide](NODE_DEVELOPMENT_GUIDE.md) and
  [methodology](METHODOLOGY.md).
- Putting a mechanism on `2026-robot`, or any robot with a full graph: the
  scaffold takes `--robot DIR`; keep each topic at eight readers or fewer and
  give the new subsystem a `[targets]` table if autonomous has no writer for it.

When a step fails, the launcher's message names the rule; the
[glossary](GLOSSARY.md) points to the section that defines each term in it.
