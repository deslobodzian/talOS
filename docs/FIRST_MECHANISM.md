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

## 2. Make your robot

`2026-robot/` is the project's simulation testbed, and its graph is full: the
launcher refuses more than eight readers of one topic, and `/hw/state/driver_station`
already has eight. A ninth node there makes `--describe-only` fail with
`/hw/state/driver_station: has 9 readers; RTMS reader limit is 8`. So the
walkthrough builds a robot of its own, `my_robot`, with the wrist as its first
mechanism:

```sh
bazel run //tools:new_robot -- my_robot --mechanism wrist --lang py
```

```text
wrote my_robot/README.md
wrote my_robot/main_processor/configuration/robot.toml
wrote my_robot/main_processor/configuration/subsystems/teleop.toml
wrote my_robot/main_processor/configuration/subsystems/telemetry.toml
wrote my_robot/main_processor/configuration/BUILD
wrote my_robot/main_processor/wrist/wrist_message.fbs
wrote my_robot/main_processor/wrist/BUILD
wrote my_robot/main_processor/wrist/wrist.py
wrote my_robot/main_processor/wrist/node_test.py
wrote my_robot/main_processor/wrist/testdata/robot.toml
wrote my_robot/main_processor/configuration/subsystems/wrist.toml
wrote my_robot/main_processor/configuration/robot_manifest.bzl
Next:
  ...
```

Run it through `bazel run`; `python3 tools/new_robot.py` alone fails with
`No module named 'tools'`. `--dry-run` prints every file without writing. A
robot needs a mechanism from the start: the hardware configuration refuses a
robot with no motor. `my_robot/README.md` says what each file is; the ones that
matter now:

- A **node** is one program (one process) that does one job; a **subsystem**
  is the robot's name for a node plus its configuration. `[[subsystems]]` in
  `robot.toml` is the only list of a robot's nodes: here `teleop`,
  `telemetry` and `wrist`. `teleop` is the framework's node that turns Driver
  Station buttons and sticks into targets, and `telemetry` feeds the logs and
  Studio; neither needs code from you
  ([architecture: configuration](../ARCHITECTURE.md#configuration)).
- `[robot] period_us` is the default **tick**: how often a node's `update()`
  runs. `command_timeout_us` is how old data may be before the robot goes
  neutral ([architecture: the tick](../ARCHITECTURE.md#the-tick)).
- `[robot.targets]` says which **target lanes** each Driver Station mode
  listens to. Here a subsystem obeys the `teleop` lane in teleop and nothing in
  autonomous. `require_target_coverage = true`, once every mechanism has a
  producer, makes the launcher refuse a subsystem nobody can drive.
- `sim_gateway` names the program that stands in for the RoboRIO, the robot's
  hardware layer, when you simulate.
- `robot_manifest.bzl` is the **manifest**: a generated list of the node
  binaries and config files the robot needs. Bazel cannot read TOML, so it
  reads this file instead. Edit `robot.toml` or a subsystem file in a way
  that adds or removes a node or file, and regenerate it with
  `bazel run //tools:sync_manifest -- my_robot/main_processor/configuration/robot.toml`
  (a stale manifest fails the build, and `sim.sh`, with that command).
- The configuration `BUILD` holds the robot's checks (`manifest_sync_test`,
  `graph_test`) and `robot_bundle`, what [deployment](DEPLOYMENT.md)
  installs on the main processor.

## 3. Python track

The wrist's package is `my_robot/main_processor/wrist/`, written by
`//tools:new_subsystem`, which `new_robot` runs for the first mechanism. A
second mechanism comes from that tool directly, as in the C++ track.

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
motor and the node's inputs. The scaffold leaves it unable to move on its
own, so that it never drives a real mechanism untuned; its `Next:` output
lists each edit, in order, with what happens if it is skipped. Here two
edits make it move (a robot that requires target coverage and lists `auto`
in autonomous, as `2026-robot` does, also needs the `[targets]` table
uncommented with `autonomous = []`). Uncomment the teleop
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

Without the binding, nothing writes the wrist's lane: the launcher warns
(`/wrist/target/teleop: is in the target policy of wrist but nothing writes it
yet`), and with `require_target_coverage = true` it refuses the graph.

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
bazel build //talOS/launcher //talOS/bridge:hardware_node //my_robot/main_processor/...
bazel run //talOS/launcher -- --config $PWD/my_robot/main_processor/configuration/robot.toml --describe-only
```

```text
================ Declared Graph ==================
Nodes described: 4/4, topics: 8
  • hardware_node (5 sources)
  • telemetry (5 sources)
  • teleop (3 sources)
  • wrist (7 sources)
  warning: /wrist/state: is published but nothing subscribes to it
==================================================
[launcher] describe-only: nothing was started
```

`bazel test //my_robot/...` runs the same check (`graph_test`), the
manifest's `manifest_sync_test` and the node test together.

The launcher runs every node binary with `--describe`, which makes the node
build itself, report the topics it reads and writes, and exit, then checks the
whole graph against the naming rules
([launcher: two phases](../talOS/launcher/README.md#two-phases)). The build
comes first because the launcher runs the built binaries; before it, you see
`did not answer --describe (execv bazel-bin/...: No such file or directory)`.
A warning is fine; an `error:` line means the launcher would refuse to start.

### Run it in simulation

```sh
./sim.sh --config my_robot/main_processor/configuration/robot.toml \
  --sim-gateway --no-studio --duration-s 8 \
  --mode-schedule 'disabled:1,teleop:10' \
  --joystick-schedule '0:axes=0;buttons=0;pov=-1|3:axes=0;buttons=4;pov=-1|6:axes=0;buttons=0;pov=-1'
```

```text
==> checking my_robot/main_processor/configuration/robot.toml
==> building
INFO: Build completed successfully, 107 total actions
==> starting node graph (log: /tmp/talos_logs/1794458306/sim/launcher.log)
==>   declared graph: /tmp/talos_logs/1794458306/graph.json
==> running. Ctrl-C stops everything.

sim.sh: shutting down
sim.sh: session logs in /tmp/talos_logs/1794458306 (launcher/RIO output in /tmp/talos_logs/1794458306/sim)
```

`sim.sh` checks that the robot's manifest is current (a stale one stops it
with the regenerate command), builds the nodes it lists, and starts them with
the launcher and `--sim-gateway`'s simulator, which plays the Driver Station
and the RoboRIO. Without `--config` it runs `2026-robot`. The mode schedule is
`mode:seconds` phases (disabled for 1 s, then teleop). The joystick schedule is
`seconds:axes=...;buttons=MASK;pov=DEG` snapshots of joystick 0, each held
until the next: button 3 is bit 2 of the mask, so `buttons=4` presses it from
second 3 to second 6 ([syntax](../2026-robot/main_processor/simulation/README.md#scripted-joystick-and-fault-schedules)).
`./sim.sh --help` lists the rest.

### See the result

A **session** is one run of the launcher; its directory, the one `sim.sh`
prints last, holds each node's log (`wrist.tlog`), its text output
(`wrist.log`) and the graph. `log_dump` reads the logs and decodes every
message:

```sh
bazel build //talOS/launcher:log_dump
./bazel-bin/talOS/launcher/log_dump --node wrist --topic /wrist/state /tmp/talos_logs/1794458306 \
  | grep WristState | awk 'NR%50==1'
```

```text
      wrist.WristState {enabled: false, velocity_rps: 0, connected: true}
      wrist.WristState {enabled: false, velocity_rps: 0, connected: true}
      wrist.WristState {enabled: false, velocity_rps: 0, connected: true}
      wrist.WristState {enabled: true, velocity_rps: 5.4203240144, connected: true}
      wrist.WristState {enabled: true, velocity_rps: 19.999999990144765, connected: true}
      wrist.WristState {enabled: true, velocity_rps: 19.999999999999986, connected: true}
      wrist.WristState {enabled: false, velocity_rps: 8.438062162477493, connected: true}
      wrist.WristState {enabled: false, velocity_rps: 0, connected: true}
```

The wrist sat neutral while disabled, spun up to 20 rps while button 3 was
held, and coasted down when the button released. Remove the `buttons=4`
snapshot, or the mode schedule's `teleop`, and it stays neutral: that is the
safety path working. [Studio](../studio/README.md) is the live viewer:
leave out `--no-studio` and `sim.sh` starts it. Reading a log is the same on
every robot ([launcher: filtering and exporting logs](../talOS/launcher/README.md#filtering-and-exporting-logs)).

## 4. C++ track

Section 1 is the same. Without a robot yet, make it with a C++ mechanism,
`bazel run //tools:new_robot -- my_robot --mechanism claw --lang cpp`. If the
Python track already made `my_robot`, add a second mechanism to it; this
track uses `claw`:

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
wrote my_robot/main_processor/claw/testdata/neighbor.toml
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
bazel test //my_robot/...
```

The `sim.sh` and `log_dump` commands from the Python track work unchanged;
read the `claw` node instead, from the session directory `sim.sh` printed:

```sh
./bazel-bin/talOS/launcher/log_dump --node claw --topic /claw/state /tmp/talos_logs/<session> \
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
- Putting a mechanism on another robot, such as `2026-robot`, whose graph is
  full: the scaffold takes `--robot DIR`; keep each topic at eight readers or
  fewer and give the new subsystem a `[targets]` table if autonomous has no
  writer for it.

When a step fails, the launcher's message names the rule; the
[glossary](GLOSSARY.md) points to the section that defines each term in it.
