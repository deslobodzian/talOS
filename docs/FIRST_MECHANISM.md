# Your first mechanism

From a fresh clone to a new mechanism, a "flywheel" with one motor, running
in simulation and following a scripted joystick button, then changed into
your own. Every command here was run on Linux; macOS differs only in how
`tools/setup.sh` installs tools. Two tracks share the setup: the Python track
comes first because it is shorter; the C++ track follows and reuses
everything but the node. [Make it yours](#5-make-it-yours) changes the
flywheel's code, and [position mechanisms](#6-position-mechanisms-the-library-or-your-own-code)
covers an arm with or without the mechanism library.

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

`2026-robot/` is the project's simulation testbed. The walkthrough builds a
robot of its own, `my_robot`, with the flywheel as its first mechanism:

```sh
bazel run //tools:new_robot -- my_robot --mechanism flywheel --lang py
```

```text
wrote my_robot/README.md
wrote my_robot/main_processor/configuration/robot.toml
wrote my_robot/main_processor/configuration/robot_sim.toml
wrote my_robot/main_processor/configuration/robot_sim_manifest.bzl
wrote my_robot/main_processor/configuration/subsystems/teleop.toml
wrote my_robot/main_processor/configuration/subsystems/telemetry.toml
wrote my_robot/main_processor/configuration/BUILD
wrote my_robot/main_processor/flywheel/flywheel_message.fbs
wrote my_robot/main_processor/flywheel/BUILD
wrote my_robot/main_processor/flywheel/flywheel.py
wrote my_robot/main_processor/flywheel/node_test.py
wrote my_robot/main_processor/flywheel/testdata/robot.toml
wrote my_robot/main_processor/configuration/subsystems/flywheel.toml
wrote my_robot/main_processor/configuration/robot_manifest.bzl
Next:
  ...
```

`--type` picks the mechanism's archetype: `velocity` (the default, a motor
driven at a speed: a flywheel, a roller) or `position` (a motor homed and
driven to a position: an arm, an elevator; see
[section 6](#6-position-mechanisms-the-library-or-your-own-code)).
Run it through `bazel run`; `python3 tools/new_robot.py` alone fails with
`No module named 'tools'`. `--dry-run` prints every file without writing.
`new_robot` writes the robot with its first mechanism; with
`--no-mechanism` in place of `--mechanism` and `--lang` it writes one with
no device (the hardware bridge, `teleop` and `telemetry`), whose
mechanisms `new_subsystem` adds later.
`my_robot/README.md` says what each file is; the ones that
matter now:

- A **node** is one program (one process) that does one job; a **subsystem**
  is the robot's name for a node plus its configuration. `[[subsystems]]` in
  `robot.toml` is the only list of a robot's nodes: here `teleop`,
  `telemetry` and `flywheel`. `teleop` is the framework's node that turns
  Driver Station buttons and sticks into targets, and `telemetry` feeds the
  logs and Studio; neither needs code from you
  ([architecture: configuration](../ARCHITECTURE.md#configuration)).
- `[robot] period_us` is the default **tick**: how often a node's `update()`
  runs. `command_timeout_us` is how old data may be before the robot goes
  neutral ([architecture: the tick](../ARCHITECTURE.md#the-tick)).
- `[robot.targets]` says which **target lanes** each Driver Station mode
  listens to. Here a subsystem obeys the `teleop` lane in teleop and nothing in
  autonomous. `require_target_coverage = true`, once every mechanism has a
  producer, makes the launcher refuse a subsystem nobody can drive; a
  commissioned robot always requires it
  ([architecture: targets](../ARCHITECTURE.md#targets)).
- `sim_gateway` names the program that stands in for the RoboRIO, the robot's
  hardware layer, when you simulate.
- `robot_sim.toml` is the **simulation config**: `include = "robot.toml"`
  plus what only a simulation may use, such as a button binding and the
  simulated motor's gains. `robot.toml` never reads it, so nothing you put
  there reaches the robot, and its `[robot] simulation_only = true` makes the
  launcher and `hardware_node` refuse it without `--sim`
  ([launcher](../talOS/launcher/README.md#lightweight-simulation-gateway)).
- `robot_manifest.bzl` is the **manifest**: a generated list of the node
  binaries and config files the robot needs. Bazel cannot read TOML, so it
  reads this file instead. Edit `robot.toml` or a subsystem file in a way
  that adds or removes a node or file, and regenerate it with
  `bazel run //tools:sync_manifest -- my_robot/main_processor/configuration/robot.toml`
  (a stale manifest fails the build, and `sim.sh`, with that command).
  `robot_sim_manifest.bzl` is the simulation config's.
- The configuration `BUILD` holds the robot's checks (`manifest_sync_test`,
  `graph_test`, and their `sim_` twins for `robot_sim.toml`) and
  `robot_bundle`, what [deployment](DEPLOYMENT.md) installs on the main
  processor.

## 3. Python track

The flywheel's package is `my_robot/main_processor/flywheel/`, written by
`//tools:new_subsystem`, which `new_robot` runs for the first mechanism. A
second mechanism comes from that tool directly, as in the C++ track. The
package is yours: the scaffold is a starting point, not a framework you
must keep ([section 5](#5-make-it-yours)).

### The message

`flywheel_message.fbs`, without the comments:

```fbs
namespace flywheel;

struct FlywheelTarget {
  velocity_rps:double;
}

struct FlywheelState {
  enabled:bool;
  velocity_rps:double;
  connected:bool;
}
```

A **topic** is a named shared-memory slot, `/flywheel/state`, holding the
latest **message** of one fixed type; nodes talk only through topics.
Messages are FlatBuffers *structs*: fixed-size records, which is what lets
them sit in shared memory. Every number carries its unit as a suffix (`_rps`,
`_m`, `_us`, `_v`), and the build fails on one without. Topic names follow
[NAMING.md](../talOS/NAMING.md#topics): `/owner/role[/qualifier]`, where the
role says who may write it (`state` is published by its owner; `target` is
written by anyone else).

### The node

`flywheel.py`, without the comments:

```python
class Flywheel:
    def __init__(self, node):
        self.motor = talos.Motor(node, "flywheel_motor")
        self.target = talos.Target(node, FlywheelTarget)
        self.state = node.publish(FlywheelState, topics.STATE)

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
  `/flywheel/state`.
- `update()` runs every tick. After it returns, the framework sends one
  hardware request for the node: a motor not set during that tick is sent
  neutral. That is why **motor setters work only inside `update()`**: a command
  from anywhere else would reach the motor for at most one tick, so the
  framework refuses it with an error
  ([node guide: threads](NODE_DEVELOPMENT_GUIDE.md#threads)).
- A **target lane** is `/flywheel/target/<source>`, one per producer
  (`teleop`, `auto`, ...). `Target.get()` returns the message from the first
  lane in the mode's priority list that is active, fresh, and issued under
  the current Driver Station mode, and only while `robot_enabled(node)` holds
  (the robot is enabled, with no fault and a hardware state younger than
  `command_timeout_us`). Otherwise it returns `None`, nothing is set, and the
  motor goes neutral. Safety needs no code from you; it is the shape of the
  node ([architecture: targets](../ARCHITECTURE.md#targets), and rule 1 of
  [RULES.md](../RULES.md)).

### Configure the subsystem

`my_robot/main_processor/configuration/subsystems/flywheel.toml` describes
the motor and the node's inputs. The scaffold leaves it unable to move: its
teleop binding is commented out and every gain is zero ("untuned"), so it
never drives a real mechanism on values nobody measured. To see it move in
simulation, append the block its `Next:` output prints to `robot_sim.toml`,
the simulation config; the subsystem file stays as it is. Nothing moves in
simulation without that block:

```toml
# flywheel: simulation values only. robot.toml never reads this file; before
# a real robot drives the flywheel, tune slot0 on the mechanism.
# Holding joystick button 3 writes velocity_rps = 20.0 on /flywheel/target/teleop.
[[subsystems.flywheel.teleop.bindings]]
button = 3
while_held = { velocity_rps = 20.0 }
# Until an autonomous node writes /flywheel/target/auto, the binding is the
# flywheel's only producer; a policy that lists "auto" with
# require_target_coverage = true refuses the graph without this.
[subsystems.flywheel.targets]
autonomous = []
# The simulated motor's feedforward (volts per rps), not the mechanism's:
# with every gain zero the physics simulation stalls the motor.
[subsystems.flywheel.motors.flywheel_motor]
slot0 = { p = 0.0, i = 0.0, d = 0.0, s = 0.0, v = 0.12, a = 0.0, g = 0.0 }
```

`[subsystems.flywheel]` is the flywheel's subsystem file as `robot.toml`
loads it, and these tables are merged over it. The binding tells the
framework's `teleop` node to write a target on `/flywheel/target/teleop`
while Driver Station joystick button 3 is held. Without it nothing writes the
flywheel's lane: the launcher warns (`/flywheel/target/teleop: is in the
target policy of flywheel but nothing writes it yet`), as it does for
`robot.toml`, and with `require_target_coverage = true` it refuses the graph.
The feedforward is there because with every gain zero the physics simulation
stalls the motor
([simulation backends](../talOS/hardware/README.md#simulation-backends-ideal-vs-physics-backed)).
A `--type position` mechanism's block also carries its simulated travel, home
switch and gains ([section 6](#6-position-mechanisms-the-library-or-your-own-code)).

The `can_id` in the subsystem file is a placeholder. On a real robot a
person sets the physical id and `commissioned = true` at the robot; nobody
does it from a desk ([bring-up](BRINGUP.md#human-commissioning)). A
commissioned robot requires target coverage, so by then every mechanism needs
a producer in each mode its policy lists. The
binding goes into the subsystem file only once its gains are tuned on the
mechanism.

### Run the generated test

```sh
bazel test //my_robot/main_processor/flywheel:all
```

```text
//my_robot/main_processor/flywheel:node_test                             PASSED in 1.4s
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
bazel run //talOS/launcher -- --config $PWD/my_robot/main_processor/configuration/robot_sim.toml --describe-only
```

```text
================ Declared Graph ==================
Nodes described: 4/4, topics: 8
  • hardware_node (5 sources)
  • flywheel (7 sources)
  • telemetry (5 sources)
  • teleop (3 sources)
  warning: /flywheel/state: is published but nothing subscribes to it
Graph written to: /tmp/talos_logs/<session>/graph.json
==================================================
[launcher] describe-only: nothing was started
```

`bazel test //my_robot/...` runs the same check (`sim_graph_test`, and
`graph_test` for `robot.toml`), the manifests' sync tests and the node test
together.

The launcher runs every node binary with `--describe`, which makes the node
build itself, report the topics it reads and writes, and exit, then checks the
whole graph against the naming rules
([launcher: two phases](../talOS/launcher/README.md#two-phases)). The build
comes first because the launcher runs the built binaries; before it, one
`error:` line names the nodes that are `not built` and ends with the
`bazel build` command that builds them.
A warning is fine; an `error:` line means the launcher would refuse to start.

### Run it in simulation

```sh
./sim.sh --config my_robot/main_processor/configuration/robot_sim.toml \
  --sim-gateway --no-studio --duration-s 8 \
  --mode-schedule 'disabled:1,teleop:10' \
  --joystick-schedule '0:axes=0;buttons=0;pov=-1|3:axes=0;buttons=4;pov=-1|6:axes=0;buttons=0;pov=-1'
```

```text
==> checking my_robot/main_processor/configuration/robot_sim.toml
==> building
INFO: Build completed successfully, 1 total action
==> starting node graph (log: /tmp/talos_logs/1794681463/sim/launcher.log)
==>   declared graph: /tmp/talos_logs/1794681463/graph.json
==> running. Ctrl-C stops everything.

sim.sh: shutting down
sim.sh: session logs in /tmp/talos_logs/1794681463 (launcher/RIO output in /tmp/talos_logs/1794681463/sim)
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
prints last, holds each node's log (`flywheel.tlog`), its text output
(`flywheel.log`) and the graph. `log_dump` reads the logs and decodes every
message. Its summary answers "did the flywheel move" in a few lines:

```sh
bazel build //talOS/launcher:log_dump
./bazel-bin/talOS/launcher/log_dump --format summary --node flywheel --topic /flywheel/state /tmp/talos_logs/1794681463
```

It prints how many records the flywheel logged on `/flywheel/state`, the
first and last publication times, the `latest` value, and one `range <field>
<min> <max>` line per numeric or bool field: `range velocity_rps 0 ...` with
a maximum near 20 means it spun up, `range enabled false true` that it was
enabled at some point ([summary format](../talOS/launcher/README.md#filtering-and-exporting-logs)).
To see the trajectory, sample the full dump:

```sh
./bazel-bin/talOS/launcher/log_dump --node flywheel --topic /flywheel/state /tmp/talos_logs/1794681463 \
  | grep FlywheelState | awk 'NR%50==1'
```

```text
      flywheel.FlywheelState {enabled: false, velocity_rps: 0, connected: true}
      flywheel.FlywheelState {enabled: false, velocity_rps: 0, connected: true}
      flywheel.FlywheelState {enabled: false, velocity_rps: 0, connected: true}
      flywheel.FlywheelState {enabled: true, velocity_rps: 10.43406201889547, connected: true}
      flywheel.FlywheelState {enabled: true, velocity_rps: 19.999999993251038, connected: true}
      flywheel.FlywheelState {enabled: true, velocity_rps: 19.999999999999986, connected: true}
      flywheel.FlywheelState {enabled: false, velocity_rps: 8.435812349999994, connected: true}
      flywheel.FlywheelState {enabled: false, velocity_rps: 0, connected: true}
```

The flywheel sat neutral while disabled, spun up to 20 rps while button 3
was held, and coasted down when the button released. Remove the `buttons=4`
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

As with `new_robot`, `--type` picks the archetype: `velocity` (the default,
as here), `position` ([section 6](#6-position-mechanisms-the-library-or-your-own-code))
or `estimator` (a pose estimator with no hardware). `--help` lists every
flag.

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
call in both languages). Append the claw's block from its `Next:` output to
`robot_sim.toml` as you did the flywheel's. Each mechanism needs a button of
its own: the scaffold picks the lowest one no mechanism of the robot binds
and prints it in `Next:`. Check that no two mechanisms bind the same button
after you edit one; change `button = ...` in the block you append to move a
mechanism (this track moved the claw to button 6). The launcher does not
warn about a shared button; one press then drives both. Then:

```sh
bazel test //my_robot/...
```

The Python track's `sim.sh` and `log_dump` commands work for the claw, with
the claw's button in the joystick schedule's mask (button `n` is bit `n-1`,
so button 6 is `buttons=32`) and the `claw` node read from the session
directory `sim.sh` printed:

```sh
./sim.sh --config my_robot/main_processor/configuration/robot_sim.toml \
  --sim-gateway --no-studio --duration-s 8 \
  --mode-schedule 'disabled:1,teleop:10' \
  --joystick-schedule '0:axes=0;buttons=0;pov=-1|3:axes=0;buttons=32;pov=-1|6:axes=0;buttons=0;pov=-1'
./bazel-bin/talOS/launcher/log_dump --format summary --node claw --topic /claw/state /tmp/talos_logs/<session>
./bazel-bin/talOS/launcher/log_dump --node claw --topic /claw/state /tmp/talos_logs/<session> \
  | grep ClawState | awk 'NR%50==1'
```

```text
      claw.ClawState {enabled: false, velocity_rps: 0, connected: true}
      claw.ClawState {enabled: false, velocity_rps: 0, connected: true}
      claw.ClawState {enabled: false, velocity_rps: 0, connected: true}
      claw.ClawState {enabled: false, velocity_rps: 0, connected: true}
      claw.ClawState {enabled: true, velocity_rps: 19.999999976099026, connected: true}
      claw.ClawState {enabled: true, velocity_rps: 19.999999999999986, connected: true}
      claw.ClawState {enabled: true, velocity_rps: 19.999999999999986, connected: true}
      claw.ClawState {enabled: false, velocity_rps: 0, connected: true}
```

The flywheel, on button 3, stays at 0 in that run.

## 5. Make it yours

The scaffold is where your code starts. This section makes one change end to
end on the Python flywheel: a second speed, a new state field, a behaviour of
your own, and its test. The C++ differences follow at the end.

### A second button

Add a second binding to the flywheel's block in `robot_sim.toml`, under the
first:

```toml
[[subsystems.flywheel.teleop.bindings]]
button = 4
while_held = { velocity_rps = 40.0 }
```

Each `[[...teleop.bindings]]` entry is one trigger. Holding button 4 now asks
for 40 rps; with both held, the binding listed first wins. Axes, the POV,
toggles and defaults are in [declarative teleop bindings](../talOS/teleop/README.md).
In `sim.sh`'s joystick schedule button 4 is `buttons=8`.

### A new state field

Add the goal and an "at speed" flag to `FlywheelState` in
`flywheel_message.fbs`:

```fbs
struct FlywheelState {
  enabled:bool;
  velocity_rps:double;
  connected:bool;
  goal_rps:double;
  at_speed:bool;
}
```

A number without a unit suffix fails the build. Named `goal:double`, the
field gives:

```text
talos_messages: //flywheel/flywheel_message.fbs: flywheel.FlywheelState.goal (Double) has no unit suffix
talos_messages: end every numeric field with a unit suffix (docs/METHODOLOGY.md §6): _ns _us _m _mps _mps2 _rad _radps _radps2 _rot _rps _rps2 _v _a _w _degc _kg _n _nm _frac _duty _ratio _count _index _id _mask _px
```

A `bool` needs none. Every field must then be passed to `send(...)`; until
`update()` passes the new ones, the node's tests fail with:

```text
TypeError: FlywheelState: missing fields goal_rps, at_speed; every field is required
```

### A behaviour of your own

`at_speed` is true once the measured speed has been within 5% of the goal
for 10 consecutive ticks. Count the ticks in `update()`, and start the count
in the constructor:

```python
class Flywheel:
    def __init__(self, node):
        self.motor = talos.Motor(node, "flywheel_motor")
        self.target = talos.Target(node, FlywheelTarget)
        self.state = node.publish(FlywheelState, topics.STATE)
        self.ticks_in_band = 0

    def update(self):
        t = self.target.get()  # None: disabled, stale or released
        drive = (t is not None and self.motor.connected
                 and math.isfinite(t.velocity_rps))
        goal = t.velocity_rps if drive else 0.0
        if drive:
            self.motor.set_velocity(goal)
        in_band = drive and abs(self.motor.velocity - goal) <= 0.05 * abs(goal)
        self.ticks_in_band = self.ticks_in_band + 1 if in_band else 0
        self.state.send(enabled=drive, velocity_rps=self.motor.velocity,
                        connected=self.motor.connected, goal_rps=goal,
                        at_speed=self.ticks_in_band >= 10)
```

The safety shape stays: the motor is set only while `Target.get()` returns a
target, so the new code cannot drive a disabled robot.

### Its test

Add a case to the `FlywheelTest` class in `node_test.py`, in the style of the
generated ones. `self.hw.set_motor(node, motor, position, velocity)` sets
what the motor reports; it keeps reporting that velocity until the next
call, whatever the node asks for (`self.hw.ideal_motors()` would instead make
it follow its demand at once). `hold` republishes a target every 20 ms, one
tick:

```python
    def test_at_speed_after_ten_ticks_within_five_percent(self):
        self.teleop()
        self.hw.set_motor("flywheel", "flywheel_motor", 0.0, 28.0)  # 7% low
        self.hold(30.0, 0.30)
        self.assertFalse(self.state().at_speed)
        self.hw.set_motor("flywheel", "flywheel_motor", 0.0, 29.0)  # 3% low
        self.hold(30.0, 0.10)  # five ticks: not yet
        self.assertFalse(self.state().at_speed)
        self.hold(30.0, 0.16)
        self.assertTrue(self.state().at_speed)
        self.hw.set_motor("flywheel", "flywheel_motor", 0.0, 25.0)  # dropped
        self.hold(30.0, 0.04)
        self.assertFalse(self.state().at_speed)
```

A new reading reaches the node one tick after `set_motor`, so the test checks
before and well after the tenth tick rather than on it.

### The loop

```sh
bazel test //my_robot/main_processor/flywheel:all
```

After the first build, an edit-and-test round takes about a second and a half
for this Python node and about three seconds for the C++ claw (measured on a
warm cache; your machine will differ). Run the simulation again with button
4 in the schedule and the summary shows `range goal_rps 0 40` and
`range at_speed false true`.

### In C++

The same change on the claw touches `claw_message.fbs`, `claw.h` and
`node_test.cc`. The state is built positionally,
`State{drive, motor_.velocity(), motor_.connected(), goal, ticks_in_band_ >= 10}`,
in the `.fbs` field order, with the counter an `int ticks_in_band_ = 0;`
member. Until every field is passed the compiler refuses it:

```text
.../claw/claw.h:37:67: error: no matching function for call to 'claw::ClawState::ClawState(<brace-enclosed initializer list>)'
      • candidate expects 5 arguments, 3 provided
```

In `node_test.cc` the call is `hw.set_motor("claw", "claw_motor", 0.0, 29.0);`
and the state `Latest().at_speed()`.

## 6. Position mechanisms: the library, or your own code

```sh
bazel run //tools:new_subsystem -- arm --lang cpp --type position --robot my_robot
```

writes an arm whose node is built on `talos::mechanisms::PositionMechanism`
from [`talOS/mechanisms`](../talOS/mechanisms/README.md), with a home switch,
`arm_home`. The library is optional: it saves you writing the parts every
position mechanism needs, and in exchange it decides them for you. It homes
in phases (unhomed, pushing toward the switch at `homing_v`, zeroing after
`home_samples` closed readings, then waiting at rest before the first move),
clamps every goal to `min_rot..max_rot`, and latches faults neutral (a homing
timeout or stall, a switch stuck closed mid-travel) until the Driver Station
reports disabled. Its README has every key and phase.

Its `Next:` block carries simulation keys a velocity block does not: the
simulated travel (`[subsystems.arm.arm] max_rot`), a home switch that closes
just below where the motor starts (`sim_motor` and `sim_closed_at_rot` on
`[subsystems.arm.digital_inputs.arm_home]`), and gains, so the arm homes by
driving to the switch, as on the robot.

### Writing the arm yourself

A node may drive its `Motor` directly instead. To replace the scaffolded
mechanism with your own code:

1. In `arm.h`, replace the `PositionMechanism` member with the devices it
   used, and `update()` with your control. One version, which zeroes on the
   switch once the arm is at rest and then runs Motion Magic inside a fixed
   travel:

   ```cpp
   #include <algorithm>
   #include <cmath>
   ...
     explicit Arm(talos::Node& node)
         : motor_{node, "arm"},
           home_{node, "arm_home"},
           target_{node},
           state_{node.publish<State>(topics::kState)} {}

     void update() {
       // The switch reads false when closed. Zero there once the arm is at
       // rest, so the first move does not start at homing speed.
       const bool on_switch = home_.connected() && !home_.get();
       if (!motor_.zeroed() && on_switch && std::abs(motor_.velocity()) < 0.05) {
         motor_.rezero(0.0);
       }
       const Target* t = target_.get();  // null: disabled, stale or released
       const bool drive = t != nullptr && motor_.connected() &&
                          std::isfinite(t->position_rot());
       double goal = 0.0;
       if (drive && motor_.zeroed()) {
         goal = std::clamp(t->position_rot(), 0.0, kMaxRot);
         motor_.set_motion_magic(goal);
       } else if (drive && !on_switch) {
         motor_.set_voltage(-0.5);  // toward the switch; on it, brake to rest
       }
       const bool at_goal = drive && motor_.zeroed() &&
                            std::abs(motor_.position() - goal) < 0.05;
       state_.send(State{drive, motor_.position(), motor_.connected(), at_goal,
                         motor_.zeroed() ? Homing::HOMED : Homing::HOMING});
     }

    private:
     static constexpr double kMaxRot = 2.0;
     talos::Motor motor_;
     talos::DigitalInput home_;
     talos::Target<Target> target_;
     talos::Publisher<State> state_;
   ```

   Drop the `position_mechanism.h` include and `HomingState()`. This arm has
   no homing timeout and never reports `FAILED`: what the library decided is
   now yours to write, or not.
2. In `BUILD`, remove `"//talOS/mechanisms"` from the `arm` library's `deps`.
3. Delete the `[arm]` table from `subsystems/arm.toml`, and
   `[subsystems.arm.arm]` from `robot_sim.toml`: those keys are the
   library's parameters. Left in, the launcher refuses the graph, one line
   per key, for example:

   ```text
   error: arm: parameter 'arm.homing' (.../subsystems/arm.toml) is read by no node's constructor; delete it from the config, or read it in the node's constructor with Node::param (node.param in Python)
   ```

   `[motors.arm]` and `[digital_inputs.arm_home]` (with its simulation keys)
   stay while the node opens them. A device the node no longer opens is not
   refused; delete its table anyway.
4. Fix the tests. Of the fourteen generated, four test the library's
   behaviour and fail:
   `DrivesAtALiveLane` and `OtherModesLaneIsIgnored` ask for goals beyond
   this arm's travel (rewrite them with goals inside it), and
   `HomesBeforeTheFirstGoal` and `HomingThatDoesNotMoveFails` test the
   library's homing (delete them, with the `StartOffHome()` helper and the
   constants only they use, and test your own homing instead). Delete the
   `[arm]` table from `testdata/arm.toml` too; the test's `Sim` does not
   refuse it, but nothing reads it.

Whatever your code does, a position or Motion Magic demand goes out neutral
until `Motor::rezero` holds for the motor, unless its config sets
`position_frame = "absolute"` ([architecture: the tick](../ARCHITECTURE.md#the-tick),
[rezero and position frames](NODE_DEVELOPMENT_GUIDE.md#rezero-and-position-frames)).
An arm that never zeroes never moves.

## 7. Next

- A worked example of three mechanisms that cooperate:
  [examples/superstructure](../examples/superstructure/README.md).
- A producer other than a button binding (autonomous, an aligner): a
  `TargetLane` ([node guide: targets](NODE_DEVELOPMENT_GUIDE.md#targets)).
- Settings read from the config (`node.param`), the full API, and the
  rules every node follows: [node guide](NODE_DEVELOPMENT_GUIDE.md) and
  [methodology](METHODOLOGY.md).
- Putting a mechanism on another robot, such as `2026-robot`: the scaffold
  takes `--robot DIR`. Its `Next:` names that robot's simulation config
  (`robot_physics_sim.toml` for `2026-robot`), or the lines that create one
  (with `[robot] simulation_only = true`).

When a step fails, the launcher's message names the rule; the
[glossary](GLOSSARY.md) points to the section that defines each term in it.
