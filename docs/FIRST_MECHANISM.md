# Your first mechanism

From nothing to a robot repository of your own with a new mechanism, a
"flywheel" with one motor, running in simulation and following a scripted
joystick button, seen in Studio, then changed into your own. A robot lives
in its own repository and uses talOS as a Bazel module, `@talos`; talOS
itself stays a separate checkout you do not edit.

Every command here was run on Linux; macOS differs only in how
`tools/setup.sh` installs tools. The Python track comes first because it is
shorter; the C++ track reuses everything but the node.
[Make it yours](#7-make-it-yours) changes the flywheel's code, and
[position mechanisms](#9-position-mechanisms-the-library-or-your-own-code)
covers an arm with or without the mechanism library.

Terms are explained where they first matter; [the glossary](GLOSSARY.md) has
all of them.

## 1. Set up

Clone talOS, then, from that checkout:

```sh
./tools/setup.sh          # Linux or macOS; on Windows run it inside WSL2
./tools/setup.sh --check  # lists each tool and whether it is acceptable
```

`setup.sh` installs the compiler, Bazel (through Bazelisk), Python, Node and
the linters ([INSTALL.md](INSTALL.md) has flags and troubleshooting). Bazel is
the build tool: a *target* such as `//tools:new_robot` names something it
can build or run. A *workspace* is a directory with a `MODULE.bazel`; talOS is
one, and your robot repository will be another. The first build in a
workspace compiles what it needs of talOS, which takes the longest of
anything here; later builds reuse that work.

## 2. Make your robot repository

From the talOS checkout, make the repository beside it:

```sh
bazel run //tools:new_robot -- my_robot --standalone ../my_robot
```

```text
wrote /home/you/my_robot/.bazelrc
wrote /home/you/my_robot/.bazelversion
wrote /home/you/my_robot/.gitignore
wrote /home/you/my_robot/BUILD.bazel
wrote /home/you/my_robot/MODULE.bazel
wrote /home/you/my_robot/README.md
wrote /home/you/my_robot/robot.toml
wrote /home/you/my_robot/robot_manifest.bzl
wrote /home/you/my_robot/studio_test.py
wrote /home/you/my_robot/subsystems/telemetry.toml
wrote /home/you/my_robot/subsystems/teleop.toml
Next:
  cd /home/you/my_robot
  bazel test //...                      # graph, manifest, simulation and Studio checks
  ...
```

`--standalone DIR` is relative to the directory you run `bazel` from. The
tool refuses a directory inside the talOS checkout, one that already holds
files, and a name that breaks the [naming rules](../talOS/NAMING.md#nodes);
it runs no git command (`git init` when you want history; the repository's
README says how to hold talOS as a git submodule instead).

The repository is a copy of talOS's
[`examples/robot_repo`](../examples/robot_repo/README.md), the robot
repository talOS's own CI builds and tests, without its two example
mechanisms: a robot starts with no mechanism, so the first one you add gets
the first CAN id and joystick button, and you write its code. The files that
matter now:

- A **node** is one program (one process) that does one job; a **subsystem**
  is the robot's name for a node plus its configuration. `[[subsystems]]` in
  `robot.toml` is the only list of a robot's nodes: here `teleop` and
  `telemetry`. `teleop` is the framework's node that turns Driver Station
  buttons and sticks into targets, and `telemetry` feeds the logs and
  Studio; neither needs code from you
  ([architecture: configuration](../ARCHITECTURE.md#configuration)). The
  hardware bridge, the courier to the RoboRIO, joins every launch on its own.
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
  hardware layer, when you simulate. Everything of talOS is named by an
  `@talos//` label.
- `robot_manifest.bzl` is the **manifest**: a generated list of the node
  binaries and config files the robot needs. Bazel cannot read TOML, so it
  reads this file instead. Edit `robot.toml` or a subsystem file in a way
  that adds or removes a node or file, and regenerate it with
  `bazel run @talos//tools:sync_manifest -- robot.toml` (a stale manifest
  fails `manifest_sync_test` with that command).
- `BUILD.bazel` holds the robot's checks (`manifest_sync_test`,
  `graph_test`, `sim_smoke_test`, `studio_test`), `//:sim`, the robot against
  the simulator, and `robot_bundle`, what [deployment](DEPLOYMENT.md)
  installs on the main processor.
- `MODULE.bazel` points `@talos` at your talOS checkout with
  `local_path_override` (relative to the repository, so the two directories
  move together), and Studio's pages at that checkout's build of them.

## 3. Build and test it

From here on, every command runs in your robot repository:

```sh
cd ../my_robot
bazel test //...
```

```text
//:graph_test                                                            PASSED in 0.2s
//:manifest_sync_test                                                    PASSED in 0.1s
//:sim_smoke_test                                                        PASSED in 1.5s
//:studio_test                                                           PASSED in 1.4s

Executed 4 out of 4 tests: 4 tests pass.
```

The first run took about a minute on an eight-core workstation whose
downloads were already cached; a later run takes seconds.
`graph_test` runs every node with `--describe` and checks the whole graph
against the naming and graph rules
([launcher: two phases](../talOS/launcher/README.md#two-phases)).
`sim_smoke_test` runs `//:sim` until the robot is up, then one second more,
and fails unless every member ran once and exited 0 and the bridge received
State from the simulator. `studio_test` starts Studio's bridge with it and
asks for the driver dashboard ([section 8](#8-see-it-in-studio)).

## 4. Run it in simulation

```sh
bazel run //:sim -- --duration-s 5
```

```text
================ Declared Graph ==================
Nodes described: 4/4, topics: 6
  • hardware_node (4 sources)
  • telemetry (5 sources)
  • teleop (2 sources)
  • sim_gateway (1 sources)
  ...
================ Session Manifest ================
Session ID: 16301377678527133833
Output Dir: /tmp/talos_logs/16301377678527133833
...
  • teleop (pid=19396, exit=0, records=476, restarts=0, failed=no, log=/tmp/talos_logs/16301377678527133833/teleop.tlog)
  • sim_gateway (pid=19398, exit=0, records=0, restarts=0, failed=no, log=)
```

`//:sim` runs the launcher on `robot.toml` with the simulator,
`sim_gateway`, which plays the Driver Station and the RoboRIO. Without
`--duration-s` it runs until Ctrl-C. A **session** is one run of the
launcher; its directory, `Output Dir`, holds each node's log (`teleop.tlog`),
its text output and the graph. Nothing moves yet: the robot has no
mechanism.

## 5. Add a mechanism: the Python track

```sh
bazel run @talos//tools:new_subsystem -- flywheel --lang py
```

```text
wrote flywheel/flywheel_message.fbs
wrote flywheel/BUILD
wrote flywheel/flywheel.py
wrote flywheel/node_test.py
wrote flywheel/testdata/robot.toml
wrote subsystems/flywheel.toml
wrote robot.toml
wrote robot_manifest.bzl
Type: velocity, a motor held at a speed, such as a flywheel or roller.
  Other types (--type velocity|position|estimator):
    position: a motor moved to a position counted from a home switch, such as an arm, wrist or elevator
    estimator: a pose estimator from camera observations, with no device
Next:
  bazel test //flywheel:all
  ...
```

It wrote the node package `flywheel/`, its subsystem file, its
`[[subsystems]]` entry, and the regenerated manifest. `--type` picks the
mechanism's archetype: `velocity` (the default, a motor driven at a speed: a
flywheel, a roller) or `position` (a motor homed and driven to a position:
an arm, an elevator; see [section 9](#9-position-mechanisms-the-library-or-your-own-code)).
The package is yours: the scaffold is a starting point, not a framework you
must keep ([section 7](#7-make-it-yours)).

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

### Run the generated test

```sh
bazel test //flywheel:all
```

```text
//flywheel:node_test                                                     PASSED in 1.3s
```

The scaffold already tests the case list every node must pass: a disabled
robot, a stale or released lane, a lost Driver Station, a disconnected motor, a
NaN demand, another mode's lane, a target left over from an earlier enable
([node test matrix](TESTING.md#7-the-node-test-matrix)). The test runs your
class on `talos.testing.Sim`, a simulated clock with in-memory topics, and
`SimHardware`, a stand-in for the hardware bridge; neither needs a robot or a
process. When you change what the node does, change `node_test.py` with it.
`bazel test //...` runs it with the robot's other checks; the graph test now
describes the flywheel too.

### Make it move in simulation

`subsystems/flywheel.toml` describes the motor and the node's inputs. The
scaffold leaves it unable to move: its teleop binding is commented out and
every gain is zero ("untuned"), so it never drives a real mechanism on
values nobody measured. To see it move in simulation, put what its `Next:`
output prints in `robot_sim.toml`, the **simulation config**, beside
`robot.toml`; the subsystem file stays as it is. The first mechanism's
`Next:` prints the whole file:

```toml
# Simulation only: the launcher runs it with --sim
# robot.toml never reads this file, so nothing here reaches the robot.
include = "robot.toml"

[robot]
# The launcher and hardware_node refuse this config without --sim.
simulation_only = true
# flywheel: simulation values only. robot.toml never reads this file; before
# a real robot drives the flywheel, tune slot0 on the mechanism.
# Holding joystick button 3 writes velocity_rps = 20.0 on /flywheel/target/teleop.
[[subsystems.flywheel.teleop.bindings]]
button = 3
while_held = { velocity_rps = 20.0 }
# Until an autonomous node writes /flywheel/target/auto, the binding is the
# flywheel's only producer; a policy that lists "auto" with
# require_target_coverage = true refuses the graph without this. Its lane
# is listed here, since the subsystem file may list none.
[subsystems.flywheel.targets]
teleop = ["teleop"]
autonomous = []
# The simulated motor's feedforward (volts per rps), not the mechanism's:
# with every gain zero the physics simulation stalls the motor.
[subsystems.flywheel.motors.flywheel_motor]
slot0 = { p = 0.0, i = 0.0, d = 0.0, s = 0.0, v = 0.12, a = 0.0, g = 0.0 }
```

`include = "robot.toml"` takes the whole robot, and `[subsystems.flywheel]`
is the flywheel's subsystem file as `robot.toml` loads it, with these tables
merged over it. `robot.toml` never reads `robot_sim.toml`, so nothing you put
there reaches the robot, and `simulation_only = true` makes the launcher and
`hardware_node` refuse it without `--sim`
([launcher](../talOS/launcher/README.md#lightweight-simulation-gateway)).
The binding tells the framework's `teleop` node to write a target on
`/flywheel/target/teleop` while Driver Station joystick button 3 is held.
Without it nothing writes the flywheel's lane, and the launcher warns. The
feedforward is there because with every gain zero the physics simulation
stalls the motor
([simulation backends](../talOS/hardware/README.md#simulation-backends-ideal-vs-physics-backed)).

The new config needs its own manifest, and the launcher checks its graph:

```sh
bazel run @talos//tools:sync_manifest -- robot_sim.toml
bazel run @talos//talOS/launcher -- --config $PWD/robot_sim.toml --describe-only
```

```text
wrote robot_sim_manifest.bzl
...
================ Declared Graph ==================
Nodes described: 4/4, topics: 8
  • hardware_node (5 sources)
  • flywheel (7 sources)
  • telemetry (5 sources)
  • teleop (3 sources)
  warning: /flywheel/state: is published but nothing subscribes to it
Graph written to: /tmp/talos_logs/<session>/graph.json
==================================================
```

A warning is fine; an `error:` line means the launcher would refuse to
start. The launcher runs the built node binaries with `--describe`; before
`bazel test //...` has built them, one `error:` line names the nodes that are
`not built`.

The `can_id` in the subsystem file is a placeholder. On a real robot a
person sets the physical id and `commissioned = true` at the robot; nobody
does it from a desk ([bring-up](BRINGUP.md#human-commissioning)). A
commissioned robot requires target coverage, so by then every mechanism needs
a producer in each mode its policy lists. The binding goes into the
subsystem file only once its gains are tuned on the mechanism. In a robot
that already requires coverage, the scaffold writes the subsystem file's
`[targets]` uncommented with every mode empty, since the coverage check
judges only a mode that lists a lane: the graph passes and the mechanism
stays neutral.

### Run it

`//:sim` runs `robot.toml`, so the simulation config runs with the launcher
itself, as `Next:` prints it:

```sh
bazel build @talos//talOS/bridge:hardware_node @talos//talOS/simulation:sim_gateway
bazel run @talos//talOS/launcher -- --config $PWD/robot_sim.toml --sim --duration-s 8 \
    --start-sim-gateway --sim-gateway-arg --physics \
    --sim-gateway-arg --mode-schedule --sim-gateway-arg 'disabled:1,teleop:10' \
    --sim-gateway-arg --joystick-schedule --sim-gateway-arg '0:axes=0;buttons=0;pov=-1|3:axes=0;buttons=4;pov=-1|6:axes=0;buttons=0;pov=-1'
```

```text
...
Session ID: 4330296920975887547
Output Dir: /tmp/talos_logs/4330296920975887547
...
  • flywheel (pid=24972, exit=0, records=4036, restarts=0, failed=no, log=/tmp/talos_logs/4330296920975887547/flywheel.tlog)
```

The mode schedule is `mode:seconds` phases (disabled for 1 s, then teleop).
The joystick schedule is `seconds:axes=...;buttons=MASK;pov=DEG` snapshots
of joystick 0, each held until the next: button 3 is bit 2 of the mask, so
`buttons=4` presses it from second 3 to second 6
([syntax](../talOS/simulation/README.md#scripted-joystick-and-fault-schedules)).

### See the result

`log_dump` reads a session's logs and decodes every message. Its summary
answers "did the flywheel move" in a few lines:

```sh
bazel run @talos//talOS/launcher:log_dump -- --format summary --node flywheel --topic /flywheel/state /tmp/talos_logs/4330296920975887547
```

```text
topic /flywheel/state flywheel.FlywheelState
  records flywheel SEND 397
  first_ms 56.644
  last_ms 7976.627
  latest {enabled: false, velocity_rps: 0, connected: true}
  range enabled false true
  range velocity_rps 0 19.999999999999986
  range connected true true
```

`range velocity_rps 0 ...` with a maximum near 20 means it spun up,
`range enabled false true` that it was enabled at some point
([summary format](../talOS/launcher/README.md#filtering-and-exporting-logs)).
To see the trajectory, sample the full dump:

```sh
bazel run @talos//talOS/launcher:log_dump -- --node flywheel --topic /flywheel/state /tmp/talos_logs/4330296920975887547 \
  | grep FlywheelState | awk 'NR%50==1'
```

```text
      flywheel.FlywheelState {enabled: false, velocity_rps: 0, connected: true}
      flywheel.FlywheelState {enabled: false, velocity_rps: 0, connected: true}
      flywheel.FlywheelState {enabled: false, velocity_rps: 0, connected: true}
      flywheel.FlywheelState {enabled: true, velocity_rps: 5.41967602880064, connected: true}
      flywheel.FlywheelState {enabled: true, velocity_rps: 19.999999989713697, connected: true}
      flywheel.FlywheelState {enabled: true, velocity_rps: 19.999999999999986, connected: true}
      flywheel.FlywheelState {enabled: false, velocity_rps: 8.436937462502495, connected: true}
      flywheel.FlywheelState {enabled: false, velocity_rps: 0, connected: true}
```

The flywheel sat neutral while disabled, spun up to 20 rps while button 3
was held, and coasted down when the button released. Remove the `buttons=4`
snapshot, or the mode schedule's `teleop`, and it stays neutral: that is the
safety path working.

## 6. The C++ track

A second mechanism comes from the same tool; this one is a C++ `claw`:

```sh
bazel run @talos//tools:new_subsystem -- claw --lang cpp
```

```text
wrote claw/claw_message.fbs
wrote claw/BUILD
wrote claw/claw.h
wrote claw/claw.cc
wrote claw/node_test.cc
wrote claw/testdata/robot.toml
wrote claw/testdata/claw.toml
wrote claw/testdata/neighbor.toml
wrote subsystems/claw.toml
wrote robot.toml
wrote robot_manifest.bzl
wrote robot_sim_manifest.bzl
...
```

It regenerates `robot_sim_manifest.bzl` too, since `robot_sim.toml` includes
`robot.toml`. The message file has the same shape as the Python one. The
node is a header plus a one-line entry point, `TALOS_NODE(claw::Claw)` in
`claw.cc`:

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
call in both languages). This time `Next:` prints lines to append to
`robot_sim.toml`; append them as printed. Each mechanism needs a button of
its own: the scaffold gives a new mechanism the lowest button no binding in
the robot's configs uses, and prints it in `Next:`. Here that is button 4,
since the flywheel holds 3. The launcher does not warn about a shared
button, and one press then drives both, so when you change a binding, check
that no two mechanisms bind the same button. Then:

```sh
bazel test //...
```

The run and `log_dump` commands work for the claw, with the claw's button in
the joystick schedule's mask (button `n` is bit `n-1`, so button 4 is
`buttons=8`, as the claw's `Next:` prints) and the `claw` node read from the
session the launcher printed:

```sh
bazel run @talos//talOS/launcher -- --config $PWD/robot_sim.toml --sim --duration-s 8 \
    --start-sim-gateway --sim-gateway-arg --physics \
    --sim-gateway-arg --mode-schedule --sim-gateway-arg 'disabled:1,teleop:10' \
    --sim-gateway-arg --joystick-schedule --sim-gateway-arg '0:axes=0;buttons=0;pov=-1|3:axes=0;buttons=8;pov=-1|6:axes=0;buttons=0;pov=-1'
bazel run @talos//talOS/launcher:log_dump -- --node claw --topic /claw/state /tmp/talos_logs/<session> \
  | grep ClawState | awk 'NR%50==1'
```

```text
      claw.ClawState {enabled: false, velocity_rps: 0, connected: true}
      claw.ClawState {enabled: false, velocity_rps: 0, connected: true}
      claw.ClawState {enabled: false, velocity_rps: 0, connected: true}
      claw.ClawState {enabled: false, velocity_rps: 0, connected: true}
      claw.ClawState {enabled: true, velocity_rps: 19.999999976116708, connected: true}
      claw.ClawState {enabled: true, velocity_rps: 19.999999999999986, connected: true}
      claw.ClawState {enabled: true, velocity_rps: 19.999999999999986, connected: true}
      claw.ClawState {enabled: false, velocity_rps: 0, connected: true}
```

The flywheel, on button 3, stays at 0 in that run.

## 7. Make it yours

The scaffold is where your code starts. This section makes one change end to
end on the Python flywheel: a second speed, a new state field, a behaviour of
your own, and its test. The C++ differences follow at the end.

### A second button

Add a second binding to the flywheel's block in `robot_sim.toml`, under the
first. It needs a button no mechanism holds: the flywheel has 3 and the claw
4, so use 5, the lowest free one:

```toml
[[subsystems.flywheel.teleop.bindings]]
button = 5
while_held = { velocity_rps = 40.0 }
```

Each `[[...teleop.bindings]]` entry is one trigger. Holding button 5 now asks
for 40 rps; with both held, the binding listed first wins. Axes, the POV,
toggles and defaults are in [declarative teleop bindings](../talOS/teleop/README.md).
In the joystick schedule button 5 is `buttons=16`. The scaffold reads this
binding too, so the next mechanism it writes gets button 6.

### A new state field

Add the goal and an "at speed" flag to `FlywheelState` in
`flywheel/flywheel_message.fbs`:

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

Add a case to the `FlywheelTest` class in `flywheel/node_test.py`, in the
style of the generated ones. `self.hw.set_motor(node, motor, position,
velocity)` sets what the motor reports; it keeps reporting that velocity
until the next call, whatever the node asks for (`self.hw.ideal_motors()`
would instead make it follow its demand at once). `hold` republishes a
target every 20 ms, one tick:

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
bazel test //flywheel:all
```

After the first build, an edit-and-test round of this Python node took
about a second (measured on a warm cache; your machine will differ). Run the
simulation again with `buttons=16` in the schedule, and the summary shows:

```text
  latest {enabled: false, velocity_rps: 0, connected: true, goal_rps: 0, at_speed: false}
  range enabled false true
  range velocity_rps 0 39.99999999999997
  range connected true true
  range goal_rps 0 40
  range at_speed false true
```

### In C++

The same change on the claw touches `claw/claw_message.fbs`, `claw/claw.h`
and `claw/node_test.cc`. The state is built positionally,
`State{drive, motor_.velocity(), motor_.connected(), goal, ticks_in_band_ >= 10}`,
in the `.fbs` field order, with the counter an `int ticks_in_band_ = 0;`
member. Until every field is passed the compiler refuses it:

```text
.../claw/claw.h:37:67: error: no matching function for call to 'claw::ClawState::ClawState(<brace-enclosed initializer list>)'
      • candidate expects 5 arguments, 3 provided
```

In `node_test.cc` the call is `hw.set_motor("claw", "claw_motor", 0.0, 29.0);`
and the state `Latest().at_speed()`.

## 8. See it in Studio

[Studio](../studio/README.md) is the live viewer, and its driver dashboard
is at `/drive`. Its bridge is a C++ program your repository builds from
`@talos`; its pages are built by talOS's own workspace, in your talOS
checkout, once and again after each talOS update (`../talos` here, a
checkout named `talos` beside the repository; `new_robot`'s `Next:` printed
the command with your checkout's path):

```sh
(cd ../talos && bazel build //studio:web_dist)
```

```text
Target //studio:web_dist up-to-date:
  bazel-bin/studio/dist
```

That folder is what `MODULE.bazel`'s `studio_web` names. Until it is built,
the robot still runs and `bazel test //...` passes, but Studio's `/` and
`/drive` answer 503 with "Studio's pages are not built" and the build
command for the talOS checkout `studio_web` reads: the command above.
`studio_test` passes
either way: it accepts that 503 as long as it names the folder `robot.toml`
gives the bridge. Bazel notices the pages when they appear, so the next run
serves them.

`bazel run //:sim -- --start-studio-bridge` starts Studio with `robot.toml`,
where nothing is bound to a button. To watch the flywheel move, run the
simulation config with the bridge, after building `//:sim`, which builds
the bridge and the pages' folder (`//:studio_webroot`, `robot.toml`'s
`[studio] webroot`):

```sh
bazel build //:sim
bazel run @talos//talOS/launcher -- --config $PWD/robot_sim.toml --sim --duration-s 10 --start-studio-bridge \
    --start-sim-gateway --sim-gateway-arg --physics \
    --sim-gateway-arg --mode-schedule --sim-gateway-arg 'disabled:1,teleop:10' \
    --sim-gateway-arg --joystick-schedule --sim-gateway-arg '0:axes=0;buttons=0;pov=-1|3:axes=0;buttons=4;pov=-1|6:axes=0;buttons=0;pov=-1'
```

```text
[launcher] Started studio_bridge (pid 39754) on http://localhost:5800
```

Open `http://localhost:5800` while it runs (raise `--duration-s`, or leave
it out and stop with Ctrl-C), and `http://localhost:5800/drive` for the
driver dashboard. Reading a log is the same on every robot
([launcher: filtering and exporting logs](../talOS/launcher/README.md#filtering-and-exporting-logs)).

## 9. Position mechanisms: the library, or your own code

```sh
bazel run @talos//tools:new_subsystem -- arm --lang cpp --type position
```

writes an arm whose node is built on `talos::mechanisms::PositionMechanism`
from [`talOS/mechanisms`](../talOS/mechanisms/README.md), with a home switch,
`arm_home`. The library is optional: it saves you writing the parts every
position mechanism needs, and in exchange it decides them for you. It homes
in phases (unhomed, pushing toward the switch at `homing_v`, zeroing after
`home_samples` closed readings, then waiting at rest before the first move),
clamps every goal to `min_rot..max_rot`, and latches faults neutral (a homing
timeout or stall, a switch stuck closed mid-travel) until the Driver Station
reports disabled. Its README has every key and phase. The scaffold's
summary line reads `Type: position, a motor moved to a position counted
from a home switch, such as an arm, wrist or elevator.`, and the next line
names the choice: `Written without --raw: on talOS/mechanisms'
PositionMechanism: less code, behaviour shared and tested there.`

Its `Next:` block carries simulation keys a velocity block does not: the
simulated travel (`[subsystems.arm.arm] max_rot`), a home switch that closes
just below where the motor starts (`sim_motor` and `sim_closed_at_rot` on
`[subsystems.arm.digital_inputs.arm_home]`), and gains, so the arm homes by
driving to the switch, as on the robot. Appended to `robot_sim.toml` and run
with button 6 (`buttons=32`; the flywheel holds 3 and 5, the claw 4), it
homes, moves to 1.0 and reports `at_goal`:

```text
topic /arm/state arm.ArmState
  ...
  latest {enabled: false, position_rot: 1.0000010965875594, connected: true, at_goal: false, homing: HOMED}
  range enabled false true
  range position_rot -0.16360441150836713 1.0107984456544017
  range connected true true
  range at_goal false true
```

### Writing the arm yourself

A node may drive its `Motor` directly instead. Then the cases the library
handles for you are yours: owning the node means owning them. An arm that
homes on a switch must at least bound its push toward the switch, check
that the switch opens when the arm leaves it, latch a failure, and push
gently. To start from a node that does all four in your own code, scaffold
the arm with `--raw` instead of the command above:

```sh
bazel run @talos//tools:new_subsystem -- arm --lang cpp --type position --raw
```

Every line of the homing and limits is then in `arm/arm.h` (`arm.py` with
`--lang py`, line for line the same), on the node's own `Motor`, home
switch `arm_home` and `talos::DriverStation`; the package does not depend
on `talOS/mechanisms`. The summary line adds `Written with --raw: every
line of homing and limits written in your own file, with its tests.`

The node zeroes the motor on the switch once the arm is at rest
(`Motor::rezero`), then runs Motion Magic to the goal inside a fixed
travel, and reports `FAILED` in `ArmState`'s `homing` field on either fault
below. It reads the Driver Station only to clear a failure. The constants at
the end of the class (`kMaxRot`, `kHomingVolts`, `kMaxPushTicks`,
`kHomeMarginRot`; upper case at the top of the Python file) are
placeholders sized for the simulation; set each for your mechanism:

- **The push is bounded.** A switch that never closes (a cut wire reads
  open) would otherwise push the arm into its bottom stop for as long as
  the button is held. In the simulation the push moves the arm about
  4 rot/s, so it crosses the 2-rotation travel (`kMaxRot`) and the 0.1
  rotations to the switch in about half a second; `kMaxPushTicks` allows
  twice that. On your mechanism, time a push across the full travel at
  `kHomingVolts` and allow about twice that, in ticks of the subsystem's
  `period_us`.
- **A stuck switch is caught early.** A switch stuck or shorted closed
  with the arm mid-travel at power-on zeroes the arm there, and Motion
  Magic would then take it up to `kMaxRot` past the real top. So after
  each zero, the arm goes no farther than `kHomeMarginRot` above it until
  the switch has read open; still closed within 0.05 of that, the switch
  cannot be right, and the arm fails. That is what is not caught: from a
  wrong zero the arm can still move up to `kHomeMarginRot`, which from a
  zero near the top is into the far stop, at the stator limit. Keep
  `kHomeMarginRot` as small as your switch allows: the distance from
  the zero at which it reliably opens, plus a little. In the simulation
  it opens about 0.06 rotations up.
- **Failed stays failed** (neutral) until the Driver Station reports
  disabled. Releasing the button, a stale lane, a non-finite target or a
  motor that briefly reads disconnected does not clear it, so none of
  them starts another push into the stop. `Target::get()` returns null for
  a release and a stale lane alike, so the node cannot tell them apart.
- **The push is gentle.** `kHomingVolts` is low, and the motor's
  `stator_limit_a` bounds how hard any command, the push included, can
  drive the arm into a stop. The subsystem file sets a provisional 20 A,
  not the motor's 80 A; choose the real value for your mechanism. Soft
  limits do not bound this travel: they stay in the motor's power-on
  frame, which a rezero does not move, and config load refuses them
  unless `position_frame = "absolute"`.

The subsystem file has no `[arm]` table, and the `Next:` simulation block
no `[subsystems.arm.arm]`: those keys are the library's parameters, and
the launcher refuses a key no node reads. `node_test.cc` carries the node
test matrix, with goals inside the travel, and five tests of the node's
own homing and limits: `HomingThatNeverReachesTheSwitchFails`,
`FailureLatchesUntilDisabled`, `StuckSwitchStopsWithinTheMargin`,
`GoalOutsideTheTravelClampsToIt` (goals of 5.0 and -1.0 demand 2.0 and 0)
and `ZeroWaitsForRest` (no zero while the motor still moves on the
switch). The
fixture starts the arm on its switch (closed); `StartOffHome()` opens it.
`hw.ideal_motors()` makes the simulated motor reach each demand at once,
so in the last test the switch, never opened, is still closed with the arm
0.15 above a zero taken 1.9 rotations up its travel. The tests' times and
positions follow the constants: change them together. The
[mechanisms README](../talOS/mechanisms/README.md#positionmechanism)
treats each case more fully (homing stall checks, settling, timeouts).

The raw arm's `Next:` also gives it button 6. To see the bounded push, set
`sim_closed_at_rot = -10.0` in the arm's block, a switch the push never
reaches, run with `buttons=32`, and read the session:

```sh
bazel run @talos//talOS/launcher:log_dump -- --node arm --topic /arm/state /tmp/talos_logs/<session> \
  | grep -E 'enabled: true|FAILED' | awk 'NR%10==1' | head -7
```

```text
      arm.ArmState {enabled: true, position_rot: 0, connected: true, at_goal: false, homing: HOMING}
      arm.ArmState {enabled: true, position_rot: -0.6280817140569875, connected: true, at_goal: false, homing: HOMING}
      arm.ArmState {enabled: true, position_rot: -1.4583854001985153, connected: true, at_goal: false, homing: HOMING}
      arm.ArmState {enabled: true, position_rot: -2.29171139162933, connected: true, at_goal: false, homing: HOMING}
      arm.ArmState {enabled: true, position_rot: -3.12503989562298, connected: true, at_goal: false, homing: HOMING}
      arm.ArmState {enabled: false, position_rot: -3.9583273858280004, connected: true, at_goal: false, homing: FAILED}
      arm.ArmState {enabled: false, position_rot: -4.041465745773393, connected: true, at_goal: false, homing: FAILED}
```

It pushed for one second, stopped and reported `FAILED`, neutral, and still
reported `FAILED` after the button was released: only a disable clears it.
Put `sim_closed_at_rot` back to `-0.1` afterwards.

Whatever your code does, a position or Motion Magic demand goes out neutral
until `Motor::rezero` holds for the motor, unless its config sets
`position_frame = "absolute"` ([architecture: the tick](../ARCHITECTURE.md#the-tick),
[rezero and position frames](NODE_DEVELOPMENT_GUIDE.md#rezero-and-position-frames)).
An arm that never zeroes never moves.

## 10. Next

- A worked example of three mechanisms that cooperate:
  [examples/superstructure](../examples/superstructure/README.md).
- A producer other than a button binding (autonomous, an aligner): a
  `TargetLane` ([node guide: targets](NODE_DEVELOPMENT_GUIDE.md#targets)).
- Settings read from the config (`node.param`), the full API, and the
  rules every node follows: [node guide](NODE_DEVELOPMENT_GUIDE.md) and
  [methodology](METHODOLOGY.md).
- Holding talOS as a git submodule, so the repository records the talOS
  commit it builds against: your repository's `README.md`, "Where talOS
  comes from", and [examples/robot_repo](../examples/robot_repo/README.md).

When a step fails, the launcher's message names the rule; the
[glossary](GLOSSARY.md) points to the section that defines each term in it.

## Inside talOS (framework developers)

talOS's own tests and its testbed robot, `2026-robot/`, keep robots inside
the talOS workspace. `bazel run //tools:new_robot -- <name> --mechanism
<mech> --lang cpp|py` (or `--no-mechanism`) writes one there, beside
`2026-robot/`, and `bazel run //tools:new_subsystem -- <name> --lang cpp|py
--robot <name>` adds a mechanism to it; `./sim.sh --config
<name>/main_processor/configuration/robot_sim.toml --sim-gateway` runs it.
`bazel run //tools:new_robot -- --help` lists the options. A team's robot
does not belong there: it is the repository this page makes.
