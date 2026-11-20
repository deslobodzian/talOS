# Your first mechanism

From nothing to a robot repository of your own with a new mechanism, a
"flywheel" with one motor, following a scripted joystick button in
simulation, seen in Studio, then changed into your own. A robot lives in its
own repository and uses talOS as a Bazel module, `@talos`; talOS stays a
separate checkout you do not edit. Every command here was run on Linux;
macOS differs only in how `tools/setup.sh` installs tools.

The main path, sections [1](#1-set-up) to [4](#4-make-it-move-in-simulation),
is seven commands, the first of them the install, and lines you append to
one file.
When a step fails,
[section 5](#5-if-it-does-not-work) lists the checks. Terms are explained
where they first matter; [the glossary](GLOSSARY.md) has all of them.

## 1. Set up

Clone talOS, then, from that checkout:

```sh
./tools/setup.sh          # Linux or macOS; on Windows run it inside WSL2
```

It installs the compiler, Bazel (through Bazelisk), Python and Node
([INSTALL.md](INSTALL.md) has flags and troubleshooting). Bazel is the build
tool: a *target* such as `//tools:new_robot` names something it can build or
run, and a *workspace* is a directory with a `MODULE.bazel`. talOS is one;
your robot repository will be another.

## 2. Make your robot repository

From the talOS checkout, make the repository beside it, then go there:

```sh
bazel run //tools:new_robot -- my_robot --standalone ../my_robot
cd ../my_robot
```

```text
wrote /home/you/my_robot/.bazelignore
wrote /home/you/my_robot/.bazelrc
...
wrote /home/you/my_robot/subsystems/teleop.toml
Next:
  cd /home/you/my_robot
  ...
```

`--standalone DIR` is relative to the directory you run `bazel` from. The
tool refuses a directory inside the talOS checkout, one that already holds
files, and a name that breaks the [naming rules](../talOS/NAMING.md#nodes);
it runs no git command (`git init` when you want history; the repository's
`README.md` says how to hold talOS as a git submodule instead).

The repository is a copy of talOS's
[`examples/robot_repo`](../examples/robot_repo/README.md), the robot
repository talOS's own CI builds and tests, without its example mechanisms:
your first mechanism gets the first CAN id and joystick button, and you
write its code. What is in it:

- `robot.toml` is the robot. `[[subsystems]]` is the only list of its
  nodes: here `teleop`, the framework's node that turns Driver Station
  buttons and sticks into targets, and `telemetry`, which feeds the logs
  and Studio. A **node** is one program that does one job; a **subsystem**
  is a node plus its configuration, one file each in `subsystems/`. The
  hardware bridge, the courier to the RoboRIO, joins every launch on its
  own ([architecture: configuration](../ARCHITECTURE.md#configuration)).
  `[robot] period_us` is the default **tick**, how often a node's
  `update()` runs; `command_timeout_us` is how old data may be before the
  robot goes neutral ([the tick](../ARCHITECTURE.md#the-tick));
  `[robot.targets]` says which **target lanes** each Driver Station mode
  obeys ([targets](../ARCHITECTURE.md#targets)).
- `robot_manifest.bzl` is the **manifest**, the node binaries and config
  files `robot.toml` needs, generated because Bazel cannot read TOML. The
  tools regenerate it; a stale one fails `manifest_sync_test` with the
  command that fixes it.
- `BUILD.bazel` holds the checks `bazel test //...` runs (`graph_test`,
  which lints the whole graph as the launcher does; `manifest_sync_test`;
  `sim_smoke_test`, which runs the robot against the simulator until it is
  up; `studio_test`; `bundle_install_test`, which installs `robot_bundle`
  into a scratch directory and checks that the installed copy describes
  every node; `talos_pin_test`, which checks that `.bazelrc`,
  `.bazelversion` and the default Python match the talOS checkout `@talos`
  is built from; `trace_test`, which checks every requirement tag against
  talOS's catalogue), `//:sim`, the robot against the simulator, and
  `robot_bundle`, what [deployment](DEPLOYMENT.md) installs.
- `MODULE.bazel` points `@talos` at your talOS checkout, relative to the
  repository, so the two directories move together (absolute, with a
  warning that it holds only on this machine, when they share nothing
  but `/`).

## 3. Add a mechanism

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
wrote robot_sim.toml
wrote robot_sim_manifest.bzl
wrote BUILD.bazel
Type: velocity, a motor held at a speed, such as a flywheel or roller.
...
```

It wrote the node package `flywheel/`, its subsystem file, its
`[[subsystems]]` entry and the regenerated manifest, and, since this is the
repository's first mechanism, the simulation config `robot_sim.toml`
([section 4](#4-make-it-move-in-simulation)), its manifest, and the two
checks of it it added to `BUILD.bazel`, pointing `//:sim` at it. `--type position` makes
an arm or elevator instead ([section 9](#9-position-mechanisms-the-library-or-your-own-code)),
and `--lang cpp` a C++ node ([section 6](#6-the-c-track)). The package is
yours: the scaffold is a starting point, not a framework you must keep
([section 7](#7-make-it-yours)).

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
Messages are FlatBuffers *structs*, fixed-size records. Every number carries
its unit as a suffix (`_rps`, `_m`, `_us`, `_v`), and the build fails on one
without. Topic names follow [NAMING.md](../talOS/NAMING.md#topics).

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
  neutral. That is why **motor setters work only inside `update()`**; from
  anywhere else the framework refuses them with an error
  ([node guide: threads](NODE_DEVELOPMENT_GUIDE.md#threads)).
- A **target lane** is `/flywheel/target/<source>`, one per producer
  (`teleop`, `auto`, ...). `Target.get()` returns the first lane in the
  mode's priority list that is active, fresh and issued under the current
  Driver Station mode, and only while the robot is enabled with a hardware
  state younger than `command_timeout_us`; otherwise `None`, nothing is
  set, and the motor goes neutral. Safety needs no code from you; it is the
  shape of the node ([architecture: targets](../ARCHITECTURE.md#targets),
  rule 1 of [RULES.md](../RULES.md)).

`node_test.py` already tests the case list every node must pass: a disabled
robot, a stale or released lane, a lost Driver Station, a disconnected
motor, a NaN demand, another mode's lane, a target left over from an earlier
enable ([node test matrix](TESTING.md#7-the-node-test-matrix)). It runs your
class on `talos.testing.Sim`, a simulated clock with in-memory topics, and
`SimHardware`, a stand-in for the hardware bridge; neither needs a robot or a
process. When you change what the node does, change the test with it.

## 4. Make it move in simulation

`subsystems/flywheel.toml` describes the motor and the node's inputs, and
leaves it unable to move: its teleop binding is commented out and every
gain is zero ("untuned"), so it never drives a real mechanism on values
nobody measured. What makes it move in simulation goes in `robot_sim.toml`,
the **simulation config**, beside `robot.toml`. The scaffold wrote its first
seven lines; `Next:` prints the flywheel's lines to append to it. Append
them as printed, and the file reads:

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

`include = "robot.toml"` takes the whole robot, and these tables are merged
over the flywheel's subsystem file. The binding tells the `teleop` node to
write a target on `/flywheel/target/teleop` while joystick button 3 is held.
The scaffold also wrote `robot_sim_manifest.bzl`, this file's manifest,
pointed `//:sim` at `robot_sim.toml`, and added `sim_manifest_sync_test`
and `sim_graph_test` to `BUILD.bazel`, so `bazel test //...` lints this
config's graph too and fails, with the regenerate command, when its
manifest goes stale.

Build and test everything, then run `//:sim` with the scripted joystick,
as `Next:` prints it:

```sh
bazel test //...
bazel run //:sim -- --duration-s 13 \
    --sim-gateway-arg --mode-schedule --sim-gateway-arg 'disabled:1,teleop:10' \
    --sim-gateway-arg --joystick-schedule --sim-gateway-arg '0:axes=0;buttons=0;pov=-1|3:axes=0;buttons=4;pov=-1|6:axes=0;buttons=0;pov=-1'
```

```text
//:bundle_install_test                                                   PASSED in 3.0s
//:graph_test                                                            PASSED in 0.3s
//:manifest_sync_test                                                    PASSED in 0.1s
//:sim_graph_test                                                        PASSED in 0.2s
//:sim_manifest_sync_test                                                PASSED in 0.1s
//:sim_smoke_test                                                        PASSED in 1.6s
//:studio_test                                                           PASSED in 1.7s
//:talos_pin_test                                                        PASSED in 0.1s
//:trace_test                                                            PASSED in 0.1s
//flywheel:node_test                                                     PASSED in 1.4s

Executed 10 out of 10 tests: 10 tests pass.
...
[launcher] Started flywheel (pid 912946, target //flywheel:node)
...
Output Dir: /tmp/talos_logs/8357740337952040940
...
  • flywheel (pid=912946, exit=0, records=4035, restarts=0, failed=no, log=/tmp/talos_logs/8357740337952040940/flywheel.tlog)
```

The first `bazel test //...` in a new repository compiles what it needs of
talOS: 169 s on an eight-core Linux workstation after `bazel clean --expunge`, with every download already in Bazel's repository cache and other builds loading the host (224 s in another run on the same machine); without cached downloads, longer. Later runs take seconds. The launcher plays the Driver Station
and the RoboRIO with `sim_gateway`: disabled for 1 s, then teleop
(`--mode-schedule`), with joystick 0's button 3 (bit 2 of the mask,
`buttons=4`) held from second 3 to second 6
([syntax](../talOS/simulation/README.md#scripted-joystick-and-fault-schedules)).
Those seconds count from the gateway's start, but `--duration-s` counts
from the launcher's first spawn, so the 13 s are the schedules' 8 s plus
the 5 s the launcher gives a member to register before it kills it (its
startup grace, [restarts](../talOS/launcher/README.md#restarts)), so a
slow gateway start cuts nothing off the end of the schedules.
A **session** is one run of the launcher; its directory, the `Output Dir`
it prints last, holds each node's log, its text output and the graph.
`log_dump` reads the logs; its summary answers "did the flywheel move":

```sh
bazel run @talos//talOS/launcher:log_dump -- --format summary --node flywheel --topic /flywheel/state <session>
```

```text
topic /flywheel/state flywheel.FlywheelState
  ...
  latest {enabled: false, velocity_rps: 0, connected: true}
  range enabled false true
  range velocity_rps 0 19.999999999999986
  range connected true true
```

`<session>` is the `Output Dir` the launcher printed
(`/tmp/talos_logs/<id>`). `range velocity_rps 0 ...` with a maximum near 20
means it spun up while button 3 was held, `range enabled false true` that it
was enabled at some point
([summary format](../talOS/launcher/README.md#filtering-and-exporting-logs)).
Remove the `buttons=4` snapshot, or the mode schedule's `teleop`, and it
stays neutral: that is the safety path working.

## 5. If it does not work

- `./tools/setup.sh --check` (in talOS) lists each tool and whether it is
  acceptable.
- `bazel run //:sim -- --duration-s 5` runs the robot against the
  simulator (`robot.toml` before the first mechanism, `robot_sim.toml` once
  `new_subsystem` creates it); every member should exit 0.
- `bazel run @talos//talOS/launcher -- --config $PWD/robot_sim.toml
  --describe-only` lints the simulation config's graph without starting
  it. A warning is fine; an `error:` line names the rule the launcher would
  refuse to start on ([launcher: two phases](../talOS/launcher/README.md#two-phases)),
  and a node not built yet is named with the `bazel build` command that
  builds it.
- A stale manifest fails `manifest_sync_test` with
  `bazel run @talos//tools:sync_manifest -- robot.toml`, the command that
  regenerates it.
- The `can_id` in a subsystem file is a placeholder. On a real robot a
  person sets the physical id and `commissioned = true` at the robot,
  never from a desk ([bring-up](BRINGUP.md#human-commissioning)); a
  commissioned robot then requires a producer for every mechanism in each
  mode its policy lists.

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

The message file has the same shape as the Python one. The
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
call in both languages). `Next:` again prints lines to append to
`robot_sim.toml`; append them as printed (the scaffold regenerated
`robot_sim_manifest.bzl` already). Each mechanism needs a button of
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
session the launcher printed (`<session>`, its `Output Dir`); the full dump,
sampled, shows the trajectory:

```sh
bazel run //:sim -- --duration-s 13 \
    --sim-gateway-arg --mode-schedule --sim-gateway-arg 'disabled:1,teleop:10' \
    --sim-gateway-arg --joystick-schedule --sim-gateway-arg '0:axes=0;buttons=0;pov=-1|3:axes=0;buttons=8;pov=-1|6:axes=0;buttons=0;pov=-1'
bazel run @talos//talOS/launcher:log_dump -- --node claw --topic /claw/state <session> \
  | grep ClawState | awk 'NR%50==1'
```

```text
      claw.ClawState {enabled: false, velocity_rps: 0, connected: true}
      claw.ClawState {enabled: false, velocity_rps: 0, connected: true}
      claw.ClawState {enabled: false, velocity_rps: 0, connected: true}
      claw.ClawState {enabled: false, velocity_rps: 0, connected: true}
      claw.ClawState {enabled: true, velocity_rps: 19.999999976103876, connected: true}
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

[Studio](../studio/README.md) is the live viewer, with the driver dashboard
at `/drive`. Its bridge is built in your repository from `@talos`; its pages
are built by talOS's own workspace, in your talOS checkout, once and again
after each talOS update. `new_robot`'s `Next:` printed the command with your
checkout's path (here `../talos`). Then run `//:sim` with the bridge (it gathers the bridge and the pages into
`//:studio_webroot` and runs `robot_sim.toml`):

```sh
(cd ../talos && bazel build //studio:web_dist)
bazel run //:sim -- --duration-s 15 --start-studio-bridge \
    --sim-gateway-arg --mode-schedule --sim-gateway-arg 'disabled:1,teleop:10' \
    --sim-gateway-arg --joystick-schedule --sim-gateway-arg '0:axes=0;buttons=0;pov=-1|3:axes=0;buttons=4;pov=-1|6:axes=0;buttons=0;pov=-1'
```

```text
[launcher] Started studio_bridge (pid 369029) on http://localhost:5800
```

Open `http://localhost:5800` and `http://localhost:5800/drive` while it runs
(raise `--duration-s`, or leave it out and stop with Ctrl-C). Until the
pages are built, everything else runs and `bazel test //...` passes, but
`/` and `/drive` answer 503 with "Studio's pages are not built" and the
build command for the talOS checkout `studio_web` reads: the first command
above. A deployed robot has no
dashboard until its `robot_bundle` carries Studio (your repository's
`README.md`).

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
  values homing HOMING HOMED
```

### Writing the arm yourself

`--raw` writes the arm without the library: every line of its homing and
limits in `arm/arm.h` (`arm.py` with `--lang py`), on the node's own
`Motor`, with a test of each rule. Owning the node means owning those
rules: [writing the arm yourself](../talOS/mechanisms/README.md#writing-the-arm-yourself)
says what each constant bounds, what is still not caught, and how to watch
the bounded push fail safe in simulation.


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

talOS's own tests and its bench robot, `bench/`, keep robots inside the
talOS workspace. `bazel run //tools:new_robot -- <name> --mechanism
<mech> --lang cpp|py` (or `--no-mechanism`) writes one there, a directory
at the workspace root beside `bench/`, and `bazel run //tools:new_subsystem -- <name> --lang cpp|py
--robot <name>` adds a mechanism to it; `./sim.sh --config
<name>/main_processor/configuration/robot_sim.toml --sim-gateway` runs it.
`bazel run //tools:new_robot -- --help` lists the options. A team's robot
does not belong there: it is the repository this page makes.
