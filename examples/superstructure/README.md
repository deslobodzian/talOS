# Superstructure example

An intake, an elevator, and a superstructure that sequences them, written as
three **separate nodes** that talk only over topics. Goals travel on target
lanes, `/<subsystem>/target/<source>`: each producer writes its own lane
(`talos::TargetLane`) and each subsystem picks the live one for the current
Driver Station mode (`talos::Target`):

```
 /superstructure/target/teleop ─┐
 /superstructure/target/auto   ─┴▶ superstructure ──▶ /intake/target/superstructure   ──▶ intake   (roller + beam break)
                                       ▲  ▲        ──▶ /elevator/target/superstructure ──▶ elevator (one motion-magic TalonFX)
                                       │  └──────────── /intake/state   ◀─────────────────┘     │
                                       └─────────────── /elevator/state ◀──────────────────────┘
```

Which lanes count in which mode is config. The superstructure uses the default
`[robot.targets]` policy; the intake and elevator each replace it with a
`[targets]` table in their subsystem file, so only the superstructure drives
them, in both modes:

| Subsystem | teleop | autonomous | Set in |
|---|---|---|---|
| superstructure | `/superstructure/target/teleop` | `/superstructure/target/auto` | default `[robot.targets]` |
| intake | `/intake/target/superstructure` | `/intake/target/superstructure` | `[targets]` in `subsystems/intake.toml` |
| elevator | `/elevator/target/superstructure` | `/elevator/target/superstructure` | `[targets]` in `subsystems/elevator.toml` |

A Target has a goal only while the robot is enabled, the Driver Station is in
an enabled mode, and that mode's lane is active (not released), fresh
(younger than `[robot] command_timeout_us`) and sent under the current mode
generation (its producer had drained the Driver Station packet of this enable
and mode). With none, a subsystem is neutral: it calls no actuator.

- **intake**: IDLE → INTAKING (roller in) → HOLDING once the beam breaks (small
  hold voltage) → EJECTING (reverse for `eject_us`, 500000 µs by default) →
  IDLE. Roller voltages are `intake_v`, `hold_v`, and `eject_v`. With no
  target it behaves as asked for IDLE with the roller neutral; the state
  machine still tracks the beam break and finishes a timed eject.
- **elevator**: motion magic to the goal height while it has a target. It
  reports STOWED at 0, MOVING, or AT_GOAL (within `tolerance_rot`), and echoes
  the goal that `at_goal` refers to. With no target the lift is neutral (it
  never holds an old goal); its status echoes goal 0 with `at_goal` false, and
  reads STOWED within `tolerance_rot` of 0, MOVING elsewhere.
- **superstructure**: STOWED → INTAKING (on INTAKE) → HOLDING (the intake
  has the piece) → RAISING (on SCORE_HIGH, elevator to `high_rot`) → SCORING
  (the elevator is at that height; intake ejects) → STOWED (the piece is gone).
  Cancellation is checked every tick: STOW or no goal (disabled, stale, released
  or never sent) abandons the active sequence immediately. With fresh mechanism
  statuses it requests intake IDLE and elevator height 0. If either status is
  stale, it releases **both** mechanism lanes instead of renewing a high
  elevator goal or EJECT. The mechanisms then have no target and are neutral;
  this is not a gravity-compensating elevator hold. The reported sequence falls
  back to HOLDING only when a fresh intake status still sees a piece, otherwise
  STOWED. Once both statuses return, sequencing starts from those observations
  and the current command, not the interrupted step: a piece that left during
  the outage is not scored again. Normal sequencing republishes both lanes
  every tick.

Every tunable (voltages, eject time, tolerance, height) is a parameter read
from the robot config: `node.param<T>("key")` in C++, `node.param("key")` in
Python.

The nodes are implemented once in C++ (`cpp/`) and wrapped in Python (`py/`)
via `TALOS_EXPORT_NODE` and `talos.cpp_node`, so both sets of binaries and
tests run the exact same implementation without duplicating logic.

For a subsystem shipped as its own Bzlmod repository, see
[`../wrist_repo/`](../wrist_repo/) and its robot-side consumer
[`../external_subsystem/`](../external_subsystem/). The root `MODULE.bazel`
imports `@wrist//:node` with `bazel_dep` and `local_path_override`; the consumer
tests its graph, generated manifest, and a relocated bundle:
`bazel test //examples/external_subsystem:all` and `bazel test @wrist//...`.

## Three layers

`talos::Node` (`//talOS/node`) is generic: it gives you topics, parameters,
`update()` every period, `StateMachine`, and `TALOS_NODE`, and knows nothing
about robots. `talOS/devices` (`//talOS/devices`) is the hardware layer on top
of it. `talos::Motor roller{node, "roller"}` names a device in your
subsystem's TOML, and the layer handles `/hw/state`, the per-node hardware
request, and neutral-when-disabled. Your code is the class in between: read
inputs, step a state machine, set outputs, publish a status. The intake and
elevator use all three layers. The superstructure has no hardware, but its goal
is a `talos::Target` and its outputs are `talos::TargetLane`s, which also come
from `talOS/devices` (a Target reads `/hw/state` and the Driver Station to know
whether the robot is enabled and in which mode). Python has the same three
layers (`talos.Node`, then `talos.Motor(node, "roller")` and
`talos.Target(node, Msg)`, then your class), as a thin binding over the same
C++ code, so both languages behave identically.

## Layout

```
messages/          shared by the C++ and Python nodes
  intake.fbs  elevator.fbs  superstructure.fbs   namespace demo; structs + enums
  BUILD                                          talos_messages(topics = {...}, include_prefix = "demo")
config/
  subsystems/      intake.toml  elevator.toml  superstructure.toml: devices + params
  robot.toml       the example robot's manifest, launching cpp/
  robot_py.toml    the same manifest, launching py/ (only the `node =` lines differ)
cpp/               the C++ node implementations, shared library, and test
py/                Python wrappers (via talos.cpp_node) and Python Sim test
```

## Test it

```sh
bazel test //examples/superstructure/...
```

`cpp/superstructure_test.cc` runs all three nodes in one `talos::testing::Sim`,
with `SimHardware` standing in for the bridge and the Driver Station (enabled,
in teleop) and `config/robot.toml` as the config. It walks the whole sequence
by publishing the driver's goal on `/superstructure/target/teleop`
(`PublishTarget`) and flipping the beam break, and it checks the roller
voltages, the elevator's motion-magic demand, `at_goal`, the eject, and the
return to STOWED. It also checks that disabling the robot neutralizes every
motor (the superstructure, losing its own goal, steps back to HOLDING), that a
missing, released or stale goal counts as STOW, and that stale mechanism
statuses release both lanes. Coordinator-only tests cover STOW during stale
RAISING, EJECT release during SCORING when either dependency goes stale,
release/disable/goal expiry with stale statuses, and recovery from fresh
observations without replaying a completed score. The elevator alone goes
neutral on a stale, released or never-sent lane, while disabled, on Driver
Station loss, with the lift disconnected and on a non-finite goal; the intake
alone goes neutral on a stale lane while keeping its state.
`py/superstructure_test.py` runs the same scenarios on the Python nodes with
`talos.testing.Sim` and `config/robot_py.toml`.

## Run it in simulation

Check the graph first. This asks every node what it publishes and subscribes
to, lints the whole graph, and exits:

```sh
bazel build //talOS/launcher //talOS/bridge:hardware_node \
  //2026-robot/main_processor/simulation:sim_gateway \
  //examples/superstructure/cpp:all //examples/superstructure/py:all
./bazel-bin/talOS/launcher/launcher --config examples/superstructure/config/robot.toml --describe-only
```

Then run it live against the lightweight sim gateway, which stands in for the
RoboRIO and moves motors straight to their demands:

```sh
./bazel-bin/talOS/launcher/launcher --config examples/superstructure/config/robot.toml \
  --sim --start-sim-gateway --duration-s 5
```

Use `--config examples/superstructure/config/robot_py.toml` in either command
to run the Python nodes instead. Both configs declare the same graph.

Nothing in the example publishes `/superstructure/target/teleop` or
`/superstructure/target/auto`; a driver or autonomous node would. Nothing
moves without a Driver Station in an enabled mode (the real Driver Station, or
`talOS/driver_station`'s `send_joystick`): every Target is empty and every
motor neutral. With an enabled teleop Driver Station, the live graph sits
STOWED and holds the elevator at 0. The check reports no errors and four
expected warnings: `/superstructure/target/teleop` and
`/superstructure/target/auto` are in the superstructure's target policy but
nothing writes them yet, and `/hw/status` and `/superstructure/state` are
published but nothing reads them, because the example has no driver,
autonomous or telemetry node. On a real robot, a driver node reads the Driver
Station, writes `/superstructure/target/teleop` with a
`talos::TargetLane<SuperstructureGoal>{node, "superstructure", "teleop"}` and
watches `/superstructure/state`.

## Write your own mechanism

The fast path is the scaffolder, from the repo root:

```sh
bazel run //tools:new_subsystem -- wrist --lang cpp   # or --lang py
```

It writes `<robot>/main_processor/wrist/` (messages with an explicit
`include_prefix`, the node, a Sim test covering the disabled and stale-target
cases), the subsystem TOML with the lowest free CAN id marked as a
placeholder, the `[[subsystems]]` entry in `robot.toml`, and the regenerated
`robot_manifest.bzl` (no BUILD file lists nodes). What follows explains
each generated piece, taking a wrist that goes to an angle as an example.

1. **Messages.** Add `messages/wrist.fbs` and list it in `messages/BUILD`:
   ```
   namespace demo;
   enum WristState : ubyte { STOWED, MOVING, AT_GOAL }
   struct WristGoal   { angle_rot:double; }
   struct WristStatus { state:WristState; angle_rot:double; at_goal:bool; }
   ```
   Use structs only, with no tables or strings. C++ gets
   `#include "demo/wrist_generated.h"` and `demo::WristGoal`, with enums
   spelled `WristState::MOVING`. Python gets `from demo.WristGoal import
   WristGoal`. Add the state topic to `topics` in `messages/BUILD`:
   `"wrist_status": "/wrist/state"`. C++ reads it as `topics::kWristStatus`
   (`#include "demo/topics.h"`), Python as `topics.WRIST_STATUS`
   (`from demo import topics`). The goal needs no topic entry: it arrives on
   target lanes, `/wrist/target/<source>`, which `talos::Target` names from
   the subsystem and its policy.
2. **The class** (`cpp/wrist.h`):
   ```cpp
   class Wrist {
    public:
     using State = WristState;
     explicit Wrist(talos::Node& node)
         : motor_{node, "motor"},
           goal_{node},
           status_{node.publish<WristStatus>(topics::kWristStatus)},
           sm_{node, State::STOWED},
           tolerance_rot_{node.param<double>("tolerance_rot")} {}
     void update() {
       const WristGoal* goal = goal_.get();  // null: disabled, stale, released
       const bool drive = goal != nullptr && motor_.connected() &&
                          std::isfinite(goal->angle_rot());
       if (drive) motor_.set_motion_magic(goal->angle_rot());
       const bool at_goal =
           drive && std::abs(motor_.position() - goal->angle_rot()) <= tolerance_rot_;
       sm_.go(at_goal ? State::AT_GOAL : State::MOVING);
       status_.send(WristStatus{sm_.state(), motor_.position(), at_goal});
     }
    private:
     talos::Motor motor_;
     talos::Target<WristGoal> goal_;
     talos::Publisher<WristStatus> status_;
     talos::StateMachine<State> sm_;
     double tolerance_rot_;
   };
   ```
   Then `cpp/wrist.cc` is two lines: the include and `TALOS_NODE(demo::Wrist)`.
   Take every handle and topic in the constructor. Call an actuator only
   inside `drive`: anything you don't set in an `update()` goes out neutral,
   so with no target the wrist is neutral rather than holding an old goal, and
   every motor is neutral while the robot is disabled. In Python it is
   `py/wrist.py`, a `class Wrist` with `__init__(self, node)` and
   `update(self)`, reading `self.goal = talos.Target(node, WristGoal)` with
   `self.goal.get()` (None when there is no target).
3. **BUILD** (`cpp/BUILD`): add `wrist.h` to `:mechanisms`, then
   ```
   talos_cc_node(name = "wrist", node_name = "wrist", srcs = ["wrist.cc"], deps = [":mechanisms"])
   ```
   The node's name and Bazel target come from this macro alone. For Python,
   use `py/BUILD`:
   ```
   talos_py_node(name = "wrist", node_name = "wrist", main = "wrist.py", class_name = "Wrist",
                 deps = ["//examples/superstructure/messages:messages_py"])
   ```
4. **Config.** Add `config/subsystems/wrist.toml`:
   ```toml
   [subsystem]
   name = "wrist"
   period_us = 10000
   tolerance_rot = 0.02

   [motors.motor]
   type = "TalonFX"
   can_id = 3
   cruise_velocity_rps = 2.0
   acceleration_rps2 = 8.0
   slot0 = { p = 30.0, i = 0.0, d = 0.5, s = 0.1, v = 0.0, a = 0.0, g = 0.2 }
   ```
   and list it in `config/robot.toml`:
   ```toml
   [[subsystems]]
   name = "wrist"
   path = "subsystems/wrist.toml"
   node = "//examples/superstructure/cpp:wrist"
   ```
   `robot_py.toml` gets the same entry with
   `node = "//examples/superstructure/py:wrist"`. The device key
   (`motors.motor`) is the name you give the handle. The wrist reads the
   default `[robot.targets]` lanes (`/wrist/target/teleop`,
   `/wrist/target/auto`); to have the superstructure drive it instead, give
   `wrist.toml` a `[targets]` table as `elevator.toml` does, and write
   `talos::TargetLane<WristGoal>{node, "wrist", "superstructure"}` in the
   superstructure. Then regenerate the manifests the graph tests read;
   `manifest_sync_test` fails until you do:
   ```sh
   bazel run //tools:sync_manifest -- examples/superstructure/config/robot.toml examples/superstructure/config/robot_py.toml
   ```
5. **Test** it in `cpp/superstructure_test.cc` or a test of its own (Python:
   the same calls on `talos.testing.Sim`, with durations in seconds):
   ```cpp
   Sim sim{RobotConfig()};
   sim.add<demo::Wrist>("wrist");
   SimHardware hw{sim};
   hw.enable();
   hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
   hw.ideal_motors();
   sim.step(10ms);  // the teleop mode begins
   PublishTarget(sim, "wrist", "teleop", WristGoal{0.25});
   sim.step(50ms);
   EXPECT_EQ(hw.motor_request("wrist", "motor").mode, Mode::kMotionMagic);
   EXPECT_TRUE(sim.latest<WristStatus>(topics::kWristStatus)->at_goal());
   ```
   and cover the disabled, stale, released and never-sent cases, which must
   all leave the motor neutral (`docs/METHODOLOGY.md` §11).

## Wrapping C++ nodes in Python (`py/`)

Instead of maintaining a duplicate Python implementation of the C++ mechanisms,
`cpp/mechanisms_abi.cc` exports the C++ classes with `TALOS_EXPORT_NODE`:

```cpp
TALOS_EXPORT_NODE(intake, demo::Intake)
TALOS_EXPORT_NODE(elevator, demo::Elevator)
TALOS_EXPORT_NODE(superstructure, demo::Superstructure)
```

and `py/intake.py`, `py/elevator.py` and `py/superstructure.py` wrap them with
`talos.cpp_node`:

```python
import talos

Elevator = talos.cpp_node(
    "examples/superstructure/cpp/libmechanisms.so", "elevator")
```

When a subsystem is written directly in Python, it uses the same `talos.Node`,
`talos.Motor`, `talos.Target`, `talos.TargetLane` and `talos.StateMachine`
concepts (all backed by the single C++ implementation through
`talOS/node_api`).

Python reads message fields as snake_case attributes named as in the .fbs
(`goal.height_rot`), and `send(**fields)` takes the same names; any
field you leave out is 0. A schema enum is a flatc class of int constants
(`from demo.IntakeCommand import IntakeCommand`, then `IntakeCommand.EJECT`).
Topic names come from the same `topics` dict as C++'s: `topics.ELEVATOR_STATUS`
is `topics::kElevatorStatus`.
