# Superstructure example

An intake, an elevator, and a superstructure that sequences them, written as
three **separate nodes** that talk only over topics:

```
 /superstructure/target ──▶ superstructure ──▶ /intake/target   ──▶ intake   (roller + beam break)
                                 ▲  ▲      ──▶ /elevator/target ──▶ elevator (one motion-magic TalonFX)
                                 │  └──────────── /intake/state   ◀──┘    │
                                 └─────────────── /elevator/state ◀───────┘
```

- **intake**: IDLE → INTAKING (roller in) → HOLDING once the beam breaks (small
  hold voltage) → EJECTING (reverse for `eject_ms`) → IDLE.
- **elevator**: motion magic to the goal height and holds it there. It reports
  STOWED at 0, MOVING, or AT_GOAL (within `tolerance_rot`), and echoes the goal
  that `at_goal` refers to. It keeps its last goal if its target goes stale.
- **superstructure**: STOWED → INTAKING (on INTAKE) → HOLDING (the intake
  has the piece) → RAISING (on SCORE_HIGH, elevator to `high_rot`) → SCORING
  (the elevator is at that height; intake ejects) → STOWED (the piece is gone).
  When either mechanism's state goes stale it holds the current step and keeps
  sending that step's goals; it never advances on old data. A stale
  `/superstructure/target` counts as STOW.

Every tunable (voltages, eject time, tolerance, height) is a parameter read
from the robot config: `node.param<T>("key")` in C++, `node.param("key")` in
Python.

The nodes are implemented once in C++ (`cpp/`) and wrapped in Python (`py/`)
via `TALOS_EXPORT_NODE` and `talos.cpp_node`, so both sets of binaries and
tests run the exact same implementation without duplicating logic.

## Three layers

`talos::Node` (`//talOS/node`) is generic: it gives you topics, parameters,
`update()` every period, `StateMachine`, and `TALOS_NODE`, and knows nothing
about robots. `talOS/devices` (`//talOS/devices`) is the hardware layer on top
of it. `talos::Motor roller{node, "roller"}` names a device in your
subsystem's TOML, and the layer handles `/hw/state`, the per-node hardware
request, and neutral-when-disabled. Your code is the class in between: read
inputs, step a state machine, set outputs, publish a status. The intake and
elevator use all three layers; the superstructure has no hardware, so it uses
only the first. Python has the same three layers (`talos.Node`, then
`talos.Motor(node, "roller")`, then your class), as a thin binding over the
same C++ code, so both languages behave identically.

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
with `SimHardware` standing in for the bridge and `config/robot.toml` as the
config. It walks the whole sequence by publishing goals and flipping the beam
break, and it checks the roller voltages, the elevator's motion-magic demand,
`at_goal`, the eject, and the return to STOWED. It also checks that disabling
the robot neutralizes every motor while every state machine keeps its state,
and that stale mechanism states never advance the superstructure.
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
RoboRIO, stays enabled with no Driver Station, and moves motors straight to
their demands:

```sh
./bazel-bin/talOS/launcher/launcher --config examples/superstructure/config/robot.toml \
  --sim --start-sim-gateway --duration-s 5
```

Use `--config examples/superstructure/config/robot_py.toml` in either command
to run the Python nodes instead. Both configs declare the same graph.

Nothing in the example publishes `/superstructure/target`; a driver or
autonomous node would. So the live graph comes up, sits STOWED, and holds the
elevator at 0. The subscription is declared `{.optional = true}`, so the
graph check accepts it having no publisher. The check reports no errors and
three expected warnings: `/hw/state/driver_station`, `/hw/status` and
`/superstructure/state` are published but nothing reads them, because the
example has no driver or telemetry node. On a real robot, a driver node reads
the driver station, publishes
`/superstructure/target` and watches `/superstructure/state`.

## Write your own mechanism

The fast path is the scaffolder, from the repo root:

```sh
bazel run //tools:new_subsystem -- wrist --lang cpp   # or --lang py
```

It writes `<robot>/main_processor/wrist/` (messages with an explicit
`include_prefix`, the node, a Sim test covering the disabled and stale-target
cases), the subsystem TOML with the lowest free CAN id marked as a
placeholder, the manifest entry and the bundle lists. What follows explains
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
   WristGoal`. Add the topic names to `topics` in `messages/BUILD`:
   `"wrist_goal": "/wrist/target", "wrist_status": "/wrist/state"`. C++ reads
   them as `topics::kWristGoal` (`#include "demo/topics.h"`), Python as
   `topics.WRIST_GOAL` (`from demo import topics`).
2. **The class** (`cpp/wrist.h`):
   ```cpp
   class Wrist {
    public:
     using State = WristState;
     explicit Wrist(talos::Node& node)
         : motor_{node, "motor"},
           goal_{node.subscribe<WristGoal>(topics::kWristGoal)},
           status_{node.publish<WristStatus>(topics::kWristStatus)},
           sm_{node, State::STOWED},
           tolerance_rot_{node.param<double>("tolerance_rot")} {}
     void update() {
       if (goal_.fresh()) goal_rot_ = goal_->angle_rot();
       motor_.set_motion_magic(goal_rot_);
       const bool at_goal = std::abs(motor_.position() - goal_rot_) <= tolerance_rot_;
       sm_.go(at_goal ? State::AT_GOAL : State::MOVING);
       status_.send(WristStatus{sm_.state(), motor_.position(), at_goal});
     }
    private:
     talos::Motor motor_;
     talos::Subscription<WristGoal> goal_;
     talos::Publisher<WristStatus> status_;
     talos::StateMachine<State> sm_;
     double tolerance_rot_;
     double goal_rot_ = 0.0;
   };
   ```
   Then `cpp/wrist.cc` is two lines: the include and `TALOS_NODE(demo::Wrist)`.
   Take every handle and topic in the constructor. Anything you don't set in
   an `update()` goes out neutral, and every motor is neutral while the robot
   is disabled. In Python it is `py/wrist.py`, a `class Wrist` with
   `__init__(self, node)` and `update(self)`; see the side-by-side below.
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
   (`motors.motor`) is the name you give the handle.
5. **Test** it in `cpp/superstructure_test.cc` or a test of its own (Python:
   the same calls on `talos.testing.Sim`, with durations in seconds):
   ```cpp
   Sim sim{RobotConfig()};
   sim.add<demo::Wrist>("wrist");
   SimHardware hw{sim};
   hw.enable();
   hw.ideal_motors();
   sim.publish(topics::kWristGoal, WristGoal{0.25});
   sim.step(50ms);
   EXPECT_EQ(hw.motor_request("wrist", "motor").mode, Mode::kMotionMagic);
   EXPECT_TRUE(sim.latest<WristStatus>(topics::kWristStatus)->at_goal());
   ```

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
`talos.Motor`, and `talos.StateMachine` concepts (all backed by the single C++
implementation through `talOS/node_api`).

Python reads message fields as snake_case attributes named as in the .fbs
(`goal.value.height_rot`), and `send(**fields)` takes the same names; any
field you leave out is 0. A schema enum is a flatc class of int constants
(`from demo.IntakeCommand import IntakeCommand`, then `IntakeCommand.EJECT`).
Topic names come from the same `topics` dict as C++'s: `topics.ELEVATOR_GOAL`
is `topics::kElevatorGoal`.
