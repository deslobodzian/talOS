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

The nodes exist twice, in C++ (`cpp/`) and in Python (`py/`), with the same
states, topics, parameters and tests. Either set runs the same robot.

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
  topics.h                                       topic names
  BUILD                                          talos_messages(include_prefix = "demo")
config/
  robot.toml       the example robot (single-file layout), launching cpp/
  robot_py.toml    the same robot, launching py/ (only the `node =` lines differ)
cpp/               the C++ nodes and their test
py/                the Python nodes and their test
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
`talos.testing.Sim`. `config/config_test.py` checks that the two config files
differ only in their `node =` lines.

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
two expected warnings: `/hw/state/driver_station` and `/superstructure/state`
are published but nothing reads them, because the example has no driver
node. On a real robot, a driver node reads the driver station, publishes
`/superstructure/target` and watches `/superstructure/state`.

## Write your own mechanism

Take a wrist that goes to an angle as an example.

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
   WristGoal`. Add the topic names (`/wrist/target`, `/wrist/state`) to
   `messages/topics.h`.
2. **The class** (`cpp/wrist.h`):
   ```cpp
   class Wrist {
    public:
     using State = WristState;
     explicit Wrist(talos::Node& node)
         : motor_{node, "motor"},
           goal_{node.subscribe<WristGoal>(kWristGoalTopic)},
           status_{node.publish<WristStatus>(kWristStatusTopic)},
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
4. **Config** (`config/robot.toml`, and the same block in `robot_py.toml`
   with `node = "//examples/superstructure/py:wrist"`):
   ```toml
   [subsystems.wrist]
   node = "//examples/superstructure/cpp:wrist"
   period_us = 10000
   tolerance_rot = 0.02

   [subsystems.wrist.motors.motor]
   type = "TalonFX"
   can_id = 3
   cruise_velocity_rps = 2.0
   acceleration_rps2 = 8.0
   slot0 = { p = 30.0, i = 0.0, d = 0.5, s = 0.1, v = 0.0, a = 0.0, g = 0.2 }
   ```
   The device key (`motors.motor`) is the name you give the handle.
5. **Test** it in `cpp/superstructure_test.cc` or a test of its own (Python:
   the same calls on `talos.testing.Sim`, with durations in seconds):
   ```cpp
   Sim sim{RobotConfig()};
   sim.add<demo::Wrist>("wrist");
   SimHardware hw{sim};
   hw.enable();
   hw.ideal_motors();
   sim.publish(kWristGoalTopic, WristGoal{0.25});
   sim.step(50ms);
   EXPECT_EQ(hw.motor_request("wrist", "motor").mode, Mode::kMotionMagic);
   EXPECT_TRUE(sim.latest<WristStatus>(kWristStatusTopic)->at_goal());
   ```

## Python twin

`py/intake.py`, `py/elevator.py` and `py/superstructure.py` are the same
nodes on `import talos`. They use the same messages (`:messages_py`) and the
same robot (`config/robot_py.toml`), and `py/superstructure_test.py` holds
the same tests. Here is the elevator side by side; the code differs only in
syntax.

<table>
<tr><th>C++ (<code>cpp/elevator.h</code>)</th><th>Python (<code>py/elevator.py</code>)</th></tr>
<tr><td>

```cpp
class Elevator {
 public:
  using State = ElevatorState;

  explicit Elevator(talos::Node& node)
      : lift_{node, "lift"},
        goal_{node.subscribe<ElevatorGoal>(
            kElevatorGoalTopic)},
        status_{node.publish<ElevatorStatus>(
            kElevatorStatusTopic)},
        sm_{node, State::STOWED},
        tolerance_rot_{
            node.param<double>("tolerance_rot")} {}

  void update() {
    if (goal_.fresh()) goal_rot_ = goal_->height_rot();
    lift_.set_motion_magic(goal_rot_);

    const double height = lift_.position();
    const bool at_goal =
        lift_.connected() &&
        std::abs(height - goal_rot_) <= tolerance_rot_;
    if (!at_goal) sm_.go(State::MOVING);
    else if (goal_rot_ == 0.0) sm_.go(State::STOWED);
    else sm_.go(State::AT_GOAL);

    status_.send(ElevatorStatus{
        sm_.state(), height, goal_rot_, at_goal});
  }

 private:
  talos::Motor lift_;
  talos::Subscription<ElevatorGoal> goal_;
  talos::Publisher<ElevatorStatus> status_;
  talos::StateMachine<State> sm_;
  double tolerance_rot_;
  double goal_rot_ = 0.0;
};

TALOS_NODE(demo::Elevator)  // elevator.cc
```

</td><td>

```python
from demo.ElevatorState import ElevatorState as State

class Elevator:
    def __init__(self, node):
        self.lift = talos.Motor(node, "lift")
        self.goal = node.subscribe(
            ElevatorGoal, "/elevator/target")
        self.status = node.publish(
            ElevatorStatus, "/elevator/state")
        self.sm = talos.StateMachine(node, State.STOWED)
        self.tolerance_rot = node.param("tolerance_rot")
        self.goal_rot = 0.0

    def update(self):
        if self.goal.fresh():
            self.goal_rot = self.goal.value.height_rot
        self.lift.set_motion_magic(self.goal_rot)

        height = self.lift.position
        at_goal = (self.lift.connected and
                   abs(height - self.goal_rot)
                   <= self.tolerance_rot)
        if not at_goal:
            self.sm.go(State.MOVING)
        elif self.goal_rot == 0.0:
            self.sm.go(State.STOWED)
        else:
            self.sm.go(State.AT_GOAL)

        self.status.send(
            state=self.sm.state, height_rot=height,
            goal_rot=self.goal_rot, at_goal=at_goal)


# talos_py_node(class_name = "Elevator") makes main
```

</td></tr>
</table>

Python reads message fields as snake_case attributes named as in the .fbs
(`goal.value.height_rot`), and `send(**fields)` takes the same names; any
field you leave out is 0. A schema enum is a flatc class of int constants
(`from demo.IntakeCommand import IntakeCommand`, then `IntakeCommand.EJECT`).
Python has no topics module, so the topic names are written out; they match
`messages/topics.h`.
