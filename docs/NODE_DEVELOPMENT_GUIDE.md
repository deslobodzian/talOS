# Node API reference

The walkthrough is [examples/superstructure/README.md](../examples/superstructure/README.md).
This page lists the API. Headers: `talOS/node/node.h`, `talOS/devices/devices.h`;
Python: `talOS/python/talos`.

## Node

A node is a class constructed from the node, taking every handle and topic in
the constructor; `update()` runs every period.

| C++ (`talos::Node& node`) | Python (`node`) | |
|---|---|---|
| `node.subscribe<M>(topic, {.optional = true})` | `node.subscribe(M, topic, optional=True)` | latest value; optional callback argument |
| `node.publish<M>(topic)` | `node.publish(M, topic)` | |
| `node.param<T>("key")`, `param<T>("key", fallback)` | `node.param("key")`, `param("key", fallback)` | this subsystem's TOML keys; dotted keys reach nested tables |
| `node.every(period, fn)` | `node.every(seconds, fn)` | extra timer |
| `name()`, `now()`, `tick()`, `period()`, `simulation()`, `config_path()` | same, as properties (seconds) | |
| `sub.value()`, `sub->field()`, `has_value()` | `sub.value` (`None` before the first), `has_value` | |
| `sub.fresh()`, `fresh(max_age)`, `age()`, `updated()` | `sub.fresh()`, `fresh(max_age)`, `age`, `updated` | fresh: younger than `[robot] command_timeout_us` |
| `pub.send(M{...})` | `pub.send(field=value, ...)` or `send(message)` | |
| `StateMachine<S> sm{node, S::A}`: `state()`, `is()`, `go()`, `previous()`, `time_in_state()`, `just_entered()` | `talos.StateMachine(node, A)`: same, `is_()` | |
| `TALOS_NODE(Class)` in one `.cc` | `talos_py_node(class_name = ...)` | |

Messages are FlatBuffers **structs** (no tables or strings), at most 8-byte
aligned.

## Hardware

Handles are named by their device key in the subsystem config
(`[motors.roller]` → `"roller"`).

| C++ | Python |
|---|---|
| `talos::Motor m{node, "roller"}`: `set_duty_cycle`, `set_voltage`, `set_velocity`, `set_position`, `set_motion_magic` (`ff_volts`, `slot`), `stop`; `position()`, `velocity()`, `voltage()`, `current()`, `connected()` | `talos.Motor(node, "roller")`, getters as properties |
| `CANcoder`, `Gyro` (`yaw`, `yaw_rate`), `Encoder`, `DigitalInput` (`get()`), `AnalogInput` (`voltage()`), `DigitalOutput`, `PwmOutput` (`set()`) | same names; `DigitalInput.value` |
| `talos::robot_enabled(node)` | `talos.robot_enabled(node)` |

Every motor not set during an `update()` is sent neutral, and all are neutral
while the robot is disabled. Units are rotations, rotations/s, volts, amps.

## BUILD

```python
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_messages", "talos_py_node")

talos_messages(name = "messages", srcs = ["shooter_message.fbs"], hdrs = ["topics.h"])
# :messages    C++: #include "<package>/shooter_message_generated.h", "<package>/topics.h"
# :messages_py Python: from <namespace>.ShooterTarget import ShooterTarget

talos_cc_node(name = "node", node_name = "shooter", srcs = ["shooter.cc"], deps = [":shooter"])
talos_py_node(name = "node", node_name = "intake", main = "intake.py",
              class_name = "Intake", deps = [":messages_py"])
```

`include_prefix` on `talos_messages` overrides the package-basename prefix.
The node's name comes from `node_name`; add the node to the robot config as
`node = "//path:node"`.

## Testing

`talos::testing::Sim` (`//talOS/node:testing`) runs nodes on one simulated clock
with in-memory topics; `talos::testing::SimHardware` (`//talOS/devices:testing`)
stands in for the bridge.

```cpp
Sim sim{config_path};                 // or Sim::FromToml(inline_toml)
auto& shooter = sim.add<Shooter>("shooter");
SimHardware hw{sim};
hw.enable();
hw.ideal_motors();                    // motors reach their demand at once
sim.publish(kTargetTopic, ShooterTarget{80.0, 0, true});
sim.step(20ms);
hw.motor_request("shooter", "flywheel").mode;  // Mode::kVelocity
sim.latest<ShooterState>(kStateTopic);         // std::optional
```

`SimHardware` also has `set_motor`, `set_digital_input`, `set_analog_input`,
`set_encoder`, `set_cancoder`, `set_gyro`, `digital_output`, `pwm`.

Python: `from talos.testing import Sim, SimHardware` (`//talOS/python:testing`),
the same methods (`Sim.from_toml`, `sim.step(seconds)`,
`sim.publish(M, topic, **fields)`, `sim.latest(M, topic)`). See
`2026-robot/main_processor/intake/node_test.py`.
