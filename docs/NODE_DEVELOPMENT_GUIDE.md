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
| `StateMachine<S> sm{node, S::A}`: `state()`, `is()`, `go()`, `previous()`, `time_in_state()`, `just_entered()` | `talos.StateMachine(node, A, states=S)`: same, `is_()`; `states=` (optional) makes reprs and errors show names | |
| `node.publish_bytes(topic, max_bytes, {.drop_newest, .external})` → `send(span<const byte>)` | `node.publish_bytes(topic, max_bytes, drop_newest=..., external=...)` → `send(bytes)` | variable-length feed outside the loop (not logged); used by telemetry |
| `TALOS_NODE(Class)` in one `.cc` | `talos_py_node(class_name = ...)` | |

Messages are FlatBuffers **structs** (no tables or strings), at most 8-byte
aligned. A relative topic name (`"state"`, `"target/teleop"`) expands to
`"/<name>/<topic>"`; `[subsystems.<name>.remap]` can redirect relative or
absolute topics without changing node code.

### When messages arrive

A subscription is updated when the node's loop drains its topics, and the
drain point depends on the node:

- A node with no message callback (no `subscribe(topic, fn)`, no
  `DriverStation(node, on_packet)` listener) drains right before each of its
  own timers, `update()` or `every()`. Every message published before that
  timer is visible inside it, and the process sleeps between timers instead of
  waking every millisecond.
- A node with any message callback, or with no timer at all, drains every
  1 ms, so a callback runs within 1 ms of the message.

`age()` and `fresh()` count from the drain that delivered the message, not
from its publication: under the first rule a message can be up to one node
period older than its `age()` says, so a target is judged stale at most one
period after `[robot] command_timeout_us`. `testing::Sim` applies the same
rules, so a test sees a message at the tick the robot would.

### Realtime scheduling

On the robot (not `--sim`, `--describe` or `--replay`) a node binary runs its
loop thread under `SCHED_FIFO` priority 50 with its memory locked (`mlockall`)
on Linux, or under the Mach time-constraint policy for its period on macOS
(which has no working `mlockall`); the bridge's loop runs at priority 60. When
the OS refuses (no privilege, a memlock limit) the node prints one line to
stderr and runs anyway. `TALOS_REALTIME=0` in the environment turns this off.

## Hardware

Handles are named by their device key in the subsystem config
(`[motors.roller]` → `"roller"`).

| C++ | Python |
|---|---|
| `talos::Motor m{node, "roller"}`: `set_duty_cycle`, `set_voltage`, `set_velocity`, `set_position`, `set_motion_magic` (`ff_volts`, `slot`), `stop`; `position()`, `velocity()`, `voltage()`, `current()`, `connected()` | `talos.Motor(node, "roller")`, getters as properties |
| `CANcoder`, `Gyro` (`yaw`, `yaw_rate`), `Encoder`, `DigitalInput` (`get()`), `AnalogInput` (`voltage()`), `DigitalOutput`, `PwmOutput` (`set()`) | same names; `DigitalInput.value` |
| `talos::DriverStation ds{node}`: `connected()`, `enabled()`, `autonomous()`, `teleop()`, `test()`, `estop()`, `alliance()`, `match_time()`, `joystick(port)` → `axis(i)`, `button(n)` (1-based), `pov(i)` | `talos.DriverStation(node)`, same as properties/methods |
| `talos::robot_enabled(node)` | `talos.robot_enabled(node)` |

Every motor not set during an `update()` is sent neutral, and all are neutral
while the robot is disabled. Units are rotations, rotations/s, volts, amps.

## BUILD

```python
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_messages", "talos_py_node")

talos_messages(name = "messages", srcs = ["shooter_message.fbs"],
               topics = {"state": "/shooter/state", "target": "/shooter/target"})
# :messages    C++: #include "<package>/shooter_message_generated.h"
#                   #include "<package>/topics.h"   shooter::topics::kTarget
# :messages_py Python: from <namespace>.ShooterTarget import ShooterTarget
#                      from <namespace> import topics   topics.TARGET

talos_cc_node(name = "node", node_name = "shooter", srcs = ["shooter.cc"], deps = [":shooter"])
talos_py_node(name = "node", node_name = "intake", main = "intake.py",
              class_name = "Intake", deps = [":messages_py"])
```

`include_prefix` on `talos_messages` overrides the package-basename prefix;
with `topics`, it must be the schema's namespace. Topic keys are
lower_snake_case: `"teleop_target"` is `kTeleopTarget` in C++ and
`TELEOP_TARGET` in Python.
To use another package's types, list its `talos_messages` target in `deps`
and include its schema by the dep's include prefix:
`deps = ["//shared/geometry:messages"]` with
`include "geometry/geometry.fbs";` in the `.fbs`, then C++ includes
`"geometry/geometry_generated.h"`. Deps are transitive. At a repository root
the package path is empty, so pass `include_prefix` explicitly there.
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
sim.publish(topics::kTarget, ShooterTarget{80.0, 0, true});
sim.step(20ms);
hw.motor_request("shooter", "flywheel").mode;  // Mode::kVelocity
sim.latest<ShooterState>(topics::kState);     // std::optional
```

`SimHardware` also has `set_motor`, `set_digital_input`, `set_analog_input`,
`set_encoder`, `set_cancoder`, `set_gyro`, `set_driver_station`,
`set_connected(node, device, false)` (the device reports disconnected),
`digital_output`, `pwm`.

Python: `from talos.testing import Sim, SimHardware` (`//talOS/python:testing`),
the same methods (`Sim.from_toml`, `sim.step(seconds)`,
`sim.publish(M, topic, **fields)`, `sim.latest(M, topic)`,
`sim.latest_bytes(topic)`). See `2026-robot/main_processor/intake/node_test.py`.
