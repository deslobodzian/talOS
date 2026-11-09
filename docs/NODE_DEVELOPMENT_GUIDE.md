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
| `node.param<T>("key")`, `param<T>("key", fallback)` | `node.param("key")`, `param("key", fallback)` | this subsystem's TOML keys; dotted keys reach nested tables. `T`: `bool`, integers, `double`, `std::string`, and TOML arrays as `std::vector<std::string>` / `std::vector<double>` (integer elements convert; `[]` is valid). Python returns arrays as `list[str]` / `list[float]`; a list default asks for an array. A wrong element type is an error naming the key and element index |
| `node.every(period, fn)` | `node.every(seconds, fn)` | extra timer |
| `name()`, `now()`, `tick()`, `period()`, `simulation()`, `config_path()` | same, as properties (seconds) | |
| `session_dir()` → `std::optional<std::filesystem::path>` | `node.session_dir` → `str \| None` | the session output directory the launcher started this node under (the parent of `--log`), `nullopt`/`None` without `--log` (tests, `--describe`, `--replay`). Nodes that record per-session artifacts (e.g. vision frame recording) write under `<session_dir>/<artifact>/` |
| `sub.value()`, `sub->field()`, `has_value()` | `sub.value` (`None` before the first), `has_value` | |
| `sub.fresh()`, `fresh(max_age)`, `age()`, `updated()` | `sub.fresh()`, `fresh(max_age)`, `age`, `updated` | fresh: younger than `[robot] command_timeout_us` |
| `pub.send(M{...})` | `pub.send(field=value, ...)` or `send(message)` | |
| `StateMachine<S> sm{node, S::A}`: `state()`, `is()`, `go()`, `previous()`, `time_in_state()`, `just_entered()` | `talos.StateMachine(node, A, states=S)`: same, `is_()`; `states=` (optional) makes reprs and errors show names | |
| `node.publish_bytes(topic, max_bytes, {.drop_newest, .external})` → `send(span<const byte>)` | `node.publish_bytes(topic, max_bytes, drop_newest=..., external=...)` → `send(bytes)` | variable-length feed outside the loop (not logged); used by telemetry |
| `TALOS_NODE(Class)` in one `.cc` | `talos_py_node(class_name = ...)` | |

Messages are FlatBuffers **structs** (no tables or strings), at most 8-byte
aligned. Topics are typed by schema identity (`Descriptor<M>`, name and
schema hash): attaching with a mismatched schema fails, naming both types.
Python publisher, target-lane, and simulation forwarding requires matching
schema names, schema hashes, and byte sizes; mismatches raise `TypeError`
before publication.
A relative topic name (`"state"`, `"event/shot"`) expands to
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

`age()` and `fresh()` count from the writer's publication timestamp: draining
an expired message cannot revive it. Plain subscriptions without callbacks
dispatch only the newest available message per drain (counting skipped messages
as dropped), while callback subscriptions retain sequence delivery capped at 64
messages per drain. `testing::Sim` and replay preserve publication-age
semantics, so a test sees messages and ages as the robot would.

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
| `talos::Motor m{node, "roller"}`: `set_duty_cycle`, `set_voltage`, `set_velocity`, `set_position`, `set_motion_magic` (`ff_volts`, `slot`), `stop`; `position()`, `velocity()`, `voltage()`, `current()`, `connected()`, `clamped_count()` | `talos.Motor(node, "roller")`, getters as properties; demands/feedforward clamp to configured limits, non-finite input neutralizes the motor; `clamped_count` counts adjusted setter calls |
| `CANcoder`, `Gyro` (`yaw`, `yaw_rate`), `Encoder`, `DigitalInput` (`get()`), `AnalogInput` (`voltage()`), `DigitalOutput`, `PwmOutput` (`set()`) | same names; `DigitalInput.value`; DIO/PWM commands are gated on `robot_enabled` (false/zero while disabled or state is stale) |
| `talos::DriverStation ds{node}`: `connected()`, `enabled()`, `autonomous()`, `teleop()`, `test()`, `estop()`, `alliance()`, `match_time()`, `joystick(port)` → `axis(i)`, `button(n)` (1-based), `pov(i)` | `talos.DriverStation(node)`, same as properties/methods |
| `talos::robot_enabled(node)` | `talos.robot_enabled(node)` |
| `node.extension<talos::HardwareLink>().sample_time()`: node-clock `Time` the latest hardware state was sampled at, `nullopt` while stale or before the bridge's clock mapping converges (8 states) | `talos.hardware_sample_time_ns(node)` (int ns, or `None`) |
| `node.extension<talos::HardwareLink>().fresh()` / `.fresh(max_age)` and `.age()` | `link = talos.HardwareLink(node)`; `link.fresh()` / `link.fresh(max_age)` (`None` uses command timeout; seconds or `datetime.timedelta`); `link.age` (float seconds, `math.inf` before the first state); `link.sample_time` (int ns or `None`) |
| `Motor` / `CANcoder`: `position_time()`, `velocity_time()`; `Gyro`: `yaw_time()`, `yaw_rate_time()` | same names as properties: integer node-clock nanoseconds, or `None` |

`Motor::rezero(position_rot = 0)` and `Gyro::rezero(yaw_rot = 0)` (same
methods in Python) set a software offset shared by all handles for that device
on the node. They do not reset hardware and work while disabled. Disconnected,
stale, or non-finite feedback and non-finite inputs leave the offset unchanged.
Motor position and gyro yaw subtract the offset; velocity/rate are unchanged.
Motor position and motion-magic goals translate back to hardware coordinates
before clamping. Configured soft limits remain in the **hardware frame** and
the gateway's safety checks are never bypassed.

Signal acquisition times subtract the signal's existing gateway age (microseconds)
from the mapped hardware State time. They are available while disabled, but
absent for disconnected feedback, unknown age (`UINT32_MAX`), stale hardware or
stale/invalid bridge clock mapping. They retain `sample_time()`'s one-sided
transport-latency uncertainty; they do not synchronize clocks or interpolate
measurements. Software rezero does not change these timestamps.
`SimHardware.set_motor`, `.set_cancoder` accept optional `position_age_us`,
`velocity_age_us`; `.set_gyro` accepts `yaw_age_us`, `yaw_rate_age_us` (C++ and
Python). Defaults are zero; `UINT32_MAX` injects unknown acquisition age.

The robot drivetrain timestamps odometry with the oldest consumed module/gyro
acquisition time, rejecting measurements older than 50 ms or skewed by more
than 20 ms rather than pretending they are simultaneous. Invalid module timing
clears `modules_valid` (and integrated yaw); invalid gyro timing clears
`yaw_valid`. This estimation policy does not gate actuation. See
`2026-robot/main_processor/drivetrain/README.md` for details.


Every motor not set during an `update()` is sent neutral, and all are neutral
while the robot is disabled. Units are rotations, rotations/s, volts, amps.

## Targets

What a subsystem is asked to do arrives on lanes `/<sub>/target/<source>`, one
per producer, each carrying `talos::TargetEnvelope<M>` (a 16-byte header: the
producer's Driver Station mode generation and `active`; then `M`). Header
`talOS/devices/target.h` (included by `devices.h`).

| C++ | Python |
|---|---|
| `talos::Target<M> target_{node}` (constructor) | `talos.Target(node, M)` |
| `const M* t = target_.get()`: null unless `robot_enabled(node)` and the current Driver Station mode has a lane that is active, fresh and stamped with the current mode generation | `t = target.get()` (`None`) |
| `target_.source()`: `"teleop"`, `"auto"`, ... or `""` | `target.source` |
| `talos::TargetLane<M> lane_{node, "wrist", "auto"}`: `send(M{...})` (stamped with this node's mode generation), `release()` | `talos.TargetLane(node, M, "wrist", "auto")`: `send(field=...)` or `send(message)`, `release()` |

Which lanes count, highest priority first, is config:

```toml
# robot.toml: every subsystem (these are the defaults)
[robot.targets]
teleop = ["teleop"]
autonomous = ["auto"]
test = []

# subsystems/drivetrain.toml: replaces only the modes it lists
[targets]
teleop = ["align", "teleop"]
```

### Binding joystick buttons and axes via TOML

A mechanism subsystem can bind Driver Station joystick buttons and axes to its
target lane without editing C++ or Python code. The framework node
`//talOS/teleop:node` (`subsystems/teleop.toml`) reads `[[teleop.bindings]]`
tables from each subsystem configuration and writes the subsystem's
`/<sub>/target/teleop` lane using its schema descriptor:

Python authors can exercise these bindings in `talos.testing.Sim` by depending on `//talOS/python:teleop`, importing `Teleop` from `talos.teleop`, and calling `sim.add(Teleop, "teleop")` after adding their subsystem nodes.

```toml
# subsystems/shooter.toml
[[teleop.bindings]]
button = 1                     # joystick 0 button (1-based); add port = N for other joysticks
while_held = { target_velocity_rps = 60.0 }

# subsystems/arm.toml
[[teleop.bindings]]
axis = 1                       # axis index [0, 11]
field = "velocity_rps"         # field in target message struct
scale = 60.0                   # in the field's unit
deadband_frac = 0.05           # optional deadband [0, 1)
```

Bindings select exactly one `button`, `axis` or `pov`. A `pov` is a compass
direction in degrees (0, 45, ..., 315); optional `pov_index` selects hat 0..3.
Button and POV bindings require exactly one nonempty scalar table:
`while_held = { ... }` or `on_toggle = { ... }`. A toggle changes on the
rising edge and resets on disable, leaving teleop or joystick disconnect.
While held, toggled on or deflected past deadband, `teleop` publishes active
targets with the bound fields encoded; released or in neutral, it calls
`release()`.

Sources follow the topic segment rules; an unknown mode is a config error. A
consumer reads its lanes without a callback, so it keeps draining right before
its timer. Republish a lane every period; `release()` hands control to the next
lane on the consumer's next tick. The launcher refuses a lane whose source no
mode lists and warns about a policy lane nobody writes. Each `get()` also
records the selected source in the node's registry row, so Studio's system view
shows who drives the subsystem ("driven by teleop", or "none") with nothing to
add in the node.

A lane drives only the Driver Station mode generation it was sent under: the
RoboRIO stamps every Driver Station packet with `mode_since_us` (the first
observation of the current enable and mode, tracked every hardware tick), a
`TargetLane` stamps the generation its node last drained, and `Target` ignores
any other. A target computed before a disable, re-enable, mode switch or
re-attach never actuates after it,
even if it arrives in the same drain as the packet that changed the mode; the
producer's next send, made from the new packet, is accepted. In tests,
`PublishTarget` / `sim.publish_target` stamp the generation of the newest
Driver Station packet; pass `mode_since_us` (C++: the last argument) to stand
in for a producer that has not seen it.

## BUILD

```python
load("//talOS/build:contract.bzl", "talos_cc_node", "talos_messages", "talos_py_node")

talos_messages(name = "messages", srcs = ["shooter_message.fbs"],
               topics = {"state": "/shooter/state"})
# :messages    C++: #include "<package>/shooter_message_generated.h"
#                   #include "<package>/topics.h"   shooter::topics::kState
#                   #include "<package>/shooter_message_descriptor.h"
#                     talos::Descriptor<shooter::ShooterState>::kHash
# :messages_py Python: from <namespace>.ShooterTarget import ShooterTarget
#                      from <namespace> import topics   topics.STATE
#                      ShooterTarget.TALOS_SCHEMA_HASH
# :messages_bfbs  shooter_message.bfbs (FlatBuffers reflection.Schema)

talos_cc_node(name = "node", node_name = "shooter", srcs = ["shooter.cc"], deps = [":shooter"])
talos_py_node(name = "node", node_name = "intake", main = "intake.py",
              class_name = "Intake", deps = [":messages_py"])
```

`include_prefix` on `talos_messages` overrides the package-basename prefix;
with `topics`, it must be the schema's namespace. Topic keys are
lower_snake_case: `"vision_state"` is `kVisionState` in C++ and
`VISION_STATE` in Python. Target lanes are not listed: `TargetLane` and
`Target` build them from the subsystem and source names.
To use another package's types, list its `talos_messages` target in `deps`
and include its schema by the dep's include prefix:
`deps = ["//shared/geometry:messages"]` with
`include "geometry/geometry.fbs";` in the `.fbs`, then C++ includes
`"geometry/geometry_generated.h"`. Deps are transitive. At a repository root
the package path is empty, so pass `include_prefix` explicitly there.
A package whose messages all come from `deps` declares only its topics:
`talos_messages(name = "messages", srcs = [], topics = {"state":
"/pose_estimator/state"}, deps = ["//talOS/msgs:geometry"])` gives
`<prefix>/topics.h` and `<prefix>.topics` plus the dep's types.
Every numeric struct field must end in a unit suffix from
[METHODOLOGY.md §6](METHODOLOGY.md#6-units) (`_rps`, `_m`, `_count`, ...);
bools, enums and structs need none. A field without one fails the build,
naming the field. Each struct also gets a schema descriptor
(`talOS/schema/descriptor.h`): its `.bfbs` bytes, its qualified name and a
64-bit schema hash that changes with any field name, type, offset, nested
struct or enum value, but not with comments or formatting. Python's
`talos_schema.struct_hash` computes the same hash from the same bytes.
Logs (`.tlog` format version 5) carry these descriptors, and `log_dump`
prints decoded fields.
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
hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop));
sim.step(10ms);                       // the mode is current
PublishTarget(sim, "shooter", "teleop", ShooterTarget{80.0});
sim.step(20ms);
hw.motor_request("shooter", "flywheel").mode;  // Mode::kVelocity
sim.latest<ShooterState>(topics::kState);     // std::optional
ReleaseTarget<ShooterTarget>(sim, "shooter", "teleop");
```

`SimHardware` also has `set_motor`, `set_digital_input`, `set_analog_input`,
`set_encoder`, `set_cancoder`, `set_gyro`, `set_driver_station`,
`stop_driver_station()` (the DS goes silent), `set_connected(node, device,
false)` (the device reports disconnected), `digital_output`, `pwm`.
`LatestTarget<M>(sim, sub, source)` reads a lane a node under test wrote.

`motor_request`, `digital_output`, and `pwm` retain each device's last
explicitly requested value. Use `applied_motor`, `applied_digital_output`, and
`applied_pwm` to inspect effective outputs: requests expire after
`[robot] command_timeout_us` without publication, omitted outputs are neutral,
and disabled hardware applies neutral outputs. Inspection never refreshes
freshness. With `ideal_motors()`, neutral velocity decays each tick by
`min(1, period/tau)`, with `tau = 50 ms` for brake and `1 s` for coast;
velocity below `0.01 rps` snaps to zero. Position continues integrating during
decay. Python exposes the same accessors.

Python: `from talos.testing import Sim, SimHardware` (`//talOS/python:testing`),
the same methods (`Sim.from_toml`, `sim.step(seconds)`,
`sim.publish(M, topic, **fields)`, `sim.latest(M, topic)`,
`sim.publish_target(M, sub, source, **fields)`, `sim.release_target(M, sub,
source)`, `sim.latest_target(M, sub, source)`, `hw.set_driver_station(enabled=True,
teleop=True)`, `hw.stop_driver_station()`, `sim.latest_bytes(topic)`). See
`2026-robot/main_processor/intake/node_test.py`.

## Adding a pose estimator

An estimator is a hardware-free node: it reads state topics and publishes a
`geometry.PoseEstimate` (`//talOS/msgs:geometry`) on its own `/<name>/state`
every tick, with independent `valid` and `localized` flags. `valid` indicates
fresh usable integrated odometry; `localized` indicates a trusted field frame
and latches for the estimator process lifetime. Field-pose consumers require
both. Absolute measurements arrive as
`geometry.PoseObservation` (`sample_time_ns`, `pose`, `x_std_m`, `y_std_m`,
`yaw_std_rad`, `valid`) on each camera node's `/<camera>/state`.

```sh
bazel run //tools:new_subsystem -- localizer --lang py --type estimator   # or --lang cpp
```

writes the package (topics-only `talos_messages`, the node, a Sim test of the
disabled and stale cases), `subsystems/localizer.toml` with `cameras = []`,
the `[[subsystems]]` entry and the manifests. The generated node subscribes
to `/<camera>/state` for each name in `cameras` with `optional = true`, so a
robot without that camera still passes the graph lint, and reads each
observation once (`updated`). Rules every estimator keeps:

- Never gate on `robot_enabled`: it writes no actuator, and the pose must be
  right before the match is enabled.
- Time everything by sample time (`DrivetrainState.sample_time_ns`,
  `PoseObservation.sample_time_ns`), never by arrival or `node.now`; with
  wpimath call `updateWithTime(sample_time_ns / 1e9, ...)`, never `update()`.
- Reject an observation that is invalid, non-finite, has a non-positive
  standard deviation, or is older or newer than the estimator can place, and
  count it; hold the pose and publish `valid = false` while the inputs are
  stale.
- Name the pose in consumers by config only:
  `[studio] pose_topic = "/localizer/state"`.

`2026-robot/main_processor/pose_estimator` is the full example: swerve
odometry plus cameras, with `backend = "kalman"` (wpimath
`SwerveDrive4PoseEstimator`) or `"factor_graph"` (a GTSAM fixed-lag
smoother), and tests that run every scenario against both.

Tune or verify offline with `launcher --replay-inputs <session> --only <node>`.
The node runs on a simulated loop fed recorded inputs from the session origin
(monotonic t = 0), including messages missed during live startup. Two replays
of a session yield bit-identical records; pass `--session-id <id>` if you need
to pin the header session ID as well.
