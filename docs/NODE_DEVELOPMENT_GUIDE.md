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
| `node.param<T>("key")`, `param<T>("key", fallback)` | `node.param("key")`, `param("key", fallback)` | this subsystem's TOML keys; dotted keys reach nested tables and an index reaches an array element (`"geometry.modules[0].x_m"`; past the end is absent). `T`: `bool`, integers, `double`, `std::string`, and TOML arrays as `std::vector<std::string>` / `std::vector<double>` (integer elements convert; `[]` is valid). Python returns arrays as `list[str]` / `list[float]`; a list default asks for an array. A wrong element type is an error naming the key and element index |
| `node.param_of<T>("sub", "key")`, `param_of<T>("sub", "key", fallback)` | `node.param_of("sub", "key")`, `param_of("sub", "key", fallback)` | the same, read-only, from another subsystem's table (`[subsystems.<sub>]`), for a value that subsystem owns so it is configured once (the pose estimator reads the drivetrain's module offsets). Constructor only: later it throws `std::logic_error` (`TalosError`). A subsystem the config lacks is an error even with a fallback, so the node fails construction and its launcher `--describe` probe |
| `node.every(period, fn)` | `node.every(seconds, fn)` | extra timer |
| `name()`, `now()`, `tick()`, `period()`, `simulation()`, `config_path()` | same, as properties (seconds) | |
| `session_dir()` → `std::optional<std::filesystem::path>` | `node.session_dir` → `str \| None` | the session output directory the launcher started this node under (the parent of `--log`), `nullopt`/`None` without `--log` (tests, `--describe`, `--replay`). Nodes that record per-session artifacts (e.g. vision frame recording) write under `<session_dir>/<artifact>/` |
| `describing()` → `bool` | `node.describing` → `bool` | true only while the binary builds the node for `--describe`, the launcher's probe, which has a deadline (`--describe-timeout-ms`) and never runs `update()`, `every()` or a callback; false under `Sim`, `--sim`, `--replay` and on the robot (a `talos.cpp_node` shares its host's node). Use it to skip building heavy native backends (the pose estimator imports neither wpimath nor gtsam then). Validate every parameter and declare every subscription and publication either way: the endpoints must be identical, since the launcher builds the graph from the probe, and it [compares each run's live endpoints with the probe](../talOS/launcher/README.md#live-endpoints) and refuses a run that differs |
| `sub.value()`, `sub->field()`, `has_value()` | `sub.value` (`None` before the first), `has_value` | |
| `sub.fresh()`, `fresh(max_age)`, `age()`, `updated()` | `sub.fresh()`, `fresh(max_age)`, `age`, `updated` | fresh: younger than `[robot] command_timeout_us` |
| `pub.send(M{...})` | `pub.send(field=value, ...)` or `send(message)` | Python names every field, as the C++ constructor takes every one: an omitted field raises `TypeError` naming it (also for `TargetLane.send`, `sim.publish`, `sim.publish_target`) |
| `StateMachine<S> sm{node, S::A}`: `state()`, `is()`, `go()`, `previous()`, `time_in_state()`, `just_entered()` | `talos.StateMachine(node, A, states=S)`: same, `is_()`; `states=` (optional) makes reprs and errors show names | |
| `node.publish_bytes(topic, max_bytes, {.drop_newest, .external})` → `send(span<const byte>)` | `node.publish_bytes(topic, max_bytes, drop_newest=..., external=...)` → `send(bytes)` | variable-length feed outside the loop (not logged); used by telemetry |
| `TALOS_NODE(Class)` in one `.cc` | `talos_py_node(class_name = ...)` | |

Message, schema identity, relative topic and remapping contracts are in
[Architecture: Topics and messages](../ARCHITECTURE.md#topics-and-messages)
and [Configuration](../ARCHITECTURE.md#configuration).

### Parameters

A `param` or `param_of` call in the constructor declares its key, whether
the config sets it or not (C++, Python and `talos.cpp_node` alike):
`--describe` lists every key the constructor read, and the launcher refuses
a key in a subsystem table that no node read, naming the subsystem, the
key, the file that sets it and the nearest declared key
([graph lint](../talOS/launcher/README.md#parameter-keys)). Framework keys
(`config::kFrameworkSubsystemKeys`: `node`, `period_us`, `motors`,
`targets`, ...) are not parameters. So read every key a config may set in
the constructor, including those of a branch the config does not select (the
pose estimator validates both backends' tables); a key read only in
`update()` or a callback is undeclared, and a table read without `param`
declares nothing.

### Threads

A node belongs to the thread that builds it, which also runs its loop (a
`Sim` builds and steps its nodes on the calling thread). Sends (`send` on a
publisher, byte topic or `TargetLane`, and `release`) and device commands
(`Motor` setters, `stop` and `rezero`, `Gyro.rezero`, `DigitalOutput.set`,
`PwmOutput.set`) are accepted only there: in the constructor, `update()`,
`every()` and message callbacks. From any other thread they throw
`std::logic_error` (`TalosError` in Python, whose C calls release the GIL), so
a worker thread hands its results to `update()` instead.

Output commands (`Motor` setters and `stop`, `DigitalOutput.set`,
`PwmOutput.set`) are narrower: they are accepted only inside `update()`. The
flush after each `update()` sends neutral for every output that `update()` did
not set, so a command from the constructor, `every()` or a callback would
reach the hardware for at most one tick (a 20 ms `every()` on a 5 ms node
would drive a 25% duty cycle). They throw `std::logic_error` there
(`TalosError` in Python), naming the device; an `every()` or callback stores
what it decided and `update()` applies it. Sends and `rezero` stay legal
anywhere on the loop thread.

### When messages arrive

See [Architecture: Targets](../ARCHITECTURE.md#targets) for drain scheduling
and publication-age semantics.

### Realtime scheduling

On the robot (not `--sim`, `--describe` or `--replay`) a node binary runs its
loop thread under a realtime policy chosen by its tier
(`talos::process::LoopTier`), which the process decides itself: the bridge
(`hardware_node`) is the highest tier; a node on the actuation path, whose own
subsystem owns a motor, digital output or PWM output or which writes a target
lane (`driver`, `teleop`, `auto`), is a control node; every other node
(telemetry, cameras, the pose estimator, Python observers) is an observer.

- Linux: `SCHED_FIFO` priority 49 for the bridge, 45 for control nodes and 40
  for observers, so a runnable loop preempts every loop of a lower tier. All
  are below the `SCHED_FIFO` 50 PREEMPT_RT gives threaded interrupt handlers
  (among them the NIC's, which carries the RoboRIO's packets), so no loop
  holds off an interrupt. A node's tick, and the bridge between its inputs,
  waits in `epoll_pwait2`, a sleeper timer that PREEMPT_RT expires in hard
  interrupt context for a realtime thread, so the woken loop runs by its
  tier. On a kernel before 5.11 that wait falls back to a timerfd
  ([talOS/events](../talOS/events/README.md)), which PREEMPT_RT expires in
  `ktimers/N` at `SCHED_FIFO` 1 (`ksoftirqd`, `SCHED_OTHER`, on some older
  RT kernels), so there a control node's tick, or the bridge's next
  time-driven duty, can wait until whatever loop holds that CPU, an observer
  included, sleeps. On a kernel without
  PREEMPT_RT timers expire in hard interrupt context, but softirq work
  deferred to `ksoftirqd` (`SCHED_OTHER`), such as network receive under
  load, waits behind every loop on that CPU (TODO.md RT-4).
- macOS: the bridge and control nodes run under the Mach time-constraint
  policy for their period (the bridge, which has none, for 1 ms after each
  wakeup; computation half of it, constraint all of it);
  observers run timeshare at `QOS_CLASS_USER_INTERACTIVE`, below every
  time-constraint thread and above helper threads. The public time-constraint
  policy has no priority (the variant with one is private, and refused
  without a boot argument): the kernel orders those threads by deadline (wake
  time plus constraint), not by tier, so on macOS the bridge does not take
  precedence over the control nodes; only observers are ordered below them.

The node also keeps the memory its loop touches resident, so a tick does not
take a page fault:

- Linux: `mlockall(MCL_CURRENT | MCL_FUTURE)` locks every mapping, present
  and future.
- macOS has no working `mlockall`, so the node locks ranges: every
  shared-memory ring and the loop thread's stack (below). Heap and other
  mappings are not locked. The `__TEXT` segment of the image holding talOS
  (the node binary, or `libtalos_node.so` in a Python node) is read in page
  by page but not locked, since macOS refuses `mlock` of executable pages;
  system libraries and the Python interpreter are left alone.
- Both: every RTMS ring the process maps, before or after the loop starts,
  is prefaulted for writing and `mlock`ed (`talOS/memory` `residency.h`), and
  the loop thread's stack is prefaulted 256 KiB deeper than where the loop
  starts (less on a smaller stack) and locked up to its top. On Linux this
  keeps the rings and stack locked even when `mlockall` is refused.

When the OS refuses a step (no privilege, `RLIMIT_MEMLOCK` or the wired-memory
limit) the node prints one line to stderr (`talos: realtime scheduling
incomplete`, naming its tier, grant and the refused steps) and runs anyway,
with whatever it could take; a ring or stack it could not lock stays
prefaulted. A ring mapped after that line whose lock is refused is reported
once more per process, on stderr only. `ConfigureRealtime` returns the outcome
(`talOS/process` `realtime_outcome.h`: the tier, whether it asked, the refused
steps, and from them the grant: `full`, `partial` when only a memory step was
refused, `refused` when the scheduling policy was, or `not_requested`), and
the node publishes it in its registry row ([live registry](../talOS/introspection/README.md#realtime-scheduling)),
where the launcher warns about a bridge or control loop without its full grant
and `session_report --strict` fails the session
([session measurements](../talOS/launcher/README.md#session-measurements)).
`TALOS_REALTIME=0` in the environment turns all of this off;
`TALOS_REALTIME=1` turns it on under `--sim` as well, so a host without a
RoboRIO runs the actuated control path, its nodes and bridge under the
production policy, against a simulated gateway (which itself never runs
realtime) (`talOS/process` `RealtimeRequested`). Nodes and the bridge
inherit the variable from the launcher.

A thread the node starts must not run at the loop's priority. On Linux a
thread created after the loop thread went realtime inherits `SCHED_FIFO`, so
an unbounded helper (capture, detection, encoding) would preempt every control
loop; under timeshare (`--sim`) it competes with them as an equal. A helper
thread calls `talos::process::ConfigureBackgroundThread()` first: on Linux it
leaves an inherited realtime policy for `SCHED_OTHER` at nice
`kBackgroundNice` (10), which threads it creates inherit; on macOS it sets
`QOS_CLASS_UTILITY`, which new threads do not inherit, so each thread calls it
itself (a dispatch queue takes the QoS in its attributes). In the vision
node the camera worker, the Metal compile thread and the frame recorder call
it; the V4L2 capture thread inherits it from the worker, the AVFoundation
capture queue is created at that QoS, and the AprilTag detector pool's
threads, which the library creates, each call it from a task
(`EnsureBackgroundWorkerPool`). Threads a Python node starts have no such
call: start none whose work is unbounded.

### Stopping

Once `talos::Run` / `talos.run` starts a live node, SIGINT or SIGTERM stops it
cleanly, at once: the signal makes the process's stop pipe
(`process::StopFd()`) readable, which ends the loop's wait wherever it waits,
and the loop exits from its own thread, so no thread polls for a stop. A stop
received while the constructor runs is held until the
constructor returns; the binary then exits 0 without running its loop. A
constructor blocked in a restartable system call keeps blocking (the call
restarts), so the launcher's SIGKILL after its stop grace is what ends one that
never returns. Before that point (a Python node's interpreter startup and
imports, `--describe`, `--replay`) signals keep their default action.

### Python garbage collection

`talos.run` (a `talos_py_node` binary, including under `--sim` and
`--describe`) calls `gc.collect()` then `gc.freeze()` once the node object is
built, before its loop runs: everything alive after construction (modules,
config, the node and what its constructor built) moves to the collector's
permanent generation, and later collections scan only objects allocated
since. For the pose estimator (about 33 000 tracked objects after
construction with the Kalman backend, 53 000 with the factor graph) a full
collection drops from 1.9–2.5 ms (4.9 ms for the first in a run) to under
15 µs, and the longest middle-generation collection from about 0.25 ms to
0.05 ms (desktop Ryzen 9 7950X3D, CPython 3.11, `update_benchmark`).
Young-generation collections still run, about 20 µs each. A reference cycle
built before the freeze and dropped afterwards is never collected, so drop
construction-time scratch inside the constructor. `talos.testing.Sim` does
not freeze.

## Hardware

Handles are named by their device key in the subsystem config
(`[motors.roller]` → `"roller"`).

| C++ | Python |
|---|---|
| `talos::Motor m{node, "roller"}`: `set_duty_cycle`, `set_voltage`, `set_velocity`, `set_position`, `set_motion_magic` (`ff_volts`, `slot`), `stop`; `position()`, `velocity()`, `voltage()`, `current()` (stator A), `temperature_c()`, `supply_current_a()`, `connected()`, `clamped_count()`, `forward_limit()`, `reverse_limit()` | `talos.Motor(node, "roller")`, getters as properties; demands/feedforward clamp to configured limits, non-finite input neutralizes the motor; `clamped_count` counts adjusted setter calls |
| `CANcoder`, `Gyro` (`yaw`, `yaw_rate`), `Encoder`, `DigitalInput` (`get()`), `AnalogInput` (`voltage()`), `DigitalOutput`, `PwmOutput` (`set()`) | same names; `DigitalInput.value`; DIO/PWM commands are gated on `robot_enabled` (false/zero while disabled or state is stale) |
| `talos::DriverStation ds{node}`: `connected()`, `enabled()`, `autonomous()`, `teleop()`, `test()`, `estop()`, `alliance()`, `match_time()`, `joystick(port)` → `axis(i)`, `button(n)` (1-based), `pov(i)` | `talos.DriverStation(node)`, same as properties/methods |
| `talos::robot_enabled(node)` | `talos.robot_enabled(node)` |
| `node.extension<talos::HardwareLink>().sample_time()`: node-clock `Time` the latest hardware state was sampled at, `nullopt` while stale or before the bridge's clock mapping converges (8 states) | `talos.hardware_sample_time_ns(node)` (int ns, or `None`) |
| `node.extension<talos::HardwareLink>().fresh()` / `.fresh(max_age)` and `.age()` | `link = talos.HardwareLink(node)`; `link.fresh()` / `link.fresh(max_age)` (`None` uses command timeout; seconds or `datetime.timedelta`); `link.age` (float seconds, `math.inf` before the first state); `link.sample_time` (int ns or `None`) |
| `Motor` / `CANcoder`: `position_time()`, `velocity_time()`; `Gyro`: `yaw_time()`, `yaw_rate_time()` | same names as properties: integer node-clock nanoseconds, or `None` |

Actuator flush and disable semantics are in
[Architecture: The tick](../ARCHITECTURE.md#the-tick).

Motor `temperature_c` is device temperature in Celsius; `supply_current_a`
is supply-side current, not `current` (stator current). All three use 0.1-unit
wire resolution. They are observations, not automatic thermal derating.
Check `connected`/link freshness before using readings; numeric getters latch
the last received sample when the hardware link is stale. SimHardware
`set_motor(..., temperature_c=..., supply_current_a=...)` injects diagnostics
through the same wire codec (C++ takes these optional arguments after ages).

`forward_limit()` / `reverse_limit()` (properties in Python) are true while an
enabled switch on the TalonFX's own forward or reverse limit input
([talOS/hardware](../talOS/hardware/README.md)) is asserted, so the TalonFX
holds output that way neutral; homing can drive toward a switch until it reads
true. Unlike the numeric getters they read false unless `connected()`; while
connected, the RoboRIO reports them only from a TalonFX FaultField signal under
50 ms old (2.5 periods at its 50 Hz rate) and reports them clear once that
signal is staler. SimHardware `set_limit_switches(node, device, forward,
reverse)` asserts them; it does not stop the request or the ideal motor.

### Rezero and position frames

`Motor::rezero(position_rot = 0)` and `Gyro::rezero(yaw_rot = 0)` (same
methods in Python) set a software offset shared by all handles for that device
on the node. They do not reset hardware and work while disabled. Disconnected,
stale, or non-finite feedback and non-finite inputs leave the offset unchanged.
Motor position and gyro yaw subtract the offset; velocity/rate are unchanged.
Motor position and motion-magic goals translate back to hardware coordinates
before clamping. Configured soft limits remain in the **hardware frame**,
which no rezero moves, so config load accepts `soft_limits = true` only with
`position_frame = "absolute"` (below); the gateway's safety checks are never
bypassed.
`zeroed()` (`zeroed` property in Python) is true while a rezero holds. The
offset lives in the node, so a restarted node starts unzeroed. A zero is
dropped when its hardware frame may have restarted, except an
absolute-frame motor's (below), which no reset or boot drops:

- a gateway boot id change drops every zero on the node;
- a device reset count change in the State whose `sequence` is the previous
  accepted State's plus one (the gateway's next tick) drops the zeros of the
  node's devices that State masks; the gateway counts a reset (a device's
  reset latch, or a rotor-feedback motor whose position stepped farther than
  its motor can turn; see [talOS/hardware](../talOS/hardware/README.md)) and
  masks the device, its followers and its remote-sensor consumers in the same
  State, so another device's reset keeps them;
- a device reset count change across any other sequence step (a lost State,
  a link outage, a gateway tick with no socket) drops every zero on the node.

A disconnect alone keeps the zero. A dropped zero keeps its offset, so
dropping does not move `position()` or `yaw()`, but `zeroed()` reads false.

A position needs a frame no reset has restarted, which the motor's
`[motors.<name>]` `position_frame` names:

- `"rezero"` (the default): the frame `Motor::rezero` sets. A TalonFX counts
  rotor rotations from where it powered on or last reset, wherever the
  mechanism was, so `set_position` and `set_motion_magic` go out neutral
  while no rezero holds: before the first and after a reset drops it. Home
  against a switch or hard stop, then rezero.
- `"absolute"`: the remote CANcoder's frame (`feedback = "RemoteCANcoder"`
  only; config load refuses it on rotor feedback). Position goals drive from
  the first state without a rezero, and a rezero, if taken, survives resets
  and gateway boots. A reset CANcoder rereads its absolute angle within one
  rotation, so this holds only while one CANcoder rotation covers the
  mechanism's whole travel: the CANcoder turns at most once over the travel
  without crossing its discontinuity, or, with `continuous_wrap`, once per
  mechanism rotation (`sensor_to_mechanism_ratio = 1`, as on swerve
  steering). Otherwise a reset shifts the mechanism frame by whole CANcoder
  rotations: at a ratio of 2, a reset at 0.7 rereads -0.3, half a turn off.

Velocity, voltage and duty cycle are never held, and the substitution does
not count in `clamped_count`.

### Signal acquisition times

Signal acquisition times subtract the signal's existing gateway age (microseconds)
from the mapped hardware State time. They are available while disabled, but
absent for disconnected feedback, unknown age (`UINT32_MAX`), stale hardware or
stale/invalid bridge clock mapping. They retain `sample_time()`'s reported
two-way clock uncertainty (`/hw/status/clock_offset_uncertainty_us`, assuming
<=1000 ppm relative clock drift); they do not interpolate measurements.
Software rezero does not change these timestamps.
`SimHardware.set_motor`, `.set_cancoder` accept optional `position_age_us`,
`velocity_age_us`; `.set_gyro` accepts `yaw_age_us`, `yaw_rate_age_us` (C++ and
Python). Defaults are zero; `UINT32_MAX` injects unknown acquisition age.

The robot-specific odometry timing policy is documented in the
[drivetrain README](../2026-robot/main_processor/drivetrain/README.md).


## Targets

See [Architecture: Targets](../ARCHITECTURE.md#targets) for lane selection,
mode generations and policy defaults. API:

| C++ | Python |
|---|---|
| `talos::Target<M> target_{node}` (constructor) | `talos.Target(node, M)` |
| `const M* t = target_.get()`: null unless `robot_enabled(node)` and the current Driver Station mode has a lane that is active, fresh and stamped with the current mode generation | `t = target.get()` (`None`) |
| `target_.source()`: `"teleop"`, `"auto"`, ... or `""` | `target.source` |
| `talos::TargetLane<M> lane_{node, "wrist", "auto"}`: `send(M{...})` (stamped with this node's mode generation), `release()` | `talos.TargetLane(node, M, "wrist", "auto")`: `send(field=...)` or `send(message)`, `release()` |
| `talos::OperatorArming arming_{node}`; `talos::TargetLane<M> lane_{node, "climber", "operator", arming_}`: `send()` goes out as a release (and returns false) while `arming_.armed()` is false | `talos.OperatorArming(node)`; `talos.TargetLane(node, M, "climber", "operator", arming=arming)` |
| `arming_.evaluate(neutral)` on every reading of the inputs: true when this reading may publish; otherwise a neutral reading arms it at once (a send on that neutral reading is active). `disarm()` for producer-specific reasons (a joystick reconnecting) | `arming.evaluate(neutral)`, `arming.armed`, `arming.disarm()` |

A producer that maps operator input (joysticks, a button board) opens its
lanes with an `OperatorArming`, so a control held through a disable, reboot,
Driver Station dropout or stall of the producer does not drive until it has
been released once (the full list:
[Architecture: Targets](../ARCHITECTURE.md#targets)):

```cpp
// in update(), with ds_ a talos::DriverStation:
const bool held = ds_.joystick(1).button(3);
if (arming_.evaluate(/*neutral=*/!held) && held) {
  lane_.send(climber::ClimbTarget{...});
} else {
  lane_.release();
}
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

Republish a lane every period; use `release()` when the producer has no target.
In tests, `PublishTarget` / `sim.publish_target` stamp the newest Driver Station
generation; pass `mode_since_us` (C++: the last argument) to model a producer
that has not seen it.

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
with `topics`, it must be the schema's namespace. Topic names follow
[NAMING.md](../talOS/NAMING.md), checked when the `BUILD` file loads. Topic
keys are lower_snake_case: `"vision_state"` is `kVisionState` in C++ and
`VISION_STATE` in Python. Target lanes are not listed: `TargetLane` and
`Target` build them from the subsystem and source names.
To use another package's types, list its `talos_messages` target in `deps`
and include its schema by the dep's include prefix:
`deps = ["//talOS/msgs:geometry"]` with
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
Logs carry these descriptors, and `log_dump` prints decoded fields.
See [Data flow: Dispatch logs](DATA_FLOW.md#dispatch-logs) for the format.
The node's name comes from `node_name`; add the node to the robot config as
`node = "//path:node"`.

### C++ nodes hosted from Python

A C++ node class can run inside a Python node: export it with
`TALOS_EXPORT_NODE(name, Class)` in a library built by
`talos_cc_shared_node(name = "libmechanisms.so", srcs = [...], deps = [...])`,
and wrap it with `Intake = talos.cpp_node("<package>/libmechanisms.so",
"intake")`. That library links the framework dynamically from
`//talOS/node_api:libtalos_node.so`, which `import talos` has already loaded,
so the hosted class runs on the process's one copy of `Node` and the devices
layer: the loader matches the library's dependency to that image by its
install name (macOS) or soname (Linux), even when a bundle holds two copies of
the file. `libtalos_node.so` links every talOS library it carries whole, so a
hosted class may call any of their functions; a symbol it does not define
fails the library's link (on Linux the macro links with `-z defs`), not its
load. Do not build it with `cc_binary(linkshared = True)`: that library would
carry a private copy of the framework (its own statics and extension type
identities) running against Python's `Node`. A library listing a target the
framework already links but does not export fails at analysis.

## Testing

Which cases every node tests, and how: [TESTING.md](TESTING.md#7-the-node-test-matrix).

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
false)` (the device reports disconnected), `reset_device(node, device)` (the
gateway reports that motor, CANcoder or gyro reset and masks it, with its
followers and remote-sensor consumers, for two States; any other name throws),
`lose_states(count)` (the next gateway ticks publish nothing), `reboot()` (a
new gateway boot id; every output neutral until a request built from it),
`digital_output`, `pwm`.
`LatestTarget<M>(sim, sub, source)` reads a lane a node under test wrote.

`motor_request`, `digital_output`, and `pwm` retain each device's last
explicitly requested value. Use `applied_motor`, `applied_digital_output`, and
`applied_pwm` to inspect effective outputs: requests merge by the bridge's
rules (`//talOS/bridge:merge`), so a node's outputs go neutral at its
[owner timeout](../ARCHITECTURE.md#the-tick) and every output stops, as the
gateway's do, once the oldest observation of any node still merged is
`command_timeout_us` old; omitted outputs are neutral, and disabled hardware
applies neutral outputs. As on the bridge and gateway, a request built from
another boot's or epoch's State is ignored, and a motor masked by
`reset_device` applies neutral (so does a later request observed before the
State that masked it); a motor disconnected with `set_connected` still
applies its request. Inspection never refreshes
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
robot without that camera still passes the graph lint, with a callback that
queues every observation in sequence (at most 64 per camera; the oldest is
dropped and counted as rejected) for `update()` to read once. A latest-value
read (`updated`) would lose every observation but the newest between ticks.
Rules every estimator keeps:

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
smoother), and tests that run its scenarios against both.

Tune or verify offline with `launcher --replay-inputs <session> --only <node>`.
The node runs on a simulated loop fed recorded inputs from the session origin
(monotonic t = 0), including messages missed during live startup. Two replays
of a session yield bit-identical records; pass `--session-id <id>` if you need
to pin the header session ID as well.
