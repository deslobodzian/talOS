# Engineering methodology

Working conventions for talos code. [RULES.md](../RULES.md) holds
the hard rules; [ARCHITECTURE.md](../ARCHITECTURE.md) holds architecture,
tick, target, topic and configuration contracts. API reference:
[NODE_DEVELOPMENT_GUIDE.md](NODE_DEVELOPMENT_GUIDE.md).

Each rule carries its status:

| Mark | Meaning |
|---|---|
| **[enforced]** | the build, the graph lint or a test fails when it is broken |
| **[review]** | a convention; the `reviewer` agent (`.agents/agents/reviewer.md`, `.claude/agents/reviewer.md`) checks it |
| **[gap]** | stated, but nothing checks it yet; names the TODO.md item that adds the check |

## 1. What the four goals mean here

| Goal | Means | Checked by |
|---|---|---|
| Robust | A node never moves a motor on a disabled robot, a stale target, missing feedback or a non-finite number; bad config fails at construction, before anything moves. | §5, §8, the [node test matrix](TESTING.md#7-the-node-test-matrix) |
| Reliable | A node does the same thing on the robot, in `Sim` and in replay; a restart mid-match does nothing unsafe; a dead node's own outputs go neutral by its owner timeout, and when the others keep driving is stated in [the tick](../ARCHITECTURE.md#the-tick). | §9, the [node test matrix](TESTING.md#7-the-node-test-matrix) |
| Performant | `update()` is bounded, allocation-free and I/O-free; every control path has a known latency. | §10 |
| Easy | Adding a subsystem means a node package, its TOML and one manifest line. Nothing else changes, in any language. | §12 |

## 2. Anatomy of a node

A node is a class. The constructor opens every handle: devices, inputs,
outputs and parameters. `update()` runs every `period_us` in three steps, in
this order: **read** inputs, **decide**, **write** outputs. Nothing is opened
after construction **[enforced: `--describe` records the constructor's topics]**.

```cpp
// wrist/wrist.h
class Wrist {
 public:
  explicit Wrist(talos::Node& node)
      : motor_{node, "motor"},                                   // [motors.motor]
        angle_{node, "angle"},                                   // [sensors.angle]
        tolerance_rot_{node.param<double>("tolerance_rot")},     // required: no fallback
        target_{node},                                           // /wrist/target/<source> lanes
        state_{node.publish<control::PositionState>("state")} {
    if (!(tolerance_rot_ > 0)) throw std::invalid_argument("tolerance_rot must be > 0");
  }

  void update() {
    // read: null unless the robot is enabled and the lane selected for the
    // current Driver Station mode is active, fresh and of this mode generation
    const control::PositionTarget* t = target_.get();
    const bool feedback = motor_.connected() && angle_.connected();
    const bool drive = t != nullptr && feedback && std::isfinite(t->position_rot());
    // decide + write: actuators only inside `drive`; everything else goes neutral by omission
    if (drive) motor_.set_motion_magic(t->position_rot());
    const double goal_rot = drive ? t->position_rot() : angle_.position();
    state_.send(control::PositionState{
        angle_.position(), angle_.velocity(), goal_rot, drive,
        drive && std::abs(angle_.position() - goal_rot) <= tolerance_rot_, feedback});
  }

 private:
  talos::Motor motor_;
  talos::CANcoder angle_;
  double tolerance_rot_;
  talos::Target<control::PositionTarget> target_;
  talos::Publisher<control::PositionState> state_;
};
// wrist.cc:  TALOS_NODE(wrist::Wrist)
```

```python
# wrist.py: the same node, line for line
class Wrist:
    def __init__(self, node):
        self.node = node
        self.motor = talos.Motor(node, "motor")
        self.angle = talos.CANcoder(node, "angle")
        self.tolerance_rot = node.param("tolerance_rot")
        if not self.tolerance_rot > 0:
            raise ValueError("tolerance_rot must be > 0")
        self.target = talos.Target(node, PositionTarget)
        self.state = node.publish(PositionState, "state")

    def update(self):
        t = self.target.get()
        feedback = self.motor.connected and self.angle.connected
        drive = t is not None and feedback and math.isfinite(t.position_rot)
        if drive:
            self.motor.set_motion_magic(t.position_rot)
        goal_rot = t.position_rot if drive else self.angle.position
        self.state.send(position_rot=self.angle.position, velocity_rps=self.angle.velocity,
                        goal_rot=goal_rot, enabled=drive,
                        at_goal=drive and abs(self.angle.position - goal_rot) <= self.tolerance_rot,
                        connected=feedback)
```

The wrist's motor closes its loop on the `angle` CANcoder:
`[motors.motor]` sets `feedback = "RemoteCANcoder"`, `feedback_sensor =
"angle"` and `position_frame = "absolute"`. On rotor feedback its
motion-magic goals stay neutral until the node homes and rezeroes it
([position frames](NODE_DEVELOPMENT_GUIDE.md#rezero-and-position-frames)).

For target selection and mode-generation semantics, see
[Architecture: Targets](../ARCHITECTURE.md#targets).

## 3. Inputs

There are exactly two kinds of input, and they are treated differently.

| Kind | Topic | Missing or stale means | Read with |
|---|---|---|---|
| **Target**: what someone asks this node to do | `/<me>/target/<source>` lanes, one per producer; priority per Driver Station mode from `[robot.targets]` / the subsystem's `[targets]` | do nothing: neutral (also when the lane was sent under another mode generation) | `talos::Target<M>::get()` (null: neutral) |
| **Observation**: what another node reports | `/<other>/state`, `/<other>/status`, `/hw/...` | the decision that needs it is not made; publish your state with the matching validity flag cleared | `fresh()` plus that message's own validity flags |

- Never dereference a subscription without checking `fresh()` in the same
  expression. `value()` before the first message returns a zero-initialized
  `M` in C++ and `None` in Python. **[review]**
- Never hold a last value past its freshness ("keep the last goal"). A
  mechanism that must hold position is told to hold by a producer that keeps
  publishing. **[review]**
- Use a waking message callback (`subscribe(topic, fn)`) only when the node
  must react as each message arrives rather than at its next tick: it makes
  every publication on that topic wake the node (§10). A node that needs every
  message but acts at its tick opens the callback with `Wake::kAtDrain`, which
  wakes nothing ([when messages arrive](NODE_DEVELOPMENT_GUIDE.md#when-messages-arrive)).
  **[review]**
- Read another subsystem only through its messages and topics, or a configured
  value it owns through `param_of` in the constructor. Never include
  its headers beyond `<pkg>/*_generated.h` and `<pkg>/topics.h`. **[enforced:
  `tools/check_layers.py` refuses a Bazel dependency on another node package's
  library]**

## 4. Outputs

- Publish `state` **every tick**, including while disabled, with validity flags
  (`connected`, `enabled`, `*_valid`). A node that goes silent looks dead to its
  consumers, and they cannot tell why. **[review]**
- Never publish NaN or infinity. Publish a validity flag and a finite
  placeholder instead. **[review]**
- Follow the [one-writer and target-lane contract](../ARCHITECTURE.md#targets).
- Something that happened once (a shot fired, a piece acquired) goes on
  `/<me>/event`, not as a pulse in `state`. **[review]**

## 5. Actuation

1. Every actuator call sits inside one `drive` condition that includes
   `robot_enabled(node)`, target freshness, feedback validity and finite
   demands. **[review; the devices layer also gates every output on robot_enabled and a live Target, REQ-SAFE-41]**
2. Neutral is reached by **not** calling the actuator. Call `stop()` only to
   override a set made earlier in the same tick. **[review]**
3. Actuator setters apply for one flush. Unset motors default to neutral;
   the bridge replaces omitted digital outputs with `false` and PWM outputs
   with off (no pulse). Disable gates explicitly set outputs too. A PWM
   output set to 0 pulses at its 1.5 ms centre, which holds a PWM motor
   controller at neutral but drives a servo to centre; leave it unset (or
   `stop()` it) for no pulse. **[enforced; hardware qualification]**
4. Never write `/hw/request/<node>` and never send hardware requests outside
   `update()`. **[enforced: `publish`/`publish_bytes` and `NodeDetail::publisher_send` refuse `/hw/request`, graph lint, Bazel visibility + `implementation_deps`, and `tools/check_layers.py`]**
5. Closed loops run on the Talon (`set_velocity`, `set_position`,
   `set_motion_magic`). Run a loop in the node only when it needs data the Talon
   does not have, such as pose or vision. **[review]**

## 6. Units

Every numeric field in a message and every numeric parameter ends in a unit
suffix from this table. The suffix **is** the unit: C++, Python, telemetry and
Studio all read it from the name. There is no second unit table anywhere: in
code the table is `talOS/schema/unit_suffix.h`.

| Quantity | Suffix | Unit | Used for |
|---|---|---|---|
| timestamp | `_ns` | ns, talos monotonic clock (`node.now().nanos()`; Python `node.now_ns`), `uint64` | when a sample was taken |
| duration | `_us` | µs, integer | periods, timeouts, ages, in TOML and messages |
| length | `_m`, `_mps`, `_mps2` | m, m/s, m/s² | field and chassis |
| image distance | `_px` | pixels | image corners, reprojection noise, detector size gates |
| field-frame angle | `_rad`, `_radps`, `_radps2` | rad, CCW+ | heading, pose yaw, chassis ω |
| shaft or joint angle | `_rot`, `_rps`, `_rps2` | rotations of the **mechanism**, after `sensor_to_mechanism_ratio` | motors, CANcoders, wrists, flywheels |
| electrical | `_v`, `_a`, `_w` | V, A, W | |
| temperature | `_degc` | °C | |
| mass, force, torque | `_kg`, `_n`, `_nm` | | |
| dimensionless | `_frac` (0..1), `_duty` (−1..1), `_ratio`, `_count`, `_index`, `_id`, `_mask` | | |
| boolean | no suffix: a predicate (`enabled`, `at_goal`, `has_piece`, `yaw_valid`) | | |
| enum | no suffix: a FlatBuffers enum type | | |

Rules:

- **One quantity, one field.** A struct never carries the same quantity twice
  in two units (`DrivetrainState` carries the chassis rate once, as
  `omega_radps`). **[review]**
- **Conversions happen at a named boundary, once.** The device layer speaks
  shaft units (`_rot`). A node that turns a shaft angle into a field-frame
  angle (a gyro yaw into a heading) converts there and publishes `_rad`. In
  C++ math, use plain `double` with unit-suffixed names and
  `talos::geometry` types for frame transforms (§13); in Python, `math.tau`.
  talos adds no units library of its own. **[review]**
- **TalonFX gains** (`slot0..2`) are in Phoenix units per **mechanism** unit,
  because `sensor_to_mechanism_ratio` is applied: `v` is volts per mechanism
  rot/s, so `v ≈ 12 / free_speed_mechanism_rps`. Rotor-side gains on a geared
  mechanism are wrong by the gear ratio. **[review]**
- In TOML, durations are `_us`. Python's API takes seconds as `float` for
  `every()` and `period`, and C++ takes `std::chrono`. That is an API
  difference, not a data unit. **[review]**
- Message field names are checked against the table at build time: the
  `talos_messages` action fails on a numeric struct field (or array of
  numbers) without a suffix, naming it. A suffix needs a name before it
  (`count` alone fails; `tick_count` passes). **[enforced]** Numeric
  parameters without a suffix get a `--describe-only` warning. **[enforced (warning)]**
- Struct field names are lower `snake_case`, with no `_<digit>` segment
  (`module0`, not `module_0`) and no interior one-letter word before a unit
  suffix (`x_std_m`, not `std_x_m`; leading one-letter names such as `x_m`
  stay valid). flatc camel-cases field names for Python, and talos derives the
  snake_case names back from them, so `std_x_m` would read as `std_xm` there.
  **[enforced: `talOS/build/descriptor_gen.cc` lint]**

## 7. Messages

- Look in `talOS/msgs` (`geometry`, `control`) before writing a new struct.
  Reuse beats a near-duplicate. **[review]**
- Declare messages with `talos_messages`; see
  [Architecture: Topics and messages](../ARCHITECTURE.md#topics-and-messages).
- A measurement message carries `sample_time_ns`: when the measurement was
  taken, in the node clock, not when it was published (`DrivetrainState`,
  `geometry.PoseObservation`, `geometry.PoseEstimate`). A node reading
  hardware stamps it with `HardwareLink::sample_time()`
  (`talos.hardware_sample_time_ns(node)` in Python), the gateway's sample
  time mapped into the node clock, with the uncertainty described in
  [Data flow: Gateway clock accuracy](DATA_FLOW.md#gateway-clock-accuracy).
  The time is unknown until the mapping converges. Publish a measurement
  without a known time with its validity flags false. **[review]**
- A message carries validity next to the value it qualifies (`yaw_valid` next
  to `yaw_rad`), not one flag for the whole struct. **[review]**
- Enums for states and reasons (`Inhibit`), with the zero value meaning
  "unknown/none". **[review]**

## 8. Parameters

- Use `node.param<T>("key")` **without a fallback** for anything specific to the
  robot (tolerances, setpoints, geometry). A missing key then fails at
  construction. Fallbacks are only for defaults that are right on every robot.
  **[review]**
- Validate at construction: finite, in range, consistent. Throw with the key
  name. **[review]**
- Lists are TOML arrays: `node.param<std::vector<std::string>>("cameras")` /
  `std::vector<double>` in C++, `node.param("cameras")` (a `list`) in Python.
  An element of the wrong type fails at construction naming the key and the
  element. **[enforced]** by the param API.
- Parameter keys follow §6 (`tolerance_rot`, `deadband_frac`,
  `heading_timeout_us`). Numeric keys without a suffix produce a
  `--describe-only` warning. **[enforced (warning)]**
- Robot-wide wiring belongs in
  [configuration](../ARCHITECTURE.md#configuration), not subsystem lists in code.

## 9. Time, determinism, restart

- Use `node.now()` only. Never `steady_clock`, `time.time()` or sleeps. This is
  what makes `Sim` and replay match the robot. **[enforced:
  `tools/lint_test_time.py` over the node packages under `bench/`, `examples/`
  and `tools/scaffold_testdata/` and the framework nodes, `talOS/teleop` and
  `talOS/telemetry`]**
- Estimators integrate on measurement sample times (`sample_time_ns`), never
  on arrival or publish time, and call wpimath's time-taking entry points
  (`UpdateWithTime`/`updateWithTime`), never `Update()`/`update()`, which read
  WPILib's own clock. **[review]**
- No randomness in decisions. Telemetry stream ids are the one exception.
  **[enforced (except `std::random_device`, which telemetry's `RandomStreamId`
  uses): `tools/lint_test_time.py` over the node packages under `bench/`,
  `examples/` and `tools/scaffold_testdata/` and the framework nodes,
  `talOS/teleop` and `talOS/telemetry`]**
- **Tune offline on recorded inputs.** A node's output is a function of its
  inputs and their times, so a new or changed node (an estimator, a vision
  filter) is tuned against a recorded match with
  `launcher --config <robot.toml> --replay-inputs <session dir> --only <node>`:
  every topic it subscribes to that the session logged arrives at its recorded
  time, its outputs go to a new log, and two runs write identical logs. A topic
  recorded with another schema fails naming both types; topics nothing
  recorded are listed (`talOS/launcher/README.md`). Inputs arrive only up to
  the first log gap or damaged stretch in a log that feeds the node: the run
  ends there, names the log, and exits 1; a feeding log that is capped, a
  run's log without its EXIT of a member the session manifest marks
  `log_gap` or `recorder_failed`, or one whose damage nothing bounds is refused before the node
  runs; another reader's damage is no loss only where an intact receipt
  follows it and the session's other logs hold what it lost
  ([rule](../talOS/events/README.md#replaying-another-sessions-inputs)).
  **[enforced:
  `input_replay_test`, `replay_inputs_test`]**
- **Restart safety.** A node can be restarted mid-match (`max_restarts`). Its
  initial state must be safe with every input present: an actuator node
  drives only on a lane `Target::get` accepts, judged by its publication
  time even on the node's first drain, and an operator producer starts
  disarmed (REQ-SAFE-31). A robot's own restart rules for its nodes (an
  autonomous routine's progress across restarts, for one) are in its
  repository. **[enforced: `//talOS/devices:target_test`,
  `//talOS/teleop:node_test`]**
- Config errors throw in the constructor, so the launcher reports them before
  the robot is enabled. **[review]**

## 10. Performance


| Rule | Budget | Status |
|---|---|---|
| `update()` p99 wall time | ≤ 10 % of `period_us` | **[enforced]** on representative bench and framework C++ nodes (`motor`, `swerve_module`, `teleop`, `telemetry`): `//bench/main_processor:update_p99_perf_test` (`REQ-RT-07`, `perf` lane) measures `update()` + devices-flush entry-to-exit wall-clock p99 with `talos::event::steady_now()` and host qualification (`StallMeter`, failing distinctly as `[HOST_UNQUALIFIED]` on a stalled host); **[review]** for other nodes; the 10 % budget's end-to-end derivation remains open (`TODO.md` `SPEC-4`). Registry / Studio `max_latency` measures maximum dispatch-start lateness, not callback duration or p99. Benchmark clocks must not replace `node.now()` in control logic. |
| Allocations, locks, file or socket I/O, logging, `print` in `update()` | none (allocate in the constructor, reuse members); one exception, inside talOS: a publish may send one 1-byte wake datagram to each reader armed on that topic, non-blocking, and connects that reader's endpoint on first use or when the reader's receiver changes ([data flow](DATA_FLOW.md#link-rates-and-endpoints)) | allocations [enforced: `talos::testing::AllocationProbe` (`//talOS/node:allocation_probe`) counts the loop thread's allocations over a window of steady-state ticks in the bench `motor` and `swerve_module`, `talOS/teleop`, `talOS/telemetry` and the examples' C++ node tests and the `talOS/mechanisms` blocks' tests, and over the bridge's `Tick()` on a live gateway link in `talOS/bridge/hardware_node_test.cc` (`//talOS/bridge:node_test`); macOS and glibc, skipped under sanitizers ([TESTING.md §7](TESTING.md#7-the-node-test-matrix))]; direct blocking I/O, logging, `print`, and locks in node packages [enforced: `tools/lint_test_time.py` `io` and `lock` kinds over the node packages under `bench/`, `examples/`, `tools/scaffold_testdata/`, `talOS/teleop`, and `talOS/telemetry`, with construction-only uses tracked in `tools/lint_test_time_allowlist.txt`]; indirect I/O or locks through helper libraries or unenumerated syscalls and Python allocations [review] |
| Python nodes | cost memory (≈ 30 MB) and add GC/interpreter jitter: a Python node whose own subsystem owns a motor, digital output or PWM output runs at `period_us` ≥ 20000, so hard 5 ms actuator loops belong in C++ (a `talos.cpp_node` counts as C++); 20 ms supervisory/estimator nodes monitor age and watchdog timeouts | the actuator period [enforced: the launcher's graph lint, `--describe-only` and `talos_graph_test` ([launcher](../talOS/launcher/README.md#python-actuator-periods))]; the rest [review] |
| Hops on a control path | each timer-drained hop adds up to one consumer period; keep driver input → actuator to ≤ 2 node hops | [enforced: the launcher's graph lint warns on an operator producer → actuator owner path longer than two node hops ([NAMING.md](../talOS/NAMING.md#topics), `REQ-CONTRACT-09`, `REQ-RT-16`)]; producer → consumer is one hop (`Target` resolves lanes in the consumer) |
| Per-tick work | O(devices), never O(history); bound every loop | [review] |

For timer-drained and callback delivery, see
[Architecture: The tick and Targets](../ARCHITECTURE.md#the-tick).
Prefer timer-drained inputs unless prompt callback reaction is required.

## 11. Testing

The node test matrix and the rest of the testing method: [TESTING.md](TESTING.md).

## 12. Adding things

| Adding | Do | Never |
|---|---|---|
| A mechanism | `bazel run @talos//tools:new_subsystem -- <name> --lang cpp\|py` from the robot repository's root ([first mechanism](FIRST_MECHANISM.md#3-add-a-mechanism)); fill in the TOML; write the [node test matrix](TESTING.md#7-the-node-test-matrix) | edit `operator_interface` or `telemetry` code |
| A producer of a target (auto routine, auto-align, operator input) | a `talos::TargetLane` on `/<sub>/target/<source>`, opened with a `talos::OperatorArming` for operator input; list `<source>` in `[robot.targets]` or that subsystem's `[targets]` (the launcher refuses a lane no mode lists) | subscribe to another producer's lane, or write a lane someone else writes |
| A process without hardware (pose estimator, vision, auto) | a node with no devices; subscribe to `state` topics; publish state with validity while disabled too; [estimator walkthrough](NODE_DEVELOPMENT_GUIDE.md#adding-a-pose-estimator) | duplicate shared geometry or hand-roll a Kalman filter |
| A message | the owning package's `.fbs` and `talos_messages(topics = ...)`; §6 and §7 | a hand-written C++/Python mirror |
| A device type | `talOS/hardware`, `talOS/devices`, `devices.py`, the RoboRIO program, in one change | |
| A node-API feature | `talOS/node` (or `devices`), `talOS/node_api`, `talOS/python/talos`, tests in both languages, in one change | a Python-only reimplementation |

## 13. Open source first

Before writing a library, a format or an algorithm, use a maintained
open-source one if it fits. talos code is the glue the robot needs: nodes,
topics, safety gating and wiring. It is not a place to reimplement math,
serialization or tooling. **[review]**

A dependency is adopted when it meets all of these:

1. It has an OSI license compatible with the repo (BSD, MIT, Apache-2.0, MPL-2.0).
2. It is maintained, and it builds on every target platform: macOS arm64 (the
   Mini and laptops), Linux x86_64/arm64 (CI) and, if it runs there, the RoboRIO.
3. Bazel consumes it through the Bazel Central Registry (`bazel_dep`), an
   upstream `MODULE.bazel` (`archive_override`/`git_override`), the pinned
   pip lock (`//talOS/python:requirements.txt`, every distribution hashed),
   or, when upstream publishes no Bazel module, its own sha256-pinned release
   artifacts fetched by a module extension (`third_party/<name>`). It is
   never vendored by copy unless upstream has no build we can use.
4. It stays off the hot path unless it meets §10 (no allocation per tick in
   `update()`).
5. It is added in one change, with the reason, to the table below, and a
   `bazel_dep` also to the "Bazel module" table after it.

| Need | Use | Status |
|---|---|---|
| Serialization, schemas, reflection | FlatBuffers (structs, and `reflection.fbs`/`.bfbs` for runtime schemas) | in use: `.bfbs` descriptors, schema hashes, hash-checked attach, descriptor-bearing logs and reflection codec |
| Config | TOML: toml++ (C++), `tomllib` (Python stdlib) | in use |
| Tests | googletest, `unittest` | in use |
| Geometry, kinematics and pose estimation | C++: header-only `talos::geometry` SO2/SE2/SO3/SE3 on Eigen 3.4 (`@eigen3`); drivetrain inverse/forward swerve kinematics use it with per-module speed limits. Python: `robotpy-wpimath` 2026.2.1.1 | in use: shared C++ frame math. `robotpy-wpimath` is pinned in `talOS/python/requirements.txt` for a robot's Python nodes, and no talOS node imports it (the 2026 robot's pose estimator, in its own repository, does); `wpilib_lockstep_test` aligns GradleRIO with the RobotPy native/wrapper wheels only |
| Smoothing and mapping (factor graphs) | GTSAM 4.3.0 (`gtsam` wheel on PyPI, BSD) | pinned in `talOS/python/requirements.txt` for a robot's Python nodes; no talOS node imports it (the 2026 robot's pose estimator, in its own repository, does) |
| Vision: AprilTag pose observations | `talOS/vision`: upstream AprilTag 3.4.5 (`@apriltag`), OpenCV 4.12 conversion/IO, Metal preprocessing/CCL on macOS, AVFoundation/V4L2 capture | in use: typed detector options, decode-margin/Hamming validity gates before localization, undistorted homography, robust joint single/multi-tag corner solving with floor/tilt priors, marginalized field-frame x/y/yaw covariance and configured uncertainty floors; ambiguity unknown=1, verified unique=0; one robot node per camera, no side-channel server |
| Autonomous paths | Choreo `.traj` files (JSON, read with nlohmann/json 3.12.0 from the BCR, MIT), loaded and followed by `talOS/trajectory` | in use by the 2026 robot's autonomous node (its own repository), which plans with `MakeStraightLine` and follows with `HolonomicFollower`. wpimath's C++ controllers have no Bazel module, so the follower is talos code |
| RoboRIO hardware | WPILib HAL and Phoenix 6 | in use |
| Rigid-body simulation of a robot and field | MuJoCo 3.17.0: MuJoCo's library target `@mujoco//:mujoco` built from source, with root-only patches for two uninitialised, never-read flags and `memcpy`/`memset` of NULL, empty arrays on a model with no degrees of freedom (`third_party/mujoco_*.patch`; a dependent builds it unpatched, [INSTALL](INSTALL.md)) (no Python bindings or viewer; it also compiles and links the classic OpenGL renderer and UI code, unused, with no GLFW and no link-time GL). It links MuJoCo (Apache-2.0), ccd (BSD-3-Clause), miniz (MIT), tinyxml2 and lodepng (Zlib), tinyobjloader (MIT), qhull (Qhull: permissive, not OSI-approved) and marchingcubecpp (public domain, no licence file). Qhull and marchingcubecpp are exceptions to condition 1, the owner's to accept | used only under `//talOS/simulation/mujoco` and its subpackages, which `tools/check_layers.py --source-only` enforces; never linked into a node or the RoboRIO program |

Every `bazel_dep` of the root `MODULE.bazel` is a row here
(`tools/check_deps_listed.py`, a source lint, fails on a module missing from
either). A module no BUILD or `.bzl` file of this repository loads is a
root pin: it sets the lowest version the module graph may resolve for
the dependencies that use it (Bzlmod picks the highest version any module
asks for; only an override, such as `single_version_override`, fixes one).

| Bazel module | For |
|---|---|
| `apple_support` | the Apple C++ toolchain on macOS under Bazel 8.6 / Xcode (`repo_name` `build_bazel_apple_support`; supplies the configured macOS C++ toolchain targets instead of transitive 1.24.2); root pin |
| `rules_cc` | C++ rules (0.2.22, the version `mujoco` asks for) |
| `googletest` | C++ tests |
| `platforms` | OS constraints (`@platforms//os:*`) in `select`s |
| `flatbuffers` | message schemas, `flatc`, the C++ and Python runtimes |
| `aspect_bazel_lib` | `copy_to_bin` and build stamping |
| `rules_python` | Python rules, the 3.11 toolchain and the pinned pip lock |
| `nlohmann_json` | Choreo `.traj` and session manifest reading |
| `opencv` | `talOS/vision` image conversion and IO (patched, root only) |
| `mujoco` | rigid-body physics under `//talOS/simulation/mujoco` |
| `wrist` | `examples/wrist_repo`, the standalone-subsystem example (dev only) |
| `hedron_compile_commands` | `compile_commands.json` for clang-tidy (dev only) |
| `aspect_rules_js` | Studio's npm dependency graph (dev only) |
| `rules_nodejs` | Studio's Node runtime (dev only) |
