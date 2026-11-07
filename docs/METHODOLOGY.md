# Engineering methodology

How talos code is written, so that every subsystem is robust, reliable,
performant and easy to work with in the same way. `.omp/RULES.md` holds the
hard rules; this page holds the method behind them. API reference:
[NODE_DEVELOPMENT_GUIDE.md](NODE_DEVELOPMENT_GUIDE.md). Planned framework
changes this page refers to: [DESIGN_AUTOMATIC_WIRING.md](DESIGN_AUTOMATIC_WIRING.md).

Each rule carries its status:

| Mark | Meaning |
|---|---|
| **[enforced]** | the build, the graph lint or a test fails when it is broken |
| **[review]** | a convention; reviewers (`WATCHDOG.md`) check it |
| **[planned: Dn]** | needs design item Dn in DESIGN_AUTOMATIC_WIRING.md; follow the current form until then |

## 1. What the four goals mean here

| Goal | Means | Checked by |
|---|---|---|
| Robust | A node never moves a motor on a disabled robot, a stale target, missing feedback or a non-finite number; bad config fails at construction, before anything moves. | §5, §8, the test matrix in §11 |
| Reliable | A node does the same thing on the robot, in `Sim` and in replay; a restart mid-match does nothing unsafe; one dead node stops only what it owns. | §9, §11 |
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

`Target<M>::get()` folds `robot_enabled(node)`, the Driver Station mode and
its generation, the lane priority, release and freshness into one null check,
so target messages carry no `enabled` field. A target sent under an earlier
enable or mode never passes it: each envelope carries the producer's mode
generation (the RoboRIO's `mode_since_us`), and only the consumer's current one
is accepted. **[enforced: `Target`, `target_test`, `talos_test`; the graph lint
checks each lane against the policy]**

## 3. Inputs

There are exactly two kinds of input, and they are treated differently.

| Kind | Topic | Missing or stale means | Read with |
|---|---|---|---|
| **Target**: what someone asks this node to do | `/<me>/target/<source>` lanes, one per producer; priority per Driver Station mode from `[robot.targets]` / the subsystem's `[targets]` | do nothing: neutral (also when the lane was sent under another mode generation) | `talos::Target<M>::get()` (null: neutral) |
| **Observation**: what another node reports | `/<other>/state`, `/<other>/status`, `/hw/...` | the decision that needs it is not made; publish your state with the matching validity flag cleared | `fresh()` plus that message's own validity flags |

- Never dereference a subscription without checking `fresh()` in the same
  expression. `value()` before the first message is undefined in C++ and
  `None` in Python. **[review]**
- Never hold a last value past its freshness ("keep the last goal"). A
  mechanism that must hold position is told to hold by a producer that keeps
  publishing. **[review]**
- Use a message callback (`subscribe(topic, fn)`) only when the node must react
  within 1 ms. A callback switches the node to 1 ms polling (§10). **[review]**
- Read another subsystem only through its messages and topics. Never include
  its headers beyond `<pkg>/*_generated.h` and `<pkg>/topics.h`. **[enforced: Bazel visibility, review]**

## 4. Outputs

- Publish `state` **every tick**, including while disabled, with validity flags
  (`connected`, `enabled`, `*_valid`). A node that goes silent looks dead to its
  consumers, and they cannot tell why. **[review]**
- Never publish NaN or infinity. Publish a validity flag and a finite
  placeholder instead. **[review]**
- One writer per topic. Several producers of one target each write their own
  `/<sub>/target/<source>` lane with `talos::TargetLane`, republished every
  period; a producer with nothing to say calls `release()` rather than
  sending zeros. **[enforced: graph lint, including a lane no policy mode
  lists]**
- Something that happened once (a shot fired, a piece acquired) goes on
  `/<me>/event`, not as a pulse in `state`. **[review]**

## 5. Actuation

1. Every actuator call sits inside one `drive` condition that includes
   `robot_enabled(node)`, target freshness, feedback validity and finite
   demands. **[review; the devices flush backstops motors only]**
2. Neutral is reached by **not** calling the actuator. Call `stop()` only to
   override a set made earlier in the same tick. **[review]**
3. Digital and PWM outputs are **not** neutralized by the flush: they keep their
   last value until the robot is disabled. Set them every tick inside `drive`,
   and set the safe value explicitly otherwise. **[review; framework fix tracked in TODO.md]**
4. Never write `/hw/request/<node>` and never send hardware requests outside
   `update()`. **[enforced: graph lint, one writer]**
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
| timestamp | `_ns` | ns, talos monotonic clock (`node.now()`), `uint64` | when a sample was taken |
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
  `omega_radps`). **[review; lint planned: D2]**
- **Conversions happen at a named boundary, once.** The device layer speaks
  shaft units (`_rot`). A node that turns a shaft angle into a field-frame
  angle (a gyro yaw into a heading) converts there and publishes `_rad`. In
  C++ math, use plain `double` with unit-suffixed names and
  `talos::geometry` types for frame transforms (§14); in Python, `math.tau`.
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
- No one-letter word before the suffix: `x_std_m`, not `std_x_m`, and
  `module0`, not `module_0`. flatc camel-cases field names for Python, and
  talos derives the snake_case names back from them, so `std_x_m` would read
  as `std_xm` there. **[review]**

## 7. Messages

- Look in `talOS/msgs` (`geometry`, `control`) before writing a new struct.
  Reuse beats a near-duplicate. **[review]**
- Messages are FlatBuffers structs owned by one package, declared with
  `talos_messages`. Never mirror a struct by hand (the earlier `GenericState`
  and `GenericTarget` side doors have been deleted; D3 and D4 replaced
  hard-coded subsystem lists with framework nodes). Each struct has a build-time
  schema descriptor (`talos::Descriptor<M>`, `<Struct>.TALOS_SCHEMA_HASH`).
  Attaching to a topic checks schema name and hash; a mismatch fails, naming
  both types. **[enforced]**
- A measurement message carries `sample_time_ns`: when the measurement was
  taken, in the node clock, not when it was published (`DrivetrainState`,
  `geometry.PoseObservation`, `geometry.PoseEstimate`). A node reading
  hardware stamps it with `HardwareLink::sample_time()`
  (`talos.hardware_sample_time_ns(node)` in Python), the gateway's sample
  time mapped through the bridge's clock offset; that is an upper-bounded
  latency estimate, never earlier than the true sample, and unknown until
  the mapping converges. A measurement without a known time is published
  with its validity flags false. **[review]**
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
- Robot-wide wiring (which producers drive which subsystem, telemetry,
  bindings) lives in TOML read by the framework. It is never a list of
  subsystem names in code. Target lanes and telemetry are config-only
  (`[robot.targets]`, `[targets]`, `[studio]`); teleop bindings are
  **[done: D4]**.

## 9. Time, determinism, restart

- Use `node.now()` only. Never `steady_clock`, `time.time()` or sleeps. This is
  what makes `Sim` and replay match the robot. **[review]**
- Estimators integrate on measurement sample times (`sample_time_ns`), never
  on arrival or publish time, and call wpimath's time-taking entry points
  (`UpdateWithTime`/`updateWithTime`), never `Update()`/`update()`, which read
  WPILib's own clock. **[review]**
- No randomness in decisions. Telemetry stream ids are the one exception. **[review]**
- **Tune offline on recorded inputs.** A node's output is a function of its
  inputs and their times, so a new or changed node (an estimator, a vision
  filter) is tuned against a recorded match with
  `launcher --config <robot.toml> --replay-inputs <session dir> --only <node>`:
  every topic it subscribes to that the session logged arrives at its recorded
  time, its outputs go to a new log, and two runs write identical logs. A topic
  recorded with another schema fails naming both types; topics nothing
  recorded are listed (`talOS/launcher/README.md`). **[enforced:
  `input_replay_test`, `replay_inputs_test`]**
- **Restart safety.** A node can be restarted mid-match (`max_restarts`). Its
  initial state must be safe with every input present. One-shot autonomous
  starts `INHIBITED` and arms only after this process observes a fresh disabled
  or non-autonomous mode. Same-generation Driver Station outages release lanes
  without restarting routine progress; new generations reset the routine.
  Shooter readiness continuously gates feeding, whose accumulated duration
  excludes released intervals. The preload deadline runs from the generation's
  first `SPINUP` entry, including feed and readiness retries; `DRIVE` deadlines
  include all outages. Autonomous field driving requires a fresh, valid,
  localized pose; pose outages pause trajectory progress but not the deadline.
  **[enforced: `autonomous` `node_test`]**
- Config errors throw in the constructor, so the launcher reports them before
  the robot is enabled. **[review]**

## 10. Performance

Measured on this stack (macOS, realtime scheduling, all cores loaded): timer
lateness p99 ≈ 0.6 ms for C++ 5 ms nodes and 1.0 ms for the Python 20 ms
intake; the whole graph used ≈ 4.5 % of one core.

| Rule | Budget | Status |
|---|---|---|
| `update()` p99 wall time | ≤ 10 % of `period_us` | [review]; read `max_latency` in the registry / Studio system view |
| Allocations, locks, file or socket I/O, logging, `print` in `update()` | none (allocate in the constructor, reuse members) | [review]; the drivetrain's `UpdateDoesNotAllocate` counts process-wide allocations (macOS only; §13) |
| Python nodes | cost memory (≈ 30 MB) and add GC/interpreter jitter: hard 5 ms actuator loops belong in C++, while 20 ms supervisory/estimator nodes monitor age and watchdog timeouts | [review] |
| Hops on a control path | each timer-drained hop adds up to one consumer period; keep driver input → actuator to ≤ 2 node hops | [review]; producer → consumer is one hop (`Target` resolves lanes in the consumer) |
| Per-tick work | O(devices), never O(history); bound every loop | [review] |

How a node receives messages decides its latency, regardless of language
(`ARCHITECTURE.md`, "The tick"):

- A node without callbacks sees a message at its next tick, up to one period
  late, and sleeps otherwise.
- A node with a callback polls every 1 ms.

Pick the first unless 1 ms reaction is a requirement.

## 11. Testing

Every node gets a `Sim`/`SimHardware` (C++) or `talos.testing` (Python) test
with **all** of these cases **[review; the scaffold generates the skeleton]**:

| Case | Expect |
|---|---|
| Happy path | the right mode and demand on each actuator; `state` reflects it |
| Robot disabled (`hw.enable(false)`) | every actuator neutral, `state.enabled == false`, `state` still published |
| Target stale (stop publishing; step past `command_timeout_us`) | neutral within one period after the timeout |
| Target released (`ReleaseTarget` / `sim.release_target`) | neutral (or the next lane in priority) on the next tick |
| Target never sent | neutral; no exception |
| Driver Station lost (`hw.stop_driver_station()`) | neutral once the DS is stale |
| Feedback disconnected (`hw.set_connected(node, dev, false)`) | neutral; validity flag cleared |
| Non-finite demand | neutral; no NaN in `state` |
| Bad parameter | the constructor throws, naming the key |
| Source switch (a node with several lanes) | the old mode's lane is ignored after a mode change |
| Target from an earlier mode generation (`PublishTarget(..., message, before)` / `sim.publish_target(..., mode_since_us=before)` after a re-enable) | neutral, even when it lands in the same drain as the re-enabling packet |

Publish targets with `talos::testing::PublishTarget` / `sim.publish_target`,
with `hw.enable()` and a teleop-enabled Driver Station
(`hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop))` /
`hw.set_driver_station(enabled=True, teleop=True)`). They stamp the mode
generation of the newest Driver Station packet
(`DriverStationGeneration(sim)` / `sim.driver_station_generation`), as a
producer that has drained it would.

- Test behaviour through topics and `SimHardware`. Never test private methods
  or log text.
- A config change runs `talos_graph_test` (CI) and launcher `--describe-only`
  **[enforced]**.
- A C ABI change keeps `node_api_test` and `talos_test` exercising the new
  symbol from both languages. `HardwareLink::fresh()` and `age()` are exposed
  in both languages; `//talOS/python:api_parity_test` enforces matching ABI
  versions, exports and parameter counts between `node_api.h` and `_abi.py`.

## 12. Adding things

| Adding | Do | Never |
|---|---|---|
| A mechanism | `bazel run //tools:new_subsystem -- <name> --lang cpp\|py`; fill in the TOML; write the §11 tests | edit `operator_interface` or `telemetry` code |
| A producer of a target (auto routine, auto-align) | a `talos::TargetLane` on `/<sub>/target/<source>`; list `<source>` in `[robot.targets]` or that subsystem's `[targets]` (the launcher refuses a lane no mode lists) | subscribe to another producer's lane, or write a lane someone else writes |
| A process without hardware (pose estimator, vision, auto) | a node with no devices; subscribe to `state` topics; publish `/<me>/state` every tick with validity, while disabled too (no actuation gate on computation); reuse `talos::geometry`/`talOS/vision` in C++, RobotPy or GTSAM in Python (§14); `bazel run //tools:new_subsystem -- <name> --type estimator`; walkthrough in DESIGN_AUTOMATIC_WIRING.md §D6 | read `/hw/state` directly; duplicate shared geometry or hand-roll a Kalman filter |
| A message | the owning package's `.fbs` and `talos_messages(topics = ...)`; §6 and §7 | a hand-written C++/Python mirror |
| A device type | `talOS/hardware`, `talOS/devices`, `devices.py`, the RoboRIO program, in one change | |
| A node-API feature | `talOS/node` (or `devices`), `talOS/node_api`, `talOS/python/talos`, tests in both languages, in one change | a Python-only reimplementation |

## 13. Current deviations

These are the places where the code breaks this page today. Fix them when you
touch the code, or under the design item named.

| Where | Deviation | Rule | Fix |
|---|---|---|---|
| `driver.h` | chassis-only teleop driving; mechanism bindings moved to `[[teleop.bindings]]` | §7, §8 | done: D4 |
| `driver.toml` | `deadband_frac` | §6, §8 | done: D4 |
| `intake.py:18` | fixed `INTAKE_RPS` ignores the requested speed | §3 | use the target's demand |
| `drivetrain/node_test.cc` `UpdateDoesNotAllocate` | the process-wide allocation check (`malloc_logger`) runs on macOS only; on Linux it is skipped | §10 | a glibc-compatible allocation hook (glibc removed `__malloc_hook` in 2.34) |
| `2026-robot/main_processor/vision/vision.h` simulation path | render/detect/solve runs synchronously in `update()`, allocating inside rasterization/detection/PnP within a configured 10 ms node period; this sim-only cost preserves deterministic replay (live work is off-thread) | §10 | reusable allocation-free detector/solver workspaces and deterministic simulated compute scheduling; fast-projection tag/detection buffers and the raster image are already reused |

## 14. Open source first

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
5. It is added in one change, with the reason, to this table.

| Need | Use | Status |
|---|---|---|
| Serialization, schemas, reflection | FlatBuffers (structs, and `reflection.fbs`/`.bfbs` for runtime schemas) | in use: `.bfbs` descriptors, schema hashes, hash-checked attach, descriptor-bearing logs and reflection codec |
| Config | TOML: toml++ (C++), `tomllib` (Python stdlib) | in use |
| Tests | googletest, `unittest` | in use |
| Geometry, kinematics and pose estimation | C++: header-only `talos::geometry` SO2/SE2/SO3/SE3 on Eigen 3.4 (`@eigen3`); drivetrain inverse/forward swerve kinematics use it with per-module speed limits. Python: `robotpy-wpimath` 2026.2.1.1 | in use: shared C++ frame math and the Python pose estimator's `kalman` backend; `wpilib_lockstep_test` aligns GradleRIO with the RobotPy native/wrapper wheels only |
| Smoothing and mapping (factor graphs) | GTSAM 4.3.0 (`gtsam` wheel on PyPI, BSD) | in use (D6): the pose estimator's `factor_graph` backend (`IncrementalFixedLagSmoother`). Python only: GTSAM has no Bazel module, and its C++ build is CMake with a large dependency surface |
| Vision: AprilTag pose observations | `talOS/vision`: upstream AprilTag 3.4.5 (`@apriltag`), OpenCV 4.12 conversion/IO, Metal preprocessing/CCL on macOS, AVFoundation/V4L2 capture | in use: typed detector options, decode-margin/Hamming validity gates before localization, undistorted homography, robust joint single/multi-tag corner solving with floor/tilt priors, marginalized field-frame x/y/yaw covariance and configured uncertainty floors; ambiguity unknown=1, verified unique=0; one robot node per camera, 100 Hz status and up to 60 fps live observations, no side-channel server |
| Autonomous paths | Choreo or PathPlanner trajectories, followed with wpimath controllers | candidate |
| Log viewing | export `.tlog` to MCAP (Foxglove) or WPILOG (AdvantageScope) rather than building more viewer features | candidate |
| RoboRIO hardware | WPILib HAL and Phoenix 6 | in use |
