# talOS subsystem architecture — implementation plan

**Status: not started.** This describes the target design and the order to build
it in. It is written for an implementing agent who has not seen this repository
before. Read `talOS/events/README.md`, `talOS/hardware/README.md` and
`2026-robot/main_processor/drivetrain/README.md` first; they describe what exists today.

Every step below has an acceptance test. Do not mark a step done without it.

---

## 1. The end state

Adding a subsystem to the robot means: declare its hardware in one TOML file,
write one node containing its control logic, and run. **No RoboRIO code changes,
no protocol changes, no changes to any other node.**

```toml
# robot.toml
[subsystems.shooter]
node = "//2026-robot/main_processor/shooter:node"
period_us = 5000

[subsystems.shooter.motors.flywheel_leader]
type = "TalonFX"
bus = "canivore"
can_id = 9
supply_limit_a = 40
sensor_to_mechanism_ratio = 1.0

[subsystems.shooter.sensors.note_beam_break]
type = "DigitalInput"
dio = 0
```

```cpp
// 2026-robot/main_processor/shooter/node.h — the whole subsystem
template <typename Loop>
class ShooterNode {
 public:
  ShooterNode(Loop& loop, const hardware::Devices& mine)
      : mine_{mine} {
    event::watch<Packet, &ShooterNode::OnState>(loop, kHwStateTopic, this);
    event::watch<ShooterTarget, &ShooterNode::OnTarget>(loop, "/shooter/tgt",
                                                        this);
    request_ = event::make_sender<Packet>(loop, "/hw/req/shooter");
    state_ = event::make_sender<ShooterState>(loop, "/shooter/state");
    timer_ = event::make_timer<&ShooterNode::Tick>(loop, "shooter", this);
  }
  // ... Tick() computes a flywheel velocity request and publishes both
  // /hw/req/shooter (actuator request) and /shooter/state (semantic state).
};
```

A node that needs no hardware — state estimation, for example — declares no
subsystem at all. It subscribes to the *semantic* state topics that subsystem
nodes publish, never to raw hardware:

```cpp
event::watch<DriveState, &Odometry::OnDrive>(loop, "/drivetrain/state", this);
event::watch<ShooterState, &Odometry::OnShooter>(loop, "/shooter/state", this);
```

That split is the point of the whole design. **Subsystem nodes own devices and
translate hardware into meaning. Every other node consumes meaning.** Only
subsystem nodes ever read `/hw/state`.

---

## 2. Architecture

```
robot.toml ── parsed once on the companion, the single source of truth
     │
     │  (config pushed to the RIO at startup, one transaction)
     ▼
┌─────────────────┐        UDP        ┌──────────────────────────────┐
│  hardware_node  │◄─────────────────►│  RoboRIO program (FINAL)     │
│  (companion)    │                   │  generic device server:      │
│                 │                   │  configure, sample, apply,   │
│  • pushes config│                   │  enforce limits + DS enable  │
│  • republishes  │                   └──────────────────────────────┘
│    /hw/state    │
│  • merges       │
│    /hw/req/*    │
└────────┬────────┘
         │ /hw/state (one message, every device on the robot)
         ├──────────────────────┬──────────────────────┐
         ▼                      ▼                      ▼
  drivetrain node         shooter node           (more subsystems)
   /hw/req/drive           /hw/req/shooter
   /drivetrain/state       /shooter/state
         │                      │
         └──────────┬───────────┘
                    ▼
             state estimation node  ── owns no hardware
                 /odometry
```

`hardware_node` replaces today's `2026-robot/main_processor/drivetrain/bridge.cc`. It is the only
process that talks to the RIO and the only publisher of `/hw/cmd`, because the
gateway validates and applies a command as one whole-robot transaction
(`talOS/hardware/gateway.cc:53`). Merging is not arbitration: device ownership
is settled at parse time, and two subsystems claiming one device is a config
error, never a runtime race.

---

## 3. Non-negotiables

### 3.1 Latency

Latency is the primary constraint. These are requirements, not preferences.

- **No virtual functions anywhere in a dispatch path.** Loops are CRTP,
  recorders and metrics are template policies, callbacks are `{object, fn}`
  thunks. `simulated_event_loop_test.cc` asserts `!std::is_polymorphic_v<>` on
  every loop; add the same assertion for anything new on the hot path.
- **No allocation after `run()` starts.** Size every buffer at registration.
- **Nodes are single-threaded.** One dispatch at a time, never re-entrant.
- **Messages are fixed-size flatbuffer structs, never tables.** Transport is a
  `memcpy`; there is no serialization step on the hot path, and replay can
  compare payloads byte for byte.
- **At most two RTMS hops between a sensor reading and an actuator request.**
  Sensor → `hardware_node` → subsystem node → `hardware_node`. A third hop in
  that chain needs justification in review.
- **Set each node's tick period to its control period.** Message delivery is a
  fixed-rate poll (`RealtimeEventLoop::Options::tick_period`), so polling faster
  than you act only burns wakeups, and polling slower adds latency directly.

Budget, to be measured and not assumed:

| Hop | Target |
|---|---|
| RIO loop period | 5 ms (200 Hz), from `robot.toml` |
| `hardware_node` tick | 1 ms |
| Subsystem node tick | its control period |
| Sensor sample → actuator request on the wire | ≤ 2 RIO periods + 1 node period |

Measure with `//talOS/events/tools:loop_perf` and the `LoopMetrics` policy.
Record p50, p99 and max — a mean alone hides exactly the behaviour that matters.
Do not claim a latency number you have not measured.

### 3.2 Determinism

Everything in `talOS/events/README.md` still holds. In particular: handlers read
time only through `loop.monotonic_now()`, every input and output is recorded,
and a recorded run replays byte-identically. Any new node must be replayable —
`node --replay <log>` reporting `diverged=0` is part of each step's acceptance
test, not an optional extra.

### 3.3 Safety

None of these may regress. They are the reason the RIO can be trusted with a
config it did not write:

- The RIO validates a whole config, and a whole command, before touching any
  actuator.
- **The RIO holds hard ceilings that no pushed config can exceed.** A companion
  bug must not be able to over-current a motor. This is new work in step 3 and
  it is the single most important thing in this plan.
- `commissioned` stays a RIO-side gate; commands are refused until it is set.
- The Driver Station enable gate stays. `//2026-robot/main_processor/drivetrain:monitor` shows it,
  and `tools/test_drivetrain.py --wpilib` asserts a disabled robot does not move.
- Command lease expiry still neutralizes; enable transitions still bump `epoch`
  and invalidate outstanding commands.

### 3.4 Frozen

- **RTMS shared-memory layout.** Additive C++ API changes are fine; the on-disk
  and in-shm layout is not.
- The dispatch-log format may change, but only by bumping `FORMAT_VERSION` in
  `talOS/events/log/format.h` and updating the reader's rejection path.

---

## 4. Platform constraints to respect

- **macOS caps POSIX shared-memory names at 31 characters including the leading
  slash** (`PSHMNAMLEN`). Topic names are `shm_open` names. There is currently
  **no guard** for this — a long topic name fails at runtime on macOS and works
  on Linux. Step 0 adds the guard.
  Budget accordingly: `/hw/req/drivetrain` is 18, `/drivetrain/state` is 17.
  Prefer short, abbreviated topic names over descriptive ones.
- `talOS/protocol/frame.h` caps a UDP payload at `kMaxPayloadSize = 1200`
  bytes. A full robot config will not fit in one datagram; step 3 chunks it.
- `MAX_READERS = 8` per RTMS topic (`talOS/rtms/rtms.h`). `/hw/state` will have
  one reader per subsystem node plus the monitor. If a robot needs more than
  eight, that is a real design change, not a constant bump — say so rather than
  raising it silently.

---

## 5. Topic naming

| Topic | Publisher | Payload | Read mode |
|---|---|---|---|
| `/hw/state` | `hardware_node` | `Packet` (encoded `hardware::State`) | LATEST |
| `/hw/cmd` | `hardware_node` | `Packet` (encoded `hardware::Command`) | LATEST |
| `/hw/req/<sub>` | subsystem node `<sub>` | `Packet` (partial `Command`) | LATEST |
| `/<sub>/state` | subsystem node `<sub>` | subsystem-defined struct | SEQUENCE |
| `/<sub>/tgt` | whoever commands it | subsystem-defined struct | LATEST |

Rules:

- **Exactly one publisher per topic.** RTMS is single-producer.
- Hardware topics use LATEST: a stale sample is worthless, and a node must never
  fall behind on them.
- Semantic state topics default to SEQUENCE so a consumer sees every sample.
  Use LATEST only where dropping is genuinely correct, and say why in a comment.
- Every message carries what a consumer needs to judge staleness. Follow
  `ChassisTarget`: an `issued_ns` from the publisher's loop clock and an
  `enabled` flag. Consumers must treat a stale message as absent, not as zero.

---

## 6. Device taxonomy

**Get this complete before step 3.** "The RIO never changes again" holds only for
device kinds it already drives; adding a kind later is a redeploy. This is the
one part of the design that is expensive to get wrong.

Required at minimum:

| Kind | Config keys | Direction |
|---|---|---|
| `TalonFX` | `bus`, `can_id`, limits, ratios, gains, soft limits | actuator + sample |
| `CANcoder` | `bus`, `can_id`, `offset_rot`, `inverted` | sample |
| `Pigeon2` | `bus`, `can_id` | sample |
| `DigitalInput` | `dio` | sample |
| `DigitalOutput` | `dio` | actuator |
| `AnalogInput` | `analog` | sample |
| `QuadratureEncoder` | `dio_a`, `dio_b`, `counts_per_rev` | sample |
| `DutyCycleEncoder` | `dio`, `offset_rot` | sample |
| `PWM` | `pwm` (servo or PWM speed controller) | actuator |

Extend `hardware::State` and `hardware::Command` with fixed-size arrays for the
new families, alongside the existing `motors[]` and `sensors[]`:
`digital_inputs[]`, `analog_inputs[]`, `encoders[]`, `pwm_outputs[]`,
`digital_outputs[]`. Keep them typed rather than collapsing into one untyped
`DeviceSample` array — the encode/decode in `talOS/hardware/messages.cc` is
explicit little-endian by hand, and a tagged union makes that harder to review
for no gain.

Size check: at 200 Hz, a ~2 KB state message copied by six subscribers is under
2.5 MB/s. That is not a latency problem. Do not compress, delta-encode or split
the state message to "optimize" it.

---

## 7. Config format

One `robot.toml` on the companion. `talOS/configuration/` already vendors
`toml++` and holds sketch files (`robot.toml`, `subsystem.toml`,
`config_parser.h`) — grow those rather than starting a new module.

```toml
[robot]
name = "comp"
period_us = 5000              # RIO loop period
command_timeout_us = 100000   # command lease
commissioned = false          # set true only after tuning on real hardware

[subsystems.drivetrain]
node = "//2026-robot/main_processor/drivetrain:node"
period_us = 5000

[subsystems.drivetrain.motors.front_left_drive]
type = "TalonFX"
bus = "canivore"
can_id = 1
inverted = false
supply_limit_a = 40
stator_limit_a = 80
sensor_to_mechanism_ratio = 6.75
max_velocity_rps = 20
slot0 = { p = 0.0, i = 0.0, d = 0.0, v = 0.0 }

[subsystems.drivetrain.sensors.front_left_encoder]
type = "CANcoder"
bus = "canivore"
can_id = 20
offset_rot = 0.0

# Subsystem-specific values the node needs but the RIO does not.
[subsystems.drivetrain.geometry]
wheel_radius_m = 0.0508
modules = [
  { name = "front_left", x_m = 0.30, y_m = 0.30,
    drive = "front_left_drive", steer = "front_left_steer" },
]
```

Rules for the parser:

- **Logical device ids are assigned by the parser, deterministically**: sort by
  subsystem name, then device name, and number from 1. Never hand-written. Both
  ends must agree, and replay requires a stable ordering.
- **Ownership is exclusive.** A device belongs to exactly one subsystem. Reject
  at parse time: duplicate `(bus, can_id)`, duplicate DIO/PWM/analog channel,
  duplicate device name within a subsystem, a `feedback_sensor_id` reference to
  a device on a different bus, and any subsystem claiming a device declared
  under another.
- Reject out-of-range numbers rather than clamping: a typo'd `supply_limit_a`
  must fail the build, not quietly become 40.
- The parser output is the existing `hardware::Config` plus a
  `hardware::Devices` view per subsystem (the ids that subsystem owns).
  `ConfigurationId(config)` keeps working unchanged.
- **`talOS/hardware/swerve_config.h` is deleted at the end of step 5.** It is
  the compiled-in config that this plan exists to remove. Do not extend it.

---

## 8. Implementation steps

### Step 0 — guard the shm name limit
- Add a length check in `RTMSQueue`'s constructor: topic names longer than 30
  characters after the leading slash throw with a message naming the limit and
  the offending topic.
- Rename existing topics to the scheme in §5.
- **Accept:** a test asserts the throw; `bazel test //talOS/...` green on macOS.

### Step 1 — TOML parser
- `talOS/configuration/`: parse `robot.toml` into `hardware::Config` plus
  per-subsystem `hardware::Devices`.
- Implement every validation rule in §7.
- **Accept:** `config_test.cc` covers each rejection rule with its own case, and
  a golden test parses a four-module swerve `robot.toml` into a `Config` equal
  to what `SwerveConfig()` produces today. That equality is what proves the
  migration is lossless.

### Step 2 — device taxonomy
- Extend `hardware::Config`, `State`, `Command` and the codecs in
  `talOS/hardware/messages.cc` for every kind in §6.
- Extend `SimBackend` to model the new kinds ideally (a `DigitalInput` reads
  back what a test sets; a `PWM` output reads back what was applied).
- **Accept:** `//talOS/hardware:gateway_test` covers encode/decode round trips
  for every device kind, including the empty and full array cases.

### Step 3 — config push, and the RIO becomes final
- New frame kinds in `talOS/protocol/types.h`: `kHardwareConfig = 22`,
  `kHardwareConfigAck = 23`.
- Chunked because of the 1200-byte payload cap: each chunk carries
  `(index, count, config_id)`; the RIO buffers, and applies only when all chunks
  have arrived and the assembled CRC matches. A partial config is discarded, and
  never applied.
- The RIO refuses all commands while unconfigured (`kConfigured` clear) and
  echoes `config_id` in every state message.
- **The RIO validates the received config and clamps against hard-coded
  ceilings** it holds itself — maximum supply and stator current, maximum
  voltage, maximum velocity. A config exceeding a ceiling is rejected outright,
  with the reason in the ack. This is the safety boundary; review it as such.
- `robot/src/main/cpp/Robot.cpp` loses `SwerveConfig()` and becomes generic.
- **Accept:** `tools/test_drivetrain.py --wpilib` passes with config pushed at
  runtime instead of compiled in; a test asserts an over-limit config is
  rejected and leaves the robot unconfigured; killing and restarting the RIO
  program mid-run re-pushes automatically with no human action.

### Step 4 — `hardware_node`
- Replace `2026-robot/main_processor/drivetrain/bridge.cc` with `talOS/bridge/node.cc`: pushes
  config, republishes `/hw/state`, subscribes every `/hw/req/<sub>`, merges into
  one `/hw/cmd`.
- Merge rules: a device nobody claimed, or whose owner's request is older than
  `command_timeout_us`, is neutral. Requests are merged in a fixed order
  (subsystem name) so the output is a deterministic function of the inputs.
- **Accept:** a test with two request publishers shows each subsystem's devices
  taking its own values, an unclaimed device neutral, and a stale request
  neutralized on schedule. The node replays with `diverged=0`.

### Step 5 — convert the drivetrain
- `DrivetrainNode` takes its `hardware::Devices` from config, subscribes
  `/hw/state`, publishes `/hw/req/drive` and a new `/drivetrain/state`.
- Delete `talOS/hardware/swerve_config.h`.
- **Accept:** `tools/test_drivetrain.py` and `--wpilib` both pass unchanged in
  behaviour; the drivetrain still replays with `diverged=0`.

### Step 6 — a shooter, as proof
- Add `[subsystems.shooter]` to `robot.toml` with a flywheel `TalonFX` and a
  `DigitalInput` beam break. Write `2026-robot/main_processor/shooter/node.h` — a timer, a target
  watcher, velocity control, `/shooter/state`.
- **Accept:** the flywheel spins in simulation **with no change to any RoboRIO
  file, to `hardware_node`, or to the drivetrain.** A diff touching those files
  means the design has failed and the step is not done. Extend
  `tools/test_drivetrain.py` to assert it.

### Step 7 — a consumer node
- A state-estimation node that declares no hardware and subscribes to
  `/drivetrain/state` and `/shooter/state`, publishing `/odometry`.
- **Accept:** it builds and replays without any hardware config entry at all.

### Step 8 — the launcher
- One program that reads `robot.toml`, starts every declared node, restarts on
  crash, and writes a session manifest (session id, per-node log path, pid, exit
  status, and each recorder's `failed()`).
- Add a **session id to `log::FileHeader`** (format v3) so every log from one run
  correlates without guessing from wall clocks.
- A merged, time-ordered `log_dump` across a session's logs.
- **Accept:** one command brings up the whole robot in simulation; the session
  manifest names every log; the merged dump interleaves them correctly.

---

## 9. How to add a subsystem, once this is done

1. Add `[subsystems.<name>]` to `robot.toml` with its motors and sensors.
2. Write `talOS/<name>/node.h`: subscribe `/hw/state`, publish
   `/hw/req/<name>` and `/<name>/state`, put the control logic in a timer.
3. Add it to the launcher's node list (same file).
4. Run. Nothing else changes.

If a step ever requires editing the RoboRIO program, `hardware_node`, or another
subsystem, stop and fix the design instead — that is the invariant this plan
exists to protect.
