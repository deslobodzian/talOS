# Robot data flow and byte budgets

This is the wire/slot contract, not the native `sizeof(State)` or a claim about
Ethernet airtime. Bytes below exclude UDP/IP/Ethernet overhead unless explicitly
included. Changes to a frame, codec, device cap, slot, or telemetry budget must
update this document and its matching compile-time assert in the same change
([hard rule](../RULES.md)). Fully populated encoder tests pin the State,
Command, and Driver Station formulas to the actual encoders.

## Processes and links

```mermaid
flowchart LR
  DS[Driver Station laptop] <-->|FRC control/status via radio| RIO[RoboRIO gateway]
  DEV[CAN motors / sensors and local IO] <-->|Phoenix / WPILib| RIO
  RIO <-->|wired UDP 5802 ↔ 5803| HW[hardware_node]
  SIM[sim_gateway or WPILib RoboRIO sim] <-->|same UDP contract in simulation| HW
  SIM -->|/sim/state, physics gateway only| V
  SIM -->|/sim/state, physics gateway only| T
  HW -->|/hw/state and /hw/state/driver_station| N[drivetrain / intake / shooter]
  N -->|/hw/request/node| HW
  HW -->|Driver Station snapshot| DRIVER[driver / teleop / autonomous]
  DRIVER -->|target lanes| N
  CAM[USB cameras] -->|AVFoundation / V4L2| V[four camera nodes]
  V -->|camera/state PoseObservation| P[pose_estimator]
  N -->|drivetrain/state| P
  N -->|state| T[telemetry]
  V -->|state / status| T
  P -->|pose_estimator/state| T
  HW -->|state / DS / status| T
  T -->|/talos/telemetry shared memory| SB[studio_bridge]
  SB <-->|HTTP / WebSocket 5800 via radio| UI[Studio browser / agents]
```

For processor ownership and layer boundaries, see
[Architecture: Two processors](../ARCHITECTURE.md#two-processors).

### Link rates and endpoints

| Link / direction | Transport / port | Trigger or rate in the 2026 configuration |
|---|---|---|
| Devices ↔ RIO backend | CAN / local DIO, analog, encoder, PWM; no IP port | Gateway samples/applies on `Config.period_us`: 5000 µs = 200 Hz. Phoenix motion feedback requests `status_hz`; voltage/stator-current/supply-current/fault diagnostics request `min(status_hz, 50)` Hz; device temperature requests `min(status_hz, 4)` Hz. Shared vendor frames apply their highest requested rate: voltage/current/temperature remain 200 Hz with 200 Hz feedback; fault field applies 50 Hz. Disabled one-shot neutral refresh remains every hardware tick. |
| Driver Station ↔ RIO | WPILib/FRC native control/status over radio; not talOS UDP | DS produces new data at 50 Hz; RIO refreshes WPILib and observes the mode generation each hardware tick; snapshot cadence is in row DriverStation (24). Native FRC packet layouts are external to this repo. |
| RIO → hardware_node: State (type 20) | UDP source gateway 5802 → bridge 5803, wired robot switch | One snapshot per gateway tick, 200 Hz; echoes clock probe send time only after a subsequent gateway tick. |
| hardware_node → RIO: Command (21) | UDP 5803 → 5802 | The bridge sleeps until a gateway datagram, a `/hw/request/<node>` publication or its next time-driven duty, and merges each request when it is published, within a CPU budget of 8 passes and 64 received datagrams per 1000 µs window, past which it sleeps through input to the window's end (`HardwareNode::kCpuWindowUs`). Send immediately when the merged command changes, so one command per accepted request that changes the merge; actuator nodes publish on their own `period_us`, each request carrying a new observation or demand, so at most drivetrain 200 + shooter 200 + intake 50 = 450 commands/s, each a 278 B frame (10 motors: 238 B payload) ≈ 125 kB/s. Otherwise heartbeat `clamp(command_timeout_us / 5, 5000, 20000)` µs: 20000 µs = 50 Hz with the configured 100000 µs timeout. Only after observed state and while a subsystem request is active. The RIO receives at most 64 frames per 5000 µs hardware tick and applies only the highest-sequence command of each tick's batch (`endpoint.h` `kReceiveBudget`), about 2 commands per tick at 450/s, so a backlog queued during a RIO stall is skipped, not replayed. |
| hardware_node → RIO: Config chunks (22) | UDP 5803 → 5802 | Whole config push on observed boot/config mismatch or while unacknowledged; retry on State after ≥50000 µs, doubling after rejected ACKs from 100 ms to 2 s. A changed boot ID pushes immediately and resets backoff. Commands continue during backoff; no pushes after successful matching acknowledgment. |
| RIO → hardware_node: ConfigAck (23) | UDP 5802 → 5803 | On completed assembly and asynchronous reconfiguration, or rejected chunk/config; no per-chunk ACK for incomplete assemblies. An assembly completed while a configure is pending (every boot restore) gets no ACK; the bridge's ≥50000 µs retry pushes again. Per-device degraded configure succeeds, with invalid samples identifying unavailable devices. |
| RIO → hardware_node: DriverStation (24) | UDP 5802 → 5803 | 50 Hz on real RIO and standalone `sim_gateway`, independently of the hardware tick: snapshots are ≥20000 µs apart, except that the tick observing a new mode sends one at once. An enabled mode switch (autonomous to teleop) changes no State; only the new generation retires the old mode's target lanes. |
| hardware_node → RIO: ClockProbe (25) | UDP 5803 → 5802 | Empty payload; bridge send time in frame header. At most 50 Hz, one outstanding, replaced after 100 ms unanswered (a lost probe or echo), well inside the 1 s exchange lifetime; independent of enable, commissioning and active requests. |
| hardware_node → device-owning nodes / DS consumers / telemetry | Shared-memory `/hw/state`, `/hw/state/driver_station`, `/hw/status`; no port | State/DS when their frame arrives; bridge status first tick then every 20000 µs (50 Hz, `node.h`'s `kStatusPeriodUs`). Consumers run at their own periods. |
| drivetrain / shooter / intake → hardware_node | Shared-memory `/hw/request/<node>` | Devices-layer flush each update: drivetrain and shooter 5000 µs (200 Hz); intake 20000 µs (50 Hz). Bridge neutralizes an owner silent past its [owner timeout](../ARCHITECTURE.md#the-tick) (50 ms for each of these). |
| driver / teleop / autonomous → actuator nodes | Shared-memory target lanes | 20000 µs (50 Hz); mode/freshness policy selects one lane. `driver` produces chassis control; `teleop` provides configured mechanism control; autonomous supplies auto lanes. |
| Camera → camera node | USB capture, AVFoundation on macOS / V4L2 on Linux; no port | Requested capture rate 60 fps; capture/processing workers run at camera availability. Node period 10000 µs (100 Hz) does not guarantee 60 fresh images/s. |
| Camera nodes → pose_estimator / telemetry | Shared-memory `/<camera>/state`, `/<camera>/status` | Four configured camera nodes: status each 100 Hz node tick; live observations once per processed frame, up to 60 fps, not republished at the node rate. |
| drivetrain → pose_estimator | Shared-memory `/drivetrain/state` | 200 Hz publication; estimator updates every 20000 µs (50 Hz), fusing latest odometry and every valid camera observation delivered since the previous update (in sequence, queued by at-drain subscription callbacks, so camera publications do not wake it). |
| `sim_gateway --physics` → simulated cameras / telemetry | Shared-memory `/sim/state` | Every physics/hardware tick (200 Hz); main-processor node-clock timestamp, true field pose and field-frame velocity. No writer under ideal or WPILib/RoboRIO simulation. |
| Any publisher → the loop of a reader asleep on its topic (wake) | AF_UNIX datagram, Linux abstract address / macOS `/tmp/talos-wake-<id>`; no port ([talOS/rtms](../talOS/rtms/README.md#wake-channel)) | 1 byte, at most one per reader sleep per topic: a write sends one to each reader slot armed since the last publication (a reader of a prompt subscription under `Delivery::ON_PUBLISH`, the bridge's reader of each `/hw/request/<node>`, or `studio_bridge`'s reader of `/talos/telemetry`, between its drain and its next block), none when no reader is armed; a writer taking over a topic sends one to every reader with a receiver. A lost datagram costs a node's loop up to its 1 s backstop, the bridge up to its longest wait, 10000 µs (`HardwareNode::kMaxWaitUs`), and `studio_bridge` up to its next duty, at most its 250 ms graph refresh ([Studio bridge](../studio/bridge/README.md#waking)). A send that fails other than to a gone receiver counts in the writer's registry row (`wake_failures`), and a request the bridge's latest-value read passes over in its subsystem's request row (`requests_skipped`; [wake loss](../talOS/introspection/README.md#wake-loss)). |
| Nodes → telemetry → studio_bridge | Shared-memory states/statuses → `/talos/telemetry` byte topic | Telemetry period 10000 µs = 100 Hz, each tick as one or more value pages; dictionary pages before the first tick and every 100th (1 Hz at this rate). |
| studio_bridge ↔ browser/agent | HTTP + WebSocket, configured port 5800 | Per client: graph, dictionary pages and a keyframe on connection, then change-only telemetry updates of at most one per producer tick and system-graph value updates of at most one per 500 ms (the whole graph again when its structure changes), deferred and coalesced when the egress budget is short. HTTP requests are not the value stream. |
| studio_bridge → driver dashboard (`/drive`) | WebSocket, the same port | Per drive client (at most the layout's `drive_clients`, outside `max_clients`): dictionary pages and a keyframe of the layout's channels on connection, then change-only updates of those channels at most `update_hz` (default 20) a second, an empty update after 100 ms without one, and the drive summary (at most 4,096 B) on change at most every 500 ms; within the layout's `budget_bytes_per_s` (default 24,000 B/s), reserved out of the bridge's 160,000 B/s ([bridge README](../studio/bridge/README.md#driver-dashboard-feed)). |

A failed send never kills the bridge (`RuntimeUdpPeer` raises no
`SIGPIPE`). When a send to the gateway fails with an error its socket never
recovers from (`protocol::SendErrorNeedsNewSocket`, whose errors per OS are
in [talOS/protocol](../talOS/protocol/README.md): a socket shut down or
marked defunct, or one whose source address left the host), the bridge
closes the socket and opens a new one on the same ports (`Reopen()`, which
keeps the frame sequence the gateway's replay gate checks): on its next pass,
at most 1000 µs later, then at most every 100000 µs
(`HardwareNode::kReopenRetryUs`). While it is closed nothing is sent or
received, so the RoboRIO neutralizes at its command timeout and `/hw/status`
`state_age_us` grows, as for an unplugged cable; with no route to the
RoboRIO at all, each new socket's open fails and is retried on that
cadence. Any other send error (`ECONNREFUSED` while the RoboRIO program is
down, for one) keeps the socket, and the failed send's duty is retried
1000 µs later. The bridge prints the errno once per
outage and one line when a send succeeds again; its exit line counts
`send_failures` and `socket_reopens`. The RoboRIO program replaces its own
socket by the same rule (`protocol::SendErrorNeedsNewSocket`), on its 100 ms
open-retry cadence ([talOS/hardware](../talOS/hardware/README.md#roborio)).

Camera observation timestamps describe exposure, not publication. Measure
timestamp bias during [camera bring-up](BRINGUP.md#camera-intrinsics-calibration).
Every live and simulated stamp subtracts `camera.latency_offset_us`, after
the per-kind exposure/readout correction described in the
[vision README](../2026-robot/main_processor/vision/README.md#camera-localization-nodes).
Sim-only `camera.sim_timestamp_lag_us` adds stamp bias and delays delivery;
an equal offset restores the truth stamp without removing delivery delay.
Both offsets default to zero and must be finite, nonnegative and below one
second. Underflow/future stamps publish invalid.

Timing sources: `2026-robot/main_processor/configuration/robot.toml:1-16`, its
`subsystems/*.toml`, `talOS/bridge/node.cc` (`Tick`, `PushConfig`,
`ShouldSendCommand`), `talOS/hardware/endpoint.h` (`Endpoint::Tick`),
`2026-robot/controller_processor/rio/src/main/cpp/Robot.cpp`
(`Robot::StartCompetition`), and
`2026-robot/main_processor/simulation/sim_gateway.cc`.

Phoenix 26.3.0 `CoreTalonFX.hpp` / `SpnValue.hpp` identify stator current,
supply current and device temperature as `PRO_SupplyAndTemp` SPNs 1779–1783.
[CTRE's shared-frame rule](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/status-signals.html)
applies the highest requested rate to a shared frame. The Phoenix desktop
rate test checks the effective rates: the current and temperature reads share
a frame already requested at 200 Hz, adding no requested status-frame traffic.
This is not a measured
whole-bus utilization percentage: use `/hw/status` CAN utilization during
physical bring-up, including all other devices and control traffic.
Temperature older than 500 ms and supply current older than 50 ms read as zero
without invalidating motion feedback, like existing effort diagnostics.
Standalone ideal/physics simulation reports ambient 25 °C and placeholder
supply current proportional to voltage effort; Phoenix desktop uses its SDK
electrical-current model and ambient 25 °C. None is a thermal model or an
automatic thermal derating policy. Raw `/hw/state` stays in local logs and
has no reflected radio channels; only the bench motor state adds these scalars.

## UDP frame and payload sizes

One datagram contains exactly one protocol frame, version **7**.
The explicit little-endian header is **40 B**: magic u64 (8),
version/header_size/type/flags u16 (8 total),
sequence u64 (8), monotonic_time_us u64 (8), payload_size u32 (4), reserved u32
(4). Payload maximum is **1432 B**; frame maximum **1472 B** = 1500-byte MTU −
20-byte IPv4 header − 8-byte UDP header. Under this IPv4/MTU contract talOS frames
need no IP fragmentation (`talOS/protocol/frame.h`, `frame.cc`, `types.h`).

| Payload | Fixed vs dynamic formula (bytes) | Maximum payload | Maximum frame | Payload headroom |
|---|---|---:|---:|---:|
| State, type 20 | `84 + 45M + 27S + 4DI + 4DO + 15AI + 23E + 11PWM + 34` | 1406 | 1446 | 26 |
| Command, type 21 | `38 + 20M + 3DO + 10PWM` | 410 | 450 | 1022 |
| Config chunk, type 22 | 24-byte header (config_id u64, total_crc32 u32, total_bytes u32, chunk_index u16, chunk_count u16, chunk_bytes u32) + ≤1408 data | 1432 | 1472 | 0 for full chunk |
| ConfigAck, type 23 | u64 config_id + u8 status + char[64] reason | 73 | 113 | 1359 |
| DriverStation, type 24 | 40 fixed + two FMS strings + six variable joysticks | 678 | 718 | 754 |
| ClockProbe, type 25 | empty; send timestamp uses existing frame header | 0 | 40 | 1432 |

State/Command maxima use caps **M=16, S=12, DI=8, DO=4, AI=4, E=4,
PWM=4** (`talOS/hardware/config.h`). The 84-byte State prefix is seven u64,
the gateway build (10: u64 revision, u8 program, u8 dirty), u32 flags and
seven u16 counts; its 34-byte suffix is health (18) + four u32 counters
(16). Command prefix is four u64 + three u16 counts = 38.
The sixth u64 is `clock_probe_time_us`, echoing a probe received before the
current gateway tick began. Zero means no probe yet. It does not affect
command acceptance, ACK sequence, epoch, or actuator watchdogs.
The seventh u64 is `sequence`, the gateway's tick count within its boot,
incremented at the top of every `Gateway::Tick` whether or not that tick's
State is sent; the devices layer takes `sequence + 1` as proof that no State
was missed (`Motor::zeroed`).
The gateway build (`hardware::GatewayBuild`) names the program that sends the
State (unknown 0, RoboRIO program 1, the RoboRIO program under WPILib
simulation 2, `sim_gateway` 3; a decoder rejects any other value, and a dirty
byte other than 0 or 1) and its source: the first 16 hex digits of its git
commit as one number (0 when unknown) and whether it was built with
uncommitted changes. GradleRIO writes them into the RoboRIO program
(`generateRioBuild` in its `build.gradle`); `sim_gateway` sends revision 0.
Every State carries it (10 B × 200 Hz = 2000 B/s on the wired link) rather
than one frame per boot, so it always arrives with the boot id it belongs to
and every State the bridge's `--log` records names its build.

| Device | State bytes / device | Command bytes / device | Encoded fields |
|---|---:|---:|---|
| Motor | 45 | 20 | State: id2 + valid1 + position/velocity/voltage doubles24 + stator-current int16(2) + three u32 fields12 + temperature int16(2) + supply-current int16(2). Command: id2 + mode1 + slot1 + demand8 + feedforward8. |
| Sensor | 27 | — | id2 + valid1 + two doubles16 + two ages8 |
| Digital input | 4 | — | id2 + valid1 + value1 |
| Digital output | 4 | 3 | State id2 + valid1 + value1; command id2 + value1 |
| Analog input | 15 | — | id2 + valid1 + voltage8 + age4 |
| Encoder | 23 | — | id2 + valid1 + position8 + velocity8 + age4 |
| PWM output | 11 | 10 | State id2 + valid1 + output8; command id2 + output8 |

These are codec sizes (`talOS/hardware/messages.h`, `messages.cc`), not native
struct padding. Position, velocity and voltage remain binary64. Motor stator
current, supply current and device temperature use signed little-endian int16
in 0.1 A / 0.1 °C units, rounded to nearest (ties away from zero), saturated
to [-3276.8, 3276.7]; nonfinite inputs fail encoding.

### Actual 2026 robot

The canonical parser merges `2026-robot/main_processor/configuration/robot.toml`
and its subsystem files. Current device counts are **10 motors, 5 sensors,
2 digital inputs, 0 digital outputs, 0 analog inputs, 0 encoders, 0 PWM outputs**.

- State: `84 + 10×45 + 5×27 + 2×4 + 34` = **711 B** payload,
  **751 B** frame, **721 B** payload headroom.
- Command: `38 + 10×20` = **238 B** payload, **278 B** frame,
  **1194 B** payload headroom.
- Serialized Config: **3251 B**; `ceil(3251 / 1408)` = **3 chunks**.
  Chunk data lengths **1408, 1408, 435 B**, payload lengths **1432, 1432,
  459 B**, datagram lengths **1472, 1472, 499 B**. One push is **3443 B**
  of talOS frames (`3251 + 3×24 + 3×40`), excluding UDP/IP headers.
- State at 200 Hz: **150200 B/s** in talOS frames (`751×200`), or
  **1.2016 Mbit/s**, on the wired internal link, not the robot radio.


## Dynamic and multi-frame messages

**State and Command:** Count prefixes select the occupied array entries; unused
capacity is not serialized. A State or Command is always wholly contained in
one datagram. Invalid counts, truncated data, and trailing bytes are rejected
by decoding; cap-sized encoder tests assert exact named worst-case lengths.

**Driver Station:** The fixed portion is **40 B**: sample/mode times and the
gateway boot id (three u64), FMS fixed fields, and the two string-length bytes.
Event and game message strings each transmit at most **31 B** (their storage
is 32 including NUL). Each of the six joysticks is **1 B** when disconnected.
Connected cost is
`1 connected + 3 counts + 4 buttons + 4×axes + 2×POVs + 1 name_length + name`.
At caps (12 axes, 4 POVs, 31 name bytes) this is **96 B** each; maximum is
`40 + 31 + 31 + 6×96` = **678 B**; empty strings/all disconnected is **46 B**.
The boot id is the one the same gateway stamps on its State; consumers order
packets by sample/mode time only within one boot, start over on a new one, and
ignore the last four retired boots (`talOS/bridge/boot_history.h`); the bridge
forwards only packets of the boot its accepted State is in.
Source: `talOS/driver_station/driver_station.h` and `.cc` (`Encode`).

**Config:** `SerializeConfig` is a variable-length, count/string-prefixed codec.
Each chunk carries config_id, CRC32 of the entire serialized config, total byte
count, chunk index/count, and chunk data length. `ConfigAssembler` accepts chunks
out of order, ignores duplicate chunks, waits until all indices are present,
checks CRC32, deserializes and validates caps, then reconfigures. A new config ID
replaces the assembly; inconsistent metadata or invalid assembled data is
rejected. Completion/backend failure or malformed data produces the 73-byte ACK
with status/reason (`config_wire.h`, `endpoint.h`).
Hardware configuration assembly rejects serialized totals above **10,083 B**
(`kMaxSerializedConfigSize`) before allocation. Chunk data is limited to
**1,408 B**; chunk counts cannot exceed 10,083 or the serialized byte count.
Transfer metadata and chunk geometry must remain consistent, and assembler
exceptions reject the transfer without escaping the gateway safety loop.

There is no selective retransmission or hot-path fragmentation protocol. A lost
chunk leaves an incomplete assembly with no success ACK; the bridge repeats the
whole config at its 50 ms retry gate while State indicates mismatch or no matching
success ACK. Lost ACK likewise causes a whole-push retry. Existing valid control
configuration is not replaced by a partially assembled transfer. Every runtime
sender, including config ACKs, writes zero header flags (`kFlagNone`, the only
declared flag): ConfigAck is a frame type, and chunking is payload-level.
Sequence numbers still expose datagram gaps.

Every multi-byte field on the link, frame header, chunk header and ACK
included, is written little-endian byte by byte, never as the host's object
representation (`frame.cc`, `config_wire.h`); the packed structs only fix the
sizes asserted above.

## Main-processor shared-memory slots

Fixed message topics store their complete native FlatBuffers struct/envelope
plus a writer publication timestamp; there is no UDP frame header. RTMS segment
version **9** stores `[payload][padding to 8][i64 publish_time_ns][tail padding]`.
For payload size M and alignment A, timestamp offset is `align_up(M, 8)` and
slot stride is `align_up(align_up(M, 8) + 8, max(A, 8))`. The **2368 B** ring
header (32 reader slots of 64 B) is additional; data starts at
`align_up(2368, max(A, 8))`. These sizes
are compile-time asserted in `talOS/rtms/rtms.h`.

The timestamp uses the host monotonic clock (macOS `CLOCK_MONOTONIC_RAW`,
Linux `CLOCK_MONOTONIC`), shared with `node.now()`; simulation uses its virtual
clock. Payload and timestamp are copied before the same slot-validity check.
For published position P and slot sequence s, readers require P − s < slots,
except that equality is accepted when the reliable-publication marker equals P.
Overwriting writers invalidate this marker before modifying slot bytes. Every
attach, publisher or reader, rejects a segment of another version, layout or
schema. A new session owner replaces any prior segment.
Writer and reader claims in the header record one 64-bit process word: the pid
(22 bits), the low 20 bits of the process start time (Linux clock ticks, macOS
milliseconds) and a 20-bit pid-namespace code (`kProcessPidBits` in
`talOS/rtms/process_liveness.h`), so a reused pid
holds no claim and a claim from another pid namespace sharing `/dev/shm` is
never probed in this one; a reader state word is that word above 2 state bits.
Two header words carry the [wake channel](../talOS/rtms/README.md#wake-channel)
within the same 64-byte writer and reader lines: the writer's `waiters`
bitmask and each reader's `wake_id`; neither changes a size.

`age()` and `fresh()` count from publication, never delivery. A stalled consumer
cannot revive an expired command by draining it. Latest-value subscriptions
dispatch only the newest available message per drain and count skipped messages
as dropped; per-message callbacks retain sequence delivery, capped at 64 messages
per topic per drain by default. Dispatch scheduling latency remains separate.

| Topic / owner → consumer | Slot bytes | Meaning / rate |
|---|---:|---|
| `/hw/state`: bridge → devices/telemetry | 1448 | 1436 B `hardware::Packet`: u32 size + 1432 data; occupied data currently 711, ≤1406 |
| `/hw/command`: bridge → observers/logging | 1448 | 1436 B Packet; merged Command occupied data currently 238, ≤410 |
| `/hw/state/driver_station`: bridge → DS consumers/telemetry | 1448 | 1436 B Packet; DS occupied data 46–678 |
| `/hw/request/<node>`: device owner → bridge | 1448 | 1436 B Packet; only that owner's actuator requests, not a full-capacity command |
| `/hw/status`: bridge → telemetry/observers | 200 | 192 B fixed `bridge.BridgeStatus`, alignment 8; `commissioned` and the six alerts fill the padding after `configured` |
| `/<sub>/target/<source>`: producer → target resolver | `align_up(16 + sizeof(message), 8) + 8` | TargetEnvelope payload: mode-generation/active header 16 + FlatBuffers struct; chassis/shooter/intake payload 40/24/20 B, slots 48/32/32 B |
| `/autonomous/state`: autonomous → telemetry | 16 | 2 B AutonomousState: phase enum and selected/latched routine_index (ubyte); routine adds one 8 B telemetry scalar |
| `/drivetrain/state`: drivetrain → estimator/driver/telemetry | 120 | 112 B DrivetrainState: padded time/flags header 16 + chassis/yaw 32 + four module positions 64 |
| `/intake/state`: intake → telemetry | 32 | 20 B IntakeState, including FlatBuffers padding |
| `/shooter/state`: shooter → telemetry | 40 | 32 B ShooterState, including FlatBuffers padding |
| `/motor/state` (bench only): motor → telemetry/logs | 104 | 96 B MotorState including device temperature, supply current and limit switches; pinned in bench motor node |
| `/<camera>/state`: each camera → estimator/telemetry | 72 | 64 B geometry.PoseObservation, including padding |
| `/<camera>/status`: each camera → telemetry | 96 | 88 B vision.VisionStatus, pinned in vision.h; every field, counters included, reaches telemetry |
| `/pose_estimator/state`: estimator → telemetry | 80 | 72 B geometry.PoseEstimate; `valid` = fresh usable odometry, `localized` = established field-frame trust; `localized` sits in existing payload padding |
| `/sim/state`: physics sim_gateway → four simulated cameras / telemetry | 64 | 56 B geometry.OdometrySample: sample_time_ns + Pose2d + Twist2d; 200 Hz, always valid while published; absent with ideal or WPILib/RoboRIO simulation |
| `/talos/telemetry`: telemetry → studio_bridge | 16392 × 64 slots | 16384 B byte payload capacity; ring = 1049088 B of slot storage; the writer overwrites, so a reader more than 63 pages behind (about 0.2 s at 100 Hz with three value pages a tick) is lapped and counts the missed pages, and a stalled reader never stops publication; the bridge never applies a tick it lost a page of |

Sources: `talOS/hardware/packet.h`, `talOS/bridge/bridge_status.fbs`,
`talOS/devices/target.h`, `talOS/msgs/geometry.fbs`, robot `*_message.fbs`,
`vision/vision.fbs`, and `talOS/telemetry/wire.h`. Struct sizes include alignment
padding: they must not be inferred by simply adding scalar field widths.
The `/sim/state` payload is pinned by `static_assert(sizeof(geometry::OdometrySample)
== 56)` in `simulation/sim_gateway.cc`; RTMS adds its 8 B publication timestamp.
This truth topic is local to the main processor and never crosses hardware UDP.

The live node registry ([talOS/introspection](../talOS/introspection/README.md))
is one more segment, `/talos_registry.6` (the number is its layout version):
a 64 B header and 32 `NodeRecord` rows of **11136 B**, alignment 64, for
**356416 B**. A row holds its node's identity, 64 source records of 136 B,
4 Target selection records of 524 B and, at its end, the wake counters
(`wake_failures`, `stalled_arms`, `wake_fallbacks`, 3 × 8 B in what was tail
padding). Every node writes its own row about four times a second; the
launcher and `studio_bridge` read it. Both sizes are compile-time asserted in
`registry.h`.

### Wake-channel descriptors

A writer holds one connected socket per reader slot it has notified, per
topic it writes ([wake channel](../talOS/rtms/README.md#wake-channel)), so a
node's wake sockets are at most the readers of the topics it writes, and never
more than `MAX_READERS` (32) per topic: the bridge, which writes four
topics, could hold 4 × 32 = 128 (139 descriptors with its 11 others, under
the macOS limit below) only if 32 readers of each slept on it; what each
armed reader adds to a publication is measured under "Publish cost" in the
[wake channel](../talOS/rtms/README.md#wake-channel). For the field config
(`2026-robot/main_processor/configuration/robot.toml`, its `--describe-only`
graph), counting every reader as armed:

| Node | Topics written (with readers) | Wake sockets, worst case |
|---|---|---:|
| `hardware_node` | `/hw/state` 3, `/hw/state/driver_station` 8, `/hw/status` 5 | 16 |
| `drivetrain`, `shooter` | own state 3, `/hw/request/<node>` 1 | 4 |
| `autonomous` | three target lanes and its state, 1 each | 4 |
| `intake`, each camera | own state 2, request or status 1 | 3 |
| `pose_estimator`, `teleop` | own state 2; two target lanes, 1 each | 2 |
| `driver`, `telemetry` | a target lane 1; `/talos/telemetry` (`studio_bridge`) 1 | 1 |

Every node held 9 to 11 descriptors otherwise in a running `--sim` session on
Linux (`/proc/<pid>/fd`: stdio and log pipes, the event-log file, the poller,
the stop pipe, its wake receiver; the bridge's UDP socket). The peak, the
bridge's 27, leaves 229 of macOS's default soft `RLIMIT_NOFILE` of 256 (and
997 of Linux's usual 1024), so nodes do not raise the limit. A node that did
run out would show it in its registry row's `wake_failures`, and in the
session manifest, where `session_report --strict` rejects it.

### Gateway clock accuracy

For a bridge probe sent at t1, echoed gateway sample t2, and bridge t4 sampled
on the tick **after** State receipt, causality gives offset in
`[t1 - t2, t4 - t2]`. The bridge publishes this interval's midpoint and
`clock_offset_uncertainty_us` (half elapsed round trip rounded up, plus 2 us
quantization and <=1000 ppm relative oscillator drift over RTT and sample age).
Gateway and bridge each deliberately defer one tick: tick-start timestamps
otherwise precede receive and cannot honestly bound the exchange. The
bridge's tick runs when the State arrives and the next one at once, without
sleeping (`HardwareNode::NextDutyUs`), so t4 follows receipt by that tick's
work alone.
No symmetry assumption is needed; processing/residence time remains in the
bound instead of requiring extra receive/transmit wire stamps.
Every exchange of the last second is kept (up to 64), and each State uses the
minimum-RTT one that is at most one second old, so the next best is already
held when the selected one expires and validity does not lapse while probes
are answered. A new boot clears them; eight ordered States and an unexpired
exchange are required for `clock_offset_valid` /
`HardwareLink::sample_time()`. One-way spread remains
diagnostic only: constant delay produces zero spread but nonzero error.
There is no specified fusion accuracy threshold, so report uncertainty rather
than inventing one. This bounds clock mapping, not CAN acquisition-age error;
bench qualification must validate the oscillator assumption and fusion budget.

Phoenix followers drive only while their master is permitted and their own
cached position/velocity feedback—including configured remote CANcoder
feedback—is fresh and finite; invalid feedback neutralizes the follower.

## Dispatch logs

`.tlog` version **7** carries a publication timestamp on each message dispatch
and send, separate from event/dispatch time. Exact replay preserves publication
age; input replay injects with the recorded publication time, at that time,
or, for a node the session ran, at the drain that received it.
Older formats are rejected with the recorded and supported versions. The packed
file header is **120 B**, manifest entry **112 B**, and record header **64 B**,
followed by stored payload bytes (`talOS/events/log/format.h`).
`Target::get` adds one payload-free READ record (a 64 B header) per `Target`
per `update()` that selects a lane, however often it calls `get()` or
`source()`: 6.4 KB/s for a 100 Hz consumer of one target.

A byte-capped log (`LogWriterOptions::max_bytes`, the launcher's per-member
share of `[robot] log_max_bytes`, in proportion to the bytes the member's log
is expected to store per second: [log budget](../talOS/launcher/README.md#session-segments-and-logs)) is split into n = min(8, cap / (prefix +
largest record)) segments of cap / n bytes, each a complete version-7 log
(its own header, manifest and schema block, then whole records). A segment
ends when the next record would cross cap / n, or, with a window
(`LogWriterOptions::window`, the launcher's `--log-window-us`), at the first
record whose time (`now_ns`) is window / n or more past the segment's first,
so a log that stays within its bytes keeps between (n - 1) / n of the window
and the window. Time never ends a segment sooner than the writer's
`sync_interval` (1 s) after its first record, whatever the window: a
rotation syncs durably and takes a buffer chunk, so faster rotation could
exhaust the writer's pool and stop capture. The newest
is at the log's path, older ones at `<stem>.segment<k>.tlog`, and starting
segment n + 1 deletes the oldest, so the log keeps its tail within the cap.
`log_dump`, `tlog_to_wpilog` and `--replay-inputs` read every segment in a
session directory, and `log_dump` or `tlog_to_wpilog` given one log's file
reads that log's kept segments with it;
exact `--replay` needs the first segment, since a later one starts
mid-session. Rotation syncs each finished segment and the next one's header
before renaming it into place, and the cap marker before deleting the oldest,
so a power loss leaves whole segments; a session reader skips (with a
warning) a file cut before its header was complete. A cap below two segments
stops capture at the cap instead.

## Telemetry, Studio, and the radio boundary

Telemetry emits size-prefixed FlatBuffers `Frame` pages: a **4-byte**
little-endian length followed by a `TLM3` buffer. A tick is as many **value
pages** as its channels need, each at most **1200 B including the prefix**
(12 B per sample); a dictionary is as many **dictionary pages** as it needs,
each at most one **16384 B** slot. A dictionary holds at most **2048**
channels (`kMaxChannels`): a keyframe of that many, at most 25 pages with
WebSocket framing, is at most 30,250 B, inside the bridge's **64,000 B** burst,
which an update must fit to be sent whole (`static_assert` in `wire.h`). The
producer refuses construction past the cap, naming the largest sources, and
refuses a channel whose dictionary entry alone cannot fit one page. Each dictionary name, unit and enum label is at most **256 B**, and an
enum channel carries at most **1024** labels (`wire.h`); Studio's decoder
accepts exactly these limits, and the producer never emits past them
([telemetry README](../talOS/telemetry/README.md#transport-budget)). Dictionary
pages before the first tick and every 100th allow late join and recovery in
about a second at 100 Hz. Integer fields travel as exact 64-bit integers, so no
channel is excluded for precision either.

Every 2026 and bench state and status channel is published; the telemetry
configs exclude nothing. The declared graphs report their pages at
construction (`MeasurePages` in `talOS/telemetry/frames.h`):

| Graph | Channels | Value pages per tick (worst case) | Dictionary |
|---|---:|---:|---:|
| `robot.toml`, `robot_field.toml` | 176 | 3, 2592 B (largest 1168 / 1200 B) | 2 pages, 11496 B |
| `robot_physics_sim.toml` + `--sim --start-sim-gateway --sim-gateway-arg --physics` | 183 | 3, 2672 B (largest 1168 / 1200 B) | 2 pages, 11880 B |
| Bench `robot.toml`, `robot_bench.toml` | 71 | 1, 1008 B | 1 page, 4296 B |

Studio's bridge sends each client its own updates
(`ClientFeed` in [`studio/bridge/telemetry.h`](../studio/bridge/telemetry.h)):
the dictionary pages and a keyframe of the latest tick on connection, then
only the channels whose value or presence changed since what that client last
received, stamped with the tick's `sequence_id` and `timestamp_ns`, and an
empty update every 100 ms when nothing changed. An update goes out only when
the budget affords all of it; otherwise it waits and the next one carries
every change since, so a client's view is always one whole producer tick.
**The live view therefore
skips intermediate samples whenever the budget is short; each node's event log
is the complete record.**

Studio's bridge sends at most **160,000 B/s** in all, charging every
WebSocket send with conservative framing overhead (`EgressBudget` in
`studio/bridge/telemetry.h`, `BridgeEgress` in `studio/bridge/drive.h`).
With a dashboard layout (`[studio] dashboard`), each driver dashboard client
has its own bucket of the layout's `budget_bytes_per_s` with a **64,000 B**
burst, and `drive_clients × budget_bytes_per_s` (at most **80,000 B/s**;
default 2 × 24,000 = **48,000 B/s**) is reserved out of the total first.
Programmer clients (`/telemetry`) share the rest, R = 160,000 B/s less that
reservation (**112,000 B/s** with the default layout, all of it without
one), through a shared bucket of R B/s with a **64,000 B** burst; each
programmer client also has an `R / max_clients` B/s bucket with the same
burst (**28,000 B/s** at the default 4 with the default layout, 40,000 B/s
without one); `[studio] max_clients` defaults to **4** (range **1–64**) and
does not count drive clients. A drive client's sends are charged to its own
bucket only, so no programmer client can defer one. At most `drive_clients`
are held; a new one evicts the oldest (close code 4000). Every programmer
send leaves in a pass over the clients (a client that joins or drains is offered its turn by one), and each
pass starts just after the last client sent anything, so the shared bucket is
granted in rotation: of the clients able to take a send (nothing buffered,
and both buckets, as they stood when the pass began, hold its whole next
message), none is sent to twice before
every other one has been sent to once (`ClientRotation` in
`studio/bridge/telemetry.h`). Messages differ in size, so a client whose
next update is larger than another's can be passed over while the shared
bucket holds the smaller one but not its own. A client that still has
output buffered is held: it skips passes, and its telemetry resumes from the
latest values once it drains. Being held never disconnects a client: the
bridge sends nothing while output is buffered, so at most one graph message
or update, never more than the **64,000 B** burst, is ever buffered, below
uWebSockets' **512 KiB** `maxBackpressure` (past which a send would be
dropped, not the client closed: `closeOnBackpressureLimit` is off). What
does disconnect a client that stops answering is the bridge's uWebSockets
idle timeout of **16 s** (`kIdleTimeoutS` in `studio/bridge/main.cc`). It
pings a client **4 to 8 s** after its last data or the last drain of its
backlog (uSockets checks timeouts every 4 s), and closes it if nothing
arrives in the next **8 s**, so the client's slot comes free **12 to 16 s**
after the last data it sent: a Driver Station laptop that drops Wi-Fi and
reconnects cannot keep every slot filled with its own dead sockets for
longer. 16 s is the
smallest timeout whose pong window (8 s) covers five retransmissions of a
lost pong at TCP's 200 ms minimum RTO (sent by 6.2 s). Studio's browser
answers pings, so a client that only receives stays connected. Each client's kernel send buffer is what its
share sends in **250 ms**, at least **16 KiB** (`ClientSendBuffer` in
`studio/bridge/telemetry.h`): 16,384 B at the default 4 clients and for a
drive client at the default 24,000 B/s, and at 1 client 40,000 B without a
dashboard layout (28,000 B with the default one). macOS uses that size as
given; Linux reserves twice it, about the size
in payload. A client that stops reading therefore backs up after that much
of its output (0.4 s of a 40,000 B/s share; up to 6.6 s of the 2,500 B/s
share at 64 clients without a dashboard layout, 9.4 s of the 1,750 B/s share
with the default one), not after the 4 MiB Linux autotuning reached, about
100 s of a 40,000 B/s share, plus whatever its own receive window takes,
which its OS sets and the bridge cannot; so those times are lower bounds.
A reading client is limited below its share only
when its round trip exceeds 250 ms; a send larger than the buffer, such as a
joining client's 64,000 B dictionary and keyframe, takes several round trips,
during which the client is held. HTTP snapshots and static files are outside
this budget.

160,000 B/s is **1.28 Mbit/s, 32 %** of the **4 Mbit/s** per-team field
bandwidth limit that WPILib's networking documentation and the FMS whitepaper
state, citing 2026 manual rule R704. Both the figure and the rule number are
unverified against the 2026 manual text itself; the event's current
allocation is authoritative. Before
TCP/IP and WebSocket overhead, measured in a scripted physics-sim match
(`robot_physics_sim.toml`, disabled 3 s / autonomous 14 s / teleop 30 s, one
viewer):

| Viewer budget | Telemetry bytes/s | System graph bytes/s | Telemetry updates/s | Total, whole run |
|---|---:|---:|---:|---:|
| 160,000 B/s (`max_clients = 1`) | 46,700–72,200 disabled/autonomous; 57,700–62,600 teleop | 5,500–12,000 | 96–100 (every tick) | 69,500 B/s |
| 40,000 B/s (4-client share, no dashboard layout) | 27,500–34,300 | 5,600–11,900 | 37–94, typically 45–55 | 39,700 B/s |

A full keyframe every tick would be 2552 B, 255,200 B/s. The system graph
(about 40 KB whole: 15 nodes, 94 sources, 28 topics) goes to a client whole
only on connect and when its structure changes; otherwise its fast values
(about 5.5 KB) go at most every 500 ms when they changed
(`GraphFeed` in [`studio/bridge/system.h`](../studio/bridge/system.h);
protocol in the [bridge README](../studio/bridge/README.md)). A client's graph
messages go before its telemetry (`ClientFeeds` in
[`studio/bridge/telemetry.h`](../studio/bridge/telemetry.h)): from the moment
one waits for budget it reserves half of the client's refill, and telemetry
may spend only what the budget holds beyond that reserve, at most one update
per 250 ms. However large a telemetry update, a waiting graph message of G
framed bytes therefore goes within

(G + n) / (ρ / 2 − n / 0.75 s)

on a client share of ρ B/s, where n is one framed deferral notice (71 B, sent
at most every 750 ms while it waits): **2.0 s** for the 40 KB whole graph at
the 40,000 B/s share of 4 clients without a dashboard layout (**2.9 s** at
the 28,000 B/s share with the default one), and **55.5 s** for a graph at the
64,000 B burst at the 2,500 B/s share of `max_clients = 64` (**82.1 s** at
its 1,750 B/s share with the default layout). The bound is on the client's own
bucket: the shared bucket refills at the sum of every share, so it adds delay
only while other clients spend bursts they saved, and a client whose socket
still has output buffered is sent nothing until it drains. Telemetry keeps
flowing beside the waiting graph, change-only, so the thinning skips samples,
never values; while a graph waits, a reading goes stale only when one update costs more than
about half of what the share refills in a second. At the 40,000 B/s share updates
arrived every 500–720 ms and no deferral notice was needed. Both rows stay
within their budget over the run; a single second can exceed the rate by up to
the 64,000 B burst.

The cost falls on structure changes. A whole graph is about a second of a
40,000 B/s share (1.4 s of the 28,000 B/s one), so after a node starts, stops
or restarts, that client's telemetry runs at four updates a second for about
that long (at most the bound above) while the graph goes. A whole graph goes at most every 2 s: in
a crash loop, a client's graph shows restarts up to 2 s late (marked
deferred, not stale), and its telemetry is thinned for about a second of
every two.

The value-page cap leaves room under a normal MTU, but TCP is a byte stream:
segmentation and coalescing are not a guaranteed
one-WebSocket-frame-per-IP-packet contract, and dictionary pages span several
TCP segments.

The `[studio]` HTTP/WebSocket port is **5800**. Browser traffic to a driver laptop
crosses the robot radio and shares the team's field allocation with DS, other
telemetry and video. RIO↔main-processor UDP stays on the robot's wired switch and
is not charged against wireless throughput. Local USB camera images also do not
consume that budget unless separately streamed off-robot (this path does not).

The rates above are WebSocket payload plus frame headers, measured on
loopback; they exclude TCP/IP overhead and retransmission.

## What never crosses the hardware UDP link

Hardware State, merged actuator Command, Config chunks/ACK, Driver Station
snapshots and ClockProbe frames cross ports 5802/5803.
In this robot only **drivetrain, intake, and shooter** own devices and produce
`/hw/request/<node>`; the bridge builds request subscriptions from device ownership.
Vision, pose_estimator, autonomous,
teleop, driver, and telemetry do not produce hardware requests. Targets,
camera images/observations/status, estimated pose, node state/status, and telemetry
frames stay on the main processor (telemetry subsequently travels to Studio on a
separate WebSocket connection). The camera nodes have no `/hw/*` source.

## Why one datagram per hardware tick?

A State is an atomic snapshot with one sample_time/config_id (and boot/epoch),
so consumers do not combine half-old and half-new device state. There is no
reassembly on the control hot path. A dropped datagram loses one snapshot, not a
piece that stalls the loop; freshness gating handles continued loss. Requests
retain the hardware identity/time they observed, the gateway rejects stale or
wrong-identity commands, and disabled/stale-target actuation is neutralized.
The configured command timeout is **100 ms**; a single loss does not imply an
immediate timeout. The maximum frame fits the IPv4 MTU without fragmentation.

The device caps are a hard one-datagram budget: State has **36 B**
worst-case payload headroom.
