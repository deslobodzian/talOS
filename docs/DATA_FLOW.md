# Robot data flow and byte budgets

This is the wire/slot contract, not the native `sizeof(State)` or a claim about
Ethernet airtime. Bytes below exclude UDP/IP/Ethernet overhead unless explicitly
included. Changes to a frame, codec, device cap, slot, or telemetry budget must
update this document and its matching compile-time assert in the same change
([hard rule](../.omp/RULES.md)). Fully populated encoder tests pin the State,
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

All robot logic runs on the main processor. The RIO owns devices and the native
Driver Station interface. Camera images go directly to the main processor, not
through the RIO. All arrows between main-processor nodes are shared memory, not
network sockets. The simulation gateway replaces the RIO; it does not add a
second hop. See [architecture](../ARCHITECTURE.md#two-processors).

### Link rates and endpoints

| Link / direction | Transport / port | Trigger or rate in the 2026 configuration |
|---|---|---|
| Devices ↔ RIO backend | CAN / local DIO, analog, encoder, PWM; no IP port | Gateway samples/applies on `Config.period_us`: 5000 µs = 200 Hz. Phoenix motion feedback requests `status_hz`; voltage/current/fault diagnostics request `min(status_hz, 50)` Hz. Shared vendor frames apply their highest requested rate: voltage/current remain 200 Hz with 200 Hz feedback; fault field applies 50 Hz. Disabled one-shot neutral refresh remains every hardware tick. No wire/size change. |
| Driver Station ↔ RIO | WPILib/FRC native control/status over radio; not talOS UDP | DS produces new data at 50 Hz; RIO refreshes WPILib each hardware tick, emits a talOS DS snapshot at most every 20000 µs. Mode generations observe every hardware tick, including transitions between published snapshots. Native FRC packet layouts are external to this repo. |
| RIO → hardware_node: State (type 20) | UDP source gateway 5802 → bridge 5803, wired robot switch | One snapshot per gateway tick, 200 Hz; echoes clock probe send time only after a subsequent gateway tick. |
| hardware_node → RIO: Command (21) | UDP 5803 → 5802 | Bridge polls every 1000 µs. Send immediately when merged command changes, otherwise heartbeat `clamp(command_timeout_us / 5, 5000, 20000)` µs: 20000 µs = 50 Hz with the configured 100000 µs timeout. Only after observed state and while a subsystem request is active; not unconditionally 1000 Hz. |
| hardware_node → RIO: Config chunks (22) | UDP 5803 → 5802 | Whole config push on observed boot/config mismatch or while unacknowledged; retry on State after ≥50000 µs, doubling after rejected ACKs from 100 ms to 2 s. A changed boot ID pushes immediately and resets backoff. Commands continue during backoff; no pushes after successful matching acknowledgment. |
| RIO → hardware_node: ConfigAck (23) | UDP 5802 → 5803 | On completed assembly and asynchronous reconfiguration, or rejected chunk/config; no per-chunk ACK for incomplete assemblies. Per-device degraded configure succeeds, with invalid samples identifying unavailable devices. |
| RIO → hardware_node: DriverStation (24) | UDP 5802 → 5803 | ≤50 Hz on real RIO and standalone `sim_gateway`: snapshots are gated to ≥20000 µs apart, independently of the hardware tick. |
| hardware_node → RIO: ClockProbe (25) | UDP 5803 → 5802 | Empty payload; bridge send time in frame header. At most 50 Hz, one outstanding, retry after 1 s; independent of enable, commissioning and active requests. |
| hardware_node → device-owning nodes / DS consumers / telemetry | Shared-memory `/hw/state`, `/hw/state/driver_station`, `/hw/status`; no port | State/DS on received frames; bridge status first tick then every 20000 µs (50 Hz, `node.h`'s `kStatusPeriodUs`). Consumers run at their own periods. |
| drivetrain / shooter / intake → hardware_node | Shared-memory `/hw/request/<node>` | Devices-layer flush each update: drivetrain and shooter 5000 µs (200 Hz); intake 20000 µs (50 Hz). Bridge neutralizes an owner silent for >2× its period. |
| driver / teleop / autonomous → actuator nodes | Shared-memory target lanes | 20000 µs (50 Hz); mode/freshness policy selects one lane. `driver` produces chassis control; `teleop` provides configured mechanism control; autonomous supplies auto lanes. |
| Camera → camera node | USB capture, AVFoundation on macOS / V4L2 on Linux; no port | Requested capture rate 60 fps; capture/processing workers run at camera availability. Node period 10000 µs (100 Hz) does not guarantee 60 fresh images/s. |
| Camera nodes → pose_estimator / telemetry | Shared-memory `/<camera>/state`, `/<camera>/status` | Four configured camera nodes: status each 100 Hz node tick; live observations once per processed frame, up to 60 fps, not republished at the node rate. |
| drivetrain → pose_estimator | Shared-memory `/drivetrain/state` | 200 Hz publication; estimator updates every 20000 µs (50 Hz), fusing latest odometry and new valid camera observations. |
| `sim_gateway --physics` → simulated cameras / telemetry | Shared-memory `/sim/state` | Every physics/hardware tick (200 Hz); main-processor node-clock timestamp, true field pose and field-frame velocity. No writer under ideal or WPILib/RoboRIO simulation. |
| Nodes → telemetry → studio_bridge | Shared-memory states/statuses → `/talos/telemetry` byte topic | Telemetry period 10000 µs = 100 Hz; dictionary first frame and every 100th frame (1 Hz at this rate). |
| studio_bridge ↔ browser/agent | HTTP + WebSocket, configured port 5800 | Forwards telemetry frames as available; initial graph/dictionary on connection. HTTP requests and graph refresh are not the 100 Hz value stream. |

Camera observation timestamps describe exposure, not publication. Measure each
camera's timestamp delay during bring-up and set its `[camera] latency_offset_us`
(default 0): live and sim both subtract this offset from the capture stamp.
`last_latency_us` measures node time minus that corrected stamp. Sim-only
`camera.sim_timestamp_lag_us` (default 0, ignored live) adds known stamp bias and
delays delivery until the biased stamp's instant; equal offset restores the
original truth stamp without removing delivery delay. Both values must be
finite, nonnegative and below one second. Underflow/future stamps publish invalid.

Timing sources: `2026-robot/main_processor/configuration/robot.toml:1-16`, its
`subsystems/*.toml`, `talOS/bridge/node.cc` (`Tick`, `PushConfig`,
`ShouldSendCommand`), `talOS/hardware/endpoint.h:25-98`,
`2026-robot/controller_processor/rio/src/main/cpp/Robot.cpp:221-258`, and
`2026-robot/main_processor/simulation/sim_gateway.cc`.

## UDP frame and payload sizes

One datagram contains exactly one protocol frame, version **3** (State layout
changed; version 2 peers are rejected). The explicit little-endian
header is **40 B**: magic u64 (8), version/header_size/type/flags u16 (8 total),
sequence u64 (8), monotonic_time_us u64 (8), payload_size u32 (4), reserved u32
(4). Payload maximum is **1432 B**; frame maximum **1472 B** = 1500-byte MTU −
20-byte IPv4 header − 8-byte UDP header. Under this IPv4/MTU contract talOS frames
need no IP fragmentation (`talOS/protocol/frame.h`, `frame.cc`, `types.h`).

| Payload | Fixed vs dynamic formula (bytes) | Maximum payload | Maximum frame | Payload headroom |
|---|---|---:|---:|---:|
| State, type 20 | `66 + 47M + 27S + 4DI + 4DO + 15AI + 23E + 11PWM + 34` | 1420 | 1460 | 12 |
| Command, type 21 | `38 + 20M + 3DO + 10PWM` | 410 | 450 | 1022 |
| Config chunk, type 22 | packed 24-byte header + ≤1408 data | 1432 | 1472 | 0 for full chunk |
| ConfigAck, type 23 | packed u64 config_id + u8 status + char[64] reason | 73 | 113 | 1359 |
| DriverStation, type 24 | 40 fixed + two FMS strings + six variable joysticks | 678 | 718 | 754 |
| ClockProbe, type 25 | empty; send timestamp uses existing frame header | 0 | 40 | 1432 |

State/Command maxima use caps **M=16, S=12, DI=8, DO=4, AI=4, E=4,
PWM=4** (`talOS/hardware/config.h`). The 66-byte State prefix is six u64 +
u32 flags + seven u16 counts; its 34-byte suffix is health (18) + four u32
counters (16). Command prefix is four u64 + three u16 counts = 38.
The sixth u64 is `clock_probe_time_us`, echoing a probe received before the
current gateway tick began. Zero means no probe yet. It does not affect
command acceptance, ACK sequence, epoch, or actuator watchdogs.

| Device | State bytes / device | Command bytes / device | Encoded fields |
|---|---:|---:|---|
| Motor | 47 | 20 | State: id2 + valid1 + four doubles32 + three u32 fields12. Command: id2 + mode1 + slot1 + demand8 + feedforward8. |
| Sensor | 27 | — | id2 + valid1 + two doubles16 + two ages8 |
| Digital input | 4 | — | id2 + valid1 + value1 |
| Digital output | 4 | 3 | State id2 + valid1 + value1; command id2 + value1 |
| Analog input | 15 | — | id2 + valid1 + voltage8 + age4 |
| Encoder | 23 | — | id2 + valid1 + position8 + velocity8 + age4 |
| PWM output | 11 | 10 | State id2 + valid1 + output8; command id2 + output8 |

These are codec sizes (`talOS/hardware/messages.h`, `messages.cc`), not native
struct padding. Binary64 values remain binary64 on the wire.

### Actual 2026 robot

The canonical parser merges `2026-robot/main_processor/configuration/robot.toml`
and its subsystem files. Current device counts are **10 motors, 5 sensors,
2 digital inputs, 0 digital outputs, 0 analog inputs, 0 encoders, 0 PWM outputs**.

- State: `66 + 10×47 + 5×27 + 2×4 + 34` = **713 B** payload,
  **753 B** frame, **719 B** payload headroom.
- Command: `38 + 10×20` = **238 B** payload, **278 B** frame,
  **1194 B** payload headroom.
- Serialized Config: **3231 B**; `ceil(3231 / 1408)` = **3 chunks**.
  Chunk data lengths **1408, 1408, 415 B**, payload lengths **1432, 1432,
  439 B**, datagram lengths **1472, 1472, 479 B**. One push is **3423 B**
  of talOS frames (`3231 + 3×24 + 3×40`), excluding UDP/IP headers.
- State at 200 Hz: **150600 B/s** in talOS frames (`753×200`), or
  **1.2048 Mbit/s**, on the wired internal link, not the robot radio.

Computed with a throwaway C++ program calling the production `ParseRobotConfig`,
`SerializeConfig`, and `CreateConfigChunks`, and the named State/Command size
formulas; source/binary were removed after measurement. `robot_field.toml`
produced the same counts and sizes. This measures the real string/config codec,
not a TOML text-length estimate.

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
Hardware configuration assembly rejects serialized totals above **10,051 B**
(`kMaxSerializedConfigSize`) before allocation. Chunk data is limited to
**1,408 B**; chunk counts cannot exceed 10,051 or the serialized byte count.
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

**Known byte-order inconsistency:** ConfigChunkHeader and ConfigAckPayload are
packed structs copied with `memcpy`, so they use host byte order. Serialized
Config, State, Command, DS, and protocol frame fields explicitly write
little-endian. Both current endpoints are little-endian; this document records
the inconsistency without changing the wire format.

## Main-processor shared-memory slots

Fixed message topics store their complete native FlatBuffers struct/envelope
plus a writer publication timestamp; there is no UDP frame header. RTMS segment
version **5** stores `[payload][padding to 8][i64 publish_time_ns][tail padding]`.
For payload size M and alignment A, timestamp offset is `align_up(M, 8)` and
slot stride is `align_up(align_up(M, 8) + 8, max(A, 8))`. The **832 B** ring
header is additional; data starts at `align_up(832, max(A, 8))`. These sizes
are compile-time asserted in `talOS/rtms/rtms.h`.

The timestamp uses the host monotonic clock (macOS `CLOCK_MONOTONIC_RAW`,
Linux `CLOCK_MONOTONIC`), shared with `node.now()`; simulation uses its virtual
clock. Payload and timestamp are copied before the same slot-validity check.
For published position P and slot sequence s, readers require P − s < slots,
except that equality is accepted when the reliable-publication marker equals P.
Overwriting writers invalidate this marker before modifying slot bytes. A
standalone publisher reclaims incompatible unheld segments; attaching readers
reject version mismatches. A new session owner replaces any prior segment.

`age()` and `fresh()` count from publication, never delivery. A stalled consumer
cannot revive an expired command by draining it. Latest-value subscriptions
dispatch only the newest available message per drain and count skipped messages
as dropped; per-message callbacks retain sequence delivery, capped at 64 messages
per topic per drain by default. Dispatch scheduling latency remains separate.

| Topic / owner → consumer | Slot bytes | Meaning / rate |
|---|---:|---|
| `/hw/state`: bridge → devices/telemetry | 1448 | 1436 B `hardware::Packet`: u32 size + 1432 data; occupied data currently 713, ≤1420 |
| `/hw/command`: bridge → observers/logging | 1448 | 1436 B Packet; merged Command occupied data currently 238, ≤410 |
| `/hw/state/driver_station`: bridge → DS consumers/telemetry | 1448 | 1436 B Packet; DS occupied data 46–678 |
| `/hw/request/<node>`: device owner → bridge | 1448 | 1436 B Packet; only that owner's actuator requests, not a full-capacity command |
| `/hw/status`: bridge → telemetry/observers | 184 | 176 B fixed `bridge.BridgeStatus`, alignment 8 |
| `/<sub>/target/<source>`: producer → target resolver | `align_up(16 + sizeof(message), 8) + 8` | TargetEnvelope payload: mode-generation/active header 16 + FlatBuffers struct; chassis/shooter/intake payload 40/24/20 B, slots 48/32/32 B |
| `/autonomous/state`: autonomous → telemetry | 16 | 2 B AutonomousState: phase enum and selected/latched routine_index (ubyte); routine adds one 8 B telemetry scalar |
| `/drivetrain/state`: drivetrain → estimator/driver/telemetry | 120 | 112 B DrivetrainState: padded time/flags header 16 + chassis/yaw 32 + four module positions 64 |
| `/intake/state`: intake → telemetry | 32 | 20 B IntakeState, including FlatBuffers padding |
| `/shooter/state`: shooter → telemetry | 40 | 32 B ShooterState, including FlatBuffers padding |
| `/<camera>/state`: each camera → estimator/telemetry | 72 | 64 B geometry.PoseObservation, including padding |
| `/<camera>/status`: each camera → telemetry | 96 | 88 B vision.VisionStatus, pinned in vision.h; append-only reconnect/stall/control-failure counters and controls_applied are excluded from robot telemetry, preserving its value-frame budget |
| `/pose_estimator/state`: estimator → telemetry | 80 | 72 B geometry.PoseEstimate; `valid` = fresh usable odometry, `localized` = established field-frame trust; `localized` sits in existing payload padding |
| `/sim/state`: physics sim_gateway → four simulated cameras / telemetry | 64 | 56 B geometry.OdometrySample: sample_time_ns + Pose2d + Twist2d; 200 Hz, always valid while published; absent with ideal or WPILib/RoboRIO simulation |
| `/talos/telemetry`: telemetry → studio_bridge | 16392 × 64 slots | 16384 B byte payload capacity; ring = 1049088 B of slot storage; actual frames have padding stripped before WebSocket forwarding |

Sources: `talOS/hardware/packet.h`, `talOS/bridge/bridge_status.fbs`,
`talOS/devices/target.h`, `talOS/msgs/geometry.fbs`, robot `*_message.fbs`,
`vision/vision.fbs`, and `talOS/telemetry/wire.h`. Struct sizes include alignment
padding: they must not be inferred by simply adding scalar field widths.
The `/sim/state` payload is pinned by `static_assert(sizeof(geometry::OdometrySample)
== 56)` in `simulation/sim_gateway.cc`; RTMS adds its 8 B publication timestamp.
This truth topic is local to the main processor and never crosses hardware UDP.

### Gateway clock accuracy

For a bridge probe sent at t1, echoed gateway sample t2, and bridge t4 sampled
on the tick **after** State receipt, causality gives offset in
`[t1 - t2, t4 - t2]`. The bridge publishes this interval's midpoint and
`clock_offset_uncertainty_us` (half elapsed round trip rounded up, plus 2 us
quantization and <=1000 ppm relative oscillator drift over RTT and sample age).
Gateway and bridge each deliberately defer one tick: tick-start timestamps
otherwise precede receive and cannot honestly bound the exchange.
No symmetry assumption is needed; processing/residence time remains in the
bound instead of requiring extra receive/transmit wire stamps.
Select the minimum-RTT exchange for at most one second. A new boot resets it;
eight ordered States and an unexpired exchange are required for
`clock_offset_valid` / `HardwareLink::sample_time()`. One-way spread remains
diagnostic only: constant delay produces zero spread but nonzero error.
There is no specified fusion accuracy threshold, so report uncertainty rather
than inventing one. This bounds clock mapping, not CAN acquisition-age error;
bench qualification must validate the oscillator assumption and fusion budget.

Autonomous phases are IDLE, SPINUP, FEED, DRIVE, HOLD, and ABORTED (append-only
ubyte; state remains one byte). SPINUP and DRIVE abort after configured deadlines;
FEED ends after its validated positive duration. Pose outages release the drive
lane and pause trajectory progress without extending the DRIVE deadline. HOLD
and ABORTED release all autonomous target lanes; leaving enabled-autonomous
resets the routine to IDLE.

Phoenix followers drive only while their master is permitted and their own
cached position/velocity feedback—including configured remote CANcoder
feedback—is fresh and finite; invalid feedback neutralizes the follower.

## Dispatch logs

`.tlog` version **6** carries a publication timestamp on each message dispatch
and send, separate from event/dispatch time. Exact replay preserves publication
age; input replay injects at the recorded publication time rather than receipt.
Older formats are rejected with the recorded and supported versions. The packed
file header remains **120 B**, manifest entry **112 B**, and record header grows
from 56 to **64 B**, followed by the stored payload bytes. Matching assertions
live in `talOS/events/log/format.h`.

## Telemetry, Studio, and the radio boundary

Telemetry emits size-prefixed FlatBuffers `Frame`s: a **4-byte** little-endian
length followed by a `TLM2` buffer. Value-only frames are dynamically sized by
channel count but limited to **1200 B including the prefix**; the producer builds
a worst-case frame at startup and refuses an over-budget dictionary. Dictionary
frames are larger/variable, bounded by the **16384 B** slot. First/every-100th-frame
dictionaries allow late join/recovery in about a second at 100 Hz.

The 2026 configuration excludes cumulative camera counters, the 64-bit
`hw/status/clock_boot_id` (a double cannot carry it exactly), the two new
clock uncertainty/minimum-RTT fields, and the simulator's timestamp/velocity at
channel level (`subsystems/telemetry.toml`), retaining
camera health, timing, dropped-frame and visible-tag diagnostics. Excludes match
topics or individual dictionary channels; only retained channels count.

Current budget, measured 2026-11-08 with the producer's own
`MeasureValueFrameBytes` / `MeasureDictionaryFrameBytes` (the telemetry node
prints this line whenever it builds from a declared graph, so every
`--describe-only` and every launch reports it):

| Graph | Channels | Worst-case value frame | Spare channels | Dictionary frame |
|---|---:|---:|---:|---:|
| `robot.toml`, `robot_field.toml` | 130 | 1168 / 1200 B | 4 | 8984 / 16384 B |
| `robot_physics_sim.toml` + `--start-sim-gateway --sim-gateway-arg --physics` | 133 | 1192 / 1200 B | 1 | 9152 / 16384 B |

A graph over the cap makes telemetry refuse construction with the largest
sources named; `--describe-only` then reports telemetry as not described and
`talos_graph_test` fails, so the overflow is caught in CI rather than as missing
Studio data. The physics-sim graph has one spare channel: a new state scalar
needs an exclude (or a frame-format change) in the same commit.

Studio's bridge forwards the occupied frame, not 16384 bytes of padding. At the
value cap, `1200×8×100` = **0.96 Mbit/s per connected client**, before TCP/IP,
WebSocket, retransmission, dictionary and graph overhead. The value cap leaves
room under a normal MTU, but TCP is a byte stream: segmentation/coalescing is not
a guaranteed one-WebSocket-frame-per-IP-packet contract. Dictionary frames can
span multiple TCP segments.

The `[studio]` HTTP/WebSocket port is **5800**. Browser traffic to a driver laptop
crosses the robot radio and shares the team's field allocation with DS, other
telemetry and video. RIO↔main-processor UDP stays on the robot's wired switch and
is not charged against wireless throughput. Local USB camera images also do not
consume that budget unless separately streamed off-robot (this path does not).

The existing telemetry README's R704 reference is supported by
[WPILib's bandwidth guidance](https://docs.wpilib.org/en/stable/docs/networking/networking-introduction/measuring-bandwidth-usage.html),
which explicitly identifies R704 in the 2026 manual and uses a 4-Mbit reference
budget. Thus 0.96 Mbit/s is 24% of that **planning reference**, not a guaranteed
per-client allowance or an overhead-inclusive measured total. The actual event's
current radio/manual allocation remains authoritative; a wired internal link is
not subject to the wireless allocation.

## What never crosses the hardware UDP link

Only hardware State, merged actuator Command, Config chunks/ACK, and Driver
Station snapshots cross ports 5802/5803. In this robot only **drivetrain, intake,
and shooter** own devices and produce `/hw/request/<node>`; the bridge builds
request subscriptions from device ownership. Vision, pose_estimator, autonomous,
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

The cost is a hard device-cap budget tied to one datagram: State has only **20 B**
worst-case headroom. Grow in this order if required: first narrow appropriate
telemetry-only binary64 fields to float32 (an explicit wire/version change), then
split State by CAN bus using an **all-parts-or-drop multipart snapshot** with a
shared identity/time. Neither growth option is implemented today; silently
raising caps or sending partial snapshots is not an acceptable substitute.
