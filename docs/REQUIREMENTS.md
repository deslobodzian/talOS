# Requirements

What talos must do, one testable sentence per requirement. Tests, commits
and design notes cite a requirement by its ID ([TESTING.md §3](TESTING.md#3-requirements-and-traceability));
a feature or fix names the one it serves ([RULES.md](../RULES.md) 15).

A requirement restates, as something a test can decide, a contract its
Source states. The Source is authoritative: a change to either changes the
other in the same commit, and adding, changing or dropping a requirement is
the user's decision, in its own commit ([DEVELOPMENT.md §1](DEVELOPMENT.md#1-intake-and-requirement)).
Open questions about what a requirement should say are TODO.md `SPEC-*`
items.

Columns: **P** is the priority: **P0** robot safety or match-losing, **P1**
correctness a developer or drive team relies on, **P2** convenience.
**Observable** is what a test reads. **Cheapest tier** is the lowest tier
([TESTING.md §2](TESTING.md#2-tiers-by-what-the-test-may-touch)) at which the
observable exists and every input that can violate the requirement can be
injected; a test above it is composition evidence, not the proof.

Source abbreviations; a quoted name is a heading in that file, `§N` a
numbered section, a number in RULES a rule:

| Abbrev | File |
|---|---|
| ARCH | [ARCHITECTURE.md](../ARCHITECTURE.md) |
| RULES | [RULES.md](../RULES.md) |
| NAMING | [talOS/NAMING.md](../talOS/NAMING.md) |
| METH | [METHODOLOGY.md](METHODOLOGY.md) |
| TEST | [TESTING.md](TESTING.md) |
| DESIGN | [DESIGN.md](DESIGN.md) |
| DF | [DATA_FLOW.md](DATA_FLOW.md) |
| NDG | [NODE_DEVELOPMENT_GUIDE.md](NODE_DEVELOPMENT_GUIDE.md) |
| LR | [talOS/launcher/README.md](../talOS/launcher/README.md) |
| HR | [talOS/hardware/README.md](../talOS/hardware/README.md) |
| IR | [talOS/introspection/README.md](../talOS/introspection/README.md) |
| ER | [talOS/events/README.md](../talOS/events/README.md) |
| RTMS | [talOS/rtms/README.md](../talOS/rtms/README.md) |
| R26 | [2026-robot/README.md](../2026-robot/README.md) |
| SR | [2026-robot/main_processor/simulation/README.md](../2026-robot/main_processor/simulation/README.md) |
| DEP | [DEPLOYMENT.md](DEPLOYMENT.md) |
| BR | [BRINGUP.md](BRINGUP.md) |
| INSTALL | [INSTALL.md](INSTALL.md) |
| STU | [studio/README.md](../studio/README.md) |
| TODO | [TODO.md](../TODO.md) |

## Catalogue

### SAFE: actuation gating, neutral, generations, arming, timeouts, watchdog

| ID | Requirement | P | Source | Observable | Cheapest tier |
|---|---|---|---|---|---|
| REQ-SAFE-01 | A motor that `update()` did not set in a tick is sent `neutral` in that tick's request. | P0 | ARCH "The tick"; METH §5 | `/hw/request/<node>` motor mode (SimHardware `motor_request`) | component |
| REQ-SAFE-02 | Every motor in a node's request is neutral while `robot_enabled(node)` is false: no accepted `/hw/state`, gateway not configured+enabled, a latched hardware fault, or `/hw/state` published longer than `command_timeout_us` ago. | P0 | ARCH "The tick"; RULES 1; code `talOS/devices/devices.cc`, `talOS/hardware/gate.h` | request bytes after `hw.enable(false)`, fault flag, `lose_states`, stepping past the timeout | component |
| REQ-SAFE-03 | A digital or PWM output is sent as set only when set during that `update()` while `robot_enabled`; otherwise DIO is false and PWM 0, and the bridge replaces an omitted DIO/PWM with false/0. | P0 | ARCH "The tick"; METH §5 | request bytes; merged `/hw/command` | unit + component |
| REQ-SAFE-04 | Motor demand and feedforward are clamped to the motor's configured limits, a non-finite value neutralizes only that motor, and `clamped_count()` rises once per adjusted setter call. | P0 | ARCH "The tick"; NDG "Hardware" | request demand; `clamped_count` | component |
| REQ-SAFE-05 | A rezero-frame motor's position and motion-magic demands go out neutral until a `rezero` holds, and the zero drops on each event NDG "Rezero and position frames" lists; an absolute-frame motor is never held. | P0 | ARCH "The tick"; NDG "Rezero and position frames" | request mode; `zeroed()` | component |
| REQ-SAFE-06 | Motor, DIO and PWM setters called outside `update()` (constructor, `every()`, callback) or off the loop thread throw `std::logic_error` (`TalosError` in Python) naming the device. | P0 | ARCH "The tick"; NDG "Threads" | exception and message | component |
| REQ-SAFE-07 | `Target::get()` returns the first lane in the current mode's policy order that is active, fresh and stamped with the generation the consumer last drained, only while `robot_enabled`; otherwise null. | P0 | ARCH "Targets"; RULES 1 | `get()`, `source()`, the actuator request | component |
| REQ-SAFE-08 | Freshness counts from the writer's publication timestamp, so a paused or stalled consumer that drains an expired message never treats it as fresh. | P0 | ARCH "The tick", "Targets"; DF "Main-processor shared-memory slots" | `fresh()` / `Target::get` after `Sim::pause`; slot `publish_time_ns` | component |
| REQ-SAFE-09 | A target queued before a mode change is rejected even when it is drained in the same drain as the packet that changes the mode. | P0 | ARCH "Targets"; TEST | `Target::get` null in that tick | component |
| REQ-SAFE-10 | A new mode generation starts on each transition ARCH "Targets" lists and not on a same-mode link drop, and the RoboRIO program and SimHardware both stamp it through `driver_station::ModeTracker`. | P0 | ARCH "Targets" | DS packet `mode_since_us` | unit + contract |
| REQ-SAFE-11 | Packets and States are ordered only within one gateway boot: a consumer restarts its ordering on a new boot and ignores recently retired ones, and the bridge forwards only DS packets of its accepted State's boot. | P0 | ARCH "Targets"; DF "Dynamic and multi-frame messages"; HR "Runtime Contract" | forwarded `/hw/state/driver_station`; `Target::get` across reboot | unit + component |
| REQ-SAFE-12 | `TargetLane::send` stamps the generation its own node last drained, which is 0, and so never selectable, while that node's DS view is stale, disabled or in no mode. | P0 | ARCH "Targets" | envelope header bytes | component |
| REQ-SAFE-13 | A lane opened with `OperatorArming` sends releases until an evaluation finds every input neutral while the DS shows a mode, and disarms on each cause ARCH "Targets" lists. | P0 | ARCH "Targets"; NDG "Targets" | envelope `active` flag per trigger | component |
| REQ-SAFE-14 | Every producer that maps operator input (`driver`, `//talOS/teleop:node`, any new one) opens its lanes with `OperatorArming`, so an input held through a disable, reboot, DS dropout or producer stall drives nothing until released once. | P0 | ARCH "Targets"; R26 "Nodes" | lane `active` after re-enable with the stick held | component |
| REQ-SAFE-15 | The bridge neutralizes a node's actuators once its last accepted request is older than its owner timeout (ARCH "The tick"), keeps the last demand for shorter gaps, and leaves other owners' outputs untouched. | P0 | ARCH "The tick"; code `talOS/bridge/merge.h` | `/hw/command` per owner | unit + component |
| REQ-SAFE-16 | The merged command carries the oldest observation of any owner still merged, so the gateway invalidates every output within `command_timeout_us` of that State. | P0 | ARCH "The tick"; code `talOS/bridge/merge.h` | merged `observed_time_us`; gateway expiry | unit |
| REQ-SAFE-17 | The gateway rejects a whole command that fails any check HR "Runtime Contract" lists (config id, boot, epoch, sequence, age, structure), applies only the newest command of a tick, and refreshes neutral every tick until a fresh command is accepted. | P0 | HR "Supported Hardware Requests", "Runtime Contract"; DF "Link rates and endpoints" | backend outputs; State counters | component |
| REQ-SAFE-18 | Enable transitions, command timeout, total feedback loss and backend stops advance the control epoch, so no request built before them actuates afterwards. | P0 | HR "Runtime Contract" | backend outputs after transition | component |
| REQ-SAFE-19 | The bridge rejects a request naming a device another subsystem owns and neutralizes every configured actuator a request omits. | P0 | HR "Runtime Contract"; DF "What never crosses the hardware UDP link" | `/hw/command`; `/hw/status` ownership rejects | component |
| REQ-SAFE-20 | The gateway refuses commands under an uncommissioned config; only `--sim` with a loopback remote commissions in memory, and the commissioned bit is part of the config id both ends compare. | P0 | HR "Configuration"; LR "Session segments and logs"; RULES 4 | gateway refusal counter; launcher exit before spawn | component + integration |
| REQ-SAFE-21 | A changed config applies only while disabled, outputs are neutralized before devices are reconfigured, an identical push may succeed enabled without device traffic, and a partial or rejected transfer never replaces the active config. | P0 | HR "Configuration", "Runtime Contract"; DF "Dynamic and multi-frame messages" | backend calls; ACK status | component |
| REQ-SAFE-22 | Invalid motion feedback or a missing remote CANcoder neutralizes only the affected motor and its followers or dependents, a follower drives only while its master may, and total loss of motor and CAN-sensor feedback neutralizes every output. | P0 | HR "Configuration", "Runtime Contract"; DF "Gateway clock accuracy" | backend outputs; counters | component |
| REQ-SAFE-23 | A device-local send failure isolates that motor chain, a global apply failure neutralizes every output, and repeated global failures latch a fault cleared only as HR "Runtime Contract" states. | P0 | HR "Runtime Contract" | fault flag; outputs | component |
| REQ-SAFE-24 | A device reset (vendor latch or impossible position step) neutralizes only the affected chain, restores it without blocking ticks, and requires a command observed after the restore before that chain drives again. | P0 | HR "Runtime Contract" | reset counter; outputs; tick continuity | component |
| REQ-SAFE-25 | When the RoboRIO loop stops beating past its command expiry, the loop watchdog neutralizes every output on each check until a beat moves the time, with one DS error per stall, at the margins HR "RoboRIO" states. | P0 | HR "RoboRIO"; ARCH "The tick" | backend neutral calls; DS error | component + bench |
| REQ-SAFE-26 | Config load refuses a `command_timeout_us` outside the range ARCH "The tick" states, an actuator owner whose `period_us` exceeds the stated fraction of it, `soft_limits = true` on a rezero-frame motor, and `position_frame = "absolute"` without `RemoteCANcoder` feedback. | P0 | ARCH "The tick"; HR "Configuration"; code `talOS/configuration/config_parser.h` | exception naming the key | unit |
| REQ-SAFE-27 | Only the bridge writes `hw` topics except `/hw/request/<node>`, which only `<node>`'s devices flush writes: `publish`/`publish_bytes` throw on any `/hw/request` topic at construction (C++ and Python) and `LintGraph` errors on any other `hw` writer. | P0 | RULES 2; NAMING "Topics" | constructor exception; lint diagnostic | unit + component |
| REQ-SAFE-28 | A node that owns a motor, DIO or PWM output or describes a target-lane sender cannot be `critical = false` (graph error), and the hardware bridge is always critical. | P0 | LR "Two phases"; DEP "Supervision and service recovery" | lint diagnostic; launcher exit | unit |
| REQ-SAFE-29 | A critical member (actuator owner, bridge) that crashes, stalls or exits 0 mid-session restarts forever on the capped backoff LR "Restarts" states, is never quarantined, and only `--max-restarts 0` disables it. | P0 | LR "Restarts"; DEP "Supervision and service recovery" | manifest `restart_count`, `runs[].reason`; restart timing | unit + integration |
| REQ-SAFE-30 | When the launcher dies, SIGKILL included, every child's process group is killed by its guardian (plus `PR_SET_PDEATHSIG` on Linux), so no node outlives its supervisor. | P0 | LR "Restarts"; DEP "Supervision and service recovery" | child pids gone after killing the launcher | integration |
| REQ-SAFE-31 | A restarted node is safe with every input present: an actuator node drives only on a lane `Target::get` accepts, an operator producer starts disarmed, and `autonomous` resumes its recorded step in the same match or starts INHIBITED. | P0 | METH §9; ARCH "Targets"; R26 "Nodes" | first `/hw/request` after restart | component |
| REQ-SAFE-32 | Every negative intake target, teleop or autonomous, is neutral unless shooter `at_speed` is fresh with a positive shooter target on that tick, and `/intake/state.inhibit` reports `SHOOTER_NOT_READY`. | P1 | R26 "Nodes" | intake request; `/intake/state` | component |
| REQ-SAFE-33 | An autonomous routine fires each action at most once per match across DS outages, re-enables and restarts, resets only on a new match as R26 "Nodes" defines it, and holds on an unreadable record. | P1 | R26 "Nodes"; METH §9 | `/autonomous/state` phase; lanes | component |
| REQ-SAFE-34 | Autonomous drive and collection stop on the tick the pose is invalid, unlocalized or stale, while both deadlines keep counting through outages. | P1 | R26 "Nodes"; METH §9 | auto lanes released | component |
| REQ-SAFE-35 | No viewer path (Studio bridge, Studio agent) writes any topic or sends any hardware frame. | P0 | DF "Processes and links"; TODO "Optional work" | Studio bridge `--describe` has no SENDER; no RTMS writer opened | contract |

### CONTRACT: wire format, schema identity, C ABI/Python parity, naming/graph rules, config validation

| ID | Requirement | P | Source | Observable | Cheapest tier |
|---|---|---|---|---|---|
| REQ-CONTRACT-01 | Every UDP datagram carries exactly one frame of the version, header and size limits in DF "UDP frame and payload sizes", and a frame of another version, magic or size is rejected rather than decoded. | P0 | DF "UDP frame and payload sizes"; HR "Runtime Contract" | decoder verdict | unit + fuzz/property |
| REQ-CONTRACT-02 | State, Command, DS, Config chunk and ConfigAck payloads follow DF's size formulas, fully capped instances encode at exactly DF's maxima, and invalid counts, truncation and trailing bytes are rejected. | P0 | DF "UDP frame and payload sizes", "Dynamic and multi-frame messages" | encoded length; decoder verdict | unit + fuzz/property + contract |
| REQ-CONTRACT-03 | Every multi-byte wire field, frame header, chunk header and ACK included, is written little-endian byte by byte, never as host object representation. | P0 | DF "Dynamic and multi-frame messages"; HR "Runtime Contract" | golden byte vectors | unit |
| REQ-CONTRACT-04 | Config chunks reassemble out of order with duplicates ignored and the CRC checked; oversized or inconsistent metadata is rejected before allocation; lost or rejected pushes retry as DF "Dynamic and multi-frame messages" states while commands keep flowing. | P1 | DF "Link rates and endpoints", "Dynamic and multi-frame messages"; HR "Configuration" | ACK; push cadence; command cadence | unit + component |
| REQ-CONTRACT-05 | Both endpoints compute the same config id over every wire-carried setting, the commissioned bit included, and the gateway refuses commands carrying another id. | P0 | HR "Configuration" | config id equality; refusal | unit + contract |
| REQ-CONTRACT-06 | The RoboRIO persists an acknowledged config durably while disabled, restores it at boot without blocking the loop, refuses commands until restored, and stays unconfigured on a missing or corrupt file. | P1 | HR "Configuration" | file contents; configured flag | component |
| REQ-CONTRACT-07 | An RTMS attach whose schema name or hash differs from the segment's fails naming both types; the hash changes with any layout-relevant schema change and not with comments, and is equal in C++ and Python. | P1 | ARCH "Topics and messages"; NDG "BUILD" | attach error; hash values | unit + contract |
| REQ-CONTRACT-08 | Topic names follow NAMING "Topics"; `talos_messages` refuses a malformed topic at BUILD load and a numeric struct field without a unit suffix at build, with the same grammar as `names.h`. | P1 | NAMING; ARCH "Topics and messages"; METH §6 | `names.h` verdict; build failure | unit + contract |
| REQ-CONTRACT-09 | The graph lint reports every error NAMING "Topics" lists (writers, reader cap, unwritten reads, schema and layout conflicts, lane policy, `hw` writers, target coverage), and the launcher then spawns nothing and exits non-zero unless `--allow-graph-errors`. | P0 | NAMING "Topics"; ARCH "Targets", "Topics and messages"; LR "Two phases" | diagnostics; exit code | unit + integration |
| REQ-CONTRACT-10 | A subsystem key that is not a framework key and that no node's constructor read is an error naming the subsystem, key, file and nearest declared key, and config load refuses a framework section spelled in the singular. | P1 | ARCH "Configuration"; LR "Parameter keys" | diagnostic text | unit |
| REQ-CONTRACT-11 | Each run's live endpoints equal its probe's; a mismatch stops the session, kills a non-critical member or is reported, per member and run as LR states. | P1 | LR "Live endpoints" | manifest `endpoint_mismatch`; exit code | integration |
| REQ-CONTRACT-12 | A node's `--describe` touches no shared memory, hardware, network or camera, exits 0 with its manifest within the describe timeout, and declares the same endpoints and params as when it runs. | P1 | NDG "Node"; LR "Two phases"; R26 "Nodes" | describe JSON; no shm created | integration |
| REQ-CONTRACT-13 | `node_api.h` and `_abi.py` agree on ABI version, exports, return and parameter types and `TALOS_*` constants, and `node_api.cc` statically asserts device kinds, motor modes and DS flags against the C++ enums. | P1 | DESIGN | parity test; compile | contract |
| REQ-CONTRACT-14 | Every node-API feature in `talOS/node` and `talOS/devices` is reachable from Python through the C ABI with the same semantics (Target, TargetLane, OperatorArming, HardwareLink, Sim/SimHardware), and Python `send` raises `TypeError` naming an omitted field. | P1 | RULES 6; NDG "Node" | Python test through the ABI | contract |
| REQ-CONTRACT-15 | Python reaches talOS only through `libtalos_node.so`'s C ABI and reimplements no wire format, memory layout, state machine or C++ node. | P1 | ARCH "Layers"; RULES 7 | import/dependency graph of `talOS/python` | contract (gap: TODO ARCH-2) |
| REQ-CONTRACT-16 | Bazel dependencies follow rtms, events, node, devices, robot code; nothing under `talOS/` depends on a robot, an example or `studio/`; and `talOS/vision` depends on `talOS/geometry`, never the reverse. | P1 | ARCH "Layers"; RULES 5 | `bazel query` result | contract (gap: TODO ARCH-2) |
| REQ-CONTRACT-17 | Config load rejects unknown device keys, duplicate ids, names or addresses, invalid numbers, invalid follower chains, missing remote references, a non-IPv4 controller address, an invalid port, and non-positive log or restart limits. | P1 | HR "Configuration"; LR "Controller addressing", "Restarts", "Session segments and logs" | exception naming key | unit + fuzz/property |
| REQ-CONTRACT-18 | `param` without a fallback throws at construction naming a missing key, a wrong array element names key and index, `param_of` throws after construction and on a subsystem the config lacks (even with a fallback). | P1 | NDG "Node", "Parameters"; METH §8 | exception text | component |
| REQ-CONTRACT-19 | `robot_manifest.bzl` (`NODES`, `CONFIG_FILES`) equals what its config launches and reads, and a stale manifest fails `manifest_sync_test` and the `talos_bundle` build with the regenerate command. | P1 | ARCH "Configuration" | test/build failure | unit + build |
| REQ-CONTRACT-20 | `.tlog` carries a CRC per record and per manifest/schema block and a schema descriptor per topic; readers reject an unknown version naming both, skip and count a damaged record, and read a truncated tail's intact prefix. | P1 | DF "Dispatch logs"; ER "Log format" | reader output; `damage()` | unit + fuzz/property |
| REQ-CONTRACT-21 | The shared-memory segment has DF "Main-processor shared-memory slots"'s layout, every attach rejects another version, layout or schema, and a claim records pid, start time and pid namespace so a reused pid holds none. | P1 | DF "Main-processor shared-memory slots" | `static_assert`; attach error; claim recovery | unit + integration |
| REQ-CONTRACT-22 | `ShmObjectName` maps names as ARCH "Topics and messages" states and is the one mapping publishers, subscribers, launcher, registry and Studio use. | P1 | ARCH "Topics and messages" | mapped names | unit |
| REQ-CONTRACT-23 | `HardwareLink::sample_time()` maps the gateway sample time into the node clock and is absent until the mapping converges, while stale or on a boot mismatch; signal acquisition times are absent for disconnected, unknown-age or unmapped samples (DF "Gateway clock accuracy"). | P1 | DF "Gateway clock accuracy"; NDG "Hardware", "Signal acquisition times"; HR "Runtime Contract" | `sample_time`, `*_time()` | unit + component |
| REQ-CONTRACT-24 | Teleop bindings are refused at construction unless well formed (NDG "Binding joystick buttons and axes via TOML"), a toggle resets on disable, leaving teleop and disconnect, and a released input sends `release()`. | P1 | NDG "Binding joystick buttons and axes via TOML" | constructor error; lane envelope | component |
| REQ-CONTRACT-25 | Every node publishes its `state` every tick, disabled included, with validity next to each value, never NaN or infinity, and a measurement carries `sample_time_ns` with its validity false while the time is unknown. | P1 | METH §4 §7 | `state` cadence and fields | component (matrix cases 2, 8; gap: TODO MATRIX-1) |
| REQ-CONTRACT-26 | The pose estimator publishes every tick, never gates on `robot_enabled`, integrates on sample time, rejects and counts invalid, non-finite, non-positive-std or unplaceable observations, holds the pose with `valid = false` on stale inputs, and latches `localized`. | P1 | NDG "Adding a pose estimator" | `/pose_estimator/state` | component |
| REQ-CONTRACT-27 | A camera observation is stamped at exposure minus a valid `latency_offset_us`, published invalid on underflow or a future stamp, status is published every node tick, and describing never opens a camera. | P1 | DF "Link rates and endpoints"; R26 "Nodes" | `/<camera>/state`, `/<camera>/status` | component |
| REQ-CONTRACT-28 | Telemetry pages stay within DF's telemetry caps, the producer refuses construction past a cap, and Studio's decoder accepts exactly those limits. | P2 | DF "Telemetry, Studio, and the radio boundary" | page sizes; Studio decode | contract |
| REQ-CONTRACT-29 | `graph.json` as the launcher writes it parses in `studio/src/system.ts`, and the checked-in fixtures are real launcher output. | P2 | LR "If you change the shape of `graph.json`" | `declared_test` | contract |
| REQ-CONTRACT-30 | Every State names its gateway build (program, revision, dirty), the decoder rejects an unknown program or a dirty byte other than 0/1, and `/hw/status` exposes it. | P2 | DF "UDP frame and payload sizes"; HR "RoboRIO" | `/hw/status` gateway fields | unit |

### RT: tick timing, drain semantics, realtime tiers, no allocation

| ID | Requirement | P | Source | Observable | Cheapest tier |
|---|---|---|---|---|---|
| REQ-RT-01 | Every node drains its topics immediately before each of its timers (`update()`, `every()`), so a timer sees everything published by its deadline whether or not the node has callbacks. | P0 | ARCH "Targets"; ER "The three loops" | value seen in the tick at the deadline | component |
| REQ-RT-02 | Latest-value subscriptions dispatch only the newest message per drain and count skips; callback subscriptions deliver in sequence up to the per-drain cap; between its timers a node is woken to drain only by a publication on a topic ARCH "Targets" says wakes it, by a writer taking over such a topic, or by its loop's wake backstop (at most once per `wake_backstop`, 1 s, while it waits on such a topic), and is otherwise asleep. | P1 | ARCH "Targets"; ER "The three loops" | `Context::dropped`; delivery count; `activity()` | component |
| REQ-RT-03 | Each dispatch samples its own `now`, frozen for the callback; an overdue periodic timer coalesces on its original phase and reports missed periods in `sequence`/`dropped`. | P1 | ARCH "Targets"; ER "The three loops", "Lateness" | `context.sequence`; counters | component |
| REQ-RT-04 | Dispatch is single-threaded and never re-entrant, expired timers run in (deadline, id) order before watched topics in registration order, and a timer change from another thread throws `std::logic_error`. | P1 | ER "Dispatch and replay" | dispatch order; exception | unit |
| REQ-RT-05 | `testing::Sim` dispatches nodes serially on one chronological frontier, so no node sees a publication from its future, same-instant nodes run in add order, and `pause`/`resume` stop one node's drains and timers while its peers advance. | P1 | ARCH "Targets" | ordering of published values | component |
| REQ-RT-06 | A C++ control node's `update()` and the bridge's `Tick()` allocate nothing on the loop thread in steady state. | P1 | RULES 3; METH §10; TEST | `AllocationProbe` count | component |
| REQ-RT-07 | `update()` p99 wall time stays within METH §10's budget. | P1 | METH §10 | callback duration distribution | perf |
| REQ-RT-08 | A process picks its own loop tier (bridge, control, observer) and runs the scheduling policy NDG "Realtime scheduling" gives that tier on its OS, unless `TALOS_REALTIME` or `--sim` says otherwise. | P1 | ARCH "Targets"; NDG "Realtime scheduling" | registry `loop_tier`; thread policy | unit + integration |
| REQ-RT-09 | Every RTMS ring and the loop stack are prefaulted and locked; a refused step is printed once and published in the registry row, the launcher warns for a bridge or control loop without a full grant, and `session_report --strict` fails it. | P1 | NDG "Realtime scheduling"; LR "Restarts" | stderr line; registry flags; report verdict | integration |
| REQ-RT-10 | Helper threads (capture, detection, recording) run below every loop thread via `ConfigureBackgroundThread`. | P1 | ARCH "Targets"; NDG "Realtime scheduling" | thread policy/QoS | component |
| REQ-RT-11 | Event-log capture never blocks, locks or allocates on the loop thread, and an exhausted pool stops capture permanently (`failed()`), leaving a prefix rather than a log with holes. | P1 | ER "Log format" | `failed()`, `capture_stopped()`; log contents | component |
| REQ-RT-12 | Publishers overwrite the oldest slot, so a stalled reader (logger, Studio bridge) never stalls a publisher, and a sequence watcher that was lapped reports the loss in `dropped`. | P1 | ER "Overflow"; DF "Main-processor shared-memory slots" | publisher progress; `dropped` | component |
| REQ-RT-13 | Bridge and gateway keep the link rates DF "Link rates and endpoints" states: commands on change or heartbeat only after an observed State and while a request is active, clock probes independent of enable, a State per gateway tick, and DS snapshots at once on a new mode. | P1 | DF "Link rates and endpoints" | frame cadence on the link | component |
| REQ-RT-14 | On the deployment host, a 5 ms control loop's tick lateness and missed periods leave margin under realistic observer and disk load. | P1 | TODO PLATFORM-1, RT-3, RT-4; BR "Link timing and fault injection" | `max_latency_ns`, timer `dropped` in a recorded session | perf (deployment host) |
| REQ-RT-15 | Target age at the selecting tick and `/hw/state` cadence pass `session_report`'s verdicts (LR "Session measurements") in a recorded session. | P1 | TODO TIME-3; LR "Session measurements" | `session_report` verdicts | perf (recorded session) |
| REQ-RT-16 | A driver-input to actuator path crosses at most two node hops. | P1 | METH §10 | declared graph | contract (gap: TODO LINT-2) |
| REQ-RT-17 | Node code takes time only from `node.now()` (no `steady_clock`, `time.time()`, sleeps) and makes no random decisions, so Sim and replay match the robot. | P1 | METH §9 | source scan | contract (gap: TODO LINT-1) |
| REQ-RT-18 | Observation never perturbs control: registry fields are lock-free atomics, readers take no lock, writers touch only their own slot, and the launcher's stall evidence is the dispatch count the loop thread stores itself, never the reporter's heartbeat. | P1 | IR "Observing must not perturb"; LR "Restarts" | `static_assert`s; no stall kill while the reporter is starved | unit + integration |
| REQ-RT-19 | Wake on publish: a prompt subscription's handler (a `subscribe` callback, a Driver Station listener) is run by a drain its publication wakes, with `latency()` counted from the publication; a publication with no armed reader makes no system call; and delivery never depends on a wake: a drain precedes every timer, a loop waiting on a prompt topic drains at least once per `wake_backstop`, a stalled arm cannot make it spin, a loop that cannot build its wake receiver falls back to polling, and a notification lost other than to a gone receiver is counted (`wake_failures`). | P1 | ARCH "Targets"; ER "The three loops"; RTMS "Wake channel" | `activity()` wakeups, drains, `stalled_arms`, `wake_fallbacks`; `wakes_sent()`, `wake_failures()`; `Context::event_time` | component |

### OPS: launcher supervision, restarts, sessions, logs/replay, observability

| ID | Requirement | P | Source | Observable | Cheapest tier |
|---|---|---|---|---|---|
| REQ-OPS-01 | The launcher probes every binary with `--describe`, lints the graph, writes `graph.json` and `declared_graph.json` atomically, and on any error spawns nothing and exits non-zero; a critical probe failure is fatal, a non-critical one a warning, and a failed spawn names its step and errno. | P0 | LR "Two phases", "When a binary does not answer", "graph.json" | exit code; files; stderr | integration |
| REQ-OPS-02 | A second launcher in the same shared-memory namespace is refused naming the holder's pid, and the lock is released when the holder dies. | P1 | LR "One session per namespace" | refusal text; exit | integration |
| REQ-OPS-03 | Every declared topic is created as a session-owned segment (schema, hash, alignment, slots) before any node spawns and unlinked at session end. | P1 | ARCH "Topics and messages"; LR "Session segments and logs" | `/dev/shm` contents | integration |
| REQ-OPS-04 | A non-critical member restarts at most `max_restarts` times per rolling window, then is quarantined with exactly one later retry; its clean exit is not restarted; nothing restarts once a stop has begun. | P1 | LR "Restarts" | manifest `quarantined`, `retry_at_monotonic_ns` | unit + integration |
| REQ-OPS-05 | A registered node whose dispatch count stops advancing for its heartbeat timeout is killed and restarted, a process with no registry row is killed after the startup grace, and construction past the construction deadline is killed (LR "Restarts"). | P1 | LR "Restarts"; ARCH "Layers" | `runs[].reason` `stalled` / `heartbeat_missing` / `construction_deadline` | integration |
| REQ-OPS-06 | SIGINT/SIGTERM stops a live node cleanly; a stop during construction is held until the constructor returns and the binary exits 0; a member still alive after the stop timeout is SIGKILLed. | P1 | NDG "Stopping"; LR "Session manifest" | exit status; reason | integration |
| REQ-OPS-07 | With `NOTIFY_SOCKET` set the launcher sends `READY=1` on entering supervision and `WATCHDOG=1` every loop iteration, and that loop waits on no lock a node can hold and never on disk. | P1 | LR "Restarts"; DEP "Supervision and service recovery" | datagrams on a test socket | integration |
| REQ-OPS-08 | `manifest.json` is written before spawn and replaced atomically at each lifecycle change by a writer thread, never the supervision loop, durably synced, with the per-node fields LR lists. | P1 | LR "Session manifest", "Session segments and logs" | manifest JSON | integration |
| REQ-OPS-09 | Each run writes its own `.tlog` (`<name>.tlog`, `<name>.restart<k>.tlog`) within its share of `log_max_bytes`, a restart keeps at least half its member's share, and a capped log keeps its tail in rotating whole segments. | P1 | LR "Restarts", "Session segments and logs"; DF "Dispatch logs" | files; manifest `log_capped` | component |
| REQ-OPS-10 | Every member's stdout and stderr land in `<session>/<name>.log` without the child ever waiting on the launcher's disk, with bounded buffering, counted drops and a per-session cap (LR). | P1 | LR "Node output" | log file; drop notice | integration |
| REQ-OPS-11 | Retention, run only by a lock-holding launch before it writes, removes sessions oldest first by class, keeps the protected matches LR states unless the free floor demands, records `retention.txt`, and never removes the current session, symlinks or foreign directories. | P1 | LR "Session segments and logs" | directory listing; `retention.txt` | unit + component |
| REQ-OPS-12 | A power loss mid-match leaves the last synced manifest and whole log segments, and readers skip a damaged tail with a warning. | P1 | LR "Session segments and logs"; ER "Log format"; DF "Dispatch logs" | reader output on truncated files | unit |
| REQ-OPS-13 | `--replay` re-runs a node on its own log and reports any send that differs from the recorded bytes, naming the dispatch and topic, and refuses a manifest mismatch. | P2 | ER "Recording and replaying" | divergence report | component |
| REQ-OPS-14 | `--replay-inputs <session> --only <node>` feeds every recorded input at its publication time from the session origin, touches no shared memory or hardware, fails on a schema mismatch naming both types, lists unrecorded topics, and writes identical logs on two runs. | P1 | METH §9; ER "Replaying another session's inputs"; LR "Replaying a session into one node" | output log bytes; error text | component |
| REQ-OPS-15 | `log_dump` merges every `*.tlog` of a session, filters by node, topic and time, decodes hardware packets with the wire codecs, labels devices only for a matching config id, and continues past a damaged log. | P1 | LR "Filtering and exporting logs" | dump output | component |
| REQ-OPS-16 | `session_report --strict` fails unless every required verdict LR "Session measurements" lists passes, and never treats NOT MEASURED as a pass. | P1 | LR "Session measurements" | exit code; verdicts | unit |
| REQ-OPS-17 | The registry registers identity before construction and endpoints only after the loop starts, frees a killed or abandoned claimant's row on the next claim, never lets a crash loop fill it, and refuses a node past its capacity (IR). | P1 | IR "Claiming a slot, and liveness", "Observing must not perturb", "Limits" | registry snapshot | integration |
| REQ-OPS-18 | A node's registry row reports which source each `Target` selected (or none), its event-log failed/capped state and its realtime outcome, without the hot path touching the segment. | P1 | IR "Who drives a subsystem"; ARCH "Targets" | `SelectionSnapshot`; flags | component |
| REQ-OPS-19 | The launcher prepares the registry before probing and refuses a launch, naming the file, owner and mode, when its nodes could not open it or when the roster does not fit beside live outside holders. | P2 | IR "Prepared by the launcher"; LR "Restarts" | refusal text | integration |
| REQ-OPS-20 | `/hw/status` carries link health, power and CAN health, motor fault and limit masks, gateway counters and build, and the clock offset with its uncertainty, at the rate DF states. | P1 | ARCH "Layers"; DF "Link rates and endpoints"; HR "Runtime Contract" | `/hw/status` fields | component |
| REQ-OPS-21 | `hardware_node --log` records the merged `/hw/command` and decoded `/hw/state`, prints the gateway build once per boot, and reports an undecodable State with one stderr line until one decodes. | P1 | HR "Runtime Contract" | log records; stderr | component |
| REQ-OPS-22 | The telemetry node publishes every state and status channel of the graph each tick, dictionary pages first and periodically, and refuses construction past its caps naming the largest sources. | P2 | DF "Link rates and endpoints", "Telemetry, Studio, and the radio boundary" | `/talos/telemetry` pages | component |
| REQ-OPS-23 | Studio shows each client whole producer ticks only, marks stale readings, continues history across a producer restart within a session, and holds all clients within DF's shared radio budget. | P2 | DF "Telemetry, Studio, and the radio boundary"; STU "Freshness" | WebSocket frames; UI state | component |
| REQ-OPS-24 | The RoboRIO publishes `companion/*` dashboard values and one DS message per change for each link condition HR "RoboRIO" lists. | P1 | HR "RoboRIO" | SmartDashboard values; DS messages | component |
| REQ-OPS-25 | The controller endpoint resolves `--remote` over `[robot] controller_address` and is passed to `hardware_node` explicitly, and a bundle records each node's binary, the config's hash and the source revision. | P1 | LR "Controller addressing", "Session manifest", "Building" | launcher output; manifests | unit + integration |

### DEV: scaffold, simulation, describe-only, standalone subsystem

| ID | Requirement | P | Source | Observable | Cheapest tier |
|---|---|---|---|---|---|
| REQ-DEV-01 | `new_subsystem` writes the package, subsystem TOML, `[[subsystems]]` entry and regenerated manifests with a Sim test skeleton, and the generated package builds and passes. | P1 | ARCH "Where a new thing goes"; METH §12; TEST; NDG "Adding a pose estimator"; RULES 10 | generated files; generated test result | contract |
| REQ-DEV-02 | `--standalone DIR` writes a Bzlmod repo that a robot consumes with `bazel_dep` + `local_path_override` and a subsystem file naming `@<name>//:node`. | P2 | ARCH "Where a new thing goes" | external build and bundle | integration |
| REQ-DEV-03 | Adding a node in one language changes no code in another; target policies, teleop bindings and telemetry are wired by TOML. | P1 | ARCH "Layers"; RULES 7; METH §1 | diff of a scaffolded addition | integration |
| REQ-DEV-04 | `launcher --describe-only` lints any config on a laptop with no hardware, writes nothing to shared memory and removes no sessions, and exits non-zero on errors; `talos_graph_test` runs it for every config in CI. | P1 | LR "Two phases", "Session segments and logs"; DESIGN | exit code | integration |
| REQ-DEV-05 | `sim.sh --sim-gateway` runs the full graph without WPILib against the ideal or `--physics` gateway, with scripted mode, joystick and fault schedules, and records the session under `/tmp/talos_logs/<id>`. | P1 | R26 "Run in simulation"; SR | session directory; behaviour | e2e |
| REQ-DEV-06 | `sim.sh` (WPILib) runs the real `Robot.cpp` under desktop simulation against the Bazel-built bridge and nodes, and every disable neutralizes. | P1 | R26 "Run in simulation"; TODO CI-4, FIELD-6 | requests and applied outputs | e2e |
| REQ-DEV-07 | `SimHardware` merges requests with the bridge's own rules (`//talOS/bridge:merge`) so `applied_*` equals what bridge and gateway apply, injects the documented faults (disconnect, reset, lost States, reboot, DS loss), and inspection never refreshes freshness. | P1 | NDG "Testing" | `applied_*` | component |
| REQ-DEV-08 | `PublishTarget` / `sim.publish_target` stamp the newest DS generation, or the `mode_since_us` given. | P1 | TEST; NDG "Binding joystick buttons and axes via TOML" | envelope header | component |
| REQ-DEV-09 | A C++ node exported with `TALOS_EXPORT_NODE` from a `talos_cc_shared_node` runs inside a Python node on that process's single framework copy, and listing an unexported framework target fails at analysis. | P2 | NDG "C++ nodes hosted from Python" | hosted node behaviour | integration |
| REQ-DEV-10 | The physics backend closes the Talon's control loops through gearing and inversion, enforces soft limits and followers, decays neutral per brake mode, and publishes truth on `/sim/state` with deterministic modelled errors. | P2 | HR "Simulation Backends: Ideal vs Physics-Backed" | backend state; `/sim/state` | component |
| REQ-DEV-11 | Under `--sim`, camera nodes render (`rasterize = true`) or project (`false`) observations from `/sim/state` truth and publish invalid observations without it. | P2 | R26 "Nodes"; HR "Ground truth and odometry error" | `/<camera>/state` validity | component |
| REQ-DEV-12 | `tools/setup.sh` installs a working dev environment on Linux, macOS and Windows inside WSL2. | P2 | AGENTS.md; INSTALL | `--check` result | integration |

## Package scope

Each package and the requirements it serves. A package or feature that
serves none is listed as such with the TODO.md item that settles it
([DESIGN.md §6](DESIGN.md#6-scope-serve-a-requirement-or-go)).

| Package | Serves |
|---|---|
| `talOS/rtms`, `talOS/memory`, `talOS/ipc` | REQ-SAFE-08, REQ-CONTRACT-07, REQ-CONTRACT-21, REQ-CONTRACT-22, REQ-RT-12, REQ-RT-19, REQ-OPS-03 |
| `talOS/events` | REQ-RT-01 to REQ-RT-05, REQ-RT-11, REQ-RT-19, REQ-CONTRACT-20, REQ-OPS-12 to REQ-OPS-14 |
| `talOS/node` | REQ-SAFE-06, REQ-SAFE-27, REQ-CONTRACT-18, REQ-RT-01, REQ-RT-05, REQ-RT-06, REQ-RT-19, REQ-DEV-08 |
| `talOS/devices` | REQ-SAFE-01 to REQ-SAFE-09, REQ-SAFE-12, REQ-SAFE-13, REQ-CONTRACT-23, REQ-DEV-07 |
| `talOS/bridge` | REQ-SAFE-11, REQ-SAFE-15, REQ-SAFE-16, REQ-SAFE-19, REQ-CONTRACT-04, REQ-RT-13, REQ-OPS-20, REQ-OPS-21 |
| `talOS/hardware` | REQ-SAFE-17 to REQ-SAFE-24, REQ-CONTRACT-02, REQ-CONTRACT-05, REQ-CONTRACT-06, REQ-CONTRACT-30, REQ-DEV-10 |
| `talOS/protocol` | REQ-CONTRACT-01, REQ-CONTRACT-03 |
| `talOS/driver_station` | REQ-SAFE-10, REQ-SAFE-11 |
| `talOS/configuration` | REQ-SAFE-26, REQ-CONTRACT-10, REQ-CONTRACT-17 |
| `talOS/launcher` | REQ-SAFE-28 to REQ-SAFE-30, REQ-CONTRACT-09, REQ-CONTRACT-11, REQ-CONTRACT-12, REQ-OPS-01 to REQ-OPS-11, REQ-OPS-14 to REQ-OPS-16, REQ-OPS-19, REQ-OPS-25, REQ-DEV-04 |
| `talOS/introspection` | REQ-CONTRACT-08, REQ-CONTRACT-09, REQ-RT-18, REQ-OPS-17, REQ-OPS-18 |
| `talOS/process` | REQ-RT-08 to REQ-RT-10, REQ-OPS-06, REQ-OPS-12 |
| `talOS/node_api`, `talOS/python` | REQ-SAFE-06, REQ-CONTRACT-13 to REQ-CONTRACT-15, REQ-DEV-09 |
| `talOS/schema`, `talOS/build` | REQ-CONTRACT-07, REQ-CONTRACT-08, REQ-CONTRACT-19, REQ-DEV-09 |
| `talOS/msgs`, `talOS/geometry` | REQ-CONTRACT-25 to REQ-CONTRACT-27 |
| `talOS/telemetry` | REQ-CONTRACT-28, REQ-OPS-22 |
| `talOS/teleop` | REQ-SAFE-13, REQ-SAFE-14, REQ-CONTRACT-24 |
| `talOS/trajectory` | REQ-RT-06 for the calls an `update()` makes (`sample`, `events_between`, `HolonomicFollower::step`); otherwise none in the catalogue (TODO.md SCOPE-3) |
| `talOS/mechanisms` | REQ-SAFE-01, REQ-SAFE-02, REQ-SAFE-04, REQ-SAFE-05 through the node test matrix, REQ-RT-06 |
| `talOS/vision` (CPU pipeline, capture, `field_sim`) | REQ-CONTRACT-27, REQ-DEV-11 |
| `talOS/vision/metal`, `talOS/vision/recording` | none in the catalogue (TODO.md SCOPE-1) |
| `talOS/aiming` | none in the catalogue (TODO.md AIM-1) |
| `2026-robot/main_processor/{shooter,drivetrain,intake}` | REQ-SAFE-01 to REQ-SAFE-09 through the node test matrix, REQ-SAFE-31, REQ-SAFE-32 (intake), REQ-CONTRACT-25 |
| `2026-robot/main_processor/driver` | REQ-SAFE-14, REQ-SAFE-31 |
| `2026-robot/main_processor/autonomous` | REQ-SAFE-31, REQ-SAFE-33, REQ-SAFE-34 |
| `2026-robot/main_processor/pose_estimator` | REQ-CONTRACT-26; its second backend: none beyond it (TODO.md SCOPE-1) |
| `2026-robot/main_processor/vision` | REQ-CONTRACT-27 |
| `2026-robot/main_processor/simulation` | REQ-DEV-05, REQ-DEV-06; composition evidence for REQ-SAFE-31, REQ-SAFE-33 |
| `2026-robot/main_processor/configuration`, `bench` | REQ-SAFE-26, REQ-CONTRACT-19; bench procedures for REQ-RT-14, REQ-RT-15 |
| `2026-robot/controller_processor/rio` | REQ-SAFE-10, REQ-SAFE-17 to REQ-SAFE-25, REQ-CONTRACT-06, REQ-OPS-24 |
| `studio` | REQ-SAFE-35, REQ-CONTRACT-28, REQ-CONTRACT-29, REQ-OPS-23 |
| `studio/agent` | none in the catalogue (TODO.md SCOPE-2) |
| `tools`, `examples` | REQ-CONTRACT-19, REQ-DEV-01 to REQ-DEV-03, REQ-DEV-12 |

