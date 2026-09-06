# Architecture

This file is normative. It states the layers, what may depend on what, and
where a new thing goes. `PLAN.md` says what is being built next;
`talOS/NAMING.md` is the authority on names. Where this file and the code
disagree, the code is what runs — fix one of them, and say which in the commit.

None of the rules below is taste. Each exists because breaking it produced a
specific failure in this tree: a robot that did not move, a recording that
refused to replay, two topics that were both individually correct and never
met. The reason is given with the rule, because a rule whose reason has been
forgotten is a rule that gets relaxed.

## One robot

```
                      2026-robot/
  +--------------------------------------+       +-----------------------+
  | main_processor/  (Bazel, on talOS)   |       | controller_processor/ |
  |                                      |       |   rio/  (GradleRIO)   |
  |  arbiter   drivetrain   shooter      |  UDP  |                       |
  |  odometry  driver_station  telemetry |<----->|  hardware gateway     |
  |  operator_interface                  | 5802  |  Driver Station reads |
  |         |                            | 5803  |  Phoenix 6 backend    |
  |         | RTMS, shared memory        |       +-----------+-----------+
  |  //talOS/bridge:hardware_node -------+                   | CAN
  +---------+----------------------------+                   v
            | /talos_studio (RTMS)                       actuators
            v
     studio/bridge --+--> WebSocket + HTTP --> studio/src   (the viewer)
                     +--> studio/agent                      (JSON-RPC)
```

A robot is two processors and one wire contract between them. The main
processor holds all the logic and runs on talOS. The controller processor is
the HAL: it owns the actuators, and it is the only thing on the robot that
talks to a motor. Today that is a RoboRIO on GradleRIO/WPILib, derived from
`frc::RobotBase` rather than `frc::TimedRobot`, with no command-based framework
and no test suite — deliberately tiny, because everything it does is a thing
that cannot be tested off the robot, and because the smaller it is the cheaper
it is to replace.

That is the point of the split. Logic that lives on the controller can only be
exercised by deploying to hardware; logic that lives on the main processor can
be run, recorded and replayed on a laptop. So the boundary is drawn to leave
the controller with nothing but device access and the safety gates that must be
enforced closest to the devices, and a SystemCore or a Jetson becomes a new
directory beside `2026-robot/controller_processor/rio/` speaking the same
frames, with no change to `main_processor/`.

## The layers

Bottom to top. The label after each name is the Bazel target a reviewer can
check the edges of.

```
  studio/                       viewer, bridge, agent RPC
  ------------------------------------------------------------
  2026-robot/main_processor/    the node graph: one process per subsystem
  ------------------------------------------------------------
  talOS/bridge   talOS/launcher assembling and fronting a session
  ------------------------------------------------------------
  introspection  hardware       what is running / the device contract
  protocol       driver_station the UDP wire / the DS codec
  configuration                 robot.toml
  ------------------------------------------------------------
  talOS/events                  the CRTP loop, the manifest, the dispatch log
  ------------------------------------------------------------
  talOS/ipc                     typed publisher/subscriber over RTMS
  ------------------------------------------------------------
  talOS/rtms   talOS/memory     shared-memory ring buffers
```

The middle tier is not flat, and the BUILD files say so: `configuration`
depends on `hardware:config`, and `hardware:hardware` depends on `protocol`.
What makes them one tier is that they are all *contracts* — a device
description, a wire format, a naming grammar — that a node consumes and none of
them consumes a node.

**`talOS/memory` — `:shared_memory`, `:ring_buffer`.** `shm_open`/`mmap` with a
lifetime, and a ring buffer with no operating system in it. Separate from RTMS
so the ring buffer can be tested without a segment and the segment can be
tested without a protocol on it.

**`talOS/rtms` — `:rtms`.** The transport: one shared-memory ring per topic,
single producer, up to `MAX_READERS = 8` consumers, addressed by exact topic
string. Lock-free, no notification channel, no name service. Everything
uncomfortable about the layers above it follows from those last two facts.

**`talOS/ipc` — `:ipc`.** A typed publisher and subscriber over RTMS. It exists
because a node should not be writing raw slots, and it will likely fold into
`rtms/` eventually (`talOS/ipc/README.md` says so). Used directly only by
things outside the event loop: `talOS/bridge`, the monitors, `send_target`.

**`talOS/events` — `:events_core`, `:events_log`, `:events_os`, `:events`.**
The event loop, and the reason the rest of the design looks the way it does. A
loop dispatches timers and message arrivals single-threaded in a fixed order,
records every input and every output, and can replay the recording
byte-identically. `:events_core` is the header-only shared machinery —
`Registration`, `Manifest`, `SourceCounters`, the scheduler, the recorder
policy — with no OS calls in it, which is what lets a registry reader depend on
manifest types without linking a loop. `:events_os` is kqueue on macOS, epoll
plus timerfd on Linux, picked by a Bazel `select()` at build time so the
concrete poller is a compile-time type and not a virtual call. `:events_log` is
the on-disk format and its reader. `:events` is the three loops. Read
`talOS/events/README.md` before touching any of it; the seven rules there are
load-bearing.

**`talOS/introspection` — `:names`, `:describe`, `:registry`, `:reporter`.**
POSIX shared-memory objects cannot be enumerated, and a process holding one
open is invisible to everyone else. So "what is running, and who publishes
what" is not a question the transport can answer — the nodes have to say. This
layer is where they say it: `:registry` is the `/talos_registry.1` segment and
both ends of it, `:reporter` is the sampling thread a node runs to keep its row
current, `:names` is the naming grammar as code, and `:describe` is what a node
answers to `--describe` before it runs. It is above `events` and below
everything else because it depends only on manifest types.

**`talOS/hardware` — `:config`, `:hardware`.** The device contract: the config
manifest, the `State`/`Command` messages and their explicit little-endian
codecs, the gateway that applies a command as one whole-robot transaction, and
`Backend` as the platform boundary. `:config` is split out from `:hardware`
because `configuration` needs the config types and not the gateway.

**`talOS/protocol` — `:protocol`.** The 40-byte frame header, `UdpPeer`, and
the `FrameType` enumeration both ends agree on. Zero dependencies, on purpose:
both ends of the link include it, and one of them builds under GradleRIO.

**`talOS/driver_station` — `:driver_station`.** The Driver Station wire codec,
and nothing else. No dependencies, so it can be included by the RoboRIO program
and by a main-processor node from the same source.

**`talOS/configuration` — `:configuration`.** `robot.toml`: the parser,
`toml++`, and the validation. It assigns logical device ids by sorting every
device in the robot on (subsystem name, device name) and numbering from one, so
both ends agree and replay has a stable ordering.

**`talOS/bridge` — `:node_lib`, `:hardware_node`.** The one process that talks
to the controller. It pushes config, republishes `/hw/state`, subscribes to
every subsystem's request topic, and merges them into one command — because the
gateway validates and applies a command as a whole, so there cannot be two
producers of it. Note that it is *not* an event-loop node: it predates the loop
and drives its own tick, which is why it declares its introspection manifest by
hand in `HardwareNode::BuildIntrospectionManifest`.

**`talOS/launcher` — `:launcher_lib`, `:launcher`, `:log_dump`.** Reads
`robot.toml`, resolves each declared node's target to a binary, spawns them all
with a shared session id, and writes `manifest.json` naming every log.
`:log_dump` merges a session's logs into one time-ordered stream, refusing a set
whose session ids disagree.

**`2026-robot/main_processor/` — `//2026-robot/main_processor/<name>:node`.**
One process per subsystem. Subsystem nodes own devices and turn hardware into
meaning; every other node consumes meaning. Only a subsystem node reads
`/hw/state`.

**`studio/` — `//studio/bridge:studio_bridge`, `//studio:web_dist`,
`//studio:agent`.** Observation. The bridge maps the telemetry topic and the
registry read-only and serves both over HTTP/WebSocket; `studio/src/` is the
React viewer; `studio/agent/` is the same query surface over JSON-RPC so an
agent gets identical answers to the UI rather than a second implementation that
would drift.

One directory is in the tree and not in this stack. `tools/subsystem_codegen/` is a
`py_binary` named `subsystem_codegen` (`//tools/subsystem_codegen:subsystem_codegen`)
that nothing else builds against. Do not treat it
as an example of the architecture.

## Dependency rules

These are the rules a reviewer applies to a diff. Each is checkable from the
BUILD files alone.

**1. talOS never depends on a robot.** No target under `talOS/` may name a
target under `2026-robot/`, and none does. A second robot is therefore a new
sibling of `2026-robot/`, not a fork of the framework. `--config` is a
required flag on both binaries that take one (`talOS/launcher` and
`talOS/bridge`), precisely so the framework never mentions a robot at all:
the binaries work against any config path, and no default names one.

**2. talOS never depends on studio; a robot may.**
`//2026-robot/main_processor/telemetry` depends on `//studio/schema:wire` and
`//studio/schema:telemetry_cc`, because the slot geometry and the frame schema
are a contract between a producer and the bridge and there should be exactly
one copy of each. The edge runs robot → studio and never the other way.

**3. studio never depends on a robot.** `//studio/bridge:system` depends on
`//talOS/introspection:registry` and `:names`, and on nothing else; the viewer
renders whatever graph the registry reports. Robot target labels appear in
`studio/src/demo.ts` and in the bridge and viewer tests as *sample data*, which
is fine and is not a dependency — `//studio/bridge:system_test` also takes
`//talOS/hardware:hardware` so it can assert on the real `/hw/*` strings rather
than its own copies of them, which is a framework edge and still not a robot
one. A Studio feature that only works for `2026-robot` is a bug in that
feature.

**4. `introspection` depends only on manifest types.** `:registry` depends on
`//talOS/events:events_core` and `:names`, never on `//talOS/events:events`.
This is load-bearing rather than tidy: `studio/bridge` is a registry reader, and
a reader that pulled in an event loop would pull in RTMS, the poller and the
log writer to render a table. `:reporter` is a separate target for the same
reason — a reader never needs it.

**5. A node never depends on another node's internals.** It may depend on
another node's *published* topic constants and message types, and on nothing
else. `//2026-robot/main_processor/arbiter` depending on
`//2026-robot/main_processor/drivetrain` is correct: it reads
`talos::drive::kTargetTopic` and `talos::drive::ChassisTarget` so the two ends
of the topic cannot drift onto differently spelled strings. Reaching into
`DrivetrainNode` itself would not be, and neither would calling a method on
another node — the only thing that crosses between nodes is a message.

Put those constants in a small header (`<name>/packet.h`) rather than in
`node.h`. `2026-robot/main_processor/driver_station` puts them in `node.h`,
which is why every consumer of `/driver_station/state` includes the node class
to get a topic string. That is the shape to avoid.

**6. One writer per topic.** Not a style preference. The event loop records a
dispatch log and replay compares manifests (`compare_manifests` in
`talOS/events/manifest.h`), refusing a log whose shape does not match the
program. Two producers racing on one topic resolve in whatever order the
scheduler happened to pick, which means they resolve differently on replay than
they did on the field — so the recording of the failure you are trying to
reproduce is worthless. RTMS is single-producer anyway; the rule is about what
the *system* does, and `naming::LintGraph` reports two writers as an error and
names both. Fan-in is spelled with an arbiter and a prefix family:
producers publish to their own qualified topic, one node picks the winner and
publishes the unqualified topic, and the actuator subscribes only to that.

**7. Observability must not perturb the observed program.**
`introspect::Reporter` runs on its own `std::jthread` and registers nothing.
The obvious alternative — a timer on the node's loop — would put introspection
into the manifest, so every recording made with it on would refuse to replay
against a build with it off; worse, source ids are handed out in registration
order, so an extra source shifts the id of everything after it and invalidates
existing logs outright. Anything new that watches a running node inherits this
rule. The realtime loop's own message-poll timer is outside the source id space
(`INTERNAL_POLL_TIMER_ID`) for exactly the same reason.

**8. Name validation lives above the transport, not in it.** `rtms` takes a
string and opens a segment; it does not judge the string, and it must not start
to. It does enforce one thing, and the line between the two is worth being
precise about: `rtms::ValidatePath` rejects a name over 30 characters after the
leading slash, because macOS caps a POSIX shm name at `PSHMNAMLEN` and a longer
name is one the OS cannot open. That is a physical limit, not an opinion about
meaning. `naming::kMaxTopicBytes` is set to that same limit for the opposite
reason — a validator that accepts what the transport will reject is worse than
no validator, because it moves the failure from review to a throw inside a
node's constructor. `talOS/introspection/names.h` is deliberately dependency-light — names, no
transport — so the launcher can check a graph that does not exist yet and the
registry can check one that does, from the same source of truth. Pushing the
check down into RTMS would only catch the failure once a process is already
running with the wrong name, which is the case the check exists to prevent.

**9. No virtual functions on a dispatch path.** Loops are CRTP, recorders and
metrics are template policies, the platform poller is chosen at build time, and
callbacks are `{object, thunk}` pairs rather than `std::function`.
`simulated_event_loop_test.cc` asserts `!std::is_polymorphic_v<>` on every
loop; add the same assertion for anything new on the hot path.

**10. Fundamental mechanisms are implemented once, in C++; other languages
bind, never reimplement.** Transport, node lifecycle, registry/heartbeat, and
the shared-memory layout exist exactly once, under `talOS/`, behind the
`talos_*` C ABI (`talOS/node_api/node_api.h`). A wrapper language (Python
today: `talOS/ipc/python/node_api.py` as the binding, `rtms.py` as
ergonomics) may hold handles, translate errors, validate early, and offer
nicer classes — it may not reimplement a wire format, a layout, or a
protocol. User logic (a state machine in Python, Lua, whatever comes next)
sits on top of the wrappers and never touches the mechanism directly. Test
doubles are the one exception: they must be marked test-only, live beside
the test, and never touch real shared memory. A new language is a new
binding file against the same ABI, not a new implementation — so an internal
C++ change must never require a wrapper to be rewritten, which is what the
additive-only ABI policy is for (never renumber, remove, or re-signature an
export; only add). Reviewer check: no `mmap`/`struct` slot arithmetic outside
`talOS/rtms`, `talOS/memory`, and marked test doubles.

## The three loops

| Loop | Clock | Transport |
|---|---|---|
| `RealtimeEventLoop` | OS monotonic, via kqueue/epoll | RTMS shared memory |
| `SimulatedEventLoop` | Virtual, jumps to the next deadline | In-memory channels |
| `ReplayEventLoop` | Timestamps from the log | The log itself |

A node is written as `template <typename Loop> class XNode`, and this is not
generality for its own sake. It is the only way the *same code* runs in all
three: a node built on the simulated loop in a unit test, on the realtime loop
on the robot, and on the replay loop against a recording is one program with
one set of handlers, so a test that passes proves something about what the
robot does. An interface with three implementations would be a second thing to
keep in agreement, and would put a virtual call in the dispatch path — the two
costs the loop design exists to avoid.

Three consequences a node author has to live with:

- **Registration order defines source ids.** Reordering the `watch`/
  `make_sender`/`make_timer` calls in a constructor invalidates existing logs.
- **Handlers read time only through the context.** `context.now` is sampled
  once per dispatch and frozen, so two reads inside one handler cannot
  disagree. A handler that reaches for `std::chrono` is not replayable.
- **`--describe` is answered from the constructor alone.** A node built on a
  simulated loop touches no shared memory, no socket and no hardware, which is
  what makes probing a whole config cheap and safe. Anything a constructor does
  that is not registration breaks that.

## What crosses each boundary, and in what encoding

| Boundary | Encoding | Why |
|---|---|---|
| Node to node, one processor | RTMS shared memory: fixed-size FlatBuffers structs, or `hardware::Packet` bytes | Transport is a `memcpy`, so there is no serialization step on the hot path and replay can compare payloads byte for byte. Structs, never tables — `LoopMessage` enforces it. |
| Main processor to controller | UDP frames, `talOS/protocol` | 40-byte header plus `kMaxPayloadSize` bytes, sized so a whole frame fits one unfragmented 1500-byte-MTU datagram. Multi-byte fields are written little-endian explicitly by `EncodeFrame`; the C++ struct layout is never the wire layout, because one end is compiled by Bazel and the other by GradleRIO. |
| Robot to Studio | FlatBuffers `TLMS` frames on a dedicated RTMS topic, forwarded as WebSocket binary or UDP | High rate, fixed shape. The size and the zero-copy read pay for the hand-written offset verifier the receiver needs, because generated JavaScript accessors do not verify untrusted offsets. |
| Registry to anything | JSON, `GET /system.json` and a WebSocket text frame | A few times a second, structure-heavy, changing shape as nodes come and go, and read by people and agents as much as by the viewer. `JSON.parse` cannot be steered by a malformed offset, and no generated code has to exist for a topic table to gain a column. `curl localhost:5800/system.json` answers "what is running" with no build step and no client. |
| Node to launcher | JSON on stdout, `--describe` | Same argument, plus: pipe it to `jq`, hand it to an agent. `describe.h` carries a purpose-built parser for exactly this shape rather than a JSON library, and `describe_test` proves the writer and reader agree. |
| Agent to anything | JSON-RPC 2.0 over HTTP POST or WebSocket, loopback only | The query surface in `studio/agent/rpc.ts` has no transport and no Node dependencies, so the Studio worker answers the Agent tab from the identical code path. |

**uint64 values in JSON are decimal strings.** Sequence numbers, event counts,
pids, session ids, nanosecond timestamps. A JSON number is a double, so
anything past 2^53 comes back quietly wrong, and both event counters and
nanosecond wall clocks get there. Values that cannot approach it — message
sizes, capacities, client counts — stay numbers, so a reader does not have to
parse what cannot overflow.

**The overflow policy is part of the contract and not part of the segment.**
Control-loop topics use `OVERWRITE_OLDEST`, so a subscriber that stops reading
gets lapped rather than stalling a control loop; readers find out through
`Context::dropped`. The Studio telemetry topic is the exception: it must be a
dedicated topic with `DROP_NEWEST`, because the bridge sends from the slot and
a producer overwriting bytes mid-send would corrupt the frame. RTMS does not
store the policy in shared memory and so cannot verify this, which is why
`studio_bridge` takes `--drop-newest-publisher` as a required argument — an
acknowledgement of a contract the transport cannot check.

## Where a new thing goes

### A new subsystem

Everything is in one place, and nothing else changes. If a step here requires
editing the RoboRIO program, the hardware bridge, or another node, stop: that
is the invariant the whole design exists to protect.

1. `2026-robot/main_processor/<name>/subsystem.toml`: a `[subsystem]` block
   with `node = "//2026-robot/main_processor/<name>:node"`, `period_us`, its
   devices under `[motors.*]`, `[sensors.*]` and friends, and node-private
   values the RIO never sees. `2026-robot/main_processor/configuration/robot.toml`
   keeps `[robot]` and the subsystem manifest — the list of subsystem files —
   and nothing else per subsystem. The parser merges every file into one
   canonical view before validating, so exclusive ownership and the
   deterministic (subsystem, device) id ordering behave exactly as if it were
   one file: a second subsystem claiming one device is a parse-time error no
   matter which file the claim is written in.
2. `2026-robot/main_processor/<name>/packet.h`: the topic constants it owns,
   `inline constexpr const char* kStateTopic = "/<name>/state";` and so on.
   One declaration per topic, in the owning package.
3. `2026-robot/main_processor/<name>/<name>_message.fbs`: its message structs.
4. `2026-robot/main_processor/<name>/node.h`: `template <typename Loop> class
   <Name>Node`, registering its watchers, senders and timer in the constructor.
5. `2026-robot/main_processor/<name>/main.cc`: `kNodeName` and `kNodeTarget` as
   file-local constants written once, argument parsing, the three branches
   (`--describe` on a `SimulatedEventLoop`, `--replay` on a `ReplayEventLoop`,
   otherwise the realtime loop), `process::InstallStopHandlers()`, and an
   `introspect::Reporter` constructed **after** the loop so reverse destruction
   order joins its thread before the counters it samples.
   `2026-robot/main_processor/arbiter/main.cc` is the worked example.
6. `2026-robot/main_processor/<name>/node_test.cc`: the node on a
   `SimulatedEventLoop`.
7. `2026-robot/main_processor/<name>/BUILD`: a `flatbuffer_cc_library`, a
   `cc_library` named `<name>` with the headers, a `cc_binary` named `node`
   over `main.cc` depending on `//talOS/introspection:describe` and
   `//talOS/introspection:reporter`, and the `cc_test`. The rule must be named
   `node` and the package leaf must be `<name>`: `naming::CheckNodeTarget`
   requires the node name to match one or the other.

Adding a device renumbers logical ids — the parser sorts every device in the
robot on (subsystem, device) and numbers from one — so
`//2026-robot/main_processor/configuration:config_golden_test` will fail until
its reference table is updated. That failure is the feature: it is how an
accidental edit to the hardware declaration gets caught.

A node that owns no hardware declares no devices, only a `subsystem.toml`
with a `node` target and a period. It subscribes to
the *semantic* state topics that subsystem nodes publish and never to
`/hw/state`. `odometry/` and `arbiter/` are the worked examples.

Steps 1–7 are the subsystem package, and the package is the future repo
boundary: `node =` is already an arbitrary Bazel label, so a subsystem that
lives in another repository registers the same way
(`node = "@intake//:node"`, its `subsystem.toml` named in the manifest) with
no launcher change. Keep the layout standard — a second repo that renames
`packet.h` or folds `main.cc` into `node.h` is a fork, not a package.

The cross-language contract is the message schema, not the node. A subsystem
written in another language is a future package whose `.fbs` generates both
bindings and whose node speaks RTMS against the frozen shared-memory layout.
Until a non-C++ RTMS client exists, other languages reach the robot through
`studio/agent`'s JSON-RPC, which is observation only. Non-C++ nodes are
second-tier by construction: they cannot make the no-allocation and
byte-identical-replay claims, so a non-C++ node is tested on its I/O at the
topic boundary and is never counted toward a replay-equality claim.

### A new topic

`talOS/NAMING.md` first, then: declare it once, in the owning package's
`packet.h`, as an `inline constexpr const char*`. Consumers include that header
and alias the constant. A bare string literal at a call site is a defect even
when it is spelled correctly, because it is a second place for the name to live
and the linter cannot see it before the process runs.

If the far end will never be a shared-memory peer — the controller processor
over UDP, or a producer that is not written yet — say so, with
`kSourceFlagExternal` or `kSourceFlagOptional` in the node's `Reporter`
options. A graph that reports every deliberate dead end as a fault is a graph
people stop reading.

### A new message type

In the owning subsystem's `.fbs`, as a **struct**. Fixed size at compile time
is not negotiable: a slot has to be sized at registration, transport is a
`memcpy`, and replay compares payloads byte for byte. Carry what a consumer
needs to judge staleness — an `issued_ns` from the publisher's loop clock and
an `enabled` flag, as `ChassisTarget` does — because a consumer must be able to
treat a stale message as absent rather than as zero.

Variable-length payloads do not belong on a loop topic. The telemetry node is
the exception, and it pays for it: it owns its RTMS queue directly, outside the
loop, and declares that topic to the registry through
`Reporter::Options::extra` with its own counters, because a loop sender cannot
express a 64 KiB variable-length slot and registering one would copy every
frame into the dispatch log.

### A new framework primitive

Under `talOS/`, in the lowest layer that can hold it, as its own `cc_library`
target with its own test. Two questions decide the placement: what is the
lowest layer that has everything it needs, and does adding it force a layer
above to grow a dependency it did not have? `//talOS/introspection:names`
depending only on `//talOS/events:events_core` is the pattern — it could have
lived next to RTMS and would then have dragged the transport into the launcher.

If it touches a dispatch path, assert it is not polymorphic and measure it with
`//talOS/events:loop_perf`. Do not claim a latency number you have not
measured.

### A new viewer tab

1. `studio/src/tabs.ts`: one entry in `TABS`, in the order it should appear.
   The list is separate from the components so a view can link to another one
   without the shell and the views importing each other in a circle.
2. `studio/src/system_view.tsx` or a sibling: the component.
3. `studio/tests/view.test.tsx`: render it with `react-dom/server`. A
   typecheck and a successful bundle prove the code compiles; this proves it
   runs.
4. If it needs a new query rather than a new rendering of existing data, add
   the method to `studio/agent/rpc.ts` — which has no transport and no Node
   dependencies — so the Agent tab and the headless JSON-RPC service answer it
   from the same code.

A tab that needs data the registry and the telemetry frame do not carry needs
the *producer* changed, not the bridge: the bridge never parses FlatBuffers and
is not the place to compute anything.

### A new controller processor

A new directory beside `2026-robot/controller_processor/rio/`, speaking the
frames in `talOS/protocol` and implementing `Backend` from
`talOS/hardware/gateway.h`. It supplies a backend, a monotonic clock and an
enable source; it does not get to define its own frames, and
`2026-robot/main_processor/` does not change. It must hold the hard ceilings
itself — maximum supply and stator current, voltage, velocity — because a
pushed config is written by the main processor and a bug there must not be able
to over-current a motor. That is the safety boundary; review it as one.

If it is a simulation stand-in rather than real hardware, it belongs to the
robot and not to the framework: name it in `[robot].sim_gateway` in
`robot.toml`, as `//2026-robot/main_processor/drivetrain:sim_gateway` is today.

## Naming

`talOS/NAMING.md` is the authority, and `talOS/introspection/names.h` is the
same rules as code — where the prose and the header disagree, the header wins,
because the launcher and the test suite read the header and nothing reads the
prose. Do not restate the grammar anywhere else, including here: a second copy
drifts, and the drift is invisible until two people spell one topic two ways.

The one thing worth repeating, because it explains why a naming document is in
this architecture at all: a talOS topic is a POSIX shared-memory object
addressed by exact string, with no name service to resolve against and no type
check at the transport. Two spellings of one idea do not fail loudly. They
produce a publisher and a subscriber that are both individually correct, both
report healthy, and never meet. Naming is a structural concern here, not a
cosmetic one.

## Still moving

Stated so a reader can tell the design from the state of the tree:

- **The declared-graph check.** `talOS/introspection/describe.h` is the
  contract, and every node main answers `--describe` today — the framework
  bridge and all seven of `2026-robot/main_processor/`. What is not wired yet
  is the launcher end: probing every binary named in the config, assembling the
  declared graph, running `naming::LintGraph` and refusing to spawn on an
  error. `talOS/launcher/launcher.h` still goes straight from
  `DiscoverNodes` to `fork`/`execv`, so until that lands a mis-wired graph is
  found at runtime instead of before anything starts.
- **Topic migration.** `talOS/hardware/packet.h` is on the new naming
  (`/hw/command`, `/hw/state/driver_station`); the robot packages still carry
  `/hw/req/drive`, `/drivetrain/tgt` and `/odometry`. Renaming a topic
  invalidates every recording that mentions it, so this is being done
  deliberately rather than opportunistically.
- **Dropping GradleRIO.** The controller processor still builds through Gradle
  because that is what supplies WPILib and Phoenix 6 natively. Cross-compiling
  that one executable from Bazel is the outstanding job, and the RoboRIO
  program was cut to the bone specifically to make it small.
- **`talOS/ipc` folding into `talOS/rtms`.** Expected, per
  `talOS/ipc/README.md`. Do not build anything new that depends on the two
  being separate.
