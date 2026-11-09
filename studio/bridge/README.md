# Studio IPC bridge

Build from the monorepo root using Bazel:

```sh
bazel build //studio/bridge:studio_bridge //studio/bridge:studio_mock
bazel test //talOS/telemetry:wire_test //studio/bridge:system_test \
  //studio/bridge:telemetry_test //studio/bridge:integration_test
```

The monorepo pins FlatBuffers, uWebSockets v20.51.0 and its uSockets
submodule. SSL and WebSocket compression are disabled for this local bridge.
The bridge forwards frames without parsing them; it also decodes the v2
feed to keep the latest dictionary and frame for the JSON views below.

Build `studio/dist` using the frontend instructions. In separate terminals:

```sh
bazel-bin/studio/bridge/studio_mock /talos/telemetry
bazel-bin/studio/bridge/studio_bridge --drop-newest-publisher /talos/telemetry 5800 studio/dist
```

The positional arguments are topic, HTTP/WebSocket port and static web
directory. `--declared PATH` and `--manifest PATH` may appear anywhere on the
line and do not shift them. Open `http://localhost:5800`; binary frames arrive
at `/telemetry`. The
frame schema and slot geometry live in `//talOS/telemetry` (`telemetry.fbs`,
`wire.h`). The mock emits v2 frames at 200 Hz — a dictionary frame first and
every 100th frame after it, value-only frames between — with nanosecond
timestamps and source sequence IDs. Sequence IDs advance even when a full
queue rejects a frame, exposing the gap downstream.

On connect, a WebSocket client gets the system graph as a text frame and then
the bridge's cached dictionary frame as a binary frame, so its first value
frame is already decodable. The cached frame carries the values it was
received with, however long ago; a client must not read them as current
(Studio treats the first binary message on a socket as of unknown age).
Two HTTP views describe the same feed as JSON:
`GET /channels.json` returns the dictionary document
(`talos.telemetry.channels`, version 2), and `GET /telemetry.json` returns
the latest frame self-described (`talos.telemetry`, version 2) — `channels`
is an object keyed by channel name, each `{"value", "unit"}` plus `label` for
an in-range enum value; uint64 as decimal strings, `NaN` (absent or stale on
the wire) as `null`, and `age_ms` since the bridge received the frame. Both answer **503**
until a dictionary (and, for `/telemetry.json`, an aligned frame) has
arrived: "not known yet" is not "the robot is doing nothing".

## Radio operating limit

Run exactly one bridge per robot. `[studio] max_clients = 4` is the default
(integer 1–64); the launcher forwards its config, and a manual bridge reads it
with `--config PATH`. Extra WebSocket upgrades receive HTTP **503 Studio client
limit**, `Retry-After: 5`, and `Studio client limit reached`.

The telemetry/pose binary stream, system graph text stream, and connect-time
snapshots share a **160,000 bytes/s** WebSocket egress token bucket with a
**64,000-byte** burst. Each client also has a `160000 / max_clients` bytes/s
bucket (64,000-byte burst), so one busy reader cannot monopolize the stream.
Bytes include conservative WebSocket framing; the shared bucket also bounds
reconnect bursts. The steady ceiling is **1.28 Mbit/s**, leaving headroom under
the whole-robot 4 Mbit/s field limit for IP/TCP/radio overhead, control and
other traffic. This is not a whole-robot bandwidth guarantee: static assets,
HTTP JSON requests, camera streams and other processes are outside this
WebSocket budget. Load Studio assets over wired/local networking before a
match; do not repeatedly fetch HTTP snapshots across the field radio.

When budget is exhausted or a client has pending userspace output, the bridge
drops that observation rather than adding old frames to its queue. A stalled
client retains its client slot until disconnected; it does not stall IPC
consumption or another reader. TCP may retain already accepted bytes for that
client, so a viewer that resumes after stalling should reconnect for fresh
data. `/system.json` and the system WebSocket graph expose `clients`,
`refused`, `ws_drops` (budget/backpressure drops), and `egress_bytes`
(cumulative accepted WebSocket bytes); counters are refreshed every 250 ticks
and also printed on shutdown.

Rerun the real-IPC load probe (Linux RSS, source-monotonic frame age, no frame
format changes):

```sh
bazel build //studio/bridge:studio_bridge //studio/bridge:studio_mock
python studio/bridge/bandwidth_test.py --measure bazel-bin/studio/bridge --seconds 8
```

The publisher emits 64 channels plus pose at 200 Hz, with a dictionary every
100 frames. The probe runs one viewer, four viewers, four plus an excess
connection, and three reading viewers plus one never-reading admitted viewer
and an excess connection. The stalled socket has a 1 KiB receive buffer.
Age excludes the intentionally cached first binary snapshot; egress measures
WebSocket bytes, not Ethernet/IP overhead.

Measured on the Linux development host, 2026-11-09, eight seconds per case:

| Admitted / attempted | Read egress/client (kB/s) | Accepted total (kB/s) | RSS (KiB) | Worst p99 / max age (ms) | Drops / refused |
|---|---:|---:|---:|---:|---:|
| 1 / 1 | 45.4 | 45.7 | 6712 | 1.61 / 2.58 | 984 / 0 |
| 4 / 4 | 24.8–45.5 | 154.6 | 6680 | 1.83 / 7.83 | 4100 / 0 |
| 4 / 5 | 24.8–45.5 | 159.1 | 6588 | 2.05 / 6.96 | 4232 / 1 |
| 4 (one stalled) / 5 | 44.7–47.6 | 165.1 | 6672 | 5.82 / 8.92 | 4424 / 1 |

Decimal kB/s; total is the graph's accepted-byte counter divided by elapsed
probe time, including bytes buffered for the stalled client. Counters can lag
one refresh. The initial burst explains short-run rates above the steady
per-client/aggregate rate. Before enforcement, the regression admitted five
clients, reported zero drops, and each reading client received 132.3 kB/s
(three reading clients alone consumed 3.18 Mbit/s, before the stalled/excess
connections and network overhead). Four such readers exceed the whole-robot
4 Mbit/s allowance. The same regression now checks refusal, drops, accepted
bytes bounded by `160000 * elapsed_seconds + 64000`, p99 age <100 ms and max
age <500 ms despite the admitted stalled socket. These are host-loopback
measurements, not field-radio or macOS qualification; repeat the probe over
the actual robot radio with other robot traffic before competition.


## The system graph

Telemetry says what one node chose to publish. It does not say what is running,
and a subscriber waiting on a topic nothing publishes looks identical from the
inside to a healthy one. So the bridge also serves the live node registry: it
maps `/talos_registry.3` read-only, rebuilds a JSON document from it every 250
ms, and sends that document two ways. WebSocket clients on `/telemetry` get it
as a **text** frame, so a client tells the two message families apart by frame
type rather than by sniffing bytes. `GET /system.json` returns the same
document, which is what makes `curl localhost:5800/system.json` a complete
answer to "what is running" with no build step and no client.

The registry is opened lazily and reopened on each refresh until it appears, so
the bridge can be started before the robot — a viewer that refuses to run until
the nodes are up is a viewer you cannot use to watch them come up. Until then
the document reports `registry.available` as false with empty node and topic
lists, which is a different statement from an empty system. A newly connected
WebSocket client is sent the current document immediately rather than waiting
for the next refresh.

Each node carries `selections`: one entry per `talos::Target` it opened, as
`{"sources": ["teleop", "auto"], "selected": "teleop"}`, with `selected: null`
when nobody drives the subsystem (robot disabled, Driver Station stale, or no
live lane in the mode). The system view shows it on the node card as
"driven by". A node without Targets sends an empty list, and a viewer reads a
missing field — an older bridge — the same way, so this needed no
`kSystemGraphVersion` bump.

This one is JSON while telemetry frames are FlatBuffers, and the difference is
not inconsistency. Telemetry is high rate and fixed shape, where the size and
the zero-copy read pay for a hand-written offset verifier on the receiving end,
because the generated JavaScript accessors do not verify untrusted offsets. The
system graph is the opposite: a few times a second, structure-heavy, nested,
changing shape as nodes come and go, and read by people and agents as much as by
the viewer. `JSON.parse` cannot be steered by a malformed offset, so the
receiver needs field checks and collection bounds and nothing more, and no
generated code has to exist for a topic table to gain a column.

uint64 values are decimal strings — sequence numbers, event counts, pids,
session ids, nanosecond timestamps. A JSON number is a double, so anything past
2^53 would come back quietly wrong, and event counters and nanosecond wall
clocks both get there. Values that cannot approach it, such as message sizes,
capacities and client counts, stay numbers so a reader does not have to parse
them. This is the same rule the telemetry timestamps already follow.

The bridge reports itself in the document as well: its topic, connected client
count, frames forwarded, and the malformed envelopes and rejected WebSocket
sends from its shutdown summary. It holds a reader slot on the
telemetry topic and it is where frames go missing when a client cannot keep up,
so it is a participant in the system rather than a neutral observer of it.

### Topic health

Every topic row carries a `health` string, and every source and topic carries
`external` and `optional` booleans taken from the flags the owning node
declared (`talOS/introspection/names.h`). The verdicts, in the order they are
tried:

| `health` | Meaning |
| --- | --- |
| `bridged` | An end is absent and was declared `external`: it lives outside talOS. |
| `unconnected` | An end is absent and was declared `optional`: absence is the design. |
| `orphaned` | Subscribed, and nothing in this system publishes it. |
| `unread` | Published, and nothing in this system reads it. |
| `lossy` | Both ends present, and the transport has dropped messages. |
| `idle` | Both ends present, and nothing has been published yet. |
| `ok` | Both ends present, traffic moving, nothing dropped. |

`bridged` and `unconnected` exist because the two states below them used to
absorb topics that were never broken. `/hw/command` is consumed by the RoboRIO
across the UDP link and will never have a shared-memory subscriber. Reporting a designed
dead-end as a fault trains people to ignore the report, which is worse than not
having one — so those get their own names and `orphaned` and `unread` keep
meaning exactly what they meant. `external` outranks `optional` when a source
declares both, because it is the more specific claim: it says where the far end
is rather than only that it may be absent.

This is why `version` is now 2. The fields are additive, but a reader that
classifies topics itself would go on calling `/hw/command` orphaned, so the bump
is what lets it refuse the document instead of drawing a fault that is not
there.

### The declared graph

The launcher probes every node with `--describe` before it spawns anything and
writes the resulting *declared* graph to `<output_dir>/<session_id>/graph.json`. Pass that
path as `--declared PATH` and the bridge serves the file at
`GET /declared.json`.

This is the other half of the comparison a viewer needs. The registry only ever
holds what did start, so a node's absence from `/system.json` says nothing on its
own — it distinguishes "a node crashed before it registered" from "a node is not
part of this session" only against the list of what was meant to run.

The file is read once at startup and served verbatim; the bridge has no JSON
parser and does not need one, and re-emitting bytes unchanged is the one
transformation that cannot be subtly wrong. A missing or unreadable path is
logged once and the bridge starts anyway: a viewer that cannot show
declared-against-actual is a much smaller failure than a bridge that will not
start. With no `--declared`, or with a file that could not be read,
`GET /declared.json` is **404** — an empty document would say the session has no
nodes in it, which is a different and false statement.

Sim sessions normally get this for free: the launcher's `--start-studio-bridge`
(which `sim.sh` passes unless `--no-studio`) declares the bridge in `graph.json`
and starts it supervised with that graph as `--declared`. A manually started
bridge is undeclared unless `--declared` points at the session's `graph.json`.

### The session manifest

The launcher also writes a session `manifest.json` beside `graph.json` — the
session id, config hash, output dir, and per-node `restart_count`, `exit_code`
and log paths — and keeps it current as nodes start, restart and exit. Pass that
path as `--manifest PATH` and the bridge serves the file at
`GET /manifest.json`.

Unlike `/declared.json`, the manifest is re-read on **every request**: the
launcher atomically replaces it throughout the session, so this view is live
rather than a startup snapshot. Served verbatim with the same content type and
size cap as the static file handler; with no `--manifest`, or with a file that
could not be read, `GET /manifest.json` is **404** — an empty document would say
the session ran no nodes, which is a different and false statement. The
launcher's `--start-studio-bridge` passes the session manifest the same way it
passes the declared graph, so supervised Studio sessions get it for free.

## Dedicated DROP_NEWEST publisher contract

Use a **dedicated telemetry topic**, with exactly one producer, 64 slots of
16,384 bytes, alignment 8, and `OverflowPolicy::DROP_NEWEST`. Each slot starts
with a four-byte little-endian FlatBuffers size prefix, followed by the `TLM2`
FlatBuffer. The bridge transmits prefix plus payload, excluding slot padding.
Value-only frames are limited to 1,200 bytes including the prefix (the radio
budget in `talOS/telemetry/README.md`); dictionary frames may use the whole
slot.

The required `--drop-newest-publisher` argument acknowledges a contract that
RTMS cannot verify: the overflow policy is not stored in shared memory. Merely
setting DROP_NEWEST in the reader does **not** change the producer's policy.
OVERWRITE_OLDEST and REMOVE_SLOWEST_READER are unsafe here: a producer could
modify slot bytes while a network send is using them. Never attach this bridge
to an existing control-loop topic that uses those policies.

The producer must treat BUFFER_FULL as a dropped telemetry frame and continue
the control loop without waiting. DROP_NEWEST protects unread bytes during the
reader callback; the cursor advances only after synchronous send calls return.
This avoids an application-side staging copy or deserialization. Kernel socket
buffers and uWebSockets backpressure buffers still copy bytes; this is not
kernel zero-copy networking.

The publisher must start first and remain alive while the bridge runs. Stop the
bridge before restarting/replacing the publisher. Graceful SIGINT/SIGTERM frees
the reader slot; a SIGKILL/crash can strand a reader and cause subsequent writes
to be dropped. Restart both processes after such a crash. Shared-memory names
are local and limited by RTMS to 30 characters after the initial slash.

Slow WebSocket clients skip new frames once buffered bytes exceed 256 KiB, with
a 512 KiB uWebSockets backpressure cap. The shutdown summary reports malformed
envelopes and rejected WebSocket sends. HTTP static file reads are synchronous
and capped at 32 MiB per file; serving large assets during streaming can
increase jitter. The server has no authentication or TLS and is intended for
the local robot/development network. The bridge runs on Linux and macOS.
