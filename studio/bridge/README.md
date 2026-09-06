# Studio IPC bridge

Build from the monorepo root using Bazel:

```sh
bazel build //studio/bridge:studio_bridge //studio/bridge:studio_mock
bazel test //studio/schema:wire_test //studio/bridge:system_test \
  //studio/bridge:integration_test
```

The monorepo pins FlatBuffers, uWebSockets v20.51.0 and its uSockets
submodule. SSL and WebSocket compression are disabled for this local bridge.
The bridge itself never parses FlatBuffers.

Build `studio/dist` using the frontend instructions. In separate terminals:

```sh
bazel-bin/studio/bridge/studio_mock /talos/telemetry
bazel-bin/studio/bridge/studio_bridge --drop-newest-publisher /talos/telemetry 127.0.0.1 5801 5800 studio/dist
```

The positional arguments are topic, destination IPv4 address, UDP destination
port, HTTP/WebSocket port and static web directory. `--declared PATH` may appear
anywhere on the line and does not shift them. Open `http://localhost:5800`;
binary frames arrive at `/telemetry`. The mock emits deterministic 200 Hz data
with nanosecond timestamps and source sequence IDs. Sequence IDs advance even
when a full queue rejects a frame, exposing the gap downstream.

## The system graph

Telemetry says what one node chose to publish. It does not say what is running,
and a subscriber waiting on a topic nothing publishes looks identical from the
inside to a healthy one. So the bridge also serves the live node registry: it
maps `/talos_registry.1` read-only, rebuilds a JSON document from it every 250
ms, and sends that document three ways. WebSocket clients on `/telemetry` get it
as a **text** frame, so a client tells the two message families apart by frame
type rather than by sniffing bytes. The desktop UDP path has no such framing, so
there it is a datagram prefixed with the four bytes `TSYS`; telemetry frames
begin with a FlatBuffers size prefix and are sent unchanged. `GET /system.json`
returns the same document, which is what makes `curl localhost:5800/system.json`
a complete answer to "what is running" with no build step and no client.

The registry is opened lazily and reopened on each refresh until it appears, so
the bridge can be started before the robot — a viewer that refuses to run until
the nodes are up is a viewer you cannot use to watch them come up. Until then
the document reports `registry.available` as false with empty node and topic
lists, which is a different statement from an empty system. A newly connected
WebSocket client is sent the current document immediately rather than waiting
for the next refresh.

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
count, frames forwarded, and the malformed envelopes and rejected UDP and
WebSocket sends from its shutdown summary. It holds a reader slot on the
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
across the UDP link and will never have a shared-memory subscriber;
`/hw/command/override` is a debug hook nothing publishes. Reporting a designed
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
writes the resulting *declared* graph to `<output_dir>/graph.json`. Pass that
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

## Dedicated DROP_NEWEST publisher contract

Use a **dedicated telemetry topic**, with exactly one producer, 256 slots of
65,536 bytes, alignment 8, and `OverflowPolicy::DROP_NEWEST`. Each slot starts
with a four-byte little-endian FlatBuffers size prefix, followed by the `TLMS`
FlatBuffer. The bridge transmits prefix plus payload, excluding slot padding.
Datagrams are limited to 65,507 bytes including the prefix. Smaller frames are
recommended because large UDP datagrams fragment and loss discards the frame.

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

UDP is nonblocking. Slow WebSocket clients skip new frames once buffered bytes
exceed 256 KiB, with a 512 KiB uWebSockets backpressure cap. The shutdown summary
reports malformed envelopes and rejected UDP/WebSocket sends. HTTP static file
reads are synchronous and capped at 32 MiB per file; serving large assets during
streaming can increase jitter. The server has no authentication or TLS and is
intended for the local robot/development network. The current POSIX daemon runs
on Linux/macOS; the desktop frontend has a separate cross-platform Rust path.
