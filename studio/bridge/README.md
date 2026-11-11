# Studio IPC bridge

Build from the monorepo root using Bazel:

```sh
bazel build //studio/bridge:studio_bridge //studio/bridge:studio_mock
bazel test //talOS/telemetry:wire_test //studio/bridge:system_test \
  //studio/bridge:telemetry_test //studio/bridge:integration_test
```

Build the frontend using the [Studio instructions](../README.md#run). In
separate terminals, from the repository root:

```sh
bazel-bin/studio/bridge/studio_mock /talos/telemetry
bazel-bin/studio/bridge/studio_bridge /talos/telemetry 5800 studio/dist
```

The mock publishes a tick every 5 ms; with `--on-stdin` it publishes one per
line read from stdin instead, so a test controls when the topic is idle.

## Arguments

The optional positions are topic (default `/talos/telemetry`), HTTP/WebSocket
port (5800), and static web directory (`studio/dist`). Once listening, the
bridge prints `listening <port>` on stdout; port 0 asks the OS for a free port,
which that line reports (how the bridge tests avoid racing other processes for
a port).

These named options are removed before positional parsing:

- `--config PATH`: read `[studio] max_clients` from robot configuration.
- `--declared PATH` or `--declared=PATH`: cache the file at startup for
  `/declared.json`.
- `--manifest PATH` or `--manifest=PATH`: read the file on each
  `/manifest.json` request.

`--describe` prints the node description and exits without opening shared memory
or sockets. In that mode, `--config PATH` supplies `[studio] topic`.
The manual runtime command still takes its topic positionally.

## HTTP and WebSocket views

| Endpoint | Contents | Unavailable response |
|---|---|---|
| `/channels.json` | Dictionary: channel names, units, kinds and enum labels | 503 until a dictionary arrives |
| `/telemetry.json` | Latest complete tick, keyed by channel name | 503 until a dictionary and a complete tick for it arrive |
| `/system.json` | Node registry and bridge statistics | Reports `registry.available: false` when the registry cannot be opened |
| `/declared.json` | Startup copy of the declared graph file | 404 if absent, empty or unreadable |
| `/manifest.json` | Current manifest file | 404 if absent or unreadable; 413 above 1 MiB (Studio's own manifest limit) |
| `/telemetry` | WebSocket: binary telemetry and text system graphs | Excess clients receive 503 |

For example:

```sh
curl localhost:5800/telemetry.json
curl localhost:5800/channels.json
curl localhost:5800/system.json
```

Telemetry JSON has `kind: "talos.telemetry"`, `version: 3`, `stream_id`,
`sequence_id`, `timestamp_ns`, `dictionary_id`, `age_ms`, `chassis` and
`channels`. Stream, sequence and timestamp uint64 values are decimal strings.
Each channel entry has `value` and `unit` (empty when unitless), plus `label`
for an in-range integer enum value. An integer value beyond ±2^53 is a decimal
string, so it stays exact; a NaN or infinite reading is the string `"NaN"`,
`"Infinity"` or `"-Infinity"`; a `Bool` value is `0` or `1`; a channel absent
in the tick (its topic was not published) is `null`. `age_ms` measures time since the bridge completed the
tick, on the bridge's monotonic clock, so a wall-clock step does not change
it; an old value does not automatically become null as time passes.
The dictionary JSON has `kind: "talos.telemetry.channels"`, `version: 3`, and
per channel `kind` (`double`, `int`, `uint` or `bool`).

The bridge verifies every page, assembles dictionaries and ticks, and applies
a tick only when all of its pages arrived. On WebSocket open it attempts to
send the current graph, then the client's telemetry feed starts: the dictionary
pages and a keyframe of the latest tick, then change-only updates
([telemetry README](../../talOS/telemetry/README.md#the-live-view-and-the-record)).
Each update goes out only when the budget affords all of it; otherwise it is
deferred and the next update carries the changes, so a slow or starved client
sees fewer, never inconsistent, ticks. A page the socket refuses resynchronizes
that client with a fresh keyframe.

The system graph includes the bridge itself and refreshes every 250 ms of
the monotonic clock (see [Waking](#waking)). `/system.json` serves the whole document,
with a `structure` id covering everything but its fast values (wall time,
counters, heartbeats, liveness, selected sources, topic verdicts). Each
WebSocket client (`GraphFeed` in [`system.h`](system.h)) is sent the whole
document on connect and when the structure changes (a node starts, stops or
restarts), and otherwise `{"kind":"talos.system_graph.update","structure":...,
"values":[...]}`: the fast values in document order, written over the
client's document, at most every 500 ms and only when they changed. A whole
document goes at most every 2 s (`GraphFeed::kResendNs`); structure changes
in between, as in a crash loop, coalesce into the next. Graph messages go
before telemetry (`ClientFeeds` in [`telemetry.h`](telemetry.h)): while one
waits for budget it reserves half of the client's refill
(`ClientFeeds::kGraphShare`), and that client's telemetry may spend only what
lies beyond the reserve, at most one update per 250 ms
(`ClientFeeds::kHoldNs`), so however large a telemetry update the message goes
within a bound ([Data flow](../../docs/DATA_FLOW.md#telemetry-studio-and-the-radio-boundary):
2.0 s for a 40 KB whole graph at the default share). A deferred message
coalesces into the next; while the client has heard nothing for 750 ms it is
sent
`{"kind":"talos.system_graph.deferred","structure":...}`, so Studio, which
marks the graph stale after `GRAPH_STALE_AFTER_MS`
([`freshness.ts`](../src/data/freshness.ts)) without a graph message, does
not mark a late graph stale. A whole graph larger than the egress burst
([Data flow](../../docs/DATA_FLOW.md#telemetry-studio-and-the-radio-boundary))
can never be sent: the bridge logs that once when the graph crosses the
burst, sends no notice (the client's graph goes stale, which is the truth),
and still serves it at `/system.json`. It opens the registry lazily. Topic health is
computed in [`system.h`](system.h): missing ends are `bridged` when external,
otherwise `unconnected` when optional, otherwise `orphaned` or `unread`;
connected topics are `lossy` when an end lost messages, `idle` before
publication, and `ok` otherwise. A topic's `dropped` sums every end except
latest-only watchers (source `latest_only: true`): the registry records one
drop count per source, and a latest-only watcher's mixes its intentional skips
with torn and slot-lost reads, so a 50 Hz reader of a 200 Hz topic is not
`lossy` and its own failed reads show only in its source's `dropped`. Which
drops are faults is in the
[event system's Lateness section](../../talOS/events/README.md#lateness).
Observer endpoints are listed separately from subscribers.

## WebSocket operating limits

Run one bridge per robot. Extra upgrades past `[studio] max_clients` receive
HTTP **503 Studio client limit**, `Retry-After: 5`, and
`Studio client limit reached`. The client limit and the egress byte budget are
in [Data flow](../../docs/DATA_FLOW.md#telemetry-studio-and-the-radio-boundary).

The bridge sends nothing to a client that already has buffered userspace
output, and never queues old observations for it: its telemetry waits and
resumes from the latest values. A stalled client holds its slot until
disconnected. Already accepted TCP bytes can still be buffered, so reconnect a
stalled viewer for fresh data.
The graph's bridge statistics include `clients`, `refused`, `ws_drops`,
`frames_forwarded` (complete producer ticks) and `egress_bytes`. `ws_drops`
counts only sends the socket refused (graph messages and telemetry pages); a
graph message or telemetry update the budget deferred is not a drop. The bridge's own
telemetry source counts as `dropped` the pages the publisher overwrote before
the bridge read them plus the ticks it discarded for a missing page. The
shutdown summary prints the cumulative counters, including `lapped`.

### Load probe

Rerun the real-IPC load probe (Linux RSS, source-monotonic frame age, no frame
format changes):

```sh
bazel build //studio/bridge:studio_bridge //studio/bridge:studio_mock
python studio/bridge/bandwidth_test.py --measure bazel-bin/studio/bridge --seconds 8
```

[`bandwidth_test.py`](bandwidth_test.py) checks client admission, the shared
egress budget, continued delivery to readers alongside a stalled client, and
that no reader gets less than half the best-served reader's frames.

## Publisher and serving constraints

The bridge reads a dedicated telemetry byte topic in sequence. The publisher
overwrites and never waits for it: a bridge that stalls is lapped, resumes at
the oldest intact page and counts the pages it missed, while publication
continues for every other reader. Each page is copied out of shared memory and
checked against the writer's position before it is ingested, so an overwrite
during the copy is retried, never applied. See
[DATA_FLOW](../../docs/DATA_FLOW.md#telemetry-studio-and-the-radio-boundary) for
page sizes and [shared-memory geometry](../../docs/DATA_FLOW.md#main-processor-shared-memory-slots).

The bridge may start before the publisher. It serves HTTP and the system graph
at once, checks for the topic's shared-memory segment every 100 ms and
attaches when it appears. It does not create the segment to wait on it: if the
name is unlinked between that check and the attach, the attach creates an
empty segment for an instant, sees the id change, closes it (removing it, as
its only holder) and retries at the next check. Once attached it
checks every second whether the name now holds a different segment (a new
launcher session, or one created after the old one's last holder closed it) and
re-attaches. A segment whose name was unlinked is kept until a new one appears.
Each wait, attachment and replacement is logged once to stderr.
Graceful SIGINT/SIGTERM releases the reader slot. Static files are read
synchronously on the thread that reads telemetry and capped at 4 MiB each
(413 above it), a few times the built Studio bundle's largest asset. The server
has no authentication or TLS; use it only on a trusted robot/development
network.

## Waking

The bridge does not poll. Its one thread sleeps in the uSockets loop until a
socket event (HTTP, a WebSocket message or a drained client socket), a
publication on the telemetry topic, a stop, or its next duty. SIGINT and
SIGTERM go through `talOS/process`'s stop handlers, whose stop pipe is in the
loop too, so a stop ends the wait at once however it lands. The reader slot names a
`talos::WakeReceiver` whose descriptor is in the loop, so a publication wakes
it at once
([wake channel](../../talOS/rtms/README.md#wake-channel)). Each wake is one
pass: drain the receiver, read the topic (at most 256 pages a pass), check the
segment and refresh the graph when due, send each client what is due, and arm
the wake. A timer is then set to the earliest duty: the 250 ms graph refresh,
the segment check (100 ms waiting, 1 s attached), and for each client its
graph update (500 ms) or deferral notice (750 ms), its telemetry heartbeat
(100 ms after its last update), the end of a telemetry hold (250 ms), and the
instant the egress budget can afford a deferred send. Idle and attached, with
no client, that is about six wakes a second: four refreshes, one segment check
and uWebSockets' own one-second date timer. A pass that stops at its page
budget, an arm that keeps finding pages (three tries), or a wake receiver that
cannot be made sends the bridge back every 1 ms instead. The shutdown summary
adds `wakes` (every return from the wait), `publication_wakes` and
`deadline_wakes`.
