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

- `--config PATH`: read `[studio] max_clients` and `[studio] dashboard` (the
  driver dashboard's layout; see [Driver dashboard feed](#driver-dashboard-feed))
  from robot configuration, and each subsystem's `critical` for the drive
  summary (its roster too, when no `--manifest` is read).
- `--idle-timeout-s S`: a client's idle timeout, 8 to 960 s (default 16; see
  [WebSocket operating limits](#websocket-operating-limits)). Tests pass 8 to
  keep their runtime short.
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
| `/channels.json` | Dictionary: channel names, units, kinds, enum labels and alert levels | 503 until a dictionary arrives |
| `/telemetry.json` | Latest complete tick, keyed by channel name | 503 until a dictionary and a complete tick for it arrive |
| `/system.json` | Node registry and bridge statistics | Reports `registry.available: false` when the registry cannot be opened |
| `/declared.json` | Startup copy of the declared graph file | 404 if absent, empty or unreadable |
| `/manifest.json` | Current manifest file | 404 if absent or unreadable; 413 above 1 MiB (Studio's own manifest limit) |
| `/telemetry` | WebSocket: binary telemetry and text system graphs | Excess clients receive 503 |
| `/drive/layout.json` | The driver dashboard's layout (`DashboardLayoutJson`) | 404 without a layout |
| `/drive` | WebSocket upgrade: the [driver dashboard feed](#driver-dashboard-feed); any other GET: `drive.html` from the web directory | 404 without a layout (the upgrade); 404 without the file (the page) |

Every 200 carries `Cache-Control` (REQ-OPS-34):

| Response | `Cache-Control` | Why |
|---|---|---|
| A content-hashed asset: a file directly under `assets/` named `<name>-<hash>.<ext>`, the hash 8 characters of `A-Za-z0-9_-` (what Studio's Vite build emits) | `public, max-age=31536000, immutable` | its bytes never change under that name |
| Every other static file: `index.html` and `drive.html` however reached (`/`, `/index.html`, `/drive`, `/drive.html`), a favicon, an unhashed file under `assets/` | `no-cache` | revalidated on every load, so a deploy's new page, and the asset names it carries, is seen at once |
| The JSON endpoints (`/channels.json`, `/telemetry.json`, `/system.json`, `/declared.json`, `/manifest.json`, `/drive/layout.json`) | `no-store` | live snapshots with no validator to revalidate against, so never stored |

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
per channel `name`, `unit`, `kind` (`double`, `int`, `uint` or `bool`),
`labels` and `alert`: `none`, or for an alert channel (a `bool` status field
named `alert_<level>_<name>`, [NAMING.md](../../talOS/NAMING.md)) its level,
`info`, `warning` or `error`. A dictionary carrying an unknown alert level is
refused like any other undecodable dictionary.

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
counters, heartbeats, liveness, selected sources, topic verdicts). Each node
carries `delivery`, its registry row's wake loss and skipped requests
(`wake_failures`, `stalled_arms`, `wake_fallbacks`, `requests_skipped`;
[wake loss](../../talOS/introspection/README.md#wake-loss)), which Studio's
node card shows as a badge only when nonzero. Each
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
2.0 s for a 40 KB whole graph at a 40,000 B/s share). A deferred message
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

The order of the fast values is the order `BuildSystemGraph` appends them;
every reader of an update walks the document in that order (Studio's
`fastSlots` in [`system.ts`](../src/system.ts), and this directory's Python
harness, `fast_slots` in `integration_test.py`). Golden files,
[`testdata/golden`](testdata/golden), pin it: `:system_test` requires the
bridge to reproduce a graph, its next update and the graph after it byte for
byte, and `//studio:system_test` and `:fast_slots_test` require each reader
to turn the first graph and the update into the last. A reordered value fails
one of them instead of showing one counter's value as another's. For an
intended change, regenerate them and change both readers in the same commit:

```sh
TALOS_UPDATE_GOLDEN=1 bazel run //studio/bridge:system_test -- \
    --gtest_filter=SystemGraph.GoldenFilesHoldTheFastValueOrder
```

## WebSocket operating limits

Run one bridge per robot. Extra `/telemetry` upgrades past
`[studio] max_clients` receive HTTP **503 Studio client limit**,
`Retry-After: 5`, and `Studio client limit reached`. Driver dashboard clients
(`/drive`) are not counted against it and are never refused; they have their
own slots and budget ([Driver dashboard feed](#driver-dashboard-feed)). The
client limit and the egress byte budget are in
[Data flow](../../docs/DATA_FLOW.md#telemetry-studio-and-the-radio-boundary).

The bridge sends nothing to a client that already has buffered userspace
output, and never queues old observations for it: its telemetry waits and
resumes from the latest values. Each client's kernel send buffer holds a
quarter second of its share, at least 16 KiB
([Data flow](../../docs/DATA_FLOW.md#telemetry-studio-and-the-radio-boundary)),
so a client that stops reading backs up after that much of its output and
whatever its own receive window takes, and is held from then on, costing
the others no turn and the budget no byte. A client that stops answering
(a laptop that dropped off the network, or a stalled viewer) holds its slot
until the bridge's 16 s idle timeout closes it, at most 16 s after the last
data it sent; a client that only receives keeps its slot by answering the
bridge's pings, as browsers do (the same Data flow section gives the
derivation). [`client_timeout_test.py`](client_timeout_test.py) checks both
at the 8 s timeout. What its send
buffer already holds is stale when it resumes, so reconnect a stalled viewer
for fresh data.
The graph's bridge statistics include `clients`, `refused`, `ws_drops`,
`frames_forwarded` (complete producer ticks) and `egress_bytes`. `ws_drops`
counts only sends the socket refused (graph messages and telemetry pages); a
graph message or telemetry update the budget deferred is not a drop. The bridge's own
telemetry source counts as `dropped` the pages the publisher overwrote before
the bridge read them plus the ticks it discarded for a missing page. The
shutdown summary prints the cumulative counters, including `lapped`,
`held` (passes in which a client was sent nothing because its output was
still buffered) and `evicted` (drive clients a newer one replaced).

### Load probe

Rerun the real-IPC load probe (Linux RSS, source-monotonic frame age, no frame
format changes):

```sh
bazel build //studio/bridge:studio_bridge //studio/bridge:studio_mock
python studio/bridge/bandwidth_test.py --measure bazel-bin/studio/bridge --seconds 8
```

[`bandwidth_test.py`](bandwidth_test.py) checks client admission, the shared
egress budget, that the shared budget is granted in rotation (over one window
of producer ticks, two readers' updates differ by at most one, plus the turns
a reader was sent a graph message instead), and that a client which stops
reading with output buffered is held: the bridge counts it `held`, and it is
sent nothing while every reader is sent at least four updates. The test
fills that client's receive window with pongs to its pings, since macOS
gives a loopback client a window of about 320 KiB however small a buffer it
asks for.

## Driver dashboard feed

The driver dashboard (REQ-OPS-35, REQ-OPS-36) is enabled when `--config`
names a config whose `[studio] dashboard` names a layout that parses
([telemetry README](../../talOS/telemetry/README.md#dashboard-layout)). A
layout that does not parse is logged (`driver dashboard disabled: ...`) and
the bridge runs on without it. The code is [`drive.h`](drive.h).

**Endpoints**, on the same port as Studio:

- `GET /drive` without a WebSocket upgrade: `drive.html` from the web
  directory (404 when it has none).
- `GET /drive/layout.json`: the layout as `DashboardLayoutJson` writes it
  (shape in the telemetry README's `/drive/layout.json`). 404 without a
  layout.
- `ws://<host>:<port>/drive`: the feed. The upgrade answers 404 without a
  layout and is otherwise always accepted: never 503.

**Slots.** At most the layout's `drive_clients` feed connections are held.
They are not counted against `[studio] max_clients`, so programmer clients
can neither refuse nor displace one. A new connection past `drive_clients`
is accepted, and the oldest held one is closed with WebSocket close code
**4000** and reason **`replaced by a newer driver dashboard`**, so a laptop
that reconnects after a Wi-Fi drop always gets in, even while its dead
connection waits for the idle timeout. A page that reconnects by itself
after that close takes the slot back from the newer page in turn, so with
more pages open than `drive_clients` they keep replacing each other. Feed
clients have the same idle timeout and pings as `/telemetry`.

**Budget.** Each slot has its own token bucket of the layout's
`budget_bytes_per_s` with the 64,000 B burst, and a feed client is charged,
for every message with its WebSocket framing, to the bucket of the slot it
holds and to nothing else. A bucket belongs to the slot, not the
connection: a new connection takes the bucket of the slot it fills (the one
it evicted or one a closed client freed) as it stands, so reconnecting never
mints bytes, and every feed client together sends at most
`drive_clients × budget_bytes_per_s` plus `drive_clients` bursts however
often pages reconnect. That much is reserved out of the bridge's
160,000 B/s before programmer clients share the rest
([Data flow](../../docs/DATA_FLOW.md#telemetry-studio-and-the-radio-boundary)),
so a programmer client's traffic never defers a feed message. A message the
bucket cannot afford waits (telemetry coalesces into the next update) and
goes once the bucket holds it; telemetry construction holds the budget at
or above the feed's worst case, so that happens only while the bucket
refills after a dictionary.

A page that connects into a bucket the slot's previous connections drained
(pages that keep reconnecting) is accepted at once and then waits for its
dictionary. Meanwhile it hears `talos.drive.heartbeat` every 100 ms (36 B
each, affordable within 1.5 ms of being due at 24,000 B/s) and possibly a
summary; the dictionary pages follow, whole and in order, as soon as the
bucket holds each, then the keyframe. From an empty bucket that takes at
most about 0.8 s on the 2026 config at the default 24,000 B/s (its
11,496 B dictionary, a keyframe of at most 2,592 B and one 4,096 B
summary, framed, at the rate less the heartbeats). Text messages can
arrive between two dictionary pages; the binary pages stay in order. The
page shows that time as connected with no telemetry yet, never as a lost
link.

**Messages.** The bridge only sends; what a client sends is ignored (at most
1024 B a message). A message is binary or text by its WebSocket frame type.

*Binary: telemetry pages.* Each binary message is one TLM3 page, exactly as
on `/telemetry` and as Studio's `TelemetryDecoder` reads them: a 4-byte
little-endian length, then a FlatBuffers `Talos.Telemetry.Frame` with file
identifier `TLM3` ([`telemetry.fbs`](../../talOS/telemetry/telemetry.fbs)).

1. **Dictionary.** On connect, and whenever the producer's `stream_id` or
   `dictionary_id` changes, every dictionary page, in order: the whole
   dictionary, not only the masked channels, so a channel's index in a
   value page is its index in this dictionary.
2. **Updates.** Then value updates. An update is the value pages of one tick,
   `page` 0 to `page_count - 1`, all with that tick's `sequence_id` and
   `timestamp_ns`, sent back to back; apply it when its last page arrives.
   The first update after a dictionary has `keyframe` true on page 0: start
   from every channel absent. After it, an update carries in `channels` and
   `values` each masked channel whose value or presence changed, and in
   `absent` each masked channel that lost its sample; every page carries the
   tick's chassis pose. An update leaves the client holding the masked
   values of the tick it is stamped with.
3. **The mask.** Channel `i` of the dictionary is in the client's mask when
   its name is in `/drive/layout.json` `"channels"`, or when that document's
   `"alerts"` is true and the channel's `alert` is not `None`
   (`/channels.json` `"alert"` other than `"none"`). Only masked channels are
   ever carried or named absent. Every other channel, and a layout channel
   the dictionary lacks, is never sent: read it as absent, never as a value.
4. **Rate.** Updates that carry a change (a value, an absence, the chassis, or
   a keyframe) go at most `update_hz` a second, at least
   `ceil(1e9 / update_hz)` ns apart; changes in between coalesce into the
   next update.
5. **Heartbeat.** When nothing has been sent for 100 ms
   (`telemetry_wire::kHeartbeatIntervalNs`) and the client holds a tick, an
   empty update goes: `keyframe` false, no `channels`, no `absent`, stamped
   with the tick whose masked values the client holds. That is the newest
   tick when no masked value changed since, and the older one while a
   change waits for the update interval, so a `sequence_id` that keeps
   advancing means the telemetry the client shows is current, and one that
   stops means the telemetry is stale.

*Text: JSON objects*, told apart by `"kind"`:

- **`talos.drive.heartbeat`**: exactly `{"kind":"talos.drive.heartbeat"}`.
  Sent only while the client holds no telemetry tick (no telemetry yet, or a
  new dictionary before its keyframe, including one that waits for the
  slot's bucket), when nothing has been sent for 100 ms. Like any message it
  goes only when the bucket affords it, which a bucket refilling at the
  layout's rate does within `36 / budget_bytes_per_s` seconds.
- **`talos.drive.summary`**: what the `subsystems` widget needs that
  channels cannot give (a dead node raises no alert). Sent on connect and
  whenever it changes, at most once per 500 ms
  (`telemetry_wire::kDriveSummaryIntervalNs`), whole, at most 4,096 bytes
  (`kDriveSummaryMaxBytes`). It is rebuilt at each 250 ms graph refresh
  while a feed client is held, from the launcher's manifest (`--manifest`,
  re-read each time), the live registry and the config:

  ```json
  {"kind":"talos.drive.summary","session":"42","supervised":true,
   "registry":true,
   "nodes":[{"name":"hardware_node","alive":true,"critical":true,"restarts":0,
             "quarantined":false,"drives":null},
            {"name":"drivetrain","alive":true,"critical":true,"restarts":1,
             "quarantined":false,"drives":"teleop"}],
   "omitted":0}
  ```

  | Key | Type | Meaning |
  |---|---|---|
  | `session` | string | the launcher's session id (the manifest's `session_id`), else a live registry row's, else `""` |
  | `supervised` | bool | the manifest was read; when false, `restarts` 0 and `quarantined` false mean unknown, not none |
  | `registry` | bool | the live registry could be read at this refresh; when false, every `alive` false and `drives` null mean unknown, not dead |
  | `nodes` | array | with a manifest, every member of its `nodes` in its order but the Studio bridge itself (`studio_bridge`), so `sim_gateway` too when the launcher supervises one; without a manifest, or while it lists no member yet, every node the config launches: `hardware_node`, then each `[subsystems.<name>]` with a `node`, by name |
  | `nodes[].name` | string | the node name |
  | `nodes[].alive` | bool | a live registry row of that name exists (a node that never registered, or died, is false) |
  | `nodes[].critical` | bool | the config's `critical` for that subsystem (default true); true for a member the config does not list (`hardware_node`, `sim_gateway`) |
  | `nodes[].restarts` | number | the manifest's `restart_count` for the node |
  | `nodes[].quarantined` | bool | the manifest's `quarantined` for the node |
  | `nodes[].drives` | string or null | the target lane source (`teleop`, `auto`, ...) its first `Target` with a selection last selected; null when it is not alive, reads no target, or its `Target::get()` was last null |
  | `omitted` | number | nodes left off the end of `nodes` to keep the message within 4,096 bytes; normally 0 |

So a feed client hears something at least every 100 ms (plus the moments
its bucket needs for a 36 B heartbeat) while the bridge runs and its socket
drains: 500 ms of silence means the dashboard-to-bridge
link, a `sequence_id` that stops advancing means stale telemetry, and the
RoboRIO link is its own alert channel (`hw/status/alert_error_link_lost`).
`DriveFeed` in [`drive.h`](drive.h) is the feed, `//studio/bridge:drive_test`
its unit tests, and `integration_test.py`'s
`test_drive_feed_slots_and_layout` the slots, eviction and 404 over a socket.

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
instant the egress budget can afford a deferred send; for each drive client,
its heartbeat (100 ms), the end of its update interval while a change waits
for it, and the end of its 500 ms summary interval while a changed summary
waits. Idle and attached, with
no client, that is about six wakes a second: four refreshes, one segment check
and uWebSockets' own one-second date timer. A pass that stops at its page
budget, an arm that keeps finding pages (three tries), or a wake receiver that
cannot be made sends the bridge back every 1 ms instead. The shutdown summary
adds `wakes` (every return from the wait), `publication_wakes` and
`deadline_wakes`.
