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

## Arguments

The optional positions are topic (default `/talos/telemetry`), HTTP/WebSocket
port (5800), and static web directory (`studio/dist`).

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
| `/channels.json` | Dictionary: channel names, units and enum labels | 503 until a dictionary arrives |
| `/telemetry.json` | Latest dictionary-aligned frame, keyed by channel name | 503 until dictionary and aligned values arrive |
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

Telemetry JSON has `kind: "talos.telemetry"`, `version: 2`, `stream_id`,
`sequence_id`, `timestamp_ns`, `dictionary_id`, `age_ms`, `chassis` and
`channels`. Stream, sequence and timestamp uint64 values are decimal strings.
Each channel entry has `value` and `unit` (empty when unitless), plus `label`
for an in-range integer enum value. Nonfinite readings become `null`.
`age_ms` measures time since the bridge received the aligned frame, on the
bridge's monotonic clock, so a wall-clock step does not change it; an old
finite value does not automatically become null as time passes.
The dictionary JSON has `kind: "talos.telemetry.channels"` and `version: 2`.

On WebSocket open the bridge attempts to send the current graph, then its
cached dictionary frame. These sends are budgeted and can be dropped. It
withholds value frames from clients missing the matching dictionary and retries
the dictionary on subsequent aligned telemetry frames. Cached dictionary values
retain their original timestamps; they are not fresh connect-time readings.
The bridge verifies and decodes frames for its JSON views before forwarding
aligned telemetry.

The system graph includes the bridge itself and refreshes every 250 poll ticks
(the timer is scheduled at 1 ms). Every refresh is re-sent to every client:
Studio judges the graph stale from its arrival time, so the resend interval
must stay below Studio's `GRAPH_STALE_AFTER_MS`
([`freshness.ts`](../src/data/freshness.ts)). A graph larger than the egress
burst ([Data flow](../../docs/DATA_FLOW.md#telemetry-studio-and-the-radio-boundary))
can never be sent; the bridge logs that once when the graph crosses the burst,
counts each attempt in `ws_drops`, and still serves it at `/system.json`. It
opens the registry lazily. Topic health is
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

The bridge drops observations when budget is exhausted or a client already has
buffered userspace output; it does not queue old observations for that client.
A stalled client holds its slot until disconnected. Already accepted TCP bytes
can still be buffered, so reconnect a stalled viewer for fresh data.
The graph's bridge statistics include `clients`, `refused`, `ws_drops` and
`egress_bytes`; the bridge's own telemetry source counts as `dropped` both
frames the publisher overwrote before the bridge read them and telemetry frames
dropped on the way to clients. The shutdown summary prints the cumulative
counters, including `lapped`.

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
the oldest intact frame and counts the frames it missed, while publication
continues for every other reader. Each frame is copied out of shared memory and
checked against the writer's position before it is forwarded, so an overwrite
during the copy is retried, never forwarded. See
[DATA_FLOW](../../docs/DATA_FLOW.md#telemetry-studio-and-the-radio-boundary) for
frame sizes and [shared-memory geometry](../../docs/DATA_FLOW.md#main-processor-shared-memory-slots).

The bridge may start before the publisher. It serves HTTP and the system graph
at once, checks for the topic's shared-memory segment every 100 poll ticks and
attaches when it appears. It does not create the segment to wait on it: if the
name is unlinked between that check and the attach, the attach creates an
empty segment for an instant, sees the id change, closes it (removing it, as
its only holder) and retries at the next check. Once attached it
checks every 1000 ticks whether the name now holds a different segment (a new
launcher session, or one created after the old one's last holder closed it) and
re-attaches. A segment whose name was unlinked is kept until a new one appears.
Each wait, attachment and replacement is logged once to stderr.
Graceful SIGINT/SIGTERM releases the reader slot. Static files are read
synchronously on the thread that polls telemetry and capped at 4 MiB each
(413 above it), a few times the built Studio bundle's largest asset. The server
has no authentication or TLS; use it only on a trusted robot/development
network.
