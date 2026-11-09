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
bazel-bin/studio/bridge/studio_bridge --drop-newest-publisher /talos/telemetry 5800 studio/dist
```

## Arguments

`--drop-newest-publisher` is required before positional arguments. The optional
positions are topic (default `/talos/telemetry`), HTTP/WebSocket port (5800),
and static web directory (`studio/dist`).

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
| `/manifest.json` | Current manifest file | 404 if absent or unreadable; 413 above 32 MiB |
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
`age_ms` measures time since the bridge received the aligned frame; an old
finite value does not automatically become null as wall time passes.
The dictionary JSON has `kind: "talos.telemetry.channels"` and `version: 2`.

On WebSocket open the bridge attempts to send the current graph, then its
cached dictionary frame. These sends are budgeted and can be dropped. It
withholds value frames from clients missing the matching dictionary and retries
the dictionary on subsequent aligned telemetry frames. Cached dictionary values
retain their original timestamps; they are not fresh connect-time readings.
The bridge verifies and decodes frames for its JSON views before forwarding
aligned telemetry.

The system graph includes the bridge itself and refreshes every 250 poll ticks
(the timer is scheduled at 1 ms). It opens the registry lazily. Topic health is
computed in [`system.h`](system.h): missing ends are `bridged` when external,
otherwise `unconnected` when optional, otherwise `orphaned` or `unread`;
connected topics are `lossy` when drops are recorded, `idle` before publication,
and `ok` otherwise. Observer endpoints are listed separately from subscribers.

## WebSocket operating limits

Run one bridge per robot. `[studio] max_clients` defaults to **4**, with an
integer range of **1–64**. Extra upgrades receive HTTP **503 Studio client
limit**, `Retry-After: 5`, and `Studio client limit reached`.

All WebSocket sends share a **160,000 bytes/s** token bucket with a
**64,000-byte** burst. Each client also has a `160000 / max_clients` bytes/s
bucket with the same burst. Accounting includes conservative WebSocket framing.
HTTP snapshots and static files are outside this budget, as are other processes.

The bridge drops observations when budget is exhausted or a client already has
buffered userspace output; it does not queue old observations for that client.
A stalled client holds its slot until disconnected. Already accepted TCP bytes
can still be buffered, so reconnect a stalled viewer for fresh data.
The graph's bridge statistics include `clients`, `refused`, `ws_drops` and
`egress_bytes`; the shutdown summary prints the cumulative counters.

### Load probe

Rerun the real-IPC load probe (Linux RSS, source-monotonic frame age, no frame
format changes):

```sh
bazel build //studio/bridge:studio_bridge //studio/bridge:studio_mock
python studio/bridge/bandwidth_test.py --measure bazel-bin/studio/bridge --seconds 8
```

[`bandwidth_test.py`](bandwidth_test.py) checks client admission, the shared
egress budget and continued delivery to readers alongside a stalled client.

## Publisher and serving constraints

The bridge requires a dedicated telemetry topic whose producer uses
`OverflowPolicy::DROP_NEWEST`; RTMS does not store the overflow policy in shared
memory. Reader configuration does not change the producer's policy. Do not
attach the bridge to an overwrite/remove-slowest control-loop topic: the
producer could change bytes during a send. See
[DATA_FLOW](../../docs/DATA_FLOW.md#telemetry-studio-and-the-radio-boundary) for
frame sizes and [shared-memory geometry](../../docs/DATA_FLOW.md#main-processor-shared-memory-slots).

Start the publisher before the bridge and stop the bridge before replacing it.
Graceful SIGINT/SIGTERM releases the reader slot. Static files are read
synchronously and capped at 32 MiB each. The server has no authentication or TLS;
use it only on a trusted robot/development network.
