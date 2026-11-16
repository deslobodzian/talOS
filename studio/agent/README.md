# Headless JSON-RPC agent

For direct HTTP snapshots without an agent, see the
[bridge endpoints](../bridge/README.md#http-and-websocket-views). Use this service
for retained telemetry history and graph queries.

From the repository root, alongside the bridge:

```sh
bazel run //studio:agent
```

[`server.ts`](server.ts) defaults to upstream
`ws://127.0.0.1:5800/telemetry` (`TELEMETRY_URL`) and listens on
`127.0.0.1:5804` (`AGENT_PORT`). It owns a `TelemetryDecoder`, `JitterBuffer` and
bounded `Timeline`, independently of the browser. The transport-independent
query implementation in [`rpc.ts`](rpc.ts) is also used by the Studio worker.
It exports `dispatch`, `handleRpc`, `RPC_METHODS` and `RpcFault`.

## Transports

This is JSON-RPC 2.0, not MCP. POST requests to
`http://127.0.0.1:5804/rpc`, or send them over
`ws://127.0.0.1:5804/rpc`. Batches and notifications are supported; HTTP
notifications receive 204 and WebSocket notifications receive no reply.
A batch holds at most 16 requests (`MAX_BATCH` in [`server.ts`](server.ts));
a larger one is rejected whole with a single `-32600` error.
Requests are limited to 64 KiB. Requests with an Origin header or a Host other
than `127.0.0.1` or `localhost` (optionally with a port) are rejected.

`GET /health` returns upstream connectivity, retained frame count and jitter
statistics, plus separate `system` and `declared` blocks. The system block
reports received/rejected graph counts (a whole graph or an update to it),
deferral notices (`deferred`), node counts and registry availability.
The declared block reports whether a declared graph was received and its
comparison counts when available.

## Methods

Nanosecond timestamps are uint64 decimal strings. Query results use retained
samples without interpolation.

| Method | Parameters | Result |
|---|---|---|
| `get_latest` | `{}` | Latest snapshot, also under `snapshot`, plus `age_ms` and `age_ns` since timeline arrival; both are null when the latest sample is the keyframe the bridge sends on connect, whose values may be arbitrarily old |
| `get_schema_tree` | `{}` | Observed leaf paths and JavaScript types, direct channel aliases labelled with their kind (`double`, `int`, `uint`, `bool`; values as a snapshot spells them), retained bounds, frame count and timestamp encoding |
| `query_state_at` | `timestamp_ns`, optional `topics`, optional `mode` | Snapshot, or timestamped topic map; mode defaults to `exact`, with `at_or_before` selecting the preceding sample |
| `scan_channel_events` | `topic`, `condition`, optional inclusive `start_ns` / `end_ns` | Contiguous matching retained-sample ranges with inclusive endpoints and sample counts |
| `get_system_graph` | `{}` | Last received live graph, source event rates and topic-health summary, plus `age_ms` since the bridge last spoke about it and `stale` |
| `describe_topic` | `topic` | One graph topic, its health and named endpoints with counters and rates |
| `get_declared_graph` | `{}` | Declared nodes and diagnostics, summary and comparison against the live graph; comparison is null until a live graph arrives |

A snapshot includes `channels`, `kinds` (name → `double`, `int`, `uint` or
`bool`), `units`, `labels`, `sequence_id`, `stream_id` and `timestamp_ns`. A
channel value, in a snapshot or a `topics` map, is a number, except that an
`int` or `uint` value beyond ±2^53 − 1 (which a JSON number would round) is
its exact decimal string, and a NaN or infinite reading is the string
`"NaN"`, `"Infinity"` or `"-Infinity"`. A channel absent from the tick (its
topic was not published) has no entry. Channel aliases such as
`intake/state/current_a` can be used directly in `topics`; nested snapshot
fields use dot paths such as `chassis.x`.

Scan conditions have `op` and, for comparisons, `value`. Supported operators:
`gt`, `gte`, `lt`, `lte`, `eq`, `ne`, `changed`, `rising`, `falling`. Integer
channels compare exactly against an integer `value`. For `eq` and `ne`, a
`value` of `"NaN"`, `"Infinity"` or `"-Infinity"` names that reading, as
answers spell it, and `eq "NaN"` matches NaN. NaN matches no ordering
comparison, and NaN following NaN is not `changed`. Rising and
falling compare truthiness, not numeric slope. Edge comparisons use the preceding
retained sample even when it is before `start_ns`. Missing packets are not
inferred to match. No arbitrary expression evaluation is accepted.

Source rates are null until a rate can be measured. Graph queries return the
last received graph: the bridge's whole graph with its later value updates
applied ([bridge README](../bridge/README.md)). Its age counts from the last
graph message, including a notice that the radio budget deferred an update.
`get_system_graph` marks it `stale` once that is
`GRAPH_STALE_AFTER_MS` (1 s, [`freshness.ts`](../src/data/freshness.ts)) old;
a stale graph's `alive` flags are last-known, not current. The agent drops the
graph when the bridge connection closes.
For graph endpoints, health semantics and optional declared files, see the
[bridge reference](../bridge/README.md#http-and-websocket-views).

## Errors

| Code | Meaning |
|---|---|
| `-32700` | JSON parse error |
| `-32600` | Invalid request, empty batch or oversized batch |
| `-32601` | Unknown method |
| `-32602` | Invalid params, including unknown requested topics |
| `-32603` | Internal error |
| `-32001` | No retained sample at the requested timestamp/mode |
| `-32002` | Required telemetry, live graph or declared graph has not arrived |

`-32002` is not an empty graph or an empty channel set. In particular,
`get_declared_graph` cannot answer when no declared graph was offered by the
bridge; other methods remain usable.

See [OBSERVING.md](OBSERVING.md) for Python examples.
