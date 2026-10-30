# Observing talOS from Python (supported surface)

Start with the bridge's plain JSON. It needs no agent and no dependencies
beyond the standard library — or just curl:

```sh
curl localhost:5800/telemetry.json   # latest sample: values, units, labels, stream
curl localhost:5800/channels.json    # channel dictionary: names, units, enum labels
curl localhost:5800/system.json      # nodes, topics, health
```

`/telemetry.json` carries `channels` (name → number; a channel with no
reading is absent, never zero-filled), `units`, `labels` (for enum channels
the value is an index into this list), `chassis`, `stream_id`,
`sequence_id`, and `timestamp_ns`. A new `stream_id` means the producer
restarted; sequence and timestamp restart with it.

Use the Studio agent's JSON-RPC service, implemented in `rpc.ts` and served
by `server.ts`, when you need history: retained samples, lookup by
timestamp, and threshold scans. Read `README.md` for the full method
reference; this note is the short Python path.

- Start: `bazel run //studio:agent` (defaults: upstream
  `ws://127.0.0.1:5800/telemetry`, serves `http://127.0.0.1:5802/rpc`).
- Transport: JSON-RPC 2.0 over `POST /rpc`. Batches and notifications
  (requests without `id`) work; notifications get no reply (`204`).
- `GET /health` reports upstream connectivity, retained frame count,
  jitter stats, and `system` / `declared` blocks. Poll it first.
- `get_latest` is the RPC form of `/telemetry.json`: the newest retained
  sample with its `units`, `labels`, `sequence_id`, `stream_id`, and age.
- Timestamps are **uint64 decimal strings** (never JSON numbers, which
  lose precision). `query_state_at` defaults to `mode: "exact"`;
  pass `"at_or_before"` for the preceding retained sample.
- Errors: `-32600` invalid request, `-32601` unknown method,
  `-32602` bad params (also unknown topic for `describe_topic`),
  `-32001` no retained sample at that timestamp,
  `-32002` graph or telemetry not received yet (bridge silent, or bridge
  started without `--declared` for `get_declared_graph`),
  `-32700` parse error.
- `-32002` is deliberately different from an empty graph: the first
  means "the agent has been told nothing", the second means "the
  bridge looked and nothing is running". Retry/back off on `-32002`;
  do not treat it as "robot stopped".
- The seven methods (`RPC_METHODS` in `rpc.ts`) are the whole surface:
  `get_latest`, `get_schema_tree`, `query_state_at`, `scan_channel_events`
  (telemetry: what was measured) and `get_system_graph`,
  `describe_topic`, `get_declared_graph` (registry: what is running
  and what was supposed to run).

## Examples (stdlib only: `urllib` + `json`)

- `examples/telemetry.py` — what was measured right now: reads
  `/telemetry.json` and `/channels.json` from the bridge directly and
  prints each channel's value with its unit or enum label.
- `examples/system_graph.py` — who is running: node counts, topic
  health tally, per-topic ends via `describe_topic`, declared-vs-live
  comparison via `get_declared_graph`.
- `examples/node_check.py` — the composed check you actually run:
  "is node N alive, what does it publish, and what are its latest
  values?" Combines `get_declared_graph`, `get_system_graph`,
  `describe_topic`, and `get_latest`.

The RPC examples take `--url` (default `http://127.0.0.1:5802/rpc`) and
exit non-zero with a one-line reason when the agent or bridge is not up
yet. `telemetry.py` takes `--bridge` (default `http://127.0.0.1:5800`).
