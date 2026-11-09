# Observing talOS from Python

Use the bridge's [HTTP JSON views](../bridge/README.md#http-and-websocket-views)
for snapshots and the agent's [JSON-RPC service](README.md) for retained history
and graph queries. The RPC reference documents startup, transports, methods,
timestamp encoding and errors.

## Examples

These scripts use the Python standard library:

- [`examples/telemetry.py`](examples/telemetry.py) reads the bridge directly and
  prints channel values (or enum labels) with units. `--bridge` defaults to
  `http://127.0.0.1:5800`.
- [`examples/system_graph.py`](examples/system_graph.py) prints the live graph's
  summary, topic detail and declared-vs-live comparison when available.
  `--topic` selects a topic; otherwise it describes the first graph topic.
- [`examples/node_check.py`](examples/node_check.py) takes a node name and
  reports its liveness, publications, source rates and retained channel values.
  If the node is absent, it consults the declared graph.

The RPC examples take `--url`, defaulting to `http://127.0.0.1:5804/rpc`.
They share [`examples/rpc_client.py`](examples/rpc_client.py), which sends a
JSON-RPC request with `urllib.request` and raises `RpcError` for RPC errors.

From the repository root, with the bridge and agent running:

```sh
python studio/agent/examples/telemetry.py
python studio/agent/examples/system_graph.py
python studio/agent/examples/node_check.py intake
```
