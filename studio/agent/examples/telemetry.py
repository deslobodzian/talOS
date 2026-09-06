#!/usr/bin/env python3
"""What was measured: schema bounds, latest snapshot, one threshold scan.

Calls get_schema_tree, query_state_at, and scan_channel_events.
"""
import argparse
import json

from rpc_client import RpcError, rpc


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:5802/rpc")
    parser.add_argument("--topic", default=None,
                        help="Numeric topic to scan (default: first number topic)")
    parser.add_argument("--above", type=float, default=40.0)
    args = parser.parse_args()

    schema = rpc(args.url, "get_schema_tree")
    bounds = schema["timestamp_bounds"]
    if not bounds:
        raise SystemExit("No retained telemetry; start the bridge first.")
    print(json.dumps({"frames": schema["frames"], "bounds": bounds}, indent=2))

    try:
        state = rpc(args.url, "query_state_at",
                    {"timestamp_ns": bounds["end"], "mode": "at_or_before"})
    except RpcError as e:
        if e.code == -32001:
            raise SystemExit("No retained sample at the latest bound.")
        raise
    print(json.dumps({"timestamp_ns": state["timestamp_ns"],
                      "sequence_id": state.get("sequence_id"),
                      "channels": list(state.get("channels", {}))[:10]}, indent=2))

    topic = args.topic or next(
        (t for t, ty in schema["topics"].items() if ty == "number"), None)
    if topic is None:
        print("No numeric topic retained; skipping scan.")
        return
    print(json.dumps(rpc(args.url, "scan_channel_events",
                         {"topic": topic,
                          "condition": {"op": "gt", "value": args.above}}),
                     indent=2))


if __name__ == "__main__":
    main()
