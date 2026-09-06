#!/usr/bin/env python3
"""Headless, standard-library-only agent comparing retained odometry estimates."""
import argparse
import json
import math
import urllib.request


def rpc(url, method, params=None):
    payload = json.dumps({"jsonrpc": "2.0", "id": 1, "method": method, "params": params or {}}).encode()
    request = urllib.request.Request(url, data=payload, headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(request, timeout=5) as response:
        result = json.load(response)
    if "error" in result:
        raise RuntimeError(result["error"])
    return result["result"]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:5802/rpc")
    parser.add_argument("--threshold", type=float, default=0.25, help="Divergence threshold in meters")
    args = parser.parse_args()
    schema = rpc(args.url, "get_schema_tree")
    bounds = schema["timestamp_bounds"]
    if not bounds:
        raise SystemExit("No retained telemetry. Start the bridge and agent server first.")
    state = rpc(args.url, "query_state_at", {"timestamp_ns": bounds["end"]})
    ghosts = {ghost["name"]: ghost["pose"] for ghost in state["ghosts"]}
    raw = ghosts.get("RawOdometry")
    fused = ghosts.get("FusedPose", state["chassis"])
    if raw is None:
        raise SystemExit("RawOdometry ghost is missing from this producer.")
    distance = math.hypot(raw["x"] - fused["x"], raw["y"] - fused["y"])
    print(json.dumps({"timestamp_ns": state["timestamp_ns"], "divergence_m": distance, "exceeds_threshold": distance > args.threshold}, indent=2))
    # A producer-supplied divergence channel permits scanning every retained sample.
    for topic in ("odometry.divergence_m", "odometry.divergence", "divergence_m"):
        if topic in schema["topics"]:
            print(json.dumps(rpc(args.url, "scan_channel_events", {"topic": topic, "condition": {"op": "gt", "value": args.threshold}}), indent=2))
            break


if __name__ == "__main__":
    main()
