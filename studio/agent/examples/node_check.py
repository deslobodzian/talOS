#!/usr/bin/env python3
"""Composed check: is node N alive, what does it publish, latest values?

Combines get_declared_graph, get_system_graph, describe_topic, and
get_latest into the one report an agent or a student actually runs first
when something looks wrong.
"""
import argparse
import json

from rpc_client import RpcError, rpc


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("node")
    parser.add_argument("--url", default="http://127.0.0.1:5802/rpc")
    args = parser.parse_args()

    try:
        graph = rpc(args.url, "get_system_graph")
    except RpcError as e:
        if e.code == -32002:
            raise SystemExit("No system graph yet; is the Studio bridge connected?")
        raise
    live = next((n for n in graph["nodes"] if n["name"] == args.node), None)
    if live is None:
        try:
            declared = rpc(args.url, "get_declared_graph")
        except RpcError as e:
            if e.code == -32002:
                raise SystemExit(f"Node {args.node!r} is not registered "
                                 "(and no declared graph to compare against).")
            raise
        missing = [m["name"] for m in (declared["comparison"] or {}).get(
            "missing", [])]
        hint = "declared but never registered" if args.node in missing \
            else "not in the declared graph either"
        raise SystemExit(f"Node {args.node!r} is not running ({hint}).")

    published = [s["name"] for s in live["sources"] if s["kind"] == "SENDER"]
    report = {"node": live["name"], "alive": live["alive"],
              "events_per_second": {
                  s["name"]: s.get("events_per_second")
                  for s in live["sources"]},
              "publishes": {}}
    try:
        latest = rpc(args.url, "get_latest")
    except RpcError as e:
        if e.code != -32002:
            raise
        latest = None
    channels = (latest or {}).get("channels", {})
    for topic in published:
        detail = rpc(args.url, "describe_topic", {"topic": topic})
        value = channels.get(topic, "<not retained>")
        report["publishes"][topic] = {"health": detail["health"], "latest": value}
    print(json.dumps(report, indent=2, default=str))


if __name__ == "__main__":
    main()
