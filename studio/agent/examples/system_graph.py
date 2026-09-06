#!/usr/bin/env python3
"""Who is running: system graph query plus registry reads.

Calls get_system_graph, describe_topic, and get_declared_graph.
Exits non-zero with a one-line reason when the agent or bridge is silent.
"""
import argparse
import json

from rpc_client import RpcError, rpc


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="http://127.0.0.1:5802/rpc")
    parser.add_argument("--topic", default=None,
                        help="Topic to describe (default: first graph topic)")
    args = parser.parse_args()

    try:
        graph = rpc(args.url, "get_system_graph")
    except RpcError as e:
        if e.code == -32002:
            raise SystemExit("No system graph yet; is the Studio bridge connected?")
        raise
    print(json.dumps({"summary": graph["summary"], "version": graph["version"]},
                     indent=2))

    names = [t["name"] for t in graph["topics"]]
    if args.topic or names:
        detail = rpc(args.url, "describe_topic",
                     {"topic": args.topic or names[0]})
        print(json.dumps(
            {"topic": detail["name"], "health": detail["health"],
             "ends": detail["ends"]}, indent=2))

    try:
        declared = rpc(args.url, "get_declared_graph")
    except RpcError as e:
        if e.code == -32002:
            print("No declared graph (bridge started without --declared).")
            return
        raise
    print(json.dumps({"declared_summary": declared["summary"],
                      "comparison": declared["comparison"]}, indent=2))


if __name__ == "__main__":
    main()
