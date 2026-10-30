#!/usr/bin/env python3
"""What was measured right now, straight from the bridge.

Reads GET /telemetry.json and /channels.json (stdlib only, no agent):
the latest sample's channels with their units and enum labels.
"""
import argparse
import json
import urllib.request


def get(base, path):
    with urllib.request.urlopen(base + path, timeout=5) as response:
        return json.load(response)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bridge", default="http://127.0.0.1:5800",
                        help="Bridge origin serving /telemetry.json")
    args = parser.parse_args()

    try:
        latest = get(args.bridge, "/telemetry.json")
        channels = get(args.bridge, "/channels.json")
    except OSError as e:
        raise SystemExit(f"Bridge unreachable at {args.bridge}: {e}")

    units = latest.get("units") or {
        c["name"]: c.get("unit", "") for c in channels.get("channels", [])}
    labels = latest.get("labels") or {
        c["name"]: c["labels"] for c in channels.get("channels", [])
        if c.get("labels")}

    values = {}
    for name, value in sorted(latest.get("channels", {}).items()):
        names = labels.get(name)
        shown = names[int(value)] if names and float(value).is_integer() \
            and 0 <= int(value) < len(names) else value
        values[name] = {"value": shown, "unit": units.get(name, "")}
    print(json.dumps({
        "stream_id": latest.get("stream_id"),
        "sequence_id": latest.get("sequence_id"),
        "timestamp_ns": latest.get("timestamp_ns"),
        "channels": values,
    }, indent=2))


if __name__ == "__main__":
    main()
