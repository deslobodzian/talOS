#!/usr/bin/env python3
"""What was measured right now, straight from the bridge.

Reads GET /telemetry.json (stdlib only, no agent): every channel's latest
value, or its enum label, with its unit.
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
    except OSError as e:
        raise SystemExit(f"Bridge unreachable at {args.bridge}: {e}")

    # channels is keyed by name; value is null when the channel had no fresh
    # reading, and enum channels carry their label alongside the number.
    values = {
        name: {"value": entry.get("label", entry["value"]),
               "unit": entry["unit"]}
        for name, entry in sorted(latest["channels"].items())
    }
    print(json.dumps({
        "stream_id": latest["stream_id"],
        "sequence_id": latest["sequence_id"],
        "timestamp_ns": latest["timestamp_ns"],
        "channels": values,
    }, indent=2))


if __name__ == "__main__":
    main()
