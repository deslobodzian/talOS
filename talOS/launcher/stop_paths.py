"""The bench's stop-path trials, read from a session's logs.

Reads the bench motor's streams (/hw/state, /hw/command,
/hw/state/driver_station, /hw/request/motor, /motor/target/teleop) with
log_dump and prints, for the publishing process of each stream, one line when
its value changes (with when the previous value was last seen), a PAUSE line
for every silence of more than 30 ms in a stream published every tick, and a
LOG GAP line where a log lost records (a pause inside one is the log's, not
the robot's). Times are publish_ms, milliseconds from the start of the
session's logs; sample_us is the RoboRIO's clock. It stops when the hardware
packets were not decoded (a session of another protocol: read it with a build
of that protocol) and prints UNDECODED for a packet its codecs rejected.
See docs/BENCH.md "Reading a trial".
"""

import argparse
import json
from pathlib import Path
import subprocess
import sys

from talOS.launcher.session_report import ENVELOPE_HEADER, HARDWARE_NOT_DECODED, log_dump_path

PAUSE_MS = 30
MOTOR = "kraken"
TOPICS = ("/hw/state", "/hw/command", "/hw/state/driver_station", "/hw/request/motor",
          "/motor/target/teleop")
# Published every tick: a silence longer than PAUSE_MS is a PAUSE.
TICKED = ("/hw/state", "/hw/request/motor", "/motor/target/teleop")


class NotDecoded(Exception):
    """The session's hardware packets were not decoded."""


def trial_lines(records):
    """The lines for log_dump's JSONL records, in their order."""
    last_key, last_ms, seq = {}, {}, {}
    for r in records:
        topic, ms = r["topic"], r["publish_ms"]
        if r["kind"] == "GAP":
            g = r["gap"]
            yield (f"{g['start_ms']:12.3f} {r['process']:<14} LOG GAP to {g['end_ms']:.3f}: "
                   f"{g['records']} records lost ({g['cause']})")
            continue
        if r["kind"] != "SEND":
            continue
        stream = (r["process"], topic)
        if topic.startswith("/hw/") and "hardware" not in r:
            raise NotDecoded("hardware packets not decoded: read this session with a "
                             "build of its protocol")
        h = r.get("hardware") or {}
        if "error" in h:
            yield f"{ms:12.3f} {r['process']:<14} {topic:<26} UNDECODED {h['error']}"
            continue
        motor = next((m for m in h.get("motors", []) if m.get("name") == MOTOR), {})
        if h.get("type") == "state":
            if h["last_command_sequence"] != seq.get("value"):
                seq = {"value": h["last_command_sequence"], "since_us": h["sample_time_us"]}
            volts = motor.get("voltage")
            key = (h["enabled"], h["command_active"], h["epoch"],
                   volts is not None and abs(volts) < 0.05)
            text = (f"sample_us={h['sample_time_us']} enabled={h['enabled']} "
                    f"active={h['command_active']} epoch={h['epoch']} "
                    f"since_us={seq['since_us']} {MOTOR}_v={volts} "
                    f"{MOTOR}_rps={motor.get('velocity_rps')}")
        elif h.get("type") == "command":
            key = (motor.get("mode"),)
            text = (f"observed_us={h['observed_time_us']} "
                    f"{MOTOR}={motor.get('mode')} {motor.get('demand')}")
        elif h.get("type") == "driver_station":
            key = (h["enabled"], h["estop"], h["mode"])
            text = f"enabled={h['enabled']} estop={h['estop']} mode={h['mode']}"
        else:
            found = ENVELOPE_HEADER.match(r["decoded"])
            key = found.groups() if found else (r["decoded"],)
            text = r["decoded"]
        if topic in TICKED and stream in last_ms and ms - last_ms[stream] > PAUSE_MS:
            yield (f"{ms:12.3f} {r['process']:<14} {topic:<26} "
                   f"PAUSE {ms - last_ms[stream]:.1f} ms after {last_ms[stream]:.3f}")
        if last_key.get(stream) != key:
            since = (f" (previous value last at {last_ms[stream]:.3f})"
                     if stream in last_ms else "")
            yield f"{ms:12.3f} {r['process']:<14} {topic:<26} {text}{since}"
            last_key[stream] = key
        last_ms[stream] = ms


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("session", type=Path)
    parser.add_argument("--config", type=Path,
                        help="robot config naming the devices, when the manifest's "
                             "config_path is not readable (a motor value reads None)")
    args = parser.parse_args()
    command = [log_dump_path(), "--format", "jsonl", "--topic", ",".join(TOPICS)]
    if args.config is not None:
        command += ["--config", str(args.config)]
    try:
        result = subprocess.run(command + [str(args.session)], capture_output=True,
                                text=True, errors="replace", check=True)
        lines = list(trial_lines(json.loads(text) for text in result.stdout.splitlines()))
    except NotDecoded as error:
        print(f"stop_paths: {error}", file=sys.stderr)
        found = HARDWARE_NOT_DECODED.search(result.stderr)
        if found:
            print(f"stop_paths: {found[1]}", file=sys.stderr)
        return 1
    except (OSError, ValueError, KeyError, TypeError, RecursionError,
            subprocess.CalledProcessError) as error:
        print(f"stop_paths: {error}", file=sys.stderr)
        if isinstance(error, subprocess.CalledProcessError):
            print(error.stderr, file=sys.stderr)
        return 1
    for text in lines:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
