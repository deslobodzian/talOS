"""Offline bench measurements from the descriptor-aware log_dump reader."""

import argparse
import bisect
import json
import math
from pathlib import Path
import re
import statistics
import subprocess
import sys

from python.runfiles import runfiles


def stats(values):
    values = sorted(values)
    if not values:
        return None
    return {"count": len(values), "median_ms": statistics.median(values),
            "p99_ms": values[math.ceil(.99 * len(values)) - 1], "max_ms": values[-1]}


def scalars(decoded):
    # DecodeToText uses descriptor field names and decimal scalar values. Only
    # top-level status scalars are consumed; nested target contents are not.
    return {key: json.loads(value) for key, value in re.findall(
        r"(?:\{|,)\s*(\w+):\s*(true|false|-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?)\s*(?=[,}])", decoded)}


def stream(records, topic):
    groups = {}
    for record in records:
        if record["topic"] == topic and record["kind"] in ("SEND", "MESSAGE"):
            groups.setdefault((record["process"], record["kind"]), []).append(record)
    # Never merge copies of the same frame from several readers. Prefer the
    # publisher (bridge IPC publication, not raw UDP arrival), else one reader.
    if not groups:
        return None, []
    key = max(groups, key=lambda k: (k[1] == "SEND", len(groups[k]), k[0]))
    return "/".join(key), sorted(groups[key], key=lambda r: r["now_ms"])


TARGET_LANE = re.compile(r"/[^/]+/target/[^/]+")
# log_dump's text for a TargetEnvelope<M> starts with its header fields.
ENVELOPE_HEADER = re.compile(r"\{mode_since_us: (\d+), active: (true|false)")
TARGET_AGE_LIMIT_MS = 25


# log_dump's stderr line for a log whose reader skipped damaged records or a
# damaged tail: the file, then its byte offsets (LogReader, DescribeDamage).
DAMAGED_LOG = re.compile(r"^warning: (damaged log .*)$", re.MULTILINE)


def analyze(manifest, records, log_damage=()):
    source, states = stream(records, "/hw/state")
    _, ds = stream(records, "/hw/state/driver_station")
    ds = [r for r in ds if "ds_enabled" in r]
    ds_times = [r["now_ms"] for r in ds]
    overall, enabled = [], []

    def enabled_at(time):
        # The logged DS enabled bit, held until its next observation.
        index = bisect.bisect_right(ds_times, time) - 1
        return index >= 0 and ds[index]["ds_enabled"]
    for previous, current in zip(states, states[1:]):
        start, end = previous["now_ms"], current["now_ms"]
        gap = end - start
        overall.append(gap)
        # Count a gap if any part overlaps a logged enabled interval, including
        # transitions inside the gap. Before the first DS observation is unknown.
        index = bisect.bisect_right(ds_times, start) - 1
        active = index >= 0 and ds[index]["ds_enabled"]
        transitions = ds[max(index + 1, 0):bisect.bisect_left(ds_times, end)]
        if active or any(r["ds_enabled"] for r in transitions):
            enabled.append(gap)
    targets, timers, status, tick_ages = {}, {}, {}, {}
    enabled_lane_processes = set()
    for r in records:
        if r["kind"] == "MESSAGE" and TARGET_LANE.fullmatch(r["topic"]):
            targets.setdefault(r["process"] + ":" + r["topic"], []).append(
                r["now_ms"] - r["publish_ms"])
            # Only an active envelope stamped with a mode generation is one a
            # Target could select: declarative teleop releases its lane every
            # tick while the sticks are neutral.
            header = ENVELOPE_HEADER.match(r["decoded"])
            if (header and int(header[1]) != 0 and header[2] == "true"
                    and enabled_at(r["now_ms"])):
                enabled_lane_processes.add(r["process"])
        # Target::get records a READ for the lane it selects inside update():
        # now is when the selection was judged, publish the producer's time.
        if r["kind"] == "READ" and TARGET_LANE.fullmatch(r["topic"]):
            tick_ages.setdefault(r["process"] + ":" + r["topic"], []).append(
                r["now_ms"] - r["publish_ms"])
        if r["kind"] == "TIMER":
            timers.setdefault(r["process"] + ":" + r["topic"], []).append(
                r["now_ms"] - r["event_ms"])
    status_source, status_records = stream(records, "/hw/status")
    for r in status_records:
        for field, value in scalars(r["decoded"]).items():
            status.setdefault(field, []).append(value)
    status_summary = {}
    for field, values in status.items():
        item = {"samples": len(values), "first": values[0], "last": values[-1],
                "min": min(values), "max": max(values)}
        if field.endswith("_count") and field not in ("subsystem_count", "clock_window_count"):
            # Negative differences indicate a producer restart/reset, not a
            # negative rejection count. Retain each epoch's observed increments.
            item["delta"] = sum(max(0, b - a) for a, b in zip(values, values[1:]))
            item["resets"] = sum(b < a for a, b in zip(values, values[1:]))
        if field == "clock_offset_valid":
            item["fraction"] = sum(values) / len(values)
        status_summary[field] = item
    verdicts = []
    def verdict(name, measured, passes, detail, required=True):
        # --strict requires every required verdict to PASS. Informational rows
        # have no recorded input or no acceptance threshold and never pass.
        verdicts.append({"measurement": name, "verdict": "NOT MEASURED" if not measured else
                         "PASS" if passes else "FAIL", "required": required, "detail": detail})
    for name, values in (("overall", overall), ("enabled", enabled)):
        s = stats(values)
        verdict(name + " state median", s is not None,
                s is not None and 4 <= s["median_ms"] <= 6,
                "about 5 ms interpreted as 4–6 ms (analysis tolerance, not an additional bring-up requirement)")
        verdict(name + " state p99", s is not None,
                s is not None and s["p99_ms"] < 10, "p99 < 10 ms; nearest rank")
    gaps = sum(v >= 100 for v in enabled)
    verdict("enabled state gaps", bool(enabled), gaps == 0, f"{gaps} gaps >= 100 ms")
    worst = max(((max(v), k) for k, v in tick_ages.items()), default=None)
    unselected = sorted(enabled_lane_processes - {k.split(":", 1)[0] for k in tick_ages})
    bound = (f"selected target age at the tick <= {TARGET_AGE_LIMIT_MS} ms per process:lane "
             "(\"about 25 ms at most\" interpreted as a 25 ms maximum; analysis "
             "interpretation), from Target::get READ records; an age at or past "
             "command_timeout_us is invisible (Target never selects such a lane), so a "
             "producer stall shows as ages rising toward the timeout")
    if unselected:
        detail = (f"{bound}; FAIL: {', '.join(unselected)} dispatched active target lanes "
                  "while enabled but Target never selected one (e.g. a mode generation mismatch)")
    elif worst is not None:
        detail = f"{bound}; worst {worst[1]} = {worst[0]:.6g} ms"
    elif not targets:
        detail = f"{bound}; no target-lane MESSAGE records"
    else:
        detail = f"{bound}; no active target lane dispatched while enabled"
    verdict("target age at tick", worst is not None or bool(unselected),
            worst is not None and not unselected and worst[0] <= TARGET_AGE_LIMIT_MS, detail)
    verdict("timer lateness", False, False,
            "Measured where TIMER dispatches exist; no acceptance threshold is specified.",
            required=False)
    issues = [] if manifest.get("end_wall_ns") else [
        "session unfinished (end_wall_ns = 0: no normal shutdown)"]
    # Only recording nodes (non-empty log_path) are acceptance evidence. Host
    # processes without an event log (studio_bridge, sim_gateway) are not.
    issues += [f"{n.get('name')}: {flag}" for n in manifest["nodes"] if n.get("log_path")
               for flag in ("quarantined", "recorder_failed", "log_capped") if n.get(flag)]
    # A record the reader skipped is missing from every measurement above.
    issues += list(log_damage)
    return {"session_id": manifest.get("session_id"),
            "config_sha256": manifest.get("config_sha256") or "NOT RECORDED (non-bundle launch)",
            # A --config=release bundle's source revision; older manifests lack it.
            "git_revision": (manifest.get("git_revision") or "NOT RECORDED (unstamped or non-bundle)")
                            + (" (dirty)" if manifest.get("git_dirty") else ""),
            # exit_signal is 0 unless a signal killed the node; older manifests lack it.
            "nodes": [{**{k: n.get(k) for k in ("name", "restart_count", "exit_code")},
                       "exit_signal": n.get("exit_signal", 0),
                       **{k: n.get(k) for k in ("quarantined", "recorder_failed", "log_capped")}}
                      for n in manifest["nodes"]],
            # Every run and why it ended (launcher README "Session manifest");
            # absent in manifests written before the launcher recorded runs.
            "runs": {n.get("name"): {"binary_sha256": n.get("binary_sha256", ""),
                                     "runs": n.get("runs", []),
                                     "runs_dropped": n.get("runs_dropped", 0)}
                     for n in manifest["nodes"]},
            "session_issues": issues,
            "state_stream": source, "state_overall": stats(overall),
            "state_enabled": stats(enabled), "enabled_gaps_ge_100_ms": gaps,
            "target_receipt_age": {k: stats(v) for k, v in sorted(targets.items())},
            "target_age_at_tick": {k: stats(v) for k, v in sorted(tick_ages.items())},
            "timer_lateness": {k: stats(v) for k, v in sorted(timers.items())},
            "status_stream": status_source, "status": status_summary, "verdicts": verdicts,
            "limitations": ["State timing is bridge IPC publication or one consumer dispatch stream, not raw UDP arrival; never pooled duplicate readers.",
                            "Enabled uses the logged DS enabled bit, held until its next observation; DS silence is not proof of continued physical enablement.",
                            "Counter deltas exclude unknown increments before the first sample and during resets; clock mapping is not absolute synchronization.",
                            "Capped, truncated or unfinished sessions cannot establish whole-session acceptance; retain raw logs."]}


def strict_failures(report):
    """Reasons a --strict run rejects the session; empty means acceptance."""
    return [f"{v['measurement']}: {v['verdict']}" for v in report["verdicts"]
            if v["required"] and v["verdict"] != "PASS"] + report["session_issues"]


def markdown(report):
    lines = ["# Session report", f"Session: {report['session_id']}",
             f"Config SHA-256: {report['config_sha256']}",
             f"Source revision: {report['git_revision']}", "", "## Nodes",
             "| Node | Restarts | Exit code | Exit signal | quarantined | recorder_failed | log_capped |",
             "|---|---:|---:|---:|---|---|---|"]
    for n in report["nodes"]:
        lines.append("| " + " | ".join(str(n[k]) for k in n) + " |")
    lines += ["", "## Runs", "| Node | Binary SHA-256 | Run | PID | Seconds | Exit code | Exit signal | Reason |",
              "|---|---|---:|---:|---:|---:|---:|---|"]
    for name, node in report["runs"].items():
        for index, run in enumerate(node["runs"]):
            ended = run["end_wall_ns"]
            seconds = f"{(ended - run['start_wall_ns']) / 1e9:.3f}" if ended else "running"
            # A run after the dropped ones is numbered as it was spawned.
            number = index if index == 0 else index + node["runs_dropped"]
            lines.append(f"| {name} | {node['binary_sha256'][:12] or '—'} | {number} | "
                         f"{run['pid']} | {seconds} | {run['exit_code']} | "
                         f"{run['exit_signal']} | {run['reason']} |")
        if node["runs_dropped"]:
            lines.append(f"| {name} | | | | | | | {node['runs_dropped']} runs after the first dropped |")
    lines += ["", "## Session completeness"] + (
        ["- " + s for s in report["session_issues"]] or ["- complete"])
    lines += ["", "## Timing (ms)", f"State stream: {report['state_stream'] or 'NOT MEASURED'}",
              "| Measurement | Count | Median | p99 | Max |", "|---|---:|---:|---:|---:|"]
    entries = {"/hw/state overall": report["state_overall"],
               "/hw/state enabled": report["state_enabled"],
               **{"receipt " + k: v for k, v in report["target_receipt_age"].items()},
               **{"target at tick " + k: v for k, v in report["target_age_at_tick"].items()},
               **{"timer " + k: v for k, v in report["timer_lateness"].items()}}
    for name, s in entries.items():
        lines.append(f"| {name} | " + ("NOT MEASURED | — | — | — |" if s is None else
                     f"{s['count']} | {s['median_ms']:.6g} | {s['p99_ms']:.6g} | {s['max_ms']:.6g} |"))
    if not report["target_receipt_age"]:
        lines.append("Target receipt age: NOT MEASURED (no target MESSAGE dispatches).")
    if not report["target_age_at_tick"]:
        lines.append("Target age at tick: NOT MEASURED (no Target selected a lane).")
    for n in report["nodes"]:
        if not any(k.startswith(n["name"] + ":") for k in report["timer_lateness"]):
            lines.append(f"{n['name']} timer lateness: NOT MEASURED (no TIMER dispatches).")
    lines += ["", "## /hw/status", f"Stream: {report['status_stream'] or 'NOT MEASURED'}"]
    for field, values in report["status"].items():
        lines.append(f"- {field}: " + ", ".join(f"{k}={v}" for k, v in values.items()))
    lines += ["", "## Verdicts"]
    lines += [f"- **{v['verdict']}** {v['measurement']}" +
              ("" if v["required"] else " (informational, not required)") + f": {v['detail']}"
              for v in report["verdicts"]]
    lines += ["", "## Measurement limits"] + ["- " + s for s in report["limitations"]]
    return "\n".join(lines)


def log_dump_path():
    files = runfiles.Create()
    binary = files.Rlocation("_main/talOS/launcher/log_dump") if files else None
    if not binary or not Path(binary).is_file():
        raise FileNotFoundError(
            "log_dump runfile not found; run with bazel run //talOS/launcher:session_report")
    return binary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--strict", action="store_true",
                        help="exit 2 unless every required verdict passes and the session is complete")
    args = parser.parse_args()
    try:
        manifest = json.loads((args.session / "manifest.json").read_text())
        result = subprocess.run([log_dump_path(), "--format", "jsonl", str(args.session)],
                                capture_output=True, text=True, check=True)
        report = analyze(manifest, [json.loads(line) for line in result.stdout.splitlines()],
                         DAMAGED_LOG.findall(result.stderr))
        print(json.dumps(report, indent=2) if args.json else markdown(report))
        if not args.strict:
            return 0
        failures = strict_failures(report)
        for failure in failures:
            print(f"session_report: strict: {failure}", file=sys.stderr)
        return 2 if failures else 0
    except (OSError, ValueError, KeyError, subprocess.CalledProcessError) as error:
        print(f"session_report: {error}", file=sys.stderr)
        if isinstance(error, subprocess.CalledProcessError):
            print(error.stderr, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
