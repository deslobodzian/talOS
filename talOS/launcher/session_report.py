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


def _reject_non_finite_json(token):
    raise ValueError(f"non-finite JSON constant: {token}")


def load_manifest(path):
    """Reads and validates a session manifest.json object."""
    try:
        manifest = json.loads(Path(path).read_text(encoding="utf-8"),
                              parse_constant=_reject_non_finite_json)
    except RecursionError as error:
        raise ValueError(f"{path}: manifest JSON nested too deeply") from error
    except UnicodeDecodeError as error:
        raise ValueError(f"{path}: manifest JSON is not valid UTF-8") from error
    if not isinstance(manifest, dict):
        raise ValueError(f"{path}: manifest root must be a JSON object")
    nodes = manifest.get("nodes")
    if not isinstance(nodes, list) or not all(isinstance(n, dict) for n in nodes):
        raise ValueError(f"{path}: manifest 'nodes' must be a list of objects")
    return manifest


def stats(values, unit="ms"):
    """Count, min, median, nearest-rank p99 and max, each named with its unit;
    None for no values (a row with no sample is not measured, not 0)."""
    values = sorted(
        v for v in values
        if isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
    )
    if not values:
        return None
    return {"count": len(values), f"min_{unit}": values[0],
            f"median_{unit}": statistics.median(values),
            f"p99_{unit}": values[math.ceil(.99 * len(values)) - 1],
            f"max_{unit}": values[-1]}


_SCALAR_VALUE = re.compile(r"true|false|-?\d+(?:\.\d+)?(?:[eE][+-]?\d+)?")
_FIELD_KEY = re.compile(r"\w+")
_MAX_DECODED_DEPTH = 64


def _parse_scalar_token(token):
    if _SCALAR_VALUE.fullmatch(token) is None:
        return None
    try:
        parsed = json.loads(token)
    except ValueError:
        return None
    if isinstance(parsed, float) and not math.isfinite(parsed):
        return None
    return parsed


def _parse_decoded_value(text, pos, depth):
    n = len(text)
    while pos < n and text[pos].isspace():
        pos += 1
    if pos >= n or depth > _MAX_DECODED_DEPTH:
        return None, n
    ch = text[pos]
    if ch == "{":
        pos += 1
        obj = {}
        while pos < n:
            while pos < n and text[pos].isspace():
                pos += 1
            if pos >= n:
                return obj, n
            if text[pos] == "}":
                return obj, pos + 1
            key_match = _FIELD_KEY.match(text, pos)
            if key_match is None:
                return obj, n
            key = key_match.group(0)
            pos = key_match.end()
            while pos < n and text[pos].isspace():
                pos += 1
            if pos >= n or text[pos] != ":":
                return obj, n
            pos += 1
            value, pos = _parse_decoded_value(text, pos, depth + 1)
            obj[key] = value
            while pos < n and text[pos].isspace():
                pos += 1
            if pos < n and text[pos] == ",":
                pos += 1
                continue
            if pos < n and text[pos] == "}":
                return obj, pos + 1
            return obj, n
        return obj, n
    if ch == "[":
        pos += 1
        items = []
        while pos < n:
            while pos < n and text[pos].isspace():
                pos += 1
            if pos >= n:
                return items, n
            if text[pos] == "]":
                return items, pos + 1
            value, pos = _parse_decoded_value(text, pos, depth + 1)
            items.append(value)
            while pos < n and text[pos].isspace():
                pos += 1
            if pos < n and text[pos] == ",":
                pos += 1
                continue
            if pos < n and text[pos] == "]":
                return items, pos + 1
            return items, n
        return items, n
    if ch == "<":
        end = text.find(">", pos)
        return None, n if end == -1 else end + 1
    end = pos
    while end < n and text[end] not in ",}]":
        end += 1
    return _parse_scalar_token(text[pos:end].strip()), end


def _parse_decoded_struct(decoded):
    if not isinstance(decoded, str):
        return {}
    pos = 0
    n = len(decoded)
    while pos < n and decoded[pos].isspace():
        pos += 1
    if pos >= n or decoded[pos] != "{":
        return {}
    parsed, _ = _parse_decoded_value(decoded, pos, 0)
    return parsed if isinstance(parsed, dict) else {}


def scalars(decoded):
    # DecodeToText uses descriptor field names and decimal scalar values. Only
    # depth-1 top-level scalar fields are returned; nested structs and arrays
    # never overwrite or leak into top-level keys.
    parsed = _parse_decoded_struct(decoded)
    return {
        key: value
        for key, value in parsed.items()
        if isinstance(value, (bool, int, float))
    }


def status_scalars(decoded):
    # Top-level BridgeStatus scalars plus active can_buses[0..can_bus_count-1]
    # slot scalars keyed as `can_buses[i].<field>`, so unused zeroed slots
    # never overwrite active CAN buses (REQ-OPS-16, REQ-OPS-20).
    parsed = _parse_decoded_struct(decoded)
    found = {
        key: value
        for key, value in parsed.items()
        if isinstance(value, (bool, int, float))
    }
    buses = parsed.get("can_buses")
    if isinstance(buses, list):
        count = found.get("can_bus_count")
        if "can_bus_count" not in parsed:
            limit = len(buses)
        elif isinstance(count, int) and not isinstance(count, bool) and count >= 0:
            limit = min(count, len(buses))
        else:
            limit = 0
        for index in range(limit):
            slot = buses[index]
            if not isinstance(slot, dict):
                continue
            for key, value in slot.items():
                if isinstance(value, (bool, int, float)):
                    found[f"can_buses[{index}].{key}"] = value
    return found


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


# A target lane, /<subsystem>/target/<source>, by the segment rules of
# talOS/NAMING.md (naming::IsTargetLane is the authority): each segment starts
# with a lowercase letter and has no doubled or trailing underscore, and the
# whole name fits kMaxTopicBytes, which also bounds both segments below
# kMaxSegmentBytes. Abbreviations and direction suffixes are not re-checked:
# the launcher refuses a graph that uses them.
_SEGMENT = r"[a-z](?:_?[a-z0-9])*"
_TARGET_LANE = re.compile(rf"/{_SEGMENT}/target/{_SEGMENT}")
MAX_TOPIC_BYTES = 31


def is_target_lane(topic):
    return (len(topic) <= MAX_TOPIC_BYTES
            and _TARGET_LANE.fullmatch(topic) is not None)


# log_dump's text for a TargetEnvelope<M> starts with its header fields.
ENVELOPE_HEADER = re.compile(r"\{mode_since_us: (\d+), active: (true|false)")
TARGET_AGE_LIMIT_MS = 25


# log_dump's stderr lines for a log whose reader skipped damaged records, a
# damaged tail, or an incomplete header (LogReader, DescribeDamage, log_merge).
# `warning: log gap in ...` is intentionally left unmatched (handled via GAP).
DAMAGED_LOG = re.compile(r"^warning: ((?:damaged log|skipping) .*)$", re.MULTILINE)

# log_dump's one stderr line for a session whose hardware packets it did
# not decode: it records another hardware protocol, or this
# build's codecs reject one of them (talOS/launcher/hardware_record.h).
HARDWARE_NOT_DECODED = re.compile(r"^log_dump: warning: (hardware packets not decoded: .*)$",
                                  re.MULTILINE)

# log_dump's one stderr line for a session whose bridge and launcher
# recorded different hardware protocols; it reads the bridge's.
HARDWARE_PROTOCOL_DISAGREES = re.compile(r"^log_dump: (warning: the session's bridge recorded .*)$",
                                         re.MULTILINE)


# The loops on the actuation path (talOS/process LoopTier): a gap in one is a
# gap in a hardware request (ARCHITECTURE.md "The tick").
ACTUATION_TIERS = ("control", "bridge")


def realtime_summary(node):
    """A node's realtime outcome as the launcher recorded it from its registry
    row: the grant, then any refused steps; NOT RECORDED in a manifest that
    predates it, NOT REPORTED for a node that never reported one."""
    if "realtime" not in node:
        return "NOT RECORDED"
    grant = node["realtime"] or "NOT REPORTED"
    refused = node.get("realtime_refused", "")
    return f"{grant} ({refused})" if refused else grant


def realtime_issue(node, simulation):
    """Why a node's loop ran without its realtime grant, or None. Only the
    actuation path counts, and under --sim only a node that asked
    (TALOS_REALTIME=1) and was refused part of it."""
    tier, grant = node.get("loop_tier", ""), node.get("realtime", "")
    if tier not in ACTUATION_TIERS or grant in ("", "full"):
        return None
    if simulation and grant == "not_requested":
        return None
    return (f"{node.get('name')}: {tier} loop ran without its realtime grant: "
            + realtime_summary(node))


# What each node's registry row says it lost (talOS/introspection README
# "Wake loss"), as the launcher records it per node, summed over its runs:
# each should stay 0. NOT RECORDED in a manifest that predates the counts,
# and for a member whose row the launcher never saw (Studio's bridge).
DELIVERY_COUNTERS = ("wake_failures", "stalled_arms", "wake_fallbacks", "requests_skipped")
# --strict rejects every one of them (REQ-OPS-43). A wake notification a
# writer could not send left its reader asleep until its backstop, and a
# request the bridge's latest-value read passed over is one its subsystem
# never merged. A stalled arm and a wake fallback lost nothing, but the
# loop drained that watcher (a stalled arm) or every input (a fallback) on
# its timers, backstop or poll tick instead of on publication: the latency
# of every prompt handler (REQ-RT-19), and for a fallback the no-polling
# guarantee (REQ-RT-02, REQ-RT-21), did not hold for it.
STRICT_DELIVERY = DELIVERY_COUNTERS


def delivery_summary(node):
    """A node's nonzero delivery counters, "none", or NOT RECORDED."""
    if not any(key in node for key in DELIVERY_COUNTERS):
        return "NOT RECORDED"
    return ", ".join(delivery_count(node, key) for key in DELIVERY_COUNTERS
                     if node.get(key)) or "none"


def delivery_count(node, key):
    """`key=value`; the bridge's skipped requests also per subsystem."""
    split = node.get("requests_skipped_by_subsystem") if key == "requests_skipped" else None
    detail = (f" ({', '.join(f'{k}={v}' for k, v in split.items())})"
              if isinstance(split, dict) and split else "")
    return f"{key}={node[key]}{detail}"


def delivery_issues(node):
    """Why --strict rejects a node's runs, one line per nonzero count."""
    return [f"{node.get('name')}: {delivery_count(node, key)}"
            for key in STRICT_DELIVERY if node.get(key)]


# The launcher's own notes in talos.log (talOS/launcher child_output.cc) of
# output it could not keep, each with its byte count and the output it was
# lost from: a member's <name>.log, talos.log itself, or the launcher's
# stdout or stderr.
LAUNCHER_DROPS = (
    (re.compile(r"^\[launcher\] (\S+): (\d+) bytes of output dropped \(marked in (.+)\)$",
                re.MULTILINE),
     lambda m: f"{m[2]} bytes dropped from {Path(m[3]).name}"),
    (re.compile(r"^\[launcher\] (\d+) bytes of output dropped here: (.+)$", re.MULTILINE),
     lambda m: f"{m[1]} bytes dropped from talos.log ({m[2]})"),
    # ", at most": part of the count was inside a write that may yet finish.
    (re.compile(r"^\[launcher\] (\d+) bytes for the launcher's (\w+) were not "
                r"written(, at most)?", re.MULTILINE),
     lambda m: f"{'up to ' if m[3] else ''}{m[1]} bytes dropped from the "
               f"launcher's {m[2]}"),
    (re.compile(r"^\[launcher\] output log reached (\d+) bytes; later output is dropped$",
                re.MULTILINE),
     lambda m: f"talos.log reached its {m[1]}-byte cap; later lines dropped"),
)


def launcher_drops(session, manifest):
    """One line per output the launcher's talos.log says lost bytes, in the
    order noted; None when the manifest names no launcher log (written before
    it did). The file is read beside the manifest, wherever the session was
    copied."""
    if not isinstance(manifest, dict):
        return None
    name = manifest.get("launcher_log")
    if not isinstance(name, str) or not name or "\x00" in name:
        return None
    path = Path(session) / Path(name).name
    try:
        text = path.read_text(errors="replace")
    except OSError as error:
        return [f"{path.name} unreadable ({error.strerror})"]
    found = [(m.start(), describe(m)) for pattern, describe in LAUNCHER_DROPS
             for m in pattern.finditer(text)]
    return [line for _, line in sorted(found)]


def missing_logs(session, manifest):
    """One issue per event log listed on a node in `manifest` that is absent
    beside `manifest.json` (neither `<name>` nor any rotated
    `<stem>.segment<k>.tlog`, talOS/events/log/format.h)."""
    if not isinstance(manifest, dict):
        return []
    nodes = manifest.get("nodes")
    if not isinstance(nodes, list):
        return []
    session_dir = Path(session)
    file_names = set()
    try:
        for entry in session_dir.iterdir():
            try:
                if entry.is_file():
                    file_names.add(entry.name)
            except OSError:
                continue
    except OSError:
        pass
    issues = []
    for node in nodes:
        if not isinstance(node, dict):
            continue
        node_name = node.get("name")
        raw_paths = node.get("log_paths")
        candidates = []
        if isinstance(raw_paths, list):
            candidates.extend(raw_paths)
        log_path = node.get("log_path")
        if log_path is not None:
            candidates.append(log_path)
        seen_raw = set()
        seen_names = set()
        for raw in candidates:
            if not isinstance(raw, str) or not raw or raw in seen_raw:
                continue
            seen_raw.add(raw)
            if "\x00" in raw:
                issues.append(f"{node_name}: missing log {raw}")
                continue
            name = Path(raw).name
            if not name:
                issues.append(f"{node_name}: missing log {raw}")
                continue
            if name in seen_names:
                continue
            seen_names.add(name)
            if name in file_names:
                continue
            stem = name[:-5] if name.endswith(".tlog") else name
            segment_re = re.compile(re.escape(stem) + r"\.segment\d+\.tlog")
            if any(segment_re.fullmatch(entry) for entry in file_names):
                continue
            issues.append(f"{node_name}: missing log {name}")
    return issues


def log_gaps(records):
    """Every GAP record log_dump exported (a stretch of records its log's
    writer dropped, talOS/events/log/format.h), as its `gap` member with the
    process whose log holds it."""
    return [{"process": r["process"], **r["gap"]} for r in records if r["kind"] == "GAP"]


def gap_issue(gap):
    """A gap as a session issue: its process, span, loss and cause."""
    return (f"{gap['process']}: log gap from {gap['start_ms']:.6g} to {gap['end_ms']:.6g} ms, "
            f"{gap['records']} records lost ({gap['cause']}"
            + (", closed at the stop)" if gap["closed_at_flush"] else ")"))


def lost_spans(gaps):
    """Per process, each of its log's gaps as the span holding every time a
    record it dropped named (its now, event and publish times):
    [min(start_ms, first_lost_ms), last_lost_ms], or [start_ms, end_ms] for a
    GAP without its detail. The record capture resumed with, at end_ms, was
    kept, so a detailed span stops at the last record lost. Overlapping
    intervals are sorted and merged per process so across_gap is O(log G)."""
    raw = {}
    for gap in gaps:
        first, last = gap.get("first_lost_ms"), gap.get("last_lost_ms")
        low = gap["start_ms"] if first is None else min(gap["start_ms"], first)
        high = gap["end_ms"] if last is None else last
        if math.isnan(low) or math.isnan(high):
            continue
        if low > high:
            low, high = high, low
        raw.setdefault(gap["process"], []).append((low, high))
    merged = {}
    for process, intervals in raw.items():
        intervals.sort()
        out = []
        for start, end in intervals:
            if out and start <= out[-1][1]:
                if end > out[-1][1]:
                    out[-1] = (out[-1][0], end)
            else:
                out.append((start, end))
        merged[process] = out
    return merged


def across_gap(spans, processes, low, high):
    """Whether [low, high] meets a lost span of any of these processes' logs:
    a record of theirs could be missing inside it, so an interval it bounds
    is not measured."""
    if math.isnan(low) or math.isnan(high):
        return False
    for process in processes:
        intervals = spans.get(process)
        if not intervals:
            continue
        index = bisect.bisect_right(intervals, (high, math.inf)) - 1
        if index >= 0 and low <= intervals[index][1]:
            return True
    return False


def request_to_command(records, spans=None):
    """The event-driven bridge's reaction to requests (REQ-RT-20), as
    (process, samples, unmeasured), each sample (request ms, command ms):
    for each observation (boot_id,
    observed_time_us) a merged /hw/command carries, its first command's time
    minus the first /hw/request/<node> carrying it. A command repeating an
    observation (the heartbeat) is no reaction, and one no request carried by
    then (a timeout's neutral command adopting the State, even if a request
    observing that State follows it) is skipped. A sample spans from the
    request before that first one in its node's request stream, or the
    stream's start (where an earlier request carrying the observation could
    be missing), to the
    command; it is unmeasured, and counted, when that span meets a log gap
    (lost_spans) of the bridge or of any node whose requests were searched."""
    requests, commands, process = [], [], None
    for r in records:
        hardware = r.get("hardware")
        if r["kind"] != "SEND" or not isinstance(hardware, dict) \
                or "observed_time_us" not in hardware:
            continue
        key = (hardware.get("boot_id"), hardware["observed_time_us"])
        if r["topic"].startswith("/hw/request/"):
            requests.append((r["now_ms"], (r["process"], r["topic"]), key))
        elif r["topic"] == "/hw/command":
            commands.append((r["now_ms"], key))
            process = r["process"]
    # Each observation's first request, and the request before it in its
    # stream (the log's start before its first: a gap at its head could hold
    # one).
    first_request, last = {}, {}
    for now, request_stream, key in sorted(requests, key=lambda q: q[0]):
        if key not in first_request:
            first_request[key] = (now, last.get(request_stream, -math.inf))
        last[request_stream] = now
    logs = {process} | {request_stream[0] for _, request_stream, _ in requests}
    samples, seen, unmeasured = [], set(), 0
    for now, key in sorted(commands, key=lambda c: c[0]):
        if key not in seen and first_request.get(key, (math.inf,))[0] <= now:
            requested, since = first_request[key]
            if across_gap(spans or {}, logs, since, now):
                unmeasured += 1
            else:
                samples.append((requested, now))
        seen.add(key)
    return process, samples, unmeasured


# REQ-RT-25: each latency row is the difference of two readings of one
# clock. Every time subtracted below is a main-processor time (a record's
# now or publish); a RoboRIO time (sample_time_us, observed_time_us) is
# only matched or ordered against another of the same boot, never
# subtracted from a main-processor one, whose mapping to it is uncertain by
# milliseconds (docs/DATA_FLOW.md).


def observations(records, prefix):
    """The SEND records on topics starting with `prefix` that carry an
    observation, as (now_ms, record, (boot_id, observed_time_us)), by time."""
    found = []
    for r in records:
        hardware = r.get("hardware")
        if (r["kind"] == "SEND" and r["topic"].startswith(prefix)
                and isinstance(hardware, dict) and "observed_time_us" in hardware):
            found.append((r["now_ms"], r, (hardware.get("boot_id"),
                                           hardware["observed_time_us"])))
    return sorted(found, key=lambda f: f[0])


# The age the RoboRIO reports for a signal whose timestamp Phoenix could not
# give (PhoenixBackend.cpp Age(): UINT32_MAX, while the sample is valid).
UNKNOWN_AGE_US = 2**32 - 1


def feedback_age(states):
    """Per motor and CAN sensor ("motor <id>", "sensor <id>"), the age of
    its motion feedback in each State of the selected /hw/state stream: the
    older of its position_age_us and velocity_age_us. The RoboRIO measured
    each (the signal's age at its read), so each is already a difference of
    its one clock; a sample the State marks invalid is skipped. An age of
    UNKNOWN_AGE_US (Phoenix's timestamp was unknown) is no age: the sample
    is left out of the statistics and counted in `sentinel`. A device with
    only such samples reads count 0, NOT MEASURED."""
    ages, sentinels = {}, {}
    for r in states:
        hardware = r.get("hardware")
        if not isinstance(hardware, dict):
            continue
        for kind, member in (("motor", "motors"), ("sensor", "sensors")):
            for sample in hardware.get(member, ()):
                if not sample.get("valid"):
                    continue
                device = (kind, sample["id"])
                values = ages.setdefault(device, [])
                age = (sample["position_age_us"], sample["velocity_age_us"])
                if UNKNOWN_AGE_US in age:
                    sentinels[device] = sentinels.get(device, 0) + 1
                else:
                    values.append(max(age))
    return {f"{kind} {device}": {**(stats(values, "us") or {"count": 0}),
                                 "sentinel": sentinels.get((kind, device), 0)}
            for (kind, device), values in sorted(ages.items())}


def state_to_request(states, records, spans):
    """Per actuator owner (the process sending /hw/request/<node>), the
    tick time of its first request carrying each State's observation less
    that State's /hw/state publication: request_to_command's join, keyed
    (boot_id, sample_time_us) to (boot_id, observed_time_us). It ends at the
    tick's start, where request to command begins, so the two rows add up
    along the path. A State the stream lacks gives no sample; one whose span
    meets a log gap of the owner's log, which could hide an earlier request
    carrying it, is unmeasured and counted. Returns ({node: [ms]}, count)."""
    published = {}
    for r in states:
        hardware = r.get("hardware")
        if isinstance(hardware, dict) and "sample_time_us" in hardware:
            published.setdefault((hardware.get("boot_id"), hardware["sample_time_us"]),
                                 r["publish_ms"])
    first = {}
    for now, r, key in observations(records, "/hw/request/"):
        first.setdefault((r["process"], key), now)
    delays, unmeasured = {}, 0
    for (node, key), now in first.items():
        since = published.get(key)
        if since is None:
            continue
        # A node whose every sample met a gap is listed, as not measured.
        samples = delays.setdefault(node, [])
        if across_gap(spans, {node}, since, now):
            unmeasured += 1
        else:
            samples.append(now - since)
    return delays, unmeasured


def driver_station_to_lane(packets, records, spans):
    """Per producer (a process publishing a target lane), each Driver
    Station packet's /hw/state/driver_station publication to that
    producer's next lane publication at or after it, a release included,
    taken only when that publication comes before the stream's next packet
    (the last packet's window is open): the row is the reaction of a
    producer that is publishing. A packet the producer did not answer in
    its window gives no sample and is counted unanswered per producer,
    apart by the packet's own enabled bit (ds_enabled, as the enabled rows
    read it): "enabled" (a stall, or by design an operator producer not yet
    armed or in a mode it does not serve) and "other" (disabled), with
    their "total"; if any unanswered packet has no enabled bit (not
    decoded), the split is not measured and both are null, as every row
    that needs the bit is in such a session. So a producer silent while
    disabled (teleop releases its lanes once and then publishes nothing) is
    no sample spanning the silence, and one that stalls while enabled shows
    in its own count. It ends where target age at the tick begins. A
    sample, or an unanswered window, that meets a log gap of the producer's
    log (which could hide a send) or of the packet stream's (which could
    hide the next packet) is unmeasured and counted.
    Returns ({node: [ms]}, {node: {"total", "enabled", "other"}}, unmeasured)."""
    sends = {}
    for r in records:
        if r["kind"] == "SEND" and is_target_lane(r["topic"]):
            sends.setdefault(r["process"], []).append(r["publish_ms"])
    packets = sorted(packets, key=lambda p: p["publish_ms"])
    windows = [(p["publish_ms"], later["publish_ms"], p.get("ds_enabled"))
               for p, later in zip(packets, packets[1:])]
    if packets:
        windows.append((packets[-1]["publish_ms"], math.inf,
                        packets[-1].get("ds_enabled")))
    stream_logs = {p["process"] for p in packets[:1]}
    delays, unanswered, unmeasured = {}, {}, 0
    for node, times in sorted(sends.items()):
        times.sort()
        samples = delays.setdefault(node, [])
        counts = {True: 0, False: 0, None: 0}
        logs = {node} | stream_logs
        for since, until, enabled in windows:
            index = bisect.bisect_left(times, since)
            answer = times[index] if index < len(times) else math.inf
            if across_gap(spans, logs, since, min(answer, until)):
                unmeasured += 1
            elif answer < until:
                samples.append(answer - since)
            else:
                counts[None if enabled is None else bool(enabled)] += 1
        # A packet without its enabled bit (not decoded) leaves the split
        # not measured: null, as every row that needs the bit is.
        split = counts[None] == 0
        unanswered[node] = {"total": sum(counts.values()),
                            "enabled": counts[True] if split else None,
                            "other": counts[False] if split else None}
    return delays, unanswered, unmeasured


def dispatches(records):
    """Each record's dispatch as (process, run, dispatch_index), keyed by
    the record's position in `records`. A restarted node logs under the same
    process name with its dispatch index counting from 0 again
    (EventLoopBase::begin_run), so a process's records start a new run
    after its EXIT record or where, in time order, the index decreases."""
    runs, found = {}, {}
    order = sorted((r["process"], r["now_ms"], r["dispatch_index"], position, r["kind"])
                   for position, r in enumerate(records)
                   if r.get("dispatch_index") is not None and r["kind"] != "GAP")
    for process, _, index, position, kind in order:
        run, last, exited = runs.get(process, (0, -1, False))
        if exited or index < last:
            run += 1
        runs[process] = (run, index, kind == "EXIT")
        found[position] = (process, run, index)
    return found


def driver_station_to_command(packets, records, spans, enabled_at, bridge):
    """For each teleop Driver Station packet that shows the robot enabled,
    its /hw/state/driver_station publication to the first /hw/command built
    from it: the earliest tick in which a Target selected a lane published
    at or after this packet and before the next one (its READ record) and
    its node sent a request (the same dispatch of the same run), then the
    bridge's first command at or after that tick that merged it: of the
    same boot, with an observation no older than the request's (a merged
    command carries its oldest owner's, so with two owners it waits for the
    other's next request). Observations are only compared. The robot must
    still be enabled at the command (enabled_at). A sample whose span meets
    a log gap of the bridge's log, or of a node that selected a lane or sent
    a request, is unmeasured and counted. Returns ([ms], count)."""
    keys = dispatches(records)
    requests = {}
    for position, r in enumerate(records):
        hardware = r.get("hardware")
        if (r["kind"] == "SEND" and r["topic"].startswith("/hw/request/")
                and isinstance(hardware, dict) and "observed_time_us" in hardware
                and position in keys):
            requests[keys[position]] = (r["now_ms"], (hardware.get("boot_id"),
                                                      hardware["observed_time_us"]))
    selections = [(position, r) for position, r in enumerate(records)
                  if r["kind"] == "READ" and is_target_lane(r["topic"])]
    packets = sorted(packets, key=lambda p: p["publish_ms"])
    published = [p["publish_ms"] for p in packets]
    # Each packet's earliest selection: a tick whose lane was published in
    # its window [this packet, the next packet). One pass over the ticks.
    first_tick = {}
    for now, lane, request in sorted((r["now_ms"], r["publish_ms"], requests[keys[position]])
                                     for position, r in selections
                                     if keys.get(position) in requests):
        packet = bisect.bisect_right(published, lane) - 1
        if packet >= 0 and packet not in first_tick:
            first_tick[packet] = request
    commands = [(now, key) for now, _, key in observations(records, "/hw/command")]
    command_times = [now for now, _ in commands]
    logs = {bridge} | {r["process"] for _, r in selections} | {key[0] for key in requests}
    delays, unmeasured = [], 0
    for packet, (requested, (boot, observed)) in sorted(first_tick.items()):
        if not packets[packet].get("ds_enabled") \
                or (packets[packet].get("hardware") or {}).get("mode") != "teleop":
            continue
        since = published[packet]
        command = None
        # Walk from the request; the walk ends at the first command that
        # merged it, of another boot, or once the robot is disabled.
        for index in range(bisect.bisect_left(command_times, requested), len(commands)):
            now, (command_boot, command_observed) = commands[index]
            if command_boot != boot or not enabled_at(now):
                break
            if command_observed >= observed:
                command = now
                break
        if command is None:
            continue
        if across_gap(spans, logs, since, command):
            unmeasured += 1
        else:
            delays.append(command - since)
    return delays, unmeasured


def clock_drift_ppm(status_records, spans):
    """How fast the bridge's clock_offset_us (its mapping of gateway time to
    node time) changes per main-processor second, in ppm: the least-squares
    slope of the valid offsets against their /hw/status publication times,
    fitted within each run of one estimator (one clock_boot_id and
    clock_resets_count, and no log gap of the status stream's log between
    two samples) and pooled across runs. It is the two oscillators' relative
    drift; no latency is computed across the clocks. None without two valid
    samples at different times in one run."""
    status_processes = {r["process"] for r in status_records[:1]}
    lost = sorted(span for process in status_processes
                  for span in spans.get(process, ()))
    lost_ends = [end for _, end in lost]
    runs = {}
    for r in status_records:
        fields = scalars(r["decoded"])
        time = r["publish_ms"]
        offset = fields.get("clock_offset_us")
        if (fields.get("clock_offset_valid") is not True
                or not isinstance(offset, (int, float)) or isinstance(offset, bool)
                or not math.isfinite(offset)
                or not isinstance(time, (int, float)) or isinstance(time, bool)
                or not math.isfinite(time)
                or across_gap(spans, status_processes, time, time)):
            continue
        run = (fields.get("clock_boot_id"), fields.get("clock_resets_count"),
               bisect.bisect_left(lost_ends, time))
        runs.setdefault(run, []).append((time, offset))
    covariance = variance = 0.0
    for samples in runs.values():
        mean_time = statistics.fmean(time for time, _ in samples)
        mean_offset = statistics.fmean(offset for _, offset in samples)
        covariance += sum((time - mean_time) * (offset - mean_offset)
                          for time, offset in samples)
        variance += sum((time - mean_time) ** 2 for time, _ in samples)
    # µs of offset per ms of time, × 1000: µs per s, which is ppm.
    if not variance or not math.isfinite(variance) or not math.isfinite(covariance):
        return None
    ppm = covariance / variance * 1000
    return ppm if math.isfinite(ppm) else None


def alert_summary(records, spans=None):
    """Each alert that was ever active, as `<topic>/<field path>`: its level,
    the stream it was read from, its first activation, activation count, and
    longest and total active time, from the `alerts` log_dump puts on a status
    topic's records (log_dump names the fields; nothing here parses a name).
    Times are publication times (publish_ms), so a reader's receipt delay
    does not stretch an interval: an alert is active from the first record
    showing it true to the first showing it false, or, still active at the
    end, to its topic's last record. An active span that meets a log gap of
    its stream's log (lost_spans) is counted as an activation but timed in
    neither longest nor total (None when no span is left), and
    `spans_across_gap` counts it."""
    summary = {}
    for topic in sorted({r["topic"] for r in records if r.get("alerts")}):
        source, rows = stream(records, topic)
        rows = sorted((r for r in rows if r.get("alerts")), key=lambda r: r["publish_ms"])
        since, intervals, levels = {}, {}, {}
        for r in rows:
            for alert in r["alerts"]:
                name = alert["name"]
                levels[name] = alert["level"]
                intervals.setdefault(name, [])
                if alert["active"] and name not in since:
                    since[name] = r["publish_ms"]
                elif not alert["active"] and name in since:
                    intervals[name].append((since.pop(name), r["publish_ms"]))
        for name, start in since.items():
            intervals[name].append((start, rows[-1]["publish_ms"]))
        for name, active in intervals.items():
            if not active:
                continue
            durations = [end - start for start, end in active
                         if not across_gap(spans or {}, {rows[0]["process"]}, start, end)]
            summary[f"{topic}/{name}"] = {
                "level": levels[name], "stream": source,
                "first_active_ms": active[0][0], "activations": len(active),
                "longest_ms": max(durations, default=None),
                "total_ms": sum(durations) if durations else None,
                "active_at_end": name in since}
            if len(durations) < len(active):
                summary[f"{topic}/{name}"]["spans_across_gap"] = len(active) - len(durations)
    return dict(sorted(summary.items()))


def controller_link(states, state_source, status_records, status_source):
    """Whether the bridge accepted a State from the controller: (measured,
    passes, first accepted State in ms from the start of the log or None,
    detail). The bridge publishes /hw/state only for a State it accepted, so
    its first record is the first acceptance; without one, /hw/status says
    whether any was accepted (state_age_us is 0 and alert_error_link_lost true
    until the first), and the first record after one, less its state_age_us,
    dates it. Under the launcher the bridge's exit status cannot say
    this: the launcher ends a bounded session by stopping it, and a stop
    exits 0 whether or not a State arrived."""
    if states:
        first = states[0]["publish_ms"]
        return True, True, first, (
            f"first accepted State at {first:.6g} ms from the start of the log "
            f"({state_source} /hw/state); the wait before it includes the controller's boot")

    def after_a_state(record):
        fields = scalars(record["decoded"])
        return fields.get("state_age_us", 0) > 0 or fields.get("alert_error_link_lost") is False
    accepted = [r for r in status_records if after_a_state(r)]
    if accepted:
        # A status record follows its State by up to one status period; its
        # state_age_us dates the State itself.
        age_us = scalars(accepted[0]["decoded"]).get("state_age_us", 0)
        first = max(0, accepted[0]["publish_ms"] - age_us / 1000)
        return True, True, first, (
            f"first accepted State at {first:.6g} ms from the start of the log "
            f"(first /hw/status after it less its state_age_us, {status_source}; "
            "no /hw/state record)")
    if status_records:
        span = status_records[-1]["publish_ms"] - status_records[0]["publish_ms"]
        return True, False, None, (
            "the bridge never received a State from the controller: no /hw/state record, "
            f"and {len(status_records)} /hw/status records ({status_source}) over "
            f"{span:.6g} ms report none accepted")
    return False, False, None, "no /hw/state or /hw/status record"


# BridgeStatus UDP socket and incoming State loss counters (REQ-OPS-20,
# TODO.md RT-17, RT-28), surfaced directly in the report alongside /hw/status.
BRIDGE_SOCKET_COUNTERS = (
    "socket_send_failures_count",
    "socket_reopens_count",
    "state_sequence_gaps_count",
    "state_decode_failures_count",
    "socket_send_would_block_count",
)


def bridge_socket_counters(status_summary):
    """Per-counter summary for the BridgeStatus UDP socket / State loss
    counters present in status_summary, or None when none were recorded."""
    found = {key: status_summary[key]
             for key in BRIDGE_SOCKET_COUNTERS if key in status_summary}
    return found or None


def state_sequence_gaps(states, spans):
    """Count missed /hw/state sequences within each gateway boot from decoded
    State records (hardware.boot_id and hardware.sequence, TODO.md RT-28).
    Consecutive pairs across a boot change are not counted; pairs meeting a
    log gap of the state stream's log are skipped and counted in `unmeasured`.
    Returns (summary_or_None, unmeasured)."""
    state_logs = {r["process"] for r in states[:1]}
    sequenced = [r for r in states
                 if isinstance(r.get("hardware"), dict) and "sequence" in r["hardware"]]
    intervals = missed = gap_count = longest_run = unmeasured = 0
    longest_at_ms = None
    for previous, current in zip(sequenced, sequenced[1:]):
        prev_hw, curr_hw = previous["hardware"], current["hardware"]
        if prev_hw.get("boot_id") != curr_hw.get("boot_id"):
            continue
        if across_gap(spans, state_logs, previous["now_ms"], current["now_ms"]):
            unmeasured += 1
            continue
        prev_seq, curr_seq = prev_hw["sequence"], curr_hw["sequence"]
        if curr_seq <= prev_seq:
            continue
        intervals += 1
        skipped = curr_seq - prev_seq - 1
        if skipped > 0:
            missed += skipped
            gap_count += 1
            if skipped > longest_run:
                longest_run = skipped
                longest_at_ms = previous["publish_ms"]
    if intervals == 0:
        return None, unmeasured
    return {"intervals": intervals, "missed": missed, "gaps": gap_count,
            "longest_run": longest_run, "longest_at_ms": longest_at_ms}, unmeasured


def recorded_bound(manifest, key):
    """The manifest's positive uint32 `key` (a [robot] value the launcher
    records), or None when it records none."""
    value = manifest.get(key)
    if (isinstance(value, int) and not isinstance(value, bool)
            and 0 < value <= 0xFFFFFFFF):
        return value
    return None


def analyze(manifest, records, log_damage=(), output_drops=None,
            hardware_not_decoded=None, missing_logs=()):
    if not isinstance(manifest, dict):
        raise ValueError("manifest must be a JSON object")
    manifest_nodes = manifest.get("nodes")
    if not isinstance(manifest_nodes, list) or not all(isinstance(n, dict) for n in manifest_nodes):
        raise ValueError("manifest 'nodes' must be a list of objects")
    # hardware_not_decoded: log_dump's line for a session recorded under
    # another hardware protocol. Its Driver Station packets were not decoded,
    # so nothing that needs the enabled bit is measured; /hw/state cadence
    # uses only record times and still is.
    gap_list = log_gaps(records)
    spans = lost_spans(gap_list)
    source, states = stream(records, "/hw/state")
    _, ds = stream(records, "/hw/state/driver_station")
    # Each stream is one process's records, so a gap of that process's log is
    # the only one that can hide a record of it.
    state_logs = {r["process"] for r in states[:1]}
    ds_logs = {r["process"] for r in ds[:1]}
    # Every packet's publication, whether or not it decodes.
    ds_packets = ds
    ds = [r for r in ds if "ds_enabled" in r]
    ds_times = [r["now_ms"] for r in ds]
    overall, enabled = [], []
    # Intervals not measured because a log gap meets them: of the /hw/state
    # log (in neither row), or of the Driver Station log while the bit is
    # held (neither enabled nor disabled).
    across_state, across_ds = 0, 0

    def held_since(index):
        return ds_times[index] if index >= 0 else -math.inf

    def enabled_at(time):
        # The logged DS enabled bit, held until its next observation; unknown
        # (not enabled) when a DS record could be missing since it.
        index = bisect.bisect_right(ds_times, time) - 1
        return (index >= 0 and ds[index]["ds_enabled"]
                and not across_gap(spans, ds_logs, held_since(index), time))

    state_times = [r["now_ms"] for r in states]

    def ready(state):
        # The bridge's GatewayReady (talOS/hardware/gate.h) of a decoded
        # State, or None when it did not decode.
        hardware = state.get("hardware")
        if not isinstance(hardware, dict) or "enabled" not in hardware:
            return None
        return (hardware.get("configured") is True and hardware["enabled"] is True
                and hardware.get("hardware_fault") is False)

    def held_ready(low, high):
        # Whether the State the bridge held was ready from low through high,
        # or None: no State yet at low, a State record could be missing in
        # between, or readiness changed or is unknown.
        index = bisect.bisect_right(state_times, low) - 1
        if index < 0 or across_gap(spans, state_logs, state_times[index], high):
            return None
        held = {ready(r) for r in states[index:bisect.bisect_right(state_times, high)]}
        return held.pop() if len(held) == 1 and None not in held else None
    for previous, current in zip(states, states[1:]):
        start, end = previous["now_ms"], current["now_ms"]
        if across_gap(spans, state_logs, start, end):
            across_state += 1
            continue
        gap = end - start
        overall.append(gap)
        # Count a gap if any part overlaps a logged enabled interval, including
        # transitions inside the gap. Before the first DS observation is unknown.
        index = bisect.bisect_right(ds_times, start) - 1
        if across_gap(spans, ds_logs, held_since(index), end):
            across_ds += 1
            continue
        active = index >= 0 and ds[index]["ds_enabled"]
        transitions = ds[max(index + 1, 0):bisect.bisect_left(ds_times, end)]
        if active or any(r["ds_enabled"] for r in transitions):
            enabled.append(gap)
    targets, timers, status, tick_ages = {}, {}, {}, {}
    enabled_lane_processes = set()
    for r in records:
        if r["kind"] == "MESSAGE" and is_target_lane(r["topic"]):
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
        if r["kind"] == "READ" and is_target_lane(r["topic"]):
            tick_ages.setdefault(r["process"] + ":" + r["topic"], []).append(
                r["now_ms"] - r["publish_ms"])
        if r["kind"] == "TIMER":
            timers.setdefault(r["process"] + ":" + r["topic"], []).append(
                r["now_ms"] - r["event_ms"])

    def processes(rows):
        return {key.split(":", 1)[0] for key in rows}

    def lost_samples(names):
        # No one-record sample spans a gap, but a gap of its process's log
        # dropped samples: say so where the row is read.
        return "".join(f"; {name} lost {lost['records']} records in {lost['gaps']} log gaps"
                       for name, lost in records_lost.items() if name in names)
    # Per process, the records its log's gaps dropped and how many gaps.
    records_lost = {}
    for gap in sorted(gap_list, key=lambda g: g["process"]):
        lost = records_lost.setdefault(gap["process"], {"records": 0, "gaps": 0})
        lost["records"] += gap["records"]
        lost["gaps"] += 1
    status_source, status_records = stream(records, "/hw/status")
    for r in status_records:
        for field, value in status_scalars(r["decoded"]).items():
            status.setdefault(field, []).append(value)
    status_summary = {}
    for field, values in status.items():
        item = {"samples": len(values), "first": values[0], "last": values[-1],
                "min": min(values), "max": max(values)}
        if field.endswith("_count") and field not in (
                "subsystem_count", "clock_window_count", "can_bus_count"):
            # Negative differences indicate a producer restart/reset, not a
            # negative rejection count. Retain each epoch's observed increments.
            item["delta"] = sum(max(0, b - a) for a, b in zip(values, values[1:]))
            item["resets"] = sum(b < a for a, b in zip(values, values[1:]))
        if field == "clock_offset_valid":
            item["fraction"] = sum(values) / len(values)
        status_summary[field] = item
    verdicts = []
    def verdict(name, measured, passes, detail, required=True, needs_hardware=False):
        # --strict requires every required verdict to PASS. Informational rows
        # have no recorded input or no acceptance threshold and never pass.
        if needs_hardware and hardware_not_decoded:
            measured, detail = False, f"{detail}; {hardware_not_decoded}"
        verdicts.append({"measurement": name, "verdict": "NOT MEASURED" if not measured else
                         "PASS" if passes else "FAIL", "required": required, "detail": detail})
    link_measured, link_passes, first_state_ms, link_detail = controller_link(
        states, source, status_records, status_source)
    verdict("controller link", link_measured, link_passes, link_detail)
    # Named in a row's detail only when a gap left an interval unmeasured.
    across = {"overall": "" if not across_state else
              f"; {across_state} intervals across a /hw/state log gap not measured",
              "enabled": "" if not across_state + across_ds else
              f"; {across_state + across_ds} intervals across a /hw/state or "
              "Driver Station log gap not measured"}
    # The bounds come from the config the session ran (the launcher records
    # its [robot] period_us and command_timeout_us); a manifest from before
    # it did has none, and the rows they judge are not measured.
    period_us = recorded_bound(manifest, "period_us")
    timeout_us = recorded_bound(manifest, "command_timeout_us")
    p99_bound_ms = None if period_us is None else 2 * period_us / 1000
    p99_rule = ("p99 < 2 x period_us; NOT MEASURED: the manifest records no period_us"
                if p99_bound_ms is None else
                f"p99 < {p99_bound_ms:g} ms (2 x period_us {period_us}); nearest rank")
    for name, values in (("overall", overall), ("enabled", enabled)):
        s = stats(values)
        verdict(name + " state median", s is not None,
                s is not None and 4 <= s["median_ms"] <= 6,
                "about 5 ms interpreted as 4–6 ms (analysis tolerance, not an additional "
                "bring-up requirement)" + across[name],
                needs_hardware=name == "enabled")
        verdict(name + " state p99", s is not None and p99_bound_ms is not None,
                s is not None and p99_bound_ms is not None and s["p99_ms"] < p99_bound_ms,
                p99_rule + across[name], needs_hardware=name == "enabled")
    gap_bound_ms = None if timeout_us is None else timeout_us / 1000
    gaps = None if gap_bound_ms is None else sum(v >= gap_bound_ms for v in enabled)
    if gap_bound_ms is None:
        gap_rule = ("gaps >= command_timeout_us; NOT MEASURED: the manifest records no "
                    "command_timeout_us")
    else:
        gap_rule = f"gaps >= {gap_bound_ms:g} ms (the manifest's command_timeout_us {timeout_us})"
        # No count for a row that is not measured: 0 would read as a result.
        if enabled and not hardware_not_decoded:
            gap_rule = f"{gaps} {gap_rule}"
    verdict("enabled state gaps", bool(enabled) and gaps is not None, gaps == 0,
            gap_rule + across["enabled"], needs_hardware=True)
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
    # Without the enabled bit a lane Target never selected cannot be found,
    # so the worst selected age alone would pass a session that has one.
    verdict("target age at tick", worst is not None or bool(unselected),
            worst is not None and not unselected and worst[0] <= TARGET_AGE_LIMIT_MS,
            detail + lost_samples(processes(targets) | processes(tick_ages)),
            needs_hardware=True)
    verdict("timer lateness", False, False,
            "Measured where TIMER dispatches exist; no acceptance threshold is specified."
            + lost_samples(processes(timers)), required=False)
    bridge, bridge_samples, across_bridge = request_to_command(records, spans)
    # Split as the bridge decides its send (CommandDueUs): while its State
    # is not ready, a changed observation waits for the heartbeat. A sample
    # whose readiness is unknown or changes between its ends is in neither.
    bridge_split = {"enabled": [], "disabled": []}
    for requested, commanded in bridge_samples:
        bit = held_ready(requested, commanded)
        if bit is not None:
            bridge_split["enabled" if bit else "disabled"].append(commanded - requested)
    # REQ-RT-25's rows; informational, no verdict (REQ-RT-26 is judged at
    # the bench).
    to_request, across_to_request = state_to_request(states, records, spans)
    to_lane, unanswered, across_to_lane = driver_station_to_lane(ds_packets, records, spans)
    to_command, across_to_command = driver_station_to_command(ds, records, spans,
                                                              enabled_at, bridge)
    seq_gaps, across_seq = state_sequence_gaps(states, spans)
    # The enabled-only rows above stay required (REQ-RT-15); in a session
    # that never enabled they cannot be measured, which is no tool fault.
    never_enabled = None
    if not hardware_not_decoded and not enabled and not any(r["ds_enabled"] for r in ds):
        never_enabled = ("robot never enabled (no Driver Station packet with enabled set)"
                         if ds else "no Driver Station packet recorded")
    issues = [] if manifest.get("end_wall_ns") else [
        "session unfinished (end_wall_ns = 0: no normal shutdown)"]
    # Only recording nodes (non-empty log_path) are acceptance evidence. Host
    # processes without an event log (studio_bridge, sim_gateway) are not.
    issues += [f"{n.get('name')}: {flag}" for n in manifest_nodes if n.get("log_path")
               for flag in ("quarantined", "recorder_failed", "log_capped", "log_gap")
               if n.get(flag)]
    # A record the reader skipped is missing from every measurement above.
    issues += list(log_damage)
    # A manifest-listed event log missing on disk is absent from every
    # measurement above.
    issues += list(missing_logs)
    # Records a writer dropped: only the intervals they could fall in went
    # unmeasured, but the session is not whole evidence.
    issues += [gap_issue(gap) for gap in gap_list]
    if hardware_not_decoded:
        issues.append(hardware_not_decoded)
    simulation = bool(manifest.get("simulation"))
    realtime_issues = [issue for issue in (realtime_issue(n, simulation)
                                           for n in manifest_nodes) if issue]
    delivery = [issue for n in manifest_nodes for issue in delivery_issues(n)]
    # Bridge and control loops --sim excuses: they never asked for realtime.
    realtime_exempt = [n.get("name") for n in manifest_nodes if simulation
                       and n.get("loop_tier") in ACTUATION_TIERS
                       and n.get("realtime") == "not_requested"]
    talos_rev = manifest.get("talos_git_revision")
    talos_rev = talos_rev if isinstance(talos_rev, str) else ""
    talos_revision = (
        talos_rev + (" (dirty)" if manifest.get("talos_git_dirty") else "")
        if talos_rev else ""
    )
    config_sha = manifest.get("config_sha256")
    config_sha = config_sha if isinstance(config_sha, str) and config_sha else "NOT RECORDED (non-bundle launch)"
    git_rev = manifest.get("git_revision")
    git_rev = git_rev if isinstance(git_rev, str) and git_rev else "NOT RECORDED (unstamped or non-bundle)"
    return {"session_id": manifest.get("session_id"),
            "config_sha256": config_sha,
            # A --config=release bundle's source revision; older manifests lack it.
            "git_revision": git_rev + (" (dirty)" if manifest.get("git_dirty") else ""),
            # Present only when a separate robot repo was stamped against a
            # talOS git checkout (REPO-4).
            **({"talos_git_revision": talos_revision} if talos_revision else {}),
            # exit_signal is 0 unless a signal killed the node; older manifests lack it.
            "nodes": [{**{k: n.get(k) for k in ("name", "restart_count", "exit_code")},
                       "exit_signal": n.get("exit_signal", 0),
                       **{k: n.get(k) for k in ("quarantined", "recorder_failed", "log_capped")},
                       # Older manifests lack both (NOT RECORDED).
                       "loop_tier": n.get("loop_tier") or "—",
                       "realtime": realtime_summary(n),
                       "delivery": delivery_summary(n)}
                      for n in manifest_nodes],
            # Every run and why it ended (launcher README "Session manifest");
            # absent in manifests written before the launcher recorded runs.
            "runs": {
                n.get("name"): {
                    "binary_sha256": n.get("binary_sha256") if isinstance(n.get("binary_sha256"), str) else "",
                    "runs": [r for r in n.get("runs", []) if isinstance(r, dict)]
                            if isinstance(n.get("runs"), list) else [],
                    "runs_dropped": n.get("runs_dropped")
                                    if isinstance(n.get("runs_dropped"), int)
                                    and not isinstance(n.get("runs_dropped"), bool)
                                    else 0,
                }
                for n in manifest_nodes
            },
            "session_issues": issues,
            "realtime_issues": realtime_issues,
            "realtime_exempt": realtime_exempt,
            "delivery_issues": delivery,
            # launcher_drops(); informational: output logs are not
            # acceptance evidence, the event logs are.
            "output_drops": output_drops,
            # controller_link(); null unless the bridge accepted a State.
            "first_state_ms": first_state_ms,
            "state_stream": source, "state_overall": stats(overall),
            "state_enabled": stats(enabled),
            # Enabled /hw/state intervals at or past the manifest's
            # command_timeout_us, and that bound; both null when the
            # manifest records none.
            "enabled_gaps": gaps, "enabled_gap_bound_ms": gap_bound_ms,
            # The p99 rows' bound, 2 x the manifest's period_us; null when
            # the manifest records none.
            "state_p99_bound_ms": p99_bound_ms,
            "state_sequence_gaps": seq_gaps,
            "target_receipt_age": {k: stats(v) for k, v in sorted(targets.items())},
            "target_age_at_tick": {k: stats(v) for k, v in sorted(tick_ages.items())},
            "timer_lateness": {k: stats(v) for k, v in sorted(timers.items())},
            "bridge_request_to_command": {"process": bridge,
                                          **{k: stats(v) for k, v in bridge_split.items()}},
            "feedback_age_us": feedback_age(states),
            "state_to_request_ms": {k: stats(v) for k, v in sorted(to_request.items())},
            "driver_station_to_lane_ms": {k: stats(v) for k, v in sorted(to_lane.items())},
            # Per producer, the packets it did not answer before the next,
            # by the packet's enabled bit.
            "driver_station_to_lane_unanswered": unanswered,
            "driver_station_to_command_ms": stats(to_command),
            "clock_drift_ppm": clock_drift_ppm(status_records, spans),
            "never_enabled": never_enabled,
            # log_dump's line, or null: the session's hardware packets were
            # recorded under another protocol and not decoded.
            "hardware_not_decoded": hardware_not_decoded,
            "status_stream": status_source, "status": status_summary,
            "bridge_socket_counters": bridge_socket_counters(status_summary),
            # alert_summary(); informational, as the operator-visibility
            # design has it: --strict does not judge alerts.
            "alerts": alert_summary(records, spans), "verdicts": verdicts,
            "limitations": ["State timing is bridge IPC publication or one consumer dispatch stream, not raw UDP arrival; never pooled duplicate readers.",
                            "Enabled uses the logged DS enabled bit, held until its next observation; DS silence is not proof of continued physical enablement.",
                            "Bridge request to command runs from the requesting tick's start to the command's publication on /hw/command (after the bridge's UDP send succeeds), not its delivery across the network to the RoboRIO; with several owners the merged observation can be another owner's older request, overstating it.",
                            "Alert times are publication times of their status topic: an alert is timed from the first record showing it active to the first showing it clear (or that topic's last record), so a topic published every P ms resolves them to P, and a publisher that stops (a crash) holds its last state until its next record.",
                            "Counter deltas exclude unknown increments before the first sample and during resets; clock mapping is not absolute synchronization.",
                            "Capped, truncated or unfinished sessions cannot establish whole-session acceptance; retain raw logs."],
            # Only in a session with a log gap, so a session without one
            # reports exactly as before: every gap (log_dump's `gap` member
            # and its process), each process's records lost and gaps, and
            # the intervals each row left unmeasured.
            **({"log_gaps": gap_list, "records_lost": records_lost,
                "intervals_across_gaps": {"state_overall": across_state,
                                          "state_enabled": across_state + across_ds,
                                          "state_sequence": across_seq,
                                          "bridge_request_to_command": across_bridge,
                                          "state_to_request": across_to_request,
                                          "driver_station_to_lane": across_to_lane,
                                          "driver_station_to_command": across_to_command}}
               if gap_list else {})}


def strict_failures(report):
    """Reasons a --strict run rejects the session; empty means acceptance."""
    return [f"{v['measurement']}: {v['verdict']}" for v in report["verdicts"]
            if v["required"] and v["verdict"] != "PASS"] + report["session_issues"] \
        + report["realtime_issues"] + report["delivery_issues"]


def stats_row(name, s, unit="ms"):
    """One table row of stats(); NOT MEASURED for a row with no sample."""
    if s is None or not s["count"]:
        return f"| {name} | NOT MEASURED | — | — | — | — |"
    return (f"| {name} | {s['count']} | {s[f'min_{unit}']:.6g} | {s[f'median_{unit}']:.6g} "
            f"| {s[f'p99_{unit}']:.6g} | {s[f'max_{unit}']:.6g} |")


# The REQ-RT-25 rows, each named as the report states it and keyed as in
# intervals_across_gaps.
LATENCY_ROWS = (("state_to_request_ms", "state to request", "state_to_request"),
                ("driver_station_to_lane_ms", "driver station to lane",
                 "driver_station_to_lane"))


def latency_lines(report):
    """The Latency section: REQ-RT-25's rows, informational (no verdict)."""
    lines = ["", "## Latency (REQ-RT-25)",
             "Informational, not judged by --strict. Each value is the difference of two "
             "readings of one clock.", "| Measurement | Count | Min | Median | p99 | Max |",
             "|---|---:|---:|---:|---:|---:|"]
    devices = report["feedback_age_us"]
    # A sentinel (unknown) age is named in the row, never in its figures.
    lines += [stats_row(f"feedback age {name} (µs"
                        + (f"; {s['sentinel']} unknown ages)" if s["sentinel"] else ")"), s, "us")
              for name, s in devices.items()]
    if not devices:
        lines.append(stats_row("feedback age (µs)", None))
    for key, name, _ in LATENCY_ROWS:
        rows = report[key]
        lines += [stats_row(f"{name} {node}", s) for node, s in rows.items()]
        if not rows:
            lines.append(stats_row(name, None))
    lines.append(stats_row("driver station to command, enabled",
                           report["driver_station_to_command_ms"]))
    drift = report["clock_drift_ppm"]
    lines.append("Clock drift: " + ("NOT MEASURED (fewer than two valid clock offsets "
                                    "of one estimator)" if drift is None else
                                    f"{drift:.6g} ppm (slope of clock_offset_us)"))
    across = report.get("intervals_across_gaps", {})
    for _, name, key in LATENCY_ROWS + (("", "driver station to command",
                                         "driver_station_to_command"),):
        if across.get(key):
            lines.append(f"{name[0].upper()}{name[1:]}: {across[key]} samples across a "
                         "log gap not measured.")
    for node, count in report["driver_station_to_lane_unanswered"].items():
        prefix = f"Driver station to lane {node}: packets not answered before the next packet: "
        if count["enabled"] is None:
            lines.append(f"{prefix}{count['total']}; split by enabled NOT MEASURED "
                         "(packets not decoded).")
        elif count["total"]:
            lines.append(f"{prefix}{count['enabled']} while enabled, {count['other']} "
                         "while disabled.")
    return lines


def markdown(report):
    lines = ["# Session report", f"Session: {report['session_id']}",
             f"Config SHA-256: {report['config_sha256']}",
             f"Source revision: {report['git_revision']}"]
    if report.get("talos_git_revision"):
        lines.append(f"talOS revision: {report['talos_git_revision']}")
    lines += ["", "## Nodes",
              "| Node | Restarts | Exit code | Exit signal | quarantined | recorder_failed | log_capped "
              "| Loop tier | Realtime | Delivery loss |",
              "|---|---:|---:|---:|---|---|---|---|---|---|"]
    for n in report["nodes"]:
        lines.append("| " + " | ".join(str(n[k]) for k in n) + " |")
    lines += ["", "## Runs", "| Node | Binary SHA-256 | Run | PID | Seconds | Exit code | Exit signal | Reason |",
              "|---|---|---:|---:|---:|---:|---:|---|"]
    for name, node in report["runs"].items():
        for index, run in enumerate(node["runs"]):
            ended = run.get("end_wall_ns")
            started = run.get("start_wall_ns")
            if (isinstance(ended, (int, float)) and not isinstance(ended, bool) and ended
                    and isinstance(started, (int, float)) and not isinstance(started, bool)):
                seconds = f"{(ended - started) / 1e9:.3f}"
            else:
                seconds = "running"
            # A run after the dropped ones is numbered as it was spawned.
            number = index if index == 0 else index + node["runs_dropped"]
            lines.append(f"| {name} | {node['binary_sha256'][:12] or '—'} | {number} | "
                         f"{run.get('pid')} | {seconds} | {run.get('exit_code')} | "
                         f"{run.get('exit_signal')} | {run.get('reason')} |")
        if node["runs_dropped"]:
            lines.append(f"| {name} | | | | | | | {node['runs_dropped']} runs after the first dropped |")
    lines += ["", "## Session completeness"] + (
        ["- " + s for s in report["session_issues"]] or ["- complete"])
    judged = ("- no bridge or control loop that reported an outcome ran without its full grant"
              if not report["realtime_exempt"] else
              "- no other bridge or control loop that reported an outcome ran without its full grant")
    lines += ["", "## Realtime scheduling"] + (
        ["- " + s for s in report["realtime_issues"]] or [judged])
    if report["realtime_exempt"]:
        lines.append("- not judged (--sim, realtime never requested): "
                     + ", ".join(str(x) for x in report["realtime_exempt"]))
    lines += ["", "## Delivery loss"] + [f"- {n['name']}: {n['delivery']}"
                                         for n in report["nodes"]]
    drops = report["output_drops"]
    lines += ["", "## Launcher output"] + (
        ["- NOT RECORDED (the manifest names no launcher log)"] if drops is None else
        ["- " + s for s in drops] or ["- talos.log notes no dropped output"])
    lines += ["", "## Timing (ms)", f"State stream: {report['state_stream'] or 'NOT MEASURED'}",
              "| Measurement | Count | Min | Median | p99 | Max |",
              "|---|---:|---:|---:|---:|---:|"]
    entries = {"/hw/state overall": report["state_overall"],
               "/hw/state enabled": report["state_enabled"],
               **{"receipt " + k: v for k, v in report["target_receipt_age"].items()},
               **{"target at tick " + k: v for k, v in report["target_age_at_tick"].items()},
               **{"timer " + k: v for k, v in report["timer_lateness"].items()}}
    bridge = report["bridge_request_to_command"]
    if bridge["process"]:
        for split in ("enabled", "disabled"):
            entries[f"bridge request to command {bridge['process']} {split}"] = bridge[split]
    lines += [stats_row(name, s) for name, s in entries.items()]
    seq = report["state_sequence_gaps"]
    if seq is None:
        lines.append("State sequence gaps: NOT MEASURED (no decoded /hw/state sequence pairs).")
    elif seq["missed"]:
        lines.append(
            f"State sequence gaps: {seq['missed']} missed states across {seq['gaps']} gaps "
            f"(longest consecutive gap: {seq['longest_run']} states at "
            f"{seq['longest_at_ms']:.6g} ms).")
    else:
        lines.append(f"State sequence gaps: 0 missed states across {seq['intervals']} intervals.")
    if not report["target_receipt_age"]:
        lines.append("Target receipt age: NOT MEASURED (no target MESSAGE dispatches).")
    if not report["target_age_at_tick"]:
        lines.append("Target age at tick: NOT MEASURED (no Target selected a lane).")
    for n in report["nodes"]:
        if any(k.startswith(f"{n['name']}:") for k in report["timer_lateness"]):
            continue
        if n["name"] == bridge["process"]:
            # The bridge sleeps until a datagram, a request or a duty is due.
            lines.append(f"{n['name']}: no timer (event-driven bridge); "
                         "see bridge request to command.")
        else:
            lines.append(f"{n['name']} timer lateness: NOT MEASURED (no TIMER dispatches).")
    if not bridge["process"]:
        lines.append("Bridge request to command: NOT MEASURED (no /hw/command SEND records).")
    sampled = {key.split(":", 1)[0] for rows in ("target_receipt_age", "target_age_at_tick",
                                                 "timer_lateness") for key in report[rows]}
    for name, lost in report.get("records_lost", {}).items():
        if name in sampled:
            lines.append(f"{name} lost {lost['records']} records in {lost['gaps']} log gaps: "
                         "its target receipt, target at tick and timer rows lack the samples "
                         "those held.")
    across = report.get("intervals_across_gaps", {}).get("bridge_request_to_command")
    if across:
        lines.append(f"Bridge request to command: {across} samples across a log gap "
                     "not measured.")
    lines += latency_lines(report)
    lines += ["", "## /hw/status", f"Stream: {report['status_stream'] or 'NOT MEASURED'}"]
    if report["bridge_socket_counters"] is not None:
        lines.append(
            "Bridge UDP/State loss counters: "
            + ", ".join(
                f"{field}=delta:{values['delta']}/last:{values['last']}"
                + (f" (resets={values['resets']})" if values["resets"] else "")
                for field, values in report["bridge_socket_counters"].items()
            )
        )
    for field, values in report["status"].items():
        lines.append(f"- {field}: " + ", ".join(f"{k}={v}" for k, v in values.items()))
    lines += ["", "## Alerts",
              "Informational, not judged by --strict. Publication times in ms; first "
              "activation from the start of the log."]
    if report["alerts"]:
        lines += ["| Alert | Level | First active | Activations | Longest | Total | Active at end |",
                  "|---|---|---:|---:|---:|---:|---|"]
        def timed(value):
            # None: every active span met a log gap (alert_summary).
            return "NOT MEASURED" if value is None else f"{value:.6g}"
        lines += [f"| {name} | {a['level']} | {a['first_active_ms']:.6g} | {a['activations']} | "
                  f"{timed(a['longest_ms'])} | {timed(a['total_ms'])} | "
                  f"{'yes' if a['active_at_end'] else 'no'} |"
                  for name, a in report["alerts"].items()]
    else:
        lines.append("- no alert was active")
    lines += ["", "## Verdicts"]
    lines += [f"- **{v['verdict']}** {v['measurement']}" +
              ("" if v["required"] else " (informational, not required)") + f": {v['detail']}"
              for v in report["verdicts"]]
    if report["never_enabled"]:
        lines.append("- Enabled state and target age at tick are NOT MEASURED: "
                     f"{report['never_enabled']}; --strict still fails them, so "
                     "acceptance needs an enabled session.")
    lines += ["", "## Measurement limits"] + ["- " + s for s in report["limitations"]]
    return "\n".join(lines)


def log_dump_path():
    """log_dump from this tool's runfiles, in whichever repository holds
    talOS: the main one (`_main`) or a dependency (its canonical name)."""
    files = runfiles.Create()
    repository = (files.CurrentRepository() or "_main") if files else ""
    binary = files.Rlocation(f"{repository}/talOS/launcher/log_dump") if files else None
    if not binary or not Path(binary).is_file():
        raise FileNotFoundError(
            "log_dump runfile not found; run with `bazel run //talOS/launcher:session_report` "
            "in talOS, or `bazel run @talos//talOS/launcher:session_report` in a "
            "repository that depends on it")
    return binary


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session", type=Path)
    parser.add_argument("--json", action="store_true")
    parser.add_argument("--strict", action="store_true",
                        help="exit 2 unless every required verdict passes and the session is complete")
    args = parser.parse_args()
    try:
        manifest = load_manifest(args.session / "manifest.json")
        result = subprocess.run([log_dump_path(), "--format", "jsonl", str(args.session)],
                                capture_output=True, text=True, errors="replace", check=True)
        not_decoded = HARDWARE_NOT_DECODED.search(result.stderr)
        for disagreement in HARDWARE_PROTOCOL_DISAGREES.findall(result.stderr):
            print(f"session_report: {disagreement}", file=sys.stderr)
        records = [json.loads(line, parse_constant=_reject_non_finite_json)
                   for line in result.stdout.splitlines()]
        report = analyze(manifest, records,
                         DAMAGED_LOG.findall(result.stderr),
                         launcher_drops(args.session, manifest),
                         not_decoded[1] if not_decoded else None,
                         missing_logs(args.session, manifest))
        print(json.dumps(report, indent=2, allow_nan=False) if args.json else markdown(report))
        if not args.strict:
            return 0
        failures = strict_failures(report)
        for failure in failures:
            print(f"session_report: strict: {failure}", file=sys.stderr)
        return 2 if failures else 0
    except (OSError, ValueError, KeyError, TypeError, RecursionError,
            subprocess.CalledProcessError) as error:
        print(f"session_report: {error}", file=sys.stderr)
        if isinstance(error, subprocess.CalledProcessError):
            print(error.stderr, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
