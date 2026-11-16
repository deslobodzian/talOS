"""Lists the tests whose last run took longer than a threshold, slowest first.

    python3 tools/slow_tests.py                 # bazel-testlogs, over 3 s
    python3 tools/slow_tests.py --threshold 5
    python3 tools/slow_tests.py --bep events.json
    python3 tools/slow_tests.py --check          # fail on a lane budget
    python3 tools/slow_tests.py --check --lanes lanes.txt
    python3 tools/slow_tests.py --check --bep events.json --enforce heavy,e2e

By default it reads every test action's result under `bazel-testlogs` (from
the workspace root), so it needs no extra `bazel test` flag: each directory
is the last run of one test action (a target, or one of its shards or
`--runs_per_test` runs), cached or not. An action's time is the wall time
Bazel records in its `test.cache_status` (`TestResultData`, Bazel's
src/main/protobuf/test_status.proto, field 12 `run_duration_millis`), in
milliseconds, the time Bazel prints after PASSED; without that file, its
`test.xml`'s: googletest's own time, or Bazel's whole seconds refined by
Python unittest's "Ran N tests in X s".

`bazel-testlogs` is whatever the latest bazel invocation pointed it at,
which another configuration's run can leave empty or stale. To read one
invocation's results, pass `--testlogs` (that invocation's `bazel info
bazel-testlogs`) or `--bep FILE` (written by `bazel test
--build_event_json_file=FILE`): its attempt durations, in milliseconds, and
each target's tags. Finding no test result at all is an error.

The budgets are docs/TESTING.md §9. Without `--check` it reports and exits
0. With `--check` it exits 1 when a test action exceeds its lane's budget: a
target with no lane tag is in the fast lane (5 s). The lane tags come from
`--lanes FILE` (lines of `<label> <lane>`; unlisted labels are fast), else
from the `--bep` file's target tags (an untagged target is fast), else from
one `bazel query` per lane; a query that fails or finds no lane-tagged test
is an error.
`--enforce LANE,...` fails only those lanes' overruns and prints the others.
"""

import argparse
import json
import os
import re
import subprocess
import sys
import xml.etree.ElementTree as ElementTree

# Python unittest's summary line, in the log Bazel copies into its test.xml.
UNITTEST_RAN = re.compile(r"^Ran \d+ tests? in ([0-9.]+)s$", re.MULTILINE)

# The directory Bazel gives one shard or one run of a target.
ACTION_DIR = re.compile(r"^(shard_\d+_of_\d+|run_\d+_of_\d+)(_run_\d+_of_\d+)?$")


def xml_seconds(text):
    """The run time a test.xml records, or None when it records none."""
    root = ElementTree.fromstring(text)
    if root.get("time"):
        return float(root.get("time"))
    suites = [root] if root.tag == "testsuite" else root.findall("testsuite")
    timed = [float(s.get("time")) for s in suites if s.get("time")]
    if timed:
        return sum(timed)
    cases = [float(c.get("time")) for c in root.iter("testcase") if c.get("time")]
    if not cases:
        return None
    # Bazel's own file: whole seconds, rounded down.
    ran = [float(m) for m in UNITTEST_RAN.findall(text)]
    return max(sum(cases), sum(ran))


def varint(data, offset):
    """(value, next offset) of the protobuf varint at `offset`."""
    value = shift = 0
    while True:
        byte = data[offset]
        offset += 1
        value |= (byte & 0x7F) << shift
        shift += 7
        if byte < 0x80:
            return value, offset


# TestResultData.run_duration_millis (Bazel's test_status.proto).
RUN_DURATION_MILLIS = 12


def cache_status_seconds(data):
    """The run time a test.cache_status records, or None when it has none."""
    offset, seconds = 0, None
    try:
        while offset < len(data):
            key, offset = varint(data, offset)
            field, wire = key >> 3, key & 7
            if wire == 0:
                value, offset = varint(data, offset)
                if field == RUN_DURATION_MILLIS:
                    seconds = value / 1000
            elif wire == 1:
                offset += 8
            elif wire == 2:
                length, offset = varint(data, offset)
                offset += length
            elif wire == 5:
                offset += 4
            else:
                return None
    except IndexError:
        return None
    return seconds


def action_seconds(directory, files):
    """One test action's run time from its test logs directory, or None."""
    if "test.cache_status" in files:
        with open(os.path.join(directory, "test.cache_status"), "rb") as handle:
            seconds = cache_status_seconds(handle.read())
        if seconds is not None:
            return seconds
    if "test.xml" not in files:
        return None
    with open(os.path.join(directory, "test.xml"), encoding="utf-8",
              errors="replace") as handle:
        try:
            return xml_seconds(handle.read())
        except ElementTree.ParseError:
            return None


def from_testlogs(testlogs):
    """(label, seconds) for each test action under the `testlogs` tree."""
    results = []
    for directory, _, files in os.walk(testlogs, followlinks=True):
        if "test.xml" not in files and "test.cache_status" not in files:
            continue
        parts = os.path.relpath(directory, testlogs).split(os.sep)
        suffix = ""
        if len(parts) > 1 and ACTION_DIR.match(parts[-1]):
            suffix = " (" + parts.pop() + ")"
        if len(parts) < 2:
            continue
        label = "//" + "/".join(parts[:-1]) + ":" + parts[-1] + suffix
        seconds = action_seconds(directory, files)
        if seconds is not None:
            results.append((label, seconds))
    return results


def duration_seconds(result):
    """A BEP testResult's attempt duration: milliseconds, or "12.3s"."""
    if "testAttemptDurationMillis" in result:
        return int(result["testAttemptDurationMillis"]) / 1000
    duration = result.get("testAttemptDuration")
    if isinstance(duration, str) and duration.endswith("s"):
        return float(duration[:-1])
    return None


def from_bep(lines):
    """([(label, seconds)] for each test attempt, {label: lane} for each
    completed target) from a BEP JSON stream."""
    results, lanes = [], {}
    for line in lines:
        if not line.strip():
            continue
        event = json.loads(line)
        completed = event.get("id", {}).get("targetCompleted")
        if completed:
            # proto3 JSON omits an empty `tag`: an untagged target is fast.
            lanes[completed["label"]] = lane_of(
                event.get("completed", {}).get("tag", []))
        key = event.get("id", {}).get("testResult")
        seconds = duration_seconds(event.get("testResult", {}))
        if key is None or seconds is None:
            continue
        label = key["label"]
        tags = [f"{name} {key[name]}" for name in ("shard", "run", "attempt")
                if key.get(name, 1) != 1]
        results.append((label + (f" ({', '.join(tags)})" if tags else ""), seconds))
    return results, lanes


# Lane budgets in seconds (docs/TESTING.md §9). An untagged test is fast.
BUDGETS = {"fast": 5.0, "heavy": 120.0, "integration": 30.0, "e2e": 180.0,
           "perf": 60.0}
LANE_TAGS = ("heavy", "integration", "e2e", "perf")


def lane_of(tags):
    """The lane a target's tags put it in."""
    return next((lane for lane in LANE_TAGS if lane in tags), "fast")


def read_lanes(lines):
    """{label: lane} from `<label> <lane>` lines."""
    lanes = {}
    for line in lines:
        fields = line.split()
        if len(fields) == 2:
            lanes[fields[0]] = fields[1]
    return lanes


def query_lanes():
    """{label: lane} for every test carrying a lane tag, from bazel query.

    Exits when a query fails (any exit but 0, or 3: a partial result under
    --keep_going) or when no test carries a lane tag: either would put every
    test in the fast lane and pass an enforced lane's overrun."""
    lanes = {}
    for lane in LANE_TAGS:
        expression = f"attr(tags, '[\\[ ]{lane}[,\\]]', tests(//...))"
        query = subprocess.run(
            ["bazel", "query", "--keep_going", "--noshow_progress", expression],
            capture_output=True, text=True, check=False)
        if query.returncode not in (0, 3):
            sys.exit(f"slow_tests: bazel query for the {lane} lane exited "
                     f"{query.returncode}; lanes unknown\n{query.stderr}")
        for label in query.stdout.split():
            lanes[label] = lane
    if not lanes:
        sys.exit("slow_tests: bazel query found no test in any lane "
                 f"({', '.join(LANE_TAGS)}); lanes unknown")
    return lanes


def over_budget(results, lanes):
    """(label, seconds, lane) for each action over its lane's budget."""
    over = []
    for label, seconds in results:
        lane = lanes.get(label.split(" ", 1)[0], "fast")
        if seconds > BUDGETS.get(lane, BUDGETS["fast"]):
            over.append((label, seconds, lane))
    return sorted(over, key=lambda r: (-r[1], r[0]))


def report(results, threshold, out):
    """Prints the results over `threshold` seconds, slowest first."""
    slow = sorted((r for r in results if r[1] > threshold),
                  key=lambda r: (-r[1], r[0]))
    for label, seconds in slow:
        print(f"{seconds:8.1f} s  {label}", file=out)
    print(f"{len(slow)} of {len(results)} test actions over {threshold:g} s",
          file=out)
    return slow


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n", 1)[0])
    parser.add_argument("--threshold", type=float, default=3.0,
                        help="seconds; report tests slower than this (default 3)")
    parser.add_argument("--testlogs", default="bazel-testlogs",
                        help="the test logs tree (default bazel-testlogs)")
    parser.add_argument("--bep", help="a --build_event_json_file to read instead")
    parser.add_argument("--check", action="store_true",
                        help="exit 1 when a test exceeds its lane's budget")
    parser.add_argument("--lanes", help="`<label> <lane>` lines (default: the "
                        "--bep file's tags, else bazel query)")
    parser.add_argument("--enforce", default=",".join(BUDGETS),
                        help="comma-separated lanes whose overruns fail "
                        "--check (default: every lane)")
    args = parser.parse_args(argv)
    enforced = set(filter(None, args.enforce.split(",")))
    if not enforced <= set(BUDGETS):
        parser.error(f"--enforce: unknown lane in {args.enforce!r}; lanes are "
                     f"{', '.join(BUDGETS)}")
    bep_lanes = {}
    if args.bep:
        with open(args.bep, encoding="utf-8") as handle:
            results, bep_lanes = from_bep(handle)
        source = args.bep
    else:
        if not os.path.isdir(args.testlogs):
            parser.error(f"{args.testlogs}: no test logs; run bazel test first")
        results = from_testlogs(args.testlogs)
        source = args.testlogs
    if not results:
        parser.error(f"{source}: no test results; pass the invocation's "
                     "--testlogs or --bep")
    if not args.check:
        report(results, args.threshold, sys.stdout)
        return 0
    if args.lanes:
        with open(args.lanes, encoding="utf-8") as handle:
            lanes = read_lanes(handle)
    elif bep_lanes and all(label.split(" ", 1)[0] in bep_lanes
                           for label, _ in results):
        lanes = bep_lanes
    else:
        lanes = query_lanes()
    over = over_budget(results, lanes)
    failed = [row for row in over if row[2] in enforced]
    for label, seconds, lane in over:
        note = "" if lane in enforced else " (not enforced)"
        print(f"{seconds:8.1f} s  {label}  over the {lane} lane's "
              f"{BUDGETS.get(lane, BUDGETS['fast']):g} s{note}", file=sys.stdout)
    print(f"{len(over)} of {len(results)} test actions over their lane budget, "
          f"{len(failed)} in an enforced lane ({', '.join(sorted(enforced))})")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
