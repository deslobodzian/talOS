"""Lists the tests whose last run took longer than a threshold, slowest first.

    python3 tools/slow_tests.py                 # bazel-testlogs, over 3 s
    python3 tools/slow_tests.py --threshold 5
    python3 tools/slow_tests.py --bep events.json

By default it reads every `test.xml` under `bazel-testlogs` (from the
workspace root), so it needs no extra `bazel test` flag: each file is the
last run of one test action (a target, or one of its shards or
`--runs_per_test` runs), cached or not. A googletest file gives the
binary's own time; for any other test Bazel writes the file itself with the
action's wall time in whole seconds, refined by Python unittest's own
"Ran N tests in X s" when the log holds one. With `--bep FILE` (written by
`bazel test --build_event_json_file=FILE`) it reads that invocation's
attempt durations instead, in milliseconds.

The budget is docs/TESTING.md §9. It reports and exits 0; it is not a check.
"""

import argparse
import json
import os
import re
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


def from_testlogs(testlogs):
    """(label, seconds) for each test.xml under the `testlogs` tree."""
    results = []
    for directory, _, files in os.walk(testlogs, followlinks=True):
        if "test.xml" not in files:
            continue
        parts = os.path.relpath(directory, testlogs).split(os.sep)
        suffix = ""
        if len(parts) > 1 and ACTION_DIR.match(parts[-1]):
            suffix = " (" + parts.pop() + ")"
        if len(parts) < 2:
            continue
        label = "//" + "/".join(parts[:-1]) + ":" + parts[-1] + suffix
        with open(os.path.join(directory, "test.xml"), encoding="utf-8",
                  errors="replace") as handle:
            try:
                seconds = xml_seconds(handle.read())
            except ElementTree.ParseError:
                continue
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
    """(label, seconds) for each test attempt in a BEP JSON stream."""
    results = []
    for line in lines:
        if not line.strip():
            continue
        event = json.loads(line)
        key = event.get("id", {}).get("testResult")
        seconds = duration_seconds(event.get("testResult", {}))
        if key is None or seconds is None:
            continue
        label = key["label"]
        tags = [f"{name} {key[name]}" for name in ("shard", "run", "attempt")
                if key.get(name, 1) != 1]
        results.append((label + (f" ({', '.join(tags)})" if tags else ""), seconds))
    return results


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
    args = parser.parse_args(argv)
    if args.bep:
        with open(args.bep, encoding="utf-8") as handle:
            results = from_bep(handle)
    else:
        if not os.path.isdir(args.testlogs):
            parser.error(f"{args.testlogs}: no test logs; run bazel test first")
        results = from_testlogs(args.testlogs)
    report(results, args.threshold, sys.stdout)
    return 0


if __name__ == "__main__":
    sys.exit(main())
