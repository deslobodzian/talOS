"""Lints tests and node code for wall-clock time and test retries.

    python3 tools/lint_test_time.py   # from the workspace root

docs/TESTING.md §6.1 and §8.1, docs/METHODOLOGY.md §9 (REQ-RT-17). Over the
tracked files it finds:

- `sleep`: a sleep call (`sleep_for`, `sleep_until`, `usleep`, `nanosleep`,
  `sleep(`, `time.sleep(`) in a test source or a node package's source;
- `clock`: a wall-clock read (`steady_clock::now`, `system_clock::now`,
  `high_resolution_clock::now`, `time.time(`, `time.monotonic(`) in the same
  files;
- `retry`: `flaky = True` or `--flaky_test_attempts` in a BUILD or `.bzl`
  file, `.bazelrc`, a workflow or a shell script.

A test source is a `*_test.cc`, `*_test.py` or `test_*.py` file. A node
package is a directory whose BUILD declares a node (talos_cc_node,
talos_py_node, talos_cc_shared_node) under 2026-robot, bench, examples or
tools/scaffold_testdata; a node package's tests are test sources. Comment
lines are skipped.

An integration, e2e, soak or perf test may use the real clock (§2); it is
listed in tools/lint_test_time_allowlist.txt, one `<path> <kind> <reason>`
per line, where kind is sleep, clock or retry. A hit not listed fails, and
so does a listed (path, kind) with no hit left, so the list only shrinks.
Exits 1 on either.
"""

import os
import re
import subprocess
import sys

ALLOWLIST = "tools/lint_test_time_allowlist.txt"

NODE_ROOTS = ("2026-robot/", "bench/", "examples/", "tools/scaffold_testdata/")
NODE_MACRO = re.compile(r"\b(talos_cc_node|talos_py_node|talos_cc_shared_node)"
                        r"\s*\(")
SOURCE_SUFFIXES = (".cc", ".h", ".cpp", ".hpp", ".py")

TIME_PATTERNS = {
    "sleep": re.compile(r"\b(sleep_for|sleep_until|usleep|nanosleep|sleep)"
                        r"\s*\("),
    "clock": re.compile(r"\b(steady_clock|system_clock|high_resolution_clock)"
                        r"::now\s*\(|\btime\.(time|monotonic)\s*\("),
}
RETRY_PATTERN = re.compile(r"\bflaky\s*=\s*True\b|--flaky_test_attempts")


def is_test_source(path):
    name = os.path.basename(path)
    return (name.endswith(("_test.cc", "_test.py")) or
            (name.startswith("test_") and name.endswith(".py")))


def is_retry_config(path):
    name = os.path.basename(path)
    return (name in ("BUILD", "BUILD.bazel", ".bazelrc") or
            name.endswith((".bzl", ".sh")) or
            (path.startswith(".github/workflows/") and
             name.endswith((".yaml", ".yml"))))


def node_packages(files, read):
    """Directories under NODE_ROOTS whose BUILD declares a node."""
    packages = set()
    for path in files:
        if (path.startswith(NODE_ROOTS) and
                os.path.basename(path) in ("BUILD", "BUILD.bazel") and
                NODE_MACRO.search(read(path))):
            packages.add(os.path.dirname(path))
    return packages


def in_package(path, packages):
    """True when `path` sits directly in one of `packages` (not below)."""
    return os.path.dirname(path) in packages


def is_comment(line):
    stripped = line.lstrip()
    return stripped.startswith(("//", "#", "*", "/*"))


def scan(files, read):
    """{(path, kind): [line numbers]} of every hit in `files`."""
    hits = {}
    packages = node_packages(files, read)
    for path in files:
        patterns = {}
        if is_test_source(path) or (path.endswith(SOURCE_SUFFIXES) and
                                    in_package(path, packages)):
            patterns.update(TIME_PATTERNS)
        if is_retry_config(path):
            patterns["retry"] = RETRY_PATTERN
        if not patterns:
            continue
        for number, line in enumerate(read(path).splitlines(), 1):
            if is_comment(line):
                continue
            for kind, pattern in patterns.items():
                if pattern.search(line):
                    hits.setdefault((path, kind), []).append(number)
    return hits


def parse_allowlist(text):
    """({(path, kind): reason}, [errors]) from the allowlist's text."""
    entries, errors = {}, []
    for number, line in enumerate(text.splitlines(), 1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split(None, 2)
        if len(fields) < 3 or fields[1] not in ("sleep", "clock", "retry"):
            errors.append(f"{ALLOWLIST}:{number}: want `<path> sleep|clock|"
                          f"retry <reason>`: {line}")
            continue
        entries[(fields[0], fields[1])] = fields[2]
    return entries, errors


def lint(files, read, allowlist_text, out=sys.stdout):
    """Prints each failure; returns how many there were."""
    hits = scan(files, read)
    allowed, errors = parse_allowlist(allowlist_text)
    for error in errors:
        print(error, file=out)
    failures = len(errors)
    for (path, kind), lines in sorted(hits.items()):
        if (path, kind) not in allowed:
            failures += 1
            where = ", ".join(str(n) for n in lines)
            print(f"{path}:{where}: {kind} (docs/TESTING.md §6.1, §8.1; "
                  f"METHODOLOGY.md §9). Wait on an event, take the clock as "
                  f"a parameter, or list it in {ALLOWLIST} with its reason.",
                  file=out)
    for path, kind in sorted(set(allowed) - set(hits)):
        failures += 1
        print(f"{ALLOWLIST}: stale entry `{path} {kind}`: no such hit left; "
              f"delete the line", file=out)
    if not failures:
        print(f"no new test-time or retry hits ({len(allowed)} allowlisted)",
              file=out)
    return failures


def tracked_files():
    listing = subprocess.run(["git", "ls-files", "-z"], capture_output=True,
                             check=True).stdout.decode()
    return [path for path in listing.split("\0")
            if path and not path.startswith("third_party/")]


def read_file(path):
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            return f.read()
    except (FileNotFoundError, IsADirectoryError):
        return ""


def main():
    files = tracked_files()
    return 1 if lint(files, read_file, read_file(ALLOWLIST)) else 0


if __name__ == "__main__":
    sys.exit(main())
