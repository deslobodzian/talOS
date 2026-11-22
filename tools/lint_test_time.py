"""Lints tests and node code for wall-clock time and test retries.

    python3 tools/lint_test_time.py   # from the workspace root

docs/TESTING.md §6.1 and §8.1, docs/METHODOLOGY.md §9 (REQ-RT-17). Over the
tracked files it finds:

- `sleep`: a sleep call (`sleep_for`, `sleep_until`, `usleep`, `nanosleep`,
  `sleep(`, `time.sleep(`) in a test source or a node package's source;
- `clock`: a wall-clock read (`steady_clock::now`, `system_clock::now`,
  `high_resolution_clock::now`, the same through an alias the file declares,
  `using Clock = std::chrono::steady_clock;` then `Clock::now()`,
  `time.time(`, `time.monotonic(`) in the same files;
- `retry`: `flaky = True` or `--flaky_test_attempts` in a BUILD or `.bzl`
  file, `.bazelrc`, a workflow or a shell script.

A test source is a `*_test.cc`, `*_test.cpp`, `*_test.py`, `test_*.py`,
`test.cc`, `test.cpp` or `test.py` file (`*_test.cpp`: the RoboRIO program's
GradleRIO tests), and so is a
test helper a test includes or imports: a source whose stem ends in
`_test_support`, `_test_helpers` or `_testing`, or is `testing` (the
framework's `Sim` headers and module, which own simulated time), or a
header under a `testdata` directory. A node
package is a directory whose BUILD declares a node (talos_cc_node,
talos_py_node, talos_cc_shared_node) under bench, examples or
tools/scaffold_testdata, or one of the framework nodes' packages
(talOS/teleop, talOS/telemetry); a node package's tests are test sources.
Other talOS packages that declare a node (talOS/launcher, talOS/node,
talOS/node_api) do so only for test fixtures beside framework code that
runs on the real clock, so only their test sources are scanned.
Studio's TypeScript tests (`studio/tests/**/*.ts`, `*.tsx`) are test
sources too: there `sleep` is a `setTimeout(` call and `clock` a
`Date.now(` or `performance.now(` call. Comment lines are skipped.

An integration, e2e, soak or perf test may use the real clock (§2); it is
listed in tools/lint_test_time_allowlist.txt, one `<path> <kind> [<count>]
<reason>` per line, where kind is sleep, clock or retry and count, a
positive number, is how many lines of that file hit that kind (a reason
therefore does not start with a digit). A hit not listed fails, and so does
a listed (path, kind) with no hit left; with a count, so do more hits than
it, or fewer (lower the count), so the list only shrinks. Exits 1 on any.
"""

import os
import re
import subprocess
import sys

ALLOWLIST = "tools/lint_test_time_allowlist.txt"

NODE_ROOTS = ("bench/", "examples/", "tools/scaffold_testdata/")
# Framework nodes: packages outside NODE_ROOTS whose sources are node code.
FRAMEWORK_NODE_PACKAGES = ("talOS/teleop", "talOS/telemetry")
STUDIO_TESTS = "studio/tests/"
NODE_MACRO = re.compile(r"\b(talos_cc_node|talos_py_node|talos_cc_shared_node)"
                        r"\s*\(")
SOURCE_SUFFIXES = (".cc", ".h", ".cpp", ".hpp", ".py")

TIME_PATTERNS = {
    "sleep": re.compile(r"\b(sleep_for|sleep_until|usleep|nanosleep|sleep)"
                        r"\s*\("),
    "clock": re.compile(r"\b(steady_clock|system_clock|high_resolution_clock)"
                        r"::now\s*\(|\btime\.(time|monotonic)\s*\("),
}
# `using Clock = std::chrono::steady_clock;` or a typedef of one: the alias's
# `::now(` is a clock read too.
CLOCK_ALIAS = re.compile(r"\busing\s+(\w+)\s*=\s*(?:std\s*::\s*)?"
                         r"(?:chrono\s*::\s*)?"
                         r"(?:steady_clock|system_clock|high_resolution_clock)"
                         r"\s*;|\btypedef\s+(?:std\s*::\s*)?"
                         r"(?:chrono\s*::\s*)?"
                         r"(?:steady_clock|system_clock|high_resolution_clock)"
                         r"\s+(\w+)\s*;")
TS_TIME_PATTERNS = {
    "sleep": re.compile(r"\bsetTimeout\s*\("),
    "clock": re.compile(r"\b(Date|performance)\.now\s*\("),
}
RETRY_PATTERN = re.compile(r"\bflaky\s*=\s*True\b|--flaky_test_attempts")


def is_test_source(path):
    name = os.path.basename(path)
    return (name in ("test.cc", "test.cpp", "test.py") or
            name.endswith(("_test.cc", "_test.cpp", "_test.py")) or
            (name.startswith("test_") and name.endswith(".py")))


def is_test_helper(path):
    stem, suffix = os.path.splitext(os.path.basename(path))
    return suffix in SOURCE_SUFFIXES and (
        stem == "testing" or
        stem.endswith(("_test_support", "_test_helpers", "_testing")) or
        (suffix in (".h", ".hpp") and "testdata" in path.split("/")[:-1]))


def is_studio_test(path):
    return path.startswith(STUDIO_TESTS) and path.endswith((".ts", ".tsx"))


def is_retry_config(path):
    name = os.path.basename(path)
    return (name in ("BUILD", "BUILD.bazel", ".bazelrc") or
            name.endswith((".bzl", ".sh")) or
            (path.startswith(".github/workflows/") and
             name.endswith((".yaml", ".yml"))))


def node_packages(files, read):
    """Directories under NODE_ROOTS whose BUILD declares a node, and the
    framework nodes' packages."""
    packages = set(FRAMEWORK_NODE_PACKAGES)
    for path in files:
        if (path.startswith(NODE_ROOTS) and
                os.path.basename(path) in ("BUILD", "BUILD.bazel") and
                NODE_MACRO.search(read(path))):
            packages.add(os.path.dirname(path))
    return packages


def in_package(path, packages):
    """True when `path` sits directly in one of `packages` (not below)."""
    return os.path.dirname(path) in packages


C_COMMENT_SUFFIXES = (".cc", ".h", ".cpp", ".hpp", ".ts", ".tsx")
PY_COMMENT_SUFFIXES = (".py", ".bzl")
PY_COMMENT_NAMES = ("BUILD", "BUILD.bazel")


def _strip_c_comments(text, is_ts=False):
    """Replaces `// ...` and `/* ... */` comments with spaces, preserving
    newlines so 1-based line numbers remain exact."""
    out = []
    i = 0
    n = len(text)
    while i < n:
        ch = text[i]
        nxt = text[i + 1] if i + 1 < n else ""
        # Line comment `// ...`.
        if ch == "/" and nxt == "/":
            while i < n and text[i] != "\n":
                out.append(" ")
                i += 1
            continue
        # Block comment `/* ... */` (multi-line state tracked across newlines).
        if ch == "/" and nxt == "*":
            out.append("  ")
            i += 2
            while i < n:
                if text[i] == "*" and i + 1 < n and text[i + 1] == "/":
                    out.append("  ")
                    i += 2
                    break
                out.append("\n" if text[i] == "\n" else " ")
                i += 1
            continue
        # C++ raw string literal `R"delim(...)delim"` so comment tokens inside
        # raw strings do not start comments.
        if (not is_ts and ch == "R" and nxt == '"' and
                (i == 0 or not (text[i - 1].isalnum() or text[i - 1] == "_"))):
            paren = text.find("(", i + 2)
            if paren != -1 and paren - (i + 2) <= 16:
                delim = text[i + 2:paren]
                if not any(c in delim for c in " \t\r\n\\()"):
                    end_marker = ")" + delim + '"'
                    end = text.find(end_marker, paren + 1)
                    if end != -1:
                        end_idx = end + len(end_marker)
                        out.append(text[i:end_idx])
                        i = end_idx
                        continue
        # C++ numeric literal with optional `'` digit separators (e.g. `1'000`)
        # so a digit separator is never mistaken for a character literal.
        if (not is_ts and ch.isdigit() and
                (i == 0 or not (text[i - 1].isalnum() or text[i - 1] == "_"))):
            out.append(ch)
            i += 1
            while i < n:
                c = text[i]
                c_next = text[i + 1] if i + 1 < n else ""
                if c == "'" and (c_next.isalnum() or c_next == "_"):
                    out.append(c)
                    out.append(c_next)
                    i += 2
                    continue
                if c in ("e", "E", "p", "P") and c_next in ("+", "-"):
                    out.append(c)
                    out.append(c_next)
                    i += 2
                    continue
                if c.isalnum() or c in ("_", "."):
                    out.append(c)
                    i += 1
                    continue
                break
            continue
        # Quoted string / char / template literals: keep verbatim so `/*` or
        # `//` inside a literal does not start a comment.
        if ch in ('"', "'") or (is_ts and ch == "`"):
            quote = ch
            out.append(ch)
            i += 1
            while i < n:
                c = text[i]
                out.append(c)
                i += 1
                if c == "\\" and i < n:
                    out.append(text[i])
                    i += 1
                    continue
                if c == quote:
                    break
                if c == "\n" and quote != "`":
                    break
            continue
        out.append(ch)
        i += 1
    return "".join(out)


def _strip_hash_comments(text, python_strings=True):
    """Replaces `# ...` comments with spaces, preserving newlines."""
    out = []
    i = 0
    n = len(text)
    while i < n:
        ch = text[i]
        if ch == "#" and (
                python_strings or i == 0 or text[i - 1] in (" ", "\t", "\n")):
            while i < n and text[i] != "\n":
                out.append(" ")
                i += 1
            continue
        if ch in ('"', "'"):
            if python_strings and text[i:i + 3] == ch * 3:
                quote = ch * 3
                out.append(quote)
                i += 3
                while i < n:
                    if text[i] == "\\" and i + 1 < n:
                        out.append(text[i:i + 2])
                        i += 2
                        continue
                    if text[i:i + 3] == quote:
                        out.append(quote)
                        i += 3
                        break
                    out.append(text[i])
                    i += 1
                continue
            quote = ch
            out.append(ch)
            i += 1
            while i < n:
                c = text[i]
                out.append(c)
                i += 1
                if c == "\\" and (python_strings or quote == '"') and i < n:
                    out.append(text[i])
                    i += 1
                    continue
                if c == quote or c == "\n":
                    break
            continue
        out.append(ch)
        i += 1
    return "".join(out)


def strip_comments(text, path):
    """Strips comments from `text` according to `path`'s syntax while keeping
    exact line numbers."""
    if path.endswith(C_COMMENT_SUFFIXES):
        return _strip_c_comments(text, is_ts=path.endswith((".ts", ".tsx")))
    name = os.path.basename(path)
    python_strings = (path.endswith(PY_COMMENT_SUFFIXES) or
                      name in PY_COMMENT_NAMES)
    return _strip_hash_comments(text, python_strings=python_strings)


def scan(files, read):
    """{(path, kind): [line numbers]} of every hit in `files`."""
    hits = {}
    packages = node_packages(files, read)
    for path in files:
        patterns = {}
        stripped = None
        if (is_test_source(path) or is_test_helper(path) or
                (path.endswith(SOURCE_SUFFIXES) and
                 in_package(path, packages))):
            patterns.update(TIME_PATTERNS)
            stripped = strip_comments(read(path), path)
            aliases = {a or b for a, b in CLOCK_ALIAS.findall(stripped)}
            if aliases:
                patterns["clock"] = re.compile(
                    TIME_PATTERNS["clock"].pattern + r"|\b(" +
                    "|".join(sorted(aliases)) + r")::now\s*\(")
        if is_studio_test(path):
            patterns.update(TS_TIME_PATTERNS)
        if is_retry_config(path):
            patterns["retry"] = RETRY_PATTERN
        if not patterns:
            continue
        if stripped is None:
            stripped = strip_comments(read(path), path)
        for number, line in enumerate(stripped.splitlines(), 1):
            for kind, pattern in patterns.items():
                if pattern.search(line):
                    hits.setdefault((path, kind), []).append(number)
    return hits


def parse_allowlist(text):
    """({(path, kind): count or None}, [errors]) from the allowlist's text."""
    entries, errors = {}, []
    for number, line in enumerate(text.splitlines(), 1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split(None, 2)
        count = None
        if len(fields) == 3 and fields[2][:1].isdigit():
            first, _, reason = fields[2].partition(" ")
            count = int(first) if first.isdigit() and int(first) > 0 else 0
            fields[2] = reason.strip()
        if (len(fields) < 3 or not fields[2] or count == 0 or
                fields[1] not in ("sleep", "clock", "retry")):
            errors.append(f"{ALLOWLIST}:{number}: want `<path> sleep|clock|"
                          f"retry [<count>] <reason>`: {line}")
            continue
        entries[(fields[0], fields[1])] = count
    return entries, errors


def lint(files, read, allowlist_text, out=sys.stdout):
    """Prints each failure; returns how many there were."""
    hits = scan(files, read)
    allowed, errors = parse_allowlist(allowlist_text)
    for error in errors:
        print(error, file=out)
    failures = len(errors)
    for (path, kind), lines in sorted(hits.items()):
        where = ", ".join(str(n) for n in lines)
        if (path, kind) not in allowed:
            failures += 1
            print(f"{path}:{where}: {kind} (docs/TESTING.md §6.1, §8.1; "
                  f"METHODOLOGY.md §9). Wait on an event, take the clock as "
                  f"a parameter, or list it in {ALLOWLIST} with its reason.",
                  file=out)
            continue
        count = allowed[(path, kind)]
        if count is not None and len(lines) > count:
            failures += 1
            print(f"{path}:{where}: {len(lines)} {kind} hits, {count} allowed "
                  f"by {ALLOWLIST} (docs/TESTING.md §6.1, §8.1). Wait on an "
                  f"event or take the clock as a parameter.", file=out)
        elif count is not None and len(lines) < count:
            failures += 1
            print(f"{ALLOWLIST}: `{path} {kind} {count}` allows more than "
                  f"its {len(lines)} hits; lower it to {len(lines)}", file=out)
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
