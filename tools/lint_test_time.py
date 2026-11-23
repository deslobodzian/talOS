"""Lints tests and node code for wall-clock time, randomness, blocking I/O,
locks, and test retries.

    python3 tools/lint_test_time.py   # from the workspace root

RULES.md 3, docs/TESTING.md §6.1 and §8.1, docs/METHODOLOGY.md §9 and §10
(REQ-RT-06, REQ-RT-17). Over the tracked files it finds:

- `sleep`: a sleep call (`sleep_for`, `sleep_until`, `usleep`, `nanosleep`,
  `sleep(`, `time.sleep(`) in a test source or a node package's source;
- `clock`: a wall-clock read (`steady_clock::now`, `system_clock::now`,
  `high_resolution_clock::now`, the same through an alias the file declares,
  `using Clock = std::chrono::steady_clock;` then `Clock::now()`,
  `time.time(`, `time.monotonic(`) in test sources and node code, and in node
  code additionally `clock_gettime(`, `gettimeofday(`, `Poller::now(` (and
  aliases of `Poller`), `std::time(`, `utc_clock`/`tai_clock`/`gps_clock`/
  `file_clock::now(`, and Python `time.monotonic_ns(`, `time.time_ns(`,
  `time.perf_counter[_ns](`, `time.process_time[_ns](`, `datetime.now(`,
  `datetime.utcnow(`, `datetime.today(`, `import time`, `from time import`;
- `random`: a randomness API in node code (`std::rand`/`rand`/`srand`,
  `drand48`/`lrand48`/`mrand48`, `arc4random*`, C++ `<random>` engines
  `mt19937[_64]`, `default_random_engine`, `minstd_rand[0]`, `ranlux*`,
  `knuth_b`, and in Python `random.*`, `import random`, `from random import`,
  `os.urandom(`, `secrets.*`);
- `io`: blocking I/O, logging, or `print` in node code (C++ `std::cout`/
  `cerr`/`clog`/`cin`, `printf`/`fprintf`/`puts`/`fopen`,
  `std::ifstream`/`ofstream`/`fstream`, bare POSIX `socket`/`bind`/`listen`/
  `accept`/`connect`/`sendto`/`recvfrom`; Python `print(`, bare `open(`,
  imports of `socket`/`urllib`/`requests`/`subprocess`/`logging`);
- `lock`: mutexes or condition variables in node code (C++ `std::mutex`/
  `shared_mutex`/`recursive_mutex`/`lock_guard`/`unique_lock`/`scoped_lock`/
  `condition_variable[_any]`, `pthread_mutex_lock`; Python
  `threading.Lock`/`RLock`/`Semaphore`/`BoundedSemaphore`/`Condition`);
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
`Date.now(` or `performance.now(` call. Comments and non-executing string
literals in C++/Python sources are skipped.

An integration, e2e, soak or perf test may use the real clock (§2), and
non-`update()` helpers (such as construction-only setup or a CLI beside a
node) may use I/O or locks; each is listed in
tools/lint_test_time_allowlist.txt, one `<path> <kind> [<count>] <reason>`
per line, where kind is sleep, clock, random, io, lock or retry and count, a
positive number, is how many lines of that file hit that kind (a reason
therefore does not start with a digit). A hit not listed fails, and so does
a listed (path, kind) with no hit left; with a count, so do more hits than
it, or fewer (lower the count), so the list only shrinks. Exits 1 on any.
"""

import bisect
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
                        r"\s*\(", re.MULTILINE),
    "clock": re.compile(r"\b(steady_clock|system_clock|high_resolution_clock)"
                        r"\s*::\s*now\s*\(|\btime\s*\.\s*(time|monotonic)\s*\(",
                        re.MULTILINE),
}
# Node code (non-test sources in node packages) is additionally barred from
# POSIX/Poller/Python wall-clock APIs, randomness (REQ-RT-17; METHODOLOGY §9),
# and blocking I/O, locks, logging, or print (RULES.md 3; METHODOLOGY §10;
# REQ-RT-06, REQ-RT-17).
NODE_TIME_PATTERNS = {
    "sleep": TIME_PATTERNS["sleep"],
    "clock": re.compile(
        TIME_PATTERNS["clock"].pattern +
        r"|\b(?:utc_clock|tai_clock|gps_clock|file_clock)\s*::\s*now\s*\(" +
        r"|\b(?:clock_gettime|gettimeofday)\s*\(" +
        r"|\bPoller\s*::\s*now\s*\(" +
        r"|\bstd\s*::\s*time\s*\(" +
        r"|\btime\s*\.\s*(?:time_ns|monotonic_ns|perf_counter(?:_ns)?|"
        r"process_time(?:_ns)?)\s*\(" +
        r"|\bdatetime\s*\.\s*(?:now|utcnow|today)\s*\(",
        re.MULTILINE,
    ),
    "random": re.compile(
        r"(?<![.>])\b(?:std\s*::\s*)?(?:s?rand|drand48|lrand48|mrand48|"
        r"arc4random(?:_uniform|_buf)?)\s*\(" +
        r"|\b(?:std\s*::\s*)?(?:mt19937(?:_64)?|default_random_engine|"
        r"minstd_rand0?|ranlux24_base|ranlux48_base|ranlux24|ranlux48|"
        r"knuth_b)\b",
        re.MULTILINE,
    ),
    "io": re.compile(
        r"\bstd\s*::\s*(?:cout|cerr|clog|cin|ifstream|ofstream|fstream)\b" +
        r"|(?:\bstd\s*::\s*|(?<![\w.>])::\s*|(?<![.>])(?<!::)\b)"
        r"(?:printf|fprintf|puts|fopen)\s*\(" +
        r"|(?:(?<![\w.>])::\s*|(?<![.>])(?<!::)\b)"
        r"(?:socket|bind|listen|accept|connect|sendto|recvfrom)\s*\(",
        re.MULTILINE,
    ),
    "lock": re.compile(
        r"\bstd\s*::\s*(?:mutex|shared_mutex|recursive_mutex|"
        r"lock_guard|unique_lock|scoped_lock|condition_variable(?:_any)?)\b" +
        r"|\bpthread_mutex_lock\b",
        re.MULTILINE,
    ),
}
# Python-only node patterns (scoped to `.py` files so C++ struct members such
# as `random.value` never false-positive).
PY_NODE_CLOCK_EXTRA = re.compile(
    r"(?:^|;)[ \t]*from\s+time\s+import\b"
    r"|(?:^|;)[ \t]*import\s+(?:[\w.]+(?:\s+as\s+\w+)?\s*,\s*)*time\b",
    re.MULTILINE,
)
PY_NODE_RANDOM_PATTERN = re.compile(
    r"\brandom\s*\.\s*\w+"
    r"|(?:^|;)[ \t]*from\s+random\s+import\b"
    r"|(?:^|;)[ \t]*import\s+(?:[\w.]+(?:\s+as\s+\w+)?\s*,\s*)*random\b"
    r"|\bos\s*\.\s*urandom\s*\("
    r"|(?:^|;)[ \t]*from\s+os\s+import(?:\s*\([^)]*?\burandom\b|[^\n;#]*\burandom\b)"
    r"|\bsecrets\s*\.\s*\w+"
    r"|(?:^|;)[ \t]*from\s+secrets\s+import\b"
    r"|(?:^|;)[ \t]*import\s+(?:[\w.]+(?:\s+as\s+\w+)?\s*,\s*)*secrets\b",
    re.MULTILINE,
)
PY_NODE_IO_PATTERN = re.compile(
    r"(?<!\.)(?<!def )\b(?:print|open)\s*\("
    r"|(?:^|;)[ \t]*from\s+(?:socket|urllib|requests|subprocess|logging)"
    r"(?:\.[\w.]+)?\s+import\b"
    r"|(?:^|;)[ \t]*import\s+(?:[\w.]+(?:\s+as\s+\w+)?\s*,\s*)*"
    r"(?:socket|urllib|requests|subprocess|logging)\b",
    re.MULTILINE,
)
PY_NODE_LOCK_PATTERN = re.compile(
    r"\bthreading\s*\.\s*(?:Lock|RLock|Semaphore|BoundedSemaphore|Condition)\b"
    r"|(?:^|;)[ \t]*from\s+threading\s+import(?:\s*\([^)]*?\b"
    r"(?:Lock|RLock|Semaphore|BoundedSemaphore|Condition)\b"
    r"|[^\n;#]*\b(?:Lock|RLock|Semaphore|BoundedSemaphore|Condition)\b)",
    re.MULTILINE,
)
# `using Clock = std::chrono::steady_clock;` or a typedef of one: the alias's
# `::now(` is a clock read too.
CLOCK_ALIAS = re.compile(r"\busing\s+(\w+)\s*=\s*(?:std\s*::\s*)?"
                         r"(?:chrono\s*::\s*)?"
                         r"(?:steady_clock|system_clock|high_resolution_clock)"
                         r"\s*;|\btypedef\s+(?:std\s*::\s*)?"
                         r"(?:chrono\s*::\s*)?"
                         r"(?:steady_clock|system_clock|high_resolution_clock)"
                         r"\s+(\w+)\s*;")
# `using P = talos::event::Poller;` (or C++20 clocks) in node code: `P::now()`
# reads the clock.
NODE_CLOCK_ALIAS = re.compile(
    r"\busing\s+(\w+)\s*=\s*(?:(?:talos|event)\s*::\s*)*Poller\s*;"
    r"|\btypedef\s+(?:(?:talos|event)\s*::\s*)*Poller\s+(\w+)\s*;"
    r"|\busing\s+(\w+)\s*=\s*(?:std\s*::\s*)?(?:chrono\s*::\s*)?"
    r"(?:utc_clock|tai_clock|gps_clock|file_clock)\s*;"
    r"|\btypedef\s+(?:std\s*::\s*)?(?:chrono\s*::\s*)?"
    r"(?:utc_clock|tai_clock|gps_clock|file_clock)\s+(\w+)\s*;"
)
TS_TIME_PATTERNS = {
    "sleep": re.compile(r"\bsetTimeout\s*\(", re.MULTILINE),
    "clock": re.compile(r"\b(Date|performance)\s*\.\s*now\s*\(", re.MULTILINE),
}
RETRY_PATTERN = re.compile(r"\bflaky\s*=\s*True\b|--flaky_test_attempts",
                           re.MULTILINE)


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


def in_package(path, packages, build_dirs=None):
    """True when `path` belongs to one of `packages` (directly in it, or in a
    subdirectory below it with no intervening BUILD/BUILD.bazel boundary)."""
    directory = os.path.dirname(path)
    while True:
        if directory in packages:
            return True
        if build_dirs is not None and directory in build_dirs:
            return False
        parent = os.path.dirname(directory)
        if parent == directory:
            return False
        directory = parent


C_COMMENT_SUFFIXES = (".cc", ".h", ".cpp", ".hpp", ".ts", ".tsx")
PY_COMMENT_SUFFIXES = (".py", ".bzl")
PY_COMMENT_NAMES = ("BUILD", "BUILD.bazel")


def _strip_c_comments(text, is_ts=False, strip_strings=False):
    """Replaces `// ...` and `/* ... */` comments (and, when `strip_strings` is
    set, non-template string/char literal interiors) with spaces, preserving
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
                        if strip_strings:
                            out.append('"')
                            for c in text[i + 1:end_idx - 1]:
                                out.append("\n" if c == "\n" else " ")
                            out.append('"')
                        else:
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
        # Quoted string / char / template literals: keep delimiters so `/*` or
        # `//` inside a literal does not start a comment; blank out interior
        # characters when `strip_strings` is set (except TS template literals,
        # which can evaluate `${...}`).
        if ch in ('"', "'") or (is_ts and ch == "`"):
            quote = ch
            blank = strip_strings and quote != "`"
            out.append(ch)
            i += 1
            while i < n:
                c = text[i]
                i += 1
                if c == "\\" and i < n:
                    escaped = text[i]
                    i += 1
                    if blank:
                        out.append("\n" if escaped == "\n" else "  ")
                    else:
                        out.append(c)
                        out.append(escaped)
                    continue
                if c == quote:
                    out.append(c)
                    break
                if c == "\n" and quote != "`":
                    out.append(c)
                    break
                out.append(("\n" if c == "\n" else " ") if blank else c)
            continue
        # Backslash line continuation outside strings: replace `\` with space
        # while keeping `\n` intact so multiline invocations match `\s*` and
        # 1-based line numbers stay exact.
        if ch == "\\":
            j = i + 1
            while j < n and text[j] in (" ", "\t", "\r"):
                j += 1
            if j < n and text[j] == "\n":
                out.append(" ")
                i += 1
                continue
        out.append(ch)
        i += 1
    return "".join(out)


def _strip_hash_comments(text, python_strings=True, strip_strings=False):
    """Replaces `# ...` comments (and, when `strip_strings` is set, non-f-string
    Python string literal interiors) with spaces, preserving newlines."""
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
            # Python f-strings (`f"..."`, `rf"..."`, `fr"..."`, case-insensitive)
            # can execute arbitrary code inside `{...}`, so keep them verbatim.
            blank = False
            if python_strings and strip_strings:
                p = i
                while p > 0 and text[p - 1] in "rRuUbBfF" and i - p < 2:
                    p -= 1
                is_fstring = (
                    p < i and
                    (p == 0 or not (text[p - 1].isalnum() or text[p - 1] == "_"))
                    and text[p:i].lower() in ("f", "fr", "rf")
                )
                blank = not is_fstring
            if python_strings and text[i:i + 3] == ch * 3:
                quote = ch * 3
                out.append(quote)
                i += 3
                while i < n:
                    if text[i] == "\\" and i + 1 < n:
                        escaped = text[i + 1]
                        if blank:
                            out.append("\n" if escaped == "\n" else "  ")
                        else:
                            out.append(text[i:i + 2])
                        i += 2
                        continue
                    if text[i:i + 3] == quote:
                        out.append(quote)
                        i += 3
                        break
                    c = text[i]
                    out.append(("\n" if c == "\n" else " ") if blank else c)
                    i += 1
                continue
            quote = ch
            out.append(ch)
            i += 1
            while i < n:
                c = text[i]
                i += 1
                if c == "\\" and (python_strings or quote == '"') and i < n:
                    escaped = text[i]
                    i += 1
                    if blank:
                        out.append("\n" if escaped == "\n" else "  ")
                    else:
                        out.append(c)
                        out.append(escaped)
                    continue
                if c == quote or c == "\n":
                    out.append(c)
                    break
                out.append(" " if blank else c)
            continue
        # Backslash line continuation outside strings: replace `\` with space
        # while keeping `\n` intact so multiline invocations match `\s*` and
        # 1-based line numbers stay exact.
        if ch == "\\":
            j = i + 1
            while j < n and text[j] in (" ", "\t", "\r"):
                j += 1
            if j < n and text[j] == "\n":
                out.append(" ")
                i += 1
                continue
        out.append(ch)
        i += 1
    return "".join(out)


def strip_comments(text, path, strip_strings=False):
    """Strips comments (and optionally non-executing string literals) from
    `text` according to `path`'s syntax while keeping exact line numbers."""
    if path.endswith(C_COMMENT_SUFFIXES):
        return _strip_c_comments(text, is_ts=path.endswith((".ts", ".tsx")),
                                 strip_strings=strip_strings)
    name = os.path.basename(path)
    python_strings = (path.endswith(PY_COMMENT_SUFFIXES) or
                      name in PY_COMMENT_NAMES)
    return _strip_hash_comments(text, python_strings=python_strings,
                                strip_strings=strip_strings)


def _line_starts(text):
    """0-based starting character offsets for each 1-based line in `text`."""
    starts = [0]
    idx = text.find("\n")
    while idx != -1:
        starts.append(idx + 1)
        idx = text.find("\n", idx + 1)
    return starts


def scan(files, read):
    """{(path, kind): [line numbers]} of every hit in `files`."""
    hits = {}
    packages = node_packages(files, read)
    build_dirs = {
        os.path.dirname(path)
        for path in files
        if os.path.basename(path) in ("BUILD", "BUILD.bazel")
    }
    for path in files:
        patterns = {}
        stripped = None
        is_test = is_test_source(path) or is_test_helper(path)
        is_node = (not is_test and path.endswith(SOURCE_SUFFIXES) and
                   in_package(path, packages, build_dirs))
        if is_test:
            patterns.update(TIME_PATTERNS)
        elif is_node:
            patterns.update(NODE_TIME_PATTERNS)
            if path.endswith(".py"):
                patterns["clock"] = re.compile(
                    patterns["clock"].pattern + "|" +
                    PY_NODE_CLOCK_EXTRA.pattern, re.MULTILINE)
                patterns["random"] = PY_NODE_RANDOM_PATTERN
                patterns["io"] = PY_NODE_IO_PATTERN
                patterns["lock"] = PY_NODE_LOCK_PATTERN
        if is_test or is_node:
            stripped = strip_comments(read(path), path, strip_strings=True)
            aliases = {a or b for a, b in CLOCK_ALIAS.findall(stripped)}
            if is_node:
                aliases |= {
                    name
                    for groups in NODE_CLOCK_ALIAS.findall(stripped)
                    for name in groups if name
                }
            if aliases:
                patterns["clock"] = re.compile(
                    patterns["clock"].pattern + r"|\b(" +
                    "|".join(sorted(aliases)) + r")\s*::\s*now\s*\(",
                    re.MULTILINE)
        if is_studio_test(path):
            patterns.update(TS_TIME_PATTERNS)
        if is_retry_config(path):
            patterns["retry"] = RETRY_PATTERN
        if not patterns:
            continue
        if stripped is None:
            stripped = strip_comments(read(path), path)
        starts = _line_starts(stripped)
        for kind, pattern in patterns.items():
            lines = []
            last_line = 0
            for match in pattern.finditer(stripped):
                number = bisect.bisect_right(starts, match.start())
                if number != last_line:
                    lines.append(number)
                    last_line = number
            if lines:
                hits[(path, kind)] = lines
    return hits


ALLOWED_KINDS = ("sleep", "clock", "random", "io", "lock", "retry")


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
                fields[1] not in ALLOWED_KINDS):
            errors.append(f"{ALLOWLIST}:{number}: want `<path> sleep|clock|"
                          f"random|io|lock|retry [<count>] <reason>`: {line}")
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
            if kind == "random":
                print(f"{path}:{where}: {kind} (METHODOLOGY.md §9; REQ-RT-17)."
                      f" Node code must be deterministic; remove randomness "
                      f"or list it in {ALLOWLIST} with its reason.",
                      file=out)
            elif kind in ("io", "lock"):
                print(f"{path}:{where}: {kind} (RULES.md 3; METHODOLOGY.md §10;"
                      f" REQ-RT-06, REQ-RT-17). Node code must not do blocking"
                      f" I/O, locks, logging, or print; move it out of node"
                      f" code or list non-update() uses in {ALLOWLIST} with a"
                      f" count and reason.", file=out)
            else:
                print(f"{path}:{where}: {kind} (docs/TESTING.md §6.1, §8.1; "
                      f"METHODOLOGY.md §9). Wait on an event, take the clock "
                      f"as a parameter, or list it in {ALLOWLIST} with its "
                      f"reason.", file=out)
            continue
        count = allowed[(path, kind)]
        if count is not None and len(lines) > count:
            failures += 1
            if kind in ("io", "lock"):
                print(f"{path}:{where}: {len(lines)} {kind} hits, {count} "
                      f"allowed by {ALLOWLIST} (RULES.md 3; METHODOLOGY.md "
                      f"§10).", file=out)
            elif kind == "random":
                print(f"{path}:{where}: {len(lines)} {kind} hits, {count} "
                      f"allowed by {ALLOWLIST} (METHODOLOGY.md §9; REQ-RT-17).",
                      file=out)
            else:
                print(f"{path}:{where}: {len(lines)} {kind} hits, {count} "
                      f"allowed by {ALLOWLIST} (docs/TESTING.md §6.1, §8.1). "
                      f"Wait on an event or take the clock as a parameter.",
                      file=out)
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
