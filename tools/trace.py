"""Ties tests to requirements: docs/TESTING.md §3, docs/DESIGN.md §6.

    python3 tools/trace.py --check    # CI lint job, from the workspace root
    python3 tools/trace.py --report   # requirement -> tagged tests

Standard library only; no Bazel. Over the tracked files:

Tags. A test source is a `*_test.{cc,cpp,py,ts,tsx}`, `test_*.py`, `test.cc`
or `*.test.{ts,tsx}` file. A tag line is a line of a test source whose text,
after an optional `//` or `#`, starts with a `REQ-<AREA>-<NN>` ID (a comment
on the line above a test, or a line of a Python docstring); every ID on that
line is a tag. An ID mentioned mid-sentence is prose, not a tag. This is the
reading the existing tags follow (`// REQ-SAFE-09`, `# REQ-A, REQ-B: why`).

`--check` fails on:

- a tag naming an ID docs/REQUIREMENTS.md's catalogue lacks (file:line);
- a SAFE or CONTRACT requirement, or any P0, with no tagged test and no
  verification cited in its row: an open TODO.md item's ID (`- [ ] **ID`),
  a quoted BRINGUP.md heading after `BR`, or a BENCH.md heading after
  `BENCH.md`. Tags in the sources of a quarantined target do not count
  (TESTING.md §8.3);
- a target tagged `quarantine` in a BUILD file without an open TODO.md item
  naming its label (`//pkg:name`), or whose latest date (YYYY-MM-DD) is past
  or more than 14 days out;
- a Bazel package (a directory with a BUILD file, outside `third_party` and
  `*testdata*` directories) that no entry of REQUIREMENTS.md's "Package
  scope" table names or contains, and an entry naming no tracked directory.
  An entry is a backticked path from the repository root;
  `a/{b,c}` expands.

Not enforced: the tier clause ("at its Cheapest tier or below"). A test's tier
is not derivable from the tree today: tiers are defined by the resources a
test touches (TESTING.md §2) and the lanes are not yet Bazel tags (LANE-1),
so `--report` prints each requirement's Cheapest tier beside its tests for a
reviewer to judge, and the check asks only for a tagged test.

Waivers. tools/trace_waivers.txt lists known gaps, one
`<REQ-ID | package path> <YYYY-MM-DD> <TODO-ID> [note]` per line: the date the
gap was written down and the open TODO.md item that will close it. A waived
gap passes. A waiver fails when its subject is no longer a gap (the
requirement has a tagged test or a cited verification, the package is in the
table), names an unknown requirement or a closed TODO item, or is dated in
the future, so the list only shrinks.
"""

import argparse
import datetime
import os
import re
import subprocess
import sys

REQUIREMENTS = "docs/REQUIREMENTS.md"
TODO = "TODO.md"
BENCH = "docs/BENCH.md"
BRINGUP = "docs/BRINGUP.md"
WAIVERS = "tools/trace_waivers.txt"

REQ_ID = re.compile(r"\bREQ-[A-Z]+-\d+\b")
TAG_LINE = re.compile(r"^\s*(?:(?://+|#)\s*)?REQ-[A-Z]+-\d+\b")
TEST_SOURCE = re.compile(r"(^|/)(\w+_test\.(cc|cpp|py|ts|tsx)|test_\w*\.py|"
                         r"test\.cc|[\w.-]+\.test\.(ts|tsx))$")
TODO_ITEM = re.compile(r"^- \[ \] \*\*([A-Z][A-Z0-9]*-\d+)\b")
TODO_ID = re.compile(r"(?<![\w-])([A-Z][A-Z0-9]*-\d+)\b")
DATE = re.compile(r"\b(\d{4}-\d{2}-\d{2})\b")
QUARANTINE_DAYS = 14


class Requirement:

    def __init__(self, req_id, priority, cells):
        self.id = req_id
        self.priority = priority
        self.area = req_id.split("-")[1]
        self.tier = cells[-1]
        self.row = " | ".join(cells)

    def must_verify(self):
        return self.area in ("SAFE", "CONTRACT") or self.priority == "P0"


# A cell boundary: a pipe Markdown does not escape (`\|` is a pipe in a cell).
CELL_BOUNDARY = re.compile(r"(?<!\\)\|")


def split_row(line):
    """A table row's cells; the outer pipes are optional, as in Markdown."""
    cells = [cell.strip() for cell in CELL_BOUNDARY.split(line.strip())]
    if len(cells) > 1 and not cells[0]:
        cells = cells[1:]
    if len(cells) > 1 and not cells[-1]:
        cells = cells[:-1]
    return cells


def parse_requirements(text):
    """({id: Requirement}, [package scope entries])."""
    catalogue, scope, section = {}, [], None
    for line in text.splitlines():
        if line.startswith("## "):
            section = line[3:].strip()
            continue
        if not line.startswith("|"):
            continue
        cells = split_row(line)
        if section == "Catalogue" and REQ_ID.fullmatch(cells[0]):
            catalogue[cells[0]] = Requirement(cells[0], cells[2], cells)
        elif (section == "Package scope" and len(cells) >= 2 and
              cells[0] not in ("Package", "---")):
            # A parenthesis qualifies the entry before it; it names no
            # package of its own.
            package_cell = re.sub(r"\([^)]*\)", "", cells[0])
            for token in re.findall(r"`([^`]+)`", package_cell):
                scope.extend(expand_braces(token))
    return catalogue, scope


def expand_braces(token):
    match = re.search(r"\{([^}]*)\}", token)
    if not match:
        return [token.rstrip("/")]
    head, tail = token[:match.start()], token[match.end():]
    return [path for part in match.group(1).split(",")
            for path in expand_braces(head + part.strip() + tail)]


def open_todo_items(text):
    """{ID: the item's text} for every open TODO.md item."""
    items, current = {}, None
    for line in text.splitlines():
        match = TODO_ITEM.match(line)
        if match:
            current = match.group(1)
            items[current] = line
        elif line.startswith(("- ", "#")):
            current = None
        elif current:
            items[current] += "\n" + line
    return items


def headings(text):
    return [line.lstrip("#").strip().lower() for line in text.splitlines()
            if line.startswith("#")]


def cited_verification(req, todo_items, bench_headings, bringup_headings):
    """What the row cites that verifies it (TODO item, bench section), or
    None."""
    for todo_id in TODO_ID.findall(req.row):
        if todo_id in todo_items:
            return f"TODO.md {todo_id}"
    for name in re.findall(r'\bBR "([^"]+)"', req.row):
        if name.lower() in bringup_headings:
            return f'BRINGUP.md "{name}"'
    for after in re.findall(r"\bBENCH\.md\b([^;|]*)", req.row):
        for heading in bench_headings:
            if heading and heading in after.lower():
                return f'BENCH.md "{heading}"'
    return None


def find_tags(files, read):
    """[(path, line number, id)] for every tag in a test source."""
    tags = []
    for path in files:
        if not TEST_SOURCE.search(path):
            continue
        for number, line in enumerate(read(path).splitlines(), 1):
            if TAG_LINE.match(line):
                tags.extend((path, number, i) for i in REQ_ID.findall(line))
    return tags


def build_rules(text):
    """Yields each top-level rule call's text in a BUILD file."""
    starts = [m.start() for m in re.finditer(r"^[A-Za-z_]\w*\(", text, re.M)]
    for begin, end in zip(starts, starts[1:] + [len(text)]):
        yield text[begin:end]


def quarantined_targets(files, read):
    """[(label, BUILD path, [source paths])] of targets tagged quarantine."""
    targets = []
    for path in files:
        if os.path.basename(path) not in ("BUILD", "BUILD.bazel"):
            continue
        package = os.path.dirname(path)
        for rule in build_rules(read(path)):
            tags = re.search(r"\btags\s*=\s*\[([^\]]*)\]", rule)
            name = re.search(r'\bname\s*=\s*"([^"]+)"', rule)
            if not (tags and name and '"quarantine"' in tags.group(1)):
                continue
            srcs = re.search(r"\bsrcs\s*=\s*\[([^\]]*)\]", rule)
            sources = [os.path.join(package, s) for s in
                       re.findall(r'"([^":]+)"', srcs.group(1))] if srcs else []
            targets.append((f"//{package}:{name.group(1)}", path, sources))
    return targets


def bazel_packages(files):
    packages = set()
    for path in files:
        if os.path.basename(path) not in ("BUILD", "BUILD.bazel"):
            continue
        package = os.path.dirname(path)
        parts = package.split("/")
        if package and parts[0] != "third_party" and not any(
                "testdata" in part for part in parts):
            packages.add(package)
    return packages


def tracked_directories(files):
    dirs = set()
    for path in files:
        parts = path.split("/")[:-1]
        for i in range(1, len(parts) + 1):
            dirs.add("/".join(parts[:i]))
    return dirs


def in_scope(package, scope):
    return any(package == entry or package.startswith(entry + "/")
               for entry in scope)


def parse_waivers(text, today):
    """({subject: (date, todo id, line number)}, [errors])."""
    waivers, errors = {}, []
    for number, line in enumerate(text.splitlines(), 1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        fields = line.split(None, 3)
        where = f"{WAIVERS}:{number}"
        if len(fields) < 3:
            errors.append(f"{where}: want `<REQ-ID|package> <YYYY-MM-DD> "
                          f"<TODO-ID> [note]`: {line}")
            continue
        subject, date, todo_id = fields[:3]
        try:
            when = datetime.date.fromisoformat(date)
        except ValueError:
            errors.append(f"{where}: `{date}` is not a YYYY-MM-DD date")
            continue
        if when > today:
            errors.append(f"{where}: {subject} is dated in the future "
                          f"({date})")
        if subject in waivers:
            errors.append(f"{where}: {subject} is waived twice")
        waivers[subject] = (date, todo_id, number)
    return waivers, errors


def trace(files, read, today):
    """(failures, {id: [(path, line)]}, {id: citation}, catalogue)."""
    failures = []
    catalogue, scope = parse_requirements(read(REQUIREMENTS))
    todo_text = read(TODO)
    todo_items = open_todo_items(todo_text)
    bench_headings = headings(read(BENCH))
    bringup_headings = headings(read(BRINGUP))
    waivers, errors = parse_waivers(read(WAIVERS), today)
    failures.extend(errors)

    quarantined = quarantined_targets(files, read)
    quarantined_sources = {s for _, _, sources in quarantined for s in sources}
    tests = {}
    for path, number, req_id in find_tags(files, read):
        if req_id not in catalogue:
            failures.append(f"{path}:{number}: tag {req_id} is not in "
                            f"{REQUIREMENTS}")
        elif path not in quarantined_sources:
            tests.setdefault(req_id, []).append((path, number))

    for label, build, _ in quarantined:
        entries = [text for text in todo_items.values() if label in text]
        dates = sorted(d for text in entries for d in DATE.findall(text))
        if not dates:
            failures.append(f"{build}: {label} is tagged quarantine without "
                            f"an open, dated {TODO} item naming it "
                            f"(docs/TESTING.md §8.3)")
            continue
        expiry = datetime.date.fromisoformat(dates[-1])
        if expiry < today:
            failures.append(f"{build}: {label}'s quarantine expired on "
                            f"{dates[-1]}: fix or delete it "
                            f"(docs/TESTING.md §8.3)")
        elif (expiry - today).days > QUARANTINE_DAYS:
            failures.append(f"{build}: {label}'s quarantine date {dates[-1]} "
                            f"is more than {QUARANTINE_DAYS} days out")

    citations = {}
    for req in catalogue.values():
        citation = cited_verification(req, todo_items, bench_headings,
                                      bringup_headings)
        if citation:
            citations[req.id] = citation
        gap = req.must_verify() and req.id not in tests and not citation
        waiver = waivers.pop(req.id, None)
        if gap and not waiver:
            failures.append(
                f"{REQUIREMENTS}: {req.id} ({req.priority}, {req.area}) has "
                f"no tagged test and cites no open TODO.md item or bench "
                f"section; tag the test that proves it (docs/TESTING.md §3)")
        elif waiver and not gap:
            why = ("needs no test by this check" if not req.must_verify() else
                   "now has a tagged test or a cited verification")
            failures.append(f"{WAIVERS}:{waiver[2]}: stale waiver: {req.id} "
                            f"{why}; delete the line")

    packages = bazel_packages(files)
    for package in sorted(packages):
        waiver = waivers.pop(package, None)
        if not in_scope(package, scope) and not waiver:
            failures.append(f"{REQUIREMENTS}: package {package} is missing "
                            f"from the package scope table "
                            f"(docs/DESIGN.md §6)")
        elif waiver and in_scope(package, scope):
            failures.append(f"{WAIVERS}:{waiver[2]}: stale waiver: package "
                            f"{package} is in the scope table; delete the "
                            f"line")
    directories = tracked_directories(files)
    for entry in sorted(set(scope)):
        if entry not in directories:
            failures.append(f"{REQUIREMENTS}: package scope entry `{entry}` "
                            f"names no tracked directory")

    for subject, (_, _, number) in sorted(waivers.items()):
        failures.append(f"{WAIVERS}:{number}: {subject} is neither a "
                        f"catalogue requirement nor a Bazel package")
    for subject, (_, todo_id, number) in sorted(
            parse_waivers(read(WAIVERS), today)[0].items()):
        if todo_id not in todo_items:
            failures.append(f"{WAIVERS}:{number}: {subject}'s {todo_id} is "
                            f"not an open {TODO} item")
    return failures, tests, citations, catalogue


def check(files, read, today, out=sys.stdout):
    """Prints each failure; returns how many there were."""
    failures, tests, _, catalogue = trace(files, read, today)
    for failure in failures:
        print(failure, file=out)
    if not failures:
        waived = len(parse_waivers(read(WAIVERS), today)[0])
        print(f"trace: {len(catalogue)} requirements, {len(tests)} with "
              f"tagged tests, {waived} waived; tier not checked (LANE-1)",
              file=out)
    return len(failures)


def report(files, read, today, out=sys.stdout):
    failures, tests, citations, catalogue = trace(files, read, today)
    for req in catalogue.values():
        print(f"{req.id} {req.priority} [{req.tier}]", file=out)
        for path, number in tests.get(req.id, []):
            print(f"    {path}:{number}", file=out)
        if req.id in citations:
            print(f"    cites {citations[req.id]}", file=out)
        if req.id not in tests and req.id not in citations:
            print("    (none)", file=out)
    for failure in failures:
        print(f"FAIL {failure}", file=out)


def tracked_files():
    listing = subprocess.run(["git", "ls-files", "-z"], capture_output=True,
                             check=True).stdout.decode()
    return [path for path in listing.split("\0") if path]


def read_file(path):
    try:
        with open(path, encoding="utf-8", errors="replace") as f:
            return f.read()
    except (FileNotFoundError, IsADirectoryError):
        return ""


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--report", action="store_true")
    args = parser.parse_args()
    files, today = tracked_files(), datetime.date.today()
    if args.report:
        report(files, read_file, today)
        return 0
    return 1 if check(files, read_file, today) else 0


if __name__ == "__main__":
    sys.exit(main())
