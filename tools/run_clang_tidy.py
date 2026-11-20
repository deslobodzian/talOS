#!/usr/bin/env python3
"""Run clang-tidy-18 on C++ sources and headers changed since a base ref.

Used by both .github/workflows/continuous-integration.yaml and tools/tidy.sh.
Analyzes TUs in compile_commands.json: each changed source (tests included),
plus up to TUS_PER_HEADER includers of each changed header (direct or
transitive; production includers for a production header). Exits 1 if any
file has findings or a changed in-graph source is missing from the database.
"""

import concurrent.futures
import json
import os
import pathlib
import re
import subprocess
import sys
import time

SOURCES = ('.c', '.cc', '.cpp')
HEADERS = ('.h', '.hpp')
# Sources the Linux build graph never compiles, so they have no compilation
# database entry: the GradleRIO program and its tests (rio-adapter job) and
# sources selected only for other platforms. Paths in .bazelignore are outside
# the graph too and are read from there.
OFF_GRAPH = (
    'controller_processor/rio/',
    'talOS/events/os/kqueue_poller.cc',
    'talOS/vision/capture/unsupported_source.cc',
)
# A changed header is analyzed through at most this many translation units
# that include it, nearest includers first.
TUS_PER_HEADER = 4
# Test code reads an optional after gtest's ASSERT_TRUE(opt), which this check
# cannot see through; every other check applies to tests.
TEST_CHECKS = '-bugprone-unchecked-optional-access'


def git(*args):
    # NUL-delimited git output handles paths containing whitespace.
    return [
        p for p in subprocess.check_output(['git', *args]).decode().split('\0') if p
    ]


def ours(rel):
    return not {'external', 'third_party'}.intersection(
        pathlib.PurePosixPath(rel).parts
    )


def production(rel):
    path = pathlib.PurePosixPath(rel)
    return ours(rel) and not (
        {'test', 'tests', 'testing', 'testdata'}.intersection(path.parts)
        or path.stem in {'test', 'testing'}
        or path.stem.endswith(('_test', '_testing'))
        or 'fuzz' in path.stem
    )


def main():
    if len(sys.argv) != 2 or sys.argv[1] in ('-h', '--help'):
        print('usage: run_clang_tidy.py <base-ref>', file=sys.stderr)
        return 0 if len(sys.argv) == 2 else 2
    base = sys.argv[1]

    with open('.bazelignore') as f:
        ignored = tuple(
            line.strip().rstrip('/') + '/'
            for line in f
            if line.strip() and not line.startswith('#')
        )
    with open('compile_commands.json') as f:
        db = json.load(f)
    root = os.getcwd()
    tus = set()
    for e in db:
        if e['file'].endswith(SOURCES):
            rel = os.path.relpath(
                os.path.realpath(os.path.join(e['directory'], e['file'])), root
            )
            if ours(rel):
                tus.add(rel)

    changed = [
        rel
        for rel in git(
            'diff',
            '--name-only',
            '--diff-filter=d',
            '-z',
            base,
            'HEAD',
            '--',
            *('*' + ext for ext in SOURCES + HEADERS),
        )
        if ours(rel) and not rel.startswith(ignored)
    ]
    targets, missing = [], []
    for rel in changed:
        if not rel.endswith(SOURCES):
            continue
        if rel in tus:
            targets.append(rel)
        elif not rel.startswith(OFF_GRAPH):
            missing.append(rel)

    headers = [rel for rel in changed if rel.endswith(HEADERS)]
    if headers:
        # Reverse include graph over tracked files. A quoted include next to
        # its includer resolves there; otherwise to every tracked header whose
        # path ends with the spelling, which over-approximates when two headers
        # share a suffix: an extra includer is analyzed, none missed.
        tracked = git('ls-files', '-z', *('*' + ext for ext in SOURCES + HEADERS))
        tracked_set = set(tracked)
        by_name = {}
        for rel in tracked:
            if rel.endswith(HEADERS):
                by_name.setdefault(os.path.basename(rel), []).append(rel)
        include = re.compile(rb'^\s*#\s*include\s*(["<])([^">]+)[">]', re.M)
        includers = {}
        for rel in tracked:
            with open(rel, 'rb') as f:
                for quote, spelled in include.findall(f.read()):
                    spelled = spelled.decode(errors='replace')
                    local = os.path.normpath(
                        os.path.join(os.path.dirname(rel), spelled)
                    )
                    if quote == b'"' and local in tracked_set:
                        resolved = [local]
                    else:
                        resolved = [
                            h
                            for h in by_name.get(os.path.basename(spelled), ())
                            if h == spelled or h.endswith('/' + spelled)
                        ]
                    for header in resolved:
                        includers.setdefault(header, set()).add(rel)
        for header in headers:
            found, seen, frontier = [], {header}, [header]
            while frontier and len(found) < TUS_PER_HEADER:
                level = sorted(
                    {i for h in frontier for i in includers.get(h, ())} - seen,
                    key=lambda p: (
                        os.path.dirname(p) != os.path.dirname(header),
                        p,
                    ),
                )
                seen.update(level)
                found += [
                    p
                    for p in level
                    if p in tus and (production(p) or not production(header))
                ][: TUS_PER_HEADER - len(found)]
                frontier = [p for p in level if p.endswith(HEADERS)]
            if found:
                print(f'{header}: via {", ".join(found)}')
                targets += found
            elif production(header):
                print(
                    f'{header}: no production translation unit includes it; not analyzed'
                )
            else:
                print(f'{header}: no translation unit includes it; not analyzed')
    targets = list(dict.fromkeys(targets))

    for rel in missing:
        print(
            f'::error file={rel}::changed source has no compilation '
            'database entry; add it to a Bazel target or to OFF_GRAPH in '
            'tools/run_clang_tidy.py'
        )
    if not targets:
        print('No changed C++ to analyze.')
        return 1 if missing else 0

    # One clang-tidy per core: a single process over a large PR's
    # translation units ran for most of an hour.
    def tidy(path):
        checks = [] if production(path) else [f'--checks={TEST_CHECKS}']
        return subprocess.run(
            ['clang-tidy-18', '-p', '.', '--quiet', *checks, path],
            capture_output=True,
            text=True,
        )

    failed = 0
    start = time.monotonic()
    with concurrent.futures.ThreadPoolExecutor(os.cpu_count()) as pool:
        for path, r in zip(targets, pool.map(tidy, targets)):
            if r.returncode != 0:
                failed += 1
                print(f'::group::{path}')
                print(r.stdout + r.stderr, end='')
                print('::endgroup::')
    print(
        f'clang-tidy: {len(targets)} files, {failed} with findings, '
        f'{time.monotonic() - start:.1f}s'
    )
    return 1 if failed or missing else 0


if __name__ == '__main__':
    sys.exit(main())
