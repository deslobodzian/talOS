#!/usr/bin/env python3
"""Run clang-tidy-18 on controller_processor/rio's GradleRIO compile_commands.json.

GradleRIO emits -nostdinc plus host GCC -isystem directories (including GCC's
private intrinsics directory, which breaks Clang on xmmintrin.h, and Debian/
Ubuntu multiarch C++ header directories that can mismatch clang-18's selected
libstdc++). This script writes a sanitized copy of the compilation database to
a temporary directory with -nostdinc and host GCC/system -isystem entries
removed so clang-tidy-18 uses Clang's own builtin headers and compatible C++
standard library, while keeping every WPILib, Phoenix 6, and talOS include flag
intact.

Setup errors, tool crashes/config errors, missing sources, and compiler parse
errors always fail closed (exit 2), even under --observe. Only genuine
clang-tidy lint findings exit 0 under --observe (1 without --observe), per
AGENTS.md verification practice #9.
"""

import argparse
import concurrent.futures
import json
import os
import pathlib
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
import time


DEFAULT_DB = pathlib.Path(
    'controller_processor/rio/build/TargetedCompileCommands/'
    'linuxx86-64release/compile_commands.json'
)
DEFAULT_SRC_DIR = pathlib.Path('controller_processor/rio/src/main/cpp')
DEFAULT_HEADER_FILTER = r'^.*(controller_processor/rio/src/main/include|talOS)/.*'

# Host GCC and system header directories stripped from GradleRIO's command
# lines so Clang resolves its own resource intrinsics (xmmintrin.h) and
# standard headers cleanly across Arch and Ubuntu (GCC 13/14/16).
STRIP_ISYSTEM_MARKERS = (
    '/lib/gcc/',
    '/c++/',
)
STRIP_ISYSTEM_EXACT = frozenset({
    '/usr/include',
    '/usr/local/include',
})

TOOL_ERROR_MARKERS = (
    '[clang-diagnostic-error]',
    'Found compiler error(s).',
    'LLVM ERROR:',
    'Error parsing ',
    'YAML:',
    'Compile command not found',
    'Unknown command line argument',
    'error: unable to handle compilation',
    'Error while trying to load a compilation database',
)
TIDY_DIAGNOSTIC_RE = re.compile(
    r':\d+:\d+: (?:warning|error): .+ \[([^\]\s]+)\]'
)


def _is_host_gcc_isystem(path_str):
    norm = os.path.normpath(path_str)
    if norm in STRIP_ISYSTEM_EXACT:
        return True
    if norm.startswith('/usr/include/') and norm.count('/') == 3:
        # Multiarch system header dir such as /usr/include/x86_64-linux-gnu.
        return True
    return any(marker in norm for marker in STRIP_ISYSTEM_MARKERS)


def sanitize_arguments(args, gcc_install_dir=''):
    """Strip -nostdinc and host GCC -isystem paths from one compile command."""
    out = []
    i = 0
    while i < len(args):
        arg = args[i]
        if arg in ('-nostdinc', '-nostdinc++'):
            i += 1
            continue
        if arg == '-isystem' and i + 1 < len(args):
            if _is_host_gcc_isystem(args[i + 1]):
                i += 2
                continue
            out.extend((arg, args[i + 1]))
            i += 2
            continue
        if arg.startswith('-isystem') and len(arg) > len('-isystem'):
            target = arg[len('-isystem'):]
            if _is_host_gcc_isystem(target):
                i += 1
                continue
        out.append(arg)
        i += 1
    if gcc_install_dir:
        out.append(f'--gcc-install-dir={gcc_install_dir}')
    return out


def _resolve_entry_file(entry):
    raw_file = pathlib.Path(entry['file'])
    if not raw_file.is_absolute() and 'directory' in entry:
        raw_file = pathlib.Path(entry['directory']) / raw_file
    return raw_file.resolve()


def sanitize_database(entries, gcc_install_dir=''):
    """Return a sanitized copy of a compile_commands.json entry list."""
    cleaned = []
    for entry in entries:
        if not isinstance(entry, dict) or 'file' not in entry:
            continue
        copy = dict(entry)
        if 'arguments' in copy:
            raw_args = list(copy['arguments'])
        elif 'command' in copy:
            raw_args = shlex.split(copy['command'])
            copy.pop('command', None)
        else:
            continue
        # Resolve symlinks (e.g. macOS /tmp -> /private/tmp) so entry['file']
        # matches the resolved source path passed on clang-tidy's command line.
        copy['file'] = str(_resolve_entry_file(copy))
        copy['arguments'] = sanitize_arguments(
            raw_args, gcc_install_dir=gcc_install_dir
        )
        cleaned.append(copy)
    return cleaned


def collect_sources(src_dir, entries):
    """Return sorted absolute source paths and any sources missing from the DB."""
    sources = sorted(p.resolve() for p in src_dir.rglob('*.cpp'))
    db_files = {_resolve_entry_file(entry) for entry in entries}
    missing = [p for p in sources if p not in db_files]
    return sources, missing


def is_lint_finding_only(returncode, output):
    """True only when clang-tidy exited 1 solely due to non-compiler lint hits.

    Any signal kill (returncode < 0), setup/CLI error (returncode > 1),
    compiler parse error, YAML config error, or output without a tidy
    diagnostic returns False so the caller fails closed (exit 2) even under
    --observe.
    """
    if returncode != 1:
        return False
    if any(marker in output for marker in TOOL_ERROR_MARKERS):
        return False
    checks = TIDY_DIAGNOSTIC_RE.findall(output)
    return bool(checks) and all(
        'clang-diagnostic-error' not in check for check in checks
    )


def run_one(tidy_bin, db_dir, source, header_filter=DEFAULT_HEADER_FILTER):
    cmd = [tidy_bin, '-p', str(db_dir)]
    if header_filter:
        cmd.append(f'--header-filter={header_filter}')
    cmd.append(str(source))
    proc = subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    return source, proc.returncode, proc.stdout


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--db',
        type=pathlib.Path,
        default=DEFAULT_DB,
        help='Path to GradleRIO compile_commands.json.',
    )
    parser.add_argument(
        '--src-dir',
        type=pathlib.Path,
        default=DEFAULT_SRC_DIR,
        help='Directory containing RoboRIO C++ sources to analyze.',
    )
    parser.add_argument(
        '--header-filter',
        default=DEFAULT_HEADER_FILTER,
        help='Regex passed to clang-tidy --header-filter.',
    )
    parser.add_argument(
        '--tidy-bin',
        default='clang-tidy-18',
        help='clang-tidy executable name or path.',
    )
    parser.add_argument(
        '--gcc-install-dir',
        default='',
        help='Optional --gcc-install-dir passed to clang-tidy.',
    )
    parser.add_argument(
        '--observe',
        action='store_true',
        help='Report lint findings without failing (setup/tool/parse errors still fail).',
    )
    parser.add_argument(
        '--jobs',
        type=int,
        default=max(1, min(os.cpu_count() or 1, 8)),
        help='Parallel clang-tidy worker count.',
    )
    args = parser.parse_args(argv)

    if not args.db.is_file():
        print(
            f'rio-clang-tidy: missing compilation database: {args.db} '
            '(run ./gradlew setupCompileCommands in controller_processor/rio)',
            file=sys.stderr,
        )
        return 2

    if shutil.which(args.tidy_bin) is None:
        print(
            f'rio-clang-tidy: executable not found: {args.tidy_bin}',
            file=sys.stderr,
        )
        return 2

    try:
        raw_entries = json.loads(args.db.read_text(encoding='utf-8'))
    except (OSError, json.JSONDecodeError) as exc:
        print(
            f'rio-clang-tidy: cannot read {args.db}: {exc}',
            file=sys.stderr,
        )
        return 2

    if not isinstance(raw_entries, list) or not raw_entries:
        print(
            f'rio-clang-tidy: empty compilation database: {args.db}',
            file=sys.stderr,
        )
        return 2

    cleaned = sanitize_database(raw_entries, gcc_install_dir=args.gcc_install_dir)
    if not cleaned:
        print(
            f'rio-clang-tidy: no usable entries in {args.db}',
            file=sys.stderr,
        )
        return 2

    sources, missing = collect_sources(args.src_dir, cleaned)
    if not sources:
        print(
            f'rio-clang-tidy: no *.cpp files under {args.src_dir}',
            file=sys.stderr,
        )
        return 2
    if missing:
        names = ', '.join(str(p.relative_to(args.src_dir.resolve())) for p in missing)
        print(
            f'rio-clang-tidy: sources missing from {args.db}: {names}',
            file=sys.stderr,
        )
        return 2

    t0 = time.monotonic()
    findings = 0
    tool_errors = 0
    repo_root = pathlib.Path.cwd().resolve()

    with tempfile.TemporaryDirectory(prefix='rio-tidy-') as tmp:
        db_dir = pathlib.Path(tmp)
        (db_dir / 'compile_commands.json').write_text(
            json.dumps(cleaned), encoding='utf-8'
        )
        with concurrent.futures.ThreadPoolExecutor(
            max_workers=max(1, args.jobs)
        ) as pool:
            futures = [
                pool.submit(
                    run_one,
                    args.tidy_bin,
                    db_dir,
                    src,
                    args.header_filter,
                )
                for src in sources
            ]
            for fut in futures:
                src, rc, out = fut.result()
                try:
                    label = str(src.relative_to(repo_root))
                except ValueError:
                    label = str(src)
                if rc != 0:
                    if is_lint_finding_only(rc, out):
                        findings += 1
                    else:
                        tool_errors += 1
                    print(f'::group::{label}')
                    if out:
                        print(out.rstrip())
                    else:
                        print(f'clang-tidy exited with code {rc} (no output)')
                    print('::endgroup::')

    elapsed = time.monotonic() - t0
    mode = ' (observe mode)' if args.observe else ''
    print(
        f'rio-clang-tidy: {len(sources)} files, {findings} with findings, '
        f'{tool_errors} with tool/parse errors, {elapsed:.1f}s{mode}'
    )
    if tool_errors > 0:
        return 2
    if findings > 0 and not args.observe:
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
