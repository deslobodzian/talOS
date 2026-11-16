#!/usr/bin/env python3
"""Fail on runfiles layouts that let a reused sandbox truncate a source file.

Bazel reuses sandbox directories across tests (--reuse_sandbox_directories).
When one test's runfiles hold an empty generated file at a path where another
test's runfiles hold a symlink to a source file, the stale symlink survives
the cleanup and the empty file is written through it, truncating the source.

Run after `bazel build //...` from the workspace root. It scans every
bazel-bin/**/*.runfiles_manifest and fails on
  - a runfiles path that is empty in one manifest and a real file in another;
  - any generated `_main/**/__init__.py` (an empty file in the manifest).
"""

import os
import sys
from collections import defaultdict


def manifests(root):
    for dirpath, dirnames, filenames in os.walk(root, followlinks=False):
        # Runfiles trees mirror the manifests; skip them.
        dirnames[:] = [d for d in dirnames if not d.endswith(".runfiles")]
        for name in filenames:
            if name.endswith(".runfiles_manifest"):
                yield os.path.join(dirpath, name)


def parse(path):
    """Yields (runfiles path, is_empty) per manifest line."""
    with open(path, "rb") as f:
        for raw in f:
            line = raw.decode("utf-8", "surrogateescape").rstrip("\n")
            if not line:
                continue
            if line.startswith(" "):
                # Escaped form (paths with spaces): " <rpath> <target>".
                rpath, _, target = line[1:].partition(" ")
                rpath = rpath.replace("\\s", " ").replace("\\n", "\n").replace("\\b", "\\")
            else:
                rpath, _, target = line.partition(" ")
            yield rpath, target == ""


def main(argv):
    root = argv[1] if len(argv) > 1 else "bazel-bin"
    if not os.path.isdir(root):
        print(f"{root} not found; run `bazel build //...` first", file=sys.stderr)
        return 2
    empty = defaultdict(set)
    real = defaultdict(set)
    count = 0
    for manifest in manifests(root):
        count += 1
        name = os.path.relpath(manifest, root)
        for rpath, is_empty in parse(manifest):
            (empty if is_empty else real)[rpath].add(name)

    errors = []
    for rpath in sorted(empty):
        if rpath in real:
            errors.append(
                f"{rpath}: empty in {len(empty[rpath])} manifest(s) "
                f"(e.g. {min(empty[rpath])}), a file in {len(real[rpath])} "
                f"(e.g. {min(real[rpath])})")
        elif rpath.startswith("_main/") and rpath.endswith("/__init__.py"):
            errors.append(f"{rpath}: generated __init__.py in {min(empty[rpath])}")

    print(f"checked {count} runfiles manifests under {root}")
    for e in errors:
        print(f"error: {e}")
    if errors:
        print("A reused sandbox can truncate a source through such a path. "
              "Keep --incompatible_default_to_explicit_init_py and add real "
              "__init__.py files instead.")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
