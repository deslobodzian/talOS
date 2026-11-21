"""Deterministic SHA-256 content stamp for Studio's web inputs under talOS.

    python3 studio/bridge/webroot_stamp.py --root <talos_root>
    python3 studio/bridge/webroot_stamp.py --root <talos_root> --list
    python3 studio/bridge/webroot_stamp.py --root <talos_root> --hash-only --out <file>

Hashes the Studio web frontend sources and telemetry FlatBuffers schema under a
talOS root in sorted POSIX-path order so `//studio:web_dist` (which writes
`bazel-bin/studio/dist/STUDIO_STAMP.txt`) and `studio_web`
(`studio/bridge/webroot.bzl`, which checks that stamp against the pinned
`@talos` checkout) compute the exact same digest on clean trees, dirty trees,
and `git archive` exports alike.

`studio/src/generated/**` is excluded because `//studio:web_dist` generates
those files from `talOS/telemetry/telemetry.fbs` (which is hashed directly)
rather than reading the checked-in copies.
"""

import argparse
import hashlib
import os
from pathlib import Path
import re
import subprocess
import sys

FIXED_INPUTS = (
    "studio/drive.html",
    "studio/index.html",
    "studio/package.json",
    "studio/pnpm-lock.yaml",
    "studio/tsconfig.json",
    "studio/vite.config.mjs",
    "talOS/telemetry/telemetry.fbs",
)
SRC_ROOT = "studio/src"
EXCLUDED_SRC_SUBDIRS = frozenset({"generated"})
STAMP_RE = re.compile(r"^[0-9a-f]{64}$")


def collect_inputs(root):
    """Return sorted POSIX-relative Studio web input paths under `root`."""
    root_path = Path(root)
    paths = []
    for rel in FIXED_INPUTS:
        full = root_path / rel
        if not full.is_file():
            raise FileNotFoundError(
                f"missing Studio input {rel!r} under {root_path}")
        paths.append(rel)

    src_dir = root_path / SRC_ROOT
    if not src_dir.is_dir():
        raise FileNotFoundError(
            f"missing Studio source directory {SRC_ROOT!r} under {root_path}")

    for dirpath, dirnames, filenames in os.walk(src_dir, followlinks=False):
        rel_dir = Path(dirpath).relative_to(src_dir)
        if rel_dir == Path("."):
            dirnames[:] = sorted(
                d for d in dirnames if d not in EXCLUDED_SRC_SUBDIRS)
        else:
            dirnames.sort()
        for name in sorted(filenames):
            file_path = Path(dirpath) / name
            if not file_path.is_file():
                continue
            rel = file_path.relative_to(root_path).as_posix()
            paths.append(rel)

    return sorted(paths)


def compute_stamp(root, paths=None):
    """Compute a deterministic SHA-256 hex digest over all Studio web inputs."""
    if paths is None:
        paths = collect_inputs(root)
    root_path = Path(root)
    digest = hashlib.sha256()
    for rel in paths:
        data = (root_path / rel).read_bytes()
        digest.update(rel.encode("utf-8"))
        digest.update(b"\0")
        digest.update(str(len(data)).encode("ascii"))
        digest.update(b"\0")
        digest.update(data)
        digest.update(b"\0")
    return digest.hexdigest()


def try_git_rev(root):
    """Return HEAD's git revision under `root` for diagnostics, or None."""
    root_path = Path(root)
    if not (root_path / ".git").exists():
        return None
    try:
        proc = subprocess.run(
            ["git", "-C", str(root_path), "rev-parse", "HEAD"],
            capture_output=True,
            text=True,
            check=True,
            timeout=5,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    rev = proc.stdout.strip()
    return rev or None


def format_stamp(digest, git_rev=None):
    """Format the stamp file content: hex digest on line 1, optional git_rev."""
    if git_rev:
        return f"{digest}\ngit_rev={git_rev}\n"
    return f"{digest}\n"


def parse_stamp(text):
    """Extract and validate the 64-char hex digest from stamp file text."""
    for line in text.splitlines():
        candidate = line.strip()
        if candidate:
            return candidate if STAMP_RE.match(candidate) else None
    return None


def main(argv):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--root",
        default=".",
        help="talOS root directory containing studio/ and talOS/telemetry/",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="print sorted POSIX-relative input paths instead of the stamp",
    )
    parser.add_argument(
        "--hash-only",
        action="store_true",
        help="emit only the 64-char hex digest line (no diagnostic git_rev)",
    )
    parser.add_argument(
        "--out",
        help="write output to this file instead of stdout",
    )
    args = parser.parse_args(argv)

    try:
        paths = collect_inputs(args.root)
        if args.list:
            output = "\n".join(paths) + "\n"
        else:
            digest = compute_stamp(args.root, paths)
            git_rev = None if args.hash_only else try_git_rev(args.root)
            output = format_stamp(digest, git_rev)
    except OSError as err:
        print(f"webroot_stamp: {err}", file=sys.stderr)
        return 1

    if args.out:
        Path(args.out).write_text(output, encoding="utf-8")
    else:
        sys.stdout.write(output)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
