#!/usr/bin/env python3

import argparse
import subprocess
import sys
from pathlib import Path


FIRST_PARTY_DIRS = ("common", "robot", "talOS", "vision")
FORMAT_SUFFIXES = {".cc", ".cpp", ".h", ".hpp"}
TIDY_SUFFIXES = {".cc", ".cpp"}


def repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def collect_files(root: Path, roots: tuple[str, ...],
                  suffixes: set[str]) -> list[Path]:
    files: list[Path] = []
    for relative_root in roots:
        scan_root = root / relative_root
        if not scan_root.exists():
            continue
        for path in scan_root.rglob("*"):
            if not path.is_file() or path.suffix not in suffixes:
                continue
            if "third_party" in path.parts:
                continue
            files.append(path)
    return sorted(files)


def run(command: list[str], root: Path) -> None:
    print(" ".join(command))
    completed = subprocess.run(command, cwd=root, check=False)
    if completed.returncode != 0:
        raise SystemExit(completed.returncode)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run TalOS C++ formatting and static analysis.")
    parser.add_argument(
        "--fix-format",
        action="store_true",
        help="Rewrite files with clang-format instead of checking them.",
    )
    args = parser.parse_args()

    root = repo_root()
    format_files = collect_files(root, FIRST_PARTY_DIRS, FORMAT_SUFFIXES)
    if not format_files:
        print("No first-party C++ files found.")
        return 0

    format_command = ["clang-format"]
    if args.fix_format:
        format_command.append("-i")
    else:
        format_command.extend(["--dry-run", "--Werror"])
    format_command.extend(str(path) for path in format_files)

    print("Running clang-format...")
    run(format_command, root)

    compile_commands = root / "compile_commands.json"
    if not compile_commands.exists():
        print("compile_commands.json not found; skipping clang-tidy.")
        print("Generate it first with the existing Gradle/Bazel workflow.")
        return 0

    tidy_files = collect_files(root, ("common/protocol",), TIDY_SUFFIXES)
    print("Running clang-tidy...")
    for path in tidy_files:
        run(["clang-tidy", str(path), "-p", str(root)], root)

    return 0


if __name__ == "__main__":
    sys.exit(main())
