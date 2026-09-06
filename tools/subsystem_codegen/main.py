import argparse
import sys
from pathlib import Path

from parser import parse_subsystem
from generator import generate, robot_stanza


def main() -> int:
    cli = argparse.ArgumentParser(
        description="Generate an ARCHITECTURE.md 'A new subsystem' package "
                    "from a split-format subsystem.toml.")
    cli.add_argument("subsystem_toml", help="path to subsystem.toml")
    cli.add_argument("--out", required=True,
                     help="scratch dir; the package lands in <out>/<name>/")
    args = cli.parse_args()

    try:
        subsystem = parse_subsystem(args.subsystem_toml)
    except ValueError as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    written = generate(subsystem, args.out)
    for path in written:
        print(f"wrote {path}")
    print("--- robot.toml stanza (merge by hand; never auto-applied) ---")
    print(robot_stanza(subsystem), end="")
    return 0


if __name__ == "__main__":
    sys.exit(main())
