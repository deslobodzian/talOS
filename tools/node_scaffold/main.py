#!/usr/bin/env python3
"""CLI entry point for talOS repository-aware node scaffolding tool."""

import argparse
import sys
from pathlib import Path

# Add script directory to sys.path to enable direct execution
_SCRIPT_DIR = Path(__file__).resolve().parent
if str(_SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(_SCRIPT_DIR))

from scaffold import scaffold_node


def parse_args(argv=None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Scaffold a modern repository-aware standalone talOS node module."
    )
    parser.add_argument(
        "--name",
        required=True,
        help="Subsystem / node name (e.g. arm, intake, shooter).",
    )
    parser.add_argument(
        "--lang",
        required=True,
        choices=["cpp", "python"],
        help="Programming language for the node (cpp or python).",
    )
    parser.add_argument(
        "--hardware",
        action="store_true",
        default=False,
        help="Include hardware device configuration and motor/sensor bindings.",
    )
    parser.add_argument(
        "--out",
        required=True,
        help="Output directory where the standalone module will be scaffolded.",
    )
    parser.add_argument(
        "--sdk-path",
        default=None,
        help="Path to talOS SDK workspace (defaults to relative path to talOS root or ../talos).",
    )
    parser.add_argument(
        "--force",
        action="store_true",
        default=False,
        help="Overwrite existing files in output directory if non-empty.",
    )
    return parser.parse_args(argv)


def main(argv=None) -> int:
    args = parse_args(argv)
    try:
        written = scaffold_node(
            name=args.name,
            lang=args.lang,
            out=args.out,
            hardware=args.hardware,
            sdk_path=args.sdk_path,
            force=args.force,
        )
    except Exception as e:
        print(f"error: {e}", file=sys.stderr)
        return 1

    print(f"Successfully scaffolded standalone talOS node '{args.name}' ({args.lang}):")
    for p in written:
        print(f"  created {p}")
    print("\nNext steps:")
    print(f"  cd {args.out}")
    print("  bazel test //...")
    return 0


if __name__ == "__main__":
    sys.exit(main())
