"""Report fused/odometry accuracy and per-camera covariance consistency."""

import argparse
from contextlib import nullcontext
from pathlib import Path
import subprocess
import sys
import tempfile

from accuracy import TIMESTAMP_LAG_US, markdown, run_scenario, summary_markdown


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration-s", type=int, default=12,
                        help="autonomous seconds after one disabled second (default: 12)")
    parser.add_argument("--repeat", type=int, default=3,
                        help="independent scenarios to summarize (default: 3)")
    parser.add_argument("--output-dir", type=Path,
                        help="retain sessions and report under this directory")
    parser.add_argument("--keep", action="store_true",
                        help="retain logs in a temporary directory without --output-dir")
    parser.add_argument("--min-yaw-std-rad", type=float,
                        help="override camera yaw std floor for calibration sweeps (radians)")
    args = parser.parse_args()
    if args.duration_s <= 0:
        parser.error("--duration-s must be positive")
    if args.repeat <= 0:
        parser.error("--repeat must be positive")
    if args.output_dir is not None:
        directory = nullcontext(args.output_dir)
    elif args.keep:
        directory = nullcontext(Path(tempfile.mkdtemp(prefix="talos-accuracy-")))
    else:
        directory = tempfile.TemporaryDirectory(prefix="talos-accuracy-")
    try:
        with directory as output:
            results = []
            root = Path(output)
            root.mkdir(parents=True, exist_ok=True)
            print(f"Injected timestamp lag: {TIMESTAMP_LAG_US / 1000:g} ms; "
                  "corrected run subtracts the same offset.\n")
            if args.min_yaw_std_rad is not None:
                print(f"Yaw std floor override: {args.min_yaw_std_rad:g} rad\n")
            for number in range(args.repeat):
                result, destination = run_scenario(
                    args.duration_s, root, args.min_yaw_std_rad)
                results.append(result)
                print(f"\nScenario {number + 1}/{args.repeat}\n")
                print(markdown(result), flush=True)
                if args.keep or args.output_dir is not None:
                    print(f"\nSession logs and report: {destination}")
            summary = summary_markdown(results)
            print("\nIndependent-session summary\n\n" + summary, flush=True)
            (root / "summary.md").write_text(summary + "\n", encoding="utf-8")
            if args.keep or args.output_dir is not None:
                print(f"\nSummary: {root / 'summary.md'}")
    except (OSError, RuntimeError, ValueError, subprocess.SubprocessError) as error:
        print(f"accuracy_scenario: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
