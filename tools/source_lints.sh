#!/usr/bin/env bash
#
# The source lints, which need no Bazel and take under a second. CI's
# "Source lints (no Bazel)" job and tools/prepush.sh's source-lints step
# both run this script, so they cannot drift apart.
#
#   tools/source_lints.sh [--base <rev>]
#
# From the workspace that holds this script, whatever the working
# directory, it runs, in order:
#
#   test-time    tools/lint_test_time.py: no wall clock in tests or node
#                code, no test retries (docs/TESTING.md §6.1, §8.1;
#                exceptions in tools/lint_test_time_allowlist.txt, which
#                may only shrink)
#   links        tools/check_links.py: every relative markdown link and
#                anchor resolves
#   trace        tools/trace.py --check: every SAFE, CONTRACT and P0
#                requirement has a tagged test (docs/TESTING.md §3,
#                docs/DESIGN.md §6; known gaps in tools/trace_waivers.txt,
#                which may only shrink)
#   node-matrix  tools/check_node_matrix.py --baseline
#                tools/check_node_matrix_baseline.txt: every node test has
#                its matrix cases (docs/TESTING.md §7; the baseline may only
#                shrink)
#   deps         tools/check_deps_listed.py: docs/METHODOLOGY.md §13's
#                "Bazel module" table lists exactly MODULE.bazel's
#                bazel_deps
#   commissioning  tools/check_commissioning.py --base <rev>: no commit
#                since the merge base with <rev> sets `commissioned = true`
#                or changes a CAN id in a robot or bench config (RULES.md 4)
#                unless it carries `Commissioned-By: <name>`; without
#                --base (no history to diff against) the line
#                `source-lints: commissioning not run (no --base)`, which
#                does not fail the run
#
# Each lint runs whatever the others find and prints its own output, then
# `source-lints: <lint> ok` or `source-lints: <lint> FAILED`.
#
# Exit: 0 every lint passed, 1 one failed, 2 a usage error or the workspace
# is unreachable.
set -uo pipefail

base=
case "${1:-}" in
  --base) [ $# -eq 2 ] && [ -n "$2" ] ||
            { echo "source-lints: --base needs one revision" >&2; exit 2; }
          base="$2" ;;
  "") ;;
  *) echo "source-lints: unknown argument: $1" >&2; exit 2 ;;
esac

cd "$(dirname "${BASH_SOURCE[0]}")/.." ||
  { echo "source-lints: cannot enter the workspace" >&2; exit 2; }

failed=0
lint() {  # lint NAME COMMAND...
  local name="$1"
  shift
  if "$@"; then
    echo "source-lints: $name ok"
  else
    echo "source-lints: $name FAILED ($*)"
    failed=1
  fi
}

lint test-time python3 tools/lint_test_time.py
lint links python3 tools/check_links.py
lint trace python3 tools/trace.py --check
lint node-matrix python3 tools/check_node_matrix.py \
  --baseline tools/check_node_matrix_baseline.txt
lint deps python3 tools/check_deps_listed.py
if [ -n "$base" ]; then
  lint commissioning python3 tools/check_commissioning.py --base "$base"
else
  echo "source-lints: commissioning not run (no --base)"
fi
exit "$failed"
