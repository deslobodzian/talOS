#!/usr/bin/env bash
#
# The source lints, which need no Bazel and take under a second. CI's
# "Source lints (no Bazel)" job and tools/prepush.sh's source-lints step
# both run this script, so they cannot drift apart.
#
#   tools/source_lints.sh
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
#
# Each lint runs whatever the others find and prints its own output, then
# `source-lints: <lint> ok` or `source-lints: <lint> FAILED`.
#
# Exit: 0 every lint passed, 1 one failed, 2 the workspace is unreachable.
set -uo pipefail

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
exit "$failed"
