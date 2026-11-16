#!/usr/bin/env bash
#
# One sanitizer lane: the whole suite but the soak, instrumented. CI's
# "Bazel (asan, Linux)" and "Bazel (tsan, Linux)" jobs and tools/prepush.sh's
# asan and tsan steps (under --full) both run this script, so they cannot
# drift apart.
#
#   tools/sanitizer_lane.sh asan|tsan [BAZEL_FLAG...]
#
# From the workspace that holds this script, whatever the working
# directory, it runs `bazel test --config=<lane> --test_output=errors //...`
# with any further flags appended. The config's own --test_tag_filters
# (talos.bazelrc) leaves out the no_asan / no_tsan targets; the soak is
# `manual`, so //... never runs it.
#
# Exit: bazel's, or 2 for a usage error.
set -uo pipefail

case "${1:-}" in
  asan|tsan) lane="$1"; shift ;;
  *) echo "usage: tools/sanitizer_lane.sh asan|tsan [BAZEL_FLAG...]" >&2; exit 2 ;;
esac
cd "$(dirname "${BASH_SOURCE[0]}")/.." ||
  { echo "sanitizer-lane: cannot enter the workspace" >&2; exit 2; }
exec bazel test "--config=$lane" --test_output=errors //... "$@"
