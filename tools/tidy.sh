#!/usr/bin/env bash
#
# Run clang-tidy 18, the version CI runs, on the C++ files changed against a
# base. The selection logic is not copied: this extracts the Python step
# "Run clang-tidy on changed sources and headers" from
# .github/workflows/continuous-integration.yaml and runs it, so the changed
# sources, the includers analysed for a changed header, the test-code check
# exemption and the OFF_GRAPH list are CI's own. It sees committed changes
# only (git diff BASE HEAD, as in CI): commit first. Tool setup:
# tools/clang18.sh.
#
#   tools/tidy.sh [--base <ref> | --all] [--refresh]
#
#   --base     diff against the merge base with <ref> (default origin/main)
#   --all      every tracked C++ file (slow)
#   --refresh  regenerate compile_commands.json even if it exists
#
# compile_commands.json comes from CI's command, `bazel run
# @hedron_compile_commands//:refresh_all`, when missing. Headers generated
# by Bazel (flatbuffers, topics.h) must exist under bazel-out, so build the
# touched packages first (`bazel build //talOS/...` or the package's
# targets), as CI's build step does before tidy.
#
# Exit: 0 clean, 1 findings, 2 usage or tool error, 3 cannot run on this host
# (no bazel, or the compilation database could not be produced).
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"
. "$(dirname "${BASH_SOURCE[0]}")/clang18.sh"

base_arg=origin/main
refresh=0
while [ $# -gt 0 ]; do
  case "$1" in
    --all) base_arg=--all ;;
    --base) [ $# -ge 2 ] || clang18_die "--base needs a ref"; base_arg="$2"; shift ;;
    --refresh) refresh=1 ;;
    -h|--help) sed -n '2,/^set /p' "$0" | sed '$d;s/^# \{0,1\}//'; exit 0 ;;
    *) clang18_die "unknown argument: $1" ;;
  esac
  shift
done

wf=.github/workflows/continuous-integration.yaml
tmp="$(mktemp -d)"
trap 'rm -rf "$tmp"' EXIT
sed -n "/python3 - \"\\\$BASE\" <<'PYEOF'/,/^ *PYEOF\$/p" "$wf" | sed '1d;$d;s/^        //' > "$tmp/tidy.py"
grep -q 'clang-tidy-18' "$tmp/tidy.py" ||
  clang18_die "could not extract the clang-tidy step from $wf; update tools/tidy.sh"

command -v python3 >/dev/null 2>&1 || clang18_die "need python3"
clang18_setup
clang-tidy-18 --version >/dev/null 2>&1 || clang18_die "clang-tidy-18 does not run"

if [ "$refresh" = 1 ] || [ ! -f compile_commands.json ]; then
  if ! command -v bazel >/dev/null 2>&1; then
    echo "tidy: cannot run here: no compile_commands.json and no bazel to make it" >&2
    exit 3
  fi
  bazel run --disk_cache="$HOME/.cache/bazel-disk" --noshow_progress \
    @hedron_compile_commands//:refresh_all ||
    { echo "tidy: cannot run here: compilation database refresh failed" >&2; exit 3; }
fi

if [ "$base_arg" = --all ]; then
  # git's empty tree: everything counts as changed.
  BASE="$(git hash-object -t tree /dev/null)"
else
  clang18_base "$base_arg"
fi
python3 "$tmp/tidy.py" "$BASE"
