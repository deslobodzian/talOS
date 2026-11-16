#!/usr/bin/env bash
#
# Builds and tests a robot repository that consumes talOS against THIS talOS
# checkout instead of the commit its submodule pins, so a talOS change is
# checked against its robot before a push. REQ-DEV-30.
#
#   tools/check_robot_repository.sh PATH [BAZEL_FLAG...]
#
# PATH is the robot repository's git checkout. Its committed tree (`git
# archive HEAD`, which holds no submodule content) is copied to
# ${TMPDIR:-/tmp}/talos-robot-check-<uid>; PATH itself is only read, never
# built in or written to (its git commands take no optional locks, so not
# even its index is refreshed). Uncommitted changes there are not checked
# (one line says so). In the copy it runs
#
#   bazel build --override_module=talos=<this checkout> //...
#   bazel test  --override_module=talos=<this checkout> --test_output=errors //...
#
# with the copy's own .bazelrc, whose `import %workspace%/talos/talos.bazelrc`
# and status command under talos/ (the submodule, empty in the copy) are
# pointed at this checkout first, and every BAZEL_FLAG (tools/prepush.sh
# passes its --disk_cache and --nocache_test_results). Each config's
# --describe-only runs as that repository's talos_graph_test targets, part
# of its `bazel test //...`.
#
# The copy's path is fixed so that every run reuses one Bazel output base;
# one run holds it at a time. On success its Bazel server is shut down and
# the copy removed; on failure the copy is kept and its path printed.
# The last line is the verdict: `robot-check: <PATH> at <commit> passed
# against talOS <commit> (<s> s)`, or `... FAILED: <phase>`, after the
# failing targets (the test phase's Bazel result lines, or the build
# phase's ERROR lines).
#
# Exit: 0 build and test passed, 1 either failed, 2 usage or setup error.
set -uo pipefail

die() { echo "robot-check: $*" >&2; exit 2; }
case "${1:-}" in
  -h|--help) sed -n '2,/^set /p' "$0" | sed '$d;s/^# \{0,1\}//'; exit 0 ;;
  "") die "usage: tools/check_robot_repository.sh PATH [BAZEL_FLAG...]" ;;
esac
source_repo="$1"
shift
bazel_flags=("$@")

talos="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
[ -d "$source_repo" ] || die "not a directory: $source_repo"
source_repo="$(cd "$source_repo" && pwd -P)"
top="$(git --no-optional-locks -C "$source_repo" rev-parse --show-toplevel 2>/dev/null)" ||
  die "not a git repository: $source_repo"
[ "$top" = "$source_repo" ] || die "not the top of a git repository: $source_repo (its top is $top)"
robot_commit="$(git --no-optional-locks -C "$source_repo" rev-parse --short --verify --quiet 'HEAD^{commit}')" ||
  die "$source_repo has no commit"
[ "$source_repo" != "$talos" ] || die "$source_repo is this talOS checkout, not a robot repository"
talos_commit="$(git -C "$talos" rev-parse --short HEAD 2>/dev/null || echo unknown)"
[ -n "$(git -C "$talos" status --porcelain 2>/dev/null)" ] && talos_commit="$talos_commit+changes"
tmp="$(cd "${TMPDIR:-/tmp}" 2>/dev/null && pwd -P)" ||
  die "TMPDIR is not a directory: ${TMPDIR:-/tmp}"
copy="$tmp/talos-robot-check-$(id -u)"
lock="$copy.lock"
start=$SECONDS

# shellcheck source=tools/check_copy_lock.sh
. "$talos/tools/check_copy_lock.sh" || die "cannot read check_copy_lock.sh"
take_copy_lock "$lock" "$copy"

bazel_started=0
passed=0
remove_copy() {
  if [ -e "$copy" ]; then
    chmod -R u+w "$copy" 2>/dev/null
    rm -rf "$copy"
  fi
}
cleanup() {
  if [ "$bazel_started" = 1 ] && [ -d "$copy" ]; then
    (cd "$copy" && bazel shutdown >/dev/null 2>&1)
  fi
  if [ "$passed" = 1 ]; then
    remove_copy
    rm -f "$copy".*.log
  fi
  rm -rf "$lock"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

if [ -n "$(git --no-optional-locks -C "$source_repo" status --porcelain)" ]; then
  echo "robot-check: $source_repo has uncommitted changes; checking its commit $robot_commit only"
fi

remove_copy
rm -f "$copy".*.log
mkdir -p "$copy" || die "cannot make $copy"
git --no-optional-locks -C "$source_repo" archive --format=tar HEAD | tar -xf - -C "$copy" ||
  die "cannot copy $source_repo's HEAD to $copy"
# Its talos.bazelrc import and status script, this checkout's.
point_rc_at_talos "$copy/.bazelrc" talos "$talos"
[ "$?" = 2 ] && die "cannot rewrite $copy/.bazelrc"
echo "robot-check: $source_repo at $robot_commit, against talOS $talos at $talos_commit, in $copy"

verdict_failed() {  # verdict_failed PHASE LOG
  echo "robot-check: failing targets:"
  if [ "$1" = test ]; then
    # Bazel's own result lines (`//pkg:t   FAILED in 1.2 s`, TIMEOUT, NO
    # STATUS, INCOMPLETE, FAILED TO BUILD) only: the test logs that
    # --test_output=errors prints may hold any text.
    grep -E '^(@@?[^ /]*)?//[^ ]+ +(\(cached\) )?(FAILED|TIMEOUT|NO STATUS|INCOMPLETE)( |$)' "$2"
  else
    # A build prints no test log; its errors are Bazel's.
    grep -E '^ERROR: ' "$2"
  fi | sort -u | sed 's/^/  /'
  echo "robot-check: copy kept at $copy, log $2"
  echo "robot-check: $source_repo at $robot_commit FAILED against talOS $talos_commit: $1 ($((SECONDS - start)) s)"
  exit 1
}
# phase NAME BAZEL_ARGS...: runs bazel in the copy, its output shown and
# kept in $copy.NAME.log.
phase() {
  local name="$1"
  shift
  local log="$copy.$name.log" t0=$SECONDS
  bazel_started=1
  echo "robot-check: $name"
  if (cd "$copy" && bazel "$@") 2>&1 | tee "$log"; [ "${PIPESTATUS[0]}" = 0 ]; then
    echo "robot-check: $name ok ($((SECONDS - t0)) s)"
    return 0
  fi
  verdict_failed "$name" "$log"
}

override="--override_module=talos=$talos"
phase build build "$override" ${bazel_flags[@]+"${bazel_flags[@]}"} //...
phase test test "$override" --test_output=errors ${bazel_flags[@]+"${bazel_flags[@]}"} //...
passed=1
echo "robot-check: $source_repo at $robot_commit passed against talOS $talos_commit ($((SECONDS - start)) s)"
