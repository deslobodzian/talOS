#!/usr/bin/env bash
#
# Builds what `bazel run @talos//tools:new_subsystem` writes into a robot
# repository: in a copy of examples/robot_repo (the fixture is never
# modified), it scaffolds a C++ and a Python subsystem and runs
# `bazel test //...`, the graph test, node tests and sim smoke test included.
# REQ-DEV-01, REQ-DEV-23, REQ-DEV-30.
#
#   tools/check_robot_repo_scaffold.sh [BAZEL_FLAG...]
#
# Three variants run in one copy, reset between them:
#   plain      the fixture as it is (target coverage not required);
#   coverage   [robot] require_target_coverage = true set before scaffolding,
#              as a commissioned robot resolves it. The fixture's own
#              mechanisms (roller, gate) have no producer, so that variant
#              first gives each the empty [targets] policy the scaffold
#              writes under coverage; otherwise their lanes fail the lint
#              before the scaffold is judged;
#   new_robot  the repository `new_robot --standalone` makes of the fixture
#              (it refuses a fixture file its TEMPLATE does not place)
#              replaces the copy's files: run as @talos//tools:new_robot in
#              the copy, whose output base has built it, into a sibling
#              directory, then moved into the copy's path so that the
#              build reuses that output base (its talOS path, relative to
#              a sibling or absolute, holds there too). Then talOS goes in
#              as its README's git submodule would: a `git archive` of this
#              checkout at talos/, the overrides pointed there, and
#              `bazel build --nobuild //...`, which loads every package and
#              fails if //... descends into talos/ (the .bazelignore
#              new_robot writes keeps it out).
# The copy's MODULE.bazel points @talos (and studio_web), and its .bazelrc
# its import of talos.bazelrc, at the talOS checkout holding this script. BAZEL_FLAGs go to every `bazel run` and
# `bazel test` (tools/prepush.sh passes its --disk_cache and
# --nocache_test_results; CI's disk cache comes from its bazelrc).
#
# The copy is ${TMPDIR:-/tmp}/talos-scaffold-check-<uid>, a fixed path so
# that every run reuses one Bazel output base; one run holds it at a time.
# On every exit its Bazel server is shut down and the copy removed. Each
# phase prints its time.
#
# Exit: 0 every variant passed, 1 a phase failed, 2 usage or setup error.
set -uo pipefail

die() { echo "scaffold-check: $*" >&2; exit 2; }
case "${1:-}" in
  -h|--help) sed -n '2,/^set /p' "$0" | sed '$d;s/^# \{0,1\}//'; exit 0 ;;
esac
bazel_flags=("$@")

talos="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
fixture="$talos/examples/robot_repo"
[ -f "$fixture/MODULE.bazel" ] || die "no fixture at $fixture"
tmp="$(cd "${TMPDIR:-/tmp}" 2>/dev/null && pwd -P)" ||
  die "TMPDIR is not a directory: ${TMPDIR:-/tmp}"
copy="$tmp/talos-scaffold-check-$(id -u)"
fresh="$tmp/talos-scaffold-new-$(id -u)"
lock="$copy.lock"
start=$SECONDS

# One run at a time owns the copy's path (and its output base).
# shellcheck source=tools/check_copy_lock.sh
. "$talos/tools/check_copy_lock.sh" || die "cannot read check_copy_lock.sh"
take_copy_lock "$lock" "$copy"

bazel_started=0
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
  remove_copy
  rm -rf "$fresh"
  rm -rf "$lock"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# phase NAME COMMAND...: runs COMMAND in the copy, its output shown, and
# prints its time; exits 1 when it fails.
phase() {
  local name="$1"
  shift
  local t0=$SECONDS
  echo "scaffold-check: $name"
  if (cd "$copy" && "$@"); then
    echo "scaffold-check: $name ok ($((SECONDS - t0)) s)"
    return 0
  fi
  echo "scaffold-check: $name FAILED ($((SECONDS - t0)) s)"
  exit 1
}

# The fixture's files at $copy, without the links and lockfile a build in
# the checkout leaves there. The copy's own bazel-* links stay, so a reset
# keeps its output base's analysis.
reset_copy() {
  mkdir -p "$copy" || die "cannot make $copy"
  find "$copy" -mindepth 1 -maxdepth 1 ! -name 'bazel-*' -exec rm -rf {} + ||
    die "cannot clear $copy"
  (cd "$fixture" && tar -cf - --exclude='./bazel-*' --exclude=./MODULE.bazel.lock .) |
    (cd "$copy" && tar -xf -) || die "cannot copy $fixture"
  # tools/prepush.sh checks a read-only worktree; the copy must be writable.
  chmod -R u+w "$copy"
  local module="$copy/MODULE.bazel"
  grep -qxF '    path = "../..",' "$module" &&
    grep -qxF '    path = "../../bazel-bin/studio/dist",' "$module" ||
    die "$fixture/MODULE.bazel no longer names talOS as ../..; update this script"
  sed -e "s|^    path = \"\\.\\./\\.\\.\",\$|    path = \"$talos\",|" \
    -e "s|^    path = \"\\.\\./\\.\\./bazel-bin/studio/dist\",\$|    path = \"$talos/bazel-bin/studio/dist\",|" \
    "$module" > "$module.new" && mv "$module.new" "$module" ||
    die "cannot rewrite $module"
  # .bazelrc takes talOS's flags from the same checkout (talos_pin_test).
  local rc="$copy/.bazelrc"
  grep -qxF 'import %workspace%/../../talos.bazelrc' "$rc" &&
    grep -qF -- '--workspace_status_command=../../tools/workspace_status.sh' "$rc" ||
    die "$fixture/.bazelrc no longer imports talOS from ../..; update this script"
  sed -e "s|^import %workspace%/\\.\\./\\.\\./talos\\.bazelrc\$|import $talos/talos.bazelrc|" \
    -e "s|--workspace_status_command=\\.\\./\\.\\./tools/|--workspace_status_command=$talos/tools/|" \
    "$rc" > "$rc.new" && mv "$rc.new" "$rc" || die "cannot rewrite $rc"
}

# [robot] require_target_coverage = true, and the empty [targets] policy
# for each fixture mechanism whose policy is still the commented default.
require_coverage() {
  awk '{ print } /^\[robot\]$/ { print "require_target_coverage = true" }' \
    robot.toml > robot.toml.new && mv robot.toml.new robot.toml || return 1
  grep -qx 'require_target_coverage = true' robot.toml || return 1
  local file
  for file in subsystems/*.toml; do
    grep -qx '# \[targets\]' "$file" || continue
    printf '%s\n' '' \
      '# Added by tools/check_robot_repo_scaffold.sh: no producer yet.' \
      '[targets]' 'teleop = []' 'autonomous = []' 'test = []' >> "$file"
  done
}

# The repository new_robot makes, in place of the fixture copy's files.
new_robot() {
  rm -rf "$fresh" &&
    bazel run ${bazel_flags[@]+"${bazel_flags[@]}"} @talos//tools:new_robot -- \
      my_robot --standalone "$fresh" || return 1
  find "$copy" -mindepth 1 -maxdepth 1 ! -name 'bazel-*' -exec rm -rf {} + &&
    (cd "$fresh" && tar -cf - .) | (cd "$copy" && tar -xf -) &&
    rm -rf "$fresh"
}

# talOS held at talos/ as a submodule is, the overrides and .bazelrc's
# import pointed there, and every package of //... loaded.
as_submodule() {
  mkdir talos && git -C "$talos" archive HEAD | tar -xf - -C talos ||
    return 1
  sed -e 's|^    path = "[^"]*/bazel-bin/studio/dist",$|    path = "talos/bazel-bin/studio/dist",|' \
    -e '/talos\/bazel-bin/!s|^    path = "[^"]*",$|    path = "talos",|' \
    MODULE.bazel > MODULE.bazel.new && mv MODULE.bazel.new MODULE.bazel ||
    return 1
  grep -qxF '    path = "talos",' MODULE.bazel &&
    grep -qxF '    path = "talos/bazel-bin/studio/dist",' MODULE.bazel ||
    { echo "MODULE.bazel: no override to point at talos/" >&2; return 1; }
  sed -e 's|^import .*/talos\.bazelrc$|import %workspace%/talos/talos.bazelrc|' \
    -e 's|--workspace_status_command=.*/tools/workspace_status\.sh$|--workspace_status_command=talos/tools/workspace_status.sh|' \
    .bazelrc > .bazelrc.new && mv .bazelrc.new .bazelrc || return 1
  grep -qxF 'import %workspace%/talos/talos.bazelrc' .bazelrc ||
    { echo ".bazelrc: no import of talos.bazelrc to point at talos/" >&2; return 1; }
  bazel build --nobuild ${bazel_flags[@]+"${bazel_flags[@]}"} //...
}

scaffold() {  # scaffold NAME LANG
  bazel run ${bazel_flags[@]+"${bazel_flags[@]}"} @talos//tools:new_subsystem -- "$1" --lang "$2"
}

for variant in plain coverage new_robot; do
  reset_copy
  if [ "$variant" = coverage ]; then
    phase "$variant: require target coverage" require_coverage
  fi
  bazel_started=1
  if [ "$variant" = new_robot ]; then
    phase "$variant: new_robot --standalone" new_robot
  fi
  phase "$variant: scaffold intake (cpp)" scaffold intake cpp
  phase "$variant: scaffold feeder (py)" scaffold feeder py
  phase "$variant: bazel test //..." \
    bazel test --test_output=errors ${bazel_flags[@]+"${bazel_flags[@]}"} //...
  if [ "$variant" = new_robot ]; then
    phase "$variant: talOS as a submodule, bazel build --nobuild //..." \
      as_submodule
  fi
done
echo "scaffold-check: every variant passed in $((SECONDS - start)) s"
