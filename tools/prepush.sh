#!/usr/bin/env bash
#
# Run RULES.md 11's pre-push checks on a clean worktree of the commits to
# push, under CI's conditions: tracked files read-only (CI's own step,
# tools/make_sources_read_only.py), nothing untracked or uncommitted from the
# checkout, every test result fresh, and no header read from the host's
# include tree (tools/check_host_includes.py). docs/DEVELOPMENT.md §5 rung 3.
#
#   tools/prepush.sh [--ref <commit>] [--keep] [--skip-gradle] [--skip-studio]
#
#   --ref          the commit to check (default HEAD)
#   --keep         leave the worktree in place afterwards
#   --skip-gradle  do not run the RoboRIO program's gradle checks
#   --skip-studio  do not run Studio's pnpm checks
#
# Steps, in order: read-only, `bazel build //...`, host-includes, `bazel test
# --nocache_test_results //...`, checkout-unchanged; then, when the commits
# to push (@{upstream}..<ref>, or every file if the branch has no upstream)
# touch them: gradle (`phoenixTest buildRio -Pheadless --offline`, for
# 2026-robot/controller_processor/, talOS/hardware/, talOS/protocol/) and
# studio-test and studio-build (`pnpm test`, `pnpm build`, for studio/).
# Each step prints one line (`prepush: <step> ok`, `prepush: <step> FAILED
# (see <log>)`, `prepush: <step> SKIPPED (<reason>)`); logs stay in a
# directory under ${TMPDIR:-/tmp}.
#
# The worktree is ${TMPDIR:-/tmp}/talos-prepush-<uid>, a fixed path so that
# every run reuses one Bazel output base; it is removed on exit, interrupt
# or failure (write bits restored first) unless --keep, and one run holds it
# at a time. Bazel reads and writes the disk cache TALOS_BAZEL_DISK_CACHE
# (default ~/.cache/talos-bazel-disk): build results may come from it, test
# results may not. JAVA_HOME defaults to ~/wpilib/2026/jdk. Studio uses the
# checkout's studio/node_modules when present, else `pnpm install
# --frozen-lockfile --offline`.
#
# Exit: 0 every step passed, 1 a step failed or was skipped unasked, 2 usage
# or setup error.
set -uo pipefail

die() { echo "prepush: $*" >&2; exit 2; }

ref=HEAD
keep=0
skip_gradle=0
skip_studio=0
while [ $# -gt 0 ]; do
  case "$1" in
    --ref) [ $# -ge 2 ] || die "--ref needs a commit"; ref="$2"; shift ;;
    --keep) keep=1 ;;
    --skip-gradle) skip_gradle=1 ;;
    --skip-studio) skip_studio=1 ;;
    -h|--help) sed -n '2,/^set /p' "$0" | sed '$d;s/^# \{0,1\}//'; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
  shift
done

tools="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
repo="$(git rev-parse --show-toplevel 2>/dev/null)" || die "not in a git checkout"
commit="$(git -C "$repo" rev-parse --verify --quiet "$ref^{commit}")" ||
  die "not a commit: $ref"
disk_cache="${TALOS_BAZEL_DISK_CACHE:-$HOME/.cache/talos-bazel-disk}"
tmp="${TMPDIR:-/tmp}"
tmp="${tmp%/}"
wt="$tmp/talos-prepush-$(id -u)"
lock="$wt.lock"
start=$SECONDS

# One run at a time owns the worktree path.
if ! mkdir "$lock" 2>/dev/null; then
  holder="$(cat "$lock/pid" 2>/dev/null || true)"
  if [ -n "$holder" ] && kill -0 "$holder" 2>/dev/null; then
    die "another prepush (pid $holder) is using $wt"
  fi
  rm -rf "$lock"
  mkdir "$lock" 2>/dev/null || die "cannot take $lock"
fi
echo $$ > "$lock/pid"

bazel_started=0
remove_worktree() {
  [ -e "$wt" ] || return 0
  chmod -R u+w "$wt" 2>/dev/null
  git -C "$repo" worktree remove --force "$wt" >/dev/null 2>&1 || rm -rf "$wt"
  git -C "$repo" worktree prune
}
cleanup() {
  if [ "$keep" = 1 ]; then
    echo "prepush: worktree kept at $wt (remove: chmod -R u+w $wt && git worktree remove --force $wt)"
  else
    if [ "$bazel_started" = 1 ] && [ -d "$wt" ]; then
      (cd "$wt" && bazel shutdown >/dev/null 2>&1)
    fi
    remove_worktree
  fi
  rm -rf "$lock"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# A worktree left by --keep or a killed run.
remove_worktree
out="$(git -C "$repo" worktree add --detach "$wt" "$commit" 2>&1)" ||
  die "git worktree add failed: $out"
logs="$(mktemp -d "$tmp/talos-prepush-logs.XXXXXX")" || die "cannot make a log directory"
echo "prepush: $(git -C "$repo" log -1 --format='%h %s' "$commit") in $wt, logs in $logs"

failed=0
report() { echo "prepush: $1"; }
# run_step NAME COMMAND...: runs COMMAND in the worktree, its output to
# $logs/NAME.log, and prints one line.
run_step() {
  local name="$1"
  shift
  local log="$logs/$name.log"
  local t0=$SECONDS
  if (cd "$wt" && "$@") >"$log" 2>&1; then
    report "$name ok ($((SECONDS - t0)) s)"
    return 0
  fi
  report "$name FAILED (see $log)"
  tail -n 40 "$log" | sed 's/^/  | /'
  failed=1
  return 1
}
skip() {  # skip NAME REASON: a step that should have run and did not.
  report "$1 SKIPPED ($2)"
  failed=1
}

# Which optional steps the commits to push need.
upstream="$(git -C "$repo" rev-parse --abbrev-ref --symbolic-full-name '@{upstream}' 2>/dev/null || true)"
touches() {  # touches PATH...: the pushed range changes something under PATH.
  [ -z "$upstream" ] && return 0
  local changed
  changed="$(git -C "$repo" diff --name-only "$upstream...$commit" -- "$@")" || return 0
  [ -n "$changed" ]
}

run_step read-only python3 "$tools/make_sources_read_only.py" || {
  echo "prepush: cannot make the worktree read-only; stopping"
  exit 1
}

bazel_started=1
# A compile that hits the disk cache leaves no `.d` file on disk under
# Bazel's defaults (it is kept in memory, and outputs that are not top-level
# are not downloaded), so host-includes would not see what it read. These
# write every compile's `.d` to the output path; the test step takes them
# too, so it reuses the build's analysis.
bazel_flags=(--noshow_progress "--disk_cache=$disk_cache"
  --experimental_inmemory_dotd_files=false '--remote_download_regex=.*\.d$')
# Before the build, every `.d` in the reused output base is deleted, so
# host-includes reads only this build's: the build rewrites or downloads
# each one it needs, and one of a target since deleted is gone.
if run_step build sh -c \
    'out="$(bazel info "$@" output_path)" &&
     { [ ! -d "$out" ] || find "$out" -name "*.d" -type f -delete; } &&
     bazel build //... "$@"' _ "${bazel_flags[@]}"; then
  run_step host-includes sh -c \
    'python3 "$1" "$(shift; bazel info "$@" output_path)"' _ \
    "$tools/check_host_includes.py" "${bazel_flags[@]}" || true
  run_step test bazel test //... --test_output=errors --nocache_test_results \
    "${bazel_flags[@]}" || true
else
  skip host-includes "the build failed"
  skip test "the build failed"
fi
run_step checkout-unchanged sh -c \
  'out="$(git status --porcelain --untracked-files=all)"; [ -z "$out" ] || { printf "%s\n" "$out"; exit 1; }' ||
  true

rio_paths=(2026-robot/controller_processor/ talOS/hardware/ talOS/protocol/)
if [ "$skip_gradle" = 1 ]; then
  report "gradle skipped (--skip-gradle)"
elif ! touches "${rio_paths[@]}"; then
  report "gradle not needed (nothing under ${rio_paths[*]} since $upstream)"
else
  export JAVA_HOME="${JAVA_HOME:-$HOME/wpilib/2026/jdk}"
  run_step gradle sh -c \
    'cd 2026-robot/controller_processor/rio && ./gradlew phoenixTest buildRio -Pheadless --offline' ||
    true
fi

if [ "$skip_studio" = 1 ]; then
  report "studio skipped (--skip-studio)"
elif ! touches studio/; then
  report "studio not needed (nothing under studio/ since $upstream)"
else
  studio_ready=1
  if [ -d "$repo/studio/node_modules" ]; then
    ln -s "$repo/studio/node_modules" "$wt/studio/node_modules"
  elif ! run_step studio-install sh -c \
      'cd studio && pnpm install --frozen-lockfile --offline'; then
    studio_ready=0
    skip studio "no studio/node_modules in $repo and an offline pnpm install failed"
  fi
  if [ "$studio_ready" = 1 ]; then
    run_step studio-test sh -c 'cd studio && pnpm test' || true
    run_step studio-build sh -c 'cd studio && pnpm build' || true
  fi
fi

if [ "$failed" = 0 ]; then
  report "all steps passed in $((SECONDS - start)) s"
  exit 0
fi
report "FAILED after $((SECONDS - start)) s"
exit 1
