#!/usr/bin/env bash
#
# Run RULES.md 11's pre-push checks on a clean worktree of the commits to
# push, under CI's conditions: tracked files read-only (CI's own step,
# tools/make_sources_read_only.py), nothing untracked or uncommitted from the
# checkout, every test result fresh, and no header read from the host's
# include tree (tools/check_host_includes.py). docs/DEVELOPMENT.md §5 rung 3.
#
#   tools/prepush.sh [--ref <commit>] [--keep] [--full] [--skip-gradle]
#                    [--skip-studio]
#
#   --ref          the commit to check (default HEAD)
#   --keep         leave the worktree in place afterwards
#   --full         also run CI's sanitizer lanes and one soak seed, and
#                  rio-in-the-loop whatever the commits touch; required
#                  for a push of more than 20 commits
#   --skip-gradle  do not run the RoboRIO program's gradle checks or
#                  rio-in-the-loop (refused with --full)
#   --skip-studio  do not run Studio's pnpm checks
#
# First, without --full, when more than 20 commits are to push (since
# <ref>@{upstream}; with no upstream, those on no remote-tracking branch),
# the line `prepush: full SKIPPED (...)`, naming the way out (pass --full,
# or set an upstream); the steps still run, and the run exits 1.
# Steps, in order: read-only, source-lints (tools/source_lints.sh, the
# lints CI's "Source lints (no Bazel)" job runs: test time, links,
# requirement trace, node test matrix, deps, and commissioning against
# format's base, not run when format checks every file; under a second),
# format
# (tools/format.sh --check: clang-format 18 on the C/C++ files changed since
# the merge base with origin/main, else with the upstream, else every file;
# CI's clang-format job checks the same branch diff; FAILED also when
# clang-format-18 cannot be installed or run), `bazel build //...`,
# host-includes, `bazel test --nocache_test_results //...`, tidy
# (tools/tidy.sh with format's base, CI's clang-tidy selection; where it
# exits 3, cannot run on this host, the line `prepush: tidy NOT RUN
# (<its reason>)`, which does not fail the run), then, when the commits to push
# (<ref>@{upstream}...<ref>, else since the merge base with origin/main,
# else every file when neither exists) touch MODULE.bazel, REPO.bazel,
# talos.bazelrc, .bazelversion, third_party/, talOS/, studio/, tools/ or
# the fixtures,
# dependent-robot_repo and dependent-wrist_repo (`bazel test //...
# @talos//talOS/configuration:config_fuzz_test` inside examples/robot_repo
# and examples/wrist_repo, the repositories that depend on talOS) and
# scaffold-robot_repo (tools/check_robot_repo_scaffold.sh: a C++ and a
# Python subsystem scaffolded into a copy of examples/robot_repo, with and
# without target coverage required, and into new_robot --standalone's copy
# of it, then `bazel test //...` in each), robot-repository (when TALOS_ROBOT_REPOSITORY
# names a robot repository's checkout: tools/check_robot_repository.sh on
# it; unset, one `skipped` line that does not fail the run; set, empty
# included, to something that is not a directory, the step FAILED, since it
# names no robot repository; tools/prepush_robot_repository.sh decides),
# gradle (`phoenixTest buildRio -Pcompanion=10.0.0.2 -Pheadless --offline`, when the commits
# touch controller_processor/ or talOS/, whose headers the RoboRIO
# program's build.gradle exports), rio-in-the-loop (`python3
# tools/test_rio_in_the_loop.py --wpilib --output-dir <logs>/rio-in-the-loop`,
# CI's job of that name, on the build step's bazel-bin; about 30 s with a
# warm Gradle daemon; under --full, or when the commits touch
# tools/test_rio_in_the_loop.py, bench/, controller_processor/, talOS/,
# MODULE.bazel, third_party/, .bazelversion or talos.bazelrc), under
# --full only tsan and asan (tools/sanitizer_lane.sh, CI's sanitizer
# jobs; 5 to 10 minutes each cold) and soak (CI's
# soak_test with seed 0 for TALOS_SOAK_SECONDS=48, the shortest run that
# injects every fault kind); after a failed build, host-includes, test,
# tidy, rio-in-the-loop, tsan, asan and soak are SKIPPED; checkout-unchanged, then, when the commits
# touch studio/,
# studio-test and studio-build (`pnpm test`, `pnpm build`).
# Each step prints one line (`prepush: <step> ok`, `prepush: <step> FAILED
# (see <log>)`, `prepush: <step> SKIPPED (<reason>)`, tidy's `NOT RUN`
# above); logs stay in a
# directory under ${TMPDIR:-/tmp}. A commit without the tool a step runs
# (source_lints.sh, format.sh, tidy.sh, check_robot_repo_scaffold.sh,
# test_rio_in_the_loop.py, sanitizer_lane.sh) has that
# step SKIPPED, unless --ref names another commit than the checkout's HEAD
# and the upstream it goes to (if any) lacks the tool too: then it predates
# the tool, and the line says `skipped`, which does not fail the run.
#
# The worktree is ${TMPDIR:-/tmp}/talos-prepush-<uid>, a fixed path so that
# every run reuses one Bazel output base; it is removed on exit, interrupt
# or failure (write bits restored first) unless --keep, and one run holds it
# at a time. Bazel reads and writes the disk cache TALOS_BAZEL_DISK_CACHE
# (default ~/.cache/talos-bazel-disk): build results may come from it, test
# results may not. Where this host allows an unprivileged user and network
# namespace (the probe below), the test step sets TALOS_REQUIRE_NETNS=1 as
# CI's Linux job does, so //talOS/protocol:udp_address_change_test fails
# rather than skips; elsewhere it may skip, and the run says so. Likewise,
# where it finds the browser //studio:drive_browser_test would use, it sets
# TALOS_REQUIRE_BROWSER=1 (and passes TALOS_BROWSER when set); with none,
# that test skips and the run says so. JAVA_HOME, when unset, defaults to
# ~/wpilib/2026/jdk if that directory exists. Studio reuses the checkout's
# installed studio/node_modules when its studio/node_modules/.pnpm/lock.yaml
# matches the worktree's studio/pnpm-lock.yaml; otherwise it runs `pnpm
# install --frozen-lockfile --offline` and reports studio SKIPPED if that
# fails.
#
# Exit: 0 every step passed, 1 a step failed or was skipped unasked, 2 usage
# or setup error.
set -uo pipefail

die() { echo "prepush: $*" >&2; exit 2; }

ref=HEAD
keep=0
full=0
skip_gradle=0
skip_studio=0
while [ $# -gt 0 ]; do
  case "$1" in
    --ref) [ $# -ge 2 ] || die "--ref needs a commit"; ref="$2"; shift ;;
    --keep) keep=1 ;;
    --full) full=1 ;;
    --skip-gradle) skip_gradle=1 ;;
    --skip-studio) skip_studio=1 ;;
    -h|--help) sed -n '2,/^set /p' "$0" | sed '$d;s/^# \{0,1\}//'; exit 0 ;;
    *) die "unknown argument: $1" ;;
  esac
  shift
done
[ "$full" = 0 ] || [ "$skip_gradle" = 0 ] ||
  die "--full runs the RoboRIO program in the loop through Gradle: drop --skip-gradle"
# More commits than this to push need --full (docs/DEVELOPMENT.md §5).
full_after_commits=20

tools="$(CDPATH='' cd -- "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# shellcheck source=tools/prepush_robot_repository.sh
. "$tools/prepush_robot_repository.sh" || die "cannot read prepush_robot_repository.sh"
repo="$(git rev-parse --show-toplevel 2>/dev/null)" || die "not in a git checkout"
commit="$(git -C "$repo" rev-parse --verify --quiet "$ref^{commit}")" ||
  die "not a commit: $ref"
disk_cache="${TALOS_BAZEL_DISK_CACHE:-$HOME/.cache/talos-bazel-disk}"
# Absolute and physical: git -C "$repo" and `cd` would resolve a relative
# TMPDIR against different directories.
tmp="$(CDPATH='' cd -- "${TMPDIR:-/tmp}" 2>/dev/null && pwd -P)" ||
  die "TMPDIR is not a directory: ${TMPDIR:-/tmp}"
wt="$tmp/talos-prepush-$(id -u)"
lock="$wt.lock"
start=$SECONDS
[ "$(CDPATH='' cd -- "$repo" && pwd -P)" != "$wt" ] ||
  die "run from the checkout, not from prepush's own worktree $wt"

bazel_started=0
dependents_started=""
worktree_owned=0
lock_owned=0
retake_owned=0
remove_worktree() {
  if [ -e "$wt" ]; then
    chmod -R u+w "$wt" 2>/dev/null
    git -C "$repo" worktree remove --force "$wt" >/dev/null 2>&1 || rm -rf "$wt"
  fi
  # Still registered with its directory gone (deleted by hand, or by the
  # rm above): only `git worktree prune` drops that, and it prunes every
  # worktree whose directory is missing, so it runs only in this case.
  if git -C "$repo" worktree list --porcelain | grep -qxF "worktree $wt"; then
    git -C "$repo" worktree prune
  fi
}
cleanup() {
  if [ "$worktree_owned" = 1 ]; then
    if [ "$keep" = 1 ]; then
      echo "prepush: worktree kept at $wt (remove: chmod -R u+w $wt && git worktree remove --force $wt)"
    else
      if [ "$bazel_started" = 1 ] && [ -d "$wt" ]; then
        (CDPATH='' cd -- "$wt" && bazel shutdown >/dev/null 2>&1)
      fi
      for dependent in $dependents_started; do
        if [ -d "$wt/examples/$dependent" ]; then
          (CDPATH='' cd -- "$wt/examples/$dependent" && bazel shutdown >/dev/null 2>&1)
        fi
      done
      remove_worktree
    fi
  fi
  if [ "$retake_owned" = 1 ]; then
    rmdir "$lock.retake" 2>/dev/null || true
  fi
  if [ "$lock_owned" = 1 ]; then
    rm -rf "$lock"
  fi
}
install_cleanup_traps() {
  trap cleanup EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM
  trap 'exit 129' HUP
}

# One run at a time owns the worktree path.
# The pid file appears whole (written beside it, then renamed), so a lock
# without one is being taken, or its run was killed while taking it.
# A dead holder's lock is retaken under a second lock, $lock.retake, which
# is held until the new pid is written: two runs that both read the dead pid
# cannot both remove and retake it, since the second finds the retake lock
# (or, after it, a live pid).
write_pid() { echo $$ > "$lock/pid.$$" && mv -f "$lock/pid.$$" "$lock/pid"; }
if ! mkdir "$lock" 2>/dev/null; then
  [ -f "$lock/pid" ] ||
    die "$lock has no pid yet: another prepush is starting (if none is, rmdir $lock)"
  holder="$(cat "$lock/pid" 2>/dev/null || true)"
  case "$holder" in
    ''|*[!0-9]*)
      die "$lock has no valid pid: another prepush is starting or releasing it (if none is, remove $lock)"
      ;;
  esac
  if kill -0 "$holder" 2>/dev/null; then
    die "another prepush (pid $holder) is using $wt"
  fi
  mkdir "$lock.retake" 2>/dev/null ||
    die "another prepush is retaking $lock from a dead run (if none is, rmdir $lock.retake)"
  retake_owned=1
  install_cleanup_traps
  # Under the retake lock: the lock must still be the dead holder's.
  if [ "$(cat "$lock/pid" 2>/dev/null || true)" != "$holder" ]; then
    rmdir "$lock.retake"
    retake_owned=0
    die "another prepush took $lock first"
  fi
  rm -rf "$lock"
  if ! mkdir "$lock" 2>/dev/null; then
    die "cannot take $lock"
  fi
  lock_owned=1
  if ! write_pid; then
    die "cannot take $lock"
  fi
  rmdir "$lock.retake"
  retake_owned=0
else
  lock_owned=1
  install_cleanup_traps
  write_pid || die "cannot write $lock/pid"
fi

worktree_owned=1

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
  if (CDPATH='' cd -- "$wt" && "$@") >"$log" 2>&1; then
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
# The range is from the checked branch's upstream (--ref's own when it
# names a branch), else from the merge base with origin/main; when neither
# exists (or shares no history with the commit), every optional step runs.
upstream="$(git -C "$repo" rev-parse --abbrev-ref --symbolic-full-name "$ref@{upstream}" 2>/dev/null || true)"
touches_base=""
touches_from=""
if [ -n "$upstream" ]; then
  touches_base="$upstream"
  touches_from="$upstream"
elif git -C "$repo" rev-parse --verify --quiet refs/remotes/origin/main >/dev/null; then
  touches_base="$(git -C "$repo" merge-base refs/remotes/origin/main "$commit" 2>/dev/null || true)"
  touches_from="origin/main"
fi
touches() {  # touches PATH...: the pushed range changes something under PATH.
  [ -z "$touches_base" ] && return 0
  local changed
  changed="$(git -C "$repo" diff --name-only "$touches_base...$commit" -- "$@")" || return 0
  [ -n "$changed" ]
}

# A long range carries more than the default steps catch (the sanitizer
# lanes and the soak run only under --full). Without an upstream, the
# commits to push are those on no remote-tracking branch.
if [ -n "$upstream" ]; then
  pushing="$(git -C "$repo" rev-list --count "$upstream..$commit")"
  pushing_from="since $upstream"
  way_out="pass --full"
else
  pushing="$(git -C "$repo" rev-list --count "$commit" --not --remotes)"
  pushing_from="on no remote-tracking branch"
  way_out="set an upstream or pass --full"
fi
if [ "$full" = 0 ] && [ "${pushing:-0}" -gt "$full_after_commits" ]; then
  skip full "$pushing commits $pushing_from: docs/DEVELOPMENT.md §5 requires --full for a push of more than $full_after_commits; $way_out"
fi

# no_tool NAME PATH: the commit has no PATH, the tool step NAME runs. It
# predates the tool, nothing to run and not a failure, only when it is
# checked by --ref as another commit than the checkout's HEAD and the
# upstream it goes to (if any) lacks PATH too; a commit that deletes or
# renames the tool would otherwise pass with the step unrun.
head="$(git -C "$repo" rev-parse --verify --quiet HEAD)"
no_tool() {
  if [ "$commit" != "$head" ] &&
      { [ -z "$upstream" ] ||
        ! git -C "$repo" cat-file -e "$upstream:$2" 2>/dev/null; }; then
    report "$1 skipped (this commit, older than the tool, has no $2)"
  else
    skip "$1" "this commit has no $2"
  fi
}

run_step read-only python3 "$tools/make_sources_read_only.py" || {
  echo "prepush: cannot make the worktree read-only; stopping"
  exit 1
}

# CI's clang-format, clang-tidy and commissioning lint check a topic
# branch's whole diff against its merge base with main; so do these, through the commit's own
# tools/format.sh and tools/tidy.sh (which take that merge base). Without
# origin/main, the diff is the upstream's; without either, every file.
if main_ref="$(git -C "$repo" rev-parse --verify --quiet refs/remotes/origin/main)"; then
  diff_base=(--base "$main_ref")
  report "format and tidy check the changes since the merge base with origin/main"
elif [ -n "$upstream" ]; then
  diff_base=(--base "$(git -C "$repo" rev-parse --verify "$upstream")")
  report "format and tidy check the changes since the merge base with $upstream (no origin/main)"
else
  diff_base=(--all)
  report "format and tidy check every file (no origin/main and no upstream)"
fi

# CI's source lints, from the commit's own script (CI runs the same one).
# Its commissioning lint diffs against format's base; with none, it does
# not run.
if [ -f "$wt/tools/source_lints.sh" ]; then
  lint_base=()
  [ "${diff_base[0]}" = --base ] && lint_base=("${diff_base[@]}")
  # The `+` form: bash 3.2 (macOS) rejects an empty array under set -u.
  run_step source-lints bash tools/source_lints.sh \
    ${lint_base[@]+"${lint_base[@]}"} || true
else
  no_tool source-lints tools/source_lints.sh
fi
if [ -f "$wt/tools/format.sh" ]; then
  run_step format bash tools/format.sh --check "${diff_base[@]}" || true
else
  no_tool format tools/format.sh
fi

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
# each one it needs, and one of a target since deleted is gone. The output
# path is asked for once, here: a `bazel info` between the build and the
# test makes the test re-run rules_python's pip extension, which fetches
# pypi index pages, so a network blip there failed the test at analysis.
built=0
if run_step build sh -c \
    'out="$(bazel info "$@" output_path)" && printf "%s\n" "$out" >"$0" &&
     { [ ! -d "$out" ] || find "$out" -name "*.d" -type f -delete; } &&
     bazel build //... "$@"' "$logs/output_path" "${bazel_flags[@]}"; then
  built=1
  run_step host-includes sh -c \
    'python3 "$1" "$(cat "$2")"' _ \
    "$tools/check_host_includes.py" "$logs/output_path" || true
  # The same probe as the test's: a user and network namespace that can
  # create a tap device.
  netns=()
  if unshare --user --map-root-user --net ip tuntap add dev tap0 mode tap \
      >/dev/null 2>&1; then
    netns=(--test_env=TALOS_REQUIRE_NETNS=1)
    report "test requires network namespaces (TALOS_REQUIRE_NETNS=1, as CI)"
  else
    report "test may skip udp_address_change_test: this host refuses an unprivileged network namespace (CI requires it)"
  fi
  # The browser the drive_browser_test would find: TALOS_BROWSER, else its
  # names on the test's default PATH, else macOS's Chrome.
  browser=()
  browser_path="${TALOS_BROWSER:-}"
  if [ -z "$browser_path" ]; then
    for name in chromium chromium-browser google-chrome google-chrome-stable; do
      browser_path="$(PATH=/bin:/usr/bin:/usr/local/bin command -v "$name")" && break
      browser_path=""
    done
  fi
  if [ -z "$browser_path" ] && [ -x "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome" ]; then
    browser_path="/Applications/Google Chrome.app/Contents/MacOS/Google Chrome"
  fi
  if [ -n "$browser_path" ]; then
    browser=(--test_env=TALOS_REQUIRE_BROWSER=1)
    [ -n "${TALOS_BROWSER:-}" ] && browser+=(--test_env=TALOS_BROWSER)
    report "test requires a browser for drive_browser_test ($browser_path; TALOS_REQUIRE_BROWSER=1, as CI)"
  else
    report "test will skip drive_browser_test: no Chromium or Chrome on this host (CI requires one)"
  fi
  run_step test bazel test //... --test_output=errors --nocache_test_results \
    ${netns[@]+"${netns[@]}"} ${browser[@]+"${browser[@]}"} \
    "${bazel_flags[@]}" || true
  # CI's clang-tidy step, through tools/tidy.sh, which reads the generated
  # headers the build wrote. Its exit 3 is a host it cannot run on (clang 18
  # cannot parse the host's C++ standard library, or no compilation
  # database): said plainly, without failing the run, since CI runs it.
  if [ ! -f "$wt/tools/tidy.sh" ]; then
    no_tool tidy tools/tidy.sh
  else
    t0=$SECONDS
    tidy_status=0
    (CDPATH='' cd -- "$wt" && bash tools/tidy.sh "${diff_base[@]}") >"$logs/tidy.log" 2>&1 ||
      tidy_status=$?
    case $tidy_status in
      0) report "tidy ok ($((SECONDS - t0)) s)" ;;
      3)
        reason="$(sed -n 's/^tidy: cannot run here: //p' "$logs/tidy.log" | head -n 1)"
        reason="${reason%:}"
        reason="${reason%; tried}"
        report "tidy NOT RUN (${reason:-tools/tidy.sh exited 3, see $logs/tidy.log})"
        ;;
      *)
        report "tidy FAILED (see $logs/tidy.log)"
        tail -n 40 "$logs/tidy.log" | sed 's/^/  | /'
        failed=1
        ;;
    esac
  fi
else
  skip host-includes "the build failed"
  skip test "the build failed"
  skip tidy "the build failed"
fi
# The repositories that depend on talOS as @talos, each its own workspace,
# as CI's robot-repo job runs them. They write only ignored files (their
# bazel-* links and lockfiles), which checkout-unchanged then confirms.
# What they build from talOS: its module file, REPO.bazel (which applies to
# @talos as to any repository), the toml++ build file and patch in
# third_party/, and packages under talOS/, studio/ (examples/robot_repo
# depends on @talos//studio/bridge and @talos//studio:web_dist), and tools/
# (`bazel query 'buildfiles(deps(//...))'` in each). Each fixture's .bazelrc
# imports talOS's talos.bazelrc, and robot_repo's talos_pin_test compares its
# .bazelversion with talOS's; talOS's own .bazelrc is not read there.
dependent_paths=(MODULE.bazel REPO.bazel talos.bazelrc .bazelversion third_party/ talOS/ studio/ tools/ examples/robot_repo/ examples/wrist_repo/)
if ! touches "${dependent_paths[@]}"; then
  report "dependents not needed (nothing under ${dependent_paths[*]} since $touches_from)"
elif [ ! -d "$wt/examples/robot_repo" ] && [ ! -d "$wt/examples/wrist_repo" ]; then
  report "dependents not needed (this commit has no examples/robot_repo or examples/wrist_repo)"
else
  for dependent in robot_repo wrist_repo; do
    if [ ! -d "$wt/examples/$dependent" ]; then
      report "dependent-$dependent not needed (this commit has no examples/$dependent)"
      continue
    fi
    dependents_started="${dependents_started:+$dependents_started }$dependent"
    run_step "dependent-$dependent" sh -c \
      'CDPATH="" cd -- "examples/$1" && shift && bazel test //... @talos//talOS/configuration:config_fuzz_test "$@"' _ \
      "$dependent" --test_output=errors --nocache_test_results --noshow_progress \
      "--disk_cache=$disk_cache" || true
  done
  # What new_subsystem writes, built in a copy of robot_repo, with and
  # without target coverage required, as CI's robot-repo job runs it.
  if [ -d "$wt/examples/robot_repo" ]; then
    if [ -f "$wt/tools/check_robot_repo_scaffold.sh" ]; then
      # The step runs from the worktree: hand it the absolute TMPDIR.
      run_step scaffold-robot_repo env "TMPDIR=$tmp" \
        bash tools/check_robot_repo_scaffold.sh \
        --noshow_progress --nocache_test_results "--disk_cache=$disk_cache" ||
        true
    else
      no_tool scaffold-robot_repo tools/check_robot_repo_scaffold.sh
    fi
  fi
fi
# A robot repository that consumes talOS (TALOS_ROBOT_REPOSITORY, its
# checkout), built and tested against this commit's talOS in a copy of its
# committed tree; only read there. Until such a repository has CI of its
# own, this is the only check of it against a talOS change.
robot_repository="$(robot_repository_decision)"
case $? in
  0)
    if [ ! -f "$wt/tools/check_robot_repository.sh" ]; then
      skip robot-repository "this commit has no tools/check_robot_repository.sh"
    else
      run_step robot-repository env "TMPDIR=$tmp" \
        bash tools/check_robot_repository.sh "$robot_repository" \
        --noshow_progress --nocache_test_results "--disk_cache=$disk_cache" ||
        true
    fi
    ;;
  1) report "robot-repository skipped ($robot_repository)" ;;
  *)
    report "robot-repository FAILED ($robot_repository)"
    failed=1
    ;;
esac
# build.gradle compiles sources from talOS/protocol, talOS/hardware and
# talOS/driver_station and exports every header under talOS/.
if [ -z "${JAVA_HOME:-}" ] && [ -d "$HOME/wpilib/2026/jdk" ]; then
  export JAVA_HOME="$HOME/wpilib/2026/jdk"
fi
rio_paths=(controller_processor/ talOS/)
if [ "$skip_gradle" = 1 ]; then
  report "gradle skipped (--skip-gradle)"
elif ! touches "${rio_paths[@]}"; then
  report "gradle not needed (nothing under ${rio_paths[*]} since $touches_from)"
else
  run_step gradle sh -c \
    'CDPATH="" cd -- controller_processor/rio && ./gradlew phoenixTest buildRio -Pcompanion=10.0.0.2 -Pheadless --offline' ||
    true
fi

# CI's "RoboRIO program in the loop (WPILib sim)" job: the bench's swerve
# module node and the bridge, from the build step's bazel-bin, against the
# RoboRIO program under WPILib desktop simulation (Gradle builds it). It
# binds UDP ports 5802 and 5803, and in the worktree writes only ignored
# files (Gradle's build/ and .gradle/, the simulation's ctre_sim/ and its
# saved talos_config.bin), which checkout-unchanged then confirms. Its
# binaries are built from much of talOS/ (events, rtms, ipc, configuration,
# schema, ...) and from the module's dependencies.
rio_loop_paths=(tools/test_rio_in_the_loop.py bench/ controller_processor/
  talOS/ MODULE.bazel third_party/ .bazelversion talos.bazelrc)
if [ "$skip_gradle" = 1 ]; then
  report "rio-in-the-loop skipped (--skip-gradle)"
elif [ "$full" = 0 ] && ! touches "${rio_loop_paths[@]}"; then
  report "rio-in-the-loop not needed (nothing under ${rio_loop_paths[*]} since $touches_from)"
elif [ ! -f "$wt/tools/test_rio_in_the_loop.py" ]; then
  no_tool rio-in-the-loop tools/test_rio_in_the_loop.py
elif [ "$built" = 0 ]; then
  skip rio-in-the-loop "the build failed"
else
  run_step rio-in-the-loop python3 tools/test_rio_in_the_loop.py --wpilib \
    --output-dir "$logs/rio-in-the-loop" || true
fi

# CI's sanitizer jobs, from the commit's own tools/sanitizer_lane.sh (CI
# runs the same one), and one soak seed: CI's soak with the seed its branch
# pushes use, for 48 s of enabled cycles rather than 180 s, the shortest run
# that injects every fault kind once (bench/main_processor/simulation/README.md).
if [ "$full" = 1 ] && [ "$built" = 0 ]; then
  for step in tsan asan soak; do skip "$step" "the build failed"; done
elif [ "$full" = 1 ]; then
  for lane in tsan asan; do
    if [ ! -f "$wt/tools/sanitizer_lane.sh" ]; then
      no_tool "$lane" tools/sanitizer_lane.sh
      continue
    fi
    run_step "$lane" bash tools/sanitizer_lane.sh "$lane" \
      --nocache_test_results --noshow_progress "--disk_cache=$disk_cache" ||
      true
  done
  run_step soak bazel test //bench/main_processor/simulation:soak_test \
    --test_output=errors --nocache_test_results \
    --test_env=TALOS_SOAK_SECONDS=48 --test_env=TALOS_SOAK_SEED=0 \
    --noshow_progress "--disk_cache=$disk_cache" || true
else
  report "tsan, asan and soak not run (they run under --full)"
fi

run_step checkout-unchanged sh -c \
  'out="$(git status --porcelain --untracked-files=all)" && { [ -z "$out" ] || { printf "%s\n" "$out"; exit 1; }; }' ||
  true

if [ "$skip_studio" = 1 ]; then
  report "studio skipped (--skip-studio)"
elif ! touches studio/; then
  report "studio not needed (nothing under studio/ since $touches_from)"
else
  studio_ready=1
  if [ -d "$repo/studio/node_modules" ] &&
      [ -f "$repo/studio/node_modules/.pnpm/lock.yaml" ] &&
      cmp -s "$repo/studio/node_modules/.pnpm/lock.yaml" "$wt/studio/pnpm-lock.yaml"; then
    ln -s "$repo/studio/node_modules" "$wt/studio/node_modules"
  elif ! run_step studio-install sh -c \
      'CDPATH="" cd -- studio && pnpm install --frozen-lockfile --offline'; then
    studio_ready=0
    skip studio "no studio/node_modules in $repo installed from this pnpm-lock.yaml, and an offline pnpm install failed"
  fi
  if [ "$studio_ready" = 1 ]; then
    run_step studio-test sh -c 'CDPATH="" cd -- studio && pnpm test' || true
    run_step studio-build sh -c 'CDPATH="" cd -- studio && pnpm build' || true
  fi
fi

if [ "$failed" = 0 ]; then
  report "all steps passed in $((SECONDS - start)) s"
  exit 0
fi
report "FAILED after $((SECONDS - start)) s"
exit 1
