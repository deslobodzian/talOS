# Development workflow

How a change goes from request to pushed, verified commit.

## 1. Intake and requirement

Classify first: a **defect** is tested first; a **flake** needs its
mechanism ([TESTING.md §8](TESTING.md#8-flakes)); a **feature** needs a
requirement and, where [DESIGN.md §1](DESIGN.md#1-when-a-change-needs-a-design-note)
says so, a design note; a **refactor** is proved by existing tests.

Every feature and defect names its `REQ-<ID>` (RULES.md 15). If none exists
and a consumer relies on the behaviour, propose one to the user; the
catalogue changes in its own commit. If no consumer relies on it, do not
build it.

Stop and ask the user before breaking a rule in RULES.md, changing a CAN id
or `commissioned`, adding, changing or dropping a requirement, changing a
wire or shared-memory layout nobody asked for, or widening a bound.

## 2. Plan and dispatch (main session)

Dispatch as much independent work in parallel as possible: `scout`s (one per
package or question) before opening unknown code, then every independent
`task` in one message. The main session itself does only trivial fixes in a
file it already has open.

Slices run in parallel when their file sets are disjoint (BUILD, `.fbs` and
docs count), every interface they share (message, topic, C ABI signature,
config key, wire layout) is written verbatim into each brief, and each has a
test that passes without the others. Otherwise it is one slice; a
cross-layer feature (RULES.md 6) splits per layer only with its interface
fixed. A brief gives: the goal and its REQ ID; the class (a defect's failing
scenario); the exact files; the fixed interfaces; the test tier and the one
narrow target the agent may run; the docs that state the changed fact; what
to report instead of fixing.

Effort follows the cost of being wrong, not the diff size: `"lower"` for a
fully specified slice, `"higher"` for a safety path (actuation gating,
`Target`, mode generations, owner timeout, wire formats, shared-memory
layout, the launcher's kill and restart decisions). An open root cause or a
design with alternatives goes to `slow` first.

## 3. Implement

- **Defect or flake:** see the new test fail for the reported reason, then
  fix ([TESTING.md §5](TESTING.md#5-regression-tests)).
- **Feature:** contract first
  ([DESIGN.md §4](DESIGN.md#4-contract-first-one-implementation)), then the
  test from the requirement, then the code. Node behaviour gets the
  [node test matrix](TESTING.md#7-the-node-test-matrix).
- **Docs in the same slice:** grep the changed number, flag or name.
- **Something else wrong:** report it, do not fix it (RULES.md 14); the main
  session adds a TODO.md entry in the form its header gives.

## 4. Review and commit

Run `reviewer` on every non-trivial diff before committing, with the diff
range, the REQ ID and the class, not the author's summary. Fix each finding
or state why it does not apply.

Commit format: [AGENTS.md "Commit log"](../AGENTS.md#commit-log-the-projects-reasoning-record).
`Why:` names the REQ ID; `Tests:` gives each test's behaviour, a defect's
red evidence, and the rung and OS reached; `Deferred:` lists the TODO.md IDs
added. A requirement change, a rule change and a bound change each get their
own commit.

## 5. Verification ladder

Climb as far as the change reaches; never skip a rung below it.

| Rung | Who | What | When |
|---|---|---|---|
| 0 | `task` | the narrow target its brief names | while implementing |
| 1 | main session | `bazel test //touched/package:all` | after integrating each slice |
| 2 | main session | the affected tests (below), and `--describe-only` on each affected config | before each commit |
| 3 | main session | RULES.md 11 in full: `tools/prepush.sh` (`--full` for more than 20 commits) | before each push, from a clean worktree of the commits to push |
| 4 | CI | the workflows on the pushed commit ([TESTING.md §9](TESTING.md#9-lanes-and-ci)) | on push |
| 5 | bench or robot | the [BENCH.md](BENCH.md) or [BRINGUP.md](BRINGUP.md) check a TODO.md item names, its result recorded in [BENCH_RESULTS.md](BENCH_RESULTS.md) | for a requirement verified by measurement |

Rung 2's affected tests:

```sh
t=$(bazel query --keep_going 'tests(rdeps(//..., set(//touched/package/...))) except attr(tags, manual, //...)' 2>/dev/null)
[ -n "$t" ] && bazel test $t
```

`--keep_going` is needed because some external packages cannot load on every
host; the query then exits 3 with a usable partial result. A change with no
affected tests (docs only) skips the test step: rung 2 is then
`--describe-only` if a config changed and `python3 tools/check_links.py`
(every relative link and anchor resolves).

Rung 3 is `tools/prepush.sh [--ref <commit>] [--full]`. It checks the commit out in a
new worktree under `${TMPDIR:-/tmp}`, makes its tracked files read-only with
CI's own step, runs CI's source lints (`tools/source_lints.sh`: test time,
links, requirement trace, node test matrix, `bazel_dep` listing,
`layers-src` (`tools/check_layers.py --source-only`),
commissioning) and `tools/format.sh --check`
on the branch's diff against its merge base with `origin/main`, as CI's
clang-format job, then `bazel build //...`, then `tools/check_host_includes.py`
on the `.d` file of every compile in that build, cache hits included (no
compile read a header from the host's include tree but the C library's,
the kernel's or the C++ standard library's), then `bazel test //...` with no
cached test result (build results come from the disk cache
`~/.cache/talos-bazel-disk`), then `tools/tidy.sh` on the same diff (on a
host where clang 18 cannot parse the standard library and no fallback is
available, it prints `tidy NOT RUN`
with the reason and does not fail the run: CI is then the first to run
clang-tidy), runs the gradle checks when the commits to
push (since the checked branch's upstream, else since the merge base with
`origin/main`; every file when neither exists) touch the RoboRIO program or
anything under `talOS/` (its
`build.gradle` takes headers from all of it), runs CI's RoboRIO-in-the-loop
check (`tools/test_rio_in_the_loop.py --wpilib`, about 30 s) when they touch
that tool, `bench/`, the RoboRIO program, anything under `talOS/`, or what
its binaries are built with (`MODULE.bazel`, `third_party/`,
`.bazelversion`, `talos.bazelrc`), checks that all of these left the worktree unchanged, and runs the pnpm
checks when they touch `studio/`. With `--full` it also runs CI's sanitizer
lanes (`tools/sanitizer_lane.sh tsan` and `asan`, the script CI's sanitizer
jobs run; 5 to 10 minutes each cold) and CI's soak with its branch seed,
for 48 s of enabled cycles rather than CI's 180 s (the shortest run that injects
every fault kind), and the RoboRIO-in-the-loop check whatever the commits
touch; after a failed build those steps are SKIPPED. **A push of more
than 20 commits runs `tools/prepush.sh --full`**: without it, such a range
(since the branch's upstream; with none, the commits on no remote-tracking
branch) is reported first as the step `full` SKIPPED, the other steps still
run, and the run exits nonzero. Every robot and example config's
`--describe-only` runs as its `talos_graph_test`, the bundle test's fixture
`bench/main_processor/bundle_testdata` included. When the commits to push
touch `MODULE.bazel`, `REPO.bazel`, `talos.bazelrc`, `.bazelversion`,
`third_party/`, `talOS/`, `studio/`, `tools/`, `examples/robot_repo/`, or
`examples/wrist_repo/`, prepush also runs `dependent-robot_repo` and
`dependent-wrist_repo` (`bazel test //... @talos//talOS/configuration:config_fuzz_test`
in each; CI's `robot-repo` job also sets `--test_env=TALOS_REQUIRE_STUDIO_PAGES=1`) and `scaffold-robot_repo` (`tools/check_robot_repo_scaffold.sh`).
CI additionally runs `buildifier`, the full `tools/check_layers.py` Bazel
query, `tools/check_runfiles_collisions.py`, the generated-TS diff,
`tools/slow_tests.py --check`, and `tools/run_rio_clang_tidy.py`; among the
steps prepush runs (besides `tidy`), it differs from CI in three places
where the host decides, and says which applies: where the host refuses an unprivileged network namespace it
leaves `TALOS_REQUIRE_NETNS=1` unset, so
`//talOS/protocol:udp_address_change_test` may skip where CI fails it;
where it finds no browser
([the test's search](../studio/README.md#driver-dashboard-in-a-browser))
it leaves `TALOS_REQUIRE_BROWSER=1` unset, so `//studio:drive_browser_test`
skips where CI fails it; and Studio's checks reuse the checkout's installed `studio/node_modules`
when its `studio/node_modules/.pnpm/lock.yaml` matches the worktree's
`studio/pnpm-lock.yaml` (otherwise running an offline `pnpm install`, and
reporting studio SKIPPED if that fails). It prints one
line per step and exits nonzero if any failed; the script's header lists
its flags and steps. With `TALOS_ROBOT_REPOSITORY` set to the checkout of
a robot repository that consumes talOS, it also runs
`tools/check_robot_repository.sh` on it: `bazel build //...` and `bazel
test //...` in a copy of that repository's committed tree, with `@talos`
overridden to the commit being pushed (the repository itself is only read).
Unset, the step says skipped and does not fail the run; set to anything
that is not a directory, an empty value included (a misspelt
`TALOS_ROBOT_REPOSITORY="$ROBOT_DIR"`), it names no robot repository, and
the step FAILED. So does a repository whose `MODULE.bazel` has no
`bazel_dep(name = "talos")`: nothing there would be checked against talOS.

A local macOS run does not exercise the Linux paths (`epoll`, glibc
allocation counting, `clang-tidy`): say so in `Tests:`; such a change is
verified at rung 4. A timing result from a busy or sleeping host is not
evidence. A failing check is reported with its output.

## 6. Push and done

Push when rung 3 is green, to a branch CI runs on
([TESTING.md §9](TESTING.md#9-lanes-and-ci)). If the last pushed commit's CI
is red, the next commit fixes or reverts it. Rerun only a host failure the
log shows.

Done means all of these can be shown: the REQ ID and, for a defect, the
red-then-green test; the highest rung reached, with command, result and OS;
`reviewer` findings addressed; every doc stating a changed fact updated in
the same commit; out-of-scope findings in TODO.md; green CI on the pushed
commit, or the work reported as "pushed, CI pending".

## 7. Drift pass (main session)

Every 25 commits or weekly, record as TODO.md entries and fixes: targets
that failed twice in the last 20 CI runs without a product change; expired
`quarantine` tags; new tests against
[TESTING.md §4](TESTING.md#4-what-makes-a-test-meaningful); a sample of 10
requirements in [REQUIREMENTS.md](REQUIREMENTS.md) audited against their
tagged tests to confirm each test proves the requirement's sentence (not
merely that a tag exists); doc numbers, flags and names against code (a
`scout` per doc); `[review]` marks that could be `[enforced]` (the global
whole-codebase clauses of `REQ-SAFE-14` and `REQ-SAFE-45` are accepted as
review-only); TODO.md items now passing or 60 days without evidence.
