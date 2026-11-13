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
| 3 | main session | RULES.md 11 in full: `tools/prepush.sh` | before each push, from a clean worktree of the commits to push |
| 4 | CI | the workflows on the pushed commit ([TESTING.md §9](TESTING.md#9-lanes-and-ci)) | on push |
| 5 | bench or robot | the [BENCH.md](BENCH.md) or [BRINGUP.md](BRINGUP.md) check a TODO.md item names | for a requirement verified by measurement |

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

Rung 3 is `tools/prepush.sh [--ref <commit>]`. It checks the commit out in a
new worktree under `${TMPDIR:-/tmp}`, makes its tracked files read-only with
CI's own step, runs `bazel build //...`, then `tools/check_host_includes.py`
on the `.d` file of every compile in that build, cache hits included (no
compile read a header from the host's include tree but the C library's,
the kernel's or the C++ standard library's), then `bazel test //...` with no
cached test result (build results come from the disk cache
`~/.cache/talos-bazel-disk`), checks that they left the worktree unchanged,
and runs the gradle and pnpm checks when the commits to push touch the
RoboRIO program or anything under `talOS/` (its `build.gradle` takes
headers from all of it) or `studio/`; every
config's `--describe-only` runs as its `talos_graph_test`. It prints one
line per step and exits nonzero if any failed; the script's header lists
its flags and steps.

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
[TESTING.md §4](TESTING.md#4-what-makes-a-test-meaningful); doc numbers,
flags and names against code (a `scout` per doc); `[review]` marks that
could be `[enforced]`; TODO.md items now passing or 60 days without evidence.
