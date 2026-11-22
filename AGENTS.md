# talos agent guide

FRC robot stack: C++/Python nodes on a main processor over shared-memory
topics; the RoboRIO is only the hardware layer, reached over UDP.

## Architecture and names (authoritative)

@ARCHITECTURE.md

@talOS/NAMING.md

API reference: `docs/NODE_DEVELOPMENT_GUIDE.md`. Node conventions (units,
node anatomy, open source first): `docs/METHODOLOGY.md`. Testing (tiers, node
test matrix, flakes, CI lanes): `docs/TESTING.md`. Requirements:
`docs/REQUIREMENTS.md`. Design notes and fitness functions: `docs/DESIGN.md`.
Mechanism walkthrough: `examples/superstructure/README.md`.

## Build, test, lint

Dev environment: `tools/setup.sh` (Linux/macOS; on Windows, inside WSL2); see docs/INSTALL.md.

```sh
bazel test --config=fast //...       # everyday loop (-O3): skips heavy, integration, e2e, perf lanes
bazel test --config=dev //...        # faster C++ edit-compile-test loop (-O1 + fast lane); switches analysis cache vs default -O3 / tools/tidy.sh
bazel build //...
bazel test //...                      # CI runs build and test with --test_output=errors
bazel test //examples/superstructure/...
bazel test //path/to/package:all      # narrow loop while iterating
bazel run //talOS/launcher -- --config $PWD/bench/main_processor/configuration/robot.toml --describe-only
```

- Packet/slot size changes: follow RULES.md 9 (`docs/DATA_FLOW.md` and the size assert).
- Graph/naming changes (topics, nodes, configs): run `--describe-only` on every
  affected config; the launcher refuses a graph that breaks a rule.
- RoboRIO program (`controller_processor/rio`) builds with GradleRIO,
  not Bazel: `./gradlew phoenixTest` and `./gradlew buildRio` from that directory.
- Studio (`studio/`): `pnpm test`, `pnpm build` (runs `tsc --noEmit`);
  regenerate TS bindings with `pnpm generate` after editing `talOS/telemetry/telemetry.fbs`.
- CI also checks changed files with `clang-format-18 --dry-run --Werror`,
  `clang-tidy-18` (repo `.clang-format` / `.clang-tidy`; tests included; a
  changed header is analyzed through sources that include it), and
  `buildifier --mode=check --lint=warn` on changed BUILD, `.bzl` and
  `MODULE.bazel` files. Match them (`tools/format.sh`, `tools/tidy.sh` run the
  CI versions locally). What CI runs where:
  `docs/TESTING.md` §9.

## Run in simulation

```sh
./sim.sh --sim-gateway --no-studio   # the bench robot on the lightweight simulator, no WPILib
./sim.sh                             # the bench robot on the WPILib RoboRIO sim + Studio
```

A robot in its own repository (the 2026 robot) runs there: `bazel run //:sim`.

## Agent roles

Subagents are defined for both Antigravity (`.agents/agents/`) and Claude Code
(`.claude/agents/`) with identical role prompts and tool-specific frontmatter
(keep both files in sync when editing a role). The main session runs on a
frontier model at high reasoning effort.

|Agent|Tier (`agy` / Claude)|Access|Use for|
|---|---|---|---|
|main session|frontier, high|full|plan slices, dispatch, integrate, run checks, commit|
|`task`|`pro` / `opus` medium|read/write, no nesting|one implementation slice: code + its test + its docs|
|`scout`|`flash` / `sonnet` low|read-only, no nesting|read-only mapping of unknown code|
|`sonic`|`flash` / `haiku` low|read/write, no nesting|mechanical edits, renames, data collection|
|`slow`|`pro` / `opus` xhigh|read-only + repro runs|hard design or root-cause analysis; no source edits|
|`reviewer`|`pro` / `opus` high|read-only, no nesting|final review pass before commit|

Planning uses built-in plan mode (`/plan` in Antigravity, `Plan` in Claude
Code); a security pass uses `/security-review` where available or a
security-focused `reviewer` pass. Shared project skills live in
`.agents/skills/<name>/SKILL.md` (symlink `.claude/skills/` if needed for
Claude Code).

Rule loading: Claude Code imports this file via `CLAUDE.md` and expands its `@`
references inline; Antigravity loads `AGENTS.md` directly, inlines `RULES.md`,
`docs/DEVELOPMENT.md`, and `talOS/NAMING.md` via `.agents/rules/*.md` (each
under its 24 KB per-file limit), and loads `ARCHITECTURE.md` on demand.

## Development workflow

@docs/DEVELOPMENT.md

## Verification practice

Adds to [docs/DEVELOPMENT.md §5](docs/DEVELOPMENT.md#5-verification-ladder)
and [docs/TESTING.md](docs/TESTING.md); where they or RULES.md state a
rule, they win.

1. **Run the change, not only its tests.** Before a change to what a
   running node, the launcher or the bridge does is called done, the main
   session runs it end to end (`./sim.sh --sim-gateway --no-studio`, or the
   launcher on the affected config) and reads what it did: console,
   `/hw/status`, the session's logs. A unit test can pass while the launch
   is broken. Subagents run only the target their brief names.
2. **Pick the test that is hard to fool.** The tier is the lowest that can
   observe the behaviour (docs/TESTING.md §2). Within it, prefer a property
   test for anything with an inverse or an invariant (a frame or payload
   codec's encode and decode, a parser, an operation that must be
   idempotent), and a differential test when behaviour must stay the same:
   old against new implementation, C++ against its Python wrapper, the sim
   gateway against the RoboRIO gateway, a replay against its live log, on
   generated inputs, with the intended differences listed.
3. **Three rounds, then a person.** A check still failing after three fix
   attempts is reported with its output and what was tried, not given a
   fourth variation. Unattended, the main session records it in TODO.md and
   continues other work; red CI on a pushed commit is still fixed or
   reverted by the next commit (docs/DEVELOPMENT.md §6).
4. **A change cannot loosen what judges it.** An edit that makes a judge
   weaker (a widened allowlist or baseline, a removed or weakened
   assertion, a disabled lint or tidy check, a relaxed `reviewer` agent in
   `.agents/agents/reviewer.md` or `.claude/agents/reviewer.md`) goes in its
   own commit with the reason in `Why:`, and the rest of the change is
   judged by the base branch's version. Edits that tighten or track (a
   lowered allowlist count, an assertion the requirement's new behaviour
   forces) stay in the change.
5. **A result belongs to one commit.** A review, suite, Mac pass or prepush
   holds for the commit it ran on and nothing later. After a fold, rebase
   or follow-up, run again what the change reaches, and prepush on the head
   being pushed; a report names the hash each result is for. A commit
   nothing was run on says `Tests: none run` and why.
6. **One reviewer, rounds that build.** Improve the reviewer's brief before
   adding a reviewer. A later round gets the earlier findings and the
   follow-ups, confirms each, and attacks what changed; it does not start
   over. A second reviewer is for safety paths only.
7. **The review brief names what the diff triggers.** Add the lines whose
   trigger the diff matches:
   - a lint or check tool: try to get a violating change past it (merge
     commits, attributes, quoted or renamed paths); it must fail closed;
   - a commit folded or reworded: its message is true to the diff it now
     holds;
   - a doc or requirement saying "never", "every" or "no": name the case
     that breaks it, or its conditions;
   - a test said to close a proof gap: name the wrong implementation the
     old test passed, and mutate to check both;
   - a lint, graph or naming change: `//bench/...` and `//examples/...`
     run as well;
   - a cherry-picked slice: `python3 tools/lint_test_time.py` (its
     allowlist holds exact counts).
8. **Turn a miss into a check, and prune.** When review, CI or the bench
   finds what an earlier check should have caught, the same group adds the
   smallest check that catches it: a test, a lint, a trigger line above.
   The drift pass
   ([docs/DEVELOPMENT.md §7](docs/DEVELOPMENT.md#7-drift-pass-main-session))
   deletes trigger lines that no longer catch anything. RULES.md changes
   only by the owner's decision.
9. **A new gate starts by observing.** A new lint first reports (a warning,
   or a baseline of its existing hits) and fails the build once its false
   positives have been read. A check guarding RULES.md 1 to 4 refuses from
   the start.

## Implementation rules

@RULES.md

## Commit log (the project's reasoning record)

`git log` is where later agents look for why the code is the way it is, so
one commit is one logical fix, and its message carries the reasoning:

```
<area>[, <area>]: <imperative summary, <= 72 chars>

Problem: what was wrong, with the concrete failure scenario.
Fix: what changed (files/symbols), and the invariant it now holds.
Why: the design reason; alternatives rejected and why.
Tests: what each new/changed test proves (fails before, passes after).
Docs: docs updated in this commit (size/frame changes: DATA_FLOW.md).
Deferred: known follow-ups, also recorded in TODO.md.
```

Work lands on a topic branch (`hardening/...`, `feature/...`), one commit per
fix, verified per RULES.md 11 (the ladder in `docs/DEVELOPMENT.md` §5). Do not squash
unrelated fixes together; do not rewrite published history.
