# talos agent guide

FRC robot stack: C++/Python nodes on a main processor over shared-memory
topics; the RoboRIO is only the hardware layer, reached over UDP.

## Architecture and names (authoritative)

@ARCHITECTURE.md

@talOS/NAMING.md

API reference: `docs/NODE_DEVELOPMENT_GUIDE.md`. Method (units, node anatomy,
test matrix, open source first): `docs/METHODOLOGY.md`. Mechanism walkthrough:
`examples/superstructure/README.md`.

## Build, test, lint

Dev environment: `tools/setup.sh` (Linux/macOS; on Windows, inside WSL2); see docs/INSTALL.md.

```sh
bazel build //...
bazel test //...                      # CI runs both with --test_output=errors
bazel test //examples/superstructure/...
bazel test //path/to/package:all      # narrow loop while iterating
bazel run //talOS/launcher -- --config $PWD/2026-robot/main_processor/configuration/robot.toml --describe-only
```

- Packet/slot size changes: follow RULES.md 9 (`docs/DATA_FLOW.md` and the size assert).
- Graph/naming changes (topics, nodes, configs): run `--describe-only` on every
  affected config; the launcher refuses a graph that breaks a rule.
- RoboRIO program (`2026-robot/controller_processor/rio`) builds with GradleRIO,
  not Bazel: `./gradlew phoenixTest` and `./gradlew buildRio` from that directory.
- Studio (`studio/`): `pnpm test`, `pnpm build` (runs `tsc --noEmit`);
  regenerate TS bindings with `pnpm generate` after editing `talOS/telemetry/telemetry.fbs`.
- CI also checks changed files (a pull request: since its merge base with
  `main`; a push: since the previous push, or since the merge base with `main`
  for a new branch or force push) with `clang-format-18 --dry-run --Werror`,
  `clang-tidy-18` (repo `.clang-format` / `.clang-tidy`; tests included; a
  changed header is analyzed through sources that include it), and
  `buildifier --mode=check --lint=warn` on changed BUILD, `.bzl` and
  `MODULE.bazel` files. Match them.

## Run in simulation

```sh
./sim.sh --sim-gateway --no-studio   # lightweight simulator, no WPILib
./sim.sh                             # WPILib RoboRIO sim + Studio
```

## Agent roles

Claude Code subagents are defined in `.claude/agents/`; each file sets its
model, effort and tools. The main session runs on Opus at high effort.

|Agent|Model, effort|Use for|
|---|---|---|
|main session|Opus, high|plan slices, dispatch, integrate, run checks, commit|
|`task`|Opus, medium|one implementation slice: code + its test + its docs|
|`scout`|Sonnet, low|read-only mapping of unknown code|
|`sonic`|Haiku, low|mechanical edits, renames, data collection|
|`slow`|Opus, xhigh|hard design or root-cause analysis; no source edits|
|`reviewer`|Opus, high|final review pass before commit|

Plan mode uses the built-in `Plan` agent; a security pass is `/security-review`.

Dispatch by default; the main session coordinates instead of doing every step itself:

- Unknown code: spawn `scout`s in parallel (one per package or question)
  before opening files.
- Multi-file or multi-package work: split into independent slices with their
  file sets and shared interfaces (messages, topics, C ABI signatures) fixed in
  each brief, and spawn all `task`s in one message. A node-API feature or
  device type (RULES.md 6) is one slice per layer only when the interface is
  fixed first; otherwise one `task` owns it.
- Effort: pass `effort: "lower"` for mechanical or fully specified slices and
  `"higher"` for safety paths (actuation gating, `Target`, mode generations,
  wire formats, shared-memory layout) or an open root cause.
- Subagents do not run `bazel build`/`bazel test`; the main session runs the
  RULES.md 11 checks once after integrating.
- Before committing a non-trivial change, run `reviewer` on the diff.

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
fix, verified with the checks above before committing. Do not squash
unrelated fixes together; do not rewrite published history.
