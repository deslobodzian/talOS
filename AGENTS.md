# talos agent guide

FRC robot stack: C++/Python nodes on a main processor over shared-memory
topics; the RoboRIO is only the hardware layer, reached over UDP.

## Architecture and names (authoritative)

@ARCHITECTURE.md

@talOS/NAMING.md

API reference: `docs/NODE_DEVELOPMENT_GUIDE.md`. Mechanism walkthrough:
`examples/superstructure/README.md`.

## Build, test, lint

Dev environment: `tools/setup.sh` (Linux/macOS) or `tools/setup.ps1` (Windows → WSL2); see docs/INSTALL.md.

```sh
bazel build //...
bazel test //...                      # CI runs both with --test_output=errors
bazel test //examples/superstructure/...
bazel test //path/to/package:all      # narrow loop while iterating
bazel run //talOS/launcher -- --config $PWD/2026-robot/main_processor/configuration/robot.toml --describe-only
```

- Graph/naming changes (topics, nodes, configs): run `--describe-only` on every
  affected config; the launcher refuses a graph that breaks a rule.
- RoboRIO program (`2026-robot/controller_processor/rio`) builds with GradleRIO,
  not Bazel: `./gradlew phoenixTest` and `./gradlew buildRio` from that directory.
- Studio (`studio/`): `pnpm test`, `pnpm build` (runs `tsc --noEmit`);
  regenerate TS bindings with `pnpm generate` after editing `talOS/telemetry/telemetry.fbs`.
- CI also checks changed files with `clang-format-18 --dry-run --Werror`,
  `clang-tidy-18` (repo `.clang-format` / `.clang-tidy`), and
  `buildifier --mode=check --lint=warn` on changed BUILD files. Match them.

## Run in simulation

```sh
./sim.sh --sim-gateway --no-studio   # lightweight simulator, no WPILib
./sim.sh                             # WPILib RoboRIO sim + Studio
```

## Agent roles

- Muse (`muse-code/muse-spark-1.3-contributor`) is the main session model and
  implements: it plans slices, writes the code, runs the checks. Generic `task`
  subagents also run on Muse (`task.agentModelOverrides`).
- Claude Architect (Opus) and Gemini Product (Gemini) are advisors on the main
  session (`~/.omp/agent/WATCHDOG.yml`): design, architecture, features and code
  review. They advise; they do not implement. Act on their notes or say why not.
- Repo-specific review priorities are in `WATCHDOG.md`; the `reviewer` agent
  uses them for a final pass.
- The user config's retry chain can move a Muse session or worker to Sonnet;
  `resolvedModelIsFallback` in the session log shows it. Report work that did
  not run on Muse.
- Muse session worktrees live under `.muse/` (ignored by `.bazelignore`).

## Implementation rules

Hard rules live in `.omp/RULES.md` (also delivered to subagents). Place new code per
"Where a new thing goes" above; do not invent a new layer or package convention.
