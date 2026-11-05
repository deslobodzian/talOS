# Automatic wiring: implementation handoff

D1 through D6 are implemented and verified in simulation on
`agent/automatic-wiring`. This is not a robot release: see "Not verified
anywhere" below and the robot-readiness backlog at the end.

Read this file first, then [METHODOLOGY.md](METHODOLOGY.md) and
[DESIGN_AUTOMATIC_WIRING.md](DESIGN_AUTOMATIC_WIRING.md). This handoff records
actual completion status where the design describes intended behavior.

## Status at the 2026-11-04 evening stop (read this before the sections below)

`agent/automatic-wiring` at `75356df` is verified:

- `bazel build //...` and 83/83 tests pass.
- `--describe-only` reports no errors on all three configs.
- `sync_manifest --check` passes on all four configs.
- `./sim.sh --sim-gateway --no-studio --no-open --duration-s 15` runs 9
  processes; every one exits 0 with no restarts.
- Changed C++ passes clang-format (21, not CI's 18).

| Item | State |
|---|---|
| D1 | Done. Mode generation in DS packets and the 16-byte envelope header; regressions in C++ and Python; selected source in registry v2 and Studio. Live sim fault table held; stick to drive request median 2.99 ms (old 4.98 ms). |
| D2 | Done, docs pass reviewed claim by claim (`e76e357`). |
| D3 | Done. `//talOS/telemetry:node` observes every `state`/`status` sender in the declared graph (observer endpoints the lint ignores). Channels are `<owner>/<role>/<field>`, units come from field suffixes, enum labels from reflection, stale values are NaN. `[studio] topic` (default `/talos/telemetry`) and `pose_topic` (missing topic or `geometry.Pose2d` is a construction error). The robot telemetry package is deleted. Gate: a scaffolded Python `wrist` appeared on the live Studio Signals page as `wrist/state/velocity_rps` [rot/s], `enabled`, `connected` with no edits outside its package. |
| D4 | Done. `//talOS/teleop:node` reads `[[teleop.bindings]]` (parsed once in `config_parser.h`, mirrored by `sync_manifest.py`, shared fixtures), resolves fields through the lane reader's schema in the declared graph, and writes `/<sub>/target/teleop`. The first live binding wins per field; it releases when nothing is live and goes silent while disabled, out of teleop, stale or disconnected. `operator_interface` became `driver` (chassis only, `deadband_frac`); shooter/intake buttons live in their own TOMLs. Gate: a throwaway `wrist` with one TOML binding, live sim plus `send_joystick --buttons 4` for 5 s: 250 `active: true, velocity_rps: 20` lane sends, 248 `WristState {enabled: true, velocity_rps: 20}`, released before and after. |
| D5 | Done. Shared accept/reject fixtures for parser and `sync_manifest`; effective-config scaffold checks; bundle identity over config-relative paths; relocated-bundle test; scaffold goldens; `@wrist//:node` second repo proven in a throwaway. |
| D6 | Done, including `--replay-inputs`. The hermetic regression `pose_estimator:node_test` replays a generated session twice per backend (kalman and factor_graph) and checks: identical record streams, delayed vision accepted, and stale and invalid observations rejected. Logs are read through a C++ `LogReader` tool, not Python. |

Decisions taken while finishing:

- The declared-graph API (`NodeAccess::declared_graph`, `add_observer`,
  `robot_config`) is framework-only C++. It has no C ABI or Python
  counterpart; only the telemetry and teleop nodes use it.
- Teleop releases its lane once when the DS is disabled, out of teleop,
  stale or disconnected, then sends nothing until conditions return.

Open, not blocking:

- Lint warnings for `target/auto` lanes that nothing writes yet.
- Unit-suffix warnings on the superstructure example's `intake` params.
- Telemetry logs schema-less topics (`/hw/state`) to stderr once at startup.

Not verified anywhere: Linux build (wpimath Linux artifacts), `phoenixTest`
(GradleRIO wants the missing `MacOSX26.sdk`; `buildRio` compiles), CI's
clang-format-18/clang-tidy-18/buildifier, the robot.

The sections below record the earlier checkpoint's plan and evidence; the
table above supersedes their status claims.


## User requirements

- Remove the hard-coded arbiter in favor of automatic, configuration-driven
  target selection, following the node model.
- Adding a subsystem or a hardware-free process must not require edits in
  another language or hard-coded lists in other nodes.
- A pose-estimator process must be straightforward to add.
- Consistent units, concrete API conventions, C++/C ABI/Python parity,
  reliability, bounded latency, and useful failure diagnostics.
- Prefer maintained open-source libraries. Use FlatBuffers reflection for
  schemas and WPILib/RobotPy for estimation; do not build new equivalents.

## What is implemented

| Item | Current state |
|---|---|
| D5: manifest single source | Substantially implemented: `tools/sync_manifest.py`, generated `<stem>_manifest.bzl` files, BUILD consumers, stale-manifest tests, scaffold integration, and bundle-relative config staging. Hardening and integrated verification remain. |
| D1: consumer-side target selection | Substantially implemented: `talOS/devices/target.{h,cc}`, C ABI version 12, Python wrappers, policy parsing, lane lint, test helpers, and robot/example/scaffold migration. Arbiter package and config entry are deleted. Safety-contract, observability, and acceptance work remain. |
| D2: reflection/transport/logging | Not implemented. Current type checks remain layout-based; Python uses layout inference. |
| D3: descriptor-driven telemetry | Not implemented. Robot-specific telemetry and `GenericState` remain. |
| D4: declarative teleop | Not implemented. Hard-coded mechanism bindings and one `GenericTarget` remain. |
| D6: estimator support | Not implemented. Dependency, sample-time, measurement-schema, array-param, replay-input, and scaffold work remain. |

The preceding design/methodology commit is `f7ae33d`. D5 work and interrupted
D1 work are included in this checkpoint. The D1 worker stopped on a provider
rate limit, not after reporting acceptance completion. Implementation included
non-Muse worker execution; the current-state assessment used Codex and scouts.

## Verification evidence and limits

The current-state assessment ran this successfully:

```sh
bazel run //tools:sync_manifest -- --check \
  2026-robot/main_processor/configuration/robot.toml \
  examples/superstructure/config/robot.toml \
  examples/superstructure/config/robot_py.toml
```

All three manifests are current and contain no stale arbiter entries.
Use Bazel's Python toolchain: direct host `python3 tools/sync_manifest.py`
failed because that interpreter lacks Python 3.11's `tomllib`.

Prior worker reports and existing test artifacts show focused successes for
manifest generation, scaffolding, bundle building/relocation, selectors,
C ABI, Python, and graphs. They do **not** prove that the complete final tree
passed together. An earlier full-suite run failed
`//talOS/introspection:names_test` in
`LintGraph.ExternalAndOptionalEndsAreNotFaults`; a later focused XML records
24 passing tests. Neither result substitutes for final integration evidence.
Inspect the failure context before the next integration run; do not rerun a
reported failure merely to confirm it.

No post-migration live latency/fault acceptance was established. The old
60-test green baseline and old live measurements precede this migration.

## Completion order

### 1. Close D1's safety contract and integration

The most important issue is mode-transition ordering:

- `talOS/devices/target.cc:144-156` compares lane receipt time against the DS
  mode's receipt time.
- All messages delivered during one drain share an observation timestamp.
- `talOS/devices/target_test.cc:167-179` intentionally accepts a target in the
  same drain as the mode transition.
- A pre-transition target queued until that drain is not distinguishable from
  a target derived from the new DS packet.

Define and enforce the intended invariant: a command from before the current
enable/mode generation cannot actuate. Use an explicit generation/ordering
mechanism or a documented conservative handoff barrier. Do not claim that
receipt-time equality proves producer ordering, and do not just change `<`
to `<=`: that would break the deliberately accepted same-drain case.

Then:

1. Add regressions for queued pre-transition targets, DS loss and same-mode
   recovery, stale high-priority fallback, and target-specific remapping.
2. Complete selected-source reporting through selector, reporter, registry,
   reader, and Studio. `Target::source()` exists; registry fields do not.
3. Check C++/C ABI/Python behavioral parity, not merely symbol presence.
4. Finish migrating callers and docs. The operator-interface `GenericTarget`
   is deliberately deferred until D4, not already removed.
5. Exercise live disable, producer pause, consumer pause, and gateway loss;
   measure stick-to-request latency against the old 4.98 ms median baseline.

### 2. Harden D5 and establish a passing checkpoint

1. Align generator validation with `config_parser.h`: duplicate subsystem
   names, unknown manifest keys, malformed explicit values, and
   include/default/node-override precedence. Use shared behavioral fixtures
   rather than maintaining two untested interpretations.
2. `tools/new_subsystem.py` currently discovers existing names and placeholder
   CAN usage only from immediate files. Follow effective included/defaulted
   and custom-path inputs so the scaffold cannot accidentally override an
   inherited subsystem. Never change physical CAN ids or commission a config.
3. Define whether moving only the root config changes bundle identity;
   current hashing includes dependency paths but omits the root path.
4. Add a relocated-bundle case using real includes, parent-directory defaults,
   custom paths, and node overrides, and parse/describe it after relocation.
5. Build and run freshly scaffolded C++ and Python packages. Template-string
   checks are not proof that generated projects work. Revisit the previously
   observed standalone toolchain/repository-label issues too.

Focused targets include:

```sh
bazel test --test_output=errors \
  //talOS/devices:target_test \
  //talOS/configuration:test \
  //talOS/introspection:names_test \
  //talOS/node_api:node_api_test \
  //talOS/python:talos_test \
  //tools:sync_manifest_test \
  //tools:new_subsystem_test \
  //talOS/build:bundle_builder_test \
  //2026-robot/main_processor:bundle_test
```

Follow fixes with the full build/test, all three graph descriptions, relocated
bundle acceptance, and live smoke. Make a reviewed passing checkpoint before
starting the D2 transport changes.

### 3. D2: deliver reflection in bounded stages

1. Upstream FlatBuffers `.bfbs` generation and C++/Python descriptors.
2. Descriptor identity in registration, describe output, and graph checks.
3. Schema-identity attach checks, including D1's framework envelope layout.
4. Session-owned shared-memory lifetime and dead-reader recovery; preserve
   standalone tools, direct node execution, Sim, and replay.
5. Descriptor-bearing logs and decoded `log_dump` payloads.
6. Reflection-only Python codec; delete the recorder-based inference path.

Important build detail: FlatBuffers 25.12.19 `--bfbs-gen-embed` requires a table
root and cannot directly embed talos struct roots. Generate `.bfbs` normally
and integrate the upstream reflection schema. The pip wheel lacks reflection
classes; generate them from upstream `reflection.fbs`.

Gate: same-size incompatible types fail; process restarts do not sever topics;
nested struct/array/enum/alignment round trips agree in both languages.

### 4. D3/D4: remove remaining hard-coded wiring

D3 discovers observations from graph descriptors, derives units and labels,
respects bandwidth limits, and removes robot telemetry plus `GenericState`.
D4 binds buttons/axes to typed target fields in config, defines conflicting
binding precedence and release semantics, and removes `GenericTarget` and
legacy binding keys. Chassis-specific transforms remain normal robot code.

Gate: add a mechanism through its own package/config/manifest only, then prove
its controls and correctly labelled telemetry on the actual Studio surface.

### 5. D6: prove the estimator workflow

Dependency feasibility may run alongside D2: verify pinned WPILib/RobotPy
packaging on macOS arm64 and supported Linux targets before building around
it. The C++ class and Python wrapper names differ; use real versioned APIs.

Then:

1. Publish wheel-module distances/angles and heading validity.
2. Migrate field/chassis angles to radians and retain shaft rotations, with
   named boundary conversions and every consumer updated together.
3. Expose actual sample times through C++, C ABI, and Python. Define clock
   mapping, uncertainty, gateway reboot, and out-of-order behavior; a
   min-filtered one-way offset is not exact clock synchronization.
4. Add shared observation/estimate schemas and array parameters.
5. Implement cross-session input replay, not just same-program replay.
6. Add a hardware-free estimator scaffold and a working WPILib-backed example.

Gate: add a new estimator by node package/config/manifest; delayed vision
corrects it, invalid/stale measurements are rejected, and recorded-input
replay is reproducible. Estimation and observations continue while disabled;
actuator gating is not a blanket gate on computation.

## Final acceptance and release boundaries

For each completed coherent stage:

```sh
bazel build //...
bazel test --test_output=errors //...
bazel run //talOS/launcher -- --config "$PWD/2026-robot/main_processor/configuration/robot.toml" --describe-only
bazel run //talOS/launcher -- --config "$PWD/examples/superstructure/config/robot.toml" --describe-only
bazel run //talOS/launcher -- --config "$PWD/examples/superstructure/config/robot_py.toml" --describe-only
./sim.sh --sim-gateway --no-studio --no-open --duration-s 15
```

Run changed-file format/lint checks; Studio changes require its build and
actual-surface verification. RIO changes require `phoenixTest` and `buildRio`.
Do not equate tests with live acceptance or simulation with hardware proof.

Keep the separate robot-readiness backlog visible: gateway fault recovery,
DIO/PWM gating, loopback config identity mismatch, macOS boot startup, CAN
utilization, physical addressing, gains, and human commissioning. Automatic
wiring does not establish readiness to drive a real robot.
