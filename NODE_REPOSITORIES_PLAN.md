# Independent node repositories with minimal authoring boilerplate

Status: Phases 0, 1, 2, 3, 4, 5, 6, and 7 complete (hybrid policy: platform
nodes standalone, mechanisms year-local; see record below).

### Continuation record — 2026-09-11 (Hybrid Node Policy)

- Only stable platform nodes stay standalone: `talos-driver_station`,
  `talos-operator_interface`, `talos-odometry`, `talos-telemetry`, plus
  `studio-schema`. Year-specific mechanisms (drivetrain, intake, shooter,
  arbiter) live in `2026-robot/main_processor/` and are rewritten each
  season; a new season clones the year folder (pins, configs, assembly).
- Unwound `talos-drivetrain`, `talos-intake`, `talos-shooter`,
  `talos-arbiter` (deleted) and the superseded in-tree platform dirs
  (`2026-robot/main_processor/{driver_station,operator_interface,odometry,
  telemetry}`), after repointing arbiter's platform deps to
  `@talos_driver_station//:contract`,
  `@talos_driver_station//:driver_station`, and
  `@talos_operator_interface//:operator_interface`.
- Platform modules read year mechanism frames through the year's public
  contract targets (`@talos//2026-robot/main_processor/<mech>:messages_cc`).
  Vendoring schema copies was tried and rejected: duplicate FlatBuffers
  types under one include path fail the build, so there is exactly one
  definition per frame type. In-tree mechanism schema filenames stand
  (e.g. `drive_message.fbs`); platform sources include the in-tree
  generated names.
- Root `MODULE.bazel` pins the platform closure only; the robot bundle
  mixes in-tree mechanism nodes with external platform nodes; `sim.sh`
  builds both; `test/fixtures/robot_assembly` assembles the platform
  modules.
- Removed the unused `aspect_rules_esbuild` 0.27.0 pins (root,
  `studio-schema`, scaffold template): 0.27.0 requires `aspect_rules_js`
  3.4.0, whose npm tag dropped the `root_package` attribute
  flatbuffers@25.12.19 passes, breaking npm-extension evaluation.

### Continuation record — 2026-09-10 (Phase 7 Cutover Applied)

- Root `MODULE.bazel` pins all eight node modules (`talos_drivetrain`,
  `talos_intake`, `talos_driver_station`, `talos_shooter`, `talos_arbiter`,
  `talos_operator_interface`, `talos_odometry`, `talos_telemetry`) plus
  `studio_schema` (telemetry's Studio frame contract) via
  `local_path_override`, following
  `test/fixtures/robot_assembly/MODULE.bazel`. The full transitive closure
  is listed because Bzlmod honors only root-module overrides.
- `//2026-robot/main_processor:robot_bundle` now assembles
  `@talos_*//:node` for all eight nodes (competition `robot.toml` and
  `configuration/subsystems/` files kept); the eight subsystem `node =`
  identities were retargeted to `@talos_*//:node` so the launcher resolves
  bundle binaries via `bundle_manifest.json` overrides.
- Deleted no in-tree fallbacks: `arbiter`, `operator_interface`,
  `odometry`, and `telemetry` still depend on in-tree `:messages_cc`, and
  arbiter tests still use in-tree `:integration_test_support` from
  drivetrain/driver_station. Removal waits for those consumers to migrate.
- Wire-compat policy lives in `docs/NODE_DEVELOPMENT_GUIDE.md`
  ("Provenance Tracking" + "Contract Compatibility"): `LintGraph` rejects
  two message sizes on one topic before launch; same-size evolution is a
  new contract by policy with whole-bundle rollback.

### Continuation record — 2026-09-09 (Phase 7 Complete)

- **Phase 7 (Migrate remaining nodes and establish releases)**:
  - Extracted `talos-driver_station`, `talos-shooter`, `talos-arbiter`,
    `talos-operator_interface`, `talos-odometry`, `talos-telemetry` as
    standalone Bazel modules (`MODULE.bazel` with `local_path_override` to the
    talOS SDK, `BUILD` via `talos_contract_cc`/`talos_cc_node`, contracts,
    fixtures), plus `studio-schema` (Studio frame contract with no
    frontend/bridge/Rust toolchain) and a C++ `contract` on `talos-intake`
    for C++ consumers.
  - Extended `talos_cc_node` with `optional_sources` (arbiter autonomous
    lanes) and `default_config_path` (standalone checkouts never fall back to
    the competition `robot.toml`); telemetry keeps a handwritten `main.cc`
    for its starter/describe/Reporter hooks behind the `talos_node` rule.
  - Pinned and launched all eight nodes together in
    `test/fixtures/robot_assembly` (9/9 describe, graph lints clean, 3s sim
    session records on every node).
  - Provenance: `bundle_manifest.json` records `config_sha256` and per-node
    `contract` labels; the launcher carries both into `manifest.json`
    (`graph.json` records `config_sha256`). Compatibility: `LintGraph`
    rejects two message sizes on one topic before launch (regression test
    pinned); same-size evolution is new-contract-by-policy with whole-bundle
    rollback.
  - Compatibility aliases in `2026-robot/` kept: in-tree consumers still
    resolve in-tree labels, so removal waits for their migration.
  - Updated `docs/NODE_DEVELOPMENT_GUIDE.md`, `README.md`, `ARCHITECTURE.md`,
    `PLAN.md`, and this plan to one authoring and deployment workflow.

### Continuation record — 2026-09-09 (Phase 6 Complete)

- **Phase 6 (Repository-Aware Node Scaffolding and Authoring Documentation)**:
  - Implemented modern repository-aware scaffolding tool in `tools/node_scaffold/` (`main.py`, `scaffold.py`, `templates.py`, `BUILD`).
  - Command line: `python3 tools/node_scaffold/main.py --name <node_name> --lang <cpp|python> [--hardware] --out <output_dir> [--sdk-path <path>] [--force]`.
  - Implemented author protection: safely aborts if destination directory contains files and `--force` is not specified.
  - Implemented complete standalone Bazel module generation:
    - `MODULE.bazel` with `module()`, `bazel_dep()`, and `local_path_override()`.
    - `.bazelrc` with C++23 platform configurations and Bzlmod enablement.
    - Minimal `BUILD` utilizing `@talos//build:defs.bzl` (`talos_contract_cc`, `talos_cc_node`, `flatbuffer_py_library`, `talos_py_node`).
    - Schema (`<name>_message.fbs`) and contract headers (`packet.h`, `packet.py`).
    - Node implementations with `from_context(NodeContext&)` factory.
    - Deterministic tests on `SimulatedEventLoop` verifying input injection, virtual clock stepping, and state updates.
    - Standalone subsystem TOML and local test fixtures (`testdata/robot.toml`, `testdata/<name>.toml`).
    - `README.md` and executable `test.sh`.
  - Maintained backward compatibility in `tools/subsystem_codegen` by adding `tomli` fallback for Python 3.10 environments.
  - Tested scaffolding and execution in fresh temporary directories:
    - Scaffolded C++ hardware node (`arm`), verified compilation, execution, and test pass (`bazel test //...` -> 1/1 PASSED). Verified `--describe` JSON manifest output.
    - Scaffolded Python node (`intake_test`), verified compilation, execution, and test pass (`bazel test //...` -> 1/1 PASSED). Verified `--describe` JSON manifest output.
    - Tested Python hardware node (`intake_hw`), verified 5 ordered sources and test pass.
    - Cleaned up all temporary directories.
  - Authored comprehensive documentation in `docs/NODE_DEVELOPMENT_GUIDE.md` covering standalone node authoring, scaffolding CLI, robot bundle integration with `talos_bundle`, and immutable dependency snapshot workflows (`git_override`, `archive_override`, `bazel vendor`).
  - Verified 12/12 test targets in workspace suite and 2/2 external consumer fixture targets pass cleanly with zero regressions.

### Continuation record — 2026-09-09 (Phase 4 Complete)

- **Phase 4 (Bring Python execution onto native dispatch and replay)**:
  - Extended native C ABI (`talOS/node_api/node_api.h`, `node_api.cc`) to version 6 with `TalosLoop`, `TalosDispatchContext`, `TalosDispatchFn`, and 19 ABI functions spanning realtime, simulation, and replay lifecycle.
  - Implemented polymorphic C++ loop adapters wrapping `talos::event::RealtimeEventLoop`, `talos::event::SimulatedEventLoop`, and `talos::event::ReplayEventLoop`. Reused ground-truth scheduling, virtual clock, `LogWriter`, and `LogReader` without creating any secondary Python scheduler or dispatch-log format.
  - Implemented Python bindings in `talos_runtime.node_api` (`DispatchContext`, `Loop`) with automatic GIL handling via `ctypes.CFUNCTYPE`, callback reference retention, read-only payload buffer slicing, and exception propagation (trapping exceptions at thunk boundary, calling `loop.exit()`, and re-raising in Python).
  - Updated standardized runner (`talos_runtime.runner`) to execute native event loops for live, simulation, and replay. Removed unsupported replay stub and implemented `--replay <path>` verifying clean exit and zero divergence.
  - Migrated `IntakeNode` (`talos_nodes.intake.node`) to register ordered sources natively (`intake` timer, `/hw/state` watcher, `/intake/target` watcher, `/hw/request/intake` sender, `/intake/state` sender) matching `describe_sources` order. Routed frozen dispatch timestamps (`ctx.now_ns`) and outputs through native `loop.send`.
  - Added deterministic intake fixtures and unit tests in `node_test.py`: simulated runs recorded to `.tlog`, verified byte-exact replay reproduction (0 divergence, clean exit, matching dispatch counts), verified divergence detection on altered node parameters, and verified CLI `--replay` execution.
  - Measured dispatch scheduling overhead in `rtms_roundtrip_test.py` (~10-40 µs/dispatch). Updated `ARCHITECTURE.md` with explicit Python dispatch and replay guarantees.
  - Verified 35/35 workspace test suite targets and 2/2 external consumer fixture targets pass cleanly with zero regressions.

### Continuation record — 2026-09-08 (Phase 1, Phase 2 & Phase 3 Complete)

- **Phase 3 (Shared Runners & Minimal Build Declarations)**:
  - Extended C ABI (`talOS/node_api/node_api.h`, `node_api.cc`) with configuration accessors (`talos_config_load`, `talos_config_get_id`, `talos_config_get_period_us`, `talos_config_get_command_timeout_us`, `talos_config_get_device_id`, `talos_config_close`) backed by `RobotConfig` in `config_parser.h`.
  - Added standalone `[subsystem]` TOML support directly in `talos::config::ParseRobotConfig`, ensuring C++ and Python configuration parse identical values across robot manifests and standalone subsystem fixtures.
  - Implemented Python `Config` wrapper and `load_config(path, simulation)` in `talos_runtime.node_api`, with unit tests in `node_api_test.cc` and `rtms_roundtrip_test.py`.
  - Implemented `talos_cc_node` macro in `talOS/build/contract.bzl` supporting both hardware-owning (`RunHardwareNode`) and hardware-free (`RunNode`) modes with custom factory callbacks.
  - Migrated Drivetrain (`2026-robot/main_processor/drivetrain`) to `talos_cc_node`, adding `drive::CreateNode` factory and deleting handwritten `drivetrain/main.cc`.
  - Implemented standardized Python runner in `talOS/ipc/python/talos_runtime/runner.py` (`NodeContext`, `resolve_config_path`, `parse_args`, `run_node_main`) handling standard flags, signal traps, exception/resource cleanup (`finally` block closing handles), Studio live graph registration, heartbeat loop, and import-light `--describe`.
  - Implemented `talos_py_node` macro in `talOS/build/contract.bzl` generating runner entry points via genrules.
  - Migrated Intake (`2026-robot/main_processor/intake`): updated `resolve_ids` to use native `node_api.load_config(config_path)` eliminating duplicate Python device numbering algorithms, added `IntakeNode.from_context(ctx)`, delegated `main.py` to `runner.run_node_main`, migrated `BUILD` to `talos_py_node`, and deleted handwritten root `intake/main.py`.
  - Replay for Python remains explicitly unsupported with code 2 error until Phase 4.
  - Verified 28/28 intake tests pass (`//2026-robot/main_processor/intake:node_test`) and 35/35 workspace test suite targets pass with zero regressions. External consumer fixture tests pass cleanly.
- **Phase 1 (Contract Isolation & Decoupling)**:
  - Added public `messages_cc` targets with normalized `#include "talos_nodes/<node>/..."` prefixes across all nodes (`shooter`, `driver_station`, `odometry`, `operator_interface`, `telemetry`, `arbiter`, `drivetrain`). Implementation libraries made private.
  - Implemented namespaced Python packages (`talos_nodes.<node>`) using PEP 420 namespace packaging to avoid initializer collisions.
  - Generated and verified Python FlatBuffers bindings and wire codecs for drivetrain (`talos.drive`, `talos_nodes.drivetrain.packet`) matching exact wire byte layouts (`ChassisTarget` 40 bytes, `DrivetrainState` 56 bytes). Added `:messages_py` and `:messages_test` passing cleanly.
  - Created shared contract generation Starlark macros in `talOS/build/contract.bzl` (`flatbuffer_py_library`, `talos_contract_cc`) and root forwarder `build/defs.bzl`.
  - Converted node unit tests to use local fixtures (`testdata/robot.toml`, `testdata/<node>.toml`) using Bazel runfiles, decoupling tests from the full competition robot config.
  - Updated `tools/subsystem_codegen` to output isolated contracts (`:messages_cc`), namespaced include prefixes, and local test fixtures.
- **Phase 2 (Externally Consumable SDK Module)**:
  - Decoupled Studio frontend (`aspect_rules_js`, `rules_nodejs`, `pnpm`, `studio_npm`) and Rust tooling (`rules_rust`, `studio_crates`) in `MODULE.bazel` using `dev_dependency = True` so external node consumers do not download or configure Studio/Rust tooling.
  - Upgraded native Python runtime loader in `talOS/ipc/python/talos_runtime/node_api.py` to resolve `libtalos_node.so` dynamically via Bazel runfiles (`rules_python.python.runfiles.Runfiles.Create()`), supporting both in-tree and external consumer workspaces.
  - Resolved toolchain and dependency compatibility for Bazel 9 (`rules_foreign_cc` 0.14.0, `aspect_bazel_lib` 2.22.5, `platforms` 1.1.0).
  - Created standalone external consumer fixture workspace at `test/fixtures/external_consumer` (isolated via root `.bazelignore`) using `local_path_override(module_name = "talos", path = "../../..")`.
  - Implemented sample external C++ node (`//cpp_node:sample_node_test`, `//cpp_node:node`) using `@talos//talOS/events:events`, `@talos//talOS/hardware:gate`, `@talos//talOS/process:node_main` and external Python node (`//py_node:sample_py_node_test`) using `@talos//talOS/ipc/python:rtms_py`.
  - Verified clean builds and test passes in `test/fixtures/external_consumer` (2/2 passed) and full root test suite (35/35 passed across `//2026-robot/main_processor/...` and `//talOS/...`).

### Implementation record — 2026-09-08

- Added public `drivetrain:messages_cc`, `intake:messages_cc`, and
  `intake:messages_py` targets. Kept `intake:intake` as a compatibility alias.
- Migrated arbiter, operator interface, odometry, and telemetry dependencies.
  Drivetrain implementation is private; the existing arbiter teleop integration
  test receives narrowly visible, test-only access until it moves to robot tests.
- Removed unnecessary drivetrain implementation dependencies from `send_target`
  and `sim_gateway`. Existing working-tree changes were preserved.
- Baseline and post-change verification used:

  ```sh
  bazel test //2026-robot/main_processor/... //talOS/launcher:launcher_test //talOS/ipc/python:rtms_roundtrip_test
  ```

  All 11 test targets passed on macOS arm64; the post-change run reused valid
  cached test results after rebuilding. Coverage includes drivetrain recorded
  input replay, configuration golden output, intake describe/policy behavior,
  and native/Python IPC byte round trips. No policy, schema, or config changed.
- Drivetrain and intake descriptions are byte-identical before/after, including
  their five ordered sources each. Whole-robot graph JSON matches except for
  session ID: all 9 nodes answered, 22 topics, and the existing unused
  `/intake/state` warning.
- Graph probing required execution outside the sandbox because telemetry's
  existing describe path accesses shared memory. Describe isolation remains open.
- Dependency queries confirmed drivetrain messages exclude events/process and
  drivetrain implementation, and intake Python messages exclude the node, RTMS
  wrapper, and native node API. C++ contracts still depend on the existing hardware
  library for their packet envelope.
- Next: stable include/package paths, generated shared contracts, standalone test
  fixtures, and a complete external-module dependency inventory. Python bindings
  are still checked in, bare Python imports remain, and drivetrain does not yet
  export Python messages. Extraction and native Python replay are not implemented.

This plan is based on the working tree reviewed on 2026-09-08, including its
uncommitted changes. It extends the repository separation goal in `PLAN.md`
section 10. Where that older plan assumes external labels already work in the
launcher or proposes a Python transport implementation, use the findings below:
the launcher needs work, and Python must bind the existing C++ mechanisms.
`ARCHITECTURE.md` remains normative until explicitly updated during implementation.

## Outcome and scope

A node author maintains control logic, message schemas, a small contract
declaration, one short build declaration, and tests. An independent repository
also has a module declaration and generated development/CI setup. No handwritten
main program, import-path searches, native-library searches, signal loop,
heartbeat loop, or duplicated device-ID assignment is required.

The robot repository pins node and SDK versions, supplies actual hardware
configuration, and builds a relocatable launch bundle. Each node remains a
separate process using the existing RTMS transport.

Repository layout:

```text
talos/                 SDK, native runtime, language bindings, build rules
talos-driver_station/  Stable platform nodes: implementation, public contracts,
talos-operator_interface/
talos-odometry/
talos-telemetry/       tests, example config (each a standalone Bazel module)
studio-schema/         Studio frame contract (schema + slot geometry)
2026-robot/            Year mechanisms (drivetrain, intake, shooter, arbiter),
                       dependency pins, actual configs, assembly and deployment
studio/                Independent tooling module when separating SDK dependencies
```

> NOTE (2026-09-11, hybrid policy): the earlier sketch above listed one
> standalone repository per node (`talos-drivetrain/`, `talos-intake/`, ...).
> That is superseded by the Continuation record at the top of this file: only
> the stable platform nodes stay standalone; year-specific mechanisms live in
> `2026-robot/main_processor/`.

Creating remote repositories or publishing releases is a later rollout action;
the implementation can prove all boundaries with local standalone checkouts.
Replacing Bazel, changing the RIO build, changing transport layouts, and adding
network discovery are outside this migration.

## Design decisions

1. **Bazel modules are the dependency boundary.** Use immutable release/commit
   pins in the robot root and local module overrides for development. Initially
   build the whole robot against one resolved SDK version; independent source
   repositories do not imply arbitrary binary compatibility.
2. **Public contracts are separate build targets.** Each node exports
   `messages_cc` and `messages_py`; implementation libraries are private except
   for explicit tests. Contracts may remain in their owning node repository.
   Extract shared interface modules if the module dependency graph would cycle.
3. **One definition per contract.** FlatBuffers remains the message layout
   authority. A small accompanying declaration defines topics, payload types,
   and transport metadata that FlatBuffers does not describe. Generate both
   language surfaces from these inputs; do not duplicate message field layouts.
4. **Runtime registration remains authoritative for introspection.** Topic
   contracts describe available ports, not a second handwritten source table.
   Describe and execution must use the same ordered registration mechanism.
5. **SDK runners own mechanisms.** C++ keeps its templated loops and factories.
   Python uses the native C ABI for transport, configuration, and lifecycle.
   Native ABI extensions are additive. Python policy code has no allocation-free
   or hard-real-time guarantee.
6. **Robot configuration owns deployment values.** Node repositories declare
   required device roles/configuration and ship standalone examples. The robot
   owns CAN IDs, buses, calibration, geometry, periods, and concrete wiring.
   Preserve the existing merged configuration parser and deterministic IDs.
7. **Build labels are not filesystem paths or stable node identities.** A build
   provider carries executable and runtime files. Assembly resolves those into
   bundle-relative paths; names identify runtime nodes, while labels/revisions
   provide provenance. Do not parse external repository directory names.

## Architectural gaps and progress

| Area | Evidence in the current tree | Required work | Status |
|---|---|---|---|
| Public interfaces | Drivetrain's `:drivetrain` library exposed packets and implementation together | Separate interface targets and stable include prefixes | **Resolved (Phase 1)**: All nodes provide `:messages_cc` with `talos_nodes/<node>/...` includes |
| Source paths | Node includes and the generator embedded `2026-robot/main_processor` | Repository-independent paths and generator inputs | **Resolved (Phase 1)**: Headers normalized; `subsystem_codegen` updated |
| Dependencies | Root module configured SDK, Studio JS/Rust, and tooling together | Separate module dependency surfaces | **Resolved (Phase 2)**: Studio/Rust marked `dev_dependency = True`; Bzlmod compatible |
| Build settings | C++ platform requirements lived in the root `.bazelrc` | Standalone bootstrap/toolchain contract | **Resolved (Phase 2)**: `.bazelrc` settings verified in external consumer fixture |
| Python packaging | Intake used bare imports and `sys.path` fallbacks | Namespaced packages and explicit native runtime packaging | **Resolved (Phase 1 & 2)**: `talos_nodes.<node>` namespace packages; runfiles runtime loader |
| C++ startup | `talOS/process/hardware_main.h` already shares hardware startup | Build-generated entry point around the existing factory model | **Open (Phase 3)**: Generate C++ entry points via `talos_cc_node` |
| Python configuration | `intake/node.py::resolve_ids` mirrors C++ sorting/parsing | Bind the C++ parsed configuration and subsystem view | **Open (Phase 3)**: Expose native config parser via C ABI (`node_api`) |
| Python execution | Intake owns a polling loop, JSON start/stop logging, rejects replay | Shared runner first; native dispatch/replay integration | **Open (Phase 3 & 4)**: Implement `talos_py_node` runner (Phase 3); native dispatch/replay (Phase 4) |
| Configuration | Real hardware values live beside node implementation | Move deployment config to robot without changing parsed output | **Resolved (Phase 5)**: Configs relocated to `2026-robot/main_processor/configuration/subsystems/`; `config_golden_test` verified |
| Launcher & Assembly | `ResolveBinary` recognizes local `//` labels and guesses paths | Build-aware assembly and relocatable runtime manifest | **Resolved (Phase 5)**: `TalosNodeInfo` provider, `talos_bundle` rule, and launcher `--bundle <dir>` with external repo label support |

## Phase 0 — Establish compatibility baselines [COMPLETED]

- [x] Inventory node target dependencies, public includes, configs, and runtime files.
- [x] Record the actual existing test/build status; do not infer completion from the
  old `PLAN.md` status. Preserve unrelated working-tree changes.
- [x] Capture drivetrain/intake descriptions, graph lint output, configuration IDs,
  source ordering, representative message bytes, and an existing C++ replay fixture.
- [x] Document Python's actual simulation/logging/replay behavior separately from C++.
- [x] Add only missing boundary fixtures needed to judge subsequent migrations.

**Acceptance**: Complete. Recorded baselines confirmed zero change in wire byte layouts,
5 ordered describe sources per node, configuration hashing, and existing C++ replay pass.

## Phase 1 — Isolate contracts and remove source-tree coupling [COMPLETED]

- [x] Split messages from implementation in every node BUILD file; update consumers
  to depend only on published contracts (`shooter`, `driver_station`, `odometry`,
  `operator_interface`, `telemetry`, `arbiter`, `drivetrain`).
- [x] Introduce stable C++ include prefixes (`talos_nodes/<node>/...`) and distinct Python package names
  (`talos_nodes.<node>`) using PEP 420 namespace packaging.
- [x] Introduce shared contract generation for C++/Python constants and FlatBuffers
  bindings (`talOS/build/contract.bzl` and `build/defs.bzl`). Wire byte sizes,
  alignments (`ChassisTarget` 40B, `DrivetrainState` 56B), and ordering verified.
- [x] Keep ordering explicit when replacing registration or describe tables.
- [x] Move node unit-test inputs to local fixtures (`testdata/robot.toml`, `testdata/<node>.toml`)
  loaded via Bazel runfiles so tests do not depend on the complete competition robot.
- [x] Update `tools/subsystem_codegen/generator.py` to output isolated contracts,
  portable include paths, and local test fixtures.

**Acceptance**: Complete. All 10 robot node tests pass (`bazel test //2026-robot/main_processor/...`).
Implementations are private to their packages; cross-language byte fixtures pass.

## Phase 2 — Publish an externally consumable SDK module [COMPLETED]

- [x] Provide a minimal SDK module without Studio frontend/Rust dependency setup.
  Studio and Rust dependencies marked `dev_dependency = True` in `MODULE.bazel`.
- [x] Expose supported C++ runtime, message generation, Python package, and native
  library targets (`@talos//talOS/events:events`, `@talos//talOS/hardware:gate`,
  `@talos//talOS/process:node_main`, `@talos//talOS/ipc/python:rtms_py`).
- [x] Package the Python native library (`libtalos_node.so`) through Bazel runfiles
  resolution in `talOS/ipc/python/talos_runtime/node_api.py`, supporting both
  in-tree execution and external consuming workspaces.
- [x] Define supported Linux/macOS architectures and Bazel 9 toolchain settings
  (`rules_foreign_cc` 0.14.0, `aspect_bazel_lib` 2.22.5, `platforms` 1.1.0).
- [x] Add external-consumer fixture workspace at `test/fixtures/external_consumer`
  using `local_path_override(module_name = "talos", path = "../../..")`.
  Verified both external C++ node (`//cpp_node:sample_node_test`) and external
  Python node (`//py_node:sample_py_node_test`) build and pass in total isolation.

**Acceptance**: Complete. Unrelated external C++ consumer and Python consumer build and pass
in standalone fixture workspace without access to source tree or Studio tooling.
Full test suite (35 targets) passes with zero regressions.

## Phase 3 — Shared runners and small node build declarations [COMPLETED]

Delivered API macros in `@talos//build:defs.bzl`:

```python
load("@talos//build:defs.bzl", "talos_cc_node", "talos_py_node")

talos_cc_node(
    name = "node",
    node_name = "drivetrain",
    hdrs = ["node.h", "geometry.h", "swerve.h"],
    factory = "drive::CreateNode",
    contract = ":contract",
    deps = [":drivetrain"],
    hardware = True,
)

talos_py_node(
    name = "node",
    node_name = "intake",
    entry_point = "talos_nodes.intake.node:IntakeNode",
    contract = ":messages_py",
    describe_sources = "talos_nodes.intake.main:describe_source_rows",
    default_config = "talos_nodes.intake:subsystem.toml",
    deps = [
        ":intake_py",
        ":main_py",
    ],
)
```

- [x] Generated C++ entry points over `RunNode`/`RunHardwareNode` via `talos_cc_node`,
  preserving the custom factory (`drive::CreateNode`) for geometry and hardware initialization.
- [x] Implemented standardized Python runner in `talOS/ipc/python/talos_runtime/runner.py`
  with standard flags (`--describe`, `--sim`, `--session-id`, `--log`, `--config`,
  `--replay`, `--duration-s`), signal trapping (SIGINT/SIGTERM), shutdown/error handling,
  Studio live graph registration, heartbeat loop, and resource cleanup (`NodeContext`,
  `close()` on registry, node, and native config).
- [x] Describe isolation preserved for Python: `--describe` path emits manifest via
  the native C ABI without opening live IPC, shared memory, or heavy dependencies.
- [x] Extended C ABI (`talOS/node_api/node_api.h`, `node_api.cc`) with configuration
  handles and accessors (`talos_config_load`, `talos_config_get_id`, `talos_config_get_period_us`,
  `talos_config_get_command_timeout_us`, `talos_config_get_device_id`, `talos_config_close`).
  Added standalone `[subsystem]` TOML support directly in `talos::config::ParseRobotConfig`.
- [x] Python intake node migrated to use native config parsing via `node_api.load_config`,
  eliminating duplicate device-kind tables, sorting, and manual device-ID numbering algorithms.
- [x] Provided hardware-owning and hardware-free node contexts via `hardware = True/False`
  in `talos_cc_node`.
- [x] Kept registration calls and control handlers readable in author code with clean
  `from_context` / factory hooks.
- [x] Replay for Python remains explicitly unsupported with code 2 error until Phase 4.

**Acceptance**: Complete. Drivetrain and intake have no handwritten `main`, import hacks,
heartbeat loops, or config-ID algorithm. Verified signal handling, constructor/callback
failure cleanup, describe isolation, and equal C++/Python configuration results across
28/28 intake tests and 35/35 workspace-wide tests. External consumer fixture passes cleanly.

## Phase 4 — Bring Python execution onto native dispatch and replay [COMPLETED]

- [x] Extend the native runtime with ordered timer/watcher/sender registrations and
  C callback thunks that dispatch Python handlers through the binding (`talOS/node_api/node_api.h`, `node_api.cc` ABI v6).
- [x] Reuse C++ scheduling, clock, recording, and replay mechanisms (`RealtimeEventLoop`,
  `SimulatedEventLoop`, `ReplayEventLoop`, `LogWriter`, `LogReader`); do not implement
  a second Python scheduler or dispatch-log format.
- [x] Specify callback lifetime, GIL ownership, exception propagation, shutdown, and
  buffer ownership. Preserve the existing C++ dispatch path and its constraints.
- [x] Route policy time and output through the supplied context (`ctx.now_ns`, `loop.send`).
  Declare explicit capabilities for live, simulation, recording, and replay.
- [x] Update architecture documentation (`ARCHITECTURE.md`) to describe the supported
  Python guarantees and limitations, resolving the older plan's second-tier replay language.

**Acceptance**: Complete. Deterministic intake fixtures execute under simulation and replay
with matching output bytes (`test_deterministic_simulation_and_replay`, `test_replay_cli_end_to_end`).
Altered node configurations reliably detect replay divergence. Exceptions stop cleanly.
Measured dispatch scheduling overhead (~10-40 µs/dispatch). All 35 workspace tests and 2 external
consumer tests pass with zero regressions.

### Phase 5 — Separate deployment configuration and assemble bundles [COMPLETED]

- [x] Move actual subsystem configuration into the robot tree while preserving the
  existing per-subsystem TOML organization and merged parser behavior.
  Subsystem configs relocated to `2026-robot/main_processor/configuration/subsystems/*.toml`.
  Verified identical hash and parsed output via `config_golden_test`.
- [x] Relocate the simulation gateway from `drivetrain` to `2026-robot/main_processor/simulation:sim_gateway`.
  Kept backwards-compatible alias in `drivetrain/BUILD`.
- [x] Add a `TalosNodeInfo` provider in `talOS/build/contract.bzl` with fields `node_name`,
  `node_target`, `executable`, `runfiles`, `language`, `contract`.
- [x] Implemented `talos_node` executable rule wrapping inner binaries. Updated `talos_cc_node`
  and `talos_py_node` macros to return `TalosNodeInfo`.
- [x] Implemented `talos_bundle` rule (`talOS/build/contract.bzl` and `talOS/build/bundle_builder.py`)
  assembling a self-contained, relocatable deployment directory with `bundle_manifest.json`,
  native binaries, Python runtime closures with relative hermetic venvs, native libraries
  (`libtalos_node.so`), configs, and an executable `run.sh` entry point.
- [x] Defined `//2026-robot/main_processor:robot_bundle` assembling all 10 nodes and the competition robot config.
- [x] Enhanced `talOS/launcher/launcher.h` and `main.cc`:
  - Added `--bundle <dir>` CLI option.
  - Automatically loads `bundle_manifest.json`, resolves config and node binary overrides,
    and sets runtime library search paths (`LD_LIBRARY_PATH`, `DYLD_LIBRARY_PATH`, `TALOS_NODE_LIB`).
  - Enhanced `ResolveBinary()` to resolve external repository labels (e.g. `@robot//...`, `@@//...`)
    in development mode, enabling mixed local and external module execution.
- [x] Added unit tests in `talOS/launcher/launcher_test.cc` verifying external repo resolution,
  bundle loading, and launcher bundle execution with `--describe-only`.

**Acceptance**: Complete. Built `//2026-robot/main_processor:robot_bundle`, verified bundle
relocatability by executing outside Bazel/repo root, verified native launcher `--bundle` support,
tested graph lint (9/9 nodes described, 22 topics, 0 errors), ran 1-second simulation with all
10 nodes executing and cleanly exiting (code 0), and verified all 44 workspace tests and 2 external
consumer tests pass with zero regressions.

## Phase 6 — Extract the pilots and improve node creation [COMPLETED]

- [x] Create local standalone drivetrain and intake modules consuming the SDK.
- [x] Assemble them from the robot module using local overrides, then verify an
  immutable dependency snapshot workflow.
- [x] Replace the existing hardcoded generator with repository-aware C++/Python
  scaffolding (`tools/node_scaffold/`): module declaration, short BUILD, contract/schema, minimal logic,
  meaningful example test, example config, README, and shared CI invocation (`test.sh`).
- [x] Keep generated glue in build outputs. Scaffolding creates author-owned source
  once and refuses to overwrite existing work (`--force` required); SDK updates do not regenerate it.
- [x] Document standalone build/test, robot integration, local overrides, and release
  pin updates (`docs/NODE_DEVELOPMENT_GUIDE.md`). Include both hardware and hardware-free examples.

**Acceptance**: Complete. In fresh directories, scaffolded C++ hardware node (`arm`) and Python
node (`intake_test`), verified compilation, execution, and deterministic unit test pass (`bazel test //...`),
and verified `--describe` JSON introspection output. Maintained backward compatibility in `tools/subsystem_codegen`.
Cleaned up test directories. Full workspace test suite and external consumer fixture pass with zero regressions.

## Phase 7 — Migrate remaining nodes and establish releases [COMPLETE]

- Extract remaining nodes in dependency order after their contracts are isolated:
  driver station and shooter, then consumers such as arbiter, operator interface,
  odometry, and telemetry, adjusted to the verified module graph.
- Ensure telemetry depends on an explicit Studio schema contract rather than
  pulling frontend/application build setup into its module.
- Publish versioned SDK/node artifacts when repository destinations are chosen;
  update robot pins through integration-tested changes.
- Store SDK/node revisions, contract fingerprints, config identity, and runtime
  capabilities in bundle/session provenance. Revisions provide traceability;
  they do not by themselves establish wire compatibility.
- Reject incompatible topic contracts before launch, including same-size schema
  mismatches. Define compatibility/version policy for fixed-size messages;
  do not assume arbitrary schema evolution is wire-compatible.
- Remove compatibility aliases/path fallbacks only after all consumers migrate.
- Update README, ARCHITECTURE, and the older PLAN to one consistent authoring and
  deployment workflow.

Acceptance: every node builds/tests in its own checkout, the robot pins and
launches them together, and no node references the competition repository's
implementation or configuration paths. Rollback means selecting a previously
validated whole bundle with its matching configuration/runtime.

**Acceptance: met 2026-09-09.** `bazel test //...` passes in each of
`talos-driver_station`, `talos-shooter`, `talos-arbiter`,
`talos-operator_interface`, `talos-odometry`, `talos-telemetry`,
`studio-schema` (plus the pilots); `test/fixtures/robot_assembly` pins all
eight nodes, describes 9/9 with a clean graph lint, and completes a 3s sim
session with records on every node. A source sweep finds no
`2026-robot` implementation or configuration path references in the new
modules. In-tree compatibility aliases were kept, not removed: in-tree
consumers still resolve in-tree labels.

## Sequencing and review units

```text
0 baseline -> 1 contracts -> 2 SDK -> 3 runners -> 4 Python parity
                                  -> 5 assembly -> 6 pilots -> 7 rollout
```

Phase 5 depends on the node/provider API from Phase 3; Phase 6 requires both.
Phase 4 can proceed independently after Phase 3, but is required before declaring
the full runtime-parity objective complete. Keep each phase in reviewable changes:
contract extraction, SDK boundary, config binding, runner API, native dispatch,
bundle provider, launcher migration, then individual repository extractions.

The first implementation change should establish the baseline and split the
drivetrain/intake message targets. This provides a small, reviewable foundation
without making repository creation or runtime redesign the first dependency.

## Current state and remaining scope breakdown

> NOTE (2026-09-11, hybrid policy): this section's "What is left (Phases 5
> through 7)" below still lists Phase 5 as `[NEXT MILESTONE]` and Phase 6 as
> `[PENDING]`, and the Phase 7 acceptance above describes all eight nodes
> building in their own checkouts. Those snapshots predate the hybrid cutover
> and are superseded by the Continuation record at the top of this file
> (Phases 0–7 complete: platform nodes standalone, mechanisms year-local,
> `talos-drivetrain`/`talos-intake`/`talos-shooter`/`talos-arbiter` deleted,
> in-tree platform dirs removed). The per-phase sections (§Phase 0–§Phase 7)
> above remain the authoritative completion record.

### What is complete (Phases 0, 1, 2)
1. **Compatibility Baselines & Zero Drift (Phase 0)**:
   - Wire layout stability preserved (`ChassisTarget` 40B, `DrivetrainState` 56B, `Packet` 1404B).
   - Introspection registration verified (5 ordered sources per node).
   - C++ replay fixtures and configuration hashing preserved without regression.
2. **Contract Isolation & Namespace Hygiene (Phase 1)**:
   - All node BUILD targets split into private implementation libraries and public `:messages_cc` / `:messages_py`.
   - Include prefixes normalized to `talos_nodes/<node>/...` across all C++ headers and callers.
   - Namespaced Python packages (`talos_nodes.<node>`) using PEP 420.
   - Shared contract generation rules in `@talos//talOS/build:contract.bzl` and forwarder `@talos//build:defs.bzl`.
   - Node unit tests run using local test fixtures (`testdata/robot.toml`, `testdata/<node>.toml`) without relying on competition robot configs.
   - Subsystem codegen (`tools/subsystem_codegen`) updated to produce the new decoupled layout.
3. **SDK Standalone Usability & External Consumer Verification (Phase 2)**:
   - Studio frontend and Rust dependencies marked `dev_dependency = True` in root `MODULE.bazel`.
   - Python native runtime (`libtalos_node.so`) loader updated to resolve via Bazel runfiles.
   - Bazel 9 toolchain settings and compatible BCR version pins established.
   - Standalone external consumer fixture workspace created at `test/fixtures/external_consumer` verifying independent C++ and Python node compilation and testing in 100% isolation.
4. **Shared Runners and Minimal Node Build Declarations (Phase 3)**:
   - Native configuration C ABI accessors (`talos_config_*`) bound to `talos::config::ParseRobotConfig`, supporting standalone subsystem TOML fixtures.
   - `talos_cc_node` Starlark macro generating standard C++ entry points (`RunNode`/`RunHardwareNode`). Migrated Drivetrain.
   - `talos_py_node` Starlark macro and `runner.py` standard Python runner handling CLI parsing, signal trapping, heartbeat, Studio live graph registration, and clean shutdown.
   - Migrated Intake pilot to `talos_py_node`.
5. **Native Dispatch and Replay Parity for Python (Phase 4)**:
   - Native C ABI extended to version 6 (`talOS/node_api/node_api.h`, `node_api.cc`) with `TalosLoop`, `TalosDispatchContext`, `TalosDispatchFn`, and 19 ABI functions spanning realtime, simulation, and replay.
   - Reused C++ scheduling, clock, recording, and replay mechanisms (`RealtimeEventLoop`, `SimulatedEventLoop`, `ReplayEventLoop`, `LogWriter`, `LogReader`) with zero second scheduler or format.
   - Python ctypes bindings (`talos_runtime.node_api::Loop`) with automatic GIL handling via `ctypes.CFUNCTYPE`, callback lifetime retention, exception propagation, and clean shutdown.
   - Standard runner updated to support `--replay <path>`.
   - Intake node adapted to ordered native loop registrations (`IntakeNode(..., loop=...)`), verified under deterministic simulation and byte-exact replay execution. Altered parameters correctly trigger divergence detection.

---

### What is left to do (Phases 5 through 7)

#### Phase 5 — Separate deployment configuration and assemble bundles [NEXT MILESTONE]
- **5.1: Robot Deployment Config Separation**:
  - Move competition-specific device mappings (CAN IDs, bus names, calibrations) into `2026-robot` while node repositories retain standalone example configs.
- **5.2: Bazel Packaging Provider (`TalosNodeInfo`)**:
  - Implement a custom Bazel provider carrying binary artifacts, contract metadata, and runfiles closures.
- **5.3: Assembly Rule (`talos_bundle`)**:
  - Rule accepting node labels and configs to produce a relocatable deployment directory/archive with a machine-readable manifest.
- **5.4: Launcher Manifest Resolution**:
  - Update `talos_launcher` to resolve node binaries from the bundle manifest and Bazel runfiles rather than guessing paths from local `//` labels.

#### Phase 6 — Extract the pilots and improve node creation [PENDING]
- **6.1: Physical Pilot Repositories**:
  - Extract `talos-drivetrain` and `talos-intake` into separate standalone Git repositories/modules with their own `MODULE.bazel`.
- **6.2: Next-Generation Scaffolding Tool**:
  - Modernize `tools/subsystem_codegen` to generate complete standalone node repository templates.

#### Phase 7 — Migrate remaining nodes and establish releases [COMPLETE]
- **7.1: Remaining Node Extraction**:
  - Extract `shooter`, `driver_station`, `odometry`, `operator_interface`, `arbiter`, `telemetry`.
- **7.2: Versioning & Release Cadence**:
  - Pin immutable releases or Git commits in `2026-robot/MODULE.bazel`.
- **7.3: Contract Compatibility Validation**:
  - Pre-launch handshake rejecting incompatible topic hash or schema mismatches.

## Verification policy

Run targeted checks for each changed boundary, then the robot integration suite
at pilot extraction and final rollout. Use isolated IPC sessions/cleanup to avoid
tests interfering with a running robot. Do not require hardware deployment to
prove repository portability. Preserve message bytes and C++ replay fixtures;
where identity/provenance fields legitimately change, assert that explicitly
rather than silently replacing goldens.

Reference for external modules, local overrides, and runfiles mapping:
https://bazel.build/external/module
