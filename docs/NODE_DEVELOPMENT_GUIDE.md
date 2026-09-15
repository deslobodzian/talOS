# talOS Node Development & Scaffolding Guide

This guide describes how to author, scaffold, test, and integrate independent subsystem nodes in talOS.

---

## 1. Overview & Architectural Principles

talOS subsystems are architected as **independent, repository-aware Bazel modules**. Rather than being tightly coupled within a monolithic robot repository, each node lives in its own repository (or module) with minimal authoring boilerplate.

### Key Architectural Tenets

1. **Bazel Modules as Dependency Boundaries (Bzlmod)**:
   - Each node declares its own `MODULE.bazel` specifying dependencies (`talos` SDK, rulesets, etc.).
   - Development uses `local_path_override` pointing to local checkouts. Production deployments pin immutable Git commits or release archive hashes.
2. **Minimal Author-Owned Boilerplate**:
   - Node authors write **only**: control logic, FlatBuffers schemas, topic declarations (`packet.h`), standalone test fixtures, and unit tests.
   - Authors do **not** write: boilerplate `main()` functions, signal handlers, heartbeat loops, CLI argument parsers, or custom IPC memory allocators.
   - The talOS SDK provides unified macros (`talos_cc_node`, `talos_py_node`) and shared runners (`talOS/process/hardware_main.h`, `talos_runtime.runner`) that own lifecycle, introspection (`--describe`), and recording/replay.
3. **Public Contracts Decoupled from Implementation**:
   - Each node publishes a contract (`contract` or `messages_cc` / `messages_py`) with stable include paths (`talos_nodes/<node>/packet.h`, `talos_nodes/<node>/<node>_message_generated.h`).
   - Sibling nodes depend exclusively on public contracts, never private implementations.
4. **Configuration Separation**:
   - Node repositories own *example* device configurations (`subsystem.toml`).
   - The competition robot repository owns *deployment* values (physical CAN IDs, CAN buses, sensor inversions, gear ratios, PID gains).
5. **Deterministic Simulation & Replay Parity**:
   - C++ nodes run on `event::SimulatedEventLoop` with virtual clock stepping.
   - Python nodes run on `talos_runtime.node_api.Loop.create_simulated()` backed by the same C ABI and virtual clock, enabling deterministic testing and zero-divergence replay reproduction.

---

## 2. Scaffolding Nodes with `tools/node_scaffold`

talOS includes a repository-aware scaffolding tool in [`tools/node_scaffold`](file:///Users/deslobodzian/Documents/Projects/talOS/tools/node_scaffold). It generates a complete standalone Bazel module configured with the modern runner and contract rules.

### Command Line Interface

```bash
python3 tools/node_scaffold/main.py --name <node_name> --lang <cpp|python> [--hardware] --out <output_dir> [--sdk-path <path>] [--force]
```

### Options

| Flag | Required | Default | Description |
|---|---|---|---|
| `--name` | Yes | - | Subsystem / node name (e.g. `arm`, `intake`, `shooter`). Must be valid snake_case. |
| `--lang` | Yes | - | Implementation language: `cpp` or `python`. |
| `--hardware` | No | `False` | When set, includes CAN motor/sensor configuration, hardware gating, and Eigen (for C++). |
| `--out` | Yes | - | Target directory where the standalone module will be scaffolded. |
| `--sdk-path` | No | Auto | Path to the talOS SDK repository for `local_path_override`. Defaults to relative path or `../talos`. |
| `--force` | No | `False` | Overwrites files in `--out` if the directory already exists and is non-empty. |

### Safety Behavior

If `--out` points to an existing directory containing files and `--force` is **not** passed, the tool safely aborts:
```
error: Refusing to overwrite existing files in '/path/to/dir'. Pass --force to overwrite author-owned source.
```
This protects author-owned source code from accidental destruction.

### Generated Layout

```text
<output_dir>/
├── .bazelrc            # C++23 platform flags and Bzlmod enablement
├── MODULE.bazel        # Standalone Bzlmod module declaration
├── BUILD               # Minimal build targets using @talos//build:defs.bzl
├── <name>_message.fbs  # FlatBuffers schema defining Target and State structs
├── packet.h            # Topic constants and wire size declarations
├── packet.py           # (Python only) Wire codecs and topic constants
├── node.h / node.py    # Node implementation with from_context factory
├── node_test.cc / .py  # Deterministic test running on SimulatedEventLoop
├── subsystem.toml      # Subsystem specification and default devices
├── testdata/
│   ├── robot.toml      # Local test robot manifest
│   └── <name>.toml     # Subsystem device mappings for testing
├── README.md           # Instructions for building, testing, and robot integration
└── test.sh             # Executable convenience test runner
```

### Build your first intake (walkthrough)

Scaffold a hardware C++ intake next to your talOS checkout, test it standalone,
then wire it into an assembly. Ten minutes, end to end:

```bash
# 1. Scaffold (writes a minimal MODULE: just talos + core toolchain pins)
python3 tools/node_scaffold/main.py --name intake --lang cpp --hardware --out ../talos_intake

# 2. Test standalone — no robot checkout, no platform siblings needed
cd ../talos_intake && bazel test //...
```

Make it yours: put control logic in `node.h`, tune the example gains in
`subsystem.toml`, and extend `intake_message.fbs` only by adding a new
contract (fixed-size messages have no evolution policy — any schema change
is a new contract; see §6).

```bash
# 3. Assemble: pin the checkout from the assembly MODULE, launch its node
#    in talos_bundle, and stage its deployment config (full details in §5)
bazel build //...   # the assembly's bundle target
bazel run <bundle>.run -- --sim
```

Single-season mechanism? Skip the standalone checkout: add the intake under
`2026-robot/main_processor/` instead (§5, year-folder path) — no `MODULE.bazel`
ceremony, rewritten next season with no repo overhead.

---

## 3. Standalone Node Authoring Guide

### 3.1 C++ Node Authoring

C++ nodes deliver hard real-time execution, deterministic memory allocation, and direct integration with hardware drivers.

#### 1. Contract Definition (`packet.h` and `<name>_message.fbs`)

Messages use FlatBuffers structs (fixed size, never tables):

```flatbuffers
// arm_message.fbs
namespace talos.arm;

struct ArmTarget {
  issued_ns:ulong;
  target_velocity_rps:double;
  enabled:bool;
}

struct ArmState {
  timestamp_ns:ulong;
  velocity_rps:double;
  target_velocity_rps:double;
  at_target:bool;
  enabled:bool;
}
```

Topic constants belong in `packet.h` (ARCHITECTURE.md Rule 5: never `node.h`):

```cpp
// packet.h
#pragma once

#include <cstdint>
#include <string>
#include "talOS/hardware/packet.h"

namespace talos::arm {

using Packet = hardware::Packet;

inline constexpr const char* kHwStateTopic = hardware::kStateTopic;
inline constexpr const char* kStateTopic = "/arm/state";
inline constexpr const char* kTargetTopic = "/arm/target";
inline const std::string kRequestTopic = hardware::RequestTopic("arm");

inline constexpr uint32_t kPeriodUs = 5000;
inline constexpr uint32_t kCommandTimeoutUs = 100000;

}  // namespace talos::arm
```

#### 2. Node Implementation (`node.h`)

Nodes are templated on `Loop` and supply a `from_context(NodeContext&)` factory:

```cpp
#pragma once

#include "talOS/events/handles.h"
#include "talOS/hardware/gate.h"
#include "talOS/process/hardware_main.h"
#include "talos_nodes/arm/packet.h"
#include "talos_nodes/arm/arm_message_generated.h"

namespace talos::arm {

template <typename Loop>
class ArmNode {
 public:
  struct NodeContext {
    Loop& loop;
    hardware::Config hardware_config{};
    hardware::Devices devices{};
    bool simulation{false};
  };

  static ArmNode from_context(const NodeContext& ctx) {
    return ArmNode(ctx.loop, ctx.hardware_config, ctx.devices, ctx.simulation);
  }

  static ArmNode from_context(Loop& loop, const talos::process::HardwareSubsystem& sub, bool simulation) {
    return from_context(NodeContext{
        .loop = loop,
        .hardware_config = sub.robot.hardware,
        .devices = sub.devices,
        .simulation = simulation,
    });
  }

  explicit ArmNode(Loop& loop, hardware::Config config, hardware::Devices devices = {}, bool simulation = false)
      : config_(std::move(config)), devices_(std::move(devices)), simulation_(simulation) {
    event::watch<Packet, &ArmNode::OnHwState>(loop, kHwStateTopic, this);
    event::watch<ArmTarget, &ArmNode::OnTarget>(loop, kTargetTopic, this);
    request_ = event::make_sender<Packet>(loop, kRequestTopic);
    state_sender_ = event::make_sender<ArmState>(loop, kStateTopic);
    timer_ = event::make_timer<&ArmNode::Tick>(loop, "arm", this);
  }

  void Start(event::MonotonicTime first) {
    timer_.setup_periodic(first, std::chrono::microseconds{config_.period_us});
  }

 private:
  void OnHwState(const event::Context& context, const Packet& packet) { /* ... */ }
  void OnTarget(const event::Context& context, const ArmTarget& target) { /* ... */ }
  void Tick(const event::Context& context) { /* ... */ }
};

// Factory adapter for talos_cc_node
template <typename Loop>
inline auto from_context(Loop& loop, const talos::process::HardwareSubsystem& sub, bool simulation) {
  return ArmNode<Loop>::from_context(loop, sub, simulation);
}

}  // namespace talos::arm
```

#### 3. BUILD Declaration (`BUILD`)

```starlark
load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")
load("@talos//build:defs.bzl", "talos_cc_node", "talos_contract_cc")

talos_contract_cc(
    name = "contract",
    node_name = "arm",
    fbs_srcs = ["arm_message.fbs"],
    packet_hdr = "packet.h",
)

cc_library(
    name = "arm",
    hdrs = ["node.h"],
    strip_include_prefix = ".",
    include_prefix = "talos_nodes/arm",
    deps = [
        ":contract",
        "@talos//talOS/configuration:configuration",
        "@talos//talOS/events:events",
        "@talos//talOS/hardware:gate",
        "@talos//talOS/hardware:hardware",
        "@talos//talOS/process:hardware_main",
        "@eigen//:eigen",
    ],
)

talos_cc_node(
    name = "node",
    node_name = "arm",
    hdrs = ["node.h"],
    factory = "talos::arm::from_context",
    contract = ":contract",
    hardware = True,
    deps = [":arm"],
)

cc_test(
    name = "node_test",
    srcs = ["node_test.cc"],
    data = ["testdata/robot.toml", "testdata/arm.toml"],
    env = {"TALOS_TEST_CONFIG": "$(rlocationpath testdata/robot.toml)"},
    deps = [
        ":arm",
        "@googletest//:gtest_main",
        "@rules_cc//cc/runfiles",
        "@talos//talOS/events:events",
    ],
)
```

---

### 3.2 Python Node Authoring

Python nodes provide rapid prototyping, dynamic scripting, and complete parity with native dispatch, virtual clock stepping, and replay.

#### 1. Contract & Codecs (`packet.py`)

Define topics, struct layouts, and ordered describe rows:

```python
# packet.py
import struct
from talos_runtime import node_api

NAME = "intake"
PERIOD_US = 5000
TARGET_TOPIC = "/intake/target"
STATE_TOPIC = "/intake/state"
TARGET_SIZE = 24
STATE_SIZE = 32

def describe_sources():
    return [
        (node_api.TIMER, NAME, 0, False, False),
        (node_api.WATCHER, TARGET_TOPIC, TARGET_SIZE, False, False),
        (node_api.SENDER, STATE_TOPIC, STATE_SIZE, False, False),
    ]

def encode_target(issued_ns: int, target_vel: float, enabled: bool) -> bytes:
    return struct.pack("<Qd?7x", issued_ns, target_vel, enabled)

def decode_target(payload: bytes):
    return struct.unpack_from("<Qd?", payload, 0)
```

#### 2. Node Implementation (`node.py`)

Implement `<Name>Node.from_context(ctx)` and register ordered native sources on the loop:

```python
# node.py
import packet
from talos_runtime import node_api, runner

def describe_source_rows():
    return packet.describe_sources()

class IntakeNode:
    @classmethod
    def from_context(cls, ctx: runner.NodeContext):
        return cls(
            node_name=ctx.node_name,
            period_us=ctx.period_us,
            simulation=ctx.simulation,
            loop=ctx.loop,
        )

    def __init__(self, node_name="intake", period_us=packet.PERIOD_US, simulation=False, loop=None):
        self.node_name = node_name
        self.period_us = period_us
        self.simulation = simulation
        self.loop = loop
        self.ticks = 0
        if loop is not None:
            self.register_sources(loop)

    def register_sources(self, loop):
        self.loop = loop
        # Order MUST match describe_sources:
        self.timer_id = loop.register_timer(
            self.node_name,
            period_ns=self.period_us * 1000,
            offset_ns=0,
            callback=self._on_timer,
        )
        self.target_sub_id = loop.register_watcher(
            packet.TARGET_TOPIC,
            packet.TARGET_SIZE,
            packet.TARGET_ALIGN,
            self._on_target,
        )
        self.state_pub_id = loop.register_sender(
            packet.STATE_TOPIC,
            packet.STATE_SIZE,
            packet.STATE_ALIGN,
        )

    def _on_target(self, ctx, payload):
        self.target = packet.decode_target(payload)

    def _on_timer(self, ctx, payload):
        self.ticks += 1
        # Send state...
```

#### 3. BUILD Declaration (`BUILD`)

```starlark
load("@rules_python//python:defs.bzl", "py_library", "py_test")
load("@talos//build:defs.bzl", "flatbuffer_py_library", "talos_py_node")

flatbuffer_py_library(
    name = "messages_py",
    srcs = ["intake_message.fbs"],
    outs = [
        "talos/intake/__init__.py",
        "talos/intake/IntakeTarget.py",
        "talos/intake/IntakeState.py",
    ],
)

py_library(
    name = "intake",
    srcs = ["node.py", "packet.py"],
    imports = ["."],
    deps = [
        ":messages_py",
        "@talos//talOS/ipc/python:rtms_py",
    ],
)

talos_py_node(
    name = "node",
    node_name = "intake",
    entry_point = "node:IntakeNode",
    contract = ":messages_py",
    describe_sources = "node:describe_source_rows",
    default_config = "subsystem.toml",
    deps = [":intake"],
)

py_test(
    name = "node_test",
    srcs = ["node_test.py"],
    data = ["testdata/robot.toml", "testdata/intake.toml"],
    deps = [":intake", "@rules_python//python/runfiles"],
)
```

### 3.3 Student motor API and teaching examples (year intake)

The year intake (`2026-robot/main_processor/intake/talos_nodes/intake/`)
keeps student behavior in one small file, `student_policy.py`, and puts
everything else behind helpers infrastructure owns. Students touch these:

| Helper | What it answers |
|---|---|
| `student_policy.Target` / `Feedback` | Typed ask (mechanism rev/s) and beam sensor side; `command_for` maps them to a demand. A broken beam holds stop against further intake, but eject still runs. |
| `motor_commands` | `duty_cycle` / `voltage` / `velocity` / `position` / `stop` by device name; `velocity`/`position` are the normal path (Talon onboard PID, `slot` selects the `slotN` gains). |
| `units` | Rotor vs mechanism rotations, rps, rad/s, volts -- with boundary validation at every conversion. |
| `control_config` | PID slot, gain units, gearing point (`sensor_to_mechanism_ratio`), and whether the loop is tuned (all gains zero = UNTUNED: demands send but the wheel lags). |
| `behaviors` | Shared `Debounce` (beam chatter), `EdgeDetector` (capture counting), `ElapsedTimer` / `run_until_or_timeout` (bounded sequences). |

Three complete examples in `talos_nodes/intake/examples/`, each with a
`CONFIG_TOML`, a `run_sim` simulation, a `diagnose` diagnostic, and
commissioning notes in its docstring: `roller_intake` (velocity +
capture), `flywheel` (velocity + at-speed, tuned gains), and
`position_mechanism` (position servo through a 25:1 gearbox).

```bash
bazel test //2026-robot/main_processor/intake:student_api_test \
           //2026-robot/main_processor/intake:examples_test
```

---

## 4. Testing Standalone Nodes

Node tests run against local fixtures in `testdata/`, keeping them completely decoupled from the competition robot configuration.

### Running C++ Tests

```bash
# In the standalone node repository
bazel test //...
```
Under the hood, `node_test.cc` constructs an `event::SimulatedEventLoop`, injects inputs, advances virtual time, and asserts dispatches:

```cpp
TEST(ArmNodeTest, SimulatedLoopExecutesAndUpdatesState) {
  event::SimulationEnvironment environment;
  event::SimulatedEventLoop<> loop{environment};
  auto node = ArmNode<decltype(loop)>::from_context({.loop = loop, ...});
  node.Start(loop.monotonic_now());

  loop.inject(kTargetTopic, target);
  loop.run_for(20ms);

  EXPECT_GT(node.ticks(), 0u);
  EXPECT_TRUE(node.latest_state().enabled());
}
```

### Running Python Tests

```bash
bazel test //:node_test
```
`node_test.py` creates a simulated loop via `node_api.Loop.create_simulated()`, runs virtual clock ticks, and asserts outputs:

```python
with node_api.Loop.create_simulated() as loop:
    ctx = runner.NodeContext(node_name="intake", config=None, simulation=True, session_id=1, loop=loop)
    node = IntakeNode.from_context(ctx)
    loop.sim_inject(packet.TARGET_TOPIC, target_bytes)
    loop.run_for(30_000_000)
    assert node.ticks > 0
```

---

## 5. Robot Integration Workflow

When a node is ready to be assembled into the competition robot, pick one of
two paths and do not mix them:

- **Standalone node** (multi-season, e.g. a gripper you will reuse): lives in
  its own checkout with its own `MODULE.bazel` (§2) and is pinned by the
  assembly that launches it — a standalone assembly workspace such as
  `test/fixtures/robot_assembly`, or the in-tree year bundle
  (`2026-robot/main_processor/BUILD`, whose platform pins resolve through
  the root `MODULE.bazel`'s dev-only entries).
- **Year-local mechanism** (single-season drivetrain, intake, shooter,
  arbiter): lives directly under `2026-robot/main_processor/` as part of the
  root `talos` module — no `MODULE.bazel`, no pin, rewritten next season.
  Skip to Step 2.

### Step 1: Pin a standalone node from the assembly `MODULE.bazel`

Only for the standalone path. The assembly declares exactly the nodes it
launches — never the platform closure. During active development, use
`local_path_override`:

```starlark
# In the assembly workspace MODULE.bazel (e.g. test/fixtures/robot_assembly)
bazel_dep(name = "talos_arm", version = "0.1.0")
local_path_override(
    module_name = "talos_arm",
    path = "../talos_arm",
)
```

### Step 2: Assemble Nodes into `talos_bundle`

In `2026-robot/main_processor/BUILD`:

```starlark
load("@talos//build:defs.bzl", "talos_bundle")

talos_bundle(
    name = "robot_bundle",
    config = "//2026-robot/main_processor/configuration:robot.toml",
    subsystems = [
        "//2026-robot/main_processor/configuration:subsystems/odometry.toml",
        "//2026-robot/main_processor/configuration:subsystems/intake.toml",
        # other subsystem configurations...
    ],
    nodes = [
        "@talos_arm//:node",
        "@talos_odometry//:node",
        # other nodes...
    ],
)
```

### Step 3: Configure Deployment Values in Robot Manifest

In `2026-robot/main_processor/configuration/robot.toml`:

```toml
[robot]
name = "2026-competition-robot"
period_us = 5000
command_timeout_us = 100000
status_hz = 200.0

[[subsystems]]
name = "arm"
path = "subsystems/arm.toml"

[[subsystems]]
name = "intake"
path = "subsystems/intake.toml"
```

In `2026-robot/main_processor/configuration/subsystems/arm.toml`:

```toml
[subsystem]
name = "arm"
node = "@talos_arm//:node"
period_us = 5000

[motors.arm_motor]
type = "TalonFX"
bus = "canivore"
can_id = 14
gear_ratio = 45.0
current_limit_a = 60.0
```

### Step 4: Build and Launch the Bundle

Build the relocatable bundle:
```bash
bazel build //2026-robot/main_processor:robot_bundle
```

Run simulated execution:
```bash
bazel run //2026-robot/main_processor:robot_bundle.run -- --sim
```

Or launch via the `launcher` binary directly:
```bash
bazel-bin/talOS/launcher/launcher --bundle bazel-bin/2026-robot/main_processor/robot_bundle --sim
```

---

## 6. Immutable Dependency Snapshot Workflow

For competition deployment, field testing, and release tagging, local overrides (`local_path_override`) are replaced with **immutable dependency pins**.

### Option A: Immutable Git Commit Pin (`git_override`)

Pins the exact Git commit SHA of the standalone node repository:

```starlark
bazel_dep(name = "talos_arm", version = "0.1.0")
git_override(
    module_name = "talos_arm",
    remote = "https://github.com/team-talos/talos-arm.git",
    commit = "7e1b93f6c8d4512a8b9f1d2e3c4b5a6f7890abcd",
)
```

### Option B: Release Archive Pin (`archive_override`)

For sealed releases and air-gapped CI, use signed release archives with strict SHA-256 verification:

```starlark
bazel_dep(name = "talos_arm", version = "0.1.0")
archive_override(
    module_name = "talos_arm",
    urls = [
        "https://releases.talos-robotics.org/nodes/talos_arm-0.1.0.tar.gz",
    ],
    sha256 = "8f3b4c5e6a7d8e9f0123456789abcdef0123456789abcdef0123456789abcdef",
    strip_prefix = "talos-arm-0.1.0",
)
```

### Option C: Offline Competition Vendor Cache (`bazel vendor`)

Bazel 9 supports full offline vendoring for competition pit environments without internet connectivity:

1. **Vendor all resolved dependencies**:
   ```bash
   bazel vendor --vendor_dir=vendor
   ```
2. **Build completely offline at the event**:
   ```bash
   bazel build --vendor_dir=vendor //2026-robot/main_processor:robot_bundle
   ```

### Provenance Tracking

When `talos_bundle` creates a deployment directory, it writes `bundle_manifest.json` containing:
- Target labels (e.g. `@talos_arm//:node`)
- Binary paths and execution wrappers
- `config_sha256`: sha256 over the robot config plus every subsystem file
  (subsystem names mixed in, so a rename is an identity change)
- Per-node `contract` labels (e.g. `@talos_odometry//:contract`; empty for
  nodes that own no schema, such as the arbiter)
- Runtime language closures

The launcher carries this provenance into the session: `manifest.json` records
`config_sha256` and each node's contract label, and `graph.json` records
`config_sha256` beside the per-topic message sizes the compatibility lint
checked. This guarantees end-to-end traceability from the pit laptop to the
roboRIO. Provenance is traceability, not a compatibility proof: a matching
contract label does not establish wire compatibility.

### Contract Compatibility

Fixed-size messages have no evolution policy: any schema change is a new
contract. Before spawning, the launcher lints the `--describe` graph and
refuses to start when two ends use one topic with different message sizes
(`LintGraph` reports `used with two message sizes`, naming both nodes),
unless `--allow-graph-errors`. A size match is not a compatibility proof — a
schema can change without changing size — so mismatched revisions are handled
by pinning and rolling back the whole bundle with its matching
configuration and runtime, never by mixing node versions.

---

## 7. Hybrid Node Policy: Platform Modules vs Year Mechanisms

Only stable platform nodes are standalone Bazel modules. Year-specific
mechanisms (drivetrain, intake, shooter, arbiter) live in the year folder
(`2026-robot/main_processor/`) and are rewritten each season with no repo
ceremony. A new season clones the year folder (pins, configs, assembly),
never the mechanisms. The assembly fixture (`test/fixtures/robot_assembly`)
pins the platform modules with `local_path_override` and launches them
together.

| Module | Node | Contract | Entry point |
|---|---|---|---|
| `talos-driver_station` | driver_station (C++) | `driver_station_message.fbs` + `packet.h` | `talos_cc_node`, fail-fast config parse |
| `talos-operator_interface` | operator_interface (C++) | none (reads platform + chassis contracts) | `talos_cc_node`, config-driven tuning |
| `talos-odometry` | odometry (C++) | `odometry_message.fbs` + `packet.h` | `talos_cc_node`, ignores config |
| `talos-telemetry` | telemetry (C++) | none (consumes into Studio frames) | handwritten `main.cc` + `talos_node` |
| `talos-chassis_contracts` | neutral chassis frames (no node) | `drive_message.fbs` + `packet.h` | `flatbuffer_cc_library` + `contract` |
| `studio-schema` | Studio frame contract | `telemetry.fbs` + `wire.h` | `flatbuffer_cc_library` + `wire` |

Notes for authors:

- Nodes that own a schema expose it through `talos_contract_cc` (aliased as
  both `:contract` and `:messages_cc`); consumers depend on the contract, never
  on the implementation library. Nodes that own no schema (operator_interface,
  telemetry) depend on the contracts they consume.
- Platform modules read the current year's mechanism frames through the year's
  public contract targets (`@talos//2026-robot/main_processor/<mech>:
  messages_cc`), e.g. telemetry reads drivetrain frames, operator_interface
  reads drivetrain/intake/shooter frames. There is exactly one definition of
  each frame type per binary: never vendor a second copy of a FlatBuffers
  schema into a consumer (duplicate types with one include path fail the
  build). Adopting new frames is a mechanical dep-label update when the year
  folder keeps its layout.
- `talos_cc_node` supports two standalone-only options: `optional_sources`
  (C++ topic expressions declared as optional graph endpoints) and
  `default_config_path` (default `--config` for standalone checkouts, so
  binaries never fall back to the competition tree's `robot.toml`).
- Telemetry keeps a handwritten `main.cc`: its explicit first-tick starter,
  external-source `--describe` hook, live-Reporter extra source, and exit
  summary cannot be expressed by `talos_cc_node`. It still publishes
  `TalosNodeInfo` via `talos_node` for `talos_bundle`.
- In-tree mechanism contracts keep their historical schema filenames (e.g.
  in-tree `drive_message.fbs`, namespace `talos.drive`); platform sources
  include the in-tree generated names (`drive_message_generated.h`).
- Telemetry depends on `@studio_schema`, the standalone Studio frame contract.
  Its only dependency is `flatbuffers`: the Node/pnpm frontend, uWS bridge,
  and Rust desktop toolchains stay `dev_dependency` in the root module and
  never enter a robot build.
- Compatibility aliases in `2026-robot/` (e.g. `:contract` next to
  `:messages_cc`) stay until every consumer migrates.
- The SDK lists platform modules as `dev_dependency` assembly deps, so they
  never enter a downstream consumer's graph: a minimal node `MODULE.bazel`
  needs only `talos` plus the core toolchain pins (exactly what
  `tools/node_scaffold` generates; see
  `test/fixtures/external_consumer/MODULE.bazel`). Bzlmod honors only
  root-module `local_path_override`s, so each standalone module declares
  `bazel_dep` + override pairs solely for the platform modules it directly
  consumes, and each assembly pins solely the nodes it launches. Every
  module pins the same toolchain versions as the
  root (`rules_foreign_cc`, `aspect_bazel_lib`, `rules_go`,
  `apple_support`, `platforms`) so minimal modules resolve the
  same toolchain graph. Deliberately absent: `aspect_rules_esbuild`
  (nothing loads it, and 0.27.0 requires `aspect_rules_js` 3.x, whose npm
  tag dropped the `root_package` attribute flatbuffers still passes).
