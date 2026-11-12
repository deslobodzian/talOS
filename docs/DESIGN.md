# Design method

How architectural decisions are made and how they stay true.
[ARCHITECTURE.md](../ARCHITECTURE.md) states **what** the architecture is;
this file states **how it changes** and **what checks it**, and restates no
contract.

An architectural rule no machine checks is a hope. Each rule below carries a
status mark ([METHODOLOGY.md](METHODOLOGY.md)): **[enforced]** names its
check, **[review]** is checked by the `reviewer` agent, **[gap]** names the
TODO.md item that adds the check.

## 1. When a change needs a design note

Most changes do not. A note is written before implementation when a change:

| Trigger | Example |
|---|---|
| changes who can actuate, or when | `robot_enabled`, `Target`, mode generations, arming, owner timeout, gateway gating, the RoboRIO watchdog |
| changes a wire or memory layout | a UDP frame, a payload codec, the RTMS segment, `.tlog`, the telemetry frame, `graph.json`, the session manifest |
| changes the C ABI incompatibly | a `talOS/node_api` signature or constant |
| adds a layer, a package under `talOS/`, a process, or a thread on a control path | a log process; a new loop tier |
| adds a topic role, a naming rule or a reserved owner | NAMING.md |
| adds a third-party dependency | [METHODOLOGY.md §13](METHODOLOGY.md#13-open-source-first) |
| changes a number in RULES.md, a budget in DATA_FLOW.md, or a bound a requirement states | the `command_timeout_us` limits; the owner timeout |
| has two or more viable designs and the choice is expensive to reverse | where node logs are written (TODO.md INC-3) |

The `slow` agent (or a person) writes the note, in at most one page:

```
Requirement:   REQ id(s) served. None: stop (§6).
Invariant:     the one sentence that is true after the change.
Design:        one recommendation.
Rejected:      each alternative, and why.
Interfaces:    messages, topics, C signatures, config keys, layouts, verbatim.
Budgets:       bytes, time, CPU, CI minutes that move (§5), with new numbers.
Failure modes: what happens when each new part stalls, dies or lies; what goes neutral.
Verification:  the tests by tier (TESTING.md §2); what only a bench can show.
Deletes:       code, config, docs and tests this makes unnecessary.
Slices:        file sets, and the order interfaces are fixed in.
```

The user approves a note triggered by a safety path, a wire or memory
layout, a requirement or bound change, or a new dependency. The main session
may approve the others after `reviewer` reads the note.

## 2. Where decisions are recorded

| What | Where | Why there |
|---|---|---|
| The architecture as it is | ARCHITECTURE.md, NAMING.md, DATA_FLOW.md, package READMEs | RULES.md 13: docs describe the code now |
| Why it is that way, and what was rejected | the commit's `Why:` | attached to the diff it explains; cannot drift |
| A rationale a reader needs in order not to undo a contract | at most five lines beside the contract ([DATA_FLOW.md](DATA_FLOW.md)'s "Why one datagram per hardware tick?") | the next reader reads the doc, not the log |
| A decision not yet made | TODO.md, with the measurement or question that decides it | open work is not documentation |
| The design note | the commit message, or the pull request description when there is one; its content lands in the rows above | a plan is not a doc (RULES.md 13) |
| What the system must do | [REQUIREMENTS.md](REQUIREMENTS.md) | one catalogue; tests and notes cite IDs |

There is no ADR directory. Find a decision with
`git log --grep='<symbol or topic>'` or `git log -S'<symbol>'`. A decision
asked about twice earns a "Why" paragraph beside its contract.

## 3. Layers and dependencies

The rules are ARCHITECTURE.md
["Allowed dependencies"](../ARCHITECTURE.md#layers). Their checks:

| Rule | Status |
|---|---|
| Layers depend only downward (rtms → events → node → devices → robot code) | **[enforced]**: `tools/check_layers.py` in CI, whose allowlist holds today's exceptions (`talOS/devices:testing` on `talOS/bridge:merge`; `talOS/node` on `talOS/hardware` through `talOS/configuration` and `talOS/introspection`); **[gap: ARCH-2]** to remove them and make core libraries non-public |
| `talOS/` never depends on a robot, an example or `studio/` | **[enforced]**: `tools/check_layers.py` |
| Nodes share only messages and topic names | **[enforced]** for Bazel dependencies: `tools/check_layers.py` (a node package depends on another only through its `talos_messages` targets or its `talos_cc_shared_node`; `driver`'s `node_test` on `drivetrain` is allowlisted); **[gap: ARCH-2]** for making node libraries package-private |
| `talOS/vision` → `talOS/geometry`, never the reverse | **[enforced]**: `tools/check_layers.py` |
| Python reaches talOS only through `talOS/node_api` | **[enforced]** for the ABI: `//talOS/python:api_parity_test`, `node_api.cc` `static_assert`s; for Bazel dependencies: `tools/check_layers.py`; **[review]** for "reimplements no wire format" |
| Only the devices-layer flush writes `/hw/request/<node>` | **[enforced]** for `publish`/`publish_bytes` and `LintGraph`; **[gap: ARCH-2]** for `detail::NodeAccess` and raw RTMS publishers, which node code can still reach |
| Output setters only inside `update()` | **[enforced]**: they throw elsewhere |
| One writer per topic; naming; schema identity; reader cap | **[enforced]**: `LintGraph` at launch and `--describe-only`, `talos_graph_test`, `talos_messages` at BUILD load, `//talOS/build:topic_grammar_test` |
| `[[subsystems]]` is the only node list | **[enforced]**: `manifest_sync_test`, the `talos_bundle` build action |

## 4. Contract first, one implementation

A change that crosses a layer or a language changes the contract and its
check first, then the implementations:

| Change | Contract edited first | Check that goes red, then green |
|---|---|---|
| A message or topic | the package's `.fbs` and `talos_messages(topics = ...)` | unit-suffix lint; schema hash; `--describe-only`; the Studio bindings diff |
| A node-API feature | `talOS/node_api/node_api.h` | `api_parity_test` (ABI version, exports, return and parameter types, `TALOS_*` constants; `talos.Mode` takes its values from `_abi.py`); then `node_api_test` and `talos_test` exercise the new symbol, one case each |
| A device type | `talOS/hardware` config and wire structs | wire `static_assert`s; codec round trip; `phoenixTest` |
| A UDP frame, slot size or telemetry budget | DATA_FLOW.md and its size assert | the `static_assert`; the Studio decoder test |
| A config key | the node constructor's `param` | the launcher refuses a key no node read; `talos_graph_test` and `--describe-only` on every affected config |
| A naming rule | `talOS/introspection/names.h` | `names_test`; `topic_grammar_test` |

With the contract fixed, per-layer slices are independent
([DEVELOPMENT.md §2](DEVELOPMENT.md#2-plan-and-dispatch-main-session)); without it, one
`task` owns the feature (RULES.md 6).

**One implementation per concept** (RULES.md 7). Search for the concept
before writing it. A second copy is allowed only where languages cannot share
code (Starlark, a build-time check), and then a contract test pins it to the
first (`topic_grammar_test`). Each known copy is a TODO.md item (ARCH-3).
**[review]**

## 5. Budgets

A budget is a number the design stays inside. Each is stated once, in its
owner document, with a check. A change that moves one updates the number and
the check in the same commit (RULES.md 9).

| Budget | Stated in | Check |
|---|---|---|
| UDP frame and payload bytes, device caps, shared-memory slot sizes | [DATA_FLOW.md](DATA_FLOW.md) | **[enforced]** `static_assert`s in `talOS/hardware`, `talOS/protocol`, `talOS/rtms`; **[gap: LINT-2]** the doc's tables are hand-kept |
| Topic name length, readers per topic | NAMING.md | **[enforced]** `names.h`, `LintGraph` |
| `command_timeout_us` range; actuator `period_us` limit | ARCHITECTURE.md "The tick" | **[enforced]** config load refuses |
| Owner timeout | ARCHITECTURE.md "The tick" | **[enforced]** bridge and `SimHardware` share `//talOS/bridge:merge` |
| `update()` allocates nothing | METHODOLOGY.md §10 | **[enforced]** C++ (`AllocationProbe`); **[review]** Python |
| `update()` p99 duration | METHODOLOGY.md §10 | **[gap: SPEC-4]** no benchmark |
| Node hops from driver input to actuator | METHODOLOGY.md §10 | **[gap: LINT-2]** computable from `graph.json` |
| `/hw/request` gap in simulation | the simulation README | asserted in simulation by `startup_test` and `match_rehearsal_test`; no requirement states it (**[gap: SPEC-4]**) |
| Target age at the selecting tick | launcher README "Session measurements" | `session_report --strict` on a recorded session; bench run open (TIME-3) |
| Fast-lane time per target | [TESTING.md §9](TESTING.md#9-lanes-and-ci) | **[gap: LANE-1]** |

## 6. Scope: serve a requirement or go

Every line is something an agent must keep true, so scope is managed, not
accumulated.

1. **Every package and optional feature names the requirement it serves** in
   REQUIREMENTS.md's package scope table. **[gap: TRACE-1]**
2. A package or feature with no requirement gets one of three states, chosen
   by the user:
   - **adopted:** a requirement is written for it;
   - **frozen:** it keeps building but gets no new features, and its README
     says "frozen" in its first line;
   - **deleted:** removed in one commit whose `Why:` records what it did and
     how to recover it (`git revert`).
3. **Two ways to do one thing** is a deletion candidate, by RULES.md 7's
   logic. Open scope questions are TODO.md SCOPE items.
4. **A feature is not started without a requirement** (RULES.md 15). TODO.md
   "Optional work" is the waiting room.
5. **A deletion deletes everything**: code, BUILD targets, config keys,
   tests, docs, TODO.md items, CI steps. Grep the name across the tree before
   committing.
6. A header-only monolith (ARCH-1) or a file over about 1,500 lines is split
   along its seams when next touched for a behaviour change.

## 7. Ambiguities are design debt

A requirement with two acceptance statements, or none, cannot be tested.
Each is resolved by a decision recorded once in the owning doc, then a test.
The open ones are TODO.md's "Open specification questions".

## 8. Fitness functions

Automated checks that keep the architecture honest.

### Existing

| Check | Guards |
|---|---|
| `LintGraph`, `--describe-only`, `talos_graph_test` per config | naming, one writer, schemas, reader cap, target policy, `hw` writers, parameter keys |
| `talos_messages` action, `unit_suffix_test`, `descriptor_gen_lint_test` | unit suffixes, topic grammar, descriptors |
| `//talOS/build:topic_grammar_test` | `contract.bzl`'s grammar equals `names.h` |
| `schema_hash_test`, `schema_hash_py_test`, the hash-checked attach | topics typed by schema identity in both languages |
| `//talOS/python:api_parity_test`, `node_api.cc` `static_assert`s (device kinds, motor modes, Driver Station flags) | the C ABI equals ctypes |
| size `static_assert`s | DATA_FLOW.md byte budgets |
| `manifest_sync_test`, the `talos_bundle` stamp | `[[subsystems]]` is the only node list |
| the Studio bindings diff in CI | checked-in TypeScript equals `telemetry.fbs` |
| `AllocationProbe` cases ([METHODOLOGY.md §10](METHODOLOGY.md#10-performance)) | RULES.md 3, allocation part |
| `publish` refusing `/hw/request`; setters throwing outside `update()` | RULES.md 2 |
| config load refusals (timeouts, periods, singular sections, soft limits) | "The tick" numbers |
| `wpilib_lockstep_test` | GradleRIO and RobotPy wheel versions agree |
| `new_subsystem_test` golden packages | the scaffold's output builds and passes |
| `tools/check_layers.py` (`bazel query`, CI Linux job) | ARCHITECTURE.md "Allowed dependencies"; its allowlist only shrinks |
| `check_runfiles_collisions.py`, read-only sources and "checkout unchanged" in CI | the build never writes the source tree |
| `clang-format-18`, `clang-tidy-18`, `buildifier` on changed files | style and bug-prone patterns |
| `input_replay_test`, `replay_inputs_test`, the `*Binary` replay cases | a node is a function of its inputs and their times |

### To add

The checks the **[gap]** marks name are TODO.md's "Method enforcement and
test corrections" items, plus ARCH-2 and ARCH-3.

A new **[review]** rule is accepted only with a sentence saying why it
cannot be enforced.
