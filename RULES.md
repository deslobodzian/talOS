# talos hard rules

Every session and subagent loads this file (AGENTS.md imports it). A rule here
is not traded for convenience: if a task seems to need breaking one, stop and
ask. ARCHITECTURE.md and talOS/NAMING.md hold the contracts these rules point
at; they are not restated here.

## Robot safety

Breaking one of these is a review blocker.

1. Actuation is gated on `robot_enabled(node)` and a live target
   (`talos::Target::get()` non-null). No motor keeps driving after disable, on
   hardware state older than `command_timeout_us`, or on a target that is not
   `fresh()`.
2. Hardware requests leave only through the devices-layer flush after
   `update()`. Never write `/hw/request/<node>` directly or send a request from
   outside `update()`.
3. `update()` does bounded work: no blocking I/O, sleeps, waits, or
   allocation in a loop that can overrun `period_us`.
4. Never set `commissioned = true` or change physical CAN ids in a robot
   config; a person does that at the robot.

## Contracts

5. Layers depend only downward (rtms → events → node → devices → robot code)
   as ARCHITECTURE.md "Allowed dependencies" states; `talOS/` never depends on
   a robot, an example or `studio/`. New code goes where "Where a new thing
   goes" says; do not invent a layer or package convention.
6. A cross-layer feature lands whole in one change: a node-API feature in
   `talOS/node`, `talOS/node_api` (C ABI) and `talOS/python/talos`; a device
   type in `talOS/hardware`, `talOS/devices`, `talOS/python/talos/devices.py`
   and the RoboRIO program.
7. One implementation per concept: framework features (`Node`,
   `StateMachine`, devices, `Sim`) are written once in C++ and wrapped for
   Python through `talOS/node_api`; a C++ node used from Python goes through
   `TALOS_EXPORT_NODE` and `talos.cpp_node`. Adding or changing a node in one
   language never requires editing another: cross-cutting wiring (target
   lanes, `driver` / `teleop`, `telemetry`) is TOML.
8. Messages are FlatBuffers structs; topic names come from the package's
   `talos_messages(topics = ...)` and follow NAMING.md. One writer per topic:
   each producer of a target writes its own `/<sub>/target/<source>` lane with
   `talos::TargetLane`, and the consumer reads them only through
   `talos::Target` (a plain subscription skips the mode, release and freshness
   rules).
9. A message size or layout change updates every C++, Python and Studio
   consumer in the same change (Studio bindings: `pnpm generate`). A change to
   a UDP frame, payload codec, device cap, shared-memory slot size or
   telemetry frame budget also updates `docs/DATA_FLOW.md` and its matching
   compile-time size assert.

## Verification

10. New node behaviour gets the node test matrix
    ([docs/TESTING.md §7](docs/TESTING.md#7-the-node-test-matrix)): every
    applicable case, or its stated n/a.
11. Before each commit the main session runs, after integrating all slices,
    the tests the change affects and launcher `--describe-only` on every
    affected config ([docs/DEVELOPMENT.md §5](docs/DEVELOPMENT.md#5-verification-ladder)).
    Before each push it runs, from a clean worktree of the commits to be
    pushed (`tools/prepush.sh` does this): `bazel build //...` and `bazel test --test_output=errors //...`;
    launcher `--describe-only` on every affected config for topic, node or
    config changes; `./gradlew phoenixTest` and `./gradlew buildRio` for
    RoboRIO program changes; `pnpm test` and `pnpm build` for `studio/`
    changes. Subagents do not run these unless their brief names one narrow
    target. Report a failing check with its output; never call unverified
    work done.
12. Tests are few and targeted: each guards a behaviour, boundary, error or
    safety invariant a consumer would notice breaking. No tests of wording,
    wiring, forwarding, incidental defaults or tautologies, and no second test
    of the same path; delete such tests when found. Never fix a test by
    raising a timeout or deleting an assertion that guards real behaviour.
    A flake is a defect: no `flaky`, no retries, no rerunning to green, and
    never widen a bound a requirement states. Method and examples:
    [docs/TESTING.md](docs/TESTING.md) §4 and §8.

## Documentation and scope

13. Documentation describes the code as it is now. A change to behaviour, an
    interface, a flag, a config key, a message, a wire format, a command or a
    number that any doc states updates every such doc (READMEs, `docs/`,
    ARCHITECTURE.md, NAMING.md, code comments) in the same commit. A statement
    that cannot be kept true is deleted, not left stale. Docs hold no plans,
    handoffs or history: open work goes in TODO.md, reasoning in the commit
    log. State each fact once and link to it rather than copying it.
14. One commit is one logical fix (format in AGENTS.md). Problems found
    outside the task are reported, and recorded in TODO.md if left open, not
    folded into the current commit.
15. A feature or fix names the requirement it serves
    ([docs/REQUIREMENTS.md](docs/REQUIREMENTS.md)). Code that serves none is
    not written; see [docs/DESIGN.md §6](docs/DESIGN.md#6-scope-serve-a-requirement-or-go).
