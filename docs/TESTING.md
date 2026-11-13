# Testing

How talos tests are chosen, written, placed and run. [RULES.md](../RULES.md)
10 to 12 are the hard rules; this file is what they point at. The
`Sim`/`SimHardware` API is in
[NODE_DEVELOPMENT_GUIDE.md](NODE_DEVELOPMENT_GUIDE.md#testing). Status marks
(**[enforced]**, **[review]**, **[gap]**) are defined in
[METHODOLOGY.md](METHODOLOGY.md); a **[gap]** names the TODO.md item that adds
its check.

## 1. What a test is for

A test exists to turn red when a behaviour a consumer relies on breaks. A
consumer is another node, the bridge, the RoboRIO, the launcher, Studio, an
operator, or a person reading a log after a match. The test's name states
that behaviour (`StaleLaneGoesNeutral`, not `TestUpdate2`).

Two goals, both required:

- **Meaningful:** every SAFE and CONTRACT requirement in
  [REQUIREMENTS.md](REQUIREMENTS.md) has a test that fails when it is broken
  (§3).
- **Fast and reliable:** a red run means a defect (§8). A test that cannot
  be both is moved down a tier, moved to a slower lane, or deleted; it is
  never retried or loosened.

## 2. Tiers: by what the test may touch

A tier is defined by the resources a test uses, not by its subject. Use the
**lowest tier that can observe the behaviour**: each tier up costs about ten
times the time and adds a source of nondeterminism.

| Tier | May touch | May not touch | Clock | Typical cost |
|---|---|---|---|---|
| **unit** | one function or class, in process | threads, files outside `TEST_TMPDIR`, sockets, shared memory, child processes | none or injected | < 10 ms |
| **component** | several real modules in one process: a node under `testing::Sim` + `SimHardware` (`talos.testing`), RTMS in one process, `Gateway` with a sim backend, a codec through real buffers | child processes, sockets, wall-clock waits | simulated (`sim.step`) or injected | < 100 ms |
| **contract** | an interface between parts that change independently: wire bytes, schema hashes, C ABI against ctypes, generated code against its source, config and graph lint, golden files | timing | none | < 1 s |
| **integration** | real OS resources across threads or processes: fork/exec, UDP loopback, multi-process shared memory, signals, the realtime scheduler | the full robot graph | real; waits on **events** with a deadline that only bounds a hang (§6) | 0.1 to 10 s |
| **e2e** | the launcher running a whole config against `sim_gateway` or the WPILib RoboRIO program, judged from the recorded session | nothing | real | 10 to 120 s |
| **soak** | e2e, long, under injected kills and stalls, seeded | nothing | real | minutes |
| **fuzz / property** | a decoder, parser or invariant over generated or corpus input | I/O other than the corpus | none | bounded corpus < 1 s |
| **perf** | a **wall-clock** bound: latency, lateness, throughput, restart time | other load on the host (§6) | real, measured | seconds |

- **Regression is an origin, not a tier.** A regression test pins one past
  failure at the lowest tier that reproduces it (§5).
- **An allocation count is not a perf test.** `AllocationProbe` counts on a
  simulated clock: component tier.
- **A bounded fuzz corpus is a fast test**; open-ended fuzzing is not.
- **One target, one lane.** A case that needs a slower tier (a node binary
  spawned for replay, a UDP case in a logic test) goes in its own target, or
  it drags the whole target into the slower lane.

### Where a behaviour is tested

| Behaviour | Tier | Reason |
|---|---|---|
| A node's decisions, gating, state, parameters | component (`Sim`) | the simulated clock makes stale and fresh exact |
| Target selection, mode generations, arming | component, in `talOS/devices` | written once in the framework; a node proves only that it uses `Target` (the matrix, §7) |
| Owner timeout, request merge, boot and epoch rejection | component on `//talOS/bridge:merge`, on `HardwareNode` through its in-process gateway (`//talOS/bridge:trust_test`), and on `SimHardware` | the bridge runs the same code; UDP adds nothing, and `//talOS/bridge:node_test` proves the socket reaches the same frame handler |
| Feedback age bound | unit on `FeedbackFresh` (`talOS/hardware/gate.h`); a node sees it through `SimHardware`, which calls it | the gateway and `SimHardware` run the same function |
| Wire layout, slot size, ABI, grammar | contract + compile-time `static_assert` | a size change must fail the build (RULES.md 9) |
| Supervision: restart, quarantine, deadlines | component with an injected clock for the policy; integration for one real spawn or kill per mechanism | a policy test should not wait out a real deadline |
| No node moves a motor while disabled, whole graph | e2e, judged from the log | only the session log sees every process |
| A latency number a requirement states | perf, on a qualified host (§6) | a shared runner cannot hold it |

## 3. Requirements and traceability

A test that guards a requirement carries its ID in a comment on the line
above it: `// REQ-SAFE-09` (C++), `# REQ-SAFE-09` (Python),
`// REQ-SAFE-09` (TypeScript). The `REQ-` prefix keeps catalogue IDs apart
from TODO.md item IDs (`REQ-RT-01` is not `RT-1`). The catalogue never lists
test names; the tags are the link. A tag line starts with its first ID
(`// REQ-SAFE-01, REQ-SAFE-02: why` tags both; a line of a Python docstring
counts too); an ID later in a sentence is prose, not a tag.

- Every SAFE and CONTRACT requirement has a tagged test at its cheapest tier
  or below, or names the bench procedure ([BENCH.md](BENCH.md)) or open
  TODO.md item that verifies it. **[enforced: `tools/trace.py --check`
  (CI lint job) for a tagged test or a cited bench section or open TODO.md
  item; known gaps are listed with their TODO item in
  `tools/trace_waivers.txt`, which only shrinks; gap: LANE-1 for the tier,
  which the lane tags (§9) record but nothing checks]** `python3 tools/trace.py --report`
  prints each requirement's tests.
- Most SAFE requirements are provable in process on a deterministic clock
  (the catalogue's last column), so the safety case rests on unit and
  component tests. e2e and soak are **composition evidence**: they do not
  re-assert a bound a lower tier proves (RULES.md 12). The soak owns what
  needs duration: restart count equals kills, nothing quarantined, nothing
  accumulates.
- Not every test needs a tag. A test of local behaviour (a parser boundary, a
  geometry identity) stands on §4. An untagged test whose author cannot name
  the consumer who would notice it break is deleted.
- RT, OPS and DEV requirements may be verified by a session measurement
  (`session_report --strict`) or a bench check. `NOT MEASURED` is never a
  pass.

## 4. What makes a test meaningful

Four checks before keeping a test. **[review]**

1. **Mutation.** Name the one-line change to production code that turns it
   red (drop `robot_enabled` from the `drive` condition; `<` for `<=` at the
   timeout; skip the generation check). If no plausible change does, the test
   proves nothing. For a new safety test, make the mutation once and watch it
   fail.
2. **Oracle strength.** Assert the observable result and its value: the
   motor's `applied_motor(...).mode == kNeutral` and `state.enabled ==
   false`, not "did not throw", "non-null", "size > 0", or a tolerance too
   wide to fail. For safety, assert the **applied** output (`applied_motor`,
   `applied_digital_output`), not the last request.
3. **One behaviour per test.** A failure names the broken behaviour without
   reading the body. The opposite fault is banned too: many tests of one
   validation path are one table-driven test.
4. **Diagnostic failure.** The message carries the inputs: node, device,
   tick or seed. Table rows are labelled (`SCOPED_TRACE`, `subTest`). A
   timing failure prints the measured value, the bound and where the bound
   comes from.

### Banned (RULES.md 12)

| Banned | Why |
|---|---|
| Wording: asserting a sentence of an error or log | wording changes are not regressions |
| Tautology: asserting what the test just set | cannot fail |
| Third-party behaviour (toml++, OpenCV, the Phoenix simulator) | upstream tests it |
| Private detail: caches, internal buffers, helper predicates | pins the implementation |
| Incidental default nobody relies on | not a behaviour |
| Wiring and forwarding: a function that passes its argument on | restates the call |
| A second test of one path | doubles cost, halves signal |
| Testing the fixture or the demo data | proves the demo, not the code |
| A skip that always skips in CI | reports green, runs nothing |

Where RULES.md 12 is read too broadly or too narrowly:

- **An identifier in an error is a contract; the sentence is not.** A bad
  parameter throws naming the key ([METHODOLOGY.md §8](METHODOLOGY.md#8-parameters)):
  assert the exception type and that the message contains the key (or topic,
  node, file). `assertRaises(Exception)` alone is too weak; matching the
  whole text is too strong.
- **A Python test of a C++ feature tests the binding, not the feature.**
  RULES.md 7 puts one implementation in C++, tested once there. The Python
  test proves marshalling, error mapping and lifetime through
  `talOS/node_api`: one case per exported symbol, plus the safety paths
  (stale target, disabled, earlier generation), because a marshalling fault
  there is a safety fault.

## 5. Regression tests

1. **Test first.** Reproduce the failure as a test at the lowest tier that
   shows it, and run it against the unfixed code. It must fail for the
   reported reason.
2. The fix commit's `Tests:` names the test and quotes the red result
   ("expects 0 allocations: 37 before the fix"). The commit is the citation
   (`git log -S'<TestName>'`); the test carries no commit hash.
3. The test's comment states the **scenario** (inputs, state, timing, wrong
   result) and the incident or CI run when there is one. It does not tell
   the history.
4. A failure seen only in e2e, soak or a field log is first reduced: find the
   mechanism, then pin the mechanism at component or integration tier.
   `PollerTest.DeadlineHoldsWhenTimerDeliveryStalls` pins a kqueue stall the
   soak found; the soak is not the regression test.
5. If the failure cannot be reproduced below e2e, the commit says so and
   TODO.md records what would make it reproducible. A passing rerun is not
   evidence.

## 6. Time and determinism

1. **No wall clock below integration.** Unit, component and contract tests
   never sleep, read `steady_clock::now` or `time.time`, or wait on a real
   timer. Nodes use `node.now()`; `Sim` owns time. **[enforced:
   `tools/lint_test_time.py` over every test source; a test of a higher tier
   that uses the real clock is listed with its reason in
   `tools/lint_test_time_allowlist.txt`]**
2. **Logic with a deadline takes its clock as a parameter**, so the policy
   (supervision deadlines, restart windows, retry intervals) is a component
   test in milliseconds and integration keeps one real spawn per mechanism.
3. **Integration tests wait for events, not durations.** Wait on a condition
   (a registry row, a log record, a child's marker line) with a deadline of
   10 s or more that only bounds a hang. The deadline is not the assertion.
   `sleep(100ms); EXPECT(...)` is banned.
4. **Ports are bound at 0** and read back. `exclusive` is for tests that own
   a host-wide resource (the whole-graph e2e tests), not for fixed ports.
5. **Randomness is seeded and printed**, settable from the environment
   (`TALOS_SOAK_SEED`).
6. **A wall-clock bound comes from a requirement.** A perf test or timed e2e
   assertion states, beside the constant, the requirement ID, the number and
   its derivation. Several current bounds (timer lateness, state cadence,
   target age, the simulated request gap) have no end-to-end budget behind
   them (TODO.md SPEC-4). **A bound is never widened to make a run pass**; the requirement
   changes first, in its own commit, with the reason.
7. **Timed tests check their host.** Before judging, a perf or timed e2e test
   verifies its preconditions: realtime granted (`talos: realtime scheduling
   incomplete` fails the macOS sim tests), the host did not sleep, and the
   graph's CPU demand fits the cores. A test that cannot hold its bound on a
   host fails as unqualified, distinctly from a product failure. **[gap:
   LANE-1]**
8. **Statistics over single samples.** A lateness or gap assertion uses the
   quantity the requirement names (the maximum inside a window, or a
   percentile over at least 50 samples), as
   `PollerTest.PeriodicDeadlinesWakeOnTimeWithEpollPwait2` does.

## 7. The node test matrix

Every node gets a `Sim`/`SimHardware` (C++) or `talos.testing` (Python) test
file with these cases, under these names (Python: `test_<snake_case>`).
`bazel run //tools:check_node_matrix -- [PATH...]` reports each node test's
cases as present, n/a or missing and fails on a missing one. **[enforced:
the CI lint job runs it with `--baseline tools/check_node_matrix_baseline.txt`,
a list of known-missing cases that only shrinks and is empty today]**

| # | Case | Arrange | Expect |
|---|---|---|---|
| 1 | `DrivesAtALiveLane` | enabled, teleop, fresh target | the right mode and demand on each actuator; `state` reflects it |
| 2 | `DisabledRobotGoesNeutral` | `hw.enable(false)` | every applied output neutral; `state.enabled == false`; `state` still published |
| 3 | `StaleLaneGoesNeutral` | stop publishing; step past `command_timeout_us` | neutral within one period after the timeout |
| 4 | `ReleasedLaneGoesNeutral` | `ReleaseTarget` / `sim.release_target` | neutral (or the next lane in priority) on the next tick |
| 5 | `NeverSentGoesNeutral` | no target | neutral; no exception |
| 6 | `DriverStationLossGoesNeutral` | `hw.stop_driver_station()` | neutral once the Driver Station view is stale |
| 7 | `DisconnectedFeedbackGoesNeutral` | `hw.set_connected(node, dev, false)`, with another motor or CAN sensor in the test config still valid (losing every one is the gateway's fault, which would neutralize the node without its own gate) | neutral while `robot_enabled` stays true; validity flag cleared |
| 8 | `NonFiniteDemandGoesNeutral` | NaN or infinity in the target | neutral; no NaN in `state` |
| 9 | `BadParameterFailsConstruction` | each required parameter missing or out of range (a table) | the constructor throws; the message contains the key |
| 10 | `OtherModesLaneIsIgnored` | a live lane of another mode's source (autonomous lane in teleop, and the reverse) | neutral, or the current mode's lane |
| 11 | `EarlierGenerationTargetIsIgnored` | a target stamped with the previous generation (`PublishTarget(..., before)` / `sim.publish_target(..., mode_since_us=before)`) landing in the same drain as the re-enabling packet | neutral |
| 12 | `UpdateDoesNotAllocate` (C++) | `AllocationProbe` over steady-state ticks | 0 |

Required cases by role:

| Role | Cases |
|---|---|
| actuator node (owns a motor, DIO or PWM) | 1 to 12 |
| producer (`TargetLane` writer: `driver`, `teleop`, `autonomous`) | 1 as `SendsAtALiveInput`; 2 and 6 as `DisabledRobotReleasesItsLanes` and `DriverStationLossReleasesItsLanes`; 9; 11 as `NewGenerationDisarms`; 12 |
| observer (`vision`, `pose_estimator`) | 1 as `PublishesFromALiveInput`; 3 as `StaleInputClearsValidity` (`state` still published, validity cleared, when its input is stale); 9; 12 |

The lint checks the node packages under the robot, example, bench and
scaffold directories and every package their configs' manifests launch. It
takes the role from the package's sources (a `Motor`, `DigitalOutput`,
`PwmOutput` or a `talos::mechanisms` block owning a motor: actuator; else a
`TargetLane`, `OperatorArming` or `detail::TargetAccess`: producer; else
observer). A
`// matrix: role <role> (reason)` comment overrides it, never away from
actuator; a Python package that only wraps C++ nodes (`talos.cpp_node`) is
checked in its C++ package.

A case that does not apply is a comment in the test file naming why:
`// matrix: n/a 7 (no feedback device)`. Case 12 does not apply to:

- a Python node: a Python tick allocates in the interpreter;
- any build under a sanitizer: `AllocationProbe` counts nothing there,
  because the sanitizer's allocator bypasses `malloc_logger` and cannot sit
  beside a replaced glibc `malloc`; those cases skip and plain builds run
  them.

Cases 2, 3 and 11 guard REQ-SAFE-02, REQ-SAFE-07 and REQ-SAFE-09. The
framework proves the mechanism once (`talOS/devices/target_test.cc`,
`TargetQueuedBeforeTheModeChangeIsRejectedInTheSameDrain` and its
neighbours); a node's matrix proves the node **uses** it. A node that reads
its lane through a plain subscription gated on `fresh()` passes every
framework test and case 3, but fails cases 10 and 11 (and 4, if it ignores
the envelope's `active` flag).

Publish targets with `talos::testing::PublishTarget` / `sim.publish_target`,
with `hw.enable()` and a teleop-enabled Driver Station
(`hw.set_driver_station(DsPacket(ds::kEnabled | ds::kTeleop))` /
`hw.set_driver_station(enabled=True, teleop=True)`). They stamp the mode
generation of the newest Driver Station packet
(`DriverStationGeneration(sim)` / `sim.driver_station_generation`), as a
producer that has drained it would.

`bazel run //tools:new_subsystem` generates the matrix skeleton; its golden
packages (`tools/scaffold_testdata/*`) are the reference, checked by
`new_subsystem_test` **[enforced]**, and every one has its whole matrix
(`check_node_matrix_test` **[enforced]**).

## 8. Flakes

A flake is a defect: in the product, in the test, or in the assumption that
the host can run it. It is never noise.

1. **No retries.** No `flaky = True`, no `--flaky_test_attempts`, no
   rerunning a job until it is green. A rerun is allowed only for a host
   failure the log shows (runner lost, download failed). **[enforced:
   `tools/lint_test_time.py` for `flaky = True` and `--flaky_test_attempts`;
   review for reruns]**
2. **Diagnose from the artifact.** `soak_test` and `match_rehearsal_test`
   write their session directories to Bazel's undeclared test outputs; the
   other e2e and perf tests write theirs to temporary directories that do
   not survive the run **[gap: LANE-1]**. A flake is closed by a named mechanism and a test
   that pins it (§5), not by a green rerun.
3. **Quarantine is explicit and expires.** A test that fails without a
   product change and cannot be fixed the same day gets the tags
   `["manual", "quarantine"]` (so `bazel test //...` skips it) and a TODO.md
   entry with the failure signature, the run, the suspected mechanism, and a
   date at most **14 days** out. On that date it is fixed or deleted. A
   quarantined test that guards a SAFE requirement leaves that requirement
   unverified until it returns.
   **[enforced: `tools/trace.py --check` finds the TODO.md item by the
   target's label and takes its latest date as the expiry]**
4. **Never widen a bound a requirement states** (the owner timeout, the
   stall floor). If the bound cannot hold on a
   host, the host is unqualified for that test (§6.7).
5. **A sanitizer changes time, not truth.** A test that fails only under a
   sanitizer by timeout either has a wall-clock wait (wrong tier) or needs
   `no_asan`/`no_tsan`/`no_macos_sanitizer` with the reason on the same line.

## 9. Lanes and CI

A **lane** is a set of tests run together at one cost. Lanes, by tier:

| Lane | Tiers | Budget per target |
|---|---|---|
| fast | unit, component, contract, bounded fuzz | 5 s |
| heavy | CPU-bound deterministic component or contract (calibration, render-and-detect) | 120 s |
| integration | integration | 30 s |
| e2e | e2e, `exclusive` | 180 s |
| perf | perf, `exclusive` | 60 s |
| soak | soak, `manual` | set by `TALOS_SOAK_SECONDS` |

A target's lane is its tag: `heavy`, `integration`, `e2e` or `perf`. An
untagged test is in the fast lane, so a new test lands there unless its rule
says otherwise; the soak is `manual`. Choose the tag from what the test does:
waits on real time with real processes (integration), runs a whole robot graph
through the launcher (e2e), CPU-bound calibration or render-and-detect
(heavy), measures timing (perf). Other tags only exclude a target from a
configuration (`manual`, `no_asan`, `no_tsan`, `no_macos_sanitizer`),
serialize it (`exclusive`) or run it unsandboxed (`local`).

Each lane has a `.bazelrc` config: `bazel test --config=fast //...` is the
everyday loop; `--config=heavy`, `--config=integration`, `--config=e2e` and
`--config=perf` run one lane, and `--config=no_e2e` every lane but e2e.
`bazel test //...` runs every lane but the soak; it is what runs before a
push ([DEVELOPMENT.md §5](DEVELOPMENT.md#5-verification-ladder)).
Bazel keeps only the last `--test_tag_filters`, so a lane is combined with a
sanitizer in one filter, not two configs:
`bazel test --config=asan --test_tag_filters=-no_asan,-heavy,-integration,-e2e,-perf //...`.

A target that takes more than 5 s states why in a comment on its BUILD
rule: what it waits for in real time and its lane. `python3
tools/slow_tests.py --threshold 5` lists, slowest first, every test action in
`bazel-testlogs` (the last result of each, cached or not, timed to the
millisecond) over the threshold; `--bep FILE` reads one invocation's
`--build_event_json_file` instead, and no test result at all is an error.
With `--check` it exits 1 when an action exceeds its lane's budget above
(lane tags from `--lanes FILE`, the `--bep` file, or `bazel query`, where a
failed query or one finding no lane-tagged test is an error).
**[enforced: the Linux job, after the suite, for the heavy, integration,
e2e and perf lanes; gap: LANE-1 for the fast lane, which a hosted runner
does not hold (the workflow step names the targets) and which is checked
locally only]**

CI policy:

- CI runs on pushes to `main`, `hardening/**` and `feature/**` and on pull
  requests to `main`, so a work branch uses one of those prefixes. A pull
  request is not required.
- Linux carries the full per-push gate: build, suite, sanitizers, lint.
- CI jobs select lanes by the configs above: Linux runs `//...` plain and under `--config=asan` and
  `--config=tsan`; hosted macOS runs only the build and `--config=no_e2e`
  on each push; macOS sanitizers and the macOS soak run nightly.

What each job runs, and on which trigger, is in
[`.github/workflows/`](../.github/workflows/); this file does not restate it.

A workstation run misses what only CI's conditions expose: a clean checkout
with no untracked or uncommitted file, every tracked file read-only (a test
that copies a source with its mode and then writes the copy fails there;
copy contents, `shutil.copyfile`), and test results never cached from an
earlier run. `tools/prepush.sh`, the pre-push rung
([DEVELOPMENT.md §5](DEVELOPMENT.md#5-verification-ladder)), reproduces
them.

## 10. Running tests locally

Which tests to run before a commit or a push is the
[verification ladder](DEVELOPMENT.md#5-verification-ladder). Two more:

| When | Command |
|---|---|
| timing work | the perf or e2e target on an idle host; on macOS under `caffeinate -s` |
| a suspected flake | `bazel test --runs_per_test=50 //pkg:target`, before and after the fix |
