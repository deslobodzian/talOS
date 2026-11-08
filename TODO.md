# Competition-readiness backlog

Scope: the reusable FRC platform needed to build robot logic reliably: actuation safety, transport, runtime/API contracts, sensing, deployment and diagnostics. Game strategy, shot timing, mechanism coordination rules and additional autonomous routines are not readiness requirements here.

This is an **open-work list**, not a changelog. Completed work and its reasoning remain in `git log`. Remove an item only after its acceptance check passes; a passing existing suite does not close a newly identified coverage gap. Physical acceptance follows [docs/BRINGUP.md](docs/BRINGUP.md).

## Assessment and evidence

Five GPT-6.1 Sol medium source audits covered runtime/APIs, hardware/transport, deployment/configuration, estimation/motion and observability. Ratings below are subjective engineering judgments, not certification or a percentage of tests passed.

| Area | Architecture / 10 | Implementation / 10 |
|---|---:|---:|
| Runtime, C ABI and Python | 7.5 | 7 |
| Hardware, transport and safety recovery | 8 | 7 |
| Configuration, deployment and supervision | 7 | 6 |
| Geometry, vision and reusable motion | 8 | 7.5 |
| Telemetry and operator diagnostics | 8 | 7 |

**Competition readiness remains unqualified.** Software issues below remain, as do physical commissioning gates. The previous claim that every match-losing software gap was fixed was too strong.

Evidence labels:
- **Reproduced:** observed in a probe of current implementation.
- **Source:** concrete source-derived failure path; integrated failure not reproduced in this audit.
- **Incident:** previously observed failure with unresolved cause; not a newly reproduced defect.
- **Hardware:** requires the deployment computer, robot, controllers or radio.

Priority: **P1** blocks dependable platform operation/authoring or safety qualification; **P2** is a bounded robustness/maintenance gap. No new physical unsafe actuation was demonstrated. In particular, an unchecked neutral-send path is a safety qualification blocker, not proof of continuing motor power.

Current audit probes:
- `./sim.sh --sim-gateway --no-studio --duration-s 12`: 14/14 declared nodes, no restarts/failures; 2,400 hardware states, 1,204 commands, zero subsystem timeouts. This exercised live processes on a local simulated link, not physical actuation or Studio rendering.
- Synchronized-camera probe using the existing estimator fixture and actual native Sim: both Kalman and factor-graph backends accepted 16 observations but stayed unlocalized; alternating camera arrivals localized after consistent evidence.
- Native Sim through Python: a 500 us consumer received a 1 ms producer's publication at consumer time 0.5 ms; `age == -0.0005` and `fresh() == True`. Reversing node insertion order changed delivery.
- Python lifetime probe: collecting Sim while retaining a returned node left non-null native node/subscription handles. The probe deliberately did not dereference freed native memory; a crash was not demonstrated.

## P1 — hardware safety and recovery

- [ ] **HW-1 — Retry and qualify watchdog neutral delivery.** **Source.** `Gateway::Invalidate` sends neutral once and clears `active_`; subsequent enabled, healthy `Gateway::Tick` calls do not retry solely because a command timed out. Phoenix ignores neutral-send results. See `talOS/hardware/gateway.cc` (`Invalidate`, `Tick`) and `2026-robot/controller_processor/rio/src/main/cpp/PhoenixBackend.cpp` (`Neutral`). Disabled/faulted paths already refresh neutral; controller-side command expiry is not established by these tests.
  - Action: bounded retry after a watchdog stop until fresh commands supersede it; expose delivery uncertainty and document controller expiry guarantees.
  - Acceptance: fail/drop the first neutral while DS remains enabled and no new commands arrive; retries occur. Independently measure removal of powered output within a documented bound, including feedback loss. Coast motion alone is not powered output.

- [ ] **HW-2 — Recover Driver Station consumers after a cold RIO reboot.** **Source.** `DriverStationLink::OnPacket` in `talOS/devices/devices.cc` rejects regressing sample/mode timestamps without a boot identity. The bridge resets its own DS ordering gate on transitions, but surviving consumers retain pre-reboot high-uptime timestamps and reject the new low-uptime stream. Program-only restart is not this case.
  - Action: propagate validated boot identity and reset ordering/generation state on a genuine boot transition; reject delayed retired-boot frames.
  - Acceptance: high-uptime boot A → low-uptime boot B recovers without restarting consumers; delayed A frames cannot revive prior controls. Disabled/stale targets remain neutral.

- [ ] **HW-3 — Reject reordered hardware State before mutating bridge state.** **Source.** `HardwareNode::Tick` in `talOS/bridge/node.cc` feeds the clock filter but accepts the decoded State regardless of its ordering result. Old samples/epochs/enable edges can replace current identity, refresh receipt time and invalidate healthy subsystem requests; retired boot frames can trigger config pushes. Gateway command checks limit unsafe acceptance but do not prevent this interruption.
  - Action: establish State ordering and boot-retirement rules before changing clock, freshness, identity, cache or publications.
  - Acceptance: inject duplicate/old samples, old enable/epoch transitions and retired-boot datagrams; current state/requests must not regress. A genuine reboot still recovers.

- [ ] **HW-4 — Isolate device-local control-send failures.** **Source; physical disconnect manifestation unmeasured.** Configure/reset/feedback isolation is implemented, but a failed `SetControl` on an otherwise available motor makes `PhoenixBackend::Apply` neutralize globally; `Gateway::Accept` invalidates the epoch and latches a global fault after three failures. See the RIO `PhoenixBackend.cpp` and `talOS/hardware/gateway.cc`.
  - Action: distinguish per-device/dependency failures from global backend failure; quarantine only the affected chain when safe.
  - Acceptance: repeated send failure for one motor leaves unrelated motor/DIO/PWM owners operational; affected devices stay neutral and recover on fresh state. Genuine global failure still neutralizes all.

## P1 — runtime and subsystem-authoring contracts

- [ ] **RT-1 — Preserve chronological causality in multi-node Sim.** **Reproduced.** `Sim::step` in `talOS/node/testing.cc` advances whole node loops in insertion order within each 1 ms slice. `talOS/events/simulated_event_loop.h` exposes publications already written by a later-time producer to an earlier-time consumer. Negative age is then accepted as fresh.
  - Action: use a shared chronological dispatch frontier with an explicit equal-time ordering; do not merely clamp negative age.
  - Acceptance: 500 us/1 ms producers/consumers in both insertion orders never receive future publications; include pause/resume, overwrite/drop accounting and stale-target cases. Serial Sim still does not prove concurrent RTMS behavior or scheduler jitter.

- [ ] **RT-2 — Tie Python borrowed handles to native owner lifetime.** **Reproduced ownership gap; no crash probe.** `talOS/python/talos/testing.py` destroys native nodes in `Sim.__del__`; returned nodes and wrappers in `talos/__init__.py` / `devices.py` retain raw handles without the Sim owner. `talOS/node_api/internal.h::AsNode` only checks null.
  - Action: retain an owner or propagate a liveness token to borrowed node/subscription/publisher/device wrappers; fail deterministically after teardown if not retaining ownership.
  - Acceptance: retained wrappers remain safe after deleting the Sim variable and collecting garbage. Exercise native teardown in an isolated subprocess; no use-after-free or silent dangling access.

- [ ] **RT-3 — Keep exact replay byte output offline.** **Source.** Exact `--replay` in `talOS/node/node.cc::RunMain` uses the ordinary `LoopPort`; `open_byte_topic` can create live RTMS publishers through `SharedByteTopic`. `--replay-inputs` already discards byte output. Typed replay verification does not cover this side effect.
  - Action: use an offline byte sink in exact replay and document which outputs replay verifies.
  - Acceptance: replay a byte-emitting node while an isolated sentinel publisher owns the same topic; replay neither attaches/writes live byte SHM nor fails because of that publisher. Typed divergence detection remains intact.

- [ ] **EST-1 — Accept independent cameras with equal capture timestamps.** **Reproduced on both estimator backends.** `2026-robot/main_processor/pose_estimator/pose_estimator.py::relocalizes` clears accumulated evidence when `sample_ns <=` the last candidate, even for a different camera. Synchronized batches prevent the three-candidate confirmation needed after enabled-first startup; the same logic handles large-innovation relocalization.
  - Action: track duplicate evidence per camera, permit equal-time independent cameras and handle out-of-order cross-camera arrival without weakening consistency/window checks.
  - Acceptance: synchronized consistent cameras localize after enabled restart and confirm legitimate large-innovation recovery. Repeated `(camera, timestamp)` pairs do not count twice; inconsistent candidates still fail.

## P1 — deployment and supervision

- [ ] **OPS-1 — Provision writable service logs.** **Source; depends on prefix ownership.** `tools/deploy.sh::cmd_install/cmd_enable_boot` selects `/opt/talos/logs` but does not provision it for the selected unprivileged service user. A root-owned fresh prefix can fail session-directory creation before nodes start; temporary writable-prefix bundle tests do not cover this.
  - Acceptance: install into a clean privileged prefix, start as the intended account, and create manifest/logs without manual permission repair. Verify bundle read/traversal access while keeping bundles read-only.

- [ ] **OPS-2 — Bound active-session recording and shutdown memory.** **Source.** `talOS/launcher/launcher.h::ApplyRetention` runs before launch, not continuously against the active session. Long-running logs can exceed `log_max_bytes`; shutdown reads whole logs through `talOS/events/log/log_reader.cc` to count records. Writer failure containment is useful but does not make retention a live disk bound.
  - Action: bounded active recording/rotation with explicit incomplete-recording status, resilient manifest updates, and streaming shutdown accounting.
  - Acceptance: exceed the budget on an isolated small filesystem; storage stays bounded, supervision survives ENOSPC, and shutdown memory does not scale with total log size.

- [ ] **OPS-3 — Restart unexpectedly successful critical processes.** **Source.** `ShouldRestartNode` / `Supervise` in `talOS/launcher/launcher.h` deliberately decline restart after exit code 0, even when a critical long-lived node exits while its peers continue. Service watchdog notifications continue, so the supervisor does not restore it. Existing predicate coverage encodes this policy.
  - Acceptance: an unexpectedly exiting critical fixture with code 0 restarts; requested stop, session deadline and intentional completion do not restart children. Crash/stall backoff and optional-node quarantine remain correct.

- [ ] **OPS-4 — Align Linux service startup timeout with probe deadlines.** **Source; deployment threshold unmeasured.** `tools/deploy.sh` emits a `Type=notify` service without an explicit `TimeoutStartSec`; `DescribeGraph` in `talOS/launcher/launcher.h` runs sequential bounded probes, including graph-reader re-probes. Several slow optional processes can exhaust the inherited startup timeout before READY.
  - Acceptance: a real service with multiple slow optional probes reaches READY within a documented finite startup policy; genuinely stuck startup is bounded. Keep running watchdog notifications tied to supervision progress, not an independent heartbeat thread.

## P1 — trustworthy diagnostics

- [ ] **OBS-1 — Do not revive cached dictionary values as fresh.** **Source.** `studio/bridge/main.cc` sends the cached dictionary on connection; `TelemetryState::dictionary_frame` in `studio/bridge/telemetry.h` includes historical values. `studio/src/worker.ts::ingest` anchors live age to their new arrival rather than their real age.
  - Acceptance: stop telemetry for >2 seconds, connect/reconnect a viewer, and discover channels without ever presenting cached values as current. Fresh value frames recover immediately. Use metadata-only initialization or explicit authoritative cache age.

- [ ] **OBS-2 — Age system graphs independently of telemetry.** **Source.** `studio/src/worker.ts::ingestSystem` and disconnect handling retain previously healthy `node.alive` values; `data/hooks.ts`, System, Health and Robot/status views lack a graph freshness boundary.
  - Acceptance: suppress text graph messages while binary telemetry continues, then separately disconnect. Every live-node conclusion becomes stale/unknown while last-known evidence remains inspectable; malformed graph replacement must not leave unqualified healthy claims.

- [ ] **OBS-3 — Invalidate current readings when the worker fails.** **Source.** `studio/src/data/studio.ts` failure handling updates connection/status but leaves `clock_ns` frozen; `data/hooks.ts::useChannelStale` and `freshness.ts` use that clock, so a fresh sample can stay fresh indefinitely. The visible worker-error banner does not correct those readings.
  - Acceptance: kill/stall the worker immediately after a fresh sample; widgets become stale/unknown without further worker messages. Preserve intentional paused/replay history semantics, RPC failure reporting and replacement-worker isolation.

- [ ] **OBS-4 — Make telemetry capacity visible before deployment.** **Source; current per-config byte counts unmeasured.** `talOS/telemetry/node.h::CheckFrameBudget` correctly rejects value frames above 1200 bytes, but automatic schema growth can make the optional telemetry node fail construction. Historical TODO/DATA_FLOW channel totals do not establish current headroom after later exclusions.
  - Action: report current channel count, value bytes, dictionary bytes and headroom for each supported graph using production measurement paths; attribute excess channels in authoring/config diagnostics. Maintain one measured budget record in `docs/DATA_FLOW.md`.
  - Acceptance: a scalar addition crossing the cap fails an actionable graph/config check before field launch, not first as missing Studio telemetry. Introduce bounded multi-frame scheduling only if essential diagnostics require it; do not assume float32 conversion is necessary.

## P2 — timing, correctness and maintainability

- [ ] **TIME-1 — Expose and consume existing acquisition ages.** **Source.** `talOS/hardware/messages.h/.cc` and RIO `PhoenixBackend.cpp` already carry/populate per-signal ages; `talOS/devices/devices.h` and drivetrain use common mapped State time instead. This is no longer a missing-wire-metadata task.
  - Acceptance: unequal known position/velocity/gyro ages reach C++, C ABI and Python and drive a documented odometry alignment/staleness policy. Add wire fields only if existing metadata cannot express the required measurement.

- [ ] **TIME-2 — Qualify clock-mapping accuracy, not just convergence.** **Source + Hardware.** `talOS/bridge/clock_offset.cc::Observe` estimates from one-way arrivals. Constant transport delay can yield low spread while shifting mapped time; `HardwareLink::sample_time()` documents the one-sided limitation.
  - Acceptance: independently vary offset, constant delay, jitter and asymmetry; measure absolute error/uncertainty against fusion requirements. Use two-way timing or conservative propagated uncertainty if necessary; do not treat spread as an accuracy bound.

- [ ] **PROTO-1 — Correct ACK sequence/flags plumbing.** **Source.** `talOS/hardware/endpoint.h::Send` calls its argument `sequence` but forwards it to the flags argument in `talOS/protocol/udp.h`; ACK callers supply an incoming sequence, setting/truncating unrelated flag bits. UDP assigns its own outgoing sequence.
  - Acceptance: transmitted ACK flags have intentional semantics regardless of incoming sequence; remove misleading plumbing and obsolete flag declarations rather than inventing unused reliability behavior.

- [ ] **OPS-5 — Make bundle selection atomic and activation explicit.** **Source.** `tools/deploy.sh` calls `ln -sfn` for selection despite its atomic-flip claim; rollback changes links, not already-running processes.
  - Acceptance: replace via temporary sibling symlink plus atomic rename; concurrent readers never observe missing `current`. Document and exercise the stop/restart activation step so rollback runs the previous binary/config together.

- [ ] **CFG-1 — Remove duplicate configuration interpretation.** **Source duplication; no current divergence demonstrated.** `talOS/configuration/config_parser.h` and `tools/sync_manifest.py` independently resolve config; `tools/new_subsystem.py` uses the Python resolver. Shared fixtures and manifest freshness checks already protect current behavior.
  - Acceptance: one canonical resolver supplies effective config and dependency closure to manifest/scaffold callers, including pending-content planning and external repositories. Preserve all existing fixtures and remove the duplicate interpreter in the same cutover. Not a reason to block current working configurations.

- [ ] **VISION-1 — Bound and expose pre-registration initialization.** **Source ordering + Incident; Metal causation unproven.** `talOS/node/node.cc::RunMain` constructs the node before its Reporter; vision pipeline/backend construction can therefore consume startup grace without an active row. A historical zero-byte-log camera startup was killed and recovered on restart.
  - Acceptance: repeated cold/loaded starts on the actual Mini record construction-to-registration time for CPU/auto backends; initialization remains observable, bounded and recoverable. Noncritical cameras must not block control. Do not simply disable supervision.

- [ ] **VERIFY-1 — Resolve sanitizer blind spots honestly.** **Source.** `.bazelrc` exempts AprilTag JPEG signed shifts and quad-threshold TSan instrumentation, and serializes OpenCV internal parallel work under TSan. Python-native/full-stack tests also have exclusions. Green filtered suites are not proof these production paths are safe.
  - Acceptance: qualify/fix or upgrade the affected dependency paths, reproduce relevant races with normal parallelism, and document unavoidable coverage limits precisely. Bounded mutation gtests are not sustained coverage-guided fuzzing; add such fuzzing based on exposed-input risk, not a test-count target.

- [ ] **RADIO-1 — Define and verify aggregate viewer bandwidth.** **Source policy gap + Hardware.** `studio/bridge/main.cc` bounds slow-client queues but not simultaneous clients or aggregate egress. Value-only bandwidth is per client; dictionaries, system graphs, retransmits and HTTP also consume radio capacity.
  - Acceptance: measure total egress/age/drops/RSS at the supported client count, plus one stalled/excess client; document the operating limit and enforce it if needed. No arbitrary universal CAN/radio percentage is established by this audit.

## Unresolved incidents — investigate, do not declare fixed by a rerun

- [ ] **INC-1 — Attribute tracked Python package truncation.** **Incident; cause unknown.** Earlier work recorded two zero-byte `talOS/python/talos/__init__.py` events and a same-commit CI import-failure/rerun-success pattern. This audit found no responsible source writer. Preserve write/truncate process/path/inode evidence during future affected runs, preferably OS audit instrumentation or a protected checkout. Close only after attribution and a workflow regression; do not blame package generation or infer runtime corruption without evidence.
- [ ] **INC-2 — Bound Phoenix simulated-device readiness.** **Incident.** `PhoenixBackendPigeon.MountYawConfiguredAndInversionFlipsReadYaw` previously failed once refreshing Pigeon config with stale/missing CAN data, then passed unchanged. If it recurs, preserve timing and establish a bounded readiness condition that still fails genuine config errors; do not hide it with unconditional retries. Linux success does not establish macOS SDK/toolchain readiness.

## Physical and deployment qualification gates

These are separate from software fixes, not evidence that a failure already exists. Agents must not change physical CAN IDs or set `commissioned = true`.

- [ ] **FIELD-1 — Network, electrical configuration and commissioning.** Verify Mini `10.56.87.11`, RIO `10.56.87.2`, field bundle selection, CAN IDs, encoder offsets, inversion and controller limits against the real robot. A responsible human commissions only after restrained disabled/enable tests; update configuration golden checks alongside deliberate hardware changes.
- [ ] **FIELD-2 — Independent stop and fault measurements.** Measure DS disable/e-stop, companion stop/stall, Ethernet removal, RIO cold reboot, single Talon/CANcoder loss and total bus loss. Confirm affected-output/dependency isolation and safe fresh recovery. Measure voltage/current or independent motion/power evidence, not merely requested neutral or invalid feedback. Include PWM zero versus pulse-loss behavior and persisted-config boot restoration.
- [ ] **FIELD-3 — Full-bus load and realtime tails.** On the actual coprocessor/RIO, record CAN utilization, per-signal ages, hardware gaps, command latency, loop lateness, callback duration and watchdog crossings under intended load, Python GC, disabled neutral refresh and reset restoration. Voltage/current remain at 200 Hz due to shared Phoenix frames despite 50 Hz requests. Historical desktop p99 numbers are not Mini acceptance evidence; callback duration differs from dispatch lateness.
- [ ] **FIELD-4 — Camera qualification and independent pose accuracy.** Set stable device IDs (currently empty), measured intrinsics/distortion/mounts, actual exposure/readout/timestamp corrections and uncertainty from residuals. Check cold start, USB reconnect, surveyed positions, motion and single-tag heavy tails. Targeted ambiguity regressions exist; they do not prove all outliers removed. Compare fused and odometry-only performance; existing accuracy thresholds do not require heading improvement. Qualify mechanically safe degraded-module driving separately from algebraic observability.
- [ ] **FIELD-5 — Real boot service and rollback.** Build/test on the deployment Mini, install and boot without login as the actual account, verify log permissions/realtime scheduling/descendant cleanup and exercise rollback plus activation. Linux systemd watchdog and macOS launchd are different: `KeepAlive` restarts exits but does not detect a hung launcher. Establish an accepted hung-supervisor recovery policy on the chosen host.
- [ ] **FIELD-6 — Enabled WPILib and platform coverage.** Exercise the enabled WPILib RoboRIO simulator path, not only the lightweight gateway or disabled GUI startup. Verify GradleRIO with the required macOS SDK if that is a supported development host. Test `tools/setup.ps1` install mode on a clean supported Windows/WSL machine if Windows remains supported; `-Check` alone is not installation evidence.

## Optional work — not competition-readiness blockers

- Loaded trajectory files/additional routines, chassis translation/rotation composition and generalized coupled-mechanism permits: implement when a concrete robot contract needs them, not to satisfy this audit. Existing trajectory/controller primitives and target lanes already exist.
- Direction-aware mount covariance and Metal performance tuning: optimize only after measured calibration and target-host timing justify it; CPU is supported.
- Consumer-receipt/drop replay and schema migration adapters: exact dispatch and counterfactual input replay already exist. Keep this separate from RT-3's live-output isolation defect.
- Studio live images, command publishing and launcher-session replay: absent features, not broken match-control paths. Studio telemetry-file replay is not launcher dispatch-log replay; avoid conflating their `.tlog` names.
- Extract Studio label/webroot/launch policy from framework configuration/launcher. The policy coupling exists, but there is no Studio build dependency; classify as maintainability, not an established match failure.

## Backlog hygiene and closure rules

Removed historical completed checklists, contradictory merge/uncommitted labels, obsolete receipt-based freshness guidance, duplicate telemetry headroom claims and the headless-browser tooling anecdote. Registry crash recovery, bounded simulated rings/pause injection, publication-age freshness, config/reset isolation and optional-node startup policy are implemented; remaining issues above identify distinct failure paths, not requests to redo those changes.

For each software fix: reproduce its failure, retain a consumer-visible regression where practical, exercise the changed path live, update affected API wrappers/callers/docs, then run `bazel build //...` and `bazel test --test_output=errors //...`. Run launcher `--describe-only` for affected graphs, GradleRIO `phoenixTest` / `buildRio` for RIO changes, and Studio checks plus actual-surface verification for UI changes. Wire/slot/budget changes require `docs/DATA_FLOW.md` and matching compile-time assertions together. Do not turn hardware-only gates into claims of software test coverage.
