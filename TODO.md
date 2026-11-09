# Competition-readiness backlog

Scope: the reusable FRC platform needed to build robot logic reliably: actuation safety, transport, runtime/API contracts, sensing, deployment and diagnostics. Game strategy, shot timing, mechanism coordination rules and additional autonomous routines are not readiness requirements here.

This is an **open-work list**, not a changelog. Completed work and its reasoning remain in `git log`. Remove an item only after its acceptance check passes; a passing existing suite does not close a newly identified coverage gap. Physical acceptance follows [docs/BRINGUP.md](docs/BRINGUP.md).

## Assessment and evidence

Five GPT-6.1 Sol medium source audits covered runtime/APIs, hardware/transport, deployment/configuration, estimation/motion and observability. Ratings below are the auditors' subjective judgments at audit time (before the fixes below), not certification or a percentage of tests passed.

| Area | Architecture / 10 | Implementation / 10 |
|---|---:|---:|
| Runtime, C ABI and Python | 7.5 | 7 |
| Hardware, transport and safety recovery | 8 | 7 |
| Configuration, deployment and supervision | 7 | 6 |
| Geometry, vision and reusable motion | 8 | 7.5 |
| Telemetry and operator diagnostics | 8 | 7 |

**Competition readiness remains unqualified until the physical gates below pass.** As of 2026-11-09 every software item the audits raised is fixed; MAC-1 awaits a macOS CI run; each closed item has a regression test that reproduces its failure path (see the table). No new physical unsafe actuation was demonstrated by the audit.

Evidence labels:
- **Reproduced:** observed in a probe of current implementation.
- **Source:** concrete source-derived failure path; integrated failure not reproduced.
- **Incident:** previously observed failure with unresolved cause.
- **Hardware:** requires the deployment computer, robot, controllers or radio.

Verification of the closing state (2026-11-09, Linux): `bazel build //...`, `bazel test //...` 151/151; `./gradlew phoenixTest buildRio`; `--describe-only` 13/13 on robot/robot_field, 15/15 physics sim (telemetry 133 channels, 1192/1200 B), 4/4 on both bench configs, 5/5 superstructure; Studio `pnpm test` 316/316 and build; match rehearsal 3/3. Sanitizer suites and macOS run in CI (PR #3).

## Closed in the 2026-11-08 pass

`git log` holds each Problem/Fix/Why/Tests.

| Item | Commit | Regression evidence |
|---|---|---|
| HW-1 watchdog neutral refreshed every tick until a fresh command | `d74d32b` | gateway_test fails before (no neutral after the timeout tick) |
| HW-2 DS consumers follow a RIO cold reboot; old boot retired | `7aa5bdf` | devices_test and target_test fail before; DS frame 40 B fixed / 678 B max (DATA_FLOW.md) |
| HW-3 reordered / retired-boot State dropped before any mutation | `5197cff` | hardware_node_test fails before |
| HW-4 device-local send failure isolates its chain; global failure still latches | `c1a8068` | gateway_test; phoenixTest |
| PROTO-1 ACK sequence no longer leaks into frame flags | `059f3ae` | endpoint test |
| RT-1 Sim dispatch on one chronological frontier; no future publication delivered | `0a554d7` | C++ and Python, both insertion orders |
| RT-2 Python wrappers keep their native Sim owner alive | `798eb74` | isolated-subprocess teardown test |
| RT-3 exact `--replay` keeps byte publications offline | `6d221c5` | sentinel-publisher replay test |
| EST-1 synchronized cameras count as independent evidence | `2fa8bce` | both backends, fails before |
| TIME-1 per-signal acquisition times in C++/C ABI/Python; drivetrain uses the oldest, rejects >50 ms age or >20 ms skew | `02b643a` | SimHardware-injected ages |
| OPS-1 enable-boot provisions service-owned logs and checks bundle access | `8affdd9` | dry-run output; privileged run is FIELD-5 |
| OPS-2 per-node byte caps from `log_max_bytes`, capped status in the manifest, streaming shutdown counts, ENOSPC-safe manifest | `4da2268`, `1d3cec2` | tiny-budget launch: 577,379 / 600,000 B |
| OPS-3 critical nodes exiting 0 mid-session restart | `527ae71` | supervision test fails before |
| OPS-4 explicit systemd `TimeoutStartSec` from a documented probe budget | `e61a363` | docs/DEPLOYMENT.md |
| OPS-5 atomic `current` selection; restart activates | `d20d169` | 69,692 concurrent reads over ten rollbacks, none missing |
| CFG-1 one config interpreter: C++ resolver CLI feeds sync_manifest / new_subsystem | `fa407ce`, `92b2659` | all fixtures; five manifests unchanged |
| VISION-1 nodes register before construction; 30 s bounded construction phase | `e709a2d` | slow, hung and failing constructor tests; Mini cold-start timing is FIELD-3 |
| VERIFY-1 AprilTag union-find and OpenCV worker-flag races reproduced and patched; exemptions removed | `d3a4527`, `ddcdc2f`, `e5cab51` | TSan reproductions with normal parallelism |
| OBS-1/2/3 Studio never shows cached, stale-graph or dead-worker readings as current | `25ae963`, `967a4d9`, `0fb4a3a` | Studio tests; live browser check |
| OBS-4 telemetry prints its headroom at every launch/describe | `b0ff0bb` | robot 130 ch 1168/1200 B; physics sim 133 ch 1192/1200 B (1 spare) |
| Studio blank page on live connect (uint32 dictionary id); bridge `--describe` refused by the launcher | `ca72565` | store/describe tests fail before; found by the live smoke |
| Studio reported a new launcher session's processes as restarts | `03eb210` | events.test |

## Closed in the 2026-11-09 pass (bench preparation)

| Item | Commit | Evidence |
|---|---|---|
| TIME-2 clock mapping has a real error bound: causal two-way probes, `clock_offset_uncertainty_us` / `clock_min_round_trip_us` in `/hw/status`, sample times valid only with a live exchange (protocol v3) | `146e78a` | old estimator: 3000 µs error at zero spread; bound contains true error under varied offset/delay/jitter/asymmetry/±1000 ppm |
| RADIO-1 Studio viewers capped (default 4, `[studio] max_clients`), 160 kB/s shared WebSocket budget, stalled client isolated | `ea92e93` | bandwidth_test fails before; measurements in studio/bridge README |
| VERIFY-2 AprilTag JPEG decoder safe on corrupt input; last sanitizer exemption removed | `ee6c35b` | corrupt-scan corpus fails before (index 64 out of bounds) |
| Supervisor stall false positive: a 300 ms pause plus 250 ms-old progress crossed the 500 ms floor and restarted a healthy node | `f3df8a1` | registry_test fails before; found by the match rehearsal |
| Match rehearsal: full 2026 graph through disabled/auto/teleop, SIGKILL of drivetrain, SIGSTOP of the driver, asserted from recorded data | `2c384ad` | 3/3 |
| Motor `temperature_c` and `supply_current_a` end to end (protocol v4; int16 tenths; State max 1388 B) | `0bbc8ff` | C++/Python/RIO tests; phoenixTest |
| Bench robot `bench/` (one Kraken), `docs/BENCH.md`, `session_report` tool | `ef12ba1`, `391c940`, `d829840`, `654aa81` | sim smoke; synthetic and real sessions |
| macOS CI failures (bridge UDP receive barriers, replay dispatch index, OpenCV GCD thread count, accuracy comparison schedule) | `7ed7329`, `7fb0a9a`, `781a4c7`, `8b1c637` | Linux under 3 constrained CPUs; macOS rerun pending (MAC-1) |
| CI clang-format job used version 18; local checks used 23 | `23ce6b3` | 46 files reformatted; use `pip install clang-format==18.1.8` |

The RoboRIO program must be redeployed with this branch: protocol v4 refuses older State layouts.

## Open software items

- [ ] **MAC-1 — CI green on this branch, macOS included.** **Reproduced (CI).** PR #3 runs 42565601869 and 42568853273 failed tests that pass locally: BSD loopback receive timing, a rehearsal that read a SIGKILLed recorder, a vision test with negative stamps, an ASan fixture race, an ASan calibration timeout and a host-dependent fusion premise. Each has a commit with a local reproduction; the next run must confirm. Two failures were infrastructure (Gradle wrapper and buildifier downloads, HTTP 504).
- [ ] **LINT-1 — clang-tidy on branch-changed files.** **Reproduced (CI).** The clang-tidy job checks every file the branch touches, whole-file: 229 findings (90 unchecked optional access, mostly in tests after `ASSERT_TRUE`; 57 missing braces; 22 narrowing; 22 string concatenation; 17 enum-cast-out-of-range; a use-after-move and two empty catches worth reading first). Either fix them or check only changed lines (`clang-tidy-diff`); that is a CI policy decision.
- [ ] **TELEMETRY-1 — One spare channel on the physics-sim graph.** **Measured.** Any new state scalar needs an exclude (or a frame-format change) in the same commit; an overflow already fails `talos_graph_test`. Multi-frame scheduling only if essential channels need it.
- [ ] **TIME-3 — Tick-time target age is not recorded.** **Source.** `session_report` measures receipt age but not the age of the target a consumer used at its tick (BRINGUP "about 25 ms at most"); that needs a logged per-tick selected-target timestamp.

## Unresolved incidents — investigate, do not declare fixed by a rerun

- [ ] **INC-1 — Attribute tracked Python package truncation.** **Incident; cause unknown.** Earlier work recorded two zero-byte `talOS/python/talos/__init__.py` events and a same-commit CI import-failure/rerun-success pattern. This audit found no responsible source writer. Preserve write/truncate process/path/inode evidence during future affected runs, preferably OS audit instrumentation or a protected checkout. Since `78cbf61` the Linux CI build job runs `bazel test //...` on a read-only checkout and fails if tracked sources change, so the writer will fail with a traceback naming itself. Close only after attribution and a workflow regression; do not blame package generation or infer runtime corruption without evidence.
- [ ] **INC-2 — Bound Phoenix simulated-device readiness.** **Incident.** `PhoenixBackendPigeon.MountYawConfiguredAndInversionFlipsReadYaw` previously failed once refreshing Pigeon config with stale/missing CAN data, then passed unchanged. If it recurs, preserve timing and establish a bounded readiness condition that still fails genuine config errors; do not hide it with unconditional retries. Linux success does not establish macOS SDK/toolchain readiness.

## Physical and deployment qualification gates

These are separate from software fixes, not evidence that a failure already exists. Agents must not change physical CAN IDs or set `commissioned = true`.

The one-Kraken bench ([docs/BENCH.md](docs/BENCH.md), config `bench/`) can close the link/stop/timing parts of FIELD-1, FIELD-2, FIELD-3 and all of FIELD-5 before the robot exists; it cannot qualify a full CAN bus, cameras or the drivetrain. Run `session_report --strict` on every bench session.

- [ ] **FIELD-1 — Network, electrical configuration and commissioning.** Verify Mini `10.56.87.11`, RIO `10.56.87.2`, field bundle selection, CAN IDs, encoder offsets, inversion and controller limits against the real robot. A responsible human commissions only after restrained disabled/enable tests; update configuration golden checks alongside deliberate hardware changes.
- [ ] **FIELD-2 — Independent stop and fault measurements.** Measure DS disable/e-stop, companion stop/stall, Ethernet removal, RIO cold reboot, single Talon/CANcoder loss and total bus loss. Confirm affected-output/dependency isolation and safe fresh recovery. Measure voltage/current or independent motion/power evidence, not merely requested neutral or invalid feedback. Include PWM zero versus pulse-loss behavior and persisted-config boot restoration.
- [ ] **FIELD-3 — Full-bus load and realtime tails.** On the actual coprocessor/RIO, record CAN utilization, per-signal ages, hardware gaps, command latency, loop lateness, callback duration and watchdog crossings under intended load, Python GC, disabled neutral refresh and reset restoration. Voltage/current remain at 200 Hz due to shared Phoenix frames despite 50 Hz requests. Historical desktop p99 numbers are not Mini acceptance evidence; callback duration differs from dispatch lateness.
- [ ] **FIELD-4 — Camera qualification and independent pose accuracy.** Set stable device IDs (currently empty), measured intrinsics/distortion/mounts, actual exposure/readout/timestamp corrections and uncertainty from residuals. Check cold start, USB reconnect, surveyed positions, motion and single-tag heavy tails. Targeted ambiguity regressions exist; they do not prove all outliers removed. Compare fused and odometry-only performance; existing accuracy thresholds do not require heading improvement. Qualify mechanically safe degraded-module driving separately from algebraic observability.
- [ ] **FIELD-5 — Real boot service and rollback.** Build/test on the deployment Mini, install and boot without login as the actual account, verify log permissions/realtime scheduling/descendant cleanup and exercise rollback plus activation. Linux systemd watchdog and macOS launchd are different: `KeepAlive` restarts exits but does not detect a hung launcher. Establish an accepted hung-supervisor recovery policy on the chosen host.
- [ ] **FIELD-6 — Enabled WPILib and platform coverage.** Exercise the enabled WPILib RoboRIO simulator path, not only the lightweight gateway or disabled GUI startup. Verify GradleRIO with the required macOS SDK if that is a supported development host. Test `tools/setup.ps1` install mode on a clean supported Windows/WSL machine if Windows remains supported; `-Check` alone is not installation evidence.

## Optional work — not competition-readiness blockers

- Loaded trajectory files/additional routines, chassis translation/rotation composition and generalized coupled-mechanism permits: implement when a concrete robot contract needs them, not to satisfy this audit. Existing trajectory/controller primitives and target lanes already exist.
- Direction-aware mount covariance and Metal performance tuning: optimize only after measured calibration and target-host timing justify it; CPU is supported.
- Consumer-receipt/drop replay and schema migration adapters: exact dispatch and counterfactual input replay already exist. Exact replay now keeps byte outputs offline (RT-3) but does not verify them.
- Studio live images, command publishing and launcher-session replay: absent features, not broken match-control paths. Studio telemetry-file replay is not launcher dispatch-log replay; avoid conflating their `.tlog` names.
- Extract Studio label/webroot/launch policy from framework configuration/launcher. The policy coupling exists, but there is no Studio build dependency; classify as maintainability, not an established match failure.

## Backlog hygiene and closure rules

Removed historical completed checklists, contradictory merge/uncommitted labels, obsolete receipt-based freshness guidance, duplicate telemetry headroom claims and the headless-browser tooling anecdote. Registry crash recovery, bounded simulated rings/pause injection, publication-age freshness, config/reset isolation and optional-node startup policy are implemented; remaining issues above identify distinct failure paths, not requests to redo those changes.

For each software fix: reproduce its failure, retain a consumer-visible regression where practical, exercise the changed path live, update affected API wrappers/callers/docs, then run `bazel build //...` and `bazel test --test_output=errors //...`. Run launcher `--describe-only` for affected graphs, GradleRIO `phoenixTest` / `buildRio` for RIO changes, and Studio checks plus actual-surface verification for UI changes. Wire/slot/budget changes require `docs/DATA_FLOW.md` and matching compile-time assertions together. Do not turn hardware-only gates into claims of software test coverage.
