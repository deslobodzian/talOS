# Competition-readiness backlog

Scope: the reusable FRC platform needed to build robot logic reliably: actuation safety, transport, runtime/API contracts, sensing, deployment and diagnostics. Game strategy, shot timing, mechanism coordination rules and additional autonomous routines are not readiness requirements here.

This is an **open-work list**, not a changelog. Completed work and its reasoning remain in `git log`. Remove an item only after its acceptance check passes; a passing existing suite does not close a newly identified coverage gap. Physical acceptance follows [docs/BRINGUP.md](docs/BRINGUP.md).

## Open software items

- [ ] **LINT-1 — clang-tidy CI green on this branch.** The job now analyzes changed production translation units only, with bug-finding checks (`.clang-tidy`); a local clang-tidy 18 run of that selection reports no findings. Close when the CI job passes.
- [ ] **TELEMETRY-1 — Preserve telemetry headroom.** Review the configured exclusions before adding state scalars. The telemetry constructor enforces the value-frame budget; transport limits live in [docs/DATA_FLOW.md](docs/DATA_FLOW.md).
- [ ] **TIME-3 — Tick-time target age is not recorded.** `session_report` measures target receipt age for MESSAGE dispatches, not the selected target's age at a consumer tick. Add per-tick evidence before claiming tick-time target-age acceptance.

## Unresolved incidents — investigate, do not declare fixed by a rerun

- [ ] **INC-1 — Attribute tracked Python package truncation.** **Incident; cause unknown.** Earlier runs observed zero-byte `talOS/python/talos/__init__.py` files. Preserve write/truncate process/path/inode evidence during affected runs. The Linux CI build job makes tracked files read-only before tests and checks for tracked-source changes afterward. Close only after attribution and a workflow regression; do not infer the writer from a passing rerun.
- [ ] **INC-2 — Bound Phoenix simulated-device readiness.** **Incident.** `PhoenixBackendPigeon.MountYawConfiguredAndInversionFlipsReadYaw` previously failed once refreshing Pigeon config with stale/missing CAN data, then passed unchanged. If it recurs, preserve timing and establish a bounded readiness condition that still fails genuine config errors; do not hide it with unconditional retries. Linux success does not establish macOS SDK/toolchain readiness.

## Physical and deployment qualification gates

These are separate from software fixes, not evidence that a failure already exists. Agents must not change physical CAN IDs or set `commissioned = true`.

Use the one-Kraken bench ([docs/BENCH.md](docs/BENCH.md), config `bench/`) for link/stop/timing qualification before the robot exists; it cannot qualify a full CAN bus, cameras or the drivetrain. Run `session_report --strict` on bench sessions; `NOT MEASURED` is not a pass.

- [ ] **FIELD-1 — Network, electrical configuration and commissioning.** Verify coprocessor/RIO addressing, field bundle selection, CAN IDs, encoder offsets, inversion and controller limits against the real robot. A responsible human commissions only after restrained disabled/enable tests; update configuration golden checks alongside deliberate hardware changes.
- [ ] **FIELD-2 — Independent stop and fault measurements.** Measure DS disable/e-stop, companion stop/stall, Ethernet removal, RIO cold reboot, single Talon/CANcoder loss and total bus loss. Confirm affected-output/dependency isolation and safe fresh recovery. Measure voltage/current or independent motion/power evidence, not merely requested neutral or invalid feedback. Include PWM zero versus pulse-loss behavior and persisted-config boot restoration.
- [ ] **FIELD-3 — Full-bus load and realtime tails.** On the actual coprocessor/RIO, record CAN utilization, per-signal ages, hardware gaps, command latency, loop lateness, callback duration and watchdog crossings under intended load, Python GC, disabled neutral refresh and reset restoration. Qualify achieved signal rates rather than assuming the requested rates. Desktop measurements are not Mini acceptance evidence; callback duration differs from dispatch lateness.
- [ ] **FIELD-4 — Camera qualification and independent pose accuracy.** Set stable device IDs (currently empty), measured intrinsics/distortion/mounts, actual exposure/readout/timestamp corrections and uncertainty from residuals. Check cold start, USB reconnect, surveyed positions, motion and single-tag heavy tails. Compare fused and odometry-only performance, including heading. Qualify mechanically safe degraded-module driving separately from algebraic observability.
- [ ] **FIELD-5 — Real boot service and rollback.** Build/test on the deployment Mini, install and boot without login as the actual account, verify log permissions/realtime scheduling/descendant cleanup and exercise rollback plus activation. Linux systemd watchdog and macOS launchd are different: `KeepAlive` restarts exits but does not detect a hung launcher. Establish an accepted hung-supervisor recovery policy on the chosen host.
- [ ] **FIELD-6 — Enabled WPILib and platform coverage.** Exercise the enabled WPILib RoboRIO simulator path, not only the lightweight gateway or disabled GUI startup. Verify GradleRIO with the required macOS SDK if that is a supported development host. Test `tools/setup.ps1` install mode on a clean supported Windows/WSL machine if Windows remains supported; `-Check` alone is not installation evidence.

## Optional work — not competition-readiness blockers

- Loaded trajectory files/additional routines, chassis translation/rotation composition and generalized coupled-mechanism permits: implement when a concrete robot contract needs them.
- Direction-aware mount covariance and Metal performance tuning: optimize only after measured calibration and target-host timing justify it.
- Consumer-receipt/drop replay and schema migration adapters.
- Studio live images, command publishing and launcher-session replay.
- Extract Studio label/webroot/launch policy from framework configuration/launcher.

