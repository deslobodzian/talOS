# One-Kraken bench bring-up

Use `bench/` for a RoboRIO, Mac mini and one restrained Kraken X60/TalonFX.
The RIO runs the existing generic gateway; no 2026 robot mechanisms are launched.
Every checked-in bench config is **uncommissioned**. CAN id 1 is a placeholder,
not a claim about the connected motor. Keep a spotter at DS disable and a safe
power disconnect; neutral output does not instantly stop a coasting rotor.

For every trial record operator/date, config hash, Mini/RIO versions, wiring,
DS mode, control inputs, session directory and RIO console output. Stop and
investigate any unexpected motion before proceeding. Disable between trials,
release all controls, and explicitly re-enable only after checking recovery.

## Build, network and deployment

- [ ] Secure the bare motor, remove loose couplings, guard its shaft, provide
  appropriate fused power/CAN termination, and leave DS disabled. Expected:
  no motion or exposed rotating hazards. Record wiring, power supply and guard.
- [ ] Assign Mini `10.56.87.11` and RIO `10.56.87.2`; verify bidirectional
  connectivity. The generic RIO's `kCompanionAddress` is the Mini address.
  Inspect `bench/main_processor/configuration/robot_bench.toml`: it includes
  the base [robot config](../bench/main_processor/configuration/robot.toml)
  and overrides the controller address. Expected: boot needs no `--remote`
  override. Record interface/subnet and connectivity results.
- [ ] On the actual macOS Mini, from the repo root run `bazel build //...`
  and `bazel test //...`. Expected: success on the deployment architecture,
  not merely Linux CI. Record revision, architecture and command results.
- [ ] Run `bazel test --test_output=errors //bench/...`, then
  `bazel run //talOS/launcher -- --config $PWD/bench/main_processor/configuration/robot.toml --describe-only`
  and repeat with `bench/main_processor/configuration/robot_bench.toml`. Expected: valid graphs. Record output.
- [ ] Before hardware deployment, run
  `bazel run //talOS/launcher -- --config $PWD/bench/main_processor/configuration/robot.toml --sim --start-sim-gateway --output-dir /tmp/bench-smoke --duration-s 20`.
  Expected: motor, teleop, telemetry, hardware_node and the generic ideal sim
  gateway remain running until normal shutdown; completed manifest has no
  crashes/restarts or failed logs. Simulation overrides commissioning only in
  memory; it does not prove physical safety or voltage dynamics. Record manifest.
- [ ] From `2026-robot/controller_processor/rio`, run `./gradlew deploy`.
  Expected: generic gateway starts on the RIO and targets the Mini. Record
  deploy result and RIO console banner.
- [ ] Build `bazel build //bench/main_processor:bench_bundle`; on the Mini run
  `./tools/deploy.sh install bazel-bin/bench/main_processor/bench_bundle`.
  Expected: `/opt/talos/current` selects a read-only bundle; previous bundle
  remains available. Record installed version and config hash. Never edit the
  installed bundle; rebuild/reinstall changes.
- [ ] Inspect `./tools/deploy.sh enable-boot --user <service-account> --dry-run`,
  then enable boot with the actual service account. Expected: bundle traversal
  succeeds, `/opt/talos/logs` is writable by that account (0755), bundle remains
  read-only. Record ownership and launchd configuration. Stop boot-managed
  execution before starting a manual launcher: exactly one stack may run.
- [ ] As that account start `/opt/talos/current/run.sh --output-dir /opt/talos/logs`.
  Run `./tools/deploy.sh logs`. Expected: one unique session below the output
  root, with the deployed config hash and each node's log paths. Record paths.

## Uncommissioned and disabled checks

- [ ] Keep `commissioned = false` and exercise controls while DS disabled.
  Expected: RIO console reports `commissioned=0`, `/hw/status.configured` is
  false, and no motor motion occurs. Record console and `/motor/state`.
  Refusal is intentional, not a CAN fault to bypass.
- [ ] Inspect `/hw/status`: receive counters should advance and identity and
  ownership rejection counters should not grow. Expected: low state age once
  frames actually arrive; zero age before the first frame proves nothing.
  Record counters and state age, plus `/motor/state` connected/fault validity.

## Human commissioning and first spin

- [ ] With power safe and DS disabled, a human identifies the one physical
  TalonFX in Phoenix Tuner X, checks firmware/CAN bus/id and inversion, and
  sets the real CAN id/inversion in [the motor config](../bench/main_processor/configuration/subsystems/motor.toml).
  Expected: exactly the intended Kraken resolves; positive demand and feedback
  direction agree after the supervised spin. Record measured id, inversion and
  firmware. Do not reuse an unchecked placeholder.
- [ ] Review supply 30 A, stator 40 A, hardware voltage ceiling 4 V, node
  `max_bench_voltage_v = 2.0`, ratio 1.0, velocity ceiling 30 rps and provisional
  slot0 gains (feedforward 0.12 V/rotor-rps). Expected: these match the bare
  motor and power source. Record reviewed limits; raise deliberately only after
  the first spin, never to hide a fault.
- [ ] Only a human, after those checks, sets commissioning true in the base
  and deployment configs and updates [the golden config test](../bench/main_processor/configuration/config_golden_test.cc) to the measured
  configuration in the same change. Repeat Mini build/test, bundle install and
  launch. Expected: RIO acknowledges configuration, `/hw/status.configured`
  becomes true. Record reviewed change/config hash and RIO acknowledgment.
- [ ] With DS disabled move joystick 0 axis 1 and hold buttons 1/2. Expected:
  neutral mode/source NONE, zero drive, no motion. Release all controls before
  enabling teleop; startup/recovery requires a neutral-input evaluation. Record
  state and physical observation.
- [ ] Enable teleop, move axis 1 slightly (10% deadband, negative axis gives
  positive voltage), then centre and disable. Expected: VOLTAGE/TELEOP only
  while deflected, the deadband-removed/rescaled axis fraction times
  `max_bench_voltage_v`, bounded to ±2 V; release and disable neutralize.
  Record applied voltage, direction, position,
  velocity and stator current. Hardware limits still clamp all commands.
- [ ] With axis centred, hold button 1 (10 rps), release, then button 2
  (30 rps); release and disable. Expected: VELOCITY/TELEOP while held, neutral
  on release. Button 1 wins if both are held; buttons override axis mode.
  Record requested/measured speed, voltage, current and overshoot. A 4 V ceiling
  may prevent reaching 30 rps under load; do not raise limits automatically.

`/motor/state` exposes position, velocity, applied voltage, stator current,
supply current (`supply_current_a`), controller temperature (`temperature_degc`),
connected status, gateway motor fault with validity, forward/reverse limit
switches (`forward_limit`, `reverse_limit`; false while disconnected or
stale), active mode/source and
hardware/acquisition ages. Supply voltage is not exposed by the public Motor
handle; capture it separately in Phoenix Tuner X. Temperature and supply current
are observations only, not a thermal protection policy. Unknown/stale ages are
UINT64_MAX; `acquisition_valid` requires connected feedback and a converged clock
mapping.
Fault validity is separate from a false fault value. Sample time in this state
is the node observation time; acquisition ages use the mapped hardware clock.

## Fault injection and timing

Use the launcher's recorded PIDs (or Studio's system view); never signal an
unidentified process. Measure actual pause durations rather than trusting shell
sleep precision. Releasing a lane is not a mechanical brake.

- [ ] While applying a small voltage, disable at DS, then repeat in velocity
  mode. Expected: drive always neutralizes; remaining motion is coast-down,
  not renewed demand. Record disable-to-neutral time and motor observations.
- [ ] `kill -STOP <teleop-pid>`, then `kill -CONT <teleop-pid>` after measured
  50 ms and 200 ms pauses, separately. Expected: an expired target neutralizes
  within `command_timeout_us` (100 ms) plus one motor tick (5 ms); the 50 ms
  pause is below timeout, so record whether it continues without assigning an
  invented pass threshold. The 200 ms recovery must require neutral inputs
  before teleop resumes. Record target publication/tick ages and drive timing.
- [ ] Pause the motor node with SIGSTOP for 200 ms, then SIGCONT. Expected:
  gateway per-owner request watchdog neutralizes within command timeout plus
  its tick even though the node cannot execute; record actual delay and timeout
  mask. Disable/release before resuming and verify no unintended motion.
- [ ] While spinning slowly, unplug RIO Ethernet. Expected: gateway watchdog
  neutralizes, Mini feedback becomes stale/disconnected and state age rises.
  Disable and release before reconnecting. Expected after recovery: no
  unintended motion. Record disconnect/neutral/recovery timestamps and counters.
- [ ] Reboot the RIO while the Mini stays running. Expected: neutral during
  outage; DS packets carry gateway `boot_id`, and consumers follow the new boot
  despite restarted low timestamps. Old targets cannot revive motion. Disable
  and release before any re-enable. Record old/new boot ids, clock reset count,
  config acknowledgment and fresh DS/target acceptance after safe recovery.
- [ ] Kill `hardware_node` while spinning slowly. Expected: RIO watchdog
  neutralizes without the bridge; launcher restart policy is visible in the
  manifest. Disable/release before recovery. Record neutral delay, PID attempts,
  restart count and recovery behavior; no automatic motion is acceptable.
- [ ] Stall the RoboRIO loop (its watchdog). Over SSH on the RoboRIO as
  `admin`, record every thread's scheduling class and realtime priority
  (`ps -eLo pid,tid,cls,rtprio,comm`): the robot program's loop thread
  (default, `TS`), `talos_watchdog` (`FF` 30), the CAN and Driver Station
  (netcomm) threads and the interrupt threads; note whether `talos_watchdog`
  sits above the threads that carry CAN frames. While applying a small
  voltage, starve every default-scheduled thread with one `SCHED_FIFO` 20
  spinner per core for a measured 300 ms:
  `for c in 0 1; do taskset -c $c chrt -f 20 timeout 0.3 sh -c 'while :; do :; done' & done; wait`
  (if the image lacks `taskset` or `chrt`, record that and skip). Expected:
  the motor goes neutral within two loop periods plus 5 ms of the command's
  expiry (`command_timeout_us`, 100 ms, after the older of its acceptance and
  the State it was built from) even though the loop cannot run; with a DIO
  output set and a PWM output driven (if the config has them), DIO false and
  PWM disabled; one `talos loop watchdog` Driver Station error per stall with
  the silence and the stall count; late ticks on the RIO console; after the
  spinners exit, the loop takes the outputs back through its own checks.
  Repeat at least 20 times and record every trial's stall-to-neutral time,
  not only the first: a stall that preempts the loop inside a device call
  leaves the watchdog waiting on that device's lock until the spinners exit.
  Also record what the Driver Station showed during the stall (mode, robot
  code, communications) and whether it disabled the robot: this is the
  check of whether a lapse in the program's mode reports disables it, so
  note which threads the listing shows the spinners also starved. Disable
  and release before any re-enable.
- [ ] RoboRIO loop priority. The loop runs at default scheduling. In the
  same enabled session as the representative captures below, record the RIO
  console's late-tick and longest-interval summaries over several minutes
  of driving, then repeat the session and the stall check above with a
  disposable build whose loop thread calls
  `frc::SetCurrentThreadPriority(true, 20)` (below the watchdog's 30) before
  its loop. Record late ticks, Driver Station and CAN health under both;
  the decision is TODO.md RT-5.
- [ ] Capture representative enabled sessions, fetch them (below) and run
  `bazel run //talOS/launcher:session_report -- --strict <session-dir>`.
  Expected: `/hw/state` median about 5 ms, p99 <10 ms, no 100 ms gap while
  enabled (PASS lines); `--strict` exits 2 unless every required verdict
  passes and the manifest shows a finished session with no quarantined,
  recorder-failed or log-capped recording node (Studio is exempt) and no
  damaged log. Keep the
  report with the session. Target age at the tick (from `Target::get` READ
  records) is required to be at most 25 ms on every lane; see
  [session_report](../talOS/launcher/README.md) for its NOT MEASURED and FAIL
  cases.
- [ ] In the same report's `/hw/status` section check `configured`, flat
  rejection counters, `state_age_us`, `timed_out_mask`, and the clock fields:
  `clock_offset_valid` (needs eight States and a live two-way probe exchange),
  `clock_offset_uncertainty_us` and `clock_min_round_trip_us`. Expected:
  mapping valid after the new boot; the uncertainty is the error bound,
  `clock_offset_spread_us` is not. Record the bound and the round trip.
  Review per-node timer lateness in the report; no
  node should log `realtime scheduling incomplete`. Do not use
  `TALOS_REALTIME=0` for acceptance.
- [ ] In a disposable session lower `[robot] log_max_bytes` and exceed each
  writer's share. Expected: control continues, each log keeps its newest
  records in whole-record segments within its share (oldest
  `<stem>.segment<k>.tlog` deleted), completed manifest reports
  `log_capped: true`. Record cap/file sizes and
  manifest, restore normal limits and reserve disk for session metadata.
- [ ] Stop the stack and fetch into a fresh destination with
  `./tools/deploy.sh fetch <archive-directory>`. Expected: complete session
  directories including restarted-node log attempts. Record config hash and
  archive path; retain raw `.tlog` and RIO console logs.
  `bazel run //talOS/launcher:log_dump -- <session-dir>` is the session decoder
  and prints timestamped records with named payload fields.

## Boot operation

- [ ] Disable/release, stop manual execution, and run
  `./tools/deploy.sh enable-boot --user <service-account>`. Inspect
  `sudo launchctl print system/org.talos.robot`, then reboot the Mini with
  nobody logged in. Expected: one installed stack starts using the bench
  address, logs are writable without repair, config hash matches, and the motor
  remains neutral while disabled. Record launchd status, boot session and
  disabled observations. For regression use `./tools/deploy.sh rollback`,
  stop/restart, and recheck the selected whole-bundle hash before use.
