# One-Kraken bench bring-up

Use `bench/` for a RoboRIO, Mac mini and one restrained Kraken X44. The bench
config names only a TalonFX, not a motor model, and its `slot0` feedforward is
provisional for any model.
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
  connectivity. The RIO program's default companion address is the Mini's
  (on another network, see [below](#bench-on-another-network)).
  Inspect `bench/main_processor/configuration/robot_bench.toml`: it includes
  the base [robot config](../bench/main_processor/configuration/robot.toml)
  and overrides the controller address. Expected: boot needs no `--remote`
  override. Record interface/subnet and connectivity results.
- [ ] On the actual macOS Mini, from the repo root run `bazel build //...`
  and `bazel test //...`. Expected: success on the deployment architecture,
  not merely Linux CI. Record revision, architecture and command results.
- [ ] Run `bazel test --test_output=errors //bench/...`, then
  `bazel run //talOS/launcher -- --config $PWD/bench/main_processor/configuration/robot.toml --describe-only`
  and repeat with `bench/main_processor/configuration/robot_bench.toml` and
  `bench/main_processor/configuration/robot_py.toml`. Expected: valid graphs. Record output.
- [ ] Before hardware deployment, run
  `bazel run //talOS/launcher -- --config $PWD/bench/main_processor/configuration/robot.toml --sim --start-sim-gateway --output-dir /tmp/bench-smoke --duration-s 20`.
  Expected: motor, teleop, telemetry, hardware_node and the generic ideal sim
  gateway remain running until normal shutdown; completed manifest has no
  crashes/restarts or failed logs. Simulation overrides commissioning only in
  memory; it does not prove physical safety or voltage dynamics. Record manifest.
- [ ] From `2026-robot/controller_processor/rio`, run `./gradlew deploy`.
  Expected: generic gateway starts on the RIO and its console line
  `[<s>.<ms>] hardware gateway build <commit> companion 10.56.87.11:5803` names the
  Mini. Record deploy result and RIO console banner.
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

### Bench on another network

On a LAN that is not `10.56.87.x`, build the RIO program for the main
processor's address, from `2026-robot/controller_processor/rio`:
`./gradlew deploy -Pcompanion=<main-processor-ipv4>` (also `buildRio`), on
the command line each time: Gradle refuses it from a `gradle.properties` or
the environment, and an empty, hostname or unusable address stops the build. Check that the build's
`RoboRIO companion address built in:` line names it. The address is fixed in the
binary, never learned from the network
([RoboRIO](../talOS/hardware/README.md#roborio)); check the console line names
it. `deploy` still finds the RIO by its team mDNS name
(`roboRIO-5687-FRC.local`). The main processor must accept UDP on port 5803
from the RIO.

To point the main processor at the RIO without editing a tracked file, write
`bench/main_processor/configuration/robot_lan.toml`, listed in
`.git/info/exclude` so it stays untracked:

```toml
include = "robot.toml"

[robot]
controller_address = "<roborio-ipv4>"
commissioned = false
```

It includes `robot.toml`, not `robot_bench.toml`: an include may not itself
include (config load refuses it). Then, from the repo root, build the nodes and
run it without `--sim` (one-off, `--remote <roborio-ipv4>` on the plain
`robot.toml` does the same):

```sh
bazel build //talOS/bridge:hardware_node //bench/main_processor/motor:node //talOS/teleop:node //talOS/telemetry:node
bazel run //talOS/launcher -- --config $PWD/bench/main_processor/configuration/robot_lan.toml --output-dir /tmp/bench-rio
```

Expected: the launcher prints `Controller <roborio-ipv4>:5802`, and
`/hw/status` counts States received. To watch it in Studio, build
`//studio/bridge:studio_bridge //studio:web_dist` too and add
`--start-studio-bridge`; the launcher then runs the
[Studio bridge](../talOS/launcher/README.md#studio-bridge) at
`http://<main-processor>:5800`.

### Bench with the Python motor node

[`robot_py.toml`](../bench/main_processor/configuration/robot_py.toml) is
`robot.toml` with the motor node swapped for its Python twin,
[`motor.py`](../bench/main_processor/motor/motor.py) (same subsystem files,
parameters, gating and `/motor/state`). Every check in this file applies
unchanged; run it against the RIO with:

```sh
bazel build //talOS/bridge:hardware_node //bench/main_processor/motor:node_py //talOS/teleop:node //talOS/telemetry:node
bazel run //talOS/launcher -- --config $PWD/bench/main_processor/configuration/robot_py.toml --remote <roborio-ipv4> --output-dir /tmp/bench-rio-py
```

## Uncommissioned and disabled checks

- [ ] Keep `commissioned = false` and exercise controls while DS disabled.
  Expected: RIO console reports `commissioned=0`; in Studio, `/hw/status`
  `configured` is true (the RoboRIO's current boot applied and acknowledged
  the uncommissioned config) while `/hw/status` `commissioned` (with
  `alert_error_uncommissioned` active) and the `/hw/state` flag `configured`
  (the hardware log's `configured`, which also requires `commissioned`) are
  false; no motor motion occurs. Record console and `/motor/state`.
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
  `max_bench_voltage_v = 2.0`, ratio 1.0, velocity ceiling 30 rps and slot0 gains
  (measured feedforward 0.095 V/rotor-rps). Expected: these match the bare
  motor and power source. Record reviewed limits; raise deliberately only after
  the first spin, never to hide a fault.
- [ ] Only a human, after those checks, sets commissioning true in the base
  and deployment configs and updates [the golden config test](../bench/main_processor/configuration/config_golden_test.cc) to the measured
  configuration in the same change. Repeat Mini build/test, bundle install and
  launch. Expected: RIO acknowledges configuration; `/hw/status`
  `configured` and `commissioned` (Studio) and the `/hw/state` flag
  `configured` become true. Record reviewed
  change/config hash and RIO acknowledgment.
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
  and release before any re-enable. Expected also: once the program runs,
  States resume within 100 ms of the RIO's network having a route to the
  Mini; a program that started before that route prints
  `[<s>.<ms>] hardware gateway socket failed: SocketError (errno 101)`
  (coalesced to one line a second) and then `[<s>.<ms>] hardware gateway
  socket open after <n> failed attempts, <s> s after the first`. Record old/new boot ids, clock reset
  count, the RIO console as written on the RIO (a network console viewer
  loses lines while the link is down; the program's lines carry the RIO's
  seconds since boot, which the first State's `sample_time_us` in the
  bridge log maps to session time) beside `dmesg`'s link-up lines, the
  first State's arrival in the bridge log, config acknowledgment and fresh
  DS/target acceptance after safe recovery.
- [ ] Reset the TalonFX alone (briefly interrupt its power) while DS is
  disabled and the motor rests at least 10 rotations from 0, with
  `hardware_node --log` recording. Expected: in the first State whose motor
  position has restarted near 0, the motor is invalid and
  `device_reset_count` has risen, whether Phoenix's reset latch (the RIO
  console's device reset line) came first or the gateway's position-jump
  check caught it; the node's `zeroed()` is false. Record, per reset over at
  least 10 trials, the States from the first restarted position frame to the
  latch: this measures the order TODO.md RESET-1 rests on.
- [ ] Kill `hardware_node` while spinning slowly. Expected: RIO watchdog
  neutralizes without the bridge; launcher restart policy is visible in the
  manifest. Disable/release before recovery. Record neutral delay, PID attempts,
  restart count and recovery behavior; no automatic motion is acceptable.
- [ ] Stall the RoboRIO loop (its watchdog). Over SSH on the RoboRIO as
  `admin`, record every thread's scheduling class and realtime priority
  (`ps -eLo pid,tid,cls,rtprio,comm`): the robot program's loop thread (its
  main thread, named after the program, `FF` 10), the interrupt threads
  (`FF` 15, `irq/61-atomiczy` `FF` 96), `talos_watchdog` (`FF` 30), the
  robot program's other threads (`TS`, among them `can_psend` and
  `can_recv`; three unnamed ones `RR` 1-2), the Driver Station (netcomm)
  threads (`TS`) and Phoenix's threads; note whether any thread runs between
  `FF` 10 and 30, record `cat /proc/sys/kernel/sched_rt_runtime_us` (-1 on
  2026-11-11: no throttling) and which CPU takes the interrupts
  (`cat /proc/interrupts`). While applying a small voltage, stand in for a
  loop that never sleeps: starve the loop and every thread below the
  interrupt threads with one `SCHED_FIFO` 12 spinner per core (above the
  loop, below the interrupt threads and the watchdog) for a measured 300 ms:
  `for c in 0 1; do taskset -c $c chrt -f 12 timeout 0.3 sh -c 'while :; do :; done' & done; wait`
  then repeat with a single spinner on the CPU that takes the interrupts,
  which leaves the other core to the CAN threads
  (if the image lacks `taskset` or `chrt`, record that and skip). This is
  the check that the watchdog's `NeutralOut` reaches the bus while
  `can_psend` and the other `SCHED_OTHER` threads are held off. Expected:
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
- [ ] RoboRIO loop priority. The loop runs at `SCHED_FIFO` 10; its start
  line on the RIO console must read `[<s>.<ms>] hardware gateway loop
  SCHED_FIFO 10, watchdog SCHED_FIFO 30`.
  In the same enabled session as the representative captures below, record
  the RIO console's loop timing lines (overruns, late ticks, max work, max
  interval) over 10 minutes of driving with an ssh session running `top` on
  the RoboRIO, and the gateway's State sample intervals (p50, p90, p99, max)
  from the session. Then `touch /home/lvuser/talos_loop_default_sched`,
  restart robot code (the start line must read `[<s>.<ms>] hardware gateway
  loop SCHED_OTHER 0, watchdog SCHED_FIFO 30
  (/home/lvuser/talos_loop_default_sched present)`), repeat
  the session and the stall check above, and `rm` the file. Record late
  ticks, State intervals, Driver Station and CAN health under both; the
  decision is TODO.md RT-5.
- [ ] Per-device tick cost. With the one motor configured and the robot
  disabled, record three consecutive `[rio] tick phases` console lines
  (one per 10 s; [format](../talOS/hardware/README.md)): each phase's mean
  and max in µs, and the motor and sensor counts. Then, with the bridge
  running, record `/hw/status` `can_utilization_frac` with the config's
  `status_hz` at 200, then at 50 (reconfigure between). For a robot
  estimate (16 Talon FX and 4 CANcoders) scale the refresh and devices
  phases per device by 20, control per motor by 16, keep health, other, link, driver
  station and loop as measured, and scale `can_utilization_frac` per device at each rate by
  20; compare with the 5 ms period and a full bus. A one-motor bench gives
  no per-device slope (a fixed cost per call is counted as per device), so
  the estimate is an upper bound. The decision is TODO.md RT-5.
- [ ] Capture representative enabled sessions, fetch them (below) and run
  `bazel run //talOS/launcher:session_report -- --strict <session-dir>`.
  Expected: `/hw/state` median about 5 ms, p99 <10 ms, no 100 ms gap while
  enabled (PASS lines); `--strict` exits 2 unless every required verdict
  passes and the manifest shows a finished session with no quarantined,
  recorder-failed or log-capped recording node (Studio is exempt), no
  damaged log, and a "Delivery loss" section with no node's
  `wake_failures`, `requests_skipped` or `wake_fallbacks` above 0 (NOT
  RECORDED there means a manifest written before the launcher recorded
  them, or a member whose registry row the launcher never saw: Studio's
  bridge, which takes none, or a run that failed to spawn). Its "Launcher output" section names any output whose bytes
  `talos.log` says were dropped. Keep the
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
  Review per-node timer lateness and the bridge's request to command (the
  bridge has no timer) in the report, and `throttled=0 wait_fallbacks=0` on
  the bridge's exit line in `hardware_node.log`; no
  node should log `realtime scheduling incomplete`. Do not use
  `TALOS_REALTIME=0` for acceptance.
- [ ] In a disposable session lower `[robot] log_max_bytes` and exceed each
  writer's share. Expected: control continues, each log keeps its newest
  records in whole-record segments within its share (oldest
  `<stem>.segment<k>.tlog` deleted), completed manifest reports
  `log_capped: true`, and every capped member's kept records span between
  7/8 of the session's log window and the window, whatever the robot state
  (REQ-OPS-32; `log_dump` on the session directory: each process's first
  record; the window is `talos.log`'s `Event-log window` line). A member that stores more than its expected rate
  keeps less. Record cap/file sizes, the window, the spans and manifest,
  restore normal limits and reserve disk for session metadata.
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
