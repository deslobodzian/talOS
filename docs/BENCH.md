# Bench bring-up

`bench/` is a RoboRIO and five CAN devices, all on one CANivore bus named
`Shooter` plugged into the RoboRIO, with the Mac mini as main processor and
a Linux workstation as the Driver Station; their addresses are under
[bench network](#bench-network). The ids and bus are the devices' own, read
from them:

| Device | CAN id | Node | Device name |
|---|---:|---|---|
| Talon FX (Kraken X44), the restrained bare motor | 1 | `motor` | `kraken` |
| Talon FX (Kraken X60), swerve module drive | 5 | `swerve_module` | `drive` |
| Talon FX (Kraken X44), swerve module steer | 6 | `swerve_module` | `steer` |
| CANcoder, swerve module azimuth | 3 | `swerve_module` | `encoder` |
| Pigeon 2 | 0 | `gyro` | `pigeon` |

The RIO runs the existing generic gateway; no 2026 robot mechanisms are
launched. Every checked-in bench config is **uncommissioned**. The bare
motor's `slot0` feedforward is measured on it; the swerve module's
reductions, gains, limits and CANcoder offset are provisional
([its subsystem file](../bench/main_processor/configuration/subsystems/swerve_module.toml)).
Keep a spotter at DS disable and a safe power disconnect; neutral output does
not instantly stop a coasting rotor or wheel.

For every trial record operator/date, config hash, Mini/RIO versions, wiring,
DS mode, control inputs, session directory and RIO console output; what was
measured goes, dated, in [BENCH_RESULTS.md](BENCH_RESULTS.md). Stop and
investigate any unexpected motion before proceeding. Disable between trials,
release all controls, and explicitly re-enable only after checking recovery.

## Build, network and deployment

- [ ] Secure the bare motor, remove loose couplings, guard its shaft, clamp
  the swerve module with its wheel off the ground, mount the Pigeon flat,
  provide appropriate fused power and CANivore termination, and leave DS
  disabled. Expected: no motion or exposed rotating hazards. Record wiring,
  power supply and guards.
- [ ] Inspect the graph the bench launches. Three node packages:
  [`motor`](../bench/main_processor/motor) (C++, or `motor.py` under
  `robot_py.toml`) publishes `/motor/state`;
  [`swerve_module`](../bench/main_processor/swerve_module) (C++) publishes
  `/swerve_module/state`; [`gyro`](../bench/main_processor/gyro) (Python, an
  observer that reads no target) publishes `/gyro/state`. Beside them run
  the framework `teleop` (writes `/motor/target/teleop` and
  `/swerve_module/target/teleop`), `telemetry` and `hardware_node`.
  Expected: the `--describe-only` runs below describe exactly these six
  nodes. Record the output.
- [ ] Check the [bench network](#bench-network) from the Mini:
  `ping -c 3 10.0.0.72` is answered and `route -n get 10.0.0.72` names
  `interface: en0`. Inspect
  `bench/main_processor/configuration/robot_bench.toml`: it includes the base
  [robot config](../bench/main_processor/configuration/robot.toml) and sets
  the controller address to the robot network's `10.56.87.2`, so on this
  bench every launch passes `--remote 10.0.0.72`. Record interface, subnet
  and connectivity results.
- [ ] On the Mini, in a clean checkout nobody else is working in, run
  `bazel build //...` and `bazel test //...`. Expected: success on the
  deployment architecture, not merely Linux CI. Record revision,
  architecture and command results.
- [ ] Run `bazel test --test_output=errors //bench/...`, then
  `bazel run //talOS/launcher -- --config $PWD/bench/main_processor/configuration/robot.toml --describe-only`
  and repeat with `bench/main_processor/configuration/robot_bench.toml` and
  `bench/main_processor/configuration/robot_py.toml`. Expected: valid graphs. Record output.
- [ ] Before hardware deployment, run
  `bazel run //talOS/launcher -- --config $PWD/bench/main_processor/configuration/robot.toml --sim --start-sim-gateway --output-dir /tmp/bench-smoke --duration-s 20`.
  Expected: motor, swerve_module, gyro, teleop, telemetry, hardware_node and
  the generic ideal sim gateway remain running until normal shutdown; completed manifest has no
  crashes/restarts or failed logs; the bridge heard the gateway:
  `bazel run //talOS/launcher:session_report -- /tmp/bench-smoke/<session-id>`
  shows **PASS** controller link (or `hardware_node.log`'s `states=` line is
  above 0, with no `hardware_node warning: ended without receiving a State`
  line). The exit status cannot show this: the launcher ends the session by
  stopping every member, and a stop exits 0 whether or not a State ever
  arrived. Simulation overrides commissioning only in
  memory; it does not prove physical safety or voltage dynamics. Record manifest.
- [ ] Deploy the RIO program from either host. From the Linux workstation:
  `(cd controller_processor/rio && ./gradlew deploy -Pcompanion=10.0.0.37)`;
  from the Mini, whose Gradle needs a JDK and no daemon:
  `(cd controller_processor/rio && JAVA_HOME=~/wpilib/2026/jdk ./gradlew deploy --no-daemon -Pcompanion=10.0.0.37)`.
  Expected: Gradle prints
  `RoboRIO companion address built in: 10.0.0.37 (from -Pcompanion)`, the
  generic gateway starts on the RIO and its console line
  `[<s>.<ms>] hardware gateway build <commit> companion 10.0.0.37:5803` names
  the Mini. Record deploy result and RIO console banner.
- [ ] Build `bazel build //bench/main_processor:bench_bundle`; on the Mini run
  `./tools/deploy.sh install bazel-bin/bench/main_processor/bench_bundle --prefix "$HOME/talos-bench"`.
  Expected: `$HOME/talos-bench/current` selects a read-only bundle; previous
  bundle remains available. Record installed version and config hash. Never
  edit the installed bundle; rebuild/reinstall changes.
- [ ] Start it by hand:
  `./tools/deploy.sh run --prefix "$HOME/talos-bench" -- --remote 10.0.0.72 --output-dir "$HOME/talos-bench/logs" --duration-s 30`,
  then `./tools/deploy.sh logs --prefix "$HOME/talos-bench"`. `deploy.sh run`
  starts the selected bundle's `run.sh` with every argument after `--`
  unchanged, under the boot service's realtime limits on Linux (it stops,
  naming the one-time grant, when the hard limits are too low; macOS needs
  none), refuses while the boot service runs that bundle, and first prints
  `deploy: launcher pid <n>; ...`. Expected: that line, then one
  unique session below the output root, with the deployed config hash and
  each node's log paths. Record paths. Exactly one stack may run: stop any
  boot-managed launcher first ([boot operation](#boot-operation)).

### Bench network

The bench LAN is `10.0.0.0/24`:

| Host | Address | Role |
|---|---|---|
| RoboRIO (`roboRIO-5687-FRC.local`) | `10.0.0.72` | controller processor |
| Mac mini, Ethernet `en0` | `10.0.0.37` | main processor: the launcher and every node |
| Mac mini, Wi-Fi `en1` | `10.0.0.25` | never the link (below) |
| Linux workstation | `10.0.0.251` | Driver Station only: no launcher, no node |

The Mini's `en0` must keep `10.0.0.37` across a cable pull, and the
RoboRIO `10.0.0.72` across its reboot (a DHCP reservation or a manual
address on each): the RIO program is built for the one, and every launch
names the other.

Neither checked-in default fits it: the RIO program's built-in companion
address (`10.56.87.11`) and `robot_bench.toml`'s `controller_address`
(`10.56.87.2`) are the robot network's. So every RIO build for this bench
is `./gradlew deploy -Pcompanion=10.0.0.37`, and every launch on the Mini
passes `--remote 10.0.0.72`.

The Mini's Wi-Fi is on the same subnet. The bridge's socket takes its source
address when it connects, and the RIO's socket accepts frames only from the
address it was built with ([RoboRIO](../talOS/hardware/README.md#roborio)).
A socket the bridge opens while the Mini routes `10.0.0.72` over Wi-Fi
(it opens a new one after a send fails, as one may while the Ethernet cable
is out) would send from `10.0.0.25`, which the RIO ignores, and keep that
source after the cable returns: the motors would stay neutral until the
session is restarted (the bridge is alive, so nothing restarts it; seen on
[2026-11-17](BENCH_RESULTS.md#2026-11-17), TODO.md RT-27). Turn the
Mini's Wi-Fi off for an enabled session; `route -n get 10.0.0.72` must name
`interface: en0`.

On any other LAN, build the RIO program for the main processor's address,
from `controller_processor/rio`:
`./gradlew deploy -Pcompanion=<main-processor-ipv4>` (also `buildRio`), on
the command line each time: Gradle refuses it from a `gradle.properties` or
the environment, and an empty, hostname or unusable address stops the build. Check that the build's
`RoboRIO companion address built in:` line names it. The address is fixed in the
binary, never learned from the network
([RoboRIO](../talOS/hardware/README.md#roborio)); check the console line names
it. `deploy` still finds the RIO by its team mDNS name
(`roboRIO-5687-FRC.local`). The main processor must accept UDP on port 5803
from the RIO, and every launch passes `--remote <roborio-ipv4>` or a
config that names it (below).

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
bazel build //talOS/bridge:hardware_node //bench/main_processor/motor:node //bench/main_processor/swerve_module:node //bench/main_processor/gyro:node //talOS/teleop:node //talOS/telemetry:node
bazel run //talOS/launcher -- --config $PWD/bench/main_processor/configuration/robot_lan.toml --output-dir /tmp/bench-rio
```

Expected: the launcher prints `Controller <roborio-ipv4>:5802`, and
`/hw/status` counts States received. On this bench the same launch from a
checkout is `--config $PWD/bench/main_processor/configuration/robot_bench.toml --remote 10.0.0.72`,
which also takes a commissioning commit's `commissioned = true`. To watch it in Studio, build
`//studio/bridge:studio_bridge //studio:web_dist` too and add
`--start-studio-bridge`; the launcher then runs the
[Studio bridge](../talOS/launcher/README.md#studio-bridge) at
`http://<main-processor>:5800`.

### Bench with the Python motor node

[`robot_py.toml`](../bench/main_processor/configuration/robot_py.toml) is
`robot.toml` with the bare motor's node swapped for its Python twin,
[`motor.py`](../bench/main_processor/motor/motor.py) (same devices,
parameters, gating and `/motor/state`; only its construction checks differ:
it cannot refuse a positive `max_bench_torque_current_a` on a motor without
`foc` and `max_torque_current_a`, see
[torque current](#torque-current-hardware-only)). Its motor subsystem file,
[`motor_py.toml`](../bench/main_processor/configuration/subsystems/motor_py.toml),
is `motor.toml` (its `defaults`) at `period_us` 20000, since a Python
actuator owner runs at 20 ms or slower
([launcher](../talOS/launcher/README.md#python-actuator-periods)): read
"one motor tick" below as 20 ms. Every other check in this file applies
unchanged; run it against the RIO with:

```sh
bazel build //talOS/bridge:hardware_node //bench/main_processor/motor:node_py //bench/main_processor/swerve_module:node //bench/main_processor/gyro:node //talOS/teleop:node //talOS/telemetry:node
bazel run //talOS/launcher -- --config $PWD/bench/main_processor/configuration/robot_py.toml --remote <roborio-ipv4> --output-dir /tmp/bench-rio-py
```

[`robot_physics_sim.toml`](../bench/main_processor/configuration/robot_physics_sim.toml)
is the same graph for the physics simulator only (`simulation_only`; launch it with `--sim`).

### Identifying devices without Phoenix Tuner X

- [ ] While a robot program (the gateway, or Phoenix's temporary diagnostics
  program) runs on the RoboRIO, read the bus from its Phoenix diagnostic
  server, from any machine on the network:
  `curl 'http://<roborio>:1250/?action=getdevices'`. Each device's entry
  gives its `ID`, `Model`, `Name`, `CurrentVers` and `CANivoreDevName`.
  Expected: the five devices of the table above, each at its id, all on
  `Shooter`. Record the response. The same server also takes write actions
  (`setid`, `setname`); only a person at the bench uses those.

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
- [ ] With the robot disabled and uncommissioned, read the three states.
  Expected: all five devices report connected: `/motor/state` `connected`;
  `/swerve_module/state` `drive_connected`, `steer_connected` and
  `encoder_connected`; `/gyro/state` `connected` (with `hardware_fresh`).
  Turn the module's azimuth by hand: `encoder_absolute_position_rot` and
  `steer_position_rot` move together and by the same amount, since the
  steer motor reads the CANcoder's frame (the absolute position wraps in
  [-0.5, 0.5); the steer position does not). Turn the Pigeon
  counter-clockwise seen from above: `yaw_rad` rises. Nothing moves under
  power. Record the states and the angles turned.

## Human commissioning and first spin

- [ ] With power safe and DS disabled, a human checks every device's
  firmware, bus and id against the table above (the
  [device listing](#identifying-devices-without-phoenix-tuner-x) or Phoenix
  Tuner X) and the bare motor's inversion in
  [the motor config](../bench/main_processor/configuration/subsystems/motor.toml).
  Expected: exactly the intended devices resolve; positive demand and
  feedback direction agree after the supervised spin. Record ids, inversion
  and firmware.
- [ ] Before commissioning, a person enters the swerve module's drive
  reduction (`[motors.drive] sensor_to_mechanism_ratio`; until then the
  drive and `max_drive_velocity_rps` are in rotor units), its steer
  reduction (`[motors.steer] rotor_to_sensor_ratio`, rotor rotations per
  CANcoder rotation) and the CANcoder's `offset_rot` (heading 0 is the
  module's forward) in
  [its subsystem file](../bench/main_processor/configuration/subsystems/swerve_module.toml),
  whose values are provisional until then, and checks the encoder's
  direction by hand (the [disabled checks](#uncommissioned-and-disabled-checks)).
  Each motor's direction (`inverted`: a positive steer demand turns the
  wheel the way `encoder_absolute_position_rot` rises, a positive drive
  demand rolls it forward at heading 0) can be checked only once
  commissioned, since the gateway refuses every command before that: in
  the swerve module steps below (TODO.md BENCH-4). Record the reductions,
  the offset and the encoder's direction.
- [ ] Review supply 30 A, stator 40 A, hardware voltage ceiling
  `max_voltage = 2.0` (every mode), node `max_bench_voltage_v = 2.0` (the
  axis scale), node `max_bench_torque_current_a = 0.0` (torque
  mode off), ratio 1.0, velocity ceiling 30 rps and slot0 gains
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
  Record requested/measured speed, voltage, current and overshoot. Button 2
  asks more than the free speed at `max_voltage` 2 V (about 21 rps,
  [below](#torque-current-hardware-only)): expected, it levels there, less
  under load, with applied voltage at most 2 V. That is the limit holding;
  do not raise limits automatically.
  Then hold button 3 (torque current): NEUTRAL/NONE and no motion, since
  `max_bench_torque_current_a = 0.0` keeps torque mode off
  ([torque current](#torque-current-hardware-only) turns it on).
- [ ] Swerve module, wheel off the ground. Its
  [bindings](../bench/main_processor/configuration/subsystems/swerve_module.toml)
  are hold-to-run: it moves only while button 6 is held; the axes alone
  send a NEUTRAL target. Joystick 0 axis 4 asks the wheel heading
  (`steer_angle_rot`, scale 0.25: a quarter turn either way at full stick,
  in the CANcoder's frame, so a centred stick asks heading 0); axis 5 asks
  the drive velocity, stick forward positive (`drive_velocity_rps`, scale
  -10), which the node caps at `max_drive_velocity_rps` (10, in the drive's
  configured unit); both have a 10% deadband. Each module motor's hardware
  ceiling is its `max_voltage` (4.0 as committed, 2.0 in the
  commissioning commit). First steer step: enable teleop, hold button
  6 with axis 5 centred, move axis 4 a little, release, disable. Expected:
  only the steer moves, to `commanded_steer_position_rot` by the short way;
  `mode` DRIVE and `source` TELEOP only while held, NEUTRAL on release and
  on disable. First drive spin: hold button 6, push axis 5 slightly
  forward, release, disable. Expected: the wheel rolls forward at
  `commanded_drive_velocity_rps` (`reversed` is true when the module took
  the opposite heading and reversed the drive), neutral on release and on
  disable. Record commanded and measured positions, velocities, voltages
  and currents.

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

### PWM output: no pulse when off (hardware only)

The bench has no PWM device and no node that drives one, so this check is
recorded "not run" until a servo (or an oscilloscope) is on RoboRIO PWM 0.
Only hardware shows that an output that is off sends no pulse on the FPGA
and that a pulse of 0 is 1.5 ms; `phoenixTest` proves it only on the
simulated HAL.

- [ ] Add `[pwm_outputs.servo] channel = 0` to a fixture subsystem whose
  node (`period_us` at most 25 ms) reads its target lane with `talos::Target`
  and sets the servo to 0.5 for one target value, to 0.0 for another, and
  leaves it unset otherwise; map joystick 0 buttons 3 and 4 to those two
  targets in `[teleop]`. (A node that read the joystick itself would have no
  live target, so the flush would hold its output off whatever it set.)
  Expected, enabled in teleop: no button, no pulse (a flat line)
  and the servo turns freely by hand; button 3, a pulse wider than 1.5 ms,
  the servo moves and holds; button 4, a 1.5 ms pulse, the servo centred and
  holding; release, no pulse and the servo where it was, not centred (most
  servos go limp without a pulse; some digital ones hold their last position).
  Hold button 3 and `kill -STOP` the node: no pulse within about 55 ms (the
  50 ms owner timeout, then the next gateway tick); `kill -CONT`, release,
  then hold button 3 again and disable: no pulse. Record the pulse widths
  and the stop time.

### FOC and the Phoenix Pro check (hardware only)

Desktop simulation needs no licence, so `phoenixTest` proves EnableFOC on
each request and the failed-device handling only through a stand-in for the
licence read. Only hardware shows that the RoboRIO reads `IsProLicensed`
truly at configure, whether a CANivore's licence shows in it, and that the
Kraken runs FOC.

This check and the [torque-current](#torque-current-hardware-only) one
need a host that runs Phoenix Tuner X (Windows, Android or iOS) on the
[bench network](#bench-network); neither the Mini nor the Linux
workstation is one. They change the motor config locally, and a person
(never an agent) makes each change, after reading the licence in the first
step below: launch it from the Mini's checkout as
[bench network](#bench-network) gives (never edit an installed bundle), with
the robot disabled, since an enabled RoboRIO refuses a changed
configuration (TODO.md RESET-4). Such a session's revision is dirty: it is
evidence for these checks, not for the
[supervised enabled session](#one-supervised-enabled-session).

- [ ] In Tuner X, record whether the Kraken and the `Shooter` CANivore each
  carry a Phoenix Pro licence.
- [ ] A person (never an agent) sets `foc = true` under `[motors.kraken]` in
  [the motor config](../bench/main_processor/configuration/subsystems/motor.toml)
  as a local change, reverted after, and launch it. Expected: no
  `[phoenix] ... check_pro_license` line on the RoboRIO console, and
  `/hw/status` `motor_fault_mask` clear. Enable teleop and deflect axis 1:
  Tuner X shows control mode `VoltageFOC`; hold button 1 (10 rps):
  `VelocityVoltageFOC`, the speed settling at 10 rps; disable: `NeutralOut`.
  Record applied voltage, stator and supply current at 10 rps with `foc`
  true and false (the same speed is expected; recheck slot0 `v` if the
  voltage differs by more than 10 %).
- [ ] The configure time the licence read adds. No console line marks a
  push or its acknowledgement, so read it from the session's `/hw/status`
  records (`bazel run //talOS/launcher:log_dump -- <session-dir>`): the
  time from the first record with `alert_error_link_lost` false to the
  first with `configured` true. Take it at the first launch after each
  change of `foc` (true, then false): a RoboRIO already holding the pushed
  configuration does not configure again. Record both times.
- [ ] A licence read with no answer. With `foc = true`, boot the RoboRIO
  with the Kraken unpowered, launch, then power the Kraken. Expected: until
  it is powered, the kraken's feedback is invalid and axis and buttons move
  nothing; the console may show the configure's first error for it (a
  timeout, during `apply_talonfx_config` or `check_pro_license`) but no
  `foc needs a Phoenix Pro licence` line; within a few seconds of power
  (the restore backoff, at most 2 s, then the restore), it drives in
  `VoltageFOC` and `motor_fault_mask` clears. Record the times.
- [ ] If a device without a licence is at hand (or the licence is not
  confirmed), the same change with it: the console prints one `[phoenix]`
  line for each failed motor, naming the motor, its bus, its CAN id and
  `check_pro_license`; `/hw/status` `motor_fault_mask` carries its bit; axis
  and buttons move nothing; the other bench devices are unaffected. Record
  the lines.

### Torque current (hardware only)

In desktop simulation Phoenix runs TorqueCurrentFOC as a current loop
against the RoboRIO program's own rotor model, so `phoenixTest` shows the
request's fields (the demand and the duty cap), the peak torque current in
the applied configuration and NeutralOut for a request out of contract; it
asserts nothing about Phoenix's simulated current loop. Only hardware shows that the Kraken tracks the stator
current and honours the peak and the duty cap. This check needs the FOC
check above passed (a licensed Kraken). The bench motor node's
`TORQUE_CURRENT` mode is bound to joystick 0 button 3 at 2 A
([`motor.toml`](../bench/main_processor/configuration/subsystems/motor.toml);
buttons 1 and 2 win over it) and clamped to the node parameter
`max_bench_torque_current_a`, which the committed config sets to 0: torque
mode off, so button 3 drives nothing until a person raises it.

It runs inside or after the
[supervised enabled session](#one-supervised-enabled-session), under its
rule for people: a spotter whose only job is the Driver Station's disable
and e-stop, hands on them for every enabled step. Leave `max_voltage` as
committed (`2.0`): torque mode is bounded only by that duty cap, not by
`max_velocity_rps` (30), which bounds velocity demands alone. The rotor's
free speed is about `max_voltage` / 0.095 rps (slot0 `v`, 0.095 V per
rps): about 21 rps at 2 V, where button 3 levels as button 2 does; a raised
`max_voltage` would take it past the velocity buttons (about 42 rps at
4 V).

- [ ] A person (never an agent) sets, as a local change, reverted after, in
  [the motor config](../bench/main_processor/configuration/subsystems/motor.toml)
  `max_bench_torque_current_a = 2.0` under `[subsystem]`, and `foc = true`
  and `max_torque_current_a = 5.0` under `[motors.kraken]`, keeping
  `stator_limit_a = 40.0` and `brake = false`, and
  launch it, the bare motor restrained with nothing on its shaft. The C++
  motor node refuses to start with a positive `max_bench_torque_current_a`
  unless both motor keys are set, naming them. The Python one
  (`robot_py.toml`) cannot read the motor's config, so it starts, and fails
  only while enabled, at the first torque target: `Motor.set_torque_current`
  raises naming the motor and the key, the node exits and the launcher
  restarts it, again and again for as long as button 3 is held. The motor
  stays neutral, and the axis and the other buttons do nothing meanwhile;
  release button 3, disable and fix the config. Expected: Tuner X reads the
  Kraken's `PeakForwardTorqueCurrent` 5 A and `PeakReverseTorqueCurrent`
  -5 A.
- [ ] Enable teleop and hold button 3. Expected: `/motor/state` `mode`
  `TORQUE_CURRENT` and `source` `TELEOP` while held (`NEUTRAL` and `NONE`
  once released); Tuner X control mode
  `TorqueCurrentFOC`; `/motor/state` stator current about 2 A while the rotor
  accelerates; the speed levelling near the free speed at the committed
  `max_voltage` (above), not the X44's full free speed, with applied
  voltage at most `max_voltage` and Tuner X duty cycle about `max_voltage`
  over the battery voltage; the stator current then falling to what friction
  takes.
  Release: neutral, the rotor coasting down. Record the stator current while
  accelerating, the levelled speed, the applied and battery voltages, the
  duty cycle and the coast-down.
- [ ] Hold button 3 and `kill -STOP` the motor node: `NeutralOut` within about
  55 ms (the 50 ms owner timeout, then the next gateway tick); `kill -CONT`
  and release. Hold button 3 and disable: `NeutralOut` at once. Record the
  stop times.
- [ ] What the motor does at exactly 0 A (TODO.md DEVICES-12). As a further
  local change set button 3's binding to `torque_current_a = 0.0`. Spin the
  rotor with the axis (voltage), then hold button 3 with the axis still
  deflected (the button wins): Tuner X control mode `TorqueCurrentFOC` at
  0 A. Record whether the rotor coasts down as it does with every control
  released (NeutralOut), or stops sooner; repeat with `brake = true` under `[motors.kraken]` and
  record the same.

Pass: `TorqueCurrentFOC` only while button 3 is held, `NeutralOut` on
release, stop and disable; stator current never above 5 A; applied voltage
never above `max_voltage`.

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
- [ ] While spinning slowly, unplug RIO Ethernet (on this bench that also
  cuts the Driver Station's link, which disables). Expected: gateway watchdog
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
- [ ] Reset the bare motor's TalonFX (id 1) alone (briefly interrupt its power) while DS is
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
  Driver Station reader `talos-ds` (`FF` 5), the watchdog's reporter
  `talos_wd_report` (`TS`), the
  robot program's other threads (`TS`, among them `can_psend` and
  `can_recv`; three unnamed ones `RR` 1-2), the Driver Station (netcomm)
  threads (`TS`) and Phoenix's threads; note whether any thread runs between
  `FF` 10 and 30, record `cat /proc/sys/kernel/sched_rt_runtime_us` (-1: no throttling) and which CPU takes the interrupts
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
  the silence and the stall count, and no `RoboRIO loop watchdog: <k> stall
  report(s) dropped` line on the RIO console (a stall whose error was lost;
  record it if one appears); late ticks on the RIO console; after the
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
  restart robot code (from the Driver Station, or where it has no such
  button, `ssh admin@10.0.0.72 '. /etc/profile.d/natinst-path.sh; /usr/local/frc/bin/frcKillRobot.sh -t -r'`,
  the command GradleRIO's deploy runs) (the start line must read `[<s>.<ms>] hardware gateway
  loop SCHED_OTHER 0, watchdog SCHED_FIFO 30
  (/home/lvuser/talos_loop_default_sched present)`), repeat
  the session and the stall check above, and `rm` the file. Record late
  ticks, State intervals, Driver Station and CAN health under both; the
  decision is TODO.md RT-5.
- [ ] Per-device tick cost. With the bench's three motors and two sensors
  configured and the robot disabled, record three consecutive `[rio] tick
  phases` console lines (one per 10 s; [format](../talOS/hardware/README.md)):
  each phase's mean and max in µs, and the motor and sensor counts. Then,
  with the bridge running, record `/hw/status` `can_buses/1/utilization_frac`
  (slot 1: on the bench, the `Shooter` CANivore all five devices are on;
  slot 0 is the RoboRIO's own bus, and `hardware_node`'s stderr names each
  slot), its `read_state` (`OK`; a `FAILED` or `STALE` slot's figure is an
  old one) and its four error counters, with the config's `status_hz` at
  200, then at 50 (reconfigure between). For a robot estimate (16 Talon FX and 4
  CANcoders) scale the refresh and devices phases per device (the measured phase over the bench's five devices) by
  20, control per motor (over its three motors) by 16, keep health, other,
  link, driver station and loop as measured, and scale
  the slot's utilization per device at each rate by 20; compare with the
  5 ms period and a full bus. One device count gives no per-device slope (a
  fixed cost per call is counted as per device), so the estimate is an
  upper bound. The decision is TODO.md RT-5.
- [ ] Capture representative enabled sessions, fetch them (below) and run
  `bazel run //talOS/launcher:session_report -- --strict <session-dir>`.
  Expected: `/hw/state` median about 5 ms, p99 <10 ms, no 100 ms gap while
  enabled (PASS lines); `--strict` exits 2 unless every required verdict
  passes and the manifest shows a finished session with no quarantined,
  recorder-failed or log-capped recording node (Studio is exempt), no
  damaged log, and a "Delivery loss" section with no node's
  `wake_failures`, `stalled_arms`, `requests_skipped` or `wake_fallbacks`
  above 0 (NOT
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
  `./tools/deploy.sh fetch --prefix "$HOME/talos-bench" <archive-directory>`. Expected: complete session
  directories including restarted-node log attempts. Record config hash and
  archive path; retain raw `.tlog` and RIO console logs.
  `bazel run //talOS/launcher:log_dump -- <session-dir>` is the session decoder
  and prints timestamped records with named payload fields.

## One supervised enabled session

One hour at the bench that turns the checks above into acceptance evidence
by this file's rules and [BENCH_RESULTS.md](BENCH_RESULTS.md)'s: a pushed,
clean revision the session records, the bridge and control loops with their
full realtime grant, an enabled session `session_report --strict` passes,
every stop path a macOS main processor can run timed with the motor
turning, and every session kept. It is
written for the [bench network](#bench-network) as it is: the Mac mini as
main processor (an installed bundle, launched by hand with `--remote`), the
Linux workstation as the Driver Station only, and the RoboRIO at `10.0.0.72`
built with `-Pcompanion=10.0.0.37`. It runs sessions A, B, C1, C2 and D, and
an optional E; each trial is read afterwards from the session's logs
([reading a trial](#reading-a-trial)).

**People.** Every enabled step needs two: the driver, who alone enables,
holds the stick and, with the other hand, presses Enter on the trial's
command already typed in T2 (or a third person at the Mini's keyboard
does); and a spotter whose only job is the Driver Station's disable and
e-stop, hands on them for the whole enabled time. No enabled step runs
without the spotter.

**What may move.** The driver drives only joystick 0 axis 1 (VOLTAGE mode:
the deadband-removed fraction times `max_bench_voltage_v`), about half
deflection, held steady for at least 2 s before each action, so the motor
turns at about 1 V. The bench teleop config
([`motor.toml`](../bench/main_processor/configuration/subsystems/motor.toml),
[`swerve_module.toml`](../bench/main_processor/configuration/subsystems/swerve_module.toml))
binds joystick 0: axis 1 the bare motor's voltage; buttons 1 and 2 its
VELOCITY 10 and 30 rps (they override the axis); button 3 its
TORQUE_CURRENT 2 A, off at the committed `max_bench_torque_current_a = 0.0`;
button 6 the swerve
module, hold-to-run, with axis 4 its wheel heading (a centred stick asks
heading 0, which with a provisional `offset_rot` can be up to half a turn
from where the wheel rests) and axis 5 its drive velocity up to 10 rps.
Never press buttons 1, 2, 3 or 6. Axis and button numbers are the
RoboRIO's (teleop's `axis` is a 0-based index, `button` 1-based); a
Driver Station program may number them otherwise, and the gamepad's
printed labels are no guide. Session A confirms them from the RoboRIO's
side before anything is enabled. The swerve module stays clamped with its wheel off the
ground all hour. Stop and disable at any unexpected motion.

**The Driver Station** runs on the Linux workstation, whichever program the
team uses there. Before session A it shows communication with the RoboRIO
and robot code, the mode Teleoperated selected, the robot disabled, and the
gamepad in joystick slot 0. Session A confirms each from the RoboRIO's
side, in what it forwarded on `/hw/state/driver_station`. The program's
messages from the RoboRIO (`companion connected`, `companion lost`) are
also printed on the RoboRIO console, so either view serves where this
section names the Driver Station log; keep whichever the team used.

Record the RoboRIO console for the whole hour, on the workstation, not the
Mini (stop path 6 pulls the Mini's cable):
`ssh admin@10.0.0.72 'tail -F /var/local/natinst/log/FRC_UserProgram.log' | tee rio_console_<date>.log`.
Before session A it must show the `hardware gateway build` line, and once
the launcher runs, `companion connected`; if it does not, stop: the dead
times below cannot be measured without it. Each line's `[<s>.<ms>]` is the
RoboRIO's seconds since boot, the clock of State's `sample_time_us`, so a
dead time is a difference of those stamps. The file goes in the hour's
archive.

Two terminals on the Mini's own screen and keyboard: T1 runs the launcher,
T2 everything else, both in the root of the Mini's checkout of the session
revision (step 2; not a checkout with someone's work in progress). Not over
ssh: stop path 6 pulls the Mini's Ethernet cable, which could end an ssh
session and the launcher with it. In both:

```sh
RIO=10.0.0.72
ME=10.0.0.37                   # the Mini's en0, the address the RoboRIO program is built for
PREFIX="$HOME/talos-bench"     # installed bundles
ROOT="$PREFIX/logs"            # one output root per session below; never /tmp
REPORT=bazel-bin/talOS/launcher/session_report
DUMP=bazel-bin/talOS/launcher/log_dump
TRIALS=bazel-bin/talOS/launcher/stop_paths
```

Each session gets its own output root, `--output-dir "$ROOT/<letter>"`:
at each launch the launcher's retention deletes the oldest sessions of
its output root while they hold more than half of `[robot] log_max_bytes`
(1 GiB at the default;
[session segments and logs](../talOS/launcher/README.md#session-segments-and-logs)),
so in one shared root a later launch would delete session B. For each
session set `S="$ROOT/<letter>/<id>"` from the launcher's `Starting session
<id>` line, and note its `[launcher] Event-log window <us> us` line: a
session longer than 7/8 of that window loses its first trials to log
rotation, so end it sooner. `REPORT`, `DUMP` and `TRIALS` are the binaries step 3
built; they run outside `bazel run`, so no Bazel server restarts on the
Mini between sessions.

### Before leaving the desk

1. **Commissioning and the low caps: a person's change.** The checked-in
   bench configs are uncommissioned, and the gateway refuses every command
   until a person has done [human commissioning](#human-commissioning-and-first-spin):
   its device check, the swerve module's reductions and offset, the limits
   review and the commissioning commit (`commissioned = true` in
   `robot.toml` and `robot_bench.toml`), all before session A; its disabled
   stick check, which needs the commissioned stack running, is in session A. Its first enabled step (axis 1, slightly) is session B's
   first spell; its button 1 and 2 step and its swerve module steps are not
   part of this hour. `commissioned = true` makes every bench device live,
   the swerve module's drive (5) and steer (6) included. Every
   voltage-based request the bench sends (VoltageOut, VelocityVoltage,
   PositionVoltage; no bench node sends DutyCycleOut, nor TorqueCurrentFOC
   at the committed `max_bench_torque_current_a = 0.0`) is bounded by the
   motor's `max_voltage`, which the Talon is configured with as its peak
   voltage; the gateway neutralizes that motor for a command whose VOLTAGE
   demand or feedforward exceeds it. The bare motor's is committed at `2.0`
   (`[motors.kraken]` in `motor.toml`): check that it reads `2.0`. The
   swerve module's are `4.0` on lines 32 and 50 of `swerve_module.toml`
   (`[motors.drive]`, `[motors.steer]`). In the same reviewed commit the
   person sets those two to `2.0` and updates the golden test
   ([`config_golden_test.cc`](../bench/main_processor/configuration/config_golden_test.cc),
   which asserts uncommissioned and each `max_voltage`), then runs the tests
   the change affects, [DEVELOPMENT.md §5](DEVELOPMENT.md#5-verification-ladder)'s
   rung 2 for `//bench/main_processor/configuration/...` (other configs
   include `robot.toml`), and `--describe-only` on `robot_bench.toml`.
   These are not an agent's edits (RULES.md 4).
2. **The revision.** On that commit, pushed, checked out on the Mini:
   `git status --porcelain` prints nothing (an untracked file counts as
   dirty, [`tools/workspace_status.sh`](../tools/workspace_status.sh)), and
   after `git fetch` the first line of `git status -sb` shows no `ahead` or
   `behind`. Record `git rev-parse HEAD`.
3. **The bundle, stamped, and the report tools**, on the Mini:
   `bazel build --config=release //bench/main_processor:bench_bundle //talOS/launcher:session_report //talOS/launcher:log_dump //talOS/launcher:stop_paths`,
   then `grep -E '^(git_revision|git_dirty|config_sha256)' bazel-bin/bench/main_processor/bench_bundle/bundle_manifest.toml`.
   Pass: `git_revision` is HEAD and `git_dirty = false`. Without
   `--config=release` every report prints `Source revision: NOT RECORDED`
   ([deployment](DEPLOYMENT.md#build-a-bundle)).
4. **Install it** where no sudo is needed:
   `./tools/deploy.sh install bazel-bin/bench/main_processor/bench_bundle --prefix "$PREFIX"`,
   then `grep -n 'commissioned =\|max_voltage =' "$PREFIX"/current/config/robot.toml "$PREFIX"/current/config/robot_bench.toml "$PREFIX"/current/config/subsystems/motor.toml "$PREFIX"/current/config/subsystems/swerve_module.toml`.
   Pass: the graph check accepts it, `current` selects it, both robot files
   read `commissioned = true` (`robot_bench.toml` overrides `robot.toml`),
   and all three `max_voltage` lines read `2.0`.
5. **The RoboRIO program** of the same revision, from a clean checkout of
   it on either host: from the workstation
   `(cd controller_processor/rio && ./gradlew deploy -Pcompanion=10.0.0.37)`,
   from the Mini
   `(cd controller_processor/rio && JAVA_HOME=~/wpilib/2026/jdk ./gradlew deploy --no-daemon -Pcompanion=10.0.0.37)`.
   Pass: Gradle prints `RoboRIO companion address built in: 10.0.0.37 (from
   -Pcompanion)`, and the RoboRIO console prints `hardware gateway build
   <HEAD> companion 10.0.0.37:5803` without `(dirty)`, `hardware gateway
   loop SCHED_FIFO 10, watchdog SCHED_FIFO 30` and `driver station reader
   SCHED_FIFO 5`. The companion address is compiled in: the Mini's `en0`
   must keep `10.0.0.37` all hour.
6. **The Mini's link.** Turn the Mini's Wi-Fi off
   ([bench network](#bench-network)). Pass: `ipconfig getifaddr en0` prints
   `10.0.0.37`, `route -n get "$RIO"` prints `interface: en0`, and
   `ping -c 3 "$RIO"` is answered.
7. **Realtime: nothing to grant on macOS, only to check.** The bridge and
   control loops take the Mach time-constraint policy and lock their rings
   and stacks themselves, with no limit to raise first
   ([realtime scheduling](NODE_DEVELOPMENT_GUIDE.md#realtime-scheduling)).
   A refusal still shows, and session A checks all three places: the
   launcher's `[launcher] warning: <name> (pid <n>) <tier> loop runs without
   its realtime grant: <grant>` line, a node log's `talos: realtime
   scheduling incomplete` line, and the report's Realtime column
   ([realtime scheduling](../talOS/introspection/README.md#realtime-scheduling)).
   Pass now: `env | grep '^TALOS_'` prints nothing (`TALOS_REALTIME=0` is
   never acceptance).
8. **A quiet host.** No Bazel build or test, no agent, no other heavy work
   on the Mini during the hour (`bazel shutdown` in the session checkout
   once step 3 is done; confirm no other checkout on the Mini is
   building). The Mini must not sleep: `./tools/deploy.sh boot-check`
   prints `deploy: ok    pmset sleep 0` (its FileVault and `autorestart` lines
   concern the boot service, not this hour); otherwise keep it awake with
   `caffeinate -i` running in a third terminal until the hour ends. Record
   `sysctl -n hw.ncpu`, `uname -rv` and `uptime` before each session; a busy
   host's timing is not evidence.

### Session A: disabled (5 min)

Robot code green on the Driver Station, robot disabled, `ping -c 3 "$RIO"`
answered. In T1:
`./tools/deploy.sh run --prefix "$PREFIX" -- --no-studio --remote "$RIO" --output-dir "$ROOT/A" --duration-s 120`.
While it runs, robot disabled, the driver deflects what they take for axis
1 and holds it, then centres it; then holds buttons 1, 2, 3 and 6 in turn,
each about 2 s, one at a time. Nothing may move.
Every launch passes `--no-studio`, so nothing but the robot runs on the
Mini.

- [ ] Launcher output: `[launcher] Controller 10.0.0.72:5802 (local port 5803)`,
  one `[launcher] Started <name> (pid <n>, ...)` per member, and no
  `[launcher] warning: ... loop runs without its realtime grant` line.
  `grep -l 'realtime scheduling incomplete' "$S"/*.log` prints nothing.
- [ ] `grep 'hardware_node:' "$S/hardware_node.log"`: `can_buses slots: 0 rio,
  1 Shooter`; `gateway boot <id>: roborio build <first 16 hex digits of
  HEAD>` without `(dirty)`; `states=` above 0. The Driver Station log shows
  `companion connected` and no `companion speaks protocol` error: with
  `states=` above 0 this is the check that the RoboRIO and the bridge
  speak one protocol (the bridge drops a State of another).
- [ ] `"$REPORT" "$S"` (no `--strict`: a disabled session cannot pass the
  enabled verdicts). Pass: `Source revision: <HEAD>` without `(dirty)`;
  the Realtime column `full` for `hardware_node`, `motor`, `swerve_module`
  and `teleop`; **PASS** controller link; in `/hw/status`, `configured`
  and `commissioned` true (the RoboRIO acknowledged this bundle's config,
  whose id covers every `max_voltage`, so the caps are in force),
  `motor_fault_mask` 0, the rejected counts flat; no `alert_error_*` in
  Alerts.
- [ ] The Driver Station as the RoboRIO saw it:
  `"$DUMP" "$S" --format summary --topic /hw/state/driver_station`. Pass:
  its `hardware` line (the session's last packet) reads `"mode":"teleop"`,
  `"ds_attached":true`, `"enabled":false` and `"estop":false`, and its
  `joysticks` list has an entry with `"port":0`. A mode of `none` means no
  Driver Station is attached; a missing port 0, the gamepad is in another
  slot.
- [ ] The axis and buttons are the ones the bench binds:
  `"$DUMP" "$S" --format jsonl --topic /hw/state/driver_station | python3 -c 'import json,sys; js=[j for l in sys.stdin for j in (json.loads(l).get("hardware") or {}).get("joysticks", []) if j["port"] == 0]; a=[j["axes"][1] for j in js if len(j["axes"]) > 1]; print("axis 1:", min(a), max(a)); print("buttons:", sorted({b + 1 for j in js for b in range(32) if j["buttons"] >> b & 1}))'`.
  Pass: axis 1 spans well past the 10 % deadband (towards -1 for the
  stick pushed forward) and `buttons: [1, 2, 3, 6]`. Otherwise the
  gamepad's numbering differs from the bench's: stop and remap the
  controls the driver uses before going on.
- [ ] The disabled stick check (commissioning's): nothing drove.
  `"$DUMP" "$S" --format jsonl --topic /motor/state,/swerve_module/state | grep -cE 'mode: (VOLTAGE|VELOCITY|TORQUE_CURRENT|DRIVE)|source: TELEOP'`
  prints `0`: `/motor/state` stayed `mode: NEUTRAL`, `source: NONE` and
  `/swerve_module/state` `mode: NEUTRAL`, `source: NONE` while the stick
  moved and the buttons were held. Record what was physically seen.
- [ ] `"$DUMP" "$S" --format summary --topic /hw/status`: in the `latest`
  line the second `can_buses` entry (slot 1, `Shooter`) ends
  `read_state: OK`. `session_report` does not show it.
- [ ] `"$DUMP" "$S" --format summary --since-ms 10000 --until-ms 50000 --topic /motor/state,/swerve_module/state,/gyro/state`:
  inside that window every `connected` range (`connected`,
  `drive_connected`, `steer_connected`, `encoder_connected`) reads
  `true true`, and `fault` `false false`. Over the whole session the
  ranges start `false`: a node reads every device disconnected until its
  first accepted State, which waits for the RoboRIO to apply the pushed
  config.

Any failure here ends the hour: fix it at the desk.

### Session B: enabled acceptance (10 min)

In T1: `./tools/deploy.sh run --prefix "$PREFIX" -- --no-studio --remote "$RIO" --output-dir "$ROOT/B" --duration-s 420`.
Enable teleop with the stick released; the first spell is commissioning's
first enabled step (axis 1 slightly, then centre). Then drive in spells of
about 20 s separated by 10 s released, for at least 3 minutes enabled.
Within them, three Driver Station disables with the motor turning (stop
path 1 below), each followed by a release and a re-enable. Disable for the
last time before the 420 s end. No other fault in this session.

- [ ] `"$REPORT" --strict "$S" > "$S/session_report.md"; echo $?` prints
  `0`: `/hw/state` enabled median about 5 ms, p99 under 10 ms, no enabled
  gap of 100 ms, target age at tick at most 25 ms
  ([session measurements](../talOS/launcher/README.md#session-measurements)),
  realtime and revision as in session A. This is REQ-RT-15's session.
  Record the `timer motor:` row's lateness (stop path 2 has no margin for
  it).
- [ ] The cap, before any stop path: `"$DUMP" "$S" --format summary --topic /motor/state`.
  Pass: `range applied_voltage_v` stays within -2.0 and 2.0. A sample
  outside it ends the hour: find why (TODO.md BENCH-2 (a)) before session C.
- [ ] Latency (REQ-RT-25, REQ-RT-26), when the session revision prints it:
  two RoboRIO console lines `[rio] latency: command age accepted
  p50/p99/max ...` taken while enabled, and the report's "Latency
  (REQ-RT-25)" rows. REQ-RT-26: an accepted command's observation is at
  most 2 × `period_us` + 1000 µs old at p99 (11 ms) and 3 × `period_us` at
  max (15 ms) when the RoboRIO applies it, and a Driver Station refresh on
  the RoboRIO reaches the first command applied from it within 40 ms at
  p99. Record each against its bound (TODO.md TIME-4).

### Sessions C1 and C2: stop paths (10 min each)

C1 runs stop paths 2 to 4 and ends with the robot disabled and Ctrl-C in
T1; C2 runs paths 6 to 8. Each launch, in T1, without
`--duration-s`:
`./tools/deploy.sh run --prefix "$PREFIX" -- --no-studio --remote "$RIO" --output-dir "$ROOT/C1"`
(`C2` for the second). Take the pids from T1's `Started` lines; after a
restart the manifest holds each node's newest:
`python3 -c 'import json,sys; [print(n["name"], n["pid"]) for n in json.load(open(sys.argv[1]))["nodes"]]' "$S/manifest.json"`.
Never signal a process by name. Each trial starts with the motor turning
steadily and the driver holding the stick. `PAUSE` and `LOG GAP` are lines
of the [reader](#reading-a-trial).

| # | Action (T2) | Requirement and bound | Pass |
|---|---|---|---|
| 1 | (session B) the spotter presses Disable | REQ-SAFE-02, REQ-SAFE-18; no requirement states a time: the gateway neutralizes in the tick that reads the disable ([runtime contract](../talOS/hardware/README.md#runtime-contract)) | the first State with `enabled=False` has `active=False`; record its `sample_us` to the first with `kraken_v` at 0 (TODO.md BENCH-2 (b)) |
| 2 | `kill -STOP <teleop pid>; sleep 0.3; kill -CONT <teleop pid>`, stick held throughout, then released and pushed again | REQ-SAFE-07, -08, -41: neutral within `command_timeout_us` (100 ms) plus one motor tick (5 ms); REQ-SAFE-13, -14 after it | 105 ms or less from the start of the teleop lane's `PAUSE` to the first `/hw/request/motor` with `kraken=neutral` (the motor's timer lateness, session B, comes on top: record it beside); no motion after `CONT` until the stick is released and pushed again (the lane reads `active: false` until then) |
| 3 | `kill -STOP <motor pid>; sleep 0.3; kill -CONT <motor pid>`, stick held | REQ-SAFE-15: neutral at the bridge once the last request it heard is older than the owner timeout, `max(2 × 5 ms, 100 ms / 2)` = 50 ms ([the tick](../ARCHITECTURE.md#the-tick)) | from the start of the `motor` `/hw/request/motor` `PAUSE` to the first `/hw/command` with `kraken=neutral`: just over 50 ms (the bridge expires an owner only strictly past its timeout, counted from when it read the request). Pass at 50 ms plus the max of session B's `bridge request to command hardware_node enabled` row or less; record the excess. After `CONT` the motor drives again at once with the stick held: expected, the motor node has no arming (REQ-SAFE-31) |
| 4 | `kill -9 <motor pid>`, stick held | REQ-SAFE-29 (a critical member's first crash restarts at once), REQ-SAFE-31; no time stated | record the `PAUSE` in the `motor` request stream (it reads up to about 10 ms long: a killed node's log loses its last write interval, REQ-OPS-31), the new run's first request, and its first `kraken=voltage`; the manifest shows one run ended `crash` and a new one. This trial does not test the owner timeout: the restarted node's first request reaches the bridge within it |
| 5 | not run on this bench: it drops only the main processor's outgoing datagrams while States keep arriving, which on Linux is `tc netem`; the tree has no tested macOS counterpart | REQ-SAFE-16: the RoboRIO neutralizes within `command_timeout_us` of the State the last command was built from, plus its loop's lateness ([the tick](../ARCHITECTURE.md#the-tick)) | record "not run (macOS main processor)"; paths 6 to 8 read the RoboRIO's own expiry time from its console instead |
| 6 | (C2) pull the Ethernet cable at the **Mini's** port (`en0`; the Driver Station's link, workstation to RoboRIO, stays up), stick held; then release the stick, reconnect after about 5 s, robot still enabled | REQ-SAFE-16 (bound as in 5); REQ-SAFE-13 | the motor stops; one RoboRIO console line `hardware gateway command expired: outputs neutral at [<s>.<ms>], <x> ms after its observation, ...`: pass at `<x>` of 105 ms or less (`command_timeout_us` plus one 5 ms loop period), recording `<x>` and any excess (the loop's lateness); the first State after the `/hw/state` `PAUSE` has `active=False` and a higher `epoch` than the last before it; no motion on reconnect; `route -n get "$RIO"` again names `interface: en0`; record the time until control returns (session D) |
| 7 | `kill -9 <hardware_node pid>`, stick held | REQ-SAFE-16, REQ-SAFE-29 (the bridge restarts at once) | the motor stops; unless the `/hw/state` `PAUSE` is under 100 ms, the restarted bridge's first State has `active=False` and a higher `epoch` (the pause reads up to about 10 ms long, as in 4); when the command expired, the console's expiry line, read and passed as in 6; record whether motion resumed before the stick was released and pushed again (it does only if the Driver Station view never went stale, REQ-SAFE-13) |
| 8 | Ctrl-C in T1, stick held, then release (the robot stays enabled, with no main processor); this ends C2 | REQ-OPS-06, REQ-SAFE-16 | the motor stops; the console's expiry line, read and passed as in 6; session D's first State has `active=False` and a higher `epoch` than C2's last |

No State reaches the Mini during paths 6 to 8, so the instant the RoboRIO
went neutral is its console's expiry line
([RoboRIO](../talOS/hardware/README.md#roborio)): `[<s>.<ms>]` is the tick
that invalidated the command, on the clock of `sample_us`, and `<x>` its
delay from the State the command was built from, the bound stop path 5 would
have measured. The Driver Station log's `companion lost: no frame for
<x> s; every output is neutral` follows each loss by 0.5 s.

### Session D: relaunch and e-stop (10 min)

At once after C2, the robot still enabled and the stick released, in T1:
`./tools/deploy.sh run --prefix "$PREFIX" -- --no-studio --remote "$RIO" --output-dir "$ROOT/D"`.

- [ ] Control returns (TODO.md ARCH-4, a process restart): the dead time is
  the first State with `active=True` in session D (`sample_us`, the
  RoboRIO's clock in µs) minus the loss, the `companion lost` line's
  `[<s>.<ms>]` less its `<x> s`, the same clock in seconds. Measure stop
  paths 6 and 7 the same way (for 7 with no `companion lost` line, from the
  last State before the `PAUSE`). It includes the person's relaunch here
  and the Mini's link and address setup in stop path 6; record them as
  such.
- [ ] Stop path 9, the e-stop: drive, then the spotter presses the Driver
  Station's e-stop. REQ-SAFE-02, REQ-SAFE-18; no stated time. Pass as for
  stop path 1, with the Driver Station packet's `estop=True`. The robot
  stays e-stopped until the RoboRIO restarts: Ctrl-C in T1, then restart
  the RoboRIO.

### Session E: main-processor power interruption (optional)

For ARCH-4's power cut, if the Mini may lose power safely: after the
RoboRIO's restart, with a session launched as in session D but with
`--output-dir "$ROOT/E"`, the robot enabled and the stick released, cut the
Mini's power and note the time. Restore it, start the Mini (its power
button, unless `pmset -g` shows `autorestart 1`), log in, open T1 and T2 as
before, repeat step 6 of the desk list and relaunch into `"$ROOT/E2"`.
Record, with a clock, power restored, the login window, the login and the
relaunch; and the dead time as in session D, which includes all of them,
since the boot service cannot reach this RoboRIO ([boot
operation](#boot-operation)). The interrupted session's manifest is
unfinished and its logs end at their last sync: keep it as it is. The cold
boot to the first accepted request without a person needs the boot service
on the robot network's addresses, which on the Mini also needs FileVault
off (TODO.md DEPLOY-8).

### Reading a trial

`"$TRIALS" "$S" > "$S/stop_paths.txt"`
([`stop_paths`](../talOS/launcher/stop_paths.py), which runs `log_dump` on
`/hw/state`, `/hw/command`, `/hw/state/driver_station`,
`/hw/request/motor` and `/motor/target/teleop`) prints, for the publishing
process of each stream, one line when its value changes (with when the
previous value was last seen), a `PAUSE` line for every silence of more than
30 ms in a stream published every tick (`/hw/state`, `/hw/request/motor`,
`/motor/target/teleop`), and a `LOG GAP` line where a log lost records (a
pause inside one is the log's, not the robot's). Times are `publish_ms`,
milliseconds from the start of the session's logs on the Mini; `sample_us`
is the RoboRIO's clock. A State line reads `sample_us`, `enabled`,
`active`, `epoch`, `since_us` (the `sample_us` of the first State that
acknowledged the current command), `kraken_v` and `kraken_rps`, and is
printed when the enabled or active flag, the epoch, or whether `kraken_v`
is at 0 (under 0.05 V) changes; a command or request line reads
`observed_us` and `kraken=<mode> <demand>`; a Driver Station line `enabled`,
`estop` and `mode`; a lane line its decoded envelope. It exits 1, printing
nothing on stdout, if the hardware packets were not decoded (a session of
another protocol: read it with a build of that protocol, as `log_dump`'s
warning says), and prints `UNDECODED` for a packet its codecs rejected.

Each trial is a few lines of `stop_paths.txt`. Where `kraken_v` reads
`None`, the log was read without the config's device names: add
`--config "$PREFIX/current/config/robot_bench.toml"` after `"$S"`.

### After the hour

- [ ] For each session: `"$REPORT" --strict "$S" > "$S/session_report.md" 2> "$S/session_report.stderr"; echo $?`.
  Session B prints `0`; A, C1, C2 and D print `2`, with their reasons in
  `session_report.stderr` (A never enabled; the others' kills and restarts
  are their purpose), and every reason there must be one this hour caused.
  Write each session's `stop_paths.txt`.
- [ ] Keep `$ROOT` whole (manifests, `.tlog` files, every `<name>.log`,
  `session_report.md`, `stop_paths.txt`) and copy it with `cp -a`, with the
  Driver Station log of the hour, to the team's archive; the results entry
  names that path. Launch nothing more into these output roots.
  `./tools/deploy.sh logs "$ROOT/B"` lists a session with its revision.
- [ ] Add a dated entry to [BENCH_RESULTS.md](BENCH_RESULTS.md) from this
  template, one line per stop path, with each measured value, its bound
  and PASS or FAIL (a path with no stated bound is recorded, not judged):

```markdown
## YYYY-MM-DD

- **BENCH-8, TIME-3, BENCH-2 (b), ARCH-4 (REQ-RT-15, REQ-SAFE-02, -07, -15,
  -16, -18, -29): enabled stop paths at a <cap> V cap.** Bench, RoboRIO
  build <gateway_build_id>, Kraken X44 id 1 on CANivore `Shooter`,
  commissioned, teleop at about <volts> V; Mac mini (<hw.ncpu> cores,
  `uname -rv` <...>, load <...>) as main processor, Linux workstation as
  Driver Station (<program>), revision <git_revision> (`git_dirty` false),
  config <config_sha256>; realtime `full` for the bridge and every control
  loop. Sessions kept at <archive path>.
  - Acceptance (session B, <id>): `session_report --strict` exit <0>;
    `/hw/state` enabled median <...> ms, p99 <...> ms (bound 10), no
    enabled gap of 100 ms; target age at tick max <...> ms (bound 25);
    applied voltage within ±<cap> V. PASS
  - Latency (REQ-RT-26): accepted command age p99 <...> ms (bound 11), max
    <...> ms (bound 15); Driver Station refresh to command applied p99
    <...> ms (bound 40). PASS|FAIL|not printed by this revision
  - Driver Station disable (3 trials): neutral in the disabling State
    <yes/no>; applied voltage at 0 after <...>, <...>, <...> ms. No stated
    bound.
  - Stale target (teleop stopped <...> ms): <...> ms to a neutral request
    (bound 105; motor timer lateness max <...> ms); re-armed only after
    release <yes/no>. PASS|FAIL
  - Owner timeout (motor stopped <...> ms): <...> ms to a neutral command
    (bound 50 plus bridge latency <...> ms; excess <...> ms). PASS|FAIL
  - Motor node killed: restarted once; first request <...> ms, driving
    again <...> ms after the last logged request. No stated bound.
  - Commands dropped (stop path 5): not run (macOS main processor; no
    tool in the tree drops only the Mini's outgoing datagrams).
  - Ethernet pulled at the Mini: first State after it neutral,
    epoch +<n>; control back <...> s after the loss. PASS|FAIL
  - Bridge killed: neutral, epoch +<n>; resumed before release <yes/no>;
    control back <...> s after the loss. PASS|FAIL
  - Launcher stopped: neutral, epoch +<n>; control back <...> s after the
    loss, relaunch included. PASS|FAIL
  - E-stop: neutral in the e-stopping State <yes/no>; applied voltage at 0
    after <...> ms. No stated bound.
  - Main-processor power cut: <not run | control back <...> s after the
    loss; power restored to login window <...> s, login <...> s,
    relaunch <...> s>.
```

### What this session does not verify

The RoboRIO loop watchdog (REQ-SAFE-25; the
[stall check](#fault-injection-and-timing)); the launcher's own death and
its guardians (REQ-SAFE-30; the launcher is stopped here, not killed);
feedback loss, apply failures and device resets (REQ-SAFE-22 to
REQ-SAFE-24); digital and PWM outputs (REQ-SAFE-03; the bench has none);
position mechanisms and rezero (REQ-SAFE-05, REQ-SAFE-36 to REQ-SAFE-38);
VELOCITY mode under the cap (buttons 1 and 2 are not pressed); the swerve
module (TODO.md BENCH-4); a loss of the Driver Station's own link to the
RoboRIO ([RoboRIO program checks](#roborio-program-checks-with-a-driver-station)); a RoboRIO reboot under a running stack
(REQ-SAFE-11); a
cold boot without a person; and loop lateness under a match's load
(REQ-RT-14, TODO.md PLATFORM-1): the Mini runs only the bench's six nodes.

## RoboRIO program checks with a Driver Station

The RoboRIO program's checks that need its reboot or a real Driver Station
(TODO.md BENCH-3). They run on the commissioned bench of the
[supervised session](#one-supervised-enabled-session), after its hour or as
their own sitting, with its people, its two terminals on the Mini, its
variables, the RoboRIO console recorded on the workstation, and the same
rules for what may move: the driver drives only axis 1 at about half
deflection, the spotter keeps the disable and e-stop. Each launch gets its
own output root and `S` as there.

1. **The saved configuration survives a reboot.** Robot disabled. In T1:
   `./tools/deploy.sh run --prefix "$PREFIX" -- --no-studio --remote "$RIO" --output-dir "$ROOT/F1" --duration-s 60`.
   Once the console shows `companion connected`, in T2:
   `ssh admin@10.0.0.72 'ls -l /home/lvuser/talos_config.bin; md5sum /home/lvuser/talos_config.bin'`.
   Expected: the file is listed and its checksum printed; record it. After
   the session ends, reboot the RoboRIO: `ssh admin@10.0.0.72 reboot`. The
   console recording ends with the link; once the Driver Station shows
   robot code again, start it again into a second file
   (`rio_console_<date>_2.log`) and repeat the `ssh ... md5sum` command.
   Pass: the same checksum, and the program's start lines read
   `[<s>.<ms>] hardware gateway config=<id> commissioned=1 (waiting for config push)`
   with `<id>` not `0` (`config=0` is a missing or unreadable file:
   [configuration](../talOS/hardware/README.md)).
2. **The `[<s>.<ms>]` prefix on the Driver Station.** In the Driver
   Station program's message view (not the console), from step 1's session
   and a second launch into `"$ROOT/F2"` stopped with Ctrl-C in T1. Pass:
   `[<s>.<ms>] companion connected` at each launch and
   `[<s>.<ms>] companion lost: no frame for <x> s; every output is neutral`
   after the stop, each with the prefix the console shows for the same
   line. The loop watchdog's message, `[<s>.<ms>] RoboRIO loop stalled past
   an expired command: ...` under `talos loop watchdog`, shows only during
   the [loop stall check](#fault-injection-and-timing); when that check runs
   with this Driver Station, the same pass applies to it.
3. **A stick held through a disable and an enable stays released**
   (REQ-SAFE-13). In T1:
   `./tools/deploy.sh run --prefix "$PREFIX" -- --no-studio --remote "$RIO" --output-dir "$ROOT/F3"`.
   Enable with the stick released, drive at about half deflection, and,
   the stick held, the spotter disables, then re-enables after about 2 s;
   the driver keeps holding for 3 s more, then centres the stick and pushes
   it again. Disable, release, Ctrl-C in T1, then
   `"$TRIALS" "$S" > "$S/stop_paths.txt"` ([reading a trial](#reading-a-trial)).
   Pass: nothing moved between the re-enable and the push after centring;
   in `stop_paths.txt`, from the disable on, `/motor/target/teleop` reads
   `active: false` and `/hw/request/motor` `kraken=neutral`, and after the
   re-enable the first `kraken=voltage` request comes after the first
   lane line that reads `active: true` again (the stick returned to
   neutral).
4. **Pulling the Driver Station's network disables.** A session as in
   step 3 into `"$ROOT/F4"`, enabled and driving with the stick held. On
   the workstation find its interface to the RoboRIO,
   `ip route get 10.0.0.72` (`dev <interface>`), then cut it, stamping the
   RoboRIO's clock first:
   `ssh admin@10.0.0.72 cat /proc/uptime; sudo ip link set <interface> down`.
   The motor must stop; release the stick, `sudo ip link set <interface> up`
   after about 5 s, leave the robot disabled, Ctrl-C in T1, and write
   `stop_paths.txt` as in step 3. The console recording ends with the link:
   restart it as in step 1. Pass: the first `/hw/state` line with
   `enabled=False` also reads `active=False`, and a later one `kraken_v`
   at 0; the Driver Station lines read `enabled=False` or `mode=none` from
   then on until the person enables again. Record the pull to the disable,
   that State's `sample_us` minus the first `/proc/uptime` figure in µs
   (the uptime is the console's clock, that of `sample_us`, read one `ssh`
   exit early). The bound is NetComm's detection time plus 20 ms (the
   `talos-ds` refresh) plus one 5 ms tick (REQ-RT-24); no document states
   NetComm's time, so record the value, not a verdict.
5. **No WPILib joystick warning.** Over every console file of steps 1
   to 4: `grep -ci joystick rio_console_<date>.log rio_console_<date>_2.log`.
   Pass: each prints `0`. The program reads the Driver Station without the
   getters that warn about a missing joystick, axis or button
   (`DriverStationReader.cpp`); a WPILib warning there means a call that
   goes through them.

Record each step, dated, in [BENCH_RESULTS.md](BENCH_RESULTS.md).

## Boot operation

The boot service runs `run.sh` with no arguments, so it uses
`robot_bench.toml`'s robot-network address (`10.56.87.2`) and cannot reach
this bench's RoboRIO ([bench network](#bench-network)); on the Mini it also
needs FileVault off (TODO.md DEPLOY-8). These checks run on the robot
network's addresses, with the bundle installed under the default prefix
(`sudo ./tools/deploy.sh install bazel-bin/bench/main_processor/bench_bundle`)
and a RoboRIO program built without `-Pcompanion`.

- [ ] Inspect `./tools/deploy.sh enable-boot --user <service-account> --dry-run`,
  then enable boot with the actual service account. Expected: bundle traversal
  succeeds, `/opt/talos/logs` is writable by that account (0755), bundle remains
  read-only. Record ownership and launchd configuration. Stop boot-managed
  execution before starting a manual launcher: exactly one stack may run.
- [ ] Disable/release, stop manual execution, and run
  `./tools/deploy.sh enable-boot --user <service-account>`. Inspect
  `sudo launchctl print system/org.talos.robot`, then reboot the Mini with
  nobody logged in. Expected: one installed stack starts using the robot
  network's address, logs are writable without repair, config hash matches, and every
  motor remains neutral while disabled. Record launchd status, boot session and
  disabled observations. For regression use `./tools/deploy.sh rollback`,
  stop/restart, and recheck the selected whole-bundle hash before use.
