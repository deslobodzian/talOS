# Hardware Gateway

The gateway configures devices, reads status signals, and applies actuator
requests. Swerve kinematics run in the companion drivetrain process. TalonFX
controllers own their PID and Motion Magic calculations; the gateway does not
implement a second motor PID loop.

`Backend` in `gateway.h` is the platform boundary. `PhoenixBackend` under
`controller_processor/rio/` implements it with Phoenix 6 on the
RoboRIO. `SimBackend` (`sim_backend.h`) provides ideal actuators and
`sim::PhysicsSimBackend` (`sim_physics.h`) a physics-backed model for software
tests and `sim_gateway`.

## Configuration

The bridge (`//talOS/bridge:hardware_node`) parses the robot's TOML
configuration and pushes it to the gateway as chunked `kHardwareConfig`
frames; `Endpoint` reassembles them, starts `Gateway::BeginReconfigure`, and
answers with a `kHardwareConfigAck` when the worker finishes. An ACK whose
send fails is sent again at the start of each later tick until a send
succeeds or a newer transfer completes (only the newest completed transfer's
ACK goes out), and `Endpoint::Tick` returns the failure,
so the RoboRIO program replaces a socket that can no longer send. A push
completed while a configure is still pending (every boot restore) is dropped
without an ACK; the bridge pushes again after its 50 ms retry gate. The
manifest holds
logical device IDs, names, CAN
addresses and bus names, inversion, encoder offsets, current/voltage limits,
gearing, three gain slots, soft limits, TalonFX limit switches, FOC, the
torque-current limit, and Motion Magic cruise velocity, acceleration, and
jerk. Changed configurations
require disabled operation: the RoboRIO console names each push refused
while enabled, `hardware gateway refused configuration <new> while enabled
(running <old>): disable to accept the new configuration` (`<old>` is `0`
with none in force; `Gateway::last_enabled_refusal()`, posted by the
program's `LoopConsole`), and the bridge pushes again, backing off to once
every 2 s. Identical pushes may succeed while enabled
without device traffic, but not while faulted or while
configuration/restoration is pending.

The RoboRIO atomically and durably (synced file, rename, synced directory)
saves acknowledged configuration bytes, tagged with the magic `TALOSCFG`
and the protocol version that wrote them and followed by a CRC32
(`PersistedConfig` in `config_wire.h` gives the layout), to
`/home/lvuser/talos_config.bin` (`talos_config.bin` in desktop simulation;
`ConfigStore` in the RoboRIO program). The loop hands the acknowledged
configuration over on its first disabled tick after the acknowledgement, or
at a clean exit (`ConfigPersister::Stop`), and a saver thread (`talos-config`, nice 19) syncs storage while the loop ticks
on: the loop only try-locks the handover, never waits on the save, and polls
its outcome on later ticks. The loop encodes the configuration into a preallocated slot and swaps it by
index under the lock; once constructed, neither the loop thread nor the saver
thread allocates or frees during configuration persistence (system calls only,
on paths built at startup). A
configuration acknowledged while a save runs is
saved after it, newest last; a failed save prints `cannot persist hardware
configuration`, keeps the previous file, and an identical push saves it
again. Startup validates
them and starts their restoration on the configure worker without waiting for
it, so robot code reports ready at once; the gateway stays unconfigured,
refusing commands, until the worker finishes. Missing or corrupt files, and
a saved configuration this build's validation refuses, leave the gateway
unconfigured. So does a file another protocol version wrote, refused by its
tag whatever its body, and a file from before the tag (protocol 11 or
older): the RoboRIO boots unconfigured, printing the same
`hardware gateway config=0 ... (waiting for config push)` line as for a
missing file, until the bridge pushes its configuration on link. The first
boot after a RoboRIO program of a new protocol version is deployed therefore
always waits for that push.
A stale file cannot accept commands from a different configuration ID.

A structurally valid configuration succeeds in degraded mode when individual
Phoenix devices time out or are absent. Failed devices publish invalid samples;
their motors, remote-feedback consumers and followers stay neutral, while
healthy devices continue accepting commands. Invalid samples feed the existing
bridge motor fault mask; sanitized demands increment the neutralized-motor
counter. Network and persisted startup configuration CAN transactions run on a
worker, not the periodic RIO loop.
Unavailable devices retry through the same per-device restore worker
after 250 ms, doubling to 2 s. Rejected whole configurations make the bridge
back off from 100 ms to 2 s without stopping its command courier; a new gateway
boot resets that delay.

Logical IDs remain independent of CAN addresses. Pigeon2 `offset_rot` configures
Phoenix MountPoseYaw (rotations in `[-0.5, 0.5)`); `inverted` reverses yaw and
yaw rate after mounting correction. Both simulation backends use the same
`(raw_yaw + offset_rot) * sign` convention. The ideal fake has zero raw yaw.
Steering uses a CANcoder on the same CAN bus as its TalonFX. Different buses
may reuse CAN addresses. Physics reset restoration preserves physical rotor
positions while clearing velocity, commands and applied voltage.

Example addresses and zero gains are not commissioned settings. After matching
the manifest to the physical robot and tuning it, explicitly set the hardware
`commissioned` flag. Until then telemetry is available but commands are
refused. A commissioned config also requires
[target coverage](../../ARCHITECTURE.md#targets).
On the main processor only `--sim` bypasses commissioning, in memory, and the
bridge and every node apply it under that same flag: the bit is part of the
configuration ID they compare. `--sim` rejects non-loopback remotes; a loopback
remote without it keeps the configured value. A config that sets
`[robot] simulation_only = true`, which the launcher starts only with `--sim`
([lightweight simulation gateway](../launcher/README.md#lightweight-simulation-gateway)),
is refused the same way by `hardware_node` started by hand without `--sim`,
before it opens a socket; its `--describe` still answers:

    hardware_node error: /path/robot_sim.toml sets [robot] simulation_only = true; hardware_node runs it only with --sim

Both endpoints compute a canonical configuration fingerprint. Changing a CAN
address, offset, limit, or gain changes that ID; commands with a different ID
are refused. This is a compatibility check, not authentication. A configuration
with no device at all, or no motor, is valid: a robot is brought up (link,
Driver Station, telemetry) before its first mechanism, and with no motor or
CAN sensor there is no feedback to lose, so no total-loss fault. Configuration
validation rejects duplicate IDs/names/addresses, invalid numeric values,
remote encoder references that do not exist on the motor's bus, and a
`status_hz` giving fewer than four status periods per `command_timeout_us`
(feedback that old is invalid; see "Fault scope" below).

Device tables reject unknown keys in both runtime parsing and manifest checking.
CANcoder and Pigeon2 accept `type`, `bus`, `can_id`, `inverted`, and `offset_rot`.
QuadratureEncoder accepts `type`, `dio_a`, `dio_b`, and `counts_per_rev`;
DutyCycleEncoder accepts `type`, `dio`, and `offset_rot`; `digital_outputs`
accepts only `dio`, `digital_inputs` accepts `dio` plus the optional
`sim_motor`, `sim_closed_at_rot`, `sim_closed_above`, `sim_closed_reads` and
`sim_sensor` keys, and analog inputs and PWM outputs accept only `channel`.

Motor tables may specify `follow = "master_motor"` and optional
`oppose_master = true` (default false). The master must be a motor owned by
the same subsystem on the same CAN bus; self-follow and follower chains are
rejected. Followers use the master's output rather than their own demand,
and neutralize when the master is neutral or loses valid feedback.
Both simulation backends support same-direction and opposed followers.

`foc = true` (default false) makes the RoboRIO send that motor every
duty-cycle, voltage, velocity, position and Motion Magic request with
Phoenix's `EnableFOC` set (FOC commutation; without it, none is). FOC needs a
Phoenix Pro licence; the device reports whether it is licensed (whether a
CANivore's licence shows there is a bench check). Config load refuses
`foc` on a follower (`foc for <follower>: a follower follows its master's
output; set foc on <master>`): the follower copies its master's output. The
key is in the configuration fingerprint and wire format.

At each configure the RoboRIO reads the `IsProLicensed` signal (waiting up
to 0.5 s, one motor after another) of each motor whose own or whose
master's `foc` is set, except a follower whose master was already read
unlicensed. A restore reads it again only for a motor whose read got no
answer, so a reset of a motor read licensed or unlicensed costs no read.
A motor read not licensed is a failed device until
the next configure, which is a changed configuration or a restart of the
RoboRIO program (a push of the same configuration touches no device):
invalid samples, `NeutralOut` whatever it is asked, and its bit in
`/hw/status` `motor_fault_mask`. A follower of such a master fails with it,
licensed or not; a master whose follower alone is unlicensed drives while
the follower stays neutral, as with an absent follower. Each failed motor
is named on the RoboRIO console (`[phoenix] config failed for '<motor>' ...
during check_pro_license`). The configure still succeeds and is
acknowledged, so no configure error crosses the link; the backend keeps the
configure's first error (`last_config_error`, RoboRIO-side) only. A read
that gets no answer (the device absent, or its frame late) is not a refusal:
the motor is an unavailable device, and each restore, with the rest of its
configuration, reads the licence again until one gets an answer. The rest
of the robot drives,
unless every motor and CAN sensor has failed: then the gateway's total
feedback loss also holds DIO and PWM outputs neutral. Desktop simulation
needs no licence and checks none.

`max_torque_current_a` (amperes, default 0) is the largest stator current a
torque-current demand may ask of the motor; 0 refuses the mode. Config load
refuses a value that is negative or non-finite, above `stator_limit_a`, set
without `foc = true` (TorqueCurrentFOC is a Phoenix Pro request), set on a
follower, or set on a motor another motor follows, naming the motor and the
key, and for a follower relation both motors (`hardware::Validate`, which the
RoboRIO also runs on a pushed config). Torque-current mode drives a single
motor: Phoenix's Follower copies its master's torque current but not its
`MaxAbsDutyCycle`, so `max_voltage` would not hold a follower. The key is in
the configuration fingerprint and wire format.

`position_frame` is `"rezero"` (the default) or `"absolute"`, which config
load accepts only with `feedback = "RemoteCANcoder"`, since a TalonFX reset
restarts its rotor position. Only the main processor reads it (it is not in
the wire format or the configuration fingerprint): the devices layer holds a
rezero-frame motor's position and motion-magic demands neutral until the
node's `Motor::rezero`, and again after a device reset or gateway boot, while
an absolute-frame motor drives in the CANcoder's frame from its first state
(see [position frames](../../docs/NODE_DEVELOPMENT_GUIDE.md#rezero-and-position-frames),
including when that frame survives a CANcoder reset). Config load refuses
`soft_limits = true` on a `"rezero"` motor: the TalonFX enforces soft limits
on its own position in every control mode, from thresholds applied once at
configuration, so after a reset at 30 rotations limits of [0, 50] would allow
30 rotations past the top, and no rezero moves them.

`motor_type` names the motor the TalonFX drives: `"kraken_x60"` (the
default), `"kraken_x44"` or `"falcon_500"`; config load refuses any other
value, naming the key, the motor and its file. It is not in the wire format or
the configuration fingerprint, so a config the bridge pushes over UDP carries
none: `sim_gateway --physics` reads it locally from its own parse of `--config`
(`PhysicsSimBackend::SetLocalMotorTypes`) for its winding model (stator
current, free speed; see the backend table below), while the WPILib desktop
simulator (`PhoenixBackend` under `./sim.sh`) receives only pushed configs and
therefore defaults to `KrakenX60` (or `KrakenX60FOC` when `foc = true`),
selecting `KrakenX44` or `Falcon500` only when `MotorConfig::motor_type` is
populated in-process (e.g., unit tests).

`forward_limit_switch` and `reverse_limit_switch` set the switches wired to
the TalonFX's own forward and reverse limit inputs: `"NormallyOpen"` (the
default, and Phoenix's), `"NormallyClosed"` or `"Disabled"`. The TalonFX
itself holds output toward an asserted switch neutral, in every control mode
and whatever its position frame, so it guards travel that no software zero or
soft limit can (a reset at height moves both). An absent key keeps Phoenix's
default, so a motor with nothing on its limit pins is unchanged, and one that
already has a switch there already obeys it. A normally closed setting with
no switch wired reads asserted and stops that direction. Only the motor's own
pins are supported (no remote CANcoder or CANdi source, no position autoset).
Both settings are in the configuration fingerprint and wire format. An
asserted switch is the ForwardHardLimit/ReverseHardLimit bit of the motor's
Phoenix fault bitfield (`kMotorForwardHardLimitFlag`,
`kMotorReverseHardLimitFlag`), which a node reads as `Motor::forward_limit()` /
`reverse_limit()`. Those bits are not counted as faults (`MotorFaulted`): a
normally closed switch whose wire is cut holds its direction neutral without
raising `/hw/status` `motor_fault_mask`, but it shows, as a switch that stays
asserted away from the end of travel, in `motor_forward_limit_mask` /
`motor_reverse_limit_mask`. Nor are the soft limits or the active supply and
stator current limits (SupplyCurrLimit, StatorCurrLimit), which the TalonFX
raises for a few milliseconds on a hard acceleration from rest: a limit or
limiter acting as configured is not a fault (`kMotorNotFaultFlags` in
`messages.h`, the one list the bridge, the hardware log and the RoboRIO's
console share). Those bits stay in each sample's `fault_flags`; every other
FaultField bit faults the motor. `PhoenixBackend` reports none of them from a
FaultField signal 50 ms old or older, since an old switch or limiter state is
a wrong display, but keeps that signal's last fault bits: losing the signal
does not un-fault a motor.
`sim::PhysicsSimBackend` holds forward or reverse output neutral, and reports
the bit, while a test presses a switch
(`SetForwardLimitPressed`/`SetReverseLimitPressed`; `sim_gateway`'s
`forward_limit`/`reverse_limit` fault schedule); the ideal `SimBackend` has no
way to press one.

A digital input may carry a simulated home switch:
`sim_motor = "<motor>"` (a motor of the same subsystem) and
`sim_closed_at_rot` (finite, required with `sim_motor`), with
`sim_closed_above` and `sim_closed_reads` (both default false). Config load
refuses `sim_closed_at_rot` or `sim_closed_above` without `sim_motor`, a
`sim_motor` that names no motor of the input's subsystem, and a non-finite
`sim_closed_at_rot`, naming
the key and the input. Like `motor_type`, every config load accepts them but
only the physics simulator reads them; they are not in the wire format or the
configuration fingerprint. `sim_gateway --physics` binds them from its own
parse of `--config` (`PhysicsSimBackend::SetHomeSwitches`; the backend table
below gives the switch's behaviour).

A digital input may instead name a sensor of the MuJoCo model
(`[simulation.mujoco]` below): `sim_sensor = "<sensor>"` (a non-empty
string), with `sim_closed_reads` (default false), the value the input reads
while that sensor detects. Config load refuses `sim_sensor` with
`sim_motor`; `sim_closed_at_rot` or `sim_closed_above` with `sim_sensor`;
and `sim_closed_reads` with neither `sim_motor` nor `sim_sensor`, naming
the key and the input. Like `sim_motor`, it is not in the wire format or the
configuration fingerprint. Config load does not open the model, so a
`sim_sensor` naming a sensor the model lacks is not refused at load.

An optional top-level `[simulation]` table configures physics chassis geometry,
end-of-travel hard stops, battery sag, chassis dynamics, per-mechanism
inertia/friction and the MuJoCo model:
- `wheel_radius_m` (positive finite double, default `0.0508` m),
- `[[simulation.swerve_modules]]` entries (`drive`, `steer`, `x_m`, `y_m`, where
  `drive` and `steer` name distinct configured motors, no motor appears twice,
  and `(x_m, y_m)` is finite and nonzero),
- `[[simulation.hard_stops]]` entries (`motor` naming a configured motor with no
  duplicates, optional finite `min_rot` and/or `max_rot` with at least one set
  and `min_rot < max_rot` when both are set, in mechanism rotations of the
  motor's hardware frame),
- `[simulation.battery]` (`nominal_voltage_v` finite in `(0, 16]`, default
  `12.8`; `internal_resistance_ohm` finite `>= 0`, default `0.0`;
  `base_current_a` finite `>= 0`, default `0.0`; `brownout_voltage_v` finite in
  `[0, nominal_voltage_v)`, default `6.75`),
- `[simulation.chassis]` (`mass_kg` required positive finite; optional positive
  finite `moment_of_inertia_kg_m2` and `traction_coefficient`), and
- `[[simulation.mechanisms]]` entries (`motor` naming a configured master motor
  with no duplicates; `inertia_kg_m2` required positive finite at the mechanism
  shaft; optional `load`, `"roller"` (default) or `"flywheel"`; optional
  non-negative finite `viscous_damping_nm_per_radps` and `static_friction_nm`),
- `[simulation.mujoco]`, the MuJoCo model: `robot_model` (required, the
  robot's MJCF file), `robot_model_files` (optional array: every other file
  the model reads, such as meshes) and `field` (optional field model name,
  today `"rebuilt_2026"`; absent or `""` is flat carpet). The file that sets
  `robot_model` sets the whole model: `robot_model_files` comes only from
  that file (absent there, none), so a file that includes another and sets
  `robot_model` discards the included file's list, while `field` overrides
  on its own. Each path is relative to the file that sets it and never
  absolute; each file must exist, and the build lists it in the config's
  `CONFIG_FILES`, so `talos_sim` and `talos_bundle` carry it, as
  `[trajectories]` files are. Config load refuses an unknown key, a
  non-string or empty path, a non-array `robot_model_files`,
  `robot_model_files` in a file that does not set `robot_model`, and an
  unknown `field`, naming the key and the file that sets it. It checks only
  that the files exist and the field name is known: it does not open the
  model, so a file the model reads but the list omits is not refused at
  load.

Like `motor_type` and `sim_motor`, `[simulation]` stays off the wire and out of
`ConfigurationId`; `sim_gateway --physics` passes `wheel_radius_m` in
`PhysicsOptions` and binds `swerve_modules` (`SetSwerveModules`), `hard_stops`
(`SetHardStops`), `battery` (`SetBatteryModel`), `chassis`
(`SetChassisDynamics`), and `mechanisms` (`SetMechanisms`) on
`PhysicsSimBackend` before `Reconfigure` so all of them survive any later
pushed config.

Each `[subsystems.<sub>.motors.<motor>.slot0]` (also `slot1`, `slot2`) accepts
`gravity_type = "ElevatorStatic"` (default; alias `"Elevator"`) or
`gravity_type = "ArmCosine"` (alias `"Arm"`). Gain `g` supplies constant
elevator compensation or cosine arm compensation, respectively, with the arm
angle measured in mechanism rotations and zero at horizontal. These fields
and follower settings participate in the configuration fingerprint and wire
format, so both endpoints must use the matching software version.

Backend configuration failures can latch a detailed `ConfigError`
(`config_error.h`) on the gateway (`Gateway::config_error()`, cleared by the
next success): device name, CAN bus, CAN ID, operation, vendor status and
message. Rejections before backend configuration need not populate it.

## Supported Hardware Requests

| Mode | Demand | Feedforward |
|---|---|---|
| Neutral | Ignored | Zero |
| Duty cycle | -1 to +1 | Zero |
| Voltage | Volts | Zero |
| Velocity | Mechanism rotations/second | Volts |
| Position | Mechanism rotations | Volts |
| Motion Magic position | Mechanism rotations | Volts |
| Torque current | Stator amperes, within ±`max_torque_current_a` | Zero |

Each closed-loop request selects gain slot 0, 1, or 2. Drive motors use rotor
feedback divided by the configured mechanism ratio. Steering uses
`RemoteCANcoder` with continuous wrapping. The backend uses Phoenix's voltage
controls, which need Phoenix Pro only for a motor configured `foc = true`
(above). A torque-current request is TorqueCurrentFOC with peak torque
current ±`max_torque_current_a` (0 A on a motor without the key) and a
maximum duty cycle of `max_voltage` over the RoboRIO's last battery reading
that was finite and above 1 V (0, no output, before the first);
`stator_limit_a` and `supply_limit_a` apply too. Positive amperes drive the
direction positive volts do, after `inverted`. The request is one-shot, so
its cap is as old as the last command the RoboRIO applied: one node period
while the demand changes, at most the bridge's command heartbeat otherwise
([DATA_FLOW.md](../../docs/DATA_FLOW.md)). It divides by the RoboRIO's input
voltage reading, not the TalonFX's bus voltage, so a reading taken in a sag
raises that interval's ceiling in proportion (read 9 V, actually 12.5 V:
4 V becomes 5.6 V) and a stale high reading errs low. The gateway neutralizes a torque-current request that
`TorqueCurrentAllowed` (`gate.h`) refuses: its motor lacks `foc = true` or a
positive `max_torque_current_a`, its demand exceeds that limit, or its
feedforward is not 0; the RoboRIO sends NeutralOut for any such request
that reaches it. A 0 A demand is a driving request, not talOS's neutral:
without a live target the devices flush holds it like any set output
(`outputs held`), otherwise it is applied as sent, as TorqueCurrentFOC; only
`NeutralOut` is neutral. What the motor does at exactly 0 A is not yet
measured: zero torque by CTRE's `Deadband` description, or its `brake`
NeutralMode by `OverrideCoastDurNeutral`'s. The physics simulator models
zero torque (a free rotor spins on); the RoboRIO's desktop simulation stops
the rotor.
FusedCANcoder, dynamic Motion Magic, and velocity or position
TorqueCurrentFOC requests are not implemented. Duty-cycle
encoder velocity is finite-difference across Reads.

Besides motors and CAN sensors, the manifest accepts RIO digital inputs and
outputs, analog inputs, quadrature and duty-cycle encoders, and PWM outputs.
Phoenix and both sims implement all of them.

### Device scope

The supported hardware scope is CTRE Phoenix 6 CAN devices (`TalonFX`,
`CANcoder`, `Pigeon2`) plus RoboRIO onboard I/O (DIO inputs and outputs, analog
inputs, quadrature and duty-cycle encoders, and PWM outputs). REV motor
controllers, pneumatics, addressable LEDs, and CANrange are out of scope. Adding
another device kind or exceeding the per-frame device caps requires growing the
hardware `State` transport beyond a single UDP payload; see
[state frame growth](../../docs/DATA_FLOW.md#state-frame-growth).

A PWM output's request is off or a pulse (`PwmMode`, `messages.h`). Off sends
no pulse (`PWM::SetDisabled`): a servo goes limp and a PWM motor controller
sees no signal. A pulse is `output` on `PWM::SetSpeed`'s scale, -1 to +1,
where 0 is the 1.5 ms centre pulse that centres a servo and holds a PWM motor
controller at neutral. Off is the PWM output's neutral, as `kNeutral` is a
motor's: the bridge sends it for an output its owner's request omits, after
the owner timeout and when a transition invalidates the cached requests, and
the gateway's own neutral turns every PWM output off. A pulse that is not
finite or lies outside -1 to +1 rejects the whole command. A State's PWM
sample reads 0 while its output sends no pulse.

CAN bus load and measured sensor age must be checked on the actual bus before
choosing production frequencies. Motion feedback requests the manifest status
rate; motor voltage, stator and supply current and fault field request
`min(status_hz, 50)` Hz, device temperature `min(status_hz, 4)` Hz. A motor
another motor follows also requests its duty cycle, at motor voltage's rate
(the two share the TalonFX's MotorOutput status frame, `PRO_MotorOutput_*`
in Phoenix's `SpnValue.hpp`, so a faster duty cycle would undo voltage's cap
on a CAN 2.0 bus), and its voltage and duty cycle at no less than 20 Hz: a Phoenix
follower copies its master's output signal (DutyCycle for duty-cycle
requests, MotorVoltage for the others) at that signal's rate, which must be
no slower than 20 Hz, while `status_hz` may be lower. Phoenix applies the
highest requested rate to signals sharing a frame: under the pinned vendor
library's simulation, where every signal but the fault field shares one
frame, voltage/current still apply at 200 Hz alongside 200 Hz motion
feedback, while the fault field applies at 50 Hz. Which signals share a
frame on a physical bus has not been measured. See [vendor signal frequency semantics](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/status-signals.html#changing-update-frequency).
The backend then calls
`OptimizeBusUtilization()` on each TalonFX, CANcoder, and Pigeon2. Per-device
optimization keeps the explicitly selected signals while stopping unused
default frames; it does not replace measuring the real bus. Reads refresh the
cached signals non-blockingly, with one `BaseStatusSignal::RefreshAll` per
CAN bus: Phoenix refreshes none of the signals of a call that spans two
networks (it returns `InvalidNetwork`), so the backend groups the signals by
bus when it configures. One-shot control requests are refreshed by the
bridge heartbeat (at most 20 ms), not on every gateway tick.
Outside pending asynchronous configuration, neutral refresh remains every
gateway tick whenever no accepted command is active: disabled, faulted,
unhealthy, or after command timeout/invalidation, until a fresh accepted
command supersedes it.
`Neutral()` uses one-shot `NeutralOut` (`UpdateFreqHz = 0`), not an
acknowledged persistent disable latch. A dropped CAN neutral or a device reset
can occur during disable or after a watchdog stop; without an established
vendor guarantee covering both cases, reducing refresh to 50 Hz would extend
the retry interval. The cheap bus reduction is therefore limited to telemetry
signals, not safety-output refresh. Phoenix counts non-OK `NeutralOut` sends in
`PhoenixBackend::neutral_send_failure_count()` and logs each motor's
failing/recovered transition; `State` has no field for it, so this delivery
uncertainty is RIO-local. Controller-side expiry of the last one-shot demand is
not established by these tests; the bound on powered output after a watchdog
stop is the gateway retry rate, not a measured device guarantee.

References: [CTRE remote sensor control](https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/remote-sensors.html)
and [Motion Magic](https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html).
Phoenix 6 is pinned in `controller_processor/rio/vendordeps/Phoenix6.json`.

## Runtime Contract

Frame types `kHardwareState` and `kHardwareCommand` use the versioned UDP frame
transport. Their payload codecs, like the config chunk header and ACK, explicitly
encode little-endian integers and IEEE-754 doubles, with exact length checks.
Native C++ object layout is never the network format. The payload limit is
`protocol::kMaxPayloadSize`, currently 1432 bytes; `Packet` and the endpoint
buffer are both sized from that constant.

Each `Endpoint::Tick` drains a bounded batch of frames and hands the gateway
only the highest-sequence command among them: superseded commands queued while
the RoboRIO loop stalled are skipped, not applied back-to-back on the CAN bus,
and are not counted as rejected. `Endpoint::drained()` reports that command
(its `observed_time_us` and whether `Gateway::Accept` took it), or none when
the Tick decoded no command; the RoboRIO program ages it for its
[latency line](#roborio).

The gateway publishes its configuration ID, boot ID, control epoch, monotonic
poll timestamp, last accepted command sequence, the echo of the last clock
probe (the bridge's clock mapping), the tick sequence (one more each gateway
tick within a boot, so a consumer can tell consecutive States from a gap; see
[DATA_FLOW.md](../../docs/DATA_FLOW.md)), its build (`GatewayBuild`: the
program, RoboRIO or WPILib simulation of it or `sim_gateway`, the git
revision it was built from and whether that checkout was dirty; set with
`Gateway::SetBuild`, never acted on), enable/configuration/fault flags,
and identified motor/sensor samples. Motor samples contain mechanism
position/velocity, voltage, stator and supply current, temperature, and the
Phoenix fault bitfield. State also carries valid power health (battery
volts, brownout), one CAN bus's health, and per-boot gateway safety
counters.

The CAN buses are `CanBuses(config)` (`config.h`): slot 0 is the RoboRIO's
own bus, `"rio"`, whether or not a device names it (the power distribution
and pneumatics modules share it), then each other bus a motor or sensor
names (a CANivore), in byte order. A config naming more than three buses
besides `"rio"` is refused, and so is one naming two buses that differ only
by case (`"Shooter"` and `"shooter"`, or `"RIO"` beside the implicit
`"rio"`): Phoenix may resolve them to one bus, whose health and load would
then be split over two slots. Names are never folded, since the wire
carries them as written. Each State carries one slot, `sequence % count`
(`HealthSample::can_bus_index`, `can_bus_count`), with the bus's read state,
utilization, bus-off and transmit-buffer-full counts, receive and transmit
error counters (saturated at 255) and the failed reads since the
configuration was applied; the bridge assembles them into `/hw/status`
`can_buses` and prints the slot table (index and bus name) on stderr when it
opens. A read state is `OK` (the last read succeeded and is current),
`PENDING` (not read yet, within the stale bound of the first tick), `FAILED`
(the last read failed), `STALE` (no read within the bound) or `SIMULATED`
(`sim_gateway`, `SimHardware` and the RoboRIO program under WPILib
simulation, which read no bus; every figure 0); in any state but `OK` the
figures are the last good read's. `alert_warning_can_bus_unread` holds while
a slot reads `FAILED` or `STALE`.

On the RoboRIO a bus status read is a vendor library call: the RoboRIO's
bus through `HAL_CAN_GetCANStatus` (a NetComm call;
`frc::RobotController::GetCANStatus` would throw on a failed one, which
reads `FAILED` instead) and a CANivore through Phoenix's
`CANBus::GetStatus`, which blocks about 1 ms. A reader thread
(`talos-canbus`, `SCHED_OTHER`, started for every configuration with a
device, never for one with none and never in simulation) reads one bus every
100 ms, the buses in turn, and the loop only copies what it published. A
bus whose last read is three of its own sampling periods old on the loop's
clock (600 ms with "rio" and one CANivore) reads `STALE`, and the RoboRIO
console prints each bus's failing spell when it starts and ends. A
configuration with no device reads no bus, and its "rio" slot is reported
as not read (`NONE`). A new configuration (or the backend's destruction)
waits at most 50 ms for the reader to stop; a reader stuck in a status read
that never returns is abandoned with a console line naming the bus, and
while two abandoned readers are still blocked no new one starts and every
bus reads `STALE`
([`CanBusHealth.h`](../../controller_processor/rio/src/main/include/CanBusHealth.h)).
Peers of another protocol version (`kProtocolVersion`, see
[DATA_FLOW.md](../../docs/DATA_FLOW.md)) are rejected rather than decoded.
CANcoder samples contain angle
and velocity; Pigeon2 samples contain yaw and yaw rate, expressed in rotations.
Position/velocity signal ages are included separately, in microseconds. Ages
describe cached Phoenix measurements at polling; samples are not a synchronized
CAN capture.

Commands echo the gateway's boot ID, epoch, and poll timestamp. Expiry is checked
using only the gateway clock, so it does not assume synchronized host clocks.
Duplicate/reordered command sequences, old sessions, expired observations, and
structural errors reject the whole command. Non-finite or out-of-range motor
demands/feedforward, and a torque-current request the motor's configuration
does not allow (above), neutralize only that motor. Per-boot counters report refused
commands, neutralized motors, apply failures, and device resets. The refused
count omits two refusals of an all-neutral command (every motor neutral with
zero demand and feedforward, every digital output off, every PWM output
off): a current one while the gateway is disabled, uncommissioned,
unconfigured, faulted or in total feedback loss (below), since the bridge keeps
forwarding those while the robot cannot drive; and one that passes every other check but carries an earlier epoch
while no command is in force, since the bridge built it from a State read
before this epoch began (at an enable or a disable, for example). Any other
refused command counts, an earlier-epoch command that drives an output
included. The bridge applies the same superseded-neutral omission to
`/hw/status` `identity_rejected_count`: an earlier-nonzero-epoch all-neutral
request of the same configuration, boot, and device ownership with an
admissible observation age, arriving before that subsystem has had a request
accepted in the new epoch, is still dropped without merging or refreshing
silence, while any driving demand, ownership violation, or stale-epoch request
after a new-epoch acceptance still increments `identity_rejected_count`. Each command supplies every configured motor in manifest
order.
The bridge neutralizes omitted subsystem actuators. `/hw/status` exposes the
latest accepted health sample, motor fault mask, gateway counters and gateway
build (`gateway_program`, `gateway_build_id`, `gateway_build_dirty`), the
bridge's own `kProtocolVersion` (`protocol_version_id`, which the log tools
read a session under:
[which logs a build can read](../launcher/README.md#which-logs-a-build-can-read)),
and while a peer is sending frames of another protocol version (within
`command_timeout_us`, clearing immediately on any valid frame)
`alert_error_protocol_mismatch` and `mismatched_protocol_version_id` (the
observed peer version, 0 otherwise),
and the bridge prints the build once for each new gateway boot
(`hardware_node: gateway boot <id>: roborio build <revision>`). A State
that does not decode is dropped, with one stderr line until a State decodes
again.
`hardware_node --log PATH` records each `/hw/command`, `/hw/state` and
`/hw/status` it publishes (the merged command, every accepted State and
every status; a refused State is not published) as `SEND`s in a `.tlog`
(format in [talOS/events](../events/README.md)), the command and State as
raw `Packet`s, each with the transport sequence and
publication time its readers receive, and the pass that produced it as its
event time. A clean stop ends the log with an EXIT record, as an event
loop's does. It holds no `/hw/state/driver_station`: the bridge forwards
each Driver Station packet with a plain write, so a session has that topic
only in its readers' logs (receipts), and none when no node reads it;
`/hw/status` `driver_station_packet_count` counts the packets forwarded.

The bridge's exit lines (stdout, so the session's
`<output_dir>/<session_id>/hardware_node.log`, [node
output](../launcher/README.md#node-output)) carry wake and CPU-budget counters
`/hw/status` does not: `passes`, the wake reasons (`udp`, `request`,
`deadline`), `wait_fallbacks` (reported as `wake_fallbacks` in the registry
and manifest), and `throttled`, the passes that spent the loop's CPU
budget (8 passes or 64 received datagrams in a 1 ms window,
`HardwareNode::kCpuWindowUs`) and so slept through input until the window
ended, alongside the five UDP/socket loss counters (`send_failures`,
`socket_reopens`, `sequence_gaps`, `decode_failures`, `send_would_block`) that
`/hw/status` also publishes. A nonzero `throttled` means a gateway or a node
flooded the bridge; normal traffic stays far below the budget.

Nodes read the gateway's poll timestamp in their own clock through
`HardwareLink::sample_time()` (C `talos_hardware_sample_time`, Python
`talos.hardware_sample_time_ns(node)`). The bridge keeps the mapping
(`talOS/hardware/clock_offset.h`) and publishes it on `/hw/status`. Empty clock
probes run at most 50 Hz even while disabled/uncommissioned, with one probe
outstanding at a time and an unanswered probe replaced after 100 ms, so one
lost probe or echo never outlasts the exchange already held.
A later State echoes the probe's bridge-clock send time; its gateway sample
lies causally between that send and bridge receipt. The midpoint of the
minimum-RTT interval estimates
offset without assuming symmetric delay. `clock_offset_uncertainty_us` includes
half RTT, timestamp quantization and <=1000 ppm relative clock drift; exchanges
expire after one second. Eight ordered States and a live exchange are required.
`clock_offset_spread_us`, the one-way delay spread over the sample window, is
diagnostic, not an accuracy bound. No fusion
accuracy threshold is specified: measure uncertainty against fusion requirements
on the bench. New boots reset the mapping; reordered/retired States are dropped.

Driver Station disable/e-stop, command timeout, or total backend feedback loss
neutralizes outputs. Outside pending asynchronous configuration, neutral is
refreshed every tick until a fresh command is accepted.
Enable transitions and timeout/backend stops invalidate
commands by advancing the epoch. A control-send failure on individual motors
(`Backend::ApplyFailedDevices()`; Phoenix: a non-OK `SetControl`) is
device-local: the backend tries neutral on that motor, the gateway still accepts
the command, and quarantines the motor like a reset (below). Global apply
failures (`Apply`/`ApplyOutputs` returning false, or every motor failing in one
apply pass, including the dependent-neutralization pass) neutralize all outputs
and retry on the next fresh command; three consecutive global failures latch a
fault. A
disabled-to-enabled transition or successful disabled reconfiguration
(including an identical push) clears the latch. An identical enabled push
does not clear a fault. A transient status failure requires a fresh valid
command after recovery.

Outside pending asynchronous configuration, each `Tick` polls
`Backend::PollDeviceResets()` (Phoenix: `HasResetOccurred()`
per TalonFX/CANcoder/Pigeon2). Phoenix can deliver a reset TalonFX's restarted
position frames before that latch, so the gateway also counts as a reset a
rotor-feedback motor whose valid position stepped farther since its last
valid sample than `Gateway::kMaxRotorRps` (250 rotor rps, twice the fastest
TalonFX motor's free speed) allows over their interval, divided by
`sensor_to_mechanism_ratio` (and by `rotor_to_sensor_ratio` when below 1).
The interval runs from the earlier sample's tick minus its age to when this
tick's feedback was read (`Gateway::SetClock`, which the RoboRIO program
sets; without it, the tick's time), plus `kPositionJumpSlackUs` (20 ms), so a
held frame, a gap of invalid samples or a late read widens the bound. The
step is taken modulo `kPositionSpanRot` (32768, the span of Phoenix's
fixed-point position), so a wrap is never a jump. The reference restarts
whenever the gateway masks the motor (a restored motor's first sample is not
compared with its pre-reset one). It catches only a restart farther from zero
than that bound; the latch, when it follows, counts once more. Remote
CANcoder motors are not checked: their frame may be absolute, which the
gateway cannot see, and their position steps legitimately when the TalonFX's remote sensor frames resume. A backend
whose positions are not physical (the ideal `SimBackend`) opts out through
`Backend::PositionsArePhysical()`. A reset neutralizes only affected motors and
their followers or remote-feedback dependents before starting
`BeginRestoreAfterReset(ids)`. The global epoch and unrelated commands survive,
unless the reset leaves no valid motion sample, which is total loss (below);
observations up to the last State that masked the affected motors cannot
authorize them after recovery.
`PollRestore()` never blocks: Phoenix reapplies only the affected device chain
on a worker while ticks, state publication and unrelated outputs continue.
Failed devices retry after 250 ms, doubling to 2 s. The gateway masks the chain
(`RestoredWith`, `feedback.h`, and any listed CAN sensor) in every State read
until its restore ends, including the State of the tick in which `PollRestore`
reports the end, since that State was read before the poll, whatever the
backend reported for it. A restored chain therefore drives again only on a
command observing a State read after its restore completed: at least one more
neutral tick, without relying on the backend to report a restoring chain
invalid.
`Gateway::restore_attempt_count()` counts all restore starts per boot. Motors whose control send failed join the
same unavailable set: their followers and dependents go neutral at once, the
rest of the command keeps applying (unless the isolation leaves the State
Accept publishes no valid motion sample: total loss, below, judged there
too), and restore re-applies the chain's config
before a fresh command retries the send. A motor that keeps failing is
re-isolated on that retry, at the restore backoff rate, without a global fault;
repeat failures do not count as `apply_failure_count`, but the motor stays
invalid and its demands count in `neutralized_motor_count`.

Fault scope: stale motor current/voltage zeroes the effort fields but keeps the
sample valid; an invalid RIO encoder invalidates only that sample. A motor's or
CAN sensor's motion feedback is invalid when the backend reports a signal
error or an isolated device, when its position or velocity is not finite,
or when its position or
velocity sample is `command_timeout_us` old or older (an unknown age,
`UINT32_MAX`, included); the gateway judges finiteness and that age bound
after `Backend::Read`, the same for every backend, whatever the backend
judged itself (`hardware::ReplaceNonFinite`, `gateway.h`; `FeedbackFresh`,
`gate.h`), so a
gap shorter than that bound does not invalidate the feedback at the gateway;
a consumer's own, tighter age bound still applies (the 2026 robot's
drivetrain drops odometry samples older than 50 ms).
A motor or CAN sensor sample marked invalid, by the backend, the age bound
or the gateway's mask of an unavailable device, its follower, or a chain whose
restore ended in that tick (above), carries zero
in every measured field (position, velocity, applied voltage, stator and
supply current; `Gateway::snapshot`), never its last values; its ages,
temperature and fault bits are the backend's. Reset detection (above)
skips an invalid sample, so its zero is never a position step.
No State the gateway publishes carries a non-finite value, whatever the
backend reads (a decoder refuses a whole State for one, which the bridge
would take for a lost link): any other non-finite reading is sent as 0, its
sample invalid where it has a validity of its own; a motor's voltage,
currents and temperature have none, so they read 0 with the motor still
valid, as for stale effort. What each field becomes is stated once, at
`hardware::ReplaceNonFinite` (`gateway.h`). `hardware::Encode(const State&,
...)` (`messages.h`) applies `ReplaceNonFinite` to a copy before encoding for
the wire, so no State producer can put a non-finite value on the wire even if
it did not sanitize beforehand; the raw unsanitized encoder is internal to
`hardware::detail::EncodeUnsanitizedState` for codec tests only.
While the gateway is unconfigured (a configure in progress, every boot
restore included, or the last configure failed) it reads no device, so every
State it publishes then carries no measurement: every motor and CAN-sensor
sample is invalid with those zeros, and every RIO device sample and the
health sample is invalid, keeping its value as last read. The same holds
under a new configuration until its first `Backend::Read`, in the tick that
completes the configure. A configure in progress is not a fault: such a
State has `kConfigured` and `kEnabled` clear and, unless the configure failed,
`kHardwareFault` clear. Invalid motor
motion feedback, or a missing or invalid remote CANcoder, neutralizes only that
motor and its followers; healthy motors in the same command continue driving.
The neutral starts in the tick that finds the feedback invalid, while a command
is in force (between commands a backend keeps the last request it sent), and
holds in every command accepted until one observes a State later than the
last one in which the motor could not drive: neither the command in force nor
one built on any earlier State moves it once its feedback returns, even a State
that showed it valid between two invalid spells (`Gateway::MayDrive`;
the same floor as a reset, above). Neutralized demands in accepted commands
increment `neutralized_motor_count`; the tick's neutral does not. The backend
remains healthy while any configured motor or CAN sensor has valid feedback
(or none are configured). Total loss of all configured motor/CAN-sensor
feedback neutralizes every output and advances
the command epoch on the tick it is found, whether or not a command is in
force; the fault is not latched, and clears in the same epoch on the first
State with valid feedback. A command observing a State of the loss still
matches that epoch once it clears, but drives nothing: its motors are sent
neutral, its digital outputs false and its PWM outputs off
(`Gateway` `outputs_drive_from_`), even a motor whose sample the backend
left valid in a failed read pass (`Backend::Read` returning false is total
loss whatever it read). Total loss is judged on the State as published:
after the age bound and after the gateway's masks (unavailable, restoring or
send-failed devices, their followers and remote-feedback motors, a chain
whose restore ends that tick), so a State of a configured gateway with at
least one motor or CAN sensor, none of them valid, always carries
`kHardwareFault`. A reset of every motor and
CAN sensor at once (a CAN brownout) is therefore total loss from the tick
that finds it through the tick whose `PollRestore` ends the restore, whatever
the backend reads for a restoring device (`PhoenixBackend` and both simulated
backends read a restoring CAN sensor valid, a restoring motor invalid): for
that time every output, DIO and PWM included, is neutral and `robot_enabled`
is false, and the first State read after the restore clears the fault. So is
a master whose feedback is invalid with only its followers configured. A
reset that leaves any motor or CAN sensor valid (one motor beside a valid
Pigeon 2) is not total loss: each masked motor goes neutral and drives again
only on a command observing a later State, while digital and PWM outputs keep
applying and `robot_enabled` stays true. Failed
global Apply or backend configuration failure also neutralizes every output;
individual device configuration or control-send failures do not. A rejected configuration push need not invalidate the active
configuration.
DIO/analog/PWM and RIO encoder validity never sink the backend.

## RoboRIO

`controller_processor/rio/src/main/cpp/Robot.cpp` is the thin WPILib
adapter. Its companion (main processor) address is fixed when it is built:
`./gradlew deploy -PteamNumber=<team> -Pcompanion=<ipv4>` (or `buildRio -Pcompanion=<ipv4>`,
or from a robot repository `bazel run @talos//controller_processor/rio:gradlew -- ...`).
There is no built-in team number or companion default: `buildRio` and `deploy`
fail without `-Pcompanion=<ipv4>`, and `deploy` also requires a team number
(`-PteamNumber=<team>` or `.wpilib/wpilib_preferences.json`). `build.gradle`
takes `-Pcompanion` only from the command line: a
`companion` set in a `gradle.properties` or `ORG_GRADLE_PROJECT_companion`
fails the build, so a bench address cannot carry into a later deploy from the
same laptop. It also fails the build on an empty value (a bare `-Pcompanion`
or an unset shell variable) and on anything but a dotted IPv4 literal without
leading zeros outside `0.0.0.0/8`, loopback `127.0.0.0/8` and `224.0.0.0/3`
(multicast, reserved, broadcast), addresses the RoboRIO's socket could not
open or send to. Every RoboRIO build prints the address it builds in
(`RoboRIO companion address built in: ...`). It is
never learned from an incoming datagram: the gateway's socket is connected to
that one peer, which is what keeps any other host on the network from
commanding the robot ([protocol](../protocol/README.md)). Runtime ports are
5802 on the gateway and 5803 on the companion.
The other direction -- the companion finding the gateway -- is
`[robot] controller_address` in the robot config (see [launcher addressing](../launcher/README.md#controller-addressing)),
overridden per launch by `launcher --remote`. `[robot] local_address` (or
`hardware_node --local-address IP`) optionally pins the bridge's UDP bind
address to a specific IPv4 interface instead of `0.0.0.0`; when pinned and
absent at startup (`EADDRNOTAVAIL` / `ENETUNREACH`), `hardware_node` logs one
stderr line naming the address and errno and retries every 100 ms until the
interface comes up rather than falling back to `0.0.0.0` or exiting. Real hardware uses Driver Station
enable and e-stop.
The socket is opened from the adapter's main loop. After a RoboRIO boot the
program can start before the network has a route to the companion, and the
open's `connect()` fails (`ENETUNREACH`, errno 101); the loop retries every
100 ms (`src/main/include/SocketRetry.h`; an attempt is a few non-blocking
calls), ticking the gateway disabled (all outputs neutral, no State sent)
until the socket opens. The console reports the failures
(`hardware gateway socket failed: <status> (errno <n>)`, coalesced to one line
a second) and, once it opens, `hardware gateway socket open after <n> failed
attempts, <s> s after the first`.
A send or receive that fails with an error the socket may never recover from
(`protocol::SendErrorNeedsNewSocket` / `protocol::ReceiveErrorNeedsNewSocket`,
whose errors are in [talOS/protocol](../protocol/README.md): among them
`ENETUNREACH`, Linux's error once the RoboRIO's address changes under the
connected socket, and `EBADF`; the bridge's rule in
[data flow](../../docs/DATA_FLOW.md#link-rates-and-endpoints)) loses the
socket: on a permanent receive error the adapter closes it immediately
(`Endpoint::Close()`), nothing more is sent that tick, and the loop opens a new
socket on the same ports (`Endpoint::Reopen`, keeping the frame sequence) on the
same 100 ms cadence, at once when no attempt was made in the last 100 ms, so a
condition that breaks every new socket costs at most ten sockets a second.
Usually the next tick replaces it and runs as any other: the replacement is
a link gap of one tick, and the command in force keeps its usual expiry
(the command timeout bounds it, as for any gap). Closing the old socket
drops what was queued on it, and a later command passes the gateway's usual
sequence, boot, epoch and age checks. A tick with no socket open (the
attempt failed, or is not due yet) ticks the gateway disabled, all outputs
neutral, and sends nothing, as before the first open. The console prints
`hardware gateway socket cannot send (errno <n>); replacing it, nothing is
sent until then` (or `hardware gateway socket cannot receive (errno <n>);
replacing it, nothing is sent until then`) and, once replaced, `hardware
gateway socket replaced after <n> failed attempts, <s> s after it was lost`
(each coalesced to one line a second). Any other send or receive error
(`ENOBUFS`, `ECONNREFUSED`, ...) keeps the socket. Every failed send is counted in a spell, which ends at a good
send one second after its last failure (with the companion's port closed
every other send fails with `ECONNREFUSED`); a spell prints `hardware
gateway socket send failed (errno <n>); keeping the socket` at its first
failure that keeps the socket and, once such a spell ends, `hardware gateway
socket sending again: <n> sends failed over <s> s (errno <n>[, last errno <m>])`,
`<s>` from its first failure to its last (at most one line a second). A
non-blocking send refused with `EAGAIN` / `EWOULDBLOCK` (`UdpStatus::kWouldBlock`,
send buffer full) keeps the socket open without triggering a reopen and prints
`hardware gateway socket send would block (EAGAIN, send buffer full): <n>
dropped since start; keeping the socket` (rate-limited by `RioLog` to at most
one line a second, bursts coalesced with the cumulative `send_would_block`
count). A send raises no `SIGPIPE` on either end
([protocol](../protocol/README.md)).

A tick that invalidates the command in force because it reached its expiry
(`command_timeout_us` after the older of its acceptance and the State it was
built from) prints, once per expiry, `hardware gateway command expired:
outputs neutral at [<s>.<ms>], <x> ms after its observation, <y> ms after
its acceptance (expiry <n>)`: that tick's time on the gateway's clock, its
delay from the State the command was built from and from its acceptance, and
the expiries since the program started (`Gateway::last_command_expiry()`,
posted by `src/main/include/LoopConsole.h`). When a lost link, bridge or
launcher caused the expiry, no State reaches the main processor and this
line is the only record of the instant. The first expiry prints at once;
further expiries within that second print as one line carrying the last
one's time and the count.

That loop enforces the command timeout and output neutral, so it never
writes a report itself. `PhoenixBackend` and the adapter
post fixed-size events (fault-field, feedback-validity, late-feedback, apply
and neutral send transitions, device resets, brownout, a CAN bus's read
failing and recovering, an abandoned CAN bus reader) to `RioLog`
(`src/main/include/RioLog.h`): a preallocated 256-event queue that never
waits, dropping and counting events when full. A logger thread at nice 19
(Linux) writes them to stderr, at most one line per second for each kind and
device: the first change at once, then, while changes continue, one line a
second with their count and, for the fault field, the bits that toggled. A
fault-field line lists the changed bits as `fault raised`/`fault cleared` or,
for the bits that are not faults (`kMotorNotFaultFlags`), `limit
raised`/`limit cleared`, then the active `faults=[...]` and `limits=[...]`;
the `motor_fault_mask changed` line counts faults only. Once
in any second that saw them, it also prints the dropped-event count and the
loop's timing faults (`[rio] loop timing:`): overruns (a tick's work, from wake
to sleep, longer than the period) and late ticks (a tick starting more than 1.5
periods after the previous, a starved loop however short its work), with that
second's longest work, longest interval, period, and that single worst-work
tick's 8-phase split (`; worst tick phases: refresh <r> us, devices <d> us,
control <c> us, health <h> us, other <o> us, link <l> us, driver station <ds>
us, loop <lp> us`). Every 10 s — and on clean exit (`RioLog::Flush` / `Stop`)
for the final partial window — when the loop ticked in that window, it prints
where those ticks' work went:

```
[rio] tick phases: refresh mean/max <a>/<b> us, devices <a>/<b> us, control <a>/<b> us, health <a>/<b> us, other <a>/<b> us, link <a>/<b> us, driver station <a>/<b> us, loop <a>/<b> us over <n> ticks (motors=<m> sensors=<s>)
```

per tick, the mean and longest time in `BaseStatusSignal::RefreshAll`, one
call per CAN bus (refresh), the per-device validity, latency and `HasResetOccurred` pass
(devices), TalonFX `SetControl` for every motor, whether a command's or
neutral (control), the copy of the CAN bus figures `talos-canbus`
published (never a CAN status read) and the battery reads (health), and the rest
of the backend's tick calls (other); and the loop's own: the socket retry,
the endpoint's tick (receive, the gateway's command and State work, the
State send; with no socket open, the gateway's tick) and the Driver Station
packet's send, less the backend phases inside them (link), taking the
Driver Station reader's latest snapshot and copying and encoding it as the
Driver Station packet (driver station; the refresh, the snapshot and the
mode report are the reader's, outside every phase, below), and
the console's checks (`LoopConsole`), watchdog beat, companion report and
config persister (loop). A tick's
phases sum to its work, the time the loop timing line's longest work
measures. `PhoenixBackend` takes one
steady-clock reading per phase boundary, ten per disabled tick, and adds
them on the loop thread without locks; the window passes to the logger at
the loop's next tick ([bench measurement](../../docs/BENCH.md#fault-injection-and-timing)).
`State` has no field for these, so they are RIO-local.

Beside it, from the same window (REQ-RT-25):

```
[rio] latency: command age accepted p50/p99/max <a>/<b>/<c> us over <n>, refused <a>/<b>/<c> us over <n>; driver station refresh to send <a>/<b>/<c> us over <n>
```

Every figure is a difference of two readings of the RoboRIO's own clock,
never a subtraction across processors. The command age is, for each tick
that drained a command (`Endpoint::drained()`, the newest of the batch), the
clock read just after the endpoint's tick less the command's
`observed_time_us`, the `sample_time_us` of the State this gateway sent and
the command was built from; accepted and refused (`Gateway::Accept`'s
answer) are counted apart. An accepted age is that State's sample-to-apply
time to within the endpoint's encode and send of its next State, which
follows `SetControl` in the same tick; it is not the age of the CAN feedback
in that State, nor the time the motor acts. A refused age while disabled
includes the bridge's hold: while the gateway cannot drive, the bridge sends
a merge whose only change is its observation on its heartbeat
(`command_timeout_us / 5`, 5 to 20 ms), not at once. The Driver
Station figure is, for each packet sent, the clock read after its send less
the refresh time of the snapshot it carries (`DriverStationReader::refreshed_us`);
the neutral snapshot sent while no refresh is current is not counted. It is
not the Driver Station's own packet age (radio and NetComm come before the
refresh). p50 and p99 are nearest-rank to the 100 µs bucket, never above the
exact max; ages from 51.2 ms on land in one overflow bucket, where the
percentiles read as the max; a figure with no samples reads `-/-/- us over
0`. The clock runs on across a restart of the program, so a command built
from a previous run's State (its boot id refused) is counted as refused with
its age on this clock: after a redeploy with the launcher running, the first
window's refused max is the restart gap. Only an observation ahead of this
clock (one from before an OS reboot, a corrupt stamp) wraps and is dropped. The loop
adds at most two counts a tick, reusing the phase boundaries' clock reads
(the work falls in the phase after each); the logger takes the window by switching
the loop to a second set of histograms (about 6 KiB each), which it clears
after printing.
Each of these lines, and the program's start lines, begins with
`[<seconds>.<milliseconds>]` on the gateway's steady clock: on the RoboRIO,
time since its kernel booted, the clock of State's `sample_time_us`, so a
`hardware_node --log` record of a State places the RIO's lines on the
session's time. A RioLog line's time is when the logger wrote it, within one
20 ms drain of the event unless that thread is starved. The program's Driver
Station messages (`talos companion link`, `talos loop watchdog`) begin their
text with the same prefix (one function, `ConsoleTime` in `RioLog.h`), the
time of the check that found the change or the stall, and so does their
console copy (`Error at talos loop watchdog: [<s>.<ms>] ...`); so do the
desktop simulation's `simulated Driver Station: ...` lines. Two lines carry
none: the desktop simulation's exit summary (`sim_robot: ...`, a record that
`tools/test_rio_in_the_loop.py` finds by its start), and the WPILib and Phoenix
libraries' own lines, which this program does not write.

`PhoenixBackend` marks a motor or CAN sensor invalid only for a signal error,
a non-finite value or an isolated device (`feedback INVALID`, then
`RECOVERED`); the age of its feedback is the gateway's to judge
([fault scope](#runtime-contract)). A CAN silence shorter than that bound
therefore changes no validity and shows as `feedback LATE
(position_age_us=..., velocity_age_us=..., late from <t> us, ...)` once the
device's older motion sample is four status periods old (`4 / status_hz`,
20 ms at 200 Hz), then `feedback ON TIME (peak_age_us=..., ...)`, with the
spell's oldest age, once both are younger again. `Apply`'s own follower
check holds a follower neutral while its master's, its own or its remote
CANcoder's feedback is `command_timeout_us` old.

If the loop stops ticking right after it applied a command, nothing else
stops the outputs: its TalonFX controls are one-shot (their device-side
expiry is unmeasured), and DIO and PWM outputs hold their last value. So
the loop beats a loop watchdog (`src/main/include/LoopWatchdog.h`) before
`Endpoint::Tick()` on an enabled tick with no active command
(`command_expiry_us() == 0`) — pre-arming it at
`now + command_timeout_us + 2 * period_us` (`WatchdogPreArmDeadlineUs` in
`src/main/include/RobotSafety.h`) so a stall inside that tick's first
`Apply()` or `ApplyOutputs()` is still neutralized — and again at the end
of every tick. That post-tick beat refines the deadline, while a command is
still in force, to the time by which the loop's next tick would have
invalidated it: the gateway's `command_expiry_us()` (the older of the
command's acceptance and the State it was built from, plus
`command_timeout_us`) plus two loop periods (one for the tick that sees the
expiry, one for that tick's work before its beat); with no command in force
it disarms back to `0` (that tick neutralized every output itself). If the
loop stalls after `Endpoint::Tick()` returns with no command (for example
in Driver Station send or console logging) before that post-tick beat runs,
the pre-arm deadline stays armed and the watchdog will fire about
`command_timeout_us + 2 * period_us` later on already-neutral outputs and
report a `talos loop watchdog` stall to the Driver Station. The watchdog's
own thread, at `SCHED_FIFO` 30 on the RoboRIO (above the loop's
`SCHED_FIFO` 10 and the kernel's interrupt threads), checks every 5 ms. Once that
time has passed it disables PWM outputs, sets DIO outputs false and sends
`NeutralOut` to every motor, on every check until a beat moves the time (so
outputs a resumed loop re-applies on a stale clock reading before stalling
again are neutralized again), and sends one Driver Station error per stall,
`talos loop watchdog`, with the silence and a stall count, from a reporter
thread of its own at the default scheduling. A stall while four reports are
still undelivered (that Driver Station call blocked) loses its report, and
the loop's console then prints, once per rise in that count
(`LoopWatchdog::dropped_reports()`, posted by `LoopConsole`), `RoboRIO loop
watchdog: <k> stall report(s) dropped (<total> since start), <n> stalls in
all: the Driver Station error channel was not taking them`. It touches only
the device handles (Phoenix 6 serializes each device's control sends behind
a per-device mutex; HAL DIO and PWM writes are safe from any thread), skips
devices a configure is rebuilding, and does not act while the loop ticks on
time. It is not a hard bound: if a device's lock (Phoenix's control mutex,
the HAL DIO mutex; neither inherits priority) is held by a thread that is
not running, the watchdog waits there, and so does every output after it.
Held by the loop, the wait lasts until no thread above the loop's priority is
runnable on its core; held by a default-scheduled thread (a vendor
library's), the holder also waits for the CPU behind the loop and every
other realtime thread. A refused realtime priority is reported the same
way, and the loop then stays at default scheduling too (below).

Thread policies on the bench RoboRIO (2026-11-11, image
FRC_roboRIO_2026_v1.2, two cores): every kernel interrupt thread runs at
`SCHED_FIFO` 15 (`irq/54-eth0`, `irq/64-e000b000`, ...) except the FPGA's
`irq/61-atomiczy` at 96; there is no kernel CAN interrupt (CAN goes through
the FPGA interrupt and user-space threads); the robot program's `can_recv`,
`can_psend`, `can_stat`, `frc_evnt_client`, `nirio::DeviceNo`,
`atomiczynqu` (two), `niriodevenum` and `frc_server_mon` are `SCHED_OTHER`,
three unnamed threads `SCHED_RR` 1, 1 and 2; every `FRC_NetCommDaemon`
thread is `SCHED_OTHER`; and realtime throttling is off
(`sched_rt_runtime_us` is -1).

The loop thread runs at `SCHED_FIFO` 10 on the RoboRIO (`kLoopPriority` in
`Robot.cpp`), so ordinary processes such as an ssh session no longer delay
its ticks, and sleeps to each tick's absolute deadline (`clock_nanosleep` on
`CLOCK_MONOTONIC`, the clock of its steady time). A `static_assert` holds it
below the interrupt threads (`kIrqThreadPriority`, 15), which a loop that
stopped sleeping would otherwise shut out of its CPU for good, and the
interrupt threads below the watchdog. That order is also checked at run
time: the loop raises itself only after `LoopWatchdog::Start` returns with
the priority the kernel gave the watchdog thread, and only if it is a
`SCHED_FIFO` priority above the loop's. Threads the loop starts afterwards
start at default scheduling (`SCHED_RESET_ON_FORK`). It keeps default
scheduling if `/home/lvuser/talos_loop_default_sched` exists when the
program starts (for a bench comparison, or as a fallback in the field
without a rebuild), if the watchdog holds no such priority, or if the kernel
refuses the request. Its start line, `hardware gateway loop <policy>
<priority>, watchdog <policy> <priority>`, gives what the kernel holds for
both and, at default scheduling, why: `(<file> present)`, `(watchdog holds
no SCHED_FIFO above 10)` or `(SCHED_FIFO 10 refused, errno <n>)`. Desktop
simulation keeps default scheduling.

The loop makes none of the Driver Station's NetComm calls. WPILib's refresh
(`frc::DriverStation::RefreshData`, `HAL_RefreshDSData`) asks NetComm for
the control word every time, and with no Driver Station attached that is a
synchronous round trip to `FRC_NetCommDaemon` (`SCHED_OTHER`), serialized
with every other thread's NetComm call; the mode report
(`HAL_ObserveUserProgram*`) is a NetComm call too. A reader thread,
`talos-ds` (`src/main/include/DriverStationReader.h`), makes them instead,
at `SCHED_FIFO` 5 on the RoboRIO (`kDriverStationPriority` in `Robot.cpp`,
below the loop and above every default-scheduled thread, with
`SCHED_RESET_ON_FORK`; default scheduling under desktop simulation). It
waits for WPILib's new-data event, which NetComm's own thread sets on each
Driver Station packet, or 20 ms without one; refreshes WPILib's cache;
snapshots all of it with HAL getters (control word, alliance and station,
match information and time, every joystick's axes, buttons and POVs in one
`HAL_GetAllJoystickData` copy, and names; no WPILib getter, whose range
warnings are NetComm calls); and publishes the snapshot with the refresh's
completion time through a triple buffer: it fills a slot the loop cannot
see and hands it over with one atomic exchange. After each refresh, while
the loop's last tick is at most 20 ms old, it reports the program's mode, so
a stalled loop still stops the reports. Each tick the loop takes the newest
snapshot (an atomic load, and an exchange when there is a new one; no lock,
no WPILib call) and uses that one snapshot for the gateway's enable, the
mode generation and, when one is due, the whole Driver Station packet. So
the packet is always one refresh's data: the HAL's cache changes only on a
refresh, only `talos-ds` refreshes (on the RoboRIO and in desktop
simulation, whose scripted Driver Station only queues its data and wakes
`talos-ds`), and it snapshots before its next one;
the joysticks are one Driver Station packet's. Once the last completed
refresh is `command_timeout_us` old (the gateway's, the default Config's
100 ms before a configuration), the loop reads no Driver Station: flags 0,
no mode, disabled, every joystick disconnected with no axis, button or POV.
A daemon that does not answer, or a starved
reader, therefore disables the gateway within `command_timeout_us` plus one
tick of the last refresh that completed, and the mode the loop reads when
refreshes resume starts a new mode generation. The reader's start line,
`driver station reader SCHED_FIFO 5` (or `... (SCHED_FIFO 5 refused, errno
<n>)`), gives what the kernel holds. A program stop waits at most 100 ms for
the reader and then abandons it in its call, with the line `driver station
reader did not stop within 100 ms (...); it was abandoned`. The loop makes
no other NetComm library call: the RoboRIO's CAN status read
(`HAL_CAN_GetCANStatus`) runs on the `talos-canbus` reader, and the loop
copies its result.

A tick that ends by its deadline sleeps to it, so the loop keeps its phase.
One that ends past it (an overrun, or a wake more than a period late) drops
the missed ticks instead of running them back to back, sleeps 500 µs and
counts the next period from there, so the phase moves
(`src/main/include/LoopSchedule.h`). Whenever it runs, the loop takes the
CPU from every thread on its core below `SCHED_FIFO` 10: the Driver Station
reader (`SCHED_FIFO` 5), the logger, the
companion reporter and the config saver (nice 19), ssh, the program's CAN threads (`can_psend`,
`can_recv`), netcomm, and the three `SCHED_RR` threads. The 500 µs bounds
the share a loop that overruns every tick takes; a loop whose work is just
under the period still takes nearly all of its core, and with throttling
off nothing bounds a loop that never reaches its sleep. The watchdog still
preempts such a loop (it wakes on its own timer and reads the beat from
atomics) and still calls `NeutralOut`, but that the frame then reaches the
bus is not established: it may depend on `can_psend` or another
`SCHED_OTHER` thread the spinning loop holds off on that core, unless the
scheduler moves it to the other one. The interrupt threads, above the loop,
keep running. The effect on late ticks is unmeasured
([bench check](../../docs/BENCH.md#fault-injection-and-timing)).

The Driver Station shows green code and communications whether or not the
companion is talking, so the adapter also reports the companion link
(`src/main/include/CompanionLink.h`). Each tick the loop copies the
endpoint's `PeerContact` (when it last received a valid frame, a command
frame, and a frame of another protocol version, with that version) into
atomics; a reporter thread at nice 19 reads them every 100 ms and publishes
SmartDashboard values under `companion/`: `connected` (a valid frame within
500 ms; the bridge probes the clock every 20 ms even with nothing to
command), `protocol_ok` (no frame of another version within 500 ms),
`protocol` and `peer_protocol`, `frame_age_s` and `command_age_s` (-1 before
any; once a node has hardware state its devices flush a request every tick,
neutral while disabled, and the bridge forwards the merge at least every
20 ms, so on a healthy link both stay near 0.02 s), `configured`,
`commissioned` and `config_id` (hex). It also sends one Driver Station
message per change: an error when the link is lost, when nothing is heard
30 s after robot code start, or when a frame of another protocol version
arrives; a warning when the link comes up, when the mismatch clears, and when
the link has been up 30 s (and robot code running 30 s) but no command frame
arrived in the last 500 ms (actuator nodes down or crash-looping), once per
such stretch. The
drive team's use of these is the [pre-match checklist](../../docs/PRE_MATCH.md).

Every State names the program's build. `build.gradle`'s `generateRioBuild`
task, which every compile depends on, writes `build/generated/talos/include/RioBuild.h`
with the checkout's git commit and whether it has uncommitted changes,
untracked files included; without git or outside a checkout the commit is
`unknown` and the State's revision 0. It reads only the local repository, so
`--offline` builds get it, and rewrites the header only when it changes. A
build from a copy with no git history passes
`-PtalosRevision=<16 lowercase hex digits> -PtalosDirty=true|false` on the
command line instead, which replaces git: the commit is those digits and the
dirty flag is `-PtalosDirty` (true when the copy differs from that commit).
The two are required together, and the pair is refused while git's work-tree
root matches the talOS root being built from, where git stamps the build itself
and an override could only hide uncommitted changes (a staged copy inside an
unrelated git repository is not stamped with that repository's commit and
accepts the override). Like `-Pcompanion` both are
refused from a `gradle.properties` or the environment, and a revision that is
not exactly 16 lowercase hex digits, or is all zeros (revision 0 means
unknown), or a `-PtalosDirty` that is not exactly `true` or `false`, fails
the build (`rioBuildStampTest`, which `phoenixTest` runs, checks the stamp). The
header also carries the companion address. The program prints both on its
console at start (`hardware gateway build <commit> companion <address>:5803`,
the address `127.0.0.1` under simulation), and the State reports the
commit's first 16 hex digits
([DATA_FLOW.md](../../docs/DATA_FLOW.md#udp-frame-and-payload-sizes)), as
`ROBORIO`, or `ROBORIO_SIMULATION` under WPILib simulation, on `/hw/status`.

Build without deploying:

```sh
cd controller_processor/rio
./gradlew buildRio -Pcompanion=<ipv4>
./gradlew phoenixTest   # desktop tests: PhoenixBackend sim, ConfigStore, SimDriverStation, RioLog, CompanionLink, LoopWatchdog, LoopConsole, SocketRetry, LoopSchedule, CanBusHealth, DriverStationReader, DriverStationSampler, RobotSafety; and rioBuildStampTest
```

WPILib desktop simulation (`./gradlew simulateNative`) runs the same gateway
against `PhoenixBackend` with Phoenix sim state. The RoboRIO program derives
from `frc::RobotBase`, not `frc::TimedRobot`, and runs its own fixed-rate
loop. Its Driver Station reader refreshes the Driver Station data the loop
forwards and reports the mode the program is running in, which the Driver
Station displays; whether the Driver Station
disables the robot when those reports stop is unverified (the loop stall
check in [BENCH.md](../../docs/BENCH.md#fault-injection-and-timing)), and
the loop watchdog does not rely on it.

### The RoboRIO program in the loop

`tools/test_rio_in_the_loop.py` runs a gateway, the bridge, the bench's
swerve module node and its `send_target` as separate processes on loopback,
on the bench's loopback config
([`robot.toml`](../../bench/main_processor/configuration/robot.toml)):

```sh
bazel build //bench/main_processor/swerve_module:all \
  //talOS/simulation:sim_gateway //talOS/bridge:hardware_node
python3 tools/test_rio_in_the_loop.py            # sim_gateway at the RoboRIO end
python3 tools/test_rio_in_the_loop.py --wpilib   # this program, under WPILib simulation
```

Each mode drives the module for two seconds, checks from the gateway's exit
summary that its drive wheel turned and that it ended neutral once the
target, node and bridge had stopped, and replays the node's log. The
summary reports the first motor by logical id, so these runs use
`robot.toml` narrowed to the module's own subsystem (the script writes it
from that file's `[robot]` tables and `swerve_module` entry; devices, CAN
ids and buses unchanged). `--wpilib` swaps `sim_gateway` for
`./gradlew simulateNative` in `controller_processor/rio/` and makes the same
assertions, because both run the same `talos::hardware::Gateway`, then runs
two more scenarios: with the Driver Station never enabled, the module must
not move at all; and with it enabled and the wheel driven, a relay standing
in for the main processor's socket cuts the link at once (the
main-processor processes then stop cleanly; nothing they send past the cut
reaches the RoboRIO), and the RoboRIO program alone must neutralize every
output on its first tick at or past the last command's expiry, which the
harness reads from the RoboRIO's own States at the relay. That scenario
runs `robot.toml` whole, so the bench's undriven devices must read neutral
too. Each `--wpilib` scenario first deletes the simulation's saved
configuration (`controller_processor/rio/talos_config.bin`) and checks that the
program starts unconfigured (`config=0`), failing if it still restores one:
`SimDriverStation` waits for a push acknowledged in this boot rather than
enabling on a restored configuration, and removing the saved file ensures each
scenario starts cleanly from an unconfigured gateway. It
runs Gradle `--offline`, so the first run needs one online
`./gradlew simulateNative` beforehand. CI's `rio-in-the-loop` job runs
`--wpilib` on Linux with `--output-dir`, which puts each scenario's log
directory under a directory it keeps when the run fails. Ports 5802/5803
must be free. Both modes fail a run during which the host slept (more than
50 ms of sleep), since a frozen host reads as a stalled robot; on macOS the
harness holds `caffeinate -s` for the run, which also holds off a headless
Mac's DarkWake maintenance sleep. `--wpilib` also fails when the RoboRIO
loop's watchdog reports a stall, when its timing report shows more than 250 ms
between ticks (naming whether the tick overran or only started late), or when a
scenario prints no `[rio] tick phases` (with positive tick count) or `[rio]
latency` line (and in the driving scenario, when accepted command or Driver
Station latency sample counts are zero).

## Simulation Backends: Ideal vs Physics-Backed

Both implement `Backend`, both share the gateway safe-state policy
(disable/timeout/failure neutralizes every output type), and both step on the
explicit `sample_time_us` clock, so Gateway ticks with explicit timestamps are
exactly reproducible.
Both publish valid, no-brownout, zero-CAN-load health resting at a charged
battery's 12.8 V (`kSimRestingSupplyV`, `sim_backend.h`) by default, so a
healthy simulated robot passes the pre-match battery check;
`SetPowerHealth(volts, brownout)` injects low-power telemetry. Physics also uses
the reported voltage as its motor supply: a duty-cycle demand applies that
fraction of it (0.5 is 6.4 V at 12.8 V rest), while voltage, closed-loop and
torque-current modes are clamped to `max_voltage` (12 V when unset) and the
motor models keep their 12 V free-speed and stall ratings. With
`SetBatteryModel` (`[simulation.battery]`), `PhysicsSimBackend` models
internal-resistance voltage sag (`V_batt = V_oc − R_int × max(0, base_current_a
+ Σ supply_current_a_i(V_batt))`, solved via 28-step bisection on `[0, V_oc]`
so it stays monotone and oscillation-free even under heavy stall load where
`R_eq < R_int`) and automatic brownout below `brownout_voltage_v`;
`SetSupplyVoltage` / `SetPowerHealth` override the bus until
`ClearSupplyVoltageOverride()` resumes the battery model. Health is observable
while disabled or without a live command. Both keep each PWM output's last
request (`GetPwm`, off when there is none) and read an output that sends no
pulse as 0, as the RoboRIO's `PWM::GetSpeed()` does after `SetDisabled`.

| | Ideal (`SimBackend`) | Physics-backed (`sim::PhysicsSimBackend`) |
|---|---|---|
| Use for | Fast logic tests: wiring, sequencing, timeouts | PID/gearing/direction/limit believability |
| Velocity demand | Applies instantly, even with zero gains | P+FF loop produces volts; zero gains stall |
| Position demand | Teleports (no position-jump reset check) | P-loop ramps through voltage; MotionMagic uses trapezoid profiles |
| Neutral | Velocity snaps to 0 | Unpowered decay: brake stops fast (shorted-winding electromagnetic braking under a `MechanismSpec`), coast rolls on (no-load + viscous/static friction decay under a `MechanismSpec`) |
| Gearing/inversion | Ignored | Rotor/gearing division, inversion flips direction; a follower drives in its master's direction (reversed if opposed), ignoring its own `inverted`; a remote-CANcoder motor reports and closes its loops on the CANcoder in the CANcoder's direction |
| Mechanism inertia | Ignored | Opt-in `SetMechanisms` (`sim::MechanismSpec`, bound from `[[simulation.mechanisms]]` by master motor name; survives `Configure`): 10-substep exact exponential rigid-body ODE reflecting `inertia_kg_m2 / gear_ratio²`, viscous damping, and Coulomb static friction onto the root master shaft, summing active follower torques and re-evaluating current limits each sub-step; motors without a spec keep the legacy first-order lag |
| Chassis dynamics | Ignored | Opt-in `SetChassisDynamics` (`sim::ChassisDynamics`, bound from `[simulation.chassis]`; survives `Configure`): coupled 2D rigid-body swerve dynamics with chassis mass, rotational inertia (defaulting to `mass_kg × mean(x_i² + y_i²)`), per-wheel friction-circle clamping (`mu × m × g / N`), lateral scrub resistance, excess-torque wheel spin-up (physical odometry slip), and exact SE(2) pose stepping; unset keeps kinematic least-squares chassis integration |
| Soft limits | Ignored | Clamped, axis stopped at the limit |
| Hard stops | Ignored | `SetHardStops` (`sim::HardStop`): physical end-of-travel bounds `[min_rot, max_rot]` per named motor (mechanism rotations in its hardware frame), enforced even when `soft_limits = false` (rezero-frame mechanisms); clamps position and zeroes velocity when driven into a stop so back-EMF drops to 0 and stator current rises to the stall/limited current while allowing motion away from the stop; bound by name, survives a pushed config. `sim_gateway --physics` binds `[[simulation.hard_stops]]` |
| CANcoder position | Its linked motor's sensor-shaft reading (`mechanism × sensor_to_mechanism_ratio`, with motor initial offset `offset_rot / sensor_to_mechanism_ratio`), position wrapped to [-0.5, 0.5) rotations as the RoboRIO reports it | The linked sensor shaft (`rotor / rotor_to_sensor_ratio = mechanism × sensor_to_mechanism_ratio`), position wrapped the same way; the remote-CANcoder motor divides `offset_rot` by `sensor_to_mechanism_ratio` to match Phoenix 6 |
| Pigeon yaw | Zero raw yaw, then mounting offset/inversion | True chassis yaw relative to boot, plus optional drift; explicit gyro overrides remain available |
| Supply voltage | Ignored | Scales free speed; `SetBatteryModel` (`[simulation.battery]`, survives `Configure`) models internal-resistance sag and automatic brownout via 28-step bisection, overridden by `SetSupplyVoltage` / `SetPowerHealth` until `ClearSupplyVoltageOverride()` resumes the battery model |
| Torque current | Not modelled: velocity and voltage read 0, as for duty cycle | The volts that pass the demand through the winding at the rotor's speed, I R + back-EMF in the motor's direction (`TorqueCurrentVolts`, using the same `R` and `k_e` as `StatorCurrent`), the current clamped to `max_torque_current_a` and `stator_limit_a`, the volts to `max_voltage` and the supply; powered at 0 A, modelled as zero stator current (slowly coasting only under the motor's no-load drag rather than braking; hardware unmeasured: [Supported Hardware Requests](#supported-hardware-requests)). FOC is not modelled |
| Stator and supply current | Stator zero; supply placeholder `|voltage| / 12 × supply_limit_a` | Winding model with free current and stall torque: `R = 12 V / stall_current`, `k_e = (12 V − free_current × R) / free_speed`, `k_t = stall_torque / stall_current`, `back_emf = k_e × rotor_rps` (`MotorModelFor`: Kraken X60 6000 rpm, 366 A stall, 2.0 A free, 7.09 N·m stall, default; Kraken X44 7530 rpm, 275 A stall, 2.0 A free, 4.05 N·m stall; Falcon 500 6380 rpm, 257 A stall, 1.5 A free, 4.69 N·m stall). Stator current is `|applied_volts − back_emf| / R` (equals `free_current` at 12 V free speed and `stall_current` at 12 V stall), 0 while unpowered. `stator_limit_a` and `supply_limit_a` clamp effective winding voltage before rotor integration and are reflected in `applied_volts_` and reported currents. Supply current follows signed power conservation `(rotor_volts × signed_stator_current) / supply_v` (positive battery draw clamped to `supply_limit_a`, negative regenerative braking kept signed, 0 when unpowered or `supply_v <= 0`). A motor held at a hard stop or soft limit is stalled. The type is the motor's `motor_type` key (above) |
| Home switch | None: a digital input reads what `SetDigitalInput` set, false if unset | `SetHomeSwitches` (`sim::HomeSwitch`): a named digital input reads closed exactly while a named motor's reported position is at or beyond a point (below or above it), with no hysteresis, and reads `closed_reads` while closed (default false: normally open); bound by name, it survives a pushed config. `sim_gateway` binds each input's `sim_motor` keys (above); an input without them reads false at every position |
| Faults | Device resets, digital/analog/encoder setters | Plus frozen feedback (valid) and per-device disconnect (invalid), each read as `PhoenixBackend` reads a device whose frames stopped: ages growing from its last sample, limit bits cleared once that sample is 50 ms old and temperature 0 once it is 500 ms old (`kFaultFieldMaxAgeUs`, `kTemperatureMaxAgeUs`, `feedback.h`); controller restart |
| Device restore | The chain `BeginRestoreAfterReset` reapplies (its motors, their followers, motors on a listed remote CANcoder: `RestoredWith`, `feedback.h`) reads invalid until `PollRestore` completes it, as `PhoenixBackend::Read` reports it (the gateway masks it in those States either way); completes at the next poll, unless `SetRestoreFails` fails restores: the chain then stays invalid until one succeeds | The same |

Tests live in `sim_physics_test.cc`. The standalone `sim_gateway` runs the
ideal backend by default and the physics backend with `--physics` (see
[`talOS/simulation/README.md`](../simulation/README.md)). The RoboRIO desktop
simulation (`PhoenixBackend`, compiled off-RoboRIO only) uses WPILib's
`LinearSystemId::DCMotorSystem` with `DCMotor` ratings (choosing the FOC
variant when `foc = true`; over UDP under `./sim.sh`, `motor_type` stays
off-wire so pushed configs default to `KrakenX60` / `KrakenX60FOC`, while
`KrakenX44` and `Falcon500` are selected when `MotorConfig::motor_type` is
populated in-process, e.g., unit tests) at
`J_rotor = 0.0005 kg·m²` reflected across each motor's gear ratio
(`J_mech = J_rotor × gear_ratio²`, ~44.5 ms electrical time constant for a
Kraken X60), writing rotor position, velocity, and acceleration plus remote
CANcoder state and forwarding any positive simulated RoboRIO battery voltage
(clamped to Phoenix 6's `4 V` floor) to each simulated device on change. Brake neutral drives `0 V` into the plant
(shorted-winding back-EMF braking), while coast neutral decays exponentially
(`tau = 1.0 s`) with the `0.01 rps` standstill snap. When swerve drive/steer
pairs (`front`/`back`/`rear`, `left`/`right`, ±0.30 m) or differential
left/right drive pairs (0.60 m trackwidth) are inferred from motor names,
centered rigid-body kinematics integrate yaw and yaw rate into every configured
`Pigeon2`; non-chassis configurations leave manually injected `Pigeon2` raw yaw
untouched.

### An external plant

`sim::PhysicsSimBackend::SetPlant` attaches a `sim::ExternalPlant`
(`sim_plant.h`), which integrates the motion of the motors it binds and, when
its model's root body has a free joint, the chassis. The backend stays the
only controller and electrical model. At each `Configure` the plant binds
motors by name (`ExternalPlant::Bind`); a program that builds its plant
before attaching it reads the same motor list (names, gears, winding
constants) from `PhysicsSimBackend::PlantMotors`. From then on a bound motor's rotor
position and velocity, and the chassis pose and velocity when the plant owns
it, are the plant's; a configuration never resets the plant's world, and the
gyro zeroes on the plant's heading at each bind and controller restart.
Unbound motors, the battery, controllers, Motion Magic profiles, CANcoders,
remote feedback, home switches, limit switches and faults stay the backend's.

Each `Read` hands every bound motor the drive its winding model gives at the
tick-start rotor speed (`sim::PlantDrive`: volts, whether back-EMF applies,
and torque bounds), in that speed's regime:

- neutral: a shorted winding under brake, an open one under coast;
- a voltage mode: the command clamped to `max_voltage` and the solved supply;
- a follower: at its own tick-start speed, exactly the current the analytic
  model gives it there (its master's applied volts through its own stator
  band and supply limit). When its master's drive, turned into the
  follower's direction and bounded only by the follower's own
  `stator_limit_a`, carries that same current there (a follower on its
  master's shaft), it takes that drive and follows the master's regime at
  every step; otherwise (a follower on another shaft, or its own supply
  limit binding) it holds its master's applied volts for the tick within
  its own limits;
- torque current: the voltage ceiling in the demand's direction, with the
  demanded current as the torque bound on that side (0 A gives no torque
  unless the rotor overspeeds the ceiling); a rotor back-driven so hard that
  the demand would need more than the opposite ceiling gets that ceiling,
  carrying more than the demand, with the demand as the bound on the other
  side;
- a `supply_limit_a` binding at the tick-start speed: its limited voltage,
  held for the tick, in every mode (torque current keeps its demand as a
  bound).

`stator_limit_a` bounds the torque on both sides in every powered mode. The
plant applies the drive's torque law at every one of its steps, so the
stator limit and the torque-current demand hold at each step's speed. At the
tick-start speed the torque is `k_t` times the stator current the analytic
model gives there, except where that current exceeds `stator_limit_a` (back-EMF
beyond the voltage ceiling plus `stator_limit_a` times the winding
resistance), which the plant caps at the limit. The ceiling is `max_voltage`
or the solved supply, whichever is lower, so the cap also applies when a
sagging battery lowers the ceiling below back-EMF. The battery sag is solved
with that same current at the tick-start speed.

The State reports what the drive carried at the speed the plant reached: the
stator current is the plant's current `I`; the voltage is the winding's,
`I` times the resistance plus back-EMF, held within the ceiling (at the
ceiling while the regeneration cap binds); the supply current is that
voltage times `I` over the bus voltage, at most `supply_limit_a`.

The plant steps in whole steps of its own `step_us`. Each `Read` adds the
time since the last one to a signed remainder and steps the nearest whole
number of steps, at most `2 * ceil(period_us / step_us)`; time past that cap
is dropped and counted (`PhysicsSimBackend::plant_time()`), never caught up.
Every other model the backend runs (Motion Magic profiles, unbound motors,
the analytic chassis, gyro drift) advances by the same plant time, and every
motor and CAN sensor sample is `max(lag_us, 0)` µs old. No wall clock is
read.

A bound motor's soft limits hold its output toward a limit it has reached
neutral (torque current judged by its demand's sign) and leave the joint
where it is; an analytic motor is clamped instead. A device reset restarts a
bound motor's controller but not its joint. `SetWorldInputs` binds a digital
input to a plant sensor by name; it reads `closed_reads` while the sensor is
closed and ignores `SetDigitalInput`.

`Configure` refuses, naming the key and the motor or input in
`last_config_error()`:

- a follower chain the plant binds only partly;
- `[[simulation.mechanisms]]` or `[[simulation.hard_stops]]` naming a bound
  motor (inertia and travel come from the model);
- a swerve module whose drive or steer motor the plant does not bind while
  the plant owns the chassis, and any swerve module while it does not;
- a digital input bound to a plant sensor twice;
- a plant `step_us` that is zero or odd;
- whatever the plant's `Bind` refuses, with the plant's error;
- a plant that has failed, with its `failure()` text.

A plant whose step fails (`ExternalPlant::Step` returning false) latches: the
backend makes no further call to it, every later `Read` reports every motor
and CAN sensor invalid and returns false, so the gateway neutralizes every
output (total loss), and world inputs hold their last reading.

### Which simulator to use for what

| Path | How to run | Motor / chassis model | Driver Station & gamepad | Truth (`/sim/state`) & faults | Use for |
|---|---|---|---|---|---|
| Ideal (`SimBackend`) | `sim_gateway` (default, no `--physics`), or `SimHardware` in unit tests | Instant velocity/position; zero gains still move; zero Pigeon2 raw yaw | Scripted `--mode-schedule` / `--joystick-schedule` | None (resets & I/O setters in C++ tests only) | Fast logic, wiring, sequencing, mode/timeout unit tests |
| Lightweight physics (`sim_gateway --physics`) | `./sim.sh --sim-gateway` (or `sim_gateway --physics`) | `sim::PhysicsSimBackend`: winding/current-limit model, gearing, opt-in rigid-body mechanism inertia, battery sag/brownout, hard stops, home switches, kinematic or dynamic mass/traction swerve ground pose & Pigeon2 yaw | Scripted `--mode-schedule` / `--joystick-schedule` (no USB gamepad) | Publishes `/sim/state` for cameras; odometry error (`--wheel-radius-error`, `--wheel-slip-gain`, `--gyro-drift-radps`) & `--fault-schedule` | Headless CI/soak runs, vision/odometry error tests, fault injection, PID/limit/homing/battery-sag verification |
| WPILib desktop sim (`PhoenixBackend`) | `./sim.sh` (or `./gradlew simulateNative` in `controller_processor/rio`) | Real Phoenix 6 simulator + WPILib `LinearSystemId::DCMotorSystem` plant (`KrakenX60`/`KrakenX60FOC` over UDP since `motor_type` is off-wire; `KrakenX44`/`Falcon500` when populated in-process) + inferred swerve/differential Pigeon2 yaw | WPILib Sim GUI with live USB gamepad (or `--headless` scripted schedules) | None (cameras emit invalid observations) | Interactive driving with a USB gamepad, testing real Phoenix 6 closed-loop/FOC/status-signal behaviour, RoboRIO-in-the-loop verification |

### Ground truth and odometry error

`PhysicsSimBackend::truth()` exposes true planar field pose (meters/radians)
and field-frame velocity (m/s, rad/s). By default, each step converts every
registered swerve wheel's surface velocity and steer azimuth into ground
velocity, solves the rigid-body least-squares chassis motion, and integrates
the field pose; when `SetChassisDynamics` (`[simulation.chassis]`) configures
a positive chassis mass, each step instead integrates coupled 2D rigid-body
mass, rotational inertia, per-wheel friction-circle traction clamping, lateral
scrub resistance, and slipping drive wheel spin-up. Conventional
`front`/`back` (`rear`), `left`/`right` drive/steer names infer module
coordinates at ±0.30 m when no explicit modules are set; `SetSwerveModules`
binds explicit drive/steer pairs by motor name (surviving every later
`Configure` and replacing name inference), and `SetSwerveModuleGeometry`
overrides or registers modules by logical id. No gyro is required to compute
truth.

`sim::PhysicsOptions` selects the start pose, nominal wheel radius
(`wheel_radius_m`, default `0.0508` m), and three deterministic errors
(all errors default to zero):

- `wheel_radius_error_frac = e`: true radius is `wheel_radius_m × (1 + e)` m.
  Rotor/encoder readings still measure wheel rotation faithfully; only ground
  travel changes, so odometry using the nominal radius drifts.
- `wheel_slip_gain = g`: each module's slip fraction is
  `min(1, g × abs(surface_acceleration_mps2) / 9.80665)`; ground speed is
  surface speed times `1 − slip`. Constant wheel speed has no slip.
  This deliberately simplified model only reduces travel during acceleration
  **and braking**; odometry over-reports travel in either case. There is no RNG.
- `gyro_drift_radps = d`: reported gyro yaw gains `d × elapsed_seconds`, and
  reported yaw rate gains `d`. Neither changes physical truth.

Pigeon yaw is relative to its boot heading (zero initially even for a nonzero
true start yaw), then mounting offset/inversion apply as usual.
`RestartController` preserves the true field pose, resets reported gyro yaw
and drift elapsed time, clears rotor motion/commands/faults as before, and
leaves config/output states intact. `SetGyro`/`SetChassisYawRate` override only
the reported gyro until configuration or controller restart, without
teleporting the robot or changing its module-derived ground motion.

Only `sim_gateway --physics` publishes truth on `/sim/state`
(`geometry.OdometrySample`) each hardware tick. It uses the main-processor node
clock, blue-alliance field coordinates (x downfield, y left, yaw CCW+), and
field-frame velocity. Its default start pose is `2,2,0`; the backend's default
start pose for tests is `0,0,0`. Ideal simulation and WPILib/RoboRIO simulation
publish no truth, so simulated cameras there produce invalid observations.
The estimator's independent initial pose is not moved to match the gateway.

