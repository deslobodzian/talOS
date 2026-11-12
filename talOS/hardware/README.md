# Hardware Gateway

The gateway configures devices, reads status signals, and applies actuator
requests. Swerve kinematics run in the companion drivetrain process. TalonFX
controllers own their PID and Motion Magic calculations; the gateway does not
implement a second motor PID loop.

`Backend` in `gateway.h` is the platform boundary. `PhoenixBackend` under
`2026-robot/controller_processor/rio/` implements it with Phoenix 6 on the
RoboRIO. `SimBackend` (`sim_backend.h`) provides ideal actuators and
`sim::PhysicsSimBackend` (`sim_physics.h`) a physics-backed model for software
tests and `sim_gateway`.

## Configuration

The bridge (`//talOS/bridge:hardware_node`) parses the robot's TOML
configuration and pushes it to the gateway as chunked `kHardwareConfig`
frames; `Endpoint` reassembles them, starts `Gateway::BeginReconfigure`, and
answers with a `kHardwareConfigAck` when the worker finishes. A push completed
while a configure is still pending (every boot restore) is dropped without an
ACK; the bridge pushes again after its 50 ms retry gate. The manifest holds
logical device IDs, names, CAN
addresses and bus names, inversion, encoder offsets, current/voltage limits,
gearing, three gain slots, soft limits, TalonFX limit switches, and Motion
Magic cruise velocity, acceleration, and jerk. Changed configurations require
disabled operation. Identical pushes may succeed while enabled without device
traffic, but not while faulted or while configuration/restoration is pending.

The RoboRIO atomically and durably (synced file, rename, synced directory)
saves acknowledged configuration bytes with a CRC32 to
`/home/lvuser/talos_config.bin` (`talos_config.bin` in desktop simulation;
`ConfigStore` in the RoboRIO program), on its first disabled tick after the
acknowledgement, since the storage sync blocks the loop. Startup validates
them and starts their restoration on the configure worker without waiting for
it, so robot code reports ready at once; the gateway stays unconfigured,
refusing commands, until the worker finishes. Missing or corrupt files leave
the gateway unconfigured. A stale file cannot accept commands from a different
configuration ID.

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
refused.
On the main processor only `--sim` bypasses commissioning, in memory, and the
bridge and every node apply it under that same flag: the bit is part of the
configuration ID they compare. `--sim` rejects non-loopback remotes; a loopback
remote without it keeps the configured value.

Both endpoints compute a canonical configuration fingerprint. Changing a CAN
address, offset, limit, or gain changes that ID; commands with a different ID
are refused. This is a compatibility check, not authentication. Configuration
validation rejects duplicate IDs/names/addresses, invalid numeric values,
remote encoder references that do not exist on the motor's bus, and a
`status_hz` giving fewer than four status periods per `command_timeout_us`
(feedback that old is invalid; see "Fault scope" below).

Device tables reject unknown keys in both runtime parsing and manifest checking.
CANcoder and Pigeon2 accept `type`, `bus`, `can_id`, `inverted`, and `offset_rot`.
QuadratureEncoder accepts `type`, `dio_a`, `dio_b`, and `counts_per_rev`;
DutyCycleEncoder accepts `type`, `dio`, and `offset_rot`; digital IO accepts only
`dio`, while analog inputs and PWM outputs accept only `channel`.

Motor tables may specify `follow = "master_motor"` and optional
`oppose_master = true` (default false). The master must be a motor owned by
the same subsystem on the same CAN bus; self-follow and follower chains are
rejected. Followers use the master's output rather than their own demand,
and neutralize when the master is neutral or loses valid feedback.
Both simulation backends support same-direction and opposed followers.

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
value, naming the key, the motor and its file. Only the physics simulator
reads it, for its winding model (stator current, free speed; see the backend
table below). It is not in the wire format or the configuration fingerprint,
so a config the bridge pushes to `sim_gateway` carries none: the physics
backend gives each pushed motor the type of the motor of the same name in
`sim_gateway`'s own parse of `--config` (`PhysicsSimBackend::SetLocalMotorTypes`).

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

Each closed-loop request selects gain slot 0, 1, or 2. Drive motors use rotor
feedback divided by the configured mechanism ratio. Steering uses
`RemoteCANcoder` with continuous wrapping. The backend uses standard voltage
controls and does not require Phoenix Pro. TorqueCurrentFOC, FusedCANcoder,
and dynamic Motion Magic requests are not implemented. Duty-cycle
encoder velocity is finite-difference across Reads.

Besides motors and CAN sensors, the manifest accepts RIO digital inputs and
outputs, analog inputs, quadrature and duty-cycle encoders, and PWM outputs.
Phoenix and both sims implement all of them.

CAN bus load and measured sensor age must be checked on the actual bus before
choosing production frequencies. Motion feedback requests the manifest status
rate; motor voltage, stator current and fault field request
`min(status_hz, 50)` Hz. Phoenix applies the highest requested rate to signals
sharing a frame: with the pinned vendor library, voltage/current still apply
at 200 Hz alongside 200 Hz motion feedback, while the fault field applies at
50 Hz. See [vendor signal frequency semantics](https://v6.docs.ctr-electronics.com/en/stable/docs/api-reference/api-usage/status-signals.html#changing-update-frequency).
The backend then calls
`OptimizeBusUtilization()` on each TalonFX, CANcoder, and Pigeon2. Per-device
optimization keeps the explicitly selected signals while stopping unused
default frames; it does not replace measuring the real bus. Reads refresh the
cached signals non-blockingly. One-shot control requests are refreshed by the
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
Phoenix 6 is pinned in `2026-robot/controller_processor/rio/vendordeps/Phoenix6.json`.

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
and are not counted as rejected.

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
Phoenix fault bitfield. State also carries valid power/CAN health (battery
volts, brownout, and CAN utilization fraction) and per-boot gateway safety
counters. Peers of another protocol version (`kProtocolVersion`, see
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
demands/feedforward neutralize only that motor. Per-boot counters report refused
commands, neutralized motors, apply failures, and device resets. The refused
count omits a current, all-neutral command while the gateway is disabled,
uncommissioned or unconfigured: the bridge keeps forwarding those while the
robot cannot drive. Each command supplies every configured motor in manifest
order.
The bridge neutralizes omitted subsystem actuators. `/hw/status` exposes the
latest accepted health sample, motor fault mask, gateway counters and gateway
build (`gateway_program`, `gateway_build_id`, `gateway_build_dirty`), and the
bridge prints the build once for each new gateway boot
(`hardware_node: gateway boot <id>: roborio build <revision>`). A State
that does not decode is dropped, with one stderr line until a State decodes
again.
`hardware_node --log PATH` records each `/hw/command` and `/hw/state` it
publishes (the merged command, every accepted State; a refused State is not
published) as raw `Packet` `SEND`s in a `.tlog` (format in
[talOS/events](../events/README.md)), each with the transport sequence and
publication time its readers receive, and the pass that produced it as its
event time.

Nodes read the gateway's poll timestamp in their own clock through
`HardwareLink::sample_time()` (C `talos_hardware_sample_time`, Python
`talos.hardware_sample_time_ns(node)`). The bridge keeps the mapping
(`talOS/bridge/clock_offset.h`) and publishes it on `/hw/status`. Empty clock
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
`BeginRestoreAfterReset(ids)`. The global epoch and unrelated commands survive;
observations up to the last State that masked the affected motors cannot
authorize them after recovery.
`PollRestore()` never blocks: Phoenix reapplies only the affected device chain
on a worker while ticks, state publication and unrelated outputs continue.
Failed devices retry after 250 ms, doubling to 2 s. A successful restore requires
a command observing a State after the last masked one before resuming.
`Gateway::restore_attempt_count()` counts all restore starts per boot. Motors whose control send failed join the
same unavailable set: their followers and dependents go neutral at once, the
rest of the command keeps applying, and restore re-applies the chain's config
before a fresh command retries the send. A motor that keeps failing is
re-isolated on that retry, at the restore backoff rate, without a global fault;
repeat failures do not count as `apply_failure_count`, but the motor stays
invalid and its demands count in `neutralized_motor_count`.

Fault scope: stale motor current/voltage zeroes the effort fields but keeps the
sample valid; an invalid RIO encoder invalidates only that sample. A motor's or
CAN sensor's motion feedback is invalid when the backend reports a signal
error, a non-finite value or an isolated device, or when its position or
velocity sample is `command_timeout_us` old or older (an unknown age,
`UINT32_MAX`, included); the gateway applies that age bound after
`Backend::Read`, the same for every backend (`FeedbackFresh`, `gate.h`), so a
gap shorter than that bound does not invalidate the feedback at the gateway;
a consumer's own, tighter age bound still applies (the drivetrain drops
odometry samples older than 50 ms, see its [README](../../2026-robot/main_processor/drivetrain/README.md)).
A motor or CAN sensor sample marked invalid, by the backend, the age bound
or the gateway's mask of an unavailable device or its follower, carries zero
in every measured field (position, velocity, applied voltage, stator and
supply current; `Gateway::snapshot`), never its last values; its ages,
temperature and fault bits are the backend's. Reset detection (above)
skips an invalid sample, so its zero is never a position step. Invalid motor
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
force. Failed global Apply or backend configuration failure also
neutralizes every output; individual device configuration or control-send
failures do not. A rejected configuration push need not invalidate the active
configuration.
DIO/analog/PWM and RIO encoder validity never sink the backend.

## RoboRIO

`2026-robot/controller_processor/rio/src/main/cpp/Robot.cpp` is the thin WPILib
adapter. Its companion (main processor) address is fixed when it is built:
`./gradlew deploy -Pcompanion=<ipv4>` (or `buildRio`), `10.56.87.11` without
the property. `build.gradle` takes it only from the command line: a
`companion` set in a `gradle.properties` or `ORG_GRADLE_PROJECT_companion`
fails the build, so a bench address cannot carry into a later deploy from the
same laptop. It also fails the build on an empty value (a bare `-Pcompanion`
or an unset shell variable) and on anything but a dotted IPv4 literal without
leading zeros outside `0.0.0.0/8`, loopback `127.0.0.0/8` and `224.0.0.0/3`
(multicast, reserved, broadcast), addresses the RoboRIO's socket could not
open or send to. Every Gradle run prints the address it builds in and whether
it is the default (`RoboRIO companion address built in: ...`). It is
never learned from an incoming datagram: the gateway's socket is connected to
that one peer, which is what keeps any other host on the network from
commanding the robot ([protocol](../protocol/README.md)). Runtime ports are
5802 on the gateway and 5803 on the companion.
The other direction -- the companion finding the gateway -- is
`[robot] controller_address` in the robot config (see [launcher addressing](../launcher/README.md#controller-addressing)),
overridden per launch by `launcher --remote`. Real hardware uses Driver Station
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

That loop enforces the command timeout and output neutral, so it never
writes a report itself. `PhoenixBackend` and the adapter
post fixed-size events (fault-field, feedback-validity, late-feedback, apply
and neutral send transitions, device resets, brownout) to `RioLog`
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
loop's timing faults: overruns (a tick's work, from wake to sleep, longer than
the period) and late ticks (a tick starting more than 1.5 periods after the
previous, a starved loop however short its work), with that second's longest
work and interval. Every 10 s, when the loop ticked in that window, it
prints where the backend's part of those ticks went:

```
[rio] tick phases: refresh mean/max <a>/<b> us, devices <a>/<b> us, control <a>/<b> us, health <a>/<b> us, other <a>/<b> us over <n> ticks (motors=<m> sensors=<s>)
```

per tick, the mean and longest time in `BaseStatusSignal::RefreshAll`
(refresh), the per-device validity, latency and `HasResetOccurred` pass
(devices), TalonFX `SetControl` for every motor, whether a command's or
neutral (control), the CAN status and battery reads (health), and the rest
of the backend's tick calls (other). `PhoenixBackend` takes one
steady-clock reading per phase boundary, ten per disabled tick, and adds
them on the loop thread without locks; the window passes to the logger at
the loop's next tick ([bench measurement](../../docs/BENCH.md#fault-injection-and-timing)).
`State` has no field for these, so they are RIO-local.
Each of these lines, and the program's three start lines, begins with
`[<seconds>.<milliseconds>]` on the gateway's steady clock: on the RoboRIO,
time since its kernel booted, the clock of State's `sample_time_us`, so a
`hardware_node --log` record of a State places the RIO's lines on the
session's time. A RioLog line's time is when the logger wrote it, within one
20 ms drain of the event unless that thread is starved. Driver Station
messages (`talos companion link`, `talos loop watchdog`) and the WPILib and
Phoenix libraries' own lines carry no such time.

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
each tick ends with a beat to a loop watchdog
(`src/main/include/LoopWatchdog.h`) carrying, while a command is still in
force, the time by which the loop's next tick would have invalidated it: the
gateway's `command_expiry_us()` (the older of the command's acceptance and
the State it was built from, plus `command_timeout_us`) plus two loop
periods (one for the tick that sees the expiry, one for that tick's work
before its beat); with no command in force it carries nothing (that tick neutralized
every output itself). The watchdog's own thread, at `SCHED_FIFO` 30 on the
RoboRIO (above the loop's `SCHED_FIFO` 10 and the kernel's interrupt
threads), checks every 5 ms. Once that
time has passed it disables PWM outputs, sets DIO outputs false and sends
`NeutralOut` to every motor, on every check until a beat moves the time (so
outputs a resumed loop re-applies on a stale clock reading before stalling
again are neutralized again), and sends one Driver Station error per stall,
`talos loop watchdog`, with the silence and a stall count. It touches only
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

A tick that ends by its deadline sleeps to it, so the loop keeps its phase.
One that ends past it (an overrun, or a wake more than a period late) drops
the missed ticks instead of running them back to back, sleeps 500 µs and
counts the next period from there, so the phase moves
(`src/main/include/LoopSchedule.h`). Whenever it runs, the loop takes the
CPU from every thread on its core below `SCHED_FIFO` 10: the logger and the
companion reporter (nice 19), ssh, the program's CAN threads (`can_psend`,
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
`--offline` builds get it, and rewrites the header only when it changes. The
header also carries the companion address. The program prints both on its
console at start (`hardware gateway build <commit> companion <address>:5803`,
the address `127.0.0.1` under simulation), and the State reports the
commit's first 16 hex digits
([DATA_FLOW.md](../../docs/DATA_FLOW.md#udp-frame-and-payload-sizes)), as
`ROBORIO`, or `ROBORIO_SIMULATION` under WPILib simulation, on `/hw/status`.

Build without deploying:

```sh
cd 2026-robot/controller_processor/rio
./gradlew buildRio
./gradlew phoenixTest   # desktop tests: PhoenixBackend sim, ConfigStore, SimDriverStation, RioLog, CompanionLink, LoopWatchdog, SocketRetry, LoopSchedule
```

WPILib desktop simulation (`./gradlew simulateNative`) runs the same gateway
against `PhoenixBackend` with Phoenix sim state. The RoboRIO program derives
from `frc::RobotBase`, not `frc::TimedRobot`, and runs its own fixed-rate
loop. It refreshes and forwards Driver Station data and reports the mode it
is running in, which the Driver Station displays; whether the Driver Station
disables the robot when those reports stop is unverified (the loop stall
check in [BENCH.md](../../docs/BENCH.md#fault-injection-and-timing)), and
the loop watchdog does not rely on it.

## Simulation Backends: Ideal vs Physics-Backed

Both implement `Backend`, both share the gateway safe-state policy
(disable/timeout/failure neutralizes every output type), and both step on the
explicit `sample_time_us` clock, so Gateway ticks with explicit timestamps are
exactly reproducible.
Both publish valid 12 V, no-brownout, zero-CAN-load health by default;
`SetPowerHealth(volts, brownout)` injects low-power telemetry. Physics also uses
the injected voltage as its motor supply. Health is observable while disabled
or without a live command. On the real backend, neutral disables PWM pulses
(`PWM::SetDisabled`) rather than commanding servo centre, and de-energizes DIO.

| | Ideal (`SimBackend`) | Physics-backed (`sim::PhysicsSimBackend`) |
|---|---|---|
| Use for | Fast logic tests: wiring, sequencing, timeouts | PID/gearing/direction/limit believability |
| Velocity demand | Applies instantly, even with zero gains | P+FF loop produces volts; zero gains stall |
| Position demand | Teleports (no position-jump reset check) | P-loop ramps through voltage; MotionMagic uses trapezoid profiles |
| Neutral | Velocity snaps to 0 | Unpowered decay: brake stops fast, coast rolls on |
| Gearing/inversion | Ignored | Rotor/gearing division, inversion flips direction; a follower drives in its master's direction (reversed if opposed), ignoring its own `inverted`; a remote-CANcoder motor reports and closes its loops on the CANcoder in the CANcoder's direction |
| Soft limits | Ignored | Clamped, axis stopped at the limit |
| CANcoder position | Its motor's position, wrapped to [-0.5, 0.5) rotations as the RoboRIO reports it | The linked mechanism's, wrapped the same way |
| Pigeon yaw | Zero raw yaw, then mounting offset/inversion | True chassis yaw relative to boot, plus optional drift; explicit gyro overrides remain available |
| Supply voltage | Ignored | Scales free speed (`SetSupplyVoltage`) |
| Stator current | Zero | (applied volts - back-EMF) / R, clamped to `stator_limit_a`; R and free speed from the motor type's published 12 V stall current and free speed (`MotorModelFor`: Kraken X60 366 A and 6000 rpm, the default; Kraken X44 275 A and 7530 rpm; Falcon 500 257 A and 6380 rpm). A motor held at a soft limit is stalled. The type is the motor's `motor_type` key (above) |
| Faults | Device resets, digital/analog/encoder setters | Plus frozen feedback (valid, its age growing), per-device disconnect, controller restart |

Tests live in `sim_physics_test.cc`. The standalone `sim_gateway` runs the
ideal backend by default and the physics backend with `--physics` (see
`2026-robot/main_processor/simulation/README.md`). The RoboRIO desktop model
keeps its own first-order Kraken model (same 100 rps at 12 V family) and the
same neutral rule: neutral removes drive and the rotor decays per `brake`
(about 0.05 s braked, 1.0 s coasting) instead of stopping instantly.

### Ground truth and odometry error

`PhysicsSimBackend::truth()` exposes true planar field pose (meters/radians)
and field-frame velocity (m/s, rad/s). Each step converts every registered
swerve wheel's surface velocity and steer azimuth into ground velocity, solves
the rigid-body least-squares chassis motion, and integrates the field pose.
Conventional `front`/`back` (`rear`), `left`/`right` drive/steer names infer
module coordinates at ±0.30 m; `SetSwerveModuleGeometry` registers other names
or dimensions. No gyro is required to compute truth.

`sim::PhysicsOptions` selects the start pose and three deterministic errors
(all errors default to zero):

- `wheel_radius_error_frac = e`: true radius is `0.0508 × (1 + e)` m.
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

