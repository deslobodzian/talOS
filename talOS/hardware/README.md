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
validation rejects duplicate IDs/names/addresses, invalid numeric values, and
remote encoder references that do not exist on the motor's bus.

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
including when that frame survives a CANcoder reset).

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
`motor_reverse_limit_mask`.
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
[DATA_FLOW.md](../../docs/DATA_FLOW.md)), enable/configuration/fault flags,
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
latest accepted health sample, motor fault mask, and gateway counters.
`hardware_node --log PATH` records merged `/hw/command` and decoded `/hw/state`
as raw `Packet` topics in a `.tlog` (format in [talOS/events](../events/README.md)).

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
per TalonFX/CANcoder/Pigeon2). A reset neutralizes only affected motors and
their followers or remote-feedback dependents before starting
`BeginRestoreAfterReset(ids)`. The global epoch and unrelated commands survive;
pre-reset observations cannot authorize the affected motors after recovery.
`PollRestore()` never blocks: Phoenix reapplies only the affected device chain
on a worker while ticks, state publication and unrelated outputs continue.
Failed devices retry after 250 ms, doubling to 2 s. A successful restore requires
a fresh command observation before resuming. `Gateway::restore_attempt_count()`
counts all restore starts per boot. Motors whose control send failed join the
same unavailable set: their followers and dependents go neutral at once, the
rest of the command keeps applying, and restore re-applies the chain's config
before a fresh command retries the send. A motor that keeps failing is
re-isolated on that retry, at the restore backoff rate, without a global fault;
repeat failures do not count as `apply_failure_count`, but the motor stays
invalid and its demands count in `neutralized_motor_count`.

Fault scope: stale motor current/voltage zeroes the effort fields but keeps the
sample valid; an invalid RIO encoder invalidates only that sample. Invalid motor
motion feedback neutralizes only that motor's non-neutral demands. A missing or
invalid remote CANcoder also neutralizes only motors using it; healthy motors in
the same command continue driving. These neutralizations increment
`neutralized_motor_count`. The backend remains healthy while any configured
motor or CAN sensor has valid feedback (or none are configured). Total loss of
all configured motor/CAN-sensor feedback neutralizes every output and advances
the command epoch. Failed global Apply or backend configuration failure also
neutralizes every output; individual device configuration or control-send
failures do not. A rejected configuration push need not invalidate the active
configuration.
DIO/analog/PWM and RIO encoder validity never sink the backend.

## RoboRIO

`2026-robot/controller_processor/rio/src/main/cpp/Robot.cpp` is the thin WPILib
adapter. Set its companion address before deployment; the example is
`10.56.87.11`. Runtime ports are 5802 on the gateway and 5803 on the companion.
The other direction -- the companion finding the gateway -- is
`[robot] controller_address` in the robot config (see [launcher addressing](../launcher/README.md#controller-addressing)),
overridden per launch by `launcher --remote`. Real hardware uses Driver Station
enable and e-stop.
The adapter retries a failed gateway socket open once per second from its main
loop, ticking the gateway disabled (all outputs neutral) until the socket opens.

Build without deploying:

```sh
cd 2026-robot/controller_processor/rio
./gradlew buildRio
./gradlew phoenixTest   # PhoenixBackend desktop simulation regressions
```

WPILib desktop simulation (`./gradlew simulateNative`) runs the same gateway
against `PhoenixBackend` with Phoenix sim state. The RoboRIO program derives
from `frc::RobotBase`, not `frc::TimedRobot`, and runs its own fixed-rate
loop. It refreshes and forwards Driver Station data and reports the current
mode to feed the Driver Station watchdog.

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
| Position demand | Teleports | P-loop ramps through voltage; MotionMagic uses trapezoid profiles |
| Neutral | Velocity snaps to 0 | Unpowered decay: brake stops fast, coast rolls on |
| Gearing/inversion | Ignored | Rotor/gearing division, inversion flips direction; a follower drives in its master's direction (reversed if opposed), ignoring its own `inverted`; a remote-CANcoder motor reports and closes its loops on the CANcoder in the CANcoder's direction |
| Soft limits | Ignored | Clamped, axis stopped at the limit |
| Pigeon yaw | Zero raw yaw, then mounting offset/inversion | True chassis yaw relative to boot, plus optional drift; explicit gyro overrides remain available |
| Supply voltage | Ignored | Scales free speed (`SetSupplyVoltage`) |
| Faults | Device resets, digital/analog/encoder setters | Plus frozen feedback, per-device disconnect, controller restart |

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

