# Hardware Gateway

The gateway configures devices, reads status signals, and applies actuator
requests. Swerve kinematics run in the companion drivetrain process. TalonFX
controllers own their PID and Motion Magic calculations; the gateway does not
implement a second motor PID loop.

`Backend` in `gateway.h` is the platform boundary. `PhoenixBackend` under
`robot/` implements it with Phoenix 6 on RoboRIO. `SimBackend` provides ideal
actuators for software tests. A SystemCore port supplies a backend, monotonic
clock, and enable source; no SystemCore SDK implementation is claimed here.

## Configuration

Edit `swerve_config.h` and rebuild both programs. The shared C++ manifest holds
logical device IDs, names, CAN addresses and bus names, inversion, encoder
offsets, current/voltage limits, gearing, three gain slots, soft limits, and
Motion Magic cruise velocity, acceleration, and jerk. Configuration is applied
once at gateway startup. There is no runtime configuration upload service yet.

The module table also contains wheel radius and module locations for companion
kinematics. Logical IDs remain independent of CAN addresses. The default IMU is
a Pigeon2, mounted in its native frame. Steering uses a CANcoder on the same CAN
bus as its TalonFX. Different buses may reuse CAN addresses.

Example addresses and zero gains are not commissioned settings. After matching
the manifest to the physical drivetrain and tuning it, explicitly set the
hardware `commissioned` flag. Until then telemetry is available but commands
are refused. The fake simulation uses a separate commissioned configuration.

Both endpoints compute a canonical configuration fingerprint. Changing a CAN
address, offset, limit, or gain changes that ID; commands with a different ID
are refused. This is a compatibility check, not authentication. Configuration
validation rejects duplicate IDs/names/addresses, invalid numeric values, and
remote encoder references that do not exist on the motor's bus.

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
`RemoteCANcoder` with continuous wrapping. This initial backend uses standard
voltage controls and does not require Phoenix Pro. TorqueCurrentFOC,
FusedCANcoder, followers, and dynamic Motion Magic requests are not implemented.

The example exchanges setpoints and telemetry at 200 Hz. That rate is distinct
from the TalonFX onboard loop rate. CAN bus load and measured sensor age must
be checked on the actual bus before choosing production frequencies. The
backend configures signal rates explicitly and uses one-shot control requests.

References: [CTRE remote sensor control](https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/remote-sensors.html)
and [Motion Magic](https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html).
Phoenix 6 is pinned to 26.3.0 in `robot/vendordeps/Phoenix6.json`.

## Runtime Contract

New frame types `kHardwareState` (20) and `kHardwareCommand` (21) use the existing
versioned UDP frame transport. Their payload codecs explicitly encode
little-endian integers and IEEE-754 doubles, with exact length checks. Native
C++ object layout is never the network format. The payload limit is
`protocol::kMaxPayloadSize`, currently 1400 bytes; `Packet` and the endpoint
buffer are both sized from that constant rather than from a literal.

The gateway publishes its configuration ID, boot ID, control epoch, monotonic
poll timestamp, last accepted command sequence, enable/configuration/fault
flags, and identified motor/sensor samples. Motor samples contain mechanism
position/velocity, voltage, and stator current. CANcoder samples contain angle
and velocity; Pigeon2 samples contain yaw and yaw rate, expressed in rotations.
Position/velocity signal ages are included separately, in microseconds. Ages
describe cached Phoenix measurements at polling; samples are not a synchronized
CAN capture. Clock synchronization with vision or another computer remains an
estimator/integration task.

Commands echo the gateway's boot ID, epoch, and poll timestamp. Expiry is checked
using only the gateway clock, so it does not assume synchronized host clocks.
Duplicate/reordered command sequences, old sessions, expired observations,
unknown modes, non-finite values, and out-of-range demands are rejected before
any motor request is applied. Each command supplies every configured motor in
manifest order; each gateway endpoint has one command producer. Supporting
additional subsystem nodes on one endpoint will require an explicit command
aggregator or separate ownership groups.

Driver Station disable/e-stop, command timeout, or invalid telemetry neutralizes
outputs. Enable transitions and timeout/telemetry stops invalidate outstanding
commands by advancing the epoch. A failed configuration or motor apply prevents
further output; an apply failure remains latched until process restart. A
transient status failure may recover, but requires a new valid command.

## RoboRIO

`robot/src/main/cpp/Robot.cpp` is the thin WPILib adapter. Set its companion
address before deployment; the example is `10.56.87.11`. Runtime ports are 5802
on the gateway and 5803 on the companion. The other direction -- the companion
finding the gateway -- is `[robot] controller_address` in `robot.toml`
(see `talOS/launcher/README.md`), overridden per launch by `launcher --remote`.
Real hardware uses Driver Station
enable and e-stop; the standalone loopback fake is the only executable that
automatically enables its simulated actuators.

Build without deploying:

```sh
cd robot
./gradlew buildRio
./gradlew frcUserProgramOsxuniversalReleaseExecutable
```

WPILib simulation runs the same gateway against `SimBackend`'s ideal actuators.
Start `./gradlew simulateNative` and connect the companion bridge and node
described in `2026-robot/main_processor/drivetrain/README.md`, or run the whole thing at once with
`python3 tools/test_drivetrain.py --wpilib`. There is no simulation GUI: the
program has no dashboard-visible state, and the desktop build supplies its own
Driver Station instead (see the drivetrain README for the environment
variables that drive it).

## Simulation Backends: Ideal vs Physics-Backed

Two backends, two jobs. Both implement `Backend`, both share the gateway
safe-state policy (disable/timeout/failure neutralizes every output type),
and both step on the explicit `sample_time_us` clock, so Gateway ticks with
explicit timestamps are exactly reproducible.

| | Ideal (`SimBackend`) | Physics-backed (`sim::PhysicsSimBackend`) |
|---|---|---|
| Use for | Fast logic tests: wiring, sequencing, timeouts | PID/gearing/direction/limit believability |
| Velocity demand | Applies instantly, even with zero gains | P+FF loop produces volts; zero gains stall |
| Position demand | Teleports | P-loop ramps through voltage (no MotionMagic profile yet) |
| Neutral | Velocity snaps to 0 | Unpowered decay: brake stops fast, coast rolls on |
| Gearing/inversion | Ignored | Rotor/gearing division, inversion flips direction |
| Soft limits | Ignored | Clamped, axis stopped at the limit |
| Pigeon yaw | Zero | Integrates `SetChassisYawRate` chassis motion |
| Supply voltage | Ignored | Scales free speed (`SetSupplyVoltage`) |
| Faults | Digital/analog/encoder setters only | Plus frozen feedback, per-device disconnect, controller restart |

Details live in `sim_physics.h` (one reusable motor/mechanism model: motor
type, gear ratio, inertia, friction, roller/flywheel load) with tests in
`sim_physics_test.cc`. The standalone `sim_gateway` runs the ideal backend
by default and the physics backend with `--physics` (see
`2026-robot/main_processor/simulation/README.md`). Scheduling and UDP timing
stays out of the physics model: `period_us` pacing and link behavior are
endpoint concerns, tested separately. The RoboRIO desktop model keeps its own
first-order Kraken model (same 100 rps at 12 V family); converging it onto
this header is future work.

The RoboRIO program uses as little of WPILib as the Driver Station allows. It
derives from `frc::RobotBase`, not `frc::TimedRobot`, and runs its own
fixed-rate loop; the only WPILib calls in it are the DS mode observations that
keep the watchdog fed. That keeps the eventual move off GradleRIO small.

## Hardware Operations

Truthful device support, commissioning, failure diagnosis, resets, fault
scope, homing, health, and characterization. Behavior lives in the headers
named below; every behavior has a test in `hardware_ops_test.cc`.

### Support Matrix: Accepted by Config vs Supported End to End

"Accepted by config" (`Validate()` parses it) is not "supported end to end"
(a Phoenix device exists, both sims emulate it, and a test pins it).
`support_matrix.h` is the one place that distinction is stated; this table
mirrors it, and the test pins every row.

| Device | Parsed | Real (Phoenix) | Ideal sim | Physics sim | Tested | End to end |
|---|---|---|---|---|---|---|
| TalonFX motor | Yes | Yes | Yes | Yes | Yes | Yes |
| CANcoder | Yes | Yes | Yes | Yes | Yes | Yes |
| Pigeon2 IMU | Yes | Yes | Yes | Yes | Yes | Yes |
| DIO input | Yes | Yes | Yes | Yes | Yes | Yes |
| DIO output | Yes | Yes | Yes | Yes | Yes | Yes |
| Analog input | Yes | Yes | Yes | Yes | Yes | Yes |
| Quadrature encoder | Yes | Yes | Yes | Yes | Yes | Yes |
| Duty-cycle encoder | Yes | Yes | Yes | Yes | Yes | Yes |
| PWM output | Yes | Yes | Yes | Yes | Yes | Yes |

Known control-mode limits (all backends): standard voltage controls only.
TorqueCurrentFOC, FusedCANcoder, followers, and dynamic Motion Magic requests
are not implemented. Pigeon2 mounting uses its native frame only.
Duty-cycle encoder velocity is finite-difference across Reads.

### Commissioning (`commissioning.h`)

Example addresses and zero gains are not commissioned settings. Before the
`commissioned` flag is set, verify per motor: identity (name/ID/CAN match the
hardware), direction (positive demand moves the mechanism positive), gearing
(ratios measured, feedback source confirmed), offsets (encoder offsets
measured, soft limits sane), and limits (current and mechanical limits
verified safe); per sensor: identity plus the CANcoder magnet offset.
Record each sign-off with evidence in a `CommissioningRecord`; `ApplyTo()`
flips `commissioned` only when every device in the manifest is fully signed
off and cross-checks (positive ratios, ordered limits, positive current
limits) pass. Until then telemetry flows but commands are refused.

### Configuration Failures (`config_error.h`)

A rejected configuration latches a `ConfigError` on the gateway
(`Gateway::config_error()`, cleared by the next success): device name, CAN
bus, CAN ID, the operation attempted (e.g. `apply_talonfx_config`), the raw
vendor status value, and a message. `PhoenixBackend` records the first failed
device apply, including signal-rate setup failures.

### Device Resets (item 36)

Each `Tick` polls `Backend::PollDeviceResets()` (Phoenix:
`HasResetOccurred()` per TalonFX/CANcoder/Pigeon2). On any reset the backend
re-applies configuration (`RestoreAfterReset()`) and the gateway invalidates
outstanding commands (epoch bump plus `Neutral`), so a freshly reset device
never resumes an obsolete demand. A failed restore latches a fault. The total
count of reset device reports is visible in health. Sim faults via
`SimulateDeviceReset()`; the physics backend zeroes rotor states on restore,
mirroring a device that lost position across the reset.

### Fault Scope (`fault_scope.h`)

| Failure | Scope | Effect |
|---|---|---|
| Stale motor current/voltage | Telemetry-only | Effort fields zero; sample stays valid, robot keeps driving |
| Invalid RIO encoder | Mechanism | That sample goes invalid; the backend stays healthy (TalonFX loops use rotor/RemoteCANcoder, so only the owning subsystem stops using that encoder) |
| Stale/invalid motor position/velocity, stale/invalid CAN sensor, any disconnect, frozen feedback, failed Apply/Configure | Robot | Gateway neutralizes every output, bumps the epoch, latches the fault where applicable |

Per-mechanism isolation for robot-scope failures is future work:
`Gateway::Accept` applies all motors atomically. DIO/analog/PWM never sink
the backend.

### Homing and Zeroing (`homing.h`, `Gateway::Zero`)

Seek by procedure (disabled, move the mechanism to its physical reference and
hold it), capture in code: `Zero()` declares the held position reads `home`.
Completion is observable as a new epoch in `State`. Captures are rejected for
unknown motors, homes outside enabled soft limits, active commands (no
zeroing under load), observations older than `command_timeout_us` (a seek
interrupted long ago cannot zero against a stale frame), and backend
failures, which latch a fault. Any enable transition bumps the epoch and
invalidates the seek the capture was based on. Continuous-wrap (steering)
mechanisms may be zeroed; the offset shifts the wrap frame.

### Health (`health.h`, `Gateway::health()`)

Per-device motion/current/connectivity already rides in `State` samples and
validity flags. `health()` adds what was missing: the supply rail in volts
(Phoenix: first motor's supply signal; sims: the settable rail), the reset
count since boot, the gateway fault latch, and the configured/enabled edges.
Poll at low rate, never on the Driver Station fast path: the Phoenix rail
refreshes a signal. Sticky per-device fault bits are future work.

### Characterization (`characterization.h`)

SysId workflow, not its app structure: drive one mechanism open-loop through
a slow bidirectional voltage ramp, record synchronized
(time, voltage, position, velocity) samples with `CharRecorder` (monotonic
time, one motor, one config enforced), fit V = kS*sgn(vel) + kV*vel with
`FitStaticGains` (least squares over samples above the crawl threshold, with
R^2; degenerate datasets fail instead of yielding confident nonsense), share
the dataset via `ExportCharCsv`, and store `FormatCharProvenance` (manifest
id, motor, sample counts, fit, operator note) alongside any config the fit
produces. Gains without that block are unreviewable.
