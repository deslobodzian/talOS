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
frames; `Endpoint` reassembles them, calls `Gateway::Reconfigure`, and answers
with a `kHardwareConfigAck`. The manifest holds logical device IDs, names, CAN
addresses and bus names, inversion, encoder offsets, current/voltage limits,
gearing, three gain slots, soft limits, and Motion Magic cruise velocity,
acceleration, and jerk. Reconfiguration is only accepted while disabled.

Logical IDs remain independent of CAN addresses. The default IMU is a Pigeon2,
mounted in its native frame. Steering uses a CANcoder on the same CAN bus as
its TalonFX. Different buses may reuse CAN addresses.

Example addresses and zero gains are not commissioned settings. After matching
the manifest to the physical robot and tuning it, explicitly set the hardware
`commissioned` flag. Until then telemetry is available but commands are
refused.

Both endpoints compute a canonical configuration fingerprint. Changing a CAN
address, offset, limit, or gain changes that ID; commands with a different ID
are refused. This is a compatibility check, not authentication. Configuration
validation rejects duplicate IDs/names/addresses, invalid numeric values, and
remote encoder references that do not exist on the motor's bus.

A rejected configuration latches a `ConfigError` (`config_error.h`) on the
gateway (`Gateway::config_error()`, cleared by the next success): device name,
CAN bus, CAN ID, the operation attempted (e.g. `apply_talonfx_config`), the raw
vendor status value, and a message.

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
followers, and dynamic Motion Magic requests are not implemented. Duty-cycle
encoder velocity is finite-difference across Reads.

Besides motors and CAN sensors, the manifest accepts RIO digital inputs and
outputs, analog inputs, quadrature and duty-cycle encoders, and PWM outputs.
Phoenix and both sims implement all of them.

CAN bus load and measured sensor age must be checked on the actual bus before
choosing production frequencies. The backend configures signal rates
explicitly and uses one-shot control requests.

References: [CTRE remote sensor control](https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/remote-sensors.html)
and [Motion Magic](https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html).
Phoenix 6 is pinned in `2026-robot/controller_processor/rio/vendordeps/Phoenix6.json`.

## Runtime Contract

Frame types `kHardwareState` and `kHardwareCommand` use the versioned UDP frame
transport. Their payload codecs explicitly encode little-endian integers and
IEEE-754 doubles, with exact length checks. Native C++ object layout is never
the network format. The payload limit is `protocol::kMaxPayloadSize`, currently
1400 bytes; `Packet` and the endpoint buffer are both sized from that constant.

The gateway publishes its configuration ID, boot ID, control epoch, monotonic
poll timestamp, last accepted command sequence, enable/configuration/fault
flags, and identified motor/sensor samples. Motor samples contain mechanism
position/velocity, voltage, and stator current. CANcoder samples contain angle
and velocity; Pigeon2 samples contain yaw and yaw rate, expressed in rotations.
Position/velocity signal ages are included separately, in microseconds. Ages
describe cached Phoenix measurements at polling; samples are not a synchronized
CAN capture.

Commands echo the gateway's boot ID, epoch, and poll timestamp. Expiry is checked
using only the gateway clock, so it does not assume synchronized host clocks.
Duplicate/reordered command sequences, old sessions, expired observations,
unknown modes, non-finite values, and out-of-range demands are rejected before
any motor request is applied. Each command supplies every configured motor in
manifest order; the bridge merges per-subsystem requests into that one command.

Driver Station disable/e-stop, command timeout, or invalid telemetry neutralizes
outputs. Enable transitions and timeout/telemetry stops invalidate outstanding
commands by advancing the epoch. A failed configuration or motor apply prevents
further output; an apply failure remains latched until process restart. A
transient status failure may recover, but requires a new valid command.

Each `Tick` polls `Backend::PollDeviceResets()` (Phoenix: `HasResetOccurred()`
per TalonFX/CANcoder/Pigeon2). On any reset the gateway invalidates outstanding
commands (epoch bump plus `Neutral`) and then re-applies configuration
(`RestoreAfterReset()`), so a freshly reset device never resumes an obsolete
demand. A failed restore latches a fault.

Fault scope: stale motor current/voltage zeroes the effort fields but keeps the
sample valid; an invalid RIO encoder invalidates only that sample; stale or
invalid motion feedback, CAN sensor loss, disconnects, and failed
Apply/Configure neutralize every output. DIO/analog/PWM never sink the backend.

## RoboRIO

`2026-robot/controller_processor/rio/src/main/cpp/Robot.cpp` is the thin WPILib
adapter. Set its companion address before deployment; the example is
`10.56.87.11`. Runtime ports are 5802 on the gateway and 5803 on the companion.
The other direction -- the companion finding the gateway -- is
`[robot] controller_address` in `robot.toml` (see `talOS/launcher/README.md`),
overridden per launch by `launcher --remote`. Real hardware uses Driver Station
enable and e-stop.

Build without deploying:

```sh
cd 2026-robot/controller_processor/rio
./gradlew buildRio
./gradlew phoenixTest   # PhoenixBackend desktop simulation regressions
```

WPILib desktop simulation (`./gradlew simulateNative`) runs the same gateway
against `PhoenixBackend` with Phoenix sim state. The RoboRIO program derives
from `frc::RobotBase`, not `frc::TimedRobot`, and runs its own fixed-rate
loop; the only WPILib calls in it are the DS mode observations that keep the
watchdog fed.

## Simulation Backends: Ideal vs Physics-Backed

Both implement `Backend`, both share the gateway safe-state policy
(disable/timeout/failure neutralizes every output type), and both step on the
explicit `sample_time_us` clock, so Gateway ticks with explicit timestamps are
exactly reproducible.

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
| Faults | Device resets, digital/analog/encoder setters | Plus frozen feedback, per-device disconnect, controller restart |

Tests live in `sim_physics_test.cc`. The standalone `sim_gateway` runs the
ideal backend by default and the physics backend with `--physics` (see
`2026-robot/main_processor/simulation/README.md`). The RoboRIO desktop model
keeps its own first-order Kraken model (same 100 rps at 12 V family).
