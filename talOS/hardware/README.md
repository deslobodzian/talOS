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
on the gateway and 5803 on the companion. Real hardware uses Driver Station
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

The RoboRIO program uses as little of WPILib as the Driver Station allows. It
derives from `frc::RobotBase`, not `frc::TimedRobot`, and runs its own
fixed-rate loop; the only WPILib calls in it are the DS mode observations that
keep the watchdog fed. That keeps the eventual move off GradleRIO small.
