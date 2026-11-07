# Drivetrain Node

Four-module, robot-relative swerve control with separate processes:

```text
ChassisTarget publisher -> drivetrain node -> hardware bridge -> RoboRIO
                              ^                    ^               |
                              |                    |               |
                              +---- hardware state +----- UDP -----+
```

The drivetrain discretizes chassis velocity by taking the SE2 logarithm of
the desired finite-period translation and rotation, compensating motion skew.
It selects the shorter steering path, cosine-scales drive speed by azimuth
error, and desaturates wheel speeds. At zero requested speed it
holds the measured steering angle. Drive requests
use TalonFX velocity control; steering uses position control with continuous
wrapping. The hardware API also supports Motion Magic when a subsystem needs
profiled positions. This is not yet a trajectory follower or field-relative
controller; pose estimation and vision run as separate nodes.

Optional `[subsystem]` parameters `max_linear_accel_mps2` and
`max_angular_accel_radps2` limit commanded chassis acceleration before
discretization. Translation uses a vector-magnitude limit, preserving the
direction of the velocity change; rotation is limited independently. Each
defaults to `0.0`; non-positive or non-finite values leave that limit disabled.
The period is the configured node update period, not elapsed wall time.
Disabled, stale/released or non-finite targets and disconnected module motors
neutralize immediately and reset both ramps to zero. Re-enabling therefore
starts a new ramp rather than resuming the previous command. The robot config
leaves the example limits commented out to preserve existing step behavior.

## Code

- `drivetrain.h`: the node, a `talos::Node` class with eight `talos::Motor`s
  and the Pigeon (`drivetrain_imu`); `drivetrain.cc` is `TALOS_NODE`.
- `swerve.h`: inverse and full least-squares forward kinematics (including
  off-center layouts), using `talos::geometry::SO2` for principal-valued
  steering deltas and `SE2::Discretize` for finite-period pose matching.
  The unit boundary
  (`ToRadians`, `ToRotations`, `WheelTravel`, `WheelSpeed`, `WheelRps`) uses
  plain doubles in radians/meters and mechanism rotations. Wheel speeds are
  desaturated together against each module's wheel radius and drive maximum.
- `geometry.h`: the geometry, read from `[subsystems.drivetrain.geometry]`;
  each module names its drive, steer and encoder devices, checked against
  this subsystem's devices at startup.
- `drive_message.fbs` and the `topics` in `BUILD`: `drivetrain::ChassisTarget`,
  `drivetrain::DrivetrainState` and `drivetrain::topics::k*`,
  `//2026-robot/main_processor/drivetrain:messages`.

`/drivetrain/state` (`DrivetrainState`) is published every tick, whatever
the hardware says:

- `modules_valid` is true when all eight module motors report and the sample
  time is known. While a motor does not report, nothing is driven (every
  module neutral), the chassis speeds read 0 and the four module positions
  (`module0`..`module3`, in geometry order: wheel travel in meters from the
  drive motor, azimuth in radians from the steer motor) hold their last valid
  values.
- `yaw_rad` is the Pigeon's yaw converted from rotations, and `yaw_valid`
  says whether the Pigeon reports. While it does not, `yaw_rad` holds the last
  valid heading (0 before the first). Under `--sim` the default instead
  integrates heading from module rotation, valid while the modules are,
  because WPILib/ideal sims report a valid, permanently zero Pigeon yaw.
  `integrate_heading = true|false` in `[subsystem]` overrides that choice.
  The physics gateway models the Pigeon, including injected gyro drift:
  `configuration/robot_physics_sim.toml` includes `robot.toml` and sets
  `[subsystems.drivetrain] integrate_heading = false` to use that modeled yaw.
  `./sim.sh --sim-gateway` selects this config; WPILib keeps `robot.toml`
  and module-integrated heading.
- `sample_time_ns` is `HardwareLink::sample_time()`: the gateway's sample
  time of the `/hw/state` the tick read, mapped into the node clock by the
  bridge's min-filtered offset (`talOS/hardware/README.md`). It reads later
  than the true sample by the link's minimum latency, never earlier. It is 0,
  with `modules_valid` and `yaw_valid` false, until the mapping converges
  (8 hardware states) and whenever hardware is stale; the modules are still
  driven then, since actuation does not need the time.

## Local Topics

Names follow `talOS/NAMING.md`, which is normative for their grammar and for
which end of a topic is allowed to publish it.

| Topic | Single publisher | Payload |
|---|---|---|
| `/drivetrain/target/<source>` | that source's producer (`teleop`: `driver`) | `talos::TargetEnvelope<ChassisTarget>` (mode generation + `active` + `ChassisTarget`) |
| `/hw/state` | Hardware bridge | Canonical hardware state in fixed `Packet` |
| `/hw/request/drivetrain` | Drivetrain | Partial command, this subsystem's devices only, in fixed `Packet` |
| `/hw/command` | Hardware bridge | Canonical hardware command in fixed `Packet` |

Each producer writes its own lane: `driver` publishes
`/drivetrain/target/teleop`, an auto routine `/drivetrain/target/auto`. The
node reads them through `talos::Target`, which follows the lane the robot's
`[robot.targets]` policy names for the current Driver Station mode (nothing
while the Driver Station is disabled, e-stopped or silent).

The driver applies a circular stick deadband and caps translation radially.
With a fresh heading, field-oriented translation is driver-relative: gyro zero
at boot is away from this driver, with no alliance flip. Joystick 0's
`rezero_button_id` (1-based, default 1; non-positive disables) latches the current
fresh heading as driver-forward on its rising edge. Heading loss stops
translation until the translation sticks are centred, then re-arms robot-relative
drive. It stays robot-relative until heading is fresh **and** translation sticks
are centred again; rotation remains available throughout. Frame transitions are
logged by the driver. `heading_hold_radps` sets angular
correction per radian of wrapped heading error (configured to 4.0, compiled
default 0.0); manual rotation resets the held heading, and stale headings or
disabled/disconnected Driver Station state cancel hold.

Chassis units are meters/second and radians/second. Axes are forward X, left Y,
CCW yaw. A released lane (`active = 0`), a lane older than the robot's
`command_timeout_us` (by receive time), stale/invalid/disabled hardware state,
or any module motor missing from the state produce neutral requests. Messages
are bounded and go through the event loop recorder.

The bridge merges each subsystem's request into the one whole-robot command,
stamping it with the epoch and observed time from the last state the controller
processor reported, so a command can never claim to answer a sample nobody saw.
It never refreshes a lease on a subsystem's behalf: a request that stops
arriving neutralizes that subsystem's devices and nothing else's. Another local
process, such as drive estimation, can subscribe to the state topic. A separate
remote observer service is not part of this change.

## Run the Simulation

Build and run the automated loopback-only integration test:

```sh
bazel build //2026-robot/main_processor/drivetrain:all
python3 tools/test_drivetrain.py
```

It starts four processes, drives the ideal simulated wheels, verifies a stop
after communication ends, and replays the recorded drivetrain log. It reports
the directory containing the log and process output. Ports 5802/5803 must be
unused, and no other process should publish these local topics.

### Against the real RoboRIO program

`sim_gateway` is a stand-in for the RoboRIO end of the UDP link. To drive the
actual robot program instead, under WPILib simulation:

```sh
python3 tools/test_drivetrain.py --wpilib
```

This swaps `sim_gateway` for `./gradlew simulateNative` in `2026-robot/controller_processor/rio/` and
makes the same assertions, because both run the same
`talos::hardware::Gateway`. It also runs a second scenario the stand-in cannot:
with the Driver Station never enabled, the drivetrain must not move at all.
`sim_gateway` hardcodes enable, so it cannot tell a working safety gate from a
missing one.

A headless run has no Driver Station and no GUI to click, so for automation the
desktop build can stand in for one. It is off unless `TALOS_SIM_DS` is set:

| Variable | Effect |
|---|---|
| `TALOS_SIM_DS=1` | enable in Teleoperated at startup |
| `TALOS_SIM_DS=0` | attach a Driver Station but stay disabled |
| `TALOS_SIM_DS_DISABLE_AFTER_MS=N` | ... then disable N ms in |
| `TALOS_SIM_RUN_MS=N` | stop after N ms and print a summary line |

None of this exists in the RoboRIO build: there the enable state comes from the
real Driver Station and nothing in the program can forge it.

### Interactive

Start each command in a separate terminal:

```sh
bazel run //2026-robot/main_processor/simulation:sim_gateway    # or, in 2026-robot/controller_processor/rio/: ./gradlew simulateNative
bazel run //talOS/bridge:hardware_node
bazel run //2026-robot/main_processor/drivetrain:node -- --sim --log /tmp/drivetrain.tlog
bazel run //2026-robot/main_processor/drivetrain:send_target -- 1 0 0 2
```

`send_target` is a sim/bench tool only: it writes a drivetrain lane
(`--source NAME`, default `teleop`) directly, bypassing the operator
interface, so the drivetrain follows it only while enabled in a Driver Station
mode whose policy selects that source (Teleoperated for `teleop`). Like a
`TargetLane`, it stamps each envelope with the mode generation of the newest
`/hw/state/driver_station` packet, so it cannot drive an enable or mode it did
not see; without Driver Station packets (the sim gateway sends none) it
stamps 0 and the drivetrain stays neutral. On a robot, drive through the
operator interface.

Wait for the gateway to print `hardware gateway config=...` before starting the
bridge; that line means its socket is open.

### Watching it work

`monitor` subscribes to one target lane (`--source NAME`, default `teleop`),
the state and the command topics above and redraws
them a few times a second. It publishes nothing and holds no hardware, so it is
safe to start and stop at any time, against a simulation or a real robot:

```sh
bazel run //2026-robot/main_processor/drivetrain:monitor
```

```text
gateway   configured yes   enabled yes   commanding yes   fault  no
          epoch 2   age 0 ms   199 Hz   seq 568

target    teleop lane   vx   1.00 m/s   vy   0.00 m/s   omega   0.00 rad/s   active yes   age 0 ms

command   age 0 ms   201 Hz   seq 567

module          steer cmd   steer now     drive cmd     drive now   travelled
                      rot         rot         rot/s         rot/s         rot
front_left         0.0000      0.0000        3.1330        3.1330        4.09
front_right        0.0000      0.0000        3.1330        3.1330        4.09
back_left          0.0000      0.0000        3.1330        3.1330        4.09
back_right         0.0000      0.0000        3.1330        3.1330        4.09
```

Read it top down. `enabled` is the Driver Station; until it says yes the gateway
refuses everything and the drive columns stay `-`. `commanding` means a command
lease is live. The Hz figures come from the publishers' sequence numbers rather
than from how often the monitor polled, so they are the real link rates: a
gateway that has stopped shows a rising `age` while the rate falls. `drive cmd`
is what the node asked for and `drive now` is what the robot reports, so the two
agreeing is the end-to-end proof. A dash means neutral, which is not the same as
a commanded zero.

The number to sanity-check by hand: 1 m/s through a 0.0508 m wheel is
1 / (2π × 0.0508) = 3.133 rot/s, which is what `drive cmd` reads above.

With `./gradlew simulateNative` as the gateway, the simulation GUI opens and the
robot starts **disabled**, like any other WPILib robot. Enable it in the *Robot
State* window by selecting **Teleoperated**; the gateway refuses every command
until you do, so the wheels stay still and `command_active` stays 0. Switching
back to Disabled neutralizes them again immediately. That gate is the real
thing, not a simulation detail: it is the same check that runs on the robot.

The last command requests forward motion at 1 m/s for two seconds, then
releases its lane. The fake provides ideal actuator responses, not tire or
motor physics. Gateway, bridge, and node run until Ctrl-C; `--duration-s N`
provides bounded runs for tests.

Replay after stopping the node:

```sh
bazel run //2026-robot/main_processor/drivetrain:node -- --sim --replay /tmp/drivetrain.tlog
```

Replay does not open hardware or shared-memory transports. It verifies the
same handler outputs against the recorded network inputs and chassis targets.

## Connect to RoboRIO

Configure and build the shared hardware manifest first, following
`talOS/hardware/README.md`. Run the companion bridge against the RoboRIO's
address and start the node without `--sim`:

```sh
bazel run //talOS/bridge:hardware_node -- --remote 10.56.87.2
bazel run //2026-robot/main_processor/drivetrain:node -- --log /tmp/drivetrain.tlog
```

The companion bridge pushes the parsed hardware configuration to the gateway
over UDP (`talOS/hardware/README.md`). The supplied addresses, geometry, offsets and gains are
examples. A successful build and simulated replay do not validate physical
inversion, steering alignment, gearing, PID tuning, or CAN timing.
