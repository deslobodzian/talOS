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
error, and desaturates wheel speeds. A wheel asked for less than 0.25% of
its own top speed (its drive `max_velocity_rps` at its wheel radius; 0.0116
m/s on the robot) holds the measured steering angle at zero drive, so
heading-hold noise does not re-steer the modules. That is a dead band for any
pose controller: autonomous's feedback at its `yaw_tolerance_rad` and
`position_tolerance_m` must exceed it, which
`configuration/config_golden_test.cc` checks. Drive requests
use TalonFX velocity control; steering uses position control with continuous
wrapping. Pose estimation and vision run as separate nodes.

Optional `[subsystem]` parameters `max_linear_accel_mps2` and
`max_angular_accel_radps2` limit commanded chassis acceleration before
discretization. Translation uses a vector-magnitude limit in the field frame,
preserving the direction of the velocity change, so a constant field velocity
held while rotating is reached rather than chased in the turning robot frame;
the previous command is rotated by the heading change since the last tick
(robot-frame limiting while the heading is invalid on either tick). Rotation is
limited independently. Each must be finite and nonnegative and defaults to
`0.0`, which leaves that limit disabled; any other value fails construction
naming the key.
The period is the configured node update period, not elapsed wall time.
Disabled, stale/released or non-finite targets and fewer than two live modules
neutralize immediately and reset both ramps to zero. Re-enabling starts a new
ramp rather than resuming the previous command.

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
  this subsystem's devices at startup. A steer motor must have
  `continuous_wrap = true`, `feedback = "RemoteCANcoder"` with
  `feedback_sensor` naming its own module's encoder,
  `position_frame = "absolute"` and `sensor_to_mechanism_ratio = 1` (the
  encoder's azimuth then survives a reset, so steering needs no rezero; see
  [position frames](../../../docs/NODE_DEVELOPMENT_GUIDE.md#rezero-and-position-frames));
  anything else is refused.
- `drive_message.fbs` and the `topics` in `BUILD`: `drivetrain::ChassisTarget`,
  `drivetrain::DrivetrainState` and `drivetrain::topics::k*`,
  `//2026-robot/main_processor/drivetrain:messages`.

`/drivetrain/state` (`DrivetrainState`) is published every tick, whatever
the hardware says:

- A module is live only when its drive motor, steer motor and CANcoder report.
  A missing device neutralizes both motors on that module. Two or more live
  modules can translate and rotate using their own inverse-kinematics setpoints
  and common desaturation; measured chassis speeds use a least-squares solve
  over only those live modules (precomputed fixed-size subset factorizations).
  Coincident or ill-conditioned subsets are not driven or solved. With fewer
  than two modules every motor is neutral. This minimum assumes a failed module
  can passively roll; a mechanically jammed wheel still requires
  the driver to stop. Hardware health identifies missing devices.
- `modules_valid` requires all four complete modules and usable acquisition
  times for each drive position, drive velocity and steer position.
  Partial odometry is deliberately not offered to four-module consumers;
  missing module positions hold their last values while live ones update.
  Wheel distance and Pigeon yaw stay continuous. A sample's step from the
  last published value is motion, including motion made while feedback was
  invalid, unless the device's count may have restarted: a
  `gateway_device_reset_count` change on `/hw/status` since the previous
  sample, more than 100 ms of acquisition time since it (a reset can reach
  `/hw/status` after the device's first new sample), or a step faster than
  four times the drive's `max_velocity_rps` (wheels) or 25 rad/s (yaw) over
  that time plus one control period. Such a sample re-anchors instead of
  moving the value. A wheel sample without an acquisition time (before the
  clock mapping converges) counts tick time.
- `yaw_rad` is the Pigeon's yaw converted from rotations, and `yaw_valid`
  requires usable yaw acquisition timing (including the skew check below).
  Otherwise `yaw_rad` holds the last valid heading (0 before the first).
  Under `--sim` the default instead
  integrates heading from module rotation, valid while the modules are,
  because WPILib/ideal sims report a valid, permanently zero Pigeon yaw.
  `integrate_heading = true|false` in `[subsystem]` overrides that choice.
  The physics gateway models the Pigeon, including injected gyro drift:
  `configuration/robot_physics_sim.toml` includes `robot.toml` and sets
  `[subsystems.drivetrain] integrate_heading = false` to use that modeled yaw.
  `./sim.sh --sim-gateway` selects this config; WPILib keeps `robot.toml`
  and module-integrated heading.
- `sample_time_ns` is the oldest relevant **signal acquisition** time, not
  packet assembly or tick time: `Motor::position_time()` and
  `Motor::velocity_time()` for drive motors, `Motor::position_time()` for
  steer motors, and `Gyro::yaw_time()` when using the Pigeon. These APIs
  subtract the signal's reported age from `HardwareLink::sample_time()`,
  mapped into the node clock by the bridge's min-filtered offset
  (`talOS/hardware/README.md`). Unused steer velocity, CANcoder measurements
  and gyro rate do not affect timing validity.
  Each consumed time must be known, strictly positive, no later than the
  tick, and at most **50 ms old**. Module acquisition times must span at most
  **20 ms**. These fixed bounds permit normal asynchronous status updates
  without claiming a coherent solution from arbitrarily old/skewed values;
  no extrapolation is performed.
  A bad module time clears `modules_valid` and module-integrated `yaw_valid`,
  but an independently usable Pigeon yaw can remain valid. A bad yaw time
  clears only `yaw_valid`; if otherwise usable modules and Pigeon yaw together
  span more than 20 ms, modules take priority and yaw is rejected.
  The timestamp is the oldest time contributing to either valid product
  (modules or yaw), or 0 when neither is valid. No negative time is cast to
  the unsigned message timestamp. Before clock convergence or with stale
  hardware, neither product is valid.
  Timing rejection does **not** alter connectivity-based degraded driving,
  target freshness or enable gating. Any signal of a motor or the Pigeon
  (used or not) `command_timeout_us` old or older, or of unknown age, is not
  a timing rejection: the gateway marks that device invalid, so it counts as
  disconnected (a motor's module goes neutral). Rejected module times stop heading
  integration; ideal simulation still accumulates an untrusted heading
  during initial clock convergence.

## Local Topics

Topic and target-lane conventions are defined in
[NAMING](../../../talOS/NAMING.md) and
[ARCHITECTURE](../../../ARCHITECTURE.md). This node consumes
`ChassisTarget` targets and publishes `/drivetrain/state`.
Chassis units are meters/second and radians/second; axes are forward X,
left Y, CCW yaw.

See [DATA_FLOW](../../../docs/DATA_FLOW.md) for the hardware bridge and
transport contracts.

## Run the Simulation

Build and run the automated loopback-only integration test:

```sh
bazel build //2026-robot/main_processor/drivetrain:all \
  //2026-robot/main_processor/simulation:sim_gateway //talOS/bridge:hardware_node
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
`talos::hardware::Gateway`. The harness also runs a second WPILib scenario:
with the Driver Station never enabled, the drivetrain must not move at all.
It runs Gradle `--offline`, so the first run needs one online
`./gradlew simulateNative` beforehand. CI's
`rio-in-the-loop` job runs it on Linux with `--output-dir`, which puts each
scenario's log directory under a directory it keeps when the run fails.
Both modes fail a run during which the host slept (more than 50 ms of
sleep), since a frozen host reads as a stalled robot; on macOS the harness
holds `caffeinate -s` for the run, which also holds off a headless Mac's
DarkWake maintenance sleep. `--wpilib` also fails when the RoboRIO loop's
watchdog reports a stall or its timing report shows more than 250 ms between
ticks, naming whether the tick overran or only started late.
The standalone gateway also supports disabled operation through
`--mode-schedule`; see [simulation](../simulation/README.md).


### Interactive

Start each command in a separate terminal:

```sh
bazel run //2026-robot/main_processor/simulation:sim_gateway -- --config "$PWD/2026-robot/main_processor/configuration/robot.toml"
bazel run //talOS/bridge:hardware_node -- --config "$PWD/2026-robot/main_processor/configuration/robot.toml" --sim
bazel run //2026-robot/main_processor/drivetrain:node -- --config "$PWD/2026-robot/main_processor/configuration/robot.toml" --sim --log /tmp/drivetrain.tlog
bazel run //2026-robot/main_processor/drivetrain:send_target -- 1 0 0 2
```

`send_target` is a sim/bench tool only: it writes a drivetrain lane
(`--source NAME`, default `teleop`) directly. Its envelopes use the mode
generation from the newest `/hw/state/driver_station` packet; without those
packets it stamps 0 and the drivetrain stays neutral. The standalone gateway
publishes Driver Station packets. On a robot, drive through the operator
interface.

### Watching it work

`monitor` subscribes to one target lane (`--source NAME`, default `teleop`),
the state and the command topics above and redraws
them a few times a second. It publishes nothing and holds no hardware, so it is
safe to start and stop at any time, against a simulation or a real robot:

```sh
bazel run //2026-robot/main_processor/drivetrain:monitor
```


With `./gradlew simulateNative` as the gateway, the simulation GUI opens and the
robot starts **disabled**, like any other WPILib robot. Enable it in the *Robot
State* window by selecting **Teleoperated**; the gateway refuses every command
until you do, so the wheels stay still and `command_active` stays 0. Switching
back to Disabled neutralizes them again immediately. That gate is the real
thing, not a simulation detail: it is the same check that runs on the robot.

The last command requests forward motion at 1 m/s for two seconds, then
releases its lane. The default gateway uses ideal actuator responses;
`--physics` selects motor/mechanism physics. Gateway, bridge, and node run
until Ctrl-C; `--duration-s N` provides bounded runs.

Replay after stopping the node:

```sh
bazel run //2026-robot/main_processor/drivetrain:node -- --config "$PWD/2026-robot/main_processor/configuration/robot.toml" --sim --replay /tmp/drivetrain.tlog
```

Replay does not open hardware or shared-memory transports. It verifies the
same handler outputs against the recorded network inputs and chassis targets.

## Connect to RoboRIO

Follow [hardware configuration](../../../talOS/hardware/README.md) and
the [bring-up procedure](../../../docs/BRINGUP.md).
A successful build and simulated replay do not validate physical inversion,
steering alignment, gearing, PID tuning, or CAN timing.
