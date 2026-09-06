# Drivetrain Node

Four-module, robot-relative swerve control with separate processes:

```text
ChassisTarget publisher -> drivetrain node -> hardware bridge -> RoboRIO
                              ^                    ^               |
                              |                    |               |
                              +---- hardware state +----- UDP -----+
```

The drivetrain converts chassis velocity to module velocity and steering
position, desaturates wheel speeds, and selects the shorter steering path.
At zero requested speed it holds the measured steering angle. Drive requests
use TalonFX velocity control; steering uses position control with continuous
wrapping. The hardware API also supports Motion Magic when a subsystem needs
profiled positions. This is not yet a trajectory follower or field-relative
controller; pose estimation and vision are separate future nodes.

## Local Topics

| Topic | Single publisher | Payload |
|---|---|---|
| `/drivetrain/tgt` | Planner/operator process | FlatBuffer `ChassisTarget` |
| `/hw/state` | Bridge | Canonical hardware state in fixed `Packet` |
| `/hw/cmd` | Drivetrain | Canonical hardware command in fixed `Packet` |

Chassis units are meters/second and radians/second. Axes are forward X, left Y,
CCW yaw. Targets carry the companion `Poller::now()` timestamp and an enable
flag. Stale targets or stale/invalid/disabled hardware state produce neutral
requests. Messages are bounded and go through the event loop recorder.

The bridge carries packet bytes without changing command timestamps or session
tokens. It never refreshes an old command's lease. Another local process, such
as drive estimation, can subscribe to the state topic. A separate remote
observer service is not part of this change.

## Run the Simulation

Build and run the automated loopback-only integration test:

```sh
bazel build //talOS/drivetrain:all
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

This swaps `sim_gateway` for `./gradlew simulateNativeRelease` in `robot/` and
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
bazel run //talOS/drivetrain:sim_gateway    # or, in robot/: ./gradlew simulateNative
bazel run //talOS/drivetrain:bridge
bazel run //talOS/drivetrain:node -- --sim --log /tmp/drivetrain.tlog
bazel run //talOS/drivetrain:send_target -- 1 0 0 2
```

Wait for the gateway to print `hardware gateway config=...` before starting the
bridge; that line means its socket is open.

### Watching it work

`monitor` subscribes to the same three topics and redraws them a few times a
second. It publishes nothing and holds no hardware, so it is safe to start and
stop at any time, against a simulation or a real robot:

```sh
bazel run //talOS/drivetrain:monitor
```

```text
gateway   configured yes   enabled yes   commanding yes   fault  no
          epoch 2   age 0 ms   199 Hz   seq 568

target    vx   1.00 m/s   vy   0.00 m/s   omega   0.00 rad/s   enabled yes   age 0 ms

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
disables its target. The fake provides ideal actuator responses, not tire or
motor physics. Gateway, bridge, and node run until Ctrl-C; `--duration-s N`
provides bounded runs for tests.

Replay after stopping the node:

```sh
bazel run //talOS/drivetrain:node -- --sim --replay /tmp/drivetrain.tlog
```

Replay does not open hardware or shared-memory transports. It verifies the
same handler outputs against the recorded network inputs and chassis targets.

## Connect to RoboRIO

Configure and build the shared hardware manifest first, following
`common/hardware/README.md`. Run the companion bridge against the RoboRIO's
address and start the node without `--sim`:

```sh
bazel run //talOS/drivetrain:bridge -- --remote 10.56.87.2
bazel run //talOS/drivetrain:node -- --log /tmp/drivetrain.tlog
```

The hardware gateway is configured locally at startup; there is no network
configuration upload. The supplied addresses, geometry, offsets and gains are
examples. A successful build and simulated replay do not validate physical
inversion, steering alignment, gearing, PID tuning, or CAN timing.
