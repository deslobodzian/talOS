# Simulator

`//talOS/simulation:sim_gateway` is a standalone hardware gateway for
host-side simulation: it stands in for the RoboRIO for any robot config. It
builds its devices only from the config's hardware and speaks the same UDP
frames as the RoboRIO program, so the bridge and the subsystem nodes run
unmodified against either. A robot names it in `[robot] sim_gateway`
(`@talos//talOS/simulation:sim_gateway` from another repository), and the
launcher starts it under `--sim --start-sim-gateway`
([launcher](../launcher/README.md#lightweight-simulation-gateway)).

```sh
# Ideal actuators (fast logic tests); defaults to enabled teleop, blue alliance.
bazel run //talOS/simulation:sim_gateway -- \
  --config "$PWD/bench/main_processor/configuration/robot.toml"

# Physics-backed ground truth with optional odometry error, bounded to ten seconds.
bazel run //talOS/simulation:sim_gateway -- \
  --config "$PWD/bench/main_processor/configuration/robot.toml" \
  --physics --start-pose 2,2,0 --wheel-radius-error 0.1 \
  --wheel-slip-gain 0.2 --gyro-drift-radps 0.01 --duration-s 10

# Script a disabled / autonomous / teleop cycle on the red alliance.
bazel run //talOS/simulation:sim_gateway -- \
  --config "$PWD/bench/main_processor/configuration/robot.toml" \
  --mode-schedule disabled:0.2,auto:0.5,teleop:0.5,disabled:0.2 \
  --alliance red --duration-s 1.5
```

The gateway prints `active_ticks`, `first_wheel_rotations`,
`first_wheel_rps` and `command_active` on shutdown. The backend choice is
logged to stderr (`backend=ideal|physics`).

`--mode-schedule` accepts comma-separated `mode:seconds` phases with positive,
finite fractional durations totaling at most 1e9 s. Modes are `disabled`, `auto` (or `autonomous`),
`teleop`, and `test`. After the schedule finishes, the final mode is held until
stopped or `--duration-s` expires (zero or omitted means unbounded).
`--alliance blue|red` defaults to blue. Driver Station packets carry the
scheduled mode and a new `mode_since_us` generation at each mode transition;
the gateway's enable gate follows that same mode. The bridge publishes these
packets on `/hw/state/driver_station`, just as it does for the RoboRIO.
Packets are emitted every 20000 µs (50 Hz), and at once on the tick a new mode
starts, matching the RoboRIO, even when physics and hardware snapshots run at
200 Hz.
The gateway listens on loopback UDP `[robot] controller_port` of its
`--config` and sends to `[robot] local_port`, the ports a bridge on the same
config uses; `--port N` and `--remote-port N` override them. The launcher
passes both flags from the ports it gives the bridge (the config's, or its
own `--remote-port` and `--local-port`), so neither a launched nor a
hand-run simulator needs a port argument written by hand, unless the launch
overrides the config's ports.
The schedules and `--duration-s` count from one time zero on the
main-processor node clock (`talos::event::Poller::now()`: `CLOCK_MONOTONIC` on
Linux, `CLOCK_MONOTONIC_RAW` on macOS, which counts host sleep). By default it
is the gateway's loop start. `--schedule-start-file PATH` instead takes it from
`PATH`, which the launcher or a harness writes (atomically, by rename) with
the time in decimal nanoseconds, so it can start the match once the stack is
up and time its own actions from the same instant. Under `--sim
--start-sim-gateway`, the launcher passes
`--schedule-start-file <session_dir>/sim_schedule_start` automatically (unless
`--sim-gateway-arg` already supplies `--schedule-start-file`) and writes it
once every non-observer member's loop has started
([launcher](../launcher/README.md#lightweight-simulation-gateway)). Until the
file exists and its time arrives, the gateway holds the schedules' time-zero
state (first mode phase, time-0 joystick and fault events). Content that is not a non-negative integer
stops the gateway with an error naming the file. A start already past (logged
to stderr) joins the schedule partway, as does a restarted gateway: fault
intervals that already ended are skipped, not replayed.

## Scripted joystick and fault schedules

Both schedules use seconds since the schedule's time zero (above) and run
alongside `--mode-schedule`. Quote the argument so the shell does not
interpret `|` or `;`.

`--joystick-schedule 'time:axes=A0,A1,...;buttons=MASK;pov=DEG|...'`
sets complete, connected joystick-0 snapshots. Times must be in `[0, 1e9]` s
and strictly increasing at microsecond resolution. Each snapshot is held until the next event; before the
first event the joystick is disconnected. All three fields are required.
Axes are one to twelve finite values in `[-1,1]`; omitted axes are zero.
Buttons are a decimal uint32 mask (bit 0 is button 1). POV is `-1` (released)
or an integer in `[0,359]` degrees. Other joysticks remain disconnected.
Both schedules are parsed by `talOS/driver_station/schedule.h`, which the
headless WPILib simulation's stand-in Driver Station shares.

`--fault-schedule 'start:end:kind:target[:volts]|...'` requires `--physics`.
Intervals are half-open `[start,end)`, with finite `0 <= start < end`.
Device targets use logical TOML device names, not physical CAN IDs.

| Kind | Target | Effect |
|---|---|---|
| `motor_disconnect` | Motor name | Invalid feedback, its ages growing, until the interval ends ([as the RoboRIO reports it](../hardware/README.md#simulation-backends-ideal-vs-physics-backed)) |
| `sensor_disconnect` | Sensor name | Invalid feedback, its ages growing, until the interval ends |
| `freeze` | Motor name | Hold feedback, its age growing, until the interval ends; the gateway marks it invalid once it is `command_timeout_us` old |
| `supply_voltage` | `all` | Require fifth field, volts in `[0,16]`; clear the override (`ClearSupplyVoltageOverride()`) at end so `[simulation.battery]` sag (or the resting 12.8 V default) resumes |
| `reset` | Motor or sensor name | One device-reset edge on the first tick at/after start |
| `udp_blackout` | `bridge` | Drop **all outbound UDP**, including DS, hardware state and config ACKs |
| `forward_limit` | Motor name | Press the switch on its TalonFX forward limit input until the interval ends |
| `reverse_limit` | Motor name | Press the switch on its TalonFX reverse limit input until the interval ends |

Unknown devices and overlapping intervals of the same kind/target are rejected.
Blackouts do not stop incoming commands, physics or gateway safety ticks;
suppressed packets are discarded, never replayed after recovery.
A pressed switch that the motor's config enables holds output toward it
neutral and reports the asserted limit (`Motor::forward_limit()` /
`reverse_limit()`); a disabled one does nothing. Hold a switch pressed for the
whole run to rehearse a normally closed switch with a cut wire.
The bridge's `/hw/status.motor_fault_mask` uses configured motor order and
marks invalid feedback as well as controller fault flags other than an
asserted soft or hard limit or an active current limit
(`talos::hardware::MotorFaulted`);
`motor_forward_limit_mask` and `motor_reverse_limit_mask`, in the same order,
mark the asserted switches.

```sh
bazel run //talOS/simulation:sim_gateway -- \
  --config "$PWD/bench/main_processor/configuration/robot_physics_sim.toml" \
  --physics --mode-schedule disabled:0.5,teleop:5 --duration-s 6 \
  --joystick-schedule '0:axes=0,0,0;buttons=0;pov=-1|1:axes=0,-0.6,0;buttons=0;pov=-1|3:axes=0,0,0;buttons=0;pov=-1|3.3:axes=0,-0.6,0;buttons=0;pov=-1' \
  --fault-schedule '2:2.5:udp_blackout:bridge|4:5:motor_disconnect:drive'
```

`./sim.sh --sim-gateway` selects the physics backend. Its repeatable
`--gateway-arg ARG` passes one argument verbatim through the launcher's
repeatable `--sim-gateway-arg ARG` interface, preserving order:

```sh
./sim.sh --sim-gateway --gateway-arg --start-pose --gateway-arg 3,2,0.5 \
  --gateway-arg --wheel-radius-error --gateway-arg 0.1
```

`--gateway-arg` requires `--sim-gateway`; the launcher forwarding flag requires
`--start-sim-gateway`. The launcher forwards these arguments to both the
gateway's `--describe` probe and runtime, so truth is declared exactly when
`--physics` is selected. The running gateway publishes its live registry row
with the session ID.

## True world and injected odometry error

Only `--physics` publishes `/sim/state` (`geometry.OdometrySample`), every
physics tick (200 Hz in the robot config). Its timestamp uses the
main-processor node clock; pose uses the blue-alliance field frame (x
downfield, y left, yaw radians CCW+). Velocity is **field-frame** m/s and
rad/s. Cameras render from this physical pose, not drivetrain odometry.
The estimator retains its own independent initial pose; a seed mismatch is
corrected by vision like on a real robot.

Physics-only flags (all values must be finite; supplying any without
`--physics` is a usage error):

| Flag | Default | Meaning |
|---|---|---|
| `--start-pose X_M,Y_M,YAW_RAD` | `2,2,0` | Physical field start pose, independent of estimator parameters |
| `--wheel-radius-error FRAC` | `0` | True radius `wheel_radius_m × (1 + FRAC)` m (`wheel_radius_m` defaults to `0.0508`); strictly `-0.5 < FRAC < 0.5` |
| `--wheel-slip-gain G` | `0` | Nonnegative acceleration-dependent traction loss |
| `--gyro-drift-radps R` | `0` | Reported yaw bias grows by `R × elapsed_seconds`, yaw rate gains `R` |

Nominal wheel radius, swerve module coordinates, mechanical end-of-travel hard
stops, battery sag, chassis mass/traction dynamics, and per-mechanism inertia
come from the optional top-level `[simulation]` table (`wheel_radius_m`,
`[[simulation.swerve_modules]]`, `[[simulation.hard_stops]]`,
`[simulation.battery]`, `[simulation.chassis]`, and
`[[simulation.mechanisms]]`), falling back to `0.0508` m, ±0.30 m name
inference, no hard stops, a constant `12.8` V bus, kinematic chassis
integration, and the legacy first-order motor lag when omitted. Hard stops are
enforced in `--physics` even when `soft_limits = false` (as required for
rezero-frame mechanisms), stopping the rotor so back-EMF drops to zero and
stator current rises for stall homing. `[simulation]` never enters the hardware
wire format or `ConfigurationId`; keys, bounds, and model semantics live in
[hardware configuration](../hardware/README.md#configuration) and
[simulation backends](../hardware/README.md#simulation-backends-ideal-vs-physics-backed).

Encoders truthfully measure wheel rotations; radius error changes ground
travel only. Each wheel's slip fraction is
`min(1, G × abs(wheel_surface_acceleration_mps2) / 9.80665)` and ground speed is
surface speed times `1 − slip`. It is deterministic and vanishes at constant
speed. This simplified model only reduces ground travel while accelerating
**or braking**, so nominal-radius odometry over-reports travel during either.
Gyro drift never rotates physical truth. All-zero errors retain nominal
behavior.

The ideal backend and WPILib/RoboRIO simulation publish **no** `/sim/state`.
Their simulated camera nodes emit invalid observations rather than inventing
truth from odometry.

For ideal/physics backend behavior, model hooks, and choosing between
`SimBackend`, `sim_gateway --physics` (scripted schedules, `/sim/state` truth,
and fault injection — no USB gamepad), and WPILib desktop simulation (`./sim.sh`
with a live USB gamepad), see
[which simulator to use for what](../hardware/README.md#which-simulator-to-use-for-what).

## Realtime scheduling

`sim_gateway` honours `TALOS_REALTIME` (`talos::process::RealtimeRequested`;
the node development guide's realtime scheduling paragraph): it stands in for
the RoboRIO, so it takes the bridge's tier for its loop period
(`[robot] period_us`) and reports the outcome in its registry row
(`loop_tier = "bridge"` in the session manifest). A gateway left at
timeshare beside time-constraint nodes stops sending State whenever the host
stalls timeshare work, and the nodes then correctly expire their targets.

## MuJoCo gateway

`//talOS/simulation/mujoco:mujoco_gateway` is this program with a MuJoCo
world as the physics backend's plant: the robot's model from the config's
`[simulation.mujoco]` on a field model. A config names it in `[robot]
sim_gateway`; its flags, output and model rules are in
[its README](mujoco/README.md#mujoco_gateway).

## Tests

`schedules_test` covers the fault schedule's parsing, resolution, application,
and `supply_voltage` restoration of a configured `[simulation.battery]` model.
`launcher_ports_test` (integration, about 1 s) has the launcher start the
bridge and the simulator on a config's non-default ports, with no port argument
written by hand, and checks that the bridge publishes the simulator's State.
`sim_gateway_test` (integration, about 1 s) runs the simulator alone:
`--describe` and its flag checks, a run without a config, the Driver Station
packets of a mode and joystick schedule, the home switches `--physics` reads
from a config, `[simulation]` wheel radius and swerve module geometry, and
`[simulation.battery]` / `[simulation.chassis]` / `[[simulation.mechanisms]]`
end-to-end physics. A robot's full-graph tests against the simulator live with
the robot (the bench's:
[bench simulation tests](../../bench/main_processor/simulation/README.md)).
