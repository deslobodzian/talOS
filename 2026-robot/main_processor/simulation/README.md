# Simulation Gateway

Standalone hardware gateway for host-side simulation. It speaks the same UDP
frames as the RoboRIO program, so the companion bridge and subsystem nodes
run unmodified against either.

```sh
# Ideal actuators (fast logic tests); defaults to enabled teleop, blue alliance.
bazel run //2026-robot/main_processor/simulation:sim_gateway -- \
  --config "$PWD/2026-robot/main_processor/configuration/robot.toml"

# Physics-backed actuators, bounded to ten seconds.
bazel run //2026-robot/main_processor/simulation:sim_gateway -- \
  --config "$PWD/2026-robot/main_processor/configuration/robot.toml" \
  --physics --duration-s 10

# Script a disabled / autonomous / teleop cycle on the red alliance.
bazel run //2026-robot/main_processor/simulation:sim_gateway -- \
  --config "$PWD/2026-robot/main_processor/configuration/robot.toml" \
  --mode-schedule disabled:0.2,auto:0.5,teleop:0.5,disabled:0.2 \
  --alliance red --duration-s 1.5
```

The summary line (`active_ticks=... first_wheel_rotations=...`) keeps its
exact format in both modes: `tools/test_drivetrain.py` parses it. The backend
choice is logged to stderr (`backend=ideal|physics`).

`--mode-schedule` accepts comma-separated `mode:seconds` phases with positive,
finite fractional durations. Modes are `disabled`, `auto` (or `autonomous`),
`teleop`, and `test`. After the schedule finishes, the final mode is held until
stopped or `--duration-s` expires (zero or omitted means unbounded).
`--alliance blue|red` defaults to blue. Driver Station packets carry the
scheduled mode and a new `mode_since_us` generation at each mode transition;
the gateway's enable gate follows that same mode. The bridge publishes these
packets on `/hw/state/driver_station`, just as it does for the RoboRIO.

## What is ideal vs physics-backed

See `talOS/hardware/README.md` ("Simulation Backends") for the full table.
Short version: the ideal backend teleports to demands (even with zero gains),
while the physics backend drives a reusable motor/mechanism model from
controller voltage output and writes rotor position/velocity back. Closed-loop
demands with missing gains honestly stall; neutral coasts/brakes per the
motor's brake flag instead of snapping to zero.

MotionMagic uses trapezoidal position/velocity setpoints when both cruise
velocity and acceleration are positive, including velocity/acceleration
feedforward and velocity-error damping. Without those limits it uses the
ordinary position loop.

## Deterministic stepping

Physics has no wall-clock or randomness: `Read()` derives `dt` from
`sample_time_us` deltas, so Gateway ticks with explicit timestamps reproduce
exactly. `PhysicsSimBackend::Step(dt)` advances the models without a Gateway
for unit tests. Scheduling (`period_us` pacing) and UDP timing are endpoint
concerns, intentionally outside the model and tested separately.

## Controlled inputs and faults

Beam breaks and limit-switch inputs via `SetDigitalInput`; soft limits come
from `MotorConfig` and are enforced in physics. `FreezeFeedback`,
`SetMotorConnected`/`SetSensorConnected` (a disconnect invalidates only that
sample; `Read` fails only when all configured motors and sensors disconnect),
`SetSupplyVoltage`, and `RestartController` (zeroes motion, keeps config)
cover the rest. Pigeon yaw derives from the tangential motion of named
`*_drive`/`*_steer` pairs, with `front`/`back` (`rear`) and `left`/`right`
locating modules at ±0.30 m and a 0.0508 m wheel radius. Tests with other
names or dimensions can register pairs using `SetSwerveModuleGeometry`.
`SetGyro` and `SetChassisYawRate` explicitly override automatic coupling
until reconfiguration or controller restart.
