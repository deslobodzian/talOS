# Simulation Gateway

Standalone hardware gateway for host-side simulation. It speaks the same UDP
frames as the RoboRIO program, so the companion bridge and subsystem nodes
run unmodified against either.

```sh
# Ideal actuators (fast logic tests); stays enabled without a Driver Station.
bazel run //2026-robot/main_processor/simulation:sim_gateway

# Physics-backed actuators (PID/gearing/direction/limits believability).
bazel run //2026-robot/main_processor/simulation:sim_gateway -- --physics

# Either mode accepts a bounded run for scripted checks (no live sim needed).
bazel run //2026-robot/main_processor/simulation:sim_gateway -- --physics --duration-s 10
```

The summary line (`active_ticks=... first_wheel_rotations=...`) keeps its
exact format in both modes: `tools/test_drivetrain.py` parses it. The backend
choice is logged to stderr (`backend=ideal|physics`).

## What is ideal vs physics-backed

See `talOS/hardware/README.md` ("Simulation Backends") for the full table.
Short version: the ideal backend teleports to demands (even with zero gains),
while the physics backend drives a reusable motor/mechanism model from
controller voltage output and writes rotor position/velocity back. Closed-loop
demands with missing gains honestly stall; neutral coasts/brakes per the
motor's brake flag instead of snapping to zero.

## Deterministic stepping

Physics has no wall-clock or randomness: `Read()` derives `dt` from
`sample_time_us` deltas, so Gateway ticks with explicit timestamps reproduce
exactly. `PhysicsSimBackend::Step(dt)` advances the models without a Gateway
for unit tests. Scheduling (`period_us` pacing) and UDP timing are endpoint
concerns, intentionally outside the model and tested separately.

## Controlled inputs and faults

Beam breaks and limit-switch inputs via `SetDigitalInput`; soft limits come
from `MotorConfig` and are enforced in physics. `FreezeFeedback`,
`SetMotorConnected`/`SetSensorConnected` (disconnect fails `Read`, which is
the gateway fault path; reconnect recovers), `SetSupplyVoltage`, and
`RestartController` (zeroes motion, keeps config) cover the rest. Pigeon yaw
integrates `SetChassisYawRate`; full swerve-chassis coupling is future work
(roller/flywheel mechanisms first).
