# Velocity-controlled flywheel — commissioning & diagnostics

Closed-loop velocity example: ramp to cruise, prove at-speed, then shoot.
Do the roller example first (wiring and limits), this one second (closing
a velocity loop), the position example last (gearing + position servo).

## Configuration

`subsystem.toml` in this directory (mirrored by `CONFIG_TOML` in
`flywheel_example.py`, loaded in tests via
`control_config.load_motor_config`). Key rows: `can_id = 9`,
`brake = false` (flywheels coast), direct-drive ratio `1.0`,
`cruise_velocity_rps = 80.0`, `acceleration_rps2 = 160.0`, and a tuned
`slot0` (`p = 0.1`, `v = 0.12`).

## Simulation path

```python
from flywheel_example import run_sim, diagnose
result = run_sim()                 # spin to 80 rps, 3 s timeout
print(diagnose(result)["lines"])
```

`result["hold_rps"]` is the cancellation mapping in code: cruise on
completion, stop on timeout. `run_sim(timeout_s=0.5, tuned=False)` shows
the timeout path.

Owner test run:

```sh
bazel test //2026-robot/main_processor/example_flywheel:flywheel_example_test
```

## Diagnostics

`diagnose` reports spin-up seconds, final error, and at-speed vs
`TIMED OUT`. A healthy run reads `spin-up ~0.6 s, final error < 2 rps,
at speed`. `TIMED OUT` means gains or current limits — never raise the
timeout to fix it. `wheel nearly dead` means the loop is untuned (all
gains zero).

## Commissioning (on the rig, in order)

1. Confirm `brake = false`: the wheel must coast down after a spin.
2. Tune `v` (kV) until the wheel holds cruise under load, then add a
   touch of `p`; back `p` off at the first oscillation (overshoot here
   means missed shots).
3. Pick `tolerance_rps` from shot data (2 rps is the starting point), not
   from the sim — the sim plant is kinder than air drag.
4. If spin-up times out, check gains and `supply_limit_a`/`stator_limit_a`
   before touching the timeout.
