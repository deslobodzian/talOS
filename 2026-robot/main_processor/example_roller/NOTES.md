# Voltage-controlled roller — commissioning & diagnostics

Bring-up example: volts in, spin out, no PID. Do this example first; the
flywheel and position examples assume the wiring, direction, and limits
below are already settled.

## Configuration

`subsystem.toml` in this directory (mirrored by `CONFIG_TOML` in
`roller_example.py`, which the tests load through
`control_config.load_motor_config`). Key rows: `can_id = 10`,
`brake = true`, direct-drive ratio `1.0`, `max_voltage = 12.0`, and an
all-zero `slot0` — untuned **by design** for open loop.

## Simulation path

```python
from roller_example import run_sim, diagnose
result = run_sim(6.0)          # hold 6 V for 1 s against the sim plant
print(diagnose(result)["lines"])
```

Expect ~50 rps at 6 V (`volts / 12 * 100`). `run_sim(0.0)` holds rest;
`run_sim(13.0)` raises `ValueError` before any motion — the same
boundary that protects the rig.

Owner test run:

```sh
bazel test //2026-robot/main_processor/example_roller:roller_example_test
```

## Diagnostics

`diagnose` reports asked volts, expected rps, and measured peak. If the
wheel does not follow the volts it names wiring, current limits, and
`inverted` — never gains (open loop has none). A healthy 6 V push reads:

- `6.0 V asked ~50.0 rps, measured peak ~50 rps`

## Commissioning (on the rig, in order)

1. With the wheel off the ground, push 2–3 V and confirm it spins the
   intended way; flip `inverted` if positive volts run backwards.
2. Confirm `brake = true` holds the wheel at 0 V; coast lets it spin down.
3. Watch supply current against `supply_limit_a`/`stator_limit_a` under
   load; lower the limits first if a stall browns out the bus.
4. Step 0 → 6 → 12 V and check measured ~= `volts / 12 * max_rps`; a big
   split is friction or a current limit, not a gain to tune.
5. Only then move to `../example_flywheel` (close the velocity loop).
