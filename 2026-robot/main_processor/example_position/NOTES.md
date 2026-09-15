# Position-controlled mechanism — commissioning & diagnostics

Position-servo example: drive a 25:1 geared stage to a shelf height in
wheel rotations and hold it there. Do this example last — it assumes the
roller (wiring/limits) and flywheel (closing a loop) examples are done.

## Configuration

`subsystem.toml` in this directory (mirrored by `CONFIG_TOML` in
`position_example.py`, loaded in tests via
`control_config.load_motor_config`). Key rows: `can_id = 11`,
`brake = true` (lifts hold), `sensor_to_mechanism_ratio = 25.0`,
`max_velocity_rps = 4.0` (cruise clamp), and tuned `slot0`
(`p = 0.5`, `s = 0.2`, `v = 0.3`).

## Simulation path

```python
from position_example import run_sim, diagnose
result = run_sim(3.0)              # drive to shelf 3.0 rot, 5 s timeout
print(diagnose(result)["lines"])
```

`result["hold"]` is the cancellation mapping in code: `("position",
target)` on arrival, `("stop", 0.0)` on timeout. Short moves (e.g.
`run_sim(0.1)`) arrive without touching the cruise clamp; `tuned=False`
shows the timeout path.

Owner test run:

```sh
bazel test //2026-robot/main_processor/example_position:position_example_test
```

## Diagnostics

`diagnose` reports elapsed seconds, final error, and the hold mode.
Healthy long move: `arrived in ~1.3 s`, `cruise clamp` line present.
`TIMED OUT` means gains or current limits — never raise the timeout to
fix it. `tolerance edge` means the position gain stops just short.

## Commissioning (on the rig, in order)

1. Verify the gearing FIRST: command 1 mechanism rotation and confirm
   the wheel turns exactly once. A wrong ratio multiplies every later
   mistake by 25.
2. Confirm `brake = true`: the stage must hold position at stop.
3. Set soft limits before tuning gains: clamp demands to the shelf range
   so a sign error cannot drive through the hard stop.
4. Tune `s` until the stage just breaks loose, then `v`, then `p`;
   gravity `g` only if the stage sags holding position.
