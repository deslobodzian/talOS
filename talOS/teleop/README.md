# Declarative teleop bindings

The framework teleop node maps joystick input to subsystem target lanes without
mechanism-specific code. Bindings are evaluated in configuration order; the
first live binding assigning a field wins. Unassigned fields are zeroed, and
when no binding is live the lane is released.

```toml
[[teleop.bindings]]
button = 1
while_held = { velocity_rps = 60.0 }

[[teleop.bindings]]
pov = 90
pov_index = 0
on_toggle = { velocity_rps = 30.0 }

[[teleop.bindings]]
axis = 1
field = "velocity_rps"
scale = -60.0
deadband_frac = 0.1
```

These tables belong in a subsystem file (or use
`[[subsystems.<name>.teleop.bindings]]` in a single-file robot configuration).
Each binding specifies exactly one trigger: `button`, `axis`, or `pov`.
`port` defaults to 0 and selects joystick ports 0–5. Buttons are numbered 1–32;
axes are numbered 0–11. A POV direction must be an integer compass angle:
0, 45, 90, 135, 180, 225, 270, or 315 degrees. `pov_index` defaults to 0 and
selects POVs 0–3; it is valid only for a POV trigger.

Button and POV triggers require exactly one non-empty scalar assignment table:
`while_held` or `on_toggle`. Values are checked against the target's reflected
schema, including numeric types, booleans and enum labels; nested struct fields
use slash-separated paths. Axis triggers instead require `field` and `scale`,
with optional `deadband_frac` in [0, 1), and reject action tables and `pov_index`.

`while_held` contributes assignments only while the button is pressed or the
selected POV matches its direction. `on_toggle` flips a latch on each rising
edge: holding the trigger does not retrigger it, releasing preserves the latch,
and the next press switches it off. A latched binding participates in the same
first-live-binding-wins field priority as a held binding.

All toggle latches and edge history reset when disabled, outside teleop, or when
the Driver Station or the binding's joystick disconnects. Re-enabling cannot
resume an earlier latch. Only enabled teleop with a connected joystick can
publish active teleop targets; a stale Driver Station also releases them.
