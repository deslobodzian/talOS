# Declarative teleop bindings

The framework teleop node maps joystick input to subsystem target lanes without
mechanism-specific code. Bindings are evaluated in configuration order; the
first live binding assigning a field wins. A field no live binding assigns takes
its explicit default, and when no binding is live the lane is released.

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

[teleop]
defaults = { feedforward_v = 0.0 }
```

`defaults` assigns target fields with the same value checks as `while_held`.
Construction fails, naming the lane and field, when a field could be left
unassigned and has no default. A field needs no default only when every binding
of the lane assigns it (an axis binding assigns its `field`).

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
use slash-separated paths. Axis triggers instead require a floating-point `field`
and `scale`, with optional `deadband_frac` in [0, 1), and reject action tables
and `pov_index`.

Numeric assignments must be finite and representable by their destination
fields. A nonfinite axis input or an axis output outside its destination field's
finite range releases the subsystem's entire teleop lane.

`while_held` contributes assignments only while the button is pressed or the
selected POV matches its direction. `on_toggle` flips a latch on each rising
edge: holding the trigger does not retrigger it, releasing preserves the latch,
and the next press switches it off. A latched binding participates in the same
first-live-binding-wins field priority as a held binding.

All toggle latches and edge history reset when disabled, outside teleop, or when
the Driver Station or the binding's joystick disconnects. Re-enabling cannot
resume an earlier latch. Only enabled teleop with a connected joystick can
publish active teleop targets; a stale Driver Station also releases them.

Teleop rearms before publishing again, through one `talos::OperatorArming`
for all its lanes ([Architecture: Targets](../../ARCHITECTURE.md#targets)):
at start, after the Driver Station was stale (sampled or not), disabled or
outside teleop, in every new Driver Station mode generation or gateway boot
(so after a disable shorter than one period, which no evaluation sampled),
after a bound joystick reconnects, and after the node's own evaluations stall
for at least `command_timeout_us`. While
rearming it publishes no active target until every connected bound input is
neutral (axes inside their deadband, buttons and POVs released) for one
evaluation. A bound POV must centre even if held in a different direction than
its binding. Edge history is seeded from that snapshot, so a button held through
a restart neither commands nor flips a toggle.
