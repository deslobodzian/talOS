# Robot models for MuJoCo

`//talOS/simulation/mujoco/robots:generate_robot` writes a robot's MuJoCo
model (MJCF) for [`mujoco_gateway`](../README.md#mujoco_gateway) from a
parameter file that describes the robot by parts: a frame and bumpers, swerve
modules, an intake ramp with rollers, a hopper whose floor falls to a feeder
roller that gates a pocket, a flywheel with its hood, beam breaks and preload
sites (REQ-DEV-41). Every part is built
from primitives (boxes, cylinders, spheres) placed by the file's numbers; no
CAD or mesh is read. `generate_robot.py` is stdlib Python 3.11.

```sh
bazel run //talOS/simulation/mujoco/robots:generate_robot -- \
  --parameters <robot_params.toml> --out <robot.xml>
```

Under `bazel run`, relative paths are from the workspace root. The output
depends only on the file's contents and name: two runs give the same bytes,
and every number is written with six significant digits. `--check` writes
nothing and exits 1, printing the regenerate command and a diff, when
`--out` is not what the parameters give.

The model follows [the rules every robot model meets](../README.md#writing-a-robot-model)
(one root body, the field's collision bits, one plain `<motor>` per talOS
motor, self-contained), so one model runs on every field and on the flat
carpet. [`examples/`](examples/) holds two parameter files and their models,
starting points for a robot's own: `swerve_robot.toml` (swerve, intake,
hopper, feeder, flywheel, two beam breaks, eight preloads) and
`shooter_rig.toml` (a welded rig with a hopper, a feeder, a flywheel, two
beam breaks, eight preloads).

## Parameter file

Lengths in metres, masses in kilograms, angles in degrees, in the robot's
frame: x forward (the intake's side), y left, z up, origin on the carpet
under the frame's centre. A name (of the robot, a motor, a module or a beam
break) is what a hardware config accepts as a device name: a non-empty
string of at most 63 bytes, used unchanged in the model, where ElementTree
escapes what XML needs escaped; one holding a control character is refused,
since XML cannot carry most of them and turns a tab or newline in an
attribute into a space. The generator refuses an unknown key, a
missing required key, a value of the wrong type or outside its range, and
each rule below, naming the key (`flywheel.exit_offset_m`,
`swerve.modules[2].name`, `preload.positions_m[4]`).

| Table | Keys |
|---|---|
| `[robot]` (required) | `name`, the model's name; `root`, `"free"` (a free joint: the model owns the chassis) or `"welded"` (fixed at the start pose); `mass_kg`, the whole robot: the chassis takes what the moving parts leave, and must get more than zero |
| `[game_piece]` (required) | `radius_m`, the piece the hopper, rollers, exit and preloads are sized for |
| `[frame]` (required) | `length_m`, `width_m`; `belly_clearance_m`, the belly pan's underside above the carpet; `rail_height_m`, `rail_width_m`, the perimeter tube |
| `[bumpers]` (required) | `bottom_m`, below `game_piece.radius_m` (refused otherwise: a bumper whose bottom is at or above a piece's centre, 0.07506 m for FUEL, rides over a piece it meets instead of pushing it); `height_m`; `thickness_m` |
| `[motors.<name>]` | one per motor a part names, and each drives a part: `gear`, the config's `rotor_to_sensor_ratio` × `sensor_to_mechanism_ratio`; `rotor_inertia_kg_m2`, J_rotor |
| `[swerve]` | `wheel_radius_m`, `wheel_mass_kg`, `module_mass_kg`, and `[[swerve.modules]]`: `name`, `x_m`, `y_m` (inside the frame), `drive_motor`, `steer_motor`; at least three modules, not all on one line. Required with `root = "free"`, refused with `"welded"` |
| `[intake]` | a ramp from the carpet over the front bumper, rollers above it: `motor`; `roller_count` (1 to 8), `roller_radius_m`, `roller_mass_kg`; `roller_first_fraction`, `roller_last_fraction`, the first and last roller's place along the ramp from its lip; `ramp_angle_deg`, `lip_height_m`, `top_above_bumper_m` (the ramp, and the bridge behind it, top out this far above the bumper); `width_m`, no wider than the hopper; `compression_m`, how much a piece is squeezed between roller and ramp |
| `[hopper]` | needs `[flywheel]` and `[feeder]`: `width_m`, its width across the robot; `step_m`, with an `[intake]` (and only then): how far the floor's front edge is below the intake's bridge, at most a piece's radius; `floor_front_m`, without an intake (and only then): the floor's height at the frame's front; `floor_slope_deg`, how steeply the floor falls toward the rear, down to the feeder, which must leave it above the belly pan; `wall_top_m`, above the feeder |
| `[feeder]` | the hopper's gate, one roller across it; needs `[hopper]`: `motor`; `ratio`, required when `motor` is the intake's (the feeder is then belted to it at this ratio) and refused otherwise; `x_m`, its axle, with the hopper's floor in front of it and room behind it, inside the frame, for the pocket; `roller_radius_m`, `roller_mass_kg`; `compression_m`, how far its underside is below a piece's diameter above the floor |
| `[flywheel]` | needs `[hopper]` (its pocket is the inlet): `motor`; `radius_m`, `width_m`, `mass_kg`, `inertia_kg_m2`; `axle_x_m`, `axle_z_m`; `compression_m`, the squeeze between wheel and hood; `hood_start_deg`, where the hood starts about the axle (from +x toward +z); `exit_elevation_deg`, the launch elevation, backward over the robot's rear; `exit_offset_m`, how far beyond the hood's end along the launch axis the exit sits; `efficiency` (0 to 1] |
| `[[beam_breaks]]` | `name` (no motor's); `position_m`, `[x, y, z]`; `direction`, the beam's direction; `cutoff_m`, the beam's length |
| `[preload]` | needs `[hopper]`: `positions_m`, at most 8 `[x, y, z]` piece centres, each in the hopper in front of the feeder |

Each motor drives one part; only the feeder may share the intake's. The
generator also refuses two moving parts that overlap (wheels, rollers, the
flywheel), an exit closer than the piece's radius to a robot geom anywhere
in the first 0.3 m along the launch axis, a preload outside the hopper
(its centre a piece's radius or more in front of the feeder's roller,
behind the floor's front edge, between the side walls less a radius, below
their top), closer than the piece's radius to a robot geom, overlapping
another preload or inside the inlet, and a flywheel low enough to reach a
piece in the pocket. It does not check that a beam
reads open on the empty robot; `robot_model_test` does for the examples.

## The model

- **Names.** The root body is `chassis` (with `chassis_joint` when free). A
  motor's actuator is `<motor>` on the joint `<motor>_joint`; a swerve
  module is the bodies `<module>_module` and `<module>_wheel`; rollers are
  `intake_roller_N` and `feeder_roller`, the flywheel `shooter_flywheel`.
  Positive rotation of a motor's joint is its part's forward action (drive:
  +x on the carpet at steer 0; intake: ingest; feeder: its underside toward
  the rear; flywheel: shoot).
- **Rotor inertia.** Each actuated joint's `armature` is gear² ×
  `rotor_inertia_kg_m2`, the rotor's inertia as the plant
  [expects it](../README.md#writing-a-robot-model).
- **Belts.** A roller that shares a motor is a joint equality on the
  actuated roller's joint (`polycoef="0 ratio 0 0 0"`, `solref="0.01 1"`):
  the plant binds one actuator per motor.
- **Contact.** Every geom collides as the field's
  [robot row](../fields/rebuilt_2026/README.md#collision-contract) (4, 21).
  Each moving body is excluded from colliding with `chassis`, since
  MuJoCo's parent filter covers neither a swerve wheel (a grandchild) nor
  any body of a welded root (whose parent is the world).
  - Wheels are spheres (condim 3, friction 1.1, MuJoCo's default `solref`
    and `solimp`): a sphere touches the carpet at one point under its
    centre whatever the steer angle. `priority="1"` keeps the wheel's own
    contact on field structure, whose `solmix` would otherwise impose its
    stiff `solref`.
  - Rollers, the flywheel and the hood are friction 1.2, `solref="0.01 1"`,
    `priority="2"`, so their own contact governs a piece's.
  - Bumpers are flat boxes (friction 0.8) whose bottom is below the
    piece's equator, so a piece they meet is pushed along the carpet, not
    wedged under them.
- **Beam breaks.** Each `[[beam_breaks]]` entry is a site of its name on
  `chassis`, +z along `direction`, and a `<rangefinder>` of the same name
  with `cutoff` = `cutoff_m`. A digital input reads it with
  `sim_sensor = "<name>"` ([configuration](../../../hardware/README.md#configuration));
  geoms of `chassis` are not seen, so the beam closes on a piece or a
  moving part.
- **Preloads.** Sites `preload_0` to `preload_7`, in `positions_m` order: a
  sphere site of the piece's radius at each piece's centre, clear of the
  robot.
- **Scripted shooter.** With `[flywheel]` the model carries the
  [scripted-shooter contract](../README.md#writing-a-robot-model) for the
  motor `<motor>` (its meaning under `mujoco_gateway` is REQ-DEV-39's): a
  box site `<motor>_inlet`, the pocket behind the feeder (below); a site `<motor>_exit` whose origin is the hood's exit line
  `exit_offset_m` beyond the hood's end and whose x axis leaves backward at
  `exit_elevation_deg` (its y axis the robot's +y, the axle's line); and
  `<numeric name="<motor>_shooter" data="efficiency radius_m"/>`.

- **Hopper and feeder.** The hopper's floor runs its full width from the
  frame's front down at `floor_slope_deg` to the feeder's roller, which spans the hopper
  with its underside a piece's diameter less `compression_m` above the
  floor; a wall closes the hopper above the roller, so pieces pass only
  under it. A positive rotation moves its underside rearward. Behind it a
  flat pocket, a piece's diameter plus 2 cm deep, is closed at the back
  (one piece plus 5 cm high) and the sides, and is the inlet: centres from
  its floor to a diameter above it, across its full width, so it holds a
  row of pieces abreast. Pieces on the slope rest against the stopped
  roller: the squeeze, with the floor rising in front of it, holds them
  while the motor is neutral, and the turning roller rolls them under it
  along the floor into the pocket. In the examples (12°, 1 cm squeeze, a
  1 in roller) all eight preloads enter the pocket in under 1 s at 2.5 to
  6 V and none passes in 5 s while it is stopped, nor while the swerve
  robot drives back and forth at 6 to 9 m/s²; `robot_model_test` holds the
  first two. A feeder belted to the intake (`ratio`) turns while the intake
  collects; at ratio -1, a robot that feeds by reversing its intake, it
  pushes the waiting pieces forward into the intake and stops it
  collecting, so the swerve example's feeder has its own motor. The model
  has no path from the pocket up to the flywheel: the scripted shooter
  stands for it, launching a piece from the exit above. While the pocket
  is full and nothing launches, the turning roller stalls at its current
  limit (the rig at 4 V and at 12 V: 0.6 rps at 80 A), unharmed.
- **The hopper's front.** Without an intake, a wall. With one, the floor's
  front edge is `step_m` below the intake's bridge, and the step's face
  rises from the floor to the bridge: a piece rolling forward, braking or
  nose-down, meets a face it cannot climb (quasi-statically below
  g·tan(acos((r − h)/r)), 28 m/s² for the example's 5 cm on FUEL). One
  piece's depth plus 1 cm behind that face a fence hangs from the walls'
  top, at least a piece's radius long, to a piece's diameter plus 3 cm
  above the floor: pieces the intake
  drops in roll under it, and a piece riding on others meets it. Measured
  on the swerve example with its eight preloads and the feeder stopped: a
  stop from 2 or 4 m/s with each drive's torque capped for 8 m/s²
  (measured: mean 6.9 m/s², peaking at 9.8 m/s² over 25 ms) loses none;
  a full-reverse stop from 4.0 m/s at 12 V on every drive motor (mean
  9.6 m/s², traction-limited) can lose up to 2 of 8 over the front, onto
  the intake's ramp; tilted 15° nose down, nose up or sideways it loses
  none. Collecting 8 loose FUEL at 1 m/s it keeps all 8 at most lateral
  placements and loses the eighth over the 0.52 m wall above the feeder
  at two or three of seven, depending on the layout; with 0.70 m walls,
  which do not fit under a TRENCH, it loses one at two
  placements as well. No piece passes the gate. Of 10, 12 and 16 pieces
  dropped into it in layers, 9, 9 and 12 come to rest inside, below its
  walls.

Besides the parameters, the generator fixes some shapes: plates 0.01 m
thick, the belly pan 6 mm thick, the hood in eight segments and 0.20 m
wide, the pocket's depth and back wall and the hopper's fence above, the
feeder's roller 1 cm
shorter than the hopper is wide, the intake's side plates from 0.08 to
0.32 m, each swerve module's steering body 0.004 kg·m² about every
axis, and the chassis's inertia that of a uniform box over the bumpers'
outline from the belly to 0.30 m; `generate_robot.py` gives the rest.

What a parameter file cannot describe: arms, elevators, climbers, turrets,
an adjustable hood, an intake that deploys or stows (the ramp is always
out), drivetrains other than swerve, meshes, and any shooter but one
flywheel behind a front intake launching backward over the rear: no
forward shooter and no second flywheel. A robot built from it therefore
aims its rear at the target: to
[`talOS/aiming`](../../../aiming/README.md) its shooter is mounted at
`yaw_rad = pi`. Masses, frame, clearances and contact values are the
parameter file's and the generator's, not measurements of a robot.

On `rebuilt_2026` the swerve example stands 0.525 m tall (its hood; the
hopper's walls and the wall above its feeder, `hopper.wall_top_m`, reach
0.52 m, and `flywheel.axle_z_m` sets the hood), so it passes under a
TRENCH, whose arm is 22.25 in (0.565 m) above the carpet. Its exit, a
site at 0.613 m, is then inside the arm (0.565 to 0.667 m): a launch
while the exit is inside any geom places the piece inside that geom, and
MuJoCo throws it out (the scripted shooter does not check the exit against
the field), so this robot does not shoot under a TRENCH. It does not cross a BUMP reliably: at rest its
intake ramp's lowest edge is about 5 mm above the carpet
(`intake.lip_height_m` less the plate's thickness at
`intake.ramp_angle_deg`), below the BUMP's 0.61 in (15.5 mm) lip, so the
ramp meets the lip's face before a wheel does. At 4 V on every drive
motor it stops there from every run-up; at 12 V it usually crosses from a
run-up of about 0.2 m or more, pitching up to about 50°, and stops from
0.1 m or less (one run stopped from 0.35 m).
`intake.lip_height_m`, `frame.belly_clearance_m`, `bumpers.bottom_m`,
the wheelbase and the wheel radius decide it. A slow crossing straddles
the BUMP's apex (6.513 in at about 16° either side) with the wheels on its
slopes, which needs about 0.078 m under every geom for the example's
0.6 m wheelbase and 2 in radius wheels, while `bumpers.bottom_m` must stay below
a FUEL's radius, 0.07506 m: for this wheelbase the two conflict by a few
millimetres.

## In a robot repository

A robot keeps its parameter file and the generated model beside the config
that uses it, both checked in: a config's model is a source file the build
lists in `CONFIG_FILES`. Its simulated config names the model in
`[simulation.mujoco] robot_model`
([keys](../../../hardware/README.md#configuration)) and the gateway in
`[robot] sim_gateway`. After editing the parameters:

```sh
bazel run @talos//talOS/simulation/mujoco/robots:generate_robot -- \
  --parameters main_processor/configuration/mujoco/robot_params.toml \
  --out main_processor/configuration/mujoco/robot.xml
```

The config's package holds a sync test, which fails with that command when
the two differ:

```starlark
load("@talos//talOS/build:robot_model.bzl", "robot_model_sync_test")

robot_model_sync_test(
    name = "robot_model_sync_test",
    model = "mujoco/robot.xml",
    parameters = "mujoco/robot_params.toml",
)
```

Both files are source files of the calling package. The macro is
`talOS/build/robot_model.bzl`, beside the other macros a robot repository
loads; the command it prints names talOS as the robot repository does
(`@talos//...`), or by its canonical name (`@@...//`) when the test's
runfiles carry no mapping for it. To check a model, run the gateway on the
config for 50 ms and read the binding table
([how](../README.md#mujoco_gateway)).

## Tests

`generate_robot_test` refuses a table of bad parameter files, each by the
key it names; accepts names a hardware config accepts; reads each
example's scripted-shooter values and exit elevation from its XML against
its TOML; holds the output to the same bytes under two hash seeds; holds
`--check` to 0 on a current model and 1 after a parameter changes; and
reports `--out` in a missing directory in one line.
`swerve_robot_sync_test` and `shooter_rig_sync_test` are the sync macro on
the examples. `robot_model_test` assembles each example model on the flat
carpet and the 2026 field, binds every motor its parameter file names,
none light, and attaches its scripted shooter (`MakeScriptedShooters`);
checks with MuJoCo's own distances that a piece launched from the exit
clears the robot for 0.3 m and that each preload site clears it; closes
each beam break's digital input with a sphere at the middle of the beam
its parameter file gives; rests each on the flat carpet for 2 s with no
drive: the chassis still (below 1e-3 m/s and rad/s), every mechanism joint
below 5e-3 rad/s, no MuJoCo warning, every beam break open, and nothing of
the robot touching anything but its wheels on `carpet_robot`; with a FUEL
at each preload site of either example and the real scripted shooter
(REQ-DEV-39), its flywheel at 4.3 V, launches nothing in 5 s with the
feeder stopped, then all eight within 3 s with the feeder at 2.5, 4 and
6 V, each along the exit's axis and touching no robot geom before the
carpet; stops the swerve example from 4 m/s with each drive's torque
capped for 8 m/s², and tilts it 15° nose down, with all eight preloads still in its
hopper; and drives it under the 2026 field's
TRENCH at 6 V, its back bumper beyond the arm within 3 s, touching no
TRENCH geom.
