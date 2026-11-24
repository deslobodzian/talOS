# MuJoCo

The [MuJoCo](https://mujoco.org) physics engine as a Bazel dependency
(`@mujoco//:mujoco`, 3.17.0 from source with a root-only patch
(`third_party/mujoco_mesh_transformed_init.patch`,
[INSTALL](../../../docs/INSTALL.md)), C engine `-O2` in every build by
`talos.bazelrc`; [why](../../../docs/METHODOLOGY.md#13-open-source-first)).
Only this package and its subpackages may depend on it, which
`tools/check_layers.py --source-only` enforces, so no node and not the RoboRIO
program links it.

`:determinism_test` pins what a simulator stepping MuJoCo relies on:

- two compiles of one model, stepped from one state, end in the same
  `mjSTATE_INTEGRATION` state to the last bit, sleeping off and on;
- a bit-exact restart must carry `mjSTATE_INTEGRATION` (`mj_getState`, then
  `mj_setState` into a fresh `mjData`): it includes the solver's warm start
  (`qacc_warmstart`), without which, under CG with 20 iterations, the
  restarted run diverges. The restart is pinned with sleeping off;
  `mjSTATE_INTEGRATION` does not carry which trees are asleep.

Bit identity holds per host and build: the same binary, thread and
floating-point environment. It is not claimed across hosts: arm64 compilers
contract `a*b+c` into a fused multiply-add, x86-64 ones do not without
`-mfma`, so a Mac and a Linux machine can produce different bits.

[`fields/rebuilt_2026`](fields/rebuilt_2026/README.md) is the 2026 field as
a generated MuJoCo model, and as a glTF binary for Studio's 3D viewport.

## `mujoco_gateway`

`//talOS/simulation/mujoco:mujoco_gateway` stands in for the RoboRIO as
`sim_gateway --physics` does ([simulator](../README.md)), with a MuJoCo
world as the plant (REQ-DEV-37): the robot's MJCF model on a field model,
attached to `sim::PhysicsSimBackend` as its
[external plant](../../hardware/README.md#an-external-plant). The backend
stays the controller and electrical model; a motor that a model actuator of
the same name binds moves as the model moves, a motor no actuator binds
stays analytic, and a digital input with `sim_sensor` reads the model's
sensor. It runs the simulator's own program (`RunSimGateway`,
`talOS/simulation/gateway_main.h`) with `MujocoGatewayExtension`
(`gateway_extension.h`), so it describes and registers as the roster member
`sim_gateway` with the same `/sim/state` publication and reports the
`sim_gateway` program in its State; its target label differs, and it also
publishes `/sim/state/match` (below).

**The match.** Before the plant's first step the program adds the world
rules: the robot model's scripted shooters (below), if it has any, then the
field's game, if the field has one (`rebuilt_2026` does:
[its rules](fields/rebuilt_2026/README.md#rules)), made with
`[simulation.mujoco] seed`. The shooters come first, so a FUEL a shooter
launches is judged by the game at the exit in the same step. A contract the
model breaks (a shooter's missing exit, a field site the game needs) is a
config refusal, naming the element. Each tick, before the plant steps, the
rules get the tick's match input (`MatchInputOf`), and observe it at once
(`WorldRules::ObserveMatch`), so the period and match time follow it even on
a tick whose plant time rounds to no step: the Driver Station's
mode (its mode bits only while one is attached), the gateway's enable
(enabled and not e-stopped), the FMS alliance and the schedule clock once
it has started (`--mode-schedule`, `--schedule-start-file`). The Driver
Station packets carry the game's match time and game message as of the
tick they are sent on, after its steps (the gateway's `Stamp` hook); with
no game they carry 0 and an empty message. After each tick the program
writes `/sim/state/match` (`sim.MatchState`, `talOS/hardware/sim.fbs`): the
game's report, the tick's node-clock time as `sample_time_ns` (the clock
`/sim/state` is stamped on, at the tick's start where `/sim/state` reads it
after the tick) and the scripted shooters' launches as
`shots_count`; with no game, period None and zero counts but the shots.
The topic is declared whatever the config, so the launcher's `--describe`
probe and the run agree. The same model, config, seed and tick times give
the same States and the same `/sim/state/match`.

A config selects it in `[robot] sim_gateway`
(`"//talOS/simulation/mujoco:mujoco_gateway"`, or
`"@talos//talOS/simulation/mujoco:mujoco_gateway"` from a robot
repository), and the launcher starts it under `--sim --start-sim-gateway`
([launcher](../../launcher/README.md#lightweight-simulation-gateway)). The
model is the config's `[simulation.mujoco]`
([keys](../../hardware/README.md#configuration)); a config without it is
refused, naming `[simulation.mujoco] robot_model`, and `sim_gateway` given a
config with it prints that it ignores the table. A field name is resolved
to the generated model in the program's Bazel runfiles: under `bazel run`,
`bazel test` and `talos_sim` (also from a robot repository, where talOS is
`@talos`), and from `bazel-bin` after a build. A binary copied without its
runfiles is refused, naming the field.

Its flags are `sim_gateway`'s, with these differences:

- `--physics` is implied and accepted;
- `--start-pose` also places the robot on the field;
- when the model owns the chassis (its root body has a free joint), a
  nonzero `--wheel-radius-error` or `--wheel-slip-gain` is refused, naming
  the flag, and so are `[simulation] wheel_radius_m`,
  `[[simulation.swerve_modules]]` and `[simulation.chassis]`, naming the
  key.

What the backend itself refuses when it binds the plant (a partly bound
follower chain, `[[simulation.mechanisms]]` or `[[simulation.hard_stops]]`
naming a bound motor, ...) is in the
[external plant](../../hardware/README.md#an-external-plant) section. Every
refusal, and every model the plant refuses, exits 1 before the socket opens,
naming the key, flag, file, element or actuator.

The model compiles before the gateway's socket and registry row exist, so
it counts against the launcher's start-up grace for a process with no
registry row; the 2026 field with its FUEL compiles in a small fraction of
it. On start the program prints `sim_gateway: backend=mujoco`, and on its
first tick, after the gateway bound the plant, the binding table (below),
to stderr. While it runs it prints to stderr, at most
once a wall second each, the plant's clock when plant time was dropped
(time dilation: the host ran slower than real time, and the plant does not
catch up; [the plant's clock](../../hardware/README.md#an-external-plant))
and the count of overspeed excursions (a bound rotor above 250 rps, logged,
not fatal). In the plant and the extension, only these reports read the
wall clock. After `sim_gateway`'s
summary line it prints to stdout `mujoco_gateway: stepped_s=...
dropped_s=... dropped_ticks=... overspeed_excursions=... period=...
blue_points=... red_points=... shots=...`, the last four from the last
`/sim/state/match`.

A world that fails (see **Failure** below) puts every output in total loss
first; at the end of that tick the program prints the plant's clock and the
failure, naming the MuJoCo warning, and exits 1 without a summary.

To check a model, run it for ten ticks and read the binding table:

```sh
bazel run //talOS/simulation/mujoco:mujoco_gateway -- \
  --config "$PWD/<robot>/main_processor/configuration/robot_sim.toml" \
  --duration-s 0.05
```

A file the model reads (a mesh, an included MJCF) that `robot_model_files`
does not list is not refused at config load, since MuJoCo opens it; it is
found from the source tree, and fails only where just the listed files are,
such as a `talos_sim_test`'s runfiles or a bundle. A robot repository
therefore gives its simulated config a `talos_sim_test`.

### Intended differences from `--physics`

Against the analytic `[[simulation.mechanisms]]` model of the same inertia a
bound motor matches within the bounds `gateway_differential_test` holds
(speeds within 0.5 % of free speed per tick, 0.6 % under a binding supply
limit, positions within |ω|h/2 + 0.5 %). These differ by design:

- soft limits: a bound motor's output toward a reached limit goes neutral
  and the joint is left where it is; an analytic motor's axis is clamped;
- hard stops: the model's joint range replaces `[[simulation.hard_stops]]`;
- regenerative current in deep overspeed is capped at the stator limit,
  also when a sagging battery lowers the voltage ceiling;
- a rotor-side inertia below h·b_em/2 has wrong transients (the binding
  table warns), though it stays stable;
- motor and CAN sensor samples are as old as the plant lags the sample
  time, up to half a step.

### Writing a robot model

A robot model is one MJCF file. Its worldbody holds exactly one body, the
root, and nothing else of its own (no geom, site, camera, light or frame):
the field is the world, and a robot with its own floor is refused, naming
the element. A robot model is self-contained: it compiles on its own and
names no field element, so one model runs on every field and on the flat
carpet. A model that does not compile alone is refused with MuJoCo's
message: a reference that does not resolve (an actuator on a misspelt joint
or tendon, a sensor on a missing site), which attaching would otherwise drop
without a word, or a reference to a field element (a `<pair>` with
`carpet_robot`, say).

A root with a `<freejoint/>` owns the chassis: the Pigeon2's yaw and yaw
rate and `/sim/state` come from the root body's own frame, and the analytic
chassis keys and flags above are refused. A root without one is welded at
the start pose. MuJoCo aligns a free joint with its body's centre of mass
and principal axes when the body has that one joint and no child body and
the joint asks for it (`align="true"`, or `align="auto"` with
`<compiler alignfree="true">`); that would move and turn the reported
chassis frame away from the model's origin, so it is refused, naming the
joint (fix: `align="false"`).

**Start pose.** `--start-pose` places a frame on the field at (x, y) turned
by the yaw; the root body's own `pos` and `quat` compose after it. A model
authored at the origin facing +x starts at the start pose, at its authored
height.

**Units and names.** Each model compiles with its own `<compiler>`, so a
robot in radians keeps its angles on a field in degrees. The robot is
attached without a prefix: actuator names stay the talOS motor names, sensor
names stay the `sim_sensor` values, and field names are unchanged. A robot
name equal to any field name, of any element type, is refused, naming both.
Default classes may share names with the field's; each model's classes apply
to its own elements.

**Collision.** Every robot geom has `contype="4" conaffinity="21"`, or is
visual (`0`, `0`), per the field's
[collision contract](fields/rebuilt_2026/README.md#collision-contract): it
then rests on `carpet_robot` and meets field structure, fuel and other robot
geoms. MuJoCo's default (1, 1) never touches `carpet_robot`, so such a geom
is refused, naming it; the fix is
`<default><geom contype="4" conaffinity="21"/></default>`. With no `field`,
the world is the field's two carpets alone, with the same bits.

**Actuators.** Every actuator is a plain `<motor>` whose name is the talOS
motor it binds; an actuator that names no configured motor is refused. A
bound actuator has no `ctrlrange`, `forcerange`, `delay` or `nsample` of its
own (the emulated TalonFX limits voltage and torque), and drives a hinge or
slide joint or a tendon. Nothing may change its force after the motor's law:
a joint or tendon `actuatorfrcrange` and a joint's `actuatorgravcomp` on a
bound transmission are refused. A motor no actuator names stays analytic, as
under `--physics`. A config pushed later that changes a bound motor's
gearing or motor type is refused; restart the gateway with it.

**Gear.** The actuator's `gear` maps its joint or tendon to mechanism
radians, the shaft talOS's gearing (`rotor_to_sensor_ratio ×
sensor_to_mechanism_ratio`, G) ends at: `gear="1"` on a hinge that is the
mechanism; `gear` = 1/r on a slide or a tendon in metres driven by a drum or
pulley of radius r. The plant applies G: the rotor reads G·(length − length
at the reference pose)/2π rotations, absolute from the model's reference
configuration (a joint's `ref`, a tendon's rest length), so a CANcoder reads
the true angle of a joint that starts away from zero.

**Rotor inertia.** The rotor's inertia is the model's: give the actuator
`armature` = G²·J_rotor, in its own coordinate (mechanism radians). The
motor's no-load drag, G²·b_m, is added to the actuator's `damping` when the
model compiles; do not model it. The binding table gives each motor's
actuator, joint or tendon, G, J_act (the actuator's inertia at the reference
pose, 1/(moment·M⁻¹·moment), armature included) and J_rotor = J_act/G², with
a warning for a rotor-side inertia below h·b_em/2 (2.8e-5 kg·m² for a Kraken
X60 at 5 ms), whose transients would be wrong. J_act is the free-flight
value at the reference pose with no contacts, so a free chassis's own
mobility lowers it for a mechanism mounted on it. An actuator with no moment
at the reference pose (a tendon across its sled's travel) has its inertias
printed as unknown, and no warning. A 0.5 kg carriage on a slide geared 20
(a 5 cm drum) at G = 10, for example, has J_act = 0.5/20² = 0.00125 kg·m²
and J_rotor = 1.25e-5 kg·m²: light until its actuator carries the rotor's
armature.

**Torque.** Each tick the plant writes the emulated controller's output into
the bound actuator; at every MuJoCo step the torque is the winding's,
k_t/R·(V − k_e·ω) clamped to the stator or torque-current limit on either
side, at that step's rotor speed.

**Digital inputs.** `sim_sensor = "<name>"` names a sensor of the model:

- a `<rangefinder>` on a site, one ray along the site's +z measuring `dist`,
  with `cutoff` > 0: closed while 0 ≤ distance < cutoff (−1, nothing hit, is
  open; geoms of the site's own body are not seen);
- a `<touch>`: closed while there is contact force in its site's zone.

Any other sensor, or a missing one, is refused. An input reads the world as
of the start of the last 5 ms step (MuJoCo computes sensors before it
integrates), one step behind the rotors and the chassis.

**Scripted shooter.** A flywheel launches by rule, not by contact, which at
5 ms is not predictive (REQ-DEV-39). The model carries, per flywheel motor
`<motor>` (a `<motor>` actuator of that name on a hinge):

- a box site `<motor>_inlet`, the trigger volume;
- a site `<motor>_exit`, where a launched body's centre reappears, its +x
  axis the launch direction, outside the inlet and clear of robot geoms by
  the body's radius;
- `<numeric name="<motor>_shooter" data="efficiency contact_radius_m"/>`:
  the launched body's speed over the flywheel's surface speed, in (0, 1],
  and the flywheel's radius where it touches the body.

The joint's positive direction shoots, and the actuator's `gear` is
positive. `MakeScriptedShooters`
(`scripted_shooter.h`) finds every such motor in a compiled model and
returns them as one world rule (below), or none when no actuator carries any
part of the contract; `mujoco_gateway` adds it (the match, above). It
refuses, naming the motor and the element, a
missing site or numeric, a numeric without exactly two values, an
efficiency outside (0, 1] or a contact radius that is not positive, an inlet
that is not a box, a flywheel that is a slide or a tendon or whose
actuator's gear is not positive, a site that is
not on the flywheel's robot, a `*_inlet` site or `*_shooter` numeric with no
actuator of its motor's name (the suffixes are the contract's), and, when
attached, an exit inside its own inlet.

After each MuJoCo step it launches every free body outside the flywheel's
robot whose centre is inside the inlet, one shooter after another in
actuator order, bodies in index order:

- the body rolls between the flywheel and a fixed hood, so the flywheel's
  share of the impulse is (1 + β)/2 · m · v with β = I/(m r²), I the mean of
  the body's principal inertias and r below, and the flywheel joint's speed
  w drops by k·v, k = (1 + β)/2 · m · r_c / J, r_c the contact radius and J
  the joint's inertia (its subtree about the axis at the reference pose,
  plus the joint's armature and its actuator's armature times gear²,
  MuJoCo's `dof_M0`);
- the body leaves at the speed the launch leaves the flywheel with: v = η ·
  r_c · (w − k·v), so v = η · r_c · w / (1 + η · r_c · k), η the
  efficiency. The flywheel never reverses, the momentum the launch carries
  is what the flywheel loses, and the body's energy is at most η times what
  the flywheel loses;
- the body's centre moves to the exit; its linear velocity becomes the
  exit's own velocity (the robot may be moving and turning) plus v along
  the exit's +x; its angular velocity becomes the exit's plus v/r about the
  exit's +y (rolling), r its first geom's bounding radius (a sphere's
  radius; no geom: no rolling spin);
- a flywheel at rest or turning backward launches nothing: the body stays
  where the contacts hold it and goes once the flywheel turns forward, as a
  ball fed to a stopped wheel does;
- one body at a time: a body waits in the inlet while another body's centre
  is nearer the exit than their two radii;
- a body is launched at most once per step, whichever shooter took it.

A body of the flywheel's own robot (its free root) is never launched, and
the shooter changes nothing but the launched body and the flywheel's speed.

**World rules.** What changes the world after a step besides physics is a
`WorldRules` (`world_rules.h`), added to the plant with
`MujocoPlant::AddRules` before its first `Step`; `mujoco_gateway` adds the
robot's scripted shooters, then the field's game. After each MuJoCo step
that passes the failure check below, the plant brings positions and
velocities up to the step's end (`mj_kinematics`, `mj_comPos`, `mj_comVel`;
not `mj_forward`, whose collision and constraint stages cost far more) and
runs every rule in the order added, with the match input last given to
`SetMatchInput`, which each rule has also observed when it was given
(`ObserveMatch`, before any step). A rule may write free joints' positions
and velocities, the velocity of a joint it names and the collision bits of
geoms its README names. A rule that reports a failure or throws stops the
world for good, as an unstable step does, with the rule's text. A rule's
`Attach` that leaves the world unstable (a bad position, velocity or
acceleration, a full arena) is refused by `AddRules`, naming the warning,
and fails the plant.

**Solver.** The robot's whole `<option>` and `<size>` are dropped when it is
attached: the field's win, MuJoCo prints a warning naming each field the
robot authored, and the robot's `<flag>`s are dropped without one. Then one
table replaces the field's solver settings: 5 ms step, CG, 20 iterations,
tolerance 1e-8, elliptic cone, `implicitfast`, sleeping off, automatic reset
off, 64 MB arena. The field's other options, such as gravity, stay.

**Failure.** An unstable step (a NaN or huge position, velocity,
acceleration or control) or a full arena stops the world for good, naming
the warning, the joint or actuator and the plant time: every output goes
neutral and the gateway exits 1. An error inside MuJoCo itself exits 1 with
a line naming the MuJoCo plant and MuJoCo's message. MuJoCo's log goes to
stderr only, never to `MUJOCO_LOG.TXT`.

### Tests

`plant_test` covers the plant alone, world rules' order, step numbers,
match input and failure included. `scripted_shooter_test` (REQ-DEV-39)
launches from a robot at rest and from one moving and turning, with the
flywheel's loss; never launches the robot; waits for a forward flywheel and
a clear exit; refuses each partial contract; and repeats bit for bit with
no allocation. `gateway_differential_test` runs a
bound motor against the analytic model, tick for tick through two
`Gateway`s, with the bounds above. `gateway_bit_identity_test` (REQ-DEV-37)
assembles the world twice as the program does and ticks both at the same
jittered times with the same commands, a tick past the step cap, a device
reset and a supply fault: their encoded States are identical, and a 1 µs
change at one tick makes them differ. It does the same with the 2026 field,
its rules and a robot whose scripted shooter launches a preload into the
blue HUB (REQ-DEV-39, REQ-DEV-40), comparing `/sim/state/match` too, over
a disabled, AUTO and TELEOP sequence: the periods come in order, the
launch scores in the step that launched it, each tick's Driver Station
stamp is that tick's match, the tick TELEOP starts on included, and
another seed draws the other alliance at TELEOP's tie.
`gateway_extension_test` holds the chassis-owner refusals, one per key
and flag, against a welded root that accepts them, and the match input
each Driver Station state gives. `fields_test` holds
the field names config load accepts to the ones the program loads, each
with its FUEL, and attaches each field's game. `match_state_test`
(integration, REQ-DEV-40) runs the program on the 2026 field through a mode
schedule: `/sim/state/match` walks None, AUTO and TELEOP counting down,
each Driver Station packet carries the time left and game message of its
own tick's `/sim/state/match`, the first TELEOP packet included, and the live
registry row declares what `--describe` declares. `mujoco_gateway_test`
(integration) runs the program: `--describe` against
`sim_gateway --physics`'s, `/sim/state/match` aside; refusals from the
extension, from the backend, from the plant's `Bind` and from a scripted
shooter's broken contract, each named on stderr; a model file
`robot_model_files` omits, against the same config where the file is; a
diverging model; and 50 ms on the 2026 field. `testdata/` holds small
models and configs (an arm on a welded root, a free-root rover with a
rangefinder input, a model reading an unlisted mesh).
