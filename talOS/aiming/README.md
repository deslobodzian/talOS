# talOS aiming

Shooter aiming maths as a library: where to point, how fast to spin, and
whether to fire, for a robot standing still or moving. No node, no topics,
no clock: a node feeds it a pose and publishes the results. No node uses it
yet, and no requirement in docs/REQUIREMENTS.md names it (TODO.md AIM-1). `//talOS/aiming`, include
`talOS/aiming/target_solver.h` and `talOS/aiming/readiness.h`.

| Piece | Header | What it does |
|---|---|---|
| `InterpolatingTable<N, Capacity>` | `interpolating_table.h` | strictly increasing key → N values, linear between rows, clamped and flagged outside |
| `ShotTable` | `target_solver.h` | the table for shots: `distance_m` → `flywheel_rps`, `hood_rad`, `time_of_flight_s` |
| `TargetSolver` | `target_solver.h` | distance, shot heading, robot heading or turret angle, table lookup; shoot-on-the-move |
| `ReadinessGate` | `readiness.h` | ready to fire, the first failing reason, debounce in ticks |

Everything is deterministic, single-threaded and has no global state.
Tables are built once (fixed-capacity storage, 32 rows for `ShotTable`);
`ShotTable::at`, the `TargetSolver` solves and `ReadinessGate::Update`
never allocate (the solves are checked with `AllocationProbe`) and never
return NaN. Bad configuration throws `std::invalid_argument` from the constructor,
naming the key or row.

## Frames, signs, units

* Field and robot frames are those of `talOS/msgs/geometry.fbs` (wpimath's):
  x forward (downfield), y left, angles CCW+ in radians. The robot pose is
  `field_to_robot`, a `talos::geometry::SE2<double>`: its translation is the
  robot origin in the field, its angle the robot's yaw. The yaw need not be
  wrapped.
* Every angle the solver returns is wrapped to `(-pi, pi]`, except
  `turret_rad` (below).
* The robot velocity has two entry points, so the frame is in the type.
  `SolveWithChassisVelocity` takes a `geometry.Twist2d`, robot-relative as
  `DrivetrainState` and `PoseEstimate.velocity` publish it, and rotates it
  by the pose's yaw (`ToFieldTwist`). `SolveWithFieldVelocity` takes a
  `FieldTwist` (`vx_mps`, `vy_mps` along the field axes, `omega_radps`
  CCW+), which a `Twist2d` does not convert to, so passing a message's
  velocity there does not compile. Both run the same solve.
* `ShooterMount {x_m, y_m, yaw_rad}` places the shooter in the robot frame:
  a fixed shooter's exit point, or a turret's axis. `yaw_rad` is the
  direction the shooter fires (a turret: at turret angle 0), CCW+ from robot
  forward; a shooter on the back has `yaw_rad = pi`.
* `turret_rad` is the turret angle CCW+ from the mount yaw. It is not
  wrapped: among the equivalent angles `a + 2 pi k` it is the one inside
  `TurretTravel [min_rad, max_rad]` nearest the measured turret angle, so a
  turret with more than a turn of travel does not unwind needlessly.
  Otherwise `turret_reachable` is false and a limit is held: the limit the
  needed angle has just crossed, if it is within `seam_hysteresis_rad`
  (default 0.05 rad) of it, so noise on the seam of a full-turn travel
  cannot sweep the turret to the other end every tick; else the limit
  nearer the measured turret angle (on a tie, the one nearer `a`), so noise
  in a dead-zone target cannot swing it between limits.
* Distances are horizontal, in metres; the vertical part of the shot lives
  in the table (`hood_rad`, `flywheel_rps`, `time_of_flight_s`), measured on
  the field.

## The maths

### Standing shot, turret

The turret axis is at `e = p + R(theta) m` (robot origin `p`, yaw `theta`,
mount offset `m`). Aiming at a field point `V`: distance `s = |V - e|`, shot
heading `phi = atan2(V - e)`, turret angle `wrap(phi - theta - mount_yaw)`,
then chosen within the travel as above. The robot keeps its heading.

### Standing shot, fixed shooter

The robot turns in place (about its origin) to heading `psi`, the shot
heading is `phi = psi + mount_yaw`, and the exit point moves with it,
`e = p + R(psi) m`. Express the mount offset in the shot frame,
`(a, b) = R(-mount_yaw) m`: the exit point is `a` ahead of and `b` left of
the line through `p` along `phi`. With `d = V - p`, `D = |d|`,
`alpha = atan2(d)`, the target is on the ray from `e` along `phi` when its
lateral coordinate in the shot frame equals `b`:

```
D sin(alpha - phi) = b   =>   phi = alpha - asin(b / D)
s = sqrt(D^2 - b^2) - a
```

This is exact for any offset; it needs `D > |b|` and `s > 0` (otherwise the
target is inside the offset or behind the exit point, and the solution is
invalid). For a centred shooter it reduces to `phi = alpha`, `s = D`.

### Shooting on the move

Model (planar, matching the table): a shot commanded for distance `s`
leaves the exit point with robot-relative horizontal velocity
`(s / tof(s)) * u(phi)` and lands after `tof(s)`. The robot adds the exit
point's field velocity

```
v_e = v + omega x r = v + omega * (-r_y, r_x),    r = e - p
```

where `omega x r` is the tangential lead: a shooter off the rotation axis
moves even when the robot only spins. The ball lands at
`e + s u(phi) + v_e tof(s)`. That is the target `T` exactly when the shot is
the standing shot at the **virtual target** `V = T - v_e t` with
`t = tof(|V - e|)`. `t` appears on both sides, so the solve iterates

```
V_0 = T
k = 1, 2, ...:  aim at V_{k-1} -> s_k, phi_k, e_k;  t_k = tof(s_k)
                stop when |t_k - t_{k-1}| <= time_of_flight_tolerance_s (k >= 2)
                V_k = T - v_e(e_k) t_k
```

for at most `max_iteration_count` iterations (default 20, tolerance 1 ms).
For a fixed shooter `e` (and so `v_e`) moves with the solved heading and is
recomputed each iteration. With zero shooter velocity it converges at
`k = 2` to the standing solution.

The map `t -> tof(|T - v_e t - e|)` has slope about `|v_e| * dtof/ds`, so
the iteration converges when that is below 1 (a typical lobbed shot,
`dtof/ds` around 0.15 s/m, converges up to about 6 m/s) and oscillates or
diverges otherwise. When it does not converge within the bound, the solve
returns the last iterate with `converged = false`: aimed roughly ahead of
the target, but not compensated exactly, and `ReadinessGate` refuses to
fire on it. Steep tables (time of flight changing fast with distance) need a
slower robot; a test with a table that jumps 1.9 s per metre at 2 m/s
proves the bound and the flag.

What the solver does not model: drag and lift on the ball beyond what the
table captures (the table is measured standing, so drag along the shot is
in it; drag on the robot-velocity component is not), the robot's motion
between the pose's sample time and release (extrapolate the pose by that
latency before solving), and a moving target.

## Readiness

`ReadinessGate::Update(input, tick_index)` is called once per control tick
with the age of the pose the tick's `Solution` was computed from, that
`Solution`'s flags, and measured errors (goal minus measured; the aim error
is the heading error for a fixed shooter, the turret error for a turret).
Every `ReadinessInput` field defaults to a value that fails its check (flags
false, numbers NaN, age unknown), so a field the caller forgets blocks the
shot; a robot without a hood sets `hood_error_rad = 0`. Checks, in order;
the first failing one is the reason:

| Reason | Fails when |
|---|---|
| `kStale` | `pose_age_us` unknown, negative, or `> max_pose_age_us` |
| `kNoSolution` | `!solution_valid` |
| `kNotConverged` | `!converged` (checked before range: a non-converged distance means nothing) |
| `kOutOfRange` | `!in_range` (the table was clamped) |
| `kTurretLimit` | `!turret_reachable` |
| `kRobotTooFast` | `|robot_speed_mps| > max_robot_speed_mps` |
| `kRobotRotatingTooFast` | `|robot_omega_radps| > max_robot_omega_radps` |
| `kAimError` | `|aim_error_rad| > aim_tolerance_rad` |
| `kFlywheelNotAtSpeed` | `|flywheel_error_rps| > flywheel_tolerance_rps` |
| `kHoodNotAtGoal` | `|hood_error_rad| > hood_tolerance_rad` |
| `kSettling` | all pass, but not yet on `settle_tick_count` consecutive ticks |

A NaN error fails its check. Readiness rises only after `settle_tick_count`
consecutive passing ticks (1 means at once) and drops on the first failing
tick. `tick_index` is the caller's own tick counter: a streak continues
only from `tick_index - 1`, so ticks the caller skipped (disabled, paused)
restart the count, and a re-enable cannot report ready without settling.
`Reset()` restarts it explicitly (a new goal). Nothing here reads a clock,
so the gate is exact in simulation and replay. `CheckReadiness` is the same check without the
debounce.

## Tuning a table from field shots

1. Mark distances from the goal, horizontal, measured to where the shooter's
   exit point (or turret axis) will be, every 0.5 to 1 m across the range
   you intend to shoot from, closer where the shot is sensitive.
2. At each mark, with the robot still, find the flywheel speed and hood angle
   that score reliably: start from the neighbouring row, adjust one at a
   time, and take the middle of the band that scores, not its edge. Record
   them in mechanism units (`flywheel_rps` after the motor's
   `sensor_to_mechanism_ratio`, `hood_rad` from the hood's zero).
3. Measure the time of flight from release to the goal: high-speed video
   (count frames from the ball leaving the shooter to crossing the goal
   plane), or the flywheel's speed dip at release to a goal sensor. Average
   several shots. This column drives shoot-on-the-move, so a 50 ms error at
   3 m/s is a 15 cm miss.
4. Write rows with strictly increasing `distance_m`. Outside the first and
   last row the solver clamps and says `in_range = false`, so the ends of
   the table are the robot's shooting range.
5. Check on the move: drive past the goal at a few speeds and directions.
   Misses along the direction of travel mean the time of flight is wrong:
   too short if the ball lands beyond the target in the direction the robot
   was moving, too long if it lands short of it. Misses that grow with spin
   rate mean the mount offset is wrong.

## Tests

`bazel test //talOS/aiming:all`:

* `interpolating_table_test`: exact rows, interpolation, the clamp and its
  flag, NaN keys, every validation error, `ShotTable` columns.
* `target_solver_test`: standing solves against hand-computed geometry
  (centred and offset fixed shooters, rear-facing mount, offset turret,
  headings across `+-pi`, unwrapped robot yaw); turret travel past a turn,
  one-sided travel, unreachable targets held at the turret's limit, and the
  seam hysteresis; the chassis-velocity rotation, and a compile-time check
  that a `Twist2d` is refused as a field velocity; moving solves checked
  against a brute-force simulation of the ball (0.1 ms steps from the
  commanded turret angle or heading and distance, the exit point's velocity
  differenced from the moving pose) that must land within 2 cm while the
  standing solution misses by more than 30 cm, including a spin-only case
  for the tangential lead; the non-convergence bound; no allocation in
  the solves (`AllocationProbe`; skipped where the probe is unsupported, as
  under a sanitizer).
* `readiness_test`: every reason in order, failing defaults, stale and
  undated poses, NaN, tick debounce, skipped ticks and reset, limit
  validation.

The tests carry no `REQ-` tag: no catalogue requirement covers aiming yet
(TODO.md AIM-1).
