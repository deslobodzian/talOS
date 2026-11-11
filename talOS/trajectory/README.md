# talOS trajectory

Time-indexed holonomic trajectories, a loader for [Choreo](https://choreo.autos)
`.traj` files, alliance flips, and `HolonomicFollower`, which turns a
trajectory and a measured pose into a chassis speed. A library beside the node
stack: it depends on `talOS/geometry` and `@nlohmann_json` only, never on
`talOS/node` or a robot. REQ-DEV-13 (loading), REQ-DEV-14 (markers, flip) and REQ-DEV-15 (follower)
state what it does; REQ-RT-06 covers the per-tick calls.

```cpp
#include "talOS/trajectory/choreo.h"
#include "talOS/trajectory/holonomic_follower.h"
```

`//talOS/trajectory` (`cc_library`, public).

## Frames and units

Field frame, as in `talOS/msgs/geometry.fbs`: origin at the blue-alliance
corner, x downfield, y left, headings CCW+ in radians. SI throughout, with the
unit in every name: `t_s`, `x_m`, `heading_rad`, `vx_mps`, `omega_radps`,
`ax_mps2`, `alpha_radps2`. Choreo writes the same frame and units, so nothing
is converted on load. Velocities are field-relative. Headings are continuous
along the path, as Choreo writes them (not normalised to (-pi, pi]):
`sample()`, `Flip()` and the stored samples never wrap them; wrap only a
difference (`WrapAngle`), as `HolonomicFollower` does for its heading error.
`pose()` wraps, since `SE2` stores a principal angle.

## `Trajectory`

A `TrajectorySample` is `{t_s, x_m, y_m, heading_rad, vx_mps, vy_mps,
omega_radps, ax_mps2, ay_mps2, alpha_radps2}` with `pose()` as a
`geometry::SE2<double>`.

| Call | Does |
|---|---|
| `Trajectory(samples, interpolation, events, split_starts)` | validates (non-empty, finite, times non-negative and strictly increasing, splits in range) and throws `std::invalid_argument` otherwise |
| `sample(t_s)` | the state at `t_s`; at or before the first sample's time (or NaN) the first sample, at or after the total time the last |
| `total_time_s()`, `initial_sample()`, `final_sample()`, `initial_pose()`, `final_pose()` | ends of the path |
| `events()` | event markers (`{name, t_s}`) in time order |
| `EventCursor::advance(trajectory, t_s)` | fires markers: the first call after construction or `reset()` returns every marker at or before `t_s` (one at 0.0 fires on the first tick), each later call those after the previous call's up to `t_s`; it counts markers, so none is skipped or repeated |
| `split_count()`, `split_starts()`, `split(i)` | Choreo splits; `split(i)` is segment `i` re-timed to start at 0 with the markers from its start up to, not including, its end (the last segment also keeps one at its end), so a marker on a boundary is in the later segment only and fires once when the segments run in turn |
| `reserve(n)`, `assign(samples, interpolation)` | rebuild in place for a trajectory made inside `update()`: `assign()` validates like the constructor (unchanged on a throw), clears markers and splits, and allocates nothing while `samples.size() <= capacity()` |
| `flipped(flip, field)`, `Flip(sample/pose, flip, field)` | the other alliance's path |

Interpolation between stored samples:

- `kConstantAcceleration` (every sample has `ax`, `ay`, `alpha`): exactly
  ChoreoLib's interpolation. From the earlier sample `a`, with
  `tau = t - a.t`: `x = a.x + a.vx tau + a.ax tau^2 / 2` (likewise `y`, and
  `heading` with `omega`, `alpha`), `vx = a.vx + a.ax tau`, accelerations
  copied. A Choreo solve obeys this model, so it lands on the next sample; a
  file that does not (hand-edited, rounded) can step at a sample, as it
  does in ChoreoLib (`TrajectoryTest.ConstantAccelerationSamplingIsExact`).
- `kHermite` (no accelerations; not something Choreo writes): cubic Hermite
  on positions and velocities, heading taken the short way to the next
  sample; accelerations are its second derivative.

Storage is allocated when the trajectory is built. `sample()`,
`EventCursor::advance()`, `Flip(sample)`, `HolonomicFollower::step()` and
`assign()` within `reserve()` never allocate (`no_allocation_test`), so they
belong in `update()`; loading, the constructor, `split()` and `flipped()`
allocate and belong in the node's constructor or on a mode change.

## Alliance flip

| Flip | Position | Heading | Velocity, acceleration |
|---|---|---|---|
| `AllianceFlip::kRotate` (180 degrees about the field centre) | `x -> L - x`, `y -> W - y` | `pi + heading` | `vx, vy, ax, ay` negated; `omega, alpha` kept |
| `AllianceFlip::kMirror` (across the centre line) | `x -> L - x`, `y` kept | `pi - heading` | `vx, ax, omega, alpha` negated |

Field size is passed in (`FieldDimensions{length_m, width_m}`), never assumed
by the maths. Symmetry by game: 2022 rotate, 2023 mirror, 2024 mirror, 2025
rotate, 2026 rotate. Presets carry both: `kField2026` (16.541 m x 8.0692 m,
rotate) and `kField2025` (17.548 m x 8.052 m, rotate):
`trajectory.flipped(kField2026.flip, kField2026.field)`. Flipping twice
returns the original (rotate: heading plus 2 pi).

## Choreo files

`LoadChoreoFile(path)` and `ParseChoreo(json, source)` return a `LoadResult`:
`trajectory` and `ok()`, or an empty trajectory and `error`, a message that
starts with the file name and names the JSON location
(`auto/two_piece.traj: trajectory.samples[3]: missing "vx"`). They never throw
or crash on malformed input, however deep or large: an error never echoes a
JSON value, only its type or the first 64 bytes of a string, so a node that
logs it on every restart cannot be taken down by its own file.

Plausibility: a sample with `|x|` or `|y|` above 100 m (`kMaxCoordinate_m`),
a time outside 0 to 3600 s (`kMaxTime_s`), or a time not after the previous
sample's is refused, naming the sample.

Accelerations: interpolation is `kConstantAcceleration` only when every
sample has `ax`, `ay` and `alpha`. If any sample lacks one, the whole file
is interpolated with `kHermite`, and `LoadResult::note` says so, naming the
first such sample, for the node to log. `note` is empty for a file Choreo
wrote.

Versions: `version` is an integer; 1 (Choreo v2025.0.x), 2 (v2026.0.0-beta-1)
and 3 (v2026.0.0 to v2026.0.3 and current) are accepted
(`kChoreoMinVersion`..`kChoreoMaxVersion`). Their sample layout is the same;
3 adds `trajectory.config`, which is not read. Any other value (0, 4, a
fraction, a string) or a missing `version` is refused with an error naming
`version`. `testdata/choreo_v3_swerve.traj` is trimmed from Choreo's own
`test-jsons/trajectory/3/swerve.traj`; the other fixtures are generated.

Top-level keys are `name`, `version`, `snapshot`, `params`, `trajectory`,
`events`. Fields read:

| Field | Use |
|---|---|
| `version` | integer 1, 2 or 3 |
| `trajectory.sampleType` | `"Swerve"`; `"Differential"` (or any other value) is refused: holonomic only |
| `trajectory.samples[]` | `t` (s), `x`, `y` (m), `heading` (rad), `vx`, `vy` (m/s), `omega` (rad/s) required; `ax`, `ay` (m/s^2), `alpha` (rad/s^2) read when present in every sample (Choreo always writes them); `fx[4]`, `fy[4]` (N) ignored, neither required nor refused |
| `trajectory.splits` | sample indices where each segment starts; 0 is prepended when the first is not 0; segment `i` runs from `splits[i]` to `splits[i+1]` inclusive of the shared sample |
| `events[]` (top level) | `{name, from: {target, targetTimestamp, offset: {exp, val}}, event}`; time `from.targetTimestamp + from.offset.val` s. As in ChoreoLib, an event whose `targetTimestamp` is not a number, whose time is negative or whose name is empty is dropped, not an error, and `LoadResult::dropped_event_count` counts them; a time past the end is kept |

`trajectory.waypoints` (the waypoints' times), `trajectory.config`,
`snapshot`, `params`, `name` and the event's command payload are ignored: the robot
decides what a marker's name does.

## `HolonomicFollower`

```cpp
talos::trajectory::FollowerConfig config;   // defaults: kp 2 (x, y), 3 (heading)
config.translation.ki = 0.5;
config.translation.integral_limit = 0.5;    // m/s
talos::trajectory::HolonomicFollower follower{config};

const auto out = follower.step(trajectory, elapsed_s, measured_pose, dt_s,
                               talos::trajectory::SpeedsFrame::kRobot);
drive_.send(out.speeds.as<drivetrain::ChassisTarget>());
if (out.finished) ...
```

Per step: the sampled `vx`, `vy`, `omega` as feedforward, plus PID on the
field-frame x and y errors (same gains) and on the heading error taken the
short way across +-pi. The linear command is scaled down to `max_linear_mps`
(keeping its direction) and omega clamped to `max_angular_radps`. `kRobot`
rotates the command into the measured heading's frame, the robot-relative
`ChassisTarget` the 2026 drivetrain takes; `kField` leaves it in the field
frame. `ChassisSpeeds::as<M>()` builds any `{vx_mps, vy_mps, omega_radps}`
struct (`drivetrain::ChassisTarget`), and
`step_pose()` takes any pose with `x_m()`, `y_m()`, `yaw_rad()`
(`geometry::Pose2d`).

`FollowerOutput` also carries the reference sample, the errors, `finished`
(the time has reached the end and the pose is within `position_tolerance_m`
and `heading_tolerance_rad` of the final pose) and `valid` (false, with zero
speeds and the controller state untouched, when the time, period or pose is
not finite). The integral is held within `integral_limit` (output units); 0
turns it off. The derivative is the change in error since it was last taken
(the heading's wrapped, so crossing +-pi is not a 2 pi step) over the time
since then; while that time is under `kMinDerivativePeriod_s` (1 ms) it holds
its last value, so a near-zero `dt_s` cannot turn noise into a huge command.
The first step after construction or `reset()` only records the error.
`reset()` clears the integral and derivative when a trajectory starts or the
robot was disabled. The constructor throws `std::invalid_argument` naming a
negative or non-finite gain, limit or tolerance. Same inputs, same outputs:
the follower is plain arithmetic, with no clock or global state.

## Using it from a node

Trajectory files live in the robot's configuration directory,
`<robot>/main_processor/configuration/trajectories/` (the Choreo project
`.chor` beside its `.traj` files). The routine's subsystem file names each
one under `[trajectories]`, relative to that subsystem file:

```toml
[trajectories]
two_piece = "../trajectories/two_piece.traj"
```

Config load checks each entry and the bundle ships each file
([Configuration](../../ARCHITECTURE.md#configuration)); regenerate the
manifest after adding one. The node opens it by name: `TrajectoryPath`
returns the absolute path, or throws naming `trajectories.<name>` when the
subsystem does not declare it:

```cpp
const auto config = talos::config::ParseRobotConfig(node.config_path());
auto loaded = talos::trajectory::LoadChoreoFile(
    config.TrajectoryPath(node.name(), "two_piece"));
if (!loaded) throw std::invalid_argument(loaded.error);   // constructor
if (!loaded.note.empty()) log(loaded.note);
const auto& game = talos::trajectory::kField2026;
trajectory_ = red ? loaded.trajectory.flipped(game.flip, game.field)
                  : std::move(loaded.trajectory);
```

Load and flip in the constructor (or once when the alliance is known). When
the trajectory starts, `follower.reset()` and `cursor.reset()`; then per
tick: `follower.step(...)`, send the speeds on the drivetrain's `auto` lane,
act on each marker in `cursor.advance(trajectory_, elapsed_s)`, and
`release()` the lane once `finished`. A trajectory built from the live pose
inside `update()` uses `reserve()` once and `assign()` per rebuild. The trajectory library does no actuation gating: the
drivetrain's `Target` and `robot_enabled` still decide whether anything moves.

`2026-robot/main_processor/autonomous` does not use this library (TODO.md
TRAJ-1).

## Tests

`bazel test //talOS/trajectory:all`. `testdata/choreo_v3_swerve.traj` is
Choreo's own output (trimmed); the rest are generated: `curve.traj` (version
3) a half circle sampled from an analytic path with a continuous heading
past pi (the follower's path), `splits_events.traj` (version 2) two
constant-acceleration segments with a split and waypoint-anchored events
(the allocation test's).
