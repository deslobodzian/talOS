# Superstructure example

An intake, an elevator, and a superstructure that sequences them, written as
three **separate nodes** that talk only over topics. Goals travel on target
lanes; see [target policy and safety](../../ARCHITECTURE.md).

```
 /superstructure/target/teleop ─┐
 /superstructure/target/auto   ─┴▶ superstructure ──▶ /intake/target/superstructure   ──▶ intake   (roller + beam break)
                                       ▲  ▲        ──▶ /elevator/target/superstructure ──▶ elevator (motion-magic TalonFX + bottom switch)
                                       │  └──────────── /intake/state   ◀─────────────────┘     │
                                       └─────────────── /elevator/state ◀──────────────────────┘
```

Which lanes count in which mode is config. The superstructure uses the default
`[robot.targets]` policy; the intake and elevator each replace it with a
`[targets]` table in their subsystem file, so only the superstructure drives
them, in both modes:

| Subsystem | teleop | autonomous | Set in |
|---|---|---|---|
| superstructure | `/superstructure/target/teleop` | `/superstructure/target/auto` | default `[robot.targets]` |
| intake | `/intake/target/superstructure` | `/intake/target/superstructure` | `[targets]` in `subsystems/intake.toml` |
| elevator | `/elevator/target/superstructure` | `/elevator/target/superstructure` | `[targets]` in `subsystems/elevator.toml` |


The intake and elevator are thin wrappers over
[`talOS/mechanisms`](../../talOS/mechanisms/README.md) building blocks: the
intake's roller is a `RollerWithSensor` and the elevator's lift a
`PositionMechanism`. Each node maps its own goal message to the block's
`set_goal()` (or `stop()` without a live target), calls its `update()`, and
maps the block's phase to its own status enum.

- **intake**: IDLE → INTAKING (roller in) → HOLDING once the beam breaks (small
  hold voltage) → EJECTING (reverse for `eject_us`, 500000 µs by default) →
  IDLE. Roller voltages are `intake_v`, `hold_v`, and `eject_v`, in the
  `[roller]` table with `sensor = "beam_break"`. An EJECT asked while IDLE
  does nothing (`idle_runs` is off). With no target it behaves as asked for
  IDLE with the roller neutral; the state machine still tracks the beam break
  and finishes a timed eject.
- **elevator**: motion magic to the goal height (clamped to
  `min_rot`..`max_rot`, 0..12, in the `[lift]` table) while it has a target. It reports STOWED at 0, MOVING,
  or AT_GOAL (within `tolerance_rot`), and echoes the goal that `at_goal`
  refers to. With no target the lift is neutral (it never holds an old goal);
  its status echoes goal 0 with `at_goal` false, and reads STOWED within
  `tolerance_rot` of 0, MOVING elsewhere. See [Homing](#homing) for HOMING and
  HOMING_FAILED. A goal above `max_rot` is clamped and echoed as
  `max_rot`, so a `high_rot` set above it keeps the superstructure in
  RAISING, waiting for a height it never asked for.
- **superstructure**: STOWED → INTAKING (on INTAKE) → HOLDING (the intake
  has the piece) → RAISING (on SCORE_HIGH, elevator to `high_rot`) → SCORING
  (the elevator is at that height; intake ejects) → STOWED (the piece is gone).
  Cancellation is checked every tick: STOW or no goal (disabled, stale, released
  or never sent) abandons the active sequence immediately. With fresh mechanism
  statuses it requests intake IDLE and elevator height 0. If either status is
  stale, it releases **both** mechanism lanes instead of renewing a high
  elevator goal or EJECT. The mechanisms then have no target and are neutral;
  this is not a gravity-compensating elevator hold. The reported sequence falls
  back to HOLDING only when a fresh intake status still sees a piece, otherwise
  STOWED. Once both statuses return, sequencing starts from those observations
  and the current command, not the interrupted step: a piece that left during
  the outage is not scored again. Normal sequencing republishes both lanes
  every tick.

Every tunable (voltages, eject time, tolerance, heights, homing) is a parameter read
from the robot config: the intake's and elevator's by their building block
from its `[roller]` or `[lift]` table (`node.param<T>("lift.tolerance_rot")`),
the superstructure's with `node.param<T>("key")` in C++, `node.param("key")`
in Python.

The nodes are implemented once in C++ (`cpp/`) and wrapped in Python (`py/`)
via `TALOS_EXPORT_NODE` and `talos.cpp_node`, so both sets of binaries and
tests run the exact same implementation without duplicating logic.

For node APIs and message/build conventions, see the
[node development guide](../../docs/NODE_DEVELOPMENT_GUIDE.md).

## Homing

A TalonFX counts rotations from where it powered on, and a reset (brownout,
CAN fault) restarts that count at 0 wherever the carriage is. A goal of 10 in
that frame is 10 rotations above the carriage, not 10 above the bottom. So
heights count from a bottom limit switch (`[digital_inputs.bottom_limit]`,
named by `[lift] limit_switch` with `homing = "limit_switch"`), and the lift keeps the default `position_frame = "rezero"`: the devices layer
sends its motion-magic goals neutral until `Motor::rezero` has zeroed it, and
again whenever a TalonFX reset or a RoboRIO reboot drops that zero
(`zeroed()` false; see the node development guide's rezero paragraph).

The switch is normally open and wired between the DIO signal pin and ground.
The RoboRIO input's pull-up reads true while the carriage is off it and false
once the carriage closes it, so the elevator takes "at the bottom" from a
false reading. A cut or unplugged wire then reads "not at the bottom": homing
fails instead of zeroing the lift mid-travel and then driving it past the top.

While unzeroed the elevator reports HOMING, with `goal_rot` 0 and `at_goal`
false. With a live, finite target it drives the lift down at `homing_v` volts
(negative: down, toward the switch), a voltage request, which a dropped zero
does not hold and which the TalonFX bounds by the lift's current and voltage
limits. Once the switch has read closed on `home_samples` consecutive elevator
periods (default 3) it calls `rezero(0)` and goes on to the target's goal in
the same tick; until then homing keeps driving, and any open reading restarts
the count, so a one-tick glitch mid-travel does not zero the lift partway up.
A lift already on the switch zeroes there without moving, whether or not the
robot is enabled.

HOMING_FAILED latches, neutral, in three cases:

- the switch has not closed within `homing_timeout_us` of the first homing
  command (time with no target counts too);
- the lift has moved slower than `homing_stall_rps` for `homing_stall_us`
  while homing pushes it (jammed, or the motor is not turning);
- the switch reads closed while the zeroed lift is more than
  `home_margin_rot` above 0 (stuck closed or shorted), which the first move up
  after the zero shows.

It clears only when the Driver Station reports disabled; the next enabled
target retries. A stale hardware link, which also makes `robot_enabled` false,
does not clear it. The constructor refuses a `homing_v` that does not point
down toward `home_rot` at `min_rot` (0), a `max_rot` not above `min_rot`, a
`homing_timeout_us`, `homing_stall_rps`, `homing_stall_us` or
`home_margin_rot` that is not positive and finite, and a `home_samples` below
1. These are `PositionMechanism`'s limit-switch homing rules
([talOS/mechanisms](../../talOS/mechanisms/README.md#positionmechanism)).

Homing runs on any live target while unzeroed rather than on a separate
operator request. Like every other motion it needs `robot_enabled` and a
non-null `Target::get()`, so a disabled robot or a stale, released or
never-sent lane moves nothing. An unzeroed elevator cannot serve any goal, the
move is down, toward the stowed position the superstructure falls back to,
slowly and time-bounded, and a separate request would leave an autonomous
routine or a mid-match reset stuck until a driver noticed.

The lift has no hardware soft limits (`soft_limits = false`), and config
load refuses `soft_limits = true` on its `"rezero"` frame. The gateway checks
soft limits only against position and motion-magic demands and passes
duty-cycle and voltage requests, but the TalonFX (and the physics simulator)
enforces them on the measured position in every control mode, in its
power-on frame, from thresholds the RoboRIO applies once with the static
motor config; no request carries them, so no rezero moves them. After a reset
at height the reverse limit of 0 sits at the carriage, so it would stop
homing at once, and the forward limit would allow travel past the top.
`max_rot` bounds goals in the homed frame instead.

In software that clamp is the only protection at the top of travel: it bounds
the goals, not a carriage that overshoots, coasts or is driven by a bad gain.
So the lift declares `forward_limit_switch = "NormallyOpen"`: a switch wired
to the TalonFX's own forward limit input stops upward output in every control
mode, whatever the frame (see [talOS/hardware](../../talOS/hardware/README.md)).
That is also Phoenix's default, so with no switch wired nothing changes; use
`"NormallyClosed"` for a switch that also stops the lift when its wire breaks.

## Layout

```
messages/          shared by the C++ and Python nodes
  intake.fbs  elevator.fbs  superstructure.fbs   namespace demo; structs + enums
  BUILD                                          talos_messages(topics = {...}, include_prefix = "demo")
config/
  subsystems/      intake.toml  elevator.toml  superstructure.toml: devices + params
  robot.toml       the example robot's manifest, launching cpp/
  robot_py.toml    the same manifest, launching py/ (only the `node =` lines differ)
cpp/               the C++ node implementations, shared library, and test
py/                Python wrappers (via talos.cpp_node) and Python Sim test
```

## Test it

```sh
bazel test //examples/superstructure/...
```

`cpp/superstructure_test.cc` runs all three nodes in one `talos::testing::Sim`,
with `SimHardware` standing in for the bridge and the Driver Station (enabled,
in teleop) and `config/robot.toml` as the config. It walks the whole sequence
by publishing the driver's goal on `/superstructure/target/teleop`
(`PublishTarget`) and flipping the beam break, and it checks the roller
voltages, the elevator's motion-magic demand, `at_goal`, the eject, and the
return to STOWED. It also checks that disabling the robot neutralizes every
motor (the superstructure, losing its own goal, steps back to HOLDING), that a
missing, released or stale goal counts as STOW, and that stale mechanism
statuses release both lanes. Coordinator-only tests cover STOW during stale
RAISING, EJECT release during SCORING when either dependency goes stale,
release/disable/goal expiry with stale statuses, and recovery from fresh
observations without replaying a completed score. The elevator alone goes
neutral on a stale, released or never-sent lane, while disabled, on Driver
Station loss, with the lift disconnected and on a non-finite goal, and clamps
goals to its travel at both ends. Its homing tests start it unzeroed and home
it on a live target before it reaches the goal, keep homing through a
one-tick closed reading mid-travel, re-home it after
`SimHardware::reset_device` at height without applying a motion-magic goal in
the restarted frame, keep it still while disabled or with a non-finite,
stale, released or never-sent lane, and latch HOMING_FAILED on the timeout
(kept through a stale-link glitch, cleared by a Driver Station disable), on a
stall and on a switch stuck closed. Another test refuses unsafe homing
parameters. The other tests start the lift on its closed switch, which reads
false. The intake alone goes neutral on a stale lane while keeping its state.
`py/superstructure_test.py` covers Python host integration, disabled operation
and target loss with `talos.testing.Sim` and `config/robot_py.toml`; the shared
C++ suite owns the detailed mechanism scenarios.

## Run it in simulation

Check the graph first. This asks every node what it publishes and subscribes
to, lints the whole graph, and exits:

```sh
bazel build //talOS/launcher //talOS/bridge:hardware_node \
  //2026-robot/main_processor/simulation:sim_gateway \
  //examples/superstructure/cpp:all //examples/superstructure/py:all
./bazel-bin/talOS/launcher/launcher --config examples/superstructure/config/robot.toml --describe-only
```

Then run it live against the lightweight sim gateway, which stands in for the
RoboRIO and moves motors straight to their demands:

```sh
./bazel-bin/talOS/launcher/launcher --config examples/superstructure/config/robot.toml \
  --sim --start-sim-gateway --duration-s 5
```

Use `--config examples/superstructure/config/robot_py.toml` in either command
to run the Python nodes instead. Both configs declare the same graph.

The gateway supplies enabled teleop Driver Station packets by default, but
nothing in this example publishes a superstructure goal. Without a goal,
the coordinator requests intake IDLE and elevator height 0 only while both
mechanism statuses are fresh; otherwise it releases both mechanism lanes.
The lightweight gateway reports every digital input false, which the
elevator reads as its bottom switch closed, so it zeroes at once where the
lift starts. Under the RoboRIO program in WPILib simulation, an input nothing
drives reads true, as an unwired pull-up does, so there the elevator homes
until its stall or timeout stop.


## Write your own mechanism

Start from a building block: most mechanisms are a
[`talOS/mechanisms`](../../talOS/mechanisms/README.md) class plus a target
lane and a state publisher, as `cpp/elevator.h` and `cpp/intake.h` are; the
library's README has a minimal node. Write the control yourself only for
what no block covers.

The scaffolder below still generates a self-contained node (TODO.md MECH-3).
Generate a position-controlled wrist package from the repo root:

```sh
bazel run //tools:new_subsystem -- wrist --lang cpp --type position
```

The default robot is `2026-robot`; use `--robot DIR` to select another robot
and `--dry-run` to inspect the changes without writing them. Use `--lang py`
for a Python implementation. Omitting `--type position` generates a velocity
mechanism, not an angle-controlled wrist.

The generator writes `2026-robot/main_processor/wrist/`:

```text
BUILD
wrist_message.fbs       wrist.WristTarget(position_rot), wrist.WristState,
                        wrist.WristHoming
wrist.h                 wrist::Wrist
wrist.cc                TALOS_NODE entry point
node_test.cc
testdata/robot.toml
testdata/wrist.toml
```

The package's `:messages` target uses `include_prefix = "wrist"` and declares
`/wrist/state`; its node target is `:node`. The position mechanism uses
`wrist_motor`, Motion Magic, and `tolerance_rot` (default `0.05`).
It drives only a finite target with connected motor feedback. It homes the way
the elevator does (see [Homing](#homing)), with the same switch wiring,
`home_samples` debounce, failure latches and parameter checks: a home switch, `wrist_home`, at the
reverse end of travel zeroes it, `position_frame = "rezero"` holds its goals
until then, and `/wrist/state` reports HOMING, HOMED or FAILED. Targets clamp to
`0..max_position_rot`; the motor has no hardware soft limits.

The subsystem file is
`2026-robot/main_processor/configuration/subsystems/wrist.toml`: it contains
`node = "//2026-robot/main_processor/wrist:node"`, a 20000 µs node period,
a TalonFX with the lowest unused CAN id in `1..62` and a home switch with the
lowest unused DIO channel in `0..31`, both explicitly marked as placeholders.
The rest of its homing setup is placeholder too: `max_position_rot = 0.001`
confines every target to within 0.001 rotations of the switch until you set
the measured travel, and `homing_v = -0.5` (with `homing_timeout_us`, 3 s by
default) needs its direction checked before the first enable. The
generator appends the subsystem name and path to
`configuration/robot.toml` and regenerates that config's manifest plus
manifests of configs that include it.

The Python variant puts `wrist.py` and `node_test.py` in the same package,
not in a separate `py/` directory. It uses the same schema and `:node` target.
For implementing and wiring additional behavior, use the
[node development guide](../../docs/NODE_DEVELOPMENT_GUIDE.md).


