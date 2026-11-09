# Superstructure example

An intake, an elevator, and a superstructure that sequences them, written as
three **separate nodes** that talk only over topics. Goals travel on target
lanes; see [target policy and safety](../../ARCHITECTURE.md).

```
 /superstructure/target/teleop ─┐
 /superstructure/target/auto   ─┴▶ superstructure ──▶ /intake/target/superstructure   ──▶ intake   (roller + beam break)
                                       ▲  ▲        ──▶ /elevator/target/superstructure ──▶ elevator (one motion-magic TalonFX)
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


- **intake**: IDLE → INTAKING (roller in) → HOLDING once the beam breaks (small
  hold voltage) → EJECTING (reverse for `eject_us`, 500000 µs by default) →
  IDLE. Roller voltages are `intake_v`, `hold_v`, and `eject_v`. With no
  target it behaves as asked for IDLE with the roller neutral; the state
  machine still tracks the beam break and finishes a timed eject.
- **elevator**: motion magic to the goal height while it has a target. It
  reports STOWED at 0, MOVING, or AT_GOAL (within `tolerance_rot`), and echoes
  the goal that `at_goal` refers to. With no target the lift is neutral (it
  never holds an old goal); its status echoes goal 0 with `at_goal` false, and
  reads STOWED within `tolerance_rot` of 0, MOVING elsewhere.
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

Every tunable (voltages, eject time, tolerance, height) is a parameter read
from the robot config: `node.param<T>("key")` in C++, `node.param("key")` in
Python.

The nodes are implemented once in C++ (`cpp/`) and wrapped in Python (`py/`)
via `TALOS_EXPORT_NODE` and `talos.cpp_node`, so both sets of binaries and
tests run the exact same implementation without duplicating logic.

For node APIs and message/build conventions, see the
[node development guide](../../docs/NODE_DEVELOPMENT_GUIDE.md).

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
Station loss, with the lift disconnected and on a non-finite goal; the intake
alone goes neutral on a stale lane while keeping its state.
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


## Write your own mechanism

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
wrist_message.fbs       wrist.WristTarget(position_rot), wrist.WristState
wrist.h                 wrist::Wrist
wrist.cc                TALOS_NODE entry point
node_test.cc
testdata/robot.toml
testdata/wrist.toml
```

The package's `:messages` target uses `include_prefix = "wrist"` and declares
`/wrist/state`; its node target is `:node`. The position mechanism uses
`wrist_motor`, Motion Magic, and `tolerance_rot` (default `0.05`).
It drives only a finite target with connected motor feedback.

The subsystem file is
`2026-robot/main_processor/configuration/subsystems/wrist.toml`: it contains
`node = "//2026-robot/main_processor/wrist:node"`, a 20000 µs node period,
and a TalonFX with the lowest unused CAN id in `1..62`, explicitly marked
as a placeholder. The generator appends the subsystem name and path to
`configuration/robot.toml` and regenerates that config's manifest plus
manifests of configs that include it.

The Python variant puts `wrist.py` and `node_test.py` in the same package,
not in a separate `py/` directory. It uses the same schema and `:node` target.
For implementing and wiring additional behavior, use the
[node development guide](../../docs/NODE_DEVELOPMENT_GUIDE.md).


