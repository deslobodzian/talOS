# talOS/mechanisms

Configured building blocks for the common FRC mechanisms, so a subsystem node
is a thin wrapper: a target lane in, a state out, and one library object that
does the control, homing, readiness and fault handling.

| Class | For | State message |
|---|---|---|
| `PositionMechanism` | elevator, arm, hood, wrist, turret | `control::PositionState` |
| `VelocityMechanism` | flywheel, roller, conveyor | `control::VelocityState` |
| `RollerWithSensor` | intake, indexer with a beam break | phase + `has_piece()` (no shared message) |

The layer sits between `talOS/devices` and robot code and uses only the
devices layer's public API (`Motor`, `DigitalInput`, `CANcoder`,
`DriverStation`, `robot_enabled`). The messages are `//talOS/msgs:control`.
Python has no binding yet (TODO.md MECH-2); a Python robot uses a C++ node
built on these classes through `TALOS_EXPORT_NODE` and `talos.cpp_node`, as
`examples/superstructure/py/` does.

## Using one

Each class is a member of a node, constructed from the `talos::Node&` and a
name. The name is both the motor (`[motors.<name>]`) and the parameter table
(`[<name>]` in the subsystem file, `[subsystems.<sub>.<name>]` in a
single-file config). Every tick the node calls `set_goal(...)` (or `hold()`)
only while its `talos::Target` is live, otherwise `stop()`, then `update()`:

```cpp
#include "control/control_generated.h"
#include "talOS/devices/devices.h"
#include "talOS/mechanisms/position_mechanism.h"

class Wrist {
 public:
  explicit Wrist(talos::Node& node)
      : wrist_{node, "wrist"},
        target_{node},
        state_{node.publish<control::PositionState>("state")} {}
  void update() {
    if (const control::PositionTarget* t = target_.get()) {
      wrist_.set_goal(t->position_rot(), t->feedforward_v());
    } else {
      wrist_.stop();
    }
    wrist_.update();
    state_.send(wrist_.state());
  }

 private:
  talos::mechanisms::PositionMechanism wrist_;
  talos::Target<control::PositionTarget> target_;
  talos::Publisher<control::PositionState> state_;
};
TALOS_NODE(Wrist)
```

```toml
[subsystem]
name = "wrist"

[wrist]
tolerance_rot = 0.01
min_rot = 0.0          # a homed mechanism needs a finite travel
max_rot = 0.4          # inside the sensor rotation around offset 0
absolute_margin_rot = 0.05  # hard stops and overshoot beyond the travel
homing = "absolute"
absolute_sensor = "angle"

[motors.wrist]
can_id = 3

[sensors.angle]
type = "CANcoder"
can_id = 4
```

Build it with `talos_cc_node(..., deps = ["//talOS/mechanisms", "//talOS/msgs:control"])`.
`examples/superstructure/cpp/elevator.h` and `intake.h` are complete nodes
built this way.

## Safety

The framework's rules hold unchanged (ARCHITECTURE.md "The tick"). A request
(`set_goal`, `hold`, `stop`, `clear_fault`) is accepted only inside the
node's `update()`; from the constructor, `every()` or a message callback it
throws `std::logic_error` naming the mechanism, as a `Motor` setter does
(`talos::RequireInUpdate`). It lasts only the tick it was made in: the
mechanism's `update()` in that same tick consumes it, and one left from a
tick whose `update()` was skipped is dropped. So a tick without `set_goal()`
drives nothing and the devices layer sends neutral. The library never
actuates on its own: no hold after the target goes, no spin-down without a
goal, no homing without a goal. It also drives nothing while
`robot_enabled(node)` is false or on a non-finite goal or feedforward;
`PositionMechanism` and `VelocityMechanism` drive nothing with a
disconnected motor, while `RollerWithSensor` reads no motor feedback and
relies on the gateway, which neutralizes a motor whose feedback is invalid.
It does not read a lane itself: gating on a live `Target` is the owner's call,
as in the example above.

Each class allocates only in its constructor. A bad parameter throws
`std::invalid_argument` naming `<name>.<key>` at construction.

## PositionMechanism

Positions are mechanism rotations as the motor reports them, in its position
frame (NODE_DEVELOPMENT_GUIDE.md "Rezero and position frames"). Phases:
`kUnhomed`, `kHoming`, `kReady`, `kMoving`, `kAtGoal`, `kFaulted`.
`state()` is `control::PositionState` with `goal_rot` 0 and `enabled` false
whenever no goal is driven.

| Key | Unit | Default | Meaning |
|---|---|---|---|
| `tolerance_rot` | rot | required, > 0 | at goal within this |
| `at_goal_debounce_us` | µs | 0 | continuously in tolerance this long before `kAtGoal`; leaving is immediate |
| `min_rot`, `max_rot` | rot | -inf, +inf | goals clamp to this travel (after wrap); required and finite when `wrap_rot > 0` or `homing` is not `"none"` |
| `units_per_rot` | your unit / rot | 1 | for `set_goal_units()` and `position_units()` |
| `wrap_rot` | rot | 0 (off) | a goal goes to its equivalent (`goal + k * wrap_rot`) nearest the current position, inside the travel when one is: a turret (whose finite travel keeps it from winding its cables) |
| `control` | | `"motion_magic"` | or `"position"` (plain position PID) |
| `slot` | | 0 | the controller gain slot, 0 to 2 |
| `gravity` | | `"elevator"` | `"elevator"`: `+gravity_v` always; `"arm"`: `gravity_v * cos(2π (position - gravity_zero_rot))` |
| `gravity_v` | V | 0 | added to the target's feedforward; refused at construction while the motor's `slot<slot>` `g` is nonzero (one gravity source) |
| `gravity_zero_rot` | rot | 0 | arm position where it is level on the positive side |
| `homing` | | `"none"` | see below |
| `stall_time_us` | µs | 0 (off) | short of the goal with `|velocity| <= stall_velocity_rps` and `|current| >= stall_current_a` this long: fault `kStall` |
| `stall_velocity_rps` | rot/s | 0.05 | |
| `stall_current_a` | A | 0 | 0 judges by velocity alone |
| `move_timeout_us` | µs | 0 (off) | one goal (a change beyond `tolerance_rot` restarts it) not reached this long: fault `kMoveTimeout` |

Homing sets the frame; until it holds the mechanism drives no goal. Only
`set_goal()` starts homing: `hold()` on an unhomed mechanism is neutral
(`kUnhomed`), since there is no position to hold. A homing `rezero` that
`Motor::rezero` refuses for a late motor position, or an absolute homing
whose two samples are too far apart (both `RezeroAgeBound`, NODE_DEVELOPMENT_GUIDE.md
"Rezero and position frames"), leaves the mechanism unhomed and is retried
on the next tick; a stall push keeps pushing meanwhile.

| `homing` | Frame from | Keys |
|---|---|---|
| `"none"` | the motor's absolute frame; construction refuses a `"rezero"`-frame motor, which would never drive | |
| `"limit_switch"` | `rezero(home_rot)` after `home_samples` consecutive closed readings while unzeroed; zeroes an idle mechanism too | `limit_switch` (digital input name), `limit_switch_closed_reads` (bool, default false: normally open to ground behind the pull-up, so a cut wire reads open), `home_samples` (default 3), `home_margin_rot` (> 0), plus the drive keys |
| `"stall"` | `rezero(home_rot)` once the push has held the mechanism still (`|velocity| < homing_stall_rps`) under load (`|current()| >= homing_stall_current_a`, stator amps) for `homing_stall_us`, counted only once the current push has moved it `homing_min_travel_rot` from where that push began or has lasted `homing_min_push_us`, so a slow start is not taken for the stop; a goal lost mid-push ends the push, and its motion does not count for the next, and a one-tick position step faster than the motor's `max_velocity_rps` allows (a frame jump, not motion) starts the push over | the drive keys, `homing_stall_current_a` (A, required, > 0 and below the motor's `stator_limit_a`), `homing_min_push_us` (µs, required, > 0), `homing_min_travel_rot` (rot, required, > 0 and less than the travel); `homing_timeout_us` must exceed `homing_min_push_us + homing_stall_us` plus two node periods. Since stillness under load after the minimum push is taken as the stop, `homing_v` must be enough to move the mechanism against its worst-case load (friction, gravity, a held piece) anywhere in its travel; a push too weak to move it homes wherever it sticks |
| `"absolute"` | `rezero(sensor * absolute_rot_per_sensor_rot + absolute_offset_rot)` whenever unzeroed and both read, with acquisition times (`position_time()`) known and at most `RezeroAgeBound` apart, without motion | `absolute_sensor` (CANcoder name), `absolute_rot_per_sensor_rot` (default 1), `absolute_offset_rot` (rot, default 0), `absolute_margin_rot` (rot, required, > 0). The RoboRIO configures every CANcoder's discontinuity at 0.5, so it reads absolute position in [-0.5, 0.5) rotations (`PhoenixBackend.cpp`); the frame can name only the one sensor rotation `absolute_offset_rot ± 0.5·|absolute_rot_per_sensor_rot|`. The whole physical range must lie strictly inside it: the travel `min_rot..max_rot` widened by `absolute_margin_rot` at each end, which must cover the hard stops and any overshoot past the travel, since a mechanism resting just past the window reads across the discontinuity and takes a frame a whole sensor rotation away. The simulators do not wrap a CANcoder reading at 0.5 (TODO.md MECH-4), so the check is what keeps a sim-tested travel valid on the robot |

Stall homing reads stator current. Component tests set it with
`SimHardware::set_motor(..., stator_current_a)`. The physics simulator
(`sim_gateway --physics`) models it with back-EMF, so it peaks at stall and
falls with speed, but scaled so that 12 V at stall draws `stator_limit_a`: a
homing push of a volt or two draws a small fraction of the limit, so stall
homing with a realistic `homing_stall_current_a` does not home there
(TODO.md MECH-5).

Drive keys, all required except `home_rot`: `homing_v` (V, nonzero, at most
half the motor's `max_voltage`, pointing at `home_rot`, which must be the
travel end that way: `min_rot` for a negative `homing_v`, `max_rot` for a
positive one), `home_rot` (rot, default 0), `homing_timeout_us` (> 0),
`homing_stall_rps` (rot/s, > 0), `homing_stall_us` (> 0). Homing drives
`homing_v` only on a live goal; losing the goal ends the push (its motion
evidence starts over with the next) but not the timeout, which counts from the first push since the last zero or Driver
Station disable. The timeout always faults; it never homes. A device reset or
a RoboRIO reboot that drops the zero sends the mechanism back to homing.

Faults latch neutral (`kFaulted`) until `clear_fault()` or a Driver Station
that reports disabled; a stale link does not clear them:
`kHomingTimeout`; `kHomingStall` (limit-switch homing pushed a mechanism
slower than `homing_stall_rps` for `homing_stall_us`); `kSwitchStuck` (the
switch reads closed with the homed mechanism more than `home_margin_rot` into
its travel); `kStall`; `kMoveTimeout`.

`hold()` drives to the position the mechanism had when the current run of
`hold()` calls began; it too needs a live target.

## VelocityMechanism

Velocities are mechanism rot/s. Goals clamp to the motor's own
`max_velocity_rps`. Phases: `kStopped`, `kSpinningUp`, `kAtSpeed`. `state()`
is `control::VelocityState` (`requested_rps` is the clamped goal).

| Key | Unit | Default | Meaning |
|---|---|---|---|
| `at_speed_entry_rps` | rot/s | 2.0 | entry band around the clamped goal |
| `at_speed_entry_tick_count` | ticks | 3 | consecutive ticks in the entry band to become at speed |
| `at_speed_exit_frac` | fraction | 0.1, in (0, 1] | stays at speed while the error is within `max(at_speed_entry_rps, at_speed_exit_frac * |goal|)` |
| `spin_down_rps2` | rot/s² | 0 (off) | a live goal smaller than (or opposite to) the last command is approached at most this fast; losing the goal is neutral at once |
| `slot` | | 0 | gain slot |

A goal of at most 1e-3 rot/s is never at speed, and a change of the asked
goal restarts the entry count. These are the 2026 shooter's readiness rules.
`VelocityMechanism` drives nothing with a disconnected motor.

## RollerWithSensor

A voltage roller and a digital piece sensor. Commands: `kIdle`, `kIntake`,
`kEject`, `kFeed`. Phases: `kIdle` (neutral), `kIntaking` (`intake_v` until
a piece), `kHolding` (`hold_v`, or neutral without one, while the piece
stays), `kEjecting` (`eject_v` for `eject_us`), `kFeeding` (`feed_v` for
`feed_us`). A run starts from `kIntaking` or `kHolding` when asked, and from
`kIdle` only with `idle_runs` and only when newly asked: a command that
differs from the last tick's request, where a tick with no live request (a
release or a stale lane) counts as `kIdle`. A held button runs once; press,
release, press runs twice. Refusing a press held through a disable or a
stale spell is the producer's job (`talos::OperatorArming`, which teleop
producers use). With no
live request the roller is neutral and the phases carry on as if asked for
`kIdle`: a held piece stays `kHolding` and a timed run finishes its clock.
Losing enable (`robot_enabled` false: a Driver Station disable, a stale
link) cancels intaking and any timed run, back to `kIdle`, so a re-enable
within `eject_us` does not resume it; a held piece stays `kHolding`.

| Key | Unit | Default | Meaning |
|---|---|---|---|
| `sensor` | | required | the `[digital_inputs]` name |
| `sensor_inverted` | | false | true when the input reads false with a piece |
| `sensor_debounce_us` | µs | 0 | a piece must read this long before `has_piece()`; losing it is immediate |
| `intake_v` | V | required | |
| `hold_v` | V | none (neutral) | |
| `eject_v` | V | required | |
| `eject_us` | µs | required, > 0 | |
| `feed_v` | V | `intake_v` | |
| `feed_us` | µs | `eject_us` | |
| `idle_runs` | | false | an eject or feed newly asked in `kIdle` starts a run |

The roller reads no motor feedback: a motor whose feedback is invalid is
neutralized by the gateway, not by the roller.

## Tests

```sh
bazel test //talOS/mechanisms:all
```

One component test per class under `testing::Sim` and `SimHardware`, with
the library's share of the node test matrix (docs/TESTING.md §7), requests
refused outside `update()` and dropped after a skipped one, and each
feature: stall homing (a stop after motion, never moving, a slow start, the
timeout), absolute homing (including a reset), upward limit-switch homing and
a stuck switch, homing paused without a goal, hold on an unhomed mechanism,
gravity for elevator and arm, turret wrap, at-goal and at-speed edges,
spin-down, faults, the roller's phases, a disable cancelling a timed run,
and a held eject running once while release-then-press runs again. Downward limit-switch homing and the
travel clamp are also tested through `examples/superstructure`'s elevator.
The tests carry the tags of REQ-SAFE-36 (`PositionMechanism`), REQ-SAFE-37 (`VelocityMechanism`) and
REQ-SAFE-38 (`RollerWithSensor`).
