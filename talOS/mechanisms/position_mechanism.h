#pragma once

// talos::mechanisms::PositionMechanism: one motor driven to a position on its
// controller's motion-magic (or plain position) loop. An elevator, arm, hood,
// wrist or turret, configured rather than written.
//
//   class Arm {
//    public:
//     explicit Arm(talos::Node& node)
//         : arm_{node, "arm"}, target_{node},
//           state_{node.publish<control::PositionState>("state")} {}
//     void update() {
//       if (const control::PositionTarget* t = target_.get()) {
//         arm_.set_goal(t->position_rot(), t->feedforward_v());
//       } else {
//         arm_.stop();
//       }
//       arm_.update();
//       state_.send(arm_.state());
//     }
//    private:
//     talos::mechanisms::PositionMechanism arm_;
//     talos::Target<control::PositionTarget> target_;
//     talos::Publisher<control::PositionState> state_;
//   };
//
// The motor is [motors.<name>]; the parameters are the [<name>] table of the
// subsystem (keys and units in talOS/mechanisms/README.md). Positions are
// mechanism rotations, as the motor reports them (the controller applies
// sensor_to_mechanism_ratio), in the motor's position frame: an absolute-frame
// motor's from the first state, a rezero-frame motor's once homing has set it
// (Motor::rezero, Motor::zeroed). A device reset or RoboRIO reboot that drops
// the zero sends the mechanism back to homing.
//
// Safety is the framework's: set_goal(), hold() and stop() are this tick's
// request and are consumed by update(); a tick without set_goal() or hold()
// drives nothing, so the devices layer sends the motor neutral. Call
// set_goal() only with a live talos::Target (non-null get()). The library
// adds a latched fault on top: once faulted it drives nothing until
// clear_fault() or a Driver Station disable.

#include <cstdint>
#include <optional>
#include <string_view>

#include "control/control_generated.h"
#include "talOS/devices/devices.h"
#include "talOS/mechanisms/params.h"
#include "talOS/node/node.h"

namespace talos::mechanisms {

// What the mechanism is doing, as of the last update().
enum class PositionPhase : std::uint8_t {
  kUnhomed,  // no position frame yet, and not driving home this tick
  kHoming,   // driving homing_v toward home this tick
  kReady,    // homed, no goal this tick (neutral)
  kMoving,   // driving to a goal, not yet at it
  kAtGoal,   // within tolerance_rot of the goal for at_goal_debounce_us
  kFaulted,  // latched neutral until clear_fault() or a Driver Station disable
};

enum class PositionFault : std::uint8_t {
  kNone,
  kHomingTimeout,  // homing pushed for homing_timeout_us without finding home
  kHomingStall,    // limit-switch homing pushed a still mechanism
  kSwitchStuck,    // the home switch reads closed well away from home
  kStall,          // short of the goal and not moving for stall_time_us
  kMoveTimeout,    // one goal not reached within move_timeout_us
};

enum class Homing : std::uint8_t {
  kNone,         // the motor's frame is absolute (position_frame)
  kLimitSwitch,  // drive homing_v until the switch reads closed
  kStall,        // drive homing_v until the mechanism stops at a hard stop
  kAbsolute,     // take the position from a CANcoder, no motion
};

enum class Gravity : std::uint8_t {
  kElevator,  // constant gravity_v
  kArm,       // gravity_v * cos(position - gravity_zero_rot), in turns
};

class PositionMechanism {
 public:
  // Opens [motors.<name>] (and the switch or CANcoder the config names) and
  // reads [<name>]. Throws std::invalid_argument, naming the key, on a
  // missing or invalid parameter.
  PositionMechanism(Node& node, std::string_view name);

  // This tick's request (the last call before update() wins).
  // Drive to `position_rot` (mechanism rotations), with `feedforward_v`
  // added to the gravity term. A non-finite value drives nothing.
  void set_goal(double position_rot, double feedforward_v = 0);
  // The same goal in configured units (position_rot * units_per_rot).
  void set_goal_units(double position_units, double feedforward_v = 0);
  // Drive to the position held when this run of hold() calls began.
  void hold(double feedforward_v = 0);
  // Drive nothing: neutral. The same as not calling set_goal()/hold().
  void stop();

  // Applies this tick's request. Call once from the owning node's update().
  void update();
  // Leaves kFaulted. A Driver Station disable does the same and also
  // restarts the homing clock.
  void clear_fault();

  PositionPhase phase() const { return phase_; }
  PositionFault fault() const { return fault_; }
  // The position frame holds: always for homing = "none", else
  // Motor::zeroed().
  bool homed() const;
  bool at_goal() const { return phase_ == PositionPhase::kAtGoal; }
  // An output (a goal, or homing) went out this tick.
  bool driving() const { return driving_; }
  bool connected() const { return connected_; }
  double position_rot() const { return position_rot_; }
  double position_units() const { return position_rot_ * units_per_rot_; }
  double velocity_rps() const { return velocity_rps_; }
  // The goal commanded this tick after wrap and soft limits; 0 when not
  // driving a goal (stopped, homing, faulted).
  double goal_rot() const { return goal_rot_; }
  // The gravity feedforward added this tick (0 when not driving a goal).
  double gravity_v() const { return gravity_ff_v_; }
  double tolerance_rot() const { return tolerance_rot_; }

  // For /<sub>/state: position, velocity, goal_rot(), enabled = driving(),
  // at_goal(), connected().
  control::PositionState state() const;

 private:
  enum class Request : std::uint8_t { kStop, kGoal, kHold };

  void fault(PositionFault reason);
  void idle(PositionPhase phase);
  void drive_goal(Request request, double request_rot, double request_ff_v,
                  Time now);
  double wrap(double goal) const;
  double gravity_at(double position_rot) const;
  bool drives_home() const {
    return homing_ == Homing::kLimitSwitch || homing_ == Homing::kStall;
  }

  Node* node_;
  Motor motor_;
  DriverStation ds_;
  std::optional<DigitalInput> limit_switch_;
  std::optional<CANcoder> absolute_;

  // Parameters.
  double tolerance_rot_{0};
  Duration at_goal_debounce_{};
  double min_rot_{0}, max_rot_{0};
  double units_per_rot_{1};
  double wrap_rot_{0};
  Gravity gravity_{Gravity::kElevator};
  double gravity_v_{0};
  double gravity_zero_rot_{0};
  bool motion_magic_{true};
  int slot_{0};
  Homing homing_{Homing::kNone};
  double homing_v_{0};
  double home_rot_{0};
  Duration homing_timeout_{};
  double homing_stall_rps_{0};
  Duration homing_stall_time_{};
  bool switch_closed_reads_{false};
  int home_samples_{1};
  double home_margin_rot_{0};
  double absolute_rot_per_sensor_rot_{1};
  double absolute_offset_rot_{0};
  double stall_velocity_rps_{0};
  double stall_current_a_{0};
  Duration stall_time_{};
  Duration move_timeout_{};

  // This tick's request.
  Request request_{Request::kStop};
  double request_rot_{0};
  double request_ff_v_{0};

  // State.
  PositionPhase phase_{PositionPhase::kUnhomed};
  PositionFault fault_{PositionFault::kNone};
  bool driving_{false};
  bool connected_{false};
  double position_rot_{0};
  double velocity_rps_{0};
  double goal_rot_{0};
  double gravity_ff_v_{0};
  int home_count_{0};  // consecutive closed switch readings while unzeroed
  std::optional<Time> homing_since_;  // first push since a zero or disable
  std::optional<Time> homing_still_since_;
  std::optional<double> held_rot_;
  std::optional<double> goal_anchor_rot_;  // move timer restarts past it
  Time goal_since_{};
  Debounce at_goal_debounce_state_;
  Debounce stall_debounce_;
};

}  // namespace talos::mechanisms
