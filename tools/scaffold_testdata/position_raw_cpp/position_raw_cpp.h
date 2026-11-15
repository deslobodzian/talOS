#pragma once

#include <algorithm>
#include <cmath>

#include "position_raw_cpp/position_raw_cpp_message_generated.h"
#include "position_raw_cpp/topics.h"
#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

namespace position_raw_cpp {

// Drives the mechanism to the position its target asks for and reports
// what it did on /position_raw_cpp/state. Producers write lanes
// /position_raw_cpp/target/<source> with talos::TargetLane; the
// policy comes from [robot.targets] or the subsystem's [targets].
//
// Every line of the control is here, in this file: it homes on the switch
// position_raw_cpp_home, then runs Motion Magic inside a fixed travel.
// Positions count from that switch, not from where the motor powered on.
// Until the switch has zeroed the motor, and again after a motor reset or a
// RoboRIO reboot drops that zero, a live goal pushes toward the switch at
// kHomingVolts instead. A push that runs past its bound, or the switch still
// reading closed after the first move off it, latches FAILED, neutral, until
// the Driver Station reports disabled. docs/FIRST_MECHANISM.md §6 in the
// talos repository explains each rule.
class PositionRawCpp {
 public:
  using Target = PositionRawCppTarget;
  using State = PositionRawCppState;
  using Homing = PositionRawCppHoming;

  explicit PositionRawCpp(talos::Node& node)
      : motor_{node, "position_raw_cpp"},
        home_{node, "position_raw_cpp_home"},
        ds_{node},
        target_{node},
        state_{node.publish<State>(topics::kState)} {}

  void update() {
    // The switch reads false when closed, true when open.
    const bool on_switch = home_.connected() && !home_.get();
    const bool off_switch = home_.connected() && home_.get();
    // Zero on the switch once the motor is at rest, so the first move does
    // not start at homing speed.
    if (!motor_.zeroed() && on_switch &&
        std::abs(motor_.velocity()) < kRestRps) {
      motor_.rezero(0.0);
    }
    if (motor_.zeroed()) {
      push_ticks_ = 0;  // homed: a later re-home pushes from zero
      if (off_switch) left_switch_ = true;
    } else {
      left_switch_ = false;  // a dropped zero homes and checks again
    }
    // Only a Driver Station that reports disabled clears a failure: not a
    // release, a stale lane or a feedback blip, any of which would otherwise
    // start another push into the stop.
    if (ds_.connected() && !ds_.enabled()) {
      failed_ = false;
      push_ticks_ = 0;
    }
    // Still closed after the first move off it: the switch is stuck, or the
    // zero is wrong. Stop before the move reaches kHomeMarginRot.
    if (motor_.zeroed() && !left_switch_ &&
        motor_.position() > kHomeMarginRot - kToleranceRot) {
      failed_ = true;
    }
    const Target* t = target_.get();  // null: disabled, stale or released
    const bool live = t != nullptr && motor_.connected() &&
                      std::isfinite(t->position_rot()) && !failed_;
    bool drove = false;
    double goal = 0.0;
    if (live && motor_.zeroed()) {
      // Until the switch has opened, no farther than kHomeMarginRot.
      const double top = left_switch_ ? kMaxRot : kHomeMarginRot;
      goal = std::clamp(t->position_rot(), 0.0, top);
      motor_.set_motion_magic(goal);
      drove = true;
    } else if (live && !on_switch) {
      // The push is bounded: a switch that never closes (a cut wire reads
      // open) would otherwise push into the stop while the button is held.
      if (++push_ticks_ > kMaxPushTicks) {
        failed_ = true;  // pushed long enough to cross the travel: no switch
      } else {
        motor_.set_voltage(kHomingVolts);  // toward the switch
        drove = true;
      }
    }
    // Otherwise nothing is set and the flush sends neutral.
    const bool at_goal = drove && motor_.zeroed() &&
                         std::abs(motor_.position() - goal) < kToleranceRot;
    Homing homing = motor_.zeroed() ? Homing::HOMED : Homing::HOMING;
    if (failed_) homing = Homing::FAILED;
    state_.send(
        State{drove, motor_.position(), motor_.connected(), at_goal, homing});
  }

 private:
  // PLACEHOLDER values, sized for the simulation new_subsystem's `Next:`
  // describes. Set each for your mechanism before the robot drives it.
  //
  // Travel above the switch, in rotations: goals clamp to 0..kMaxRot.
  // Measure it on the mechanism.
  static constexpr double kMaxRot = 2.0;
  // The homing push, in volts; negative drives toward the switch. Keep it
  // low: with the motor's stator_limit_a it bounds how hard the push meets
  // the stop. Check its direction before the first enable.
  static constexpr double kHomingVolts = -0.5;
  // The push bound, in ticks of the subsystem's period_us: time a push
  // across the full travel at kHomingVolts and allow about twice that.
  // 50 ticks is 1 s at period_us = 20000.
  static constexpr int kMaxPushTicks = 50;
  // The switch must read open by here after each zero, in rotations: the
  // distance from the zero at which it reliably opens, plus a little. Keep
  // it small: from a wrong zero the mechanism can still move this far.
  static constexpr double kHomeMarginRot = 0.15;
  // Within this of the goal, /position_raw_cpp/state reports at_goal.
  static constexpr double kToleranceRot = 0.05;
  // Slower than this the motor is at rest, and the switch may zero it.
  static constexpr double kRestRps = 0.05;

  talos::Motor motor_;
  talos::DigitalInput home_;
  talos::DriverStation ds_;
  talos::Target<Target> target_;
  talos::Publisher<State> state_;
  bool failed_ = false;
  bool left_switch_ = false;
  int push_ticks_ = 0;
};

}  // namespace position_raw_cpp
