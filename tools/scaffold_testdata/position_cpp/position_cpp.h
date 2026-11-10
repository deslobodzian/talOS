#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <optional>
#include <stdexcept>
#include <string>

#include "position_cpp/position_cpp_message_generated.h"
#include "position_cpp/topics.h"
#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

namespace position_cpp {

// Drives the motor to the position its target asks for with Motion Magic
// and reports what it did on /position_cpp/state. Producers write lanes
// /position_cpp/target/<source> with talos::TargetLane; the policy comes from
// [robot.targets] or the subsystem's [targets].
//
// Positions count from the home switch at the reverse end of travel
// (position_cpp_home), not from where the motor powered on. The motor keeps
// position_frame = "rezero", so its Motion Magic goals go out neutral until the
// switch has zeroed it (Motor::rezero), and again after a motor reset or a
// RoboRIO reboot drops that zero. While unzeroed with a live, finite target it
// homes: it drives toward the switch at homing_v (negative: toward the reverse
// end; a voltage request, which a dropped zero does not hold) until the switch
// has read closed on home_samples consecutive ticks (default 3), then rezeroes
// to 0 and goes to the target. One closed sample, a glitch mid-travel, does not
// zero it: homing keeps driving and any open sample restarts the count. Targets
// clamp to 0..max_position_rot in the homed frame; the motor has no hardware
// soft limits, which would act in its power-on frame and could stop homing.
//
// The switch is normally open, wired to ground, read through the input's
// pull-up: closed reads false, and an open or cut wire reads "not home", so a
// wiring fault makes homing fail rather than zero mid-travel.
//
// FAILED latches, neutral, when homing has not found the switch within
// homing_timeout_us, when the motor has not moved (|velocity| below
// homing_stall_rps) for homing_stall_us while homing pushes it, or when the
// switch reads closed with the zeroed motor more than home_margin_rot from
// it (a stuck switch). It clears only when the Driver Station reports
// disabled, not when the hardware link merely goes stale.
class PositionCpp {
 public:
  using Target = PositionCppTarget;
  using State = PositionCppState;
  using Homing = PositionCppHoming;

  explicit PositionCpp(talos::Node& node)
      : node_{node},
        motor_{node, "position_cpp_motor"},
        home_{node, "position_cpp_home"},
        ds_{node},
        tolerance_rot_{node.param<double>("tolerance_rot", 0.05)},
        max_position_rot_{node.param<double>("max_position_rot")},
        homing_volts_{node.param<double>("homing_v")},
        homing_timeout_{
            std::chrono::microseconds{node.param<int>("homing_timeout_us")}},
        stall_rps_{node.param<double>("homing_stall_rps")},
        stall_time_{
            std::chrono::microseconds{node.param<int>("homing_stall_us")}},
        home_margin_rot_{node.param<double>("home_margin_rot")},
        home_samples_{node.param<int>("home_samples", 3)},
        target_{node},
        state_{node.publish<State>(topics::kState)} {
    Check(max_position_rot_ > 0 && std::isfinite(max_position_rot_),
          "max_position_rot must be finite and positive");
    Check(homing_volts_ < 0 && std::isfinite(homing_volts_),
          "homing_v must be negative (toward the reverse end)");
    Check(homing_timeout_ > talos::Duration::zero(),
          "homing_timeout_us must be positive");
    Check(stall_rps_ > 0 && std::isfinite(stall_rps_),
          "homing_stall_rps must be finite and positive");
    Check(stall_time_ > talos::Duration::zero(),
          "homing_stall_us must be positive");
    Check(home_margin_rot_ > 0 && std::isfinite(home_margin_rot_),
          "home_margin_rot must be finite and positive");
    Check(home_samples_ >= 1, "home_samples must be at least 1");
  }

  void update() {
    const Target* t = target_.get();  // null: disabled, stale or released
    const bool connected = motor_.connected();
    const bool at_home = !home_.get();  // normally open, to ground
    // A Driver Station disable is the operator's reset.
    if (ds_.connected() && !ds_.enabled()) {
      failed_ = false;
      homing_since_.reset();
      stalled_since_.reset();
    }
    // Debounced: home_samples consecutive closed readings while unzeroed.
    home_count_ = !motor_.zeroed() && at_home ? home_count_ + 1 : 0;
    if (home_count_ >= home_samples_) motor_.rezero(0.0);  // no-op if stale
    const double pos = motor_.position();
    if (motor_.zeroed()) {
      homing_since_.reset();
      stalled_since_.reset();
      if (connected && at_home && pos > home_margin_rot_) failed_ = true;
    }
    if (failed_ || !motor_.zeroed()) {
      const bool drive = !failed_ && t != nullptr && connected &&
                         std::isfinite(t->position_rot()) && home();
      if (!drive) stalled_since_.reset();  // counts only while pushing
      state_.send(State{drive, pos, connected, false,
                        failed_ ? Homing::FAILED : Homing::HOMING});
      return;
    }
    const bool drive =
        t != nullptr && connected && std::isfinite(t->position_rot());
    bool at_goal = false;
    if (drive) {
      const double goal = std::clamp(t->position_rot(), 0.0, max_position_rot_);
      motor_.set_motion_magic(goal);
      at_goal = std::abs(pos - goal) <= tolerance_rot_;
    }
    state_.send(State{drive, pos, connected, at_goal, Homing::HOMED});
  }

 private:
  static void Check(bool ok, const char* what) {
    if (!ok) throw std::invalid_argument(std::string{"position_cpp: "} + what);
  }

  // One homing tick: drives toward the switch unless homing has timed out
  // (counted from the first homing command since the last zero or Driver
  // Station disable; losing the target pauses the motion, not that clock) or
  // stalled. Returns whether it drove.
  bool home() {
    const talos::Time now = node_.now();
    if (!homing_since_) homing_since_ = now;
    if (std::abs(motor_.velocity()) >= stall_rps_)
      stalled_since_.reset();
    else if (!stalled_since_)
      stalled_since_ = now;
    if (now - *homing_since_ >= homing_timeout_ ||
        (stalled_since_ && now - *stalled_since_ >= stall_time_)) {
      failed_ = true;
      return false;
    }
    motor_.set_voltage(homing_volts_);
    return true;
  }

  talos::Node& node_;
  talos::Motor motor_;
  talos::DigitalInput home_;
  talos::DriverStation ds_;
  double tolerance_rot_;
  double max_position_rot_;
  double homing_volts_;
  talos::Duration homing_timeout_;
  double stall_rps_;
  talos::Duration stall_time_;
  double home_margin_rot_;
  int home_samples_;
  int home_count_ = 0;
  std::optional<talos::Time> homing_since_;
  std::optional<talos::Time> stalled_since_;
  bool failed_ = false;
  talos::Target<Target> target_;
  talos::Publisher<State> state_;
};

}  // namespace position_cpp
