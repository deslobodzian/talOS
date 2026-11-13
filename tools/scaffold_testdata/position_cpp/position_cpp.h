#pragma once

#include "position_cpp/position_cpp_message_generated.h"
#include "position_cpp/topics.h"
#include "talOS/devices/devices.h"
#include "talOS/mechanisms/position_mechanism.h"
#include "talOS/node/node.h"

namespace position_cpp {

// Drives the mechanism to the position its target asks for and reports
// what it did on /position_cpp/state. Producers write lanes
// /position_cpp/target/<source> with talos::TargetLane; the
// policy comes from [robot.targets] or the subsystem's [targets].
//
// The control is a talos::mechanisms::PositionMechanism, configured by the
// subsystem file: the motor [motors.position_cpp] and the
// parameter table [position_cpp] (keys and units in
// talOS/mechanisms/README.md). It runs Motion Magic to the goal, clamped to
// min_rot..max_rot, and homes on the switch position_cpp_home:
// positions count from that switch, not from where the motor powered on.
// Until the switch has zeroed the motor, and again after a motor reset or a
// RoboRIO reboot drops that zero, a live goal drives toward the switch at
// homing_v instead. A homing timeout, a homing push that does not move, or
// the switch reading closed well into the travel (stuck) latches FAILED,
// neutral, until the Driver Station reports disabled.
class PositionCpp {
 public:
  using Target = PositionCppTarget;
  using State = PositionCppState;
  using Homing = PositionCppHoming;

  explicit PositionCpp(talos::Node& node)
      : mechanism_{node, "position_cpp"},
        target_{node},
        state_{node.publish<State>(topics::kState)} {}

  void update() {
    // The mechanism drives only on this tick's request. Without a live
    // target (disabled, stale, released, another mode's lane) stop(): the
    // motor goes neutral rather than holding an old goal.
    if (const Target* t = target_.get()) {
      mechanism_.set_goal(t->position_rot());  // homes first; NaN: neutral
    } else {
      mechanism_.stop();
    }
    mechanism_.update();
    state_.send(State{mechanism_.driving(), mechanism_.position_rot(),
                      mechanism_.connected(), mechanism_.at_goal(),
                      HomingState()});
  }

 private:
  Homing HomingState() const {
    using Phase = talos::mechanisms::PositionPhase;
    if (mechanism_.phase() == Phase::kFaulted) return Homing::FAILED;
    return mechanism_.homed() ? Homing::HOMED : Homing::HOMING;
  }

  talos::mechanisms::PositionMechanism mechanism_;
  talos::Target<Target> target_;
  talos::Publisher<State> state_;
};

}  // namespace position_cpp
