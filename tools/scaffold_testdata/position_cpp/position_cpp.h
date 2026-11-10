#pragma once

#include <cmath>

#include "position_cpp/position_cpp_message_generated.h"
#include "position_cpp/topics.h"
#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

namespace position_cpp {

// Drives the motor to the position its target asks for with Motion Magic
// and reports what it did on /position_cpp/state. Producers write lanes
// /position_cpp/target/<source> with talos::TargetLane; the policy comes from
// [robot.targets] or the subsystem's [targets].
class PositionCpp {
 public:
  using Target = PositionCppTarget;
  using State = PositionCppState;

  explicit PositionCpp(talos::Node& node)
      : motor_{node, "position_cpp_motor"},
        tolerance_rot_{node.param<double>("tolerance_rot", 0.05)},
        target_{node},
        state_{node.publish<State>(topics::kState)} {}

  void update() {
    const Target* t = target_.get();  // null: disabled, stale or released
    const bool drive =
        t != nullptr && motor_.connected() && std::isfinite(t->position_rot());
    const double pos = motor_.position();
    bool at_goal = false;
    if (drive) {
      motor_.set_motion_magic(t->position_rot());
      at_goal = std::abs(pos - t->position_rot()) <= tolerance_rot_;
    }
    state_.send(State{drive, pos, motor_.connected(), at_goal});
  }

 private:
  talos::Motor motor_;
  double tolerance_rot_;
  talos::Target<Target> target_;
  talos::Publisher<State> state_;
};

}  // namespace position_cpp
