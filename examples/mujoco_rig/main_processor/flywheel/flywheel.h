#pragma once

#include <cmath>

#include "flywheel/flywheel_message_generated.h"
#include "flywheel/topics.h"
#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

namespace flywheel {

// Drives the motor at the velocity its target asks for and reports what it
// did on /flywheel/state. Gains live in the robot config, slot0 of
// [motors.flywheel_motor]; the motor controller closes the velocity loop
// onboard.
//
// In simulation the teleop bindings in robot_sim.toml ask. A producer writes
// its own lane, /flywheel/target/<source>, with talos::TargetLane;
// talos::Target picks the live lane by the policy in [robot.targets] or the
// subsystem's [targets].
class Flywheel {
 public:
  using Target = FlywheelTarget;
  using State = FlywheelState;

  explicit Flywheel(talos::Node& node)
      : motor_{node, "flywheel_motor"},
        target_{node},
        state_{node.publish<State>(topics::kState)} {}

  void update() {
    const Target* t = target_.get();  // null: disabled, stale or released
    const bool drive =
        t != nullptr && motor_.connected() && std::isfinite(t->velocity_rps());
    // Otherwise nothing is set and the flush sends neutral.
    if (drive) motor_.set_velocity(t->velocity_rps());
    state_.send(State{drive, motor_.velocity(), motor_.connected()});
  }

 private:
  talos::Motor motor_;
  talos::Target<Target> target_;
  talos::Publisher<State> state_;
};

}  // namespace flywheel
