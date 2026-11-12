#pragma once

#include <cstdint>

#include "shooter/shooter_message_generated.h"
#include "shooter/topics.h"
#include "talOS/devices/devices.h"
#include "talOS/mechanisms/velocity_mechanism.h"
#include "talOS/node/node.h"

namespace shooter {

// Shared-memory slot contract for /shooter/state: docs/DATA_FLOW.md.
static_assert(sizeof(ShooterState) == 32,
              "ShooterState size changed; update docs/DATA_FLOW.md");

// Spins the flywheel to the selected target lane. The clamp to the motor's
// max_velocity_rps and the at-speed readiness are VelocityMechanism's
// (talOS/mechanisms/README.md); its keys are the [flywheel] table.
class Shooter {
 public:
  explicit Shooter(talos::Node& node)
      : node_{node},
        flywheel_{node, "flywheel"},
        beam_break_{node, "beam_break"},
        target_{node},
        state_{node.publish<ShooterState>(topics::kState)} {}

  void update() {
    if (const ShooterTarget* t = target_.get()) {
      flywheel_.set_goal(t->target_velocity_rps());
    } else {
      flywheel_.stop();  // the flush sends neutral
    }
    flywheel_.update();
    state_.send(ShooterState{static_cast<std::uint64_t>(node_.now().nanos()),
                             flywheel_.velocity_rps(), flywheel_.goal_rps(),
                             flywheel_.at_speed(), beam_break_.get(),
                             talos::robot_enabled(node_)});
  }

 private:
  talos::Node& node_;
  talos::mechanisms::VelocityMechanism flywheel_;
  talos::DigitalInput beam_break_;
  talos::Target<ShooterTarget> target_;
  talos::Publisher<ShooterState> state_;
};

}  // namespace shooter
