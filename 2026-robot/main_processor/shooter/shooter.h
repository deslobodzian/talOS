#pragma once

#include <cmath>
#include <cstdint>

#include "shooter/shooter_message_generated.h"
#include "shooter/topics.h"
#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

namespace shooter {

// Spins the flywheel to the selected target lane and reports whether it is at
// speed and whether a game piece is breaking the beam.
class Shooter {
 public:
  static constexpr double kAtSpeedToleranceRps = 2.0;

  explicit Shooter(talos::Node& node)
      : node_{node},
        flywheel_{node, "flywheel"},
        beam_break_{node, "beam_break"},
        target_{node},
        state_{node.publish<ShooterState>(topics::kState)} {}

  void update() {
    const bool enabled = talos::robot_enabled(node_);
    double goal = 0.0;
    bool spin = false;
    if (const ShooterTarget* t = target_.get()) {
      goal = t->target_velocity_rps();
      spin = flywheel_.connected() && std::isfinite(goal);
      if (spin)
        flywheel_.set_velocity(goal);  // otherwise the flush sends neutral
    }

    const double velocity = flywheel_.velocity();
    const bool at_speed = spin && std::abs(goal) > 1e-3 &&
                          std::abs(velocity - goal) <= kAtSpeedToleranceRps;
    state_.send(ShooterState{static_cast<std::uint64_t>(node_.now().nanos()),
                             velocity, spin ? goal : 0.0, at_speed,
                             beam_break_.get(), enabled});
  }

 private:
  talos::Node& node_;
  talos::Motor flywheel_;
  talos::DigitalInput beam_break_;
  talos::Target<ShooterTarget> target_;
  talos::Publisher<ShooterState> state_;
};

}  // namespace shooter
