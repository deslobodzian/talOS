#include "control/control_generated.h"
#include "talOS/devices/devices.h"
#include "talOS/mechanisms/position_mechanism.h"
#include "talOS/node/node.h"

namespace {

// Minimal testonly node hosting PositionMechanism for end-to-end physics
// simulation tests (stall_homing_sim_test).
class Lift {
 public:
  explicit Lift(talos::Node& node)
      : lift_{node, "lift"},
        target_{node},
        state_{node.publish<control::PositionState>("state")} {}

  void update() {
    if (const control::PositionTarget* t = target_.get()) {
      lift_.set_goal(t->position_rot(), t->feedforward_v());
    } else {
      lift_.stop();
    }
    lift_.update();
    state_.send(lift_.state());
  }

 private:
  talos::mechanisms::PositionMechanism lift_;
  talos::Target<control::PositionTarget> target_;
  talos::Publisher<control::PositionState> state_;
};

}  // namespace

TALOS_NODE(Lift)
