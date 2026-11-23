#include "control/control_generated.h"
#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

namespace {

// Minimal testonly actuator node for sim_gateway bench-fault integration tests
// (bench_faults_test): drives one velocity-controlled motor from its target
// lane and mirrors its feedback on /bench_motor/state.
class BenchMotor {
 public:
  explicit BenchMotor(talos::Node& node)
      : node_{node},
        motor_{node, "kraken"},
        target_{node},
        state_{node.publish<control::VelocityState>("state")} {}

  void update() {
    double requested_rps = 0.0;
    const bool enabled = talos::robot_enabled(node_);
    if (const control::VelocityTarget* t = target_.get()) {
      if (motor_.connected()) {
        requested_rps = t->velocity_rps();
        motor_.set_velocity(requested_rps, t->feedforward_v());
      } else {
        motor_.stop();
      }
    } else {
      motor_.stop();
    }
    state_.send(control::VelocityState{motor_.velocity(), requested_rps,
                                       motor_.current(), enabled,
                                       motor_.connected()});
  }

 private:
  talos::Node& node_;
  talos::Motor motor_;
  talos::Target<control::VelocityTarget> target_;
  talos::Publisher<control::VelocityState> state_;
};

}  // namespace

TALOS_NODE(BenchMotor)
