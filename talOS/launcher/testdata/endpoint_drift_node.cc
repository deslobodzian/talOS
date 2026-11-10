// supervision_test's endpoint check fixture, run as `alpha`. It always
// publishes /alpha/state and reads /beta/state (optional, latest-only: the
// loop flags it, the probe does not), so a well-behaved run publishes ends
// that must compare equal to its probe's. With `drift = 1` in its subsystem
// table it also publishes /alpha/event, but only when not describing: the
// endpoint mismatch Node::describing() must never cause.

#include "continuity_test/continuity_message_generated.h"
#include "talOS/node/node.h"

class EndpointDrift {
 public:
  explicit EndpointDrift(talos::Node& node)
      : state_{node.publish<continuity_test::Count>("/alpha/state")},
        peer_{node.subscribe<continuity_test::Count>("/beta/state",
                                                     {.optional = true})} {
    if (!node.describing() && node.param<int>("drift", 0) != 0) {
      drift_ = node.publish<continuity_test::Count>("/alpha/event");
    }
  }

  void update() {
    state_.send(continuity_test::Count{peer_.has_value() ? 1u : 0u, 0u});
  }

 private:
  talos::Publisher<continuity_test::Count> state_;
  talos::Subscription<continuity_test::Count> peer_;
  talos::Publisher<continuity_test::Count> drift_;
};

TALOS_NODE(EndpointDrift)
