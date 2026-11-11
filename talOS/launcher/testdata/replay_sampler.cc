// replay_inputs_test's sampler, a node the recorded session ran: a plain
// subscription to the source's readings, drained before each of its ticks,
// and the newest reading sent on each tick. Which reading a tick saw depends
// on when the tick ran against when the source published.

#include <cstdint>

#include "replay_inputs_test/replay_message_generated.h"
#include "talOS/node/node.h"

class ReplaySampler {
 public:
  explicit ReplaySampler(talos::Node& node)
      : in_{node.subscribe<replay_inputs_test::Reading>(
            "/replay_source/state")},
        out_{node.publish<replay_inputs_test::Summary>("state")} {}

  void update() {
    if (!in_.has_value()) return;
    out_.send(replay_inputs_test::Summary{in_->reading_index() + 1,
                                          in_->position_m()});
  }

 private:
  talos::Subscription<replay_inputs_test::Reading> in_;
  talos::Publisher<replay_inputs_test::Summary> out_;
};

TALOS_NODE(ReplaySampler)
