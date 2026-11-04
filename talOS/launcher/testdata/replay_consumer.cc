// replay_inputs_test's new node: it was not in the recorded session. It
// counts and sums every reading it is handed and publishes the running
// summary each tick.

#include <cstdint>

#include "replay_inputs_test/replay_message_generated.h"
#include "talOS/node/node.h"

class ReplayConsumer {
 public:
  explicit ReplayConsumer(talos::Node& node)
      : in_{node.subscribe<replay_inputs_test::Reading>(
            "/replay_source/state",
            [this](const replay_inputs_test::Reading& reading) {
              ++count_;
              total_m_ += reading.position_m();
            })},
        out_{node.publish<replay_inputs_test::Summary>("state")} {}

  void update() { out_.send(replay_inputs_test::Summary{count_, total_m_}); }

 private:
  talos::Subscription<replay_inputs_test::Reading> in_;
  talos::Publisher<replay_inputs_test::Summary> out_;
  std::uint32_t count_ = 0;
  float total_m_ = 0.0F;
};

TALOS_NODE(ReplayConsumer)
