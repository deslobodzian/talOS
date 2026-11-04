// replay_inputs_test's recorded producer: kReadings readings, one per tick,
// then nothing until the session stops it.

#include <cstdint>

#include "replay_inputs_test/replay_message_generated.h"
#include "talOS/node/node.h"

class ReplaySource {
 public:
  static constexpr std::uint32_t kReadings = 100;

  explicit ReplaySource(talos::Node& node)
      : out_{node.publish<replay_inputs_test::Reading>("state")} {}

  void update() {
    if (sent_ == kReadings) return;
    out_.send(
        replay_inputs_test::Reading{sent_, 0.5F * static_cast<float>(sent_)});
    ++sent_;
  }

 private:
  talos::Publisher<replay_inputs_test::Reading> out_;
  std::uint32_t sent_ = 0;
};

TALOS_NODE(ReplaySource)
