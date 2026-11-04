// replay_inputs_test's misreader: it reads /replay_source/state as a type of
// the same size that is not the one the session recorded.

#include "replay_inputs_test/replay_message_generated.h"
#include "talOS/node/node.h"

class ReplayImpostor {
 public:
  explicit ReplayImpostor(talos::Node& node)
      : in_{node.subscribe<replay_inputs_test::Impostor>(
            "/replay_source/state")} {}

  void update() {}

 private:
  talos::Subscription<replay_inputs_test::Impostor> in_;
};

TALOS_NODE(ReplayImpostor)
