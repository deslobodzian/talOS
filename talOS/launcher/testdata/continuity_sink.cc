// continuity_test's subscriber: it only reads /continuity_source/state, and
// its event log records every message it was handed, which the test checks.

#include "continuity_test/continuity_message_generated.h"
#include "talOS/node/node.h"

class Sink {
 public:
  explicit Sink(talos::Node& node)
      : in_{node.subscribe<continuity_test::Count>(
            "/continuity_source/state")} {}

  void update() {}

 private:
  talos::Subscription<continuity_test::Count> in_;
};

TALOS_NODE(Sink)
