// log_budget_test's sender: declares one 512-byte state topic, which the
// launcher's log model counts as one send per tick. With `only_enabled` it
// sends only while the Driver Station is enabled, like an operator-input
// producer, so a disabled session stores a fraction of its predicted rate;
// with `sends_per_tick` above one it stores several times its predicted
// rate.

#include <cstdint>

#include "budget_test/budget_message_generated.h"
#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

class BudgetSender {
 public:
  explicit BudgetSender(talos::Node& node)
      : driver_station_{node},
        state_{node.publish<budget_test::Block>("state")},
        only_enabled_{node.param<bool>("only_enabled")},
        sends_per_tick_{node.param<int64_t>("sends_per_tick")} {
    for (uint32_t i = 0; i < 128; ++i)
      block_.mutable_fill_count()->Mutate(i, i + 1);
  }

  void update() {
    if (only_enabled_ && !driver_station_.enabled()) return;
    for (int64_t i = 0; i < sends_per_tick_; ++i) state_.send(block_);
  }

 private:
  talos::DriverStation driver_station_;
  talos::Publisher<budget_test::Block> state_;
  bool only_enabled_;
  int64_t sends_per_tick_;
  budget_test::Block block_{};
};

TALOS_NODE(BudgetSender)
