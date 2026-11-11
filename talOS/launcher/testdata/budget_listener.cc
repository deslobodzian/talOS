// log_budget_test's Driver Station reader, like an operator-input producer:
// no device, so its log holds its ticks and the Driver Station packets and
// little else.

#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

class BudgetListener {
 public:
  explicit BudgetListener(talos::Node& node) : driver_station_{node} {}

  void update() { enabled_ = driver_station_.enabled(); }

 private:
  talos::DriverStation driver_station_;
  bool enabled_{false};
};

TALOS_NODE(BudgetListener)
