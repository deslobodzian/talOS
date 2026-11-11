// log_budget_test's hardware node: owns the motor its `device` parameter
// names and drives it with a voltage that changes every tick, so each of its
// requests changes the bridge's merged command (the enabled, commanding
// case the launcher's log model sizes). Its log holds every /hw/state, every
// Driver Station packet, the bridge's status and its own requests, at the
// rate of its subsystem's period_us. Run only against a simulated gateway.

#include <string>

#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

class BudgetMotor {
 public:
  explicit BudgetMotor(talos::Node& node)
      : motor_{node, node.param<std::string>("device")} {}

  void update() {
    tick_ = (tick_ + 1) % 10;
    motor_.set_voltage(0.5 + 0.1 * tick_);
  }

 private:
  talos::Motor motor_;
  int tick_{0};
};

TALOS_NODE(BudgetMotor)
