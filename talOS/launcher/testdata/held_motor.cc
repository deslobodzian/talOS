// outputs_held_test's node: owns one motor and sets it to 1 V every tick
// without a talos::Target, so while the robot is enabled the devices flush
// holds the motor neutral (REQ-SAFE-41) and the node's registry row carries
// outputs_held. Run only against a simulated gateway.

#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

class HeldMotor {
 public:
  explicit HeldMotor(talos::Node& node) : motor_{node, "m1"} {}

  void update() { motor_.set_voltage(1.0); }

 private:
  talos::Motor motor_;
};

TALOS_NODE(HeldMotor)
