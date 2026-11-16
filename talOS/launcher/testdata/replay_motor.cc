// replay_inputs_test's hardware reader: one motor read through the devices
// layer, which takes only a /hw/state whose config id is its config's, and
// maps its gateway sample time through /hw/status's clock mapping. It sends
// what it reads each tick.

#include <cstdint>

#include "replay_inputs_test/replay_message_generated.h"
#include "talOS/devices/devices.h"
#include "talOS/node/node.h"

class ReplayMotor {
 public:
  explicit ReplayMotor(talos::Node& node)
      : node_{node},
        motor_{node, "m2"},
        out_{node.publish<replay_inputs_test::MotorReading>("state")} {}

  void update() {
    const auto sample = node_.extension<talos::HardwareLink>().sample_time();
    out_.send(replay_inputs_test::MotorReading{
        motor_.connected(), motor_.position(),
        sample ? sample->nanos() : std::int64_t{-1}});
  }

 private:
  talos::Node& node_;
  talos::Motor motor_;
  talos::Publisher<replay_inputs_test::MotorReading> out_;
};

TALOS_NODE(ReplayMotor)
