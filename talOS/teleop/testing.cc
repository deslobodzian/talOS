#include "talOS/teleop/testing.h"

#include <string>

#include "talOS/node/testing.h"
#include "talOS/teleop/node.h"

namespace talos::teleop {

void AddTeleop(talos::testing::Sim& sim, const std::string& name) {
  sim.add<Teleop>(name);
}

}  // namespace talos::teleop
