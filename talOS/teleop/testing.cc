#include "talOS/teleop/testing.h"

#include <cstdint>
#include <string>
#include <vector>

#include "talOS/node/node.h"
#include "talOS/node/testing.h"
#include "talOS/teleop/node.h"

namespace talos::teleop {

void AddTeleop(talos::testing::Sim& sim, const std::string& name) {
  sim.add<Teleop>(name);
}

void AddTimedTeleop(talos::testing::Sim& sim,
                    std::vector<std::int64_t>& samples_ns,
                    const bool& recording, const std::string& name) {
  auto& timed = sim.add<talos::testing::TimedNode<Teleop>>(name);
  timed.bind(&samples_ns, &recording);
}

}  // namespace talos::teleop
