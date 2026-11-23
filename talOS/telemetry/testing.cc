#include "talOS/telemetry/testing.h"

#include <cstdint>
#include <string>
#include <vector>

#include "talOS/node/node.h"
#include "talOS/node/testing.h"
#include "talOS/telemetry/node.h"

namespace talos::telemetry {

void AddTelemetry(talos::testing::Sim& sim, const std::string& name) {
  sim.add<Telemetry>(name);
}

void AddTimedTelemetry(talos::testing::Sim& sim,
                       std::vector<std::int64_t>& samples_ns,
                       const bool& recording, const std::string& name) {
  auto& timed = sim.add<talos::testing::TimedNode<Telemetry>>(name);
  timed.bind(&samples_ns, &recording);
}

}  // namespace talos::telemetry
