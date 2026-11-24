// Compile-time checks of the simulation truth messages declared in sim.fbs:
// building this file is the check, so a layout change fails the build until
// docs/DATA_FLOW.md states it. Each field's offset is pinned by the telemetry
// channel model's test (talOS/telemetry/tests/channels_test.cc).

#include "sim/sim_generated.h"
#include "talOS/node/node.h"

namespace talos::hardware {

// REQ-DEV-40: /sim/state/match's shared-memory slot contract.
static_assert(sizeof(sim::MatchState) == 64,
              "sim.MatchState size changed; update docs/DATA_FLOW.md");
static_assert(alignof(sim::MatchState) == 8,
              "sim.MatchState alignment changed; update docs/DATA_FLOW.md");
static_assert(Message<sim::MatchState>);

}  // namespace talos::hardware
