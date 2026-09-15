#include "cpp_node/node.h"

#include <gtest/gtest.h>
#include <chrono>

#include "talOS/events/simulated_event_loop.h"

namespace sample {
namespace {
using namespace std::chrono_literals;

TEST(SampleNodeTest, TicksOnSimulatedLoop) {
  talos::event::SimulationEnvironment env;
  talos::event::SimulatedEventLoop<> loop{env};

  SampleNode node{loop};
  node.Start(talos::event::MonotonicTime::from_nanos(1'000'000));

  EXPECT_EQ(node.ticks(), 0u);
  loop.run_for(25ms);
  EXPECT_GE(node.ticks(), 2u);
}

}  // namespace
}  // namespace sample
