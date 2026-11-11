// sample(), events_between(), Flip(sample) and HolonomicFollower::step() run
// in a control node's update() and must not allocate (RULES.md 3).

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>

#include "talOS/geometry/se2.h"
#include "talOS/node/allocation_probe.h"
#include "talOS/trajectory/choreo.h"
#include "talOS/trajectory/holonomic_follower.h"
#include "talOS/trajectory/test_fixtures.h"
#include "talOS/trajectory/trajectory.h"

namespace talos::trajectory {
namespace {

using talos::testing::AllocationProbe;

// REQ-RT-06
// Guards: a control tick on a loaded trajectory (sample, marker lookup,
// follower step with integral and derivative terms in both frames, flip of
// one sample) allocates nothing; loading and construction may.
TEST(NoAllocationTest, ControlTickDoesNotAllocate) {
  if (const auto why = AllocationProbe::Unsupported(); !why.empty()) {
    GTEST_SKIP() << why;
  }
  const LoadResult loaded =
      LoadChoreoFile(testing::FixturePath("TALOS_TRAJECTORY_SPLITS_EVENTS"));
  ASSERT_TRUE(loaded) << loaded.error;
  const Trajectory& trajectory = loaded.trajectory;
  FollowerConfig config;
  config.translation.ki = 0.5;
  config.translation.integral_limit = 1.0;
  config.translation.kd = 0.1;
  HolonomicFollower follower(config);
  const FieldDimensions field{16.54, 8.07};

  double sink = 0;
  std::size_t events = 0;
  double previous_s = -1;
  AllocationProbe probe;
  probe.start();
  for (int tick = 0; tick < 300; ++tick) {
    const double t_s = 0.02 * tick - 0.5;
    const TrajectorySample reference = trajectory.sample(t_s);
    events += trajectory.events_between(previous_s, t_s).size();
    previous_s = t_s;
    const geometry::SE2<double> measured(
        reference.heading_rad + 0.01,
        geometry::SE2<double>::Vector2(reference.x_m + 0.02, reference.y_m));
    const FollowerOutput out = follower.step(
        trajectory, t_s, measured, 0.02,
        tick % 2 == 0 ? SpeedsFrame::kRobot : SpeedsFrame::kField);
    sink += out.speeds.vx_mps + out.speeds.omega_radps +
            Flip(reference, AllianceFlip::kRotate, field).x_m;
  }
  const std::uint64_t allocations = probe.stop();
  EXPECT_EQ(allocations, 0U) << probe.sites();
  EXPECT_EQ(events, 2U);
  EXPECT_TRUE(std::isfinite(sink));
}

}  // namespace
}  // namespace talos::trajectory
