// sample(), EventCursor::advance(), Flip(sample), HolonomicFollower::step(),
// Trajectory::assign() within its reserve() and MakeStraightLine() into a
// reserved trajectory run in a control node's update() and must not
// allocate (RULES.md 3).

#include <gtest/gtest.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "talOS/geometry/se2.h"
#include "talOS/node/allocation_probe.h"
#include "talOS/trajectory/choreo.h"
#include "talOS/trajectory/holonomic_follower.h"
#include "talOS/trajectory/straight_line.h"
#include "talOS/trajectory/test_fixtures.h"
#include "talOS/trajectory/trajectory.h"

namespace talos::trajectory {
namespace {

using talos::testing::AllocationProbe;

// REQ-RT-06, REQ-DEV-20
// Guards: a control tick on a loaded trajectory (sample, marker cursor,
// follower step with integral and derivative terms on translation and heading
// in both frames, flip of one sample, rebuilding a reserved trajectory in both
// constant-acceleration and Hermite modes, and generating or refusing a
// straight line from the live pose into a reserved trajectory) allocates
// nothing; loading and construction may.
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
  config.heading.ki = 0.3;
  config.heading.integral_limit = 1.0;
  config.heading.kd = 0.05;
  HolonomicFollower follower(config);
  const FieldDimensions field{16.54, 8.07};

  double sink = 0;
  std::size_t events = 0;
  EventCursor cursor;
  Trajectory rebuilt;
  rebuilt.reserve(4);
  Trajectory hermite;
  hermite.reserve(4);
  std::vector<TrajectorySample> fresh(4);
  Trajectory line;
  line.reserve(kStraightLineSamples);
  Trajectory small_line;
  small_line.reserve(kStraightLineSamples - 1);
  const StraightLineLimits limits{2.0, 3.0, 3.0, 6.0};
  const StraightLineLimits bad_limits{0.0, 3.0, 3.0, 6.0};
  const geometry::SE2<double> goal(2.0, geometry::SE2<double>::Vector2(4, 3));
  const geometry::SE2<double> nan_pose(
      0.0, geometry::SE2<double>::Vector2(
               std::numeric_limits<double>::quiet_NaN(), 0.0));
  const geometry::SE2<double> far_start(
      0.0, geometry::SE2<double>::Vector2(-1e308, 0.0));
  const geometry::SE2<double> far_end(
      0.0, geometry::SE2<double>::Vector2(1e308, 0.0));
  std::size_t lines = 0;
  std::size_t refused_non_finite = 0;
  std::size_t refused_limits = 0;
  std::size_t refused_capacity = 0;
  std::size_t refused_unrepresentable = 0;
  AllocationProbe probe;
  probe.start();
  for (int tick = 0; tick < 300; ++tick) {
    const double t_s = 0.02 * tick - 0.5;
    const TrajectorySample reference = trajectory.sample(t_s);
    events += cursor.advance(trajectory, t_s).size();
    for (std::size_t i = 0; i < fresh.size(); ++i) {
      fresh[i] = reference;
      fresh[i].t_s = static_cast<double>(i);
    }
    rebuilt.assign(fresh);
    sink += rebuilt.sample(1.5).x_m;
    hermite.assign(fresh, Interpolation::kHermite);
    sink += hermite.sample(1.5).x_m;
    const geometry::SE2<double> measured(
        reference.heading_rad + 0.01,
        geometry::SE2<double>::Vector2(reference.x_m + 0.02, reference.y_m));
    const FollowerOutput out = follower.step(
        trajectory, t_s, measured, 0.02,
        tick % 2 == 0 ? SpeedsFrame::kRobot : SpeedsFrame::kField);
    sink += out.speeds.vx_mps + out.speeds.omega_radps +
            Flip(reference, AllianceFlip::kRotate, field).x_m;
    if (MakeStraightLine(measured, goal, limits, line)) ++lines;
    sink += line.sample(0.3).x_m;
    if (MakeStraightLine(nan_pose, goal, limits, line).error ==
            StraightLineError::kNonFinitePose &&
        line.empty()) {
      ++refused_non_finite;
    }
    if (MakeStraightLine(measured, goal, bad_limits, line).error ==
            StraightLineError::kInvalidLimits &&
        line.empty()) {
      ++refused_limits;
    }
    if (MakeStraightLine(measured, goal, limits, small_line).error ==
            StraightLineError::kCapacity &&
        small_line.empty()) {
      ++refused_capacity;
    }
    if (MakeStraightLine(far_start, far_end, limits, line).error ==
            StraightLineError::kUnrepresentable &&
        line.empty()) {
      ++refused_unrepresentable;
    }
  }
  const std::uint64_t allocations = probe.stop();
  EXPECT_EQ(allocations, 0U) << probe.sites();
  EXPECT_EQ(events, 2U);
  EXPECT_EQ(lines, 300U);
  EXPECT_EQ(refused_non_finite, 300U);
  EXPECT_EQ(refused_limits, 300U);
  EXPECT_EQ(refused_capacity, 300U);
  EXPECT_EQ(refused_unrepresentable, 300U);
  EXPECT_TRUE(std::isfinite(sink));
}

}  // namespace
}  // namespace talos::trajectory
