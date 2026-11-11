// Tests for talOS/aiming readiness: each reason in its documented order,
// failing defaults, staleness, NaN handling, and the tick debounce.

#include "talOS/aiming/readiness.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>

namespace talos::aiming {
namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

ReadinessLimits Limits(int settle_tick_count = 1) {
  ReadinessLimits limits;
  limits.max_pose_age_us = 40000;
  limits.aim_tolerance_rad = 0.02;
  limits.flywheel_tolerance_rps = 1.5;
  limits.hood_tolerance_rad = 0.01;
  limits.max_robot_speed_mps = 3.0;
  limits.max_robot_omega_radps = 2.0;
  limits.settle_tick_count = settle_tick_count;
  return limits;
}

ReadinessInput AllGood() {
  ReadinessInput input;
  input.pose_age_us = 10000;
  input.solution_valid = true;
  input.converged = true;
  input.in_range = true;
  input.turret_reachable = true;
  input.robot_speed_mps = 1.0;
  input.robot_omega_radps = -0.5;
  input.aim_error_rad = -0.01;
  input.flywheel_error_rps = 1.0;
  input.hood_error_rad = 0.005;
  return input;
}

// Every condition can block the shot, and the reason named is the first
// failing one in the documented order (staleness first; convergence before
// range, since a non-converged distance means nothing).
TEST(Readiness, EachConditionBlocksWithItsReasonInOrder) {
  const ReadinessLimits limits = Limits();
  EXPECT_EQ(CheckReadiness(AllGood(), limits), ReadinessReason::kReady);

  ReadinessInput in = AllGood();
  in.hood_error_rad = -0.02;
  EXPECT_EQ(CheckReadiness(in, limits), ReadinessReason::kHoodNotAtGoal);
  in.flywheel_error_rps = -2.0;
  EXPECT_EQ(CheckReadiness(in, limits), ReadinessReason::kFlywheelNotAtSpeed);
  in.aim_error_rad = 0.05;
  EXPECT_EQ(CheckReadiness(in, limits), ReadinessReason::kAimError);
  in.robot_omega_radps = -2.5;
  EXPECT_EQ(CheckReadiness(in, limits), ReadinessReason::kRobotRotatingTooFast);
  in.robot_speed_mps = 3.5;
  EXPECT_EQ(CheckReadiness(in, limits), ReadinessReason::kRobotTooFast);
  in.turret_reachable = false;
  EXPECT_EQ(CheckReadiness(in, limits), ReadinessReason::kTurretLimit);
  in.in_range = false;
  EXPECT_EQ(CheckReadiness(in, limits), ReadinessReason::kOutOfRange);
  in.converged = false;
  EXPECT_EQ(CheckReadiness(in, limits), ReadinessReason::kNotConverged);
  in.solution_valid = false;
  EXPECT_EQ(CheckReadiness(in, limits), ReadinessReason::kNoSolution);
  in.pose_age_us = 40001;
  EXPECT_EQ(CheckReadiness(in, limits), ReadinessReason::kStale);

  // Exactly at a tolerance or the age limit passes.
  in = AllGood();
  in.aim_error_rad = -0.02;
  in.robot_speed_mps = 3.0;
  in.pose_age_us = 40000;
  EXPECT_EQ(CheckReadiness(in, limits), ReadinessReason::kReady);
}

// A field the caller never fills blocks the shot: a default input is
// stale, and with only the age and solution flags filled in each numeric
// check still fails in turn.
TEST(Readiness, UnfilledFieldsFail) {
  const ReadinessLimits limits = Limits();
  EXPECT_EQ(CheckReadiness(ReadinessInput{}, limits), ReadinessReason::kStale);

  ReadinessInput in;
  in.pose_age_us = 0;
  in.solution_valid = true;
  in.converged = true;
  in.in_range = true;
  in.turret_reachable = true;
  EXPECT_EQ(CheckReadiness(in, limits), ReadinessReason::kRobotTooFast);
  in.robot_speed_mps = 0.0;
  in.robot_omega_radps = 0.0;
  in.aim_error_rad = 0.0;
  // The flywheel error was forgotten: not ready while it spins up.
  EXPECT_EQ(CheckReadiness(in, limits), ReadinessReason::kFlywheelNotAtSpeed);
  in.flywheel_error_rps = 0.0;
  EXPECT_EQ(CheckReadiness(in, limits), ReadinessReason::kHoodNotAtGoal);
  in.hood_error_rad = 0.0;  // a robot without a hood says so
  EXPECT_EQ(CheckReadiness(in, limits), ReadinessReason::kReady);
}

// A pose the gate cannot date, or one from the future, is stale too.
TEST(Readiness, UnknownOrNegativePoseAgeIsStale) {
  const ReadinessLimits limits = Limits();
  ReadinessInput in = AllGood();
  in.pose_age_us = ReadinessInput::kUnknownAge_us;
  EXPECT_EQ(CheckReadiness(in, limits), ReadinessReason::kStale);
  in.pose_age_us = -1;
  EXPECT_EQ(CheckReadiness(in, limits), ReadinessReason::kStale);
}

// A NaN measurement never reads as within tolerance.
TEST(Readiness, NaNMeasurementFails) {
  const ReadinessLimits limits = Limits();
  ReadinessInput in = AllGood();
  in.aim_error_rad = kNaN;
  EXPECT_EQ(CheckReadiness(in, limits), ReadinessReason::kAimError);
  in = AllGood();
  in.robot_omega_radps = kNaN;
  EXPECT_EQ(CheckReadiness(in, limits), ReadinessReason::kRobotRotatingTooFast);
}

// Ready only after every condition has held on settle_tick_count
// consecutive ticks; one failing tick drops it at once and restarts the
// count.
TEST(ReadinessGate, DebouncesInTicksAndDropsImmediately) {
  ReadinessGate gate(Limits(3));
  Readiness r = gate.Update(AllGood(), 10);
  EXPECT_FALSE(r.ready);
  EXPECT_EQ(r.reason, ReadinessReason::kSettling);
  EXPECT_EQ(r.passing_tick_count, 1);
  r = gate.Update(AllGood(), 11);
  EXPECT_FALSE(r.ready);
  EXPECT_EQ(r.passing_tick_count, 2);
  r = gate.Update(AllGood(), 12);
  EXPECT_TRUE(r.ready);
  EXPECT_EQ(r.reason, ReadinessReason::kReady);
  r = gate.Update(AllGood(), 13);
  EXPECT_TRUE(r.ready);
  EXPECT_EQ(r.passing_tick_count, 3);  // saturates

  ReadinessInput wobble = AllGood();
  wobble.aim_error_rad = 0.1;
  r = gate.Update(wobble, 14);
  EXPECT_FALSE(r.ready);
  EXPECT_EQ(r.reason, ReadinessReason::kAimError);
  EXPECT_EQ(r.passing_tick_count, 0);
  EXPECT_EQ(gate.Update(AllGood(), 15).reason, ReadinessReason::kSettling);
  EXPECT_EQ(gate.Update(AllGood(), 16).reason, ReadinessReason::kSettling);
  EXPECT_TRUE(gate.Update(AllGood(), 17).ready);

  gate.Reset();
  EXPECT_EQ(gate.Update(AllGood(), 18).reason, ReadinessReason::kSettling);
}

// Ticks the caller skipped (disabled, paused) do not count: a gap in the
// tick index, a repeated index or a step back restarts the settle count, so
// a re-enable cannot report ready without settling again.
TEST(ReadinessGate, SkippedTicksRestartTheSettleCount) {
  ReadinessGate gate(Limits(3));
  gate.Update(AllGood(), 1);
  gate.Update(AllGood(), 2);
  ASSERT_TRUE(gate.Update(AllGood(), 3).ready);

  // Disabled for ticks 4..99: the caller does not call Update.
  Readiness r = gate.Update(AllGood(), 100);
  EXPECT_FALSE(r.ready);
  EXPECT_EQ(r.reason, ReadinessReason::kSettling);
  EXPECT_EQ(r.passing_tick_count, 1);

  gate.Update(AllGood(), 101);
  r = gate.Update(AllGood(), 101);  // repeated
  EXPECT_FALSE(r.ready);
  EXPECT_EQ(r.passing_tick_count, 1);
  r = gate.Update(AllGood(), 50);  // stepped back
  EXPECT_EQ(r.passing_tick_count, 1);
}

// settle_tick_count = 1 means no debounce.
TEST(ReadinessGate, OneTickIsImmediate) {
  ReadinessGate gate(Limits(1));
  EXPECT_TRUE(gate.Update(AllGood(), 0).ready);
}

// Bad limits are refused, naming the key; an infinite speed limit is no
// limit.
TEST(ReadinessGate, RejectsBadLimits) {
  EXPECT_THROW(ReadinessGate{Limits(0)}, std::invalid_argument);
  ReadinessLimits negative = Limits();
  negative.hood_tolerance_rad = -0.1;
  EXPECT_THROW(ReadinessGate{negative}, std::invalid_argument);
  ReadinessLimits no_age = Limits();
  no_age.max_pose_age_us = 0;
  EXPECT_THROW(ReadinessGate{no_age}, std::invalid_argument);
  ReadinessLimits nan = Limits();
  nan.aim_tolerance_rad = kNaN;
  try {
    ReadinessGate gate(nan);
    FAIL() << "expected a throw";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("aim_tolerance_rad"),
              std::string::npos);
  }
  ReadinessLimits unlimited = Limits();
  unlimited.max_robot_speed_mps = std::numeric_limits<double>::infinity();
  ReadinessGate gate(unlimited);
  ReadinessInput fast = AllGood();
  fast.robot_speed_mps = 100.0;
  EXPECT_TRUE(gate.Update(fast, 0).ready);
}

}  // namespace
}  // namespace talos::aiming
