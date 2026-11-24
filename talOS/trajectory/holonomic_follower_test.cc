// HolonomicFollower: convergence on a simulated holonomic point mass along
// the curved fixture and a generated straight line (with an initial offset,
// a shove and a drift), output
// clamps and frames, heading wrap, validation and determinism.

#include "talOS/trajectory/holonomic_follower.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

#include "talOS/geometry/se2.h"
#include "talOS/trajectory/choreo.h"
#include "talOS/trajectory/straight_line.h"
#include "talOS/trajectory/test_fixtures.h"
#include "talOS/trajectory/trajectory.h"

namespace talos::trajectory {
namespace {

using Pose = geometry::SE2<double>;

Pose MakePose(double x_m, double y_m, double heading_rad) {
  return {WrapAngle(heading_rad), Pose::Vector2(x_m, y_m)};
}

Trajectory Curve() {
  const LoadResult result =
      LoadChoreoFile(testing::FixturePath("TALOS_TRAJECTORY_CURVE"));
  if (!result) throw std::runtime_error(result.error);
  return result.trajectory;
}

struct Disturbance {
  double shove_at_s = -1;  // adds shove_y_m to the robot's y once
  double shove_y_m = 0;
  double drift_x_mps = 0;  // added to the robot's x velocity every tick
};

struct RunResult {
  double max_error_after_m = 0;  // position error after settle_after_s
  double final_error_m = 0;
  double final_heading_error_rad = 0;
  bool finished = false;
  double finished_at_s = -1;
};

// A holonomic point mass that moves at the commanded speeds for each 20 ms
// tick. Robot-relative commands are applied in the heading the robot had
// when the command was computed, as a drivetrain would.
RunResult Simulate(const Trajectory& trajectory, const FollowerConfig& config,
                   SpeedsFrame frame, const Disturbance& disturbance,
                   double settle_after_s) {
  constexpr double kDt = 0.02;
  HolonomicFollower follower(config);
  const TrajectorySample start = trajectory.initial_sample();
  double x_m = start.x_m + 0.3;
  double y_m = start.y_m - 0.2;
  double heading_rad = start.heading_rad + 0.2;
  RunResult result;
  for (int tick = 0; tick <= 250; ++tick) {
    const double t_s = tick * kDt;
    if (std::abs(t_s - disturbance.shove_at_s) < kDt / 2) {
      y_m += disturbance.shove_y_m;
    }
    const FollowerOutput out =
        follower.step(trajectory, t_s, MakePose(x_m, y_m, heading_rad),
                      tick > 0 ? kDt : 0.0, frame);
    EXPECT_TRUE(out.valid);
    const double error_m = std::hypot(out.x_error_m, out.y_error_m);
    if (t_s >= settle_after_s) {
      result.max_error_after_m = std::max(result.max_error_after_m, error_m);
    }
    result.final_error_m = error_m;
    result.final_heading_error_rad = std::abs(out.heading_error_rad);
    if (out.finished && !result.finished) {
      result.finished = true;
      result.finished_at_s = t_s;
    }
    double vx_mps = out.speeds.vx_mps;
    double vy_mps = out.speeds.vy_mps;
    if (frame == SpeedsFrame::kRobot) {
      const double c = std::cos(heading_rad);
      const double s = std::sin(heading_rad);
      vx_mps = out.speeds.vx_mps * c - out.speeds.vy_mps * s;
      vy_mps = out.speeds.vx_mps * s + out.speeds.vy_mps * c;
    }
    x_m += (vx_mps + disturbance.drift_x_mps) * kDt;
    y_m += vy_mps * kDt;
    heading_rad += out.speeds.omega_radps * kDt;
  }
  return result;
}

// Guards: from a 0.36 m, 0.2 rad start offset, the follower converges onto
// the curved path (heading through +-pi) and finishes at the end within
// tolerance, with field- and robot-relative output alike.
// REQ-DEV-15
TEST(HolonomicFollowerTest, ConvergesOnCurvedPath) {
  const Trajectory trajectory = Curve();
  for (const SpeedsFrame frame : {SpeedsFrame::kField, SpeedsFrame::kRobot}) {
    SCOPED_TRACE(frame == SpeedsFrame::kField ? "field" : "robot");
    const RunResult result = Simulate(trajectory, {}, frame, {}, 1.5);
    EXPECT_LT(result.max_error_after_m, 0.06);
    EXPECT_LT(result.final_error_m, 0.01);
    EXPECT_LT(result.final_heading_error_rad, 0.01);
    EXPECT_TRUE(result.finished);
    EXPECT_GE(result.finished_at_s, trajectory.total_time_s());
  }
}

// Guards: a generated straight line (3.6 m and a 2.5 rad turn, built from
// the pose a node would hold) is followed unchanged: from the same start
// offset the follower converges onto it and finishes at its end.
// REQ-DEV-20
TEST(HolonomicFollowerTest, ConvergesOnStraightLine) {
  Trajectory line;
  line.reserve(kStraightLineSamples);
  ASSERT_TRUE(MakeStraightLine(MakePose(1, 1, 0), MakePose(4, 3, 2.5),
                               {2.0, 3.0, 3.0, 6.0}, line));
  for (const SpeedsFrame frame : {SpeedsFrame::kField, SpeedsFrame::kRobot}) {
    SCOPED_TRACE(frame == SpeedsFrame::kField ? "field" : "robot");
    const RunResult result = Simulate(line, {}, frame, {}, 1.5);
    EXPECT_LT(result.max_error_after_m, 0.06);
    EXPECT_LT(result.final_error_m, 0.01);
    EXPECT_LT(result.final_heading_error_rad, 0.01);
    EXPECT_TRUE(result.finished);
    EXPECT_GE(result.finished_at_s, line.total_time_s());
  }
}

// Guards: a 0.25 m sideways shove mid-path is recovered within a second.
// REQ-DEV-15
TEST(HolonomicFollowerTest, RecoversFromAShove) {
  const RunResult result =
      Simulate(Curve(), {}, SpeedsFrame::kRobot,
               {.shove_at_s = 1.0, .shove_y_m = 0.25}, 2.0);
  EXPECT_LT(result.max_error_after_m, 0.06);
  EXPECT_LT(result.final_error_m, 0.01);
  EXPECT_TRUE(result.finished);
}

// Guards: the integral term removes the steady error a constant 0.3 m/s
// drift leaves under P alone.
// REQ-DEV-15
TEST(HolonomicFollowerTest, IntegralRejectsConstantDrift) {
  const Disturbance drift{.drift_x_mps = 0.3};
  const RunResult p_only =
      Simulate(Curve(), {}, SpeedsFrame::kField, drift, 1.5);
  EXPECT_GT(p_only.final_error_m, 0.1);
  EXPECT_FALSE(p_only.finished);

  FollowerConfig config;
  config.translation.ki = 1.0;
  config.translation.integral_limit = 0.5;
  const RunResult with_integral =
      Simulate(Curve(), config, SpeedsFrame::kField, drift, 1.5);
  EXPECT_LT(with_integral.final_error_m, 0.03);
  EXPECT_TRUE(with_integral.finished);
}

// At rest at the origin for 1 s (two samples, both still), facing
// `heading_rad`, so the command is all feedback.
Trajectory AtOrigin(double heading_rad = 0) {
  std::vector<TrajectorySample> samples(2);
  samples[0].heading_rad = heading_rad;
  samples[1].heading_rad = heading_rad;
  samples[1].t_s = 1.0;
  return Trajectory{samples};
}

// Guards: a large error is limited to max_linear_mps without changing the
// command's direction, and omega to max_angular_radps.
TEST(HolonomicFollowerTest, ClampsOutput) {
  FollowerConfig config;
  config.max_linear_mps = 1.5;
  config.max_angular_radps = 2.0;
  HolonomicFollower follower(config);
  const FollowerOutput out = follower.step(
      AtOrigin(), 0, MakePose(-30, -40, 2.0), 0, SpeedsFrame::kField);
  ASSERT_TRUE(out.valid);
  EXPECT_NEAR(std::hypot(out.speeds.vx_mps, out.speeds.vy_mps), 1.5, 1e-12);
  EXPECT_NEAR(out.speeds.vx_mps, 1.5 * 0.6, 1e-12);
  EXPECT_NEAR(out.speeds.vy_mps, 1.5 * 0.8, 1e-12);
  EXPECT_EQ(out.speeds.omega_radps, -2.0);
}

struct PidStep {
  double x_error_m;
  double heading_error_rad;
  double dt_s;
};

struct PidRow {
  const char* what;
  PidGains translation;
  PidGains heading;
  std::vector<PidStep> steps;
  double vx_mps;  // after the last step
  double omega_radps;
};

std::vector<PidStep> Repeat(PidStep step, int count) {
  return std::vector<PidStep>(static_cast<std::size_t>(count), step);
}

std::vector<PidStep> Join(std::vector<PidStep> a,
                          const std::vector<PidStep>& b) {
  a.insert(a.end(), b.begin(), b.end());
  return a;
}

// Guards: the D term's sign and magnitude (change in error over the time
// since it was last taken), the heading's D across +-pi (wrapped, no 2 pi
// step), D held through a near-zero dt, and the integral held at its clamp
// and unwinding from it at once. Flipping D's sign, dropping the wrap, the
// minimum period or the clamp each fails a row.
TEST(HolonomicFollowerTest, PidTermsFromKnownErrors) {
  const double wrap_step_rad = 2 * std::numbers::pi - 6.2;  // 3.1 -> -3.1
  const PidRow rows[] = {
      {"D of a shrinking error opposes it",
       {.kd = 0.5},
       {},
       {{1.0, 0, 0.02}, {0.9, 0, 0.02}},
       0.5 * (0.9 - 1.0) / 0.02,
       0},
      {"D of a growing error follows it",
       {.kd = 0.5},
       {},
       {{0.9, 0, 0.02}, {1.0, 0, 0.02}},
       0.5 * (1.0 - 0.9) / 0.02,
       0},
      {"the first step only records the error",
       {.kd = 0.5},
       {},
       {{1.0, 0, 0.02}},
       0,
       0},
      {"heading D across +-pi is the short change",
       {},
       {.kd = 0.5},
       {{0, 3.1, 0.02}, {0, -3.1, 0.02}},
       0,
       0.5 * wrap_step_rad / 0.02},
      {"a near-zero dt holds the last D",
       {.kd = 0.5},
       {},
       {{1.0, 0, 0.02}, {0.9, 0, 0.02}, {0.5, 0, 1e-6}},
       0.5 * (0.9 - 1.0) / 0.02,
       0},
      {"after a near-zero dt, D spans both steps",
       {.kd = 0.5},
       {},
       {{1.0, 0, 0.02}, {0.9, 0, 0.02}, {0.5, 0, 1e-6}, {0.4, 0, 0.02}},
       0.5 * (0.4 - 0.9) / (0.02 + 1e-6),
       0},
      {"the integral is held at its clamp",
       {.ki = 1.0, .integral_limit = 0.3},
       {},
       Repeat({1.0, 0, 0.02}, 100),
       0.3,
       0},
      {"the integral unwinds from its clamp at once",
       {.ki = 1.0, .integral_limit = 0.3},
       {},
       Join(Repeat({1.0, 0, 0.02}, 100), {{-1.0, 0, 0.02}}),
       0.3 - 0.02,
       0},
  };
  for (const PidRow& row : rows) {
    SCOPED_TRACE(row.what);
    FollowerConfig config;
    config.translation = row.translation;
    config.heading = row.heading;
    config.max_linear_mps = 100;
    config.max_angular_radps = 100;
    HolonomicFollower follower(config);
    const Trajectory trajectory = AtOrigin();
    FollowerOutput out;
    for (const PidStep& step : row.steps) {
      out = follower.step(trajectory, 0.5,
                          MakePose(-step.x_error_m, 0, -step.heading_error_rad),
                          step.dt_s, SpeedsFrame::kField);
    }
    ASSERT_TRUE(out.valid);
    EXPECT_NEAR(out.speeds.vx_mps, row.vx_mps, 1e-9);
    EXPECT_NEAR(out.speeds.omega_radps, row.omega_radps, 1e-9);
  }
}

// Guards: robot-relative output is the field command seen from the
// measured heading.
TEST(HolonomicFollowerTest, RobotFrameRotatesByMeasuredHeading) {
  HolonomicFollower follower(
      FollowerConfig{.translation = {.kp = 1.0}, .heading = {.kp = 0.0}});
  // 1 m behind the reference in x: field command +1 m/s in x. Facing +y,
  // that is to the robot's right (-y).
  const FollowerOutput out = follower.step(
      AtOrigin(std::numbers::pi / 2), 0, MakePose(-1, 0, std::numbers::pi / 2),
      0, SpeedsFrame::kRobot);
  EXPECT_NEAR(out.speeds.vx_mps, 0.0, 1e-12);
  EXPECT_NEAR(out.speeds.vy_mps, -1.0, 1e-12);
}

// Guards: heading error is taken the short way across +-pi.
TEST(HolonomicFollowerTest, HeadingErrorWrapsTheShortWay) {
  HolonomicFollower follower;
  const FollowerOutput out = follower.step(
      AtOrigin(3.1), 0, MakePose(0, 0, -3.1), 0, SpeedsFrame::kField);
  EXPECT_NEAR(out.heading_error_rad, 6.2 - 2 * std::numbers::pi, 1e-12);
  EXPECT_LT(out.speeds.omega_radps, 0);
}

// Guards: finished needs both the end of time and both tolerances.
// REQ-DEV-15
TEST(HolonomicFollowerTest, FinishesOnlyAtTheEndAndInTolerance) {
  HolonomicFollower follower;
  const Trajectory trajectory = AtOrigin();
  EXPECT_FALSE(
      follower.step(trajectory, 0.5, MakePose(0, 0, 0), 0, SpeedsFrame::kField)
          .finished);
  EXPECT_TRUE(follower
                  .step(trajectory, 1.0, MakePose(0.03, 0.03, 0.04), 0,
                        SpeedsFrame::kField)
                  .finished);
  EXPECT_FALSE(
      follower
          .step(trajectory, 2.0, MakePose(0.1, 0, 0), 0, SpeedsFrame::kField)
          .finished);
  EXPECT_FALSE(
      follower
          .step(trajectory, 2.0, MakePose(0, 0, 0.1), 0, SpeedsFrame::kField)
          .finished);
}

// Guards: a non-finite pose, time or period, or a negative period gives zero
// speeds and !valid, and leaves every axis's integrator, derivative history
// and pending_dt_s undisturbed.
// REQ-DEV-15
TEST(HolonomicFollowerTest, NonFiniteInputCommandsZero) {
  FollowerConfig config;
  config.translation = {.kp = 2.0, .ki = 1.0, .kd = 0.5, .integral_limit = 5.0};
  config.heading = {.kp = 3.0, .ki = 1.0, .kd = 0.5, .integral_limit = 5.0};
  config.max_linear_mps = 100.0;
  config.max_angular_radps = 100.0;
  HolonomicFollower follower(config);
  HolonomicFollower reference(config);
  const Trajectory trajectory = AtOrigin();

  // Pre-populate nonzero integral, previous_error, derivative and pending_dt_s
  // on x, y and heading on both followers before injecting bad inputs.
  const auto prime = [&](HolonomicFollower& f) {
    ASSERT_TRUE(f.step(trajectory, 0.1, MakePose(-1.0, -0.5, -0.4), 0.02,
                       SpeedsFrame::kField)
                    .valid);
    ASSERT_TRUE(f.step(trajectory, 0.12, MakePose(-0.6, -0.2, -0.1), 0.02,
                       SpeedsFrame::kField)
                    .valid);
    ASSERT_TRUE(f.step(trajectory, 0.1205, MakePose(-0.5, -0.15, -0.05),
                       0.5 * HolonomicFollower::kMinDerivativePeriod_s,
                       SpeedsFrame::kField)
                    .valid);
  };
  prime(follower);
  prime(reference);

  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  for (const FollowerOutput& out :
       {follower.step(trajectory, 0.1, Pose(0, Pose::Vector2(nan, 0)), 0.02,
                      SpeedsFrame::kRobot),
        follower.step(trajectory, 0.1, Pose(0, Pose::Vector2(inf, 0)), 0.02,
                      SpeedsFrame::kRobot),
        follower.step(trajectory, 0.1, Pose(0, Pose::Vector2(-inf, 0)), 0.02,
                      SpeedsFrame::kRobot),
        follower.step(trajectory, 0.1, Pose(0, Pose::Vector2(0, nan)), 0.02,
                      SpeedsFrame::kRobot),
        follower.step(trajectory, 0.1, Pose(0, Pose::Vector2(0, inf)), 0.02,
                      SpeedsFrame::kRobot),
        follower.step(trajectory, 0.1, Pose(0, Pose::Vector2(0, -inf)), 0.02,
                      SpeedsFrame::kRobot),
        follower.step(trajectory, 0.1, Pose(nan, Pose::Vector2(0, 0)), 0.02,
                      SpeedsFrame::kRobot),
        follower.step(trajectory, 0.1, Pose(inf, Pose::Vector2(0, 0)), 0.02,
                      SpeedsFrame::kRobot),
        follower.step(trajectory, 0.1, Pose(-inf, Pose::Vector2(0, 0)), 0.02,
                      SpeedsFrame::kRobot),
        follower.step(trajectory, nan, MakePose(0, 0, 0), 0.02,
                      SpeedsFrame::kRobot),
        follower.step(trajectory, inf, MakePose(0, 0, 0), 0.02,
                      SpeedsFrame::kRobot),
        follower.step(trajectory, -inf, MakePose(0, 0, 0), 0.02,
                      SpeedsFrame::kRobot),
        follower.step(trajectory, 0.1, MakePose(0, 0, 0), nan,
                      SpeedsFrame::kRobot),
        follower.step(trajectory, 0.1, MakePose(0, 0, 0), -0.02,
                      SpeedsFrame::kRobot),
        follower.step(trajectory, 0.1, MakePose(0, 0, 0), inf,
                      SpeedsFrame::kRobot),
        follower.step(trajectory, 0.1, MakePose(0, 0, 0), -inf,
                      SpeedsFrame::kRobot),
        follower.step(Trajectory{}, 0.1, MakePose(0, 0, 0), 0.02,
                      SpeedsFrame::kRobot)}) {
    EXPECT_FALSE(out.valid);
    EXPECT_EQ(out.speeds.vx_mps, 0.0);
    EXPECT_EQ(out.speeds.vy_mps, 0.0);
    EXPECT_EQ(out.speeds.omega_radps, 0.0);
  }

  // First valid step at dt = 0 checks preserved integral and held derivative.
  const FollowerOutput a1 = follower.step(
      trajectory, 0.14, MakePose(-0.4, -0.1, -0.02), 0.0, SpeedsFrame::kField);
  const FollowerOutput b1 = reference.step(
      trajectory, 0.14, MakePose(-0.4, -0.1, -0.02), 0.0, SpeedsFrame::kField);
  ASSERT_TRUE(a1.valid);
  ASSERT_TRUE(b1.valid);
  EXPECT_EQ(a1.speeds.vx_mps, b1.speeds.vx_mps);
  EXPECT_EQ(a1.speeds.vy_mps, b1.speeds.vy_mps);
  EXPECT_EQ(a1.speeds.omega_radps, b1.speeds.omega_radps);

  // Second valid step crosses kMinDerivativePeriod_s only if pending_dt_s was
  // preserved, differentiating against preserved previous_error on all axes.
  const double next_dt_s = 0.6 * HolonomicFollower::kMinDerivativePeriod_s;
  const FollowerOutput a2 =
      follower.step(trajectory, 0.15, MakePose(-0.3, -0.05, -0.01), next_dt_s,
                    SpeedsFrame::kField);
  const FollowerOutput b2 =
      reference.step(trajectory, 0.15, MakePose(-0.3, -0.05, -0.01), next_dt_s,
                     SpeedsFrame::kField);
  ASSERT_TRUE(a2.valid);
  ASSERT_TRUE(b2.valid);
  EXPECT_EQ(a2.speeds.vx_mps, b2.speeds.vx_mps);
  EXPECT_EQ(a2.speeds.vy_mps, b2.speeds.vy_mps);
  EXPECT_EQ(a2.speeds.omega_radps, b2.speeds.omega_radps);
}

// Guards: huge finite position errors (where kp * error or hypot(vx, vy)
// overflows double, or error flips sign with kd == 0) still clamp cleanly to
// max_linear_mps with finite speeds and valid == true, and a huge finite dt_s
// spike keeps integrator anti-windup and derivative outputs finite.
// REQ-DEV-15
TEST(HolonomicFollowerTest, HugeFiniteErrorAndDtClampWithoutNaN) {
  const Trajectory trajectory = AtOrigin();
  {
    FollowerConfig config;
    config.translation.kp = 2.0;
    config.max_linear_mps = 1.5;
    config.max_angular_radps = 2.0;
    HolonomicFollower follower(config);

    // kp * x_error overflows to +inf; vy is 0.
    const FollowerOutput single_axis = follower.step(
        trajectory, 0.0, MakePose(-1e308, 0.0, 0.0), 0.02, SpeedsFrame::kField);
    ASSERT_TRUE(single_axis.valid);
    EXPECT_TRUE(std::isfinite(single_axis.speeds.vx_mps));
    EXPECT_TRUE(std::isfinite(single_axis.speeds.vy_mps));
    EXPECT_NEAR(single_axis.speeds.vx_mps, 1.5, 1e-12);
    EXPECT_NEAR(single_axis.speeds.vy_mps, 0.0, 1e-12);

    // Opposite-sign huge error on the next step with kd == 0 must not produce
    // 0 * (-inf) = NaN in the derivative term.
    const FollowerOutput sign_flip = follower.step(
        trajectory, 0.02, MakePose(1e308, 0.0, 0.0), 0.02, SpeedsFrame::kField);
    ASSERT_TRUE(sign_flip.valid);
    EXPECT_TRUE(std::isfinite(sign_flip.speeds.vx_mps));
    EXPECT_NEAR(sign_flip.speeds.vx_mps, -1.5, 1e-12);

    // Both axes overflow kp * error to +inf.
    follower.reset();
    const FollowerOutput both_inf =
        follower.step(trajectory, 0.0, MakePose(-1e308, -1e308, 0.0), 0.0,
                      SpeedsFrame::kField);
    ASSERT_TRUE(both_inf.valid);
    EXPECT_TRUE(std::isfinite(both_inf.speeds.vx_mps));
    EXPECT_TRUE(std::isfinite(both_inf.speeds.vy_mps));
    EXPECT_NEAR(std::hypot(both_inf.speeds.vx_mps, both_inf.speeds.vy_mps), 1.5,
                1e-12);
    EXPECT_GT(both_inf.speeds.vx_mps, 0.0);
    EXPECT_GT(both_inf.speeds.vy_mps, 0.0);

    // Both components finite (1.5e308) while hypot(vx, vy) overflows to +inf.
    follower.reset();
    const FollowerOutput hypot_overflow =
        follower.step(trajectory, 0.0, MakePose(-0.75e308, -0.75e308, 0.0), 0.0,
                      SpeedsFrame::kField);
    ASSERT_TRUE(hypot_overflow.valid);
    EXPECT_NEAR(hypot_overflow.speeds.vx_mps, 1.5 / std::sqrt(2.0), 1e-12);
    EXPECT_NEAR(hypot_overflow.speeds.vy_mps, 1.5 / std::sqrt(2.0), 1e-12);
  }

  // Huge finite dt_s spike with integral and derivative gains enabled.
  {
    FollowerConfig config;
    config.translation = {
        .kp = 0.0, .ki = 1.0, .kd = 0.5, .integral_limit = 0.5};
    config.max_linear_mps = 4.0;
    HolonomicFollower follower(config);
    const FollowerOutput spike = follower.step(
        trajectory, 0.1, MakePose(-10.0, 0.0, 0.0), 1e308, SpeedsFrame::kField);
    ASSERT_TRUE(spike.valid);
    EXPECT_NEAR(spike.speeds.vx_mps, 0.5, 1e-12);
    EXPECT_TRUE(std::isfinite(spike.speeds.vy_mps));
    EXPECT_TRUE(std::isfinite(spike.speeds.omega_radps));

    // Following normal step uses clamped integral and remains finite.
    const FollowerOutput next = follower.step(
        trajectory, 0.12, MakePose(0.0, 0.0, 0.0), 0.02, SpeedsFrame::kField);
    ASSERT_TRUE(next.valid);
    EXPECT_TRUE(std::isfinite(next.speeds.vx_mps));
  }
}

// Guards: an overflowing derivative (from error - previous_error, division by
// dt_s, kd * derivative, or an overflowing position error) never latches
// non-finite state into Axis across a subsequent dt_s < kMinDerivativePeriod_s
// tick, and stays deterministic and finite.
// REQ-DEV-15
TEST(HolonomicFollowerTest, DerivativeOverflowDoesNotLatchAcrossZeroDt) {
  const Trajectory trajectory = AtOrigin();

  // Case 1: error - previous_error overflows to -inf with kd > 0; a following
  // dt_s == 0 tick at zero error must not stay saturated at -max_linear_mps.
  {
    FollowerConfig config;
    config.translation = {.kp = 2.0, .kd = 0.5};
    config.max_linear_mps = 1.5;
    config.max_angular_radps = 2.0;
    HolonomicFollower a(config);
    HolonomicFollower b(config);

    const auto run_seq = [&](HolonomicFollower& f) {
      const FollowerOutput s1 = f.step(trajectory, 0.0, MakePose(-1e308, 0, 0),
                                       0.02, SpeedsFrame::kField);
      EXPECT_TRUE(s1.valid);
      EXPECT_NEAR(s1.speeds.vx_mps, 1.5, 1e-12);

      const FollowerOutput s2 = f.step(trajectory, 0.02, MakePose(1e308, 0, 0),
                                       0.02, SpeedsFrame::kField);
      EXPECT_TRUE(s2.valid);
      EXPECT_NEAR(s2.speeds.vx_mps, -1.5, 1e-12);

      const FollowerOutput s3 =
          f.step(trajectory, 0.02, MakePose(0, 0, 0), 0.0, SpeedsFrame::kField);
      EXPECT_TRUE(s3.valid);
      EXPECT_TRUE(std::isfinite(s3.speeds.vx_mps));
      EXPECT_NEAR(s3.speeds.vx_mps, 0.0, 1e-12);
      EXPECT_NEAR(s3.speeds.vy_mps, 0.0, 1e-12);

      const FollowerOutput s4 = f.step(trajectory, 0.0201, MakePose(0, 0, 0),
                                       1e-4, SpeedsFrame::kField);
      EXPECT_TRUE(s4.valid);
      EXPECT_NEAR(s4.speeds.vx_mps, 0.0, 1e-12);
      return s3;
    };
    const FollowerOutput oa = run_seq(a);
    const FollowerOutput ob = run_seq(b);
    EXPECT_EQ(oa.speeds.vx_mps, ob.speeds.vx_mps);
    EXPECT_EQ(oa.speeds.vy_mps, ob.speeds.vy_mps);
    EXPECT_EQ(oa.speeds.omega_radps, ob.speeds.omega_radps);
  }

  // Case 2: finite error delta divided by kMinDerivativePeriod_s overflows to
  // +inf; subsequent dt_s == 0 tick at zero error must output zero, not +1.5.
  {
    FollowerConfig config;
    config.translation = {.kp = 0.0, .kd = 0.5};
    config.max_linear_mps = 1.5;
    HolonomicFollower follower(config);

    ASSERT_TRUE(
        follower
            .step(trajectory, 0.0, MakePose(0, 0, 0), 0.02, SpeedsFrame::kField)
            .valid);
    const FollowerOutput overflow = follower.step(
        trajectory, 0.001, MakePose(-1e307, 0, 0),
        HolonomicFollower::kMinDerivativePeriod_s, SpeedsFrame::kField);
    ASSERT_TRUE(overflow.valid);
    EXPECT_TRUE(std::isfinite(overflow.speeds.vx_mps));

    const FollowerOutput zero_dt = follower.step(
        trajectory, 0.001, MakePose(0, 0, 0), 0.0, SpeedsFrame::kField);
    ASSERT_TRUE(zero_dt.valid);
    EXPECT_NEAR(zero_dt.speeds.vx_mps, 0.0, 1e-12);
    EXPECT_NEAR(zero_dt.speeds.vy_mps, 0.0, 1e-12);
  }

  // Case 3: finite derivative whose kd * derivative product overflows to +inf
  // must neither saturate a following dt_s == 0 tick at zero error nor cancel
  // an opposite-sign p_term into NaN.
  {
    FollowerConfig config;
    config.translation = {.kp = 200.0, .kd = 100.0};
    config.max_linear_mps = 1.5;
    HolonomicFollower follower(config);

    ASSERT_TRUE(
        follower
            .step(trajectory, 0.0, MakePose(0, 0, 0), 0.01, SpeedsFrame::kField)
            .valid);
    const FollowerOutput spike = follower.step(
        trajectory, 0.01, MakePose(-1e306, 0, 0), 0.01, SpeedsFrame::kField);
    ASSERT_TRUE(spike.valid);
    EXPECT_NEAR(spike.speeds.vx_mps, 1.5, 1e-12);

    const FollowerOutput zero_dt = follower.step(
        trajectory, 0.01, MakePose(0, 0, 0), 0.0, SpeedsFrame::kField);
    ASSERT_TRUE(zero_dt.valid);
    EXPECT_NEAR(zero_dt.speeds.vx_mps, 0.0, 1e-12);

    const FollowerOutput opposite = follower.step(
        trajectory, 0.01, MakePose(1e307, 0, 0), 0.0, SpeedsFrame::kField);
    ASSERT_TRUE(opposite.valid);
    EXPECT_NEAR(opposite.speeds.vx_mps, -1.5, 1e-12);
  }

  // Case 4: reference.x_m - measured_x_m overflowing to +inf must not latch
  // non-finite previous_error into subsequent ticks.
  {
    std::vector<TrajectorySample> samples(2);
    samples[0].x_m = 1e308;
    samples[1].x_m = 1e308;
    samples[1].t_s = 1.0;
    const Trajectory far_trajectory{samples};

    FollowerConfig config;
    config.translation = {.kp = 2.0, .kd = 0.5};
    config.max_linear_mps = 1.5;
    HolonomicFollower follower(config);

    const FollowerOutput s1 = follower.step(
        far_trajectory, 0.0, MakePose(-1e308, 0, 0), 0.02, SpeedsFrame::kField);
    ASSERT_TRUE(s1.valid);
    EXPECT_NEAR(s1.speeds.vx_mps, 1.5, 1e-12);

    const FollowerOutput s2 = follower.step(
        far_trajectory, 0.0, MakePose(1e308, 0, 0), 0.0, SpeedsFrame::kField);
    ASSERT_TRUE(s2.valid);
    EXPECT_NEAR(s2.speeds.vx_mps, 0.0, 1e-12);

    const FollowerOutput s3 = follower.step(
        far_trajectory, 0.02, MakePose(1e308, 0, 0), 0.02, SpeedsFrame::kField);
    ASSERT_TRUE(s3.valid);
    EXPECT_NEAR(s3.speeds.vx_mps, 0.0, 1e-12);
  }
}

// Guards: bad configuration is refused at construction, naming the field.
TEST(HolonomicFollowerTest, RejectsBadConfig) {
  FollowerConfig negative_gain;
  negative_gain.heading.kp = -1;
  try {
    HolonomicFollower follower(negative_gain);
    ADD_FAILURE() << "accepted a negative gain";
  } catch (const std::invalid_argument& e) {
    EXPECT_NE(std::string(e.what()).find("heading.kp"), std::string::npos);
  }
  FollowerConfig zero_speed;
  zero_speed.max_linear_mps = 0;
  EXPECT_THROW(static_cast<void>(HolonomicFollower{zero_speed}),
               std::invalid_argument);
  FollowerConfig nan_tolerance;
  nan_tolerance.position_tolerance_m = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW(static_cast<void>(HolonomicFollower{nan_tolerance}),
               std::invalid_argument);
}

// Guards: the same inputs give bit-identical outputs (replay determinism),
// and reset() returns the follower to its initial state.
TEST(HolonomicFollowerTest, DeterministicAndResettable) {
  FollowerConfig config;
  config.translation = {.kp = 2.0, .ki = 0.5, .kd = 0.1, .integral_limit = 1};
  config.heading = {.kp = 3.0, .ki = 0.2, .kd = 0.05, .integral_limit = 1};
  const Trajectory trajectory = Curve();
  HolonomicFollower a(config);
  HolonomicFollower b(config);
  std::vector<double> first_run;
  for (int tick = 0; tick < 100; ++tick) {
    const double t_s = 0.02 * tick;
    const Pose pose = MakePose(8 + 0.01 * tick, 2, 2.0 + 0.01 * tick);
    const FollowerOutput oa =
        a.step(trajectory, t_s, pose, 0.02, SpeedsFrame::kRobot);
    const FollowerOutput ob =
        b.step(trajectory, t_s, pose, 0.02, SpeedsFrame::kRobot);
    EXPECT_EQ(oa.speeds.vx_mps, ob.speeds.vx_mps);
    EXPECT_EQ(oa.speeds.vy_mps, ob.speeds.vy_mps);
    EXPECT_EQ(oa.speeds.omega_radps, ob.speeds.omega_radps);
    first_run.push_back(oa.speeds.vx_mps);
  }
  a.reset();
  for (int tick = 0; tick < 100; ++tick) {
    const double t_s = 0.02 * tick;
    const Pose pose = MakePose(8 + 0.01 * tick, 2, 2.0 + 0.01 * tick);
    EXPECT_EQ(
        a.step(trajectory, t_s, pose, 0.02, SpeedsFrame::kRobot).speeds.vx_mps,
        first_run[static_cast<std::size_t>(tick)]);
  }
}

// Guards: step_pose() matches step() for any pose type exposing x_m(), y_m(),
// and yaw_rad(), and ChassisSpeeds::as<M>() converts to a target struct.
// REQ-DEV-15
TEST(HolonomicFollowerTest, StepPoseAndChassisSpeedsAsMatchStep) {
  struct PoseLike {
    double x = 0;
    double y = 0;
    double yaw = 0;
    double x_m() const { return x; }
    double y_m() const { return y; }
    double yaw_rad() const { return yaw; }
  };
  struct TargetLike {
    double vx_mps = 0;
    double vy_mps = 0;
    double omega_radps = 0;
  };

  FollowerConfig config;
  config.translation = {.kp = 2.0, .ki = 0.4, .kd = 0.1, .integral_limit = 1.0};
  config.heading = {.kp = 3.0, .ki = 0.2, .kd = 0.05, .integral_limit = 1.0};
  const Trajectory trajectory = Curve();

  for (const SpeedsFrame frame : {SpeedsFrame::kField, SpeedsFrame::kRobot}) {
    SCOPED_TRACE(frame == SpeedsFrame::kField ? "field" : "robot");
    HolonomicFollower via_se2(config);
    HolonomicFollower via_pose(config);
    for (int tick = 0; tick < 20; ++tick) {
      const double t_s = 0.02 * tick;
      const double x = 8.0 + 0.03 * tick;
      const double y = 2.0 - 0.02 * tick;
      const double yaw = 1.5 + 0.05 * tick;
      const FollowerOutput expected =
          via_se2.step(trajectory, t_s, Pose(yaw, Pose::Vector2(x, y)),
                       tick > 0 ? 0.02 : 0.0, frame);
      const FollowerOutput actual = via_pose.step_pose(
          trajectory, t_s, PoseLike{x, y, yaw}, tick > 0 ? 0.02 : 0.0, frame);
      EXPECT_EQ(actual.valid, expected.valid);
      EXPECT_EQ(actual.finished, expected.finished);
      EXPECT_DOUBLE_EQ(actual.x_error_m, expected.x_error_m);
      EXPECT_DOUBLE_EQ(actual.y_error_m, expected.y_error_m);
      EXPECT_DOUBLE_EQ(actual.heading_error_rad, expected.heading_error_rad);
      EXPECT_DOUBLE_EQ(actual.speeds.vx_mps, expected.speeds.vx_mps);
      EXPECT_DOUBLE_EQ(actual.speeds.vy_mps, expected.speeds.vy_mps);
      EXPECT_DOUBLE_EQ(actual.speeds.omega_radps, expected.speeds.omega_radps);

      const TargetLike target = actual.speeds.as<TargetLike>();
      EXPECT_DOUBLE_EQ(target.vx_mps, actual.speeds.vx_mps);
      EXPECT_DOUBLE_EQ(target.vy_mps, actual.speeds.vy_mps);
      EXPECT_DOUBLE_EQ(target.omega_radps, actual.speeds.omega_radps);
    }
  }
}

}  // namespace
}  // namespace talos::trajectory
