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

// Guards: a non-finite pose, time or period gives zero speeds and !valid,
// and does not disturb the integrator.
// REQ-DEV-15
TEST(HolonomicFollowerTest, NonFiniteInputCommandsZero) {
  FollowerConfig config;
  config.translation.ki = 1.0;
  config.translation.integral_limit = 1.0;
  HolonomicFollower follower(config);
  HolonomicFollower reference(config);
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const Trajectory trajectory = AtOrigin();
  for (const FollowerOutput& out :
       {follower.step(trajectory, 0.1, MakePose(nan, 0, 0), 0.02,
                      SpeedsFrame::kRobot),
        follower.step(trajectory, nan, MakePose(0, 0, 0), 0.02,
                      SpeedsFrame::kRobot),
        follower.step(trajectory, 0.1, MakePose(0, 0, 0), nan,
                      SpeedsFrame::kRobot),
        follower.step(Trajectory{}, 0.1, MakePose(0, 0, 0), 0.02,
                      SpeedsFrame::kRobot)}) {
    EXPECT_FALSE(out.valid);
    EXPECT_EQ(out.speeds.vx_mps, 0.0);
    EXPECT_EQ(out.speeds.vy_mps, 0.0);
    EXPECT_EQ(out.speeds.omega_radps, 0.0);
  }
  const FollowerOutput a = follower.step(trajectory, 0.1, MakePose(-1, 0, 0),
                                         0.02, SpeedsFrame::kField);
  const FollowerOutput b = reference.step(trajectory, 0.1, MakePose(-1, 0, 0),
                                          0.02, SpeedsFrame::kField);
  EXPECT_EQ(a.speeds.vx_mps, b.speeds.vx_mps);
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

}  // namespace
}  // namespace talos::trajectory
