// MakeStraightLine: the profile against hand-computed trapezoids, its
// limits, synchronised finish, the triangular and degenerate cases, the
// heading wrap, refusals and alliance flips.

#include "talOS/trajectory/straight_line.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <numbers>
#include <string>
#include <vector>

#include "talOS/geometry/se2.h"
#include "talOS/trajectory/trajectory.h"

namespace talos::trajectory {
namespace {

using Pose = geometry::SE2<double>;

constexpr double kPi = std::numbers::pi;
// 2 m/s, 4 m/s^2, 3 rad/s, 6 rad/s^2.
constexpr StraightLineLimits kLimits{2.0, 4.0, 3.0, 6.0};

Pose MakePose(double x_m, double y_m, double heading_rad) {
  return {heading_rad, Pose::Vector2(x_m, y_m)};
}

Trajectory Line(const Pose& start, const Pose& end,
                const StraightLineLimits& limits = kLimits) {
  Trajectory out;
  out.reserve(kStraightLineSamples);
  const StraightLineResult result = MakeStraightLine(start, end, limits, out);
  EXPECT_TRUE(result) << ToString(result.error);
  return out;
}

double AngleDistance(double a_rad, double b_rad) {
  return std::abs(WrapAngle(a_rad - b_rad));
}

// first_s, first_s + step_s, ... up to last_s.
std::vector<double> Times(double first_s, double last_s, double step_s) {
  std::vector<double> times;
  for (int k = 0; first_s + static_cast<double>(k) * step_s <= last_s; ++k) {
    times.push_back(first_s + static_cast<double>(k) * step_s);
  }
  return times;
}

// Guards: a 4 m diagonal move at 2 m/s and 4 m/s^2 is the trapezoid worked
// by hand (0.5 s ramps, 2.5 s total; 0.125 m in at 0.25 s, 2 m at 1.25 s,
// 3.875 m at 2.25 s) along the segment, between samples as well as on
// them, and ends exactly on the end pose at rest at 2.5 s.
// REQ-DEV-20
TEST(StraightLineTest, TrapezoidMatchesHandComputedProfile) {
  // Direction (0.6, 0.8): a 3-4-5 triangle scaled to 4 m.
  const Pose end = MakePose(3.4, 4.2, 0.2);
  const Trajectory line = Line(MakePose(1.0, 1.0, 0.2), end);
  EXPECT_NEAR(line.total_time_s(), 2.5, 1e-12);
  struct Point {
    double t_s;
    double along_m;
    double speed_mps;
  };
  for (const Point p : {Point{0.25, 0.125, 1.0}, Point{1.0, 1.5, 2.0},
                        Point{1.25, 2.0, 2.0}, Point{2.25, 3.875, 1.0}}) {
    SCOPED_TRACE("t = " + std::to_string(p.t_s));
    const TrajectorySample s = line.sample(p.t_s);
    EXPECT_NEAR(s.x_m, 1.0 + 0.6 * p.along_m, 1e-12);
    EXPECT_NEAR(s.y_m, 1.0 + 0.8 * p.along_m, 1e-12);
    EXPECT_NEAR(s.vx_mps, 0.6 * p.speed_mps, 1e-12);
    EXPECT_NEAR(s.vy_mps, 0.8 * p.speed_mps, 1e-12);
    EXPECT_NEAR(s.heading_rad, 0.2, 1e-12);
    EXPECT_EQ(s.omega_radps, 0.0);
  }
  const TrajectorySample last = line.sample(line.total_time_s());
  EXPECT_EQ(last.x_m, end.translation().x());
  EXPECT_EQ(last.y_m, end.translation().y());
  EXPECT_EQ(last.heading_rad, 0.2);
  EXPECT_EQ(last.vx_mps, 0.0);
  EXPECT_EQ(last.vy_mps, 0.0);
  EXPECT_EQ(last.omega_radps, 0.0);
}

struct Move {
  const char* name;
  Pose start;
  Pose end;
  double total_s = 0;  // worked by hand where set
};

// Translation-limited (4 m vs a 1 rad turn) and rotation-limited (3 rad vs
// 0.5 m) moves.
std::vector<Move> UnequalMoves() {
  return {{"translation-limited", MakePose(0, 0, 0), MakePose(4, 0, 1.0), 2.5},
          {"rotation-limited", MakePose(1, 1, 0), MakePose(1, 1.5, 3.0), 1.5}};
}

// Guards: speed, acceleration, angular rate and angular acceleration stay
// within their limits at every instant sample() returns, for a long move, a
// short one, a pure rotation and both unequal moves.
// REQ-DEV-20
TEST(StraightLineTest, StaysWithinLimits) {
  std::vector<Move> moves = UnequalMoves();
  moves.push_back({"long", MakePose(-2, 3, -1), MakePose(6, -1, 2)});
  moves.push_back({"short", MakePose(0, 0, 0), MakePose(0.1, 0.05, 0.1)});
  moves.push_back({"rotation", MakePose(0, 0, 0), MakePose(0, 0, -2.5)});
  constexpr double kSlack = 1e-9;
  for (const Move& move : moves) {
    SCOPED_TRACE(move.name);
    const Trajectory line = Line(move.start, move.end);
    for (const double t_s : Times(0, line.total_time_s() + 0.01, 1e-3)) {
      const TrajectorySample s = line.sample(t_s);
      ASSERT_LE(std::hypot(s.vx_mps, s.vy_mps), kLimits.max_linear_mps + kSlack)
          << "t = " << t_s;
      ASSERT_LE(std::hypot(s.ax_mps2, s.ay_mps2),
                kLimits.max_linear_accel_mps2 + kSlack)
          << "t = " << t_s;
      ASSERT_LE(std::abs(s.omega_radps), kLimits.max_angular_radps + kSlack)
          << "t = " << t_s;
      ASSERT_LE(std::abs(s.alpha_radps2),
                kLimits.max_angular_accel_radps2 + kSlack)
          << "t = " << t_s;
    }
  }
}

// Guards: when one axis needs longer, the other is slowed to arrive with
// it: both are short of the end at every instant before the total time,
// which is the slower axis's own minimum (translation 4 m: 4/2 + 2/4 =
// 2.5 s; rotation 3 rad: 3/3 + 3/6 = 1.5 s), and both arrive at it.
// REQ-DEV-20
TEST(StraightLineTest, TranslationAndRotationFinishTogether) {
  for (const Move& move : UnequalMoves()) {
    SCOPED_TRACE(move.name);
    const Trajectory line = Line(move.start, move.end);
    ASSERT_NEAR(line.total_time_s(), move.total_s, 1e-12);
    const double end_x = move.end.translation().x();
    const double end_y = move.end.translation().y();
    const double end_heading = move.end.angle();
    for (const double t_s : Times(0.01, line.total_time_s() - 0.01, 0.01)) {
      const TrajectorySample s = line.sample(t_s);
      ASSERT_GT(std::hypot(end_x - s.x_m, end_y - s.y_m), 1e-6)
          << "translation arrived early at t = " << t_s;
      ASSERT_GT(AngleDistance(end_heading, s.heading_rad), 1e-6)
          << "rotation arrived early at t = " << t_s;
    }
    const TrajectorySample last = line.sample(line.total_time_s());
    EXPECT_NEAR(std::hypot(end_x - last.x_m, end_y - last.y_m), 0, 1e-12);
    EXPECT_LT(AngleDistance(end_heading, last.heading_rad), 1e-12);
  }
}

// Guards: a move too short to reach the speed limit is a triangle: 0.5 m
// at 4 m/s^2 takes 2 sqrt(0.5 / 4) s and peaks at sqrt(4 * 0.5) m/s, below
// the 2 m/s limit, at its midpoint.
// REQ-DEV-20
TEST(StraightLineTest, ShortMoveIsTriangular) {
  const Trajectory line = Line(MakePose(0, 0, 0), MakePose(0, 0.5, 0));
  EXPECT_NEAR(line.total_time_s(), std::sqrt(0.5), 1e-12);
  double peak_mps = 0;
  for (const double t_s : Times(0, line.total_time_s(), 1e-4)) {
    const TrajectorySample s = line.sample(t_s);
    peak_mps = std::max(peak_mps, std::hypot(s.vx_mps, s.vy_mps));
  }
  EXPECT_NEAR(peak_mps, std::sqrt(2.0), 1e-3);
  EXPECT_NEAR(line.sample(line.total_time_s() / 2).vy_mps, std::sqrt(2.0),
              1e-12);
}

// Guards: a pure rotation from 3.0 to -3.0 rad turns the short way through
// pi (+0.283 rad, never negative omega) with the position held, the heading
// continuous, ending on the end heading.
// REQ-DEV-20
TEST(StraightLineTest, PureRotationTurnsTheShortWayAcrossPi) {
  const Trajectory line = Line(MakePose(2, 3, 3.0), MakePose(2, 3, -3.0));
  const double turn_rad = 2 * kPi - 6.0;
  // Triangular: 0.283 rad is below 3^2 / 6 = 1.5 rad.
  EXPECT_NEAR(line.total_time_s(), 2 * std::sqrt(turn_rad / 6.0), 1e-12);
  for (const double t_s : Times(0, line.total_time_s(), 1e-3)) {
    const TrajectorySample s = line.sample(t_s);
    ASSERT_EQ(s.x_m, 2.0) << "t = " << t_s;
    ASSERT_EQ(s.y_m, 3.0) << "t = " << t_s;
    ASSERT_GE(s.omega_radps, 0.0) << "t = " << t_s;
  }
  EXPECT_NEAR(line.final_sample().heading_rad, 3.0 + turn_rad, 1e-12);
  EXPECT_LT(AngleDistance(line.final_pose().angle(), -3.0), 1e-12);
}

// Guards: start == end is one sample at rest at t = 0, not a refusal.
// REQ-DEV-20
TEST(StraightLineTest, ZeroLengthMoveIsOneSampleAtRest) {
  const Trajectory line = Line(MakePose(1, 2, 0.5), MakePose(1, 2, 0.5));
  ASSERT_EQ(line.samples().size(), 1U);
  EXPECT_EQ(line.total_time_s(), 0.0);
  const TrajectorySample s = line.sample(0.3);
  EXPECT_EQ(s.x_m, 1.0);
  EXPECT_EQ(s.y_m, 2.0);
  EXPECT_EQ(s.heading_rad, 0.5);
  EXPECT_EQ(std::hypot(s.vx_mps, s.vy_mps), 0.0);
  EXPECT_EQ(s.omega_radps, 0.0);
}

// Guards: a non-finite pose, a limit that is not finite and positive, a
// capacity below kStraightLineSamples and an overflowing move are refused
// with their error, leaving `out` empty (so a follower commands zero), not
// holding the previous line.
// REQ-DEV-20
TEST(StraightLineTest, RefusesInvalidInputAndClears) {
  constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
  constexpr double kInf = std::numeric_limits<double>::infinity();
  const Pose origin = MakePose(0, 0, 0);
  const Pose goal = MakePose(1, 1, 1);
  struct Case {
    const char* name;
    Pose start;
    Pose end;
    StraightLineLimits limits;
    std::size_t capacity;
    StraightLineError error;
  };
  const auto with = [](double StraightLineLimits::* field, double value) {
    StraightLineLimits limits = kLimits;
    limits.*field = value;
    return limits;
  };
  const std::vector<Case> cases = {
      {"nan start x", MakePose(kNan, 0, 0), goal, kLimits, kStraightLineSamples,
       StraightLineError::kNonFinitePose},
      {"infinite end heading", origin, MakePose(1, 1, kInf), kLimits,
       kStraightLineSamples, StraightLineError::kNonFinitePose},
      {"zero speed", origin, goal, with(&StraightLineLimits::max_linear_mps, 0),
       kStraightLineSamples, StraightLineError::kInvalidLimits},
      {"negative acceleration", origin, goal,
       with(&StraightLineLimits::max_linear_accel_mps2, -1),
       kStraightLineSamples, StraightLineError::kInvalidLimits},
      {"nan angular rate", origin, goal,
       with(&StraightLineLimits::max_angular_radps, kNan), kStraightLineSamples,
       StraightLineError::kInvalidLimits},
      {"infinite angular acceleration", origin, goal,
       with(&StraightLineLimits::max_angular_accel_radps2, kInf),
       kStraightLineSamples, StraightLineError::kInvalidLimits},
      {"capacity", origin, goal, kLimits, kStraightLineSamples - 1,
       StraightLineError::kCapacity},
      {"overflow", MakePose(-1e308, 0, 0), MakePose(1e308, 0, 0), kLimits,
       kStraightLineSamples, StraightLineError::kUnrepresentable},
  };
  for (const Case& c : cases) {
    SCOPED_TRACE(c.name);
    Trajectory out;
    out.reserve(c.capacity);
    out.assign(std::vector<TrajectorySample>(1));
    const StraightLineResult result =
        MakeStraightLine(c.start, c.end, c.limits, out);
    EXPECT_FALSE(result);
    EXPECT_EQ(result.error, c.error);
    EXPECT_TRUE(out.empty());
  }
}

// Guards: the line flipped for the other alliance is the line between the
// flipped poses, for both symmetries, so a node may flip either the goal or
// the trajectory.
// REQ-DEV-14
TEST(StraightLineTest, FlippedLineIsTheLineBetweenFlippedPoses) {
  const FieldDimensions field = kField2026.field;
  const Pose start = MakePose(2, 1, 0.3);
  const Pose end = MakePose(5, 3, 2.0);
  for (const AllianceFlip flip :
       {AllianceFlip::kRotate, AllianceFlip::kMirror}) {
    SCOPED_TRACE(flip == AllianceFlip::kRotate ? "rotate" : "mirror");
    const Trajectory flipped = Line(start, end).flipped(flip, field);
    const Trajectory direct =
        Line(Flip(start, flip, field), Flip(end, flip, field));
    ASSERT_NEAR(flipped.total_time_s(), direct.total_time_s(), 1e-12);
    for (const double t_s : Times(0, direct.total_time_s(), 0.01)) {
      const TrajectorySample a = flipped.sample(t_s);
      const TrajectorySample b = direct.sample(t_s);
      ASSERT_NEAR(a.x_m, b.x_m, 1e-9) << "t = " << t_s;
      ASSERT_NEAR(a.y_m, b.y_m, 1e-9) << "t = " << t_s;
      ASSERT_LT(AngleDistance(a.heading_rad, b.heading_rad), 1e-9)
          << "t = " << t_s;
      ASSERT_NEAR(a.vx_mps, b.vx_mps, 1e-9) << "t = " << t_s;
      ASSERT_NEAR(a.vy_mps, b.vy_mps, 1e-9) << "t = " << t_s;
      ASSERT_NEAR(a.omega_radps, b.omega_radps, 1e-9) << "t = " << t_s;
    }
  }
}

}  // namespace
}  // namespace talos::trajectory
