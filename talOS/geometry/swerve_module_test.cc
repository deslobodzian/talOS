#include "talOS/geometry/swerve_module.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>
#include <random>

#include "talOS/geometry/so2.h"

namespace talos::geometry {
namespace {

constexpr double kPi = std::numbers::pi;
constexpr double kQuarter = kPi / 2.0;
constexpr double kTight = 1e-12;

// REQ-DEV-26: a steering change of exactly a quarter turn, either way, keeps
// the asked heading; a hair past it takes the opposite heading and reverses
// the drive.
TEST(OptimizeSwerveModule, QuarterTurnIsTheFlipBoundary) {
  for (const double sign : {1.0, -1.0}) {
    SCOPED_TRACE(::testing::Message() << "sign " << sign);
    const auto at = OptimizeSwerveModule(2.0, sign * kQuarter, 0.0);
    EXPECT_FALSE(at.reversed);
    EXPECT_DOUBLE_EQ(at.steer_delta_rad, sign * kQuarter);
    EXPECT_GE(at.speed, 0.0);
    EXPECT_NEAR(at.speed, 0.0, kTight);

    const auto past = OptimizeSwerveModule(2.0, sign * (kQuarter + 1e-6), 0.0);
    EXPECT_TRUE(past.reversed);
    EXPECT_NEAR(past.steer_delta_rad, -sign * (kQuarter - 1e-6), kTight);
    EXPECT_LT(past.speed, 0.0);
    EXPECT_NEAR(past.speed, -2.0 * std::sin(1e-6), kTight);

    const auto next_past = OptimizeSwerveModule(
        2.0,
        sign *
            std::nextafter(kQuarter, std::numeric_limits<double>::infinity()),
        0.0);
    EXPECT_TRUE(next_past.reversed);
    EXPECT_LE(std::abs(next_past.steer_delta_rad), kQuarter);
    EXPECT_LE(next_past.speed, 0.0);
  }
}

// REQ-DEV-26: the measured angle is the motor's unwrapped position. Seven
// turns plus 0.2 rad, asked pi + 0.5 rad: the shortest change is 0.3 rad on
// the opposite heading, not seven turns back.
TEST(OptimizeSwerveModule, FlipsAgainstAnUnwrappedMeasuredAngle) {
  for (const double turns : {7.0, -7.0}) {
    SCOPED_TRACE(::testing::Message() << "turns " << turns);
    const auto setpoint =
        OptimizeSwerveModule(3.0, kPi + 0.5, 2.0 * kPi * turns + 0.2);
    EXPECT_TRUE(setpoint.reversed);
    EXPECT_NEAR(setpoint.steer_delta_rad, 0.3, 1e-9);
    EXPECT_NEAR(setpoint.speed, -3.0 * std::cos(0.3), 1e-9);
  }
}

// REQ-DEV-26
TEST(OptimizeSwerveModule, NonFiniteInputGivesZero) {
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  constexpr double kInf = std::numeric_limits<double>::infinity();
  struct Case {
    double speed, heading, measured;
  };
  for (const Case& c :
       {Case{kNaN, 0.5, 0.0}, Case{kInf, 0.5, 0.0}, Case{1.0, kNaN, 0.0},
        Case{1.0, -kInf, 0.0}, Case{1.0, 0.5, kNaN}, Case{1.0, 0.5, kInf}}) {
    SCOPED_TRACE(::testing::Message()
                 << "speed " << c.speed << " heading " << c.heading
                 << " measured " << c.measured);
    const auto setpoint = OptimizeSwerveModule(c.speed, c.heading, c.measured);
    EXPECT_EQ(setpoint.speed, 0.0);
    EXPECT_EQ(setpoint.steer_delta_rad, 0.0);
    EXPECT_FALSE(setpoint.reversed);
  }
}

// REQ-DEV-26: exact half-turn (+-pi) reverses the drive with zero steering
// change; negative and zero asked speeds preserve sign/zero behavior.
TEST(OptimizeSwerveModule, ExactHalfTurnNegativeAndZeroSpeed) {
  for (const double half_turn : {kPi, -kPi}) {
    SCOPED_TRACE(half_turn);
    const auto sp = OptimizeSwerveModule(2.5, half_turn, 0.0);
    EXPECT_TRUE(sp.reversed);
    EXPECT_NEAR(sp.steer_delta_rad, 0.0, kTight);
    EXPECT_NEAR(sp.speed, -2.5, kTight);
  }

  // Negative asked speed: kept negative without flip, negated to positive on
  // flip.
  const auto neg_no_flip = OptimizeSwerveModule(-2.5, 0.4, 0.0);
  EXPECT_FALSE(neg_no_flip.reversed);
  EXPECT_NEAR(neg_no_flip.steer_delta_rad, 0.4, kTight);
  EXPECT_NEAR(neg_no_flip.speed, -2.5 * std::cos(0.4), kTight);

  const auto neg_flip = OptimizeSwerveModule(-2.5, kPi - 0.3, 0.0);
  EXPECT_TRUE(neg_flip.reversed);
  EXPECT_NEAR(neg_flip.steer_delta_rad, -0.3, kTight);
  EXPECT_NEAR(neg_flip.speed, 2.5 * std::cos(0.3), kTight);

  // Zero speed: stays zero with shortest steering path.
  const auto zero_no_flip = OptimizeSwerveModule(0.0, 0.4, 0.0);
  EXPECT_FALSE(zero_no_flip.reversed);
  EXPECT_NEAR(zero_no_flip.steer_delta_rad, 0.4, kTight);
  EXPECT_EQ(zero_no_flip.speed, 0.0);

  const auto zero_flip = OptimizeSwerveModule(0.0, kPi - 0.3, 0.0);
  EXPECT_TRUE(zero_flip.reversed);
  EXPECT_NEAR(zero_flip.steer_delta_rad, -0.3, kTight);
  EXPECT_EQ(zero_flip.speed, 0.0);
}

// REQ-DEV-26: across arbitrary speeds, headings and unwrapped measured angles,
// the steering change never exceeds a quarter turn and the resulting heading
// (accounting for reversal) matches the asked heading.
TEST(OptimizeSwerveModule, PropertySweepMaintainsQuarterTurnAndHeading) {
  std::mt19937_64 rng(0x5AE0F10EULL);
  std::uniform_real_distribution<double> speed_dist(-5.0, 5.0);
  std::uniform_real_distribution<double> asked_dist(-4.0 * kPi, 4.0 * kPi);
  std::uniform_real_distribution<double> measured_dist(-10.0 * kPi, 10.0 * kPi);
  for (int i = 0; i < 1000; ++i) {
    const double speed = speed_dist(rng);
    const double asked = asked_dist(rng);
    const double measured = measured_dist(rng);
    const auto sp = OptimizeSwerveModule(speed, asked, measured);
    ASSERT_TRUE(std::isfinite(sp.speed));
    ASSERT_TRUE(std::isfinite(sp.steer_delta_rad));
    EXPECT_LE(std::abs(sp.steer_delta_rad), kQuarter);
    const double effective_heading =
        measured + sp.steer_delta_rad + (sp.reversed ? kPi : 0.0);
    EXPECT_NEAR(SO2<double>(effective_heading).boxminus(SO2<double>(asked)),
                0.0, 1e-9);
    const double expected_speed =
        (sp.reversed ? -speed : speed) * std::cos(sp.steer_delta_rad);
    EXPECT_NEAR(sp.speed, expected_speed, 1e-9);
  }
}

}  // namespace
}  // namespace talos::geometry
