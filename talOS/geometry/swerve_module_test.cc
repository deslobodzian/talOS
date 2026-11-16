#include "talOS/geometry/swerve_module.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <numbers>

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

// REQ-DEV-26: a wheel one radian off its heading drives at cos(1) of the
// asked speed.
TEST(OptimizeSwerveModule, ScalesSpeedByCosineOfSteeringChange) {
  const auto setpoint = OptimizeSwerveModule(3.0, 1.0, 0.0);
  EXPECT_FALSE(setpoint.reversed);
  EXPECT_NEAR(setpoint.steer_delta_rad, 1.0, kTight);
  EXPECT_NEAR(setpoint.speed, 3.0 * std::cos(1.0), kTight);
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

}  // namespace
}  // namespace talos::geometry
