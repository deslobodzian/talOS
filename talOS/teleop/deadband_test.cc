#include "talOS/teleop/deadband.h"

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

// matrix: n/a 1, 2, 6, 9, 11, 12 (deadband.h unit test; node in node_test.cc)

namespace talos::teleop {
namespace {

// REQ-CONTRACT-24, REQ-SAFE-14
TEST(DeadbandTest, BoundaryTransitionsAndContinuousRescaling) {
  constexpr double kDb = 0.1;
  EXPECT_DOUBLE_EQ(Deadband(0.0, kDb), 0.0);
  EXPECT_DOUBLE_EQ(Deadband(0.05, kDb), 0.0);
  EXPECT_DOUBLE_EQ(Deadband(-0.05, kDb), 0.0);
  // Inclusive threshold edge.
  EXPECT_DOUBLE_EQ(Deadband(kDb, kDb), 0.0);
  EXPECT_DOUBLE_EQ(Deadband(-kDb, kDb), 0.0);

  // Continuous just outside the threshold (no step discontinuity).
  const double just_above = std::nextafter(kDb, 1.0);
  const double just_below = std::nextafter(-kDb, -1.0);
  EXPECT_GT(Deadband(just_above, kDb), 0.0);
  EXPECT_LT(Deadband(just_above, kDb), 1e-12);
  EXPECT_LT(Deadband(just_below, kDb), 0.0);
  EXPECT_GT(Deadband(just_below, kDb), -1e-12);

  // Linear ramp to full scale on both signs.
  EXPECT_DOUBLE_EQ(Deadband(0.55, kDb), 0.5);
  EXPECT_DOUBLE_EQ(Deadband(-0.55, kDb), -0.5);
  EXPECT_DOUBLE_EQ(Deadband(1.0, kDb), 1.0);
  EXPECT_DOUBLE_EQ(Deadband(-1.0, kDb), -1.0);

  // Zero deadband passes through unchanged.
  EXPECT_DOUBLE_EQ(Deadband(0.0, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(Deadband(0.25, 0.0), 0.25);
  EXPECT_DOUBLE_EQ(Deadband(-0.25, 0.0), -0.25);
}

// REQ-CONTRACT-24
TEST(DeadbandTest, DeadbandAtOrAboveOneGuardsAgainstDivisionByZero) {
  constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
  constexpr double kInf = std::numeric_limits<double>::infinity();
  for (const double db : {1.0, 1.5, kInf}) {
    for (const double x :
         {0.0, 0.5, -0.5, 1.0, -1.0, 2.0, -2.0, kNan, kInf, -kInf}) {
      EXPECT_DOUBLE_EQ(Deadband(x, db), 0.0) << "x=" << x << " db=" << db;
    }
  }
}

// REQ-SAFE-14
TEST(DeadbandTest, NonFiniteInputsPropagateForValidDeadband) {
  constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
  constexpr double kInf = std::numeric_limits<double>::infinity();
  EXPECT_TRUE(std::isnan(Deadband(kNan, 0.1)));
  const double pos_inf = Deadband(kInf, 0.1);
  EXPECT_TRUE(std::isinf(pos_inf));
  EXPECT_GT(pos_inf, 0.0);
  const double neg_inf = Deadband(-kInf, 0.1);
  EXPECT_TRUE(std::isinf(neg_inf));
  EXPECT_LT(neg_inf, 0.0);
}

// REQ-SAFE-14, REQ-CONTRACT-24
TEST(DeadbandTest, ArmingNeutralRejectsNonFiniteAndMatchesDeadbandZeroZone) {
  constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
  constexpr double kInf = std::numeric_limits<double>::infinity();
  for (const double db : {0.0, 0.05, 0.2, 1.0}) {
    EXPECT_FALSE(ArmingNeutral(kNan, db));
    EXPECT_FALSE(ArmingNeutral(kInf, db));
    EXPECT_FALSE(ArmingNeutral(-kInf, db));
  }

  constexpr double kDb = 0.125;
  EXPECT_TRUE(ArmingNeutral(0.0, kDb));
  EXPECT_TRUE(ArmingNeutral(kDb, kDb));
  EXPECT_TRUE(ArmingNeutral(-kDb, kDb));
  EXPECT_FALSE(ArmingNeutral(std::nextafter(kDb, 1.0), kDb));
  EXPECT_FALSE(ArmingNeutral(std::nextafter(-kDb, -1.0), kDb));
  EXPECT_TRUE(ArmingNeutral(0.0, 0.0));
  EXPECT_FALSE(ArmingNeutral(std::nextafter(0.0, 1.0), 0.0));

  // For every finite reading and valid deadband < 1, ArmingNeutral is true iff
  // Deadband maps that reading to zero.
  for (const double x : {-1.0, std::nextafter(-kDb, -1.0), -kDb, -0.05, 0.0,
                         0.05, kDb, std::nextafter(kDb, 1.0), 1.0}) {
    EXPECT_EQ(ArmingNeutral(x, kDb), Deadband(x, kDb) == 0.0) << "x=" << x;
  }
}

// REQ-SAFE-14, REQ-CONTRACT-24
TEST(DeadbandTest, NegativeAndNanDeadbandFailSafeToZeroAndNeverArm) {
  constexpr double kNan = std::numeric_limits<double>::quiet_NaN();
  constexpr double kInf = std::numeric_limits<double>::infinity();
  for (const double db : {-0.1, -1.0, -kInf, kNan}) {
    for (const double x : {0.0, -0.0, 0.5, -0.5, 1.0, kNan, kInf, -kInf}) {
      EXPECT_DOUBLE_EQ(Deadband(x, db), 0.0) << "x=" << x << " db=" << db;
    }
    EXPECT_FALSE(ArmingNeutral(0.0, db)) << "db=" << db;
  }
}

}  // namespace
}  // namespace talos::teleop
