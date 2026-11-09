#include <gtest/gtest.h>

#include "control/control_generated.h"
#include "geometry/geometry_generated.h"
#include "talOS/node/node.h"

namespace talos::msgs {
namespace {

TEST(StandardMessages, WireLayoutCompatibility) {
  EXPECT_TRUE(Message<geometry::Translation2d>);
  EXPECT_TRUE(Message<geometry::Rotation2d>);
  EXPECT_TRUE(Message<geometry::Pose2d>);
  EXPECT_TRUE(Message<geometry::Twist2d>);
  EXPECT_TRUE(Message<geometry::OdometrySample>);
  EXPECT_TRUE(Message<geometry::SwerveModulePosition>);
  EXPECT_TRUE(Message<geometry::PoseObservation>);
  EXPECT_TRUE(Message<geometry::PoseEstimate>);
  // PoseEstimate's shared-memory ABI is recorded in docs/DATA_FLOW.md.
  EXPECT_EQ(sizeof(geometry::PoseEstimate), 72u);
  EXPECT_EQ(alignof(geometry::PoseEstimate), 8u);
  EXPECT_TRUE(Message<control::VelocityTarget>);
  EXPECT_TRUE(Message<control::VelocityState>);
  EXPECT_TRUE(Message<control::PositionTarget>);
  EXPECT_TRUE(Message<control::PositionState>);
  EXPECT_TRUE(Message<control::ChassisTarget>);
}

}  // namespace
}  // namespace talos::msgs
