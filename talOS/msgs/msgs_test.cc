#include <gtest/gtest.h>

#include "control/control_generated.h"
#include "geometry/geometry_generated.h"
#include "talOS/node/node.h"

namespace talos::msgs {
namespace {
// PoseEstimate's shared-memory slot contract is recorded in docs/DATA_FLOW.md.
static_assert(sizeof(geometry::PoseEstimate) == 72,
              "PoseEstimate size changed; update docs/DATA_FLOW.md");
static_assert(alignof(geometry::PoseEstimate) == 8,
              "PoseEstimate alignment changed; update docs/DATA_FLOW.md");

TEST(StandardMessages, WireLayoutCompatibility) {
  EXPECT_TRUE(Message<geometry::Translation2d>);
  EXPECT_TRUE(Message<geometry::Rotation2d>);
  EXPECT_TRUE(Message<geometry::Pose2d>);
  EXPECT_TRUE(Message<geometry::Twist2d>);
  EXPECT_TRUE(Message<geometry::OdometrySample>);
  EXPECT_TRUE(Message<geometry::SwerveModulePosition>);
  EXPECT_TRUE(Message<geometry::PoseObservation>);
  EXPECT_TRUE(Message<geometry::PoseEstimate>);
  EXPECT_TRUE(Message<control::VelocityTarget>);
  EXPECT_TRUE(Message<control::VelocityState>);
  EXPECT_TRUE(Message<control::PositionTarget>);
  EXPECT_TRUE(Message<control::PositionState>);
}

}  // namespace
}  // namespace talos::msgs
