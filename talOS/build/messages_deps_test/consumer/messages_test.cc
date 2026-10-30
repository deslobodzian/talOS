// Round-trip of a struct embedded through a talos_messages `deps` edge:
// consumer/consumer.fbs includes deps_geometry/geometry.fbs, so this checks
// the C++ generated header names the dep's include prefix and the layout
// agrees with the Python bindings (both SizeOf 40, Pose2d 24).

#include <gtest/gtest.h>

#include "deps_consumer/consumer_generated.h"
#include "deps_consumer/topics.h"
#include "deps_geometry/geometry_generated.h"

namespace deps_consumer {
namespace {

TEST(MessagesDepsTest, EmbeddedStructRoundTrip) {
  const deps_geometry::Pose2d pose{1.0, 2.0, 0.5};
  EXPECT_DOUBLE_EQ(pose.x(), 1.0);
  EXPECT_DOUBLE_EQ(pose.y(), 2.0);
  EXPECT_DOUBLE_EQ(pose.heading_rad(), 0.5);

  const ConsumerState state{pose, 10.0, true};
  EXPECT_DOUBLE_EQ(state.pose().x(), 1.0);
  EXPECT_DOUBLE_EQ(state.pose().y(), 2.0);
  EXPECT_DOUBLE_EQ(state.pose().heading_rad(), 0.5);
  EXPECT_DOUBLE_EQ(state.velocity_rps(), 10.0);
  EXPECT_TRUE(state.enabled());
}

TEST(MessagesDepsTest, LayoutMatchesPythonBindings) {
  // Pose2d is 3 doubles; ConsumerState is Pose2d + double + bool padded to 8.
  EXPECT_EQ(sizeof(deps_geometry::Pose2d), 24u);
  EXPECT_EQ(sizeof(ConsumerState), 40u);
  EXPECT_STREQ(topics::kState, "/deps_consumer/state");
}

}  // namespace
}  // namespace deps_consumer
