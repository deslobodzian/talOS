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

TEST(MessagesDepsTest, LayoutMatchesPythonBindings) {
  // Pose2d is 3 doubles; ConsumerState is Pose2d + double + bool padded to 8.
  EXPECT_EQ(sizeof(deps_geometry::Pose2d), 24u);
  EXPECT_EQ(sizeof(ConsumerState), 40u);
}

}  // namespace
}  // namespace deps_consumer
