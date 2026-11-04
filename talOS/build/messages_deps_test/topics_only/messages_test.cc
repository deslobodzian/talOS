// A topics-only talos_messages target: topics.h, and the dep's generated
// types through the same label (no direct dep on the geometry package).

#include <gtest/gtest.h>

#include "deps_geometry/geometry_generated.h"
#include "deps_topics_only/topics.h"

namespace deps_topics_only {
namespace {

TEST(TopicsOnlyMessagesTest, TopicsAndDepTypesComeThroughOneLabel) {
  EXPECT_STREQ(topics::kState, "/deps_topics_only/state");
  const deps_geometry::Pose2d pose{1.0, 2.0, 0.5};
  EXPECT_DOUBLE_EQ(pose.heading_rad(), 0.5);
}

}  // namespace
}  // namespace deps_topics_only
