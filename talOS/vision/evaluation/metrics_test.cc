// The tracking statistics recording_eval reports: an id switch is counted
// only between detections of one physical tag, a dropout only while the tag
// was in view on both sides, and jitter excludes constant-velocity motion.

#include "talOS/vision/evaluation/metrics.h"

#include <gtest/gtest.h>

#include <optional>
#include <vector>

namespace talos::vision::evaluation {
namespace {

TEST(MetricsTest, IdSwitchNeedsTheSameTagNotAnotherTagNearby) {
  const std::vector<TagSighting> before{{3, {100, 100}, 40},
                                        {4, {300, 100}, 40}};
  // Tag 3 moved 15 px (under half its side) and decoded as 5: one switch.
  // Tag 4 moved 30 px, beyond half its side: not associated, not a switch.
  const std::vector<TagSighting> after{{5, {115, 100}, 40},
                                       {9, {330, 100}, 40}};
  EXPECT_EQ(CountIdSwitches(before, after), 1);
  // Two tags crossing in one frame: each associates with its nearer self.
  const std::vector<TagSighting> crossing{{4, {290, 100}, 40},
                                          {3, {105, 100}, 40}};
  EXPECT_EQ(CountIdSwitches(before, crossing), 0);
}

TEST(MetricsTest, DropoutIsAGapBetweenSightingsNotEntryOrExit) {
  EXPECT_EQ(LongestDropout({false, false, true, false, false, false, true, true,
                            false, true, false, false, false, false}),
            3u);
  EXPECT_EQ(LongestDropout({false, true, true, false}), 0u);
}

TEST(MetricsTest, JitterIgnoresConstantVelocityAndSkipsGaps) {
  using V = Eigen::Matrix<double, 1, 1>;
  std::vector<std::optional<V>> track;
  // Constant velocity with +-1 alternating noise: residual 1, 2, 2, ...
  for (int i = 0; i < 6; ++i) track.push_back(V(10.0 * i + (i % 2 ? 1 : -1)));
  track.push_back(std::nullopt);  // a dropout ends the run
  track.push_back(V(1000.0));
  track.push_back(V(-1000.0));
  const auto jitter = SecondDifferenceJitter<1>(track);
  EXPECT_EQ(jitter.samples, 4u);
  EXPECT_DOUBLE_EQ(jitter.rms_per_axis, 2.0);
}

}  // namespace
}  // namespace talos::vision::evaluation
