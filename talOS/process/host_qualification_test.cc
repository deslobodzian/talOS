#include "talOS/process/host_qualification.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <stdexcept>
#include <vector>

namespace talos::process {
namespace {

TEST(HostQualificationTest, ComputePercentilesRejectsEmptyInput) {
  EXPECT_THROW(static_cast<void>(ComputePercentiles({})),
               std::invalid_argument);
}

TEST(HostQualificationTest, ComputePercentilesHandlesSingleElement) {
  const DurationPercentiles p = ComputePercentiles({42'500});
  EXPECT_DOUBLE_EQ(p.p50_us, 42.5);
  EXPECT_DOUBLE_EQ(p.p95_us, 42.5);
  EXPECT_DOUBLE_EQ(p.p99_us, 42.5);
  EXPECT_DOUBLE_EQ(p.max_us, 42.5);
}

TEST(HostQualificationTest, ComputePercentilesSortsAndComputesNearestRank) {
  std::vector<std::int64_t> samples;
  samples.reserve(100);
  // Reverse order to prove ComputePercentiles sorts before indexing.
  for (std::int64_t i = 100; i >= 1; --i) {
    samples.push_back(i * 1'000);
  }
  const DurationPercentiles p = ComputePercentiles(std::move(samples));
  EXPECT_DOUBLE_EQ(p.p50_us, 50.0);
  EXPECT_DOUBLE_EQ(p.p95_us, 95.0);
  EXPECT_DOUBLE_EQ(p.p99_us, 99.0);
  EXPECT_DOUBLE_EQ(p.max_us, 100.0);
}

TEST(HostQualificationTest, StallMeterTakesAtLeastOneSampleOnImmediateStop) {
  StallMeter meter;
  const std::uint64_t worst = meter.Stop();
  EXPECT_GE(meter.samples(), 1u);
  // Stopping a second time is a no-op returning the same recorded overrun.
  EXPECT_EQ(meter.Stop(), worst);
}

}  // namespace
}  // namespace talos::process
