#include "talOS/node/allocation_probe.h"

#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <thread>

namespace talos::testing {
namespace {

// Volatile so the compiler cannot elide an allocation paired with its free
// (allowed for new/delete, and done for malloc).
void* (*volatile g_malloc)(std::size_t) = std::malloc;

// The node checks pass vacuously if the probe stops seeing allocations, and
// fail spuriously if it counts another thread's.
TEST(AllocationProbe, CountsEveryAllocationOnTheCallingThreadOnly) {
  if (const auto why = AllocationProbe::Unsupported(); !why.empty()) {
    GTEST_SKIP() << why;
  }
  // Started before the probe, since creating a thread allocates.
  std::atomic<int> phase{0};
  std::thread elsewhere{[&phase] {
    while (phase.load() != 1) std::this_thread::yield();
    for (int i = 0; i < 100; ++i) std::free(g_malloc(24));
    phase = 2;
  }};

  AllocationProbe probe;
  probe.start();
  std::free(g_malloc(24));
  int* volatile boxed = new int{1};
  delete boxed;
  phase = 1;
  while (phase.load() != 2) std::this_thread::yield();
  const std::uint64_t counted = probe.stop();
  elsewhere.join();

  EXPECT_EQ(counted, 2u) << probe.sites();
}

}  // namespace
}  // namespace talos::testing
