#include "talOS/node/allocation_probe.h"

#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <new>
#include <thread>

namespace talos::testing {
namespace {

// Volatile function pointers so the compiler cannot elide an allocation paired
// with its free (allowed for new/delete, and done for malloc/calloc/realloc).
void* (*volatile g_malloc)(std::size_t) = std::malloc;
void* (*volatile g_calloc)(std::size_t, std::size_t) = std::calloc;
void* (*volatile g_realloc)(void*, std::size_t) = std::realloc;
void* (*volatile g_aligned_alloc)(std::size_t,
                                  std::size_t) = std::aligned_alloc;
int (*volatile g_posix_memalign)(void**, std::size_t,
                                 std::size_t) = ::posix_memalign;
void* (*volatile g_aligned_new)(std::size_t, std::align_val_t) =
    [](std::size_t n, std::align_val_t a) { return ::operator new(n, a); };
void (*volatile g_aligned_delete)(void*, std::align_val_t) =
    [](void* p, std::align_val_t a) { ::operator delete(p, a); };

// The node checks pass vacuously if the probe stops seeing allocations, and
// fail spuriously if it counts another thread's.
// REQ-RT-06
TEST(AllocationProbe, CountsEveryAllocationOnTheCallingThreadOnly) {
  if (const auto why = AllocationProbe::Unsupported(); !why.empty()) {
    GTEST_SKIP() << why;
  }
  // Started before the probe, since creating a thread allocates.
  std::atomic<int> phase{0};
  std::thread elsewhere{[&phase] {
    while (phase.load(std::memory_order_acquire) != 1) {
      std::this_thread::yield();
    }
    for (int i = 0; i < 100; ++i) std::free(g_malloc(24));
    phase.store(2, std::memory_order_release);
  }};

  AllocationProbe probe;
  probe.start();
  std::free(g_malloc(24));
  int* volatile boxed = new int{1};
  delete boxed;
  phase.store(1, std::memory_order_release);
  while (phase.load(std::memory_order_acquire) != 2) {
    std::this_thread::yield();
  }
  const std::uint64_t counted = probe.stop();
  elsewhere.join();

  EXPECT_EQ(counted, 2u) << probe.sites();
}

// REQ-RT-06: AllocationProbe counts every C and C++ heap entry point
// (calloc, realloc, aligned_alloc, posix_memalign, and over-aligned operator
// new), so steady-state zero-allocation checks cannot miss any allocator path.
TEST(AllocationProbe, CountsCallocReallocAlignedAndOverAlignedNew) {
  if (const auto why = AllocationProbe::Unsupported(); !why.empty()) {
    GTEST_SKIP() << why;
  }
  AllocationProbe probe;

  // calloc
  probe.start();
  void* p_calloc = g_calloc(4, 16);
  const std::uint64_t calloc_count = probe.stop();
  ASSERT_NE(p_calloc, nullptr);
  EXPECT_GE(calloc_count, 1u) << probe.sites();

  // realloc (grow an existing allocation)
  probe.start();
  void* p_realloc = g_realloc(p_calloc, 512);
  const std::uint64_t realloc_count = probe.stop();
  ASSERT_NE(p_realloc, nullptr);
  EXPECT_GE(realloc_count, 1u) << probe.sites();
  std::free(p_realloc);

  // aligned_alloc
  probe.start();
  void* p_aligned = g_aligned_alloc(64, 128);
  const std::uint64_t aligned_count = probe.stop();
  ASSERT_NE(p_aligned, nullptr);
  EXPECT_GE(aligned_count, 1u) << probe.sites();
  std::free(p_aligned);

  // posix_memalign
  void* p_posix = nullptr;
  probe.start();
  const int pm_rc = g_posix_memalign(&p_posix, 64, 128);
  const std::uint64_t posix_count = probe.stop();
  ASSERT_EQ(pm_rc, 0);
  ASSERT_NE(p_posix, nullptr);
  EXPECT_GE(posix_count, 1u) << probe.sites();
  std::free(p_posix);

  // Over-aligned C++17 operator new (alignof == 64 >
  // __STDCPP_DEFAULT_NEW_ALIGNMENT__)
  struct alignas(64) OverAligned {
    std::uint64_t words[8];
  };
  probe.start();
  void* p_over = g_aligned_new(sizeof(OverAligned),
                               std::align_val_t{alignof(OverAligned)});
  const std::uint64_t over_count = probe.stop();
  ASSERT_NE(p_over, nullptr);
  EXPECT_GE(over_count, 1u) << probe.sites();
  g_aligned_delete(p_over, std::align_val_t{alignof(OverAligned)});
}

}  // namespace
}  // namespace talos::testing
