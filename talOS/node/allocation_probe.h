#pragma once

// talos::testing::AllocationProbe: counts the heap allocations one thread
// makes between start() and stop(), for RULES.md 3 checks that a node's
// update() allocates nothing in steady state.
//
//   if (const auto why = AllocationProbe::Unsupported(); !why.empty()) {
//     GTEST_SKIP() << why;
//   }
//   ... warm up: step the Sim with a live target ...
//   AllocationProbe probe;
//   probe.start();
//   sim.step(60ms);
//   EXPECT_EQ(probe.stop(), 0u) << probe.sites();
//
// Every malloc-family call counts, from any library, including those made by
// operator new: macOS reports them through libsystem_malloc's malloc_logger,
// and on glibc this library (linked into the test binary) replaces malloc,
// calloc, realloc and the aligned allocators with counting wrappers around
// glibc's own. Only the thread that called start() is counted, so a node's
// helper threads (a camera worker) are not. A simulated step on that thread
// counts the node, its loop and SimHardware, whose steady state allocates
// nothing (Sim topics are looked up without building a key string); build
// topic strings and read results outside the window.
// A sanitizer's allocator bypasses both hooks, so the probe reports itself
// unsupported there, as on any other platform.

#include <cstdint>
#include <string>
#include <string_view>

namespace talos::testing {

class AllocationProbe {
 public:
  // Empty when this build counts allocations; otherwise why it cannot.
  static std::string_view Unsupported();

  AllocationProbe() = default;
  ~AllocationProbe();
  AllocationProbe(const AllocationProbe&) = delete;
  AllocationProbe& operator=(const AllocationProbe&) = delete;

  // Counts allocations made on the calling thread until stop(). Only one
  // probe counts at a time; throws std::logic_error if another is counting
  // or this build is unsupported.
  void start();
  // Stops counting and returns the allocations since start().
  std::uint64_t stop();

  // The call stacks of the first allocations counted, symbolized, for a
  // failure message. Call after stop() (it allocates).
  std::string sites() const;

 private:
  bool counting_{false};
};

}  // namespace talos::testing
