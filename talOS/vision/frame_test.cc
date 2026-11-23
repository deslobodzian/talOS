#include "talOS/vision/frame.h"

#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

namespace talos::vision {
namespace {

TEST(FrameTest, AllocatesPackedImageStorage) {
  const auto t0 = talos::event::MonotonicTime::from_nanos(123456789);
  Frame f(1, 42, 100, t0, 640, 480, PixelFormat::kRgb8);

  EXPECT_EQ(f.stride(), 640 * 3);
  EXPECT_EQ(f.size(), 640 * 3 * 480);
  EXPECT_FALSE(f.empty());
  EXPECT_TRUE(f.holds_image());
  EXPECT_NE(f.data(), nullptr);
  EXPECT_NE(f.mutable_data(), nullptr);
  EXPECT_EQ(reinterpret_cast<uintptr_t>(f.data()) % 4096, 0u);
}

TEST(FrameTest, CloneAndShareSemantics) {
  Frame orig(0, 1, 10, talos::event::MonotonicTime::from_nanos(100), 10, 10,
             PixelFormat::kGray8);
  orig.mutable_data()[0] = 42;
  orig.mutable_data()[99] = 99;

  // Share buffer
  Frame shared = orig.Share();
  EXPECT_TRUE(shared.is_shared());
  EXPECT_EQ(shared.data()[0], 42);
  EXPECT_EQ(shared.data()[99], 99);
  EXPECT_EQ(shared.data(), orig.data());

  // Mutating shared frame causes copy-on-write
  shared.mutable_data()[0] = 100;
  EXPECT_FALSE(shared.is_shared());
  EXPECT_EQ(shared.data()[0], 100);
  EXPECT_EQ(shared.data()[99], 99);  // Unmodified byte preserved across COW
  EXPECT_EQ(orig.data()[0], 42);     // Original unchanged

  // Clone frame creates independent copy
  Frame cloned = orig.Clone();
  EXPECT_FALSE(cloned.is_shared());
  EXPECT_EQ(cloned.data()[0], 42);
  EXPECT_EQ(cloned.data()[99], 99);
  EXPECT_NE(cloned.data(), orig.data());
  cloned.mutable_data()[0] = 77;
  EXPECT_EQ(cloned.data()[0], 77);
  EXPECT_EQ(orig.data()[0], 42);
}

TEST(FrameTest, CopyAndCowPreservePixelsAcrossHeapAlignments) {
  constexpr uint32_t kWidth = 64;
  constexpr uint32_t kHeight = 32;
  constexpr size_t kBytes = static_cast<size_t>(kWidth) * kHeight;
  const auto t0 = talos::event::MonotonicTime::from_nanos(500);

  Frame orig(2, 7, 11, t0, kWidth, kHeight, PixelFormat::kGray8);
  ASSERT_EQ(orig.size(), kBytes);
  ASSERT_EQ(reinterpret_cast<uintptr_t>(orig.data()) % 4096, 0u);
  for (size_t i = 0; i < kBytes; ++i) {
    orig.mutable_data()[i] = static_cast<uint8_t>((i * 37 + 13) & 0xFF);
  }

  // Keep multiple copies alive simultaneously so heap allocations land at
  // differing (addr % 4096) offsets.
  std::vector<Frame> copies;
  copies.reserve(8);
  for (int c = 0; c < 8; ++c) {
    copies.push_back(orig);
    const Frame& copy = copies.back();
    ASSERT_NE(copy.data(), orig.data());
    EXPECT_EQ(reinterpret_cast<uintptr_t>(copy.data()) % 4096, 0u);
    for (size_t i = 0; i < kBytes; ++i) {
      ASSERT_EQ(copy.data()[i], static_cast<uint8_t>((i * 37 + 13) & 0xFF))
          << "copy " << c << " byte " << i;
    }
  }

  // Copy assignment preserves every pixel too.
  Frame assigned;
  assigned = orig;
  ASSERT_NE(assigned.data(), orig.data());
  EXPECT_EQ(reinterpret_cast<uintptr_t>(assigned.data()) % 4096, 0u);
  for (size_t i = 0; i < kBytes; ++i) {
    ASSERT_EQ(assigned.data()[i], static_cast<uint8_t>((i * 37 + 13) & 0xFF))
        << "assigned byte " << i;
  }

  // Share from a genuinely const Frame (no const_cast UB) and COW-mutate
  // across multiple live unshared copies.
  const Frame const_orig = orig;
  std::vector<Frame> cow_copies;
  cow_copies.reserve(8);
  for (int c = 0; c < 8; ++c) {
    Frame shared = const_orig.Share();
    ASSERT_TRUE(shared.is_shared());
    ASSERT_EQ(shared.data(), const_orig.data());
    shared.mutable_data()[0] = static_cast<uint8_t>(c);
    ASSERT_FALSE(shared.is_shared());
    EXPECT_EQ(reinterpret_cast<uintptr_t>(shared.data()) % 4096, 0u);
    EXPECT_EQ(shared.data()[0], static_cast<uint8_t>(c));
    for (size_t i = 1; i < kBytes; ++i) {
      ASSERT_EQ(shared.data()[i], static_cast<uint8_t>((i * 37 + 13) & 0xFF))
          << "cow " << c << " byte " << i;
    }
    cow_copies.push_back(std::move(shared));
  }
  EXPECT_EQ(const_orig.data()[0], static_cast<uint8_t>(13));

  // Caller-supplied vector whose size exceeds required + 4096 must start at
  // byte 0 rather than shifting by an implicit alignment offset.
  std::vector<uint8_t> oversized(kBytes + 4096);
  for (size_t i = 0; i < kBytes; ++i) {
    oversized[i] = static_cast<uint8_t>((i * 19 + 5) & 0xFF);
  }
  Frame from_vec(0, 1, 2, t0, kWidth, kHeight, kWidth, PixelFormat::kGray8,
                 std::move(oversized));
  for (size_t i = 0; i < kBytes; ++i) {
    ASSERT_EQ(from_vec.data()[i], static_cast<uint8_t>((i * 19 + 5) & 0xFF))
        << "from_vec byte " << i;
  }
}

TEST(FrameTest, CaptureMidpointGeometryAndOverflowBoundaries) {
  constexpr uint64_t kMax = std::numeric_limits<uint64_t>::max();

  // Geometry per TimestampKind with latency_offset_ns subtracted from all.
  EXPECT_EQ(CaptureMidpoint(1000, TimestampKind::kExposure, 400, 200, 50),
            950u);
  EXPECT_EQ(CaptureMidpoint(1000, TimestampKind::kDelivery, 400, 200, 50),
            950u);
  EXPECT_EQ(CaptureMidpoint(1000, TimestampKind::kPresentation, 400, 200, 50),
            950u);
  EXPECT_EQ(
      CaptureMidpoint(1000, TimestampKind::kStartOfExposure, 400, 200, 50),
      1150u);
  EXPECT_EQ(CaptureMidpoint(1000, TimestampKind::kEndOfFrame, 400, 200, 50),
            550u);

  // Exact zero result is valid, one nanosecond earlier underflows.
  EXPECT_EQ(CaptureMidpoint(450, TimestampKind::kEndOfFrame, 400, 200, 50), 0u);
  EXPECT_EQ(CaptureMidpoint(449, TimestampKind::kEndOfFrame, 400, 200, 50),
            std::nullopt);
  EXPECT_EQ(CaptureMidpoint(49, TimestampKind::kExposure, 0, 0, 50),
            std::nullopt);

  // Overflow guards on kStartOfExposure and kEndOfFrame.
  EXPECT_EQ(CaptureMidpoint(kMax, TimestampKind::kStartOfExposure, 2, 0, 0),
            std::nullopt);
  EXPECT_EQ(CaptureMidpoint(kMax, TimestampKind::kEndOfFrame, 2, kMax, 0),
            std::nullopt);
  EXPECT_EQ(CaptureMidpoint(kMax, TimestampKind::kEndOfFrame, 0, kMax - 10, 20),
            std::nullopt);
}

TEST(FrameTest, HoldsImageValidatesDimensionsStrideAndStorage) {
  const auto t0 = talos::event::MonotonicTime::from_nanos(1);

  EXPECT_FALSE(Frame().holds_image());
  EXPECT_FALSE(Frame(0, 1, 1, t0, 0, 10, PixelFormat::kGray8).holds_image());
  EXPECT_FALSE(Frame(0, 1, 1, t0, 10, 0, PixelFormat::kGray8).holds_image());

  // Stride shorter than width * bytes_per_pixel is rejected.
  std::vector<uint8_t> raw(100, 0);
  EXPECT_FALSE(
      Frame(0, 1, 1, t0, 10, 4, 29, PixelFormat::kRgb8, raw).holds_image());
  EXPECT_FALSE(
      Frame(0, 1, 1, t0, 10, 4, 19, PixelFormat::kYuyv, raw).holds_image());
  EXPECT_FALSE(
      Frame(0, 1, 1, t0, 10, 4, 39, PixelFormat::kBgra8, raw).holds_image());

  // Truncated owned, shared, and borrowed buffers are rejected.
  EXPECT_FALSE(Frame(0, 1, 1, t0, 10, 10, 10, PixelFormat::kGray8,
                     std::vector<uint8_t>(99))
                   .holds_image());
  EXPECT_FALSE(Frame(0, 1, 1, t0, 10, 10, 10, PixelFormat::kGray8,
                     std::make_shared<const std::vector<uint8_t>>(99))
                   .holds_image());
  EXPECT_FALSE(
      Frame(0, 1, 1, t0, 10, 10, 10, PixelFormat::kGray8, raw.data(), 99)
          .holds_image());
  EXPECT_FALSE(Frame(0, 1, 1, t0, 10, 10, 10, PixelFormat::kGray8, nullptr, 100)
                   .holds_image());

  // Exact-fit buffer holds an image.
  EXPECT_TRUE(
      Frame(0, 1, 1, t0, 10, 10, 10, PixelFormat::kGray8, raw.data(), 100)
          .holds_image());

  // Alignment padding must not disguise an undersized buffer if dimensions
  // grow.
  Frame allocated(0, 1, 1, t0, 10, 10, PixelFormat::kGray8);
  EXPECT_TRUE(allocated.holds_image());
  allocated.set_height(11);
  EXPECT_FALSE(allocated.holds_image());
}

TEST(FrameTest, ConcurrentConstShareIsRaceFreeAndPreservesCow) {
  constexpr int kThreads = 8;
  constexpr int kRounds = 64;
  constexpr uint32_t kWidth = 32;
  constexpr uint32_t kHeight = 16;
  constexpr size_t kBytes = static_cast<size_t>(kWidth) * kHeight;
  const auto t0 = talos::event::MonotonicTime::from_nanos(777);

  for (int round = 0; round < kRounds; ++round) {
    Frame mutable_frame(0, 1, static_cast<uint64_t>(round), t0, kWidth, kHeight,
                        PixelFormat::kGray8);
    for (size_t i = 0; i < kBytes; ++i) {
      mutable_frame.mutable_data()[i] =
          static_cast<uint8_t>((i + static_cast<size_t>(round)) & 0xFF);
    }
    const Frame& orig = mutable_frame;
    const uint8_t* expected_ptr = orig.data();

    std::atomic<int> ready{0};
    std::atomic<bool> go{false};
    std::atomic<int> failures{0};
    std::vector<std::thread> threads;
    threads.reserve(kThreads);
    for (int t = 0; t < kThreads; ++t) {
      threads.emplace_back([&]() {
        ready.fetch_add(1, std::memory_order_release);
        while (!go.load(std::memory_order_acquire)) {
          std::this_thread::yield();
        }
        Frame shared = orig.Share();
        if (!shared.is_shared() || !shared.holds_image() ||
            shared.data() != expected_ptr ||
            shared.data()[0] != static_cast<uint8_t>(round & 0xFF) ||
            shared.data()[kBytes - 1] !=
                static_cast<uint8_t>((kBytes - 1 + round) & 0xFF)) {
          failures.fetch_add(1, std::memory_order_relaxed);
        }
      });
    }
    while (ready.load(std::memory_order_acquire) < kThreads) {
      std::this_thread::yield();
    }
    go.store(true, std::memory_order_release);
    for (auto& th : threads) th.join();
    ASSERT_EQ(failures.load(), 0) << "round " << round;
  }

  // Mutating the original frame after Share() must copy-on-write without
  // mutating previously shared handles.
  Frame orig(0, 1, 1, t0, kWidth, kHeight, PixelFormat::kGray8);
  orig.mutable_data()[0] = 11;
  orig.mutable_data()[kBytes - 1] = 22;
  const Frame shared = orig.Share();
  EXPECT_EQ(shared.data(), orig.data());
  orig.mutable_data()[0] = 99;
  EXPECT_NE(shared.data(), orig.data());
  EXPECT_EQ(shared.data()[0], 11);
  EXPECT_EQ(shared.data()[kBytes - 1], 22);
  EXPECT_EQ(orig.data()[0], 99);
  EXPECT_EQ(orig.data()[kBytes - 1], 22);
}

}  // namespace
}  // namespace talos::vision
