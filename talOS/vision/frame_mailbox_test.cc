#include "talOS/vision/frame_mailbox.h"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <thread>
#include <vector>

#include "talOS/vision/frame.h"

namespace talos::vision {
namespace {

constexpr uint32_t kWidth = 64;
constexpr uint32_t kHeight = 8;

// A borrowed frame whose every byte is the low byte of its sequence number.
Frame Borrowed(std::vector<uint8_t>& bytes, uint64_t sequence) {
  std::fill(bytes.begin(), bytes.end(), static_cast<uint8_t>(sequence));
  return Frame(
      0, 1, sequence,
      talos::event::MonotonicTime::from_nanos(static_cast<int64_t>(sequence)),
      kWidth, kHeight, kWidth, PixelFormat::kGray8, bytes.data(), bytes.size());
}

bool Uniform(const Frame& frame, uint8_t value) {
  const uint8_t* data = frame.data();
  return frame.holds_image() &&
         std::all_of(data, data + frame.size(),
                     [value](uint8_t byte) { return byte == value; });
}

// A producer faster than a consumer that is still processing: the processed
// frame is never overwritten, the next take is the newest frame, and each
// replaced frame is counted.
TEST(FrameMailboxTest, SlowConsumerGetsNewestFrameAndDropsAreCounted) {
  FrameMailbox mailbox;
  mailbox.Reserve(kWidth * kHeight);
  std::vector<uint8_t> driver(kWidth * kHeight);
  EXPECT_EQ(mailbox.TakeNewest(), nullptr);

  ASSERT_TRUE(mailbox.Publish(Borrowed(driver, 1)));
  const Frame* processing = mailbox.TakeNewest();
  ASSERT_NE(processing, nullptr);
  for (uint64_t sequence = 2; sequence <= 6; ++sequence) {
    ASSERT_TRUE(mailbox.Publish(Borrowed(driver, sequence)));
  }
  // The driver buffer was reused five times; the frame being processed holds
  // its own copy, untouched by the later publications.
  EXPECT_EQ(processing->sequence_number(), 1u);
  EXPECT_TRUE(Uniform(*processing, 1));

  const Frame* newest = mailbox.TakeNewest();
  ASSERT_NE(newest, nullptr);
  EXPECT_EQ(newest->sequence_number(), 6u);
  EXPECT_EQ(newest->capture_time().nanos(), 6);
  EXPECT_TRUE(Uniform(*newest, 6));
  EXPECT_EQ(mailbox.dropped(), 4u);  // Frames 2 to 5 were never taken.
  EXPECT_EQ(mailbox.TakeNewest(), nullptr);

  // A frame without an image, and a ready frame discarded with its capture
  // session, are drops too.
  EXPECT_FALSE(mailbox.Publish(Frame()));
  ASSERT_TRUE(mailbox.Publish(Borrowed(driver, 7)));
  mailbox.DiscardReady();
  EXPECT_EQ(mailbox.TakeNewest(), nullptr);
  EXPECT_EQ(mailbox.dropped(), 6u);
}

// Concurrent producer and consumer: no frame is torn, frames arrive in order,
// and every published frame is either taken or counted as dropped.
TEST(FrameMailboxTest, ConcurrentHandoffNeverTearsOrLosesCount) {
  constexpr uint64_t kFrames = 5000;
  FrameMailbox mailbox;
  mailbox.Reserve(kWidth * kHeight);
  std::atomic<bool> done{false};
  std::thread producer([&] {
    std::vector<uint8_t> driver(kWidth * kHeight);
    for (uint64_t sequence = 1; sequence <= kFrames; ++sequence) {
      mailbox.Publish(Borrowed(driver, sequence));
    }
    done.store(true, std::memory_order_release);
  });
  uint64_t taken = 0;
  uint64_t last = 0;
  bool torn = false;
  bool reordered = false;
  for (;;) {
    const bool finished = done.load(std::memory_order_acquire);
    while (const Frame* frame = mailbox.TakeNewest()) {
      ++taken;
      reordered |= frame->sequence_number() <= last;
      last = frame->sequence_number();
      torn |= !Uniform(*frame, static_cast<uint8_t>(last));
    }
    if (finished) break;
  }
  producer.join();
  EXPECT_FALSE(torn);
  EXPECT_FALSE(reordered);
  EXPECT_EQ(last, kFrames);
  EXPECT_EQ(taken + mailbox.dropped(), kFrames);
}

}  // namespace
}  // namespace talos::vision
