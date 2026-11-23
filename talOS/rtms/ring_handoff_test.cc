// The ring's publish hand-off with writer and readers on one mapping, the only
// arrangement in which the thread sanitizer can judge it (see the test).

#include <gtest/gtest.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <format>
#include <latch>
#include <optional>
#include <span>
#include <string>
#include <thread>

#include "talOS/rtms/ring_test_payload.h"
#include "talOS/rtms/rtms.h"
#include "talOS/schema/identity.h"

namespace {

using talos::rtms_test::FillRingPayload;
using talos::rtms_test::FirstForeignWord;
using talos::rtms_test::RingPublishTime;

constexpr std::size_t kWords = 32;  // a 256-byte message
using Payload = std::array<std::uint64_t, kWords>;

const talos::schema::Identity kSchema{"test.RingWords", 0x52494e47};

// REQ-CONTRACT-36: every reader of a topic receives every message within the
// overwrite window, intact.
//
// Writer and readers share one RTMSQueue, so one mapping of the segment. The
// thread sanitizer tracks memory by virtual address: two queues in one process
// map the segment twice, at two addresses, and it then never sees a reader's
// slot read and the writer's slot write as the same bytes. Here it does.
//
// The run stays inside the overwrite window (kSlots - 1 publications, so no
// slot is written twice), where the ring promises a hand-off free of races: a
// reader copies a slot only after acquiring the sequence the writer released
// once the slot was filled. Under --config=tsan a publication made visible
// before its slot is filled is reported as a data race whether or not a copy
// happens to tear; the payload check is the oracle the plain build has. Slot
// reuse when a writer laps its reader (using relaxed atomic word copies under
// the seqlock) is covered under ThreadSanitizer by //talOS/rtms:ring_lap_test.
TEST(RingHandoff, EveryReaderGetsEveryMessageIntactOnOneMapping) {
  constexpr std::size_t kSlots = 1024;
  constexpr std::uint64_t kMessages = kSlots - 1;
  constexpr std::size_t kReaders = 3;

  RTMSQueue queue{
      "/ring_handoff_" + std::to_string(::getpid()),
      sizeof(Payload),
      alignof(Payload),
      kSchema,
      kSlots,
      RTMSOptions{.overflow_policy = OverflowPolicy::OVERWRITE_OLDEST,
                  .read_mode = ReadMode::SEQUENCE,
                  .publisher = true},
  };

  std::array<std::size_t, kReaders> reader_ids{};
  for (auto& id : reader_ids) {
    const std::optional<std::size_t> registered = queue.register_reader();
    ASSERT_TRUE(registered.has_value());
    id = registered.value();
  }

  std::latch start{static_cast<std::ptrdiff_t>(kReaders) + 1};
  std::atomic<bool> writer_done{false};
  std::array<std::uint64_t, kReaders> received{};
  std::array<std::string, kReaders> failure{};

  std::array<std::thread, kReaders> readers;
  for (std::size_t r = 0; r < kReaders; ++r) {
    readers[r] = std::thread{[&, r] {
      start.arrive_and_wait();
      Payload copy{};
      bool final_read = false;
      while (received[r] < kMessages) {
        // Reading done after the writer finished: the last chance to see
        // what it published.
        final_read = writer_done.load(std::memory_order_acquire);
        MessageInfo info{};
        const ReadResult result = queue.read_next(
            reader_ids[r], std::as_writable_bytes(std::span{copy}), info);
        if (result == ReadResult::EMPTY) {
          if (final_read) {
            failure[r] = "caught up with a finished writer";
            break;
          }
          std::this_thread::yield();
          continue;
        }
        const std::uint64_t expected = received[r];
        if (result != ReadResult::OK || info.sequence != expected ||
            info.dropped != 0 ||
            info.publish_time_ns != RingPublishTime(expected) ||
            FirstForeignWord(copy, expected) != kWords) {
          failure[r] = std::format(
              "message {}: result {}, sequence {}, dropped {}, publish time "
              "{}, first foreign word {}",
              expected, static_cast<int>(result), info.sequence, info.dropped,
              info.publish_time_ns, FirstForeignWord(copy, expected));
          break;
        }
        ++received[r];
      }
    }};
  }

  start.arrive_and_wait();
  Payload message{};
  std::uint64_t written = 0;
  for (; written < kMessages; ++written) {
    FillRingPayload(message, written);
    if (queue.write(std::as_bytes(std::span{message}), RingPublishTime(written))
            .result != WriteResult::SUCCESS) {
      break;
    }
  }
  writer_done.store(true, std::memory_order_release);

  for (auto& reader : readers) reader.join();
  EXPECT_EQ(written, kMessages) << "the writer was refused a publication";
  for (std::size_t r = 0; r < kReaders; ++r) {
    SCOPED_TRACE(std::format("reader {}", r));
    EXPECT_EQ(failure[r], "");
    EXPECT_EQ(received[r], kMessages);
  }
}

}  // namespace
