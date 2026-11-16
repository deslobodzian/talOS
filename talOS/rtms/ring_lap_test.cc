// The ring's torn-read guard under a writer that laps its readers. Not run
// under the thread sanitizer: the copy it guards is a race by design (below).

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

// A 4 KiB message: the longer the copy, the more often the writer reaches the
// slot while a reader is still copying it.
constexpr std::size_t kWords = 512;
using Payload = std::array<std::uint64_t, kWords>;

const talos::schema::Identity kSchema{"test.RingWords", 0x52494e47};

struct ReaderResult {
  std::uint64_t accepted{0};
  std::uint64_t dropped{0};
  std::optional<std::uint64_t> last;
  std::string failure;
};

// REQ-RT-12: a publisher overwrites the oldest slot whatever its readers are
// doing, and a lapped sequence reader counts every message it lost in
// `dropped`; what it accepts is one whole publication, never a torn one.
//
// Two slots and a writer that never waits, so the writer reaches the slot a
// reader is copying. A reader copies a slot and then checks the writer had not
// reached it (a seqlock: RTMSQueue::read_next, slot_still_valid). The copy and
// the overwrite are a data race by the C++ memory model, by design, and the
// thread sanitizer reports it on any run, so this target is `no_tsan` and the
// guard's proof is this test's check on every accepted message: each of its
// words, and its publication time, must belong to the sequence it was
// accepted as. Without the check after the copy, accepted messages tear.
TEST(RingLap, LappedReaderAcceptsOnlyWholeMessagesAndCountsEverySkip) {
  constexpr std::size_t kSlots = 2;
  constexpr std::uint64_t kMessages = 200'000;
  constexpr std::size_t kReaders = 3;

  RTMSQueue queue{
      "/ring_lap_" + std::to_string(::getpid()),
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
  std::array<ReaderResult, kReaders> results{};

  std::array<std::thread, kReaders> readers;
  for (std::size_t r = 0; r < kReaders; ++r) {
    readers[r] = std::thread{[&, r] {
      ReaderResult& result = results[r];
      start.arrive_and_wait();
      Payload copy{};
      for (;;) {
        // An EMPTY read begun after the writer finished means caught up.
        const bool final_read = writer_done.load(std::memory_order_acquire);
        MessageInfo info{};
        const ReadResult read = queue.read_next(
            reader_ids[r], std::as_writable_bytes(std::span{copy}), info);
        if (read == ReadResult::EMPTY) {
          if (final_read) break;
          std::this_thread::yield();
          continue;
        }
        // Lapped mid-copy on every retry: nothing accepted, try again.
        if (read == ReadResult::TORN) continue;
        if (read != ReadResult::OK ||
            FirstForeignWord(copy, info.sequence) != kWords ||
            info.publish_time_ns != RingPublishTime(info.sequence) ||
            (result.last.has_value() && info.sequence <= *result.last)) {
          result.failure = std::format(
              "accepted #{}: result {}, sequence {} after {}, first foreign "
              "word {} (holds {}), publish time {}",
              result.accepted, static_cast<int>(read), info.sequence,
              result.last.value_or(0), FirstForeignWord(copy, info.sequence),
              copy[FirstForeignWord(copy, info.sequence) % kWords],
              info.publish_time_ns);
          return;
        }
        ++result.accepted;
        result.dropped += info.dropped;
        result.last = info.sequence;
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
  EXPECT_EQ(written, kMessages) << "the overwriting writer was refused";
  std::uint64_t total_dropped = 0;
  for (std::size_t r = 0; r < kReaders; ++r) {
    SCOPED_TRACE(std::format("reader {}", r));
    const ReaderResult& result = results[r];
    EXPECT_EQ(result.failure, "");
    EXPECT_EQ(result.last, std::optional<std::uint64_t>{kMessages - 1});
    // The cursor starts at 0 and each accepted message moves it past every
    // message it skipped: nothing is lost uncounted.
    EXPECT_EQ(result.accepted + result.dropped, kMessages)
        << result.accepted << " accepted, " << result.dropped << " dropped";
    total_dropped += result.dropped;
  }
  // A run in which the writer never lapped a reader exercised no guard.
  EXPECT_GT(total_dropped, 0U);
}

}  // namespace
