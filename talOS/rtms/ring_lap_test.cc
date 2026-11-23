// The ring's torn-read guard and slot reuse on one mapping, so ThreadSanitizer
// observes writer and reader accesses to the same slot addresses.

#include <gtest/gtest.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
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

constexpr std::size_t kTailMessageBytes = 13;
const talos::schema::Identity kTailSchema{"test.RingTail", 0x5441494c};

constexpr std::size_t kDropNewestWords = 16;
using DropNewestPayload = std::array<std::uint64_t, kDropNewestWords>;
const talos::schema::Identity kDropNewestSchema{"test.RingDropNewest",
                                                0x44524f50};

std::size_t ExpectedWriteSize(std::uint64_t sequence) {
  switch (sequence % 4) {
    case 0:
      return kTailMessageBytes;
    case 1:
      return 5;
    case 2:
      return 9;
    default:
      return 0;
  }
}

std::array<std::byte, kTailMessageBytes> ExpectedTailPayload(
    std::uint64_t sequence) {
  std::array<std::byte, kTailMessageBytes> out{};
  const std::size_t len = ExpectedWriteSize(sequence);
  if (len == 0) return out;
  const std::uint64_t lo = sequence ^ 0xA5A5A5A55A5A5A5AULL;
  const std::uint64_t hi = (sequence * 0x9E3779B97F4A7C15ULL) | 1ULL;
  std::array<std::byte, 16> raw{};
  std::memcpy(raw.data(), &lo, sizeof(lo));
  std::memcpy(raw.data() + sizeof(lo), &hi, sizeof(hi));
  std::memcpy(out.data(), raw.data(), len);
  return out;
}

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
// reader is copying. Slot payload and timestamp copies use uniform 8-byte
// relaxed atomic words paired with the writer's release fence and
// slot_still_valid()'s acquire fence, so this target runs under ThreadSanitizer
// with zero data races while verifying every accepted message's words and time.
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

// REQ-RT-12: non-multiple-of-8 message sizes and short writes (including zero
// bytes) zero-pad trailing bytes and remain race-free and tear-free under
// concurrent lapping, including with unaligned caller buffers.
TEST(RingLap, LappedReaderHandlesUnalignedTailAndShortWrites) {
  constexpr std::size_t kSlots = 2;
  constexpr std::uint64_t kMessages = 100'000;
  constexpr std::size_t kReaders = 3;

  RTMSQueue queue{
      "/ring_lap_tail_" + std::to_string(::getpid()),
      kTailMessageBytes,
      1,
      kTailSchema,
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
      // Intentionally unaligned destination slice to exercise memcpy unpacking.
      alignas(8) std::array<std::byte, kTailMessageBytes + 3> dst_storage{};
      const std::span<std::byte> dst{dst_storage.data() + 3, kTailMessageBytes};
      for (;;) {
        const bool final_read = writer_done.load(std::memory_order_acquire);
        std::ranges::fill(dst, std::byte{0xCC});
        MessageInfo info{};
        const ReadResult read = queue.read_next(reader_ids[r], dst, info);
        if (read == ReadResult::EMPTY) {
          if (final_read) break;
          std::this_thread::yield();
          continue;
        }
        if (read == ReadResult::TORN) continue;
        const auto expected = ExpectedTailPayload(info.sequence);
        if (read != ReadResult::OK ||
            std::memcmp(dst.data(), expected.data(), kTailMessageBytes) != 0 ||
            info.publish_time_ns != RingPublishTime(info.sequence) ||
            (result.last.has_value() && info.sequence <= *result.last)) {
          result.failure = std::format(
              "accepted #{}: result {}, sequence {} after {}, publish time {}",
              result.accepted, static_cast<int>(read), info.sequence,
              result.last.value_or(0), info.publish_time_ns);
          return;
        }
        ++result.accepted;
        result.dropped += info.dropped;
        result.last = info.sequence;
      }
    }};
  }

  start.arrive_and_wait();
  // Intentionally unaligned source slice to exercise memcpy packing.
  alignas(8) std::array<std::byte, kTailMessageBytes + 1> src_storage{};
  std::uint64_t written = 0;
  for (; written < kMessages; ++written) {
    const std::size_t len = ExpectedWriteSize(written);
    const auto expected = ExpectedTailPayload(written);
    std::span<const std::byte> view{};
    if (len > 0) {
      std::memcpy(src_storage.data() + 1, expected.data(), len);
      view = std::span<const std::byte>{src_storage.data() + 1, len};
    }
    if (queue.write(view, RingPublishTime(written)).result !=
        WriteResult::SUCCESS) {
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
    EXPECT_EQ(result.accepted + result.dropped, kMessages)
        << result.accepted << " accepted, " << result.dropped << " dropped";
    total_dropped += result.dropped;
  }
  EXPECT_GT(total_dropped, 0U);
}

// REQ-CONTRACT-36: DROP_NEWEST slot reuse on one mapping delivers every
// publication intact with zero drops and zero data races under ThreadSanitizer.
TEST(RingLap, DropNewestDeliversEveryMessageOnOneMapping) {
  constexpr std::size_t kSlots = 4;
  constexpr std::uint64_t kMessages = 4'000;
  constexpr std::size_t kReaders = 3;

  RTMSQueue queue{
      "/ring_drop_newest_" + std::to_string(::getpid()),
      sizeof(DropNewestPayload),
      alignof(DropNewestPayload),
      kDropNewestSchema,
      kSlots,
      RTMSOptions{.overflow_policy = OverflowPolicy::DROP_NEWEST,
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
  std::atomic<bool> failed{false};
  std::array<std::uint64_t, kReaders> received{};
  std::array<std::string, kReaders> failure{};

  std::array<std::thread, kReaders> readers;
  for (std::size_t r = 0; r < kReaders; ++r) {
    readers[r] = std::thread{[&, r] {
      start.arrive_and_wait();
      DropNewestPayload copy{};
      while (received[r] < kMessages &&
             !failed.load(std::memory_order_acquire)) {
        MessageInfo info{};
        const ReadResult read = queue.read_next(
            reader_ids[r], std::as_writable_bytes(std::span{copy}), info);
        if (read == ReadResult::EMPTY) {
          std::this_thread::yield();
          continue;
        }
        const std::uint64_t expected = received[r];
        if (read != ReadResult::OK || info.sequence != expected ||
            info.dropped != 0 ||
            info.publish_time_ns != RingPublishTime(expected) ||
            FirstForeignWord(copy, expected) != kDropNewestWords) {
          failure[r] = std::format(
              "message {}: result {}, sequence {}, dropped {}, publish time "
              "{}, first foreign word {}",
              expected, static_cast<int>(read), info.sequence, info.dropped,
              info.publish_time_ns, FirstForeignWord(copy, expected));
          failed.store(true, std::memory_order_release);
          queue.release_reader(reader_ids[r]);
          return;
        }
        ++received[r];
      }
      if (failed.load(std::memory_order_acquire)) {
        queue.release_reader(reader_ids[r]);
      }
    }};
  }

  start.arrive_and_wait();
  DropNewestPayload message{};
  std::uint64_t written = 0;
  while (written < kMessages && !failed.load(std::memory_order_acquire)) {
    FillRingPayload(message, written);
    const WriteStatus status = queue.write(std::as_bytes(std::span{message}),
                                           RingPublishTime(written));
    if (status.result == WriteResult::BUFFER_FULL) {
      std::this_thread::yield();
      continue;
    }
    if (status.result != WriteResult::SUCCESS) {
      failed.store(true, std::memory_order_release);
      break;
    }
    ++written;
  }

  for (auto& reader : readers) reader.join();
  EXPECT_EQ(written, kMessages);
  for (std::size_t r = 0; r < kReaders; ++r) {
    SCOPED_TRACE(std::format("reader {}", r));
    EXPECT_EQ(failure[r], "");
    EXPECT_EQ(received[r], kMessages);
  }
}

}  // namespace
