// The ring's torn-read guard and slot reuse on one mapping, so ThreadSanitizer
// observes writer and reader accesses to the same slot addresses.
// Also a reader registered while the writer publishes, its two steps run one
// at a time.

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

namespace talos::rtms_test {

// Runs register_reader()'s two steps apart, and stands in for a writer
// part-way through write(), with the same stores write() makes.
class RegistrationPeer {
 public:
  static std::optional<std::uint64_t> Claim(RTMSQueue& queue,
                                            std::size_t reader_id) {
    return queue.claim_reader_slot(reader_id);
  }

  static bool Activate(RTMSQueue& queue, std::size_t reader_id,
                       std::uint64_t current) {
    return queue.activate_reader_slot(reader_id, current);
  }

  // write()'s slot stores for publication `sequence`: payload words, then
  // the publication time, all 8-byte relaxed atomic words; nothing published.
  static void StoreSlot(RTMSQueue& queue, std::uint64_t sequence,
                        std::span<const std::uint64_t> words,
                        std::int64_t publish_time_ns) {
    std::byte* slot = queue.slot_address(sequence);
    for (std::size_t off = 0; off < queue.publish_time_offset_;
         off += sizeof(std::uint64_t)) {
      const std::size_t index = off / sizeof(std::uint64_t);
      std::atomic_ref<std::uint64_t>(
          *reinterpret_cast<std::uint64_t*>(slot + off))
          .store(index < words.size() ? words[index] : 0,
                 std::memory_order_relaxed);
    }
    std::atomic_ref<std::int64_t>(
        *reinterpret_cast<std::int64_t*>(slot + queue.publish_time_offset_))
        .store(publish_time_ns, std::memory_order_relaxed);
  }

  // write()'s DROP_NEWEST publication of `sequence` after StoreSlot(). No
  // reader armed a wake, so there is nobody to notify.
  static void Publish(RTMSQueue& queue, std::uint64_t sequence) {
    queue.header_->writer.reliable_sequence.store(sequence + 1,
                                                  std::memory_order_release);
    queue.header_->writer.sequence.store(sequence + 1,
                                         std::memory_order_seq_cst);
  }
};

}  // namespace talos::rtms_test

namespace {

using talos::rtms_test::RegistrationPeer;

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

RTMSQueue MakeDropNewestQueue(const std::string& name, std::size_t slots) {
  return RTMSQueue{
      name + "_" + std::to_string(::getpid()),
      sizeof(DropNewestPayload),
      alignof(DropNewestPayload),
      kDropNewestSchema,
      slots,
      RTMSOptions{.overflow_policy = OverflowPolicy::DROP_NEWEST,
                  .read_mode = ReadMode::SEQUENCE,
                  .publisher = true},
  };
}

void WriteSequences(RTMSQueue& queue, std::uint64_t first,
                    std::uint64_t count) {
  DropNewestPayload message{};
  for (std::uint64_t sequence = first; sequence < first + count; ++sequence) {
    FillRingPayload(message, sequence);
    ASSERT_EQ(
        queue
            .write(std::as_bytes(std::span{message}), RingPublishTime(sequence))
            .result,
        WriteResult::SUCCESS)
        << "sequence " << sequence;
  }
}

std::string Describe(ReadResult read, const MessageInfo& info,
                     const DropNewestPayload& copy, std::uint64_t sequence) {
  return std::format(
      "result {}, sequence {}, dropped {}, publish time {}, words that are "
      "sequence {}'s before the first that is not: {} of {}",
      static_cast<int>(read), info.sequence, info.dropped, info.publish_time_ns,
      sequence, FirstForeignWord(copy, sequence), kDropNewestWords);
}

// REQ-CONTRACT-36: a DROP_NEWEST writer that saw no active reader may publish
// a full ring while a reader registers, and be admitted for the next write
// into the slot the reader's starting cursor names. The reader then starts at
// the writer, never reading that slot's next publication under the old
// sequence.
TEST(RingLap, ReaderRegisteredAcrossAFullRingStartsAtTheWriter) {
  constexpr std::size_t kSlots = 4;
  constexpr std::size_t kReader = 0;
  RTMSQueue queue = MakeDropNewestQueue("/ring_join_full", kSlots);
  constexpr std::uint64_t kStart = 2;  // the cursor the claim takes
  ASSERT_NO_FATAL_FAILURE(WriteSequences(queue, 0, kStart));

  const std::optional<std::uint64_t> current =
      RegistrationPeer::Claim(queue, kReader);
  ASSERT_EQ(current, std::optional<std::uint64_t>{kStart});
  ASSERT_NO_FATAL_FAILURE(WriteSequences(queue, kStart, kSlots));
  constexpr std::uint64_t kNext = kStart + kSlots;  // in kStart's slot
  ASSERT_EQ(queue.writer_sequence(), kNext);
  // The writer's own admission check, before the reader is active: the
  // write of kNext is admitted.
  ASSERT_LE((queue.writer_sequence() + 1) - queue.minimum_read_position(),
            kSlots);
  ASSERT_TRUE(RegistrationPeer::Activate(queue, kReader, current.value()));

  // That admitted writer, part-way through: kNext's words are in the slot,
  // not yet published.
  DropNewestPayload message{};
  FillRingPayload(message, kNext);
  RegistrationPeer::StoreSlot(queue, kNext, message, RingPublishTime(kNext));

  DropNewestPayload copy{};
  MessageInfo info{};
  ReadResult read =
      queue.read_next(kReader, std::as_writable_bytes(std::span{copy}), info);
  ASSERT_EQ(read, ReadResult::EMPTY) << Describe(read, info, copy, kNext);

  RegistrationPeer::Publish(queue, kNext);
  read =
      queue.read_next(kReader, std::as_writable_bytes(std::span{copy}), info);
  EXPECT_EQ(read, ReadResult::OK);
  EXPECT_EQ(info.sequence, kNext);
  EXPECT_EQ(info.dropped, 0U);
  EXPECT_EQ(info.publish_time_ns, RingPublishTime(kNext));
  EXPECT_EQ(FirstForeignWord(copy, kNext), kDropNewestWords)
      << Describe(read, info, copy, kNext);
}

// REQ-CONTRACT-36: a reader registered while fewer than a ring of messages
// were published starts where its claim found the writer and receives each
// of them, none dropped.
TEST(RingLap, ReaderRegisteredAcrossLessThanARingReadsEveryMessage) {
  constexpr std::size_t kSlots = 4;
  constexpr std::size_t kReader = 0;
  RTMSQueue queue = MakeDropNewestQueue("/ring_join_partial", kSlots);
  constexpr std::uint64_t kStart = 2;
  ASSERT_NO_FATAL_FAILURE(WriteSequences(queue, 0, kStart));

  const std::optional<std::uint64_t> current =
      RegistrationPeer::Claim(queue, kReader);
  ASSERT_EQ(current, std::optional<std::uint64_t>{kStart});
  ASSERT_NO_FATAL_FAILURE(WriteSequences(queue, kStart, kSlots - 1));
  ASSERT_TRUE(RegistrationPeer::Activate(queue, kReader, current.value()));

  DropNewestPayload copy{};
  for (std::uint64_t sequence = kStart; sequence < kStart + kSlots - 1;
       ++sequence) {
    MessageInfo info{};
    const ReadResult read =
        queue.read_next(kReader, std::as_writable_bytes(std::span{copy}), info);
    ASSERT_TRUE(read == ReadResult::OK && info.sequence == sequence &&
                info.dropped == 0 &&
                info.publish_time_ns == RingPublishTime(sequence) &&
                FirstForeignWord(copy, sequence) == kDropNewestWords)
        << "expected sequence " << sequence << ": "
        << Describe(read, info, copy, sequence);
  }
  MessageInfo info{};
  EXPECT_EQ(
      queue.read_next(kReader, std::as_writable_bytes(std::span{copy}), info),
      ReadResult::EMPTY);
}

}  // namespace
