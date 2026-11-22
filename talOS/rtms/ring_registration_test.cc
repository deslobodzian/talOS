// Deterministic reader-registration boundary tests: a reader registered while
// a DROP_NEWEST writer publishes, its two steps run one at a time.

#include <gtest/gtest.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <format>
#include <optional>
#include <span>
#include <string>

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

using talos::rtms_test::FillRingPayload;
using talos::rtms_test::FirstForeignWord;
using talos::rtms_test::RegistrationPeer;
using talos::rtms_test::RingPublishTime;

constexpr std::size_t kDropNewestWords = 16;
using DropNewestPayload = std::array<std::uint64_t, kDropNewestWords>;
const talos::schema::Identity kDropNewestSchema{"test.RingDropNewest",
                                                0x44524f50};

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
TEST(RingRegistration, ReaderRegisteredAcrossAFullRingStartsAtTheWriter) {
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
TEST(RingRegistration, ReaderRegisteredAcrossLessThanARingReadsEveryMessage) {
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
