// Deterministic reader-registration boundary tests: a reader registered while
// a DROP_NEWEST writer publishes, its two steps run one at a time.

#include <fcntl.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <format>
#include <latch>
#include <limits>
#include <mutex>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include "talOS/memory/shared_memory_ptr.h"
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

  static const RTMSHeader* Header(const RTMSQueue& queue) {
    return queue.header_;
  }

  static void SetOnReclaimWait(void (*callback)(std::uint64_t, int)) {
    RTMSQueue::on_reclaim_wait_ = callback;
  }

  static void SetOnUnlinkVerified(void (*callback)(std::uint64_t)) {
    RTMSQueue::on_unlink_verified_ = callback;
  }

  static void SetOnSessionClaimed(void (*callback)(std::uint64_t)) {
    RTMSQueue::on_session_claimed_ = callback;
  }

  static int ClaimantWaitAttempts() {
    return RTMSQueue::claimant_wait_attempts_;
  }

  static void SetClaimantWaitAttempts(int attempts) {
    RTMSQueue::claimant_wait_attempts_ = attempts;
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

std::string MakeUniquePath(const char* prefix) {
  static std::atomic<int> counter{0};
  return std::string{prefix} + "_" + std::to_string(::getpid()) + "_" +
         std::to_string(counter.fetch_add(1, std::memory_order_relaxed));
}

RTMSQueue OpenSubscriber(const std::string& path, std::size_t slots) {
  return RTMSQueue{
      path,
      sizeof(DropNewestPayload),
      alignof(DropNewestPayload),
      kDropNewestSchema,
      slots,
      RTMSOptions{.overflow_policy = OverflowPolicy::DROP_NEWEST,
                  .read_mode = ReadMode::SEQUENCE,
                  .publisher = false},
  };
}

// Creates a standalone segment left in `lifetime_word` with no live writer or
// reader claims, and returns a mapping of its header plus its segment_id.
std::pair<SharedMemoryPtr, std::uint64_t> MakeAbandonedClosingSegment(
    const std::string& path, std::size_t slots, std::uint64_t lifetime_word) {
  ::shm_unlink(ShmObjectName(path).c_str());
  std::optional<RTMSQueue> creator;
  creator.emplace(OpenSubscriber(path, slots));
  SharedMemoryPtr mapping{path, sizeof(RTMSHeader)};
  auto* header = static_cast<RTMSHeader*>(mapping.ptr());
  const std::uint64_t segment_id = header->segment_id;
  // Hold an extra synthetic count so destroying `creator` leaves the segment
  // on disk without unlinking it.
  header->lifetime.store(2, std::memory_order_release);
  creator.reset();
  header->lifetime.store(lifetime_word, std::memory_order_release);
  return {std::move(mapping), segment_id};
}

// REQ-RT-01, REQ-RT-02, REQ-OPS-03: destroying an RTMSQueue without calling
// release_reader() frees its reader slots, wake receivers, and wake arms so
// repeated open/register/destroy cycles never exhaust MAX_READERS.
TEST(RingRegistration, DestroyedQueueReleasesReaderSlotsAndWakeArms) {
  static_assert(std::is_nothrow_destructible_v<RTMSQueue>);
  static_assert(std::is_nothrow_move_assignable_v<RTMSQueue>);
  static_assert(noexcept(std::declval<RTMSQueue&>().release_reader(0)));

  constexpr std::size_t kSlots = 4;
  const std::string path = MakeUniquePath("/ring_close_slots");
  ::shm_unlink(ShmObjectName(path).c_str());
  RTMSQueue publisher{
      path,
      sizeof(DropNewestPayload),
      alignof(DropNewestPayload),
      kDropNewestSchema,
      kSlots,
      RTMSOptions{.overflow_policy = OverflowPolicy::DROP_NEWEST,
                  .read_mode = ReadMode::SEQUENCE,
                  .publisher = true},
  };
  const RTMSHeader* header = RegistrationPeer::Header(publisher);

  for (std::size_t i = 0; i < MAX_READERS + 4; ++i) {
    std::size_t slot_id = MAX_READERS;
    {
      RTMSQueue subscriber = OpenSubscriber(path, kSlots);
      const std::optional<std::size_t> reader = subscriber.register_reader();
      ASSERT_TRUE(reader.has_value()) << "exhausted MAX_READERS at cycle " << i;
      slot_id = *reader;
      const std::uint64_t wake_id = 0x1000U + i;
      subscriber.set_reader_wake(slot_id, wake_id);
      ASSERT_TRUE(subscriber.arm_wake(slot_id));
      ASSERT_NE(header->readers[slot_id].state.load(std::memory_order_acquire),
                0U);
      ASSERT_EQ(
          header->readers[slot_id].wake_id.load(std::memory_order_acquire),
          wake_id);
      ASSERT_NE(header->writer.waiters.load(std::memory_order_acquire) &
                    (std::uint64_t{1} << slot_id),
                0U);
    }
    EXPECT_EQ(header->readers[slot_id].state.load(std::memory_order_acquire),
              0U)
        << "leaked reader slot at cycle " << i;
    EXPECT_EQ(header->readers[slot_id].wake_id.load(std::memory_order_acquire),
              0U)
        << "leaked wake_id at cycle " << i;
    EXPECT_EQ(header->writer.waiters.load(std::memory_order_acquire) &
                  (std::uint64_t{1} << slot_id),
              0U)
        << "leaked wake arm at cycle " << i;
  }
}

// REQ-RT-01, REQ-RT-02: destroying an unread subscriber without calling
// release_reader() unblocks a DROP_NEWEST publisher rather than permanently
// stalling it on BUFFER_FULL.
TEST(RingRegistration, DestroyedReaderDoesNotStallDropNewestWriter) {
  constexpr std::size_t kSlots = 4;
  const std::string path = MakeUniquePath("/ring_close_stall");
  ::shm_unlink(ShmObjectName(path).c_str());
  RTMSQueue publisher{
      path,
      sizeof(DropNewestPayload),
      alignof(DropNewestPayload),
      kDropNewestSchema,
      kSlots,
      RTMSOptions{.overflow_policy = OverflowPolicy::DROP_NEWEST,
                  .read_mode = ReadMode::SEQUENCE,
                  .publisher = true},
  };

  {
    RTMSQueue subscriber = OpenSubscriber(path, kSlots);
    ASSERT_TRUE(subscriber.register_reader().has_value());
    ASSERT_NO_FATAL_FAILURE(WriteSequences(publisher, 0, kSlots));
    DropNewestPayload message{};
    FillRingPayload(message, kSlots);
    ASSERT_EQ(
        publisher
            .write(std::as_bytes(std::span{message}), RingPublishTime(kSlots))
            .result,
        WriteResult::BUFFER_FULL);
  }

  DropNewestPayload message{};
  FillRingPayload(message, kSlots);
  EXPECT_EQ(
      publisher
          .write(std::as_bytes(std::span{message}), RingPublishTime(kSlots))
          .result,
      WriteResult::SUCCESS);
}

// REQ-RT-01, REQ-RT-02: move-assigning over an RTMSQueue releases the
// overwritten queue's reader slots and wake arms while keeping the moved-from
// queue's slot active.
TEST(RingRegistration, MoveAssignmentReleasesOverwrittenQueueReaderSlots) {
  constexpr std::size_t kSlots = 4;
  const std::string path = MakeUniquePath("/ring_move_assign");
  ::shm_unlink(ShmObjectName(path).c_str());
  RTMSQueue publisher{
      path,
      sizeof(DropNewestPayload),
      alignof(DropNewestPayload),
      kDropNewestSchema,
      kSlots,
      RTMSOptions{.overflow_policy = OverflowPolicy::DROP_NEWEST,
                  .read_mode = ReadMode::SEQUENCE,
                  .publisher = true},
  };
  const RTMSHeader* header = RegistrationPeer::Header(publisher);

  std::size_t id_a = MAX_READERS;
  std::size_t id_b = MAX_READERS;
  {
    RTMSQueue sub_a = OpenSubscriber(path, kSlots);
    RTMSQueue sub_b = OpenSubscriber(path, kSlots);
    const auto reg_a = sub_a.register_reader();
    const auto reg_b = sub_b.register_reader();
    ASSERT_TRUE(reg_a.has_value());
    ASSERT_TRUE(reg_b.has_value());
    id_a = *reg_a;
    id_b = *reg_b;
    ASSERT_NE(id_a, id_b);
    sub_a.set_reader_wake(id_a, 0x2222U);
    ASSERT_TRUE(sub_a.arm_wake(id_a));

    sub_a = std::move(sub_b);

    EXPECT_EQ(header->readers[id_a].state.load(std::memory_order_acquire), 0U);
    EXPECT_EQ(header->readers[id_a].wake_id.load(std::memory_order_acquire),
              0U);
    EXPECT_EQ(header->writer.waiters.load(std::memory_order_acquire) &
                  (std::uint64_t{1} << id_a),
              0U);
    EXPECT_NE(header->readers[id_b].state.load(std::memory_order_acquire), 0U);

    ASSERT_NO_FATAL_FAILURE(WriteSequences(publisher, 0, 1));
    DropNewestPayload copy{};
    MessageInfo info{};
    EXPECT_EQ(
        sub_a.read_next(id_b, std::as_writable_bytes(std::span{copy}), info),
        ReadResult::OK);
    EXPECT_EQ(FirstForeignWord(copy, 0), kDropNewestWords);
  }
  EXPECT_EQ(header->readers[id_b].state.load(std::memory_order_acquire), 0U);
}

// REQ-RT-01, REQ-OPS-03: a forked child destroying an inherited RTMSQueue must
// not release its parent's reader slot or wake arm.
TEST(RingRegistration, ForkedChildDestructionPreservesParentReaderSlot) {
  constexpr std::size_t kSlots = 4;
  const std::string path = MakeUniquePath("/ring_fork_close");
  ::shm_unlink(ShmObjectName(path).c_str());
  RTMSQueue owner{
      path,
      sizeof(DropNewestPayload),
      alignof(DropNewestPayload),
      kDropNewestSchema,
      kSlots,
      RTMSOptions{.overflow_policy = OverflowPolicy::DROP_NEWEST,
                  .read_mode = ReadMode::SEQUENCE,
                  .publisher = true,
                  .session_owner = true},
  };
  const RTMSHeader* header = RegistrationPeer::Header(owner);

  std::optional<RTMSQueue> subscriber;
  subscriber.emplace(OpenSubscriber(path, kSlots));
  const auto reg = subscriber->register_reader();
  ASSERT_TRUE(reg.has_value());
  const std::size_t slot_id = *reg;
  constexpr std::uint64_t kWakeId = 0xBEEFU;
  subscriber->set_reader_wake(slot_id, kWakeId);
  ASSERT_TRUE(subscriber->arm_wake(slot_id));

  const pid_t child = ::fork();
  ASSERT_NE(child, -1);
  if (child == 0) {
    subscriber.reset();
    ::_exit(0);
  }
  int status = 0;
  ASSERT_EQ(::waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0);

  EXPECT_NE(header->readers[slot_id].state.load(std::memory_order_acquire), 0U);
  EXPECT_EQ(header->readers[slot_id].wake_id.load(std::memory_order_acquire),
            kWakeId);
  EXPECT_NE(header->writer.waiters.load(std::memory_order_acquire) &
                (std::uint64_t{1} << slot_id),
            0U);

  ASSERT_NO_FATAL_FAILURE(WriteSequences(owner, 0, 1));
  DropNewestPayload copy{};
  MessageInfo info{};
  EXPECT_EQ(subscriber->read_next(
                slot_id, std::as_writable_bytes(std::span{copy}), info),
            ReadResult::OK);
  EXPECT_EQ(FirstForeignWord(copy, 0), kDropNewestWords);

  subscriber.reset();
  EXPECT_EQ(header->readers[slot_id].state.load(std::memory_order_acquire), 0U);
  EXPECT_EQ(header->readers[slot_id].wake_id.load(std::memory_order_acquire),
            0U);
  EXPECT_EQ(header->writer.waiters.load(std::memory_order_acquire) &
                (std::uint64_t{1} << slot_id),
            0U);
}

// REQ-OPS-03, REQ-CONTRACT-21: a forked child destroying inherited standalone
// or session_owner RTMSQueues must not decrement the parent's lifetime count,
// mark the segment closing, or unlink the parent's live shared-memory segment.
TEST(RingRegistration, ForkedChildDestructionPreservesParentSegmentLifetime) {
  constexpr std::size_t kSlots = 4;
  const std::string standalone_path = MakeUniquePath("/ring_fork_standalone");
  const std::string session_path = MakeUniquePath("/ring_fork_session");
  ::shm_unlink(ShmObjectName(standalone_path).c_str());
  ::shm_unlink(ShmObjectName(session_path).c_str());

  std::optional<RTMSQueue> standalone;
  standalone.emplace(RTMSQueue{
      standalone_path,
      sizeof(DropNewestPayload),
      alignof(DropNewestPayload),
      kDropNewestSchema,
      kSlots,
      RTMSOptions{.overflow_policy = OverflowPolicy::DROP_NEWEST,
                  .read_mode = ReadMode::SEQUENCE,
                  .publisher = true},
  });
  std::optional<RTMSQueue> session;
  session.emplace(RTMSQueue{
      session_path,
      sizeof(DropNewestPayload),
      alignof(DropNewestPayload),
      kDropNewestSchema,
      kSlots,
      RTMSOptions{.overflow_policy = OverflowPolicy::DROP_NEWEST,
                  .read_mode = ReadMode::SEQUENCE,
                  .publisher = true,
                  .session_owner = true},
  });

  const RTMSHeader* standalone_hdr = RegistrationPeer::Header(*standalone);
  const std::uint64_t standalone_id = standalone_hdr->segment_id;
  const std::uint64_t session_id =
      RegistrationPeer::Header(*session)->segment_id;
  ASSERT_EQ(standalone_hdr->lifetime.load(std::memory_order_acquire), 1U);

  const pid_t child = ::fork();
  ASSERT_NE(child, -1);
  if (child == 0) {
    standalone.reset();
    session.reset();
    ::_exit(0);
  }
  int status = 0;
  ASSERT_EQ(::waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0);

  EXPECT_EQ(standalone_hdr->lifetime.load(std::memory_order_acquire), 1U)
      << "forked child corrupted parent's standalone lifetime count";
  {
    RTMSQueue peer = OpenSubscriber(standalone_path, kSlots);
    EXPECT_EQ(RegistrationPeer::Header(peer)->segment_id, standalone_id)
        << "forked child unlinked parent's standalone segment";
  }
  {
    RTMSQueue peer = OpenSubscriber(session_path, kSlots);
    EXPECT_EQ(RegistrationPeer::Header(peer)->segment_id, session_id)
        << "forked child unlinked parent's session_owner segment";
  }
}

// REQ-CONTRACT-36: when all MAX_READERS slots hold dead processes and multiple
// live subscribers call register_reader() concurrently, a subscriber that loses
// the reclaim_dead_readers() race (freeing 0 slots itself because a peer just
// freed them) must still rescan and claim a free slot instead of returning
// nullopt.
TEST(RingRegistration,
     ConcurrentRegisterReclaimingFullDeadTableSucceedsForAll) {
  constexpr std::size_t kSlots = 4;
  constexpr std::size_t kThreads = 16;
  const std::string path = MakeUniquePath("/ring_dead_race");
  ::shm_unlink(ShmObjectName(path).c_str());

  // Obtain a guaranteed-dead pid.
  const pid_t dead_child = ::fork();
  ASSERT_NE(dead_child, -1);
  if (dead_child == 0) ::_exit(0);
  int status = 0;
  ASSERT_EQ(::waitpid(dead_child, &status, 0), dead_child);
  const std::uint64_t dead_state =
      (static_cast<std::uint64_t>(dead_child) << 2) |
      static_cast<std::uint64_t>(ReaderState::ACTIVE);

  RTMSQueue owner{
      path,
      sizeof(DropNewestPayload),
      alignof(DropNewestPayload),
      kDropNewestSchema,
      kSlots,
      RTMSOptions{.overflow_policy = OverflowPolicy::DROP_NEWEST,
                  .read_mode = ReadMode::SEQUENCE,
                  .publisher = true,
                  .session_owner = true},
  };
  SharedMemoryPtr mapping{path, sizeof(RTMSHeader)};
  auto* header = static_cast<RTMSHeader*>(mapping.ptr());
  ASSERT_NE(header, nullptr);

  std::vector<RTMSQueue> subscribers;
  subscribers.reserve(kThreads);
  for (std::size_t i = 0; i < kThreads; ++i) {
    subscribers.push_back(OpenSubscriber(path, kSlots));
  }

  for (int round = 0; round < 10; ++round) {
    for (std::size_t i = 0; i < MAX_READERS; ++i) {
      header->readers[i].state.store(dead_state, std::memory_order_release);
    }

    std::latch ready{static_cast<std::ptrdiff_t>(kThreads)};
    std::array<std::optional<std::size_t>, kThreads> slots_claimed{};
    std::vector<std::thread> threads;
    threads.reserve(kThreads);
    for (std::size_t t = 0; t < kThreads; ++t) {
      threads.emplace_back([&, t] {
        ready.arrive_and_wait();
        slots_claimed[t] = subscribers[t].register_reader();
      });
    }
    for (auto& thread : threads) thread.join();

    std::array<bool, MAX_READERS> seen{};
    for (std::size_t t = 0; t < kThreads; ++t) {
      ASSERT_TRUE(slots_claimed[t].has_value())
          << "round " << round << " thread " << t
          << " spuriously failed to register while slots were free";
      const std::size_t id = *slots_claimed[t];
      ASSERT_LT(id, MAX_READERS);
      EXPECT_FALSE(seen[id]) << "duplicate slot " << id;
      seen[id] = true;
      subscribers[t].release_reader(id);
    }
  }
}

// REQ-OPS-03: concurrent openers racing to reclaim a kLifetimeClosing segment
// must elect a single claimant and converge on one fresh segment without
// split-brain unlinking.
TEST(RingRegistration, ConcurrentOpenersReclaimingClosingSegmentShareOne) {
  constexpr std::size_t kSlots = 4;
  constexpr std::size_t kOpeners = 8;
  for (int round = 0; round < 3; ++round) {
    const std::string path = MakeUniquePath("/ring_reclaim_race");
    auto [old_mapping, old_segment_id] =
        MakeAbandonedClosingSegment(path, kSlots, kLifetimeClosing);

    std::latch ready{static_cast<std::ptrdiff_t>(kOpeners)};
    std::vector<std::optional<RTMSQueue>> queues(kOpeners);
    std::vector<std::string> errors(kOpeners);
    std::vector<std::thread> threads;
    threads.reserve(kOpeners);
    for (std::size_t i = 0; i < kOpeners; ++i) {
      threads.emplace_back([&, i] {
        ready.arrive_and_wait();
        try {
          queues[i].emplace(OpenSubscriber(path, kSlots));
        } catch (const std::exception& e) {
          errors[i] = e.what();
        }
      });
    }
    for (auto& thread : threads) thread.join();

    for (std::size_t i = 0; i < kOpeners; ++i) {
      ASSERT_TRUE(errors[i].empty())
          << "round " << round << " opener " << i << ": " << errors[i];
      ASSERT_TRUE(queues[i].has_value());
    }

    const auto* old_header = static_cast<const RTMSHeader*>(old_mapping.ptr());
    EXPECT_EQ(old_header->lifetime.load(std::memory_order_acquire),
              kLifetimeClosing | talos::OwnProcessWord())
        << "round " << round << ": the old segment's claimant";

    const RTMSHeader* shared_header = RegistrationPeer::Header(*queues[0]);
    ASSERT_NE(shared_header->segment_id, old_segment_id);
    for (std::size_t i = 1; i < kOpeners; ++i) {
      EXPECT_EQ(RegistrationPeer::Header(*queues[i])->segment_id,
                shared_header->segment_id)
          << "round " << round << " split-brain on opener " << i;
    }
    EXPECT_EQ(shared_header->lifetime.load(std::memory_order_acquire) &
                  kLifetimeCountMask,
              kOpeners)
        << "round " << round;

    std::vector<std::size_t> reader_ids(kOpeners);
    for (std::size_t i = 1; i < kOpeners; ++i) {
      const auto reg = queues[i]->register_reader();
      ASSERT_TRUE(reg.has_value());
      reader_ids[i] = *reg;
    }
    ASSERT_NO_FATAL_FAILURE(WriteSequences(*queues[0], 0, 1));
    for (std::size_t i = 1; i < kOpeners; ++i) {
      DropNewestPayload copy{};
      MessageInfo info{};
      EXPECT_EQ(
          queues[i]->read_next(reader_ids[i],
                               std::as_writable_bytes(std::span{copy}), info),
          ReadResult::OK)
          << "round " << round << " opener " << i << " missed publication";
      EXPECT_EQ(FirstForeignWord(copy, 0), kDropNewestWords);
    }
  }
}

RTMSQueue OpenPublisher(const std::string& path, std::size_t slots) {
  return RTMSQueue{
      path,
      sizeof(DropNewestPayload),
      alignof(DropNewestPayload),
      kDropNewestSchema,
      slots,
      RTMSOptions{.overflow_policy = OverflowPolicy::DROP_NEWEST,
                  .read_mode = ReadMode::SEQUENCE,
                  .publisher = true},
  };
}

// Every queue in `queues` is attached to one segment other than
// `old_segment_id`, which counts exactly them, and each reads the publication
// of a publisher opened on `path` afterwards (on whatever segment the name
// then holds).
void ExpectOneSegmentDelivering(const std::string& path, std::size_t slots,
                                const std::vector<RTMSQueue*>& queues,
                                std::uint64_t old_segment_id) {
  ASSERT_FALSE(queues.empty());
  const RTMSHeader* shared_header = RegistrationPeer::Header(*queues[0]);
  EXPECT_NE(shared_header->segment_id, old_segment_id);
  for (std::size_t i = 1; i < queues.size(); ++i) {
    EXPECT_EQ(RegistrationPeer::Header(*queues[i])->segment_id,
              shared_header->segment_id)
        << "queue " << i << " is on another segment";
  }
  EXPECT_EQ(shared_header->lifetime.load(std::memory_order_acquire) &
                kLifetimeCountMask,
            queues.size());

  std::vector<std::size_t> reader_ids(queues.size());
  for (std::size_t i = 0; i < queues.size(); ++i) {
    const auto reg = queues[i]->register_reader();
    ASSERT_TRUE(reg.has_value()) << "queue " << i;
    reader_ids[i] = *reg;
  }
  RTMSQueue publisher = OpenPublisher(path, slots);
  ASSERT_NO_FATAL_FAILURE(WriteSequences(publisher, 0, 1));
  for (std::size_t i = 0; i < queues.size(); ++i) {
    DropNewestPayload copy{};
    MessageInfo info{};
    EXPECT_EQ(queues[i]->read_next(
                  reader_ids[i], std::as_writable_bytes(std::span{copy}), info),
              ReadResult::OK)
        << "queue " << i << " missed the publication";
    EXPECT_EQ(FirstForeignWord(copy, 0), kDropNewestWords) << "queue " << i;
  }
}

// More waits for a live claimant than the 200 after which an opener once
// took over from it: a claimant held this long is held past that wait.
constexpr int kWaitsPastSupersede = 250;

// Holds the first unlink of a closing segment (the claimant's, between its
// check of the name and its shm_unlink) until either every other opener's
// open has returned, or an opener has waited kWaitsPastSupersede times for
// the held claimant. Later unlinks pass.
struct UnlinkHold {
  std::mutex mutex;
  std::condition_variable changed;
  std::atomic<bool> taken{false};
  bool entered{false};
  bool released{false};
  std::size_t returned{0};
  int max_wait{0};
};
UnlinkHold* g_hold = nullptr;

void HoldFirstUnlink(std::uint64_t /*segment_id*/) {
  if (g_hold->taken.exchange(true)) return;
  std::unique_lock lock{g_hold->mutex};
  g_hold->entered = true;
  g_hold->changed.notify_all();
  g_hold->changed.wait(lock, [] { return g_hold->released; });
}

void ReleaseAfterLongWait(std::uint64_t /*segment_id*/, int wait_count) {
  const std::lock_guard lock{g_hold->mutex};
  g_hold->max_wait = std::max(g_hold->max_wait, wait_count);
  if (wait_count > kWaitsPastSupersede) {
    g_hold->released = true;
    g_hold->changed.notify_all();
  }
}

// An opener's open() returned; once `others` have, the hold is released.
void OpenerReturned(std::size_t others) {
  const std::lock_guard lock{g_hold->mutex};
  if (++g_hold->returned >= others) {
    g_hold->released = true;
    g_hold->changed.notify_all();
  }
}

struct HoldGuard {
  explicit HoldGuard(UnlinkHold& hold) {
    g_hold = &hold;
    RegistrationPeer::SetOnUnlinkVerified(HoldFirstUnlink);
    RegistrationPeer::SetOnReclaimWait(ReleaseAfterLongWait);
  }
  ~HoldGuard() { Clear(); }
  HoldGuard(const HoldGuard&) = delete;
  HoldGuard& operator=(const HoldGuard&) = delete;
  static void Clear() {
    RegistrationPeer::SetOnUnlinkVerified(nullptr);
    RegistrationPeer::SetOnReclaimWait(nullptr);
    g_hold = nullptr;
  }
};

// The waits for a live claimant one open() makes before it gives up, lowered
// from the production bound (about a second) so a test reaches it at once.
constexpr int kHeldWaitAttempts = 3;

// Sets how many times an open() waits for live claimants, restoring the
// production bound on destruction.
struct WaitAttemptsGuard {
  explicit WaitAttemptsGuard(int attempts)
      : saved{RegistrationPeer::ClaimantWaitAttempts()} {
    RegistrationPeer::SetClaimantWaitAttempts(attempts);
  }
  ~WaitAttemptsGuard() { RegistrationPeer::SetClaimantWaitAttempts(saved); }
  WaitAttemptsGuard(const WaitAttemptsGuard&) = delete;
  WaitAttemptsGuard& operator=(const WaitAttemptsGuard&) = delete;
  int saved;
};

// REQ-OPS-03: eight openers find an abandoned closing segment with no
// claimant; one claims it and is descheduled between checking that the name
// still holds it and unlinking it (held in the hook). The others wait for
// that live claimant however long it takes and never unlink in its place, so
// its unlink cannot remove a segment one of them created meanwhile: all eight
// end on one fresh segment. Seen in the Cloud Build stress lane (run
// 45133738392): seven openers on one segment, one alone on another.
TEST(RingRegistration, LiveReclaimerHeldPastTheWaitIsNotSuperseded) {
  constexpr std::size_t kSlots = 4;
  constexpr std::size_t kOpeners = 8;
  const std::string path = MakeUniquePath("/ring_reclaim_held");
  auto [old_mapping, old_segment_id] =
      MakeAbandonedClosingSegment(path, kSlots, kLifetimeClosing);

  UnlinkHold hold;
  HoldGuard guard{hold};
  const WaitAttemptsGuard unbounded{std::numeric_limits<int>::max()};
  std::latch ready{static_cast<std::ptrdiff_t>(kOpeners)};
  std::vector<std::optional<RTMSQueue>> queues(kOpeners);
  std::vector<std::string> errors(kOpeners);
  std::vector<std::thread> threads;
  threads.reserve(kOpeners);
  for (std::size_t i = 0; i < kOpeners; ++i) {
    threads.emplace_back([&, i] {
      ready.arrive_and_wait();
      try {
        queues[i].emplace(OpenSubscriber(path, kSlots));
      } catch (const std::exception& e) {
        errors[i] = e.what();
      }
      OpenerReturned(kOpeners - 1);
    });
  }
  for (auto& thread : threads) thread.join();
  const int max_wait = hold.max_wait;
  HoldGuard::Clear();

  ASSERT_TRUE(hold.taken.load()) << "no opener unlinked the closing segment";
  std::vector<RTMSQueue*> opened;
  for (std::size_t i = 0; i < kOpeners; ++i) {
    ASSERT_TRUE(errors[i].empty()) << "opener " << i << ": " << errors[i];
    ASSERT_TRUE(queues[i].has_value());
    opened.push_back(&*queues[i]);
  }
  SCOPED_TRACE(std::format("most waits by one opener for the held claimant: {}",
                           max_wait));
  ASSERT_NO_FATAL_FAILURE(
      ExpectOneSegmentDelivering(path, kSlots, opened, old_segment_id));
}

// REQ-OPS-03: the only queue of a segment closes, marks it closing and is
// descheduled between checking that the name still holds it and unlinking it
// (held in the hook). Four openers that arrive meanwhile wait for that live
// closer and never unlink in its place, so its unlink cannot remove the
// segment one of them created: they and a subscriber that opens after
// everything has finished share one segment.
TEST(RingRegistration, LiveCloserHeldBeforeUnlinkIsNotSuperseded) {
  constexpr std::size_t kSlots = 4;
  constexpr std::size_t kOpeners = 4;
  const std::string path = MakeUniquePath("/ring_closer_held");
  ::shm_unlink(ShmObjectName(path).c_str());

  UnlinkHold hold;
  HoldGuard guard{hold};
  const WaitAttemptsGuard unbounded{std::numeric_limits<int>::max()};
  std::optional<RTMSQueue> only;
  only.emplace(OpenSubscriber(path, kSlots));
  const std::uint64_t old_segment_id =
      RegistrationPeer::Header(*only)->segment_id;
  std::thread closer{[&] { only.reset(); }};
  {
    std::unique_lock lock{hold.mutex};
    hold.changed.wait(lock, [&] { return hold.entered; });
  }

  std::vector<std::optional<RTMSQueue>> queues(kOpeners);
  std::vector<std::string> errors(kOpeners);
  std::vector<std::thread> threads;
  threads.reserve(kOpeners);
  for (std::size_t i = 0; i < kOpeners; ++i) {
    threads.emplace_back([&, i] {
      try {
        queues[i].emplace(OpenSubscriber(path, kSlots));
      } catch (const std::exception& e) {
        errors[i] = e.what();
      }
      OpenerReturned(kOpeners);
    });
  }
  for (auto& thread : threads) thread.join();
  closer.join();
  const int max_wait = hold.max_wait;
  HoldGuard::Clear();

  RTMSQueue late = OpenSubscriber(path, kSlots);
  std::vector<RTMSQueue*> opened;
  for (std::size_t i = 0; i < kOpeners; ++i) {
    ASSERT_TRUE(errors[i].empty()) << "opener " << i << ": " << errors[i];
    ASSERT_TRUE(queues[i].has_value());
    opened.push_back(&*queues[i]);
  }
  opened.push_back(&late);
  SCOPED_TRACE(std::format(
      "queue {} opened last; most waits by one opener for the held closer: {}",
      kOpeners, max_wait));
  ASSERT_NO_FATAL_FAILURE(
      ExpectOneSegmentDelivering(path, kSlots, opened, old_segment_id));
}

// Forks a child that runs `in_child` and sends back what it returns (a
// trivially copyable report), then exits; waits for the child and stores the
// report in `report`.
template <typename Report, typename F>
void RunChild(F&& in_child, Report& report) {
  static_assert(std::is_trivially_copyable_v<Report>);
  std::array<int, 2> pipe_fds{};
  ASSERT_EQ(::pipe(pipe_fds.data()), 0);
  const pid_t child = ::fork();
  ASSERT_NE(child, -1);
  if (child == 0) {
    ::close(pipe_fds[0]);
    const Report sent = in_child();
    const bool ok = ::write(pipe_fds[1], &sent, sizeof(sent)) ==
                    static_cast<ssize_t>(sizeof(sent));
    ::_exit(ok ? 0 : 1);
  }
  ::close(pipe_fds[1]);
  report = Report{};
  const ssize_t got = ::read(pipe_fds[0], &report, sizeof(report));
  ::close(pipe_fds[0]);
  int status = 0;
  ASSERT_EQ(::waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFEXITED(status) && WEXITSTATUS(status) == 0);
  ASSERT_EQ(got, static_cast<ssize_t>(sizeof(report)));
}

// REQ-OPS-03: an opener that finds a closing segment whose claimant died
// before unlinking it (a child process that recorded itself and exited)
// claims the segment in its place without waiting, unlinks it, and reopens
// the name on a fresh segment.
TEST(RingRegistration, OpenerClaimsAClosingSegmentFromADeadClaimant) {
  constexpr std::size_t kSlots = 4;
  std::uint64_t dead_word = 0;
  ASSERT_NO_FATAL_FAILURE(
      RunChild([] { return talos::OwnProcessWord(); }, dead_word));

  const std::string path = MakeUniquePath("/ring_reclaim_dead");
  auto [old_mapping, old_segment_id] =
      MakeAbandonedClosingSegment(path, kSlots, kLifetimeClosing | dead_word);

  struct HookGuard {
    ~HookGuard() { RegistrationPeer::SetOnReclaimWait(nullptr); }
  } guard;
  static int dead_waits = 0;
  dead_waits = 0;
  RegistrationPeer::SetOnReclaimWait(
      [](std::uint64_t /*segment_id*/, int /*wait_count*/) { ++dead_waits; });

  RTMSQueue reopened = OpenSubscriber(path, kSlots);
  RegistrationPeer::SetOnReclaimWait(nullptr);

  EXPECT_EQ(dead_waits, 0) << "a dead claimant must not be waited for";
  const auto* old_header = static_cast<const RTMSHeader*>(old_mapping.ptr());
  EXPECT_EQ(old_header->lifetime.load(std::memory_order_acquire),
            kLifetimeClosing | talos::OwnProcessWord())
      << "the opener claims the segment in the dead claimant's place";
  EXPECT_NE(RegistrationPeer::Header(reopened)->segment_id, old_segment_id);
  EXPECT_EQ(RegistrationPeer::Header(reopened)->lifetime.load(
                std::memory_order_acquire) &
                kLifetimeCountMask,
            1U);
  EXPECT_EQ(reopened.writer_sequence(), 0U);
}

// What the child of ClaimantWhoseUnlinkFailedGivesTheClaimUp saw.
struct GiveUpReport {
  int failed_step{0};  // 0 when every step ran
  std::array<char, 200> error{};
  std::uint64_t old_segment_id{0};
  std::uint64_t lifetime_after_close{0};
  std::uint64_t reopened_segment_id{0};
  std::uint64_t reopened_count{0};
};

// REQ-OPS-03: the last queue of a segment closes, marking it closing and
// claiming it, but cannot check the name before unlinking (its process is
// out of file descriptors, so shm_open fails with EMFILE). It gives the claim
// up, leaving the segment closing with no claimant, so the next open of the
// name, once descriptors are available again, claims and unlinks it at once
// rather than wait for the still-live process and fail. Runs in a child, whose
// descriptor limit it lowers. (An open under the lowered limit fails in
// SharedMemoryPtr's own shm_open before it reaches the closing segment, so
// this does not reach an opener's own failed unlink.)
TEST(RingRegistration, ClaimantWhoseUnlinkFailedGivesTheClaimUp) {
  constexpr std::size_t kSlots = 4;
  const std::string path = MakeUniquePath("/ring_unlink_failed");
  ::shm_unlink(ShmObjectName(path).c_str());

  GiveUpReport report{};
  ASSERT_NO_FATAL_FAILURE(RunChild(
      [&path] {
        GiveUpReport seen{};
        int step = 1;
        try {
          // Computed now: the process word is read from /proc, which needs a
          // descriptor.
          (void)talos::OwnProcessWord();
          std::optional<RTMSQueue> queue;
          queue.emplace(OpenSubscriber(path, kSlots));
          seen.old_segment_id = RegistrationPeer::Header(*queue)->segment_id;
          SharedMemoryPtr mapping{path, sizeof(RTMSHeader)};
          const auto* header = static_cast<const RTMSHeader*>(mapping.ptr());

          step = 2;  // every descriptor below the limit in use
          rlimit original{};
          if (::getrlimit(RLIMIT_NOFILE, &original) != 0) {
            throw std::runtime_error("getrlimit failed");
          }
          const int lowest_free = ::open("/dev/null", O_RDONLY);
          if (lowest_free < 0) throw std::runtime_error("open failed");
          ::close(lowest_free);
          rlimit lowered = original;
          lowered.rlim_cur = static_cast<rlim_t>(lowest_free);
          if (::setrlimit(RLIMIT_NOFILE, &lowered) != 0) {
            throw std::runtime_error("setrlimit failed");
          }
          const int refused = ::open("/dev/null", O_RDONLY);
          if (refused != -1 || errno != EMFILE) {
            throw std::runtime_error("the lowered limit did not refuse");
          }

          step = 3;  // the close whose unlink cannot check the name
          queue.reset();
          seen.lifetime_after_close =
              header->lifetime.load(std::memory_order_acquire);

          step = 4;  // the name reopened with descriptors available
          if (::setrlimit(RLIMIT_NOFILE, &original) != 0) {
            throw std::runtime_error("restoring the limit failed");
          }
          RTMSQueue reopened = OpenSubscriber(path, kSlots);
          seen.reopened_segment_id =
              RegistrationPeer::Header(reopened)->segment_id;
          seen.reopened_count =
              RegistrationPeer::Header(reopened)->lifetime.load(
                  std::memory_order_acquire) &
              kLifetimeCountMask;
        } catch (const std::exception& e) {
          seen.failed_step = step;
          std::strncpy(seen.error.data(), e.what(), seen.error.size() - 1);
        }
        return seen;
      },
      report));

  ASSERT_EQ(report.failed_step, 0)
      << "the child failed at step " << report.failed_step << ": "
      << report.error.data();
  EXPECT_EQ(report.lifetime_after_close, kLifetimeClosing)
      << "the close whose unlink failed must give its claim up";
  EXPECT_NE(report.reopened_segment_id, report.old_segment_id);
  EXPECT_EQ(report.reopened_count, 1U);
}

RTMSQueue OpenSessionPublisher(const std::string& path, std::size_t slots) {
  return RTMSQueue{
      path,
      sizeof(DropNewestPayload),
      alignof(DropNewestPayload),
      kDropNewestSchema,
      slots,
      RTMSOptions{.overflow_policy = OverflowPolicy::DROP_NEWEST,
                  .read_mode = ReadMode::SEQUENCE,
                  .publisher = true,
                  .session_owner = true},
  };
}

// The name holds `session`'s segment: a queue opening it now attaches to that
// segment, session-owned, and reads what `session` then publishes.
void ExpectNameHoldsSessionSegment(const std::string& path, std::size_t slots,
                                   RTMSQueue& session) {
  const std::uint64_t session_id =
      RegistrationPeer::Header(session)->segment_id;
  RTMSQueue late = OpenSubscriber(path, slots);
  const RTMSHeader* late_header = RegistrationPeer::Header(late);
  EXPECT_EQ(late_header->segment_id, session_id)
      << "a queue opened after the session is on another segment";
  EXPECT_NE(late_header->lifetime.load(std::memory_order_acquire) &
                kLifetimeSessionOwned,
            0U)
      << "the name holds a standalone segment";
  const std::optional<std::size_t> reader = late.register_reader();
  ASSERT_TRUE(reader.has_value());
  ASSERT_NO_FATAL_FAILURE(WriteSequences(session, 0, 1));
  DropNewestPayload copy{};
  MessageInfo info{};
  EXPECT_EQ(
      late.read_next(*reader, std::as_writable_bytes(std::span{copy}), info),
      ReadResult::OK)
      << "the session's publication did not reach a queue opened after it";
  EXPECT_EQ(FirstForeignWord(copy, 0), kDropNewestWords);
}

// The session owner's waits for the held claimant before it is released:
// past the first, so a session owner that superseded a live claimant after
// one wait would do so while the claimant is still held.
constexpr int kSessionWaitsBeforeRelease = 2;

void ReleaseAfterSessionWaits(std::uint64_t /*segment_id*/, int wait_count) {
  const std::lock_guard lock{g_hold->mutex};
  g_hold->max_wait = std::max(g_hold->max_wait, wait_count);
  if (wait_count >= kSessionWaitsBeforeRelease) {
    g_hold->released = true;
    g_hold->changed.notify_all();
  }
}

// REQ-OPS-03: the only queue of a standalone segment closes, claims it and is
// descheduled between checking that the name still holds it and unlinking it
// (held in the hook) when a launcher session opens the name. The session
// owner waits for that live claimant rather than replace the segment under
// it, so the claimant's unlink cannot remove the session's segment: a queue
// opened afterwards is on the session's segment and receives its
// publication. The claimant is released once the session owner has waited
// for it kSessionWaitsBeforeRelease times, or once its open returned; the
// wait is unbounded here, so only an event ends it.
TEST(RingRegistration, SessionOwnerWaitsForALiveClaimantHeldBeforeItsUnlink) {
  constexpr std::size_t kSlots = 4;
  const std::string path = MakeUniquePath("/ring_session_held");
  ::shm_unlink(ShmObjectName(path).c_str());

  UnlinkHold hold;
  HoldGuard guard{hold};
  const WaitAttemptsGuard unbounded{std::numeric_limits<int>::max()};
  RegistrationPeer::SetOnReclaimWait(ReleaseAfterSessionWaits);
  std::optional<RTMSQueue> only;
  only.emplace(OpenSubscriber(path, kSlots));
  std::thread closer{[&] { only.reset(); }};
  {
    std::unique_lock lock{hold.mutex};
    hold.changed.wait(lock, [&] { return hold.entered; });
  }

  std::optional<RTMSQueue> session;
  std::string error;
  try {
    session.emplace(OpenSessionPublisher(path, kSlots));
  } catch (const std::exception& e) {
    error = e.what();
  }
  OpenerReturned(1);
  closer.join();
  const int max_wait = hold.max_wait;
  HoldGuard::Clear();

  ASSERT_TRUE(error.empty()) << "the session owner's open: " << error;
  ASSERT_TRUE(session.has_value());
  SCOPED_TRACE(
      std::format("session owner's waits for the held claimant: {}", max_wait));
  ASSERT_NO_FATAL_FAILURE(
      ExpectNameHoldsSessionSegment(path, kSlots, *session));
}

// What the holder process of ClaimedSegmentHolderClosesWithoutUnlinking sends
// its parent: that it attached, that its close is about to unlink (held
// there until the parent releases it), or that its close returned.
struct HolderMessage {
  enum Kind : int { kAttached = 1, kUnlinking = 2, kClosed = 3 };
  int kind{0};
  int failed{0};  // 1 when an exception ended the holder early
  std::uint64_t segment_id{0};
  std::uint64_t lifetime_after_close{0};
  std::array<char, 160> error{};
};

// What the parent sends the holder process, one byte each.
constexpr char kHolderClose = 'c';
constexpr char kHolderRelease = 'r';
constexpr char kHolderFinish = 'f';

// The two pipes between the parent and the holder process.
struct HolderLink {
  int to_holder{-1};    // the parent writes
  int from_holder{-1};  // the parent reads
  int holder_reads{-1};
  int holder_writes{-1};
  bool in_holder{false};  // set in the holder process after fork
  bool session_claimed{false};
  // A read from the holder failed (it died or closed its end): the parent
  // writes nothing more, so a dead holder cannot raise SIGPIPE in it.
  bool broken{false};
  HolderMessage first_after_close{};
};
HolderLink* g_link = nullptr;

bool WriteFull(int fd, const void* data, std::size_t size) {
  const auto* bytes = static_cast<const char*>(data);
  while (size > 0) {
    const ssize_t wrote = ::write(fd, bytes, size);
    if (wrote < 0 && errno == EINTR) continue;
    if (wrote <= 0) return false;
    bytes += wrote;
    size -= static_cast<std::size_t>(wrote);
  }
  return true;
}

bool ReadFull(int fd, void* data, std::size_t size) {
  auto* bytes = static_cast<char*>(data);
  while (size > 0) {
    const ssize_t got = ::read(fd, bytes, size);
    if (got < 0 && errno == EINTR) continue;
    if (got <= 0) return false;
    bytes += got;
    size -= static_cast<std::size_t>(got);
  }
  return true;
}

// The parent's side of the link: one command byte to the holder, one
// message from it. Both fail, without touching the pipe, once the link broke.
bool SendToHolder(char command) {
  if (g_link->broken) return false;
  if (!WriteFull(g_link->to_holder, &command, 1)) g_link->broken = true;
  return !g_link->broken;
}

bool ReceiveFromHolder(HolderMessage& message) {
  if (g_link->broken) return false;
  if (!ReadFull(g_link->from_holder, &message, sizeof(message))) {
    g_link->broken = true;
  }
  return !g_link->broken;
}

// The parent tells the holder process to close and reads what its close did
// first: returned (kClosed), or reached its unlink (kUnlinking, held there).
HolderMessage CloseHolder() {
  HolderMessage message{};
  if (!SendToHolder(kHolderClose) || !ReceiveFromHolder(message)) {
    message.failed = 1;
  }
  return message;
}

// Closes `fd` if it is open and marks it closed.
void CloseFd(int& fd) {
  if (fd >= 0) ::close(fd);
  fd = -1;
}

// on_session_claimed_, in the parent: the holder closes between the session
// owner's claim and its unlink.
void CloseHolderAtClaim(std::uint64_t /*segment_id*/) {
  if (g_link->in_holder || g_link->session_claimed) return;
  g_link->session_claimed = true;
  g_link->first_after_close = CloseHolder();
}

// on_unlink_verified_: in the holder process, holds the holder's own unlink,
// should its close make one, until the parent releases it; passes the
// parent's.
void HoldHolderUnlink(std::uint64_t /*segment_id*/) {
  if (!g_link->in_holder) return;
  HolderMessage message{};
  message.kind = HolderMessage::kUnlinking;
  char release = 0;
  if (!WriteFull(g_link->holder_writes, &message, sizeof(message)) ||
      !ReadFull(g_link->holder_reads, &release, 1)) {
    ::_exit(2);
  }
}

// The holder process: attaches to `path` (creating it, count 1), reports,
// closes when told, and reports the old segment's lifetime word as it saw it
// after its close.
[[noreturn]] void RunHolder(const std::string& path, std::size_t slots) {
  g_link->in_holder = true;
  ::close(g_link->to_holder);
  ::close(g_link->from_holder);
  HolderMessage message{};
  try {
    std::optional<RTMSQueue> holder;
    holder.emplace(OpenSubscriber(path, slots));
    SharedMemoryPtr mapping{path, sizeof(RTMSHeader)};
    const auto* header = static_cast<const RTMSHeader*>(mapping.ptr());
    message.kind = HolderMessage::kAttached;
    message.segment_id = header->segment_id;
    char command = 0;
    if (!WriteFull(g_link->holder_writes, &message, sizeof(message)) ||
        !ReadFull(g_link->holder_reads, &command, 1) ||
        command != kHolderClose) {
      ::_exit(2);
    }
    holder.reset();
    message.kind = HolderMessage::kClosed;
    message.lifetime_after_close =
        header->lifetime.load(std::memory_order_acquire);
  } catch (const std::exception& e) {
    message.kind = HolderMessage::kClosed;
    message.failed = 1;
    std::strncpy(message.error.data(), e.what(), message.error.size() - 1);
  }
  char finish = 0;
  const bool ok = WriteFull(g_link->holder_writes, &message, sizeof(message)) &&
                  ReadFull(g_link->holder_reads, &finish, 1) &&
                  finish == kHolderFinish;
  ::_exit(ok ? 0 : 1);
}

// REQ-OPS-03: a launcher session opens a name whose standalone segment is
// still attached by a queue of another process (a node: count 1). The
// session owner takes the segment's unlink from it (marks it closing with
// itself as claimant), and that queue, closing between the claim and the
// session owner's unlink (in the hook), recognises the other process's claim:
// it leaves the claim as it is and unlinks nothing, even with its unlink
// held until the session's segment exists. The name ends on the session's
// segment, and the old segment's word, as the holder saw it after its close,
// names the session owner's process.
TEST(RingRegistration, ClaimedSegmentHolderClosesWithoutUnlinking) {
  constexpr std::size_t kSlots = 4;
  const std::string path = MakeUniquePath("/ring_session_count");
  ::shm_unlink(ShmObjectName(path).c_str());

  std::array<int, 2> to_holder{-1, -1};
  std::array<int, 2> from_holder{-1, -1};
  struct PipeGuard {
    PipeGuard(std::array<int, 2>& first_in, std::array<int, 2>& second_in)
        : first{first_in}, second{second_in} {}
    std::array<int, 2>& first;
    std::array<int, 2>& second;
    PipeGuard(const PipeGuard&) = delete;
    PipeGuard& operator=(const PipeGuard&) = delete;
    ~PipeGuard() {
      for (int& fd : first) CloseFd(fd);
      for (int& fd : second) CloseFd(fd);
    }
  } pipes{to_holder, from_holder};
  ASSERT_EQ(::pipe(to_holder.data()), 0);
  ASSERT_EQ(::pipe(from_holder.data()), 0);
  HolderLink link{.to_holder = to_holder[1],
                  .from_holder = from_holder[0],
                  .holder_reads = to_holder[0],
                  .holder_writes = from_holder[1]};
  g_link = &link;
  struct LinkGuard {
    ~LinkGuard() {
      RegistrationPeer::SetOnSessionClaimed(nullptr);
      RegistrationPeer::SetOnUnlinkVerified(nullptr);
      g_link = nullptr;
    }
  } link_guard;
  RegistrationPeer::SetOnSessionClaimed(CloseHolderAtClaim);
  RegistrationPeer::SetOnUnlinkVerified(HoldHolderUnlink);
  const pid_t child = ::fork();
  ASSERT_NE(child, -1);
  if (child == 0) RunHolder(path, kSlots);
  CloseFd(to_holder[0]);
  CloseFd(from_holder[1]);

  HolderMessage attached{};
  const bool heard = ReceiveFromHolder(attached);
  std::optional<RTMSQueue> session;
  std::string error;
  if (heard && attached.kind == HolderMessage::kAttached) {
    try {
      session.emplace(OpenSessionPublisher(path, kSlots));
    } catch (const std::exception& e) {
      error = e.what();
    }
  }
  RegistrationPeer::SetOnSessionClaimed(nullptr);
  RegistrationPeer::SetOnUnlinkVerified(nullptr);
  HolderMessage closed{};
  bool held_unlink = false;
  bool finished = false;
  if (heard && attached.kind == HolderMessage::kAttached) {
    // A session owner that never claimed still lets the holder close.
    closed = link.session_claimed ? link.first_after_close : CloseHolder();
    held_unlink = closed.kind == HolderMessage::kUnlinking;
    if (held_unlink &&
        (!SendToHolder(kHolderRelease) || !ReceiveFromHolder(closed))) {
      closed.failed = 1;
    }
    finished = SendToHolder(kHolderFinish);
  }
  CloseFd(to_holder[1]);
  CloseFd(from_holder[0]);
  int status = 0;
  ASSERT_EQ(::waitpid(child, &status, 0), child);

  ASSERT_TRUE(heard && attached.kind == HolderMessage::kAttached &&
              attached.failed == 0)
      << "the holder did not attach: " << attached.error.data();
  ASSERT_TRUE(finished && WIFEXITED(status) && WEXITSTATUS(status) == 0)
      << "the holder process did not finish";
  ASSERT_TRUE(error.empty()) << "the session owner's open: " << error;
  ASSERT_TRUE(session.has_value());
  EXPECT_TRUE(link.session_claimed)
      << "the session owner never claimed the segment";
  EXPECT_FALSE(held_unlink) << "the holder's close unlinked the name";
  EXPECT_EQ(closed.failed, 0) << closed.error.data();
  EXPECT_EQ(closed.lifetime_after_close,
            kLifetimeClosing | talos::OwnProcessWord())
      << "the holder's close changed the session owner's claim";
  EXPECT_NE(RegistrationPeer::Header(*session)->segment_id,
            attached.segment_id);
  ASSERT_NO_FATAL_FAILURE(
      ExpectNameHoldsSessionSegment(path, kSlots, *session));
}

// REQ-OPS-03: a closing segment whose claimant died before unlinking it
// neither delays nor refuses a launch: the session owner does not wait for
// the dead claimant, its open succeeds, and the name holds its segment. With
// the claimant wait bound lowered, a session owner that waited for a dead
// claimant would fail here at once rather than after a second.
TEST(RingRegistration, SessionOwnerClaimsAClosingSegmentFromADeadClaimant) {
  constexpr std::size_t kSlots = 4;
  std::uint64_t dead_word = 0;
  ASSERT_NO_FATAL_FAILURE(
      RunChild([] { return talos::OwnProcessWord(); }, dead_word));

  const std::string path = MakeUniquePath("/ring_session_dead");
  auto [old_mapping, old_segment_id] =
      MakeAbandonedClosingSegment(path, kSlots, kLifetimeClosing | dead_word);

  const WaitAttemptsGuard bounded{kHeldWaitAttempts};
  struct HookGuard {
    ~HookGuard() { RegistrationPeer::SetOnReclaimWait(nullptr); }
  } guard;
  static int dead_waits = 0;
  dead_waits = 0;
  RegistrationPeer::SetOnReclaimWait(
      [](std::uint64_t /*segment_id*/, int /*wait_count*/) { ++dead_waits; });

  std::optional<RTMSQueue> session;
  std::string error;
  try {
    session.emplace(OpenSessionPublisher(path, kSlots));
  } catch (const std::runtime_error& e) {
    error = e.what();
  }
  RegistrationPeer::SetOnReclaimWait(nullptr);

  EXPECT_EQ(dead_waits, 0) << "a dead claimant must not be waited for";
  ASSERT_TRUE(error.empty()) << "the session owner's open: " << error;
  ASSERT_TRUE(session.has_value());
  EXPECT_NE(RegistrationPeer::Header(*session)->segment_id, old_segment_id);
  ASSERT_NO_FATAL_FAILURE(
      ExpectNameHoldsSessionSegment(path, kSlots, *session));
}

// The standalone queue a test's on_session_claimed_ opens on a name it has
// just unlinked, and the path it does so for (once).
std::optional<RTMSQueue>* g_recreated = nullptr;
const std::string* g_recreated_path = nullptr;

// on_session_claimed_: the first time, the segment the session owner has just
// claimed leaves the name (its claimant's unlink) and a standalone queue
// creates another there, before the session owner's unlink.
void RecreateAtClaim(std::uint64_t /*segment_id*/) {
  if (g_recreated->has_value()) return;
  ::shm_unlink(ShmObjectName(*g_recreated_path).c_str());
  g_recreated->emplace(OpenSubscriber(*g_recreated_path, 4));
}

// REQ-OPS-03: a launcher session claims a closing segment whose claimant died,
// but meanwhile the name came to hold another segment (the dead claimant had
// unlinked the claimed one, and a standalone queue created a new one), so
// the session owner's unlink of the claimed segment unlinks nothing. It looks
// at the name again and replaces the new segment too: its open succeeds and
// the name holds the session's segment, while the standalone queue keeps its
// own.
TEST(RingRegistration, SessionOwnerReplacesASegmentCreatedDuringItsClaim) {
  constexpr std::size_t kSlots = 4;
  std::uint64_t dead_word = 0;
  ASSERT_NO_FATAL_FAILURE(
      RunChild([] { return talos::OwnProcessWord(); }, dead_word));

  const std::string path = MakeUniquePath("/ring_session_recreated");
  auto [old_mapping, old_segment_id] =
      MakeAbandonedClosingSegment(path, kSlots, kLifetimeClosing | dead_word);

  std::optional<RTMSQueue> recreated;
  g_recreated = &recreated;
  g_recreated_path = &path;
  struct HookGuard {
    ~HookGuard() {
      RegistrationPeer::SetOnSessionClaimed(nullptr);
      g_recreated = nullptr;
      g_recreated_path = nullptr;
    }
  } guard;
  RegistrationPeer::SetOnSessionClaimed(RecreateAtClaim);

  std::optional<RTMSQueue> session;
  std::string error;
  try {
    session.emplace(OpenSessionPublisher(path, kSlots));
  } catch (const std::runtime_error& e) {
    error = e.what();
  }
  RegistrationPeer::SetOnSessionClaimed(nullptr);

  ASSERT_TRUE(recreated.has_value()) << "the name was never recreated";
  ASSERT_TRUE(error.empty()) << "the session owner's open: " << error;
  ASSERT_TRUE(session.has_value());
  EXPECT_NE(RegistrationPeer::Header(*session)->segment_id, old_segment_id);
  EXPECT_NE(RegistrationPeer::Header(*recreated)->segment_id,
            RegistrationPeer::Header(*session)->segment_id);
  ASSERT_NO_FATAL_FAILURE(
      ExpectNameHoldsSessionSegment(path, kSlots, *session));
}

// The segment the name holds now, read without attaching to it.
struct NamedSegment {
  std::uint64_t segment_id{0};
  std::uint64_t lifetime{0};
};

std::optional<NamedSegment> SegmentAtName(const std::string& path) {
  const int fd = ::shm_open(ShmObjectName(path).c_str(), O_RDONLY, 0);
  if (fd == -1) return std::nullopt;
  void* mapped =
      ::mmap(nullptr, sizeof(RTMSHeader), PROT_READ, MAP_SHARED, fd, 0);
  ::close(fd);
  if (mapped == MAP_FAILED) return std::nullopt;
  const auto* header = static_cast<const RTMSHeader*>(mapped);
  const NamedSegment seen{header->segment_id,
                          header->lifetime.load(std::memory_order_acquire)};
  ::munmap(mapped, sizeof(RTMSHeader));
  return seen;
}

// What an open made while a live claimant was held before its unlink.
struct HeldClaimantOutcome {
  std::string error;  // what the open threw; empty when it returned
  std::optional<RTMSQueue> opened;
  std::uint64_t claimant_segment_id{0};
  // What the name held once the open had returned or thrown, while the
  // claimant was still held.
  std::optional<NamedSegment> named;
  int waits{0};  // the most waits for the held claimant on_reclaim_wait_ saw
};

// Releases a held claimant and joins its thread however the scope ends.
struct ReleaseAndJoin {
  ReleaseAndJoin(UnlinkHold& hold_in, std::thread& closer_in)
      : hold{hold_in}, closer{closer_in} {}
  ReleaseAndJoin(const ReleaseAndJoin&) = delete;
  ReleaseAndJoin& operator=(const ReleaseAndJoin&) = delete;
  ~ReleaseAndJoin() {
    {
      const std::lock_guard lock{hold.mutex};
      hold.released = true;
      hold.changed.notify_all();
    }
    if (closer.joinable()) closer.join();
  }
  UnlinkHold& hold;
  std::thread& closer;
};

// The only queue of a standalone segment on `path` closes and is held between
// its check of the name and its unlink while `open` runs, with the claimant
// wait bound at kHeldWaitAttempts; the close then finishes.
template <typename Open>
HeldClaimantOutcome OpenPastAHeldClaimant(const std::string& path,
                                          std::size_t slots, Open open) {
  ::shm_unlink(ShmObjectName(path).c_str());
  HeldClaimantOutcome outcome;
  UnlinkHold hold;
  HoldGuard guard{hold};
  const WaitAttemptsGuard attempts{kHeldWaitAttempts};
  std::optional<RTMSQueue> only;
  only.emplace(OpenSubscriber(path, slots));
  outcome.claimant_segment_id = RegistrationPeer::Header(*only)->segment_id;
  std::thread closer{[&] { only.reset(); }};
  const ReleaseAndJoin finish{hold, closer};
  {
    std::unique_lock lock{hold.mutex};
    hold.changed.wait(lock, [&] { return hold.entered; });
  }
  try {
    outcome.opened.emplace(open());
  } catch (const std::runtime_error& e) {
    outcome.error = e.what();
  }
  outcome.named = SegmentAtName(path);
  const std::lock_guard lock{hold.mutex};
  outcome.waits = hold.max_wait;
  return outcome;
}

// REQ-OPS-03: a launcher session opens a name whose standalone segment's live
// claimant is held before its unlink for longer than the session owner waits.
// The session owner refuses, naming the topic and the claimant's pid, and
// neither claims nor unlinks the segment: the name still holds it, closing
// with the same claimant.
TEST(RingRegistration, SessionOwnerRefusesALiveClaimantThatOutlastsTheWait) {
  constexpr std::size_t kSlots = 4;
  const std::string path = MakeUniquePath("/ring_session_bound");
  const HeldClaimantOutcome outcome = OpenPastAHeldClaimant(
      path, kSlots, [&] { return OpenSessionPublisher(path, kSlots); });

  SCOPED_TRACE(std::format("session owner's waits for the held claimant: {}",
                           outcome.waits));
  EXPECT_FALSE(outcome.error.empty()) << "the session owner's open returned";
  EXPECT_NE(outcome.error.find(path), std::string::npos) << outcome.error;
  EXPECT_NE(outcome.error.find("pid " + std::to_string(::getpid())),
            std::string::npos)
      << outcome.error;
  ASSERT_TRUE(outcome.named.has_value()) << "the name was unlinked";
  EXPECT_EQ(outcome.named->segment_id, outcome.claimant_segment_id)
      << "the name holds another segment";
  EXPECT_EQ(outcome.named->lifetime, kLifetimeClosing | talos::OwnProcessWord())
      << "the session owner changed the claim";
}

// REQ-OPS-03: a queue opens a name whose standalone segment's live claimant
// is held before its unlink for longer than an opener waits. The open gives
// up with the "stayed closing" error naming the topic, and neither claims nor
// unlinks the segment: the name still holds it, closing with the same
// claimant.
TEST(RingRegistration, OpenerRefusesALiveClaimantThatOutlastsTheWait) {
  constexpr std::size_t kSlots = 4;
  const std::string path = MakeUniquePath("/ring_opener_bound");
  const HeldClaimantOutcome outcome = OpenPastAHeldClaimant(
      path, kSlots, [&] { return OpenSubscriber(path, kSlots); });

  SCOPED_TRACE(
      std::format("opener's waits for the held claimant: {}", outcome.waits));
  EXPECT_FALSE(outcome.error.empty()) << "the open returned";
  EXPECT_NE(outcome.error.find(path), std::string::npos) << outcome.error;
  ASSERT_TRUE(outcome.named.has_value()) << "the name was unlinked";
  EXPECT_EQ(outcome.named->segment_id, outcome.claimant_segment_id)
      << "the name holds another segment";
  EXPECT_EQ(outcome.named->lifetime, kLifetimeClosing | talos::OwnProcessWord())
      << "the opener changed the claim";
}

}  // namespace
