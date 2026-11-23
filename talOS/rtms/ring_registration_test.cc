// Deterministic reader-registration boundary tests: a reader registered while
// a DROP_NEWEST writer publishes, its two steps run one at a time.

#include <gtest/gtest.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <format>
#include <latch>
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

// REQ-OPS-03: concurrent openers racing to reclaim a kLifetimeClosing segment
// must elect a single reclaimer and converge on one fresh segment without
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
    EXPECT_NE(old_header->lifetime.load(std::memory_order_acquire) &
                  kLifetimeReclaiming,
              0U)
        << "round " << round;

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

// REQ-OPS-03: an opener encountering a closing segment whose reclamation is
// already claimed (kLifetimeReclaiming):
// (a) yields to a live/recent claim rather than racing into UnlinkIfSame()
//     immediately, attaching to the winner's replacement segment once ready;
// (b) eventually supersedes and reclaims an abandoned/stalled claim whose
//     winner died before UnlinkIfSame() instead of wedging permanently; and
// (c) resets its wait budget if a different closing segment_id appears
// mid-loop.
TEST(RingRegistration, OpenerDoesNotUnlinkAlreadyClaimedClosingSegment) {
  constexpr std::size_t kSlots = 4;

  struct HookGuard {
    ~HookGuard() { RegistrationPeer::SetOnReclaimWait(nullptr); }
  } guard;

  // (a) Live/recent claim: loser yields while winner finishes UnlinkIfSame()
  // and creates a fresh segment; loser then attaches to that winner segment.
  {
    const std::string path = MakeUniquePath("/ring_reclaim_live");
    auto [old_mapping, old_segment_id] = MakeAbandonedClosingSegment(
        path, kSlots, kLifetimeClosing | kLifetimeReclaiming);

    struct LiveState {
      std::string path;
      std::size_t slots{0};
      std::uint64_t old_segment_id{0};
      int wait_calls{0};
      bool untouched_on_first_wait{false};
      std::optional<RTMSQueue> winner;
      std::uint64_t winner_segment_id{0};
    };
    static LiveState* live = nullptr;
    LiveState state{path, kSlots, old_segment_id};
    live = &state;
    RegistrationPeer::SetOnReclaimWait(
        [](std::uint64_t segment_id, int wait_count) {
          ++live->wait_calls;
          if (wait_count == 1) {
            SharedMemoryPtr probe{live->path, sizeof(RTMSHeader)};
            const auto* hdr = static_cast<const RTMSHeader*>(probe.ptr());
            live->untouched_on_first_wait =
                probe.mode() == SharedMemoryMode::ATTACH &&
                segment_id == live->old_segment_id &&
                hdr->segment_id == live->old_segment_id;
          } else if (wait_count == 2) {
            ::shm_unlink(ShmObjectName(live->path).c_str());
            live->winner.emplace(OpenSubscriber(live->path, live->slots));
            live->winner_segment_id =
                RegistrationPeer::Header(*live->winner)->segment_id;
          }
        });

    RTMSQueue loser = OpenSubscriber(path, kSlots);
    RegistrationPeer::SetOnReclaimWait(nullptr);
    live = nullptr;

    EXPECT_EQ(state.wait_calls, 2);
    EXPECT_TRUE(state.untouched_on_first_wait)
        << "loser must yield on kLifetimeReclaiming rather than unlink "
           "immediately";
    ASSERT_TRUE(state.winner.has_value());
    ASSERT_NE(state.winner_segment_id, old_segment_id);
    EXPECT_EQ(RegistrationPeer::Header(loser)->segment_id,
              state.winner_segment_id);
    EXPECT_EQ(RegistrationPeer::Header(loser)->lifetime.load(
                  std::memory_order_acquire) &
                  kLifetimeCountMask,
              2U);
  }

  // (b) Abandoned/stalled claim: winner died before UnlinkIfSame(). After
  // exhausting its reclaim wait budget, the opener supersedes and reclaims the
  // segment rather than failing permanently.
  {
    const std::string path = MakeUniquePath("/ring_reclaim_stalled");
    auto [old_mapping, old_segment_id] = MakeAbandonedClosingSegment(
        path, kSlots, kLifetimeClosing | kLifetimeReclaiming);

    struct StalledState {
      std::uint64_t expected_segment_id{0};
      int max_wait_count{0};
      bool segment_matched_every_wait{true};
    };
    static StalledState* stalled = nullptr;
    StalledState state{old_segment_id};
    stalled = &state;
    RegistrationPeer::SetOnReclaimWait(
        [](std::uint64_t segment_id, int wait_count) {
          stalled->max_wait_count = wait_count;
          if (segment_id != stalled->expected_segment_id) {
            stalled->segment_matched_every_wait = false;
          }
        });

    RTMSQueue reopened = OpenSubscriber(path, kSlots);
    RegistrationPeer::SetOnReclaimWait(nullptr);
    stalled = nullptr;

    EXPECT_GT(state.max_wait_count, 1)
        << "opener must exhaust reclaim wait budget before superseding";
    EXPECT_TRUE(state.segment_matched_every_wait);
    EXPECT_NE(RegistrationPeer::Header(reopened)->segment_id, old_segment_id);
    EXPECT_EQ(RegistrationPeer::Header(reopened)->lifetime.load(
                  std::memory_order_acquire) &
                  kLifetimeCountMask,
              1U);
    EXPECT_EQ(reopened.writer_sequence(), 0U);
  }

  // (c) Segment rotation mid-retry: if a second closing segment replaces the
  // first right as reclaim_waits reaches its budget, per-segment wait counters
  // reset so the new segment receives its full reclaim wait budget too.
  {
    const std::string path = MakeUniquePath("/ring_reclaim_rotated");
    auto [seg1_mapping, seg1_id] = MakeAbandonedClosingSegment(
        path, kSlots, kLifetimeClosing | kLifetimeReclaiming);

    struct RotatedState {
      std::string path;
      std::size_t slots{0};
      std::uint64_t seg1_id{0};
      std::uint64_t seg2_id{0};
      std::optional<SharedMemoryPtr> seg2_mapping;
      int seg1_waits{0};
      int seg2_waits{0};
    };
    static RotatedState* rotated = nullptr;
    RotatedState state{path, kSlots, seg1_id};
    rotated = &state;
    RegistrationPeer::SetOnReclaimWait(
        [](std::uint64_t segment_id, int wait_count) {
          if (segment_id == rotated->seg1_id) {
            rotated->seg1_waits = wait_count;
            if (wait_count == 1) {
              auto [mapping, id] = MakeAbandonedClosingSegment(
                  rotated->path, rotated->slots,
                  kLifetimeClosing | kLifetimeReclaiming);
              rotated->seg2_mapping.emplace(std::move(mapping));
              rotated->seg2_id = id;
            }
          } else if (segment_id == rotated->seg2_id) {
            rotated->seg2_waits = wait_count;
          }
        });

    RTMSQueue reopened = OpenSubscriber(path, kSlots);
    RegistrationPeer::SetOnReclaimWait(nullptr);
    rotated = nullptr;

    ASSERT_NE(state.seg2_id, 0U);
    ASSERT_NE(state.seg2_id, seg1_id);
    EXPECT_EQ(state.seg1_waits, 1);
    EXPECT_EQ(state.seg2_waits, 200)
        << "rotated closing segment must get a full reset reclaim wait budget";
    EXPECT_NE(RegistrationPeer::Header(reopened)->segment_id, seg1_id);
    EXPECT_NE(RegistrationPeer::Header(reopened)->segment_id, state.seg2_id);
  }
}

}  // namespace
