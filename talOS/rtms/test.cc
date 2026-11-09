#include <fcntl.h>
#include <flatbuffers/flatbuffers.h>
#include <gtest/gtest.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <future>
#include <iterator>
#include <latch>
#include <memory>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "rtms.h"
#include "rules_cc/cc/runfiles/runfiles.h"
#include "talOS/rtms/test_message_generated.h"
#include "talOS/schema/schema_hash.h"

namespace {

using namespace std::chrono_literals;

constexpr auto kTestTimeout = 5s;

// The schema identity every queue of these tests uses.
const talos::schema::Identity kSchema{"Message.TestMessage", 0x5157};

std::string make_test_path(std::string_view test_name) {
  return "/rtms_" + std::string{test_name} + "_" + std::to_string(::getpid());
}

RTMSOptions sequence_options() {
  return RTMSOptions{
      .overflow_policy = OverflowPolicy::DROP_NEWEST,
      .read_mode = ReadMode::SEQUENCE,
  };
}

std::span<const std::byte> BytesOf(const Message::TestMessage& message) {
  return std::as_bytes(std::span<const Message::TestMessage, 1>{&message, 1});
}

RTMSOptions latest_options() {
  return RTMSOptions{
      .overflow_policy = OverflowPolicy::OVERWRITE_OLDEST,
      .read_mode = ReadMode::LATEST,
  };
}

// Never blocks the writer; read_next() reports laps via MessageInfo::dropped
// instead of the reader silently missing them.
RTMSOptions overwrite_sequence_options() {
  return RTMSOptions{
      .overflow_policy = OverflowPolicy::OVERWRITE_OLDEST,
      .read_mode = ReadMode::SEQUENCE,
  };
}

}  // namespace
TEST(LayoutValidation, RejectsInvalidOptionsBeforeMapping) {
  const auto path = make_test_path("invalid");
  EXPECT_THROW((RTMSQueue{path, 0, 8, kSchema, 2}), std::invalid_argument);
  for (std::size_t alignment : {0u, 3u}) {
    EXPECT_THROW((RTMSQueue{path, 8, alignment, kSchema, 2}),
                 std::invalid_argument);
  }
  for (std::size_t slots : {0u, 1u, 3u, 2048u}) {
    EXPECT_THROW((RTMSQueue{path, 8, 8, kSchema, slots}),
                 std::invalid_argument);
  }
}

TEST(LayoutValidation, RejectsCorruptOffsetAndStride) {
  const auto path = make_test_path("layout");
  RTMSQueue owner{path, 8, 8, kSchema, 2, {.session_owner = true}};
  SharedMemoryPtr mapping{path, sizeof(RTMSHeader)};
  auto* header = static_cast<RTMSHeader*>(mapping.ptr());
  ASSERT_NE(header, nullptr);
  const auto offset = header->data_offset;
  header->data_offset += 8;
  EXPECT_THROW((RTMSQueue{path, 8, 8, kSchema, 2}), std::runtime_error);
  header->data_offset = offset;
  const auto stride = header->slot_stride;
  header->slot_stride += 8;
  EXPECT_THROW((RTMSQueue{path, 8, 8, kSchema, 2}), std::runtime_error);
  header->slot_stride = stride;
}

TEST(LayoutValidation, SlotsIncludeAlignedPublicationTime) {
  struct Layout {
    std::size_t message_size;
    std::size_t alignment;
    std::size_t time_offset;
    std::size_t stride;
  };
  for (const Layout layout : {Layout{1, 1, 8, 16}, Layout{8, 4, 8, 16},
                              Layout{16, 8, 16, 24}, Layout{16, 16, 16, 32}}) {
    const auto path =
        make_test_path("stamp_layout_" + std::to_string(layout.message_size) +
                       "_" + std::to_string(layout.alignment));
    RTMSQueue queue{path, layout.message_size, layout.alignment, kSchema, 2};
    SharedMemoryPtr mapping{path, sizeof(RTMSHeader)};
    const auto* header = static_cast<const RTMSHeader*>(mapping.ptr());
    EXPECT_EQ(header->version, 5u);
    EXPECT_EQ(header->slot_stride, layout.stride);
    EXPECT_EQ(header->data_offset % rtms_slot_alignment(layout.alignment), 0u);
    EXPECT_EQ(rtms_publish_time_offset(layout.message_size),
              layout.time_offset);
    EXPECT_EQ(header->total_bytes, header->data_offset + 2 * layout.stride);
  }
}

TEST(ReaderOwnership, ReleaseRejectsInvalidFreeAndForeignReaders) {
  const auto path = make_test_path("release");
  RTMSQueue queue{path, 8, 8, kSchema, 2};
  SharedMemoryPtr mapping{path, sizeof(RTMSHeader)};
  auto* header = static_cast<RTMSHeader*>(mapping.ptr());
  ASSERT_NE(header, nullptr);
  queue.release_reader(MAX_READERS);
  queue.release_reader(static_cast<std::size_t>(-1));
  queue.release_reader(0);
  EXPECT_EQ(header->readers[0].state.load(), 0u);
  const auto foreign = (static_cast<uint64_t>(::getpid() + 1) << 2) |
                       static_cast<uint64_t>(ReaderState::ACTIVE);
  header->readers[0].state.store(foreign);
  queue.release_reader(0);
  EXPECT_EQ(header->readers[0].state.load(), foreign);
  header->readers[0].state.store(0);
  const auto reader = queue.register_reader();
  ASSERT_TRUE(reader);
  queue.release_reader(*reader);
  EXPECT_EQ(header->readers[*reader].state.load(), 0u);
}

TEST(WriterOwnership, RefusesSecondLiveWriterAndReleasesOnClose) {
  const auto path = make_test_path("writer");
  RTMSQueue session{path,
                    sizeof(Message::TestMessage),
                    alignof(Message::TestMessage),
                    kSchema,
                    64,
                    {.session_owner = true}};
  {
    RTMSQueue writer{path,
                     sizeof(Message::TestMessage),
                     alignof(Message::TestMessage),
                     kSchema,
                     64,
                     {.publisher = true}};
    EXPECT_THROW((RTMSQueue{path,
                            sizeof(Message::TestMessage),
                            alignof(Message::TestMessage),
                            kSchema,
                            64,
                            {.publisher = true}}),
                 std::runtime_error);
  }
  EXPECT_NO_THROW((RTMSQueue{path,
                             sizeof(Message::TestMessage),
                             alignof(Message::TestMessage),
                             kSchema,
                             64,
                             {.publisher = true}}));
}

TEST(WriterOwnership, ReclaimsCrashedWriterWithoutLosingReaders) {
  const auto path = make_test_path("crash_writer");
  RTMSQueue session{path,
                    sizeof(Message::TestMessage),
                    alignof(Message::TestMessage),
                    kSchema,
                    64,
                    {.session_owner = true}};
  const auto reader = session.register_reader();
  if (!reader.has_value()) {
    FAIL() << "Expected a registered reader";
  }
  const pid_t child = ::fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    RTMSQueue writer{path,
                     sizeof(Message::TestMessage),
                     alignof(Message::TestMessage),
                     kSchema,
                     64,
                     {.publisher = true}};
    writer.write(BytesOf(Message::TestMessage{1, 1.0F}));
    ::_exit(0);  // Skip destruction, like a crash.
  }
  int status = 0;
  ASSERT_EQ(::waitpid(child, &status, 0), child);
  ASSERT_TRUE(WIFEXITED(status));
  ASSERT_EQ(WEXITSTATUS(status), 0);
  ASSERT_EQ(session.writer_sequence(), 1u);
  RTMSQueue restarted{path,
                      sizeof(Message::TestMessage),
                      alignof(Message::TestMessage),
                      kSchema,
                      64,
                      {.publisher = true}};
  EXPECT_EQ(restarted.write(BytesOf(Message::TestMessage{2, 2.0F})),
            WriteResult::SUCCESS);
  EXPECT_EQ(session.writer_sequence(), 2u);
  session.release_reader(*reader);
}

TEST(WrapTest, RTMSSequenceWrapsSlots) {
  const std::string path = make_test_path("wrap");

  constexpr std::size_t slots = 64;
  constexpr int iterations = 4 * static_cast<int>(slots);

  RTMSQueue queue{
      path,
      sizeof(Message::TestMessage),
      alignof(Message::TestMessage),
      kSchema,
      slots,
      sequence_options(),
  };

  const auto reader_id = queue.register_reader();
  ASSERT_TRUE(reader_id.has_value());
  const std::size_t id = reader_id.value_or(std::size_t{0});

  for (int i = 0; i < iterations; ++i) {
    const Message::TestMessage expected{
        i,
        static_cast<float>(i),
    };

    const auto rtms_message = BytesOf(expected);

    ASSERT_EQ(queue.write(rtms_message), WriteResult::SUCCESS);

    Message::TestMessage received{};

    ASSERT_TRUE(queue.read(id, [&received](std::span<const std::byte> bytes) {
      std::memcpy(&received, bytes.data(), sizeof(received));
    }));

    EXPECT_EQ(received, expected);
  }

  EXPECT_FALSE(queue.read(id, [](std::span<const std::byte>) {}));

  queue.release_reader(id);
}

TEST(PublishTime, ExplicitTimestampsSurviveSequenceAndLatestReads) {
  for (const ReadMode mode : {ReadMode::SEQUENCE, ReadMode::LATEST}) {
    auto options = sequence_options();
    options.read_mode = mode;
    const auto path = make_test_path(
        mode == ReadMode::SEQUENCE ? "stamp_seq" : "stamp_latest");
    RTMSQueue queue{path,
                    sizeof(Message::TestMessage),
                    alignof(Message::TestMessage),
                    kSchema,
                    4,
                    options};
    const auto reader = queue.register_reader();
    ASSERT_TRUE(reader.has_value());
    // Non-monotonic, negative, and zero values must be preserved verbatim.
    constexpr std::array<std::int64_t, 4> times{0, 9'876'543'210'123,
                                                -5'000'000'000, 42};
    for (int batch = 0; batch < 3; ++batch) {
      for (int i = 0; i < 4; ++i) {
        const Message::TestMessage message{batch * 4 + i,
                                           static_cast<float>(i)};
        ASSERT_EQ(queue.write(BytesOf(message), times[i]),
                  WriteResult::SUCCESS);
      }
      const int first = mode == ReadMode::LATEST ? 3 : 0;
      for (int i = first; i < 4; ++i) {
        Message::TestMessage received{};
        MessageInfo info{};
        ASSERT_EQ(
            queue.read_next(
                *reader, std::as_writable_bytes(std::span{&received, 1}), info),
            ReadResult::OK);
        EXPECT_EQ(received.id(), batch * 4 + i);
        EXPECT_EQ(received.value(), static_cast<float>(i));
        EXPECT_EQ(info.sequence, static_cast<std::uint64_t>(batch * 4 + i));
        EXPECT_EQ(info.dropped, mode == ReadMode::LATEST ? 3u : 0u);
        EXPECT_EQ(info.publish_time_ns, times[i]);
      }
    }
    queue.release_reader(*reader);
  }
}

TEST(PublishTime, ConvenienceWriteSamplesHostMonotonicClock) {
  RTMSQueue queue{make_test_path("stamp_host"), sizeof(Message::TestMessage),
                  alignof(Message::TestMessage), kSchema, 2};
  const auto reader = queue.register_reader();
  ASSERT_TRUE(reader.has_value());
  const auto now_ns = [] {
    struct timespec now {};
#if defined(__APPLE__)
    ::clock_gettime(CLOCK_MONOTONIC_RAW, &now);
#else
    ::clock_gettime(CLOCK_MONOTONIC, &now);
#endif
    return static_cast<std::int64_t>(now.tv_sec) * 1'000'000'000LL +
           now.tv_nsec;
  };
  const auto before = now_ns();
  ASSERT_EQ(queue.write(BytesOf(Message::TestMessage{7, 7.0F})),
            WriteResult::SUCCESS);
  const auto after = now_ns();
  Message::TestMessage received{};
  MessageInfo info{};
  ASSERT_EQ(queue.read_next(
                *reader, std::as_writable_bytes(std::span{&received, 1}), info),
            ReadResult::OK);
  EXPECT_GE(info.publish_time_ns, before);
  EXPECT_LE(info.publish_time_ns, after);
  queue.release_reader(*reader);
}

TEST(SingleProcess, ShortWritesZeroPadPreviouslyUsedSlots) {
  constexpr std::size_t slots = 4;
  std::array<std::byte, 16> full;
  full.fill(std::byte{0xa5});
  RTMSQueue queue{make_test_path("short_pad"), full.size(), 8, kSchema, slots,
                  overwrite_sequence_options()};
  for (std::size_t i = 0; i < slots; ++i) {
    ASSERT_EQ(queue.write(full), WriteResult::SUCCESS);
  }
  const auto reader = queue.register_reader();
  ASSERT_TRUE(reader.has_value());
  for (const std::size_t size : {std::size_t{3}, std::size_t{0}}) {
    for (std::size_t i = 0; i < slots; ++i) {
      ASSERT_EQ(queue.write(std::span<const std::byte>{full}.first(size)),
                WriteResult::SUCCESS);
      std::array<std::byte, 16> received;
      received.fill(std::byte{0xff});
      MessageInfo info;
      ASSERT_EQ(queue.read_next(*reader, received, info), ReadResult::OK);
      for (std::size_t k = 0; k < received.size(); ++k) {
        EXPECT_EQ(received[k], k < size ? std::byte{0xa5} : std::byte{0});
      }
    }
  }
  queue.release_reader(*reader);
}

TEST(SingleProcess, MismatchedReaderPolicyRejectsOverwrittenSequence) {
  const auto path = make_test_path("policy_mismatch");
  constexpr std::size_t slots = 4;
  RTMSQueue writer{path,
                   sizeof(Message::TestMessage),
                   alignof(Message::TestMessage),
                   kSchema,
                   slots,
                   overwrite_sequence_options()};
  RTMSQueue subscriber{path,
                       sizeof(Message::TestMessage),
                       alignof(Message::TestMessage),
                       kSchema,
                       slots,
                       sequence_options()};
  const auto reader = subscriber.register_reader();
  ASSERT_TRUE(reader.has_value());
  for (std::size_t i = 0; i <= slots; ++i) {
    ASSERT_EQ(writer.write(BytesOf(Message::TestMessage{static_cast<int>(i),
                                                        static_cast<float>(i)}),
                           1'000 + i),
              WriteResult::SUCCESS);
  }
  std::array<std::byte, sizeof(Message::TestMessage)> received{};
  MessageInfo info{.publish_time_ns = 123};
  // DROP_NEWEST readers do not skip sequences, but may not return bytes from
  // a different sequence when the writer actually uses OVERWRITE_OLDEST.
  EXPECT_EQ(subscriber.read_next(*reader, received, info), ReadResult::TORN);
  EXPECT_EQ(info.sequence, 0u);
  EXPECT_EQ(info.dropped, 0u);
  EXPECT_EQ(info.publish_time_ns, 0);
  subscriber.release_reader(*reader);
}

TEST(SingleProcess, InProgressOverwriteRejectsMixedSlot) {
  for (const std::size_t slots : {2u, 4u}) {
    const auto path = make_test_path("inprog_" + std::to_string(slots));
    std::array<std::byte, 16> original;
    original.fill(std::byte{0xa5});
    std::array<std::byte, 16> newer;
    newer.fill(std::byte{0x5a});
    RTMSQueue writer{path,    original.size(), 8,
                     kSchema, slots,           overwrite_sequence_options()};
    // A mismatched DROP_NEWEST reader must validate its original sequence,
    // rather than skipping it before the post-copy validity check is reached.
    RTMSQueue subscriber{path,    original.size(), 8,
                         kSchema, slots,           sequence_options()};
    RTMSQueue recovering{
        path, original.size(), 8, kSchema, slots, overwrite_sequence_options()};
    RTMSQueue latest{path,    original.size(), 8,
                     kSchema, slots,           latest_options()};
    const auto reader = subscriber.register_reader();
    const auto recovering_reader = recovering.register_reader();
    const auto latest_reader = latest.register_reader();
    ASSERT_TRUE(reader.has_value());
    ASSERT_TRUE(recovering_reader.has_value());
    ASSERT_TRUE(latest_reader.has_value());
    ASSERT_EQ(writer.write(original, 100), WriteResult::SUCCESS);
    for (std::size_t i = 1; i < slots; ++i) {
      ASSERT_EQ(writer.write(newer, 100 + i), WriteResult::SUCCESS);
    }

    SharedMemoryPtr header_mapping{path, sizeof(RTMSHeader)};
    auto* header = static_cast<RTMSHeader*>(header_mapping.ptr());
    ASSERT_NE(header, nullptr);
    SharedMemoryPtr mapping{
        path, header->data_offset + header->slots * header->slot_stride};
    auto* first_slot =
        static_cast<std::byte*>(mapping.ptr()) + header->data_offset;
    // Simulate a writer stopped halfway through sequence `slots`: sequence 0
    // now contains mixed bytes, but the published position is still `slots`.
    std::memcpy(first_slot, newer.data(), newer.size() / 2);
    const std::int64_t in_progress_time = 999'999;
    std::memcpy(first_slot + rtms_publish_time_offset(original.size()),
                &in_progress_time, sizeof(in_progress_time));
    ASSERT_EQ(writer.writer_sequence(), slots);
    EXPECT_EQ(first_slot[0], newer[0]);
    EXPECT_EQ(first_slot[original.size() - 1], original.back());

    std::array<std::byte, 16> received{};
    MessageInfo info{.publish_time_ns = 123};
    EXPECT_EQ(subscriber.read_next(*reader, received, info), ReadResult::TORN);
    EXPECT_EQ(subscriber.reader_sequence(*reader), 0u);
    EXPECT_EQ(info.sequence, 0u);
    EXPECT_EQ(info.dropped, 0u);
    EXPECT_EQ(info.publish_time_ns, 0);

    ASSERT_EQ(recovering.read_next(*recovering_reader, received, info),
              ReadResult::OK);
    EXPECT_EQ(info.sequence, 1u);
    EXPECT_EQ(info.dropped, 1u);
    EXPECT_EQ(info.publish_time_ns, 101);
    EXPECT_EQ(received, newer);
    ASSERT_EQ(latest.read_next(*latest_reader, received, info), ReadResult::OK);
    EXPECT_EQ(info.sequence, slots - 1);
    EXPECT_EQ(info.dropped, slots - 1);
    EXPECT_EQ(info.publish_time_ns, 100 + slots - 1);
    EXPECT_EQ(received, newer);
    subscriber.release_reader(*reader);
    recovering.release_reader(*recovering_reader);
    latest.release_reader(*latest_reader);
  }
}

TEST(SingleProcess, OverwriteRevokesReliableFullRingGuarantee) {
  const auto path = make_test_path("reliable_swap");
  RTMSQueue subscriber{path, 8, 8, kSchema, 2, sequence_options()};
  const auto reader = subscriber.register_reader();
  ASSERT_TRUE(reader.has_value());
  std::array<std::byte, 8> message{};
  SharedMemoryPtr mapping{path, sizeof(RTMSHeader)};
  auto* header = static_cast<RTMSHeader*>(mapping.ptr());
  ASSERT_NE(header, nullptr);
  {
    RTMSQueue reliable{path, message.size(), 8, kSchema, 2, sequence_options()};
    ASSERT_EQ(reliable.write(message), WriteResult::SUCCESS);
    ASSERT_EQ(reliable.write(message), WriteResult::SUCCESS);
    EXPECT_EQ(header->writer.reliable_sequence.load(), 2u);
    MessageInfo info;
    std::array<std::byte, 8> received{};
    ASSERT_EQ(subscriber.read_next(*reader, received, info), ReadResult::OK);
    EXPECT_EQ(info.sequence, 0u);
  }
  {
    RTMSQueue overwriting{
        path, message.size(), 8, kSchema, 2, overwrite_sequence_options()};
    ASSERT_EQ(overwriting.write(message), WriteResult::SUCCESS);
    EXPECT_EQ(header->writer.reliable_sequence.load(), 0u);
    MessageInfo info;
    std::array<std::byte, 8> received{};
    EXPECT_EQ(subscriber.read_next(*reader, received, info), ReadResult::TORN);
    EXPECT_EQ(subscriber.reader_sequence(*reader), 1u);
  }
  subscriber.release_reader(*reader);
}

TEST(SingleProcess, ReadNextEmptyInvalidAndInactiveCases) {
  const std::string path = make_test_path("rn_invalid");

  RTMSQueue queue{
      path,
      sizeof(Message::TestMessage),
      alignof(Message::TestMessage),
      kSchema,
      8,
      overwrite_sequence_options(),
  };

  const auto reader_id = queue.register_reader();
  ASSERT_TRUE(reader_id.has_value());
  const std::size_t id = reader_id.value_or(std::size_t{0});

  std::array<std::byte, sizeof(Message::TestMessage)> buffer{};
  MessageInfo info{};

  // Nothing published yet: EMPTY, not OK.
  EXPECT_EQ(queue.read_next(id, std::span<std::byte>(buffer), info),
            ReadResult::EMPTY);

  const Message::TestMessage message{7, 7.0F};
  const auto rtms_message = BytesOf(message);
  ASSERT_EQ(queue.write(rtms_message).result, WriteResult::SUCCESS);

  ASSERT_EQ(queue.read_next(id, std::span<std::byte>(buffer), info),
            ReadResult::OK);

  // Caught up again: EMPTY, and the same message must not repeat.
  EXPECT_EQ(queue.read_next(id, std::span<std::byte>(buffer), info),
            ReadResult::EMPTY);

  // Out-of-range reader ids are INVALID, not INACTIVE.
  EXPECT_EQ(queue.read_next(MAX_READERS, std::span<std::byte>(buffer), info),
            ReadResult::INVALID);
  EXPECT_EQ(
      queue.read_next(MAX_READERS + 100, std::span<std::byte>(buffer), info),
      ReadResult::INVALID);

  // A destination smaller than the message is INVALID.
  ASSERT_EQ(queue.write(rtms_message).result, WriteResult::SUCCESS);
  std::array<std::byte, sizeof(Message::TestMessage) - 1> small_buffer{};
  EXPECT_EQ(queue.read_next(id, std::span<std::byte>(small_buffer), info),
            ReadResult::INVALID);

  // A released reader is INACTIVE.
  queue.release_reader(id);
  EXPECT_EQ(queue.read_next(id, std::span<std::byte>(buffer), info),
            ReadResult::INACTIVE);
}

TEST(SingleProcess, ReadNextLapsUsingDocumentedFormula) {
  const std::string path = make_test_path("rn_lap");

  constexpr std::uint64_t slots = 8;
  constexpr std::uint64_t writes = slots * 5 + 3;

  RTMSQueue queue{
      path,
      sizeof(Message::TestMessage),
      alignof(Message::TestMessage),
      kSchema,
      slots,
      overwrite_sequence_options(),
  };

  // Register before publishing so the reader's cursor starts at 0 and the
  // entire run below laps it.
  const auto reader_id = queue.register_reader();
  ASSERT_TRUE(reader_id.has_value());
  const std::size_t id = reader_id.value_or(std::size_t{0});

  for (std::uint64_t i = 0; i < writes; ++i) {
    const Message::TestMessage message{
        static_cast<int>(i),
        static_cast<float>(i),
    };

    const auto rtms_message = BytesOf(message);

    ASSERT_EQ(queue.write(rtms_message).result, WriteResult::SUCCESS);
  }

  ASSERT_EQ(queue.writer_sequence(), writes);

  // Documented rule: on a lap the reader jumps to
  // writer_position - slots + 1, the oldest slot the writer is not
  // currently about to reuse. Derive the expectation from that rule
  // instead of hardcoding it.
  const std::uint64_t expected_first = queue.writer_sequence() - slots + 1;

  std::array<std::byte, sizeof(Message::TestMessage)> buffer{};
  MessageInfo info{};

  ASSERT_EQ(queue.read_next(id, std::span<std::byte>(buffer), info),
            ReadResult::OK);
  EXPECT_EQ(info.sequence, expected_first);
  EXPECT_EQ(info.dropped, expected_first);

  Message::TestMessage received{};
  std::memcpy(&received, buffer.data(), sizeof(received));
  EXPECT_EQ(received.id(), static_cast<int>(expected_first));

  std::uint64_t last_sequence = info.sequence;

  for (std::uint64_t expected = expected_first + 1; expected < writes;
       ++expected) {
    ASSERT_EQ(queue.read_next(id, std::span<std::byte>(buffer), info),
              ReadResult::OK);

    // Strictly increasing: the reader must never re-observe a sequence.
    EXPECT_GT(info.sequence, last_sequence);
    EXPECT_EQ(info.sequence, expected);
    EXPECT_EQ(info.dropped, 0u);

    last_sequence = info.sequence;
  }

  EXPECT_EQ(queue.read_next(id, std::span<std::byte>(buffer), info),
            ReadResult::EMPTY);

  queue.release_reader(id);
}

TEST(SingleProcess, ReadNextLatestReportsDroppedThenEmpty) {
  const std::string path = make_test_path("rn_latest");

  constexpr int writes = 5;

  RTMSQueue queue{
      path,
      sizeof(Message::TestMessage),
      alignof(Message::TestMessage),
      kSchema,
      64,
      latest_options(),
  };

  const auto reader_id = queue.register_reader();
  ASSERT_TRUE(reader_id.has_value());
  const std::size_t id = reader_id.value_or(std::size_t{0});

  for (int i = 0; i < writes; ++i) {
    const Message::TestMessage message{i, static_cast<float>(i)};

    const auto rtms_message = BytesOf(message);

    ASSERT_EQ(queue.write(rtms_message).result, WriteResult::SUCCESS);
  }

  std::array<std::byte, sizeof(Message::TestMessage)> buffer{};
  MessageInfo info{};

  ASSERT_EQ(queue.read_next(id, std::span<std::byte>(buffer), info),
            ReadResult::OK);

  EXPECT_EQ(info.sequence, static_cast<std::uint64_t>(writes - 1));
  EXPECT_EQ(info.dropped, static_cast<std::uint64_t>(writes - 1));

  Message::TestMessage received{};
  std::memcpy(&received, buffer.data(), sizeof(received));
  EXPECT_EQ(received.id(), writes - 1);

  // No intervening write: a second read must be EMPTY, not a repeat of
  // the latest message.
  EXPECT_EQ(queue.read_next(id, std::span<std::byte>(buffer), info),
            ReadResult::EMPTY);

  queue.release_reader(id);
}

TEST(SingleProcess, DropNewestBufferFullThenRecovers) {
  const std::string path = make_test_path("dn_recover");

  constexpr std::uint64_t slots = 8;

  RTMSQueue queue{
      path,
      sizeof(Message::TestMessage),
      alignof(Message::TestMessage),
      kSchema,
      slots,
      sequence_options(),
  };

  const auto reader_id = queue.register_reader();
  ASSERT_TRUE(reader_id.has_value());
  const std::size_t id = reader_id.value_or(std::size_t{0});

  // Fill every slot without reading; the reader is fully stalled.
  for (std::uint64_t i = 0; i < slots; ++i) {
    const Message::TestMessage message{
        static_cast<int>(i),
        static_cast<float>(i),
    };

    const auto rtms_message = BytesOf(message);

    ASSERT_EQ(queue.write(rtms_message).result, WriteResult::SUCCESS);
  }

  const Message::TestMessage overflow_message{999, 999.0F};
  const auto overflow_rtms_message = BytesOf(overflow_message);

  EXPECT_EQ(queue.write(overflow_rtms_message).result,
            WriteResult::BUFFER_FULL);

  // Drain the stalled reader completely. Every message must come back, in
  // order, with nothing dropped: a full buffer under this policy means the
  // writer stopped, not that anything was lost.
  std::array<std::byte, sizeof(Message::TestMessage)> buffer{};
  MessageInfo info{};
  std::uint64_t drained = 0;

  while (queue.read_next(id, std::span<std::byte>(buffer), info) ==
         ReadResult::OK) {
    EXPECT_EQ(info.sequence, drained);
    EXPECT_EQ(info.dropped, 0u);

    Message::TestMessage received{};
    std::memcpy(&received, buffer.data(), sizeof(received));
    EXPECT_EQ(received.id(), static_cast<int>(drained));

    ++drained;
    ASSERT_LE(drained, slots);  // safety net against an infinite loop
  }

  EXPECT_EQ(drained, slots);

  // Writes must succeed again now that the slowest reader has caught up.
  const Message::TestMessage recovered_message{42, 42.0F};
  const auto recovered_rtms_message = BytesOf(recovered_message);

  const WriteStatus recovered_status = queue.write(recovered_rtms_message);

  EXPECT_EQ(recovered_status.result, WriteResult::SUCCESS);
  EXPECT_EQ(recovered_status.sequence, slots);

  queue.release_reader(id);
}

namespace {

void WriteThread(const std::string& path, int iterations,
                 const std::shared_future<void>& start_signal,
                 std::atomic<bool>& failed, std::atomic<bool>& writer_done) {
  try {
    RTMSQueue queue{
        path,
        sizeof(Message::TestMessage),
        alignof(Message::TestMessage),
        kSchema,
        64,
        sequence_options(),
    };

    start_signal.wait();

    const auto deadline = std::chrono::steady_clock::now() + kTestTimeout;

    for (int sequence = 0; sequence < iterations && !failed.load();
         ++sequence) {
      const Message::TestMessage message{
          sequence,
          static_cast<float>(sequence),
      };

      const auto rtms_message = BytesOf(message);

      for (;;) {
        const WriteResult result = queue.write(rtms_message).result;

        if (result == WriteResult::SUCCESS) {
          break;
        }

        // In reliable sequence mode, retry until readers make room.
        if (result != WriteResult::BUFFER_FULL) {
          failed.store(true);
          writer_done.store(true);
          return;
        }

        if (std::chrono::steady_clock::now() >= deadline) {
          failed.store(true);
          writer_done.store(true);
          return;
        }

        std::this_thread::yield();
      }
    }
  } catch (...) {
    failed.store(true);
  }

  writer_done.store(true);
}

void ReadThread(const std::string& path, int iterations,
                std::latch& readers_ready,
                const std::shared_future<void>& start_signal,
                std::atomic<bool>& failed,
                const std::atomic<bool>& writer_done) {
  std::optional<std::size_t> reader_id;

  try {
    RTMSQueue queue{
        path,
        sizeof(Message::TestMessage),
        alignof(Message::TestMessage),
        kSchema,
        64,
        sequence_options(),
    };

    reader_id = queue.register_reader();

    if (!reader_id.has_value()) {
      failed.store(true);
      readers_ready.count_down();
      return;
    }

    // The writer cannot begin until every reader has a cursor.
    readers_ready.count_down();
    start_signal.wait();

    const auto deadline = std::chrono::steady_clock::now() + kTestTimeout;

    int received_count = 0;

    while (received_count < iterations && !failed.load()) {
      Message::TestMessage received{};

      const bool did_read =
          queue.read(*reader_id, [&received](std::span<const std::byte> bytes) {
            std::memcpy(&received, bytes.data(), sizeof(received));
          });

      if (did_read) {
        const int expected_sequence = received_count;

        if (received.id() != expected_sequence ||
            received.value() != static_cast<float>(expected_sequence)) {
          failed.store(true);
          break;
        }

        ++received_count;
        continue;
      }

      if (std::chrono::steady_clock::now() >= deadline) {
        failed.store(true);
        break;
      }

      // read() returning false only means this reader is currently
      // caught up. The writer may publish another message afterward.
      if (writer_done.load() && received_count < iterations) {
        // Give the final writer release-store one more opportunity
        // to become visible before declaring failure.
        std::this_thread::yield();

        if (!queue.read(
                *reader_id, [&received](std::span<const std::byte> bytes) {
                  std::memcpy(&received, bytes.data(), sizeof(received));
                })) {
          failed.store(true);
          break;
        }

        const int expected_sequence = received_count;

        if (received.id() != expected_sequence ||
            received.value() != static_cast<float>(expected_sequence)) {
          failed.store(true);
          break;
        }

        ++received_count;
        continue;
      }

      std::this_thread::yield();
    }

    if (received_count != iterations) {
      failed.store(true);
    }

    queue.release_reader(*reader_id);
  } catch (...) {
    // Ensure the main thread does not wait forever if construction or
    // reader registration throws before the latch is decremented.
    failed.store(true);

    if (!reader_id.has_value()) {
      readers_ready.count_down();
    }
  }
}

}  // namespace

TEST(MultiReader, EveryReaderReceivesEveryMessage) {
  const std::string path = make_test_path("multi_reader");

  constexpr int iterations = 640;
  constexpr int num_readers = 4;

  // Create the shared-memory object before launching attaching threads.
  // The owner remains alive for the entire test.
  //
  RTMSQueue owner{
      path,
      sizeof(Message::TestMessage),
      alignof(Message::TestMessage),
      kSchema,
      64,
      sequence_options(),
  };

  std::latch readers_ready{num_readers};

  std::promise<void> start_promise;
  std::shared_future<void> start_signal = start_promise.get_future().share();

  std::atomic<bool> failed{false};
  std::atomic<bool> writer_done{false};

  std::thread writer{
      WriteThread,           path, iterations, start_signal, std::ref(failed),
      std::ref(writer_done),
  };

  std::array<std::thread, num_readers> readers;

  for (auto& reader : readers) {
    reader = std::thread{
        ReadThread,
        path,
        iterations,
        std::ref(readers_ready),
        start_signal,
        std::ref(failed),
        std::cref(writer_done),
    };
  }

  // All reader slots must be ACTIVE before publication starts.
  readers_ready.wait();
  start_promise.set_value();

  writer.join();

  for (auto& reader : readers) {
    reader.join();
  }

  EXPECT_FALSE(failed.load());
}

namespace {

// A writer that never throttles itself and a small slot count, so that
// within the test's deadline the writer reliably laps the reader(s) many
// times over. Uses OVERWRITE_OLDEST + SEQUENCE so read_next() reports the
// laps via MessageInfo::dropped instead of silently losing them.

void FreeRunningWriteThread(const std::string& path, std::size_t slots,
                            const std::shared_future<void>& start_signal,
                            std::chrono::steady_clock::time_point deadline,
                            std::atomic<bool>& failed) {
  try {
    RTMSQueue queue{
        path,
        sizeof(Message::TestMessage),
        alignof(Message::TestMessage),
        kSchema,
        slots,
        overwrite_sequence_options(),
    };

    start_signal.wait();

    int sequence = 0;

    while (std::chrono::steady_clock::now() < deadline) {
      const Message::TestMessage message{
          sequence,
          static_cast<float>(sequence),
      };

      const auto rtms_message = BytesOf(message);

      // OVERWRITE_OLDEST never blocks the writer. Anything but
      // SUCCESS here is a real bug: sizes match exactly, so
      // ERROR_SIZE_MISSMATCH cannot legitimately happen either.
      const std::int64_t publish_time_ns =
          static_cast<std::int64_t>(sequence) * 1'000 + 17;
      if (queue.write(rtms_message, publish_time_ns).result !=
          WriteResult::SUCCESS) {
        failed.store(true);
        return;
      }

      ++sequence;
    }
  } catch (...) {
    failed.store(true);
  }
}

void FreeRunningReadThread(const std::string& path, std::size_t slots,
                           const std::shared_future<void>& start_signal,
                           std::chrono::steady_clock::time_point deadline,
                           std::atomic<bool>& failed,
                           std::atomic<std::uint64_t>& total_dropped,
                           std::atomic<std::uint64_t>& messages_read,
                           std::latch& readers_ready) {
  std::optional<std::size_t> reader_id;

  try {
    RTMSQueue queue{
        path,
        sizeof(Message::TestMessage),
        alignof(Message::TestMessage),
        kSchema,
        slots,
        overwrite_sequence_options(),
    };

    reader_id = queue.register_reader();

    if (!reader_id.has_value()) {
      failed.store(true);
      readers_ready.count_down();
      return;
    }

    readers_ready.count_down();
    start_signal.wait();

    std::array<std::byte, sizeof(Message::TestMessage)> buffer{};
    std::optional<std::uint64_t> last_sequence;

    while (std::chrono::steady_clock::now() < deadline) {
      MessageInfo info{};
      const ReadResult result =
          queue.read_next(*reader_id, std::span<std::byte>(buffer), info);

      if (result == ReadResult::EMPTY) {
        continue;
      }

      if (result == ReadResult::TORN) {
        // Documented as recoverable: the writer lapped us mid-copy
        // more than MAX_READ_ATTEMPTS times in a row. Retry.
        continue;
      }

      if (result != ReadResult::OK) {
        failed.store(true);
        break;
      }

      Message::TestMessage received{};
      std::memcpy(&received, buffer.data(), sizeof(received));

      // Payload, sequence, and publication time must all belong to one slot
      // generation, even when the writer laps us during the copy.
      if (received.id() != static_cast<int>(received.value()) ||
          info.sequence != static_cast<std::uint64_t>(received.id()) ||
          info.publish_time_ns !=
              static_cast<std::int64_t>(received.id()) * 1'000 + 17) {
        failed.store(true);
        break;
      }

      if (last_sequence.has_value() && info.sequence <= *last_sequence) {
        failed.store(true);
        break;
      }

      last_sequence = info.sequence;
      total_dropped.fetch_add(info.dropped);
      messages_read.fetch_add(1);
    }

    queue.release_reader(*reader_id);
  } catch (...) {
    failed.store(true);

    if (!reader_id.has_value()) {
      readers_ready.count_down();
    }
  }
}

}  // namespace

TEST(Concurrency, OverwriteOldestNoTornReadsMultiReader) {
  const std::string path = make_test_path("conc_multi");

  constexpr std::size_t slots = 8;
  constexpr int num_readers = 3;

  RTMSQueue owner{
      path,
      sizeof(Message::TestMessage),
      alignof(Message::TestMessage),
      kSchema,
      slots,
      overwrite_sequence_options(),
  };

  std::latch readers_ready{num_readers};

  std::promise<void> start_promise;
  std::shared_future<void> start_signal = start_promise.get_future().share();

  std::atomic<bool> failed{false};
  std::array<std::atomic<std::uint64_t>, num_readers> dropped_per_reader{};
  std::array<std::atomic<std::uint64_t>, num_readers> read_per_reader{};

  const auto deadline = std::chrono::steady_clock::now() + 400ms;

  std::array<std::thread, num_readers> readers;

  for (int i = 0; i < num_readers; ++i) {
    readers[i] = std::thread{
        FreeRunningReadThread,
        path,
        slots,
        start_signal,
        deadline,
        std::ref(failed),
        std::ref(dropped_per_reader[i]),
        std::ref(read_per_reader[i]),
        std::ref(readers_ready),
    };
  }

  readers_ready.wait();

  std::thread writer{
      FreeRunningWriteThread, path,     slots,
      start_signal,           deadline, std::ref(failed),
  };

  start_promise.set_value();

  writer.join();

  for (auto& reader : readers) {
    reader.join();
  }

  EXPECT_FALSE(failed.load());

  std::uint64_t total_dropped = 0;

  for (int i = 0; i < num_readers; ++i) {
    // Each reader independently verifies id/value consistency and
    // sequence monotonicity inside FreeRunningReadThread; here we just
    // confirm every reader actually did work.
    EXPECT_GT(read_per_reader[i].load(), 0u);
    total_dropped += dropped_per_reader[i].load();
  }

  EXPECT_GT(total_dropped, 0u);
}

TEST(RTMSQueue, RejectsTopicNamesLongerThan30CharsAfterLeadingSlash) {
  // 31 characters after leading slash
  const std::string too_long = "/1234567890123456789012345678901";
  EXPECT_THROW(RTMSQueue(too_long, sizeof(Message::TestMessage),
                         alignof(Message::TestMessage), kSchema),
               std::invalid_argument);

  // Exactly 30 characters after leading slash should not throw invalid_argument
  const std::string exactly_30 =
      make_test_path("123456789012345678901234567890");
  const std::string capped_30 = exactly_30.substr(0, 31);  // '/' + 30 chars
  EXPECT_NO_THROW({
    RTMSQueue q(capped_30, sizeof(Message::TestMessage),
                alignof(Message::TestMessage), kSchema);
  });
}

// A process killed before it could unlink leaves its segment behind. When the
// next build changes a message's size, that leftover has the wrong shape and
// nothing can attach to it, so the publisher -- which owns the topic's layout
// -- clears it instead of every node failing to start until someone unlinks it
// by hand.
namespace {

RTMSOptions publisher_options() {
  RTMSOptions options = sequence_options();
  options.publisher = true;
  return options;
}

// Leaves a segment of `bytes` behind at `path` with no live owner, the way a
// SIGKILLed process does.
void leak_segment(const std::string& path, std::size_t bytes) {
  const int fd = shm_open(ShmObjectName(path).c_str(), O_CREAT | O_RDWR, 0666);
  ASSERT_NE(fd, -1);
  ASSERT_EQ(ftruncate(fd, static_cast<off_t>(bytes)), 0);
  ASSERT_EQ(close(fd), 0);
}

bool SegmentExists(const std::string& path) {
  const int fd = shm_open(ShmObjectName(path).c_str(), O_RDONLY, 0);
  if (fd == -1) return false;
  close(fd);
  return true;
}

}  // namespace

// The publisher reclaims the leftover; a reader that disagrees with it is a
// version skew against a possibly live publisher, so it must refuse rather
// than delete the topic out from under that publisher.
TEST(RTMSStaleSegment, PublisherReclaimsALeftoverOfTheWrongShapeReaderDoesNot) {
  const std::string path = make_test_path("stale_reclaim");
  shm_unlink(ShmObjectName(path).c_str());
  leak_segment(path, 4096);

  EXPECT_THROW(
      RTMSQueue(path, sizeof(Message::TestMessage),
                alignof(Message::TestMessage), kSchema, 64, sequence_options()),
      std::runtime_error);
  // The segment is still there for the owner to deal with.
  EXPECT_TRUE(SegmentExists(path));

  RTMSQueue queue{path,
                  sizeof(Message::TestMessage),
                  alignof(Message::TestMessage),
                  kSchema,
                  64,
                  publisher_options()};
  const Message::TestMessage message{7, 1.5F};
  EXPECT_EQ(WriteResult(queue.write(BytesOf(message))), WriteResult::SUCCESS);
  shm_unlink(ShmObjectName(path).c_str());
}

TEST(RTMSStaleSegment, PublisherLeavesAHealthySegmentAloneUnderALiveReader) {
  const std::string path = make_test_path("stale_healthy");
  shm_unlink(ShmObjectName(path).c_str());

  // A reader gets there first and takes a slot. The publisher that follows
  // has an identical layout, so there is nothing to reclaim -- and unlinking
  // here would strand this reader on memory no one else can reach. The shm
  // object is page-rounded, so its measured size is larger than the layout
  // asked for: only the recorded header can decide whether it matches.
  RTMSQueue reader{path,
                   sizeof(Message::TestMessage),
                   alignof(Message::TestMessage),
                   kSchema,
                   64,
                   sequence_options()};
  const auto reader_id = reader.register_reader();
  ASSERT_TRUE(reader_id.has_value());
  const std::size_t id = reader_id.value_or(std::size_t{0});

  RTMSQueue writer{path,
                   sizeof(Message::TestMessage),
                   alignof(Message::TestMessage),
                   kSchema,
                   64,
                   publisher_options()};
  const Message::TestMessage sent{42, 3.25F};
  ASSERT_EQ(WriteResult(writer.write(BytesOf(sent))), WriteResult::SUCCESS);

  Message::TestMessage received{};
  MessageInfo info{};
  ASSERT_EQ(reader.read_next(
                id, {reinterpret_cast<std::byte*>(&received), sizeof(received)},
                info),
            ReadResult::OK);
  EXPECT_EQ(received.id(), sent.id());
  shm_unlink(ShmObjectName(path).c_str());
}

// --- schema identity
// ----------------------------------------------------------

namespace {

std::vector<std::uint8_t> VariantBfbs(const char* env) {
  std::string error;
  std::unique_ptr<rules_cc::cc::runfiles::Runfiles> runfiles(
      rules_cc::cc::runfiles::Runfiles::CreateForTest(BAZEL_CURRENT_REPOSITORY,
                                                      &error));
  const char* location = std::getenv(env);
  if (!runfiles || location == nullptr) {
    throw std::runtime_error(std::string{"runfile unavailable: "} + env + " " +
                             error);
  }
  std::ifstream in(runfiles->Rlocation(location), std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

// schema_variant.Reading as one of //talOS/schema:variants compiled it.
talos::schema::Identity Reading(const char* env) {
  const std::vector<std::uint8_t> bfbs = VariantBfbs(env);
  const auto hash = talos::schema::struct_hash(bfbs.data(), bfbs.size(),
                                               "schema_variant.Reading");
  if (!hash) throw std::runtime_error("no schema_variant.Reading");
  return {"schema_variant.Reading", *hash,
          std::string{bfbs.begin(), bfbs.end()}};
}

}  // namespace

// The D2 gate: a field whose type changed while its size did not (level_m
// float -> int, both 4 bytes) must not attach, however equal the layouts are.
TEST(RTMSSchema, AChangedFieldTypeOfTheSameSizeIsRejectedAtAttach) {
  const talos::schema::Identity before = Reading("TALOS_VARIANT_FLOAT_LEVEL");
  const talos::schema::Identity after = Reading("TALOS_VARIANT_INT_LEVEL");
  ASSERT_NE(before.hash, after.hash);
  const std::string path = make_test_path("schema_changed");
  shm_unlink(ShmObjectName(path).c_str());

  constexpr std::size_t kBytes = 8;  // float or int, then two bytes, padded
  RTMSQueue publisher{path, kBytes, 4, before, 16, publisher_options()};
  EXPECT_THROW((RTMSQueue{path, kBytes, 4, after, 16, sequence_options()}),
               std::runtime_error);
  // The live publisher holds the segment, so a second publisher of the new
  // schema is refused as well rather than reclaiming it.
  EXPECT_THROW(RTMSQueue(path, kBytes, 4, after, 16, publisher_options()),
               std::runtime_error);
  // The same schema attaches.
  RTMSQueue same{path, kBytes, 4, before, 16, sequence_options()};
  EXPECT_EQ(same.schema_hash(), before.hash);
  EXPECT_EQ(same.schema_name(), "schema_variant.Reading");
}

TEST(RTMSSchema, ANameMismatchIsRejectedEvenWithTheSameHash) {
  const std::string path = make_test_path("schema_name");
  shm_unlink(ShmObjectName(path).c_str());
  RTMSQueue publisher{path,
                      sizeof(Message::TestMessage),
                      alignof(Message::TestMessage),
                      kSchema,
                      16,
                      publisher_options()};
  EXPECT_THROW(
      RTMSQueue(path, sizeof(Message::TestMessage),
                alignof(Message::TestMessage),
                {"Message.Renamed", kSchema.hash}, 16, sequence_options()),
      std::runtime_error);
}

// --- segment lifetime
// -----------------------------------------------------------

namespace {

// Runs `body` in a child process that then dies by SIGKILL, with whatever it
// opened still held: the way a crashed node leaves shared memory.
template <class Body>
void RunInChildThenKill(Body body) {
  int ready[2];
  ASSERT_EQ(pipe(ready), 0);
  const pid_t child = fork();
  ASSERT_GE(child, 0);
  if (child == 0) {
    close(ready[0]);
    body();
    const char done = 1;
    (void)!write(ready[1], &done, 1);
    for (;;) pause();
  }
  close(ready[1]);
  char done = 0;
  ASSERT_EQ(read(ready[0], &done, 1), 1)
      << "the child failed before signalling";
  close(ready[0]);
  ASSERT_EQ(kill(child, SIGKILL), 0);
  int status = 0;
  ASSERT_EQ(waitpid(child, &status, 0), child);
}

Message::TestMessage ReadOne(RTMSQueue& queue, std::size_t reader) {
  Message::TestMessage received{};
  MessageInfo info{};
  EXPECT_EQ(
      queue.read_next(
          reader, {reinterpret_cast<std::byte*>(&received), sizeof(received)},
          info),
      ReadResult::OK);
  return received;
}

}  // namespace

// HotPath C8: a publisher that exits cleanly must not take its topic with it.
TEST(RTMSLifetime, APublishersCleanExitDoesNotSeverItsSubscriber) {
  const std::string path = make_test_path("clean_exit");
  shm_unlink(ShmObjectName(path).c_str());
  RTMSQueue subscriber{path,
                       sizeof(Message::TestMessage),
                       alignof(Message::TestMessage),
                       kSchema,
                       16,
                       overwrite_sequence_options()};
  const auto reader = subscriber.register_reader();
  if (!reader.has_value()) {
    FAIL() << "Expected registered reader";
  }
  const auto& reader_id = reader.value();
  {
    RTMSQueue publisher{path,
                        sizeof(Message::TestMessage),
                        alignof(Message::TestMessage),
                        kSchema,
                        16,
                        publisher_options()};
    ASSERT_EQ(publisher.write(BytesOf({1, 1.0F})), WriteResult::SUCCESS);
  }
  EXPECT_TRUE(SegmentExists(path));
  EXPECT_EQ(ReadOne(subscriber, reader_id).id(), 1);
  // A restarted publisher reaches the same subscriber, continuing the
  // sequence where the first one stopped.
  RTMSQueue restarted{path,
                      sizeof(Message::TestMessage),
                      alignof(Message::TestMessage),
                      kSchema,
                      16,
                      publisher_options()};
  ASSERT_EQ(restarted.write(BytesOf({2, 2.0F})), WriteResult::SUCCESS);
  EXPECT_EQ(ReadOne(subscriber, reader_id).id(), 2);
  subscriber.release_reader(reader_id);
}

TEST(RTMSLifetime, TheLastQueueToCloseAStandaloneSegmentUnlinksIt) {
  const std::string path = make_test_path("last_close");
  shm_unlink(ShmObjectName(path).c_str());
  {
    RTMSQueue first{path,
                    sizeof(Message::TestMessage),
                    alignof(Message::TestMessage),
                    kSchema,
                    16,
                    publisher_options()};
    {
      RTMSQueue second{path,
                       sizeof(Message::TestMessage),
                       alignof(Message::TestMessage),
                       kSchema,
                       16,
                       sequence_options()};
    }
    EXPECT_TRUE(SegmentExists(path));
  }
  EXPECT_FALSE(SegmentExists(path));
}

TEST(RTMSLifetime, ASessionOwnedSegmentOutlivesEveryNodeUntilTheSessionEnds) {
  const std::string path = make_test_path("session");
  shm_unlink(ShmObjectName(path).c_str());
  // A leftover of the wrong shape is replaced, not attached to.
  {
    RTMSQueue leftover{path, 32, 8, {"Message.Old", 1}, 4, publisher_options()};
    RTMSOptions session = overwrite_sequence_options();
    session.session_owner = true;
    RTMSQueue owner{path,
                    sizeof(Message::TestMessage),
                    alignof(Message::TestMessage),
                    kSchema,
                    16,
                    session};
    EXPECT_TRUE(owner.session_owned());
    {
      RTMSQueue node{path,
                     sizeof(Message::TestMessage),
                     alignof(Message::TestMessage),
                     kSchema,
                     16,
                     publisher_options()};
      EXPECT_TRUE(node.session_owned());
      ASSERT_EQ(node.write(BytesOf({5, 5.0F})), WriteResult::SUCCESS);
    }
    // Every node closed; the session still holds the topic and its data.
    EXPECT_TRUE(SegmentExists(path));
    RTMSQueue again{path,
                    sizeof(Message::TestMessage),
                    alignof(Message::TestMessage),
                    kSchema,
                    16,
                    sequence_options()};
    EXPECT_EQ(again.writer_sequence(), 1u);
  }
  EXPECT_FALSE(SegmentExists(path));
}

// --- dead readers
// ---------------------------------------------------------------

// A subscriber killed while holding its reader slot would hold a reliable
// (drop-newest) writer back forever.
TEST(RTMSDeadReader, AKilledReaderNoLongerHoldsADropNewestWriterBack) {
  const std::string path = make_test_path("dead_reader");
  shm_unlink(ShmObjectName(path).c_str());
  constexpr std::size_t kSlots = 4;
  RTMSQueue publisher{path,
                      sizeof(Message::TestMessage),
                      alignof(Message::TestMessage),
                      kSchema,
                      kSlots,
                      publisher_options()};
  RunInChildThenKill([&path] {
    // Leaked on purpose: the child dies holding the slot.
    auto* queue = new RTMSQueue{path,
                                sizeof(Message::TestMessage),
                                alignof(Message::TestMessage),
                                kSchema,
                                kSlots,
                                sequence_options()};
    if (!queue->register_reader()) _exit(1);
  });
  // With the dead reader at sequence 0 the writer may fill the ring and no
  // further; its slot is then freed and writing continues.
  for (int i = 0; i < 3 * static_cast<int>(kSlots); ++i) {
    EXPECT_EQ(publisher.write(BytesOf({i, 0.0F})), WriteResult::SUCCESS) << i;
  }
}

TEST(RTMSDeadReader, SlotsOfKilledReadersAreReclaimedWhenNoneAreFree) {
  const std::string path = make_test_path("dead_slots");
  shm_unlink(ShmObjectName(path).c_str());
  RTMSQueue publisher{path,
                      sizeof(Message::TestMessage),
                      alignof(Message::TestMessage),
                      kSchema,
                      16,
                      publisher_options()};
  RunInChildThenKill([&path] {
    auto* queue = new RTMSQueue{path,
                                sizeof(Message::TestMessage),
                                alignof(Message::TestMessage),
                                kSchema,
                                16,
                                overwrite_sequence_options()};
    for (std::size_t i = 0; i < MAX_READERS; ++i) {
      if (!queue->register_reader()) _exit(1);
    }
  });
  const auto reader = publisher.register_reader();
  if (!reader.has_value()) {
    FAIL() << "Expected registered reader";
  }
  const auto& reader_id = reader.value();
  ASSERT_EQ(publisher.write(BytesOf({9, 9.0F})), WriteResult::SUCCESS);
  EXPECT_EQ(ReadOne(publisher, reader_id).id(), 9);
  // A live reader is never reclaimed.
  EXPECT_EQ(publisher.reclaim_dead_readers(), 0u);
  publisher.release_reader(reader_id);
}

// A publisher reclaims a segment of another schema that only dead processes
// held, rather than every later run failing until someone unlinks it.
TEST(RTMSDeadReader,
     APublisherReclaimsAnotherSchemasSegmentNoLiveProcessHolds) {
  const std::string path = make_test_path("dead_schema");
  shm_unlink(ShmObjectName(path).c_str());
  RunInChildThenKill([&path] {
    auto* queue = new RTMSQueue{path,
                                sizeof(Message::TestMessage),
                                alignof(Message::TestMessage),
                                {"Message.Old", 7},
                                16,
                                publisher_options()};
    if (!queue->register_reader()) _exit(1);
  });
  RTMSQueue publisher{path,
                      sizeof(Message::TestMessage),
                      alignof(Message::TestMessage),
                      kSchema,
                      16,
                      publisher_options()};
  EXPECT_EQ(publisher.schema_name(), "Message.TestMessage");
}
