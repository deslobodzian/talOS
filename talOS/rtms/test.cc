#include <fcntl.h>
#include <flatbuffers/flatbuffers.h>
#include <gtest/gtest.h>
#include <sys/mman.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstring>
#include <future>
#include <latch>
#include <string>
#include <string_view>
#include <thread>

#include "rtms.h"
#include "talOS/rtms/test_message_generated.h"

namespace {

using namespace std::chrono_literals;

constexpr auto kTestTimeout = 5s;

std::string make_test_path(std::string_view test_name) {
#if defined(__linux__)
    return "/rtms_" +
           std::string{test_name} +
           "_" +
           std::to_string(::getpid());
#else
    return "/tmp/rtms/" +
           std::string{test_name} +
           "_" +
           std::to_string(::getpid());
#endif
}

RTMSOptions sequence_options() {
    return RTMSOptions{
        .overflow_policy = OverflowPolicy::DROP_NEWEST,
        .read_mode = ReadMode::SEQUENCE,
    };
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

TEST(SingleProcess, RTMSSequence) {
    const std::string path = make_test_path("single_sequence");

    RTMSQueue queue{
        path,
        sizeof(Message::TestMessage),
        alignof(Message::TestMessage),
        64,
        sequence_options(),
    };

    // Readers must be registered before messages are published.
    const auto reader_id = queue.register_reader();
    ASSERT_TRUE(reader_id.has_value());

    const auto messages = std::array{
        Message::TestMessage{10, 200.0F},
        Message::TestMessage{23, 23.2F},
        Message::TestMessage{0, 0.111111F},
    };

    for (const auto& message : messages) {
        const RTMSMessage rtms_message{
            .size = sizeof(message),
            .data = &message,
        };

        ASSERT_EQ(
            queue.write(rtms_message),
            WriteResult::SUCCESS);
    }

    for (const auto& expected : messages) {
        Message::TestMessage received{};

        const bool did_read = queue.read(
            *reader_id,
            [&received](std::span<const std::byte> bytes) {
                std::memcpy(
                    &received,
                    bytes.data(),
                    sizeof(received));
            });

        ASSERT_TRUE(did_read);
        EXPECT_EQ(received, expected);
    }

    // The reader cursor should now equal writer.sequence.
    EXPECT_FALSE(queue.read(
        *reader_id,
        [](std::span<const std::byte>) {}));

    queue.release_reader(*reader_id);
}

TEST(SingleProcess, RTMSLatest) {
    const std::string path = make_test_path("single_latest");

    RTMSQueue queue{
        path,
        sizeof(Message::TestMessage),
        alignof(Message::TestMessage),
        64,
        latest_options(),
    };

    const auto reader_id = queue.register_reader();
    ASSERT_TRUE(reader_id.has_value());

    const auto messages = std::array{
        Message::TestMessage{10, 200.0F},
        Message::TestMessage{23, 23.2F},
        Message::TestMessage{99, 42.0F},
    };

    for (const auto& message : messages) {
        const RTMSMessage rtms_message{
            .size = sizeof(message),
            .data = &message,
        };

        ASSERT_EQ(
            queue.write(rtms_message),
            WriteResult::SUCCESS);
    }

    Message::TestMessage received{};

    ASSERT_TRUE(queue.read(
        *reader_id,
        [&received](std::span<const std::byte> bytes) {
            std::memcpy(
                &received,
                bytes.data(),
                sizeof(received));
        }));

    // LATEST skips the first two unread messages.
    EXPECT_EQ(received, messages.back());

    // The reader cursor should now equal writer.sequence.
    // The latest message must not be returned repeatedly.
    EXPECT_FALSE(queue.read(
        *reader_id,
        [](std::span<const std::byte>) {}));

    queue.release_reader(*reader_id);
}

TEST(WrapTest, RTMSSequenceWrapsSlots) {
    const std::string path = make_test_path("wrap");

    constexpr std::size_t slots = 64;
    constexpr int iterations = 4 * static_cast<int>(slots);

    RTMSQueue queue{
        path,
        sizeof(Message::TestMessage),
        alignof(Message::TestMessage),
        slots,
        sequence_options(),
    };

    const auto reader_id = queue.register_reader();
    ASSERT_TRUE(reader_id.has_value());

    for (int i = 0; i < iterations; ++i) {
        const Message::TestMessage expected{
            i,
            static_cast<float>(i),
        };

        const RTMSMessage rtms_message{
            .size = sizeof(expected),
            .data = &expected,
        };

        ASSERT_EQ(
            queue.write(rtms_message),
            WriteResult::SUCCESS);

        Message::TestMessage received{};

        ASSERT_TRUE(queue.read(
            *reader_id,
            [&received](std::span<const std::byte> bytes) {
                std::memcpy(
                    &received,
                    bytes.data(),
                    sizeof(received));
            }));

        EXPECT_EQ(received, expected);
    }

    EXPECT_FALSE(queue.read(
        *reader_id,
        [](std::span<const std::byte>) {}));

    queue.release_reader(*reader_id);
}

TEST(SingleProcess, WriteSequenceIncrementsMonotonically) {
    const std::string path = make_test_path("write_sequence");

    constexpr std::size_t slots = 64;
    constexpr int writes = 10;

    RTMSQueue queue{
        path,
        sizeof(Message::TestMessage),
        alignof(Message::TestMessage),
        slots,
        sequence_options(),
    };

    for (int i = 0; i < writes; ++i) {
        const Message::TestMessage message{i, static_cast<float>(i)};

        const RTMSMessage rtms_message{
            .size = sizeof(message),
            .data = &message,
        };

        const WriteStatus status = queue.write(rtms_message);

        ASSERT_EQ(status.result, WriteResult::SUCCESS);
        EXPECT_EQ(status.sequence, static_cast<std::uint64_t>(i));
    }

    EXPECT_EQ(queue.writer_sequence(), static_cast<std::uint64_t>(writes));
}

TEST(SingleProcess, ReadNextSequenceNoDropsWhileCaughtUp) {
    const std::string path = make_test_path("rn_no_drop");

    constexpr std::size_t slots = 64;
    constexpr int writes = 20;

    RTMSQueue queue{
        path,
        sizeof(Message::TestMessage),
        alignof(Message::TestMessage),
        slots,
        overwrite_sequence_options(),
    };

    const auto reader_id = queue.register_reader();
    ASSERT_TRUE(reader_id.has_value());

    std::array<std::byte, sizeof(Message::TestMessage)> buffer{};

    for (int i = 0; i < writes; ++i) {
        const Message::TestMessage message{i, static_cast<float>(i)};

        const RTMSMessage rtms_message{
            .size = sizeof(message),
            .data = &message,
        };

        ASSERT_EQ(queue.write(rtms_message).result, WriteResult::SUCCESS);

        MessageInfo info{};
        const ReadResult result = queue.read_next(
            *reader_id, std::span<std::byte>(buffer), info);

        ASSERT_EQ(result, ReadResult::OK);
        EXPECT_EQ(info.sequence, static_cast<std::uint64_t>(i));
        EXPECT_EQ(info.dropped, 0u);

        Message::TestMessage received{};
        std::memcpy(&received, buffer.data(), sizeof(received));
        EXPECT_EQ(received, message);
    }

    queue.release_reader(*reader_id);
}

TEST(SingleProcess, ReadNextEmptyAndInvalidCases) {
    const std::string path = make_test_path("rn_invalid");

    RTMSQueue queue{
        path,
        sizeof(Message::TestMessage),
        alignof(Message::TestMessage),
        8,
        overwrite_sequence_options(),
    };

    const auto reader_id = queue.register_reader();
    ASSERT_TRUE(reader_id.has_value());

    std::array<std::byte, sizeof(Message::TestMessage)> buffer{};
    MessageInfo info{};

    // Nothing published yet: EMPTY, not OK.
    EXPECT_EQ(
        queue.read_next(*reader_id, std::span<std::byte>(buffer), info),
        ReadResult::EMPTY);

    const Message::TestMessage message{7, 7.0F};
    const RTMSMessage rtms_message{
        .size = sizeof(message),
        .data = &message,
    };
    ASSERT_EQ(queue.write(rtms_message).result, WriteResult::SUCCESS);

    ASSERT_EQ(
        queue.read_next(*reader_id, std::span<std::byte>(buffer), info),
        ReadResult::OK);

    // Caught up again: EMPTY, and the same message must not repeat.
    EXPECT_EQ(
        queue.read_next(*reader_id, std::span<std::byte>(buffer), info),
        ReadResult::EMPTY);

    // Out-of-range reader ids are INVALID, not INACTIVE.
    EXPECT_EQ(
        queue.read_next(MAX_READERS, std::span<std::byte>(buffer), info),
        ReadResult::INVALID);
    EXPECT_EQ(
        queue.read_next(
            MAX_READERS + 100, std::span<std::byte>(buffer), info),
        ReadResult::INVALID);

    // A destination smaller than the message is INVALID.
    ASSERT_EQ(queue.write(rtms_message).result, WriteResult::SUCCESS);
    std::array<std::byte, sizeof(Message::TestMessage) - 1> small_buffer{};
    EXPECT_EQ(
        queue.read_next(
            *reader_id, std::span<std::byte>(small_buffer), info),
        ReadResult::INVALID);

    queue.release_reader(*reader_id);
}

TEST(SingleProcess, ReadNextInactiveAfterRelease) {
    const std::string path = make_test_path("rn_inactive");

    RTMSQueue queue{
        path,
        sizeof(Message::TestMessage),
        alignof(Message::TestMessage),
        8,
        overwrite_sequence_options(),
    };

    const auto reader_id = queue.register_reader();
    ASSERT_TRUE(reader_id.has_value());

    queue.release_reader(*reader_id);

    std::array<std::byte, sizeof(Message::TestMessage)> buffer{};
    MessageInfo info{};

    EXPECT_EQ(
        queue.read_next(*reader_id, std::span<std::byte>(buffer), info),
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
        slots,
        overwrite_sequence_options(),
    };

    // Register before publishing so the reader's cursor starts at 0 and the
    // entire run below laps it.
    const auto reader_id = queue.register_reader();
    ASSERT_TRUE(reader_id.has_value());

    for (std::uint64_t i = 0; i < writes; ++i) {
        const Message::TestMessage message{
            static_cast<int>(i),
            static_cast<float>(i),
        };

        const RTMSMessage rtms_message{
            .size = sizeof(message),
            .data = &message,
        };

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

    ASSERT_EQ(
        queue.read_next(*reader_id, std::span<std::byte>(buffer), info),
        ReadResult::OK);
    EXPECT_EQ(info.sequence, expected_first);
    EXPECT_EQ(info.dropped, expected_first);

    Message::TestMessage received{};
    std::memcpy(&received, buffer.data(), sizeof(received));
    EXPECT_EQ(received.id(), static_cast<int>(expected_first));

    std::uint64_t last_sequence = info.sequence;

    for (std::uint64_t expected = expected_first + 1;
         expected < writes;
         ++expected) {
        ASSERT_EQ(
            queue.read_next(*reader_id, std::span<std::byte>(buffer), info),
            ReadResult::OK);

        // Strictly increasing: the reader must never re-observe a sequence.
        EXPECT_GT(info.sequence, last_sequence);
        EXPECT_EQ(info.sequence, expected);
        EXPECT_EQ(info.dropped, 0u);

        last_sequence = info.sequence;
    }

    EXPECT_EQ(
        queue.read_next(*reader_id, std::span<std::byte>(buffer), info),
        ReadResult::EMPTY);

    queue.release_reader(*reader_id);
}

TEST(SingleProcess, ReadNextLatestReportsDroppedThenEmpty) {
    const std::string path = make_test_path("rn_latest");

    constexpr int writes = 5;

    RTMSQueue queue{
        path,
        sizeof(Message::TestMessage),
        alignof(Message::TestMessage),
        64,
        latest_options(),
    };

    const auto reader_id = queue.register_reader();
    ASSERT_TRUE(reader_id.has_value());

    for (int i = 0; i < writes; ++i) {
        const Message::TestMessage message{i, static_cast<float>(i)};

        const RTMSMessage rtms_message{
            .size = sizeof(message),
            .data = &message,
        };

        ASSERT_EQ(queue.write(rtms_message).result, WriteResult::SUCCESS);
    }

    std::array<std::byte, sizeof(Message::TestMessage)> buffer{};
    MessageInfo info{};

    ASSERT_EQ(
        queue.read_next(*reader_id, std::span<std::byte>(buffer), info),
        ReadResult::OK);

    EXPECT_EQ(info.sequence, static_cast<std::uint64_t>(writes - 1));
    EXPECT_EQ(info.dropped, static_cast<std::uint64_t>(writes - 1));

    Message::TestMessage received{};
    std::memcpy(&received, buffer.data(), sizeof(received));
    EXPECT_EQ(received.id(), writes - 1);

    // No intervening write: a second read must be EMPTY, not a repeat of
    // the latest message.
    EXPECT_EQ(
        queue.read_next(*reader_id, std::span<std::byte>(buffer), info),
        ReadResult::EMPTY);

    queue.release_reader(*reader_id);
}

TEST(SingleProcess, DropNewestBufferFullThenRecovers) {
    const std::string path = make_test_path("dn_recover");

    constexpr std::uint64_t slots = 8;

    RTMSQueue queue{
        path,
        sizeof(Message::TestMessage),
        alignof(Message::TestMessage),
        slots,
        sequence_options(),
    };

    const auto reader_id = queue.register_reader();
    ASSERT_TRUE(reader_id.has_value());

    // Fill every slot without reading; the reader is fully stalled.
    for (std::uint64_t i = 0; i < slots; ++i) {
        const Message::TestMessage message{
            static_cast<int>(i),
            static_cast<float>(i),
        };

        const RTMSMessage rtms_message{
            .size = sizeof(message),
            .data = &message,
        };

        ASSERT_EQ(queue.write(rtms_message).result, WriteResult::SUCCESS);
    }

    const Message::TestMessage overflow_message{999, 999.0F};
    const RTMSMessage overflow_rtms_message{
        .size = sizeof(overflow_message),
        .data = &overflow_message,
    };

    EXPECT_EQ(
        queue.write(overflow_rtms_message).result,
        WriteResult::BUFFER_FULL);

    // Drain the stalled reader completely. Every message must come back, in
    // order, with nothing dropped: a full buffer under this policy means the
    // writer stopped, not that anything was lost.
    std::array<std::byte, sizeof(Message::TestMessage)> buffer{};
    MessageInfo info{};
    std::uint64_t drained = 0;

    while (queue.read_next(*reader_id, std::span<std::byte>(buffer), info) ==
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
    const RTMSMessage recovered_rtms_message{
        .size = sizeof(recovered_message),
        .data = &recovered_message,
    };

    const WriteStatus recovered_status = queue.write(recovered_rtms_message);

    EXPECT_EQ(recovered_status.result, WriteResult::SUCCESS);
    EXPECT_EQ(recovered_status.sequence, slots);

    queue.release_reader(*reader_id);
}

// DROP_NEWEST fills to exactly `slots` before refusing a write, and at that
// distance the oldest unread message has NOT been overwritten: its slot is the
// next one the writer would reuse, but that write has not happened yet, and
// admission control will block it until this reader advances.
//
// A reader must therefore deliver sequence 0 with nothing dropped. Skipping it
// would break the one guarantee that makes the reliable mode worth choosing.
// This was a real off-by-one in select_sequence(), which applied the
// overwriting policy's lap correction regardless of the configured policy.
TEST(SingleProcess, DropNewestKeepsOldestMessageAtExactCapacity) {
    const std::string path = make_test_path("dn_exact_cap");

    constexpr std::uint64_t slots = 8;

    RTMSQueue queue{
        path,
        sizeof(Message::TestMessage),
        alignof(Message::TestMessage),
        slots,
        sequence_options(),  // DROP_NEWEST + SEQUENCE
    };

    const auto reader_id = queue.register_reader();
    ASSERT_TRUE(reader_id.has_value());

    // Fill the queue to exactly capacity. write() itself treats this as
    // completely safe: BUFFER_FULL only triggers on the *next* write.
    for (std::uint64_t i = 0; i < slots; ++i) {
        const Message::TestMessage message{
            static_cast<int>(i),
            static_cast<float>(i),
        };

        const RTMSMessage rtms_message{
            .size = sizeof(message),
            .data = &message,
        };

        ASSERT_EQ(queue.write(rtms_message).result, WriteResult::SUCCESS);
    }

    std::array<std::byte, sizeof(Message::TestMessage)> buffer{};
    MessageInfo info{};

    ASSERT_EQ(
        queue.read_next(*reader_id, std::span<std::byte>(buffer), info),
        ReadResult::OK);

    EXPECT_EQ(info.sequence, 0u);
    EXPECT_EQ(info.dropped, 0u);

    queue.release_reader(*reader_id);
}

namespace {

void WriteThread(
    const std::string& path,
    int iterations,
    const std::shared_future<void>& start_signal,
    std::atomic<bool>& failed,
    std::atomic<bool>& writer_done)
{
    try {
        RTMSQueue queue{
            path,
            sizeof(Message::TestMessage),
            alignof(Message::TestMessage),
            64,
            sequence_options(),
        };

        start_signal.wait();

        const auto deadline =
            std::chrono::steady_clock::now() + kTestTimeout;

        for (int sequence = 0;
             sequence < iterations && !failed.load();
             ++sequence)
        {
            const Message::TestMessage message{
                sequence,
                static_cast<float>(sequence),
            };

            const RTMSMessage rtms_message{
                .size = sizeof(message),
                .data = &message,
            };

            for (;;) {
                const WriteResult result =
                    queue.write(rtms_message).result;

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

void ReadThread(
    const std::string& path,
    int iterations,
    std::latch& readers_ready,
    const std::shared_future<void>& start_signal,
    std::atomic<bool>& failed,
    const std::atomic<bool>& writer_done)
{
    std::optional<std::size_t> reader_id;

    try {
        RTMSQueue queue{
            path,
            sizeof(Message::TestMessage),
            alignof(Message::TestMessage),
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

        const auto deadline =
            std::chrono::steady_clock::now() + kTestTimeout;

        int received_count = 0;

        while (received_count < iterations && !failed.load()) {
            Message::TestMessage received{};

            const bool did_read = queue.read(
                *reader_id,
                [&received](std::span<const std::byte> bytes) {
                    std::memcpy(
                        &received,
                        bytes.data(),
                        sizeof(received));
                });

            if (did_read) {
                const int expected_sequence = received_count;

                if (received.id() != expected_sequence ||
                    received.value() !=
                        static_cast<float>(expected_sequence))
                {
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
            if (writer_done.load() &&
                received_count < iterations)
            {
                // Give the final writer release-store one more opportunity
                // to become visible before declaring failure.
                std::this_thread::yield();

                if (!queue.read(
                        *reader_id,
                        [&received](std::span<const std::byte> bytes) {
                            std::memcpy(
                                &received,
                                bytes.data(),
                                sizeof(received));
                        }))
                {
                    failed.store(true);
                    break;
                }

                const int expected_sequence = received_count;

                if (received.id() != expected_sequence ||
                    received.value() !=
                        static_cast<float>(expected_sequence))
                {
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
        64,
        sequence_options(),
    };

    std::latch readers_ready{num_readers};

    std::promise<void> start_promise;
    std::shared_future<void> start_signal =
        start_promise.get_future().share();

    std::atomic<bool> failed{false};
    std::atomic<bool> writer_done{false};

    std::thread writer{
        WriteThread,
        path,
        iterations,
        start_signal,
        std::ref(failed),
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

void FreeRunningWriteThread(
    const std::string& path,
    std::size_t slots,
    const std::shared_future<void>& start_signal,
    std::chrono::steady_clock::time_point deadline,
    std::atomic<bool>& failed)
{
    try {
        RTMSQueue queue{
            path,
            sizeof(Message::TestMessage),
            alignof(Message::TestMessage),
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

            const RTMSMessage rtms_message{
                .size = sizeof(message),
                .data = &message,
            };

            // OVERWRITE_OLDEST never blocks the writer. Anything but
            // SUCCESS here is a real bug: sizes match exactly, so
            // ERROR_SIZE_MISSMATCH cannot legitimately happen either.
            if (queue.write(rtms_message).result != WriteResult::SUCCESS) {
                failed.store(true);
                return;
            }

            ++sequence;
        }
    } catch (...) {
        failed.store(true);
    }
}

void FreeRunningReadThread(
    const std::string& path,
    std::size_t slots,
    const std::shared_future<void>& start_signal,
    std::chrono::steady_clock::time_point deadline,
    std::atomic<bool>& failed,
    std::atomic<std::uint64_t>& total_dropped,
    std::atomic<std::uint64_t>& messages_read,
    std::latch& readers_ready)
{
    std::optional<std::size_t> reader_id;

    try {
        RTMSQueue queue{
            path,
            sizeof(Message::TestMessage),
            alignof(Message::TestMessage),
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
            const ReadResult result = queue.read_next(
                *reader_id, std::span<std::byte>(buffer), info);

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

            // id and value are written together from the same counter, so
            // any mismatch means the reader observed a torn slot.
            if (received.id() != static_cast<int>(received.value())) {
                failed.store(true);
                break;
            }

            if (last_sequence.has_value() &&
                info.sequence <= *last_sequence)
            {
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

TEST(Concurrency, OverwriteOldestNoTornReadsSingleReader) {
    const std::string path = make_test_path("conc_single");

    constexpr std::size_t slots = 8;

    // Keeps the shared memory object alive for the whole test.
    RTMSQueue owner{
        path,
        sizeof(Message::TestMessage),
        alignof(Message::TestMessage),
        slots,
        overwrite_sequence_options(),
    };

    std::latch readers_ready{1};

    std::promise<void> start_promise;
    std::shared_future<void> start_signal =
        start_promise.get_future().share();

    std::atomic<bool> failed{false};
    std::atomic<std::uint64_t> total_dropped{0};
    std::atomic<std::uint64_t> messages_read{0};

    const auto deadline = std::chrono::steady_clock::now() + 400ms;

    std::thread reader{
        FreeRunningReadThread,
        path,
        slots,
        start_signal,
        deadline,
        std::ref(failed),
        std::ref(total_dropped),
        std::ref(messages_read),
        std::ref(readers_ready),
    };

    // The reader must be ACTIVE before the writer starts publishing.
    readers_ready.wait();

    std::thread writer{
        FreeRunningWriteThread,
        path,
        slots,
        start_signal,
        deadline,
        std::ref(failed),
    };

    start_promise.set_value();

    writer.join();
    reader.join();

    EXPECT_FALSE(failed.load());
    EXPECT_GT(messages_read.load(), 0u);

    // If this is ever 0 the test stopped exercising the lapping path it
    // claims to cover (e.g. deadline too short for the host machine).
    EXPECT_GT(total_dropped.load(), 0u);
}

TEST(Concurrency, OverwriteOldestNoTornReadsMultiReader) {
    const std::string path = make_test_path("conc_multi");

    constexpr std::size_t slots = 8;
    constexpr int num_readers = 3;

    RTMSQueue owner{
        path,
        sizeof(Message::TestMessage),
        alignof(Message::TestMessage),
        slots,
        overwrite_sequence_options(),
    };

    std::latch readers_ready{num_readers};

    std::promise<void> start_promise;
    std::shared_future<void> start_signal =
        start_promise.get_future().share();

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
        FreeRunningWriteThread,
        path,
        slots,
        start_signal,
        deadline,
        std::ref(failed),
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
    EXPECT_THROW(
        RTMSQueue(too_long, sizeof(Message::TestMessage), alignof(Message::TestMessage)),
        std::invalid_argument);

    // Exactly 30 characters after leading slash should not throw invalid_argument
    const std::string exactly_30 = make_test_path("123456789012345678901234567890");
    const std::string capped_30 = exactly_30.substr(0, 31);  // '/' + 30 chars
    EXPECT_NO_THROW({
        RTMSQueue q(capped_30, sizeof(Message::TestMessage), alignof(Message::TestMessage));
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
  options.reclaim_mismatched_segment = true;
  return options;
}

// Leaves a segment of `bytes` behind at `path` with no live owner, the way a
// SIGKILLed process does.
void leak_segment(const std::string& path, std::size_t bytes) {
  const int fd = shm_open(path.c_str(), O_CREAT | O_RDWR, 0666);
  ASSERT_NE(fd, -1);
  ASSERT_EQ(ftruncate(fd, static_cast<off_t>(bytes)), 0);
  ASSERT_EQ(close(fd), 0);
}

}  // namespace

TEST(RTMSStaleSegment, PublisherReclaimsALeftoverOfTheWrongShape) {
  const std::string path = make_test_path("stale_reclaim");
  shm_unlink(path.c_str());
  leak_segment(path, 4096);

  // Without the reclaim the leftover wins and construction fails outright.
  EXPECT_THROW(RTMSQueue(path, sizeof(Message::TestMessage),
                         alignof(Message::TestMessage), 64, sequence_options()),
               std::runtime_error);

  RTMSQueue queue{path, sizeof(Message::TestMessage),
                  alignof(Message::TestMessage), 64, publisher_options()};
  const Message::TestMessage message{7, 1.5F};
  EXPECT_EQ(WriteResult(queue.write(RTMSMessage{sizeof(message), &message})),
            WriteResult::SUCCESS);
  shm_unlink(path.c_str());
}

TEST(RTMSStaleSegment, ReaderReportsSkewInsteadOfClearingTheSegment) {
  const std::string path = make_test_path("stale_reader");
  shm_unlink(path.c_str());
  leak_segment(path, 4096);

  // A reader that disagrees with a live publisher is a version skew, so it
  // must say so rather than delete the topic out from under that publisher.
  EXPECT_THROW(RTMSQueue(path, sizeof(Message::TestMessage),
                         alignof(Message::TestMessage), 64, sequence_options()),
               std::runtime_error);

  // The segment is still there for the owner to deal with.
  const int fd = shm_open(path.c_str(), O_RDWR, 0666);
  EXPECT_NE(fd, -1);
  if (fd != -1) close(fd);
  shm_unlink(path.c_str());
}

TEST(RTMSStaleSegment, PublisherLeavesAHealthySegmentAloneUnderALiveReader) {
  const std::string path = make_test_path("stale_healthy");
  shm_unlink(path.c_str());

  // A reader gets there first and takes a slot. The publisher that follows
  // has an identical layout, so there is nothing to reclaim -- and unlinking
  // here would strand this reader on memory no one else can reach. The shm
  // object is page-rounded, so its measured size is larger than the layout
  // asked for: only the recorded header can decide whether it matches.
  RTMSQueue reader{path, sizeof(Message::TestMessage),
                   alignof(Message::TestMessage), 64, sequence_options()};
  const auto reader_id = reader.register_reader();
  ASSERT_TRUE(reader_id.has_value());

  RTMSQueue writer{path, sizeof(Message::TestMessage),
                   alignof(Message::TestMessage), 64, publisher_options()};
  const Message::TestMessage sent{42, 3.25F};
  ASSERT_EQ(WriteResult(writer.write(RTMSMessage{sizeof(sent), &sent})),
            WriteResult::SUCCESS);

  Message::TestMessage received{};
  MessageInfo info{};
  ASSERT_EQ(
      reader.read_next(
          *reader_id,
          {reinterpret_cast<std::byte*>(&received), sizeof(received)}, info),
      ReadResult::OK);
  EXPECT_EQ(received.id(), sent.id());
  shm_unlink(path.c_str());
}
