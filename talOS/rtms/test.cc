#include <flatbuffers/flatbuffers.h>
#include <gtest/gtest.h>
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
                    queue.write(rtms_message);

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
