#include <flatbuffers/flatbuffers.h>
#include <gtest/gtest.h>
#include <sched.h>
#include <unistd.h>

#include <array>
#include <future>
#include <latch>
#include <random>
#include <thread>

#include "rtms.h"
#include "talOS/rtms/test_message_generated.h"

TEST(SingleProcess, RTMS) {
#if defined(__linux__)
    auto path = "/rtms_test";
#else
    auto path = "/tmp/rtms/test";
#endif

    RTMSQueue queue = RTMSQueue::create(path, sizeof(Message::TestMessage),
                                        alignof(Message::TestMessage));

    auto messages = std::array{
        Message::TestMessage{10, 200},
        Message::TestMessage{23, 23.2},
        Message::TestMessage{0, 0.111111},
    };

    for (auto& message : messages) {
        RTMSMessage rtms_message{sizeof(Message::TestMessage),
            static_cast<const void*>(&message)};
        queue.write(rtms_message);
    }
    std::vector<RTMSMessage> new_messages;

    for (auto& message : messages) {
        Message::TestMessage new_fb{};
        queue.read(0, [&new_fb](std::span<const std::byte> bytes) {
            std::memcpy(&new_fb, bytes.data(), bytes.size());
        });
        EXPECT_EQ(new_fb, message);
        std::printf("new_fb: %i, %f\n", new_fb.id(), new_fb.value());
    }
}

TEST(WrapTest, RTMS) {
#if defined(__linux__)
    auto path = "/rtms_test";
#else
    auto path = "/tmp/rtms/test";
#endif

    RTMSQueue queue = RTMSQueue::create(path, sizeof(Message::TestMessage),
                                        alignof(Message::TestMessage));
    auto messages = std::array{
        Message::TestMessage{10, 200},
        Message::TestMessage{23, 23.2},
        Message::TestMessage{0, 0.111111},
    };

    for (auto& message : messages) {
        RTMSMessage rtms_message{sizeof(Message::TestMessage),
            static_cast<const void*>(&message)};
        queue.write(rtms_message);

        Message::TestMessage new_fb{};
        queue.read(0, [&new_fb](std::span<const std::byte> bytes) {
            std::memcpy(&new_fb, bytes.data(), bytes.size());
        });
        EXPECT_EQ(new_fb, message);
        std::printf("new_fb: %i, %f\n", new_fb.id(), new_fb.value());
    }
}

void WriteThread(std::string_view path, int iterations,
                 const std::shared_future<void>& start_signal) {
    std::random_device rd;   // a seed source for the random number engine
    std::mt19937 gen(rd());  // mersenne_twister_engine seeded with rd()
    std::uniform_int_distribution<> distrib(1, 100);
    std::uniform_real_distribution<float> distrib_real(0.0, 300.0);
    RTMSQueue queue = RTMSQueue::attach(path, sizeof(Message::TestMessage),
                                        alignof(Message::TestMessage));

    start_signal.wait();
    for (int i = 0; i < iterations; ++i) {
        Message::TestMessage msg{distrib(gen), distrib_real(gen)};
        RTMSMessage rtms_msg{sizeof(Message::TestMessage),
            static_cast<const void*>(&msg)};
        queue.write(rtms_msg);
    }
}

void ReadThread(
    std::string_view path,
    int reader,
    int iterations,
    std::latch& readers_ready,
    const std::shared_future<void>& start_signal) {
    RTMSQueue queue = RTMSQueue::attach(path, sizeof(Message::TestMessage),
                                        alignof(Message::TestMessage));
    auto reader_id = queue.register_reader();
    ASSERT_TRUE(reader_id.has_value());
    std::printf("Reader ID: %lu\n", reader_id.value());

    readers_ready.count_down();
    start_signal.wait();
    for (int i = 0; i < iterations; ++i) {
        queue.read(reader_id.value(), [reader](std::span<const std::byte> bytes) {
            Message::TestMessage new_fb{};
            std::memcpy(&new_fb, bytes.data(), bytes.size());
            std::printf("reader %i: new_fb: %i, %f\n", reader, new_fb.id(),
                        new_fb.value());
        });
    }

}

TEST(MutliReader, RTMS) {
#if defined(__linux__)
    auto path = "/rtms_test";
#else
    auto path = "/tmp/rtms/test";
#endif
    constexpr int iterations = 1000;
    constexpr int num_readers = 4;

    RTMSQueue owner = RTMSQueue::create(path, sizeof(Message::TestMessage),
                                        alignof(Message::TestMessage));

    std::latch readers_ready{num_readers};
    std::promise<void> start_promise;
    std::shared_future<void> start_signal = start_promise.get_future().share();

    std::thread writer{WriteThread, path, iterations, start_signal};

    std::array<std::thread, 8> readers{};

    for (int i = 0; i < num_readers; ++i) {
        readers[i] = std::thread{
            ReadThread,
            path,
            i,
            iterations,
            std::ref(readers_ready),
            start_signal
        };
    }
    readers_ready.wait();
    start_promise.set_value();

    writer.join();
    for (auto& reader : readers) {
        if (reader.joinable()) {
            reader.join();
        }
    }
}
