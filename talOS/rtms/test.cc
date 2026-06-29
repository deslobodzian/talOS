#include <gtest/gtest.h>
#include "rtms.h"
#include <flatbuffers/flatbuffers.h>
#include "talOS/rtms/test_message_generated.h"

TEST(SingleProcess, RTMS) {
    auto path = "/tmp/rtms/test";

    RTMSQueue queue{path, sizeof(Message::TestMessage)};
    auto messages = std::array{
        Message::TestMessage{10, 200},
        Message::TestMessage{23, 23.2},
        Message::TestMessage{0, 0.111111},
    };

    for (auto& message : messages) {
        RTMSMessage rtms_message{sizeof(Message::TestMessage), static_cast<void*>(&message)};
        queue.write(rtms_message);
    }
    std::vector<RTMSMessage> new_messages;

    for (auto& message : messages) {
        Message::TestMessage new_fb{};
        queue.read(
            0,
            [&new_fb](std::span<const std::byte> bytes) {
                std::memcpy(&new_fb, bytes.data(), bytes.size());
             }
        );
        EXPECT_EQ(new_fb, message);
        std::printf("new_fb: %i, %f\n", new_fb.id(), new_fb.value());
    }
}

TEST(RTMSTest, RTMS) {
    auto path = "/tmp/rtms/test";
    RTMSQueue queue{path, sizeof(Message::TestMessage)};

    EXPECT_EQ(path, queue.path());
    EXPECT_EQ(sizeof(Message::TestMessage), queue.message_size());
}

