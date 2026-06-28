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
        auto new_message = queue.read(0);
        EXPECT_EQ(new_message.size, sizeof(Message::TestMessage));
        auto new_fb = static_cast<const Message::TestMessage*>(new_message.data);
        Message::TestMessage new_stack_fb{new_fb->id(), new_fb->value()};
        EXPECT_EQ(new_stack_fb, message);
        std::printf("new_fb: %i, %f\n", new_fb->id(), new_fb->value());
    }
}

TEST(RTMSTest, RTMS) {
    auto path = "/tmp/rtms/test";
    RTMSQueue queue{path, sizeof(Message::TestMessage)};

    EXPECT_EQ(path, queue.path());
    EXPECT_EQ(sizeof(Message::TestMessage), queue.message_size());
}

