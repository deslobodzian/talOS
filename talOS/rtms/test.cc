#include <gtest/gtest.h>
#include "rtms.h"
#include <flatbuffers/flatbuffers.h>
#include "talOS/rtms/test_message_generated.h"

TEST(RTMSMessage, RTMS) {
    flatbuffers::FlatBufferBuilder builder(1024);


//    auto name = builder.CreateString("TestMessage");
//    Message::TestMessageBuilder message_builder(builder);
//    message_builder.add_name(name);
//    message_builder.add_id(12);
//
//    auto message = message_builder.Finish();
//    builder.Finish(message);
//
//    uint8_t* buf = builder.GetBufferPointer();
//    size_t size = builder.GetSize();
//
//
//    std::printf("Message ptr: %p size: %zu", &buf, size);
//
//    [[maybe_unused]]RTMSMessage rtms_message{size, buf};

    EXPECT_EQ(5, 5);
}

TEST(RTMSTest, RTMS) {
    auto path = "/tmp/rtms/test";
    RTMSQueue queue{path, sizeof(Message::TestMessage)};

    EXPECT_EQ(path, queue.path());
    EXPECT_EQ(sizeof(Message::TestMessage), queue.message_size());
}
