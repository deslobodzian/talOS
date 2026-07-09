#include <gtest/gtest.h>
#include "publisher.h"
#include "subscriber.h"
#include "talOS/ipc/ipc_test_message_generated.h"

TEST(PublisherTest, IPC) {
    ipc::Publisher<IPCMessage::TestMessage> pub{"/test"};
    IPCMessage::TestMessage message{10, 200.0};
    pub.write(message);
}

TEST(PubSubTest, IPC) {
    ipc::Publisher<IPCMessage::TestMessage> pub{"/test"};
    ipc::Subscriber<IPCMessage::TestMessage> sub{"/test"};

    IPCMessage::TestMessage message{10, 200.0};
    pub.write(message);

    auto msg = sub.read();
    std::cout << message.id();
    EXPECT_EQ(msg, message);
}

