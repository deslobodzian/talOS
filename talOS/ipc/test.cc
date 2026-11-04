#include <gtest/gtest.h>
#include <unistd.h>

#include <string>

#include "ipc_test/ipc_test_message_generated.h"
#include "publisher.h"
#include "subscriber.h"

namespace {

std::string Topic(const char* name) {
  return std::string{"/ipc_"} + name + "_" + std::to_string(::getpid());
}

}  // namespace

TEST(PublisherTest, IPC) {
  ipc::Publisher<IPCMessage::TestMessage> pub{Topic("pub")};
  IPCMessage::TestMessage message{10, 200.0};
  EXPECT_EQ(pub.write(message), WriteResult::SUCCESS);
}

TEST(PubSubTest, IPC) {
  const std::string topic = Topic("pubsub");
  ipc::Publisher<IPCMessage::TestMessage> pub{topic};
  ipc::Subscriber<IPCMessage::TestMessage> sub{topic};

  IPCMessage::TestMessage message{10, 200.0};
  pub.write(message);

  auto msg = sub.read();
  EXPECT_EQ(msg, message);
}

TEST(SubThenPub, IPC) {
  const std::string topic = Topic("subpub");
  ipc::Subscriber<IPCMessage::TestMessage> sub{topic};
  ipc::Publisher<IPCMessage::TestMessage> pub{topic};
  EXPECT_TRUE(sub.registered());
  EXPECT_EQ(sub.queue().schema_name(), "IPCMessage.TestMessage");
  EXPECT_EQ(sub.queue().schema_hash(),
            talos::Descriptor<IPCMessage::TestMessage>::kHash);
}
