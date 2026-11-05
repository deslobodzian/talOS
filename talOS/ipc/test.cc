#include <gtest/gtest.h>
#include <unistd.h>

#include <memory>
#include <string>
#include <vector>

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

TEST(SubscriberTest, ReaderExhaustionNamesTopicAndLimit) {
  const std::string topic = Topic("full");
  std::vector<std::unique_ptr<ipc::Subscriber<IPCMessage::TestMessage>>>
      readers;
  for (std::size_t i = 0; i < MAX_READERS; ++i) {
    readers.push_back(
        std::make_unique<ipc::Subscriber<IPCMessage::TestMessage>>(topic));
  }
  try {
    ipc::Subscriber<IPCMessage::TestMessage> extra{topic};
    FAIL() << "reader exhaustion must not silently create a deaf subscriber";
  } catch (const std::runtime_error& error) {
    EXPECT_NE(std::string{error.what()}.find(topic), std::string::npos);
    EXPECT_NE(std::string{error.what()}.find(std::to_string(MAX_READERS)),
              std::string::npos);
  }
  readers.pop_back();
  EXPECT_NO_THROW((ipc::Subscriber<IPCMessage::TestMessage>{topic}));
}
